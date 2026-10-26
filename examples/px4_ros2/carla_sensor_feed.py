#!/usr/bin/env python3
"""Mount CARLA sensors on CarlaAir's AirSim drone and stream their raw data.

CarlaAir registers the AirSim drone as the CARLA actor ``airsim.drone``, so
CARLA sensors can be attached to it. They render every game frame (or every
``sensor_tick``) without the per-request wait of AirSim's simGetImages, and
CARLA's lidar ray-casts in parallel outside AirSim's lockstep physics.

This process needs CarlaAir's ``carla`` module (Python 3.10), which cannot be
imported next to ROS 2 Jazzy (Python 3.12). It therefore only connects to
carla_sensor_bridge.py over a Unix socket, receives the sensor list from it,
spawns the sensors, and streams every measurement as

    4-byte big-endian header length | JSON header | payload bytes

with the header {"sensor", "frame", "timestamp" (CARLA game time), "wall"
(wall clock when CARLA delivered the data),
"transform" (sensor world pose: x, y, z, roll, pitch, yaw in CARLA/Unreal
units), "size", ...}. carla_sensor_bridge.py converts the game time into the
AirSim clock and publishes on ROS 2. The sensors are destroyed on exit.

Usage: carla_sensor_feed.py --socket PATH [--host 127.0.0.1] [--port 2000]
"""

from __future__ import annotations

import argparse
import json
import queue
import signal
import socket
import struct
import sys
import threading
import time

import carla


def send(connection: socket.socket, header: dict, payload: bytes = b"") -> None:
    header["size"] = len(payload)
    data = json.dumps(header).encode()
    connection.sendall(struct.pack(">I", len(data)) + data + payload)


def receive_json(connection: socket.socket) -> dict:
    def exactly(n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = connection.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("bridge closed the connection")
            buf += chunk
        return buf
    return json.loads(exactly(struct.unpack(">I", exactly(4))[0]))


def pose(transform: carla.Transform) -> list[float]:
    l, r = transform.location, transform.rotation
    return [l.x, l.y, l.z, r.roll, r.pitch, r.yaw]


def measurement(name: str, data) -> tuple[dict, bytes]:
    header = {"sensor": name, "frame": data.frame, "timestamp": data.timestamp, "wall": time.time(),
              "transform": pose(data.transform)}
    if isinstance(data, carla.Image):
        header.update(kind="image", width=data.width, height=data.height)
    elif isinstance(data, carla.LidarMeasurement):
        header.update(kind="lidar", channels=data.channels, horizontal_angle=data.horizontal_angle,
                      channel_counts=[data.get_point_count(c) for c in range(data.channels)])
    elif isinstance(data, carla.RadarMeasurement):
        header.update(kind="radar")
    else:
        raise TypeError(f"unsupported measurement {type(data).__name__}")
    return header, bytes(data.raw_data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--socket", required=True, help="carla_sensor_bridge.py's Unix socket")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    args = parser.parse_args()

    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda *_: stop.set())

    connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    while not stop.is_set():
        try:
            connection.connect(args.socket)
            break
        except OSError:
            time.sleep(0.5)
    if stop.is_set():
        return 0
    config = receive_json(connection)

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    drone = None
    while drone is None and not stop.is_set():
        drone = next((a for a in world.get_actors() if a.type_id == "airsim.drone"), None)
        if drone is None:
            time.sleep(0.5)
    if drone is None:
        return 0

    # Spawn all sensors in one batch, so sensors with the same sensor_tick fire
    # on the same frames.
    library = world.get_blueprint_library()
    commands, names = [], []
    for spec in config["sensors"]:
        bp = library.find(spec["type"])
        for key, value in spec.get("attributes", {}).items():
            bp.set_attribute(key, str(value))
        m = spec["mount"]
        transform = carla.Transform(carla.Location(m["x"], m["y"], m["z"]),
                                    carla.Rotation(roll=m.get("roll", 0.0), pitch=m.get("pitch", 0.0), yaw=m.get("yaw", 0.0)))
        commands.append(carla.command.SpawnActor(bp, transform, drone.id))
        names.append(spec["name"])
    results = client.apply_batch_sync(commands, True)
    sensors = []
    for name, result in zip(names, results):
        if result.error:
            print(f"[carla_sensor_feed] could not spawn {name}: {result.error}", flush=True)
            continue
        sensors.append((name, world.get_actor(result.actor_id)))

    # About 1 s of messages: a bridge that falls behind loses frames instead of
    # receiving them seconds late in a burst (200 held several seconds of images).
    outbox: queue.Queue = queue.Queue(maxsize=60)
    for name, sensor in sensors:
        def callback(data, name=name):
            try:
                outbox.put_nowait(measurement(name, data))
            except queue.Full:  # the bridge is behind; drop rather than stall CARLA
                pass
        sensor.listen(callback)
    print(f"[carla_sensor_feed] streaming {', '.join(n for n, _ in sensors)} from drone actor {drone.id}", flush=True)

    try:
        while not stop.is_set():
            try:
                header, payload = outbox.get(timeout=0.2)
            except queue.Empty:
                continue
            send(connection, header, payload)
    except (ConnectionError, BrokenPipeError, OSError) as error:
        print(f"[carla_sensor_feed] bridge connection lost: {error}", flush=True)
    finally:
        for _, sensor in sensors:
            sensor.stop()
        client.apply_batch_sync([carla.command.DestroyActor(s.id) for _, s in sensors], False)
        connection.close()
        print(f"[carla_sensor_feed] removed {len(sensors)} sensors", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
