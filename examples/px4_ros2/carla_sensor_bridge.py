#!/usr/bin/env python3
"""Publish CARLA sensors mounted on CarlaAir's drone on ROS 2, in AirSim time.

carla_sensor_feed.py (Python 3.10, CarlaAir's carla module) attaches the
sensors of the configuration to the drone actor and streams their data to
this node (Python 3.12, ROS 2) over a Unix socket. This node

* stamps every measurement in the AirSim clock that px4_sensor_bridge.py
  publishes (/clock, IMU): CARLA stamps sensors in Unreal game time, which
  drifts against AirSim's lockstep clock. Each measurement comes with its
  sensor's world pose, i.e. the drone pose it was rendered from. This node
  samples AirSim's ground-truth trajectory (~250 Hz, AirSim time) and stamps
  the measurement with the time at which the drone had that pose. While the
  drone does not move the pose does not tell the time; then a linear
  game-time -> AirSim-time model fitted to the recent matches is used.
* publishes cameras (rgb8/bgr8/mono8, depth as 32FC1 metres) with CameraInfo,
  in one group with one stamp and a fixed topic order (``camera_order``);
  lidar sweeps as PointCloud2 (x, y, z, intensity, ring, time); radar
  detections as PointCloud2 (x, y, z, velocity, intensity) with measurement
  noise; and /tf_static from the body frame (FLU) to every sensor (cameras
  as optical frames).

Mounts are given in CARLA/Unreal units relative to the drone actor (x forward,
y right, z up, metres; roll/pitch/yaw in degrees).
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import os
import socket
import struct
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

from airsim_rpc import AirSimRpc

ROT_WEIGHT_M = 0.5  # metres per radian when comparing poses


def stamp(nanoseconds: int) -> Time:
    return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def rotation_ned(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation body -> world for Unreal/NED Euler angles in degrees (Z-Y-X).

    Unreal (x forward, y right, z up, left-handed) and NED share x and y and
    the sense of roll, pitch (nose up) and yaw (clockwise seen from above);
    only z is flipped.
    """
    r, p, y = (math.radians(a) for a in (roll, pitch, yaw))
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    return rz @ ry @ rx


def rotation_from_quaternion(q: dict) -> np.ndarray:
    w, x, y, z = q["w_val"], q["x_val"], q["y_val"], q["z_val"]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    trace = np.trace(r)
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    return q[0], q[1], q[2], (r[k, j] - r[j, k]) / s


def angle_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rotation angles between stacked rotations a (N, 3, 3) and one rotation b."""
    cos = (np.einsum("nij,ij->n", a, b) - 1.0) / 2.0
    return np.arccos(np.clip(cos, -1.0, 1.0))


def cloud(header_stamp: Time, frame_id: str, columns: dict[str, np.ndarray]) -> PointCloud2:
    kinds = {np.dtype(np.float32): PointField.FLOAT32, np.dtype(np.uint16): PointField.UINT16}
    dtype = np.dtype([(name, values.dtype) for name, values in columns.items()])
    data = np.empty(len(next(iter(columns.values()))), dtype=dtype)
    for name, values in columns.items():
        data[name] = values
    msg = PointCloud2()
    msg.header.stamp = header_stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = 1, len(data)
    msg.fields = [PointField(name=name, offset=dtype.fields[name][1], datatype=kinds[dtype.fields[name][0]], count=1)
                  for name in columns]
    msg.is_bigendian = False
    msg.point_step = dtype.itemsize
    msg.row_step = dtype.itemsize * len(data)
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


class Trajectory:
    """AirSim ground-truth body poses with AirSim time stamps."""

    def __init__(self, host: str, port: int, vehicle: str, rate_hz: float = 250.0, seconds: float = 20.0):
        self.host, self.port, self.vehicle = host, port, vehicle
        self.period = 1.0 / rate_hz
        self.samples: collections.deque = collections.deque(maxlen=int(rate_hz * seconds))
        self.lock = threading.Lock()

    def run(self, stop: threading.Event) -> None:
        with AirSimRpc(self.host, self.port) as rpc:
            last = None
            while not stop.is_set():
                imu = rpc.call("getImuData", "", self.vehicle)  # AirSim time + true attitude
                if imu["time_stamp"] != last:
                    last = imu["time_stamp"]
                    k = rpc.call("simGetGroundTruthKinematics", self.vehicle)
                    p = k["position"]
                    with self.lock:
                        self.samples.append((imu["time_stamp"], np.array([p["x_val"], p["y_val"], p["z_val"]]),
                                             rotation_from_quaternion(imu["orientation"]), time.time_ns()))
                time.sleep(self.period)

    def window(self, t0: int, t1: int):
        with self.lock:
            rows = [s for s in self.samples if t0 <= s[0] <= t1]
        if len(rows) < 3:
            return None
        return (np.array([r[0] for r in rows], dtype=np.int64), np.array([r[1] for r in rows]),
                np.array([r[2] for r in rows]))

    def latest(self):
        with self.lock:
            return self.samples[-1] if self.samples else None

    def airsim_minus_wall(self) -> float | None:
        """Median of AirSim time - wall clock [ns] over the last 2 s of samples."""
        with self.lock:
            if not self.samples or len(self.samples[-1]) < 4:
                return None
            newest = self.samples[-1][3]
            rows = [s[0] - s[3] for s in reversed(self.samples) if len(s) > 3 and s[3] > newest - 2_000_000_000]
        return float(np.median(rows)) if len(rows) >= 10 else None


class Clock:
    """Maps CARLA game time to AirSim time with the drone's rendered poses.

    The two clocks drift only slowly, so AirSim time = game time + delta with a
    slowly varying delta. Every 0.5 s, delta is chosen so that the CARLA poses
    of the last 2 s of frames best fit AirSim's ground-truth trajectory. A
    pose sequence is unambiguous where a single pose is not (a hovering yaw
    sweep passes every heading twice). The two position frames have different
    origins (and AirSim's ground truth jumps when the vehicle first moves), so
    the fit removes the best constant translation for every candidate delta.
    The clocks drift (game time ran ~0.15 % fast: ~1.4 ms/s). While the drone
    does not move (no fit possible), delta follows that drift: from the last
    good fit on, by how AirSim time and game time each moved against the wall
    clock (the feed sends the wall time of every frame; constant latencies
    cancel), else by the drift rate fitted over the good fits of the last
    minutes. A hover of a few minutes otherwise left the image stamps 0.2-0.4 s
    off, and the VIO lost tracking at the next turn.

    Robustness: once fitted, one fit moves delta by at most MAX_STEP_NS unless
    the next fit confirms the step: the clocks do jump (AirSim's lockstep clock
    fell 2 s behind game time within a minute when the XR session started and
    the drone flew; slewing at 20 ms per fit left the images up to 2 s off and
    the VIO died). A frame is never stamped later than AirSim's clock now
    (newest ground truth plus the wall time since): delta is pulled back at
    once. If no ground truth lies in the
    search window, or no fit has succeeded for REFIT_AFTER_S while moving,
    delta is re-seeded from the newest samples and searched again over +-1 s:
    a stuck delta stamps every image at the wrong time and stops the VIO.
    """

    MAX_STEP_NS = 20_000_000
    STEP_AGREE_NS = 10_000_000
    FUTURE_SLACK_NS = 30_000_000
    REFIT_AFTER_S = 10.0

    def __init__(self, trajectory: Trajectory, logger):
        self.trajectory = trajectory
        self.logger = logger
        self.delta = None  # AirSim ns - game ns
        self.fitted = False
        self.history: collections.deque = collections.deque(maxlen=400)  # (game s, position, rotation)
        self.next_fit = 0.0
        self.last_fit_ok = time.monotonic()
        self.fits: collections.deque = collections.deque(maxlen=600)  # (game s, delta ns) of good fits
        self.walls: collections.deque = collections.deque(maxlen=200)  # (game s, wall - game ns) of frames
        self.anchor = None  # (delta, AirSim - wall, wall - game) at the last good fit
        self.step_candidate = None  # delta of a large step seen once
        self.drift = 0.0  # d(delta)/d(game time), from self.fits
        self.lock = threading.Lock()
        self.stats = collections.Counter()

    def airsim_time(self, game_time: float, p_body: np.ndarray, r_body: np.ndarray,
                    wall: float | None = None) -> int | None:
        latest = self.trajectory.latest()
        if latest is None:
            return None
        with self.lock:
            if wall is not None and (not self.walls or self.walls[-1][0] != game_time):
                self.walls.append((game_time, wall * 1e9 - game_time * 1e9))
            if self.delta is None:
                self.delta = latest[0] - int(game_time * 1e9)
            if not self.history or self.history[-1][0] != game_time:
                self.history.append((game_time, p_body, r_body))
            if time.monotonic() >= self.next_fit:
                self.next_fit = time.monotonic() + 0.5
                self._fit()
            if self.fits and self.history[-1][0] - self.fits[-1][0] > 1.0:
                # No fit for a while (hovering): follow the clock drift, measured
                # against the wall clock since the last good fit if possible.
                now = self._wall_offsets()
                if self.anchor is not None and now is not None:
                    self.delta = self.anchor[0] + int((now[0] - self.anchor[1]) + (now[1] - self.anchor[2]))
                    self.stats["wall_tracked"] += 1
                else:
                    self.delta = self.fits[-1][1] + int(self.drift * (game_time - self.fits[-1][0]) * 1e9)
            t = int(game_time * 1e9) + self.delta
            if len(latest) > 3:
                # A frame cannot be newer than AirSim's clock now: a stamp from
                # the future makes the VIO wait for IMU data that never fits.
                now = latest[0] + (time.time_ns() - latest[3])
                if t > now + self.FUTURE_SLACK_NS:
                    self.delta -= t - now
                    if self.anchor is not None:
                        self.anchor = (self.anchor[0] - (t - now), *self.anchor[1:])
                    self.stats["future_capped"] += 1
                    t = now
            return t

    def _wall_offsets(self) -> tuple[float, float] | None:
        """(AirSim - wall, wall - game) [ns], medians over the last 2 s."""
        airsim_wall = self.trajectory.airsim_minus_wall()
        if airsim_wall is None or len(self.walls) < 10:
            return None
        newest = self.walls[-1][0]
        return airsim_wall, float(np.median([w[1] for w in self.walls if w[0] > newest - 2.0]))

    def _fit_drift(self) -> None:
        """Clock drift rate from the good fits of the last 10 min (needs >= 60 s)."""
        while self.fits and self.fits[-1][0] - self.fits[0][0] > 600.0:
            self.fits.popleft()
        if len(self.fits) < 20 or self.fits[-1][0] - self.fits[0][0] < 60.0:
            return
        t = np.array([f[0] for f in self.fits])
        d = np.array([f[1] for f in self.fits], dtype=np.float64) * 1e-9
        rate = float(np.polyfit(t - t[0], d - d[0], 1)[0])
        self.drift = max(-0.05, min(0.05, rate))  # at most 5 % (under load AirSim ran 1.2 % slow)
        self.stats["drift_ppm"] = self.drift * 1e6

    def _reseed(self, why: str) -> None:
        latest = self.trajectory.latest()
        if latest is None:
            return
        self.delta = latest[0] - int(self.history[-1][0] * 1e9)
        self.fitted = False  # the next fits search +-1 s
        self.last_fit_ok = time.monotonic()
        self.stats["reseeded"] += 1
        self.logger.warning(f"time alignment re-seeded ({why}): AirSim - game time {self.delta / 1e9:.3f} s")

    def _fit(self) -> None:
        newest = self.history[-1][0]
        frames = [h for h in self.history if h[0] > newest - 2.0]
        if len(frames) < 10:
            self.stats["few_frames"] += 1
            return
        g = np.array([f[0] for f in frames]) * 1e9
        c = np.array([f[1] for f in frames])  # (K, 3) CARLA-derived positions
        span = 300_000_000 if self.fitted else 1_000_000_000
        rows = self.trajectory.window(int(g[0]) + self.delta - span - 50_000_000,
                                      int(g[-1]) + self.delta + span + 50_000_000)
        if rows is None:
            self.stats["no_ground_truth"] += 1
            if time.monotonic() - self.last_fit_ok > 2.0:
                self._reseed("no ground truth in the search window")
            return
        t, p, r = rows
        motion = (np.linalg.norm(c.max(axis=0) - c.min(axis=0))
                  + ROT_WEIGHT_M * angle_between(np.array([f[2] for f in frames]), frames[0][2]).max())
        if motion < 0.05:
            self.stats["still"] += 1
            return
        deltas = self.delta + np.arange(-span, span + 1, 2_000_000)
        query = g[None, :] + deltas[:, None]  # (D, K) AirSim times
        tf = (t - t[0]).astype(np.float64)
        q = (query - t[0]).astype(np.float64)
        interp = np.stack([np.interp(q, tf, p[:, a]) for a in range(3)], axis=-1)  # (D, K, 3)
        diff = c[None, :, :] - interp
        residual = diff - diff.mean(axis=1, keepdims=True)  # remove the best constant translation
        angles = np.stack([angle_between(r, f[2]) for f in frames])  # (K, N)
        nearest = np.clip(np.searchsorted(tf, q), 0, len(tf) - 1)  # (D, K)
        rot = angles[np.arange(len(frames))[None, :], nearest]
        total = np.sum(residual ** 2, axis=(1, 2)) + np.sum((ROT_WEIGHT_M * rot) ** 2, axis=1)
        i = int(np.argmin(total))
        rms = math.sqrt(total[i] / len(frames))
        if rms > 0.05 or i in (0, len(deltas) - 1):
            self.stats["fit_bad"] += 1
            self.stats["fit_bad_rms"] = rms
            if self.fitted and time.monotonic() - self.last_fit_ok > self.REFIT_AFTER_S:
                self._reseed(f"no good fit for {self.REFIT_AFTER_S:.0f} s while moving")
            return
        c0, c1, c2 = total[i - 1], total[i], total[i + 1]
        denom = c0 - 2 * c1 + c2
        frac = max(-1.0, min(1.0, 0.5 * (c0 - c2) / denom)) if denom > 1e-12 else 0.0
        new_delta = int(deltas[i] + frac * 2_000_000)
        if self.fitted and abs(new_delta - self.delta) > self.MAX_STEP_NS:
            if self.step_candidate is not None and abs(new_delta - self.step_candidate) < self.STEP_AGREE_NS:
                self.stats["stepped"] += 1  # two fits in a row agree: the clocks did jump
                self.step_candidate = None
            else:
                self.stats["clamped"] += 1
                self.step_candidate = new_delta
                new_delta = self.delta + int(math.copysign(self.MAX_STEP_NS, new_delta - self.delta))
        else:
            self.step_candidate = None
        self.delta = new_delta
        self.fitted = True
        self.last_fit_ok = time.monotonic()
        self.fits.append((newest, self.delta))
        self._fit_drift()
        offsets = self._wall_offsets()
        self.anchor = (self.delta, *offsets) if offsets is not None else None
        self.stats["fit_ok"] += 1
        self.stats["fit_rms_mm"] = rms * 1000.0


class CarlaSensorBridge:
    def __init__(self, node, config: dict, socket_path: str):
        self.node = node
        self.config = config
        airsim = config.get("airsim", {})
        self.trajectory = Trajectory(airsim.get("host", "127.0.0.1"), int(airsim.get("port", 41451)), airsim["vehicle"])
        self.clock = Clock(self.trajectory, node.get_logger())
        self.stop = threading.Event()
        self.socket_path = socket_path
        self.specs = {s["name"]: s for s in config["sensors"]}
        self.mounts = {}
        for name, s in self.specs.items():
            m = s["mount"]
            self.mounts[name] = (np.array([m["x"], m["y"], -m["z"]]),
                                 rotation_ned(m.get("roll", 0.0), m.get("pitch", 0.0), m.get("yaw", 0.0)))
        # Publishers
        self.image_pubs = {}
        for s in config["sensors"]:
            for out in s.get("publish", []):
                if s["type"].startswith("sensor.camera"):
                    self.image_pubs[out["topic"]] = (node.create_publisher(Image, out["topic"], 5),
                                                     node.create_publisher(CameraInfo, out["info_topic"], 5))
                else:
                    self.image_pubs[out["topic"]] = (node.create_publisher(PointCloud2, out["topic"], 5), None)
        self.camera_order = config.get("camera_order", [])
        self.topic_sensor = {out["topic"]: s["name"] for s in config["sensors"] for out in s.get("publish", [])}
        self.group = sorted({self.topic_sensor[t] for t in self.camera_order})
        self.pending: dict[int, dict] = {}
        self.sweeps: dict[str, list] = {}
        self.last_partial: dict[str, float] = {}
        self.last_radar: dict[str, float] = {}
        self.rng = np.random.default_rng(0)
        self.counts = collections.Counter()
        self.last_stamp: dict[str, int] = {}  # topic -> newest published image stamp [ns]
        self._publish_tf(config.get("body_frame", "base_link"))

    # --- static frames --------------------------------------------------------
    def _publish_tf(self, body_frame: str) -> None:
        flip = np.diag([1.0, -1.0, -1.0])  # NED/FRD <-> FLU
        transforms = []
        for name, s in self.specs.items():
            t_ned, r_ned = self.mounts[name]
            r_flu = flip @ r_ned @ flip
            if s["type"].startswith("sensor.camera"):
                # optical frame: x right, y down, z forward (sensor x)
                r_flu = r_flu @ np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
            for frame in {out["frame_id"] for out in s.get("publish", [])}:
                msg = TransformStamped()
                msg.header.frame_id = body_frame
                msg.child_frame_id = frame
                msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = flip @ t_ned
                q = msg.transform.rotation
                q.x, q.y, q.z, q.w = quaternion_xyzw(r_flu)
                transforms.append(msg)
        self.tf = StaticTransformBroadcaster(self.node)
        self.tf.sendTransform(transforms)

    # --- threads ----------------------------------------------------------------
    def start(self) -> None:
        threading.Thread(target=self._guard, args=(self.trajectory.run, self.stop), daemon=True).start()
        threading.Thread(target=self._guard, args=(self._serve,), daemon=True).start()

    def _guard(self, loop, *args) -> None:
        try:
            loop(*args)
        except Exception as error:
            if not self.stop.is_set():
                self.node.get_logger().fatal(f"{loop.__name__} failed: {error!r}")
                self.stop.set()

    def _serve(self) -> None:
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(self.socket_path)
        server.listen(1)
        server.settimeout(0.5)
        while not self.stop.is_set():
            try:
                connection, _ = server.accept()
            except socket.timeout:
                continue
            self.node.get_logger().info("carla_sensor_feed connected")
            with connection:
                data = json.dumps({"sensors": self.config["sensors"]}).encode()
                connection.sendall(struct.pack(">I", len(data)) + data)
                reader = connection.makefile("rb", buffering=1 << 22)
                try:
                    while not self.stop.is_set():
                        n = reader.read(4)
                        if len(n) < 4:
                            break
                        header = json.loads(reader.read(struct.unpack(">I", n)[0]))
                        payload = reader.read(header["size"])
                        self._handle(header, payload)
                except (ConnectionError, OSError):
                    pass
            self.node.get_logger().warn("carla_sensor_feed disconnected")
        server.close()

    # --- measurements -------------------------------------------------------------
    def _airsim_time(self, header: dict) -> int | None:
        x, y, z, roll, pitch, yaw = header["transform"]
        r_sensor = rotation_ned(roll, pitch, yaw)
        t_mount, r_mount = self.mounts[header["sensor"]]
        r_body = r_sensor @ r_mount.T
        p_body = np.array([x, y, -z]) - r_body @ t_mount
        return self.clock.airsim_time(header["timestamp"], p_body, r_body, header.get("wall"))

    def _handle(self, header: dict, payload: bytes) -> None:
        kind = header["kind"]
        if kind == "image":
            if header["sensor"] in self.group:
                frame = self.pending.setdefault(header["frame"], {})
                frame[header["sensor"]] = (header, payload)
                if all(s in frame for s in self.group):
                    del self.pending[header["frame"]]
                    t = self._airsim_time(header)
                    if t is not None:
                        for topic in self.camera_order:
                            h, p = frame[self.topic_sensor[topic]]
                            self._publish_image(topic, h, p, t)
                for f in [f for f in self.pending if f < header["frame"] - 40]:
                    del self.pending[f]
            else:
                t = self._airsim_time(header)
                if t is not None:
                    for out in self.specs[header["sensor"]].get("publish", []):
                        self._publish_image(out["topic"], header, payload, t)
        elif kind == "lidar":
            self._lidar(header, payload)
        elif kind == "radar":
            self._radar(header, payload)

    def _out(self, topic: str) -> dict:
        return next(o for o in self.specs[self.topic_sensor[topic]]["publish"] if o["topic"] == topic)

    def _publish_image(self, topic: str, header: dict, payload: bytes, t: int) -> None:
        # A stamp going backwards (delta re-seeded) would corrupt the VIO
        # state: drop the frame.
        if t <= self.last_stamp.get(topic, -1):
            self.clock.stats["out_of_order"] += 1
            return
        self.last_stamp[topic] = t
        spec = self.specs[header["sensor"]]
        out = self._out(topic)
        w, h = header["width"], header["height"]
        bgra = np.frombuffer(payload, dtype=np.uint8).reshape(h, w, 4)
        encoding = out.get("encoding", "rgb8")
        if spec["type"] == "sensor.camera.depth":
            rgb = bgra[:, :, :3].astype(np.float32)
            pixels = (rgb[:, :, 2] + rgb[:, :, 1] * 256.0 + rgb[:, :, 0] * 65536.0) / 16777215.0 * 1000.0
            if out.get("max_depth_m") is not None:
                pixels = np.where(pixels > out["max_depth_m"], np.float32(np.inf), pixels)
            pixels = pixels.astype(np.float32)
            encoding = "32FC1"
        elif encoding == "rgb8":
            pixels = bgra[:, :, 2::-1]
        elif encoding == "bgr8":
            pixels = bgra[:, :, :3]
        elif encoding == "mono8":
            pixels = (bgra[:, :, :3] @ np.array([0.114, 0.587, 0.299])).astype(np.uint8)
        else:
            raise ValueError(f"unsupported encoding {encoding!r}")
        pixels = np.ascontiguousarray(pixels)
        msg = Image()
        msg.header.stamp = stamp(t)
        msg.header.frame_id = out["frame_id"]
        msg.width, msg.height, msg.encoding = w, h, encoding
        msg.step = pixels.strides[0]
        msg.data = pixels.tobytes()
        image_pub, info_pub = self.image_pubs[topic]
        image_pub.publish(msg)
        fov = math.radians(float(spec["attributes"]["fov"]))
        f = w / 2.0 / math.tan(fov / 2.0)
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [f, 0.0, w / 2.0, 0.0, f, h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, w / 2.0, 0.0, 0.0, f, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        info_pub.publish(info)
        self.counts[topic] += 1

    def _lidar(self, header: dict, payload: bytes) -> None:
        """Collect CARLA's per-frame partial scans into full sweeps."""
        spec = self.specs[header["sensor"]]
        t = self._airsim_time(header)
        if t is None:
            return
        points = np.frombuffer(payload, dtype=np.float32).reshape(-1, 4)
        ring = np.repeat(np.arange(header["channels"], dtype=np.uint16), header["channel_counts"])
        # Each partial scan covers the game time since the previous one; a sweep
        # is complete when they add up to one rotation.
        previous = self.last_partial.get(header["sensor"])
        self.last_partial[header["sensor"]] = header["timestamp"]
        if previous is None:
            return
        sweep = self.sweeps.setdefault(header["sensor"], [])
        sweep.append((t, header["timestamp"] - previous, points, ring))
        period = 1.0 / float(spec["attributes"]["rotation_frequency"])
        if sum(s[1] for s in sweep) < period - 0.5 * sweep[-1][1]:  # closest whole number of frames
            return
        self.sweeps[header["sensor"]] = []
        t_end = sweep[-1][0]
        xyz = np.concatenate([s[2] for s in sweep])
        times = np.concatenate([np.full(len(s[2]), (s[0] - t_end) * 1e-9, dtype=np.float32) for s in sweep])
        rings = np.concatenate([s[3] for s in sweep])
        out = spec["publish"][0]
        noise = float(out.get("range_noise_m", 0.0))
        if noise > 0 and len(xyz):
            r = np.linalg.norm(xyz[:, :3], axis=1, keepdims=True)
            xyz = xyz.copy()
            xyz[:, :3] *= (1.0 + self.rng.normal(0.0, noise, (len(xyz), 1)) / np.maximum(r, 1e-3))
        self.image_pubs[out["topic"]][0].publish(cloud(stamp(t_end), out["frame_id"], {
            "x": xyz[:, 0], "y": -xyz[:, 1], "z": xyz[:, 2], "intensity": xyz[:, 3].astype(np.float32),
            "ring": rings, "time": times}))
        self.counts[out["topic"]] += 1

    def _radar(self, header: dict, payload: bytes) -> None:
        # CARLA's radar must run every frame: it derives its own velocity from
        # the position change since the previous frame, so with a sensor_tick
        # its Doppler velocities come out too large. Publish at rate_hz instead.
        spec = self.specs[header["sensor"]]
        out = spec["publish"][0]
        last = self.last_radar.get(header["sensor"])
        period = 1.0 / float(out.get("rate_hz", 10.0))
        if last is not None and header["timestamp"] - last < period - 0.02:
            return
        self.last_radar[header["sensor"]] = header["timestamp"]
        t = self._airsim_time(header)
        if t is None:
            return
        d = np.frombuffer(payload, dtype=np.float32).reshape(-1, 4)  # velocity, azimuth, altitude, depth
        n = len(d)
        noise = out.get("noise", {})
        vel = d[:, 0] * float(out.get("velocity_sign", 1.0)) + self.rng.normal(0, noise.get("velocity_mps", 0.0), n)
        az = d[:, 1] + self.rng.normal(0, math.radians(noise.get("azimuth_deg", 0.0)), n)
        el = d[:, 2] + self.rng.normal(0, math.radians(noise.get("elevation_deg", 0.0)), n)
        rng = d[:, 3] + self.rng.normal(0, noise.get("range_m", 0.0), n)
        snr = out.get("snr_at_1m_db", 60.0) - 40.0 * np.log10(np.maximum(rng, 1.0)) + self.rng.normal(0, 2.0, n)
        x = rng * np.cos(el) * np.cos(az)
        y = -(rng * np.cos(el) * np.sin(az))  # Unreal y right -> ROS y left
        z = rng * np.sin(el)
        self.image_pubs[out["topic"]][0].publish(cloud(stamp(t), out["frame_id"], {
            "x": x.astype(np.float32), "y": y.astype(np.float32), "z": z.astype(np.float32),
            "velocity": vel.astype(np.float32), "intensity": snr.astype(np.float32)}))
        self.counts[out["topic"]] += 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, required=True, help="CARLA sensor YAML configuration")
    parser.add_argument("--socket", required=True, help="Unix socket for carla_sensor_feed.py")
    args, ros_args = parser.parse_known_args()
    config = yaml.safe_load(args.config.read_text())

    rclpy.init(args=ros_args)
    node = rclpy.create_node(config.get("node_name", "carlaair_carla_sensor_bridge"),
                             namespace=config.get("namespace", ""))
    bridge = CarlaSensorBridge(node, config, args.socket)
    bridge.start()
    node.get_logger().info(f"waiting for carla_sensor_feed on {args.socket}: "
                           f"{', '.join(s['name'] for s in config['sensors'])}")
    last_report = time.monotonic()
    try:
        while rclpy.ok() and not bridge.stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.monotonic() - last_report > 30.0:
                rates = ", ".join(f"{k} {v / 30.0:.1f} Hz" for k, v in sorted(bridge.counts.items()))
                node.get_logger().info(f"published (wall clock): {rates or 'nothing yet'}")
                s = bridge.clock.stats
                delta = bridge.clock.delta
                node.get_logger().info(
                    f"time alignment: {s['fit_ok']} fits (last rms {s['fit_rms_mm']:.1f} mm), {s['fit_bad']} rejected "
                    f"(rms {s['fit_bad_rms'] * 1000:.0f} mm), {s['still']} while still, {s['clamped']} clamped, {s['stepped']} stepped, {s['future_capped']} capped to AirSim now, "
                    f"{s['no_ground_truth']} without ground truth, {s['few_frames']} with few frames, "
                    f"{s['reseeded']} re-seeded, {s['wall_tracked']} wall-tracked, {s['out_of_order']} out-of-order frames dropped; drift {bridge.clock.drift * 1e6:.0f} ppm; "
                    f"AirSim - game time {(delta or 0) / 1e9:.3f} s")
                bridge.counts.clear()
                s.clear()
                last_report = time.monotonic()
    except KeyboardInterrupt:
        pass
    failed = bridge.stop.is_set()
    bridge.stop.set()
    node.destroy_node()
    rclpy.try_shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
