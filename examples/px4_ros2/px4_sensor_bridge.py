#!/usr/bin/env python3
"""Publish the sensors of a PX4-flown CarlaAir drone on ROS 2.

PX4 SITL flies the vehicle through AirSim's lockstep HIL link (see README.md);
this node only reads sensors over the AirSim RPC API and publishes them:

* ``/clock`` and IMU samples, from a polling thread (one message per new
  AirSim IMU sample);
* camera images and ``CameraInfo``, from a capture thread. All images of one
  capture share one stamp, so stereo pairs stay paired. AirSim stamps images
  when it returns them, a few render frames after it drew them;
  ``cameras.latency_ms`` is subtracted to stamp them at render time;
* ground-truth odometry in an ENU world frame with an FLU body frame.

Everything is configured by a YAML file (``--config``); see
``config/example_bridge.yaml``. Stamps are AirSim simulation time.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu

from airsim_rpc import AirSimRpc

IMAGE_TYPES = {"scene": 0, "depth_planar": 1, "depth_perspective": 2}
# AirSim world NED -> ROS world ENU, and AirSim body FRD -> ROS body FLU.
R_ENU_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
R_FRD_FLU = np.diag([1.0, -1.0, -1.0])


def stamp(nanoseconds: int) -> Time:
    return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def vector(value: dict) -> np.ndarray:
    return np.array([value["x_val"], value["y_val"], value["z_val"]])


def rotation_from_quaternion(q: dict) -> np.ndarray:
    w, x, y, z = q["w_val"], q["x_val"], q["y_val"], q["z_val"]
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quaternion_from_rotation(r: np.ndarray) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) for a rotation matrix."""
    trace = np.trace(r)
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w, x = 0.25 * s, (r[2, 1] - r[1, 2]) / s
        y, z = (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
        q = [0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
        w = (r[k, j] - r[j, k]) / s
        x, y, z = q
    return x, y, z, w


@dataclass(frozen=True)
class Stream:
    """One published image topic, fed by one AirSim capture."""

    capture: tuple[str, str]  # (camera, image type)
    image_topic: str
    info_topic: str
    frame_id: str
    encoding: str
    max_depth_m: float | None


class SensorBridge:
    def __init__(self, node, config: dict):
        self.node = node
        airsim = config.get("airsim", {})
        self.host = airsim.get("host", "127.0.0.1")
        self.port = int(airsim.get("port", 41451))
        self.vehicle = airsim["vehicle"]
        self.stop = threading.Event()

        self.clock_pub = (
            node.create_publisher(Clock, "/clock", 10) if config.get("publish_clock", True) else None
        )

        self.imus = [
            (node.create_publisher(Imu, imu["topic"], 100), imu["frame_id"],
             np.array(imu.get("rotation_from_body_frd", np.eye(3)), dtype=float))
            for imu in config.get("imu", [])
        ]

        cameras = config.get("cameras", {})
        self.fov_deg = {name: float(c["fov_deg"]) for name, c in cameras.get("fov", {}).items()}
        self.max_capture_rate_hz = float(cameras.get("max_rate_hz", 0.0))
        self.latency_ns = int(float(cameras.get("latency_ms", 0.0)) * 1e6)
        self.streams = []
        for s in cameras.get("streams", []):
            image_type = s.get("image", "scene")
            if image_type not in IMAGE_TYPES:
                raise ValueError(f"unknown image type {image_type!r}")
            self.streams.append(Stream(
                capture=(s["camera"], image_type),
                image_topic=s["image_topic"],
                info_topic=s["info_topic"],
                frame_id=s["frame_id"],
                encoding=s.get("encoding", "32FC1" if image_type != "scene" else "rgb8"),
                max_depth_m=s.get("max_depth_m"),
            ))
        self.captures = sorted({s.capture for s in self.streams})
        self.image_pubs = {s: node.create_publisher(Image, s.image_topic, 5) for s in self.streams}
        self.info_pubs = {s: node.create_publisher(CameraInfo, s.info_topic, 5) for s in self.streams}

        truth = config.get("ground_truth")
        self.truth = None
        if truth:
            self.truth = (node.create_publisher(Odometry, truth["topic"], 10),
                          truth.get("world_frame", "world"), truth.get("body_frame", "base_link"),
                          float(truth.get("rate_hz", 50.0)))

    # --- threads -----------------------------------------------------------

    def start(self) -> list[threading.Thread]:
        with AirSimRpc(self.host, self.port) as rpc:
            rpc.call("ping")
            for camera, fov in self.fov_deg.items():
                # The CaptureSettings FOV of settings.json is not applied to
                # CarlaAir's cameras, so set it explicitly.
                rpc.call("simSetCameraFov", camera, fov, self.vehicle, False)
        loops = [self._imu_loop]
        if self.captures:
            loops.append(self._camera_loop)
        if self.truth:
            loops.append(self._truth_loop)
        threads = [threading.Thread(target=self._guard, args=(loop,), daemon=True) for loop in loops]
        for thread in threads:
            thread.start()
        return threads

    def _guard(self, loop) -> None:
        try:
            loop()
        except Exception as error:  # report and stop the node instead of dying silently
            if not self.stop.is_set():
                self.node.get_logger().fatal(f"{loop.__name__} failed: {error!r}")
                self.stop.set()

    def _imu_loop(self) -> None:
        last = -1
        with AirSimRpc(self.host, self.port) as rpc:
            while not self.stop.is_set():
                data = rpc.call("getImuData", "", self.vehicle)
                t = data["time_stamp"]
                if t == last:
                    time.sleep(0.0002)
                    continue
                last = t
                header_stamp = stamp(t)
                if self.clock_pub:
                    self.clock_pub.publish(Clock(clock=header_stamp))
                gyro = vector(data["angular_velocity"])
                accel = vector(data["linear_acceleration"])
                for publisher, frame_id, rotation in self.imus:
                    msg = Imu()
                    msg.header.stamp = header_stamp
                    msg.header.frame_id = frame_id
                    msg.orientation_covariance[0] = -1.0  # orientation not provided
                    w = rotation @ gyro
                    a = rotation @ accel
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = w
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
                    publisher.publish(msg)

    def _camera_loop(self) -> None:
        requests = [{"camera_name": camera, "image_type": IMAGE_TYPES[kind],
                     "pixels_as_float": kind != "scene", "compress": False}
                    for camera, kind in self.captures]
        period = 1.0 / self.max_capture_rate_hz if self.max_capture_rate_hz > 0 else 0.0
        with AirSimRpc(self.host, self.port) as rpc:
            while not self.stop.is_set():
                started = time.monotonic()
                responses = rpc.call("simGetImages", requests, self.vehicle, False)
                if len(responses) != len(requests) or not all(r["width"] for r in responses):
                    continue
                header_stamp = stamp(responses[0]["time_stamp"] - self.latency_ns)
                images = dict(zip(self.captures, responses))
                for stream in self.streams:
                    self._publish_image(stream, images[stream.capture], header_stamp)
                if period:
                    time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _publish_image(self, stream: Stream, response: dict, header_stamp: Time) -> None:
        width, height = response["width"], response["height"]
        msg = Image()
        msg.header.stamp = header_stamp
        msg.header.frame_id = stream.frame_id
        msg.width, msg.height = width, height
        if stream.capture[1] == "scene":
            bgr = np.frombuffer(response["image_data_uint8"], dtype=np.uint8).reshape(height, width, 3)
            if stream.encoding == "rgb8":
                pixels = bgr[:, :, ::-1]
            elif stream.encoding == "bgr8":
                pixels = bgr
            elif stream.encoding == "mono8":
                pixels = (bgr @ np.array([0.114, 0.587, 0.299])).astype(np.uint8)
            else:
                raise ValueError(f"unsupported colour encoding {stream.encoding!r}")
        else:
            pixels = np.asarray(response["image_data_float"], dtype=np.float32).reshape(height, width)
            if stream.max_depth_m is not None:
                pixels = np.where(pixels > stream.max_depth_m, np.float32(np.inf), pixels)
        pixels = np.ascontiguousarray(pixels)
        msg.encoding = stream.encoding
        msg.step = pixels.strides[0]
        msg.data = pixels.tobytes()
        self.image_pubs[stream].publish(msg)
        self.info_pubs[stream].publish(self._camera_info(stream, width, height, header_stamp))

    def _camera_info(self, stream: Stream, width: int, height: int, header_stamp: Time) -> CameraInfo:
        fov = math.radians(self.fov_deg.get(stream.capture[0], 90.0))
        f = width / 2.0 / math.tan(fov / 2.0)
        cx, cy = width / 2.0, height / 2.0
        info = CameraInfo()
        info.header.stamp = header_stamp
        info.header.frame_id = stream.frame_id
        info.width, info.height = width, height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _truth_loop(self) -> None:
        publisher, world_frame, body_frame, rate_hz = self.truth
        with AirSimRpc(self.host, self.port) as rpc:
            while not self.stop.is_set():
                k = rpc.call("simGetGroundTruthKinematics", self.vehicle)
                t = rpc.call("getImuData", "", self.vehicle)["time_stamp"]
                r_enu_flu = R_ENU_NED @ rotation_from_quaternion(k["orientation"]) @ R_FRD_FLU
                msg = Odometry()
                msg.header.stamp = stamp(t)
                msg.header.frame_id = world_frame
                msg.child_frame_id = body_frame
                p = R_ENU_NED @ vector(k["position"])
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p
                o = msg.pose.pose.orientation
                o.x, o.y, o.z, o.w = quaternion_from_rotation(r_enu_flu)
                v = r_enu_flu.T @ (R_ENU_NED @ vector(k["linear_velocity"]))  # body frame, as ROS odometry
                w = R_FRD_FLU @ vector(k["angular_velocity"])
                msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = v
                msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = w
                publisher.publish(msg)
                time.sleep(1.0 / rate_hz)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", type=Path, required=True, help="bridge YAML configuration")
    args, ros_args = parser.parse_known_args()
    config = yaml.safe_load(args.config.read_text())

    rclpy.init(args=ros_args)
    node = rclpy.create_node(config.get("node_name", "carlaair_px4_sensor_bridge"),
                             namespace=config.get("namespace", ""))
    bridge = SensorBridge(node, config)
    bridge.start()
    node.get_logger().info(
        f"bridging AirSim vehicle '{bridge.vehicle}': {len(bridge.imus)} IMU topic(s), "
        f"{len(bridge.streams)} image topic(s) from {len(bridge.captures)} capture(s)")
    try:
        while rclpy.ok() and not bridge.stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    failed = bridge.stop.is_set()
    bridge.stop.set()
    node.destroy_node()
    rclpy.try_shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
