#!/usr/bin/env python3
"""End-to-end acceptance test for the complete simulated sensor contract."""

from __future__ import annotations

import math
import sys
import time
from collections import defaultdict

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2
from tf2_msgs.msg import TFMessage


TOPICS = {
    "/clock": Clock,
    "/imu/data": Imu,
    "/odom": Odometry,
    "/scanner/scan": LaserScan,
    "/scanner/cloud": PointCloud2,
    "/camera/color/image_raw": Image,
    "/camera/depth/image_raw": Image,
    "/camera/color/camera_info": CameraInfo,
    "/camera/depth/camera_info": CameraInfo,
}


class SensorAcceptance(Node):
    def __init__(self) -> None:
        super().__init__("lite3_sensor_acceptance")
        self.arrivals: dict[str, list[float]] = defaultdict(list)
        self.messages: dict[str, object] = {}
        self.clock_values: list[tuple[float, float]] = []
        self.odom_positions: list[tuple[float, np.ndarray]] = []
        self.gyro_norms: list[tuple[float, float]] = []
        self.base_tilts: list[tuple[float, float]] = []
        self.static_children: set[str] = set()
        for topic, message_type in TOPICS.items():
            self.create_subscription(
                message_type, topic,
                lambda message, name=topic: self._receive(name, message),
                qos_profile_sensor_data,
            )
        static_qos = QoSProfile(depth=1)
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(TFMessage, "/tf_static", self._receive_static, static_qos)

    def _receive(self, topic: str, message: object) -> None:
        now = time.monotonic()
        self.arrivals[topic].append(now)
        self.messages[topic] = message
        if topic == "/clock":
            clock = message.clock
            self.clock_values.append((now, clock.sec + clock.nanosec * 1e-9))
        elif topic == "/odom":
            p = message.pose.pose.position
            self.odom_positions.append((now, np.array([p.x, p.y, p.z])))
        elif topic == "/imu/data":
            w = message.angular_velocity
            self.gyro_norms.append((now, math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)))
            q = message.orientation
            tilt = math.acos(float(np.clip(1.0 - 2.0 * (q.x*q.x + q.y*q.y), -1.0, 1.0)))
            self.base_tilts.append((now, tilt))

    def _receive_static(self, message: TFMessage) -> None:
        self.static_children.update(item.child_frame_id for item in message.transforms)


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def rate(times: list[float], start: float) -> float:
    selected = [value for value in times if value >= start]
    if len(selected) < 2:
        return 0.0
    return (len(selected) - 1) / (selected[-1] - selected[0])


def main() -> int:
    rclpy.init()
    node = SensorAcceptance()
    warmup_end = time.monotonic() + 7.0
    while rclpy.ok() and time.monotonic() < warmup_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    sample_start = time.monotonic()
    sample_end = sample_start + 30.0
    while rclpy.ok() and time.monotonic() < sample_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    failures: list[str] = []
    minimum_rates = {
        "/clock": 70.0,
        "/imu/data": 70.0,
        "/odom": 70.0,
        "/scanner/scan": 7.0,
        "/scanner/cloud": 7.0,
        "/camera/color/image_raw": 7.0,
        "/camera/depth/image_raw": 7.0,
        "/camera/color/camera_info": 7.0,
        "/camera/depth/camera_info": 7.0,
    }
    measured_rates = {topic: rate(node.arrivals[topic], sample_start) for topic in TOPICS}
    for topic, minimum in minimum_rates.items():
        require(measured_rates[topic] >= minimum,
                f"{topic} rate {measured_rates[topic]:.1f} Hz < {minimum:.1f} Hz", failures)

    scan = node.messages.get("/scanner/scan")
    require(isinstance(scan, LaserScan), "missing LaserScan", failures)
    if isinstance(scan, LaserScan):
        finite_ranges = np.asarray(scan.ranges)[np.isfinite(scan.ranges)]
        require(scan.header.frame_id == "livox_frame", "LaserScan frame mismatch", failures)
        require(len(scan.ranges) == 360, "LaserScan must contain 360 rays", failures)
        require(finite_ranges.size > 0, "LaserScan contains no finite returns", failures)
        require(bool(np.all((finite_ranges >= scan.range_min) & (finite_ranges <= scan.range_max))),
                "LaserScan ranges violate declared bounds", failures)

    cloud = node.messages.get("/scanner/cloud")
    require(isinstance(cloud, PointCloud2), "missing PointCloud2", failures)
    if isinstance(cloud, PointCloud2):
        require(cloud.header.frame_id == "livox_frame", "PointCloud2 frame mismatch", failures)
        require(cloud.width > 0 and cloud.point_step == 12, "PointCloud2 layout/size invalid", failures)

    color = node.messages.get("/camera/color/image_raw")
    depth = node.messages.get("/camera/depth/image_raw")
    require(isinstance(color, Image), "missing RGB image", failures)
    require(isinstance(depth, Image), "missing depth image", failures)
    if isinstance(color, Image):
        require((color.width, color.height, color.encoding, color.header.frame_id) ==
                (320, 180, "rgb8", "camera_color_optical_frame"),
                "RGB image dimensions/encoding/frame mismatch", failures)
        require(len(color.data) == color.step * color.height, "RGB image payload size mismatch", failures)
    if isinstance(depth, Image):
        require((depth.width, depth.height, depth.encoding, depth.header.frame_id) ==
                (320, 180, "32FC1", "camera_depth_optical_frame"),
                "depth image dimensions/encoding/frame mismatch", failures)
        values = np.frombuffer(depth.data, dtype=np.float32)
        require(values.size == depth.width * depth.height, "depth image payload size mismatch", failures)
        require(bool(np.isfinite(values).any() and np.nanmax(values) > 0.1),
                "depth image has no positive finite measurements", failures)

    for topic, expected_frame in (
        ("/camera/color/camera_info", "camera_color_optical_frame"),
        ("/camera/depth/camera_info", "camera_depth_optical_frame"),
    ):
        info = node.messages.get(topic)
        require(isinstance(info, CameraInfo), f"missing {topic}", failures)
        if isinstance(info, CameraInfo):
            require(info.header.frame_id == expected_frame and info.k[0] > 0.0 and info.k[4] > 0.0,
                    f"{topic} frame/intrinsics invalid", failures)

    expected_static = {
        "livox_frame", "camera_bottom_screw_frame", "camera_link",
        "camera_depth_frame", "camera_depth_optical_frame",
        "camera_color_frame", "camera_color_optical_frame",
    }
    missing_static = expected_static - node.static_children
    require(not missing_static, f"missing static TF frames: {sorted(missing_static)}", failures)

    clocks = [(wall, sim) for wall, sim in node.clock_values if wall >= sample_start]
    window_rtfs: list[float] = []
    if len(clocks) >= 2:
        rtf = (clocks[-1][1] - clocks[0][1]) / (clocks[-1][0] - clocks[0][0])
    else:
        rtf = 0.0
    require(rtf >= 0.85,
            f"real-time factor {rtf:.3f} < 0.85", failures)
    if len(clocks) >= 2:
        for offset in (0.0, 10.0, 20.0):
            window = [(wall, sim) for wall, sim in clocks
                      if sample_start + offset <= wall < sample_start + offset + 10.0]
            if len(window) < 2:
                require(False, f"insufficient /clock samples in RTF window {offset:.0f}s", failures)
                continue
            window_rtf = ((window[-1][1] - window[0][1]) /
                          (window[-1][0] - window[0][0]))
            window_rtfs.append(window_rtf)
            require(window_rtf >= 0.85,
                    f"RTF window {offset:.0f}s: {window_rtf:.3f} < 0.85", failures)

    recent_odom = [position for wall, position in node.odom_positions if wall >= sample_start + 3.0]
    recent_gyro = [value for wall, value in node.gyro_norms if wall >= sample_start + 3.0]
    recent_tilts = [value for wall, value in node.base_tilts if wall >= sample_start + 3.0]
    if recent_odom:
        drift = float(np.linalg.norm(np.max(recent_odom, axis=0) - np.min(recent_odom, axis=0)))
        min_height = float(np.min(np.asarray(recent_odom)[:, 2]))
    else:
        drift = math.inf
        min_height = -math.inf
    gyro_p95 = float(np.percentile(recent_gyro, 95)) if recent_gyro else math.inf
    max_tilt = float(np.max(recent_tilts)) if recent_tilts else math.inf
    require(drift < 0.08, f"stationary odom drift {drift:.3f} m >= 0.08 m", failures)
    require(gyro_p95 < 0.50, f"stationary gyro p95 {gyro_p95:.3f} rad/s >= 0.50", failures)
    require(min_height >= 0.25, f"standing base height {min_height:.3f} m < 0.25 m", failures)
    require(max_tilt < 0.35, f"standing base tilt {max_tilt:.3f} rad >= 0.35 rad", failures)

    print("Simulated sensor acceptance")
    for topic in TOPICS:
        print(f"  {topic:34s} {measured_rates[topic]:6.1f} Hz")
    print(f"  real-time factor (30s sample)        {rtf:7.4f}")
    print(f"  real-time factor (10s windows)       {', '.join(f'{value:.4f}' for value in window_rtfs)}")
    print(f"  stationary odom span                {drift:6.3f} m")
    print(f"  stationary gyro p95                 {gyro_p95:6.3f} rad/s")
    print(f"  standing minimum height             {min_height:6.3f} m")
    print(f"  standing maximum tilt               {max_tilt:6.3f} rad")
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        status = 1
    else:
        print("PASS: sensor topics, frames, payloads, rates, and stability")
        status = 0
    node.destroy_node()
    rclpy.shutdown()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
