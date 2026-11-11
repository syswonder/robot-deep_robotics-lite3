#!/usr/bin/env python3
"""End-to-end ROS 2 acceptance test for the Lite3 MuJoCo bridge."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from tf2_msgs.msg import TFMessage


EXPECTED_JOINTS = [
    f"{leg}_{joint}_joint"
    for leg in ("FL", "FR", "HL", "HR")
    for joint in ("HipX", "HipY", "Knee")
]


class Acceptance(Node):
    def __init__(self) -> None:
        super().__init__("lite3_bridge_acceptance")
        self.counts = {name: 0 for name in ("clock", "joint", "imu", "odom", "tf")}
        self.errors: list[str] = []
        self.clock_values: list[int] = []
        self.nonzero_command_seen = False
        self.zero_after_command_seen = False
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        self.create_subscription(
            JointState, "/joint_states", self.on_joint, qos_profile_sensor_data)
        self.create_subscription(
            Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.create_subscription(Twist, "/lite3_sim/commanded_velocity",
                                 self.on_effective_command, 10)

    def on_clock(self, message: Clock) -> None:
        self.counts["clock"] += 1
        value = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if self.clock_values and value <= self.clock_values[-1]:
            self.errors.append("/clock is not strictly increasing")
        self.clock_values.append(value)

    def on_joint(self, message: JointState) -> None:
        self.counts["joint"] += 1
        if list(message.name) != EXPECTED_JOINTS:
            self.errors.append("joint name/order mismatch")
        values = list(message.position) + list(message.velocity) + list(message.effort)
        if len(message.position) != 12 or not all(math.isfinite(v) for v in values):
            self.errors.append("invalid joint state")

    def on_imu(self, message: Imu) -> None:
        self.counts["imu"] += 1
        if message.header.frame_id != "base_link":
            self.errors.append("invalid IMU frame")

    def on_odom(self, message: Odometry) -> None:
        self.counts["odom"] += 1
        if message.header.frame_id != "odom" or message.child_frame_id != "base_link":
            self.errors.append("invalid odometry frames")
        position = message.pose.pose.position
        if not all(math.isfinite(v) for v in (position.x, position.y, position.z)):
            self.errors.append("invalid odometry position")

    def on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (transform.header.frame_id == "odom" and
                    transform.child_frame_id == "base_link"):
                self.counts["tf"] += 1

    def on_effective_command(self, message: Twist) -> None:
        magnitude = abs(message.linear.x) + abs(message.linear.y) + abs(message.angular.z)
        if magnitude > 0.01:
            self.nonzero_command_seen = True
        elif self.nonzero_command_seen:
            self.zero_after_command_seen = True


def main() -> None:
    rclpy.init()
    node = Acceptance()
    started = time.monotonic()
    try:
        while time.monotonic() - started < 4.0:
            elapsed = time.monotonic() - started
            if 0.8 < elapsed < 1.8:
                command = Twist()
                command.linear.x = 0.12
                command.linear.y = -0.05
                command.angular.z = 0.20
                node.publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.03)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    for topic, count in node.counts.items():
        if count < 20:
            node.errors.append(f"insufficient {topic} samples: {count}")
    if not node.nonzero_command_seen:
        node.errors.append("/cmd_vel was not forwarded")
    if not node.zero_after_command_seen:
        node.errors.append("command watchdog did not return velocity to zero")
    if node.errors:
        print("\n".join(sorted(set(node.errors))), file=sys.stderr)
        raise SystemExit(1)
    print(f"BRIDGE_ACCEPTANCE_OK counts={node.counts}")


if __name__ == "__main__":
    main()
