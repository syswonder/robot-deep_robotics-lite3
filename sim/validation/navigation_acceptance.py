#!/usr/bin/env python3
"""End-to-end mapping and short-goal Nav2 acceptance for Lite3 simulation."""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class Acceptance(Node):
    def __init__(self) -> None:
        super().__init__("lite3_navigation_acceptance")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_message: OccupancyGrid | None = None
        self.odom_messages: list[Odometry] = []
        self.commands: list[tuple[float, float, float]] = []
        self.create_subscription(OccupancyGrid, "/map", self._map, map_qos)
        self.create_subscription(Odometry, "/odom", self.odom_messages.append,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._command, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _map(self, message: OccupancyGrid) -> None:
        self.map_message = message

    def _command(self, message: Twist) -> None:
        self.commands.append((time.monotonic(), message.linear.x, message.angular.z))


def spin_until(node: Node, predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    return False


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> int:
    rclpy.init()
    node = Acceptance()
    failures: list[str] = []

    ready = spin_until(
        node,
        lambda: node.map_message is not None and bool(node.odom_messages),
        45.0,
    )
    if not ready:
        failures.append("/map or /odom did not become ready")

    transform = None
    tf_ready = spin_until(
        node,
        lambda: (
            node.tf_buffer.can_transform("map", "base_link", Time())
            and node.tf_buffer.can_transform("odom", "base_link", Time())
            and node.tf_buffer.can_transform("map", "odom", Time())
        ),
        15.0,
    )
    if tf_ready:
        transform = node.tf_buffer.lookup_transform("map", "base_link", Time())
    else:
        failures.append("required TF chain did not become ready")

    known = occupied = 0
    if node.map_message is not None:
        values = np.asarray(node.map_message.data, dtype=np.int16)
        known = int(np.count_nonzero(values >= 0))
        occupied = int(np.count_nonzero(values >= 50))
        if node.map_message.header.frame_id != "map":
            failures.append(f"map frame is {node.map_message.header.frame_id!r}, expected 'map'")
        if known < 100:
            failures.append(f"occupancy grid has too few known cells: {known}")
        if occupied < 1:
            failures.append("occupancy grid has no occupied cells")

    if not node.nav.wait_for_server(timeout_sec=30.0):
        failures.append("navigate_to_pose action server unavailable")

    start_xy = None
    end_xy = None
    end_height = None
    result_status = 0
    nonzero_commands = 0
    if not failures and transform is not None and node.odom_messages:
        start_pose = node.odom_messages[-1].pose.pose.position
        start_xy = np.array([start_pose.x, start_pose.y])
        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = yaw_from_quaternion(q)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal_distance = float(os.environ.get("LITE3_NAV_GOAL_DISTANCE", "0.60"))
        goal_bearing = math.radians(
            float(os.environ.get("LITE3_NAV_GOAL_BEARING_DEG", "30.0")))
        goal_yaw = yaw + goal_bearing
        goal.pose.pose.position.x = t.x + goal_distance * math.cos(goal_yaw)
        goal.pose.pose.position.y = t.y + goal_distance * math.sin(goal_yaw)
        goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

        send_future = node.nav.send_goal_async(goal)
        if not spin_until(node, send_future.done, 15.0):
            failures.append("navigation goal was not acknowledged")
        else:
            goal_handle = send_future.result()
            if not goal_handle.accepted:
                failures.append("navigation goal was rejected")
            else:
                result_future = goal_handle.get_result_async()
                if not spin_until(node, result_future.done, 90.0):
                    goal_handle.cancel_goal_async()
                    failures.append("navigation goal timed out")
                else:
                    result_status = result_future.result().status
                    if result_status != GoalStatus.STATUS_SUCCEEDED:
                        failures.append(f"navigation result status={result_status}, expected SUCCEEDED")

        if node.odom_messages:
            end_pose = node.odom_messages[-1].pose.pose.position
            end_xy = np.array([end_pose.x, end_pose.y])
            end_height = float(end_pose.z)
        nonzero_commands = sum(
            1 for _, linear, angular in node.commands
            if abs(linear) > 0.02 or abs(angular) > 0.02
        )
        if nonzero_commands < 2:
            failures.append("Nav2 did not emit sustained non-zero /cmd_vel")
        if end_xy is None or float(np.linalg.norm(end_xy - start_xy)) < 0.20:
            failures.append("robot odometry moved less than 0.20 m")

        quiet_start = time.monotonic()
        spin_until(node, lambda: False, 1.5)
        tail = [item for item in node.commands if item[0] >= quiet_start]
        if not tail or abs(tail[-1][1]) > 0.02 or abs(tail[-1][2]) > 0.02:
            failures.append("/cmd_vel did not return to zero after navigation")
        if end_height is None or end_height < 0.25:
            failures.append(f"robot base height dropped below standing threshold: {end_height}")

    displacement = (float(np.linalg.norm(end_xy - start_xy))
                    if start_xy is not None and end_xy is not None else 0.0)
    print("Navigation acceptance")
    print(f"  map known cells:       {known}")
    print(f"  map occupied cells:    {occupied}")
    print(f"  non-zero cmd samples:  {nonzero_commands}")
    print(f"  odom displacement:     {displacement:.3f} m")
    print(f"  final base height:     {end_height if end_height is not None else 0.0:.3f} m")
    print(f"  navigation status:     {result_status}")
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: mapping, TF, Nav2 motion, goal completion, and stop")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
