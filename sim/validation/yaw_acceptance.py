#!/usr/bin/env python3
"""Measure physical yaw-rate tracking, direction changes, and final stop."""

import argparse
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger


class YawAcceptance(Node):
    def __init__(self):
        super().__init__("lite3_yaw_acceptance")
        self.poses = []
        self.commands = []
        self.policy = []
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Twist, "/lite3_sim/commanded_velocity",
                                 lambda msg: self.commands.append((time.monotonic(), msg.angular.z)), 10)
        self.create_subscription(Twist, "/lite3_sim/policy_velocity",
                                 lambda msg: self.policy.append((time.monotonic(), msg.angular.z)), 10)
        self.reset_client = self.create_client(Trigger, "/lite3_sim/reset")
        self.stop_client = self.create_client(Trigger, "/lite3_sim/emergency_stop")

    def on_odom(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        tilt = math.acos(float(np.clip(1-2*(q.x*q.x + q.y*q.y), -1, 1)))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        self.poses.append((time.monotonic(), stamp, p.x, p.y, p.z, yaw,
                           message.twist.twist.angular.z, tilt))

    def drive(self, duration, angular, linear=0.0, check_posture=True):
        end = time.monotonic()+duration
        next_publish = 0.0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_publish:
                message = Twist()
                message.linear.x = linear
                message.angular.z = angular
                self.publisher.publish(message)
                next_publish = now+0.04
            rclpy.spin_once(self, timeout_sec=0.005)
            if check_posture and self.poses:
                pose = self.poses[-1]
                if (not all(math.isfinite(v) for v in pose)
                        or pose[4] < 0.25 or pose[7] > 0.35):
                    raise RuntimeError("robot posture unsafe during yaw acceptance")

    def reset_pose(self):
        # Leave RL control before resetting into a crouched stand-up pose.
        for client in (self.stop_client, self.reset_client):
            if not client.wait_for_service(timeout_sec=5):
                raise RuntimeError("posture service unavailable")
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic()+5
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not future.done() or not future.result().success:
                raise RuntimeError("posture service failed")
            if client is self.stop_client:
                # The vendor damping state must complete its 3 s simulation
                # timer before /clock is reset, otherwise it stays latched.
                self.drive(3.5, 0.0, check_posture=False)
        self.poses.clear()
        self.drive(7.0, 0.0, check_posture=False)

    def measure(self, start):
        selected = np.asarray([p for p in self.poses if p[0] >= start])
        if len(selected) < 100:
            raise RuntimeError("insufficient odometry samples")
        elapsed = selected[-1, 1]-selected[0, 1]
        if elapsed <= 0 or np.any(np.diff(selected[:, 1]) <= 0):
            raise RuntimeError("simulation time reset or stopped during measurement")
        yaw = np.unwrap(selected[:, 5])
        command = [value for wall, value in self.commands if wall >= start]
        policy = [value for wall, value in self.policy if wall >= start]
        if not command or not policy:
            raise RuntimeError("missing command or policy telemetry")
        return {
            "yaw_rate": float((yaw[-1]-yaw[0])/elapsed),
            "gyro_mean": float(np.mean(selected[:, 6])),
            "gyro_std": float(np.std(selected[:, 6])),
            "rtf": float(elapsed/(selected[-1, 0]-selected[0, 0])),
            "command_mean": float(np.mean(command)),
            "policy_peak": float(np.max(np.abs(policy))),
            "min_height": float(np.min(selected[:, 4])),
            "max_tilt": float(np.max(selected[:, 7])),
            "sim_seconds": float(elapsed),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--values", default="0.01,-0.01,0.02,-0.02,0.05,-0.05,0.1,-0.1,0.2,-0.2,0.35,-0.35,0.5,-0.5")
    parser.add_argument("--settle-s", type=float, default=10.0)
    parser.add_argument("--sample-s", type=float, default=12.0)
    parser.add_argument("--reset-between", action="store_true",
                        help="reset pose and /clock between cases; use without mapping/Nav2")
    parser.add_argument("--linear", type=float, default=0.0,
                        help="optional forward velocity for moving-turn checks")
    args = parser.parse_args()
    values = [float(value) for value in args.values.split(",")]
    if (not values or not all(math.isfinite(value) and 0 < abs(value) <= 0.5 for value in values)
            or not math.isfinite(args.settle_s) or args.settle_s < 0
            or not math.isfinite(args.sample_s) or args.sample_s < 5
            or not math.isfinite(args.linear) or abs(args.linear) > 0.15):
        parser.error("use finite nonzero targets within ±0.5, sample >= 5s and |linear| <= 0.15")
    rclpy.init(args=[])
    node = YawAcceptance()
    failures, results = [], []
    try:
        node.drive(7.0, 0.0, check_posture=False)
        for target in values:
            if args.reset_between:
                node.reset_pose()
            node.drive(args.settle_s, target, args.linear)
            start = time.monotonic()
            node.drive(args.sample_s, target, args.linear)
            result = node.measure(start)
            result["target"] = target
            result["error"] = result["yaw_rate"]-target
            tolerance = max(0.003, 0.05*abs(target))
            result["tolerance"] = tolerance
            if abs(result["error"]) > tolerance:
                failures.append(f"yaw {target:+.3f}: measured {result['yaw_rate']:+.4f}")
            if result["yaw_rate"] * target <= 0 or abs(result["yaw_rate"]) < 0.5*abs(target):
                failures.append(f"yaw {target:+.3f}: missing or reversed low-speed response")
            if abs(result["command_mean"]-target) > 1e-5:
                failures.append(f"yaw {target:+.3f}: another command publisher or unexpected clipping")
            if result["rtf"] < 0.85 or result["policy_peak"] > 0.80001:
                failures.append(f"yaw {target:+.3f}: RTF or policy envelope violated")
            stop_start = time.monotonic()
            node.drive(3.0, 0.0)
            stopped = node.measure(stop_start+1.0)
            result["stop_yaw_rate"] = stopped["yaw_rate"]
            if abs(stopped["yaw_rate"]) > 0.015 or stopped["policy_peak"] > 1e-6:
                failures.append(f"yaw {target:+.3f}: did not stop cleanly")
            results.append(result)
            print(json.dumps(result), flush=True)
        # Stop publishing entirely: the command watchdog must also clear the
        # feedback state, not merely stop after an explicit zero Twist.
        node.drive(3.0, 0.2)
        expired_start = time.monotonic()
        end = expired_start+3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        expired = node.measure(expired_start+1)
        if abs(expired["yaw_rate"]) > 0.015 or expired["policy_peak"] > 1e-6:
            failures.append("command watchdog left a residual turning command")
        print("WATCHDOG", json.dumps(expired), flush=True)
    finally:
        for _ in range(5):
            node.publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()
    if failures:
        for failure in failures:
            print("FAIL:", failure)
        return 1
    print("PASS: physical yaw tracking, posture, policy limits, zero command, and watchdog")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
