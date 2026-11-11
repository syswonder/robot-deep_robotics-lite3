#!/usr/bin/env python3
"""Minimal terminal teleop for mapping the Lite3 simulation."""

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist


HELP = """
Lite3 mapping teleop
  w/s  forward/backward
  a/d  turn left/right
  x    stop
  q    stop and quit
Each command stays active until the next key. Keep this terminal focused.
"""


def command(key: str) -> tuple[float, float] | None:
    return {
        "w": (0.20, 0.0),
        "s": (-0.12, 0.0),
        "a": (0.0, 0.55),
        "d": (0.0, -0.55),
        "x": (0.0, 0.0),
    }.get(key)


def publish(pub, linear: float, angular: float) -> None:
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    pub.publish(msg)


def main() -> int:
    if not sys.stdin.isatty():
        print("teleop requires an interactive terminal", file=sys.stderr)
        return 2
    rclpy.init()
    node = rclpy.create_node("lite3_mapping_teleop")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    old = termios.tcgetattr(sys.stdin)
    linear = angular = 0.0
    print(HELP, flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if readable:
                key = sys.stdin.read(1).lower()
                if key == "q":
                    break
                next_command = command(key)
                if next_command is not None:
                    linear, angular = next_command
                    print(
                        f"cmd_vel linear={linear:+.2f} angular={angular:+.2f}",
                        flush=True,
                    )
            publish(pub, linear, angular)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        for _ in range(3):
            publish(pub, 0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
