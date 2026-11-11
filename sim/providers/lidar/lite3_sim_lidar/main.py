#!/usr/bin/env python3
"""Register existing MuJoCo lidar topics without starting hardware drivers."""

from __future__ import annotations

import logging
import os
import subprocess
import time

from robonix_api import Deferred, Ok, Primitive


logging.basicConfig(
    level=os.environ.get("LITE3_SIM_LIDAR_LOG_LEVEL", "INFO"),
    format="[lite3-sim-lidar] %(message)s",
)
log = logging.getLogger("lite3_sim_lidar")

lidar = Primitive(id="lite3_sim_lidar", namespace="robonix/primitive/lidar")


def wait_for_message(topic: str, message_type: str, timeout_s: float) -> tuple[bool, str]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = max(0.1, deadline - time.monotonic())
        try:
            discovered = subprocess.run(
                ["ros2", "topic", "type", topic],
                capture_output=True, text=True, timeout=min(3.0, remaining), check=False,
            )
        except subprocess.TimeoutExpired:
            continue
        if discovered.returncode == 0 and discovered.stdout.strip() == message_type:
            break
        time.sleep(min(0.2, remaining))
    else:
        return False, f"{topic} [{message_type}] was not discovered"

    try:
        active = subprocess.run(
            ["ros2", "topic", "echo", "--once", topic, message_type],
            capture_output=True, text=True,
            timeout=max(0.1, deadline - time.monotonic()), check=False,
        )
    except subprocess.TimeoutExpired:
        return False, f"{topic} exists but did not publish within {timeout_s:.1f}s"
    if active.returncode != 0:
        detail = (active.stderr or active.stdout).strip()
        return False, f"{topic} activity check failed: {detail}"
    return True, ""


@lidar.on_init
def init(cfg: dict):
    scan_topic = str(cfg.get("scan_topic", "/scanner/scan"))
    cloud_topic = str(cfg.get("cloud_topic", "/scanner/cloud"))
    timeout_s = float(cfg.get("sentinel_timeout_s", 30.0))
    for topic, message_type in (
        (scan_topic, "sensor_msgs/msg/LaserScan"),
        (cloud_topic, "sensor_msgs/msg/PointCloud2"),
    ):
        ok, reason = wait_for_message(topic, message_type, timeout_s)
        if not ok:
            return Deferred(reason)
        log.info("active topic verified: %s [%s]", topic, message_type)

    lidar.declare_ros2_topic(
        "robonix/primitive/lidar/lidar", topic=scan_topic, qos="best_effort")
    lidar.declare_ros2_topic(
        "robonix/primitive/lidar/lidar3d", topic=cloud_topic, qos="best_effort")
    log.info("registered LaserScan=%s PointCloud2=%s", scan_topic, cloud_topic)
    return Ok()


@lidar.on_shutdown
def shutdown():
    return Ok()


if __name__ == "__main__":
    lidar.run()
