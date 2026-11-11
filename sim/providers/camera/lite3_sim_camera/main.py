#!/usr/bin/env python3
"""Register the existing MuJoCo RGB stream without starting a driver."""

from __future__ import annotations

import logging
import os
import subprocess
import time

from robonix_api import Deferred, Ok, Primitive


logging.basicConfig(
    level=os.environ.get("LITE3_SIM_CAMERA_LOG_LEVEL", "INFO"),
    format="[lite3-sim-camera] %(message)s",
)
log = logging.getLogger("lite3_sim_camera")

camera = Primitive(id="lite3_sim_camera", namespace="robonix/primitive/camera")


def wait_for_message(topic: str, message_type: str, timeout_s: float) -> tuple[bool, str]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = max(0.1, deadline - time.monotonic())
        try:
            discovered = subprocess.run(
                ["ros2", "topic", "type", topic],
                capture_output=True,
                text=True,
                timeout=min(3.0, remaining),
                check=False,
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
            capture_output=True,
            text=True,
            timeout=max(0.1, deadline - time.monotonic()),
            check=False,
        )
    except subprocess.TimeoutExpired:
        return False, f"{topic} exists but did not publish within {timeout_s:.1f}s"
    if active.returncode != 0:
        detail = (active.stderr or active.stdout).strip()
        return False, f"{topic} activity check failed: {detail}"
    return True, ""


@camera.on_init
def init(cfg: dict):
    rgb_topic = str(cfg.get("rgb_topic", "/camera/color/image_raw"))
    intrinsics_topic = str(
        cfg.get("intrinsics_topic", "/camera/color/camera_info"))
    timeout_s = float(cfg.get("sentinel_timeout_s", 30.0))
    for topic, message_type in (
        (rgb_topic, "sensor_msgs/msg/Image"),
        (intrinsics_topic, "sensor_msgs/msg/CameraInfo"),
    ):
        ok, reason = wait_for_message(topic, message_type, timeout_s)
        if not ok:
            return Deferred(reason)
        log.info("active topic verified: %s [%s]", topic, message_type)

    camera.declare_ros2_topic(
        "robonix/primitive/camera/rgb", topic=rgb_topic, qos="best_effort")
    camera.declare_ros2_topic(
        "robonix/primitive/camera/intrinsics",
        topic=intrinsics_topic,
        qos="best_effort",
    )
    log.info(
        "registered RGB=%s intrinsics=%s",
        rgb_topic,
        intrinsics_topic,
    )
    return Ok()


@camera.on_shutdown
def shutdown():
    return Ok()


if __name__ == "__main__":
    camera.run()
