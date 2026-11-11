#!/usr/bin/env python3
"""ROS 2 bridge and headless MuJoCo runtime for the Lite3 simulator."""

from __future__ import annotations

import argparse
import io
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from array import array
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import mujoco
import numpy as np
import rclpy
from PIL import Image
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image as ImageMessage, Imu, JointState
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from yaw_velocity_controller import YawVelocityController


JOINT_NAMES = [
    f"{leg}_{joint}_joint"
    for leg in ("FL", "FR", "HL", "HR")
    for joint in ("HipX", "HipY", "Knee")
]
RUNTIME_DIR = Path(__file__).resolve().parent
WEB3D_ROOT = Path(os.environ.get(
    "LITE3_WEB3D_ROOT", str(RUNTIME_DIR.parent / "web3d")))
ENVIRONMENT = os.environ.get("LITE3_ENVIRONMENT", "scenesmith_house_187")
if os.environ.get("LITE3_MJCF"):
    MODEL_PATH = Path(os.environ["LITE3_MJCF"])
    SPAWN_XY = (0.0, 0.0)
else:
    environment_root = WEB3D_ROOT / "assets" / "environments" / ENVIRONMENT
    if not (environment_root / "scene.xml").is_file():
        raise RuntimeError(f"unknown Lite3 environment: {ENVIRONMENT}")
    spawn_x = os.environ.get("LITE3_SPAWN_X", "").strip()
    spawn_y = os.environ.get("LITE3_SPAWN_Y", "").strip()
    if bool(spawn_x) != bool(spawn_y):
        raise RuntimeError("LITE3_SPAWN_X and LITE3_SPAWN_Y must be set together")
    spawn_override = (float(spawn_x), float(spawn_y)) if spawn_x else None
    spawn_yaw = os.environ.get("LITE3_SPAWN_YAW", "").strip()
    yaw_override = float(spawn_yaw) if spawn_yaw else None
    from lite3_room_scene import build_room_scene
    MODEL_PATH, SPAWN_XY = build_room_scene(
        WEB3D_ROOT, ENVIRONMENT, spawn_override, yaw_override
    )
DT = 0.001
# Retain ordinary scheduling jitter, but do not replay an unbounded backlog
# after a suspended process or a long stall with an external RL controller.
MAX_CATCHUP_S = 0.05
LIDAR_RATE_HZ = 10.0
LIDAR_RAYS = 360
LIDAR_RANGE_MIN = 0.10
LIDAR_RANGE_MAX = 12.0
CAMERA_RATE_HZ = 10.0
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 180
CAMERA_FOV_Y_DEG = 58.0
STATE_ADDRESS = ("127.0.0.1", 30010)
MOTOR_PORT = 20001
COMMAND_ADDRESS = ("127.0.0.1", 30011)
STATE_HEADER = struct.Struct("<4sHHId")
STATE_FLOATS = struct.Struct("<45f")
MOTOR_PACKET = struct.Struct("<60f")

WAITING = 0
STANDING = 1
DAMPING = 2
LIE_DOWN = 3
RL_CONTROL = 6


def sim_time(seconds: float) -> Time:
    nanoseconds = max(0, int(round(seconds * 1_000_000_000)))
    return Time(sec=nanoseconds // 1_000_000_000,
                nanosec=nanoseconds % 1_000_000_000)


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return ROS quaternion order (x, y, z, w)."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Lite3Bridge(Node):
    def __init__(self, viewer: str = "headless", viewer_port: int = 8080) -> None:
        super().__init__("lite3_mujoco_bridge")
        self.declare_parameter("cmd_timeout_s", 0.3)
        self.declare_parameter("motor_command_timeout_s", 0.5)
        self.declare_parameter("auto_stand", True)
        self.declare_parameter("stand_duration_s", 5.0)
        self.declare_parameter("max_vx", 0.35)
        self.declare_parameter("max_vy", 0.20)
        self.declare_parameter("max_wz", 0.50)
        self.declare_parameter("policy_vx", 0.8)
        self.declare_parameter("policy_vy", 0.8)
        self.declare_parameter("policy_wz", 0.8)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self.motor_command_timeout = float(
            self.get_parameter("motor_command_timeout_s").value)
        self.max_velocity = np.array([
            self.get_parameter("max_vx").value,
            self.get_parameter("max_vy").value,
            self.get_parameter("max_wz").value,
        ], dtype=np.float64)
        self.policy_velocity = np.array([
            self.get_parameter("policy_vx").value,
            self.get_parameter("policy_vy").value,
            self.get_parameter("policy_wz").value,
        ], dtype=np.float64)

        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH.resolve()))
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)
        self.joint_qpos = []
        self.joint_dof = []
        for name in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError(f"missing joint: {name}")
            self.joint_qpos.append(int(self.model.jnt_qposadr[joint_id]))
            self.joint_dof.append(int(self.model.jnt_dofadr[joint_id]))
        self.root_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base")
        if self.root_joint < 0 or self.model.nu != 12:
            raise RuntimeError("unexpected Lite3 model contract")
        self.root_qpos = int(self.model.jnt_qposadr[self.root_joint])
        self.root_dof = int(self.model.jnt_dofadr[self.root_joint])

        self.motor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.motor_socket.bind(("0.0.0.0", MOTOR_PORT))
        self.motor_socket.settimeout(0.02)
        self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motor_lock = threading.Lock()
        self.motor_command = np.zeros(60, dtype=np.float32)
        self.last_motor_command = time.monotonic()
        self.running = True
        self.receiver = threading.Thread(target=self._receive_motor_commands,
                                         daemon=True)
        self.receiver.start()

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.joint_pub = self.create_publisher(
            JointState, "/joint_states", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(
            Imu, "/imu/data", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(
            Odometry, "/odom", 10)
        self.leg_odom_pub = self.create_publisher(
            Odometry, "/leg_odom2", 10)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scanner/scan", 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/scanner/cloud", qos_profile_sensor_data)
        self.color_pub = self.create_publisher(
            ImageMessage, "/camera/color/image_raw", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(
            ImageMessage, "/camera/depth/image_raw", qos_profile_sensor_data)
        self.color_info_pub = self.create_publisher(
            CameraInfo, "/camera/color/camera_info", qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data)
        self.command_pub = self.create_publisher(
            Twist, "/lite3_sim/commanded_velocity", 10)
        self.policy_command_pub = self.create_publisher(
            Twist, "/lite3_sim/policy_velocity", 10)
        self.yaw_controller = YawVelocityController(float(self.policy_velocity[2]))
        self.last_yaw_control_time = 0.0
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        # Nav2 and the Robonix chassis primitive publish reliable commands.
        # Match that QoS explicitly so host-network containers exchange data.
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_service(Trigger, "/lite3_sim/reset", self._reset)
        self.create_service(Trigger, "/lite3_sim/stand", self._stand)
        self.create_service(Trigger, "/lite3_sim/lie_down", self._lie_down)
        self.create_service(Trigger, "/lite3_sim/emergency_stop", self._stop)

        self.latest_cmd = np.zeros(3, dtype=np.float64)
        self.last_cmd_time = -math.inf
        self.target_mode = STANDING if self.get_parameter("auto_stand").value else WAITING
        self.stand_duration = float(self.get_parameter("stand_duration_s").value)
        self.sequence = 0
        self.start_wall = time.monotonic()
        self.state_sequence = 0
        self.reset_requested = False
        self.paused = False
        self.viewer = viewer
        self.viewer_port = viewer_port
        # EGL contexts are thread-affine; the sensor worker creates it.
        self.renderer = None
        self.sensor_data = mujoco.MjData(self.model)
        self.data_lock = threading.Lock()
        self.sensor_thread: threading.Thread | None = None
        self.livox_site = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "livox_frame")
        self.torso_body = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "TORSO")
        self.rgbd_camera = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "lite3_rgbd_camera")
        if min(self.livox_site, self.torso_body, self.rgbd_camera) < 0:
            raise RuntimeError("simulated sensor sites/camera are missing from Lite3 MJCF")
        self.ray_geomgroup = np.ones(6, dtype=np.uint8)
        self.ray_geomid = np.array([-1], dtype=np.int32)
        self.latest_frame: bytes | None = None
        self.httpd: ThreadingHTTPServer | None = None
        self.http_thread: threading.Thread | None = None
        self._initialize_pose()
        if self.viewer == "web":
            self._start_web_viewer()
        self._publish_static_transforms()
        self.sensor_thread = threading.Thread(
            target=self._sensor_loop, daemon=True)
        self.sensor_thread.start()
        self.get_logger().info(
            f"model={MODEL_PATH} environment={ENVIRONMENT} spawn={SPAWN_XY} "
            f"joints=12 dt={DT} motor_udp={MOTOR_PORT} "
            f"command_udp=30011 lidar={LIDAR_RATE_HZ:.0f}Hz "
            f"rgbd={CAMERA_WIDTH}x{CAMERA_HEIGHT}@{CAMERA_RATE_HZ:.0f}Hz")

    def _start_web_viewer(self) -> None:
        bridge = self

        class Handler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(WEB3D_ROOT), **kwargs)

            def do_GET(self) -> None:  # noqa: N802
                path = self.path.split("?", 1)[0]
                if path == "/api/state":
                    with bridge.data_lock:
                        payload = json.dumps({
                            "environment": ENVIRONMENT,
                            "simulationTime": float(bridge.data.time),
                            "qpos": bridge.data.qpos.tolist(),
                            "qvel": bridge.data.qvel.tolist(),
                            "paused": bool(bridge.paused),
                            "mode": {
                                WAITING: "waiting", STANDING: "standing",
                                DAMPING: "damping", LIE_DOWN: "lie_down",
                                RL_CONTROL: "rl_control",
                            }.get(bridge.target_mode, str(bridge.target_mode)),
                            "command": bridge.latest_cmd.tolist(),
                            "yawTracking": {
                                "target": bridge.yaw_controller.previous_target,
                                "filteredRate": bridge.yaw_controller.filtered_rate,
                                "integral": bridge.yaw_controller.integral,
                                "policyInput": bridge.yaw_controller.output,
                                "angleError": bridge.yaw_controller.angle_error,
                                "holding": bridge.yaw_controller.holding,
                            },
                        }, separators=(",", ":")).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                if path == "/frame.jpg" and bridge.latest_frame:
                    frame = bridge.latest_frame
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    return
                super().do_GET()

            def do_POST(self) -> None:  # noqa: N802
                action = self.path.removeprefix("/api/").split("?", 1)[0]
                ok, message = bridge.web_action(action)
                payload = message.encode("utf-8")
                self.send_response(200 if ok else 400)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def log_message(self, _format: str, *_args) -> None:
                return

        self.httpd = ThreadingHTTPServer(("0.0.0.0", self.viewer_port), Handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f"web viewer listening on 0.0.0.0:{self.viewer_port}")

    def web_action(self, action: str) -> tuple[bool, str]:
        if action == "stand":
            self.target_mode = STANDING
            self.start_wall = time.monotonic()
        elif action == "lie_down":
            self.target_mode = LIE_DOWN
            self.latest_cmd.fill(0.0)
        elif action == "reset":
            self.reset_requested = True
        elif action == "emergency_stop":
            self.target_mode = DAMPING
            self.latest_cmd.fill(0.0)
        elif action == "pause":
            self.paused = True
        elif action == "resume":
            self.paused = False
        else:
            return False, f"unknown action: {action}"
        return True, f"{action} accepted"

    def _publish_static_transforms(self, data=None) -> None:
        data = self.data if data is None else data
        stamp = sim_time(float(data.time))

        def transform(parent: str, child: str, xyz: tuple[float, float, float],
                      rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> TransformStamped:
            message = TransformStamped()
            message.header.stamp = stamp
            message.header.frame_id = parent
            message.child_frame_id = child
            (message.transform.translation.x,
             message.transform.translation.y,
             message.transform.translation.z) = xyz
            qx, qy, qz, qw = quaternion_from_rpy(*rpy)
            message.transform.rotation.x = qx
            message.transform.rotation.y = qy
            message.transform.rotation.z = qz
            message.transform.rotation.w = qw
            return message

        optical = (-math.pi / 2.0, 0.0, -math.pi / 2.0)
        transforms = [
            transform("base_link", "livox_frame", (0.045, 0.0, 0.10)),
            transform("base_link", "camera_bottom_screw_frame", (0.22, 0.0, 0.06)),
            transform("camera_bottom_screw_frame", "camera_link", (0.0148, 0.0175, 0.0125)),
            transform("camera_link", "camera_depth_frame", (0.0, 0.0, 0.0)),
            transform("camera_depth_frame", "camera_depth_optical_frame", (0.0, 0.0, 0.0), optical),
            transform("camera_link", "camera_color_frame", (0.0, 0.015, 0.0)),
            transform("camera_color_frame", "camera_color_optical_frame", (0.0, 0.0, 0.0), optical),
        ]
        self.static_tf.sendTransform(transforms)
        # Compatibility path for Fast DDS host-network containers where a
        # late-joining transient-local reader can miss /tf_static history.
        # The same Bridge remains the sole publisher of odom -> base_link.
        self.tf.sendTransform(transforms)

    def _camera_info(self, stamp: Time, frame: str) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame
        info.width = CAMERA_WIDTH
        info.height = CAMERA_HEIGHT
        fy = CAMERA_HEIGHT / (2.0 * math.tan(math.radians(CAMERA_FOV_Y_DEG) / 2.0))
        fx = fy
        cx = (CAMERA_WIDTH - 1.0) / 2.0
        cy = (CAMERA_HEIGHT - 1.0) / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _publish_camera(self, data=None) -> None:
        data = self.data if data is None else data
        stamp = sim_time(float(data.time))
        self.renderer.update_scene(data, camera="lite3_rgbd_camera")
        rgb = self.renderer.render().copy()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().astype(np.float32, copy=True)
        self.renderer.disable_depth_rendering()

        color = ImageMessage()
        color.header.stamp = stamp
        color.header.frame_id = "camera_color_optical_frame"
        color.height, color.width = CAMERA_HEIGHT, CAMERA_WIDTH
        color.encoding = "rgb8"
        color.is_bigendian = False
        color.step = CAMERA_WIDTH * 3
        # ROS validates bytes element by element in Python. A typed array
        # takes its fast path and avoids holding the GIL across an image.
        color.data = array("B", rgb.tobytes())
        self.color_pub.publish(color)

        depth_message = ImageMessage()
        depth_message.header.stamp = stamp
        depth_message.header.frame_id = "camera_depth_optical_frame"
        depth_message.height, depth_message.width = CAMERA_HEIGHT, CAMERA_WIDTH
        depth_message.encoding = "32FC1"
        depth_message.is_bigendian = False
        depth_message.step = CAMERA_WIDTH * 4
        depth_message.data = array("B", depth.tobytes())
        self.depth_pub.publish(depth_message)
        self.color_info_pub.publish(self._camera_info(stamp, color.header.frame_id))
        self.depth_info_pub.publish(self._camera_info(stamp, depth_message.header.frame_id))

        if self.viewer == "web":
            output = io.BytesIO()
            Image.fromarray(rgb).save(output, "JPEG", quality=80)
            self.latest_frame = output.getvalue()

    def _publish_lidar(self, data=None) -> None:
        data = self.data if data is None else data
        stamp = sim_time(float(data.time))
        origin = data.site_xpos[self.livox_site].copy()
        rotation = data.site_xmat[self.livox_site].reshape(3, 3)
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / LIDAR_RAYS
        ranges: list[float] = []
        points: list[tuple[float, float, float]] = []
        for index in range(LIDAR_RAYS):
            angle = angle_min + index * angle_increment
            local_direction = np.array([math.cos(angle), math.sin(angle), 0.0])
            world_direction = rotation @ local_direction
            distance = float(mujoco.mj_ray(
                self.model, data, origin, world_direction,
                self.ray_geomgroup, 1, self.torso_body, self.ray_geomid))
            if LIDAR_RANGE_MIN <= distance <= LIDAR_RANGE_MAX:
                ranges.append(distance)
                points.append((distance * math.cos(angle), distance * math.sin(angle), 0.0))
            else:
                ranges.append(math.inf)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "livox_frame"
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (LIDAR_RAYS - 1) * angle_increment
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / LIDAR_RATE_HZ
        scan.range_min = LIDAR_RANGE_MIN
        scan.range_max = LIDAR_RANGE_MAX
        scan.ranges = ranges
        self.scan_pub.publish(scan)

        cloud = PointCloud2()
        cloud.header = scan.header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = array("B", np.asarray(points, dtype=np.float32).tobytes())
        cloud.is_dense = True
        self.cloud_pub.publish(cloud)

    def _initialize_pose(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.yaw_controller.reset()
        self.last_yaw_control_time = float(self.data.time)
        initial = np.array([0.0, -1.35453, 2.54948] * 4)
        self.data.qpos[self.joint_qpos] = initial
        mujoco.mj_forward(self.model, self.data)
        with self.motor_lock:
            self.motor_command.fill(0.0)
        self.last_motor_command = time.monotonic()
        self.latest_cmd.fill(0.0)
        self.last_cmd_time = -math.inf
        self.start_wall = time.monotonic()
        self.target_mode = STANDING

    def _receive_motor_commands(self) -> None:
        while self.running:
            try:
                payload, _ = self.motor_socket.recvfrom(MOTOR_PACKET.size)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(payload) != MOTOR_PACKET.size:
                continue
            with self.motor_lock:
                self.motor_command[:] = MOTOR_PACKET.unpack(payload)
                self.last_motor_command = time.monotonic()

    def _on_cmd_vel(self, message: Twist) -> None:
        requested = np.array([
            message.linear.x, message.linear.y, message.angular.z
        ], dtype=np.float64)
        self.latest_cmd = np.clip(requested, -self.max_velocity, self.max_velocity)
        self.last_cmd_time = time.monotonic()

    def _reset(self, _request, response):
        self.reset_requested = True
        response.success = True
        response.message = "reset scheduled"
        return response

    def _stand(self, _request, response):
        self.target_mode = STANDING
        self.start_wall = time.monotonic()
        response.success = True
        response.message = "standing trajectory requested"
        return response

    def _lie_down(self, _request, response):
        self.target_mode = LIE_DOWN
        self.latest_cmd.fill(0.0)
        response.success = True
        response.message = "lie-down trajectory requested"
        return response

    def _stop(self, _request, response):
        self.target_mode = DAMPING
        self.latest_cmd.fill(0.0)
        response.success = True
        response.message = "joint damping requested"
        return response

    def _apply_motor_command(self) -> None:
        with self.motor_lock:
            command = self.motor_command.copy()
            command_age = time.monotonic() - self.last_motor_command
        if command_age > self.motor_command_timeout:
            self.data.ctrl[:] = 0.0
            return
        kp, pos, kd, vel, feed_forward = np.split(command, 5)
        q = self.data.qpos[self.joint_qpos]
        dq = self.data.qvel[self.joint_dof]
        self.data.ctrl[:] = kp * (pos - q) + kd * (vel - dq) + feed_forward

    def _send_state(self) -> None:
        root_q = self.data.qpos[self.root_qpos:self.root_qpos + 7]
        root_v = self.data.qvel[self.root_dof:self.root_dof + 6]
        rpy = np.zeros(3, dtype=np.float64)
        w, x, y, z = root_q[3:7]
        rpy[:] = (
            math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
            math.asin(np.clip(2 * (w*y - z*x), -1.0, 1.0)),
            math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)),
        )
        acceleration = self.data.sensordata[16:19]
        values = np.concatenate((
            rpy, acceleration, root_v[3:6],
            self.data.qpos[self.joint_qpos],
            self.data.qvel[self.joint_dof], self.data.ctrl,
        )).astype(np.float32)
        header = STATE_HEADER.pack(b"L3ST", 1, 0, self.state_sequence,
                                   float(self.data.time))
        self.state_socket.sendto(header + STATE_FLOATS.pack(*values), STATE_ADDRESS)
        self.state_sequence = (self.state_sequence + 1) & 0xFFFFFFFF

    def _send_command(self) -> None:
        wall_now = time.monotonic()
        if self.target_mode == STANDING and wall_now - self.start_wall >= self.stand_duration:
            self.target_mode = RL_CONTROL
        velocity = self.latest_cmd.copy()
        if wall_now - self.last_cmd_time > self.cmd_timeout:
            velocity.fill(0.0)
        policy_command = velocity.copy()
        sim_now = float(self.data.time)
        dt = sim_now - self.last_yaw_control_time
        self.last_yaw_control_time = sim_now
        motor_fresh = wall_now - self.last_motor_command <= self.motor_command_timeout
        if self.target_mode == RL_CONTROL and not self.paused and motor_fresh:
            measured_yaw_rate = float(self.data.qvel[self.root_dof + 5])
            policy_command[2] = self.yaw_controller.update(
                float(velocity[2]), measured_yaw_rate, dt)
        else:
            self.yaw_controller.reset()
            policy_command[2] = 0.0
        normalized = np.clip(policy_command / self.policy_velocity, -1.0, 1.0)
        effective = Twist()
        effective.linear.x = float(velocity[0])
        effective.linear.y = float(velocity[1])
        effective.angular.z = float(velocity[2])
        self.command_pub.publish(effective)
        policy_message = Twist()
        policy_message.linear.x = float(policy_command[0])
        policy_message.linear.y = float(policy_command[1])
        policy_message.angular.z = float(policy_command[2])
        self.policy_command_pub.publish(policy_message)
        self.sequence += 1
        payload = (f"LITE3_CMD_V1 {self.sequence} {self.target_mode} "
                   f"{normalized[0]:.6f} {normalized[1]:.6f} {normalized[2]:.6f}")
        self.command_socket.sendto(payload.encode("ascii"), COMMAND_ADDRESS)

    def _publish_ros(self) -> None:
        stamp = sim_time(float(self.data.time))
        self.clock_pub.publish(Clock(clock=stamp))
        root_q = self.data.qpos[self.root_qpos:self.root_qpos + 7]
        root_v = self.data.qvel[self.root_dof:self.root_dof + 6]

        joint = JointState()
        joint.header.stamp = stamp
        joint.name = JOINT_NAMES
        joint.position = self.data.qpos[self.joint_qpos].tolist()
        joint.velocity = self.data.qvel[self.joint_dof].tolist()
        joint.effort = self.data.ctrl.tolist()
        self.joint_pub.publish(joint)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "base_link"
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = root_q[3:7]
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = root_v[3:6]
        acceleration = self.data.sensordata[16:19]
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acceleration
        imu.orientation_covariance[0] = imu.orientation_covariance[4] = imu.orientation_covariance[8] = 0.01
        imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] = imu.angular_velocity_covariance[8] = 0.02
        imu.linear_acceleration_covariance[0] = imu.linear_acceleration_covariance[4] = imu.linear_acceleration_covariance[8] = 0.04
        self.imu_pub.publish(imu)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = root_q[:3]
        odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z = root_q[3:7]
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = root_v[:3]
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = root_v[3:6]
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.02
        odom.pose.covariance[14] = odom.pose.covariance[21] = odom.pose.covariance[28] = 99999.0
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance = list(odom.pose.covariance)
        self.odom_pub.publish(odom)
        self.leg_odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)

    def _sensor_loop(self) -> None:
        self.renderer = mujoco.Renderer(
            self.model, height=CAMERA_HEIGHT, width=CAMERA_WIDTH)
        period = 1.0 / min(LIDAR_RATE_HZ, CAMERA_RATE_HZ)
        next_sensor = time.perf_counter()
        while self.running and rclpy.ok():
            with self.data_lock:
                self.sensor_data.qpos[:] = self.data.qpos
                self.sensor_data.qvel[:] = self.data.qvel
                self.sensor_data.act[:] = self.data.act
                self.sensor_data.ctrl[:] = self.data.ctrl
                self.sensor_data.time = self.data.time
            mujoco.mj_forward(self.model, self.sensor_data)
            self._publish_static_transforms(self.sensor_data)
            self._publish_lidar(self.sensor_data)
            self._publish_camera(self.sensor_data)
            next_sensor += period
            remaining = next_sensor - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            else:
                next_sensor = time.perf_counter()

    def run(self) -> None:
        next_step = time.perf_counter()
        step = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.reset_requested:
                self.reset_requested = False
                self._initialize_pose()
            with self.data_lock:
                self._apply_motor_command()
                if not self.paused:
                    mujoco.mj_step(self.model, self.data)
            self._send_state()
            if step % 10 == 0:
                self._send_command()
            if step % 10 == 0:
                self._publish_ros()
            step += 1
            next_step += DT
            remaining = next_step - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            else:
                # Keep the absolute deadline so sensor/rendering jitter is
                # recovered on subsequent steps instead of slowing /clock.
                # Every step still polls ROS, samples the latest motor command,
                # and sends fresh state to the external controller. Yield to
                # that controller between catch-up steps and cap severe lag.
                next_step = max(next_step, time.perf_counter() - MAX_CATCHUP_S)
                time.sleep(0)

    def destroy_node(self) -> bool:
        self.running = False
        if self.httpd is not None:
            self.httpd.shutdown()
            self.httpd.server_close()
        if self.http_thread is not None:
            self.http_thread.join(timeout=2.0)
        if self.sensor_thread is not None:
            self.sensor_thread.join(timeout=2.0)
        self.motor_socket.close()
        self.state_socket.close()
        self.command_socket.close()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--viewer", choices=("headless", "web"), default="headless")
    parser.add_argument("--viewer-port", type=int, default=8080)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Lite3Bridge(viewer=args.viewer, viewer_port=args.viewer_port)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
