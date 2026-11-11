from __future__ import annotations

import json
import math
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


def build_room_scene(
    web_root: Path,
    environment: str = "scenesmith_house_187",
    spawn_override: tuple[float, float] | None = None,
    yaw_override: float | None = None,
) -> tuple[Path, tuple[float, float]]:
    env_root = web_root / "assets" / "environments" / environment
    robot_root = web_root / "assets" / "robots" / "lite3"
    manifest = json.loads((env_root / "spawn.json").read_text(encoding="utf-8"))
    spawn = (
        spawn_override
        if spawn_override is not None
        else tuple(float(value) for value in manifest["position"])
    )
    yaw = (
        float(yaw_override)
        if yaw_override is not None
        else float(manifest.get("yaw", 0.0))
    )
    yaw_quaternion = [math.cos(yaw * 0.5), 0.0, 0.0, math.sin(yaw * 0.5)]
    scene_dir = Path(tempfile.mkdtemp(prefix=f"lite3-{environment}-"))
    shutil.copytree(env_root, scene_dir, dirs_exist_ok=True)
    shutil.copytree(robot_root / "meshes", scene_dir / "meshes", dirs_exist_ok=True)

    robot_tree = ET.parse(robot_root / "Lite3.xml")
    robot_xml = robot_tree.getroot()
    worldbody = robot_xml.find("worldbody")
    torso = None if worldbody is None else worldbody.find("body[@name='TORSO']")
    if torso is None:
        raise RuntimeError("Lite3.xml is missing TORSO")
    xyz = [float(value) for value in torso.get("pos", "0 0 1").split()]
    torso.set("pos", f"{spawn[0]:.9g} {spawn[1]:.9g} {xyz[2]:.9g}")
    local_quaternion = [float(value) for value in torso.get("quat", "1 0 0 0").split()]
    lw, lx, ly, lz = yaw_quaternion
    rw, rx, ry, rz = local_quaternion
    torso.set("quat", " ".join(f"{value:.9g}" for value in [
        lw*rw-lx*rx-ly*ry-lz*rz, lw*rx+lx*rw+ly*rz-lz*ry,
        lw*ry-lx*rz+ly*rw+lz*rx, lw*rz+lx*ry-ly*rx+lz*rw]))
    for key in robot_xml.findall(".//keyframe/key[@qpos]"):
        qpos = [float(value) for value in key.get("qpos", "").split()]
        if len(qpos) >= 7:
            qpos[0], qpos[1] = qpos[0] + spawn[0], qpos[1] + spawn[1]
            rw, rx, ry, rz = qpos[3:7]
            lw, lx, ly, lz = yaw_quaternion
            qpos[3:7] = [lw*rw-lx*rx-ly*ry-lz*rz, lw*rx+lx*rw+ly*rz-lz*ry,
                          lw*ry-lx*rz+ly*rw+lz*rx, lw*rz+lx*ry-ly*rx+lz*rw]
            key.set("qpos", " ".join(f"{value:.9g}" for value in qpos))
    robot_tree.write(scene_dir / "Lite3.xml", encoding="utf-8", xml_declaration=True)

    scene_tree = ET.parse(env_root / "scene.xml")
    scene_root = scene_tree.getroot()
    scene_root.set("model", f"lite3_{environment}")
    scene_root.insert(0, ET.Element("include", {"file": "Lite3.xml"}))
    scene_path = scene_dir / "scene.xml"
    scene_tree.write(scene_path, encoding="utf-8", xml_declaration=True)
    return scene_path, spawn
