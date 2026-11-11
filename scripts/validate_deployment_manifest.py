#!/usr/bin/env python3
"""Validate mode-specific safety invariants in a checked-in deployment manifest."""

import argparse
from pathlib import Path
from typing import Any

import yaml

FORBIDDEN_SIMULATION_KEYS = {"target_ip", "target_port", "local_port", "app_port"}


def walk(value: Any, path: str = "root"):
    if isinstance(value, dict):
        for key, child in value.items():
            yield path, key, child
            yield from walk(child, f"{path}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from walk(child, f"{path}[{index}]")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", required=True, choices=("hardware", "simulation"))
    parser.add_argument("manifest", type=Path)
    args = parser.parse_args()

    manifest = yaml.safe_load(args.manifest.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise SystemExit("manifest must be a YAML mapping")

    lite3 = next(
        (item for item in manifest.get("primitive", []) if item.get("name") == "lite3_quadruped"),
        None,
    )
    if lite3 is None:
        raise SystemExit("manifest has no lite3_quadruped primitive")
    backend = lite3.get("config", {}).get("backend")
    if backend != args.mode:
        raise SystemExit(f"lite3_quadruped backend={backend!r}, expected {args.mode!r}")

    if args.mode == "simulation":
        for path, key, value in walk(manifest):
            if key in FORBIDDEN_SIMULATION_KEYS and value not in (None, ""):
                raise SystemExit(f"simulation manifest contains forbidden {path}.{key}={value!r}")
        env = manifest.get("env", {})
        if str(env.get("ROS_DOMAIN_ID")) != "42":
            raise SystemExit("simulation manifest must use ROS_DOMAIN_ID=42")


if __name__ == "__main__":
    main()
