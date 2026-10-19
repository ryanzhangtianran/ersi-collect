#!/usr/bin/env python3
# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
"""Extract a raw ROS2 mcap recording that is missing recording_info.json."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _object_asset_dict(saved_task: dict) -> dict[str, str]:
    assets: dict[str, str] = {}
    for obj in saved_task.get("objects", []):
        object_id = obj["object_id"]
        data_dir = obj.get("data_info_dir", "").rstrip("/")
        if data_dir:
            assets[f"/World/Objects/{object_id}"] = f"{data_dir}/Aligned.usd"
    return assets


def build_recording_info(
    recording_dir: Path,
    saved_task: dict,
    reference_info: dict,
    task_name: str | None = None,
) -> dict:
    object_asset_dict = _object_asset_dict(saved_task)
    scene_usd = saved_task.get("scene_usd")
    if isinstance(scene_usd, list):
        scene_usd = scene_usd[0]
    scene_name = Path(scene_usd).parent.name if scene_usd else reference_info["scene_name"]

    container_root = "/geniesim/main/data_collection"
    recording_name = recording_dir.name
    container_path = f"{container_root}/recording_data/{recording_name}"

    info = {
        "bag_file": container_path,
        "output_dir": container_path,
        "robot_init_position": reference_info["robot_init_position"],
        "robot_init_rotation": reference_info["robot_init_rotation"],
        "camera_info": reference_info["camera_info"],
        "scene_name": scene_name,
        "scene_usd": scene_usd or reference_info["scene_usd"],
        "object_names": {
            "object_prims": list(object_asset_dict.keys()),
            "articulated_object_prims": [],
        },
        "fps": reference_info.get("fps", 30),
        "robot_name": reference_info["robot_name"],
        "frame_status": [],
        "light_config": saved_task.get("lights") or [],
        "gripper_names": reference_info["gripper_names"],
        "with_img": True,
        "with_video": True,
        "playback_timerange": [],
        "end_effector_prim_path": reference_info["end_effector_prim_path"],
        "end_effector_center_prim_path": reference_info["end_effector_center_prim_path"],
        "arm_base_prim_path": reference_info["arm_base_prim_path"],
        "task_name": task_name or recording_name,
        "fail_stage_step": [-1, -1],
        "object_asset_dict": object_asset_dict,
        "code_dict": reference_info.get("code_dict", {}),
    }
    if reference_info.get("scene_glb"):
        info["scene_glb"] = reference_info["scene_glb"]
    return info


def build_metric_config(saved_task: dict) -> dict:
    return saved_task.get("task_metric", {"filter_rules": []})


def write_extract_configs(
    recording_dir: Path,
    saved_task_path: Path,
    reference_info_path: Path,
) -> tuple[Path, Path]:
    saved_task = _load_json(saved_task_path)
    reference_info = _load_json(reference_info_path)

    recording_info = build_recording_info(recording_dir, saved_task, reference_info)
    metric_config = build_metric_config(saved_task)

    recording_info_path = recording_dir / "recording_info.json"
    metric_config_path = recording_dir / "metric_config.json"

    with recording_info_path.open("w", encoding="utf-8") as f:
        json.dump(recording_info, f, indent=4, ensure_ascii=False)
    with metric_config_path.open("w", encoding="utf-8") as f:
        json.dump(metric_config, f, indent=4, ensure_ascii=False)

    frame_state_path = recording_dir / "frame_state.json"
    with frame_state_path.open("w", encoding="utf-8") as f:
        json.dump(recording_info.get("frame_status", []), f, indent=4, ensure_ascii=False)

    return recording_info_path, metric_config_path


def run_convert(recording_dir: Path, recording_info_path: Path, metric_config_path: Path) -> int:
    repo = _repo_root()
    recording_info = _load_json(recording_info_path)
    metric_config = _load_json(metric_config_path)
    container_path = f"/geniesim/main/data_collection/recording_data/{recording_dir.name}"

    script = f"""
import json
import sys
sys.path.insert(0, "{repo}")
sys.path.insert(0, "{repo / "server" / "recording"}")
from sim_data_converter import SimDataConverter
from common.data_filter.check_collected_data import filter_folder_data
from common.base_utils.logger import logger

path = "{container_path}"
with open(path + "/recording_info.json", "r") as f:
    task_info = json.load(f)
converter = SimDataConverter(
    path, path, 0, 0, 0, task_info.get("gripper_names"), task_info.get("robot_name")
)
converter.convert()
data_valid, result_code, status = filter_folder_data(path, {json.dumps(metric_config)})
result = {{
    "task_name": task_info.get("task_name"),
    "fail_stage_step": task_info.get("fail_stage_step"),
    "fps": task_info.get("fps"),
    "task_status": data_valid,
    "camera_info": task_info.get("camera_info_list"),
    "return_code": result_code,
    "metric_status": status,
    "playback_times": len(task_info.get("playback_timerange", [])),
}}
with open(path + "/task_result.json", "w", encoding="utf-8") as f:
    json.dump(result, f, ensure_ascii=False)
logger.info(f"Status: {{status}} (Result Code: {{result_code}})")
"""

    if Path("/.dockerenv").exists():
        cmd = ["/isaac-sim/python.sh", "-c", script]
        return subprocess.call(cmd, cwd=repo / "server" / "recording")

    env = os.environ.copy()
    assets_src = env.get("GENIESIM_ASSETS_SRC", "/home/user/djy/geniesim_assets")
    docker_cmd = [
        "docker",
        "run",
        "--rm",
        "--entrypoint",
        "",
        "-u",
        "1234:1234",
        "-v",
        f"{assets_src}:/geniesim_assets:ro",
        "-v",
        f"{repo}:/geniesim/main/data_collection:rw",
        "-w",
        "/geniesim/main/data_collection/server/recording",
        "registry.agibot.com/genie-sim/geniesim3-data-collection:latest",
        "/isaac-sim/python.sh",
        "-c",
        script,
    ]
    print("Running convert step in Docker")
    return subprocess.call(docker_cmd)


def run_extract(recording_dir: Path, recording_info_path: Path, metric_config_path: Path) -> int:
    repo = _repo_root()
    extract_script = repo / "server" / "recording" / "extract_and_convert_data.py"
    container_path = f"/geniesim/main/data_collection/recording_data/{recording_dir.name}"

    extract_args = [
        "--path_to_save",
        container_path,
        "--task_info_path",
        f"{container_path}/recording_info.json",
        "--metric_config_path",
        f"{container_path}/metric_config.json",
    ]

    if Path("/.dockerenv").exists():
        cmd = ["/isaac-sim/python.sh", str(extract_script), *extract_args]
        print("Running:", " ".join(cmd))
        return subprocess.call(cmd, cwd=repo)

    env = os.environ.copy()
    assets_src = env.get("GENIESIM_ASSETS_SRC")
    if not assets_src:
        for candidate in (
            Path("/home/user/djy/geniesim_assets"),
            Path("/geniesim_assets"),
        ):
            if (candidate / "pyproject.toml").exists():
                assets_src = str(candidate)
                break

    docker_cmd = [
        "docker",
        "run",
        "--rm",
        "--entrypoint",
        "",
        "-u",
        "1234:1234",
    ]
    if assets_src:
        docker_cmd.extend(["-v", f"{assets_src}:/geniesim_assets:ro"])
    docker_cmd.extend(
        [
            "-v",
            f"{repo}:/geniesim/main/data_collection:rw",
            "-w",
            "/geniesim/main/data_collection/server/recording",
            "registry.agibot.com/genie-sim/geniesim3-data-collection:latest",
            "/isaac-sim/python.sh",
            "extract_and_convert_data.py",
            *extract_args,
        ]
    )

    print("Running:", " ".join(docker_cmd))
    return subprocess.call(docker_cmd)


def main() -> int:
    parser = argparse.ArgumentParser(description="Extract raw mcap recording data")
    parser.add_argument(
        "recording_dir",
        type=Path,
        help="Path to recording_data/[task_name]/ directory containing *.mcap",
    )
    parser.add_argument(
        "--saved-task",
        type=Path,
        default=None,
        help="Saved task instance JSON (defaults to first_run_demos match)",
    )
    parser.add_argument(
        "--reference-recording",
        type=Path,
        default=_repo_root()
        / "recording_data"
        / "[sort_the_fruit_into_the_box_apple_g2_1]"
        / "recording_info.json",
        help="Reference recording_info.json for G2 camera/robot metadata",
    )
    parser.add_argument(
        "--configs-only",
        action="store_true",
        help="Only write recording_info.json, metric_config.json, and frame_state.json",
    )
    parser.add_argument(
        "--convert-only",
        action="store_true",
        help="Skip mcap extract and only run convert + metric filter",
    )
    args = parser.parse_args()

    recording_dir = args.recording_dir.resolve()
    if not recording_dir.is_dir():
        print(f"Recording directory not found: {recording_dir}", file=sys.stderr)
        return 1
    if not args.convert_only and not any(recording_dir.glob("*.mcap")):
        print(f"No .mcap file found in {recording_dir}", file=sys.stderr)
        return 1

    saved_task_path = args.saved_task
    if saved_task_path is None:
        task_stem = recording_dir.name.strip("[]").rsplit("_", 1)[0]
        saved_task_path = (
            _repo_root()
            / "saved_task"
            / "first_run_demos"
            / task_stem
            / f"{task_stem}_0.json"
        )
    saved_task_path = saved_task_path.resolve()
    reference_info_path = args.reference_recording.resolve()

    if not saved_task_path.is_file():
        print(f"Saved task not found: {saved_task_path}", file=sys.stderr)
        return 1
    if not reference_info_path.is_file():
        print(f"Reference recording_info not found: {reference_info_path}", file=sys.stderr)
        return 1

    recording_info_path, metric_config_path = write_extract_configs(
        recording_dir, saved_task_path, reference_info_path
    )
    print(f"Wrote {recording_info_path}")
    print(f"Wrote {metric_config_path}")

    if args.configs_only:
        return 0

    if args.convert_only:
        return run_convert(recording_dir, recording_info_path, metric_config_path)

    return run_extract(recording_dir, recording_info_path, metric_config_path)


if __name__ == "__main__":
    raise SystemExit(main())
