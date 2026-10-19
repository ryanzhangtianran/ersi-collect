"""Prepare fixed collection tasks from archived failed benchmark episodes."""

from __future__ import annotations

import copy
import hashlib
import json
import os
import shutil
import tempfile
from pathlib import Path


MODULE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = MODULE_ROOT.parents[3]


def _read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _write(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _artifact_path(value, result_path):
    path = Path(value)
    if not path.is_absolute():
        path = Path(result_path).resolve().parent / path
    if not path.is_file() and str(path).startswith("/workspace/"):
        path = REPO_ROOT / path.relative_to("/workspace")
    if not path.is_file():
        raise FileNotFoundError(f"Snapshot is unavailable: {path}")
    return path


def convert_failure(result_path, episode_id, output_dir, assets_root,
                    robot_cfg="G2_omnipicker_fixed_dual.json", runtime_assets_root="/geniesim_assets"):
    from pxr import Sdf, UsdUtils

    result = _read(result_path)
    rows = result.get("details", [result])
    matches = [r for r in rows if r.get("rsi_episode", {}).get("id") == episode_id]
    if len(matches) != 1:
        raise ValueError("Select exactly one identified episode from an axis result or outcome.json")
    outcome = matches[0]
    if outcome.get("result", {}).get("scores", {}).get("E2E") != 0:
        raise ValueError("Only a completed, scored failure can be converted")
    if not outcome.get("rsi_snapshot"):
        raise ValueError("This result predates episode snapshots; its initial state cannot be reconstructed")
    snapshot_path = _artifact_path(outcome["rsi_snapshot"], result_path)
    snapshot = _read(snapshot_path)
    if snapshot.get("schema_version") != 1 or snapshot["episode"] != outcome["rsi_episode"]:
        raise ValueError("Snapshot identity/schema does not match the failed result")
    if snapshot["task_config"].get("sub_task_name") != "pick_block_color":
        raise ValueError("Recollection conversion currently supports pick_block_color only")
    instruction, target_id, grippers = snapshot["instruction"]
    sides = {entry["side"] for entry in grippers.values()}
    if len(sides) != 1 or not sides <= {"left", "right"}:
        raise ValueError("Expected a single-hand pick instruction")
    arm = sides.pop()
    state = snapshot["initial_state"]
    if target_id not in state["objects"]:
        raise ValueError("Target object is missing from the captured initial state")
    collection_robot = _read(MODULE_ROOT / "config/robot_cfg" / robot_cfg)
    robot_prim = collection_robot["robot"]["base_prim_path"]
    source_robot_prim = state["robot"]["prim_path"]
    if source_robot_prim.startswith("/World/"):
        raise ValueError("A robot nested in /World requires a separate scene adapter")
    assets_root = Path(assets_root).resolve()
    layout = snapshot["static_scene"]["scene_info.json"]["layout"]
    objects = []
    for object_id, pose in state["objects"].items():
        if object_id not in layout:
            continue
        asset_name = layout[object_id]["usd"]
        matches = list(assets_root.glob(f"objects/**/{asset_name}/object_parameters.json"))
        if len(matches) != 1:
            raise ValueError(f"Expected one asset definition for {asset_name}; found {len(matches)}")
        asset_dir = matches[0].parent
        info = _read(matches[0])
        scale = pose["scale"]
        if max(scale) - min(scale) > 1e-5:
            raise ValueError("Nonuniform object scaling requires a grasp geometry adapter")
        info.update(object_id=object_id, prim_path=pose["prim_path"], scene_object=True,
                    position=pose["position"], quaternion=pose["quaternion"], scale=scale[0],
                    data_info_dir=str(Path(runtime_assets_root) / asset_dir.relative_to(assets_root)))
        interaction = assets_root / "interaction" / asset_name / "interaction.json"
        if not interaction.is_file():
            raise FileNotFoundError(f"Missing collection interaction labels: {interaction}")
        info["interaction"] = _read(interaction)["interaction"]
        objects.append(info)
    if target_id not in {obj["object_id"] for obj in objects}:
        raise ValueError("Target has no collection asset definition")
    target_asset = layout[target_id]["usd"]
    grasp_file = assets_root / "interaction" / target_asset / "grasp_pose/grasp_pose.pkl"
    if not grasp_file.is_file():
        raise FileNotFoundError(f"Missing expert grasp candidates: {grasp_file}")

    cameras = {}
    for spec in state["camera_specs"]:
        source_path = spec["prim_path"]
        if source_path not in state["cameras"]:
            raise ValueError(f"Missing camera state: {source_path}")
        cam = state["cameras"][source_path]
        path = robot_prim + source_path[len(source_robot_prim):] if source_path.startswith(source_robot_prim + "/") else source_path
        cameras[path] = {"position": cam["local_position"], "quaternion": cam["local_quaternion"],
                         "width": spec["width"], "height": spec["height"], "is_local": True,
                         **{key: cam[key] for key in ("focal_length", "horizontal_aperture", "vertical_aperture")}}

    if not cameras:
        raise ValueError("Snapshot has no registered recording cameras")
    task_name = "rsi_pick_block_color_" + hashlib.sha256(
        json.dumps([snapshot["round"], snapshot["axis"], snapshot["episode"]], sort_keys=True).encode()
    ).hexdigest()[:16]
    provenance = {"schema_version": 1, "source_episode": snapshot["episode"],
                  "source_axis": snapshot["axis"], "source_round": snapshot["round"],
                  "task_folder": "saved_tasks", "scene_file": "snapshot/scene.usda",
                  "review_required": True, "auto_execute": False,
                  "differences": [
                      f"Collection uses {robot_cfg}; evaluation used {snapshot['task_config']['robot']['robot_cfg']}.",
                      "Expert planner regenerates actions and opens grippers; this is not action replay.",
                      "Physics solver state, joint PD overrides and image-space noise are archived but not replayed.",
                  ]}
    template = copy.deepcopy(snapshot["task_config"])
    template.update(task=task_name, generalization={},
                    origin={"position": [0, 0, 0], "quaternion": [1, 0, 0, 0]},
                    rsi_recollection=provenance)
    template["robot"].update(robot_cfg=robot_cfg, arm=arm,
                              robot_init_pose={k: state["robot"][k] for k in ("position", "quaternion")},
                              init_arm_pose=state["robot"]["joint_positions"])
    template["robot"].pop("init_arm_pose_noise", None)
    template["scene"]["scene_usd"] = "snapshot/scene.usda"
    template["recording_setting"].update(num_of_episode=1, camera_list=list(cameras))
    task = {"task": task_name, "arm": arm, "objects": objects, "cameras": cameras,
            "task_description": {"english_task_name": instruction},
            "stages": [{"action": "grasp", "group": 0, "active": {"object_id": "gripper", "primitive": None},
                        "passive": {"object_id": target_id, "primitive": None},
                        "extra_params": {"arm": arm},
                        "action_description": {"action_text": instruction, "english_action_text": instruction}},
                       {"action": "transition", "group": 0,
                        "extra_params": {"arm": arm, "target": "hold_offset", "carry_object_id": target_id,
                                         "pick_up_direction": "z", "pick_up_distance": 0.12},
                        "action_description": {"action_text": instruction, "english_action_text": instruction}}],
            "rsi_recollection": provenance}
    output_dir = Path(output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(f"Refusing to overwrite collection bundle: {output_dir}")
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".recollection-", dir=output_dir.parent))
    try:
        shutil.copytree(snapshot_path.parent, staging / "snapshot")
        # The collector runs as a different container UID and only needs read access.
        staging.chmod(0o755)
        (staging / "snapshot").chmod(0o755)
        # Rewrite asset references using USD APIs, retaining bundle-local dependencies.
        source_assets = Path(snapshot["assets_root"])
        def remap(path):
            value = Path(path)
            for root in (source_assets, Path("/geniesim_assets")):
                if value.is_absolute() and value.is_relative_to(root):
                    return str(Path(runtime_assets_root) / value.relative_to(root))
            return path
        for path in (staging / "snapshot").rglob("*"):
            if path.suffix in (".usd", ".usda", ".usdc"):
                layer = Sdf.Layer.FindOrOpen(str(path))
                UsdUtils.ModifyAssetPaths(layer, remap)
                layer.Save()
        (staging / "saved_tasks").mkdir()
        _write(staging / "saved_tasks" / f"{task_name}_0.json", task)
        _write(staging / "task_template.json", template)
        _write(staging / "source_result.json", outcome)
        staging.rename(output_dir)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return output_dir / "task_template.json"


def load_recollection(task_template_file, task_info):
    """Resolve a prepared bundle without invoking randomized TaskGenerator."""
    root = Path(task_template_file).resolve().parent
    info = task_info["rsi_recollection"]
    if info.get("schema_version") != 1:
        raise ValueError("Unsupported recollection bundle schema")
    folder = (root / info["task_folder"]).resolve()
    scene = (root / info["scene_file"]).resolve()
    if not folder.is_relative_to(root) or not scene.is_relative_to(root):
        raise ValueError("Recollection files must stay inside their bundle")
    if len(list(folder.glob("*.json"))) != 1 or not scene.is_file():
        raise ValueError("Incomplete recollection bundle")
    task_info["scene"]["scene_usd"] = str(scene)
    return str(folder), task_info["robot"]["robot_init_pose"], task_info["scene"].get("function_space_objects", {})
