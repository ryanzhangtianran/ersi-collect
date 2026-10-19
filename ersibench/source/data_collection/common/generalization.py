"""Fold a task template's `generalization` block into the fields the collector already reads.

    "generalization": {
        "in_domain": {"lights": {...ratio...}, "camera": {...thresholds...}},      # always on, small
        "ood": {<RSIBench axis>: {...}, ...}                                       # ERSIBENCH_OOD_AXES=light,camera_pose
    }

The in-domain tier of robot_initial_pose / object_position is the template's own `robot_init_pose.random`,
`init_arm_pose_noise`, `sub_workspace` / `random_range` (task-tuned, untouched here); a selected OOD axis overrides them:
  light               lights {intensity: [...], temperature: [...]}   -> task_info["lights_config"] (absolute, all stage lights)
  camera_pose         camera {x,y,z (m), roll,pitch,yaw (deg)}         -> task_info["camera_pose_noise"]
  robot_initial_pose  init_base {delta_position}, init_joint {thresh}  -> robot.robot_init_pose.random, robot.init_arm_pose_noise
  object_position     sub_workspace {...} / range_scale                -> every task_related_object's sub_workspace / random_range
  unseen_object       candidate_objects {object_id: [...]}             -> that object's candidate_objects
Axis names are the RSIBench profile's, so `ERSIBENCH_OOD_AXES=camera_pose` collects for the `camera_pose` eval axis.
Base offsets, joints, lights and cameras are all drawn once per episode (the base per layout, TaskGenerator.draw_robot_init_pose).
"""
import os


def ood_axes_from_env():
    return [a for a in os.environ.get("ERSIBENCH_OOD_AXES", "").split(",") if a]


def apply(task_info, ood_axes=None):
    g = task_info.get("generalization", {})
    if not g:
        return task_info
    ood_axes = ood_axes_from_env() if ood_axes is None else ood_axes
    unknown = set(ood_axes) - set(g.get("ood", {}))
    if unknown:
        raise ValueError(f"ERSIBENCH_OOD_AXES {sorted(unknown)} not in generalization.ood {sorted(g.get('ood', {}))}")
    cfg = dict(g.get("in_domain", {}))
    for axis in ood_axes:
        cfg.update(g["ood"][axis])
    if "lights" in cfg:
        task_info["lights_config"] = cfg["lights"]
    if "camera" in cfg:
        task_info["camera_pose_noise"] = {"enable": True, "threshold": cfg["camera"]}
    robot = task_info["robot"]
    if "init_base" in cfg:
        pose = robot["robot_init_pose"]
        for p in ([pose] if "position" in pose else pose.values()):
            p["random"] = {"delta_position": cfg["init_base"]["delta_position"]}
    if "init_joint" in cfg:
        t = cfg["init_joint"]["thresh"]
        robot["init_arm_pose_noise"] = {".*_arm_.*": {"noise_type": "uniform", "low": -t, "high": t}}
    objs = task_info["objects"]
    for o in [*(objs.get("task_related_objects") or []), *(objs.get("scene_objects") or []), *(objs.get("fix_objects") or [])]:
        if "sub_workspace" in o and "sub_workspace" in cfg:  # placed objects and sampled distractor groups alike
            o["sub_workspace"] = cfg["sub_workspace"]
        if "random_range" in o and "range_scale" in cfg:
            o["random_range"] = [r * cfg["range_scale"] for r in o["random_range"]]
        if o.get("object_id") in cfg.get("candidate_objects", {}):
            o["candidate_objects"] = cfg["candidate_objects"][o["object_id"]]
    task_info["ood_axes"] = list(ood_axes)  # recorded with the layout so saved_task says how it was collected
    return task_info
