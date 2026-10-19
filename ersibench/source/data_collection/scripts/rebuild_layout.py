#!/usr/bin/env python3
"""Rebuild a collection layout JSON from a recorded episode.

A layout (``saved_task/<task>/<task>_N.json``) is what turns a recorded episode into a benchmark
eval scene (see scripts/build_eval_scenes.py).  When one is lost, everything it holds is still in
the episode itself:

  objects' poses     <- state.json frame 0 (after the spawn drop has settled)
  slot positions     <- metric_config.json filter_rules[*].params.region_center
  block -> asset     <- recording_info.json object_asset_dict
  everything else    <- any surviving layout of the same task (stages, task_metric skeleton, ...)

The recorded pose is the settled one while a layout stores the spawn pose, ~9.4 cm higher; the
offset is measured from the episodes that still have their layout so rebuilt files keep the same
convention.  Run --check first: it rebuilds the layouts that do exist and reports the deviation.

  rebuild_layout.py --task arrange_blocks --check
  rebuild_layout.py --task arrange_blocks --episodes 2 3 4 5 6 7 8 14 26
"""

import argparse
import json
import os
import pathlib
import re

import numpy as np
from scipy.spatial.transform import Rotation


def mat2quat_wxyz(matrix):
    x, y, z, w = Rotation.from_matrix(matrix).as_quat()
    return [float(w), float(x), float(y), float(z)]


def read_episode(nas, task, index):
    d = nas / "recording_data" / task / f"[{task}_{index}]"
    frame0 = json.loads((d / "state.json").read_text())["frames"][0]["objects"]
    rules = json.loads((d / "metric_config.json").read_text())["filter_rules"]
    assets = json.loads((d / "recording_info.json").read_text())["object_asset_dict"]
    centers = [r["params"]["region_center"] for r in rules if "region_center" in r["params"]]
    poses = {name: np.array(o["pose"]) for name, o in frame0.items()}
    assets = {p.rsplit("/", 1)[-1]: a.rsplit("Aligned.usd", 1)[0] for p, a in assets.items()}
    return poses, centers, assets


def spawn_offsets(nas, task, indices):
    """z offset between the stored spawn pose and the settled pose, per object, from real layouts."""
    per_object = {}
    for i in indices:
        lay = {o["object_id"]: o for o in json.loads((nas / "saved_task" / task / f"{task}_{i}.json").read_text())["objects"]}
        poses, _, _ = read_episode(nas, task, i)
        for name, pose in poses.items():
            per_object.setdefault(name, []).append(lay[name]["position"][2] - float(pose[2, 3]))
    return {name: float(np.median(v)) for name, v in per_object.items()}


def rebuild(template, poses, centers, assets, offsets):
    layout = json.loads(json.dumps(template))  # deep copy
    fix_names = [o["object_id"] for o in template["objects"] if o.get("data_info_dir") == "fix_pose"]
    if len(fix_names) != len(centers):
        raise ValueError(f"template has {len(fix_names)} fix_pose objects but the episode recorded {len(centers)}")

    objects, centers_iter = [], iter(centers)
    for entry in template["objects"]:
        item = json.loads(json.dumps(entry))
        name = item["object_id"]
        if item.get("data_info_dir") == "fix_pose":
            item["position"] = list(next(centers_iter))
        else:
            pose = poses[name]
            item["position"] = [float(pose[0, 3]), float(pose[1, 3]), float(pose[2, 3]) + offsets[name]]
            item["quaternion"] = mat2quat_wxyz(pose[:3, :3])
            item["data_info_dir"] = assets[name]
        objects.append(item)
    layout["objects"] = objects

    centers_iter = iter(centers)
    for rule in layout["task_metric"]["filter_rules"]:
        if "region_center" in rule["params"]:
            rule["params"]["region_center"] = list(next(centers_iter))
    return layout


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--task", required=True)
    p.add_argument("--data-root", default=os.environ.get("ERSIBENCH_DATA_ROOT", str(pathlib.Path.home() / "NAS/ERSI")))
    p.add_argument("--episodes", type=int, nargs="+", help="episode indices to rebuild (default: every one missing a layout)")
    p.add_argument("--template", type=int, help="episode index whose layout is used as the skeleton (default: the highest available)")
    p.add_argument("--check", action="store_true", help="rebuild the layouts that already exist and report the deviation; writes nothing")
    p.add_argument("--dry-run", action="store_true")
    a = p.parse_args()

    nas = pathlib.Path(a.data_root).expanduser()
    have = sorted(int(re.search(r"_(\d+)\.json$", str(f)).group(1))
                  for f in (nas / "saved_task" / a.task).glob(f"{a.task}_*.json"))
    recorded = sorted(int(re.search(r"_(\d+)\]$", d.name).group(1))
                      for d in (nas / "recording_data" / a.task).glob(f"[[]{a.task}_*]") if (d / "state.json").exists())
    if not have:
        raise SystemExit(f"no surviving layout for {a.task}; cannot rebuild without a skeleton")

    template_index = a.template if a.template is not None else have[-1]
    template = json.loads((nas / "saved_task" / a.task / f"{a.task}_{template_index}.json").read_text())
    usable = [i for i in have if i in recorded]
    offsets = spawn_offsets(nas, a.task, usable)
    print(f"template: episode {template_index}; spawn-z offsets from {len(usable)} layouts: "
          + ", ".join(f"{k}={v * 1000:.1f}mm" for k, v in offsets.items()))

    if a.check:
        dp = dq = 0.0
        mismatched = []
        for i in usable:
            real = json.loads((nas / "saved_task" / a.task / f"{a.task}_{i}.json").read_text())
            got = rebuild(template, *read_episode(nas, a.task, i), offsets)
            for ro, go in zip(real["objects"], got["objects"]):
                dp = max(dp, float(np.linalg.norm(np.array(ro["position"]) - np.array(go["position"]))))
                q1, q2 = np.array(ro["quaternion"]), np.array(go["quaternion"])
                dq = max(dq, float(min(np.linalg.norm(q1 - q2), np.linalg.norm(q1 + q2))))
                if ro["object_id"] != go["object_id"] or ro.get("data_info_dir") != go.get("data_info_dir"):
                    mismatched.append((i, ro["object_id"]))
            for key in real:
                if key in ("objects", "task_metric"):
                    continue
                if json.dumps(real[key], sort_keys=True) != json.dumps(got[key], sort_keys=True):
                    mismatched.append((i, f"field:{key}"))
        print(f"checked {len(usable)} rebuilt-vs-real layouts")
        print(f"  max position deviation  : {dp * 1000:.2f} mm")
        print(f"  max quaternion deviation: {dq:.2e}")
        print(f"  id/asset/field mismatches: {mismatched or 'none'}")
        return

    targets = a.episodes if a.episodes else [i for i in recorded if i not in have]
    if not targets:
        print("every recorded episode already has a layout")
        return
    for i in targets:
        out = nas / "saved_task" / a.task / f"{a.task}_{i}.json"
        if out.exists():
            print(f"skip {out.name}: already exists")
            continue
        layout = rebuild(template, *read_episode(nas, a.task, i), offsets)
        if a.dry_run:
            print(f"would write {out.name}: " + ", ".join(
                f"{o['object_id']}@{np.round(o['position'], 3).tolist()}" for o in layout["objects"]))
            continue
        out.write_text(json.dumps(layout, ensure_ascii=False, indent=4) + "\n")
        print(f"wrote {out.name}")


if __name__ == "__main__":
    main()
