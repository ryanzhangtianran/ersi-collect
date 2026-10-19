---
name: create-task
description: >
  Turn a one-sentence task description ("把最大的积木放到目标位置") into a runnable collection task for the
  `ersibench-collect` image: write <work>/scenes/<TASK>.json + <work>/tasks/<TASK>/g2/<TASK>.json starting from the
  shipped pick_biggest_block task, check them, generate layouts offline, run one smoke attempt in Isaac Sim, then
  collect with skills/collect. Trigger: "新建任务", "生成任务", "写一个任务 json", "改成…的任务", "create task".
license: MPL-2.0
metadata:
  author: ersi
  version: "1.0"
inputs:
  - name: DESCRIPTION
    desc: What the robot should do, one sentence (Chinese or English)
    required: true
  - name: TASK
    desc: Task name (letters, digits, _); derive one from DESCRIPTION if not given. Must not be pick_biggest_block
    required: false
  - name: WORK
    desc: Host dir holding the user's tasks (mounted at /work); created if missing
    required: false
    default: "./ersi_work"
  - name: BACKGROUND
    desc: "Background with a table to put the scene in (`list` prints them: home/home_b_with_table, home/home_1_with_table); default: the template's"
    required: false
outputs:
  - desc: "WORK/scenes/<TASK>.json, WORK/tasks/<TASK>/g2/<TASK>.json, OUT/layouts/<TASK>/ (offline layouts), OUT/logs/<TASK>/ (smoke run)"
---

The image ships one task, `pick_biggest_block` (scene `pick_block_size`); every other task is written with this skill.

The user's files live in a host dir mounted at `/work`, laid out like the collector's own tree:

```
<WORK>/scenes/<TASK>.json               where: background, origin, robot, objects, workspaces, generalization, recording_setting
<WORK>/tasks/<TASK>/g2/<TASK>.json      what: scene_file, task, task_description, stages, task_metric
```

At every start the container copies them over its built-in ones, so a user file wins on a name clash.

Set these once per shell (`IMG` must be loaded; `A` = geniesim_assets checkout, `O` = output dir):

```bash
IMG=ersibench-collect:latest; W=$PWD/ersi_work; A=/path/to/geniesim_assets; O=/path/to/out
mkdir -p $W/scenes $W/tasks
GPU="--gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0"      # the same GPU twice
```

Field meanings: `docker run --rm --entrypoint cat $IMG /opt/ersibench/source/data_collection/TASK_CONFIG_GUIDE.md`.
Schemas: `/opt/ersibench/skills/collect-episodes/{scene,task}.schema.json` (same `cat`). JSON Schema 2020-12;
unknown fields are errors.

## 1. Start from the template

```bash
docker run --rm -v $W:/work $IMG list               # tasks, scene jsons, backgrounds with a table
D=/opt/ersibench/source/data_collection
docker run --rm --entrypoint cat $IMG $D/scenes/pick_block_size.json > $W/scenes/<TASK>.json
mkdir -p $W/tasks/<TASK>/g2
docker run --rm --entrypoint cat $IMG $D/tasks/pick_biggest_block/g2/pick_biggest_block.json > $W/tasks/<TASK>/g2/<TASK>.json
```

Read both in full before editing. In the task json, set `"scene_file": "<TASK>"` and `"task": "<TASK>"`.

Every task gets its own scene json, named like the task. Never point a new task at a scene another task uses, so that
later edits to one cannot change the other.

## 2. The scene

- **Same table, other objects**: edit `objects`.
  - `scene_objects` are the randomly placed ones: `sample.min_num`/`max_num`, `workspace_id`, `sub_workspace` ratios,
    `available_objects`.
  - `task_related_objects` are the ones the task acts on.
  - `fix_objects` are fixed targets such as a `fix_pose` point.
  - Asset paths (`data_info_dir`) are relative to the geniesim_assets checkout (`ls $A/objects/benchmark`).
- **Other background with a table** (`list`): change only these, and keep `work_table`, the robot pose and every
  object position as they are; they are in the origin frame, which the new `origin` moves onto the new table.
  - `scene.scene_usd` = `["background/<key>/background.usda"]`
  - `scene.scene_id` = `scene.scene_info_dir` = `"background/<key>/"`
  - `origin.position`: home_b → `[2.91, 0.76, 0.0]`, home_1 → `[2.23, -0.98, 0.141]`
- **A room without a table** is not supported in the image: adding a table to a background is done in ERSI-Bench
  (collect service `put_background`), then synced into a new image.

Workspaces are defined in `scene.function_space_objects` (`work_table`: `position`, `quaternion`, `size`). A top-level
`"workspaces": {"<id>": {...}}` block (scene or task json) overrides fields of them. Use it for
`"blocked_zone": [[x0, x1], [y0, y1]]` (workspace frame, metres) around a target on the table, so nothing spawns on it.

## 3. The task

- Stages are only `grasp`, `transition`, `ungrasp`, `bimanual`.
- Every non-bimanual stage has `extra_params.arm` (`left`, `right`, `dual`, or `auto` = the arm of the last `auto`
  grasp).
- Every stage has `action_description` (`action_text` with `{左/右}`, `english_action_text` with `{Left/Right}`).
- `group`: a grasp and the transition lifting it share one group; the last descent and the ungrasp share another. On
  failure the collector retries from the start of the group.
- Positions are in metres in the `origin` frame; quaternions are `[w, x, y, z]`. The default table top
  (`benchmark_table_019`) is at z = 0.726, so a `fix_pose` target for an object of height h sits at z = 0.726 + h/2.
- `task_description.english_task_name` is the training instruction.
- `task_metric.filter_rules` name objects by `object_id`. The last rule should prove the goal: `is_object_end_in_region`,
  `is_object_end_higher_than_start`, `is_object_end_pose_up`, `is_objects_ordered_along_axis`.
- Keep `recording_setting.noised_probability` at 0.0. The image overrides `num_of_episode` / `num_of_success` per round.
- Keep `generalization.*.camera` x/y/z at 0 (rotation only): a few mm of backward offset pushes the head or wrist
  camera into its shell and the image becomes a circle in black.

Stage patterns (`check` prints a task in this notation; `T[a>b]` = a transition with two legs):

| Kind | Stages | Key params |
|---|---|---|
| pick up (shipped) | `G T[hold_offset]` | grasp `passive.object_id`; transition `target: hold_offset`, `pick_up_distance` |
| pick and place at a point (worked example below, passes) | `G T[hold_offset>world>world] U T[world>reset]` | `target: world`, `carry_object_id`, `target_object_id` (a `fix_pose` object), `target_world_offset` above it, then `[0,0,0.002]`; `motion_type` `AvoidObs` for the carry, `Simple` for the descent |
| put into an object | same | `target_object_id` = the container, `target_object_offset` = the opening in its frame; `level_object: "y"` keeps a long object upright |
| stand an object upright | `G T[hold_offset>world>world>world] U T[reset]` | a `transition` with `level_object`, `lift_height`, `max_spin_deg`; then `target_position: "origin"` + offsets |
| top-down grasp of a block | any of the above | grasp `fixed_top_grasp: true` + `filter_grasp_pose: {gripper_up_axis: z, threshold: 0.8, target_direction: -z}` |
| two arms carry one object | `bi(G\|G) T[world>world>world] U` | `bimanual` with `left`/`right` grasps, then transitions with `arm: dual`; ungrasp `synchronized_gripper: true` |

The first two run end to end in this image; the rest are patterns from ERSI-Bench tasks that use other scenes and
assets. Smoke-test them before collecting.

## 4. Check (seconds, no GPU)

```bash
docker run --rm -v $W:/work $IMG check <TASK>
```

It prints `stages: G T[...] U ...` and `2/2 ok`, or `path: message` per error. Compare the stage line with the
intended motion.

The check covers the schemas, the scene file and the background key. It cannot see an unknown `workspace_id`, a metric
naming a missing object, or a missing asset: those fail in step 5 or 6.

## 5. Layouts (seconds, no Isaac server)

```bash
docker run --rm $GPU -v $A:/assets:ro -v $O:/output -v $W:/work $IMG layouts <TASK> 10
python3 -c "
import json, glob
o = json.load(open('$W/scenes/<TASK>.json'))['origin']['position']      # layouts are in world coordinates
for p in sorted(glob.glob('$O/layouts/<TASK>/*.json'))[:3]:
    d = json.load(open(p))
    print(p.split('/')[-1], {x['object_id']: [round(a - b, 3) for a, b in zip(x['position'], o)] for x in d['objects']})"
```

Expect `Layouts (10)`. Fewer means the workspace is too small for the objects (`sub_workspace`, `max_num`,
`blocked_zone`). Print object positions relative to `origin`, as above, to show where things landed.

## 6. Smoke attempt (~10 min, one GPU, ~17 GB)

```bash
docker run --rm $GPU -e BATCH=3 -e MAX_ROUNDS=1 -e CONVERT=0 \
    -v ersi-isaac-cache:/isaac-sim/.cache -v ersi-isaac-cc:/isaac-sim/.nv/ComputeCache \
    -v $A:/assets:ro -v $O:/output -v $W:/work $IMG <TASK> 1
grep -E "Stage .* (failed|success)|Task .* failed|successful episode" $O/logs/<TASK>/round_1/run_data_collection.log | tail
tail -20 $O/logs/<TASK>/round_1/data_collector_server.log        # planner reasons
```

- It stops at the first passed episode. `1 / 1 passed` means the task works end to end; continue with
  `skills/collect/SKILL.md`, using the same `-v $W:/work`.
- On failure, read which stage failed in the client log and why in the server log before changing any parameter.
- `Stage N transition failed due to no more action sequence` = the stage ran out of candidates. Look at the line
  before it: `GRASP_FOLLOW_FAILED ... actual_delta=[0, 0, 0]` = the object did not move with the hand, i.e. the grasp
  missed or slipped; `plan did not converge … IK_FAIL` in the server log = no reachable pose. One is luck; every
  attempt failing at the same stage is the config. (Grasps missing in every attempt was a code bug until 2026-10-19:
  the client kept the first base pose while the robot moved each episode. An image built before that cannot collect
  tasks whose scene has `robot_init_pose.random`.)
- The smoke episodes stay in `$O`. Delete `$O/{recording_data,saved_task,failed}/<TASK>` before a clean collection if
  you change the task afterwards.

## 7. Report

Files written, the stage sequence, layouts n/10, the smoke result (the failing stage and the reason, if any), and the
collect command for the user.

## Worked example: "把最小的积木放到目标位置" (place_smallest_block)

Collected in this image on 2026-10-19, after the base-sync fix: 2/2 attempts passed with per-episode base jitter. The
grasp and place stages are ERSI-Bench `arrange_blocks`' proven ones for its small block, with the object ids renamed.

- **Scene** `place_smallest_block` = a copy of `pick_block_size` with a target point; the small block is 3 cm, so the
  target z = 0.726 + 0.0147:
  ```json
  "objects": {..., "fix_objects": [{"data_info_dir": "fix_pose", "object_id": "fix_pose_target",
    "position": [-0.1, 0.0, 0.7407], "quaternion": [1, 0, 0, 0], "direction": [0, 0, -1]}]},
  "workspaces": {"work_table": {"blocked_zone": [[-0.21, 0.01], [-0.12, 0.12]]}}
  ```
- **Task** `place_smallest_block` (`"scene_file": "place_smallest_block"`), stages, each with an `action_description`:
  ```json
  {"action": "grasp", "group": 0, "passive": {"object_id": "geniesim_2025_building_blocks_smallest", "primitive": null},
   "extra_params": {"arm": "auto", "disable_upside_down": false, "flip_grasp": true, "grasp_offset": -0.007, "fixed_top_grasp": true,
                    "filter_grasp_pose": {"gripper_up_axis": "z", "threshold": 0.8, "target_direction": "-z"}}},
  {"action": "transition", "group": 0, "extra_params": {"arm": "auto", "target": "hold_offset",
   "carry_object_id": "geniesim_2025_building_blocks_smallest", "pick_up_distance": 0.07, "pick_up_direction": "z"}},
  {"action": "transition", "extra_params": {"arm": "auto", "target": "world", "carry_object_id": "geniesim_2025_building_blocks_smallest",
   "target_object_id": "fix_pose_target", "target_world_offset": [0, 0, 0.09], "level_object": "y", "max_spin_deg": 180, "motion_type": "AvoidObs"}},
  {"action": "transition", "group": 2, "extra_params": {"arm": "auto", "target": "world", "carry_object_id": "geniesim_2025_building_blocks_smallest",
   "target_object_id": "fix_pose_target", "target_world_offset": [0, 0, 0.002], "motion_type": "Simple"}},
  {"action": "ungrasp", "group": 2, "extra_params": {"arm": "auto"}},
  {"action": "transition", "extra_params": {"arm": "auto", "target": "world", "target_world_offset": [0, 0, 0.05], "motion_type": "Simple"}},
  {"action": "transition", "extra_params": {"arm": "auto", "target": "reset"}}
  ```
- **Metric**: the template's two `is_gripper_in_view` rules, then
  `{"rule_name": "is_object_end_in_region", "result_code": 1, "params": {"objects": ["geniesim_2025_building_blocks_smallest"], "region_center_from": "fix_pose_target", "region_size": [0.1, 0.1, 0.1]}}`.
  `region_center_from` names a `fix_pose` object; the layout generator writes its world position as `region_center`.
- **Instruction**: `english_task_name` = "Place the smallest building block at the target position".

Result: `check` → `G T[hold_offset>world>world] U T[world>reset]`, 2/2 ok; layouts 10/10; collection 2/2 attempts
passed, LeRobot written; `video place_smallest_block` renders the episodes.
