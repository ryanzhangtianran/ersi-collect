# Task Configuration Guide

This document describes how to create task configuration files (JSON format) for defining robot simulation tasks. Configuration files are parsed by `client/layout/task_generate.py` to generate specific task instances.

## Table of Contents

1. [Basic Structure](#basic-structure)
2. [Scene Origin (origin)](#scene-origin-origin)
3. [Object Configuration (objects)](#object-configuration-objects)
4. [Scene Configuration (scene)](#scene-configuration-scene)
5. [Robot Configuration (robot)](#robot-configuration-robot)
6. [Task Stages (stages)](#task-stages-stages)
7. [Other Configurations](#other-configurations)
8. [Complete Examples](#complete-examples)
9. [Quick Start](#quick-start)

---

## Basic Structure

A task is two JSON files: a **scene json** (where it happens) and a **task json** (what the robot does there).

`scenes/<scene>.json`:

```json
{
    "origin": { ... },           // Scene origin (optional)
    "objects": { ... },           // Object configuration (required)
    "scene": { ... },             // Background / work table (required)
    "robot": { ... },             // Robot configuration (optional)
    "workspaces": { ... },        // Workspace overrides (optional)
    "generalization": { ... },    // Randomisation tiers (optional)
    "recording_setting": { ... }  // Recording settings (optional)
}
```

`tasks/<task>/g2/<task>.json`:

```json
{
    "scene_file": "<scene>",      // scenes/<scene>.json
    "task": "任务名称",
    "stages": [ ... ],            // Task stages (required)
    "task_description": { ... },  // Task description (optional)
    "task_metric": { ... }        // Task evaluation metrics (optional)
}
```

`common/task_file.load_task` merges them (scene first, then the task), so a task json may also carry any scene key to
override it for its own case, e.g. its own `recording_setting`. Several tasks can name one scene; derive a variant of a
scene by copying its json under a new name and editing it. The stages and metric refer to the scene's objects by
`object_id`. A file without `scene_file` is read as one whole configuration (saved layouts, older templates, the
worker's per-round `task.json`). Every reader goes through `load_task`: the collector, `generate_layout.py`,
`run_saved_task_collection.py`, `collect_worker.sh`, `scripts/build_eval_scenes.py` and the HTTP collect service.

---

## Scene Origin (origin)

The `origin` field defines the global origin position of the scene. **All object and robot positions will be transformed relative to this origin**.

### Structure

```json
{
    "origin": {
        "position": [2.91, 0.76, 0.0],
        "quaternion": [1, 0, 0, 0]
    }
}
```

### Field Description

- **position** (required): Origin position in world coordinates [x, y, z] (unit: meters)
- **quaternion** (required): Origin rotation (quaternion format w, x, y, z)

### Important Notes

- If `origin` is configured, all workspace, object, and robot positions will be transformed relative to `origin` first, then converted to world coordinates
- If `origin` is not configured, it defaults to `position: [0, 0, 0]` and `quaternion: [1, 0, 0, 0]`
- **Recommendation for choosing origin position**: Usually choose the midpoint of the robot's operating space in front of it, such as the center of a table. This simplifies the configuration of subsequent workspaces and object positions

---

## Object Configuration (objects)

The `objects` field defines the configuration of all objects in the scene, containing the following sub-fields:

### Structure

```json
{
    "objects": {
        "task_related_objects": [ ... ],  // Task-related objects (required)
        "scene_objects": [ ... ],         // Scene objects (optional)
        "attach_objects": [ ... ],        // Attached objects (optional)
        "fix_objects": [ ... ],           // Fixed position objects (optional)
        "constraints": null              // Constraints (optional)
    }
}
```

### 1. task_related_objects (Task-Related Objects)

Main objects involved in task execution. These objects will be placed with priority.

#### Basic Format

```json
{
    "object_id": "unique_object_id",
    "data_info_dir": "objects/path/to/object",
    "workspace_id": "work_table",
    "mass": 0.05
}
```

`mass`, `static_friction`, `dynamic_friction`, `material`, `model_type` and `add_rigid_body` written on an object reach the
simulator even when the asset's `object_parameters.json` lacks them (other keys only override what the asset defines).

#### Inline grasp labels (interaction)

Assets without `assets/interaction/<name>/` (e.g. `benchmark_pot_007`) can carry their labels in the template; nothing is
written into `assets/`. Poses use the convention of the `.pkl` files: object frame, annotation gripper axes
(`robot gripper rotation = annotation rotation @ robot_gripper_2_grasp_gripper`). The part name is what a stage's
`passive.primitive` selects.

```json
"interaction": {"passive": {"grasp": {"left_handle": {"grasp_pose": [[[...4x4...]], ...], "width": [0.03, ...]}}}}
```

#### Candidate Objects (candidate_objects)

If the same `object_id` needs to be randomly selected from multiple candidates, use `candidate_objects`:

```json
{
    "object_id": "geniesim_2025_storage_box_2",
    "candidate_objects": [
        {
            "data_info_dir": "objects/benchmark/storage_box/benchmark_storage_box_006/",
            "object_id": "geniesim_2025_storage_box_006_green"
        },
        {
            "data_info_dir": "objects/benchmark/storage_box/benchmark_storage_box_007/",
            "object_id": "geniesim_2025_storage_box_007_blue"
        }
    ],
    "mass": 1000,
    "workspace_id": "box_poses"
}
```

**Notes**:
- The system will randomly select one from `candidate_objects`
- External fields (such as `mass`, `workspace_id`) will override corresponding fields in candidate objects
- The `object_id` used is the one defined externally, not the one in candidate objects

#### Workspace Relative Position

If an object needs to be placed relative to a workspace:

```json
{
    "object_id": "geniesim_2025_storage_box_2",
    "workspace_id": "box_poses",
    "workspace_relative_position": [0.0, 0.0, 0.0],
    "workspace_relative_orientation": [0.5, 0.5, 0.5, 0.5]
}
```

### 2. scene_objects (Scene Objects)

Background objects used to increase scene complexity, supporting sampling configuration.

#### Sampling Configuration

```json
{
    "sample": {
        "min_num": 2,      // Minimum sampling count
        "max_num": 7,      // Maximum sampling count
        "max_repeat": 1    // Maximum repeat count
    },
    "workspace_id": "work_table",
    "available_objects": [
        {
            "data_info_dir": "objects/benchmark/apple/benchmark_apple_000/",
            "object_id": "geniesim_2025_apple_000",
            "mass": 0.05
        },
        {
            "data_info_dir": "objects/benchmark/orange/benchmark_orange_001/",
            "object_id": "geniesim_2025_orange_001",
            "mass": 0.05
        }
    ]
}
```

**Notes**:
- `sample` defines sampling rules
- `available_objects` is the candidate object list
- The system will randomly sample `min_num` to `max_num` objects from `available_objects`
- Nested sampling is supported (`available_objects` can also contain object groups with `sample`)

### 3. attach_objects (Attached Objects)

Objects attached to other objects, such as items placed in boxes.

```json
{
    "sample": {
        "min_num": 1,
        "max_num": 2,
        "max_repeat": 2
    },
    "anchor_info": {
        "anchor_object": "geniesim_2025_target_storage_box",  // Anchor object ID
        "position": [0, 0.07, 0.04],                          // Relative position
        "quaternion": [1, 0, 0, 0],                          // Relative rotation (optional)
        "random_range": [0.04, 0.0, 0.04]                    // Random range (optional)
    },
    "workspace_id": "work_table",
    "available_objects": [ ... ]
}
```

**Notes**:
- `anchor_object` specifies the object ID to attach to
- `position` and `quaternion` define the pose relative to the anchor object
- `random_range` allows random offset within the specified range

### 4. fix_objects (Fixed Position Objects)

Fixed position objects. Format is the same as `task_related_objects`, but position is directly specified by configuration.

### Field Description

All object types support the following fields:

- **object_id** (required): Unique identifier for the object
- **data_info_dir** (required): Object data directory path, relative to the `$SIM_ASSETS` environment variable
- **workspace_id** (optional): Workspace ID, specifying the workspace where the object is placed
- **mass** (optional): Object mass (kg)
- **workspace_relative_position** (optional): Position relative to workspace [x, y, z]
- **workspace_relative_orientation** (optional): Rotation relative to workspace (quaternion w, x, y, z)
- **chinese_semantic_name** (optional): Chinese semantic name, can be a list (random selection)
- **english_semantic_name** (optional): English semantic name, can be a list (random selection)
- **allow_duplicate** (optional): Whether to allow duplicates (default `false`)

---

## Scene Configuration (scene)

The `scene` field defines the basic information and workspaces of the scene.

### Structure

```json
{
    "scene": {
        "scene_id": "background/home_b/",
        "scene_info_dir": "background/home_b/",
        "scene_usd": "background/home_b/home_b_00.usda",  // or list
        "function_space_objects": { ... }
    }
}
```

### Field Description

- **scene_id** (required): Scene ID, used to match robot initial pose
- **scene_info_dir** (required): Scene information directory path
- **scene_usd** (required): USD scene file path, can be a string or list (random selection when list)
- **function_space_objects** (optional): Workspace definitions

### function_space_objects (Workspaces)

Defines workspace areas for object placement. Depending on the configuration method, the system uses two different layout generators (refer to `GeneratorType.SPACE` and `GeneratorType.SAMPLE`):

#### 1. Workspace Area Mode (SPACE)

When the workspace **does not contain** the `poses` field, the system uses `GeneratorType.SPACE` mode:

```json
{
    "work_table": {
        "position": [0.0, 0.01, 0.9],
        "quaternion": [1, 0, 0, 0],
        "size": [0.5, 0.8, 0.2]
    }
}
```

**Characteristics**:
- Multiple objects will be **simultaneously arranged** in this area
- The system uses a 2D layout solver (`LayoutSolver2D`) to automatically calculate positions
- Positions are selected within the area with **collision avoidance** as the principle
- Suitable for scenarios where multiple objects need to be randomly distributed in the same area (e.g., fruits to be grasped placed on a table, can be randomly arranged within the area)

**Field Description**:
- `position`: Workspace area center position [x, y, z]
- `quaternion`: Workspace area rotation (quaternion w, x, y, z)
- `size`: Workspace area dimensions [x, y, z] (unit: meters)
- `blocked_zone` (optional): Prohibited placement areas

#### 2. Candidate Position Mode (SAMPLE)

When the workspace **contains** the `poses` field, the system uses `GeneratorType.SAMPLE` mode:

```json
{
    "box_poses": {
        "poses": [
            {
                "position": [0.11, 0.09, 0.9],
                "quaternion": [1, 0, 0, 0],
                "random": {
                    "delta_position": [0.03, 0.0, 0]
                }
            },
            {
                "position": [0.11, -0.1, 0.9],
                "quaternion": [1, 0, 0, 0],
                "random": {
                    "delta_position": [0.03, 0.0, 0]
                }
            }
        ]
    }
}
```

**Characteristics**:
- Multiple objects will **sample positions** from candidate positions in the `poses` list
- **Different objects will not be placed at the same position** (each pose is assigned to at most one object)
- If the number of objects exceeds the number of poses, the system will report an error
- Each object randomly selects one from candidate positions, and can only perform small random offsets near the point through `random.delta_position`
- Suitable for scenarios where objects need to be placed at specific position points (e.g., multiple boxes need to be placed left and right on a table, need to sample placement points from two positions)

**Field Description**:
- `poses`: Candidate position list, each element contains:
  - `position`: Position [x, y, z]
  - `quaternion`: Rotation (quaternion w, x, y, z)
  - `random` (optional): Random offset configuration
    - `delta_position`: Position random range [x, y, z]
    - `delta_angle`: Angle random range (radians)
  - `chinese_position_semantic` (optional): Chinese position semantic name
  - `english_position_semantic` (optional): English position semantic name

**Comparison of Two Modes**:

| Feature | SPACE (Workspace Area) | SAMPLE (Candidate Positions) |
|---------|----------------------|----------------------------|
| Configuration | Contains `size`, does not contain `poses` | Contains `poses` array |
| Object Placement | Simultaneously arranged in area, can be random within area | Sampled from candidate positions, can only be random near points |
| Position Calculation | 2D layout solver automatically calculates | Selected from predefined positions |
| Collision Detection | Automatically avoids collisions | Does not reuse the same position |
| Use Case | Fruits to be grasped placed on table, can be randomly arranged in area | Multiple boxes need to be placed left and right on table, need to sample placement points from two positions |

---

## Robot Configuration (robot)

The `robot` field defines the robot configuration and initial state.

### Structure

```json
{
    "robot": {
        "arm": "dual",              // "left", "right", "dual"
        "robot_id": "G2",           // "G1", "G2"
        "robot_cfg": "G2_omnipicker_fixed_dual.json",
        "robot_init_pose": { ... },
        "init_arm_pose": { ... },
        "init_arm_pose_noise": { ... }
    }
}
```

### robot_init_pose (Robot Initial Pose)

```json
{
    "robot_init_pose": {
        "position": [-0.66, 0.0, 0.0],
        "quaternion": [1, 0, 0, 0],
        "random": {
            "delta_position": [0.06, 0.06, 0]
        }
    }
}
```

**Notes**:
- `position` and `quaternion` define the initial pose of the robot base
- `random.delta_position` allows random position offset within the specified range (half-widths). It is drawn per layout
  (`TaskGenerator.draw_robot_init_pose`) and the agent moves the robot there before the episode, so every episode has its
  own base pose. The pose actually used is written into every layout as `robot_init_pose` (world frame).

### init_arm_pose (Arm Initial Joint Angles)

Define the initial angles (radians) of each robot joint:

```json
{
    "init_arm_pose": {
        "idx21_arm_l_joint1": 0.739033,
        "idx22_arm_l_joint2": -0.717023,
        "idx61_arm_r_joint1": -0.739033,
        "idx62_arm_r_joint2": -0.717023,
        // ... other joints
        "idx41_gripper_l_outer_joint1": 0.85,
        "idx81_gripper_r_outer_joint1": 0.85
    }
}
```

### init_arm_pose_noise (Joint Angle Noise)

Add random noise to joint angles:

```json
{
    "init_arm_pose_noise": {
        ".*_arm_.*": {
            "noise_type": "uniform",
            "low": -0.05,
            "high": 0.05
        }
    }
}
```

**Notes**:
- Keys are regular expressions matching joint names
- `noise_type` can be `"uniform"` (uniform distribution)
- `low` and `high` define the noise range

### generalization (Domain Randomization, two tiers)

Top-level block, present in every task template, one entry per RSIBench OOD axis. `in_domain` is always applied
(small); an axis listed in `ERSIBENCH_OOD_AXES` (comma-separated, RSIBench axis names) replaces its entry with the
`ood` one (at or beyond the eval magnitude). `common/generalization.py` folds the selected tier into the fields the
collector already reads, at launch; the layout records `ood_axes`.

```json
{
    "generalization": {
        "in_domain": {
            "lights": {"intensity_ratio": 0.15, "temperature_ratio": 0.10},
            "camera": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 2.0, "pitch": 2.0, "yaw": 2.0}
        },
        "ood": {
            "light":              {"lights": {"intensity": [6000, 8000, 12000, 16000, 20000], "temperature": [2500, 3000, 6000, 9000, 11000]}},
            "camera_pose":        {"camera": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 15.0, "pitch": 15.0, "yaw": 15.0}},
            "robot_initial_pose": {"init_base": {"delta_position": [0.15, 0.15, 0]}, "init_joint": {"thresh": 0.15}},
            "object_position":    {"sub_workspace": {"x": {"min": 0.0, "max": 1.0}, "y": {"min": 0.0, "max": 1.0}}, "range_scale": 2.0},
            "unseen_object":      {"candidate_objects": {}}
        }
    }
}
```

| axis | in-domain source | what the `ood` entry overrides | eval magnitude |
|---|---|---|---|
| light | `lights` ratio: every scene light's own intensity/temperature × U(1−r, 1+r), per episode | absolute pick from `intensity` × `temperature`, all lights | {8000,12000,16000} × {3000,6000,9000} |
| camera_pose | `camera`: offset on each `recording_setting.camera_list` prim's local pose, per episode (m / deg, parent frame) | same, rotation 15°; no translation | ±0.02 m, ±10° |
| robot_initial_pose | the template's own `robot.robot_init_pose.random` (base, drawn per layout; the agent moves the robot there each episode) and `robot.init_arm_pose_noise` (per episode) | both replaced by `init_base.delta_position` / uniform ±`init_joint.thresh` on `.*_arm_.*` | base ±0.10 m, joints ±0.1 rad |
| object_position | the layout's own `sub_workspace` / `random_range` | every object's and distractor group's `sub_workspace`; `random_range` × `range_scale` | ±0.04 m shift |
| unseen_object | the template's `candidate_objects` | `candidate_objects[object_id]` replaces that object's pool (fill per task; peaches are the eval hold-out) | blocks → peaches |

Keep camera translation at 0: the G2 head camera sits 4 mm behind its shell (inside the 1 cm near clip), with only
6 mm of free space backwards (−x) — a larger backward offset renders the shell and the head image is black (ray-cast of
`robot_fix.usda`, 2026-09-23). Even 3–5 mm backwards vignettes the head and the left wrist camera to a circle (episodes
recorded 2026-10-19 with x = −4.8 / −3.3 mm), so the old 5 mm in-domain noise spoilt about half the episodes.
Rotation carries the camera-pose OOD: 2 cm at 50 cm is ≈ 2°, 15° is not.

The camera offset and the light retune are composed on the scene's original values (`?local_delta` / `*` on the
`set_object_pose` / `set_light` prim path), so nothing accumulates across episodes.

---

## Task Stages (stages)

`stages` is an array that defines the various stages of task execution.

### Basic Structure

```json
{
    "stages": [
        {
            "action": "grasp",          // grasp | transition | ungrasp | bimanual
            "action_description": { ... },
            "active": { ... },          // Active object
            "passive": { ... },         // Passive object
            "extra_params": { ... },    // Extra parameters
            "checker": [ ... ]          // Checker (optional)
        }
    ]
}
```

### Action Types

A stage is one of three primitives (`client/planner/action/action_script.py`), or a `bimanual` pair of them:

- **grasp**: approach and close on `passive.object_id`. Grasp selection params: see [pick Action](#pick-action).
- **transition**: the hand moves, gripper state unchanged. `extra_params.target`:
  - `"world"` (default): `carry_object_id` (the held object must follow; omit to move the empty hand) to
    `target_object_id` + `target_object_offset` (that object's frame) or `target_position` (`"origin"` = where it was
    grasped), plus `target_world_offset` (or `lift_height`); `level_object` / `level_either_end` / `max_spin_deg` stand
    it upright on the way; `gripper_approach` + `gripper_close` turn the hand; `motion_type` (`Simple` / `AvoidObs`),
    `via_jitter`, `min_upright`, `pos_threshold`. Full list: `TransitionStage` docstring in `zero_dim/transition.py`.
  - `"hold_offset"`: straight pick-up after a grasp: `carry_object_id`, `pick_up_distance` (0.12), `pick_up_direction` (`z`).
  - `"reset"`: arm back to its home pose.
  - `"legs": [<transition>, ...]`: several moves as one transition; each leg is planned after the previous one ran.
- **ungrasp**: open (`gripper_state`, default `open`), then back off by `retreat: [{"direction": [x,y,z], "distance": m}]`
  in the frame of `retreat_frame_object_id` (world when absent).
- **bimanual**: `left` + `right`, one primitive each (see [Coordinated dual-arm stages](#coordinated-dual-arm-stages)).

Stages with the same `"group": <n>` retry together: give a grasp and the transition that lifts the object one group, so a
failed pick-up goes back to the next grasp candidate. `pick` / `lift` / `pull` / `reset` are only read from layouts
archived before 2026-10-09 (`saved_task/`); `place`, `rotate`, `insert` are gone. Sections below that still name them
describe the same parameters on `grasp` (pick) and `transition` (lift, reset).

Check a task offline (stdlib python, no Isaac) against `skills/collect-episodes/task.schema.json`; prints each error, or
the stages as the collector runs them:

```bash
python3 skills/collect-episodes/task_schema.py source/data_collection/tasks/<TASK>/g2/<TASK>.json
# place_glue_stick_into_holder -> G T[hold_offset>world>world] U T[world>reset]
```

### active and passive

```json
{
    "active": {
        "object_id": "gripper",     // or specific object ID
        "primitive": null           // Grasp primitive (optional)
    },
    "passive": {
        "object_id": "geniesim_2025_target_grasp_object",
        "primitive": null
    }
}
```

**Notes**:
- `active` is the object performing the action (usually `"gripper"`)
- `passive` is the object being operated on
- `object_id` can be a string, list, or dictionary (when dictionary, selected based on `arm`)

### extra_params (Extra Parameters)

Different action types have different parameters. For detailed parameter descriptions, please refer to [Action Extra Parameters Detailed Description](#action-extra-parameters-detailed-description).

For unconstrained `Simple` motions, `ik_fallback_rotation_deg` bounds deterministic
tool-frame rotation alternatives (default 8 degrees; zero disables them; must be
finite and non-negative). The target position is never perturbed. `insert`, `rotate`, path-constrained, offset and
current-pose actions do not relax the target. If no IK candidate succeeds, the
action fails without issuing a motion command. These options do not change the
completion checker thresholds.

#### pick Action

```json
{
    "extra_params": {
        "arm": "auto",                    // "left", "right", "auto"
        "disable_upside_down": true,      // Disable upside-down grasping
        "flip_grasp": true,               // Flip grasp
        "grasp_offset": 0.01,            // Grasp offset
        "pick_up_distance": 0.1,          // Lift distance
        "grasp_upper_percentile": 75     // Grasp upper percentile
    }
}
```

#### reset Action

```json
{
    "extra_params": {
        "arm": "auto",
        "plan_type": "AvoidObs"           // "Simple", "AvoidObs"
    }
}
```

**Note**: For complete parameter lists and descriptions of each action, please refer to [Action Extra Parameters Detailed Description](#action-extra-parameters-detailed-description).

### action_description (Action Description)

Used to generate task description text:

```json
{
    "action_description": {
        "action_text": "{左/右}臂拿起桌面上的苹果",
        "english_action_text": "{Left/Right} arm picks up the apple on the table"
    }
}
```

**Placeholders**:
- `{左/右}` or `{Left/Right}` or `{left/right}`: Automatically replaced based on the arm used
- `{object:object_id}`: Replaced with the object's Chinese/English semantic name
- `{position:object_id}`: Replaced with the object's position semantic name

### checker (Checker)

Used to verify whether a stage has been successfully completed. For detailed checker parameter descriptions, please refer to [Runtime Checker Description](./common/data_filter/README.md#runtime-checker-runtime-checker).

**Basic Structure**:
```json
{
    "checker": [
        {
            "checker_name": "distance_to_target",
            "params": {
                "object_id": "geniesim_2025_target_grasp_object",
                "target_id": "gripper",
                "rule": "lessThan",
                "value": 0.08
            }
        }
    ]
}
```

**Available Checkers**:
- `distance_to_target`: Check distance between object and target
- `local_axis_angle`: Check angle between object's local axis and target vector
- `prim_clearance`: minimum surface distance between two groups of robot links, measured on their meshes (the planner only
  knows collision spheres). `{"prims_a": ["/G2/gripper_l"], "prims_b": ["/G2/gripper_r"], "rule": "greaterThan", "value": 0.004}`
  fails a handover stage whose hands touch; a failed checker plays the scene back and tries the next grasp candidate.

For detailed parameter descriptions and examples, please refer to [Runtime Checker Description](./common/data_filter/README.md#runtime-checker-runtime-checker).

---

## Other Configurations

### recording_setting (Recording Settings)

```json
{
    "recording_setting": {
        "camera_list": [
            "/G2/head_link3/head_front_Camera",
            "/G2/gripper_r_base_link/Right_Camera"
        ],
        "fps": 30,
        "num_of_episode": 8,          // attempt budget: failed attempts are deleted, not kept
        "num_of_success": 5,          // optional: stop early once this many episodes were kept
        "noised_probability": 0.1
    }
}
```

### task_description (Task Description)

```json
{
    "task_description": {
        "task_name": "将苹果放入对应的收纳盒中",
        "english_task_name": "sort the apple into the corresponding storage box",
        "init_scene_text": "机器人在桌面前，桌面上放着一个水果和两个盛有不同水果的收纳盒"
    }
}
```

Supports placeholders (same as `action_description`).

### task_metric (Task Evaluation Metrics)

Defines data filtering rules used to verify whether collected data meets quality requirements after data collection. For detailed filtering rule descriptions, please refer to [Data Filter Rules Description](./common/data_filter/README.md#data-filter-rules-filter-rules).

**Basic Structure**:
```json
{
    "task_metric": {
        "filter_rules": [
            {
                "rule_name": "is_gripper_in_view",
                "params": {
                    "camera": "head",
                    "gripper": "right",
                    "out_view_allow_time": 0.2
                },
                "result_code": 4
            },
            {
                "rule_name": "is_object_relative_position_in_target",
                "params": {
                    "objects": ["geniesim_2025_target_grasp_object"],
                    "target": "geniesim_2025_target_storage_box",
                    "relative_position_range": [[-0.06, 0.06], [-0.05, 0.05], [-0.12, 0.12]]
                },
                "result_code": 1
            }
        ]
    }
}
```

**Available Filter Rules**:
- `is_object_reach_target`: Check if object reaches target area
- `is_object_pose_similar2start`: Check if object pose is similar to initial
- `is_object_in_view`: Check if object is within image bounds
- `is_gripper_in_view`: Check if gripper is in camera view. `downsample_ratio` keeps every n-th frame in order;
  `out_view_allow_time` is the longest time (seconds) the gripper may be out of view in a row (same for `is_object_in_view`)
- `is_object_end_pose_up`: Check if object's final pose is vertically upward
- `is_object_end_higher_than_start`: Check if object's final position is higher than initial position (an object id missing
  from the recording fails the rule)
- `is_objects_distance_greater_than`: Check distance between objects
- `is_object_end_in_region`: Check if object's final position is in specified region
- `is_object_grasped_by_gripper`: Check if object is grasped by gripper
- `is_object_relative_position_in_target`: Check object's relative position to target object

For detailed parameter descriptions and examples, please refer to [Data Filter Rules Description](./common/data_filter/README.md#data-filter-rules-filter-rules).

---

## Complete Examples

For complete task configuration examples, please refer to configuration files in the `tasks/` directory, for example:
- `tasks/pick_biggest_block/g2/pick_biggest_block.json` - Fruit sorting task
- Other task configuration files

---

## Quick Start

### Creating Task Configuration from Scratch

This section uses `tasks/pick_biggest_block/g2/pick_biggest_block.json` as an example to describe in detail how to create a task configuration from scratch.

#### 1. Configure Scene and Origin (origin)

**Design Rationale**: First, you need to select a scene file and determine the scene origin. The origin is usually chosen as the midpoint of the operating space in front of the robot (such as the center of a table), which simplifies the configuration of subsequent workspaces and object positions.

```json
{
    "origin": {
        "position": [2.91, 0.76, 0.0],
        "quaternion": [1, 0, 0, 0]
    },
    "scene": {
        "scene_id": "background/home_b/",
        "scene_info_dir": "background/home_b/",
        "scene_usd": ["background/home_b/home_b_00.usda"],
        "function_space_objects": {
            "work_table": {
                "position": [-0.15, 0.01, 0.9],
                "quaternion": [1, 0, 0, 0],
                "size": [0.24, 0.8, 0.2]
            },
            "box_poses": {
                "poses": [
                    {
                        "position": [0.11, 0.09, 0.9],
                        "quaternion": [1, 0, 0, 0],
                        "random": {
                            "delta_position": [0.03, 0.0, 0]
                        }
                    },
                    {
                        "position": [0.11, -0.1, 0.9],
                        "quaternion": [1, 0, 0, 0],
                        "random": {
                            "delta_position": [0.03, 0.0, 0]
                        }
                    }
                ]
            }
        }
    }
}
```

**Notes**:
- `work_table` uses SPACE mode (contains `size`), used for placing fruits to be grasped, can be randomly arranged within the area
- `box_poses` uses SAMPLE mode (contains `poses`), used for placing two storage boxes, sampling from two candidate positions

#### 2. Configure Robot

```json
{
    "robot": {
        "arm": "dual",
        "robot_id": "G2",
        "robot_cfg": "G2_omnipicker_fixed_dual.json",
        "robot_init_pose": {
            "position": [-0.66, 0.0, 0.0],
            "quaternion": [1, 0, 0, 0],
            "random": {
                "delta_position": [0.06, 0.06, 0]
            }
        },
        "init_arm_pose": {
            "idx21_arm_l_joint1": 0.739033,
            "idx22_arm_l_joint2": -0.717023,
            // ... other joints
        },
        "init_arm_pose_noise": {
            ".*_arm_.*": {
                "noise_type": "uniform",
                "low": -0.05,
                "high": 0.05
            }
        }
    }
}
```

#### 3. Configure Task-Related Objects (task_related_objects)

**Design Rationale**: The task needs to grasp an apple and place it into the corresponding storage box. Therefore, we need to configure:
- Target grasp object: Randomly select one from multiple apple assets
- Two storage boxes: Randomly select from multiple storage box assets, use SAMPLE mode to sample placement from two candidate positions

```json
{
    "objects": {
        "task_related_objects": [
            {
                "object_id": "geniesim_2025_target_grasp_object",
                "candidate_objects": [
                    {
                        "data_info_dir": "objects/benchmark/apple/benchmark_apple_000/",
                        "object_id": "geniesim_2025_apple_000"
                    },
                    {
                        "data_info_dir": "objects/benchmark/apple/benchmark_apple_001/",
                        "object_id": "geniesim_2025_apple_001"
                    },
                    {
                        "data_info_dir": "objects/benchmark/apple/benchmark_apple_002/",
                        "object_id": "geniesim_2025_apple_002"
                    },
                    {
                        "data_info_dir": "objects/benchmark/apple/benchmark_apple_003/",
                        "object_id": "geniesim_2025_apple_003"
                    }
                ],
                "mass": 0.05,
                "workspace_id": "work_table"
            },
            {
                "object_id": "geniesim_2025_target_storage_box",
                "candidate_objects": [
                    {
                        "data_info_dir": "objects/benchmark/storage_box/benchmark_storage_box_006/",
                        "object_id": "geniesim_2025_storage_box_006_green"
                    },
                    {
                        "data_info_dir": "objects/benchmark/storage_box/benchmark_storage_box_007/",
                        "object_id": "geniesim_2025_storage_box_007_blue"
                    },
                    {
                        "data_info_dir": "objects/benchmark/storage_box/benchmark_storage_box_008/",
                        "object_id": "geniesim_2025_storage_box_008_white"
                    },
                    {
                        "data_info_dir": "objects/benchmark/storage_box/benchmark_storage_box_009/",
                        "object_id": "geniesim_2025_storage_box_009_red"
                    },
                    {
                        "data_info_dir": "objects/benchmark/storage_box/benchmark_storage_box_010/",
                        "object_id": "geniesim_2025_storage_box_010_grey"
                    },
                    {
                        "data_info_dir": "objects/benchmark/storage_box/benchmark_storage_box_011/",
                        "object_id": "geniesim_2025_storage_box_011_black"
                    }
                ],
                "mass": 1000,
                "workspace_id": "box_poses",
                "workspace_relative_position": [0.0, 0.0, 0.0],
                "workspace_relative_orientation": [0.5, 0.5, 0.5, 0.5]
            },
            {
                "object_id": "geniesim_2025_storage_box_2",
                "candidate_objects": [
                    {
                        "data_info_dir": "objects/benchmark/storage_box/benchmark_storage_box_006/",
                        "object_id": "geniesim_2025_storage_box_006_green"
                    },
                    // ... other storage box options
                ],
                "mass": 1000,
                "workspace_id": "box_poses",
                "workspace_relative_position": [0.0, 0.0, 0.0],
                "workspace_relative_orientation": [0.5, 0.5, 0.5, 0.5]
            }
        ]
    }
}
```

#### 4. Configure Attached Objects (attach_objects)

**Design Rationale**: To increase scene realism and complexity, we need to place 1-2 apples in the target storage box, and 1-2 other types of fruits (peaches, oranges, lemons, pomegranates, etc.) in another storage box. This demonstrates nested sampling: outer layer samples from multiple fruit groups, inner layer samples from multiple assets in each fruit group.

```json
{
    "objects": {
        "attach_objects": [
            {
                "sample": {
                    "min_num": 1,
                    "max_num": 2,
                    "max_repeat": 2
                },
                "anchor_info": {
                    "anchor_object": "geniesim_2025_target_storage_box",
                    "position": [0, 0.07, 0.04],
                    "quaternion": [1, 0, 0, 0],
                    "random_range": [0.04, 0.0, 0.04]
                },
                "workspace_id": "work_table",
                "available_objects": [
                    {
                        "data_info_dir": "objects/benchmark/apple/benchmark_apple_000/",
                        "object_id": "geniesim_2025_apple_000"
                    },
                    {
                        "data_info_dir": "objects/benchmark/apple/benchmark_apple_001/",
                        "object_id": "geniesim_2025_apple_001"
                    },
                    {
                        "data_info_dir": "objects/benchmark/apple/benchmark_apple_002/",
                        "object_id": "geniesim_2025_apple_002"
                    },
                    {
                        "data_info_dir": "objects/benchmark/apple/benchmark_apple_003/",
                        "object_id": "geniesim_2025_apple_003"
                    }
                ]
            },
            {
                "sample": {
                    "min_num": 1,
                    "max_num": 1,
                    "max_repeat": 1
                },
                "workspace_id": "work_table",
                "anchor_info": {
                    "anchor_object": "geniesim_2025_storage_box_2",
                    "position": [0, 0.12, 0],
                    "quaternion": [1, 0, 0, 0],
                    "random_range": [0.04, 0.0, 0.04]
                },
                "available_objects": [
                    {
                        "sample": {
                            "min_num": 1,
                            "max_num": 2,
                            "max_repeat": 2
                        },
                        "workspace_id": "work_table",
                        "available_objects": [
                            {
                                "data_info_dir": "objects/benchmark/peach/benchmark_peach_000/",
                                "object_id": "geniesim_2025_peach_000"
                            },
                            {
                                "data_info_dir": "objects/benchmark/peach/benchmark_peach_019/",
                                "object_id": "geniesim_2025_peach_019"
                            },
                            {
                                "data_info_dir": "objects/benchmark/peach/benchmark_peach_020/",
                                "object_id": "geniesim_2025_peach_020"
                            },
                            {
                                "data_info_dir": "objects/benchmark/peach/benchmark_peach_021/",
                                "object_id": "geniesim_2025_peach_021"
                            }
                        ]
                    },
                    {
                        "sample": {
                            "min_num": 1,
                            "max_num": 2,
                            "max_repeat": 2
                        },
                        "workspace_id": "work_table",
                        "available_objects": [
                            {
                                "data_info_dir": "objects/benchmark/orange/benchmark_orange_001/",
                                "object_id": "geniesim_2025_orange_001"
                            },
                            {
                                "data_info_dir": "objects/benchmark/orange/benchmark_orange_002/",
                                "object_id": "geniesim_2025_orange_002"
                            },
                            {
                                "data_info_dir": "objects/benchmark/orange/benchmark_orange_004/",
                                "object_id": "geniesim_2025_orange_004"
                            }
                        ]
                    },
                    {
                        "sample": {
                            "min_num": 1,
                            "max_num": 2,
                            "max_repeat": 2
                        },
                        "workspace_id": "work_table",
                        "available_objects": [
                            {
                                "data_info_dir": "objects/benchmark/lemon/benchmark_lemon_027/",
                                "object_id": "geniesim_2025_lemon_027"
                            },
                            {
                                "data_info_dir": "objects/benchmark/lemon/benchmark_lemon_028/",
                                "object_id": "geniesim_2025_lemon_028"
                            },
                            {
                                "data_info_dir": "objects/benchmark/lemon/benchmark_lemon_029/",
                                "object_id": "geniesim_2025_lemon_029"
                            },
                            {
                                "data_info_dir": "objects/benchmark/lemon/benchmark_lemon_030/",
                                "object_id": "geniesim_2025_lemon_030"
                            }
                        ]
                    }
                ]
            }
        ]
    }
}
```

#### 5. Configure Task Stages (stages)

**Design Rationale**: grasp the apple and lift it (one group), carry it over the storage box (transition), release (ungrasp), reset arm (transition to reset). Each stage needs to configure action type, action description, checker, etc.

```json
{
    "stages": [
        {
            "action": "grasp",
            "group": 0,
            "action_description": {
                "action_text": "{左/右}臂拿起桌面上的苹果",
                "english_action_text": "{Left/Right} arm picks up the apple on the table"
            },
            "active": {
                "object_id": "gripper",
                "primitive": null
            },
            "passive": {
                "object_id": "geniesim_2025_target_grasp_object",
                "primitive": null
            },
            "extra_params": {
                "arm": "auto",
                "disable_upside_down": true,
                "flip_grasp": true,
                "grasp_offset": 0.01,
                "grasp_upper_percentile": 75
            }
        },
        {
            "action": "transition",
            "group": 0,
            "action_description": {
                "action_text": "{左/右}臂拿起桌面上的苹果",
                "english_action_text": "{Left/Right} arm picks up the apple on the table"
            },
            "extra_params": {
                "arm": "auto",
                "target": "hold_offset",
                "carry_object_id": "geniesim_2025_target_grasp_object",
                "pick_up_distance": 0.1,
                "pick_up_direction": "z"
            },
            "checker": [
                {
                    "checker_name": "distance_to_target",
                    "params": {
                        "object_id": "geniesim_2025_target_grasp_object",
                        "target_id": "gripper",
                        "rule": "lessThan",
                        "value": 0.08
                    }
                }
            ]
        },
        {
            "action": "transition",
            "action_description": {
                "action_text": "将拿着的苹果归类到桌面上对应的收纳筐中",
                "english_action_text": "Place the apple into the corresponding storage bin"
            },
            "extra_params": {
                "arm": "auto",
                "carry_object_id": "geniesim_2025_target_grasp_object",
                "target_object_id": "geniesim_2025_target_storage_box",
                "target_world_offset": [
                    0,
                    0,
                    0.08
                ],
                "motion_type": "AvoidObs"
            }
        },
        {
            "action": "ungrasp",
            "action_description": {
                "action_text": "松开苹果",
                "english_action_text": "Release the apple"
            },
            "extra_params": {
                "arm": "auto",
                "retreat": [
                    {
                        "direction": [
                            0,
                            0,
                            1
                        ],
                        "distance": 0.1
                    }
                ]
            },
            "checker": [
                {
                    "checker_name": "distance_to_target",
                    "params": {
                        "object_id": "geniesim_2025_target_grasp_object",
                        "target_id": "geniesim_2025_target_storage_box",
                        "target_offset": {
                            "frame": "world",
                            "position": [
                                0,
                                0,
                                0.02
                            ]
                        },
                        "rule": "lessThan",
                        "value": 0.12
                    }
                }
            ]
        },
        {
            "action": "transition",
            "action_description": {
                "action_text": "{左/右}臂复位",
                "english_action_text": "{Left/Right} arm resets"
            },
            "extra_params": {
                "arm": "auto",
                "target": "reset"
            }
        }
    ]
}
```

#### 6. Configure Task Evaluation Metrics (task_metric)

**Design Rationale**: After data collection, we need to verify the quality of collected data, such as checking if the gripper is in view, if objects are successfully placed at target positions, etc.

```json
{
    "task_metric": {
        "filter_rules": [
            {
                "rule_name": "is_gripper_in_view",
                "params": {
                    "camera": "head",
                    "gripper": "right",
                    "out_view_allow_time": 0.2
                },
                "result_code": 4
            },
            {
                "rule_name": "is_gripper_in_view",
                "params": {
                    "camera": "head",
                    "gripper": "left",
                    "out_view_allow_time": 0.1
                },
                "result_code": 4
            },
            {
                "rule_name": "is_object_relative_position_in_target",
                "params": {
                    "objects": ["geniesim_2025_target_grasp_object"],
                    "target": "geniesim_2025_target_storage_box",
                    "relative_position_range": [[-0.06, 0.06], [-0.05, 0.05], [-0.12, 0.12]]
                },
                "result_code": 1
            }
        ]
    }
}
```

#### 7. Configure Other Settings

```json
{
    "task": "sort_the_fruit_into_the_box_apple_g2",
    "task_description": {
        "task_name": "将苹果放入对应的收纳盒中",
        "english_task_name": "sort the apple into the corresponding storage box",
        "init_scene_text": "机器人在桌面前，桌面上放着一个水果和两个盛有不同水果的收纳盒"
    },
    "recording_setting": {
        "camera_list": [
            "/G2/head_link3/head_front_Camera",
            "/G2/gripper_r_base_link/Right_Camera",
            "/G2/gripper_l_base_link/Left_Camera",
            "/G2/head_link3/head_right_Camera",
            "/G2/head_link3/head_left_Camera"
        ],
        "fps": 30,
        "num_of_episode": 8,
        "noised_probability": 0.1
    }
}
```

### Modifying Existing Configuration

If you already have a similar task configuration, you can find a similar task in the `tasks/` directory as a template, then modify the corresponding configuration items according to your needs.

---

## Frequently Asked Questions

### Q: How to implement random object selection?

A: Use the `candidate_objects` field, and the system will randomly select one from the candidates.

### Q: How to attach objects to other objects?

A: Use `attach_objects`, and specify the anchor object and relative position through `anchor_info`.

### Q: How to define multiple optional positions?

A: Use the `poses` array in workspace configuration. Note: Workspaces containing `poses` use SAMPLE mode, where multiple objects sample positions from candidate positions, and different objects will not be placed at the same position. If you need multiple objects arranged in the same area, use the workspace area mode (SPACE) containing `size`.

### Q: How to implement sampling multiple objects?

A: Use `sample` configuration in `scene_objects` or `attach_objects`, specifying `min_num` and `max_num`.

### Q: How to specify object paths?

A: `data_info_dir` is a path relative to the `$SIM_ASSETS` environment variable, for example `"objects/benchmark/apple/benchmark_apple_000/"`.

---

## Action Extra Parameters Detailed Description

Action extra parameters are defined in the `extra_params` field of `stages` to control the specific behavior of actions.

### pick Action

**Parameters**:
- `arm` (str, optional): Arm to use, `"left"`, `"right"` or `"auto"`, default `"right"`
- `grasp_offset` (float, optional): Grasp offset (unit: meters), default `0.03`
- `pre_grasp_offset` (float, optional): Pre-grasp offset (unit: meters), default `0.0`
- `grasp_lower_percentile` (float, optional): Grasp lower percentile (0-100), default `0`
- `grasp_upper_percentile` (float, optional): Grasp upper percentile (0-100), default `100`
- `disable_upside_down` (bool, optional): Disable upside-down grasping, default `false`
- `flip_grasp` (bool, optional): Flip grasp (180 degrees around z-axis), default `false`
- `pick_up_distance` (float, optional): Lift distance (unit: meters), default `0.12`. With `0` nothing follows the close, so
  instead of the follow check the pick verifies that the fingers closed on something: finger joint above `min_grasp_opening`
  (default `0.02` rad, ~0 when closed on nothing), else `GRASP_EMPTY` and the next grasp candidate is tried
- `pick_up_type` (str, optional): Lift type, `"Simple"` or `"AvoidObs"`, default `"Simple"`
- `use_near_point` (bool, optional): Whether to use nearby point, default `false`
- `error_data` (dict, optional): Error data configuration
  - `type` (str): Error type, such as `"RandomPerturbations"`, `"MissGrasp"`, `"WrongTarget"`, `"KeepClose"`
  - `params` (dict): Error parameters

**Example**:
```json
{
    "extra_params": {
        "arm": "auto",
        "disable_upside_down": true,
        "flip_grasp": true,
        "grasp_offset": 0.01,
        "pick_up_distance": 0.1,
        "grasp_upper_percentile": 75
    }
}
```

**Deterministic grasps** (`fixed_world_grasp`): for round objects whose own frame says nothing (fruit), and when the contact
points are decided up front (handover). Gripper z = approach, gripper x = finger closing axis, both given as WORLD vectors;
the grasp goes through the object origin (+ `grasp_offset` along the approach). Candidates are the closing axis and its 180 deg
flip, each fanned by `yaws_deg` about the approach. Every grasp (any task) tries its candidates nearest the current arm pose
in joint space first.

**Diversity** (both off by default; a scene is otherwise always grasped and moved the same way):
`grasp_topk_shuffle: k` on a pick draws the order of its k nearest-in-joint-space candidates at random (the rest stay as
fallbacks). `via_jitter: [x, y, z]` (metres) on a pick or a single-arm lift passes a random via point on the way: xy around
the midpoint of start and goal +- x / y, height 0..z above both ends; on a pick it precedes the pre-grasp. A bimanual
pair ignores `via_jitter` (its hands must move rigidly together).

```json
"fixed_world_grasp": {"approach": [0, 1, 0], "close": [0, 0, 1], "yaws_deg": [0, 10, -10]}
"fixed_world_grasp": {"approach": [0, -1, 0], "elevation_deg": [20, 70], "azimuth_deg": [-25, 25], "samples": 4, "close": [1, 0, 0]}
```

The second form draws the approach at random per attempt: `approach` is the horizontal heading, the hand comes down at the
drawn elevation and is swung about the vertical by the drawn azimuth (`tasks/handover_apple`).

**Carry lookahead**: a pick checks IK for the `lift` stages that follow with the same arm (carrying its object, or empty after
a release) exactly as `LiftStage` will move the hand, including the turn of `level_object` and `gripper_approach`. Stages of
the other arm are skipped (a handover). If no grasp candidate can reach one of them the pick yields nothing and logs
`CARRY_UNREACHABLE arm=... lift_stage=+N hand_target=...` before the robot moves; candidates that only fail some are dropped.

### reset Action

**Parameters**:
- `arm` (str, optional): Arm to use, `"left"`, `"right"` or `"auto"`, default `"right"`
- `plan_type` (str, optional): Planning type, `"Simple"` or `"AvoidObs"`, default `"AvoidObs"`

**Example**:
```json
{
    "extra_params": {
        "arm": "auto",
        "plan_type": "AvoidObs"
    }
}
```


## Coordinated dual-arm stages

Use `"action": "bimanual"` when both arms must advance through one shared action index. The stage contains nested `left` and `right` normal stage definitions, each with its own `active`, `passive`, and `extra_params.arm` (`"left"` or `"right"`). The nested action sequences must have equal lengths. Each pair is solved to per-arm IK targets, interpolated in joint space with normalized progress, and sent in one server RPC so both arms are applied in the same physics update.

Example shape:

```json
{
  "action": "bimanual",
  "active": {"object_id": "gripper", "primitive": null},
  "passive": {"object_id": "gripper", "primitive": null},
  "extra_params": {},
  "left": {
    "action": "grasp",
    "active": {"object_id": "gripper", "primitive": null},
    "passive": {"object_id": "left_handle", "primitive": null},
    "extra_params": {"arm": "left", "pick_up_distance": 0.0}
  },
  "right": {
    "action": "grasp",
    "active": {"object_id": "gripper", "primitive": null},
    "passive": {"object_id": "right_handle", "primitive": null},
    "extra_params": {"arm": "right", "pick_up_distance": 0.0}
  }
}
```

For a lift or carry phase, use one top-level `transition` stage with `extra_params.arm: "dual"`; the agent expands it into a `bimanual` pair of `transition` sub-stages, one per arm (any stage with `arm: "dual"` is expanded the same way). A `lift` stage translates the held object rigidly: it computes one world displacement `target - object` and applies it to the arm's current end-effector pose, so both arms of a pair get the identical delta and the object keeps its orientation. `extra_params`:

- `carry_object_id`: the object whose pose defines the displacement and whose final pose is checked (it must follow the hand and keep the
  orientation it had at the start of the stage). Omit it to move an empty hand by `target_world_offset` / `lift_height`; only the gripper
  is checked then. With `gripper_action: "open"` and no offset that is a release in place, e.g. the giving hand of a handover
  (`tasks/handover_apple`: pick left, lift to `handover_point`, pick right, release left, retreat, reset, carry right).
- `lift_height`: raise the object vertically by this many metres.
- `target_workspace_id`: move the object origin to this `function_space_objects` entry: a SPACE workspace's centre, or one of a SAMPLE workspace's poses (random choice, jittered by its `random.delta_position`). Omit to move relative to the object's current position.
- `target_prim_path`: instead of a workspace, take the world position of a prim of the loaded scene (its origin), read when the task is loaded, e.g.
  `"/World/Cube"`, the burner support of `kitchen/kitchen_1`. The target then follows the background catalogue instead of being typed
  into the task; add the vertical part (half the support's height + the object's origin above its base) with `target_world_offset`.
- `target_world_offset`: added on top of the target (or the current position); overrides `lift_height`.
- `motion_type`, `gripper_action`, `pos_threshold` (default `0.03`), `min_upright` (default `0.8`, cosine of the tilt allowed since the stage began).

- `gripper_approach` + `gripper_close` (world vectors, gripper z and x): single arm only, also turn the hand to that
  orientation while it moves; the carried object keeps its place in the hand and its origin still ends at the target
  (presents the apple sideways in `handover_apple`).
- `level_object: "y"`: turn the carried object's own up axis to world z by the smallest rotation, applied about the object's
  origin to BOTH hands (`place_pot_stove` levels a pot that starts leaning on the burner grate). Use this, not per-hand
  orientations, in a pair: per-hand targets are not one rigid motion and are refused (see below).
- `synchronized_gripper: true` on a `dual` stage: both grippers act in one command (release a two-handed object at the same instant).

**Two hands on one object are a closed chain.** When both sub-stages have the same `carry_object_id`, `move_dual_pose` treats
the grasps as fixed (measured at the start of every stage, so earlier slip is not inherited):
- the two hand targets must be the same rigid motion of the current hand poses, within 3 mm / 1 deg, or the move is refused
  with `BIMANUAL_NOT_RIGID ... disagree by X mm / Y deg`;
- the path is that motion's screw interpolation `exp(s log D)` applied to both hands (a waypoint every 1.5 cm / 3 deg), so the
  hands keep their relative pose all the way, not just at the goal;
- waypoint IK is warm-started from the previous waypoint's solution (the server does this for any batch of neighbouring poses);
  a jump of more than 0.5 rad between waypoints is refused as `BIMANUAL_IK_BRANCH_JUMP`;
- timing is a minimum-jerk profile; the server reports the move done when the arms arrive, and `BIMANUAL_BLOCKED x rad` when
  they stop more than 0.05 rad short (an arm held back by the object or an obstacle).
Free hands (no shared object) move on Cartesian waypoints for `Straight` (position lerp, orientation slerp) or straight to the
IK solution for `Simple`. Measured on the pot carry: relative hand drift 45-75 mm / 12-17 deg and 1-2.6 cm of slip in the
fingers before, 5-10 mm / 2-4 deg and 1.5-4.6 mm after.

Pinched wire handles hold vertically and along the bar only: carry such an object along its handle bars, not across them.

Completion requires the gripper within `pos_threshold` of its target and the carried object within `pos_threshold` of its target, tilted no more than `min_upright` allows since the stage began. Do not use `from_current_pose` or per-arm `goal_offset` in a coordinated pair.
