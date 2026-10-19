# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""A task's stages as the three zero-dimensional primitives: grasp, transition, ungrasp.

Task templates are written in these three. Layouts archived before 2026-10-09 (saved_task/) still say pick / lift / pull /
reset; expand_primitives rewrites those at load, split where the gripper state changes. Nothing downstream (stages, agent, recorder) sees the old names.

One transition may have several legs ({"action": "transition", "legs": [<transition>, ...]}): pick up, then over the
slot, then down. primitive_stages gives a task in that form (grasp, transition, ungrasp per object). A leg's target is
planned only once the leg before it has been executed (a place after a levelling lift sees the turned object), so each leg
runs as a stage of its own and legs retry like the parts of a pick / place (see `group`).
"""

import copy

from common.base_utils.logger import logger

PRIMITIVES = ("grasp", "transition", "ungrasp")


def _object_id(ref):
    return ref["object_id"] + ("/%s" % ref["part_id"] if "part_id" in ref else "")


def _gripper_stage(stage, action, extra_params):
    """A part of `stage` that only concerns the hand: same arm, same description, same injected error."""
    params = stage.get("extra_params", {})
    part = {"action": action, "extra_params": {"arm": params.get("arm", "right"), **extra_params}}
    if "error_data" in params:
        part["extra_params"]["error_data"] = params["error_data"]
    if "action_description" in stage:
        part["action_description"] = stage["action_description"]
    return part


def _retarget(stage, target, drop=()):
    params = {k: v for k, v in stage.get("extra_params", {}).items() if k not in drop}
    return {**stage, "action": "transition", "extra_params": {**params, "target": target}}


def _moves_nothing(params):
    offset = params.get("target_world_offset", [0, 0, params.get("lift_height", 0.0)])
    turns = ("carry_object_id", "target_position", "gripper_approach", "level_object")
    return not any(offset) and not any(key in params for key in turns)


def expand_primitives(stage, group=None):
    """One task-file stage -> the primitive stages that perform it. Parts of one stage share `group` (a failed part
    retries from the first part that still has candidates) and its checkers run after the last part."""
    action, params = stage["action"], stage.get("extra_params", {})
    if action in PRIMITIVES:
        return [stage]
    if action == "bimanual":
        pair = dict(stage)
        for arm in ("left", "right"):
            parts = expand_primitives(stage[arm])
            if len(parts) != 1:
                raise ValueError(f"bimanual {arm} stage '{stage[arm]['action']}' is {len(parts)} primitives; pair them one by one")
            pair[arm] = parts[0]
        return [pair]
    if action == "pick":
        parts = [{**stage, "action": "grasp"}]
        if params.get("pick_up_distance", 0.12) != 0 and params.get("error_data", {}).get("type") != "WrongTarget":
            parts.append(_gripper_stage(stage, "transition", {
                "target": "hold_offset",
                "carry_object_id": _object_id(stage["passive"]),
                "pick_up_distance": params.get("pick_up_distance", 0.12),
                "pick_up_direction": params.get("pick_up_direction", "z"),
            }))
    elif action in ("lift", "pull"):
        parts = [_retarget(stage, "world")]
        if params.get("gripper_action") == "open":
            release = _gripper_stage(stage, "ungrasp", {})
            parts = [release] if _moves_nothing(params) else [_retarget(stage, "world", drop=("gripper_action",)), release]
    elif action == "reset":
        parts = [_retarget(stage, "reset")]
    else:
        raise ValueError(f"Unknown action '{action}': use grasp / transition / ungrasp")
    checker = stage.get("checker", [])
    for part in parts:
        part.pop("checker", None)
        if len(parts) > 1:
            part["group"] = group
    parts[-1]["checker"] = checker
    return parts


def primitive_stages(stages):
    """A task's stages as grasp / transition / ungrasp, consecutive transitions of one arm fused into one with legs."""
    out = []
    for group, stage in enumerate(stages):
        for part in expand_primitives(stage, group):
            legs = part.get("legs") if part["action"] == "transition" else None
            for leg in legs or [part]:
                arm = leg.get("extra_params", {}).get("arm", "right")
                if legs is not None:
                    leg = {"group": group, **leg}  # authored legs retry among themselves
                last = out[-1] if out else None
                if leg["action"] == "transition" and last and last["action"] == "transition" and last["extra_params"]["arm"] == arm:
                    last["legs"].append(leg)
                elif leg["action"] == "transition":
                    out.append({"action": "transition", "extra_params": {"arm": arm}, "legs": [leg]})
                else:
                    out.append(leg)
    return out


def describe(stages):
    """'G T[hold_offset>place] U T[reset]' """
    def name(stage):
        if stage["action"] == "bimanual":
            return "bi(%s|%s)" % (name(stage["left"]), name(stage["right"]))
        legs = ">".join(leg.get("extra_params", {}).get("target", "world") for leg in stage.get("legs", [stage]))
        return stage["action"][0].upper() + ("[%s]" % legs if stage["action"] == "transition" else "")
    return " ".join(name(stage) for stage in stages)


class StageFactory:
    @staticmethod
    def create_stage(stage_config, objects):
        # imported here: the stage modules pull in the robot stack, and bimanual imports this module back
        from client.planner.action.bimanual import BimanualStage
        from client.planner.action.zero_dim.grasp import GraspStage
        from client.planner.action.zero_dim.transition import HoldOffsetTransition, TransitionStage
        from client.planner.action.zero_dim.ungrasp import UngraspStage

        action = stage_config["action"]
        target = stage_config.get("extra_params", {}).get("target", "world") if action == "transition" else None
        return {
            ("grasp", None): GraspStage,
            ("ungrasp", None): UngraspStage,
            ("bimanual", None): BimanualStage,
            ("transition", "world"): TransitionStage,
            ("transition", "reset"): TransitionStage,  # alone it is executed by the agent (robot.reset_pose)
            ("transition", "hold_offset"): HoldOffsetTransition,
        }[action, target](stage_config, objects)


class ActionScript:
    """The stages of one task, linked in order."""

    def __init__(self):
        self.stages = []

    def initialize(self, task_definition, objects):
        self.stages = []
        stages = primitive_stages(copy.deepcopy(task_definition.get("stages", [])))
        for stage in stages:
            for stage_config in stage.get("legs", [stage]):
                self.append_stage(StageFactory.create_stage(stage_config, objects))
        logger.info(f"ActionScript: {describe(stages)}")

    def __iter__(self):
        return iter(self.stages)

    def append_stage(self, stage):
        if self.stages:
            stage.set_previous_stage(self.stages[-1])
            self.stages[-1].set_next_stage(stage)
        self.stages.append(stage)


if __name__ == "__main__":
    text = {"action_text": "放", "english_action_text": "put"}
    block, slot = {"object_id": "block", "primitive": None}, {"object_id": "fix_pose_a", "primitive": None}
    pick = {"action": "pick", "active": {"object_id": "gripper"}, "passive": {"object_id": "kit", "part_id": "drawer"},
            "extra_params": {"arm": "left", "pick_up_direction": "x"}, "checker": [{"checker_name": "c"}], "action_description": text}
    grasp, up = expand_primitives(pick, 3)
    assert (grasp["action"], up["action"], up["extra_params"]["target"]) == ("grasp", "transition", "hold_offset")
    assert up["extra_params"] == {"arm": "left", "target": "hold_offset", "carry_object_id": "kit/drawer", "pick_up_distance": 0.12, "pick_up_direction": "x"}
    assert "checker" not in grasp and up["checker"] == [{"checker_name": "c"}] and grasp["group"] == up["group"] == 3
    assert up["action_description"] == text and "checker" in pick  # the task file's stage is not touched
    held = expand_primitives({**pick, "extra_params": {"pick_up_distance": 0.0}})
    assert [p["action"] for p in held] == ["grasp"] and "group" not in held[0] and held[0]["checker"] == pick["checker"]
    try:
        expand_primitives({"action": "place", "extra_params": {}})
        raise AssertionError("place and rotate are gone: a lift with target_object_id / level_object replaces them")
    except ValueError:
        pass
    let_go = {"action": "lift", "extra_params": {"arm": "left", "lift_height": 0.0, "gripper_action": "open", "motion_type": "Simple"}}
    assert [p["action"] for p in expand_primitives(let_go)] == ["ungrasp"]
    away = {"action": "lift", "extra_params": {"arm": "left", "target_world_offset": [-0.2, 0, 0.1], "gripper_action": "open"}}
    move, release = expand_primitives(away)
    assert "gripper_action" not in move["extra_params"] and release["action"] == "ungrasp"
    assert expand_primitives({"action": "reset", "extra_params": {"arm": "left"}})[0]["extra_params"] == {"arm": "left", "target": "reset"}
    pair = expand_primitives({"action": "bimanual", "left": {**pick, "extra_params": {"pick_up_distance": 0}}, "right": let_go})[0]
    assert (pair["left"]["action"], pair["right"]["action"]) == ("grasp", "ungrasp")
    try:
        expand_primitives({"action": "bimanual", "left": pick, "right": pick})
        raise AssertionError("a pick with a pick-up is two primitives")
    except ValueError:
        pass
    assert expand_primitives(grasp) == [grasp]
    carry = {"action": "lift", "extra_params": {"arm": "left", "carry_object_id": "block", "target_object_id": "fix_pose_a"}}
    reset = {"action": "reset", "extra_params": {"arm": "left"}}
    other = {"action": "lift", "extra_params": {"arm": "right", "lift_height": 0.1}}
    task = primitive_stages([pick, carry, {**away, "extra_params": {**away["extra_params"], "arm": "left"}}, reset, other])
    assert describe(task) == "G T[hold_offset>world>world] U T[reset] T[world]", describe(task)  # the other arm's move is its own transition
    assert [leg.get("group") for leg in task[1]["legs"]] == [0, None, 2]  # a failed pick-up regrasps; the release belongs to its lift
    assert describe(primitive_stages(task)) == describe(task)  # a task already written in primitives comes back as it is
    authored = primitive_stages([{"action": "transition", "legs": [{"action": "transition", "extra_params": {"arm": "left", "lift_height": 0.1}}]}])
    assert authored[0]["legs"][0]["group"] == 0
    print("ok")
