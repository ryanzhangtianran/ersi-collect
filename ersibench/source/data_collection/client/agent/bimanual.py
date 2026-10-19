"""Agent execution for paired dual-arm action sequences."""

import numpy as np

from common.base_utils.logger import logger


def step_bimanual(self, action_pair, stage, objects, workspaces, stage_id, index, step_index):
    left_action, right_action = action_pair
    left_stage = stage.left_stage
    right_stage = stage.right_stage

    objects = self.update_objects(objects, arm="left")
    left_target, left_motion, left_gripper, _, left_description, left_extra = left_stage.parse_action(
        left_action, objects
    )
    objects = self.update_objects(objects, arm="right")
    right_target, right_motion, right_gripper, _, right_description, right_extra = right_stage.parse_action(
        right_action, objects
    )
    # no target = the hand stays where it is (an ungrasp only releases)
    left_target = self.robot.get_ee_pose(id="left") if left_target is None else left_target
    right_target = self.robot.get_ee_pose(id="right") if right_target is None else right_target
    if left_extra.get("from_current_pose") or right_extra.get("from_current_pose"):
        logger.warning("Bimanual action does not support from_current_pose; use explicit paired targets")
        return False, True

    motion_type = left_motion if left_motion == right_motion else "Simple"
    if left_extra.get("goal_offset", [0, 0, 0, 1, 0, 0, 0]) != [0, 0, 0, 1, 0, 0, 0]:
        logger.warning("Bimanual action ignores left goal_offset; use explicit target poses")
    if right_extra.get("goal_offset", [0, 0, 0, 1, 0, 0, 0]) != [0, 0, 0, 1, 0, 0, 0]:
        logger.warning("Bimanual action ignores right goal_offset; use explicit target poses")

    self.robot.client.set_frame_state(
        "bimanual",
        step_index,
        left_stage.active_obj_id,
        left_stage.passive_obj_id,
        any(self.attached_obj_ids.values()),
        arm="left",
        target_pose=left_target,
        action_description=left_description,
    )
    self.robot.client.set_frame_state(
        "bimanual",
        step_index,
        right_stage.active_obj_id,
        right_stage.passive_obj_id,
        any(self.attached_obj_ids.values()),
        arm="right",
        target_pose=right_target,
        action_description=right_description,
    )

    # both sub-stages carry the same object: a closed chain, the hands must move as one rigid body
    carried = left_stage.extra_params.get("carry_object_id")
    rigid = carried is not None and carried == right_stage.extra_params.get("carry_object_id")
    if not self.robot.move_dual_pose(left_target, right_target, motion_type=motion_type, rigid=rigid):
        return False, True
    if stage.extra_params.get("synchronized_gripper", False):
        left_width = 0.1 if left_gripper == "open" else 0.0
        right_width = 0.1 if right_gripper == "open" else 0.0
        if left_gripper is not None or right_gripper is not None:
            response = self.robot.client.set_bimanual_gripper_state(
                left_gripper,
                right_gripper,
                left_width,
                right_width,
            )
            if not response.success:
                logger.warning("Synchronized gripper command failed: %s", response.message)
                return False, True
    else:
        self.robot.set_gripper_action(left_gripper, arm="left")
        self.robot.set_gripper_action(right_gripper, arm="right")

    objects = self.update_objects(objects, arm="left")
    objects["gripper"].obj_pose = self.robot.get_ee_pose(id="left")
    left_success = left_stage.check_completion(objects, self.robot)
    objects = self.update_objects(objects, arm="right")
    objects["gripper"].obj_pose = self.robot.get_ee_pose(id="right")
    right_success = right_stage.check_completion(objects, self.robot)
    step_success = left_success and right_success
    if not step_success:
        left_actual = self.robot.get_ee_pose(id="left")
        right_actual = self.robot.get_ee_pose(id="right")
        logger.warning(
            "Bimanual completion failed: left=%s right=%s "
            "left_target=%s left_actual=%s right_target=%s right_actual=%s",
            left_success,
            right_success,
            np.round(left_target[:3, 3], 4).tolist(),
            np.round(left_actual[:3, 3], 4).tolist(),
            np.round(right_target[:3, 3], 4).tolist(),
            np.round(right_actual[:3, 3], 4).tolist(),
        )
        return False, True

    for arm, sub_stage, gripper_action in (
        ("left", left_stage, left_gripper),
        ("right", right_stage, right_gripper),
    ):
        if sub_stage.action_type == "grasp" and gripper_action == "close":
            object_id = sub_stage.passive_obj_id
            self.attached_obj_ids[arm] = object_id
            if object_id.split("/")[0] not in self.articulated_objs:
                self.robot.client.attach_obj(
                    prim_paths=[objects[object_id].prim_path + ("?stable_grasp" if sub_stage.extra_params.get("stable_grasp") else "")],
                    is_right=arm == "right",
                )
        elif gripper_action == "open":
            self.attached_obj_ids[arm] = None
    self.attached_obj_id = next((obj for obj in self.attached_obj_ids.values() if obj), None)
    return True, True
