"""Transition: the hand moves, the gripper state does not change (zero-dimensional: the goal is one pose).

TransitionStage (target "world", the old lift / pull) moves the held object rigidly, up or to a target, and can stand it
upright on the way (level_object; with target_position "origin" it replaces the old rotate stage).
HoldOffsetTransition (target "hold_offset") is the pick-up that follows a grasp. The target "place" lives in place.py;
"reset" is executed by the agent.

Both arms of a bimanual pair compute the same world displacement
``target - object`` and apply it to their own end-effector pose, so the object
keeps its orientation and both hands keep their relative pose (same idea as
RoboTwin's ``move_by_displacement`` with an identical delta for both arms).
"""

import copy

import numpy as np

from client.planner.action.stage import Action, ActionSequence, Stage, simple_check_completion
from common.base_utils.logger import logger
from common.base_utils.transform_utils import pose_difference


def carry_displacement(object_pose, target_position=None, world_offset=(0, 0, 0)):
    """World translation that moves the object origin to target (+ offset)."""
    object_position = np.asarray(object_pose, dtype=float)[:3, 3]
    base = object_position if target_position is None else np.asarray(target_position, dtype=float)
    return base + np.asarray(world_offset, dtype=float) - object_position


def resting_position(rest_pose, size, rotation):
    """Where the object's origin is when it is put back on the spot it rested on (`rest_pose`), turned to `rotation`:
    same xy, and the height follows its vertical extent (a pen case stood up sits higher than it lay).
    Assumes the origin is the box centre."""
    extent = lambda R: float(np.abs(np.asarray(R, dtype=float)[2]) @ np.asarray(size, dtype=float))
    rest_pose = np.asarray(rest_pose, dtype=float)
    position = rest_pose[:3, 3].copy()
    position[2] += (extent(rotation) - extent(rest_pose[:3, :3])) / 2
    return position


def gripper_rotations(approach, close):
    """Both world rotations with gripper z = approach and gripper x = +-close (the same grip, flipped 180 deg)."""
    approach = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    close = np.asarray(close, dtype=float)
    close = close - approach * (close @ approach)
    close /= np.linalg.norm(close)
    return [np.column_stack([x, np.cross(approach, x), approach]) for x in (close, -close)]


def levelling_rotation(object_pose, up_axis="y", either_end=False):
    """Smallest world rotation that brings the object's own up axis ("x" / "y" / "z") to world z.
    either_end: a stick has no top, stand it on whichever end is nearer."""
    up = np.asarray(object_pose, dtype=float)[:3, "xyz".index(up_axis)]
    up = up / np.linalg.norm(up)
    if either_end and up[2] < 0:
        up = -up
    axis = np.cross(up, [0.0, 0.0, 1.0])
    sine, cosine = np.linalg.norm(axis), float(up[2])
    if sine < 1e-9:
        return np.eye(3)
    k = axis / sine
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + sine * K + (1 - cosine) * K @ K  # Rodrigues


def spins_about_z(max_deg, step=10):
    """World rotations about z, least twist first: an object stood upright may face any way round."""
    degrees = [0] + sorted((d for d in range(-180, 181, step) if d and abs(d) <= max_deg), key=abs)
    return [np.array([[np.cos(t), -np.sin(t), 0.0], [np.sin(t), np.cos(t), 0.0], [0.0, 0.0, 1.0]]) for t in np.radians(degrees)]


def turn_about(gripper_pose, rotation, displacement, pivot):
    """Gripper pose after turning to `rotation` about `pivot` (a world point that stays fixed in the hand)
    and translating by `displacement`: the pivot ends at pivot + displacement."""
    gripper_pose = np.asarray(gripper_pose, dtype=float)
    in_hand = gripper_pose[:3, :3].T @ (np.asarray(pivot, dtype=float) - gripper_pose[:3, 3])
    target = np.eye(4)
    target[:3, :3] = rotation
    target[:3, 3] = np.asarray(pivot, dtype=float) + np.asarray(displacement, dtype=float) - rotation @ in_hand
    return target


def via_pose(start, goal, jitter):
    """A random waypoint on the way from start to goal (world 4x4 poses), for trajectory diversity: xy = their midpoint
    +- jitter[0] / jitter[1], height = above both ends by 0..jitter[2] (never down into the table), goal orientation."""
    start, via = np.asarray(start, dtype=float), np.array(goal, dtype=float)
    mid = (start[:3, 3] + via[:3, 3]) / 2
    jx, jy, jz = jitter
    via[:3, 3] = [mid[0] + np.random.uniform(-jx, jx), mid[1] + np.random.uniform(-jy, jy), max(start[2, 3], via[2, 3]) + np.random.uniform(0, jz)]
    return via


def up_alignment(start_pose, pose):
    """cos of the tilt since start_pose: 1 = the axis that pointed up at the start still does.
    Assets are not all z-up (fruit and blocks are y-up), so compare with the start, not with the object z axis."""
    up_in_object = np.asarray(start_pose, dtype=float)[:3, :3].T @ [0, 0, 1]
    return float((np.asarray(pose, dtype=float)[:3, :3] @ up_in_object)[2])


class TransitionStage(Stage):
    """extra_params:
    arm, carry_object_id, lift_height (vertical raise, metres) or
    target_position (resolved from target_workspace_id by the agent; "origin" = where the carried object was grasped) plus
    target_world_offset, motion_type, gripper_action, pos_threshold. target_object_id (+ target_object_offset, in that
    object's frame) aims at a point of another object instead: a slot, the mouth of a pen cup.
    level_object "<axis>" also stands the carried object upright (level_either_end: on whichever end is nearer;
    max_spin_deg: offer it turned about the vertical too).

    Without carry_object_id the hand moves empty by target_world_offset / lift_height
    (release in place, retreat after a handover) and only the gripper is checked.

    gripper_approach + gripper_close (world vectors: gripper z and x) also turn the hand to that
    orientation, e.g. to present a held object sideways for a handover. The object keeps its place
    in the hand, so the carried object's origin still ends at the target. min_upright (default 0.8)
    is the cosine of the tilt the object may pick up; use -1 when its orientation does not matter.
    """

    def select_pose(self, objects, robot):
        params = self.extra_params
        if self.target == "reset":
            # Alone, a reset is executed by the agent (collision-aware plan, skipped between picks of one arm). Here it is
            # one hand of a bimanual pair: home this arm while the other one moves.
            self.displacement = np.zeros(3)
            return [{"target_pose": np.array(robot.reset_pose[params.get("arm", "right")], dtype=float)}]
        offset = params.get("target_world_offset", [0, 0, params.get("lift_height", 0.0)])
        if "carry_object_id" in params:
            carried = objects[params["carry_object_id"]]
            object_pose = carried.obj_pose
            target_position = params.get("target_position")
            if "target_object_id" in params:  # a point of another object: its origin + target_object_offset in its own frame
                anchor = np.asarray(objects[params["target_object_id"]].obj_pose, dtype=float)
                target_position = anchor[:3, 3] + anchor[:3, :3] @ np.asarray(params.get("target_object_offset", [0, 0, 0]), dtype=float)
            if isinstance(target_position, str):  # "origin": back where it was grasped, in whatever orientation it has now
                grasp = self.previous_stage
                while grasp is not None and getattr(grasp, "object_rest_pose", {}).get("id") != params["carry_object_id"]:
                    grasp = grasp.previous_stage
                if grasp is None:
                    logger.warning(f"target_position origin: no grasp of {params['carry_object_id']} before this stage")
                    return []
                target_position = resting_position(grasp.object_rest_pose["pose"], carried.obj_length, object_pose[:3, :3])
            self.displacement = carry_displacement(object_pose, target_position, offset)
            self.object_target_position = object_pose[:3, 3] + self.displacement
            self.object_start_pose = np.array(object_pose, dtype=float)
        else:
            self.displacement = np.asarray(offset, dtype=float)
        current = np.array(robot.get_ee_pose(id=params.get("arm", "right")), dtype=float)
        self.start_pose = current  # where a via point (via_jitter) is drawn from
        if "level_object" in params:
            # Turn the carried object upright (its `level_object` axis, e.g. "y", to world z) by the smallest rotation and
            # apply that SAME rotation to this hand about the object's origin. Both hands of a pair get the identical rigid
            # transform, so their pose relative to each other, and to the object, does not change: nothing is squeezed.
            rotation = levelling_rotation(objects[params["carry_object_id"]].obj_pose, params["level_object"], params.get("level_either_end", False))
            pivot = self._pivot(objects, current)
            # max_spin_deg > 0: also offer the upright object turned about the vertical (it does not matter which way a
            # stick faces), so IK has a choice; reachable ones only, the closest in joint space first.
            poses = np.array([turn_about(current, spin @ rotation @ current[:3, :3], self.displacement, pivot)
                              for spin in spins_about_z(float(params.get("max_spin_deg", 0)))])
            if len(poses) > 1:
                from client.planner.func.common import sorted_by_joint_pos_dist

                arm = params.get("arm", "right")
                ok, info = robot.solve_ik(poses, ee_type="gripper", type="AvoidObs", arm=arm, output_link_pose=True)
                poses = poses[ok]
                if len(poses):
                    poses = poses[sorted_by_joint_pos_dist(robot, arm, info["joint_positions"][ok], info["joint_names"][ok], info["jacobian_score"][ok])]
            return [{"target_pose": pose} for pose in poses]
        if "gripper_approach" not in params:
            target_pose = current.copy()
            target_pose[:3, 3] += self.displacement
            return [{"target_pose": target_pose}]
        candidates = [turn_about(current, rotation, self.displacement, pivot=self._pivot(objects, current))
                      for rotation in gripper_rotations(params["gripper_approach"], params["gripper_close"])]
        # the smaller wrist turn first; the 180 deg flipped grip is the fallback
        candidates.sort(key=lambda pose: pose_difference(current, pose)[1])
        return [{"target_pose": pose} for pose in candidates]

    def _pivot(self, objects, current):
        carry = self.extra_params.get("carry_object_id")
        return np.asarray(objects[carry].obj_pose, dtype=float)[:3, 3] if carry else current[:3, 3]

    def generate_action_sequence(self, target_pose):
        sequence = ActionSequence()
        jitter = self.extra_params.get("via_jitter")  # [x, y, z] m: pass a random via point on the way (not for resets)
        if jitter and self.target == "world":
            via = via_pose(self.start_pose, target_pose, jitter)
            sequence.add_action(Action(via, None, np.eye(4), self.extra_params.get("motion_type", "Simple")))
        sequence.add_action(
            Action(
                target_pose,
                self.extra_params.get("gripper_action"),
                np.eye(4),
                self.extra_params.get("motion_type", "Simple"),
            )
        )
        return sequence

    def check_completion(self, objects, robot=None):
        """Gripper reached its target and the carried object followed it upright."""
        pos_threshold = self.extra_params.get("pos_threshold", 0.03)
        target_pose = self.active_action_sequence.get_current_action().grasp_pose
        pos_diff, angle_diff = pose_difference(objects["gripper"].obj_pose, target_pose)
        sequence = self.active_action_sequence
        if sequence.get_step_index() < len(sequence) - 1:  # a via point: the hand only has to pass near it
            return bool(pos_diff < 0.1)
        if "carry_object_id" not in self.extra_params:
            return bool(pos_diff < pos_threshold and angle_diff < 30)
        object_pose = objects[self.extra_params["carry_object_id"]].obj_pose
        object_diff = np.linalg.norm(object_pose[:3, 3] - self.object_target_position)
        upright = up_alignment(self.object_start_pose, object_pose)
        # min_upright: cos of the allowed tilt; a round object may roll in the fingers without being dropped
        logger.info("lift check: gripper_pos_diff=%.4f angle_diff=%.2f object=%s object_target=%s", pos_diff, angle_diff, np.round(object_pose[:3, 3], 3).tolist(), np.round(self.object_target_position, 3).tolist())
        ok = pos_diff < pos_threshold and angle_diff < 30 and object_diff < pos_threshold and upright > self.extra_params.get("min_upright", 0.8)
        if not ok:
            logger.warning(
                "lift completion failed: gripper_pos_diff=%.3f angle_diff=%.1f object_diff=%.3f upright=%.2f object=%s object_target=%s",
                pos_diff,
                angle_diff,
                object_diff,
                upright,
                np.round(object_pose[:3, 3], 3).tolist(),
                np.round(self.object_target_position, 3).tolist(),
            )
        return ok


class HoldOffsetTransition(Stage):
    """target "hold_offset": the pick-up of a pick. A constrained curobo move by pick_up_distance along
    pick_up_direction (robot base frame) from wherever the hand is; the object in carry_object_id must follow."""

    def select_pose(self, objects, robot):
        return [{"target_pose": np.array(robot.get_ee_pose(id=self.extra_params.get("arm", "right")), dtype=float)}]

    def generate_action_sequence(self, target_pose):
        distance = self.extra_params.get("pick_up_distance", 0.12)
        axis = "xyz".index(self.extra_params.get("pick_up_direction", "z"))
        goal_offset = [0, 0, 0, 1, 0, 0, 0]
        goal_offset[axis] = distance
        path_constraint = [0.1] * 6  # orientation x3, position x3: only the travel axis is free
        path_constraint[3 + axis] = 0.0
        return ActionSequence([Action(target_pose, "close", np.eye(4), "AvoidObs", extra_params={
            "goal_offset": goal_offset,
            "path_constraint": path_constraint,
            "offset_and_constraint_in_goal_frame": False,
            "from_current_pose": True,  # the planner starts from the live ee pose; target_pose only feeds the check
        })])

    def check_completion(self, objects, robot=None):
        if self.error_type:  # injected failures are recorded, not judged
            return True
        goal = [self.active_obj_id, self.passive_obj_id] + self.active_action_sequence.get_current_action().unpack_as_list()
        return simple_check_completion(goal, objects, last_statement=self.last_statement, robot=robot,
                                       held_object_id=self.extra_params.get("carry_object_id"))


def expand_dual_stage(stage):
    """Turn a top-level stage with ``extra_params.arm == "dual"`` into a bimanual pair of itself."""
    if stage["action"] == "bimanual" or stage.get("extra_params", {}).get("arm") != "dual":
        return stage
    pair = {"action": "bimanual", "extra_params": {"arm": "dual"}}
    if "synchronized_gripper" in stage["extra_params"]:  # a pair-level switch: both grippers in one command
        pair["extra_params"]["synchronized_gripper"] = stage["extra_params"]["synchronized_gripper"]
    if "action_description" in stage:
        pair["action_description"] = stage["action_description"]
    for arm in ("left", "right"):
        sub = copy.deepcopy(stage)
        sub["extra_params"]["arm"] = arm
        sub.setdefault("active", {"object_id": "gripper", "primitive": None})
        sub.setdefault("passive", {"object_id": "gripper", "primitive": None})
        pair[arm] = sub
    return pair


if __name__ == "__main__":
    pot = np.eye(4)
    pot[:3, 3] = [1.0, 2.0, 0.5]
    assert np.allclose(carry_displacement(pot, world_offset=[0, 0, 0.12]), [0, 0, 0.12])
    assert np.allclose(carry_displacement(pot, target_position=[0.0, 3.0, 0.5], world_offset=[0, 0, 0.01]), [-1, 1, 0.01])
    y_up = np.eye(4)
    y_up[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]  # object y axis -> world z, like the fruit assets
    assert np.isclose(up_alignment(y_up, y_up), 1) and np.isclose(up_alignment(np.eye(4), y_up), 0)
    hand = np.eye(4)
    hand[:3, 3] = [0.0, 0.3, 0.9]  # top-down hand would have z = -world z; identity is enough to test the algebra
    apple = np.array([0.0, 0.3, 0.85])
    sideways = gripper_rotations(approach=[0, -1, 0], close=[1, 0, 0])[0]
    turned = turn_about(hand, sideways, displacement=[0, -0.3, 0.25], pivot=apple)
    assert np.allclose(turned[:3, 2], [0, -1, 0]) and np.allclose(turned[:3, 0], [1, 0, 0])
    assert np.allclose(turned[:3, 3] + turned[:3, :3] @ (apple - hand[:3, 3]), [0, 0, 1.10])  # the apple lands on the target
    leaning = np.eye(4)
    leaning[:3, :3] = y_up[:3, :3] @ np.array([[1, 0, 0], [0, np.cos(0.4), -np.sin(0.4)], [0, np.sin(0.4), np.cos(0.4)]])  # a y-up pot leaning 0.4 rad
    level = levelling_rotation(leaning, "y")
    assert np.allclose((level @ leaning[:3, :3])[:, 1], [0, 0, 1]) and np.isclose(np.arccos((np.trace(level) - 1) / 2), 0.4)
    left, right = np.eye(4), np.eye(4)
    left[:3, 3], right[:3, 3] = [0.1, 0, 0], [-0.1, 0, 0]
    moved = [turn_about(hand, level @ hand[:3, :3], [0, 0, 0.12], pivot=[0, 0, 0]) for hand in (left, right)]
    assert np.allclose(np.linalg.inv(moved[0]) @ moved[1], np.linalg.inv(left) @ right)  # the hands stay rigid to each other
    lying = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])  # a 4 x 20 x 4 cm case (long axis y) on its side: y horizontal
    down = np.eye(4)
    down[:3, :3] = [[1, 0, 0], [0, -0.8, -0.6], [0, -0.6, 0.8]]  # y axis pointing down-ish
    assert np.allclose((levelling_rotation(down, "y", either_end=True) @ down[:3, :3])[:, 1], [0, 0, -1])  # stands on the near end
    assert np.allclose((levelling_rotation(down, "y") @ down[:3, :3])[:, 1], [0, 0, 1])
    assert len(spins_about_z(0)) == 1 and len(spins_about_z(20)) == 5 and np.allclose(spins_about_z(20)[0], np.eye(3))
    from types import SimpleNamespace as NS

    cup, stick = np.eye(4), np.eye(4)
    cup[:3, 3], stick[:3, 3] = [0.6, 0.5, 0.8], [0.4, 0.4, 0.76]
    stage = TransitionStage.__new__(TransitionStage)
    stage.target, stage.extra_params = "world", {"carry_object_id": "stick", "target_object_id": "cup",
                                                 "target_object_offset": [0, 0.1, 0], "target_world_offset": [0, 0, 0.05]}
    hand_pose = stick.copy()
    poses = stage.select_pose({"stick": NS(obj_pose=stick), "cup": NS(obj_pose=cup)}, NS(get_ee_pose=lambda id: hand_pose))
    assert len(poses) == 1 and np.allclose(poses[0]["target_pose"][:3, 3], [0.6, 0.6, 0.85])  # the stick ends on the cup's point
    rest = np.eye(4)
    rest[:3, :3] = lying @ np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    rest[:3, 3] = [0.5, -0.4, 0.82]
    stood = resting_position(rest, [0.04, 0.20, 0.04], np.eye(3)[[0, 2, 1]] * [1, -1, 1])  # y up
    assert np.allclose(stood, [0.5, -0.4, 0.82 + (0.20 - 0.04) / 2])
    lift = {"action": "lift", "extra_params": {"arm": "dual", "carry_object_id": "pot", "lift_height": 0.12}}
    pair = expand_dual_stage(lift)
    assert pair["action"] == "bimanual"
    assert pair["left"]["extra_params"]["arm"] == "left" and pair["right"]["extra_params"]["arm"] == "right"
    assert pair["left"]["action"] == "lift" and pair["right"]["extra_params"]["lift_height"] == 0.12
    assert expand_dual_stage(pair) is pair
    start, goal = np.eye(4), np.eye(4)
    start[:3, 3], goal[:3, 3], goal[:3, :3] = [0.4, -0.2, 0.9], [0.6, 0.2, 1.0], lying
    for _ in range(50):
        via = via_pose(start, goal, [0.05, 0.05, 0.03])
        assert abs(via[0, 3] - 0.5) <= 0.05 and abs(via[1, 3]) <= 0.05 and 1.0 <= via[2, 3] <= 1.03 and np.allclose(via[:3, :3], lying)
    stage = TransitionStage.__new__(TransitionStage)
    stage.target, stage.extra_params, stage.start_pose = "world", {"via_jitter": [0.05, 0.05, 0.03]}, start
    stage.active_action_sequence = stage.generate_action_sequence(goal)
    assert len(stage.active_action_sequence) == 2
    via = next(stage.active_action_sequence).grasp_pose
    assert stage.check_completion({"gripper": NS(obj_pose=via)}) and not stage.check_completion({"gripper": NS(obj_pose=start)})
    stage.target = "reset"
    assert len(stage.generate_action_sequence(goal)) == 1
    print("ok")
