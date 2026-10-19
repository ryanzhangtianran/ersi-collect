# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

import numpy as np

from client.planner.action.zero_dim.transition import gripper_rotations, levelling_rotation, via_pose
from client.planner.action.stage import (
    Action,
    ActionSequence,
    Stage,
    simple_check_completion,
)
from client.planner.common import generate_random_pose, overweite_grasp_data
from client.planner.func.common import (
    filter_grasp_pose_by_gripper_up_direction,
    filter_grasp_poses_with_humanlike_posture,
    random_downsample,
    sorted_by_joint_pos_dist,
)
from common.base_utils.logger import logger
from common.base_utils.transform_utils import (
    calculate_rotation_matrix2,
    pose_from_position_quaternion,
    rotate_along_axis,
)


def world_grasp_candidates(object_pose, approach, close, yaws_deg):
    """Gripper poses in the object frame for a grasp whose approach (gripper z) and closing axis (gripper x) are world vectors."""
    approach = np.asarray(approach, dtype=float) / np.linalg.norm(approach)
    close = np.asarray(close, dtype=float)
    close = close - approach * (close @ approach)  # make it exactly perpendicular to the approach
    close /= np.linalg.norm(close)
    to_object = np.asarray(object_pose, dtype=float)[:3, :3].T
    candidates = []
    for sign in (1.0, -1.0):
        for yaw in np.deg2rad(yaws_deg):
            x = sign * (close * np.cos(yaw) + np.cross(approach, close) * np.sin(yaw))  # spin about the approach axis
            pose = np.eye(4)
            pose[:3, :3] = to_object @ np.column_stack([x, np.cross(approach, x), approach])
            candidates.append(pose)
    return np.stack(candidates)


def carry_hand_poses(pick_stage, object_pose, grasp_poses, scene=None):
    """[(n, hand poses (N,4,4))]: where each grasp candidate puts the hand at the end of the n-th lift stage of the same arm
    after the pick, so a pick can drop candidates whose carry is unreachable before the robot moves.
    Follows TransitionStage exactly: a translation (lift_height / target_world_offset, or target_position for the carried object's
    origin), turned about the object origin by level_object or to gripper_approach/gripper_close (the flip nearer the current
    hand). Stages of the other arm are skipped (a handover: they do not move this hand); the walk stops at the first stage of
    this arm that is not a lift, or that carries another object."""
    arm = pick_stage.extra_params.get("arm", "right")
    hands = np.array(grasp_poses, dtype=float)
    objects = np.repeat(np.asarray(object_pose, dtype=float)[np.newaxis], len(hands), axis=0)  # the object, as each hand would carry it
    out, n, stage = [], 0, pick_stage.next_stage
    while stage is not None:
        stage = getattr(stage, f"{arm}_stage", stage)  # in a bimanual pair, this arm's half
        params = stage.extra_params
        if params.get("arm", "right") != arm:
            stage = stage.next_stage
            continue
        if getattr(stage, "target", None) == "hold_offset":  # the pick-up of this grasp: a short vertical raise, not a carry target
            stage = stage.next_stage
            continue
        if getattr(stage, "target", None) != "world" or params.get("carry_object_id") not in (None, pick_stage.passive_obj_id):
            break
        carried = params.get("carry_object_id") is not None
        offset = np.asarray(params.get("target_world_offset", [0, 0, params.get("lift_height", 0.0)]), dtype=float)
        for i in range(len(hands)):
            origin = objects[i][:3, 3]
            target = params.get("target_position")
            if "target_object_id" in params and scene is not None:
                anchor = np.asarray(scene[params["target_object_id"]].obj_pose, dtype=float)
                target = anchor[:3, 3] + anchor[:3, :3] @ np.asarray(params.get("target_object_offset", [0, 0, 0]), dtype=float)
            # "origin" = back to where it was picked from: no horizontal move (the few cm of standing height are ignored)
            move = offset + (np.asarray(target, dtype=float) - origin if carried and target is not None and not isinstance(target, str) else 0.0)
            turn, pivot = np.eye(3), (origin if carried else hands[i][:3, 3])
            if carried and "level_object" in params:
                turn = levelling_rotation(objects[i], params["level_object"], params.get("level_either_end", False))
            elif "gripper_approach" in params:
                options = [target @ hands[i][:3, :3].T for target in gripper_rotations(params["gripper_approach"], params["gripper_close"])]
                turn = max(options, key=np.trace)  # the flip that turns the hand least (largest trace = smallest angle)
            step = np.eye(4)
            step[:3, :3], step[:3, 3] = turn, pivot + move - turn @ pivot
            hands[i] = step @ hands[i]
            if carried:
                objects[i] = step @ objects[i]
        n += 1
        out.append((n, hands.copy()))
        stage = stage.next_stage
    return out


def close_across(object_pose, approach, cfg):
    """Finger closing axis of a fixed_world_grasp: cfg["close"], or perpendicular to the object axis named by cfg["across"]."""
    if "across" not in cfg:
        return cfg["close"]
    close = np.cross(approach, np.asarray(object_pose, dtype=float)[:3, "xyz".index(cfg["across"])])
    return close if np.linalg.norm(close) > 0.2 else cfg["close"]


def sample_approaches(cfg, rng=np.random):
    """Approach directions of a fixed_world_grasp: the given one, or `samples` random ones in the elevation/azimuth ranges."""
    if "elevation_deg" not in cfg:
        return [cfg["approach"]]
    heading = np.array([cfg["approach"][0], cfg["approach"][1], 0.0], dtype=float)
    heading /= np.linalg.norm(heading)
    side = np.cross([0.0, 0.0, 1.0], heading)
    out = []
    for _ in range(int(cfg.get("samples", 4))):
        elevation = np.deg2rad(rng.uniform(*cfg["elevation_deg"]))
        azimuth = np.deg2rad(rng.uniform(*cfg.get("azimuth_deg", [0.0, 0.0])))
        out.append(np.cos(elevation) * (np.cos(azimuth) * heading + np.sin(azimuth) * side) - np.sin(elevation) * np.array([0.0, 0.0, 1.0]))
    return out


class GraspStage(Stage):
    def __init__(self, stage_config, objects):
        super().__init__(stage_config, objects)
        self.use_pre_grasp = True

    def select_pose(self, objects, robot):
        gripper2obj = []
        arm = self.extra_params.get("arm", "right")
        grasp_offset = self.extra_params.get("grasp_offset", 0.03)
        pre_grasp_offset = self.extra_params.get("pre_grasp_offset", 0.0)
        grasp_lower_percentile = self.extra_params.get("grasp_lower_percentile", 0)
        grasp_upper_percentile = self.extra_params.get("grasp_upper_percentile", 100)
        disable_upside_down = self.extra_params.get("disable_upside_down", False)
        flip_grasp = self.extra_params.get("flip_grasp", False)
        error_data = self.extra_params.get("error_data", {})
        error_type = error_data.get("type", None)
        set_grasp_pose_xy = self.extra_params.get("set_grasp_pose_xy", False)
        set_grasp_vertical = self.extra_params.get("set_grasp_vertical", False)

        # where the object rests before it is touched: a later transition may put it back (target_position "origin")
        self.object_rest_pose = {"id": self.passive_obj_id, "pose": np.array(objects[self.passive_obj_id].obj_pose, dtype=float)}

        """Select grasp points from Stage class"""
        """Filter out grasp poses without IK solutions"""
        grasp_poses_canonical = self.passive_element["grasp_pose"].copy()
        grasp_widths = self.passive_element["width"]
        if set_grasp_pose_xy:
            grasp_poses_canonical[:, 0, 3] = 0
            grasp_poses_canonical[:, 2, 3] = 0

        z_values = grasp_poses_canonical[:, 1, 3]
        z_lower_threshold = np.percentile(z_values, grasp_lower_percentile)
        z_upper_threshold = np.percentile(z_values, grasp_upper_percentile)
        grasp_poses_canonical_bak = grasp_poses_canonical.copy()
        # filter grasp pose with z_min and z_max value
        mask = (z_values <= z_upper_threshold) & (z_values >= z_lower_threshold)
        grasp_poses_canonical = grasp_poses_canonical_bak[mask]
        grasp_widths = grasp_widths[mask]
        grasp_poses_canonical[:, :3, :3] = (
            grasp_poses_canonical[:, :3, :3] @ robot.robot_gripper_2_grasp_gripper[np.newaxis, ...]
        )
        if set_grasp_vertical:
            # Directly align local y-axis positive direction with coordinate system y-axis (right hand), left hand aligns with negative direction
            for i in range(grasp_poses_canonical.shape[0]):
                local_y = grasp_poses_canonical[i][:3, 1]
                target_y = np.array([0, 1, 0]) if arm == "right" else np.array([0, -1, 0])
                calc_rotation_matrix = calculate_rotation_matrix2(local_y, target_y)
                grasp_poses_canonical[i][:3, :3] = calc_rotation_matrix @ grasp_poses_canonical[i][:3, :3]

        if flip_grasp:
            grasp_poses_canonical_flip = []
            for _i in range(grasp_poses_canonical.shape[0]):
                grasp_poses_canonical_flip.append(
                    rotate_along_axis(grasp_poses_canonical[_i], 180, "z", use_local=True)
                )
            grasp_poses_canonical_flip = np.stack(grasp_poses_canonical_flip)
            grasp_poses_canonical = np.concatenate([grasp_poses_canonical, grasp_poses_canonical_flip], axis=0)
            grasp_widths = np.concatenate([grasp_widths, grasp_widths], axis=0)

        random_grasp_pose = self.extra_params.get("random_grasp_pose", False)
        if random_grasp_pose:
            grasp_poses_canonical, grasp_widths = overweite_grasp_data(100)

        if self.extra_params.get("fixed_top_grasp", False):
            # Deterministic top-down grasp at the object centre instead of the sampled candidates
            # (object frame: y up). Gripper z = approach, straight down; gripper x = finger closing
            # axis along a horizontal object axis; 4 yaws = two axes x their 180 deg flips. Depth
            # via grasp_offset as usual (0 = ee origin at the object centre).
            # Face-normal yaws only (2 for a cube) leave the wrist no room: G2 joint7 spans +-90 deg, so a
            # block rotated the wrong way is unreachable for both arms. Fan the closing axis out around the
            # approach axis as well; the fingers then bite slightly off-normal, which still holds a 3-5 cm
            # block, and the extra yaws give IK something to work with.
            yaw_fan = np.deg2rad(self.extra_params.get("fixed_top_grasp_yaws", [0, 15, -15]))
            fixed = []
            for close_axis in (np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])):
                for sgn in (1.0, -1.0):
                    T = np.eye(4)
                    T[:3, 0] = sgn * close_axis
                    T[:3, 2] = np.array([0.0, -1.0, 0.0])
                    T[:3, 1] = np.cross(T[:3, 2], T[:3, 0])
                    for yaw in yaw_fan:
                        c, s_ = np.cos(yaw), np.sin(yaw)
                        spin = np.eye(4)  # rotate about the gripper approach axis (its own z)
                        spin[:2, :2] = [[c, -s_], [s_, c]]
                        fixed.append(T @ spin)
            grasp_poses_canonical = np.stack(fixed)
            grasp_widths = np.full(len(fixed), float(np.max(grasp_widths)))
        world_grasp = self.extra_params.get("fixed_world_grasp")
        if world_grasp:
            # Deterministic grasp through the object centre, given in WORLD axes: for round objects (fruit) whose own
            # frame says nothing, and for handovers where both hands' contact points are decided up front.
            # {"approach": [x,y,z], "close": [x,y,z], "yaws_deg": [0, 10, -10]}: gripper z = approach, gripper x =
            # finger closing axis; candidates = the closing axis and its 180 deg flip, each fanned about the approach.
            # With "elevation_deg": [lo, hi] (and optionally "azimuth_deg": [lo, hi], "samples": n) the approach is drawn
            # at random instead: `approach` gives the horizontal heading, the hand comes down at the drawn elevation and is
            # swung about the vertical by the drawn azimuth. `close` is re-squared to each drawn approach.
            # "across": "y" closes the fingers across the object's own y axis (a stick lying at any yaw is pinched
            # square on), wherever that axis points; `close` remains the fallback when the axis is along the approach.
            grasp_poses_canonical = np.concatenate([
                world_grasp_candidates(objects[self.passive_obj_id].obj_pose, approach,
                                       close_across(objects[self.passive_obj_id].obj_pose, approach, world_grasp), world_grasp.get("yaws_deg", [0, 10, -10]))
                for approach in sample_approaches(world_grasp)
            ])
            grasp_widths = np.full(len(grasp_poses_canonical), float(np.max(grasp_widths)))
        # With fixed_top_grasp there are only 4 candidates, and each solver rejects a different set:
        # Lula ("Simple") is a single-seed local solver that misses reachable top-down poses once the arm
        # sits in an awkward configuration, while curobo ("AvoidObs") is collision-aware and rejects a
        # grasp at the object centre because the target itself is still an obstacle. Take the union and
        # let the downstream curobo IK / planner / in-hand checker decide.
        permissive_ik = self.extra_params.get("fixed_top_grasp", False) or bool(world_grasp)

        grasp_poses = objects[self.passive_obj_id].obj_pose[np.newaxis, ...] @ grasp_poses_canonical

        filter_grasp_pose_info = self.extra_params.get("filter_grasp_pose", {})
        if filter_grasp_pose_info:
            grasp_poses, grasp_widths, _ = filter_grasp_pose_by_gripper_up_direction(
                filter_grasp_pose_info, grasp_poses, grasp_widths
            )

        if self.extra_params.get("humanlike_filter", False):
            grasp_poses, grasp_widths, _ = filter_grasp_poses_with_humanlike_posture(grasp_poses, grasp_widths)

        if disable_upside_down:
            if "omnipicker" in robot.robot_cfg:
                if arm == "left":
                    upright_mask = grasp_poses[:, 2, 1] < 0.0
                else:
                    upright_mask = grasp_poses[:, 2, 1] > 0.0
            else:
                upright_mask = grasp_poses[:, 2, 0] > 0.0
            grasp_poses = grasp_poses[upright_mask]
            grasp_widths = grasp_widths[upright_mask]
            logger.info(
                f"{self.action_type}, {self.passive_obj_id}, Filtered upside-down grasp_poses: {grasp_poses.shape[0]}"
            )

        if len(grasp_poses) == 0:
            logger.warning(f"{self.action_type}: No grasp_gripper_pose can pass gripper y direction filter")
            return []

        # Downsample if there are too many grasp points
        grasp_poses, random_indices = random_downsample(grasp_poses, 2000, False)  # was 300: keep (nearly) all candidates so IK/lookahead filters see the whole set
        if random_indices is not None:
            grasp_widths = grasp_widths[random_indices]

        # grasp offset
        grasp_rotate = grasp_poses.copy()
        grasp_rotate[:, :3, 3] = np.array([0, 0, 0])
        transport_vector = grasp_rotate @ np.array([0, 0, 1, 0])[:, np.newaxis]
        transport_vector = transport_vector[:, :3, 0]
        transport_vector = transport_vector / np.linalg.norm(transport_vector, axis=1, keepdims=True)

        grasp_poses[:, :3, 3] = grasp_poses[:, :3, 3] + transport_vector * grasp_offset
        object_pose_inverse = np.linalg.inv(objects[self.passive_obj_id].obj_pose)
        grasp_poses_canonical = object_pose_inverse[np.newaxis, ...] @ grasp_poses

        # filter with IK-checking
        ik_success, _ = robot.solve_ik(grasp_poses, ee_type="gripper", arm=arm, type="Simple")
        if permissive_ik and not ik_success.all():
            curobo_success, _ = robot.solve_ik(grasp_poses, ee_type="gripper", arm=arm, type="AvoidObs")
            ik_success = ik_success | curobo_success
        grasp_poses_canonical, grasp_poses = (
            grasp_poses_canonical[ik_success],
            grasp_poses[ik_success],
        )
        grasp_widths = grasp_widths[ik_success]

        # Carry lookahead: the lift stages that follow move the hand by known world displacements, so an unreachable
        # carry target (e.g. a stove too far away) is found here, before the robot moves, not after the object is up.
        keep = np.ones(len(grasp_poses), dtype=bool)
        # The check screens with a small IK budget from the pre-pick configuration, so a carry the stage
        # itself can plan (place reaches these poses routinely) can still be screened out. skip_carry_check
        # trades the early exit for letting the carry stage decide on its own.
        carry_stages = [] if self.extra_params.get("skip_carry_check", False) else carry_hand_poses(
            self, objects[self.passive_obj_id].obj_pose, grasp_poses, scene=objects)
        for stage_number, ahead in carry_stages:
            reachable, _ = robot.solve_ik(ahead[keep], ee_type="gripper", arm=arm, type="Simple")
            reachable = np.asarray(reachable, dtype=bool)
            if permissive_ik and not reachable.all():
                # Same union as the grasp filter above: Lula is a single-seed local solver and the hand is
                # still at the pre-pick pose here, so it misses carries curobo can actually plan. Only veto
                # a layout when both solvers refuse.
                curobo_solved, _ = robot.solve_ik(ahead[keep], ee_type="gripper", arm=arm, type="AvoidObs")
                reachable = reachable | np.asarray(curobo_solved, dtype=bool)
            keep[np.flatnonzero(keep)[~reachable]] = False
            if not keep.any():
                # `ahead` is empty when every candidate was already filtered out upstream; formatting
                # a sample target from it then kills the whole run instead of rejecting this layout.
                sample = np.round(ahead[0, :3, 3], 3).tolist() if len(ahead) else None
                logger.warning(
                    f"CARRY_UNREACHABLE arm={arm} object={self.passive_obj_id} lift_stage=+{stage_number} "
                    f"hand_target={sample}: no grasp candidate can reach it"
                )
                return []
        grasp_poses_canonical, grasp_poses, grasp_widths = grasp_poses_canonical[keep], grasp_poses[keep], grasp_widths[keep]

        if error_type == "RandomPerturbations":
            error_params = error_data.get("params", {})
            pos_std = error_params.get("pos_std", [0.2, 0.2, 0.2])
            rot_std = error_params.get("rot_std", [0.5, 0.5, 0.5])
            gripper2obj = []
            ik_joint_positions = []
            ik_joint_names = []
            ik_jacobian_score = []
            max_try = 10
            while len(gripper2obj) < 5 and max_try > 0:
                random_poses = generate_random_pose(
                    grasp_poses[max_try % grasp_poses.shape[0]],
                    position_std=pos_std,
                    rotation_std=rot_std,
                    num=40,
                )
                if disable_upside_down:
                    if "omnipicker" in robot.robot_cfg:
                        upright_mask = random_poses[:, 2, 1] > 0.0
                    else:
                        upright_mask = random_poses[:, 2, 0] > 0.0
                    random_poses = random_poses[upright_mask]
                ik_success, ik_info = robot.solve_ik(
                    random_poses,
                    ee_type="gripper",
                    arm=arm,
                    type="AvoidObs",
                    output_link_pose=False,
                )
                success_poses = random_poses[ik_success]
                if success_poses.shape[0] > 0:
                    gripper2obj += (
                        np.linalg.inv(objects[self.passive_obj_id].obj_pose)[np.newaxis, ...] @ success_poses
                    ).tolist()
                    ik_joint_positions += ik_info["joint_positions"][ik_success].tolist()
                    ik_joint_names += ik_info["joint_names"][ik_success].tolist()
                    ik_jacobian_score += ik_info["jacobian_score"][ik_success].tolist()
                max_try -= 1
            if len(gripper2obj):
                joint_names = robot.joint_names[arm]
                target_joint_positions = []
                for ik_joint_position, ik_joint_name in zip(ik_joint_positions, ik_joint_names):
                    temp_target_joint_positions = []
                    for joint_name in joint_names:
                        temp_target_joint_positions.append(ik_joint_position[list(ik_joint_name).index(joint_name)])
                    target_joint_positions.append(np.array(temp_target_joint_positions))
                target_joint_positions = np.array(target_joint_positions)
                cur_joint_states = robot.client.get_joint_positions().states
                cur_joint_positions = []
                for key in cur_joint_states:
                    if key.name in joint_names:
                        cur_joint_positions.append(key.position)
                cur_joint_positions = np.array(cur_joint_positions)
                joint_pos_dist = np.linalg.norm(target_joint_positions - cur_joint_positions[np.newaxis, :], axis=1)
                dist_mean = np.mean(joint_pos_dist)
                dist_std = np.std(joint_pos_dist)
                joint_pos_dist = (joint_pos_dist - dist_mean) / dist_std
                ik_jacobian_score = np.array(ik_jacobian_score)
                ik_jacobian_score = (ik_jacobian_score - np.min(ik_jacobian_score)) / (
                    np.max(ik_jacobian_score) - np.min(ik_jacobian_score)
                )
                cost = joint_pos_dist - ik_jacobian_score
                idx_sorted = np.argsort(cost)
                gripper2obj = np.array(gripper2obj)[idx_sorted]
            return gripper2obj

        if len(grasp_poses) == 0:
            logger.warning(f"{self.action_type}: No grasp_gripper_pose can pass Isaac IK")
            return []

        # Whether what follows is reachable with a candidate is the carry lookahead's job (above): every leg of the
        # transition that follows is an ordinary target now, a placement included.
        best_grasp_poses = grasp_poses
        # downsample grasp pose
        best_grasp_poses, _ = random_downsample(best_grasp_poses, 400, False)  # was 100
        if best_grasp_poses.shape[0] >= 1:
            if error_type == "KeepClose" or self.extra_params.get("remove_from_obstacle", False):
                robot.client.remove_objs_from_obstacle([objects[self.passive_obj_id].prim_path])
            if self.extra_params.get("skip_curobo_grasp_filter", False):
                pre_grasp_distance = self.extra_params.get("pre_grasp_distance", 0.05)
                pre_grasp_offset = pose_from_position_quaternion(
                    [0, 0, -pre_grasp_distance], [1, 0, 0, 0]
                )
                pre_grasp_poses = best_grasp_poses @ pre_grasp_offset if self.use_pre_grasp else None
                if pre_grasp_poses is not None:
                    pre_grasp_ik, _ = robot.solve_ik(
                        pre_grasp_poses,
                        ee_type="gripper",
                        arm=arm,
                        type="Simple",
                    )
                    best_grasp_poses = best_grasp_poses[pre_grasp_ik]
                    pre_grasp_poses = pre_grasp_poses[pre_grasp_ik]
                if len(best_grasp_poses) == 0:
                    logger.warning(f"{self.action_type}: No grasp poses can pass pre-grasp Isaac IK")
                    return []
                object_pose_inverse = np.linalg.inv(objects[self.passive_obj_id].obj_pose)
                result = []
                for index, grasp_pose in enumerate(best_grasp_poses):
                    item = {
                        "grasp_pose": object_pose_inverse @ grasp_pose,
                        "pre_grasp_pose": (
                            object_pose_inverse @ pre_grasp_poses[index]
                            if pre_grasp_poses is not None
                            else None
                        ),
                    }
                    result.append(item)
                return result
            joint_names = robot.joint_names[arm]

            ik_success, ik_info = robot.solve_ik(
                best_grasp_poses,
                ee_type="gripper",
                type="AvoidObs",
                arm=arm,
                output_link_pose=True,
            )
            best_grasp_poses = best_grasp_poses[ik_success]
            ik_joint_positions = ik_info["joint_positions"][ik_success]
            ik_joint_names = ik_info["joint_names"][ik_success]
            ik_jacobian_score = ik_info["jacobian_score"][ik_success]
            ik_link_poses = ik_info["link_poses"][ik_success]
            if len(best_grasp_poses) == 0:
                logger.warning(f"{self.action_type}: No best_grasp_poses can pass curobo IK")
                return []
            pre_grasp_poses = None
            if len(best_grasp_poses) == 0:
                logger.warning(f"{self.action_type}: No best_grasp_poses can pass curobo IK")
                return []
            if self.use_pre_grasp:
                # calculate pre grasp pose
                pre_grasp_distance = self.extra_params.get("pre_grasp_distance", 0.05)
                pre_grasp_offset = [0, 0, -pre_grasp_distance, 1, 0, 0, 0]
                pre_grasp_offset_matrix = pose_from_position_quaternion(pre_grasp_offset[:3], pre_grasp_offset[3:])
                pre_grasp_poses = best_grasp_poses @ pre_grasp_offset_matrix
                ik_success_pre, ik_info_pre = robot.solve_ik(
                    pre_grasp_poses,
                    ee_type="gripper",
                    type="AvoidObs",
                    arm=arm,
                    output_link_pose=True,
                )
                best_grasp_poses = best_grasp_poses[ik_success_pre]
                pre_grasp_poses = pre_grasp_poses[ik_success_pre]
                ik_joint_positions = ik_joint_positions[ik_success_pre]
                ik_joint_names = ik_joint_names[ik_success_pre]
                ik_jacobian_score = ik_jacobian_score[ik_success_pre]
                ik_link_poses = ik_link_poses[ik_success_pre]
            if len(best_grasp_poses) == 0:
                logger.warning(f"{self.action_type}: No best_grasp_poses can pass curobo IK")
                return []
            # Every task: closest IK solution to the current joint state first (least joint motion), like the levelling
            # spins and the planner's path choice; avoids contorted wrist/elbow picks.
            idx_sorted = sorted_by_joint_pos_dist(robot, arm, ik_joint_positions, ik_joint_names, ik_jacobian_score)
            # grasp_topk_shuffle k: draw the order of the k best candidates at random, so a scene is not always grasped the
            # same way; the rest keep their rank as fallbacks. 0 (default) = the deterministic order.
            topk = int(self.extra_params.get("grasp_topk_shuffle", 0))
            if topk > 1:
                idx_sorted[:topk] = np.random.permutation(idx_sorted[:topk])
            grasp_poses_sorted = best_grasp_poses[idx_sorted]
            pre_grasp_poses_sorted = pre_grasp_poses[idx_sorted]

        else:
            logger.info("No grasp pose found")
            return []

        grasp_pose_canonical_sorted = (
            np.linalg.inv(objects[self.passive_obj_id].obj_pose)[np.newaxis, ...] @ grasp_poses_sorted
        )
        gripper2obj = grasp_pose_canonical_sorted
        if pre_grasp_poses is not None and len(pre_grasp_poses_sorted) == len(grasp_poses_sorted):
            pre_grasp_pose_canonical_sorted = (
                np.linalg.inv(objects[self.passive_obj_id].obj_pose)[np.newaxis, ...] @ pre_grasp_poses_sorted
            )
        jitter = self.extra_params.get("via_jitter")  # [x, y, z] m: approach the pre-grasp through a random via point
        start = robot.get_ee_pose(id=arm) if jitter else None
        result = []
        for i in range(len(gripper2obj)):
            tmp_result = {}
            tmp_result["grasp_pose"] = gripper2obj[i]
            tmp_result["pre_grasp_pose"] = pre_grasp_pose_canonical_sorted[i] if pre_grasp_poses is not None else None
            if jitter and pre_grasp_poses is not None:
                tmp_result["via"] = object_pose_inverse @ via_pose(start, pre_grasp_poses_sorted[i], jitter)
            result.append(tmp_result)
        return result

    def generate_action_sequence(self, grasp_pose, pre_grasp_pose=None, via=None):
        """[via ->] pre-grasp -> grasp pose + close. The pick-up that used to follow is a transition (target "hold_offset")."""
        action_sequence = ActionSequence()
        pre_grasp_vector = self.extra_params.get("pre_grasp_vector", [])

        if self.use_pre_grasp:
            if pre_grasp_pose is None:
                logger.info("No pre_grasp_pose found")
                return None
            if via is not None:
                action_sequence.add_action(Action(via, None, np.eye(4), "AvoidObs"))
            action_sequence.add_action(
                Action(
                    grasp_pose=pre_grasp_pose,
                    gripper_action=None,
                    transform_world=np.eye(4),
                    motion_type=self.extra_params.get("pre_grasp_motion_type", "AvoidObs"),
                )
            )
            action_sequence.add_action(
                Action(
                    grasp_pose=grasp_pose,
                    gripper_action="close",
                    transform_world=np.eye(4),
                    motion_type=self.extra_params.get("grasp_motion_type", "AvoidObs"),
                    extra_params={
                        "path_constraint": [0.1, 0.1, 0.1, 0.1, 0.1, 0.0],
                        "offset_and_constraint_in_goal_frame": True,
                    },
                )
            )
        else:
            if len(pre_grasp_vector) > 0:
                transform = np.eye(4)
                transform[:3, 3] = np.array(pre_grasp_vector)
                action_sequence.add_action(Action(grasp_pose, "open", transform, "AvoidObs"))
            if self.error_type == "MissGrasp":
                action_sequence.add_action(Action(grasp_pose, None, np.eye(4), "AvoidObs"))
            else:
                action_sequence.add_action(Action(grasp_pose, "close", np.eye(4), "AvoidObs"))
                if self.error_type == "RandomPerturbations":
                    action_sequence.add_action(Action(grasp_pose, "open", np.eye(4), "AvoidObs"))
        if self.error_type == "WrongTarget":
            action = action_sequence[0]
            action.gripper_action = "open"
            action_sequence = ActionSequence([action])
        return action_sequence

    def check_completion(self, objects, robot=None):
        assert self.active_action_sequence is not None, f"Active action for stage {self.action_type} is None"

        goal_datapack = [
            self.active_obj_id,
            self.passive_obj_id,
        ] + self.active_action_sequence.get_current_action().unpack_as_list()
        succ = True
        if not self.error_type:
            succ = simple_check_completion(goal_datapack, objects, last_statement=self.last_statement, robot=robot)
        sequence = self.active_action_sequence
        closes_last = sequence.get_current_action().gripper_action == "close" and sequence.get_step_index() == len(sequence.actions) - 1
        picked_up_next = getattr(getattr(self, "next_stage", None), "target", None) == "hold_offset"
        if succ and closes_last and not picked_up_next and robot is not None and hasattr(robot, "gripper_opening"):
            # No pick-up follows the close, so no follow check runs and a missed grasp would only surface in a later
            # stage, whose retries do not re-grasp. Fingers that closed on nothing sit at ~0 rad.
            arm = self.extra_params.get("arm", "right")
            opening = robot.gripper_opening(arm)
            if opening < self.extra_params.get("min_grasp_opening", 0.02):
                logger.warning(f"GRASP_EMPTY arm={arm} object={self.passive_obj_id}: finger joint {opening:.3f} rad, closed on nothing")
                succ = False
        return succ
