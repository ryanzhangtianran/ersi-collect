# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

import os
import sys

import numpy as np

from common.base_utils.logger import logger

current_directory = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_directory)


# Sort based on joint movement cost
def sorted_by_joint_pos_dist(robot, arm, ik_joint_positions, ik_joint_names, ik_jacobian_score, joint_weights=None):

    joint_names = robot.joint_names[arm]
    # ['idx61_arm_r_joint1', 'idx62_arm_r_joint2', ...]

    # If no weights provided, use default equal weights (maintain original behavior)
    if joint_weights is None:
        joint_weights = [
            2.0,  # joint1
            1.8,  # joint2
            1.6,  # joint3
            1.4,  # joint4
            1.2,  # joint5
            1.0,  # joint6
            0.8,  # joint7
        ]
    else:
        # Ensure weight vector matches number of joints
        assert len(joint_weights) == len(
            joint_names
        ), f"Weight count ({len(joint_weights)}) does not match joint count ({len(joint_names)})"

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

    # Normalize Jacobian score (0-1 range)
    ik_jacobian_score = (ik_jacobian_score - np.min(ik_jacobian_score)) / (
        np.max(ik_jacobian_score) - np.min(ik_jacobian_score) + 1e-8
    )

    # Calculate joint movement cost using weighted Euclidean distance
    diff = target_joint_positions - cur_joint_positions[np.newaxis, :]
    weighted_sq_diff = (diff**2) * joint_weights  # Apply weights
    joint_pos_dist = np.sqrt(np.sum(weighted_sq_diff, axis=1))

    cost = joint_pos_dist - ik_jacobian_score
    idx_sorted = np.argsort(cost)

    return idx_sorted


# Random downsampling
def random_downsample(transforms: np.ndarray, downsample_num: int, replace: bool = False) -> np.ndarray:
    random_indices = None
    if transforms.shape[0] > downsample_num:
        random_indices = np.random.choice(
            transforms.shape[0],
            downsample_num,
            replace=replace,
        )
        transforms = transforms[random_indices]
    return transforms, random_indices


def filter_grasp_poses_with_humanlike_posture(grasp_poses: np.ndarray, grasp_widths: np.ndarray) -> tuple:
    mask = [pose[2, 1] > 0.0 and pose[0, 2] > 0 and pose[1, 2] > 0 for pose in grasp_poses]
    mask = np.array(mask)
    filtered_grasp_poses = grasp_poses[mask]
    filtered_grasp_widths = grasp_widths[mask]

    origin_length = len(grasp_poses)
    filtered_length = len(filtered_grasp_poses)
    logger.info(f"Filtered grasp poses by humanlike posture: {filtered_length}/{origin_length}")

    return filtered_grasp_poses, filtered_grasp_widths, mask


# Convert input string or vector list to corresponding np.array format vector
def get_corresponding_vector(input_vec: str | list) -> np.array:
    input_vec_premitted = ["x", "y", "z", "+x", "+y", "+z", "-x", "-y", "-z"]
    # Process direction parameter
    if isinstance(input_vec, str):
        # Convert string direction to vector
        if input_vec == "x" or input_vec == "+x":
            input_vec_right = np.array([1.0, 0.0, 0.0])
        elif input_vec == "y" or input_vec == "+y":
            input_vec_right = np.array([0.0, 1.0, 0.0])
        elif input_vec == "z" or input_vec == "+z":
            input_vec_right = np.array([0.0, 0.0, 1.0])
        elif input_vec == "-x":
            input_vec_right = np.array([-1.0, 0.0, 0.0])
        elif input_vec == "-y":
            input_vec_right = np.array([0.0, -1.0, 0.0])
        elif input_vec == "-z":
            input_vec_right = np.array([0.0, 0.0, -1.0])
        else:
            raise ValueError(f"if type of input_vec is str, input_vec must in {input_vec_premitted}")
    elif isinstance(input_vec, list) and len(input_vec) == 3:
        # Ensure vector is numpy array and normalized
        input_vec_right = np.array(input_vec, dtype=np.float64)
        input_vec_right = input_vec_right / np.linalg.norm(input_vec_right)
    else:
        raise TypeError(f"input_vec must be str in [{input_vec_premitted}] or list length is 3")

    return input_vec_right


def filter_grasp_pose_by_gripper_up_direction(
    filter_grasp_pose_info: dict,
    grasp_poses: np.ndarray,
    grasp_widths: np.ndarray,
) -> tuple:

    gripper_up_axis = filter_grasp_pose_info.get("gripper_up_axis", "y")
    # "targrt_direction" is the historical misspelling, still accepted; the documented default threshold is pi/4
    # (the old default 0.0 rejected every grasp, since angles < 0 never holds)
    target_direction = filter_grasp_pose_info.get("target_direction", filter_grasp_pose_info.get("targrt_direction", "x"))
    threshold = filter_grasp_pose_info.get("threshold", np.pi / 4)

    """
    Filter grasp poses based on angle between grasp axis and target direction.

    Args:
        grasp_poses: numpy array, shape (N,4,4), storing N 4x4 transformation matrices
        grasp_widths: numpy array, shape (N,), storing corresponding grasp widths
        axis: Can be one of two types:
              1. String: Specify target direction axis. Options: 'x', 'y', 'z', '-x', '-y', '-z'
              2. 3D vector: List or array of length 3, specifying target direction vector
        arm: String, specify which arm to use. Determines whether to use positive or negative y-axis.
              'left': Use negative y-axis direction
              Others: Use positive y-axis direction
        threshold: Angle threshold (radians), only keep grasp poses with angle less than this value. Default is π/4 (45°)

    Returns:
        filtered_grasp_poses: numpy array, shape (M,4,4), only retained transformation matrices
        filtered_grasp_widths: numpy array, shape (M,), corresponding grasp widths
        mask: Boolean array, shape (N,), indicating which grasp poses are retained
    """
    if grasp_poses.shape[1:] != (4, 4):
        raise ValueError("grasp_poses must be of shape (N,4,4)")
    if not isinstance(grasp_widths, np.ndarray) or len(grasp_widths) != len(grasp_poses):
        raise ValueError("grasp_widths must be a numpy array with same length as grasp_poses")

    # Handle different target_direction input types
    if isinstance(target_direction, str):

        # Convert string direction to vector
        if target_direction == "x" or target_direction == "+x":
            target_direction_vec = np.array([1.0, 0.0, 0.0])
        elif target_direction == "y" or target_direction == "+y":
            target_direction_vec = np.array([0.0, 1.0, 0.0])
        elif target_direction == "z" or target_direction == "+z":
            target_direction_vec = np.array([0.0, 0.0, 1.0])
        elif target_direction == "-x":
            target_direction_vec = np.array([-1.0, 0.0, 0.0])
        elif target_direction == "-y":
            target_direction_vec = np.array([0.0, -1.0, 0.0])
        elif target_direction == "-z":
            target_direction_vec = np.array([0.0, 0.0, -1.0])
        else:
            raise ValueError(
                "Invalid target_direction string. Must be one of: 'x', 'y', 'z', '-x', '-y', '-z', '+x', '+y', '+z'"
            )
        # Handle string input (maintain original logic)
        if target_direction not in ["x", "y", "z", "-x", "-y", "-z", "+x", "+y", "+z"]:
            raise ValueError(
                "Invalid target_direction string. Must be one of: 'x', 'y', 'z', '-x', '-y', '-z', '+x', '+y', '+z'"
            )

    elif isinstance(target_direction, (list, tuple, np.ndarray)):
        # Handle vector input
        target_direction = np.asarray(target_direction)
        if target_direction.shape != (3,):
            raise ValueError("Vector target_direction must have shape (3,)")
        # Normalize vector
        norm = np.linalg.norm(target_direction)
        if norm < 1e-6:
            raise ValueError("Axis vector cannot be zero vector")
        target_direction_vec = target_direction / norm
    else:
        raise TypeError("Axis must be either a string or a 3D vector")

    # Extract grasp direction
    if gripper_up_axis == "x" or gripper_up_axis == "+x":
        grasp_direction_vec = grasp_poses[:, :3, 0]
    elif gripper_up_axis == "y" or gripper_up_axis == "+y":
        grasp_direction_vec = grasp_poses[:, :3, 1]
    elif gripper_up_axis == "z" or gripper_up_axis == "+z":
        grasp_direction_vec = grasp_poses[:, :3, 2]
    elif gripper_up_axis == "-x":
        grasp_direction_vec = -grasp_poses[:, :3, 0]
    elif gripper_up_axis == "-y":
        grasp_direction_vec = -grasp_poses[:, :3, 1]
    elif gripper_up_axis == "-z":
        grasp_direction_vec = -grasp_poses[:, :3, 2]
    else:
        raise ValueError(
            "Invalid gripper_up_axis string. Must be one of: 'x', 'y', 'z', '-x', '-y', '-z', '+x', '+y', '+z'"
        )

    # Calculate dot product of gripper direction vector and target vector
    cosines = np.einsum("ij,j->i", grasp_direction_vec, target_direction_vec)
    # Ensure numerical stability
    cosines = np.clip(cosines, -1.0, 1.0)
    angles = np.arccos(cosines)

    mask = angles < threshold
    filtered_poses = grasp_poses[mask]
    filtered_widths = grasp_widths[mask]

    logger.info(f"Filtered grasp poses by gripper direction: {len(filtered_poses)}/{len(grasp_poses)}")

    return filtered_poses, filtered_widths, mask
