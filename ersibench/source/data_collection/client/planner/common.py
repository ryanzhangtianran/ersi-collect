# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

from typing import Tuple

import numpy as np

from common.base_utils.transform_utils import euler2mat


def overweite_grasp_data(N=100):
    x = np.random.uniform(-0.01, 0.01, N)
    y = np.random.uniform(-0.01, 0.01, N)
    z = np.random.uniform(-0.01, 0.01, N)

    transforms = np.zeros((N, 4, 4))
    grasp_widths = np.full(N, 0.02)

    for i in range(N):
        position = np.array([x[i], y[i], z[i]])
        # Direction vector from grasp point to origin (i.e., -z direction)
        direction_to_origin = -position
        # Handle case when position is at origin
        if np.linalg.norm(direction_to_origin) < 1e-6:
            # If at origin, use default orientation (z-axis forward)
            rotation = np.eye(3)
        else:
            direction_to_origin = direction_to_origin / np.linalg.norm(direction_to_origin)
            # Align gripper's y-axis with world coordinate system y-axis as much as possible
            y_axis = np.array([0, 1, 0])
            # Calculate x-axis perpendicular to main direction
            if np.abs(np.dot(direction_to_origin, y_axis)) > 0.99:
                # Avoid parallel case, use backup reference axis
                y_axis = np.array([0, 0, 1])
                z_axis = direction_to_origin
                x_axis = np.cross(y_axis, z_axis)
                x_axis = x_axis / np.linalg.norm(x_axis)
                y_axis = np.cross(z_axis, x_axis)
            else:
                z_axis = direction_to_origin
                x_axis = np.cross(y_axis, z_axis)
                x_axis = x_axis / np.linalg.norm(x_axis)
                y_axis = np.cross(z_axis, x_axis)

            # Build rotation matrix
            rotation = np.vstack((x_axis, y_axis, z_axis)).T

        # 3. Build 4x4 homogeneous transformation matrix
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = position

        # 4. Store result
        transforms[i] = T

    return transforms, grasp_widths


def generate_random_pose(
    pose: np.ndarray,
    position_std: Tuple[float, float, float] = (0.1, 0.1, 0.1),
    rotation_std: Tuple[float, float, float] = (0.1, 0.1, 0.1),
    num: int = 10,
):
    """
    Generate a random pose by adding Gaussian noise to the position and rotation.
    Args:
        pose (np.ndarray): The original pose in the form of a 4x4 transformation matrix.
        position_std (Tuple[float, float, float]): Standard deviation for the position noise.
        rotation_std (Tuple[float, float, float]): Standard deviation for the rotation noise in radians.
        num (int): Number of random poses to generate.
    Returns:
        np.ndarray: A set of new poses with added noise.
    """
    assert pose.shape == (4, 4), "Pose must be a 4x4 transformation matrix."
    # Generate random noise for position
    res = []
    for _ in range(num):
        position_noise = np.random.normal(loc=0.0, scale=position_std, size=(3,))
        # Generate random noise for rotation
        rotation_noise = np.random.normal(loc=0.0, scale=rotation_std, size=(3,))
        # Create a rotation matrix from the noise
        rotation_matrix = euler2mat(rotation_noise, order="xyz")
        # Create a new pose by applying the noise
        new_pose = np.eye(4)
        new_pose[:3, :3] = pose[:3, :3] @ rotation_matrix
        new_pose[:3, 3] = pose[:3, 3] + position_noise
        res.append(new_pose)
    return np.array(res)
