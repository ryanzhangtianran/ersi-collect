# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

# This file stores some additional math calculation related functions

from ast import Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R


def check_pose_similar(
    frame0: dict,
    obj0: Tuple(str, str),
    frame1: dict,
    obj1: Tuple(str, str),
    pos_threshold: list = [0.01, 0.01, 0.01],
    euler_threshold: list = [5, 5, 5],
    check_exist: bool = True,
):
    """
    Check if poses of two objects in two frames are consistent
    Objects can be the same, meaning checking if same object's poses in two frames are consistent
    Input obj0 and obj1 are Tuple variables containing two str variables,
    first represents object type, can be one of 'object','camera','gripper'
    second represents object name, if 'object' is scene object name, 'camera' is scene camera name, 'gripper' is left or right
    euler_threshold unit is degrees
    """

    if obj0[0] == "object":
        if obj0[1] not in frame0["objects"]:
            return not check_exist
        obj0_pose = np.array(frame0["objects"][obj0[1]]["pose"])
    elif obj0[0] == "camera":
        if obj0[1] not in frame0["cameras"]:
            return not check_exist
        obj0_pose = np.array(frame0["cameras"][obj0[1]]["pose"])
    elif obj0[0] == "gripper":
        if obj0[1] not in frame0["ee"]:
            return not check_exist
        obj0_pose = np.array(frame0["ee"][obj0[1]]["pose"])
    else:
        raise ValueError(f"obj0[0] must be one of ['object','camera','gripper'], current is {obj0[0]}")
    obj0_pos = obj0_pose[:3, 3]
    if abs(obj0_pos[0]) > 500 or abs(obj0_pos[1]) > 500 or abs(obj0_pos[2]) > 500:
        return not check_exist
    obj0_euler = R.from_matrix(obj0_pose[:3, :3]).as_euler("yzx", degrees=True)

    if obj1[0] == "object":
        obj1_pose = np.array(frame1["objects"][obj1[1]]["pose"])
    elif obj1[0] == "camera":
        obj1_pose = np.array(frame1["cameras"][obj1[1]]["pose"])
    elif obj1[0] == "gripper":
        obj1_pose = np.array(frame1["ee"][obj1[1]]["pose"])
    else:
        raise ValueError(f"obj1[0] must be one of ['object','camera','gripper'], current is {obj1[0]}")
    obj1_pos = obj1_pose[:3, 3]
    obj1_euler = R.from_matrix(obj1_pose[:3, :3]).as_euler("yzx", degrees=True)

    error = np.abs(obj0_pos - obj1_pos)
    is_pos_similar2start = np.all((error >= 0) & (error <= np.array(pos_threshold)))
    if not is_pos_similar2start:
        return False
    error = np.abs(obj0_euler - obj1_euler)
    is_euler_similar2start = np.all((error >= 0) & (error <= np.array(euler_threshold)))
    if not is_euler_similar2start:
        return False

    return True


# Convert input string or list representing vector to corresponding np.array format vector
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
