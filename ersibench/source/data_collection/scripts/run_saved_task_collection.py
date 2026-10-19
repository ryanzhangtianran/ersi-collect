# -*- coding: utf-8 -*-
# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

import argparse
import copy
import json
import os
import sys

root_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_directory)

from client.agent.omniagent import DataCollectionAgent
from client.layout.task_generate import TaskGenerator
from client.robot.omni_robot import IsaacSimRpcRobot
from common.base_utils.logger import logger
from common import generalization
from common.task_file import load_task


def _parse_float_list(value, expected_len):
    values = [float(item) for item in value.split(",")]
    if len(values) != expected_len:
        raise argparse.ArgumentTypeError(f"expected {expected_len} comma-separated values, got {len(values)}")
    return values


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run data collection from pre-generated saved_task JSON files.")
    parser.add_argument("--client_host", type=str, default="localhost:50051")
    parser.add_argument("--use_recording", action="store_true", default=False)
    parser.add_argument("--task_template", type=str, required=True)
    parser.add_argument("--task_folder", type=str, required=True)
    parser.add_argument("--robot_position", type=lambda v: _parse_float_list(v, 3), default=None)
    parser.add_argument("--robot_quaternion", type=lambda v: _parse_float_list(v, 4), default=None)
    args = parser.parse_args()

    task_info = load_task(args.task_template)  # task json + its scene json
    task_info = generalization.apply(task_info)  # in_domain tier + ERSIBENCH_OOD_AXES

    # Use the template only for scene, robot, workspace and recording settings.
    # The task instances themselves are read directly from args.task_folder.
    task_generator = TaskGenerator(copy.deepcopy(task_info))
    robot_position = args.robot_position or task_generator.robot_init_pose["position"]
    robot_rotation = args.robot_quaternion or task_generator.robot_init_pose["quaternion"]

    stand = task_info.get("robot", {}).get("stand", {"stand_type": "cylinder", "stand_size_x": 0.1, "stand_size_y": 0.1})
    robot_init_arm_pose = task_info.get("robot", {}).get("init_arm_pose")
    robot_init_arm_pose_noise = task_info.get("robot", {}).get("init_arm_pose_noise")
    robot_cfg = task_info["robot"]["robot_cfg"]

    robot = IsaacSimRpcRobot(
        robot_cfg=robot_cfg,
        # Task templates may list several candidate scenes. TaskGenerator
        # resolves that list to the selected scalar scene for this run.
        scene_usd=task_generator.scene_usd,
        client_host=args.client_host,
        position=robot_position,
        rotation=robot_rotation,
        stand_type=stand["stand_type"],
        stand_size_x=stand["stand_size_x"],
        stand_size_y=stand["stand_size_y"],
        robot_init_arm_pose=robot_init_arm_pose,
        robot_init_arm_pose_noise=robot_init_arm_pose_noise,
    )

    agent = DataCollectionAgent(robot)
    render_semantic = task_info.get("recording_setting", {}).get("render_semantic", False)
    agent.run_saved_tasks(
        task_folder=args.task_folder,
        camera_list=task_info["recording_setting"]["camera_list"],
        use_recording=args.use_recording,
        workspaces=task_generator.workspaces_in_world_frame,
        fps=task_info["recording_setting"]["fps"],
        render_semantic=render_semantic,
        origin_task_info=task_info,
    )
    logger.info("job done")
    robot.client.exit()
