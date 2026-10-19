# -*- coding: utf-8 -*-
# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

import argparse
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

def resolve_scene_usd(scene_info):
    scene_usd = scene_info.get("scene_usd")
    if isinstance(scene_usd, list):
        if not scene_usd:
            raise ValueError("scene.scene_usd must contain at least one USD path")
        scene_usd = scene_usd[0]
    if not isinstance(scene_usd, str) or not scene_usd:
        raise ValueError("scene.scene_usd must be a non-empty string or list")
    if os.path.dirname(scene_usd):
        return scene_usd
    scene_info_dir = scene_info.get("scene_info_dir", "")
    return os.path.join(scene_info_dir, scene_usd) if scene_info_dir else scene_usd

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SimGraspingAgent Command Line Interface")
    parser.add_argument(
        "--client_host",
        type=str,
        default="localhost:50051",
        help="The client host for SimGraspingAgent (default: localhost:50051)",
    )
    parser.add_argument(
        "--use_recording",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=10,
    )
    parser.add_argument(
        "--task_template",
        type=str,
        default="tasks/task.json",
        help="",
    )
    args = parser.parse_args()
    task_template_file = args.task_template
    task_info = load_task(task_template_file)  # task json + its scene json
    task_info = generalization.apply(task_info)  # in_domain tier + ERSIBENCH_OOD_AXES

    # The layouts are no longer generated up front: agent.run() draws one
    # immediately before each attempt and keeps it only if the episode
    # succeeded, so saved_task/ ends up mirroring recording_data/ one-to-one.
    if "rsi_recollection" in task_info:  # frozen scene from a scored RSI failure (djy_dev0)
        from common.eval_recollection import load_recollection
        task_folder, robot_pose, workspaces = load_recollection(task_template_file, task_info)
        task_generator = None
    else:
        task_generator = TaskGenerator(task_info)
        task_folder = "saved_task/%s" % (task_info["task"])
        robot_pose = task_generator.robot_init_pose
        workspaces = task_generator.workspaces_in_world_frame
    recording_folder = "recording_data"
    # Finished episodes may have been moved to an archive (bind-mounted here
    # read-only by run_data_collection.sh); it only feeds the episode numbering.
    archive_folder = "recording_archive"
    robot_position = robot_pose["position"]
    robot_rotation = robot_pose["quaternion"]
    stand = {"stand_type": "cylinder", "stand_size_x": 0.1, "stand_size_y": 0.1}
    robot_init_arm_pose = None
    robot_init_arm_pose_noise = None
    robot_cfg = task_info["robot"]["robot_cfg"]
    if "stand" in task_info["robot"]:
        stand = task_info["robot"]["stand"]
    if "init_arm_pose" in task_info["robot"]:
        robot_init_arm_pose = task_info["robot"]["init_arm_pose"]
    if "init_arm_pose_noise" in task_info["robot"]:
        robot_init_arm_pose_noise = task_info["robot"]["init_arm_pose_noise"]

    robot = IsaacSimRpcRobot(
        robot_cfg=robot_cfg,
        scene_usd=resolve_scene_usd(task_info["scene"]),
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
    render_semantic = False
    if "render_semantic" in task_info["recording_setting"]:
        render_semantic = task_info["recording_setting"]["render_semantic"]
    recording_setting = task_info["recording_setting"]
    agent.run(
        task_generator=task_generator,
        save_dir=task_folder,
        recording_dir=recording_folder,
        camera_list=recording_setting["camera_list"],
        use_recording=args.use_recording,
        workspaces=workspaces,
        # num_of_episode has always meant "attempts", not "episodes kept".
        # num_of_success is optional and lets a caller stop as soon as it has
        # enough, instead of burning the rest of the budget.
        max_attempts=recording_setting["num_of_episode"],
        target_success=recording_setting.get("num_of_success"),
        archive_dir=archive_folder,
        fps=recording_setting["fps"],
        render_semantic=render_semantic,
        origin_task_info=task_info,
    )
    logger.info("job done")
    robot.client.exit()
