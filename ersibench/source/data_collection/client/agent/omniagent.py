# -*- coding: utf-8 -*-
# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

import copy
import glob
import json
import os
import pickle
import re
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from client.agent.base import BaseAgent
from client.layout.object import OmniObject
from client.planner.action.action_script import ActionScript
from client.planner.action.zero_dim.transition import expand_dual_stage
from client.planner.action.stage import Action, Stage
from client.planner.ik_fallback import reachable_pose
from client.agent.bimanual import step_bimanual
from client.robot import Robot
from common.base_utils.logger import logger
from common.base_utils.transform_utils import (
    calculate_rotation_matrix,
    pose_difference,
    pose_from_position_quaternion,
    quaternion_rotate,
)

MAX_ATTEMPTIONS = 8  # was 4: chained pick/place tasks need more per-stage retries


def load_task_solution(task_info):
    stages = task_info["stages"]

    stage_related_objs = []
    processed_stages = []
    for i, stage in enumerate(stages):
        stage_related_objs.append(stage.get("active", {}).get("object_id", None))
        stage_related_objs.append(stage.get("passive", {}).get("object_id", None))
        for arm_stage in (stage.get("left", {}), stage.get("right", {})):
            stage_related_objs.append(arm_stage.get("active", {}).get("object_id", None))
            stage_related_objs.append(arm_stage.get("passive", {}).get("object_id", None))
        processed_stages.append(copy.deepcopy(stage))
    stage_related_objs = list(set(stage_related_objs))
    task_info["stages"] = processed_stages

    objects = {"gripper": OmniObject("gripper")}

    for obj_info in task_info["objects"]:
        obj_id = obj_info["object_id"]
        if "fix_pose" in obj_id:
            obj = OmniObject(obj_id)
            if "position" not in obj_info or "direction" not in obj_info:
                logger.error(f"Error: Missing position/direction in object {obj_id}")
                continue
            obj.set_pose(np.array(obj_info["position"]), np.array([0.001, 0.001, 0.001]))
            obj.elements = {
                "active": {},
                "passive": {
                    "place": {
                        "default": [
                            {
                                "xyz": np.array([0, 0, 0]),
                                "direction": np.array(obj_info["direction"]),
                            }
                        ]
                    }
                },
            }
            objects[obj_id] = obj
        else:
            obj_dir = obj_info["data_info_dir"]
            obj = OmniObject.from_obj_dir(
                obj_dir,
                obj_info=obj_info,
                object_id=obj_id,
                is_key_object=obj_id in stage_related_objs,
            )
            objects[obj_id] = obj

            if hasattr(obj, "part_ids"):
                if hasattr(obj, "part_joint_limits") and obj.part_joint_limits is not None:
                    obj_parts_joint_limits = obj.part_joint_limits
                for part_id in obj.part_ids:
                    id = obj_id + "/%s" % part_id
                    objects[id] = copy.deepcopy(obj)
                    objects[id].name = id
                    objects[id].prim_path += f"/{part_id}"
                    # pose_prim is fixed at construction (object.py:113), so without this a part
                    # reports the object root's pose: interaction labels would be read in the root
                    # frame and would not follow a part that moves (a drawer sliding open).
                    objects[id].pose_prim = objects[id].prim_path
                    if part_id in obj_parts_joint_limits:
                        objects[id].part_joint_limit = obj_parts_joint_limits[part_id]
                if len(obj.part_ids):
                    del objects[obj_id]
    return objects


def workspace_target(workspace):
    """World position a lift is sent to: a SPACE workspace's centre, or one of a SAMPLE workspace's poses (random choice)
    moved by its random jitter, the same draw the layout makes for objects placed there. Was: always poses[0], unjittered,
    and a KeyError for SPACE workspaces."""
    if "poses" not in workspace:
        return list(workspace["position"])
    pose = workspace["poses"][np.random.randint(len(workspace["poses"]))]
    delta = np.asarray(pose.get("delta_position", [0.0, 0.0, 0.0]), dtype=float)
    return (np.asarray(pose["position"], dtype=float) + np.random.uniform(-delta, delta)).tolist()


class DataCollectionAgent(BaseAgent):
    def __init__(self, robot: Robot):
        super().__init__(robot)
        self.action_script = ActionScript()
        self.attached_obj_id = None
        self.attached_obj_ids = {"left": None, "right": None}

    def start_recording(
        self,
        task_name,
        camera_prim_list,
        fps,
        render_semantic=False,
        recording_setting={},
    ):
        self.robot.client.start_recording(
            task_name=task_name,
            fps=fps,
            data_keys={
                "camera": {
                    "camera_prim_list": camera_prim_list,
                    "render_depth": False,
                    "render_semantic": render_semantic,
                },
                "joint_position": True,
                "gripper": True,
                "additional_parameters": json.dumps(recording_setting),
            },
        )

    def generate_layout(self, task_file):
        self.task_file = task_file
        with open(task_file, "r") as f:
            task_info = json.load(f)

        self.articulated_objs = []
        set_poses = []
        for object_info in task_info["objects"]:
            if "fix_pose" in object_info["object_id"]:
                continue
            is_articulated = object_info.get("is_articulated", False)
            if is_articulated:
                self.articulated_objs.append(object_info["object_id"])
            object_info["material"] = "general"
            if object_info.get("scene_object", False):
                if task_info.get("rsi_recollection"):
                    # The archived USD already includes parent and rigid-body transforms.
                    continue
                if "position" not in object_info or "quaternion" not in object_info:
                    logger.error("Scene object %s missing position or quaternion, skipping." % object_info["object_id"])
                    continue
                object_pose = {}
                name = object_info["object_id"]
                object_pose["prim_path"] = object_info.get("prim_path", "/World/Objects/%s" % name)
                object_pose["position"] = np.array(object_info["position"])
                object_pose["rotation"] = np.array(object_info["quaternion"])
                set_poses.append(object_pose)
            else:
                self.add_object(object_info)
                time.sleep(0.1)
                self.add_object(object_info)
                time.sleep(0.2)
        if len(set_poses):
            self.robot.client.set_object_pose(set_poses, [])
        time.sleep(2)

        self.arm = task_info["arm"]

        task_related_objs = []
        for stage in task_info["stages"]:
            # gripper-only stages (bimanual wrapper, lift, reset) may leave active/passive out
            stage.setdefault("active", {"object_id": "gripper", "primitive": None})
            stage.setdefault("passive", {"object_id": "gripper", "primitive": None})
            for type in ["active", "passive"]:
                obj_id = stage[type]["object_id"]
                if obj_id == "gripper" or obj_id in task_related_objs:
                    continue
                task_related_objs.append(obj_id)

        target_lookat_point = []
        for obj in task_info["objects"]:
            if obj["object_id"] not in task_related_objs or "position" not in obj:
                continue
            target_lookat_point.append(obj["position"])
        if len(target_lookat_point) and not task_info.get("rsi_recollection"):
            target_lookat_point = np.mean(np.stack(target_lookat_point), axis=0)
            self.robot.client.set_target_point(target_lookat_point.tolist())

        """ Set light """
        light_infos = []
        if "lights" in task_info:
            for key in task_info["lights"]:
                light_infos += task_info["lights"][key]
            if len(light_infos):
                original_light_infos = copy.deepcopy(light_infos)
                for idx, info in enumerate(light_infos):
                    if "light_temperature" in info:
                        temperature_random_ratio = min(max(0.4, np.random.normal(1, 0.2)), 1.6)
                        info["light_temperature"] = (
                            original_light_infos[idx]["light_temperature"] * temperature_random_ratio
                        )
                    if "light_intensity" in info:
                        intensity_random_ratio = max(0.1, np.random.normal(1, 0.3))
                        info["light_intensity"] = original_light_infos[idx]["light_intensity"] * intensity_random_ratio
                    if "rotation" in info:
                        info["rotation"] = original_light_infos[idx]["rotation"]
                    else:
                        info["rotation"] = quaternion_rotate(
                            np.array([1, 0.0, 0.0, 0.0]),
                            "z",
                            np.random.choice(np.arange(0, 180, 15)),
                        ).tolist()
                self.robot.client.set_light(light_infos)
                time.sleep(2)

        """ Set camera"""
        if "cameras" in task_info:
            for cam_id in task_info["cameras"]:
                cam_info = task_info["cameras"][cam_id]
                self.robot.client.add_camera(
                    cam_id,
                    cam_info["position"],
                    cam_info["quaternion"],
                    cam_info["width"],
                    cam_info["height"],
                    cam_info["focal_length"],
                    cam_info["horizontal_aperture"],
                    cam_info["vertical_aperture"],
                    cam_info["is_local"],
                )

    def place_robot_base(self, pose):
        """Move the robot base to the layout's robot_init_pose (TaskGenerator draws it per layout). The server moves the
        whole robot and what reads its pose (ROBOT_BASE in command_controller); layouts without the field keep the base
        the container started with."""
        if not pose:
            return
        logger.info(f"robot base -> {pose}")
        self.robot.client.set_object_pose(
            [{"prim_path": "robot", "position": pose["position"], "rotation": pose["quaternion"]}], []
        )
        # The client converts world targets to the base frame with its own copy of the base pose (omni_robot.move for
        # AvoidObs motions, stage.py's offset check). Left at the pose the robot was built with, every grasp landed off
        # by the jitter difference (up to ~12 cm) and closed on air (GRASP_FOLLOW_FAILED).
        self.robot.init_position = list(pose["position"])
        self.robot.init_rotation = list(pose["quaternion"])

    def perturb_cameras(self, camera_list, noise):
        """Offset each recorded camera's local pose by U(-thresh, thresh) per axis, the benchmark's
        generalization.camera.position contract (x/y/z in m, roll/pitch/yaw in deg, rotation applied in the parent frame).
        Same thresholds as eval_tasks/*_campose.json make the collected data cover that OOD axis. A fresh draw every
        episode; the server composes the offset on the camera's original pose, so nothing accumulates."""
        if not noise.get("enable"):
            return
        th = noise["threshold"]
        poses = []
        for cam in camera_list:
            d = {k: np.random.uniform(-th.get(k, 0.0), th.get(k, 0.0)) for k in ("x", "y", "z", "roll", "pitch", "yaw")}
            q = R.from_euler("xyz", [d["roll"], d["pitch"], d["yaw"]], degrees=True).as_quat()  # xyzw
            poses.append({"prim_path": cam + "?local_delta", "position": [d["x"], d["y"], d["z"]],
                          "rotation": [q[3], q[0], q[1], q[2]]})
            logger.info(f"camera pose noise {cam}: {d}")
        self.robot.client.set_object_pose(poses, [])

    def perturb_lights(self, cfg):
        """Retune every light in the scene for this episode (server ALL_LIGHTS path). Two forms, from
        generalization.in_domain / ood.light: {intensity_ratio, temperature_ratio} = U(1-r, 1+r) scale of the scene's own
        values; {intensity: [...], temperature: [...]} = one absolute pick each, the benchmark's light generalization."""
        if not cfg:
            return
        if "intensity" in cfg or "temperature" in cfg:
            kind, i, t = "absolute", float(np.random.choice(cfg["intensity"])), float(np.random.choice(cfg["temperature"]))
        else:
            ri, rt = cfg.get("intensity_ratio", 0.0), cfg.get("temperature_ratio", 0.0)
            kind, i, t = "ratio", np.random.uniform(1 - ri, 1 + ri), np.random.uniform(1 - rt, 1 + rt)
        logger.info(f"lights {kind}: intensity {i:.3g} temperature {t:.3g}")
        self.robot.client.set_light([{"light_type": kind, "light_prim": "*", "light_temperature": t, "light_intensity": i,
                                      "rotation": [1.0, 0.0, 0.0, 0.0], "texture": ""}])

    def update_objects(self, objects, arm="right"):
        # update gripper pose
        objects["gripper"].obj_pose = self.robot.get_ee_pose(ee_type="gripper", id=arm)

        # update object pose
        for obj_id in objects:
            if obj_id == "gripper":
                continue
            if "fix_pose" in obj_id:
                if len(objects[obj_id].obj_pose) == 3:
                    position = objects[obj_id].obj_pose
                    rotation_matrix = calculate_rotation_matrix(objects[obj_id].direction, [0, 0, 1])
                    objects[obj_id].obj_pose = np.eye(4)
                    objects[obj_id].obj_pose[:3, 3] = position.flatten()
                    objects[obj_id].obj_pose[:3, :3] = rotation_matrix
                continue
            if "/" in obj_id:
                obj_id.split("/")[0]
                part_name = obj_id.split("/")[1]
                dof_joint = self.robot.client.get_part_dof_joint(objects[obj_id].parent_prim_path, part_name)
                objects[obj_id].joint_position = dof_joint.joint_position
                objects[obj_id].joint_velocity = dof_joint.joint_velocity

            objects[obj_id].obj_pose = self.robot.get_prim_world_pose(objects[obj_id].pose_prim)
            if (
                hasattr(objects[obj_id], "info")
                and "simple_place" in objects[obj_id].info
                and objects[obj_id].info["simple_place"]
            ):
                down_direction_world = (np.linalg.inv(objects[obj_id].obj_pose) @ np.array([0, 0, -1, 1]))[:3]
                down_direction_world = down_direction_world / np.linalg.norm(down_direction_world) * 0.08
                objects[obj_id].elements["active"]["place"]["direction"] = down_direction_world

        return objects

    def check_task_file(self, task_file):
        with open(task_file, "r") as f:
            task_info = json.load(f)

        objs_dir = {}
        objs_interaction = {}
        for obj_info in task_info["objects"]:
            obj_id = obj_info["object_id"]
            if "fix_pose" in obj_id:
                continue
            objs_dir[obj_id] = obj_info["data_info_dir"]
            if "interaction" in obj_info:
                objs_interaction[obj_id] = obj_info["interaction"]
            else:
                object_dir_base_name = os.path.basename(os.path.normpath(objs_dir[obj_id]))
                interaction_label_file = os.path.join(
                    os.environ.get("SIM_ASSETS"),
                    "interaction",
                    object_dir_base_name,
                    "interaction.json",
                )
                assert os.path.exists(interaction_label_file), f"interaction.json not found in {interaction_label_file}"
                objs_interaction[obj_id] = json.load(open(interaction_label_file))["interaction"]

        def labelled_inline(obj_id, primitive):
            """Grasp labels written into the task template are poses, not a path: nothing to check on disk
            (object.py:164 tells the two apart the same way)."""
            grasp = objs_interaction.get(obj_id, {}).get("passive", {}).get("grasp", {})
            return isinstance(grasp.get(primitive if primitive is not None else "default"), dict)

        for stage in task_info["stages"]:
            active_obj_id = stage.get("active", {"object_id": "gripper"})["object_id"]
            passive_obj_id = stage.get("passive", {"object_id": "gripper"})["object_id"]

            if active_obj_id != "gripper":
                if isinstance(active_obj_id, list):
                    for id in active_obj_id:
                        if id not in objs_dir:
                            logger.info("Active obj not in objs_dir: %s" % id)
                            return False
                elif isinstance(active_obj_id, dict):  # If it's a dictionary
                    for id in active_obj_id.values():  # Iterate over all values in the dictionary
                        if id not in objs_dir and "fix_pose" not in id:
                            logger.info("Active obj not in objs_dir: %s" % id)
                            return False
                else:
                    if active_obj_id not in objs_dir:
                        logger.info("Active obj not in objs_dir: %s" % active_obj_id)
                        return False

            if passive_obj_id != "gripper" and "fix_pose" not in passive_obj_id:
                if isinstance(passive_obj_id, list):
                    for id in passive_obj_id:
                        if id not in objs_dir:
                            logger.info("Passive obj not in objs_dir: %s" % id)
                            return False
                elif isinstance(passive_obj_id, dict):  # If it's a dictionary
                    for id in passive_obj_id.values():  # Iterate over all values in the dictionary
                        if id not in objs_dir and "fix_pose" not in id:
                            logger.info("Passive obj not in objs_dir: %s" % id)
                            return False
                else:
                    if passive_obj_id not in objs_dir:
                        logger.info("Passive obj not in objs_dir: %s" % passive_obj_id)
                        return False

            if stage["action"] in ["grasp", "pick"]:
                passive_obj_id = stage["passive"]["object_id"]
                if isinstance(passive_obj_id, list):
                    for id in passive_obj_id:
                        if labelled_inline(id, stage["passive"]["primitive"]):
                            continue
                        obj_dir = objs_dir[id]
                        object_dir_base_name = os.path.basename(os.path.normpath(obj_dir))
                        primitive = stage["passive"]["primitive"]
                        if primitive is None:
                            file = "grasp_pose/grasp_pose.pkl"
                        else:
                            file = objs_interaction[id]["passive"]["grasp"][primitive]
                            if isinstance(file, list):
                                file = file[0]
                        grasp_file = os.path.join(
                            os.environ.get("SIM_ASSETS"), "interaction", object_dir_base_name, file
                        )
                        if not os.path.exists(grasp_file):
                            logger.info("-- Grasp file not exist: %s" % grasp_file)
                            return False

                        _data = pickle.load(open(grasp_file, "rb"))
                        if len(_data["grasp_pose"]) == 0:
                            logger.info("-- Grasp file empty: %s" % grasp_file)
                            return False
                elif not labelled_inline(passive_obj_id, stage["passive"]["primitive"]):
                    obj_dir = objs_dir[passive_obj_id]
                    object_dir_base_name = os.path.basename(os.path.normpath(obj_dir))
                    primitive = stage["passive"]["primitive"]
                    if primitive is None:
                        file = "grasp_pose/grasp_pose.pkl"
                    else:
                        file = objs_interaction[passive_obj_id]["passive"]["grasp"][primitive]
                        if isinstance(file, list):
                            file = file[0]
                    grasp_file = os.path.join(os.environ.get("SIM_ASSETS"), "interaction", object_dir_base_name, file)
                    if not os.path.exists(grasp_file):
                        logger.info("-- Grasp file not exist: %s" % grasp_file)
                        return False

                    _data = pickle.load(open(grasp_file, "rb"))
                    if len(_data["grasp_pose"]) == 0:
                        logger.info("-- Grasp file empty: %s" % grasp_file)
                        return False
        return True

    def _select_pick_arm(self, stage_config, following_configs, objects):
        object_pose = objects[stage_config["passive"]["object_id"]].obj_pose
        gripper_poses = {
            arm: self.robot.get_ee_pose(ee_type="gripper", id=arm) for arm in ("left", "right")
        }
        arms = sorted(gripper_poses, key=lambda arm: pose_difference(gripper_poses[arm], object_pose)[0])
        if stage_config["extra_params"].get("error_data"):
            return arms[0]

        # The grasp's lookahead checks that this arm can also reach what follows (the carry to a slot, a cup, ...), so
        # the probe needs those stages too: everything up to the next pick.
        configs = [stage_config]
        for config in following_configs:
            if config["action"] in ("pick", "grasp", "bimanual"):
                break
            configs.append(config)
        for arm in arms:
            # Lookahead updates object alignment and the next stage's primitive; isolate the probe.
            probe_objects = copy.deepcopy(objects)
            probe_objects["gripper"].obj_pose = gripper_poses[arm]
            probe_configs = copy.deepcopy(configs)
            for config in probe_configs:
                config.setdefault("extra_params", {})["arm"] = arm
                if isinstance(config.get("passive", {}).get("object_id"), dict):
                    config["passive"]["object_id"] = config["passive"]["object_id"][arm]
            probe = ActionScript()
            probe.initialize({"stages": probe_configs}, probe_objects)
            if probe.stages[0].select_pose(probe_objects, self.robot):
                return arm
            logger.info(f"Auto pick {stage_config['passive']['object_id']}: no feasible sequence for {arm}")
        return None

    # Load task after self.check_task_file succeeds
    def load_task(
        self,
        task_file,
        use_recording,
        camera_list,
        fps,
        render_semantic,
        workspaces,
        origin_task_info,
    ):
        logger.info(f"Start Task{task_file}")
        self.reset()
        self.attached_obj_id = None
        self.attached_obj_ids = {"left": None, "right": None}

        self.place_robot_base(json.load(open(task_file)).get("robot_init_pose"))
        self.generate_layout(task_file)
        self.perturb_cameras(camera_list, origin_task_info.get("camera_pose_noise", {}))
        self.perturb_lights(origin_task_info.get("lights_config", {}))
        self.robot.open_gripper(id="right", detach=False)
        self.robot.open_gripper(id="left", detach=False)
        self.robot.client.detach_obj()
        time.sleep(2.0)

        self.robot.reset_pose = {
            "right": self.robot.get_ee_pose(ee_type="gripper", id="right"),
            "left": self.robot.get_ee_pose(ee_type="gripper", id="left"),
        }
        logger.info(f"Reset pose{self.robot.reset_pose}")

        task_info = json.load(open(task_file, "rb"))

        objects = load_task_solution(task_info)
        objects = self.update_objects(objects)

        # Modify arm parameters for pick and place stages
        task_info["stages"] = [expand_dual_stage(stage) for stage in task_info["stages"]]
        for stage_index, stage in enumerate(task_info["stages"]):
            # only a grasp names objects; a transition, an ungrasp or a pair may leave them out
            stage.setdefault("active", {"object_id": "gripper", "primitive": None})
            stage.setdefault("passive", {"object_id": "gripper", "primitive": None})
            stage.setdefault("extra_params", {})
            # Resolve a lift stage's target workspace to a world position
            for sub_stage in (stage, stage.get("left", {}), stage.get("right", {})):
                workspace_id = sub_stage.get("extra_params", {}).get("target_workspace_id")
                if workspace_id:
                    sub_stage["extra_params"]["target_position"] = workspace_target(workspaces[workspace_id])
                # ... or to a prim of the loaded scene (e.g. the burner of this kitchen): the target then follows the
                # background instead of being typed into the task; target_world_offset still applies on top
                target_prim = sub_stage.get("extra_params", {}).get("target_prim_path")
                if target_prim:
                    sub_stage["extra_params"]["target_position"] = self.robot.get_prim_world_pose(target_prim)[:3, 3].tolist()
            # Check if active_object_id and passive_object_id are lists
            if isinstance(stage["active"]["object_id"], list):
                rule = stage["active"].get("rule", "random_choice")
                if rule == "random_choice":
                    active_obj_id = np.random.choice(stage["active"]["object_id"])
                    stage["active"]["object_id"] = active_obj_id
                    logger.info(f"Stage {stage['action']} randomly selected active object id: {active_obj_id}")
                else:
                    raise NotImplementedError(f"Rule {rule} for active object selection is not implemented")
            if isinstance(stage["passive"]["object_id"], list):
                rule = stage["passive"].get("rule", "random_choice")
                if rule == "random_choice":
                    passive_obj_id = np.random.choice(stage["passive"]["object_id"])
                    stage["passive"]["object_id"] = passive_obj_id
                    logger.info(f"Stage {stage['action']} randomly selected passive object id: {passive_obj_id}")
                else:
                    raise NotImplementedError(f"Rule {rule} for passive object selection is not implemented")

            if stage["action"] in ("pick", "grasp") and stage["extra_params"]["arm"] == "auto":
                arm = self._select_pick_arm(stage, task_info["stages"][stage_index + 1:], objects)
                if arm is None:
                    logger.warning(f"Reject layout {task_file}: no feasible arm for {stage['passive']['object_id']}")
                    return None, objects
                logger.info("Auto select arm: %s" % arm)
                # Update arm parameter in stage
                stage["extra_params"]["arm"] = arm

            # Every stage after an auto pick carries on with the same hand; without the substitution "auto"
            # reaches the robot verbatim and the move is rejected ("robot: armauto").
            if stage.get("extra_params", {}).get("arm") == "auto":
                stage["extra_params"]["arm"] = arm

            # Only select based on arm when passive_obj_id is a dictionary
            if isinstance(stage["passive"]["object_id"], dict):
                stage["passive"]["object_id"] = stage["passive"]["object_id"][arm]

            if "checker" in stage:
                for checker in stage["checker"]:
                    if "params" not in checker:
                        continue
                    for key, value in checker["params"].items():
                        if isinstance(value, str):
                            if value == "gripper":
                                checker["params"][key] = stage["extra_params"]["arm"]
                            elif value in objects:
                                checker["params"][key] = objects[value].prim_path
        # Modify action_text and english_action_text in checker
        for stage in task_info["stages"]:
            if "action_description" in stage:
                stage_arm = stage.get("extra_params", {}).get("arm")
                if stage_arm not in ["left", "right"]:
                    continue
                if stage_arm == "left":
                    chinese_arm = "左"
                    english_arm_upper = "Left"
                    english_arm_lower = "left"
                elif stage_arm == "right":
                    chinese_arm = "右"
                    english_arm_upper = "Right"
                    english_arm_lower = "right"
                if "action_text" in stage["action_description"]:
                    stage["action_description"]["action_text"] = stage["action_description"]["action_text"].replace(
                        "{左/右}", chinese_arm
                    )
                if "english_action_text" in stage["action_description"]:
                    stage["action_description"]["english_action_text"] = stage["action_description"][
                        "english_action_text"
                    ].replace("{Left/Right}", english_arm_upper)
                    stage["action_description"]["english_action_text"] = stage["action_description"][
                        "english_action_text"
                    ].replace("{left/right}", english_arm_lower)
                if "action_text" in stage["action_description"]:
                    match = re.search(r"\{object:.*\}", stage["action_description"]["action_text"])
                    if match:
                        object_name = match.group(0).replace("{object:", "").replace("}", "")
                        for obj in task_info["objects"]:
                            if obj["object_id"] == object_name:
                                if "chinese_semantic_name" in obj:
                                    stage["action_description"]["action_text"] = stage["action_description"][
                                        "action_text"
                                    ].replace(match.group(0), obj["chinese_semantic_name"])
                                else:
                                    raise ValueError(f"Object {object_name} has no chinese_semantic_name")
                                break
                    match = re.search(r"\{position:.*\}", stage["action_description"]["action_text"])
                    if match:
                        object_name = match.group(0).replace("{position:", "").replace("}", "")
                        for obj in task_info["objects"]:
                            if obj["object_id"] == object_name:
                                if "chinese_position_semantic" in obj:
                                    stage["action_description"]["action_text"] = stage["action_description"][
                                        "action_text"
                                    ].replace(match.group(0), obj["chinese_position_semantic"])
                                else:
                                    raise ValueError(f"Object {object_name} has no chinese_position_semantic")
                                break
                if "english_action_text" in stage["action_description"]:
                    match = re.search(
                        r"\{object:.*\}",
                        stage["action_description"]["english_action_text"],
                    )
                    if match:
                        object_name = match.group(0).replace("{object:", "").replace("}", "")
                        for obj in task_info["objects"]:
                            if obj["object_id"] == object_name:
                                if "english_semantic_name" in obj:
                                    stage["action_description"]["english_action_text"] = stage["action_description"][
                                        "english_action_text"
                                    ].replace(match.group(0), obj["english_semantic_name"])
                                else:
                                    raise ValueError(f"Object {object_name} has no english_semantic_name")
                                break
                    match = re.search(r"\{position:.*\}", stage["action_description"]["english_action_text"])
                    if match:
                        object_name = match.group(0).replace("{position:", "").replace("}", "")
                        for obj in task_info["objects"]:
                            if obj["object_id"] == object_name:
                                if "english_position_semantic" in obj:
                                    stage["action_description"]["english_action_text"] = stage["action_description"][
                                        "english_action_text"
                                    ].replace(match.group(0), obj["english_position_semantic"])
                                else:
                                    raise ValueError(f"Object {object_name} has no english_position_semantic")
                                break
        # Modify english_task_name, task_name, init_scene_text in task_description
        if "task_description" in task_info:
            task_description = task_info["task_description"]
            if "english_task_name" in task_description:
                match = re.search(r"\{object:.*\}", task_description["english_task_name"])
                if match:
                    object_name = match.group(0).replace("{object:", "").replace("}", "")
                    for obj in task_info["objects"]:
                        if obj["object_id"] == object_name:
                            if "english_semantic_name" in obj:
                                task_description["english_task_name"] = task_description["english_task_name"].replace(
                                    match.group(0), obj["english_semantic_name"]
                                )
                            else:
                                raise ValueError(f"Object {object_name} has no english_semantic_name")
                            break
                match = re.search(r"\{position:.*\}", task_description["english_task_name"])
                if match:
                    object_name = match.group(0).replace("{position:", "").replace("}", "")
                    for obj in task_info["objects"]:
                        if obj["object_id"] == object_name:
                            if "english_position_semantic" in obj:
                                task_description["english_task_name"] = task_description["english_task_name"].replace(
                                    match.group(0), obj["english_position_semantic"]
                                )
                            else:
                                raise ValueError(f"Object {object_name} has no english_position_semantic")
                            break
            if "task_name" in task_description:
                match = re.search(r"\{object:.*\}", task_description["task_name"])
                if match:
                    object_name = match.group(0).replace("{object:", "").replace("}", "")
                    for obj in task_info["objects"]:
                        if obj["object_id"] == object_name:
                            if "chinese_semantic_name" in obj:
                                task_description["task_name"] = task_description["task_name"].replace(
                                    match.group(0), obj["chinese_semantic_name"]
                                )
                            else:
                                raise ValueError(f"Object {object_name} has no chinese_semantic_name")
                            break
                match = re.search(r"\{position:.*\}", task_description["task_name"])
                if match:
                    object_name = match.group(0).replace("{position:", "").replace("}", "")
                    for obj in task_info["objects"]:
                        if obj["object_id"] == object_name:
                            if "chinese_position_semantic" in obj:
                                task_description["task_name"] = task_description["task_name"].replace(
                                    match.group(0), obj["chinese_position_semantic"]
                                )
                            else:
                                raise ValueError(f"Object {object_name} has no chinese_position_semantic")
                            break
            if "init_scene_text" in task_description:
                match = re.search(r"\{object:.*\}", task_description["init_scene_text"])
                if match:
                    object_name = match.group(0).replace("{object:", "").replace("}", "")
                    for obj in task_info["objects"]:
                        if obj["object_id"] == object_name:
                            if "english_semantic_name" in obj:
                                task_description["init_scene_text"] = task_description["init_scene_text"].replace(
                                    match.group(0), obj["english_semantic_name"]
                                )
                            else:
                                raise ValueError(f"Object {object_name} has no english_semantic_name")
                            break
                match = re.search(r"\{position:.*\}", task_description["init_scene_text"])
                if match:
                    object_name = match.group(0).replace("{position:", "").replace("}", "")
                    for obj in task_info["objects"]:
                        if obj["object_id"] == object_name:
                            if "english_position_semantic" in obj:
                                task_description["init_scene_text"] = task_description["init_scene_text"].replace(
                                    match.group(0), obj["english_position_semantic"]
                                )
                            else:
                                raise ValueError(f"Object {object_name} has no english_position_semantic")
                            break

        self.action_script.initialize(task_info, objects)
        if use_recording:
            recording_setting = origin_task_info.get("recording_setting", {})
            self.start_recording(
                task_name="[%s]" % (os.path.basename(os.path.normpath(task_file)).split(".")[0]),
                camera_prim_list=camera_list,
                fps=fps,
                render_semantic=render_semantic,
                recording_setting=recording_setting,
            )

        return task_info, objects

    step_bimanual = step_bimanual

    def step(
        self,
        action: Action,
        stage: Stage,
        objects,
        workspaces,
        stage_id,
        index,
        step_index,
    ):
        logger.info(f"==> [RUN] Stage {stage_id} Substage {index} Step {step_index}: {stage.action_type}")
        step_success = False
        need_retry = True
        extra_params = stage.extra_params.copy()
        arm = extra_params.get("arm", "right")
        action_type = stage.action_type
        active_id = stage.active_obj_id
        passive_id = stage.passive_obj_id

        objects = self.update_objects(objects, arm=arm)

        (
            target_gripper_pose,
            motion_type,
            gripper_action,
            arm,
            action_description,
            action_extra_params,
        ) = stage.parse_action(action, objects)
        extra_params.update(action_extra_params)
        arm = extra_params.get("arm", "right")
        motion_run_ratio = 1.0
        remove_obstacles = False
        gripper_action_timing = {}
        error_data = {}
        if "error_data" in extra_params:
            error_data = extra_params["error_data"]
            error_params = error_data.get("params", {})
            motion_run_ratio = error_params.get("motion_run_ratio", 1.0)
            if error_data.get("type", None) == "Drop":
                timing = error_params.get("drop_timing", 0.2)
                gripper_action_timing = {
                    "state": "open",
                    "timing": timing,
                    "is_right": arm == "right",
                }
                gripper_action = "open"
            elif error_data.get("type", None) == "KeepClose":
                if action_type == "grasp":
                    remove_obstacles = True

        self.robot.client.set_frame_state(
            action_type,
            step_index,
            active_id,
            passive_id,
            self.attached_obj_id is not None,
            arm=arm,
            target_pose=target_gripper_pose,
            action_description=action_description,
            error_description=error_data,
        )

        # execution action
        goal_offset = extra_params.get("goal_offset", [0, 0, 0, 1, 0, 0, 0])
        path_constraint = extra_params.get("path_constraint", [])
        from_current_pose = extra_params.get("from_current_pose", False)
        offset_and_constraint_in_goal_frame = extra_params.get("offset_and_constraint_in_goal_frame", True)
        disable_collision_links = extra_params.get("disable_collision_links", [])
        if remove_obstacles:
            self.robot.client.remove_objs_from_obstacle([objects[stage.passive_obj_id].prim_path])

        # check if target_gripper_pose is reachable

        if target_gripper_pose is None:
            state = True
        else:
            if motion_type == "Simple":
                ik_success, _ = self.robot.solve_ik(
                    np.array([target_gripper_pose]),
                    ee_type="gripper",
                    arm=arm,
                    type="Simple",
                )
                if not ik_success[0]:
                    constrained = (
                        bool(path_constraint)
                        or from_current_pose or goal_offset != [0, 0, 0, 1, 0, 0, 0]
                    )
                    fallback = None if constrained else reachable_pose(
                        self.robot, target_gripper_pose, arm,
                        rotation_tolerance=extra_params.get("ik_fallback_rotation_deg", 8.0),
                    )
                    if fallback is None:
                        logger.warning(f"IK_UNREACHABLE stage={stage_id} step={step_index} arm={arm}")
                        return False, True
                    target_gripper_pose = fallback
                    logger.info(f"Substage {index} step_id {step_index}: selected nearest reachable fallback pose")
            state = self.robot.move_pose(
                target_gripper_pose,
                motion_type,
                arm=arm,
                block=True,
                goal_offset=goal_offset,
                path_constraint=path_constraint,
                offset_and_constraint_in_goal_frame=offset_and_constraint_in_goal_frame,
                disable_collision_links=disable_collision_links,
                motion_run_ratio=motion_run_ratio,
                gripper_action_timing=gripper_action_timing,
                from_current_pose=from_current_pose,
            )
        if not state:
            step_success = False
            need_retry = True
            return step_success, need_retry
        else:
            set_gripper_open = gripper_action == "open"
            set_gripper_close = gripper_action == "close"
            self.robot.client.set_frame_state(
                action_type,
                step_index,
                active_id,
                passive_id,
                self.attached_obj_id is not None,
                set_gripper_open,
                set_gripper_close,
                arm=arm,
                target_pose=target_gripper_pose,
                action_description=action_description,
                error_description=error_data,
            )
            self.robot.set_gripper_action(gripper_action, arm=arm)
            if set_gripper_open or set_gripper_close:
                time.sleep(1.0)

            self.robot.client.set_frame_state(
                action_type,
                step_index,
                active_id,
                passive_id,
                self.attached_obj_id is not None,
                arm=arm,
                target_pose=target_gripper_pose,
                action_description=action_description,
                error_description=error_data,
            )

            # check sub-stage completion
            objects["gripper"].obj_pose = self.robot.get_ee_pose(ee_type="gripper", id=arm)
            objects = self.update_objects(objects, arm=arm)

            step_success = stage.check_completion(objects, self.robot)
            self.robot.client.set_frame_state(
                action_type,
                step_index,
                active_id,
                passive_id,
                self.attached_obj_id is not None,
                arm=arm,
                target_pose=target_gripper_pose,
                action_description=action_description,
                error_description=error_data,
            )
            if not step_success:
                self.attached_obj_id = None
            else:
                logger.info(
                    f"action_stages: {stage_id}, antion: {action_type}, [stage {index} step_id {step_index}] success"
                )
                if not len(stage.active_action_sequence.actions) == step_index + 1:
                    logger.info("jump to next action")
                else:
                    logger.info("\n")

                newly_attached_obj_id = None
                if gripper_action == "close" and action_type == "grasp" and self.attached_obj_id is None:
                    self.attached_obj_id = stage.passive_obj_id
                    newly_attached_obj_id = self.attached_obj_id
                elif gripper_action == "open":
                    self.attached_obj_id = None
                if newly_attached_obj_id is not None:
                    arm = stage.extra_params.get("arm", "right")
                    if newly_attached_obj_id.split("/")[0] not in self.articulated_objs:
                        self.robot.client.attach_obj(
                            prim_paths=[objects[newly_attached_obj_id].prim_path + ("?stable_grasp" if stage.extra_params.get("stable_grasp") else "")],
                            is_right=arm == "right",
                        )
                self.robot.client.set_frame_state(
                    action_type,
                    step_index,
                    active_id,
                    passive_id,
                    self.attached_obj_id is not None,
                    arm=arm,
                    target_pose=target_gripper_pose,
                    action_description=action_description,
                    error_description=error_data,
                )
            objects = self.update_objects(objects, arm=arm)
        return step_success, need_retry

    def _next_layout_index(self, task_name, *recording_dirs):
        """One past the highest index any recording uses, across all given dirs.

        Recordings (not layouts) are the authority: the two are created and
        deleted together. Finished episodes may have been archived elsewhere
        (mounted read-only), hence several dirs; missing ones are skipped.
        Trailing digits are the server's collision counter on legacy names.
        """
        pattern = re.compile(r"^\[%s_(\d+)\]\d*$" % re.escape(task_name))
        highest = -1
        for directory in recording_dirs:
            if not directory or not os.path.isdir(directory):
                continue
            for entry in os.listdir(directory):
                match = pattern.match(entry)
                if match:
                    highest = max(highest, int(match.group(1)))
        return highest + 1

    def _generate_layout(self, task_generator, save_dir, task_name, index, max_attempt=5):
        """Write one randomized layout to ``<save_dir>/<task>_<index>.json``.

        Returns the path, or None if the generator could not place the
        objects — sampling can fail when the drawn poses do not fit the
        workspace, and upstream retried a few times before giving up.
        """
        output_file = os.path.join(save_dir, "%s_%d.json" % (task_name, index))
        for attempt in range(max_attempt):
            if task_generator.generate(output_file):
                return output_file
            logger.error(f"Attempt {attempt+1}/{max_attempt} failed to generate {output_file}, retrying...")
        return None

    def _run_task_file(
        self,
        task_file,
        use_recording,
        camera_list,
        fps,
        render_semantic,
        workspaces,
        origin_task_info,
    ):
        """Run one layout file end to end; True if the episode was kept."""
        success = True
        if not self.check_task_file(task_file):
            logger.error(f"Task file {task_file} check failed, skip this task")
            return False
        task_info, objects = self.load_task(
            task_file,
            use_recording,
            camera_list,
            fps,
            render_semantic,
            workspaces,
            origin_task_info,
        )
        if task_info is None:
            return None  # layout rejected before execution; the caller retries with a new one for free
        time.sleep(1)
        if "task_description" in task_info:
            self.robot.client.set_task_task_basic_info(task_info["task_description"])
        if "task_metric" in task_info:
            self.robot.client.set_task_metric(task_info["task_metric"])
        logger.info(f"Task {task_file} loaded")

        task_success = True
        last_pick_arm = None
        stages = list(self.action_script)
        stage_id, resumed = 0, False
        attempts, attached_at_start = {}, {}
        while stage_id < len(stages):
            stage = stages[stage_id]
            if "action_text" in stage.action_description and "english_action_text" in stage.action_description:
                logger.info(
                    f"RUNNING TASK: Stage {stage_id} action_text: {stage.action_description['action_text']}"
                )
                logger.info(
                    f"RUNNING TASK: Stage {stage_id} english_action_text: {stage.action_description['english_action_text']}"
                )
            # Special handling for reset and place stages
            if getattr(stage, "target", None) == "reset":
                extra_params = stage.extra_params
                arm = extra_params.get("arm", "right")
                next_stage = stages[stage_id + 1] if stage_id + 1 < len(stages) else None
                if (
                    next_stage is not None
                    and next_stage.action_type == "grasp"
                    and arm == last_pick_arm == next_stage.extra_params.get("arm")
                ):
                    logger.info(f"Skip reset: consecutive picks use {arm} arm")
                    stage_id += 1
                    continue
                plan_type = extra_params.get("plan_type", "AvoidObs")
                init_pose = self.robot.reset_pose[arm]
                curr_pose = self.robot.get_ee_pose(ee_type="gripper", id=arm)
                interp_pose = init_pose.copy()
                interp_pose[:3, 3] = curr_pose[:3, 3] + (init_pose[:3, 3] - curr_pose[:3, 3]) * 0.25
                task_success = self.robot.move_pose(
                    self.robot.reset_pose[arm],
                    type=plan_type,
                    arm=arm,
                    block=True,
                )
                if not task_success and plan_type == "AvoidObs":
                    logger.error(f"Stage {stage_id} reset planning failed")
                if not task_success:
                    break
                stage_id += 1
                continue
            store_name = f"stage_{stage_id}"
            if resumed:
                # Rolled back to this stage: its state is restored and its remaining candidates are still valid
                resumed = False
            else:
                if not stage.initialize_action_sequence_buffer(objects, self.robot):
                    task_success = False
                    logger.warning(f"Stage {stage_id} {stage.action_type} initialize action sequence buffer failed")
                    break
                attempts[stage_id] = 0
                attached_at_start[stage_id] = self.attached_obj_id
                self.robot.client.store_current_state(store_name)
                logger.info(f"Store state {store_name}")
            attempt_count = attempts[stage_id]
            stored_attached_obj_id = attached_at_start[stage_id]
            stage_success = False
            try_next_sequence = True
            while not stage_success and try_next_sequence and attempt_count < MAX_ATTEMPTIONS:
                # Execution
                stage_success = True
                attempt_count += 1
                action_sequence = stage.get_action_sequence()
                if not action_sequence:
                    # no more action sequence to try, fail this stage
                    stage_success = False
                    try_next_sequence = False
                    logger.info(f"Stage {stage_id} {stage.action_type} failed due to no more action sequence")
                    break
                for action in action_sequence:
                    if getattr(stage, "is_bimanual", False):
                        step_success, try_next_sequence = self.step_bimanual(
                            action, stage, objects, workspaces, stage_id, attempt_count, action_sequence.get_step_index()
                        )
                    else:
                        step_success, try_next_sequence = self.step(
                            action, stage, objects, workspaces, stage_id, attempt_count, action_sequence.get_step_index()
                        )
                    if not step_success:
                        # for first step fail, try next sequence. Otherwise, fail this stage
                        if try_next_sequence:
                            logger.warning(
                                f"Stage {stage_id} {stage.action_type} fail at first step, try next sequence"
                            )
                        else:
                            logger.warning(
                                f"Stage {stage_id} {stage.action_type} fail at step {action_sequence.get_step_index()}, fail this stage"
                            )
                        stage_success = False
                        break
                if not stage_success and try_next_sequence and not stage.action_sequence_buffer.empty():
                    self.attached_obj_id = stored_attached_obj_id
                    self.robot.client.playback(store_name)
                    logger.info(f"Playback state {store_name}")
                elif stage.checker_config:
                    for check_config in stage.checker_config:
                        checker_status = self.robot.client.get_checker_status(check_config).msg
                        logger.info(f"stage finish: {stage_id}, checker: {check_config}, status: {checker_status}")
                        if checker_status == "fail":
                            stage_success = False
                            try_next_sequence = True
                            if not stage.action_sequence_buffer.empty():
                                self.attached_obj_id = stored_attached_obj_id
                                self.robot.client.playback(store_name)
                                logger.info(f"Playback state {store_name}")
                            break

            attempts[stage_id] = attempt_count
            if not stage_success:
                # The parts of one pick / place share a group. A part that ran out of candidates (the object did not
                # follow the pick-up, the release failed its checker) retries from the nearest earlier part that
                # still has one, i.e. with another grasp / another placement, as the unsplit stage did.
                group = getattr(stage, "group", None)
                back = stage_id - 1
                while back >= 0 and group is not None and getattr(stages[back], "group", None) == group:
                    if not stages[back].action_sequence_buffer.empty() and attempts[back] < MAX_ATTEMPTIONS:
                        break
                    back -= 1
                if back >= 0 and group is not None and getattr(stages[back], "group", None) == group:
                    logger.warning(f"Stage {stage_id} {stage.action_type} failed, retry from stage {back} {stages[back].action_type}")
                    self.attached_obj_id = attached_at_start[back]
                    self.robot.client.playback(f"stage_{back}")
                    stage_id, resumed = back, True
                    continue

            task_success = task_success and stage_success
            if not task_success:
                logger.warning(f"Stage {stage_id} {stage.action_type} failed")
                break

            if stage.action_type == "grasp":
                last_pick_arm = stage.extra_params.get("arm", "right")
            time.sleep(1)
            stage_id += 1

        if not task_success:
            success = False
            logger.warning(f"Task {task_file} failed")
        time.sleep(2)
        self.robot.client.stop_recording()
        # reset object pose
        for obj_id in objects:
            if obj_id == "gripper":
                continue
            object_pose = {}
            object_pose["prim_path"] = objects[obj_id].prim_path
            object_pose["position"] = [99, 99, 0]
            object_pose["rotation"] = [0, 0, 0, 1]
            poses = []
            poses.append(object_pose)
            self.robot.client.set_object_pose(poses, [])

        step_id = -1
        fail_stage_step = [stage_id, step_id] if not success else [-1, -1]

        task_info.copy()
        self.robot.client.send_task_status(success, fail_stage_step)
        return success

    def run_saved_tasks(
        self,
        task_folder,
        camera_list,
        use_recording,
        workspaces,
        fps=10,
        render_semantic=False,
        origin_task_info={},
    ):
        """Replay every layout already in ``task_folder``; nothing is generated or deleted."""
        for task_file in sorted(glob.glob(os.path.join(task_folder, "*.json"))):
            self._run_task_file(
                task_file, use_recording, camera_list, fps, render_semantic, workspaces, origin_task_info
            )
        return True

    def run(
        self,
        task_generator,
        save_dir,
        recording_dir,
        camera_list,
        use_recording,
        workspaces,
        max_attempts,
        target_success=None,
        archive_dir=None,
        fps=10,
        render_semantic=False,
        origin_task_info={},
    ):
        """Collect episodes, generating each layout right before it is run.

        One layout per attempt, numbered from the existing recordings: a kept
        episode advances the number, a failed one is deleted (recording by the
        server, layout here) and its number reused. ``max_attempts`` bounds
        the work; ``target_success`` stops early.
        """
        os.makedirs(save_dir, exist_ok=True)
        task_name = origin_task_info["task"]
        next_index = self._next_layout_index(task_name, recording_dir, archive_dir)
        n_success = 0

        attempt = 0
        rejected = 0
        max_rejected = 10 * max_attempts  # a rejected layout costs ~10 s, a real attempt ~5 min
        while attempt < max_attempts:
            if target_success is not None and n_success >= target_success:
                logger.info(f"Reached {n_success}/{target_success} successful episodes, stopping early")
                break

            # The index advances only when the episode is kept, so successful
            # episodes are numbered contiguously: a failed attempt's recording
            # is removed synchronously by the server (handle_task_status) and
            # its layout below, freeing the number for the next attempt.
            logger.info(f"ATTEMPT_BEGIN {attempt+1}/{max_attempts}")
            task_file = self._generate_layout(task_generator, save_dir, task_name, next_index)
            if task_file is None:
                rejected += 1  # counts against the same budget, or the loop would spin forever
                if rejected >= max_rejected:
                    logger.warning(f"Gave up after {rejected} layouts that never ran")
                    break
                continue
            logger.info(f"Attempt {attempt+1}/{max_attempts} using layout {task_file}")

            success = self._run_task_file(
                task_file, use_recording, camera_list, fps, render_semantic, workspaces, origin_task_info
            )
            if success is None:
                os.remove(task_file)
                rejected += 1
                if rejected >= max_rejected:
                    logger.warning(f"Gave up after {rejected} layouts rejected before execution")
                    break
                continue
            attempt += 1
            if success:
                n_success += 1
                next_index += 1
                logger.info(">>>>>>>>>>>>>>>>>>>>  TASK SUCCESS ! <<<<<<<<<<<<<<<<<<<<")
            else:
                # The server has just deleted this attempt's recording
                # (handle_task_status); drop the layout with it so what stays
                # on disk is exactly the episodes that were kept.
                os.remove(task_file)

        logger.info(f"Collected {n_success} successful episode(s)")
        return True
