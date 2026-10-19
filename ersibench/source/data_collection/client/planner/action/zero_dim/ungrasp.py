"""Ungrasp: release the held object where the hand is, then back the empty hand away.

extra_params:
    arm, gripper_state (default "open"),
    retreat: [{"direction": [x, y, z], "distance": m}, ...] successive moves of the empty hand, in the frame of
        retreat_frame_object_id (the object placed onto; world axes when absent),
    pos_threshold (default 0.03).
"""

import numpy as np

from client.planner.action.stage import Action, ActionSequence, Stage
from common.base_utils.transform_utils import pose_difference


def retreat_poses(gripper_pose, moves, frame_rotation=np.eye(3)):
    """World gripper poses after each successive retreat move; the orientation does not change."""
    poses, pose = [], np.array(gripper_pose, dtype=float)
    for move in moves:
        direction = np.asarray(move.get("direction", [0, 0, 1]), dtype=float)
        pose = pose.copy()
        pose[:3, 3] += frame_rotation @ direction * move.get("distance", 0.02) / np.linalg.norm(direction)
        poses.append(pose)
    return poses


class UngraspStage(Stage):
    def select_pose(self, objects, robot):
        params = self.extra_params
        frame = params.get("retreat_frame_object_id")
        rotation = np.asarray(objects[frame].obj_pose, dtype=float)[:3, :3] if frame else np.eye(3)
        current = robot.get_ee_pose(id=params.get("arm", "right"))
        return [{"poses": retreat_poses(current, params.get("retreat", []), rotation)}]

    def generate_action_sequence(self, poses):
        # KeepClose is an injected failure: the hand backs away without letting go
        gripper_cmd = None if self.error_type == "KeepClose" else self.extra_params.get("gripper_state", "open")
        return ActionSequence([Action(None, gripper_cmd, np.eye(4), "Simple")]
                              + [Action(pose, None, np.eye(4), "Simple") for pose in poses])

    def check_completion(self, objects, robot=None):
        target_pose = self.active_action_sequence.get_current_action().grasp_pose
        if target_pose is None:  # the release itself: what it achieved is for the stage's json checkers to judge
            return True
        pos_diff, angle_diff = pose_difference(objects["gripper"].obj_pose, target_pose)
        return bool(pos_diff < self.extra_params.get("pos_threshold", 0.03) and angle_diff < 30)


if __name__ == "__main__":
    hand = np.eye(4)
    hand[:3, 3] = [0.5, 0.0, 0.9]
    turned = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)  # holder frame: its y axis is world -x
    first, second = retreat_poses(hand, [{"direction": [0, 2, 0], "distance": 0.02}, {"direction": [0, 1, 0], "distance": 0.1}], turned)
    assert np.allclose(first[:3, 3], [0.48, 0.0, 0.9]) and np.allclose(second[:3, 3], [0.38, 0.0, 0.9])
    assert np.allclose(second[:3, :3], np.eye(3)) and np.allclose(hand[:3, 3], [0.5, 0.0, 0.9])
    assert retreat_poses(hand, []) == []
    print("ok")
