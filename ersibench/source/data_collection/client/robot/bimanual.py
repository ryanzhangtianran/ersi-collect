"""Synchronized dual-arm motion helpers for IsaacSimRpcRobot."""

import numpy as np
from scipy.linalg import expm, logm
from scipy.spatial.transform import Rotation, Slerp

from common.base_utils.logger import logger

# Two hands on one rigid object are a closed kinematic chain: both hands must undergo the SAME rigid motion, at the goal and
# at every point on the way. Independent per-hand targets / per-arm joint interpolation squeeze the object by centimetres
# (measured: 75 mm / 16 deg of relative hand drift in a "successful" pot carry), which a pinch grasp absorbs by slipping.
RIGID_TOLERANCE = (0.003, 1.0)  # metres, degrees the two hands' motions may disagree before the move is refused
WAYPOINT_STEP = (0.015, 3.0)  # metres, degrees between Cartesian waypoints
BRANCH_JUMP = 0.5  # rad between consecutive waypoint IK solutions: the solver switched arm configuration


def rigid_motion(start_pose, end_pose):
    """World transform D with end = D @ start."""
    return np.asarray(end_pose, dtype=float) @ np.linalg.inv(np.asarray(start_pose, dtype=float))


def rigid_mismatch(motion_a, motion_b, point):
    """How differently two rigid motions move a world point: (metres, degrees)."""
    point = np.append(np.asarray(point, dtype=float), 1.0)
    relative = motion_a[:3, :3].T @ motion_b[:3, :3]
    angle = np.degrees(np.arccos(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)))
    return float(np.linalg.norm((motion_a @ point - motion_b @ point)[:3])), float(angle)


def screw_fractions(motion, point):
    """Number of waypoints for a rigid motion, from how far it moves `point` and how much it turns."""
    travelled, _ = rigid_mismatch(motion, np.eye(4), point)
    _, turned = rigid_mismatch(motion, np.eye(4), point)
    count = max(1, int(np.ceil(max(travelled / WAYPOINT_STEP[0], turned / WAYPOINT_STEP[1]))))
    return np.linspace(0.0, 1.0, count + 1)[1:]


def screw_waypoints(motion, pose, fractions):
    """pose carried along the screw motion exp(s log D): the one-parameter rigid motion from identity to D."""
    twist = np.real(logm(motion))
    return [np.real(expm(fraction * twist)) @ np.asarray(pose, dtype=float) for fraction in fractions]


def pose_waypoints(start_pose, end_pose, fractions):
    """Straight line in position, slerp in orientation (a free hand; no object between the hands)."""
    start_pose, end_pose = np.asarray(start_pose, dtype=float), np.asarray(end_pose, dtype=float)
    slerp = Slerp([0.0, 1.0], Rotation.from_matrix([start_pose[:3, :3], end_pose[:3, :3]]))
    out = []
    for fraction in fractions:
        pose = np.eye(4)
        pose[:3, :3] = slerp(fraction).as_matrix()
        pose[:3, 3] = start_pose[:3, 3] + fraction * (end_pose[:3, 3] - start_pose[:3, 3])
        out.append(pose)
    return out


def min_jerk_retime(path):
    """Resample a uniformly spaced path (N+1 points) so progress follows s = 10t^3 - 15t^4 + 6t^5: zero velocity and
    acceleration at both ends, same geometry. Returns ceil(1.875 N) + 1 points, so the largest step equals the input spacing."""
    path = np.asarray(path, dtype=float)
    segments = len(path) - 1
    if segments < 2:
        return path
    tau = np.linspace(0.0, 1.0, int(np.ceil(1.875 * segments)) + 1)
    progress = (10 * tau**3 - 15 * tau**4 + 6 * tau**5) * segments
    low = np.minimum(progress.astype(int), segments - 1)
    return path[low] + (progress - low)[:, None] * (path[low + 1] - path[low])


def move_dual_pose(self, left_pose, right_pose, motion_type="Simple", max_joint_step=0.008, rigid=False):
    """Move both arms along one synchronized trajectory. max_joint_step: peak rad per physics step.
    rigid=True: the hands hold one object. Their targets must be the same rigid motion of their current poses, and the
    path is that motion's screw interpolation applied to both hands, so the grasp is never stretched on the way."""
    left_pose = np.asarray(left_pose)
    right_pose = np.asarray(right_pose)
    # Curobo returns the full c-space for the arm-specific model, including
    # joints locked to the other arm.  Keep only the active arm joints before
    # merging the two solutions into one trajectory point.
    active_names = self.joint_names
    left_names = [str(name) for name in active_names["left"]]
    right_names = [str(name) for name in active_names["right"]]

    def select_active_joints(info, names):
        returned_positions = np.asarray(info["joint_positions"], dtype=float)
        returned_names = np.asarray(info["joint_names"], dtype=object)
        if returned_positions.ndim == 1:
            returned_positions = returned_positions[np.newaxis, :]
            returned_names = returned_names[np.newaxis, :]
        selected = []
        for positions, names_for_pose in zip(returned_positions, returned_names):
            names_for_pose = [str(name) for name in names_for_pose]
            selected.append([positions[names_for_pose.index(name)] for name in names])
        return np.asarray(selected, dtype=float)

    joint_state = self.client.get_joint_positions()
    current = {state.name: state.position for state in joint_state.states}
    if any(name not in current for name in left_names + right_names):
        logger.warning("Bimanual trajectory IK returned an unknown joint")
        return False

    left_start = np.array([current[name] for name in left_names], dtype=float)
    right_start = np.array([current[name] for name in right_names], dtype=float)

    left_start_pose = np.asarray(self.get_ee_pose(id="left"), dtype=float)
    right_start_pose = np.asarray(self.get_ee_pose(id="right"), dtype=float)
    left_waypoints, right_waypoints = [left_pose], [right_pose]
    if rigid:
        left_motion, right_motion = rigid_motion(left_start_pose, left_pose), rigid_motion(right_start_pose, right_pose)
        between_hands = (left_start_pose[:3, 3] + right_start_pose[:3, 3]) / 2.0
        metres, degrees = rigid_mismatch(left_motion, right_motion, between_hands)
        if metres > RIGID_TOLERANCE[0] or degrees > RIGID_TOLERANCE[1]:
            logger.error(
                "BIMANUAL_NOT_RIGID the two hand targets are not one rigid motion: they disagree by %.1f mm / %.2f deg, "
                "the held object would be stretched by that much", metres * 1000.0, degrees)
            return False
        fractions = screw_fractions(left_motion, between_hands)
        left_waypoints = screw_waypoints(left_motion, left_start_pose, fractions)
        right_waypoints = screw_waypoints(left_motion, right_start_pose, fractions)  # the SAME motion for both hands
    elif motion_type == "Straight":
        travelled = max(np.linalg.norm(left_pose[:3, 3] - left_start_pose[:3, 3]), np.linalg.norm(right_pose[:3, 3] - right_start_pose[:3, 3]))
        fractions = np.linspace(0.0, 1.0, max(1, int(np.ceil(travelled / WAYPOINT_STEP[0]))) + 1)[1:]
        left_waypoints = pose_waypoints(left_start_pose, left_pose, fractions)
        right_waypoints = pose_waypoints(right_start_pose, right_pose, fractions)
    ik_type = "Simple" if (rigid or motion_type == "Straight") else motion_type
    left_ok, left_info = self.solve_ik(np.asarray(left_waypoints), arm="left", type=ik_type)
    right_ok, right_info = self.solve_ik(np.asarray(right_waypoints), arm="right", type=ik_type)
    if not bool(np.all(left_ok)) or not bool(np.all(right_ok)):
        logger.warning(
            "Bimanual IK failed: left=%s right=%s left_target=%s right_target=%s",
            np.asarray(left_ok).tolist(), np.asarray(right_ok).tolist(),
            np.round(left_pose[:3, 3], 4).tolist(), np.round(right_pose[:3, 3], 4).tolist())
        return False
    left_targets = select_active_joints(left_info, left_names)
    right_targets = select_active_joints(right_info, right_names)
    for name, start_joints, targets in (("left", left_start, left_targets), ("right", right_start, right_targets)):
        jumps = np.abs(np.diff(np.vstack([start_joints, targets]), axis=0)).max(axis=1)
        if len(targets) > 1 and jumps.max() > BRANCH_JUMP:
            logger.warning("BIMANUAL_IK_BRANCH_JUMP %s arm: %.2f rad between waypoints %d and %d", name, jumps.max(), jumps.argmax(), jumps.argmax() + 1)
            return False

    # Dense joint-space polyline through the IK targets (one shared path parameter keeps the arms synchronized) ...
    path = [np.concatenate([left_start, right_start])]
    for left_target, right_target in zip(left_targets, right_targets):
        target = np.concatenate([left_target, right_target])
        sample_count = max(1, int(np.ceil(float(np.max(np.abs(target - path[-1]))) / max_joint_step)))
        path += [path[-1] + alpha * (target - path[-1]) for alpha in np.linspace(0.0, 1.0, sample_count + 1)[1:]]
    # ... re-timed with a minimum-jerk profile: played at constant speed it starts and stops with a velocity step, which
    # shakes a held object loose. max_joint_step is now the PEAK step per physics tick (the profile peaks at 1.875x its mean).
    trajectory = [
        {"left": list(zip(left_names, point[: len(left_names)])), "right": list(zip(right_names, point[len(left_names) :]))}
        for point in min_jerk_retime(np.asarray(path))[1:]
    ]
    response = self.client.set_bimanual_joint_trajectory(trajectory)
    if not response.success:
        logger.warning("Bimanual trajectory failed: %s", response.message)
    return bool(response.success)
