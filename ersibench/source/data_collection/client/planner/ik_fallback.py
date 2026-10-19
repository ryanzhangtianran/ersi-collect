"""Bounded, deterministic pose alternatives for unconstrained Simple motions."""

import numpy as np
from scipy.spatial.transform import Rotation


def reachable_pose(robot, target, arm, rotation_tolerance=8.0):
    """Return the nearest feasible alternative, or None; never mutate RNG state.

    Only rotation is perturbed (degrees, tool frame): moving a grasp/insert point changes the task.
    """
    if not np.isfinite(rotation_tolerance) or rotation_tolerance < 0:
        raise ValueError("IK fallback tolerance must be finite and non-negative")
    target = np.asarray(target, dtype=float)
    if target.shape != (4, 4) or not np.isfinite(target).all():
        raise ValueError("IK fallback requires a finite 4x4 pose")
    if not rotation_tolerance:
        return None
    candidates = []
    for fraction in (1 / 3, 2 / 3, 1):
        for axis in np.eye(3):
            for sign in (-1, 1):
                pose = target.copy()
                rotation = Rotation.from_rotvec(axis * np.deg2rad(rotation_tolerance * fraction * sign))
                pose[:3, :3] = target[:3, :3] @ rotation.as_matrix()
                candidates.append(pose)
    candidates = np.asarray(candidates)
    success, _ = robot.solve_ik(candidates, ee_type="gripper", arm=arm, type="Simple")
    mask = np.asarray(success, dtype=bool)
    if mask.shape != (len(candidates),):
        raise ValueError("IK response count does not match candidate count")
    candidates = candidates[mask]
    if not len(candidates):
        return None
    angles = Rotation.from_matrix(target[:3, :3].T @ candidates[:, :3, :3]).magnitude()
    return candidates[int(np.argmin(angles))]
