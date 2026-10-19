"""Server-side atomic execution for paired dual-arm joint trajectories."""

import numpy as np

from isaacsim.core.utils.types import ArticulationAction
from server.command_enum import Command
from common.base_utils.logger import logger


SETTLE_TOLERANCE = 0.01  # rad
SETTLE_STEPS = 300  # physics steps to wait for the arms to arrive before giving up
BLOCKED_TOLERANCE = 0.05  # rad: further than this from the last point when the arms stop = the move failed


def _prepare(self):
    if getattr(self, "_bimanual_trajectory", None) is not None:
        return True
    trajectory = self.data.get("trajectory", [])
    articulation = self.ui_builder.articulation
    if articulation is None:
        self._bimanual_trajectory_error = "Robot articulation is not initialized"
        return False
    prepared = []
    known_indices = {}
    for index, point in enumerate(trajectory):
        names = []
        positions = []
        for arm_name in ("left", "right"):
            for name, position in point[arm_name]:
                if name in known_indices:
                    joint_index = known_indices[name]
                else:
                    joint_index = articulation.get_dof_index(name)
                    known_indices[name] = joint_index
                names.append(name)
                positions.append(position)
        if len(names) != len(set(names)):
            self._bimanual_trajectory_error = f"Duplicate joint in trajectory point {index}"
            return False
        prepared.append((np.asarray(positions, dtype=float), np.asarray([known_indices[name] for name in names])))
    self._bimanual_trajectory = prepared
    self._bimanual_trajectory_index = 0
    self._bimanual_settle_steps = 0
    return True


def _handle(self):
    self.data_to_send = None
    if not _prepare(self):
        self.data_to_send = f"ERROR: {self._bimanual_trajectory_error}"
        return
    if self._bimanual_trajectory_index >= len(self._bimanual_trajectory):
        # Every point has been commanded, which is not the same as reached: the drives lag, more so with a load in
        # both hands. Report completion once the joints sit on the last point (or after SETTLE_STEPS, and say so).
        positions, indices = self._bimanual_trajectory[-1]
        current = self.ui_builder.articulation.get_joint_positions()[indices]
        error = float(np.max(np.abs(current - positions)))
        previous, self._bimanual_last_positions = getattr(self, "_bimanual_last_positions", None), current
        # arrived, or stopped: under a load the drives hold a few hundredths of a rad of steady-state error
        still = previous is not None and float(np.max(np.abs(current - previous))) < 1e-4 and self._bimanual_settle_steps > 10
        if error > SETTLE_TOLERANCE and not still and self._bimanual_settle_steps < SETTLE_STEPS:
            return
        self._bimanual_last_positions = None
        # Stopped far from the commanded joints = an arm is held back (by the object between the hands, by an obstacle):
        # that is a failed move, not a finished one. Reporting it as done only moved the failure to a later, vaguer check.
        blocked = error > BLOCKED_TOLERANCE
        if blocked:
            worst = int(np.argmax(np.abs(current - positions)))
            logger.warning("BIMANUAL_BLOCKED stopped %.3f rad from the last trajectory point (joint index %d)", error, int(indices[worst]))
        self.data_to_send = f"BIMANUAL_BLOCKED {error:.3f} rad" if blocked else True
        self._bimanual_trajectory = None
        self._bimanual_trajectory_index = 0


def _physics_step(self):
    trajectory = getattr(self, "_bimanual_trajectory", None)
    if not trajectory:
        return
    if self._bimanual_trajectory_index >= len(trajectory):
        self._bimanual_settle_steps += 1
        return
    positions, indices = trajectory[self._bimanual_trajectory_index]
    action = ArticulationAction(joint_positions=positions, joint_indices=indices)
    self.ui_builder.articulation.apply_action(action)
    self._bimanual_trajectory_index += 1


def _merge_gripper_actions(actions):
    """Merge left/right gripper commands into one articulation action."""
    merged = {}
    for attribute in ("joint_positions", "joint_velocities"):
        values = [getattr(action, attribute, None) for action in actions if action is not None]
        values = [value for value in values if value is not None]
        if not values:
            continue
        result = [None] * len(values[0])
        for value in values:
            for index, item in enumerate(value):
                if item is not None:
                    result[index] = item
        merged[attribute] = result
    return ArticulationAction(**merged)


def _handle_gripper(self):
    """Start and wait for both grippers as one blocking command."""
    left_state = self.data.get("left_gripper_state", "")
    right_state = self.data.get("right_gripper_state", "")
    if not getattr(self, "_bimanual_gripper_pending", False):
        self.robot = self._init_grippers()
        actions = []
        if left_state:
            actions.append(self._get_gripper_action(left_state, False))
        if right_state:
            actions.append(self._get_gripper_action(right_state, True))
        if actions:
            self.robot.apply_action(_merge_gripper_actions(actions))
        self._bimanual_gripper_pending = True

    left_done = not left_state or self.gripper_L.is_reached
    right_done = not right_state or self.gripper_R.is_reached
    if not (left_done and right_done):
        return
    if left_state == "open" or right_state == "open":
        self.ui_builder.detach_objs()
        self.attach_states = {}
    self._bimanual_gripper_pending = False
    self.data_to_send = True


def install(command_controller_class):
    """Add the command and physics hooks without changing Isaac startup code."""
    if getattr(command_controller_class, "_bimanual_installed", False):
        return
    original_command_step = command_controller_class.on_command_step
    original_physics_step = command_controller_class.on_physics_step

    def on_command_step(self):
        with self.condition:
            if self.data_to_send is not None:
                return
            if self.Command == Command.BIMANUAL_TRAJECTORY:
                try:
                    _handle(self)
                except Exception as exc:
                    logger.error("Bimanual trajectory failed: %s", exc)
                    self.data_to_send = f"ERROR: {exc}"
                self.condition.notify_all()
                return
            if self.Command == Command.BIMANUAL_GRIPPER:
                try:
                    _handle_gripper(self)
                except Exception as exc:
                    logger.error("Bimanual gripper failed: %s", exc)
                    self.data_to_send = f"ERROR: {exc}"
                self.condition.notify_all()
                return
            original_command_step(self)

    def on_physics_step(self):
        _physics_step(self)
        original_physics_step(self)

    command_controller_class.on_command_step = on_command_step
    command_controller_class.on_physics_step = on_physics_step
    command_controller_class._bimanual_installed = True
