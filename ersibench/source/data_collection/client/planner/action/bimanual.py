"""Coordinated dual-arm stages.

The nested stages are planned independently, then their action sequences are
paired by index. Execution is delegated to one dual-arm joint trajectory RPC.
"""

from queue import Queue

from client.planner.action.stage import Stage
from common.base_utils.logger import logger


class BimanualActionSequence:
    def __init__(self, left_sequence, right_sequence):
        if len(left_sequence) != len(right_sequence):
            raise ValueError(
                "Bimanual stages require equal left/right action sequence lengths "
                f"(left={len(left_sequence)}, right={len(right_sequence)})"
            )
        self.left_sequence = left_sequence
        self.right_sequence = right_sequence
        self.action_index = -1

    def __iter__(self):
        return self

    def __next__(self):
        if self.action_index >= len(self.left_sequence) - 1:
            raise StopIteration
        self.action_index += 1
        self.left_sequence.action_index = self.action_index
        self.right_sequence.action_index = self.action_index
        return self.left_sequence[self.action_index], self.right_sequence[self.action_index]

    def get_step_index(self):
        return self.action_index

    def __len__(self):
        return len(self.left_sequence)


class BimanualStage(Stage):
    """Pair two normal stages and execute corresponding actions together."""

    def __init__(self, stage_config, objects):
        from client.planner.action.action_script import StageFactory

        self.is_bimanual = True
        self.left_config = stage_config["left"]
        self.right_config = stage_config["right"]
        base_config = dict(stage_config)
        base_config["active"] = stage_config.get("active", {"object_id": "gripper", "primitive": None})
        base_config["passive"] = stage_config.get("passive", {"object_id": "gripper", "primitive": None})
        super().__init__(base_config, objects)
        self.left_stage = StageFactory.create_stage(self.left_config, objects)
        self.right_stage = StageFactory.create_stage(self.right_config, objects)
        for stage in (self.left_stage, self.right_stage):  # each hand would draw its own via point and tear a closed chain
            stage.extra_params = {k: v for k, v in stage.extra_params.items() if k != "via_jitter"}
        self.action_sequence_buffer = Queue()

    def set_next_stage(self, stage):
        """Link the per-arm stages too, so their lookahead (e.g. a pick checking the lifts that follow) sees the chain."""
        super().set_next_stage(stage)
        for arm_stage, following in ((self.left_stage, getattr(stage, "left_stage", stage)), (self.right_stage, getattr(stage, "right_stage", stage))):
            arm_stage.set_next_stage(following)

    def initialize(self, objects, robot):
        self.left_stage.initialize(objects, robot)
        self.right_stage.initialize(objects, robot)
        self.initialized = True

    def initialize_action_sequence_buffer(self, objects, robot):
        self.left_stage.initialize_action_sequence_buffer(objects, robot)
        self.right_stage.initialize_action_sequence_buffer(objects, robot)
        self.action_sequence_buffer = Queue()
        left_sequences = []
        right_sequences = []
        while not self.left_stage.action_sequence_buffer.empty():
            left_sequences.append(self.left_stage.action_sequence_buffer.get())
        while not self.right_stage.action_sequence_buffer.empty():
            right_sequences.append(self.right_stage.action_sequence_buffer.get())
        for left_sequence, right_sequence in zip(left_sequences, right_sequences):
            try:
                self.action_sequence_buffer.put(BimanualActionSequence(left_sequence, right_sequence))
            except ValueError as exc:
                logger.warning("Skip incompatible bimanual candidate: %s", exc)
        return not self.action_sequence_buffer.empty()

    def get_action_sequence(self):
        self.active_action_sequence = (
            self.action_sequence_buffer.get() if not self.action_sequence_buffer.empty() else None
        )
        if self.active_action_sequence is not None:
            self.left_stage.active_action_sequence = self.active_action_sequence.left_sequence
            self.right_stage.active_action_sequence = self.active_action_sequence.right_sequence
        return self.active_action_sequence

