"""Scene json + task json.

A task file (tasks/<task>/g2/<cfg>.json) holds what the robot does: task, task_description, stages, task_metric, and
names its scene with "scene_file": "<name>" = scenes/<name>.json, which holds where it happens: scene (background),
origin, robot, objects, workspaces, generalization, recording_setting. load_task merges the two, the task's keys
winning, so a task may override any scene key for its own case. A file without scene_file is already whole (saved
layouts, uploaded full templates) and comes back as it is.
"""

import json
from pathlib import Path

SCENES = Path(__file__).resolve().parents[1] / "scenes"
TASK_KEYS = ("task", "task_description", "stages", "task_metric")


def load_task(path):
    task = json.loads(Path(path).read_text(encoding="utf-8"))
    name = task.pop("scene_file", None)
    if name is None:
        return task
    scene_path = SCENES / f"{name}.json"
    if not scene_path.is_file():
        raise FileNotFoundError(f"{path}: scene_file {name!r} -> {scene_path} does not exist")
    return {**json.loads(scene_path.read_text(encoding="utf-8")), **task}


if __name__ == "__main__":
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        whole = Path(tmp) / "whole.json"
        whole.write_text(json.dumps({"task": "t", "robot": {"arm": "dual"}}))
        assert load_task(whole) == {"task": "t", "robot": {"arm": "dual"}}
        SCENES, real = Path(tmp), SCENES
        (Path(tmp) / "s.json").write_text(json.dumps({"robot": {"arm": "dual"}, "objects": {"a": 1}}))
        split = Path(tmp) / "split.json"
        split.write_text(json.dumps({"scene_file": "s", "task": "t", "robot": {"arm": "left"}}))
        assert load_task(split) == {"robot": {"arm": "left"}, "objects": {"a": 1}, "task": "t"}  # the task wins
        split.write_text(json.dumps({"scene_file": "missing", "task": "t"}))
        try:
            load_task(split)
            raise AssertionError("a missing scene must fail loudly")
        except FileNotFoundError:
            pass
        SCENES = real
    print("ok")
