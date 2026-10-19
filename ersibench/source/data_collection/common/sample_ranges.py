"""Training sample ranges of a recorded episode: which frames a training set should use, as consecutive pieces.

The episode on disk keeps every frame. Long stretches of the robot standing still (planning, waiting) are left out of
the pieces, and the LeRobot converter turns each piece into its own episode, so an action chunk never spans a cut: a
chunk that runs past the end of a piece is padded with its last frame, and the robot really did stand still there.

Rules (not DROID's: there is no filter in the training code, the pieces do the work):
  still frame    no arm joint moved more than `arm_still` rad since the previous frame, no gripper joint more than
                 `grip_still` rad, no object more than `obj_still` m (a block settling after release is motion)
  cut            a run of still frames of at least `min_still_s`, less `margin` frames at each end, so every piece
                 starts and ends at rest
  necessary      a still run within `around_s` of a gripper command or an attach / detach (grasp, release, contact)
  pause          keeps `pause_s` at each end instead of `margin`, and the run after the last motion keeps `pause_s`
                 at its start (task completion). Only that much: two events close together used to protect a whole
                 second of standing still
  piece          what lies between cuts (a cut shorter than `min_cut_s` is not worth splitting the episode for);
                 a piece in which nothing moves is left out

  python -m common.sample_ranges <episode dir> ... [--write] [--sweep]     (run from source/data_collection)
"""

import json
import sys
from pathlib import Path

import numpy as np

DEFAULTS = {"arm_still": 1e-4, "grip_still": 1e-3, "obj_still": 1e-3, "min_still_s": 0.3, "around_s": 1 / 3,
            "pause_s": 0.2, "min_cut_s": 0.2, "margin": 2}


def _runs(mask):
    """(start, end) of every run of True in a 1-D bool array, end exclusive."""
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.where(edges == 1)[0].tolist(), np.where(edges == -1)[0].tolist()))


def ranges(arm, grip, objects, events, fps=30, arm_still=1e-4, grip_still=1e-3, obj_still=1e-3, min_still_s=0.3,
           around_s=1 / 3, pause_s=0.2, min_cut_s=0.2, margin=2):
    """Pieces [start, end) of an episode. arm / grip: (T, n) joint positions, objects: (T, k, 3) positions,
    events: frame indices of gripper commands and attach changes."""
    arm, grip, objects = (np.asarray(x, dtype=float) for x in (arm, grip, objects))
    n = len(arm)

    def steps(x):
        d = np.abs(np.diff(x.reshape(n, -1), axis=0))
        return d.max(axis=1) if d.shape[1] else np.zeros(n - 1)

    obj = np.linalg.norm(np.diff(objects, axis=0), axis=2).max(axis=1) if objects.size else np.zeros(n - 1)
    moving = (steps(arm) >= arm_still) | (steps(grip) >= grip_still) | (obj >= obj_still)
    still = np.r_[not moving[0] if n > 1 else True, ~moving]
    near = np.zeros(n, bool)
    around = int(round(around_s * fps))
    for e in events:
        near[max(0, e - around):e + around + 1] = True
    last = int(np.where(moving)[0][-1]) + 2 if moving.any() else n  # first still frame after the last motion (moving[j]: frame j+1 moved)
    pause = int(round(pause_s * fps))
    cut = np.zeros(n, bool)
    for a, b in _runs(still):
        if b - a < min_still_s * fps:
            continue
        necessary = near[a:b].any()
        head = 0 if a == 0 else pause if necessary or a <= last < b else margin
        tail = 0 if b == n else pause if necessary else margin
        if b - a - head - tail >= max(1, min_cut_s * fps):
            cut[a + head:b - tail] = True
    return [[a, b] for a, b in _runs(~cut) if not still[a + 1:b].all()]


def episode_inputs(episode_dir):
    """ranges() inputs from an episode's state.json."""
    frames = json.loads((Path(episode_dir) / "state.json").read_text())["frames"]
    names = frames[0]["robot"]["joints"]["joint_name"]
    q = np.array([f["robot"]["joints"]["joint_position"] for f in frames])
    arm = q[:, [i for i, name in enumerate(names) if "arm" in name]]
    grip = q[:, [i for i, name in enumerate(names) if "gripper" in name]]
    ids = list(frames[0]["objects"])
    objects = np.array([[np.array(f["objects"][o]["pose"])[:3, 3] for o in ids] for f in frames]).reshape(len(frames), len(ids), 3)
    flags = [(f["frame_state"].get("set_gripper_open"), f["frame_state"].get("set_gripper_close"),
              f["frame_state"].get("if_attached")) for f in frames]
    events = [i for i in range(1, len(flags)) if flags[i] != flags[i - 1]]
    return arm, grip, objects, events


def write(episode_dir, **params):
    """Compute the pieces of one episode and write them to <episode>/sample_ranges.json. Returns the record."""
    params = {**DEFAULTS, **params}
    arm, grip, objects, events = episode_inputs(episode_dir)
    pieces = ranges(arm, grip, objects, events, **params)
    kept = sum(b - a for a, b in pieces)
    record = {"frames": len(arm), "kept": kept, "ranges": pieces, "params": params}
    (Path(episode_dir) / "sample_ranges.json").write_text(json.dumps(record))
    return record


def sweep(episode_dirs):
    """Share of frames left out per (arm_still, min_still_s), summed over the episodes."""
    inputs = [episode_inputs(d) for d in episode_dirs]
    total = sum(len(x[0]) for x in inputs)
    print(f"{len(inputs)} episodes, {total} frames; left out (pieces) per threshold:")
    print("arm_still \\ min_still  " + "  ".join(f"{s:>12}" for s in ("0.3 s", "0.5 s", "1.0 s")))
    for arm_still in (1e-5, 1e-4, 1e-3):
        cells = []
        for min_still_s in (0.3, 0.5, 1.0):
            res = [ranges(*x, **{**DEFAULTS, "arm_still": arm_still, "min_still_s": min_still_s}) for x in inputs]
            kept = sum(b - a for r in res for a, b in r)
            cells.append(f"{1 - kept / total:6.0%} ({sum(map(len, res)):3d})")
        print(f"{arm_still:>9g}              " + "  ".join(f"{c:>12}" for c in cells))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        sys.exit(__doc__)
    if "--sweep" in sys.argv:
        sweep(args)
    for d in args:
        if "--write" in sys.argv:
            r = write(d)
        else:
            arm, grip, objects, events = episode_inputs(d)
            pieces = ranges(arm, grip, objects, events, **DEFAULTS)
            r = {"frames": len(arm), "kept": sum(b - a for a, b in pieces), "ranges": pieces}
        print(f"{d}: keep {r['kept']}/{r['frames']} in {len(r['ranges'])} pieces {r['ranges']}")
