"""Training sample ranges of a recorded episode: which frames a training set should use, as consecutive pieces.

The episode on disk keeps every frame. Long stretches of the robot standing still (planning, waiting) are left out of
the pieces, and the LeRobot converter turns each piece into its own episode, so an action chunk never spans a cut: a
chunk that runs past the end of a piece is padded with its last frame, and the robot really did stand still there.

Rules (not DROID's: there is no filter in the training code, the pieces do the work):
  still frame    what looks still: no hand (end effector) faster than `hand_still` m/s, no gripper joint moved more
                 than `grip_still` rad, no object more than `obj_still` m since the previous frame (a block settling
                 after release is motion). Gripper motion only counts while the gripper is commanded to open or close
                 (`around_s` either side): an idle gripper's fingers jitter up to ~0.02 rad/frame, which kept up to
                 2 s of standing still. Joint thresholds kept the slow tails of every move (7 -> 0.4 cm/s over ten
                 frames, creeping at 1-2 cm/s), which look like standing still.
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

DEFAULTS = {"hand_still": 0.01, "grip_still": 1e-3, "obj_still": 1e-3, "min_still_s": 0.2, "around_s": 1 / 3,
            "pause_s": 0.1, "min_cut_s": 0.1, "margin": 2}


def _runs(mask):
    """(start, end) of every run of True in a 1-D bool array, end exclusive."""
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.where(edges == 1)[0].tolist(), np.where(edges == -1)[0].tolist()))


def ranges(hands, grip, objects, events, fps=30, hand_still=0.01, grip_still=1e-3, obj_still=1e-3, min_still_s=0.2,
           around_s=1 / 3, pause_s=0.1, min_cut_s=0.1, margin=2):
    """Pieces [start, end) of an episode. hands / objects: (T, k, 3) positions (m), grip: (T, n) gripper joint positions,
    events: frame indices of gripper commands and attach changes."""
    hands, grip, objects = (np.asarray(x, dtype=float) for x in (hands, grip, objects))
    n = len(hands)

    def speed(x):  # largest displacement per frame among the points of x (T, k, 3)
        return np.linalg.norm(np.diff(x, axis=0), axis=2).max(axis=1) if x.size else np.zeros(n - 1)

    gripper = np.abs(np.diff(grip.reshape(n, -1), axis=0))
    gripper = gripper.max(axis=1) if gripper.shape[1] else np.zeros(n - 1)
    moving = (speed(hands) >= hand_still / fps) | (gripper >= grip_still) | (speed(objects) >= obj_still)
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


def gate_gripper(grip, commanded, around):
    """Gripper positions with every change outside the commanded spans (+- `around` frames) held at the previous value,
    so only commanded opening / closing counts as gripper motion."""
    grip = np.array(grip, dtype=float)
    active = np.convolve(np.asarray(commanded, dtype=int), np.ones(2 * around + 1, dtype=int), "same") > 0
    for t in range(1, len(grip)):
        if not active[t]:
            grip[t] = grip[t - 1]
    return grip


def episode_inputs(episode_dir, fps=30):
    """ranges() inputs from an episode's state.json."""
    frames = json.loads((Path(episode_dir) / "state.json").read_text())["frames"]
    names = frames[0]["robot"]["joints"]["joint_name"]
    q = np.array([f["robot"]["joints"]["joint_position"] for f in frames])
    commanded = [bool(f["frame_state"].get("set_gripper_open") or f["frame_state"].get("set_gripper_close")) for f in frames]
    grip = gate_gripper(q[:, [i for i, name in enumerate(names) if "gripper" in name]], commanded,
                        int(round(DEFAULTS["around_s"] * fps)))
    sides = list(frames[0]["ee"])
    hands = np.array([[np.array(f["ee"][s]["pose"])[:3, 3] for s in sides] for f in frames]).reshape(len(frames), len(sides), 3)
    ids = list(frames[0]["objects"])
    objects = np.array([[np.array(f["objects"][o]["pose"])[:3, 3] for o in ids] for f in frames]).reshape(len(frames), len(ids), 3)
    flags = [(f["frame_state"].get("set_gripper_open"), f["frame_state"].get("set_gripper_close"),
              f["frame_state"].get("if_attached")) for f in frames]
    events = [i for i in range(1, len(flags)) if flags[i] != flags[i - 1]]
    return hands, grip, objects, events


def write(episode_dir, **params):
    """Compute the pieces of one episode and write them to <episode>/sample_ranges.json. Returns the record."""
    params = {**DEFAULTS, **params}
    hands, grip, objects, events = episode_inputs(episode_dir)
    pieces = ranges(hands, grip, objects, events, **params)
    kept = sum(b - a for a, b in pieces)
    record = {"frames": len(hands), "kept": kept, "ranges": pieces, "params": params}
    (Path(episode_dir) / "sample_ranges.json").write_text(json.dumps(record))
    return record


def sweep(episode_dirs):
    """Share of frames left out per (hand_still, min_still_s), summed over the episodes."""
    inputs = [episode_inputs(d) for d in episode_dirs]
    total = sum(len(x[0]) for x in inputs)
    print(f"{len(inputs)} episodes, {total} frames; left out (pieces) per threshold:")
    print("hand_still \\ min_still " + "  ".join(f"{s:>12}" for s in ("0.2 s", "0.3 s", "0.5 s")))
    for hand_still in (0.005, 0.01, 0.02):
        cells = []
        for min_still_s in (0.2, 0.3, 0.5):
            res = [ranges(*x, **{**DEFAULTS, "hand_still": hand_still, "min_still_s": min_still_s}) for x in inputs]
            kept = sum(b - a for r in res for a, b in r)
            cells.append(f"{1 - kept / total:6.0%} ({sum(map(len, res)):3d})")
        print(f"{hand_still * 100:>6.1f} cm/s           " + "  ".join(f"{c:>12}" for c in cells))


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
            hands, grip, objects, events = episode_inputs(d)
            pieces = ranges(hands, grip, objects, events, **DEFAULTS)
            r = {"frames": len(hands), "kept": sum(b - a for a, b in pieces), "ranges": pieces}
        print(f"{d}: keep {r['kept']}/{r['frames']} in {len(r['ranges'])} pieces {r['ranges']}")
