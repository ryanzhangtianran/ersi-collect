#!/usr/bin/env python3
"""Compose several agibot task directories into ONE LeRobot v2.1 dataset.

The single-task converter (agibot_to_lerobot.py) writes task_index 0 everywhere and
only the first episode's task name. This wrapper reuses its episode conversion and
meta generation unchanged, then fixes the task bookkeeping:

  * one task_index per distinct ``english_task_name`` (from each episode's data_info.json)
  * episodes shuffled with ``--seed`` so tasks interleave in episode order
  * tasks.jsonl / episodes.jsonl / episodes_stats.jsonl / info.json carry the real tasks
  * video shapes in info.json read from the recorded frames (the converter hard-codes 400x640)
  * an episode with a sample_ranges.json (common/sample_ranges.py, written at extraction) becomes one LeRobot episode
    per range: long stretches of the robot standing still are left out without a time jump inside any episode, so
    action chunks stay continuous. --whole-episodes ignores the file.

Usage::

    python -m geniesim_benchmark.dataset.convert.agibot_to_lerobot_compose \
        --agibot-dir <task_A_dir> <task_B_dir> --output-dir <lerobot_dir> [--seed 0]
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from collections import Counter
from pathlib import Path

from geniesim_benchmark.dataset.convert.agibot_to_lerobot import (
    _load_episode_task_name,
    _require_ffmpeg,
    _require_heavy_deps,
    convert_episode,
    detect_episodes,
    generate_meta,
    load_lerobot_reference,
)

VIDEO_FRAME_FILES = {  # info.json video key -> jpg inside <episode>/camera/0/
    "observation.images.top_head": "head_color.jpg",
    "observation.images.hand_left": "hand_left_color.jpg",
    "observation.images.hand_right": "hand_right_color.jpg",
}


def _pieces_of(episode: Path, whole: bool = False) -> list:
    """(episode, (start, end)) per sample range of an episode; the whole episode without a sample_ranges.json."""
    ranges_file = episode / "sample_ranges.json"
    if whole or not ranges_file.exists():
        return [(episode, None)]
    return [(episode, (a, b)) for a, b in json.loads(ranges_file.read_text())["ranges"]]


def _episodes_of(agibot_dir: Path) -> list:
    agibot_dir = agibot_dir.resolve()
    if (agibot_dir / "aligned_joints.h5").exists():
        return [agibot_dir]
    return detect_episodes(agibot_dir)


def _rewrite_jsonl(path: Path, fn) -> None:
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    path.write_text("".join(json.dumps(fn(r), ensure_ascii=False) + "\n" for r in rows))


def compose(agibot_dirs: list, output_dir: Path, seed: int = 0, lerobot_ref_dir=None, fps: float = 30.0,
            whole_episodes: bool = False) -> dict:
    _require_ffmpeg()
    _require_heavy_deps()
    import pyarrow as pa
    import pyarrow.parquet as pq
    from PIL import Image

    output_dir = Path(output_dir).resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(f"output dir is not empty: {output_dir} (the converter does not clean it; remove it first)")

    episodes = [ep for d in agibot_dirs for ep in _episodes_of(Path(d))]
    if not episodes:
        raise RuntimeError("no episode directories found (each must contain aligned_joints.h5)")
    names = {ep: _load_episode_task_name(ep) for ep in episodes}
    missing = [str(ep) for ep, n in names.items() if not n]
    if missing:
        raise RuntimeError("episodes without english_task_name in data_info.json:\n  " + "\n  ".join(missing))
    tasks = sorted(set(names.values()))
    task_index = {t: i for i, t in enumerate(tasks)}
    random.Random(seed).shuffle(episodes)
    pieces = [piece for ep in episodes for piece in _pieces_of(ep, whole_episodes)]  # an episode's pieces stay adjacent

    print(f"{len(episodes)} episodes -> {len(pieces)} LeRobot episodes, {len(tasks)} tasks (seed={seed}):")
    for t in tasks:
        print(f"  [{task_index[t]}] {t}: {sum(1 for ep in episodes if names[ep] == t)} episodes")
    print("episode order:", " ".join(str(task_index[names[ep]]) for ep, _ in pieces))

    template = load_lerobot_reference(Path(lerobot_ref_dir).resolve()) if lerobot_ref_dir else None
    episode_info = []
    for i, (ep, frames) in enumerate(pieces):
        info = convert_episode(ep, output_dir, i, template, fps=fps, frames=frames)
        # the converter writes task_index 0; overwrite the column, keeping the schema metadata
        parquet = output_dir / "data" / f"chunk-{i // 1000:03d}" / f"episode_{i:06d}.parquet"
        table = pq.read_table(str(parquet))
        col = pa.array([task_index[names[ep]]] * table.num_rows, type=pa.int64())
        table = table.set_column(table.schema.get_field_index("task_index"), "task_index", col)
        pq.write_table(table, str(parquet), row_group_size=1000)
        episode_info.append(info)

    generate_meta(output_dir, episode_info, [ep for ep, _ in pieces], offsets=[f[0] if f else 0 for _, f in pieces])
    meta = output_dir / "meta"
    (meta / "tasks.jsonl").write_text(
        "".join(json.dumps({"task_index": i, "task": t}, ensure_ascii=False) + "\n" for t, i in task_index.items())
    )
    ep_task = {i: task_index[names[ep]] for i, (ep, _) in enumerate(pieces)}
    _rewrite_jsonl(meta / "episodes.jsonl", lambda r: {**r, "tasks": [tasks[ep_task[r["episode_index"]]]]})

    def _fix_stats(r):
        ti = ep_task[r["episode_index"]]
        r["stats"]["task_index"].update({"min": [ti], "max": [ti], "mean": [float(ti)], "std": [0.0]})
        return r

    _rewrite_jsonl(meta / "episodes_stats.jsonl", _fix_stats)
    info_path = meta / "info.json"
    info = json.loads(info_path.read_text())
    info["total_tasks"] = len(tasks)
    for key, jpg in VIDEO_FRAME_FILES.items():
        frame = episodes[0] / "camera" / "0" / jpg
        if key in info["features"] and frame.exists():
            w, h = Image.open(frame).size
            info["features"][key]["shape"] = [h, w, 3]
    info_path.write_text(json.dumps(info, indent=4, ensure_ascii=False))
    return {"episodes": episode_info, "tasks": tasks, "output_dir": str(output_dir)}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--agibot-dir", type=Path, nargs="+", required=True, help="task dirs (parents of episode dirs) or episode dirs")
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--seed", type=int, default=0, help="shuffle seed for the episode order")
    p.add_argument("--lerobot-ref-dir", type=Path, default=None)
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--whole-episodes", action="store_true", help="ignore sample_ranges.json: one LeRobot episode per recording")
    a = p.parse_args(argv)
    try:
        compose(a.agibot_dir, a.output_dir, seed=a.seed, lerobot_ref_dir=a.lerobot_ref_dir, fps=a.fps,
                whole_episodes=a.whole_episodes)
    except RuntimeError as exc:
        print(f"❌ {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
