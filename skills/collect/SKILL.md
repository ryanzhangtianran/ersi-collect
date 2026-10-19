---
name: collect
description: >
  Collect N *passed* episodes of one ERSI G2 task on one GPU with the `ersibench-collect` Docker image
  (rounds of server + client inside one container, failed episodes moved aside, LeRobot v2.1 conversion at the end),
  on a machine that has only Docker, an NVIDIA driver and a geniesim_assets checkout. Trigger: "采数据", "采 N 条
  <task>", "补采", "转 LeRobot", "collect episodes", "用镜像采集", "ersibench-collect". Also covers rebuilding and
  shipping the image (sync from ERSI-Bench, docker build/save).
license: MPL-2.0
metadata:
  author: ersi
  version: "1.0"
inputs:
  - name: TASK
    desc: "pick_biggest_block (the one built-in task) or a task of your own in WORK (skills/create-task); `list` prints them"
    required: true
  - name: TARGET
    desc: Passed episodes wanted in total in /output (what is already there counts, so re-running tops up)
    required: false
    default: "10"
  - name: WORK
    desc: "Host dir with your own tasks (scenes/<scene>.json, tasks/<TASK>/g2/<TASK>.json), mounted at /work; needed for any task but pick_biggest_block"
    required: false
  - name: GPU
    desc: "One GPU index, given twice: --gpus device=<GPU> and -e NVIDIA_VISIBLE_DEVICES=<GPU>"
    required: true
  - name: ASSETS
    desc: Host path of a geniesim_assets checkout (has pyproject.toml), mounted read-only at /assets
    required: true
  - name: OUT
    desc: Host output dir, mounted at /output
    required: true
  - name: BATCH
    desc: Attempts per round (Isaac Sim restarts every round)
    required: false
    default: "20"
  - name: MAX_ROUNDS
    desc: Give up after this many rounds even if TARGET is not reached
    required: false
    default: "25"
  - name: ROUND_TIMEOUT
    desc: Seconds before a round is killed (Isaac/curobo hangs)
    required: false
    default: "600 + 360*BATCH"
  - name: ERSIBENCH_OOD_AXES
    desc: "Collect the template's generalization.ood tier for these RSIBench axes (light, camera_pose, robot_initial_pose, object_position, unseen_object; any subset); output under <TASK>__<axes>/ with , -> +"
    required: false
  - name: CONVERT
    desc: "0 = skip the LeRobot conversion at the end"
    required: false
    default: "1"
outputs:
  - desc: "OUT/recording_data/<TASK>/[<TASK>_N]/ (passed episodes), OUT/saved_task/<TASK>/<TASK>_N.json (their layouts), OUT/failed/<TASK>/, OUT/lerobot/<TASK>/ (LeRobot v2.1), OUT/logs/<TASK>/"
---

## Interface (one command)

```bash
docker run -d --name ersi_<TASK> --gpus device=<GPU> -e NVIDIA_VISIBLE_DEVICES=<GPU> \
    [-e BATCH=20] [-e MAX_ROUNDS=25] [-e ERSIBENCH_OOD_AXES=light] [-e CONVERT=0] \
    -v ersi-isaac-cache:/isaac-sim/.cache -v ersi-isaac-cc:/isaac-sim/.nv/ComputeCache \
    -v <ASSETS>:/assets:ro -v <OUT>:/output [-v <WORK>:/work] \
    ersibench-collect:latest <TASK> <TARGET>
docker logs -f ersi_<TASK>
```

- The positional arguments are `TASK` and `TARGET`; everything else is `-e`.
- The image ships one task, `pick_biggest_block`. Any other task comes from `-v <WORK>:/work`: write it with
  `skills/create-task/SKILL.md`, and pass the same mount on every run of it.
- `--gpus` and `NVIDIA_VISIBLE_DEVICES` must name the **same** GPU. The base image bakes in `NVIDIA_VISIBLE_DEVICES=all`,
  so `--gpus` alone exposes every GPU and Isaac Sim lands on GPU 0.
- The two named volumes keep the Isaac Sim shader cache between runs. They are optional, but without them every round
  recompiles shaders for a few minutes.
- Run detached (`-d --name`). The entrypoint is PID 1 and ignores SIGINT/SIGTERM: Ctrl+C on an attached run does nothing,
  and `docker stop` only kills it after its 10 s grace period. Stop with `docker rm -f` (see "Stop").
- Exit code 0 means TARGET was reached and, unless `CONVERT=0`, the LeRobot dataset was written.

## How it runs

`/usr/local/bin/ersibench-collect` (`entrypoint.sh` in this repo):

1. As root: creates `OUT/{recording_data,saved_task,failed,logs,lerobot}/…` world-writable, then drops to uid 1234.
2. Builds `/geniesim_assets` as symlinks into `/assets`, with `background/` replaced by the ERSI catalogue
   (`build_backgrounds.py` → `/tmp/ersi/background`). Links the collector's `recording_data`, `saved_task/<TASK>` and
   `recording_archive` to the matching `OUT` dirs.
3. Each round:
   - writes the task json with `num_of_episode = BATCH` and `num_of_success = TARGET − passed`;
   - runs `data_collection_entrypoint.sh --headless` (Isaac Sim server + client) under `timeout ROUND_TIMEOUT`;
   - moves every episode that did not pass, and its layout, to `OUT/failed/<TASK>/`.

   Stage failures are deleted by the server itself; only metric failures and episodes cut off by a timeout reach
   `failed/`.
4. When `passed ≥ TARGET` or the round budget is spent, it converts **all** passed episodes with the compose
   converter. The output is written to `lerobot/<TASK>.tmp` and swapped in only after the conversion succeeds.

Contract:
- An episode counts when it has `aligned_joints.h5` **and** `task_result.json: task_status == true`.
- Numbers only grow. The client numbers past everything in `recording_data/` and `failed/`, so a moved failure never
  frees its number.
- Gaps in the `[<TASK>_N]` sequence are normal; they are the failures.
- One container per GPU and per `OUT/<TASK>`. Two containers writing the same task dir would race on numbering;
  different tasks or different OUT dirs are fine.

## Preconditions (check before launching)

```bash
nvidia-smi --query-gpu=index,name,driver_version,memory.used --format=csv,noheader   # driver >= 570; GPU idle (~17 GB needed)
docker images | grep ersibench-collect                       # image built (USAGE §2.2) or loaded from a tar.zst
docker run --rm --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 --entrypoint nvidia-smi ersibench-collect:latest -L   # toolkit works: lists exactly one GPU
test -f <ASSETS>/pyproject.toml && echo ASSETS_OK
df -BG --output=avail <OUT> | tail -1                        # >= 80 GB: raw mcap reaches 10 GB per attempt before extraction
docker ps --format '{{.Names}}' | grep ersi_                 # nothing else collecting on this GPU / OUT
docker run --rm [-v <WORK>:/work] ersibench-collect:latest list   # TASK must be in the task list
docker run --rm [-v <WORK>:/work] ersibench-collect:latest check <TASK>   # own tasks: 2/2 ok
```

- GPU: Ada (RTX 40xx, L40) or Blackwell (RTX 50xx) only, because cuRobo is compiled for sm_89 / sm_120.
  On A100, 3090 or H100 the planner fails.
- Host distro does not matter (the image is Ubuntu 24.04; a 20.04 host works).

## Launch

```bash
O=/data/ersi; A=/data/geniesim_assets
docker run -d --name ersi_pick_biggest_block --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 \
    -v ersi-isaac-cache:/isaac-sim/.cache -v ersi-isaac-cc:/isaac-sim/.nv/ComputeCache \
    -v $A:/assets:ro -v $O:/output ersibench-collect:latest pick_biggest_block 50
```

Smoke test for a new machine or new image (~10 min):

```bash
docker run --rm ... -e BATCH=2 -e MAX_ROUNDS=1 ... <TASK> 1
```

Success is not guaranteed in 2 attempts. A run that reaches `Both services are running!` and ends with `round 1 done`
proves the setup.

Other cases:

| Need | Command |
|---|---|
| Top up an existing dataset | the same command with a larger TARGET; existing passed episodes count |
| Convert only (no collection) | TARGET ≤ the number already passed: the loop is skipped and only the conversion runs |
| OOD axis data | `-e ERSIBENCH_OOD_AXES=light,camera_pose`; output under `<TASK>__light+camera_pose/`, never mixed into `<TASK>/` |
| Several GPUs | one container per GPU, **each with its own task or its own OUT** (they do not share numbering) |

## Monitor

```bash
docker logs -f ersi_<TASK>                                   # [collect] round R: need n … / round R done: +g, d / TARGET
tail -3 <OUT>/logs/<TASK>/round_<R>/run_data_collection.log      # client: stages, "Task … failed", "Collected k successful"
tail -3 <OUT>/logs/<TASK>/round_<R>/data_collector_server.log    # Isaac Sim / planner
ls <OUT>/recording_data/<TASK> | wc -l                       # passed so far (failures are in failed/)
```

Timing: 2-3 min Isaac Sim start per round, plus 2-5 min per attempt. Per-attempt pass rate depends on the task.

| Symptom | Meaning | Action |
|---|---|---|
| exits within seconds with usage | wrong TASK, `/assets` without `pyproject.toml`, or `/output` not mounted | the message above the usage says which |
| `collection did not start, see …/round_1` | no output after < 60 s: GPU busy or not visible, Isaac Sim crashed at start | read `round_1/data_collector_server.log`; check the GPU flags |
| `round R exited 1` but `+g` > 0 | normal: the inner entrypoint reports the server stopping after `job done` | none |
| `Stage 1 transition failed due to no more action sequence` | planner found no path for that layout (a normal failure) | many in a row: task/GPU problem, see below |
| yield 0 for 2+ full rounds | config or hardware, not luck: e.g. wrong GPU arch (cuRobo), `plan got an exception: CUDA out of memory` | read the server log before changing anything |
| server log silent > 10 min, GPU 99 % | Isaac Sim hung | `ROUND_TIMEOUT` kills the round and the loop continues |
| `LeRobot conversion failed` | read `<OUT>/logs/<TASK>/lerobot.log` | the episodes are intact; rerun with the same TARGET to retry conversion only |

## Stop

```bash
docker rm -f ersi_<TASK>
```

This kills the container immediately. Passed episodes already written stay. A half-written episode of the interrupted
attempt stays in `recording_data/` without `aligned_joints.h5`: it does not count, and the next run moves it to
`failed/` at the end of its first round. Nothing needs cleaning by hand.

## Verify results

```bash
O=<OUT>; T=<TASK>
python3 - "$O" "$T" <<'EOF'
import json, pathlib, sys
o, t = pathlib.Path(sys.argv[1]), sys.argv[2]
eps = sorted((o / "recording_data" / t).glob("[[]*"))
ok = [p for p in eps if (p / "aligned_joints.h5").exists() and json.load(open(p / "task_result.json")).get("task_status")]
print(f"{len(ok)}/{len(eps)} passed, layouts {len(list((o / 'saved_task' / t).glob('*.json')))}, failed {len(list((o / 'failed' / t).glob('[[]*')))}")
info = o / "lerobot" / t / "meta" / "info.json"
if info.exists():
    i = json.load(open(info)); print("lerobot:", i["total_episodes"], "episodes,", i["total_frames"], "frames,", i["total_tasks"], "tasks")
EOF
```

Episode dir:
- `aligned_joints.h5`: joints and actions at 30 fps.
- `camera/<frame>/`: head and both wrist jpgs.
- `state.json`: per-frame object and end-effector poses.
- `task_result.json`: `task_status`, `metric_status`.
- `sample_ranges.json`: the frame ranges training uses (`common/sample_ranges.py`, written after extraction; every frame
  stays on disk). A still frame = arm joints < 1e-4 rad, gripper joints < 1e-3, objects < 1 mm since the previous frame.
  Cuts are still runs ≥ 0.3 s (2 rest frames kept at each end; no cut that would remove < 0.2 s); a run within 0.33 s
  of a gripper command or an attach / detach keeps 0.2 s at each end instead, and so does the pause after the last
  motion. ~35 % of frames left out, ~17 % of the kept ones still (the slowing-down ends of moves). The LeRobot converter makes one episode per range (no time jumps
  inside an episode, so action chunks stay continuous); `--whole-episodes` makes one per recording. Recompute or sweep
  thresholds on existing episodes: `python -m common.sample_ranges <episode dir>... [--sweep] [--write]`.
- `data_info.json`: carries `english_task_name`, the training instruction.

LeRobot v2.1:
- `data/chunk-000/episode_*.parquet`, `videos/chunk-000/observation.images.{top_head,hand_left,hand_right}/*.mp4`
  (HEVC, NVENC when available), `meta/{info.json,episodes.jsonl,episodes_stats.jsonl,tasks.jsonl}`.
- 159-dim `observation.state`, 40-dim `action`.

Several tasks into one dataset (one `task_index` per instruction, episodes shuffled):

```bash
docker run --rm --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 -v <OUT>:/output --user 1234:1234 \
    -e PYTHONPATH=/opt/ersibench/source/geniesim_benchmark/src --entrypoint python3 ersibench-collect:latest \
    -m geniesim_benchmark.dataset.convert.agibot_to_lerobot_compose \
    --agibot-dir /output/recording_data/<A> /output/recording_data/<B> --output-dir /output/lerobot/<name> --seed 0
```

The output dir must not exist or must be empty.

## Rebuild and ship the image (maintainers, on a machine with ERSI-Bench)

```bash
cd ~/Repos/ERSI-Collect
./sync_from_ersibench.sh ~/Repos/ERSI-Bench 9396cdd   # refresh ersibench/ from a commit (git archive); omit the ref for the
                                                      # working tree; records it in ersibench/SOURCE_COMMIT
docker build -f base.Dockerfile -t ersibench-collect-base .   # once (~1 h); mirrors / CUDA_ARCHS: see its header
docker build -t ersibench-collect:latest .       # the code layer, seconds
docker run --rm --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 -e BATCH=2 -e MAX_ROUNDS=1 \
    -v <ASSETS>:/assets:ro -v /tmp/ersi_smoke:/output ersibench-collect:latest pick_biggest_block 1   # smoke test
docker save ersibench-collect:latest | zstd -T0 > ersibench-collect.tar.zst
```

- `ersibench/` is a copy. Change code in ERSI-Bench, sync, rebuild; never edit the snapshot here.
- More GPU architectures: rebuild the base with `--build-arg CUDA_ARCHS="8.6;8.9;9.0;12.0"` (cuRobo has no PTX
  fallback: an unlisted GPU cannot plan). Check with `cuobjdump --list-elf` on `curobo/curobolib/*.so`.
- The base must keep `CUROBO_TORCH_CUDA_GRAPH_RESET=1` (without it every grasp fails with "changing goal type not
  supported in cuda graph mode") and warp-lang 1.17 / scipy 1.13.1, the versions the internal image collects with.

## Files

- `entrypoint.sh`: the collection loop and conversion; installed as `/usr/local/bin/ersibench-collect`.
- `base.Dockerfile`: nvcr.io Isaac Sim 5.1 + ROS 2 Jazzy (bag record, image_transport) + CUDA 12.8 + cuRobo v0.7.6 +
  the collector's pip deps; built once.
- `Dockerfile`: `BASE` (default `ersibench-collect-base`) plus `ersibench/` at `/opt/ersibench` and the entrypoint.
- `sync_from_ersibench.sh`: the list of what the image takes from ERSI-Bench.
- Inside the image:
  - `/opt/ersibench/source/data_collection` (also `/geniesim/main/data_collection`): tasks, scenes, collector;
  - `scripts/data_collection_entrypoint.sh`: one round;
  - `/opt/ersibench/source/geniesim_benchmark/src/geniesim_benchmark/dataset/convert/`: LeRobot converter.
- Task json fields: `ersibench/source/data_collection/TASK_CONFIG_GUIDE.md`. New tasks: `skills/create-task/SKILL.md`
  (files in WORK, no rebuild).
