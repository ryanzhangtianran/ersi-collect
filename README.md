# ERSI-Collect

中文使用说明（安装、下载 assets、采集、自定义任务）：[USAGE.md](USAGE.md)

One Docker image that collects ERSI-Bench G2 tasks on a single GPU and converts the result to LeRobot v2.1.
No checkout, conda env or Isaac Sim install needed on the host.
Full procedure (preconditions, monitoring, troubleshooting, stop, verify): [`skills/collect/SKILL.md`](skills/collect/SKILL.md).

## Host requirements

- A recent NVIDIA driver (tested with 580; CUDA 12.8 inside), Docker ≥ 19.03, nvidia-container-toolkit. Any distro.
- GPU: RTX 40xx / L40 (sm_89) and RTX 50xx (sm_120) by default; others via `--build-arg CUDA_ARCHS=...` on the base.
- A geniesim_assets checkout (~47 GB), shipped separately from the image.

## Build (once), then run

```bash
docker build -f base.Dockerfile -t ersibench-collect-base .   # once, ~1 h: Isaac Sim 5.1 (nvcr.io) + ROS 2 + CUDA + cuRobo
docker build -t ersibench-collect:latest .                    # seconds; again after every git pull
docker run --rm ersibench-collect:latest                     # usage
docker run --rm -v $PWD/ersi_work:/work ersibench-collect:latest list   # tasks (built-in + yours), backgrounds

docker run --rm --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 \
    -v /path/to/geniesim_assets:/assets:ro -v /path/to/out:/output \
    ersibench-collect:latest pick_biggest_block 50
```

`--gpus` and `NVIDIA_VISIBLE_DEVICES` must name the same GPU. The container collects round after round until 50
episodes pass, then writes:

| path under /output | |
|---|---|
| `recording_data/<TASK>/[<TASK>_N]/` | passed episodes (raw agibot format) |
| `saved_task/<TASK>/<TASK>_N.json` | their layouts |
| `failed/<TASK>/` | failed episodes and layouts, for diagnosis |
| `lerobot/<TASK>/` | LeRobot v2.1 dataset of every passed episode, rebuilt at the end of each run |
| `logs/<TASK>/` | per-round server / client logs, `lerobot.log` |

Running again with a larger TARGET continues from what is already in `/output`.

The image ships one task, `pick_biggest_block`. Write your own with [`skills/create-task/SKILL.md`](skills/create-task/SKILL.md)
(scene + task json in a dir mounted at `/work`, checked with `check`, offline layouts with `layouts`, one smoke attempt)
and collect it with the same command plus `-v <work>:/work`.
Options (`-e`): `BATCH` attempts per round (20), `MAX_ROUNDS` (25), `ROUND_TIMEOUT` seconds, `ERSIBENCH_OOD_AXES`
(e.g. `light,camera_pose`, output under `<TASK>__light+camera_pose/`), `CONVERT=0` to skip LeRobot.
Shader cache across runs (saves minutes per start): `-v ersi-isaac-cache:/isaac-sim/.cache -v ersi-isaac-cc:/isaac-sim/.nv/ComputeCache`.

Mirrors for building inside mainland China (NVIDIA registry, apt, ROS, CUDA, pip) and the GPU list: the header of
`base.Dockerfile` and USAGE.md §2.2.

## Maintainers

```bash
./sync_from_ersibench.sh ~/Repos/ERSI-Bench <commit>   # refresh ersibench/ (omit the commit: working tree)
docker build -t ersibench-collect:latest .             # on ersibench-collect-base; the internal image works too:
                                                       #   --build-arg BASE=registry.agibot.com/genie-sim/geniesim3-data-collection:latest
```

`ersibench/` is a snapshot of the parts of ERSI-Bench the image needs (`source/data_collection`, the background
catalogue, the LeRobot converter). Change code in ERSI-Bench and sync; do not edit the snapshot here.
