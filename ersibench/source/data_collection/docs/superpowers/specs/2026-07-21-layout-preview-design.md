# Layout generate + Isaac preview (no trajectory)

## Goal

Generate layouts via `client/layout/TaskGenerator`, load them into Isaac Sim over
the existing data_collection gRPC server, and preview — **without** running
`DataCollectionAgent.run()` trajectory collection / recording.

The preview script does not start or stop Isaac Sim. The operator starts
`scripts/data_collector_server.py` first; the script owns only layout generation
and preview loading through the existing gRPC client API.

## Modes

| Flag | Behavior |
|------|----------|
| `--gui` (default) | Load scene+objects, pause for interactive viewing; optional `--save-images` |
| `--headless` | Load, capture policy cameras, write PNGs, exit |
| `--layout-only` | Only run `TaskGenerator` (no Isaac / gRPC) |

## Flow

1. Read task template JSON → `TaskGenerator.generate_tasks()` → `--output-dir/<task>/*.json`
2. Connect `IsaacSimRpcRobot` to `--client-host` (default `localhost:50051`)
3. For each instance: `reset` → `DataCollectionAgent.generate_layout(json)` → wait `--settle-seconds` (default 5) for dropped objects → capture or wait
4. Never call `agent.run()` / `start_recording`

## Asset paths

Host generation writes absolute `$SIM_ASSETS/...` paths. The Isaac server joins
`$SIM_ASSETS` again, so before load the script rewrites `data_info_dir` /
`obj_path` to **paths relative to the host assets root** (works for Docker
`SIM_ASSETS=/geniesim_assets` and local).

## Out of scope

- Trajectory planning / recording
- Converting layouts into benchmark `llm_task` bundles
- Starting, stopping, or otherwise managing Docker/Isaac lifecycle

## Entry

`scripts/generate_layout.py`

## How to start the Isaac server

Preview **client** expects a gRPC server on `localhost:50051`.

### Option A — Docker GUI (recommended for `--gui`)

```bash
# Host: assets must be editable-installed
pip install -e /home/user/djy/geniesim_assets

cd /home/user/djy/genie_sim/source/data_collection
# or: geniesim autocollect up
./scripts/start_gui.sh run

# Enter container (second terminal)
./scripts/start_gui.sh exec

# Inside container — GUI server (no --headless)
export SIM_ASSETS=/geniesim_assets
python scripts/data_collector_server.py --enable_physics
# Optional if you also collect later: --enable_curobo --publish_ros
```

### Option B — Docker headless server (for `--headless --save-images`)

Same as A, but:

```bash
python scripts/data_collector_server.py --headless --enable_physics
```

### Option C — Local conda (no Docker)

```bash
conda activate <data_collect_or_geniesim_env>
cd /home/user/djy/genie_sim/source/data_collection
export SIM_ASSETS=/home/user/djy/geniesim_assets
python scripts/data_collector_server.py --enable_physics   # add --headless if needed
```

### Then run the preview client (host or same container)

```bash
cd /home/user/djy/genie_sim/source/data_collection
export SIM_ASSETS=/home/user/djy/geniesim_assets   # host path if client on host
export PYTHONPATH=$PWD:$PYTHONPATH

# GUI
python scripts/generate_layout.py --gui \
  --task-template tasks/pick_biggest_block/g2/pick_biggest_block.json \
  --output-dir /home/user/djy/genie_sim/output --num-episodes 2

# Headless + PNGs under output/<task>/preview/
python scripts/generate_layout.py --headless --save-images \
  --task-template tasks/pick_biggest_block/g2/pick_biggest_block.json \
  --output-dir /home/user/djy/genie_sim/output --num-episodes 2
```

If the client runs on the **host** against a **Docker** server (`--network=host`), keep
default asset rewriting (relative to `$SIM_ASSETS`). Use `--keep-absolute-assets`
only when client and server share the same filesystem paths.
