---
name: run-data-collection
description: >
  Launch a data_collection automated trajectory-collection task on a GPU host
  using the `geniesim autocollect run` CLI verb (which wraps
  scripts/run_data_collection.sh: docker run -d + in-container server+client).
  Trigger: when the user asks to "采集数据", "跑数据采集", "run data collection",
  "collect a task", "生产轨迹", "launch a tasks/<task>/g2/<task>.json", or wants
  to produce agibot-format episodes from a data_collection task template.
license: MPL-2.0
metadata:
  author: genie-sim
  version: "1.0"
---

## When to Use

- User wants to **produce trajectory episodes** from a `data_collection` task
  template on a workstation with Docker + an NVIDIA GPU.
- User references a task under `source/data_collection/tasks/`.

Do **not** use for:
- Running a benchmark/evaluation task → `run-benchmark`.
- Just listing/inspecting tasks → `geniesim autocollect list` directly.

## Critical Patterns

1. **`run` is host-orchestrated, not an in-container exec.** It shells out to
   `scripts/run_data_collection.sh`, which does `docker run -d` against
   `geniesim3-data-collection:latest` and the entrypoint launches **two**
   processes (Isaac Sim server + task client). Don't treat it like
   `benchmark run`.
2. **Collect the inputs first**: the task (basename / path / unique substring)
   and the run flags (`--headless`, `--no-record`, `--standalone`,
   `--container-name`). Use `--dry-run` to confirm resolution before launching.
3. **Prerequisites**: Docker + NVIDIA GPU; the image
   `registry.agibot.com/genie-sim/geniesim3-data-collection:latest` built/pulled;
   `geniesim_assets` pip-installed (editable) on the host — the CLI discovers it via `find_spec` and bind-mounts it at `/geniesim_assets`.
4. **Unattended works.** `run_data_collection.sh` grants uid 1234 access
   preferring `sudo setfacl`, degrading to `chmod -R a+rwX` when sudo isn't
   usable — so headless/background runs work without a tty. (The fallback
   world-writes the output dirs on the host.)
5. **Confirm before launching.** A real run spawns a GPU container, takes
   minutes, and writes **~1.5 GB per episode**. Ask before kicking it off.

## Workflow

### Step 1 — Resolve the task

```bash
geniesim autocollect list --robot=g2 <substr>     # discover
geniesim autocollect run <TASK> --headless --standalone --dry-run   # preview
```

`--dry-run` prints the resolved task path + the exact `run_data_collection.sh`
command without launching. Disambiguate if it reports multiple matches.

### Step 2 — Check prerequisites

```bash
docker images | grep geniesim3-data-collection      # image present?
nvidia-smi                                             # GPU free?
python3 -c "import importlib.util as u; print('geniesim_assets OK' if u.find_spec('geniesim_assets') else 'NOT INSTALLED')"   # assets pkg editable-installed?
```

### Step 3 — Launch

Interactive terminal (sudo can prompt):

```bash
pip install -e /path/to/geniesim_assets   # once on the host (editable)
geniesim autocollect run <TASK> --headless --standalone
```

Unattended / detached (no tty) — works directly (the script degrades to `chmod`
when sudo is unavailable; no PTY trick needed):

```bash
cd <repo-root>
PYTHONPATH=source/geniesim_cli/src \
  nohup python3 -m geniesim_cli autocollect run <TASK> --headless --standalone \
  > /tmp/dc-run.log 2>&1 &
```

(Use `python3 -m geniesim_cli …` if the `geniesim` console script isn't on PATH.)

### Step 4 — Monitor & verify

```bash
tail -f source/data_collection/logs/<TASK>/data_collector_server.log   # Isaac Sim startup
tail -f source/data_collection/logs/<TASK>/run_data_collection.log     # stages / TASK SUCCESS / job done
docker ps | grep data_collection                                       # container up
ls source/data_collection/recording_data/                              # episodes landing
```

Success looks like `job done` in the client log, the container auto-removed
(EXIT trap), and one `recording_data/[{TASK}_{INDEX}]/` dir per episode with
`aligned_joints*.h5`, `observations/videos/*`, `state.json`, `data_info.json`.

## Notes

- `--no-record` disables recording (drops `--publish_ros` + `--use_recording`);
  omit it to record.
- Recording produces ~1.5 GB/episode — watch disk; clean `recording_data/` after
  validating.
- The container is ephemeral; only the mounted `recording_data/`, `logs/`,
  `saved_task/` and the Isaac cache survive a run.
- Full task-config authoring: `source/data_collection/TASK_CONFIG_GUIDE.md`.
  Module reference: `source/data_collection/AGENTS.md`.
