# data_collection — Agent Development Guide

## Bounded Headless Test

`python scripts/test_arrange_blocks_50.py REPORT_DIR` runs 50 total attempts in
batches of five through `geniesim autocollect run`, preserving per-round logs,
metric results, and the final recorded frame. **No NAS access, archival, pruning,
or renumbering:** recordings stay in local `recording_data/arrange_blocks`.
Only metric-passing recordings qualify as successful; metric failures remain local
for inspection. The server still removes recordings when action execution fails.
`--attempts N` sets the total budget; `--resume-from OLD_REPORT_DIR` includes the
old report's attempts in that budget. `summary.json` separates current-phase
results from earlier attempts and updates completed attempts during each batch.
Startup errors or a one-hour round timeout stop the test. Do not run another
collection concurrently, or launch alongside a GPU-heavy benchmark.
The per-round task copy changes only the recording budget; the production task
template is left untouched. Defer formal archival until explicitly requested.
`RUN_REPORT_DIR` makes the worker (`collect_worker.sh`) snapshot logs/results before pruning each round.

Standalone, host-orchestrated **automated trajectory data-collection** module:
Isaac Sim 5.1 + cuRobo, a gRPC **client/server** pair that drives a robot
through declarative task templates and records full **agibot-format episodes**.
This is the open-source incarnation of the automated production code behind the
data-collection pipeline — episodes carry the same `aligned_joints*.h5` +
`observations/` + `data_info.json` layout used downstream.

Source: [source/data_collection/](.)
License: [MPL-2.0](LICENSE.txt)
Human docs: [README.md](README.md) (install/run) · [TASK_CONFIG_GUIDE.md](TASK_CONFIG_GUIDE.md) (task JSON schema)
CLI: `geniesim autocollect` — owned by
[geniesim_cli/commands/data_collection.py](../geniesim_cli/src/geniesim_cli/commands/data_collection.py)
Skills (Claude Code / opencode): [skills/](skills/)

> **Status** — [source/AGENTS.md](../AGENTS.md) lists this as a *legacy /
> non-geniesim* module: it ships its own Docker image + conda path and the
> **runtime** code is not a Python package. A thin **locator shim**
> ([`pyproject.toml`](pyproject.toml) + [`__init__.py`](__init__.py)) makes the
> distribution `geniesim-data-collection` `pip install -e`-able so the CLI can
> `find_spec("data_collection")` this dir (parity with `geniesim_benchmark`).
> The CLI **never imports the runtime tree** and **falls back** to a cwd /
> `$GENIESIM_REPO_ROOT` walk when the shim isn't installed. To fully revert:
> `pip uninstall geniesim-data-collection` and delete `pyproject.toml` +
> `__init__.py`.

> **Maintenance contract** — when you change a script's CLI flags, the task
> JSON schema, the gRPC interface, the `data_filter` rules, or the Docker
> image/entrypoint, **update this file in the same diff**. Agents read this as
> the source of truth.

---

## 1. The CLI surface

`geniesim autocollect <subcommand>`. If the `geniesim`
console script isn't on `$PATH`, substitute `python3 -m geniesim_cli
autocollect …` — same dispatch.

| Subcommand | What it does |
|---|---|
| `list [--robot=R] [--task=T] [SUBSTR]` | List task templates under `tasks/` |
| `tasks` | Distinct tasks (families) + counts |
| `robots` | Distinct robots (`g1` / `g2`) + counts |
| `run <TASK> [flags]` | Collect one task — **host-orchestrated** (see §2) |
| `build [--image=TAG] [docker args…]` | Build the image (`docker build -f dockerfile`; base `geniesim3:latest` must exist) |
| `up` / `into` / `down` `[--container-name=N]` | Interactive GUI container: create+enter / enter running / stop+remove (wraps `start_gui.sh`) |

`run` accepts **only** the flags below. Unlike `benchmark run`, the underlying
script is a strict parser and does **not** forward arbitrary `--key=value`:

| Flag | Effect |
|---|---|
| `--headless` | No GUI; required on unattended / no-X11 hosts |
| `--no-record` | Disable recording (drops `--publish_ros` + `--use_recording`) |
| `--standalone` | Logs to files only, no terminal echo |
| `--container-name=N` | Override container name (default `data_collection_open_source`) |
| `--dry-run` | Resolve the task + print the command, **don't** launch |

### `<TASK>` resolution

First hit wins: **literal path** (abs / relative to cwd / to the module dir) →
**exact basename** (stem, `.yaml`-style auto-suffix `.json`) → **unique
substring** against task stems. Ambiguous substring → error with candidates.

---

## 2. How an agent drives this module (read first)

**`autocollect run` is NOT the same shape as `benchmark run`.** `benchmark
run` execs a single `app.py` *inside* an already-running container.
`autocollect run` is a **host-side orchestrator**: it shells out to
[`scripts/run_data_collection.sh`](scripts/run_data_collection.sh), which does
`docker run -d` against this module's **own** image
(`registry.agibot.com/genie-sim/geniesim3-data-collection:latest`), and the
in-container entrypoint launches **two** processes (Isaac Sim server + task
client) over gRPC.

- The **agent runs on the host**; the checkout is bind-mounted into the
  container at `/geniesim/main/data_collection`, so host edits are live inside.
- **Prefer `run --headless --standalone`** for unattended runs: detached
  container + file logs, no X11. The interactive two-terminal flow (§5) is for
  human debugging and maps poorly onto an agent's non-persistent shell.
- **Outputs** land in `recording_data/[{TASK}_{INDEX}]/` — one dir per episode,
  **~1.5 GB each** (`camera/` raw mcap + `observations/videos/*` +
  `aligned_joints*.h5` + `state.json` + `data_info.json` with action labels).
  Per-run logs land in `logs/{TASK}/` (`data_collector_server.log`,
  `run_data_collection.log`, …).
- The container is **ephemeral** — `run_data_collection.sh` traps EXIT and
  `docker stop/rm`s it. State survives only in the mounted dirs above.

> ✅ **Unattended / no-tty.** `run_data_collection.sh` grants the container's
> uid 1234 access to the bind-mounted dirs. It **prefers** `sudo setfacl` when
> passwordless sudo is available and **degrades to `chmod -R a+rwX`** otherwise,
> so headless/background runs no longer hard-fail when sudo can't cache its
> tty-keyed credentials (`timestamp_type=tty`). Caveat: the fallback makes
> `recording_data/` / `saved_task/` world-writable on the host — tighten
> afterward if that matters.

> **`geniesim_assets` must be pip-installed (editable)** on the host — the CLI
> discovers it via `find_spec` (same as `geniesim docker`), bind-mounts it to
> `/geniesim_assets`, and the entrypoint editable-installs it + sets
> `SIM_ASSETS=/geniesim_assets` (from which it copies the cuRobo robot assets).

---

## 3. Architecture (client ↔ server)

Two processes, one container, gRPC between them:

| Side | Entry | Key flags | Internals |
|---|---|---|---|
| **server** | [`scripts/data_collector_server.py`](scripts/data_collector_server.py) | `--enable_physics --enable_curobo --publish_ros [--headless]` | `server/grpc_server.py`; `controllers/` (kinematics, parallel_gripper, ruckig); `motion_generator/` (cuRobo reacher); `recording/`; `ros_publisher/` (camera, tf, joints) |
| **client** | [`scripts/run_data_collection.py`](scripts/run_data_collection.py) | `--task_template <json> [--use_recording]` | `client/layout/` (layout/task gen) → `client/planner/` (grasp/place/insert/rotate/stage) → `client/agent/omniagent.py` → `client/robot/` (gRPC client) |

The `GetIKStatus` gRPC path returns one three-field record per requested pose:
`(success, joint_positions, link_poses)`. Every record uses the same joint-name
schema and vector length; failed IK candidates carry zero-valued joint
positions and must be discarded using `success`.

[`scripts/data_collection_entrypoint.sh`](scripts/data_collection_entrypoint.sh)
maps the high-level flags onto the two processes: `--headless` → server
`--headless`; recording on → server `--publish_ros` **and** client
`--use_recording`; `--task` → client `--task_template`. It starts the server,
waits ~10–15 s for Isaac Sim, starts the client, monitors both, and exits 0 on
client `job done`.

**`--publish_ros` is required for recording** — the ROS publishers are the
recording source. `--no-record` drops both it and `--use_recording`.

---

## 4. Tasks & config naming

### Failed-evaluation recollection bundles

`scripts/convert_eval_failure.py --result FILE --episode-id ID --output-dir DIR
--assets-root DIR [--runtime-assets-root /geniesim_assets]
[--robot-cfg G2_omnipicker_fixed_dual.json]` prepares one `pick_block_color`
expert collection task from a scored failure with an RSI episode snapshot.
It requires USD Python bindings, writes a new bundle, and never starts collection
or fine tuning. Old results without snapshots are rejected. Keep the output
under `source/data_collection/tasks/` for the existing container mount.

The generated `task_template.json` has `rsi_recollection.schema_version=1`,
relative `task_folder` and `scene_file`, source episode/axis/round, and explicit
`differences`. `run_data_collection.py --task_template FILE` detects this field,
loads the frozen saved task and robot pose directly, and skips `TaskGenerator`.
Frozen scene objects are not repositioned, and automatic head look-at is skipped;
captured camera poses/intrinsics are applied through the existing camera RPC.
Normal templates keep the existing generation behavior. The default collection
robot is the available fixed-dual G2 configuration, with `/genie` camera paths
mapped to `/G2`. The bundle records this difference. Physics solver internals,
joint-PD overrides and inference image noise are not replayed by the collector.
The usual expert grasp-file and data quality checks still apply.

See [README_EVAL_RECOLLECTION.md](README_EVAL_RECOLLECTION.md) for the offline
conversion command and artifact layout.

```
tasks/<collection>/<task>/<robot>/<name>.json
# e.g. tasks/pick_biggest_block/g2/pick_biggest_block.json
```

- `<robot>` is `g1` / `g2`; `<task>` is the task (type) — the dir above the
  robot, grouping its variant files. **Task dir names differ from the variant
  stems** (task `sort_fruit` vs files `sort_the_fruit_…`); `list <SUBSTR>`
  matches against `task/robot/name` so either works. Filter with `--task=<dir>`.
- Each JSON's top-level `task` is the episode/task name. Full schema (objects,
  scene, robot, stages, checkers, `recording_setting`, `task_metric`) is in
  [TASK_CONFIG_GUIDE.md](TASK_CONFIG_GUIDE.md), parsed by
  `client/layout/task_generate.py`.
- Data-quality checkers / filter rules live in
  [`common/data_filter/`](common/data_filter/) — see its
  [README](common/data_filter/README.md).

---

## 5. Path map & workflows

| Artifact | Location |
|---|---|
| Task templates | [`tasks/<task>/g2/<task>.json`](tasks/) |
| Server / client entries | [`scripts/data_collector_server.py`](scripts/), [`scripts/run_data_collection.py`](scripts/) |
| Layout generation + Isaac preview (no trajectory) | [`scripts/generate_layout.py`](scripts/generate_layout.py) · design: [`docs/superpowers/specs/2026-07-21-layout-preview-design.md`](docs/superpowers/specs/2026-07-21-layout-preview-design.md) |
| Orchestrators | [`scripts/run_data_collection.sh`](scripts/) (one-shot), [`scripts/start_gui.sh`](scripts/) (interactive), [`scripts/*entrypoint*.sh`](scripts/) |
| Robot / cuRobo configs | [`config/robot_cfg/G2_omnipicker_fixed_dual.json`](config/robot_cfg/), [`config/curobo/configs/`](config/curobo/) |
| aimdk protocol | [`common/aimdk/protocol/`](common/aimdk/) |
| Recording / conversion | [`server/recording/`](server/recording/) (`extract_ros_bag.py`, `sim_data_converter.py`) |
| Outputs | `recording_data/[{TASK}_{INDEX}]/` · logs `logs/{TASK}/` |
| Docker image | built from [`dockerfile`](dockerfile) → `registry.agibot.com/genie-sim/geniesim3-data-collection:latest` |

### Collect one task (recommended, unattended)

```bash
pip install -e /path/to/geniesim_assets   # once on the host (editable)
geniesim autocollect run sort_the_fruit_into_the_box_apple_g2 --headless --standalone
# preview without launching:
geniesim autocollect run apple_g2 --dry-run
# N *successful* episodes, archived, one worker per GPU, container restarted every BATCH attempts
# (entry point at the repo root; see skills/collect-episodes/SKILL.md):
../../scripts/collect.sh start TASK=my_task TARGET=50
```

Layouts are generated one per attempt right before it runs and kept only when
the episode succeeds, so `saved_task/<task>/` mirrors `recording_data/<task>/`
one-to-one and episode numbers are contiguous. `recording_setting.num_of_episode`
is the attempt budget; optional `num_of_success` stops early once that many
episodes were kept.

### Generate layouts and preview them in Isaac (no trajectory collection)

This is a manually orchestrated, two-terminal workflow. Both terminals must be
in `source/data_collection` with `SIM_ASSETS` pointing at the correct asset
root exported explicitly; the standalone server does not provide a package
fallback. Start the GUI server yourself in Terminal 1:

```bash
export SIM_ASSETS=/path/to/geniesim_assets
python scripts/data_collector_server.py --enable_physics
```

Then generate and load layouts from Terminal 2:

```bash
export SIM_ASSETS=/path/to/geniesim_assets
python scripts/generate_layout.py --gui \
  --task-template tasks/pick_biggest_block/g2/pick_biggest_block.json \
  --output-dir /path/to/genie_sim/output \
  --num-episodes 2
```

Only `--enable_physics` is needed on the server: this path does not collect
trajectories, record data, use cuRobo, or require a ROS publisher. The preview
script does not manage the server, Isaac Sim, or Docker lifecycle. Its control
flow calls `TaskGenerator.generate()` once per episode, atomically publishes
the completed task directory, then uses gRPC `reset()` / `generate_layout()`
and waits for inspection in the Isaac GUI. The target task directory must not
already exist; use `--skip-generate` to reuse it or choose a new
`--output-dir`. It never calls `agent.run()`, `start_recording()`, or the
trajectory planner.

The default gRPC endpoint is `localhost:50051`. A missing server fails fast
after the connection timeout and reports the exact `--enable_physics` command.
`--connect-timeout` is one shared total budget across the gRPC readiness
preflight, lazy client imports, and the actual `RpcClient` readiness check;
each stage receives only the remaining time. It does not set a deadline on the
subsequent `init_robot` RPC. Camera capture is strict: every requested camera
must return an image and all PNGs must stage successfully before the script
starts publishing the final image files one by one.
`generate_layout.py` accepts the complete CLI surface below:

| Flag | Effect |
|---|---|
| `--task-template PATH` | Source task JSON; defaults to the G2 apple sorting task shown above. |
| `--output-dir DIR` | Root for generated layout JSONs and preview images. |
| `--num-episodes N` | Override the template's `recording_setting.num_of_episode`. |
| `--skip-generate` | Reuse existing layouts under `output-dir/<task>/`. |
| `--instance-ids IDS` | Preview only comma-separated generated indices, for example `0,2`. |
| `--client-host HOST:PORT` | gRPC endpoint (default `localhost:50051`). |
| `--connect-timeout SECONDS` | Positive shared total budget for gRPC readiness, lazy client import, and the actual `RpcClient` readiness check (default 5 seconds); it does not bound the later `init_robot` RPC. |
| `--layout-only` | Generate/reuse layouts without connecting to Isaac. |
| `--gui` | Wait for Enter between layouts in the Isaac window; this is the default mode unless `--headless` or `--layout-only` is set. |
| `--headless` | Load each scene without waiting and save resolved camera images. |
| `--save-images` | Save resolved robot camera PNGs (also implied by `--headless`). |
| `--cameras NAMES` | Comma-separated camera aliases to capture (default `head,left_hand,right_hand`). |
| `--keep-absolute-assets` | Keep absolute asset paths instead of rewriting them relative to `SIM_ASSETS`. |

### Interactive (human debugging — two terminals)

```bash
pip install -e /path/to/geniesim_assets   # once on the host (editable)
./scripts/start_gui.sh run my_container          # Terminal A: create container
./scripts/start_gui.sh exec my_container         # Terminal B: enter; then inside:
python scripts/data_collector_server.py --enable_physics --enable_curobo --publish_ros
# Terminal C (same container):
python scripts/run_data_collection.py --task_template tasks/.../<name>.json --use_recording
```

---

## 6. Environment variables

| Var | Effect |
|---|---|
| `GENIESIM_ASSETS_SRC` / `SIM_ASSETS` | CLI auto-discovers the editable-installed `geniesim_assets` (`find_spec`) → `GENIESIM_ASSETS_SRC`; launch scripts bind-mount it to `/geniesim_assets`; the entrypoint sets `SIM_ASSETS=/geniesim_assets` and copies cuRobo assets from there. No manual export needed (just `pip install -e geniesim_assets`). |
| `GENIESIM_REPO_ROOT` | Override repo-root detection used by the CLI to locate `source/data_collection`. |
| `GENIESIM_RECORDING_ROOT` | Host dir bind-mounted as the container's `recording_data/` (default: the repo's own `recording_data/`). Record on local disk; a NAS is too slow for the raw mcap stream. |
| `GENIESIM_RECORDING_ARCHIVE` | Host dir of already-archived episodes, mounted read-only at `recording_archive/`; only used to continue episode numbering. |
| `TORCH_CUDA_ARCH_LIST` | cuRobo build arch (`dockerfile` sets `12.0` = RTX 5090 / Blackwell; the fused LBFGS kernel is disabled for it in `config/curobo/configs/task/*.yml`). |
| ROS (`ROS_DISTRO=jazzy`, `RMW_IMPLEMENTATION`, `LD_LIBRARY_PATH` → isaacsim ros2 bridge) | Set by the entrypoint / README's local-deploy steps. |

---

## 7. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `sudo: a terminal is required` then early exit | Handled — `run_data_collection.sh` now falls back to `chmod` when sudo is unusable (§2). If you still hit it, your copy predates that fix. |
| CLI errors `geniesim_assets is not pip-installed (editable)` | Run `pip install -e /path/to/geniesim_assets` on the host first (the CLI discovers it via `find_spec`). |
| `recording_data/` stays empty | Recording needs `--publish_ros` (server) **and** `--use_recording` (client); via the CLI just **don't** pass `--no-record`. |
| cuRobo install/import fails | GPU arch mismatch — image is built for `TORCH_CUDA_ARCH_LIST=8.9` (4090D); 50-series (SM_120) may be unsupported by cuRobo. |
| `Ambiguous task 'X'` | Substring matched ≥2 task stems — use a longer substring or full basename. |
| Container left running after a crash | `run_data_collection.sh` traps EXIT to clean up; if killed hard, `docker rm -f data_collection_open_source`. |

---

## 8. Do not

- Don't pin a host/secret inside a task JSON — keep them runtime args.
- Don't bypass `data_filter` quality checks when producing deliverable data.
- Don't change the gRPC interface on one side only — `server/grpc_server.py`
  and `client/robot/client.py` must stay in sync.
- Don't assume cuRobo builds on any GPU — the image targets 4090D (SM 8.9).
- Don't treat `run` as an in-container exec like `benchmark run` — it brings up
  its own container from the host (§2).

---

## 9. Output format

### Online motion completion

Shared completion checks evaluate `goal_offset` from the pre-action snapshot,
using the goal frame or robot base frame to match cuRobo. Missing required
state fails the check. Pick's post-close lift also verifies that the selected
object follows the expected gripper displacement, including x/y lifts and
the non-pregrasp path. Position/rotation tolerances are capped at one third
of the requested nonzero displacement/rotation so short motions cannot pass
without moving. Failed checks use the existing bounded candidate retry and
scene playback; task JSON metrics remain the final acceptance gate.
`MOTION_CHECK_FAILED` and `GRASP_FOLLOW_FAILED` distinguish endpoint and object
following errors. Explicit `skip_check` and injected pick errors retain their
existing bypass behavior. No new task configuration is required.

CPU regression check (NumPy and SciPy, no Isaac or GPU):
`python -m unittest discover -s source/data_collection/tests -p test_completion.py -v`

### Planner recovery

`Simple` actions first check the exact target, then use deterministic tool-frame
rotation alternatives (up to 8 degrees). Stage/action `extra_params` may set
`ik_fallback_rotation_deg` (finite, non-negative; zero disables the alternatives).
The target position is never perturbed. Insert, rotate, path-constrained,
offset and current-pose actions never perturb the target. Exhausted IK returns
failure without issuing motion. Reset failures stop the episode; collision-aware
reset no longer falls back to unchecked Simple motion.

cuRobo batch planning tries direct optimization with 70% of the configured
timeout, then graph-seeded optimization only on TRAJOPT/FINETUNE failure, valid
queries and unconstrained paths. Graph gets the remaining time and at most four
attempts, with one graph/trajopt seed as required by cuRobo 0.7.6 batch mode.
Timing is checked between cuRobo iterations, not a hard interruption deadline.
Constrained paths use the full budget without graph fallback. Failed plans clear
cached commands and collision masks are restored in `finally`.

CPU regression check (NumPy, SciPy, PyTorch required, no Isaac or GPU):
`python -m unittest discover -s source/data_collection/tests -p test_planning.py -v`

Episodes produced here are **agibot-format** (`.h5` + videos + `data_info.json`
with pick/place action labels). This tree is the data *generation* side.


## Coordinated dual-arm extension

The `bimanual_trajectory` gRPC service accepts paired left and right joint samples and applies each sample to the same articulation update. The `bimanual` task stage pairs nested `left` and `right` stages by action index; both sequences must have the same length. Keep the protocol source, runtime protobuf bindings, client robot helper, server hook, and task schema documentation synchronized when changing this interface.
