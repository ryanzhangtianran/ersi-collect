# Recollect a Failed Evaluation Scene

The two-tier RSI runner saves every episode's initial scene after reset and
before the policy's first action. Both successful and failed episodes retain
their snapshot. Videos and successful recovery trajectories keep their existing
recording behavior; snapshots do not enable full-episode trajectory recording.

```text
output/rsibench/agent/<experiment>/trials/round_XXX/
  episodes/attempt_N/<axis>/<identity_hash>/
    snapshot.json
    stage.usda
    scene.usda
    dependencies/
    outcome.json
  results/<axis>.json
```

`snapshot.json` includes the generated episode content, full task/runtime
configuration, instruction/target/hand, static scene metadata, RNG state,
measured initial robot joints, object and rigid-body poses, velocities, and
camera poses/intrinsics. Camera noise parameters, arm offsets and joint-PD
settings are explicit; sampled material/HDR changes are retained in USD
overrides. The USD files preserve external asset references instead of copying
all meshes per episode. Non-asset scene dependencies are copied into the archive.

`outcome.json` is written after evaluation completes. The raw axis results link
to the archive through `rsi_snapshot`, and paired metrics expose `snapshot`.
An interrupted episode may have a snapshot but no scored outcome. Existing
results produced before this change cannot be retroactively reconstructed.

## Prepare a Collection Task

Run from the repository root in an environment with USD Python bindings
(the installed `geniesim` conda environment provides them):

```bash
/home/admin01/miniconda3/envs/geniesim/bin/python \
  source/data_collection/scripts/convert_eval_failure.py \
  --result output/rsibench/agent/<experiment>/trials/round_XXX/results/reference.json \
  --episode-id 'table_task_1_g2_op/3/0/4' \
  --assets-root /home/admin01/djy/geniesim_assets \
  --output-dir source/data_collection/tasks/recollections/<unique_name>
```

Use an actually failed episode ID from the chosen result file. `--result` also
accepts a snapshot's `outcome.json`. Conversion rejects successful, unscored or
unidentified episodes, missing snapshots/assets/grasp labels, and an existing
output directory. It currently supports single-hand `pick_block_color` tasks.

The bundle contains `task_template.json`, one frozen task in `saved_tasks/`,
the copied snapshot, and the source result. The target, hand, actual initial
object poses, robot joints and camera parameters come from the snapshot.
`rsi_recollection.differences` describes collection-versus-evaluation differences.
No simulator, collection process, or training job is launched by conversion.

The existing entry accepts the resulting template:

```bash
python3 -m geniesim_cli autocollect run \
  source/data_collection/tasks/recollections/<unique_name>/task_template.json \
  --dry-run
```

The bundle must be inside `source/data_collection` to be visible to its standard
container. Actual expert collection later uses the existing `--headless
--standalone` workflow and its recording/data quality checks. Conversion itself
does not require or automatically perform that step.

## Replay Limits

The current collector uses `G2_omnipicker_fixed_dual.json`, whereas evaluation
uses `G2_omnipicker.json`. The adapter preserves matching joint targets and maps
camera paths from `/genie` to `/G2`; the robot model difference needs review before
using resulting data for training. The expert planner generates new actions and
opens both grippers. Scene objects retain their archived USD transforms, and
automatic target look-at is disabled for these frozen tasks.

The archive is a scene/initial-state record, not a resumable physics-engine
checkpoint. Future per-frame image noise, physics solver internals and joint-PD
overrides are not replayed by this first collection adapter. Large external
assets must stay available. Full Isaac recollection and trajectory quality have
not been validated by the offline tests.
