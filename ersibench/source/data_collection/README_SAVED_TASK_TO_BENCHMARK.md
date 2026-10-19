# 从 saved task 到 benchmark 评测场景

saved task 是 `TaskGenerator` 生成的布局 JSON（`saved_task/<task>/<task>_N.json`，一条 episode 一份）。
采集时 `scripts/collect.sh` 会把通过的 episode 的布局归档到 `$ERSIBENCH_DATA_ROOT/saved_task/<task>/`（默认
`~/NAS/ERSI`），评测场景就从这里生成。下面以 `pick_biggest_block` 为例：

1. 只生成布局，不采集轨迹；
2. 在 Isaac Sim 里逐个预览布局；
3. 把布局变成 benchmark 评测场景。

容器内路径以 `run_data_collection.sh` 的挂载为准：本目录挂在 `/geniesim/main/data_collection`，
资产挂在 `/geniesim_assets`。

## 1. 只生成布局

在采集容器（`data_collection_open_source`）里执行：

```bash
export SIM_ASSETS=/geniesim_assets
cd /geniesim/main/data_collection

/isaac-sim/python.sh scripts/generate_layout.py \
  --layout-only \
  --task-template tasks/pick_biggest_block/g2/pick_biggest_block.json \
  --output-dir /geniesim/main/data_collection/saved_task \
  --num-episodes 5
```

`--layout-only` 只调用 `TaskGenerator.generate()`，不连 gRPC、不做运动规划、不录制。输出目录按模板顶层的
`task` 字段命名：`saved_task/pick_biggest_block/pick_biggest_block_{0..4}.json`。

脚本不覆盖已有的任务目录：复用已有布局加 `--skip-generate`；要重新随机生成，先把原目录挪走或换一个
`--output-dir`（没有 `--force`）。`--output-task-name` 可以改输出子目录和文件名前缀。

## 2. 在 Isaac Sim 里预览布局

布局 JSON 不能用 Isaac Sim 的 `File -> Open` 打开，要由 `generate_layout.py` 解析后通过采集服务的 gRPC
加载背景、物体和位姿。不采集轨迹，也不跑 policy。

终端 1，在采集容器里启动 Isaac Sim 服务端，等它启动完成：

```bash
docker exec -it data_collection_open_source bash -lc '
  export SIM_ASSETS=/geniesim_assets
  cd /geniesim/main/data_collection
  /isaac-sim/python.sh scripts/data_collector_server.py --enable_physics
'
```

终端 2，加载第 0 份布局（去掉 `--instance-ids 0` 则依次加载全部，GUI 模式在布局之间等 Enter）：

```bash
docker exec -it data_collection_open_source bash -lc '
  export SIM_ASSETS=/geniesim_assets
  cd /geniesim/main/data_collection
  /isaac-sim/python.sh scripts/generate_layout.py \
    --gui --skip-generate --instance-ids 0 \
    --task-template tasks/pick_biggest_block/g2/pick_biggest_block.json \
    --output-dir /geniesim/main/data_collection/saved_task
'
```

`--headless` 加载后不等待直接退出；`--save-images` 把机器人相机画面存到
`saved_task/<task>/preview/`。关掉预览脚本后，终端 1 的服务可以继续用。

## 3. 生成 benchmark 评测场景

评测场景不提交、不手工转换：每个评测 sub_task 在
`source/geniesim_benchmark/src/geniesim_benchmark/benchmark/config/eval_specs/<sub_task>.json` 里声明用哪些
saved task、prompt 和判定；`scripts/run_benchmark.sh` 启动时调用 `scripts/build_eval_scenes.py`，从
`$ERSIBENCH_DATA_ROOT/saved_task` 生成到 `/tmp/ersibench/llm_task` 并挂进容器。手动生成：

```bash
python3 scripts/build_eval_scenes.py g2op_if_pick_biggest_block_ersi   # 仓库根目录执行
```

底层转换器是 `source/geniesim_benchmark/scripts/convert_saved_tasks_to_benchmark.py`
（完整说明在该文件开头的 docstring），它负责：

- 把采集世界坐标换到 benchmark 工作空间坐标（模板 origin）；
- USD 的 WXYZ 四元数转成 `scene_info.json` 用的 XYZW；
- 补偿 `Aligned.usda` wrapper 的 `entity` 旋转，保持物体最终姿态一致；
- 按目标物体的 y 坐标选 `left_gripper` 或 `right_gripper`，生成默认 checker（随后由 eval spec 的
  `stages` 覆盖）。

完整流程和评测见 `skills/eval-pi05/SKILL.md`。

## 数据流

```text
tasks/<task>/g2/<task>.json
  -> TaskGenerator.generate()                    (采集或 --layout-only)
  -> $ERSIBENCH_DATA_ROOT/saved_task/<task>/*.json
  -> build_eval_scenes.py + eval_specs/<sub_task>.json
       -> convert_saved_tasks_to_benchmark.py
  -> /tmp/ersibench/llm_task/<sub_task>/<id>/{scene.usda,scene_info.json,instructions.json,problems.json}
  -> run_benchmark.sh（挂载到容器里的 benchmark/config/llm_task）
```
