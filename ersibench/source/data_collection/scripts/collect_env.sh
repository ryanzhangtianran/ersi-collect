# Sourced by scripts/collect.sh (the entry point) and collect_worker.sh (one worker): where a task's data lives.
# Reads TASK, ERSIBENCH_DATA_ROOT, ERSIBENCH_LEROBOT_ROOT, ERSIBENCH_OOD_AXES, REC, LEROBOT_OUT from the environment, defaults below.
# shellcheck disable=SC2034  # everything here is for the scripts that source it

DC="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

TASK="${TASK:-pick_biggest_block}"
ERSIBENCH_DATA_ROOT="${ERSIBENCH_DATA_ROOT:-$HOME/NAS/ERSI}"
export TASK ERSIBENCH_DATA_ROOT

# OOD 采集（ERSIBENCH_OOD_AXES=light,camera_pose，见 TASK_CONFIG_GUIDE §generalization）归档到 <TASK>__<axes>/，
# 不能混进 <TASK>/：评测的 in-domain 场景就是从 saved_task/<TASK>/ 建的。
ARCHIVE_TASK="$TASK${ERSIBENCH_OOD_AXES:+__${ERSIBENCH_OOD_AXES//,/+}}"
REC="${REC:-$ERSIBENCH_DATA_ROOT/recording_data/$ARCHIVE_TASK}"   # 归档：[<TASK>_N]/
SAVED_NAS="$ERSIBENCH_DATA_ROOT/saved_task/$ARCHIVE_TASK"         # 归档：<TASK>_N.json（第 N 集的布局）
ERSIBENCH_LEROBOT_ROOT="${ERSIBENCH_LEROBOT_ROOT:-${HF_HOME:-$HOME/NAS/HuggingFace}/lerobot/ERSI}"
LEROBOT_OUT="${LEROBOT_OUT:-$ERSIBENCH_LEROBOT_ROOT/$ARCHIVE_TASK}"

# 放了这个文件，这个任务的每一路都在本轮结束、归档完后退出（scripts/collect.sh stop；下次 run 前清掉）
STOP_FILE="$DC/recording_data/$TASK/STOP"
