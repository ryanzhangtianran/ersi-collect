#!/usr/bin/env bash
# 一路采集 worker：在一张 GPU 上一轮轮采 TASK，直到归档里有 TARGET 条**成功**的 episode。
# 只由入口 scripts/collect.sh 启动（它给每路设 WORKER_ID / GPUS / WORKERS），不要直接调用。
#
# 为什么要循环：num_of_episode 是尝试次数，失败的 episode 会被 server 删掉，
# 所以只能一轮轮补，每轮结束重新数磁盘上的成品。
#
# 每路自己的容器（data_collection_w<ID>，bridge 网络：独占 localhost:50051 和 ROS 话题）、GPU 和本地目录
# workers/w<ID>/{recording_data,saved_task,logs,background,isaac-sim,task.json}；共享的只有归档目录，编号靠 mkdir 预留。

set -euo pipefail

# shellcheck source=collect_env.sh
source "$(dirname -- "${BASH_SOURCE[0]}")/collect_env.sh"
# geniesim CLI and a python with json: default to whatever is on PATH (activate your venv first)
GENIESIM="${GENIESIM:-$(command -v geniesim || true)}"
PY="${PY:-$(command -v python3)}"

[ -n "${WORKER_ID:-}" ] || { echo "collect_worker.sh 由 scripts/collect.sh 启动（缺 WORKER_ID）" >&2; exit 2; }
WORKERS="${WORKERS:-1}"                               # 总路数：每路本轮只领 1/WORKERS 的缺口
# 同一任务的多个配置（tasks/<TASK>/g2/<cfg>.json，"task" 字段都等于 TASK），逐轮轮转；不给就用 <TASK>.json
CFGS="${CFGS:-$TASK}"
MY_DIR="$DC/tasks/${TASK}/g2"

# 录制必须在本地盘（mcap ≈ 480 MB/s，NAS 只有 28~111 MB/s，直录会让 recorder 收尾被
# SIGKILL）。每轮结束把提取完的 episode 归档到 $REC。归档目录只读挂进容器，让编号接着排。
WDIR="$DC/workers/w$WORKER_ID"
CONTAINER="${CONTAINER:-data_collection_w$WORKER_ID}"
REC_LOCAL="$WDIR/recording_data/$TASK"
SAVED_LOCAL="$WDIR/saved_task/$TASK"
LOGS="$WDIR/logs/$TASK"
export DC_WORKER_DIR="$WDIR" DC_GPUS="${GPUS:-device=$WORKER_ID}" DC_NETWORK="${DC_NETWORK:-bridge}"   # run_data_collection.sh
export GENIESIM_RECORDING_ROOT="$REC_LOCAL"
export GENIESIM_RECORDING_ARCHIVE="$REC"

TARGET="${TARGET:-10}"
MAX_ROUNDS="${MAX_ROUNDS:-25}"
# 一轮尝试几次 = 多少次尝试重建一次容器（冷启动约 2~3 分钟）。实测（2026-09-23，胶棒 60 次、handover）：成功率不随
# 一个容器里的尝试次数变化；以前越跑越慢（每次渲染 +15 ms/次尝试，第 20 次单次耗时约为第 1 次的 2~3 倍）是读相机内参时
# 每次新建渲染输出造成的，已修（ui_builder.get_camera）。修复后渲染耗时平稳，所以大 BATCH 只省冷启动，不再有代价。
BATCH="${BATCH:-20}"
# 一轮的超时：正常一次尝试 2~6 分钟，按 BATCH 放宽（Isaac/curobo 偶尔挂死，容器活着但没日志）
ROUND_TIMEOUT="${ROUND_TIMEOUT:-$((600 + BATCH * 360))}"
MIN_FREE_GB="${MIN_FREE_GB:-80}"                      # 单集原始 mcap 可达 10 GB
GUI="${GUI:-0}"          # 1 = 不加 --headless，弹 Isaac Sim 窗口（需要 DISPLAY）
HEADLESS=(--headless); [ "$GUI" = 1 ] && HEADLESS=()
REVIEW="${REVIEW:-0}"    # 1 = 每集后处理完就在终端问"接受？"，人工判定优先于指标；答案写在集目录 manual_review 里

TAG="collect w$WORKER_ID"
log() { printf '\033[36m[%s]\033[0m %s\n' "$TAG" "$*"; }
die() { die_msg="$*"; printf '\033[31m[%s] ✗ %s\033[0m\n' "$TAG" "$*" >&2; exit 1; }

# RUN_REPORT_DIR 给定时，任何退出（含 die）都写 result.json：脚本/agent 读这个，不用解析上面的中文日志。
# 失败标签数自 $RUN_REPORT_DIR/logs/，所以每次调用给一个新目录。
start_n=0; metric_failed=0; die_msg=""
write_result() {
    local rc=$?
    [ -n "${RUN_REPORT_DIR:-}" ] || return 0
    mkdir -p "$RUN_REPORT_DIR"
    "$PY" - "$RUN_REPORT_DIR" "$TASK" "$TARGET" "$start_n" "$(count_done 2>/dev/null || echo "$start_n")" \
        "$metric_failed" "$SECONDS" "$rc" "$die_msg" "$WORKER_ID" <<'EOF' || true
import glob, json, sys
out, task, target, start, done, metric_failed, secs, rc, msg, worker = sys.argv[1:]
logs = "".join(open(p, errors="replace").read() for p in glob.glob(out + "/logs/round_*/*.log"))
tags = ("IK_UNREACHABLE", "MOTION_CHECK_FAILED", "GRASP_FOLLOW_FAILED", "plan got an exception", "Reject layout", "Traceback")
json.dump({
    "worker": worker, "task": task, "target": int(target), "done": int(done), "gained": int(done) - int(start),
    "rounds": len(glob.glob(out + "/logs/round_*")), "attempts": logs.count("ATTEMPT_BEGIN"),
    "metric_failed": int(metric_failed), "elapsed_s": int(secs), "exit_code": int(rc), "error": msg or None,
    "log_tags": {t: logs.count(t) for t in tags},
    "log_tail": [l for l in logs.splitlines() if any(t in l for t in tags)][-30:],
}, open(out + "/result.json", "w"), ensure_ascii=False, indent=2)
EOF
}
trap write_result EXIT

# ---------------------------------------------------------------- 前置检查
[ -n "$GENIESIM" ] || die "geniesim CLI 不在 PATH 上：先激活装了 geniesim_cli 的环境，或 export GENIESIM=/path/to/geniesim"
[ -f "$MY_DIR/${TASK}.json" ] || die "找不到任务模板 $MY_DIR/${TASK}.json（新任务：复制最接近的模板改名，或 POST /tasks/<名>）"
mkdir -p "$REC" 2>/dev/null && [ -w "$REC" ] || die "归档目录不可写：$REC（NAS 挂了吗：findmnt /nas）"
# 目录不存在时 docker 会以 root 建挂载点，容器用户就写不进去了
mkdir -p "$REC_LOCAL"; chmod a+rwX "$REC_LOCAL" 2>/dev/null || true
[ -w "$REC_LOCAL" ] || die "本地录制目录不可写：$REC_LOCAL，重建：rmdir $REC_LOCAL && mkdir -m 777 $REC_LOCAL"
mkdir -p "$SAVED_NAS"
[ "$REVIEW" != 1 ] || ( : < /dev/tty ) 2>/dev/null || die "REVIEW=1 要在前台终端里跑（需要 /dev/tty 读答案）"
# 任务发现按文件名 stem 匹配，同名即歧义，每轮秒退
for cfg in $CFGS; do
    [ "$(find "$DC/tasks" -name "${cfg}.json" | wc -l)" -le 1 ] || die "配置名 '${cfg}' 不唯一：$(find "$DC/tasks" -name "${cfg}.json" | tr '\n' ' ')"
    [ -f "$MY_DIR/${cfg}.json" ] || die "找不到配置：$MY_DIR/${cfg}.json"
done

# 本任务可用的 episode（有 aligned_joints.h5 = 后处理完成，且 task_result.json 的 task_status 为 true：
# server 只删阶段失败的集，指标不过的集会留下并标 false）：本地 + 归档，按编号去重、
# 归档优先（先列、sort -s 保序）、数字序。目录名是上游格式 [<task>_N]，尾部数字兼容
# 撞名时 server 追加的后缀；正则收紧到编号，避免前缀相同的任务被误数。
passed() {
    local r; r=$(cat "$1/manual_review" 2>/dev/null)
    [ "$r" = accept ] && return 0
    [ "$r" = reject ] && return 1
    grep -q '"task_status": true' "$1/task_result.json" 2>/dev/null
}
# 人工判定：REVIEW=1 时后台轮询本地集，后处理完（有 task_result.json）就在终端问；轮末再补问漏掉的
ask_review() {   # $1 = episode 目录
    local b m ans
    b=$(basename "$1"); m=$(grep -o '"metric_status": "[^"]*"' "$1/task_result.json" 2>/dev/null)
    printf '\033[33m[review]\033[0m %s 完成（%s，%s）。接受这集？[y/n] ' "$b" "$m" "$(date -r "$1/task_result.json" +%H:%M)" > /dev/tty
    while read -r ans < /dev/tty; do   # 只认 y/n，回车或误输入重新问（sudo/其他提示可能抢走一次输入）
        case "$ans" in
            y|Y|yes) echo accept > "$1/manual_review"; return 0;;
            n|N|no)  echo reject > "$1/manual_review"; return 0;;
            *) printf '请输入 y 或 n：' > /dev/tty;;
        esac
    done
}
ask_pending() {   # 问所有还没判定的本地集（显式 return 0：没有待问的集时 && 链返回 1，set -e 会把脚本杀掉）
    local d
    for d in "$REC_LOCAL"/\[${TASK}_*; do
        [ -f "$d/task_result.json" ] && [ ! -f "$d/manual_review" ] && ask_review "$d"
    done
    return 0
}
review_loop() { while true; do ask_pending; sleep 5; done; }
list_done() {
    local d b
    for d in "$REC"/\[${TASK}_* "$REC_LOCAL"/\[${TASK}_*; do
        [ -d "$d" ] || continue
        b=$(basename "$d")
        [[ "$b" =~ ^\[${TASK}_([0-9]+)\]([0-9]*)$ ]] || continue
        [ -f "$d/aligned_joints.h5" ] && passed "$d" || continue
        printf '%s\t%s\t%s\n' "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]:-0}" "$d"
    done | sort -s -k1,1n -k2,2n | awk -F'\t' '!seen[$1 FS $2]++' | cut -f3-
}

# 归档：每集一个子进程并行搬（NFS 小文件是 close→COMMIT 延迟瓶颈，靠并发摊平），
# 先拷再删源（rsync 幂等，中断重跑即续传）。mcap 不搬、留本地。布局按 episode 逐个搬：
# 归档在后台跑，下一轮正往同一个 saved_task/<task>/ 写新布局，不能整目录同步。
archive_one() {   # $1 = 本地 episode 目录
    local d=$1 b lay
    b=$(basename "$d")
    [[ "$b" =~ ^\[${TASK}_([0-9]+)\]([0-9]*)$ ]] || return 0
    lay="$SAVED_LOCAL/${TASK}_${BASH_REMATCH[1]}.json"
    rsync -aW --inplace --no-g --exclude='*.mcap' "$d/" "$REC/$b/" || { log "✗ 归档失败：$b"; return 1; }
    find "$d" -mindepth 1 -not -name '*.mcap' -delete 2>/dev/null
    rmdir "$d" 2>/dev/null && log "  $b 归档完成" || log "  $b 归档完成（本地还留着 mcap 或删不掉）"
    [ -f "$lay" ] && { rsync -a "$lay" "$SAVED_NAS/" && rm -f "$lay"; } || log "  ⚠️  $b 没有对应布局"
}
# 轮末、归档前：删掉指标不过的本地集（及其布局），幸存集改用归档里最小的空闲编号。
# 编号靠在归档目录 mkdir 空的 [<task>_N] 预留：mkdir 是原子的（NFS 上也是），并行 worker
# 甚至别的机器同时归档也不会撞号；空目录没有 h5，不算成品，归档时被填满。
# 幸存集先挪进暂存区再逐个取号，免得新编号撞上本地还没改名的集。
reserve_index() {   # 打印一个刚预留的编号
    local n
    while true; do
        n=$(ls "$REC" 2>/dev/null | sed -n "s/^\[${TASK}_\([0-9]*\)\][0-9]*\$/\1/p" | sort -un \
            | awk 'BEGIN { n = 0 } $1 == n { n++ } $1 > n { exit } END { print n }')
        mkdir "$REC/[${TASK}_${n}]" 2>/dev/null && { echo "$n"; return 0; }
        [ -d "$REC/[${TASK}_${n}]" ] || return 1   # 不是被别人抢了，是写不进去
    done
}
prune_and_compact() {
    local d b i n stage="$REC_LOCAL/.renumber"
    rm -rf "$stage"; mkdir -p "$stage"
    while read -r d; do   # 数字序（sort -V）：先产出的集拿小编号
        [ -d "$d" ] || continue
        b=$(basename "$d"); [[ "$b" =~ ^\[${TASK}_([0-9]+)\]([0-9]*)$ ]] || continue; i=${BASH_REMATCH[1]}
        if ! passed "$d"; then
            log "  ✗ $b 指标不过（$(grep -o '"metric_status": "[^"]*"' "$d/task_result.json" 2>/dev/null)），删除"
            metric_failed=$((metric_failed + 1))
            rm -rf "$d" "$SAVED_LOCAL/${TASK}_${i}.json"; continue
        fi
        mv "$d" "$stage/$b"
        mv "$SAVED_LOCAL/${TASK}_${i}.json" "$stage/$b.json" 2>/dev/null || true
    done < <(printf '%s\n' "$REC_LOCAL"/\[${TASK}_* | sort -V)
    while read -r d; do
        [ -d "$d" ] || continue
        b=$(basename "$d"); [[ "$b" =~ ^\[${TASK}_([0-9]+)\] ]]; i=${BASH_REMATCH[1]}
        n=$(reserve_index) || die "归档目录里预留不了编号：$REC"
        mv "$d" "$REC_LOCAL/[${TASK}_${n}]"
        mv "$stage/$b.json" "$SAVED_LOCAL/${TASK}_${n}.json" 2>/dev/null || true
        if [ "$b" != "[${TASK}_${n}]" ]; then
            log "  $b → [${TASK}_${n}]"
            sed -i "s/\[${TASK}_${i}\]/[${TASK}_${n}]/g" "$REC_LOCAL/[${TASK}_${n}]"/{task_result.json,recording_info.json,metadata.yaml,extract.log} 2>/dev/null || true
        fi
    done < <(printf '%s\n' "$stage"/\[${TASK}_* | sort -V)
    rm -rf "$stage"
}
archive_done() {
    local d pids=() rc=0
    for d in "$REC_LOCAL"/\[${TASK}_*; do
        [ -d "$d" ] && [ -f "$d/aligned_joints.h5" ] || continue
        archive_one "$d" & pids+=($!)
    done
    for p in "${pids[@]:-}"; do [ -n "$p" ] && { wait "$p" || rc=1; }; done
    [ "$rc" -eq 0 ] || die "本轮归档有失败（见上面），中止"
}

# 归档后台跑、和下一轮重叠；同一时刻只一个，起新的前先等上一个。
archive_pid=""
wait_archive() {
    [ -n "$archive_pid" ] || return 0
    wait "$archive_pid" || die "上一轮归档失败，本地数据还在 $REC_LOCAL"
    archive_pid=""
}

count_done() { list_done | wc -l; }

free_gb() { df -BG --output=avail "$1" | tail -1 | tr -dc '0-9'; }

# num_of_episode = 本轮尝试上限，num_of_success = 够了就停
set_round_budget() {   # $1 = 配置 json, $2 = 尝试数, $3 = 目标数
    $PY - "$1" "$2" "$3" "$DC" <<'EOF'
import json, sys
p, attempts, wanted = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
sys.path.insert(0, sys.argv[4])
from common.task_file import load_task
d = load_task(p)  # written back whole (task + scene): the container reads this one file
rs = d.setdefault("recording_setting", {})
rs["num_of_episode"] = attempts
rs["num_of_success"] = wanted
json.dump(d, open(p, "w"), ensure_ascii=False, indent=4)
EOF
}

# ---------------------------------------------------------------- 采集循环
# 上次被立即停掉（stop --now）的运行会在本地留下集：合格的照常重新编号、归档，半截的和不合格的删掉
if ls -d "$REC_LOCAL"/\[${TASK}_* >/dev/null 2>&1; then
    log "本地有上次留下的集，先整理归档"
    prune_and_compact
    archive_done
fi
done_n=$(count_done); start_n=$done_n
log "已有可用 episode: $done_n / $TARGET"

for round in $(seq 1 "$MAX_ROUNDS"); do
    [ "$done_n" -ge "$TARGET" ] && break
    [ -e "$STOP_FILE" ] && { log "检测到 STOP，收尾退出"; break; }   # 由 scripts/collect.sh 放和清
    free=$(free_gb "$REC_LOCAL")
    [ "$free" -lt "$MIN_FREE_GB" ] && die "本地盘只剩 ${free}G，中止"

    need=$((TARGET - done_n))
    share=$(( (need + WORKERS - 1) / WORKERS ))
    set -- $CFGS; cfg=${!#}; [ "$#" -gt 1 ] && { i=$(( (round - 1) % $# + 1 )); cfg=${!i}; }
    # 共享的配置不改，改自己的副本，run_data_collection.sh 把它挂到原路径上
    mkdir -p "$WDIR"; cp "$MY_DIR/${cfg}.json" "$WDIR/task.json"
    set_round_budget "$WDIR/task.json" "$BATCH" "$share"
    log "第 $round 轮：配置 $cfg，还差 $need 条$([ "$WORKERS" = 1 ] || echo "（本路领 $share）")，本轮最多尝试 $BATCH 次（本地剩 ${free}G）"

    # 不要 Ctrl+C：后处理是异步子进程，中途杀掉整轮白做
    t0=$SECONDS
    review_pid=""
    [ "$REVIEW" = 1 ] && { ask_pending; review_loop & review_pid=$!; }   # 先把上次留下的问完，再起后台轮询
    # Isaac/curobo can hang with the container alive and no log output; without a cap one hang stalls the whole collection
    timeout -k 30 "$ROUND_TIMEOUT" "$GENIESIM" autocollect run "$cfg" "${HEADLESS[@]}" --standalone --container-name "$CONTAINER" || {
        rc=$?
        [ "$rc" = 124 ] && { log "⚠️  本轮超时 ${ROUND_TIMEOUT}s，杀掉容器继续"; docker rm -f "$CONTAINER" >/dev/null 2>&1 || true; } || log "⚠️  autocollect 退出码 $rc"
    }
    elapsed=$((SECONDS - t0))
    [ -n "$review_pid" ] && { kill "$review_pid" 2>/dev/null; wait "$review_pid" 2>/dev/null; }
    [ "$REVIEW" != 1 ] || ask_pending   # 补问：后台循环没来得及问的

    wait_archive
    if [ -n "${RUN_REPORT_DIR:-}" ]; then
        mkdir -p "$RUN_REPORT_DIR/logs/round_$round" "$RUN_REPORT_DIR/results"
        cp "$LOGS/"*.log "$RUN_REPORT_DIR/logs/round_$round/" 2>/dev/null || true   # 没起来就没有日志，别让 set -e 吞掉下面的诊断
        for d in "$REC_LOCAL"/\[${TASK}_*; do
            [ -f "$d/task_result.json" ] || continue
            mkdir -p "$RUN_REPORT_DIR/results/$(basename "$d")"
            cp "$d/task_result.json" "$RUN_REPORT_DIR/results/$(basename "$d")/"
        done
    fi
    prune_and_compact
    archive_done & archive_pid=$!
    new_done=$(count_done); gained=$((new_done - done_n)); done_n=$new_done
    log "第 $round 轮结束：本轮 $gained/$BATCH，累计 $done_n / $TARGET"
    # 光起容器就要 ~100s：不到 60s 就零产出退出 = 配置故障（任务名歧义、镜像缺失、GPU 被占）
    [ "$gained" -eq 0 ] && [ "$elapsed" -lt 60 ] && die "本轮 ${elapsed}s 就退出且无产出，采集没起来，看上面 autocollect 的报错"
done

wait_archive
[ "$done_n" -ge "$TARGET" ] || log "⚠️  达到轮次上限，只采到 $done_n / $TARGET 条"

# 本地只是中转，清掉（下次运行会重建）；还剩 episode 目录时保留给人看（归档失败/诊断 mcap）
if ls -d "$REC_LOCAL"/\[${TASK}_* >/dev/null 2>&1; then
    log "⚠️  本地还有未清理的 episode，保留 $REC_LOCAL"
else
    rm -rf "$REC_LOCAL" "$SAVED_LOCAL"
    rmdir "$WDIR/recording_data" "$WDIR/saved_task" "$WDIR" 2>/dev/null || true   # $WDIR 只在空时删（count 用的那个）
fi
log "完成：$(count_done) 条 → $REC（布局 $SAVED_NAS）"
