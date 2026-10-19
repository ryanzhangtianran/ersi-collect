#!/usr/bin/env bash
# ersibench-collect image entrypoint: collect TASK on this container's GPU until TARGET episodes pass, then convert
# them to LeRobot v2.1; plus list / check / layouts for writing new tasks (skills/create-task).
# Single-GPU, in-container counterpart of scripts/collect.sh + collect_worker.sh: rounds of
# data_collection_entrypoint.sh (server + client), each round tops up what is still missing.
set -euo pipefail

DC=/geniesim/main/data_collection
RUN='docker run --rm --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 -v <assets>:/assets:ro -v <out>:/output [-v <work>:/work]'
usage() {
    cat >&2 <<EOF
usage:
  $RUN ersibench-collect:latest TASK [TARGET=10]       collect until TARGET episodes pass, then convert
  docker run --rm [-v <work>:/work] ersibench-collect:latest list         tasks, scene jsons, backgrounds with a table
  docker run --rm [-v <work>:/work] ersibench-collect:latest check TASK   schema + scene file + stage sequence
  $RUN ersibench-collect:latest layouts TASK [N=10]    N layouts offline (no Isaac server) -> /output/layouts/TASK/
  docker run --rm -v <out>:/output ersibench-collect:latest video TASK   one mp4 per passed episode (left wrist |
                                                     head | right wrist) -> /output/videos/TASK/

  /work (optional) holds your own tasks: /work/scenes/<scene>.json, /work/tasks/<TASK>/g2/<TASK>.json; copied over
  the built-in ones at every start (yours win on a name clash).
  --gpus and NVIDIA_VISIBLE_DEVICES must name the same GPU (the base image bakes in NVIDIA_VISIBLE_DEVICES=all).
  Optional -e: BATCH (attempts per round, default 20), MAX_ROUNDS (25), ROUND_TIMEOUT (s),
               ERSIBENCH_OOD_AXES (e.g. light,camera_pose; output goes to <TASK>__<axes>/),
               CONVERT=0 (skip the LeRobot conversion).
  Optional cache (skips minutes of shader compiling on later runs):
      -v ersi-isaac-cache:/isaac-sim/.cache -v ersi-isaac-cc:/isaac-sim/.nv/ComputeCache

  Output: /output/recording_data/<TASK>/[<TASK>_N]/  passed episodes
          /output/saved_task/<TASK>/<TASK>_N.json    their layouts
          /output/failed/<TASK>/                     failed episodes (+ layouts), kept for diagnosis
          /output/lerobot/<TASK>/                    LeRobot v2.1 dataset of all passed episodes (rebuilt each run)
          /output/logs/<TASK>/round_<R>/             server / client logs

tasks: $(ls "$DC/tasks" | tr '\n' ' ')
EOF
    exit 2
}
die() { echo "$*" >&2; usage; }

# root only opens /output up (the host dir may not be writable by the container user), then drops to uid 1234
if [ "$(id -u)" = 0 ]; then
    if [ -d /output ]; then
        mkdir -p /output/{recording_data,saved_task,failed,logs,lerobot,layouts,videos}
        chmod a+rwX /output/{recording_data,saved_task,failed,logs,lerobot,layouts,videos}
    fi
    exec gosu 1234:1234 "$0" "$@"
fi

umask 000   # outputs belong to uid 1234: world-writable so the host user can read and delete them
log() { printf '\033[36m[collect]\033[0m %s\n' "$*"; }
if [ -d /work ]; then
    cp /work/scenes/*.json "$DC/scenes/" 2>/dev/null || true
    [ ! -d /work/tasks ] || cp -r /work/tasks/. "$DC/tasks/"
fi
need_task() { [ -f "$DC/tasks/$1/g2/$1.json" ] || die "no task '$1' (built-in or /work/tasks/$1/g2/$1.json)"; }
need_mounts() {
    [ -f /assets/pyproject.toml ] || die "mount a geniesim_assets checkout at /assets (no /assets/pyproject.toml)"
    [ -d /output ] || die "mount an output dir at /output"
    # /geniesim_assets = /assets with background/ swapped for the rendered ERSI catalogue (run_data_collection.sh does
    # the same with bind mounts; there is no mounting from inside a container)
    python3 /opt/ersibench/scripts/build_backgrounds.py --assets /assets --out /tmp/ersi/background >/dev/null
    for p in /assets/*; do [ "${p##*/}" = background ] || ln -sfn "$p" /geniesim_assets/; done
    ln -sfn /tmp/ersi/background /geniesim_assets/background
    export SIM_ASSETS=/geniesim_assets
}

case "${1:-}" in
    ""|-h|--help|help) usage ;;
    list)
        python3 - "$DC" /opt/ersibench/scripts/backgrounds.json <<'PY'
import glob, json, os, sys
dc, catalog = sys.argv[1:]
print("tasks:", sorted(os.path.basename(p)[:-5] for p in glob.glob(dc + "/tasks/*/g2/*.json")))
print("scene json:", sorted(os.path.basename(p)[:-5] for p in glob.glob(dc + "/scenes/*.json")))
b = json.load(open(catalog))
tab = lambda k: k in b and isinstance(b[k], dict) and ("table" in b[k] or tab(b[k].get("base")))
print("backgrounds with a table:", [k for k in b if tab(k) and not k.endswith("_at_origin")])
PY
        exit 0 ;;
    check)
        T="${2:-}"; need_task "$T"
        scene=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("scene_file", ""))' "$DC/tasks/$T/g2/$T.json")
        # Isaac's python has the collector's imports, so the stage sequence prints too
        exec /isaac-sim/python.sh /opt/ersibench/skills/collect-episodes/task_schema.py \
            ${scene:+"$DC/scenes/$scene.json"} "$DC/tasks/$T/g2/$T.json" ;;
    layouts)
        T="${2:-}"; need_task "$T"; need_mounts
        cd "$DC"
        rc=0; /isaac-sim/python.sh scripts/generate_layout.py --layout-only --num-episodes "${3:-10}" \
            --output-dir /output/layouts --task-template "tasks/$T/g2/$T.json" || rc=$?
        chmod -R a+rwX "/output/layouts/$T" 2>/dev/null || true   # generate_layout.py makes its dirs 0700
        exit $rc ;;
    video)
        T="${2:-}"; ls -d "/output/recording_data/$T"/\[* >/dev/null 2>&1 || die "no episodes in /output/recording_data/$T"
        mkdir -p "/output/videos/$T"
        for e in "/output/recording_data/$T"/\[*; do   # camera/<frame>/<cam>_color.jpg at 30 fps
            ( cd "$e/camera" && ffmpeg -hide_banner -loglevel error -y \
                -framerate 30 -i %d/hand_left_color.jpg -framerate 30 -i %d/head_color.jpg -framerate 30 -i %d/hand_right_color.jpg \
                -filter_complex "[0]scale=-2:528[l];[1]scale=-2:528[h];[2]scale=-2:528[r];[l][h][r]hstack=inputs=3" \
                -c:v libx264 -pix_fmt yuv420p -crf 23 "/output/videos/$T/${e##*/}.mp4" ) && echo "/output/videos/$T/${e##*/}.mp4"
        done
        exit 0 ;;
esac

TASK="$1"; TARGET="${2:-10}"
need_task "$TASK"; need_mounts
ARCH="$TASK${ERSIBENCH_OOD_AXES:+__${ERSIBENCH_OOD_AXES//,/+}}"
REC=/output/recording_data/$ARCH LAY=/output/saved_task/$ARCH FAILED=/output/failed/$ARCH LOGS=/output/logs/$ARCH
LEROBOT=/output/lerobot/$ARCH
mkdir -p "$REC" "$LAY" "$FAILED" "$LOGS"
# the client numbers episodes past everything in recording_data and recording_archive: failed/ is the archive,
# so moving failures out never makes a number come back
mkdir -p "$DC/saved_task"
ln -sfn "$REC" "$DC/recording_data"
ln -sfn "$LAY" "$DC/saved_task/$TASK"
ln -sfn "$FAILED" "$DC/recording_archive"
export RECORDING_DATA_ROOT="$REC" PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
    PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True PRIVACY_CONSENT=Y OMNI_USER=geniesim OMNI_PASS=geniesim

passed() { [ -f "$1/aligned_joints.h5" ] && grep -q '"task_status": true' "$1/task_result.json" 2>/dev/null; }
count() { local d n=0; for d in "$REC"/\[${TASK}_*; do [ -d "$d" ] && passed "$d" && n=$((n + 1)); done; echo $n; }

BATCH="${BATCH:-20}" MAX_ROUNDS="${MAX_ROUNDS:-25}"
ROUND_TIMEOUT="${ROUND_TIMEOUT:-$((600 + BATCH * 360))}"   # Isaac/curobo occasionally hang with no output
done_n=$(count)
log "$ARCH: $done_n / $TARGET passed"
for round in $(seq 1 "$MAX_ROUNDS"); do
    [ "$done_n" -ge "$TARGET" ] && break
    # num_of_episode = attempts this round, num_of_success = stop once the gap is filled
    python3 - "$DC/tasks/$TASK/g2/$TASK.json" /tmp/ersi/task.json "$BATCH" $((TARGET - done_n)) "$DC" <<'EOF'
import json, sys
src, dst, attempts, wanted, dc = sys.argv[1:]
sys.path.insert(0, dc)
from common.task_file import load_task  # task json + the scene json it names
d = load_task(src)
d.setdefault("recording_setting", {}).update(num_of_episode=int(attempts), num_of_success=int(wanted))
json.dump(d, open(dst, "w"), ensure_ascii=False, indent=4)
EOF
    log "round $round: need $((TARGET - done_n)), up to $BATCH attempts"
    t0=$SECONDS
    LOG_DIR="$LOGS/round_$round" timeout -k 60 "$ROUND_TIMEOUT" \
        "$DC/scripts/data_collection_entrypoint.sh" --headless --task /tmp/ersi/task.json || log "round $round exited $?"
    pkill -f 'data_collector_server.py|run_data_collection.py' 2>/dev/null || true   # a timeout KILL skips its trap
    for d in "$REC"/\[${TASK}_*; do   # metric failures and episodes cut off by a timeout
        [ -d "$d" ] && ! passed "$d" || continue
        b=${d##*/}; [[ "$b" =~ ^\[${TASK}_([0-9]+)\] ]] || continue
        mv "$d" "$FAILED/"
        mv "$LAY/${TASK}_${BASH_REMATCH[1]}.json" "$FAILED/" 2>/dev/null || true
    done
    new_n=$(count)
    log "round $round done: +$((new_n - done_n)), $new_n / $TARGET"
    # a cold start alone takes ~100 s: no output in under a minute means the setup is broken, not bad luck
    [ "$new_n" -eq "$done_n" ] && [ $((SECONDS - t0)) -lt 60 ] && { log "collection did not start, see $LOGS/round_$round"; exit 1; }
    done_n=$new_n
done
log "$done_n / $TARGET passed -> $REC"

# the converter wants an empty output dir: build next to the old dataset, swap only once it succeeded
if [ "${CONVERT:-1}" = 1 ] && [ "$done_n" -gt 0 ]; then
    log "LeRobot: $REC -> $LEROBOT"
    rm -rf "$LEROBOT.tmp"
    PYTHONPATH=/opt/ersibench/source/geniesim_benchmark/src python3 -m geniesim_benchmark.dataset.convert.agibot_to_lerobot_compose \
        --agibot-dir "$REC" --output-dir "$LEROBOT.tmp" --seed 0 > "$LOGS/lerobot.log" 2>&1 \
        || { tail -20 "$LOGS/lerobot.log"; log "LeRobot conversion failed, see $LOGS/lerobot.log"; exit 1; }
    rm -rf "$LEROBOT"; mv "$LEROBOT.tmp" "$LEROBOT"
    log "LeRobot dataset: $LEROBOT"
fi
[ "$done_n" -ge "$TARGET" ]
