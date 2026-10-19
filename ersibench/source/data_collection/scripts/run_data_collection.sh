#!/bin/bash

# Enhanced Data Collection Startup Script
# Usage: ./scripts/run_data_collection.sh [--headless] [--no-record] [--task TASK_PATH] [--saved-task-folder FOLDER] [--container-name NAME]
# ERSIBENCH_OOD_AXES=light,camera_pose (RSIBench axis names) collects that OOD tier of the template's `generalization` block.

set -eo pipefail
source "$(cd "$(dirname "$0")/../../.." && pwd)/scripts/ersibench_env.sh"   # ERSIBENCH_ASSETS etc.; no need to source it first

# Default values
HEADLESS=false
RECORD=true
TASK="tasks/pick_biggest_block/g2/pick_biggest_block.json"
SAVED_TASK_FOLDER=""
STANDALONE=false  # false = print to terminal, true = only save to file
CONTAINER_NAME="data_collection_open_source"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --headless)
            HEADLESS=true
            shift
            ;;
        --no-record)
            RECORD=false
            shift
            ;;
        --task)
            TASK="$2"
            shift 2
            ;;
        --saved-task-folder)
            SAVED_TASK_FOLDER="$2"
            shift 2
            ;;
        --standalone)
            STANDALONE=true
            shift
            ;;
        --container-name)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: ./scripts/run_data_collection.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --headless              Run in headless mode (default: false)"
            echo "  --no-record             Disable recording (default: record enabled)"
            echo "  --task TASK_PATH        Task template path (default: tasks/pick_biggest_block/g2/pick_biggest_block.json)"
            echo "  --saved-task-folder FOLDER  Run pre-generated task JSON files from this folder"
            echo "  --standalone             Run in standalone mode (only save logs, no terminal output) (default: false, prints to terminal)"
            echo "  --container-name NAME   Container name (default: data_collection_open_source)"
            echo "  --help, -h              Show this help message"
            echo ""
            echo "Examples:"
            echo "  ./scripts/run_data_collection.sh --headless --task tasks/pick_biggest_block/g2/pick_biggest_block.json"
            echo "  ./scripts/run_data_collection.sh --standalone --headless"
            exit 0
            ;;
        *)
            echo "Error: Unknown option '$1'"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# geniesim_assets source root: the CLI passes the editable-installed host path
# via GENIESIM_ASSETS_SRC; it is bind-mounted to /geniesim_assets below and
# editable-installed by the entrypoint.
ASSETS_SRC="${GENIESIM_ASSETS_SRC:-${ERSIBENCH_ASSETS:-}}"
if [ -z "$ASSETS_SRC" ] || [ ! -f "$ASSETS_SRC/pyproject.toml" ]; then
    echo "Error: geniesim_assets is not pip-installed (editable) on the host."
    echo "Install it first, e.g.: pip install -e /path/to/geniesim_assets"
    exit 1
fi
# Collection workers (collect_worker.sh, started by scripts/collect.sh) set these; unset = one standalone container
# (geniesim autocollect run, for debugging).
#   DC_GPUS        docker --gpus value, e.g. device=3                         (default: all)
#   DC_NETWORK     docker network; workers use bridge so each container has its own loopback:
#                  gRPC :50051 and the ROS 2 topics ros2 bag records stay private (default: host)
#   DC_WORKER_DIR  per-worker host dir holding saved_task/, logs/, background/, the round's task json
#                  and the Isaac Sim cache, mounted over the shared checkout
DC_GPUS="${DC_GPUS:-all}"
# The image bakes in NVIDIA_VISIBLE_DEVICES=all, which overrides --gpus device=N and exposes every GPU
# to each worker (all of them then land on GPU 0: OOM, and Isaac Sim FD_SETSIZE crashes). Pin it explicitly.
DC_NV_DEVICES="${DC_GPUS//\"/}"; DC_NV_DEVICES="${DC_NV_DEVICES#device=}"
# --privileged exposes every /dev/nvidia* regardless of the above, so keep it only for the unpinned (all) case.
DC_PRIV_ARGS=""; [ "$DC_NV_DEVICES" = all ] && DC_PRIV_ARGS="--privileged"
DC_NETWORK="${DC_NETWORK:-host}"
DC_WORKER_DIR="${DC_WORKER_DIR:-}"
# ERSI backgrounds: rendered from scripts/backgrounds.json into a scratch dir and mounted over /geniesim_assets/background/<key>
# (build_backgrounds.py deletes and rebuilds the dir, so every worker needs its own)
BG_DIR="${ERSIBENCH_BACKGROUNDS:-${DC_WORKER_DIR:+$DC_WORKER_DIR/background}}"
BG_DIR="${BG_DIR:-/tmp/ersibench/background}"
python3 "$(cd "$(dirname "$0")/../../.." && pwd)/scripts/build_backgrounds.py" --assets "$ASSETS_SRC" --out "$BG_DIR" >/dev/null
BG_MOUNTS="-v $BG_DIR:/geniesim_assets/background:ro -v $ASSETS_SRC/background:/upstream_background:ro"

# ERSI-owned assets: a separate tree ($ERSIBENCH_ASSETS_ERSI, default assets_ersi/) laid out like
# geniesim_assets and mounted over the matching subdirectories, so its objects resolve under the
# same SIM_ASSETS root without copying anything into the upstream checkout.
ERSI_ASSETS="${ERSIBENCH_ASSETS_ERSI:-$(cd "$(dirname "$0")/../../.." && pwd)/assets_ersi}"
ERSI_MOUNTS=""
if [ -d "$ERSI_ASSETS/objects" ]; then
    for d in "$ERSI_ASSETS"/objects/*/; do
        ERSI_MOUNTS="$ERSI_MOUNTS -v $d:/geniesim_assets/objects/$(basename "$d"):ro"
    done
fi
if [ -d "$ERSI_ASSETS/interaction" ]; then
    for d in "$ERSI_ASSETS"/interaction/*/; do
        ERSI_MOUNTS="$ERSI_MOUNTS -v $d:/geniesim_assets/interaction/$(basename "$d"):ro"
    done
fi

# Get current directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CURRENT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Grant the container user (uid 1234) write access to bind-mounted dirs.
# Prefer sudo+setfacl (original behavior); degrade to plain mkdir + chmod when
# sudo isn't usable (e.g. unattended / no-tty, where sudo can't cache its
# tty-keyed timestamp) so the run doesn't hard-fail under `set -e`.
grant_access() {  # $1 = target dir
    mkdir -p "$1" 2>/dev/null || sudo mkdir -p "$1" 2>/dev/null || true
    if sudo -n true 2>/dev/null; then
        sudo setfacl -m u:1234:rwX -R "$1" 2>/dev/null || true
    else
        chmod -R a+rwX "$1" 2>/dev/null || true
    fi
}
grant_access "$CURRENT_DIR/scripts"
SAVED_TASK_ROOT="$CURRENT_DIR/saved_task"
WORKER_MOUNTS=""
if [ -n "$DC_WORKER_DIR" ]; then
    SAVED_TASK_ROOT="$DC_WORKER_DIR/saved_task"
    WORKER_MOUNTS="-v $SAVED_TASK_ROOT:/geniesim/main/data_collection/saved_task:rw"
    # the worker rewrites the round budget into its own copy of the task json, mounted over the shared file
    [ -f "$DC_WORKER_DIR/task.json" ] && WORKER_MOUNTS="$WORKER_MOUNTS -v $DC_WORKER_DIR/task.json:/geniesim/main/data_collection/$TASK:ro"
fi
grant_access "$SAVED_TASK_ROOT"
# Where episodes land. The server always writes to
# /geniesim/main/data_collection/recording_data inside the container; that path
# is bind-mounted from GENIESIM_RECORDING_ROOT when set (e.g. a NAS export), and
# from the repo's own recording_data/ otherwise. Resolved through symlinks
# because the docker daemon mounts host paths literally.
RECORDING_ROOT="$(readlink -f "${GENIESIM_RECORDING_ROOT:-${DATA_COLLECTION_OUTPUT_ROOT:-$CURRENT_DIR/recording_data}}")"
OUTPUT_ROOT="$RECORDING_ROOT"  # djy_dev0 name for the same mount
mkdir -p "$RECORDING_ROOT"
grant_access "$RECORDING_ROOT"
# Archive of finished episodes (e.g. a NAS), mounted read-only so the client can
# continue the numbering from it. Unset -> the recording root itself (harmless).
RECORDING_ARCHIVE="$(readlink -f "${GENIESIM_RECORDING_ARCHIVE:-$RECORDING_ROOT}")"

# Isaac Sim cache, persisted across the containers each round creates.  Kept under ~/.cache (XDG)
# rather than ~/docker: docker used to create it as root inside $HOME, which then needed sudo to
# manage and broke `mkdir` in run_benchmark.sh.  The container runs as uid 1234, so the dirs are
# created here by the host user and opened up with chmod instead of a sudo chown.
CACHE_ROOT="$(eval echo "${GENIESIM_CACHE_ROOT:-$HOME/.cache/geniesim/isaac-sim}")"
if [ -n "$DC_WORKER_DIR" ]; then
    # Kit's config/logs/data are not safe to share between live processes; seed a worker's cache from the shared one once
    [ -d "$DC_WORKER_DIR/isaac-sim" ] || { mkdir -p "$DC_WORKER_DIR/isaac-sim"; cp -a "$CACHE_ROOT/." "$DC_WORKER_DIR/isaac-sim/" 2>/dev/null || true; }
    CACHE_ROOT="$DC_WORKER_DIR/isaac-sim"
fi
mkdir -p "$CACHE_ROOT"/{cache/main,cache/computecache,logs,config,data/documents,data/Kit,pkg}
chmod -R a+rwX "$CACHE_ROOT" 2>/dev/null || true
[ -d "$CURRENT_DIR/config" ] && grant_access "$CURRENT_DIR/config"
# Extract task name from task path or JSON file
TASK_NAME=""
if [ -f "$TASK" ]; then
    # Try to extract task name from JSON file
    TASK_NAME=$(python3 -c "import json, sys;
try:
    with open('$TASK', 'r') as f:
        data = json.load(f)
        task_name = data.get('task', '')
        if task_name:
            print(task_name)
except:
    pass" 2>/dev/null || echo "")
fi

# If task name not found in JSON, use filename without extension
if [ -z "$TASK_NAME" ] || [ "$TASK_NAME" = "" ]; then
    TASK_NAME=$(basename "$TASK" .json)
fi

# Sanitize task name for directory name (remove special characters)
TASK_NAME=$(echo "$TASK_NAME" | sed 's/[^a-zA-Z0-9_-]/_/g')

# Create log directory based on task name
LOG_DIR="${DC_WORKER_DIR:-$CURRENT_DIR}/logs/${TASK_NAME}"
mkdir -p "$LOG_DIR"

# Set permissions for log directory (container runs as user 1234:1234)
# Make sure the directory is writable by the container user
chmod 777 "$LOG_DIR" 2>/dev/null || true
# Also try to set ownership if possible (may require sudo)
if command -v sudo >/dev/null 2>&1; then
    sudo -n chown -R 1234:1234 "$LOG_DIR" 2>/dev/null || true
    sudo -n chmod -R 777 "$LOG_DIR" 2>/dev/null || true
fi

# Log file for script output
SCRIPT_LOG="${LOG_DIR}/run_data_collection_sh.log"

# Function to log and optionally print
log_and_print() {
    if [ "$STANDALONE" = true ]; then
        # Standalone mode: only save to file
        echo "$(date '+%Y-%m-%d %H:%M:%S') - $*" >> "$SCRIPT_LOG"
    else
        # Default mode: print to terminal and save to file
        echo "$(date '+%Y-%m-%d %H:%M:%S') - $*" | tee -a "$SCRIPT_LOG"
    fi
}

# Function to cleanup container
cleanup_container() {
    log_and_print "Cleaning up container..."
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log_and_print "Stopping container: $CONTAINER_NAME"
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
        log_and_print "Removing container: $CONTAINER_NAME"
        docker rm "$CONTAINER_NAME" 2>/dev/null || true
        log_and_print "Container cleaned up"
    fi
}

# Set trap for cleanup
trap cleanup_container EXIT INT TERM

# Check if container already exists
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    log_and_print "Container '$CONTAINER_NAME' already exists. Removing it..."
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
fi

log_and_print "=========================================="
log_and_print "Data Collection Startup"
log_and_print "=========================================="
log_and_print "Configuration:"
log_and_print "  Headless mode: $HEADLESS"
log_and_print "  Recording: $RECORD"
log_and_print "  Task template: $TASK"
log_and_print "  Task name: $TASK_NAME"
log_and_print "  Recording root: $RECORDING_ROOT"
log_and_print "  Recording archive: $RECORDING_ARCHIVE"
log_and_print "  Container name: $CONTAINER_NAME"
log_and_print "  GPUs / network: $DC_GPUS / $DC_NETWORK"
log_and_print "  Worker dir: ${DC_WORKER_DIR:-<none>}"
log_and_print "  Log directory: $LOG_DIR"
log_and_print "  Recording output: $OUTPUT_ROOT"
log_and_print "  Standalone mode: $STANDALONE (false = print to terminal, true = only save to file)"
log_and_print "=========================================="
log_and_print ""

# Build entrypoint arguments
DISPLAY_ARGS=""
ENTRYPOINT_ARGS=""
if [ "$HEADLESS" = true ]; then
    ENTRYPOINT_ARGS="$ENTRYPOINT_ARGS --headless"
else
    xhost +
    # X unix socket must be in the container: the host Xorg runs with -nolisten tcp
    DISPLAY_ARGS="-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
fi
if [ "$RECORD" = false ]; then
    ENTRYPOINT_ARGS="$ENTRYPOINT_ARGS --no-record"
fi
if [ -n "$TASK" ]; then
    ENTRYPOINT_ARGS="$ENTRYPOINT_ARGS --task $TASK"
fi
if [ -n "$SAVED_TASK_FOLDER" ]; then
    ENTRYPOINT_ARGS="$ENTRYPOINT_ARGS --saved-task-folder $SAVED_TASK_FOLDER"
fi

# Start container with log directory mounted
log_and_print "Starting Docker container..."
log_and_print "Command: docker run -d --name $CONTAINER_NAME ..."
log_and_print "Log directory mounted: $LOG_DIR -> /geniesim/main/data_collection/logs/${TASK_NAME}"

CONTAINER_ID=$(docker run -d --name $CONTAINER_NAME \
    --user 1234:1234 \
    --entrypoint ./scripts/data_collection_entrypoint.sh \
    --gpus "$DC_GPUS" \
    -e "NVIDIA_VISIBLE_DEVICES=$DC_NV_DEVICES" \
    --network="$DC_NETWORK" \
    $DC_PRIV_ARGS \
    $DISPLAY_ARGS \
    -e "ACCEPT_EULA=Y" \
    -e "PRIVACY_CONSENT=Y" \
    -e "PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python" \
    -e "ERSIBENCH_OOD_AXES=${ERSIBENCH_OOD_AXES:-}" \
    -e "PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True" \
    -e "OMNI_USER=geniesim" \
    -e "OMNI_PASS=geniesim" \
    -e "LOG_DIR=/geniesim/main/data_collection/logs/${TASK_NAME}" \
    -e "RECORDING_DATA_ROOT=/geniesim/main/data_collection/recording_data" \
    -e "SIM_ASSETS=/geniesim_assets" \
    -v "$CACHE_ROOT/cache/main:/isaac-sim/.cache:rw" \
    -v "$CACHE_ROOT/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
    -v "$CACHE_ROOT/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
    -v "$CACHE_ROOT/config:/isaac-sim/.nvidia-omniverse/config:rw" \
    -v "$CACHE_ROOT/data:/isaac-sim/.local/share/ov/data:rw" \
    -v "$CACHE_ROOT/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
    -v /dev/input:/dev/input:rw \
    -v $ASSETS_SRC:/geniesim_assets:rw \
    $BG_MOUNTS \
    $ERSI_MOUNTS \
    -v $CURRENT_DIR:/geniesim/main/data_collection:rw \
    $WORKER_MOUNTS \
    -v "$RECORDING_ROOT":/geniesim/main/data_collection/recording_data:rw \
    -v "$RECORDING_ARCHIVE":/geniesim/main/data_collection/recording_archive:ro \
    -v $LOG_DIR:/geniesim/main/data_collection/logs/${TASK_NAME}:rw \
    -w /geniesim/main/data_collection \
    registry.agibot.com/genie-sim/geniesim3-data-collection:latest \
    $ENTRYPOINT_ARGS)

if [ -z "$CONTAINER_ID" ]; then
    log_and_print "Error: Failed to start container"
    exit 1
fi

log_and_print "Container started: $CONTAINER_ID"
log_and_print ""

# Wait a bit for container to initialize
sleep 3

# Function to monitor container and optionally print logs
monitor_container() {
    if [ "$STANDALONE" = true ]; then
        # Standalone mode: only monitor, logs are already being written directly
        log_and_print "Monitoring container in standalone mode (logs saved to: $LOG_DIR)..."
        log_and_print "Press Ctrl+C to stop"
        log_and_print ""

        while docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; do
            sleep 10
        done

        log_and_print "Container stopped"
    else
        # Default mode: print logs to terminal while saving to file
        log_and_print "Monitoring container (logs printed to terminal and saved to: $LOG_DIR)..."
        log_and_print "Press Ctrl+C to stop"
        log_and_print ""

        # Tail container logs to terminal and file
        # Container logs (from entrypoint script) go to docker logs
        docker logs -f "$CONTAINER_NAME" 2>&1 | tee -a "${LOG_DIR}/container.log" &
        CONTAINER_LOG_PID=$!

        # Wait a bit for application logs to be created
        sleep 5

        # Tail application logs if they exist
        SERVER_LOG_PID=""
        MAIN_LOG_PID=""

        if [ -f "${LOG_DIR}/data_collector_server.log" ]; then
            tail -f "${LOG_DIR}/data_collector_server.log" &
            SERVER_LOG_PID=$!
        fi

        if [ -f "${LOG_DIR}/run_data_collection.log" ]; then
            tail -f "${LOG_DIR}/run_data_collection.log" &
            MAIN_LOG_PID=$!
        fi

        # Wait for container to stop
        wait $CONTAINER_LOG_PID 2>/dev/null || true

        # Kill log tail processes
        [ -n "$SERVER_LOG_PID" ] && kill $SERVER_LOG_PID 2>/dev/null || true
        [ -n "$MAIN_LOG_PID" ] && kill $MAIN_LOG_PID 2>/dev/null || true

        log_and_print "Container stopped"
    fi
}

# Start monitoring
monitor_container

# Logs are already being written directly, no need to copy

log_and_print ""
log_and_print "=========================================="
log_and_print "Data collection completed"
log_and_print "=========================================="
log_and_print "Logs saved to: $LOG_DIR"
log_and_print "  - run_data_collection_sh.log (script output)"
log_and_print "  - container.log (container logs)"
log_and_print "  - data_collector_server.log (if available)"
log_and_print "  - run_data_collection.log (if available)"
log_and_print ""

# Cleanup will be done by trap
log_and_print "Exiting (container will be cleaned up automatically)..."
