#!/usr/bin/env bash
# Refresh the ersibench/ snapshot (only what the image needs) from an ERSI-Bench checkout. One task ships
# (TASKS below, with the scene json it names); users write their own (skills/create-task) and mount them at /work.
#   ./sync_from_ersibench.sh ~/Repos/ERSI-Bench            # working tree, uncommitted changes included
#   ./sync_from_ersibench.sh ~/Repos/ERSI-Bench 9396cdd    # a commit (git archive, the checkout is not touched)
set -euo pipefail
SRC="${1:?usage: $0 /path/to/ERSI-Bench [REF]}"; REF="${2:-}"
if [ -n "$REF" ]; then
    TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
    git -C "$SRC" archive "$REF" scripts skills source/data_collection source/geniesim_benchmark/src | tar -x -C "$TMP"
    VERSION="$(git -C "$SRC" rev-parse --short "$REF")"; SRC="$TMP"
fi
DST="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/ersibench"
mkdir -p "$DST/source/data_collection"
rsync -a --delete --exclude __pycache__ --exclude .pytest_cache --exclude '*_tmp.yaml' \
    --exclude /workers --exclude /recording_data --exclude /recording_archive --exclude /saved_task --exclude /logs \
    --exclude /tasks --exclude /scenes --exclude /tests \
    "$SRC/source/data_collection/" "$DST/source/data_collection/"
TASKS="pick_biggest_block"
rm -rf "$DST/source/data_collection/tasks" "$DST/source/data_collection/scenes" "$DST/source/data_collection/tests"
for t in $TASKS; do
    f=source/data_collection/tasks/$t/g2/$t.json; install -D -m 644 "$SRC/$f" "$DST/$f"
    scene=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("scene_file", ""))' "$SRC/$f")
    [ -z "$scene" ] || install -D -m 644 "$SRC/source/data_collection/scenes/$scene.json" "$DST/source/data_collection/scenes/$scene.json"
done
BM=source/geniesim_benchmark/src/geniesim_benchmark
CK=skills/collect-episodes   # the task / scene checker and its schemas
for f in scripts/build_backgrounds.py scripts/backgrounds.json $CK/task_schema.py $CK/task.schema.json $CK/scene.schema.json $BM/__init__.py $BM/_version.py $BM/dataset/__init__.py \
         $BM/dataset/convert/__init__.py $BM/dataset/convert/agibot_to_lerobot.py $BM/dataset/convert/agibot_to_lerobot_compose.py; do
    install -D -m 644 "$SRC/$f" "$DST/$f"
done
echo "${VERSION:-$(git -C "$SRC" describe --always --dirty)}" > "$DST/SOURCE_COMMIT"
echo "synced from $SRC @ $(cat "$DST/SOURCE_COMMIT")"
