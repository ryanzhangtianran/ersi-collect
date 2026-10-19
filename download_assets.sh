#!/usr/bin/env bash
# Fetch the GenieSim asset library (ModelScope, git + LFS, ~47 GB); mount the result at /assets.
#   ./download_assets.sh                    # into ./geniesim_assets; re-running resumes / updates
#   ./download_assets.sh /data/geniesim_assets
set -euo pipefail
DIR="${1:-geniesim_assets}"
URL="${ASSETS_URL:-https://www.modelscope.cn/datasets/agibot_world/GenieSimAssets.git}"
command -v git-lfs >/dev/null || { echo "git-lfs is required: sudo apt install git-lfs" >&2; exit 1; }
git lfs install --skip-repo >/dev/null
if [ -d "$DIR/.git" ]; then
    git -C "$DIR" pull --ff-only
else
    GIT_LFS_SKIP_SMUDGE=1 git clone "$URL" "$DIR"   # pointers first, then the blobs in one resumable pass
fi
git -C "$DIR" lfs pull
[ -f "$DIR/pyproject.toml" ] || { echo "clone looks incomplete: no pyproject.toml in $DIR" >&2; exit 1; }
missing=$(git -C "$DIR" lfs ls-files | grep -c '^[0-9a-f]* - ' || true)
echo "done: $(git -C "$DIR" lfs ls-files | wc -l) LFS files, $missing still pointers (re-run if > 0) -> mount $(cd "$DIR" && pwd) at /assets"
