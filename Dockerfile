# Self-contained collection image: one container = one GPU, collects TASK until TARGET episodes pass, converts them
# to LeRobot v2.1, exits. Code (ersibench/, a snapshot of ERSI-Bench) is baked in; assets and the output dir are mounted.
# Only this thin layer is rebuilt when the code changes (seconds); the heavy part is base.Dockerfile, built once.
#
# Build:  docker build -f base.Dockerfile -t ersibench-collect-base .     (once; see that file for mirrors / GPUs)
#         docker build -t ersibench-collect:latest .                       (after every git pull)
# Run:    docker run --rm ersibench-collect:latest                         (prints usage)
# Maintainers with the internal image: --build-arg BASE=registry.agibot.com/genie-sim/geniesim3-data-collection:latest

ARG BASE=ersibench-collect-base:latest
FROM ${BASE}

ARG PIP_INDEX=https://pypi.org/simple
USER root
# LeRobot converter deps for the system python3 (already in ersibench-collect-base; the internal base lacks them)
RUN python3 -m pip install --no-cache-dir -i "$PIP_INDEX" h5py pyarrow
COPY --chown=1234:1234 ersibench /opt/ersibench
# data_collection_entrypoint.sh works in /geniesim/main/data_collection and loads assets from /geniesim_assets;
# build_backgrounds.py points non-scene files at /upstream_background.
RUN mkdir -p /geniesim/main /geniesim_assets && chown 1234:1234 /geniesim_assets && \
    ln -sfn /opt/ersibench/source/data_collection /geniesim/main/data_collection && \
    ln -sfn /assets/background /upstream_background
COPY entrypoint.sh /usr/local/bin/ersibench-collect

ENTRYPOINT ["/usr/local/bin/ersibench-collect"]
