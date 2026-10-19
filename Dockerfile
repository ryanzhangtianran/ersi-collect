# Self-contained collection image: one container = one GPU, collects TASK until TARGET episodes pass, converts them
# to LeRobot v2.1, exits. Code (ersibench/, a snapshot of ERSI-Bench) is baked in; assets and the output dir are mounted.
#
# Build:  docker build -t ersibench-collect:latest .        (--build-arg PIP_INDEX=https://pypi.org/simple outside China)
# Ship:   docker save ersibench-collect:latest | zstd -T0 > ersibench-collect.tar.zst
# Load:   zstd -dc ersibench-collect.tar.zst | docker load
# Run:    docker run --rm ersibench-collect:latest           (prints usage)

FROM registry.agibot.com/genie-sim/geniesim3-data-collection:latest

ARG PIP_INDEX=https://pypi.tuna.tsinghua.edu.cn/simple
USER root
# LeRobot converter deps for the system python3 (numpy, PIL and ffmpeg with NVENC are already there)
RUN python3 -m pip install --no-cache-dir -i "$PIP_INDEX" h5py pyarrow
COPY --chown=1234:1234 ersibench /opt/ersibench
# data_collection_entrypoint.sh works in /geniesim/main/data_collection and loads assets from /geniesim_assets;
# build_backgrounds.py points non-scene files at /upstream_background.
RUN mkdir -p /geniesim/main /geniesim_assets && chown 1234:1234 /geniesim_assets && \
    ln -s /opt/ersibench/source/data_collection /geniesim/main/data_collection && \
    ln -s /assets/background /upstream_background
COPY entrypoint.sh /usr/local/bin/ersibench-collect

ENTRYPOINT ["/usr/local/bin/ersibench-collect"]
