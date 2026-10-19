# Base image for ersibench-collect, built from NVIDIA's public Isaac Sim image: everything the collector needs that is
# not our code. Build it once (about an hour, ~110 GB of disk); code updates only rebuild ./Dockerfile on top of it.
#
#   docker build -f base.Dockerfile -t ersibench-collect-base .
#   # inside mainland China:
#   docker build -f base.Dockerfile -t ersibench-collect-base \
#       --build-arg ISAACSIM_REGISTRY=nvcr.m.daocloud.io/nvidia \
#       --build-arg APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu \
#       --build-arg ROS_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
#       --build-arg CUDA_REPO=https://developer.download.nvidia.cn/compute/cuda/repos/ubuntu2404/x86_64 \
#       --build-arg PIP_INDEX=https://pypi.tuna.tsinghua.edu.cn/simple .
#
# CUDA_ARCHS: the GPU generations cuRobo is compiled for (no PTX fallback: a GPU not listed cannot plan).
#   8.6 RTX 30xx / A10 · 8.9 RTX 40xx / L40 · 9.0 H100 · 12.0 RTX 50xx. Default: 8.9;12.0.

ARG ISAACSIM_REGISTRY=nvcr.io/nvidia
ARG ISAACSIM_TAG=5.1.0
FROM ${ISAACSIM_REGISTRY}/isaac-sim:${ISAACSIM_TAG}

ARG APT_MIRROR=""
ARG ROS_MIRROR=""
ARG CUDA_REPO=https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64
ARG PIP_INDEX=https://pypi.org/simple
ARG CUROBO_GIT=https://github.com/NVlabs/curobo.git
ARG CUDA_ARCHS="8.9;12.0"

ENV ACCEPT_EULA=Y OMNI_KIT_ACCEPT_EULA=Y PRIVACY_CONSENT=Y DEBIAN_FRONTEND=noninteractive PIP_BREAK_SYSTEM_PACKAGES=1
USER root

# system packages: sudo / gosu / setfacl for the uid-1234 runtime, ffmpeg for videos, build tools for cuRobo
RUN if [ -n "$APT_MIRROR" ]; then \
        sed -i -E "s#https?://[^ ]*(archive|security).ubuntu.com/ubuntu#$APT_MIRROR#g" \
            /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list 2>/dev/null || true; \
    fi && \
    apt-get update && apt-get install -y --no-install-recommends \
        sudo gosu acl ffmpeg git curl wget ca-certificates gnupg locales software-properties-common \
        build-essential ninja-build python3 python3-pip python3-dev \
        libvulkan1 libgl1 libegl1 libglu1-mesa libx11-6 libxrandr2 libxss1 libxcursor1 libxi6 libnss3 \
        libxcomposite1 libxdamage1 libxext6 && \
    locale-gen en_US.UTF-8 && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ROS 2 Jazzy, only what recording uses: `ros2 bag record` (mcap), `ros2 run image_transport republish` (compressed)
RUN add-apt-repository universe -y && \
    curl -fsSL -o /tmp/ros2-apt-source.deb \
        https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.noble_all.deb && \
    dpkg -i /tmp/ros2-apt-source.deb && rm /tmp/ros2-apt-source.deb && \
    sed -i 's/^Types: deb deb-src$/Types: deb/' /etc/apt/sources.list.d/ros2.sources && \
    if [ -n "$ROS_MIRROR" ]; then sed -i "s|http://packages.ros.org/ros2/ubuntu|$ROS_MIRROR|g" /etc/apt/sources.list.d/ros2.sources; fi && \
    if [ -n "$APT_MIRROR" ]; then \
        sed -i -E "s#https?://[^ ]*(archive|security).ubuntu.com/ubuntu#$APT_MIRROR#g" \
            /etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list 2>/dev/null || true; \
    fi && \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-image-transport ros-jazzy-image-transport-plugins ros-jazzy-vision-msgs && \
    rm -rf /var/lib/apt/lists/*

# CUDA 12.8 toolkit: nvcc for compiling cuRobo against Isaac Sim's torch (cu128)
RUN curl -fsSL -o /tmp/cuda-keyring.deb "$CUDA_REPO/cuda-keyring_1.1-1_all.deb" && \
    dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb && \
    sed -i "s|https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64|$CUDA_REPO|g" \
        /etc/apt/sources.list.d/cuda-ubuntu2404-x86_64.list && \
    apt-get update && apt-get install -y --no-install-recommends cuda-toolkit-12-8 && rm -rf /var/lib/apt/lists/*
ENV CUDA_HOME=/usr/local/cuda-12.8

# the container runs as Isaac Sim's uid 1234 with passwordless sudo (the collector's entrypoint uses it)
RUN (id isaac-sim >/dev/null 2>&1 || useradd -m -u 1234 -s /bin/bash isaac-sim) && \
    echo "isaac-sim ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/isaac-sim && chmod 0440 /etc/sudoers.d/isaac-sim && \
    chmod o+rx /isaac-sim

# collector deps in Isaac Sim's python, then cuRobo compiled for CUDA_ARCHS
COPY ersibench/source/data_collection/requirements.txt /tmp/requirements.txt
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -i "$PIP_INDEX" -U pip && \
    /isaac-sim/python.sh -m pip install --no-cache-dir -i "$PIP_INDEX" -r /tmp/requirements.txt
RUN git clone --depth 1 --branch v0.7.6 "$CUROBO_GIT" /tmp/curobo && cd /tmp/curobo && \
    TORCH_CUDA_ARCH_LIST="$CUDA_ARCHS" /isaac-sim/python.sh -m pip install --no-cache-dir --no-build-isolation \
        -i "$PIP_INDEX" ".[isaacsim]"
# the versions the internal image collects with (cuRobo pulls warp-lang in unpinned; Isaac Sim's python wants scipy 1.13)
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -i "$PIP_INDEX" warp-lang==1.17.0 scipy==1.13.1
# the pip prebundles: core_archive lacks packaging, and pip_archive's numpy would shadow the pinned 1.26.4
RUN /isaac-sim/python.sh -m pip install --no-cache-dir -i "$PIP_INDEX" packaging \
        -t /isaac-sim/exts/omni.isaac.core_archive/pip_prebundle/ && \
    rm -rf /isaac-sim/extscache/omni.kit.pip_archive-*/pip_prebundle/numpy \
           /isaac-sim/extscache/omni.kit.pip_archive-*/pip_prebundle/numpy-*.dist-info \
           /isaac-sim/extscache/omni.kit.pip_archive-*/pip_prebundle/numpy.libs

# the system python runs the entrypoint's helpers and the LeRobot converter. pyarrow needs numpy 2; the apt numpy 1.26
# (a ROS dependency) cannot be uninstalled by pip, so numpy 2 goes to /usr/local in front of it, as in the internal
# image. Isaac Sim's own python is separate and keeps its pinned 1.26.4.
RUN python3 -m pip install --no-cache-dir -i "$PIP_INDEX" --ignore-installed numpy && \
    python3 -m pip install --no-cache-dir -i "$PIP_INDEX" h5py pyarrow pillow

# cuRobo rebuilds its CUDA graph when the planner switches goal type (pose / goalset / batch); without this it raises
# "changing goal type not supported in cuda graph mode" on the first grasp and nothing can be collected
ENV CUROBO_TORCH_CUDA_GRAPH_RESET=1
