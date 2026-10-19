# ERSI-Collect 使用说明

ERSI-Collect 把 ERSI-Bench 的数据采集打包成一个 Docker 镜像 `ersibench-collect`，能做这几件事：

- 用 Isaac Sim 在一张 GPU 上采集 G2 双臂机器人的操作轨迹；
- 一轮一轮地采，直到成功的 episode 数达到你设的目标；
- 采完自动转成 LeRobot v2.1 数据集；
- 给每条 episode 生成一段视频，三路相机拼在一起。

宿主机只要装 Docker 和 NVIDIA 驱动，不需要 conda、Isaac Sim 或源码。镜像用仓库里的 Dockerfile 在本地构建，分两层：基础层只构建一次；代码层很薄，代码更新后重建只要几十秒。

镜像只自带一个任务 `pick_biggest_block`（拿起最大的积木）。其他任务按 `skills/create-task/SKILL.md` 自己写，放进挂载到容器里的目录就能用，不用重新构建镜像。

> 已验证的配置（2026-10-19）：RTX 5090 D、驱动 580、Ubuntu 主机，基础层用国内镜像源从头构建，`pick_biggest_block` 第一次尝试就采集成功，`sample_ranges` 和 LeRobot 转换都正常。

## 0. 快速上手

```bash
git clone https://github.com/ryanzhangtianran/ersi-collect.git && cd ersi-collect
docker build -f base.Dockerfile -t ersibench-collect-base .   # 一次，约 20–60 分钟（国内加镜像源参数，见 2.2）
docker build -t ersibench-collect:latest .                    # 约 40 秒
./download_assets.sh                                          # assets，约 47 GB
mkdir -p out
docker run --rm --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 \
    -v $PWD/geniesim_assets:/assets:ro -v $PWD/out:/output \
    ersibench-collect:latest pick_biggest_block 10
```

## 1. 环境要求

**构建镜像的机器**（不需要 GPU，可以在一台机器上构建，再拿到 GPU 机器上运行）：

| 项目 | 要求 |
|---|---|
| CPU 架构 | x86_64（amd64）。NVIDIA 的 Isaac Sim 5.1 基础镜像只有 amd64 版本 |
| 磁盘 | 至少 80 GB 空闲。基础层构建完占 46 GB，构建过程还要放中间层 |
| 内存 | 建议 16 GB 以上（nvcc 编译 cuRobo 比较吃内存） |
| Docker | 19.03 或更新 |
| 网络 | 要能访问 NVIDIA 镜像仓库、Ubuntu / ROS / CUDA 软件源、PyPI，前面这些都可以换成国内镜像源；另外还要访问 GitHub 两次：下载 `ros2-apt-source` 和克隆 cuRobo（克隆地址可以用 `CUROBO_GIT` 改） |

**运行采集的机器**：

| 项目 | 要求 |
|---|---|
| 系统 | 任意 Linux 发行版，Ubuntu 20.04、22.04、24.04 都可以（镜像内部是 24.04） |
| GPU | 默认支持 RTX 40 系和 L40（sm_89）、RTX 50 系（sm_120），显存至少 24 GB。其他型号要在构建基础层时用 `CUDA_ARCHS` 加上，比如 3090 / A10 是 8.6，H100 是 9.0。没编译进去的卡，cuRobo 不能做规划 |
| 驱动 | 建议用新驱动，已测过 580。镜像里是 CUDA 12.8，靠 CUDA 小版本兼容，一般 525 以上的驱动都能跑；但 Isaac Sim 5.1 在 550 上能不能用还没实测 |
| 软件 | Docker 和 nvidia-container-toolkit（见 2.1） |
| 磁盘 | 镜像约 46 GB，assets 约 47 GB，输出目录至少留 80 GB（每次尝试的原始录制最多约 10 GB，处理完就删） |

## 2. 安装

### 2.1 Docker 和 NVIDIA Container Toolkit

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER            # 重新登录后生效

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
```

### 2.2 构建镜像

镜像分两层：

- `base.Dockerfile`：基础层。从 NVIDIA 公开的 `nvcr.io/nvidia/isaac-sim:5.1.0` 开始，装 ROS 2 Jazzy（只装录制要用的部分）、CUDA 12.8、cuRobo v0.7.6 和采集的 Python 依赖。只构建一次。
- `Dockerfile`：代码层。只放 ERSI-Bench 的代码快照和入口脚本，每次更新代码后重建。

```bash
git clone https://github.com/ryanzhangtianran/ersi-collect.git && cd ersi-collect

# 1) 基础层，构建一次
docker build -f base.Dockerfile -t ersibench-collect-base .

#    在国内建议换成国内镜像源（这套参数已经实测能构建成功）：
docker build -f base.Dockerfile -t ersibench-collect-base \
    --build-arg ISAACSIM_REGISTRY=nvcr.m.daocloud.io/nvidia \
    --build-arg APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu \
    --build-arg ROS_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
    --build-arg CUDA_REPO=https://developer.download.nvidia.cn/compute/cuda/repos/ubuntu2404/x86_64 \
    --build-arg PIP_INDEX=https://pypi.tuna.tsinghua.edu.cn/simple .

#    要支持其他 GPU，再加一个参数：--build-arg CUDA_ARCHS="8.6;8.9;9.0;12.0"

# 2) 代码层，大约 40 秒
docker build -t ersibench-collect:latest .
```

**更新代码**时执行 `git pull && docker build -t ersibench-collect:latest .`，基础层不用重建。

### 2.3 下载 assets

assets 是 GenieSim 的物体、场景和机器人模型库，托管在 ModelScope，用 git-lfs 下载，大约 47 GB：

```bash
sudo apt install -y git-lfs
./download_assets.sh                          # 下载到 ./geniesim_assets；中断了重新运行会接着下
./download_assets.sh /data/geniesim_assets    # 也可以下到别的位置
```

最后一行显示 `0 still pointers`，才说明下载完整了。

### 2.4 验证

```bash
docker run --rm --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 --entrypoint nvidia-smi ersibench-collect:latest -L
# 应该只列出一张卡
docker run --rm ersibench-collect:latest list
# 应该输出 tasks: ['pick_biggest_block'] ...
```

## 3. 采集

```bash
A=$PWD/geniesim_assets      # assets 目录
O=$PWD/out; mkdir -p $O     # 输出目录
docker run -d --name ersi_pick --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 \
    -v ersi-isaac-cache:/isaac-sim/.cache -v ersi-isaac-cc:/isaac-sim/.nv/ComputeCache \
    -v $A:/assets:ro -v $O:/output \
    ersibench-collect:latest pick_biggest_block 50
docker logs -f ersi_pick     # 看进度，日志里是 round R done: +g, d / 50
```

- 最后两个参数分别是任务名和目标条数，这里要采到 50 条成功的 episode 才结束。
- `--gpus device=N` 和 `NVIDIA_VISIBLE_DEVICES=N` 必须写同一张卡。一张卡跑一个容器；多卡就起多个容器，每个容器用不同的任务或不同的输出目录。
- 两个 `ersi-isaac-cache` 卷用来缓存 shader，可以不挂；挂了以后，从第二次启动开始能省几分钟。
- 每一轮都要先启动 Isaac Sim，大约 2–3 分钟；之后每次尝试 2–5 分钟。
- 停止用 `docker rm -f ersi_pick`。已经采成功的数据会保留，再用同样的命令运行，会接着已有的条数继续采。

可选环境变量，用 `-e` 传入：

| 变量 | 默认值 | 作用 |
|---|---|---|
| `BATCH` | 20 | 每轮最多尝试几次 |
| `MAX_ROUNDS` | 25 | 最多跑几轮 |
| `CONVERT` | 1 | 设成 0 就不转 LeRobot |
| `ERSIBENCH_OOD_AXES` | 不设 | 采 OOD 数据，比如 `light,camera_pose`，输出放在 `<任务>__light+camera_pose/` 下 |

输出目录：

| 路径 | 内容 |
|---|---|
| `out/recording_data/<任务>/[<任务>_N]/` | 成功的 episode，保留全部帧：关节 h5、三路相机图片、状态、`sample_ranges.json` |
| `out/saved_task/<任务>/` | 每条 episode 对应的场景布局 |
| `out/failed/<任务>/` | 失败的 episode，留着排查问题用 |
| `out/lerobot/<任务>/` | LeRobot v2.1 数据集，每次运行结束后重新生成 |
| `out/logs/<任务>/` | 每一轮的 server / client 日志 |

### 3.1 静止段怎么处理

原始 episode 保留所有帧，采集时不删任何帧。每条 episode 提取完成后，会生成一个 `sample_ranges.json`，记录训练要用的帧段。转成 LeRobot 时，每个帧段单独作为一个 episode。

- **静止帧**：和前一帧比，手臂关节位移小于 1e-4 rad、夹爪关节小于 1e-3 rad、所有物体位移小于 1 mm，三个条件同时满足才算静止。所以积木松手后还在落定的那几帧不算静止。
- **在哪里切**：静止帧连续 0.3 s 以上，就在这里切开，切口两端各留 2 帧静止，这样每一段都从静止起步、在静止收尾。如果切掉的部分不到 0.2 s，就不切。
- **必要停顿**：夹爪开合指令、抓取附着变化（抓取、释放、接触）前后 0.33 s 内的静止段，两端各保留 0.2 s；最后一个动作结束后，也保留 0.2 s（任务完成）。
- **整段丢弃**：完全没有运动的段直接丢掉。

每个 episode 内部的帧是连续的，没有时间跳变，所以 action chunk 也连续。chunk 超出段尾时用最后一帧补齐，这和实际一致：那个时刻机器人确实是静止的。训练代码不用做任何改动。

如果要一条录制对应一个 LeRobot episode，转换时加 `--whole-episodes`。

参数扫描结果（6 条 episode，共 4901 帧）：

| 规则 | 被排除 | 保留帧里的静止占比 | 保留下来的最长停顿 |
|---|---|---|---|
| 最小静止 0.5 s，必要停顿 0.2 s | 34% | 18% | 0.47 s |
| **最小静止 0.3 s，必要停顿 0.2 s（默认）** | 35% | 17% | 0.47 s |
| 最小静止 0.3 s，必要停顿 0.1 s | 38% | 14% | 0.27 s |

保留下来的静止帧，大多是每段动作起步和收尾时的减速帧；如果把它们也切掉，动作就会带着速度开始或结束。要对已有数据重新计算，或者换阈值扫描，在 `ersibench/source/data_collection` 目录下运行：

```bash
python -m common.sample_ranges <episode 目录>... [--sweep] [--write]
```

## 4. 生成视频

```bash
docker run --rm -v $O:/output ersibench-collect:latest video pick_biggest_block
```

输出在 `out/videos/pick_biggest_block/[pick_biggest_block_N].mp4`，画面从左到右是左腕、头部、右腕三路相机，30 fps，包含整条录制。

## 5. 自己写任务

1. 建一个工作目录：`mkdir -p work/scenes work/tasks`。
2. 按 `skills/create-task/SKILL.md`，从自带模板改出两个文件：
   - `work/scenes/<任务>.json`：场景、物体、机器人；
   - `work/tasks/<任务>/g2/<任务>.json`：动作阶段和成功判据。
3. 检查和预览：
   ```bash
   # 校验 json，并打印动作序列
   docker run --rm -v $PWD/work:/work ersibench-collect:latest check <任务>
   # 离线生成 10 个布局，几秒钟就好
   docker run --rm --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 -v $A:/assets:ro -v $O:/output \
       -v $PWD/work:/work ersibench-collect:latest layouts <任务> 10
   ```
4. 采集：在第 3 节的命令里加上 `-v $PWD/work:/work`，再把任务名换成你的任务。建议先加 `-e BATCH=3 -e MAX_ROUNDS=1` 做一次冒烟测试。

`skills/create-task/SKILL.md` 里有一个已经实测采集成功的例子：`place_smallest_block`，把最小的积木放到目标位置。用 Claude Code 之类的 agent 时，让它读 `skills/create-task/SKILL.md` 和 `skills/collect/SKILL.md`，再用一句话描述你的任务就行。

## 6. 常见问题

| 现象 | 原因和处理 |
|---|---|
| 构建卡在 ROS 那一步，或者 `apt update` 报 404 | 访问不了 GitHub（`ros2-apt-source`），或者 ROS 镜像源有问题。换个网络重试，或者换一个 ROS 镜像源 |
| 构建卡在克隆 cuRobo | 访问不了 GitHub。用 `--build-arg CUROBO_GIT=<镜像地址>` 换成可访问的地址 |
| 容器几秒就退出，并打印用法 | 任务名写错了、`/assets` 里没有 `pyproject.toml`，或者没挂 `/output`。看用法上面那一行报错 |
| `collection did not start` | GPU 被占用或者看不到。检查 `nvidia-smi` 和两处 GPU 参数，再看 `out/logs/<任务>/round_1/data_collector_server.log` |
| 每次都在第一次抓取就失败，server 日志里有 `changing goal type not supported in cuda graph mode` | 基础层缺了环境变量 `CUROBO_TORCH_CUDA_GRAPH_RESET=1`。用仓库里的 `base.Dockerfile` 重建基础层 |
| server 日志里有 `no kernel image is available` 之类的 CUDA 报错 | 这张卡的架构没编进 cuRobo。构建基础层时用 `CUDA_ARCHS` 加上这张卡的架构 |
| 一直是 `GRASP_FOLLOW_FAILED` | 抓取没抓住。偶尔出现是正常的；如果每次都这样，说明任务的抓取参数或物体尺寸不合适，见 create-task 里的说明 |
| 连续两轮成功数都是 0 | 是配置问题，不是运气不好。先看 server 日志里的失败原因，再改参数 |
| 相机画面变成黑边圆形 | 相机平移噪声把相机推进了外壳里。场景 json 里 `generalization.*.camera` 的 x/y/z 必须是 0 |
| `CUDA out of memory` | 显存不够。确认这张卡上没有别的程序在跑，至少要 24 GB |

## 7. 维护者：同步代码和构建镜像

```bash
./sync_from_ersibench.sh ~/Repos/ERSI-Bench <commit>   # 从 ERSI-Bench 同步 ersibench/ 快照，不要直接改快照
docker build -t ersibench-collect:latest .             # 默认以 ersibench-collect-base 为基础层
                                                       # 有内部镜像时也可以用它：
                                                       #   --build-arg BASE=registry.agibot.com/genie-sim/geniesim3-data-collection:latest
```

如果要分发镜像文件（文件约 34 GB，超过 git 的大小限制，不放进仓库）：

```bash
docker tag ersibench-collect:latest ersibench-collect:$(cat ersibench/SOURCE_COMMIT)
docker save ersibench-collect:latest ersibench-collect:$(cat ersibench/SOURCE_COMMIT) | zstd -T0 > ersibench-collect.tar.zst
sha256sum ersibench-collect.tar.zst > ersibench-collect.tar.zst.sha256   # 把 .sha256 提交到仓库
```

拿到文件的人先校验，再加载：

```bash
sha256sum -c ersibench-collect.tar.zst.sha256
zstd -dc ersibench-collect.tar.zst | docker load
```

注意：仓库里现有的那份 `.sha256`，对应的是 2026-10-19 基于内部镜像导出的文件。那份文件里的 cuRobo 只编译了 sm_120，只能在 RTX 50 系上做规划。
