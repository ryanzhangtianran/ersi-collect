# ERSI-Collect 使用说明

ERSI-Collect 把 ERSI-Bench 的数据采集打包成一个 Docker 镜像 `ersibench-collect`：

- 在一张 GPU 上用 Isaac Sim 采集 G2 双臂机器人的操作轨迹；
- 一轮轮采集，直到成功的 episode 数达到目标；
- 结束时自动转成 LeRobot v2.1 数据集；
- 可以给每条 episode 生成一段三路相机拼接的视频。

宿主机上只需要 Docker 和 NVIDIA 驱动，不需要装 conda、Isaac Sim 或源码。镜像自带一个任务 `pick_biggest_block`（拿起最大的积木）；其他任务按 `skills/create-task/SKILL.md` 自己写，放在挂载进容器的目录里，不用重新构建镜像。

## 1. 环境要求

| 项目 | 要求 |
|---|---|
| 系统 | 任意 Linux 发行版，Ubuntu 20.04 / 22.04 / 24.04 都可以（镜像内部是 24.04） |
| GPU | RTX 40 系、L40（Ada）或 RTX 50 系（Blackwell），显存 ≥ 24 GB。cuRobo 只编译了 sm_89 / sm_120，A100、3090、H100 用不了 |
| 驱动 | NVIDIA 驱动 ≥ 570（CUDA 12.8） |
| 磁盘 | 镜像约 110 GB（解压后）；assets 约 47 GB；输出目录至少留 80 GB（单次尝试的原始录制最多约 10 GB，处理完会删） |

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

### 2.2 加载镜像

镜像包含 Isaac Sim，压缩后约 30 GB，不放在公开的镜像仓库里，需要向维护者要 `ersibench-collect.tar.zst`。拿到后加载：

```bash
sudo apt install -y zstd
zstd -dc ersibench-collect.tar.zst | docker load
docker run --rm --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 --entrypoint nvidia-smi ersibench-collect:latest -L   # 应列出一张卡
```

如果手上有基础镜像 `geniesim3-data-collection:latest`，也可以自己构建（见第 7 节）。

### 2.3 下载 assets

assets 是 GenieSim 的物体、场景和机器人模型库，托管在 ModelScope，用 git-lfs 下载，约 47 GB：

```bash
git clone https://github.com/ryanzhangtianran/ersi-collect.git && cd ersi-collect
sudo apt install -y git-lfs
./download_assets.sh                      # 下载到 ./geniesim_assets；中断后重跑会接着下
./download_assets.sh /data/geniesim_assets   # 或者下载到别的位置
```

最后一行显示 `0 still pointers` 才算下载完整。

### 2.4 验证

```bash
docker run --rm ersibench-collect:latest list
# tasks: ['pick_biggest_block']  ...  backgrounds with a table: [...]
```

## 3. 采集

```bash
A=$PWD/geniesim_assets      # assets 目录
O=$PWD/out; mkdir -p $O     # 输出目录
docker run -d --name ersi_pick --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 \
    -v ersi-isaac-cache:/isaac-sim/.cache -v ersi-isaac-cc:/isaac-sim/.nv/ComputeCache \
    -v $A:/assets:ro -v $O:/output \
    ersibench-collect:latest pick_biggest_block 50
docker logs -f ersi_pick     # 看进度：round R done: +g, d / 50
```

- 最后两个参数是任务名和目标条数：采到 50 条成功的 episode 才结束。
- `--gpus device=N` 和 `NVIDIA_VISIBLE_DEVICES=N` 必须写同一张卡。
- 两个 `ersi-isaac-cache` 卷用来保存 shader 缓存，可以不挂；挂上之后，第二次启动能省几分钟。
- 每轮要先启动 Isaac Sim，大约 2–3 分钟；之后每次尝试 2–5 分钟。
- 可选环境变量（用 `-e` 传入）：

| 变量 | 默认值 | 作用 |
|---|---|---|
| `BATCH` | 20 | 每轮最多尝试几次 |
| `MAX_ROUNDS` | 25 | 最多跑几轮 |
| `CONVERT` | 1 | 设成 0 就不转 LeRobot |
| `ERSIBENCH_OOD_AXES` | 不设 | 采 OOD 数据，例如 `light,camera_pose` |

- 停止：`docker rm -f ersi_pick`。已经成功的数据会保留；用同样的命令重新运行，会接着已有的条数继续采。

输出目录结构：

| 路径 | 内容 |
|---|---|
| `out/recording_data/<任务>/[<任务>_N]/` | 成功的 episode（关节 h5、三路相机图片、状态） |
| `out/saved_task/<任务>/` | 每条 episode 对应的场景布局 |
| `out/failed/<任务>/` | 失败的 episode，留着排查用 |
| `out/lerobot/<任务>/` | LeRobot v2.1 数据集（每次运行结束后重新生成） |
| `out/logs/<任务>/` | 每轮的 server / client 日志 |

## 4. 生成视频

```bash
docker run --rm -v $O:/output ersibench-collect:latest video pick_biggest_block
# 输出 out/videos/pick_biggest_block/[pick_biggest_block_N].mp4：左腕 | 头部 | 右腕，30 fps
```

## 5. 自己写任务

1. 新建一个工作目录：`mkdir -p work/scenes work/tasks`。
2. 按 `skills/create-task/SKILL.md` 从自带模板改出：
   - `work/scenes/<任务>.json`：场景、物体、机器人；
   - `work/tasks/<任务>/g2/<任务>.json`：动作阶段、成功判据。
3. 检查和预览：
   ```bash
   docker run --rm -v $PWD/work:/work ersibench-collect:latest check <任务>        # 校验 json，打印动作序列
   docker run --rm --gpus device=0 -e NVIDIA_VISIBLE_DEVICES=0 -v $A:/assets:ro -v $O:/output \
       -v $PWD/work:/work ersibench-collect:latest layouts <任务> 10              # 离线生成 10 个布局，几秒钟
   ```
4. 采集：在第 3 节的命令里加上 `-v $PWD/work:/work`，再把任务名换成你的任务。建议先用 `-e BATCH=3 -e MAX_ROUNDS=1` 做一次冒烟测试。

用 Claude Code 等 agent 时，直接让它读 `skills/create-task/SKILL.md` 和 `skills/collect/SKILL.md`，一句话描述任务即可。

## 6. 常见问题

| 现象 | 原因和处理 |
|---|---|
| 容器几秒就退出，并打印用法 | 任务名写错、`/assets` 里没有 `pyproject.toml`，或者没挂 `/output`。看用法上面那行报错 |
| `collection did not start` | GPU 被占用或看不到。检查 `nvidia-smi` 和两处 GPU 参数，再看 `out/logs/<任务>/round_1/data_collector_server.log` |
| 一直是 `GRASP_FOLLOW_FAILED` | 抓取没抓住。偶尔出现是正常的；如果每次都这样，说明任务的抓取参数或物体尺寸不合适（见 create-task 的说明） |
| 连续两轮成功数都是 0 | 配置问题，不是运气。先看 server 日志里的失败原因，再改参数 |
| 相机画面变成黑边圆形 | 相机平移噪声把相机推进了外壳。场景 json 里 `generalization.*.camera` 的 x/y/z 必须是 0 |

## 7. 维护者：同步代码和构建镜像

```bash
./sync_from_ersibench.sh ~/Repos/ERSI-Bench [commit]   # 从 ERSI-Bench 同步 ersibench/ 快照；不要直接改快照
docker build -t ersibench-collect:latest .             # 需要本地有 geniesim3-data-collection:latest
docker save ersibench-collect:latest | zstd -T0 > ersibench-collect.tar.zst
```
