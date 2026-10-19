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

镜像包含 Isaac Sim，压缩后约 34 GB，超过 git 和 GitHub 的文件大小限制，所以不在仓库的 git 历史里。需要向维护者要 `ersibench-collect.tar.zst` 和 `ersibench-collect.tar.zst.sha256`：维护者那边这两个文件放在仓库根目录下，已经被 `.gitignore` 排除，不会提交。拿到后先校验，再加载：

```bash
sudo apt install -y zstd
sha256sum -c ersibench-collect.tar.zst.sha256     # 输出 OK 再继续
zstd -dc ersibench-collect.tar.zst | docker load  # 会加载 ersibench-collect:latest，以及对应 ERSI-Bench 提交号的标签
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

**静止段处理。** 原始 episode 保留所有帧，静止段不在采集时删除。提取完成后，每条 episode 会写一个 `sample_ranges.json`，记录训练要用的帧段（`common/sample_ranges.py`）。规则如下：

- **静止帧**：和前一帧相比，手臂关节位移 < 1e-4 rad，夹爪关节 < 1e-3 rad，所有物体位移 < 1 mm，三个条件都满足才算。所以物体还在动的帧（比如积木松手后还在落定）不算静止。
- **切段**：静止帧连续 ≥ 0.3 s 就在这里切开，切口两端各留 2 帧静止，保证每段都从静止起步、在静止收尾。短于 0.3 s 的停顿不切；切掉的部分不足 0.2 s 时也不切，不值得为这么一点把 episode 拆开。
- **必要停顿**：如果这段静止在夹爪开合指令或抓取附着变化（抓取、释放、接触）前后 0.33 s 内，两端改为各保留 0.2 s；最后一个动作结束后也保留 0.2 s（任务完成）。只保留这么多：之前是整段保护，两个事件挨得近时会把一整秒的静止都留下。
- **整段丢弃**：没有任何运动的段丢掉。

转 LeRobot 时，每个帧段单独作为一个 episode。段内的帧是连续的，没有时间跳变，所以 action chunk 也是连续的。chunk 超出段尾时用最后一帧补齐，这和实际情况一致：那里机器人确实是静止的。训练代码不需要任何改动。如果要一条录制对应一个 episode，转换器加 `--whole-episodes` 即可。

扫过的参数（6 条 episode，4901 帧）：

| 规则 | 被排除 | 保留帧里的静止占比 | 保留下来的最长停顿 |
|---|---|---|---|
| 旧：整段保护，最小静止 0.5 s | 30% | 23% | 1.10 s |
| 最小静止 0.5 s，必要停顿 0.2 s | 34% | 18% | 0.47 s |
| **最小静止 0.3 s，必要停顿 0.2 s（默认）** | 35% | 17% | 0.47 s |
| 最小静止 0.3 s，必要停顿 0.1 s | 38% | 14% | 0.27 s |

剩下的静止帧大多是每段动作起步、收尾时的减速帧。把它们也切掉，动作就会带着速度开始或结束，所以保留。对已有数据重算或者扫阈值：`python -m common.sample_ranges <episode 目录>... [--sweep] [--write]`（在 `ersibench/source/data_collection` 下运行）。

阶段之间原来固定等待 1 s，夹爪动作后固定等待 2 s，现在都去掉了，改成等夹爪真正停稳就继续，采一条也更快。

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
docker tag ersibench-collect:latest ersibench-collect:$(cat ersibench/SOURCE_COMMIT)
docker save ersibench-collect:latest ersibench-collect:$(cat ersibench/SOURCE_COMMIT) | zstd -T0 > ersibench-collect.tar.zst
sha256sum ersibench-collect.tar.zst > ersibench-collect.tar.zst.sha256
```
