#!/usr/bin/env python3
# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""Generate layouts with client/layout and preview them in Isaac Sim.

Does **not** run trajectory collection / recording. Requires a running
data_collection Isaac gRPC server (see --help / docs for how to start it).

Examples
--------
# Layout only (no Isaac)
python scripts/generate_layout.py --layout-only \\
  --task-template tasks/pick_biggest_block/g2/pick_biggest_block.json \\
  --output-dir /path/to/ERSIBench/output

# GUI preview (Isaac window stays open; press Enter between instances)
python scripts/generate_layout.py --gui \\
  --task-template tasks/pick_biggest_block/g2/pick_biggest_block.json \\
  --output-dir /path/to/ERSIBench/output --num-episodes 2

# Headless: load + save camera PNGs
python scripts/generate_layout.py --headless --save-images \\
  --task-template tasks/pick_biggest_block/g2/pick_biggest_block.json \\
  --output-dir /path/to/ERSIBench/output --num-episodes 2
"""

from __future__ import annotations

import argparse
import copy
import ctypes
import errno
import fcntl
import json
import math
import os
import re
import secrets
import stat
import sys
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path
from urllib.parse import urlsplit

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.base_utils.logger import logger
from common.task_file import load_task


TaskGenerator = None


def _get_task_generator_class():
    global TaskGenerator
    if TaskGenerator is None:
        from client.layout.task_generate import TaskGenerator as task_generator_class

        TaskGenerator = task_generator_class
    return TaskGenerator


def _import_isaac_client():
    """Lazy import: pinocchio / gRPC only needed when talking to Isaac."""
    from client.agent.omniagent import DataCollectionAgent
    from client.robot.client import RpcConnectionError
    from client.robot.omni_robot import IsaacSimRpcRobot
    from common.aimdk.protocol.sim import (
        sim_observation_service_pb2,
        sim_observation_service_pb2_grpc,
    )

    return (
        DataCollectionAgent,
        IsaacSimRpcRobot,
        sim_observation_service_pb2,
        sim_observation_service_pb2_grpc,
        RpcConnectionError,
    )

CAMERA_ALIAS = {
    "head_front_Camera": "head",
    "Left_Camera": "left_hand",
    "Right_Camera": "right_hand",
    "head_left_Camera": "head_left",
    "head_right_Camera": "head_right",
}


class PreviewError(Exception):
    """Expected preview CLI error that can be shown without a traceback."""


def positive_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected a positive number: {value}") from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError(f"expected a positive number: {value}")
    return parsed


def positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"expected a positive integer: {value}"
        ) from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError(f"expected a positive integer: {value}")
    return parsed


def parse_grpc_endpoint(client_host: str) -> tuple[str, int]:
    error = f"Invalid gRPC endpoint {client_host!r}; expected HOST:PORT"
    try:
        parsed = urlsplit(f"//{client_host}")
        port = parsed.port
    except ValueError as exc:
        raise ValueError(error) from exc
    invalid_components = (
        parsed.username is not None
        or parsed.password is not None
        or bool(parsed.path)
        or bool(parsed.query)
        or bool(parsed.fragment)
    )
    if (
        any(char.isspace() for char in client_host)
        or "?" in client_host
        or "#" in client_host
        or invalid_components
        or not parsed.hostname
        or port is None
        or not 1 <= port <= 65535
    ):
        raise ValueError(error)
    return parsed.hostname, port


def require_server(client_host: str, timeout: float) -> None:
    try:
        parse_grpc_endpoint(client_host)
    except ValueError as exc:
        raise PreviewError(str(exc)) from exc
    import grpc

    channel = None
    try:
        channel = grpc.insecure_channel(client_host)
        grpc.channel_ready_future(channel).result(timeout=timeout)
    except (grpc.FutureTimeoutError, grpc.RpcError, OSError) as exc:
        raise PreviewError(
            f"Cannot connect to preview server at {client_host}. Start it with: "
            "python scripts/data_collector_server.py --enable_physics"
        ) from exc
    finally:
        if channel is not None:
            channel.close()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "--task-template",
        type=Path,
        default=ROOT / "tasks/pick_biggest_block/g2/pick_biggest_block.json",
        help="data_collection task template JSON",
    )
    p.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "output",
        help="Where to write generated layout JSONs and preview images",
    )
    p.add_argument("--num-episodes", type=positive_int, default=None, help="Override template num_of_episode")
    p.add_argument(
        "--output-task-name",
        type=str,
        default=None,
        help="Override output subdir and JSON filename prefix (default: template 'task' field)",
    )
    p.add_argument("--skip-generate", action="store_true", help="Reuse existing layouts under output-dir/<task>/")
    p.add_argument("--layout-only", action="store_true", help="Only generate layouts; do not connect to Isaac")
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--gui", action="store_true", default=False, help="Interactive Isaac GUI preview (default if not --headless)")
    mode.add_argument("--headless", action="store_true", help="Load scenes without waiting for Enter; exit after done")
    p.add_argument("--save-images", action="store_true", help="Capture robot cameras to PNG under output-dir/<task>/preview/")
    p.add_argument(
        "--cameras",
        type=str,
        default="head,left_hand,right_hand",
        help="Comma aliases (head,left_hand,right_hand) mapped from template camera_list",
    )
    p.add_argument("--client-host", type=str, default="localhost:50051")
    p.add_argument(
        "--connect-timeout",
        type=positive_float,
        default=5.0,
        help="Seconds to wait for the Isaac gRPC server",
    )
    p.add_argument(
        "--settle-seconds",
        type=positive_float,
        default=5.0,
        help="Seconds to wait after loading a layout so dropped objects can settle",
    )
    p.add_argument(
        "--instance-ids",
        type=str,
        default="",
        help="Comma indices to preview (default: all generated/found)",
    )
    p.add_argument(
        "--keep-absolute-assets",
        action="store_true",
        help="Do not rewrite data_info_dir to paths relative to $SIM_ASSETS (needed for Docker server)",
    )
    return p.parse_args()


def ensure_sim_assets() -> Path:
    root = os.environ.get("SIM_ASSETS")
    if not root:
        try:
            import geniesim_assets

            root = os.path.dirname(geniesim_assets.__file__)
            os.environ["SIM_ASSETS"] = root
        except ImportError as e:
            raise PreviewError(
                "SIM_ASSETS is unset and geniesim_assets is not importable. "
                "export SIM_ASSETS=/path/to/geniesim_assets or pip install -e geniesim_assets"
            ) from e
    path = Path(root).resolve()
    if not path.is_dir():
        raise PreviewError(f"SIM_ASSETS does not exist: {path}")
    return path


def rewrite_asset_paths_relative(task_info: dict, assets_root: Path) -> dict:
    """Rewrite absolute asset dirs to paths relative to assets_root for the Isaac server."""
    root = str(assets_root.resolve())
    keys = ("data_info_dir", "obj_path", "model_path", "original_model_path")
    marker = "/geniesim_assets/"
    for obj in task_info.get("objects", []):
        for key in keys:
            val = obj.get(key)
            if not val or not isinstance(val, str):
                continue
            normalized = val.replace("\\", "/")
            if marker in normalized:
                obj[key] = normalized.split(marker, 1)[1]
                continue
            abs_val = str(Path(val).resolve()) if os.path.isabs(val) else str((assets_root / val).resolve())
            if abs_val == root or abs_val.startswith(root + os.sep):
                obj[key] = os.path.relpath(abs_val, root)
    return task_info


def resolve_output_root(output_dir: Path, *, create: bool = False) -> Path:
    output_path = output_dir.expanduser().absolute()
    if output_path.is_symlink():
        raise PreviewError(f"Preview output root must not be a symlink: {output_path}")
    if create:
        output_path.mkdir(parents=True, exist_ok=True)
    if output_path.is_symlink():
        raise PreviewError(f"Preview output root must not be a symlink: {output_path}")
    output_root = output_path.resolve()
    if output_path != output_root:
        raise PreviewError(f"Preview output root must not contain symlinks: {output_path}")
    return output_root


def resolve_save_path(output_dir: Path, task_name: object) -> Path:
    if (
        not isinstance(task_name, str)
        or not task_name.strip()
        or task_name in {".", ".."}
        or "/" in task_name
        or "\\" in task_name
        or Path(task_name).is_absolute()
    ):
        raise PreviewError(f"Invalid task name for preview output: {task_name!r}")

    output_root = resolve_output_root(output_dir)
    unresolved_save_path = output_root / task_name
    if unresolved_save_path.is_symlink():
        raise PreviewError(
            f"Task output must not be a symlink: {unresolved_save_path}"
        )
    save_path = unresolved_save_path.resolve()
    if save_path == output_root or output_root not in save_path.parents:
        raise PreviewError(
            f"Resolved task output must be inside output directory: {save_path}"
        )
    return save_path


def discover_layout_files(save_path: Path, task_name: str) -> list[Path]:
    if not save_path.is_dir():
        return []
    pattern = re.compile(rf"{re.escape(task_name)}_(\d+)\.json")
    matches = []
    for path in save_path.iterdir():
        match = pattern.fullmatch(path.name)
        if match and path.is_file():
            matches.append((int(match.group(1)), path.name, path))
    return [path for _, _, path in sorted(matches)]


@contextmanager
def layout_lock(root_fd: int, task_name: str):
    lock_name = f".{task_name}.preview.lock"
    flags = os.O_CREAT | os.O_RDWR | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(lock_name, flags, 0o600, dir_fd=root_fd)
    except OSError as exc:
        raise PreviewError(f"Cannot open preview generation lock: {lock_name}") from exc

    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise PreviewError(
                f"Layout generation already in progress for task {task_name!r}"
            ) from exc
        yield
    finally:
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
        except OSError:
            pass
        try:
            os.close(fd)
        except OSError:
            pass


def _rename_noreplace(
    old_dir_fd: int,
    old_name: str,
    new_dir_fd: int,
    new_name: str,
) -> None:
    try:
        renameat2 = ctypes.CDLL(None, use_errno=True).renameat2
    except AttributeError as exc:
        raise PreviewError(
            "Atomic no-replace layout publishing is unavailable on this platform"
        ) from exc

    renameat2.argtypes = (
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    )
    renameat2.restype = ctypes.c_int
    rename_noreplace = 1
    if renameat2(
        old_dir_fd,
        os.fsencode(old_name),
        new_dir_fd,
        os.fsencode(new_name),
        rename_noreplace,
    ) == 0:
        return

    error_number = ctypes.get_errno()
    if error_number == errno.EEXIST:
        raise PreviewError(f"Layout destination already exists: {new_name}")

    # renameat2(RENAME_NOREPLACE) is unsupported on Docker overlay, FUSE, NFS.
    # Fall back to existence check + plain os.rename through /proc/self/fd/<fd>.
    try:
        os.stat(new_name, dir_fd=new_dir_fd, follow_symlinks=False)
        raise PreviewError(f"Layout destination already exists: {new_name}")
    except FileNotFoundError:
        pass

    old_path = os.path.join(f"/proc/self/fd/{old_dir_fd}", os.fsdecode(old_name))
    new_path = os.path.join(f"/proc/self/fd/{new_dir_fd}", os.fsdecode(new_name))
    os.rename(old_path, new_path)


def _remove_directory_contents(dir_fd: int) -> None:
    for entry in os.scandir(dir_fd):
        if entry.is_dir(follow_symlinks=False):
            child_fd = os.open(
                entry.name,
                os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0),
                dir_fd=dir_fd,
            )
            try:
                _remove_directory_contents(child_fd)
            finally:
                os.close(child_fd)
            os.rmdir(entry.name, dir_fd=dir_fd)
        else:
            os.unlink(entry.name, dir_fd=dir_fd)


@contextmanager
def staging_directory(root_fd: int, task_name: str):
    for _ in range(10):
        staging_name = f".{task_name}.preview-{secrets.token_hex(8)}"
        try:
            os.mkdir(staging_name, mode=0o700, dir_fd=root_fd)
            break
        except FileExistsError:
            continue
    else:
        raise PreviewError("Could not allocate a unique layout staging directory")

    staging_fd = None
    try:
        staging_fd = os.open(
            staging_name,
            os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=root_fd,
        )
        os.mkdir(task_name, mode=0o700, dir_fd=staging_fd)
        yield staging_name, staging_fd
    finally:
        if staging_fd is not None:
            try:
                _remove_directory_contents(staging_fd)
            except OSError:
                pass
            try:
                os.close(staging_fd)
            except OSError:
                pass
        try:
            os.rmdir(staging_name, dir_fd=root_fd)
        except OSError:
            pass


def output_root_identity(output_dir: Path) -> tuple[int, int]:
    output_path = output_dir.expanduser().absolute()
    try:
        current = os.stat(output_path, follow_symlinks=False)
    except OSError as exc:
        raise PreviewError(f"Preview output root identity changed: {output_path}") from exc
    if not stat.S_ISDIR(current.st_mode):
        raise PreviewError(f"Preview output root identity changed: {output_path}")
    return current.st_dev, current.st_ino


def require_positive_episode_count(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise PreviewError(f"Episode count must be a positive integer: {value!r}")
    return value


def resolve_output_task_name(task_info: dict, output_task_name: str | None) -> str:
    task_name = output_task_name or task_info["task"]
    if (
        not isinstance(task_name, str)
        or not task_name.strip()
        or task_name in {".", ".."}
        or "/" in task_name
        or "\\" in task_name
        or Path(task_name).is_absolute()
    ):
        raise PreviewError(f"Invalid output task name: {task_name!r}")
    return task_name


def generate_layouts(
    template_path: Path,
    output_dir: Path,
    num_episodes: int | None,
    output_task_name: str | None = None,
) -> tuple[dict, Path, list[Path]]:
    task_info = load_task(template_path)  # task json + its scene json
    task_name = resolve_output_task_name(task_info, output_task_name)
    if num_episodes is None:
        num_episodes = task_info.get("recording_setting", {}).get("num_of_episode", 1)
    num_episodes = require_positive_episode_count(num_episodes)
    output_root = resolve_output_root(output_dir, create=True)
    save_path = resolve_save_path(output_dir, task_name)
    root_fd = os.open(
        output_root,
        os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0),
    )
    root_stat = os.fstat(root_fd)
    root_identity = (root_stat.st_dev, root_stat.st_ino)
    try:
        if output_root_identity(output_dir) != root_identity:
            raise PreviewError(f"Preview output root identity changed: {output_root}")
        with layout_lock(root_fd, task_name):
            try:
                os.stat(task_name, dir_fd=root_fd, follow_symlinks=False)
            except FileNotFoundError:
                pass
            else:
                raise PreviewError(
                    f"Layout destination already exists: {save_path}. "
                    "Use --skip-generate to reuse it or choose a new --output-dir."
                )

            logger.info(f"Generating {num_episodes} layouts -> {save_path}")
            tg = _get_task_generator_class()(copy.deepcopy(task_info))
            with staging_directory(root_fd, task_name) as (staging_name, staging_fd):
                task_fd_path = Path(f"/proc/self/fd/{staging_fd}") / task_name
                for episode_id in range(num_episodes):
                    output_file = task_fd_path / f"{task_name}_{episode_id}.json"
                    for _ in range(5):
                        if tg.generate(str(output_file)):
                            break
                    else:
                        raise PreviewError(
                            f"Failed to generate layout {episode_id} after 5 attempts"
                        )

                if output_root_identity(output_dir) != root_identity:
                    raise PreviewError(
                        f"Preview output root identity changed: {output_root}"
                    )
                try:
                    os.stat(task_name, dir_fd=root_fd, follow_symlinks=False)
                except FileNotFoundError:
                    pass
                else:
                    raise PreviewError(
                        f"Layout destination already exists: {save_path}"
                    )
                _rename_noreplace(staging_fd, task_name, root_fd, task_name)

            files = discover_layout_files(save_path, task_name)
            return task_info, save_path, files
    finally:
        try:
            os.close(root_fd)
        except OSError:
            pass


def discover_layouts(
    template_path: Path,
    output_dir: Path,
    output_task_name: str | None = None,
) -> tuple[dict, Path, list[Path]]:
    task_info = load_task(template_path)  # task json + its scene json
    task_name = resolve_output_task_name(task_info, output_task_name)
    save_path = resolve_save_path(output_dir, task_name)
    validated_task_name = save_path.name
    files = discover_layout_files(save_path, validated_task_name)
    if not files:
        raise FileNotFoundError(f"No layouts in {save_path}; omit --skip-generate first")
    return task_info, save_path, files


def select_files(files: list[Path], instance_ids: str) -> list[Path]:
    if not instance_ids.strip():
        return files
    wanted = {int(x) for x in instance_ids.split(",") if x.strip() != ""}
    selected = []
    for path in files:
        stem = path.stem
        idx_str = stem.rsplit("_", 1)[-1]
        if idx_str.isdigit() and int(idx_str) in wanted:
            selected.append(path)
    if not selected:
        raise ValueError(f"No layouts matched --instance-ids={instance_ids}")
    return selected


def resolve_camera_prims(template: dict, aliases: str) -> list[tuple[str, str]]:
    """Return list of (alias, prim_path) from template recording_setting.camera_list."""
    camera_list = template.get("recording_setting", {}).get("camera_list", [])
    wanted = {a.strip() for a in aliases.split(",") if a.strip()}
    resolved = []
    for prim in camera_list:
        leaf = prim.rstrip("/").split("/")[-1]
        alias = CAMERA_ALIAS.get(leaf, leaf)
        if alias in wanted or leaf in wanted:
            resolved.append((alias, prim))
    if not resolved and camera_list:
        # fallback: first cameras matching common names
        for prim in camera_list:
            leaf = prim.rstrip("/").split("/")[-1]
            alias = CAMERA_ALIAS.get(leaf, leaf)
            if alias in ("head", "left_hand", "right_hand"):
                resolved.append((alias, prim))
    return resolved


def capture_cameras(robot, camera_prims: list[str]) -> list[np.ndarray | None]:
    _, _, sim_observation_service_pb2, sim_observation_service_pb2_grpc, _ = (
        _import_isaac_client()
    )
    stub = sim_observation_service_pb2_grpc.SimObservationServiceStub(robot.client.channel)
    req = sim_observation_service_pb2.GetObservationReq()
    req.isCam = True
    req.CameraReq.render_depth = False
    req.CameraReq.render_semantic = False
    for prim in camera_prims:
        req.CameraReq.camera_prim_list.append(prim)
    rsp = stub.get_observation(req)
    images: list[np.ndarray | None] = []
    for cam in rsp.camera:
        w = int(cam.camera_info.width) or 0
        h = int(cam.camera_info.height) or 0
        raw = cam.rgb_camera.data
        if not raw or w <= 0 or h <= 0:
            images.append(None)
            continue
        arr = np.frombuffer(raw, dtype=np.uint8)
        if arr.size == h * w * 4:
            img = arr.reshape(h, w, 4)[:, :, :3]
        elif arr.size == h * w * 3:
            img = arr.reshape(h, w, 3)
        else:
            logger.warning(f"Unexpected RGB buffer size {arr.size} for {w}x{h}")
            images.append(None)
            continue
        images.append(img.copy())
    return images


def save_preview_images(
    robot,
    cameras: list[tuple[str, str]],
    out_dir: Path,
    instance_stem: str,
) -> dict[str, str]:
    try:
        import cv2
    except ImportError as e:
        raise PreviewError("opencv-python (cv2) is required for --save-images") from e

    out_dir.mkdir(parents=True, exist_ok=True)
    prims = [p for _, p in cameras]
    images = capture_cameras(robot, prims)
    aliases = [alias for alias, _ in cameras]
    if len(images) != len(cameras):
        raise PreviewError(
            f"Camera response count mismatch for aliases {aliases}: "
            f"expected {len(cameras)}, got {len(images)}"
        )
    missing = [alias for (alias, _), image in zip(cameras, images) if image is None]
    if missing:
        raise PreviewError(f"Missing camera images for aliases: {', '.join(missing)}")

    pending = []
    with tempfile.TemporaryDirectory(prefix=".preview-images-", dir=out_dir) as temp_dir:
        temp_path = Path(temp_dir)
        for (alias, _prim), img in zip(cameras, images):
            idx = instance_stem.rsplit("_", 1)[-1]
            name = f"{alias}.png"
            if idx.isdigit() and int(idx) != 0:
                name = f"{alias}_{int(idx):02d}.png"
            final_path = out_dir / name
            staged_path = temp_path / name
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            try:
                saved = cv2.imwrite(str(staged_path), bgr)
            except Exception as exc:
                raise PreviewError(
                    f"Failed to write preview image: {final_path}"
                ) from exc
            if not saved:
                raise PreviewError(f"Failed to write preview image: {final_path}")
            pending.append((alias, staged_path, final_path))

        for _alias, staged_path, final_path in pending:
            os.replace(staged_path, final_path)

    written = {alias: str(final_path) for alias, _, final_path in pending}
    for path in written.values():
        logger.info(f"Saved {path}")
    return written


def build_robot(template: dict, client_host: str, connect_timeout: float):
    deadline = time.monotonic() + connect_timeout
    _, IsaacSimRpcRobot, _, _, _ = _import_isaac_client()
    tg = _get_task_generator_class()(template)
    robot_position = tg.robot_init_pose["position"]
    robot_rotation = tg.robot_init_pose["quaternion"]
    stand = {"stand_type": "cylinder", "stand_size_x": 0.1, "stand_size_y": 0.1}
    if "stand" in template.get("robot", {}):
        stand = template["robot"]["stand"]
    robot_cfg = template["robot"]["robot_cfg"]
    scene_usd = template["scene"]["scene_usd"]
    if isinstance(scene_usd, list):
        scene_usd = scene_usd[0]
    robot_init_arm_pose = template["robot"].get("init_arm_pose")
    robot_init_arm_pose_noise = template["robot"].get("init_arm_pose_noise")
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise PreviewError("Preview connection timeout expired before robot construction")
    return IsaacSimRpcRobot(
        robot_cfg=robot_cfg,
        scene_usd=scene_usd,
        client_host=client_host,
        connect_timeout=remaining,
        position=robot_position,
        rotation=robot_rotation,
        stand_type=stand["stand_type"],
        stand_size_x=stand["stand_size_x"],
        stand_size_y=stand["stand_size_y"],
        robot_init_arm_pose=robot_init_arm_pose,
        robot_init_arm_pose_noise=robot_init_arm_pose_noise,
    )


def prepare_instance_file(
    src: Path,
    assets_root: Path,
    rewrite: bool,
    temporary_dir: Path,
) -> Path:
    """Optionally rewrite asset paths into a caller-owned temporary directory."""
    if not rewrite:
        return src
    task_info = load_task(src)
    rewrite_asset_paths_relative(task_info, assets_root)
    out = temporary_dir / f"{src.stem}_server.json"
    with open(out, "w", encoding="utf-8") as f:
        json.dump(task_info, f, indent=2)
    return out


def preview_instances(
    template: dict,
    files: list[Path],
    *,
    client_host: str,
    connect_timeout: float,
    gui: bool,
    save_images: bool,
    cameras: list[tuple[str, str]],
    preview_dir: Path,
    assets_root: Path,
    rewrite_assets: bool,
    settle_seconds: float = 5.0,
) -> int:
    deadline = time.monotonic() + connect_timeout
    DataCollectionAgent, _, _, _, RpcConnectionError = _import_isaac_client()

    logger.info(f"Connecting to Isaac server at {client_host}")
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise PreviewError("Preview connection timeout expired before robot construction")
    try:
        robot = build_robot(template, client_host, remaining)
    except RpcConnectionError as exc:
        raise PreviewError(
            f"Cannot connect to preview server at {client_host}. Start it with: "
            "python scripts/data_collector_server.py --enable_physics"
        ) from exc

    written_count = 0
    try:
        with tempfile.TemporaryDirectory(prefix="geniesim-preview-") as temp_dir:
            temporary_dir = Path(temp_dir)
            agent = DataCollectionAgent(robot)
            for i, path in enumerate(files):
                load_path = prepare_instance_file(
                    path, assets_root, rewrite_assets, temporary_dir
                )
                logger.info(f"[{i+1}/{len(files)}] Loading layout {path.name}")
                agent.reset()
                time.sleep(0.5)
                agent.generate_layout(str(load_path))
                logger.info(
                    f"Waiting {settle_seconds:.1f}s for objects to settle after drop"
                )
                time.sleep(settle_seconds)

                if save_images and cameras:
                    written_count += len(
                        save_preview_images(robot, cameras, preview_dir, path.stem)
                    )

                if gui:
                    print(
                        f"\n=== Preview ready: {path.name} ===\n"
                        f"Inspect the Isaac Sim window, then press Enter for next "
                        f"(or Ctrl-C to stop)...",
                        flush=True,
                    )
                    try:
                        input()
                    except EOFError:
                        break
                else:
                    logger.info(f"Headless preview done for {path.name}")
        return written_count
    finally:
        try:
            channel = getattr(robot.client, "channel", None)
            if channel is not None:
                channel.close()
        except Exception:
            pass


def main() -> int:
    args = parse_args()
    assets_root = ensure_sim_assets()
    template_path = args.task_template.expanduser().resolve()
    output_dir = args.output_dir.expanduser().absolute()
    resolve_output_root(output_dir, create=True)

    if not args.gui and not args.headless and not args.layout_only:
        args.gui = True  # default interactive

    if args.skip_generate:
        output_task_name = getattr(args, "output_task_name", None)
        if output_task_name is None:
            template, save_path, files = discover_layouts(template_path, output_dir)
        else:
            template, save_path, files = discover_layouts(
                template_path, output_dir, output_task_name
            )
    else:
        template, save_path, files = generate_layouts(
            template_path, output_dir, args.num_episodes, args.output_task_name
        )

    try:
        files = select_files(files, args.instance_ids)
    except ValueError as exc:
        raise PreviewError(str(exc)) from exc
    logger.info(f"Layouts ({len(files)}): {[p.name for p in files]}")

    if args.layout_only:
        print(f"Layout-only done. Saved under {save_path}")
        return 0

    connection_deadline = time.monotonic() + args.connect_timeout
    require_server(args.client_host, args.connect_timeout)
    remaining = connection_deadline - time.monotonic()
    if remaining <= 0:
        raise PreviewError("Preview connection timeout expired before robot construction")

    cameras = resolve_camera_prims(template, args.cameras)
    if (args.headless or args.save_images) and not cameras:
        raise PreviewError("No cameras resolved from template for image preview")

    preview_dir = save_path / "preview"
    written_count = preview_instances(
        template,
        files,
        client_host=args.client_host,
        connect_timeout=remaining,
        gui=bool(args.gui and not args.headless),
        save_images=args.save_images or args.headless,
        cameras=cameras,
        preview_dir=preview_dir,
        assets_root=assets_root,
        rewrite_assets=not args.keep_absolute_assets,
        settle_seconds=args.settle_seconds,
    )
    print(f"Done. Layouts: {save_path}")
    if written_count:
        print(f"Preview images: {preview_dir}")
    return 0


def run_cli() -> None:
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130)
    except PreviewError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    run_cli()
