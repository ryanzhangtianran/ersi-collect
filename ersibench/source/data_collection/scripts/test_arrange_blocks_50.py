"""Run bounded headless attempts locally, without archival, pruning, or renumbering."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time


DC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DC))
from common.task_file import load_task  # noqa: E402
REPO = DC.parents[1]
TASK = "arrange_blocks"
BEGIN = re.compile(r"^([^\n]*ATTEMPT_BEGIN (\d+)/(\d+)[^\n]*)$", re.M)


def read(path):
    return path.read_text(errors="replace") if path.exists() else ""


def parse_round(client, server, results):
    starts = list(BEGIN.finditer(client))
    rows = []
    for i, start in enumerate(starts):
        end = starts[i + 1].start() if i + 1 < len(starts) else len(client)
        block = client[start.start():end]
        layout = re.search(r"using layout \S*/(arrange_blocks_\d+)\.json", block)
        name = layout[1] if layout else None
        completed = "TASK SUCCESS !" in block
        metric = results.get(name)
        warnings = [line for line in block.splitlines() if re.search(r"WARNING|ERROR|Traceback|Error:", line)]
        since = start[1][:20]
        until = starts[i + 1][1][:20] if i + 1 < len(starts) else "9999"
        planner = [line for line in server.splitlines()
                   if since <= line[:20] < until and re.search(r"TRAJOPT_FAIL|IK_FAIL|Invalid Problem|Start or End state", line)]
        if completed:
            reason = "success" if metric and metric.get("task_status") is True else (
                "metric:" + str(metric.get("metric_status")) if metric else "missing_metric_result")
        elif "Reject layout" in block:
            reason = "auto_arm_preflight_failed"
        elif "check_completion fail" in block or "checker failed" in block.lower():
            reason = "execution_check_failed"
        elif "No grasp pose can pass next action IK" in block:
            reason = "next_action_ik_failed"
        elif "can pass Isaac IK" in block or "can pass isaac-sim IK" in block:
            reason = "lula_ik_failed"
        elif "TRAJOPT_FAIL" in "\n".join(planner):
            reason = "curobo_trajopt_failed"
        elif not name:
            reason = "layout_generation_failed"
        elif "Task " in block and " failed" in block:
            reason = "task_execution_failed"
        else:
            reason = "interrupted_or_process_error"
        rows.append(dict(attempt_in_round=int(start[2]), layout=name, execution_success=completed,
                         success=reason == "success", reason=reason, metric=metric,
                         warnings=warnings, planner_errors=planner))
    return rows


def snapshot(round_dir):
    destination = round_dir / "logs"
    destination.mkdir(exist_ok=True)
    for path in (DC / "logs" / TASK).glob("*.log"):
        shutil.copy2(path, destination / path.name)


def read_results(root):
    results = {}
    for path in root.glob("*/task_result.json"):
        try:
            results[path.parent.name.strip("[]")] = json.loads(path.read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            continue  # The recorder may still be writing the current episode.
    return results


def run(report, target_attempts=50, resume_from=None):
    previous = json.loads((resume_from / "summary.json").read_text()) if resume_from else {}
    previous_rows = previous.get("attempts", [])
    if target_attempts <= len(previous_rows):
        raise ValueError("The attempt target must exceed the completed attempts.")
    if previous.get("status") == "running":
        raise RuntimeError("Previous test is still marked running; do not overlap tests.")
    report.mkdir(parents=True, exist_ok=False)
    lock = open("/tmp/ersi-arrange-blocks-test.lock", "a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    running = subprocess.check_output(["docker", "ps", "--filter", "name=^/data_collection_open_source$",
                                       "--format", "{{.Names}}"], text=True).strip()
    if running:
        raise RuntimeError("An existing collection container is running; leave it untouched.")
    recording = DC / "recording_data" / TASK
    recording.mkdir(parents=True, exist_ok=True)
    baseline = {path.parent.name for path in recording.glob("*/aligned_joints.h5")}
    report.joinpath("baseline.json").write_text(json.dumps(sorted(baseline), indent=2))
    report.joinpath("code.patch").write_bytes(subprocess.check_output(["git", "diff", "--", "source/data_collection"], cwd=REPO))
    template = load_task(DC / "tasks/arrange_blocks/g2/arrange_blocks.json")  # task json + its scene json
    (report / "task_template.json").write_text(json.dumps(template, ensure_ascii=False, indent=4))
    state = dict(status="running", pid=os.getpid(), target_attempts=target_attempts, attempts_completed=0,
                 successes=0, attempts=previous_rows.copy(), rounds=[], baseline_episodes=len(baseline),
                 previous_report=str(resume_from) if resume_from else None,
                 recording_root=str(recording), archival="disabled",
                 phase_start_attempt=len(previous_rows) + 1,
                 commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip())

    def save():
        state["updated_at"] = datetime.now(timezone.utc).isoformat()
        state["attempts_completed"] = len(state["attempts"])
        state["successes"] = sum(row["success"] for row in state["attempts"])
        state["failure_reasons"] = dict(Counter(row["reason"] for row in state["attempts"] if not row["success"]))
        current = state["attempts"][len(previous_rows):]
        state["current_phase"] = dict(attempts_completed=len(current), successes=sum(row["success"] for row in current),
                                     failure_reasons=dict(Counter(row["reason"] for row in current if not row["success"])))
        temporary = report / "summary.json.tmp"
        temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2))
        temporary.replace(report / "summary.json")

    save()
    try:
        while len(state["attempts"]) < target_attempts:
            number = len(state["rounds"]) + 1
            round_dir = report / f"round_{number:02d}"
            round_dir.mkdir()
            completed_before_round = state["attempts"].copy()
            budget = min(5, target_attempts - len(state["attempts"]))
            template["recording_setting"].update(num_of_episode=budget, num_of_success=budget)
            task_path = round_dir / "task_template.json"
            task_path.write_text(json.dumps(template, ensure_ascii=False, indent=2))
            env = dict(os.environ, GENIESIM_RECORDING_ROOT=str(recording), GENIESIM_RECORDING_ARCHIVE=str(recording))
            state["current_round"] = number
            state["current_round_dir"] = str(round_dir)
            save()
            print(f"Round {number}: {budget} attempts, completed {len(state['attempts'])}/{target_attempts}; no archival", flush=True)
            with open(round_dir / "collect.log", "w") as output:
                wall_start = time.time()
                process = subprocess.Popen([os.environ.get("GENIESIM", "geniesim"), "autocollect", "run",
                                            str(task_path.relative_to(DC)), "--headless", "--standalone"],
                                           cwd=REPO, env=env, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
                started = time.monotonic()
                round_error = None
                while process.poll() is None:
                    if time.monotonic() - started > 3600:
                        os.killpg(process.pid, signal.SIGTERM)
                        try:
                            process.wait(timeout=30)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait()
                        round_error = "Round exceeded one hour; stopped instead of restarting indefinitely."
                        break
                    time.sleep(5)
                    client_path = DC / "logs" / TASK / "run_data_collection.log"
                    if client_path.exists() and client_path.stat().st_mtime >= wall_start:
                        client = read(client_path)
                        rows = parse_round(client, read(DC / "logs" / TASK / "data_collector_server.log"), read_results(recording))
                        state["attempts_started_in_round"] = len(rows)
                        # The next marker closes the preceding attempt; the last one is provisional.
                        for index, row in enumerate(rows[:-1]):
                            row.update(round=number, attempt=len(completed_before_round) + index + 1, report=str(report))
                        state["attempts"] = completed_before_round + rows[:-1]
                    save()
            snapshot(round_dir)
            results = read_results(recording)
            for name, metric in results.items():
                destination = round_dir / "results" / f"[{name}]"
                destination.mkdir(parents=True, exist_ok=True)
                (destination / "task_result.json").write_text(json.dumps(metric, indent=2))
                state_path = recording / f"[{name}]" / "state.json"
                if state_path.exists():
                    frames = json.loads(state_path.read_text()).get("frames", [])
                    if frames:
                        (destination / "last_frame.json").write_text(json.dumps(frames[-1]))
            client = read(round_dir / "logs/run_data_collection.log")
            client_path = round_dir / "logs/run_data_collection.log"
            if not client_path.exists() or client_path.stat().st_mtime < wall_start:
                client = ""
            rows = parse_round(client, read(round_dir / "logs/data_collector_server.log"), results)
            for index, row in enumerate(rows):
                row.update(round=number, attempt=len(completed_before_round) + index + 1, report=str(report))
            state["attempts"] = completed_before_round + rows
            state["rounds"].append(dict(round=number, returncode=process.returncode, attempts=len(rows)))
            save()
            print(f"Result: {state['successes']}/{state['attempts_completed']}; {state['failure_reasons']}", flush=True)
            if round_error:
                raise RuntimeError(round_error)
            if process.returncode != 0 or not rows or "job done" not in client:
                raise RuntimeError("Collection did not finish normally; inspect the saved round logs before restarting.")
        state["status"] = "complete"
    except Exception as error:
        state["status"] = "stopped_with_error"
        state["error"] = str(error)
        raise
    finally:
        if state.get("current_round_dir"):
            snapshot(Path(state["current_round_dir"]))
        save()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--attempts", type=int, default=50, help="Total attempt target, including the resumed report")
    parser.add_argument("--resume-from", type=Path, help="Stopped report whose attempts count toward the target")
    args = parser.parse_args()
    run(args.report_dir.resolve(), args.attempts, args.resume_from.resolve() if args.resume_from else None)
