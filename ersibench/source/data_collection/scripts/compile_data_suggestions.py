#!/usr/bin/env python3
"""Compile GPT data suggestions into reviewable autocollect task templates.

The compiler deliberately keeps qualitative perturbations in metadata until a
human assigns numeric ranges. Generated templates remain valid collection
templates and can be previewed with ``geniesim autocollect run --dry-run``.
"""

from __future__ import annotations

import argparse
import copy
import json
import re
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.task_file import load_task  # noqa: E402


def _positive(value: str) -> int:
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def _slug(value: str, fallback: str) -> str:
    text = re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_")
    return text[:48] or fallback


def compile_suggestions(suggestions_path: Path, template_path: Path, output_dir: Path) -> dict:
    source = json.loads(suggestions_path.read_text(encoding="utf-8"))
    template = load_task(template_path)  # task json + its scene json
    rows = source.get("data_suggestions", source.get("recommendations"))
    if not isinstance(rows, list) or not rows:
        raise ValueError("suggestions file must contain a non-empty data_suggestions list")
    output_dir.mkdir(parents=True, exist_ok=True)
    compiled = []
    for index, row in enumerate(rows, 1):
        if not isinstance(row, dict):
            raise ValueError(f"suggestion {index} must be an object")
        count = row.get("suggested_episodes")
        if type(count) is not int or count < 1:
            raise ValueError(f"suggestion {index} has invalid suggested_episodes")
        slug = _slug(row.get("axis", "suggestion"), f"suggestion_{index:02d}")
        task = copy.deepcopy(template)
        recording = task.setdefault("recording_setting", {})
        recording["num_of_episode"] = count
        task["rsi_compilation"] = {
            "source": str(suggestions_path),
            "suggestion_index": index,
            "axis": row.get("axis", ""),
            "failure_hypothesis": row.get("failure_hypothesis", ""),
            "task_variants": row.get("task_variants", []),
            "perturbations": row.get("perturbations", []),
            "quality_checks": row.get("quality_checks", []),
            "manual_review": [
                "Assign numeric ranges and distributions to qualitative perturbations.",
                "Confirm task variants map to assets and arm-specific instructions.",
                "Reserve held-out evaluation scenes before collection.",
            ],
        }
        path = output_dir / f"{index:02d}_{slug}.json"
        path.write_text(json.dumps(task, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        compiled.append({"index": index, "axis": row.get("axis", ""), "episodes": count, "template": str(path)})
    plan = {
        "schema_version": "0.1.0",
        "status": "compiled_pending_review",
        "source_suggestions": str(suggestions_path),
        "base_template": str(template_path),
        "total_suggested_episodes": sum(item["episodes"] for item in compiled),
        "tasks": compiled,
        "execution": {"auto_execute": False, "command": "geniesim autocollect run <template> --dry-run"},
    }
    (output_dir / "collection_plan.json").write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return plan


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suggestions", type=Path, required=True)
    parser.add_argument("--template", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    plan = compile_suggestions(args.suggestions, args.template, args.output_dir)
    print(json.dumps({"status": plan["status"], "tasks": len(plan["tasks"]),
                      "total_suggested_episodes": plan["total_suggested_episodes"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
