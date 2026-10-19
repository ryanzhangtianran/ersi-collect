#!/usr/bin/env python3
"""Check a synthesized task before collecting it: schema (task.schema.json / scene.schema.json), scene file and
background exist; prints the stages as the collector will run them. Stdlib only; the collect service runs the same
check on upload.

  python3 skills/collect-episodes/task_schema.py <task.json | scene.json> ...
  python3 skills/collect-episodes/task_schema.py --all      # every scenes/*.json and tasks/*/g2/*.json
"""
import json, pathlib, re, sys

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[1]
DC = REPO / "source/data_collection"
SCHEMAS = {kind: json.loads((HERE / f"{kind}.schema.json").read_text()) for kind in ("task", "scene")}
sys.path.insert(0, str(DC))
from common.task_file import TASK_KEYS as _TASK_KEYS  # noqa: E402
TASK_KEYS = ("scene_file", *_TASK_KEYS)
TYPES = {"object": dict, "array": list, "string": str, "boolean": bool, "null": type(None), "integer": int, "number": (int, float)}


def _is(value, t):
    return isinstance(value, TYPES[t]) and not (t in ("integer", "number") and isinstance(value, bool))


def errors(value, schema, root, path="$"):
    """JSON Schema errors as 'path: message'.
    ponytail: only the keywords the two schema files use; switch to the jsonschema package if they grow past that."""
    if "$ref" in schema:
        schema = root["$defs"][schema["$ref"].rsplit("/", 1)[1]]
    t = schema.get("type")
    if t and not any(_is(value, x) for x in ([t] if isinstance(t, str) else t)):
        return [f"{path}: expected {t}, got {type(value).__name__}"]
    out = []
    if "const" in schema and value != schema["const"]:
        out.append(f"{path}: must be {schema['const']!r}")
    if "enum" in schema and value not in schema["enum"]:
        out.append(f"{path}: {value!r} not one of {schema['enum']}")
    if "pattern" in schema and isinstance(value, str) and not re.search(schema["pattern"], value):
        out.append(f"{path}: {value!r} does not match {schema['pattern']}")
    if _is(value, "number"):
        if value < schema.get("minimum", value) or value > schema.get("maximum", value):
            out.append(f"{path}: {value} outside [{schema.get('minimum')}, {schema.get('maximum')}]")
    if isinstance(value, list):
        if not schema.get("minItems", 0) <= len(value) <= schema.get("maxItems", len(value)):
            out.append(f"{path}: {len(value)} items, need {schema.get('minItems', 0)}..{schema.get('maxItems', 'any')}")
        for i, item in enumerate(value):
            out += errors(item, schema.get("items", {}), root, f"{path}[{i}]")
    if isinstance(value, dict):
        props = schema.get("properties", {})
        out += [f"{path}: missing {k!r}" for k in schema.get("required", []) if k not in value]
        extra = schema.get("additionalProperties", True)
        for k, v in value.items():
            if k in props:
                out += errors(v, props[k], root, f"{path}.{k}")
            elif extra is False:
                out.append(f"{path}: unknown field {k!r}; allowed: {sorted(props)}")
            elif isinstance(extra, dict):
                out += errors(v, extra, root, f"{path}.{k}")
    for sub in schema.get("allOf", []):
        out += errors(value, sub, root, path)
    if "if" in schema:
        branch = schema.get("else", {}) if errors(value, schema["if"], root, path) else schema.get("then", {})
        out += errors(value, branch, root, path)
    return out


def stage_sequence(stages):
    """'G T[hold_offset>world] U T[reset]': the stages as the collector will run them."""
    from client.planner.action.action_script import describe, primitive_stages  # noqa: E402
    return describe(primitive_stages(json.loads(json.dumps(stages))))


def check_task(task, scenes=DC / "scenes"):
    """Schema and scene file. Returns error strings."""
    errs = errors(task, SCHEMAS["task"], SCHEMAS["task"])
    if errs:
        return errs
    if "scene_file" in task and not (scenes / f"{task['scene_file']}.json").is_file():
        return [f"$.scene_file: no scenes/{task['scene_file']}.json"]
    return []


def check_scene(scene, catalog=REPO / "scripts/backgrounds.json"):
    errs = errors(scene, SCHEMAS["scene"], SCHEMAS["scene"])
    if errs:
        return errs
    usd = scene["scene"]["scene_usd"]
    usd = usd[0] if isinstance(usd, list) and len(usd) == 1 else usd
    m = re.fullmatch(r"background/(.+)/background\.usda", usd) if isinstance(usd, str) else None
    if not m or m.group(1) not in json.loads(pathlib.Path(catalog).read_text()):
        return [f"$.scene.scene_usd: {usd!r} is not background/<key>/background.usda with <key> in scripts/backgrounds.json"]
    return []


def check(doc):
    """A task json, a scene json, or a whole template (both parts in one file)."""
    if "stages" not in doc:
        return check_scene(doc)
    if "scene_file" in doc or "scene" not in doc:
        return check_task(doc)
    return check_scene({k: v for k, v in doc.items() if k not in TASK_KEYS}) + check_task({k: doc[k] for k in TASK_KEYS if k in doc})


if __name__ == "__main__":
    paths = sorted([*DC.glob("scenes/*.json"), *DC.glob("tasks/*/g2/*.json")]) if sys.argv[1:] == ["--all"] else sys.argv[1:]
    if not paths:
        sys.exit(__doc__)
    bad = 0
    for p in paths:
        print(p)
        doc = json.loads(pathlib.Path(p).read_text())
        errs = check(doc)
        for e in errs:
            print("  " + e)
        if not errs and "stages" in doc:
            try:
                print("  stages:", stage_sequence(doc["stages"]))
            except ImportError:  # the collector's logger needs colorama; the schema check above does not
                pass
        bad += bool(errs)
    print(f"{len(paths) - bad}/{len(paths)} ok")
    sys.exit(1 if bad else 0)
