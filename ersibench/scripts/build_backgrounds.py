#!/usr/bin/env python3
"""Background scene catalogue: scripts/backgrounds.json -> <out>/<key>/background.usda (default /tmp/ersibench/background).
<out> is a complete background/ tree: EVERY catalogue entry (ERSI scenes and the imported upstream ones alike) is rendered
from the json; only what is not a scene (common/ payloads, files next to a background.usda) is a symlink to
UPSTREAM_MOUNT/<...>. The launchers mount <out> over /geniesim_assets/background and assets/background at UPSTREAM_MOUNT,
so nothing is ever written into assets/ and no run ever loads an upstream background.usda directly.

Entry key = path under background/ (e.g. "home/home_1_with_table"). Fields, any mix:
  base       reference an existing background's /World (e.g. "home/home_1")
  translate  move the whole world (metres)
  shell      house shell payload {asset, collision: [parts that keep collision]}
  items      static assets [{name, asset, position, quaternion(wxyz), scale, active, visible, collision, approximation, group}]
             imported prims (items, shell, table) also carry meta / attrs / overrides / arc: header lines, attributes and
             child overs kept verbatim (physics and material overrides are too irregular to model). When `overrides` is
             present it replaces the collision/approximation fields; pose, asset, active and visible stay editable.
  lights     [{type, name, apis, lines}] verbatim light prims;  colliders  [{name, position, quaternion, scale}] invisible cubes
  table      the work table {asset, position, quaternion}, collision on
  source     the upstream scene an entry was imported from; --check holds the entry to it. Entries without it are ERSI scenes.
  raw        [{kind, name, text}] prims kept verbatim (inline collision meshes/planes, materials, cameras, instancers)
  unsupported informational, written by the importer: prims it could not place (typed prims inside a transformed group)

  scripts/build_backgrounds.py [--out DIR]      # render every entry (default: $ERSIBENCH_BACKGROUNDS or /tmp/ersibench/background)
  scripts/build_backgrounds.py --import-all     # (re)decompose every upstream scene into the catalogue
  scripts/build_backgrounds.py --check          # exit 1 unless every `source` entry renders to a scene that composes exactly like
                                                # its upstream original (needs pxr: run it in the `ersi` env after editing the
                                                # importer/renderer or updating assets; without pxr only the json structure is compared)
"""
import argparse, collections, json, math, os, pathlib, re, shutil, tempfile, textwrap

ROOT = pathlib.Path(__file__).resolve().parents[1]
ASSETS = pathlib.Path(os.environ.get("ERSIBENCH_ASSETS") or ROOT / "assets")
OUT = pathlib.Path(os.environ.get("ERSIBENCH_BACKGROUNDS") or "/tmp/ersibench/background")
UPSTREAM_MOUNT = "/upstream_background"  # where the launchers mount assets/background inside the container
CATALOG = ROOT / "scripts/backgrounds.json"
HEADER = '#usda 1.0\n(\n    defaultPrim = "World"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
SHELL = re.compile(r"common/(home_|kitchen/|laboratory|market|office|popcorn|restaurant|room|study|supermarket|warehouse)")

# ---------------------------------------------------------------- usda parsing
PRIM_START = {s: re.compile(r'^([ \t]*)(?:%s) (\w+ )?"([^"]+)"' % s, re.M) for s in ("def", "def|over")}


def _balanced(text, i, open_ch, close_ch):
    """Index just past the bracket at text[i]'s matching close, skipping "..." and @...@ literals."""
    depth, j = 0, i
    while j < len(text):
        c = text[j]
        if c in '"@':
            j = text.index(c, j + 1)
        elif c == open_ch:
            depth += 1
        elif c == close_ch:
            depth -= 1
            if depth == 0:
                return j + 1
        j += 1
    raise ValueError("unbalanced usda")


def prims(text, specs="def"):
    """Direct child prims of a usda body: dicts with kind, name, header, body."""
    out, i = [], 0
    while (m := PRIM_START[specs].search(text, i)):
        j = m.end()
        header = ""
        k = j + len(text[j:]) - len(text[j:].lstrip())
        if text[k] == "(":
            e = _balanced(text, k, "(", ")"); header = text[k + 1:e - 1]; k = e
        k += len(text[k:]) - len(text[k:].lstrip())
        assert text[k] == "{", (m.group(3), text[k:k + 40])
        e = _balanced(text, k, "{", "}")
        out.append({"kind": (m.group(2) or "").strip(), "name": m.group(3), "header": header, "body": text[k + 1:e - 1], "text": text[m.start():e]})
        i = e
    return out


def asset_of(header):
    """Assets-root-relative asset (dir, or bare .usd file) referenced by a prim header, else None."""
    for line in header.splitlines():
        m = re.search(r"prepend (?:payload|references) = .*?@((?:\.\./)+)([^@]+?)@", line, re.S)
        if not m:
            m = re.search(r"prepend (?:payload|references) = \[", line) and re.search(r"@((?:\.\./)+)([^@]+?)@", header[header.index(line) + len(line):])
        if m:
            return ("background/" if m.group(1) == "../../" else "") + m.group(2)
    return None


def own_lines(body):
    """Attribute lines of a prim itself (not of nested overs/defs)."""
    for c in prims(body, "def|over"):
        body = body.replace(c["text"], "")
    return [t for line in body.splitlines() if (t := line.strip())]


def pose_of(body):
    t = q = sc = piv = None
    for s in own_lines(body):
        if "xformOp:translate =" in s: t = vec(s)
        elif "xformOp:translate:pivot =" in s: piv = vec(s)
        elif "xformOp:orient =" in s: q = vec(s)
        elif "xformOp:scale =" in s: sc = vec(s)
        elif "xformOp:rotateZYX =" in s or "xformOp:rotateXYZ =" in s: q = euler_q(vec(s), "ZYX" in s)
    t, q, sc = t or [0, 0, 0], q or [1, 0, 0, 0], sc or [1, 1, 1]
    if piv:   # T * P * R * S * P^-1 (the DCC pivot convention)  ==  translate by t + p - R S p
        t = [a + b - c for a, b, c in zip(t, piv, qrot(q, [x * y for x, y in zip(piv, sc)]))]
    return t, q, sc


def vec(s):
    return [float(x) for x in s[s.index("(") + 1:s.index(")")].split(",")]


def euler_q(deg, zyx):
    r = [math.radians(v) for v in deg]
    ax = {"x": (r[0], [1, 0, 0]), "y": (r[1], [0, 1, 0]), "z": (r[2], [0, 0, 1])}
    q = [1, 0, 0, 0]
    for a in ("zyx" if zyx else "xyz"):  # usd applies the ops right-to-left: rotateZYX = Rz * Ry * Rx
        t, (x, y, z) = ax[a]; q = qmul(q, [math.cos(t / 2), x * math.sin(t / 2), y * math.sin(t / 2), z * math.sin(t / 2)])
    return q


def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def qrot(q, v):
    return qmul(qmul(q, [0, *v]), [q[0], -q[1], -q[2], -q[3]])[1:]


def compose(parent, child):
    """(t, q, s) of child expressed in the parent's parent frame."""
    pt, pq, ps = parent; ct, cq, cs = child
    return [a + b for a, b in zip(pt, qrot(pq, [ci * si for ci, si in zip(ct, ps)]))], qmul(pq, cq), [a * b for a, b in zip(ps, cs)]


IDENT = ([0, 0, 0], [1, 0, 0, 0], [1, 1, 1])
POSE_LINE = re.compile(r"xformOp|^token visibility\b")


def extras(header, body):
    """What pose/asset/active/visible do not express, kept verbatim so a rendered scene composes like the original:
    meta = other header lines, attrs = other own attributes, overrides = child prims (physics/material overs)."""
    meta, skip = [], False
    for line in header.strip().splitlines():
        t = line.strip()
        if skip or re.match(r"(prepend|append|add)? ?(payload|references) = |active = ", t):
            skip = (skip or t.endswith("[")) and not t.endswith("]")   # multi-line reference list
            continue
        meta.append(t)
    x = {"meta": meta, "attrs": [l for l in own_lines(body) if not POSE_LINE.search(l)],
         "arc": "references" if re.search(r"references = ", header) else ""}
    # `overrides` is always present on imported prims: it is what tells render() not to fall back to the semantic fields
    return {**{k: v for k, v in x.items() if v}, "overrides": [textwrap.dedent(c["text"]) for c in prims(body, "def|over")]}


def import_text(text):
    world = next(p for p in prims(text) if p["name"] == "World")
    e = {"items": [], "lights": [], "colliders": [], "raw": [], "unsupported": []}
    tr = [s for s in own_lines(world["body"]) if "xformOp:translate =" in s]
    if tr:
        e["translate"] = vec(tr[0])

    seen = collections.Counter()

    def unique(name):   # groups are flattened into one scope, so a name used in two groups needs a suffix
        seen[name] += 1
        return name if seen[name] == 1 else f"{name}__{seen[name]}"

    def walk(body, xf, group):
        for p in prims(body):
            kind, name, hdr, bd = p["kind"], p["name"], p["header"], p["body"]
            if kind == "Scope":
                walk(bd, xf, group)
            elif kind == "Xform" and name == "Table" and (inner := next((c for c in prims(bd) if asset_of(c["header"])), None)):
                t, q, _ = compose(xf, pose_of(bd))
                e["table"] = {"asset": asset_of(inner["header"]), "position": t, "quaternion": q, "local": pose_of(inner["body"]), **extras(inner["header"], inner["body"])}
            elif kind == "Xform" and re.search(r"prepend (?:payload|references)", bd):
                walk(bd, compose(xf, pose_of(bd)), name)
            elif kind == "":
                asset = asset_of(hdr)
                if asset is None:
                    e["unsupported"].append(f"{name} (no asset)"); continue
                t, q, s = compose(xf, pose_of(bd))
                it = {"name": name, "asset": asset, "position": t, "quaternion": q}
                if s != [1, 1, 1]: it["scale"] = s
                if "active = false" in hdr: it["active"] = False
                if any('visibility = "invisible"' in l for l in own_lines(bd)): it["visible"] = False
                if group: it["group"] = group
                it.update(extras(hdr, bd))
                if SHELL.search(asset) and "shell" not in e:   # the house itself; further matches (e.g. popcorn props) are ordinary items
                    e["shell"] = it
                else:
                    it["name"] = unique(name); e["items"].append(it)
            elif kind.endswith("Light"):
                if xf != IDENT:
                    e["unsupported"].append(f"{kind} {name} (inside a transformed group)"); continue
                apis = re.search(r"apiSchemas = (\[[^\]]*\])", hdr)
                e["lights"].append({"type": kind, "name": unique(name), "apis": apis.group(1) if apis else "", "lines": own_lines(bd), **({"active": False} if "active = false" in hdr else {})})
            elif kind == "Cube" and 'visibility = "invisible"' in bd and "collisionEnabled = 1" in bd:
                t, q, s = compose(xf, pose_of(bd)); e["colliders"].append({"name": unique(name), "position": t, "quaternion": q, "scale": s})
            elif xf == IDENT:  # inline geometry, materials, cameras, instancers: kept verbatim
                new = unique(name)
                e["raw"].append({"kind": kind, "name": new, "text": textwrap.dedent(p["text"]).replace(f'"{name}"', f'"{new}"', 1)})
            else:
                e["unsupported"].append(f"{kind} {name} (inside a transformed group)")
    walk(world["body"], IDENT, "")
    for k in ("lights", "colliders", "raw", "unsupported"):
        if not e[k]: del e[k]
    return e


def import_scene(source):
    return import_text((ASSETS / "background" / source / "background.usda").read_text())


# ---------------------------------------------------------------- rendering
def usd(v):
    return "(" + ", ".join(f"{float(x):.12g}" for x in v) + ")"


def xform(pos, quat=(1, 0, 0, 0), scale=(1, 1, 1), ind="            "):
    return (f"{ind}quatf xformOp:orient = {usd(quat)}\n{ind}float3 xformOp:scale = {usd(scale)}\n"
            f"{ind}double3 xformOp:translate = {usd(pos)}\n"
            f'{ind}uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]\n')


def asset_file(asset):
    if asset.endswith((".usd", ".usda")):
        return f"../../../{asset}"
    d = ASSETS / asset
    for n in ("Aligned.usda", "Aligned.usd"):
        if (d / n).exists():
            return f"../../../{asset}/{n}"
    raise FileNotFoundError(d)


def physics(collision, approximation, ind="            "):
    i2 = ind + "    "; i3 = i2 + "    "; i4 = i3 + "    "
    apx = f'{i4}uniform token physics:approximation = "{approximation}"\n' if collision and approximation else ""
    return (f'{ind}over "entity"\n{ind}{{\n{i2}bool physics:rigidBodyEnabled = 0\n{i2}over "body"\n{i2}{{\n{i3}over "visual"\n{i3}{{\n'
            f"{apx}{i4}bool physics:collisionEnabled = {int(bool(collision))}\n{i3}}}\n{i2}}}\n{ind}}}\n")


def ref_prim(ind, name, it, pose, fallback):
    """def "<name>" (payload) { pose; the imported extras verbatim, or `fallback` for hand-written (ERSI) entries }"""
    i2 = ind + "    "
    head = ("".join(f"{i2}{m}\n" for m in it.get("meta", [])) + (f"{i2}active = false\n" if it.get("active") is False else "")
            + f'{i2}prepend {it.get("arc", "payload")} = @{asset_file(it["asset"])}@\n')
    body = (f'{i2}token visibility = "invisible"\n' if it.get("visible") is False else "") + (xform(*pose, ind=i2) if pose else "")
    if "overrides" in it:
        body += "".join(f"{i2}{a}\n" for a in it.get("attrs", [])) + "".join(textwrap.indent(o.rstrip("\n") + "\n", i2) for o in it["overrides"])
    else:
        body += fallback
    return f'{ind}def "{name}" (\n{head}{ind})\n{ind}{{\n{body}{ind}}}\n'


def render(key, e):
    head = 'def Xform "World"' + (f' (\n    prepend references = @../{e["base"].split("/")[-1]}/background.usda@</World>\n)' if "base" in e else "")
    body = f"    double3 xformOp:translate = {usd(e['translate'])}\n    uniform token[] xformOpOrder = [\"xformOp:translate\"]\n" if "translate" in e else ""
    if "shell" in e or e.get("items"):
        body += '    def Scope "background"\n    {\n'
        if "shell" in e:
            s = e["shell"]
            parts = "".join(f'            over "{part}"\n            {{\n                over "visual" (\n'
                            '                    prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxConvexHullCollisionAPI"]\n'
                            '                )\n                {\n                    uniform token physics:approximation = "none"\n'
                            '                    bool physics:collisionEnabled = 1\n                }\n            }\n' for part in s.get("collision", []))
            body += ref_prim("        ", s.get("name", "shell"), s, (s["position"], s["quaternion"], s.get("scale", (1, 1, 1))) if "position" in s else None, parts)
        for i, it in enumerate(e.get("items", [])):
            nm = it.get("name") or f'{it["asset"].split("/")[-1].split(".")[0]}_{i:02d}'
            body += ref_prim("        ", nm, it, (it["position"], it.get("quaternion", (1, 0, 0, 0)), it.get("scale", (1, 1, 1))),
                             physics(it.get("collision", False), it.get("approximation")))
        body += "    }\n"
    if e.get("lights"):
        body += '    def Scope "light"\n    {\n'
        for l in e["lights"]:
            meta = ("            active = false\n" if l.get("active") is False else "") + (f'            prepend apiSchemas = {l["apis"]}\n' if l.get("apis") else "")
            apis = f" (\n{meta}        )" if meta else ""
            body += f'        def {l["type"]} "{l["name"]}"{apis}\n        {{\n' + "".join(f"            {s}\n" for s in l["lines"]) + "        }\n"
        body += "    }\n"
    for r in e.get("raw", []):
        body += textwrap.indent(r["text"].rstrip("\n"), "    ") + "\n"
    for c in e.get("colliders", []):
        body += (f'    def Cube "{c["name"]}" (\n        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]\n    )\n    {{\n'
                 '        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]\n        bool physics:collisionEnabled = 1\n'
                 '        double size = 1\n        token visibility = "invisible"\n' + xform(c["position"], c.get("quaternion", (1, 0, 0, 0)), c.get("scale", (1, 1, 1)), "        ") + "    }\n")
    if "table" in e:
        t = e["table"]
        # hand-written tables: no rigid body, the top slab (entity/part_00) collides as a box, like upstream's home_b_with_table
        slab = ('            over "entity"\n            {\n                bool physics:rigidBodyEnabled = 0\n'
                '                over "body"\n                {\n                    over "visual"\n                    {\n                        bool physics:collisionEnabled = 0\n                    }\n                }\n'
                '                over "part_00"\n                {\n                    over "visual"\n                    {\n'
                '                        uniform token physics:approximation = "boundingCube"\n                        bool physics:collisionEnabled = 1\n                    }\n                }\n            }\n')
        body += ('    def Xform "Table"\n    {\n' + xform(t["position"], t.get("quaternion", (1, 0, 0, 0)), ind="        ")
                 + ref_prim("        ", "table", t, t.get("local", ((0, 0, 0.2609),)), slab) + "    }\n")
    return HEADER + f"# generated by scripts/build_backgrounds.py from backgrounds.json entry '{key}'\n{head}\n{{\n{body}}}\n"


# ---------------------------------------------------------------- check
def close(a, b, tol=1e-6):
    if isinstance(a, list):
        return len(a) == len(b) and all(close(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, float) or isinstance(b, float):
        return abs(a - b) < tol
    return a == b


def same_quat(a, b):
    return close(a, b) or close(a, [-x for x in b])


def diff_entry(a, b):
    """Human-readable differences between two imported entries (after a render round trip)."""
    out = []
    for k in ("shell", "table", "translate"):
        if k in a or k in b:
            x, y = a.get(k), b.get(k)
            if x is None or y is None or isinstance(x, dict) and any(
                    not (same_quat(x[f], y.get(f, [])) if f == "quaternion" else close(x[f], y.get(f))) for f in x if f != "group") or not isinstance(x, dict) and not close(x, y):
                out.append(f"{k}: {x} != {y}")
    ai = {i["name"]: i for i in a.get("items", [])}; bi = {i["name"]: i for i in b.get("items", [])}
    if ai.keys() != bi.keys():
        out.append(f"items: {len(ai)} vs {len(bi)}: {sorted(ai.keys() ^ bi.keys())[:5]}")
    for n in ai.keys() & bi.keys():
        x, y = dict(ai[n]), dict(bi[n]); x.pop("group", None); y.pop("group", None)
        for f in x.keys() | y.keys():
            if f == "quaternion" and not same_quat(x[f], y[f]) or f != "quaternion" and not close(x.get(f), y.get(f)):
                out.append(f"item {n}.{f}: {x.get(f)} != {y.get(f)}")
    if [(r["kind"], r["name"], r["text"].strip()) for r in a.get("raw", [])] != [(r["kind"], r["name"], r["text"].strip()) for r in b.get("raw", [])]:
        out.append(f"raw prims differ: {[r['name'] for r in a.get('raw', [])]} vs {[r['name'] for r in b.get('raw', [])]}")
    for k in ("lights", "colliders"):
        if a.get(k, []) != b.get(k, []) and not (k == "colliders" and close([c["position"] for c in a.get(k, [])], [c["position"] for c in b.get(k, [])])):
            out.append(f"{k}: {len(a.get(k, []))} vs {len(b.get(k, []))} or content differs")
    return out


def stage_signature(path):
    """What the simulator gets from a composed scene: every geometry / light / camera / physics-carrying prim with its
    world transform, visibility, physics attributes and light inputs. Prim paths are ignored (the catalogue flattens groups)."""
    from pxr import Usd, UsdGeom, UsdLux
    stage = Usd.Stage.Open(str(path)); xf = UsdGeom.XformCache(); sig = collections.Counter()
    for prim in stage.Traverse():
        phys = tuple(sorted((a.GetName(), str(a.Get())) for a in prim.GetAttributes() if a.GetName().startswith(("physics:", "inputs:")) and a.HasAuthoredValue()))
        leaf = prim.IsA(UsdGeom.Gprim) or prim.IsA(UsdGeom.PointInstancer) or prim.IsA(UsdGeom.Camera) or prim.HasAPI(UsdLux.LightAPI)
        if leaf or phys:
            m = xf.GetLocalToWorldTransform(prim)
            sig[(prim.GetTypeName(), re.sub(r"__\d+$", "", prim.GetName()) if prim.IsA(UsdGeom.Gprim) else "", tuple(round(m[i][j], 4) for i in range(4) for j in range(4)),
                 UsdGeom.Imageable(prim).ComputeVisibility() if prim.IsA(UsdGeom.Imageable) else "", phys)] += 1
    return sig


def build_tree(cat, out, upstream):
    """<out> = a complete background/ tree: every catalogue entry rendered, everything else a symlink under `upstream`."""
    if out.exists():
        shutil.rmtree(out)
    for key, e in cat.items():
        if not key.startswith("_"):
            (out / key).mkdir(parents=True); (out / key / "background.usda").write_text(render(key, e))
    for sub in sorted((ASSETS / "background").glob("*/*")):   # common/ payloads, scenes not in the catalogue, files next to a background.usda
        dst = out / sub.parent.name / sub.name
        if not dst.exists():
            dst.parent.mkdir(exist_ok=True); dst.symlink_to(f"{upstream}/{sub.parent.name}/{sub.name}")
        elif sub.is_dir():
            for f in sub.iterdir():
                if f.name != "background.usda":
                    (dst / f.name).symlink_to(f"{upstream}/{sub.parent.name}/{sub.name}/{f.name}")


def check(cat):
    """Every `source` entry, rendered from the catalogue, must equal the upstream scene it was imported from."""
    try:
        import pxr  # noqa: F401  (usd-core; in the `ersi` conda env)
    except ImportError:
        pxr = None
        print("pxr not importable: only the structural comparison runs; use the ersi env for the composed-stage comparison")
    bad = 0
    with tempfile.TemporaryDirectory() as tmp:
        tmp = pathlib.Path(tmp)
        for d in ASSETS.iterdir():   # same depth as assets/, so the rendered ../../../<asset> paths resolve
            if d.name != "background":
                (tmp / d.name).symlink_to(d)
        build_tree(cat, tmp / "background", ASSETS / "background")
        for key, e in cat.items():
            if key.startswith("_") or "source" not in e:
                continue
            d = diff_entry(import_scene(e["source"]), import_text(render(key, e)))
            try:
                a, b = (stage_signature(ASSETS / "background" / e["source"] / "background.usda"), stage_signature(tmp / "background" / key / "background.usda")) if pxr else (collections.Counter(), collections.Counter())
                d += [f"stage: upstream only  {x[0]} {x[1]} {x[3]} {x[4]}" for x in (a - b)] + [f"stage: rendered only  {x[0]} {x[1]} {x[3]} {x[4]}" for x in (b - a)]
            except Exception as exc:   # a rendered file USD refuses to open is a difference, not a crash
                d.append(f"stage: {type(exc).__name__}: {' '.join(str(exc).split())[:300]}")
            bad += bool(d)
            print(f"{'DIFF' if d else 'OK  '} {key:34s} items {len(e.get('items', [])):4d}  lights {len(e.get('lights', [])):2d}  "
                  f"raw {len(e.get('raw', [])):2d}  differences {len(d):4d}  unsupported {e.get('unsupported', [])}")
            for line in d[:int(os.environ.get("SHOW", 4))]:
                print("      ", line[:220])
    print(f"{bad} of {sum('source' in e for e in cat.values() if isinstance(e, dict))} upstream scenes differ from their catalogue entry")
    return bad


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--import-all", action="store_true", help="decompose every upstream scene into the catalogue (keeps ERSI entries)")
    ap.add_argument("--check", action="store_true", help="compare every rendered `source` entry with its upstream scene (composed USD stage; needs pxr)")
    ap.add_argument("--assets", type=pathlib.Path, default=ASSETS, help="geniesim_assets checkout (read only, for asset lookups)")
    ap.add_argument("--out", type=pathlib.Path, default=OUT, help="where the rendered background tree goes")
    a = ap.parse_args()
    globals()["ASSETS"] = a.assets
    cat = json.loads(CATALOG.read_text())
    if a.import_all:
        for p in sorted(ASSETS.glob("background/*/*/background.usda")):
            key = f"{p.parent.parent.name}/{p.parent.name}"
            if p.parent.parent.name == "common" or (key in cat and "source" not in cat[key]):
                continue
            cat[key] = {"source": key, **import_scene(key)}
            print(f"imported {key}: {len(cat[key]['items'])} items")
        CATALOG.write_text(json.dumps(cat, indent=1) + "\n")
    if a.check:
        raise SystemExit(1 if check(cat) else 0)
    if not (ASSETS / "background").is_dir():  # e.g. a launcher test with a stub assets dir
        print(f"no backgrounds under {ASSETS}; nothing rendered"); return
    build_tree(cat, a.out, UPSTREAM_MOUNT)
    print(f"rendered {sum(not k.startswith('_') for k in cat)} backgrounds into {a.out}")


if __name__ == "__main__":
    assert close(compose(([1, 0, 0], euler_q([0, 0, 90], True), [1, 1, 1]), ([1, 0, 0], [1, 0, 0, 0], [1, 1, 1]))[0], [1, 1, 0])  # self-check: 90 deg about z
    main()
