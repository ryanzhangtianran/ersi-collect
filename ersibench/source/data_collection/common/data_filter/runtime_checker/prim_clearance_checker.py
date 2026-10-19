# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# License: Mozilla Public License Version 2.0
"""Minimum surface distance between two groups of robot links, e.g. the two grippers during a handover.

The planner only knows collision spheres and the last centimetres of a grasp are driven without a collision
check, so hand-to-hand contact has to be measured on the real meshes. Links are the rigid bodies whose prim
name starts with one of the given prefixes; their meshes are sampled once (they are rigid) and moved with the
link's current world pose, so the check costs one KD-tree query.

    {"checker_name": "prim_clearance",
     "params": {"prims_a": ["/G2/gripper_l"], "prims_b": ["/G2/gripper_r"], "rule": "greaterThan", "value": 0.003}}
"""

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from common.base_utils.logger import logger
from common.data_filter.runtime_checker.checker_base import ValueChecker
from common.data_filter.runtime_checker.checker_factory import register_checker


def sample_mesh(points, face_counts, face_indices, n, rng):
    """n area-weighted surface samples of a polygon mesh (fan-triangulated), plus its vertices."""
    points = np.asarray(points, dtype=float)
    tris, start = [], 0
    for count in face_counts:
        tris += [(face_indices[start], face_indices[start + k], face_indices[start + k + 1]) for k in range(1, count - 1)]
        start += count
    if not tris:
        return points
    a, b, c = (points[np.asarray(tris)[:, k]] for k in range(3))
    area = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    if area.sum() <= 0:
        return points
    pick = rng.choice(len(tris), size=n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    return np.vstack([points, a[pick] + u[:, None] * (b - a)[pick] + v[:, None] * (c - a)[pick]])


def link_clouds(stage, prefixes, samples_per_link=1500):
    """{link prim path: points in the link frame} for every rigid body whose path starts with a prefix."""
    from pxr import Usd, UsdGeom, UsdPhysics

    cache, rng, clouds = UsdGeom.XformCache(), np.random.default_rng(0), {}
    roots = {str(prefix).rsplit("/", 1)[0] or "/" for prefix in prefixes}
    for root in roots:
        root_prim = stage.GetPrimAtPath(root)
        if not root_prim:
            continue
        for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            link = prim
            while link and not link.HasAPI(UsdPhysics.RigidBodyAPI):
                link = link.GetParent()
            if not link or not any(str(link.GetPath()).startswith(prefix) for prefix in prefixes):
                continue
            mesh = UsdGeom.Mesh(prim)
            points = mesh.GetPointsAttr().Get()
            if not points:
                continue
            to_link = cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(link).GetInverse()
            local = sample_mesh(points, mesh.GetFaceVertexCountsAttr().Get() or [], mesh.GetFaceVertexIndicesAttr().Get() or [],
                                samples_per_link, rng)
            local = np.c_[local, np.ones(len(local))] @ np.array(to_link)  # USD matrices act on row vectors
            clouds.setdefault(str(link.GetPath()), []).append(local[:, :3])
    return {path: np.vstack(parts) for path, parts in clouds.items()}


def clearance(clouds_a, clouds_b, pose_of):
    """Minimum distance between two cloud groups; pose_of(path) -> (position, quaternion wxyz) in the world."""
    def world(clouds):
        out = []
        for path, local in clouds.items():
            position, quat = pose_of(path)
            out.append(Rotation.from_quat(np.roll(np.asarray(quat, dtype=float), -1)).apply(local) + np.asarray(position, dtype=float))
        return np.vstack(out)

    distances, _ = cKDTree(world(clouds_a)).query(world(clouds_b))
    return float(distances.min())


@register_checker("prim_clearance")
class PrimClearanceChecker(ValueChecker):
    def __init__(self, prims_a, prims_b, **kwargs):
        super().__init__(name="prim_clearance", **kwargs)
        self.prims_a, self.prims_b = list(prims_a), list(prims_b)

    def get_value(self):
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        clouds_a, clouds_b = link_clouds(stage, self.prims_a), link_clouds(stage, self.prims_b)
        if not clouds_a or not clouds_b:
            raise ValueError(f"prim_clearance: no meshes under {self.prims_a if not clouds_a else self.prims_b}")
        value = clearance(clouds_a, clouds_b, self.command_controller._get_object_pose)
        logger.info(f"PRIM_CLEARANCE {self.prims_a} vs {self.prims_b}: {value:.4f} m (limit {self.value})")
        return value


if __name__ == "__main__":  # python prim_clearance_checker.py  (needs pxr): two 10 cm cubes, 3 cm apart, one of them moved by its link pose
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    for name in ("hand_l", "hand_r"):
        link = UsdGeom.Xform.Define(stage, f"/R/{name}")
        UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        cube = UsdGeom.Mesh.Define(stage, f"/R/{name}/visual/mesh")
        corners = [(x, y, z) for x in (-0.05, 0.05) for y in (-0.05, 0.05) for z in (-0.05, 0.05)]
        cube.GetPointsAttr().Set(corners)
        cube.GetFaceVertexCountsAttr().Set([4] * 6)
        cube.GetFaceVertexIndicesAttr().Set([0, 1, 3, 2, 4, 5, 7, 6, 0, 1, 5, 4, 2, 3, 7, 6, 0, 2, 6, 4, 1, 3, 7, 5])
    a, b = link_clouds(stage, ["/R/hand_l"]), link_clouds(stage, ["/R/hand_r"])
    assert list(a) == ["/R/hand_l"] and list(b) == ["/R/hand_r"]
    poses = {"/R/hand_l": ([0, 0, 0], [1, 0, 0, 0]), "/R/hand_r": ([0.13, 0, 0], [1, 0, 0, 0])}
    assert abs(clearance(a, b, poses.get) - 0.03) < 0.004, clearance(a, b, poses.get)
    poses["/R/hand_r"] = ([0.08, 0, 0], [1, 0, 0, 0])  # overlapping: surface samples of one cube lie inside the other's faces
    assert clearance(a, b, poses.get) < 0.02
    print("ok")
