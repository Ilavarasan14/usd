"""environment/infrastructure.usdc -- dock doors, chargers, bollards, floor
markings, and racked pallet inventory."""
import random
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
from wh_common import *
from author_env import pallet_slots

SEED = 20260822
DOCK_DOOR_Y = [-6.75, -2.25, 2.25, 6.75]
CHARGER_Y = [-2.4, 0.0, 2.4]


def floor_strip(stage, path, x0, x1, y_c, width, lift=0.002, step=1.0):
    """A marking strip conformed to the slab -- sampled against floor_z so it
    never floats over or sinks into the drainage fall."""
    # Normalise the span: callers that pass x0 > x1 (the give-way bars at
    # negative X did exactly that) would otherwise reverse the winding and the
    # strip would face DOWN -- it renders black and is invisible to a
    # downward-looking sensor. Explicit +Z normals belt-and-brace it.
    if x1 < x0:
        x0, x1 = x1, x0
    pts, counts, idx, normals, sts = [], [], [], [], []
    n = max(2, int(abs(x1 - x0) / step) + 1)
    for i in range(n):
        x = x0 + (x1 - x0) * i / (n - 1)
        for y in (y_c - width / 2, y_c + width / 2):
            pts.append(Gf.Vec3f(x, y, floor_z(x, y) + lift))
    for i in range(n - 1):
        a = i * 2
        counts.append(4)
        idx.extend([a, a + 2, a + 3, a + 1])
        normals.extend([Gf.Vec3f(0, 0, 1)] * 4)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(pts)
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(idx)
    m.CreateNormalsAttr(normals)
    m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateExtentAttr(UsdGeom.PointBased(m).ComputeExtent(pts))
    set_xform(m.GetPrim())
    return m


def author_infrastructure():
    stage = new_layer("environment/infrastructure.usdc",
                      "Dock doors, chargers, bollards, floor markings, racked "
                      "pallet inventory. Static colliders only.")
    UsdGeom.Xform.Define(stage, "/World")
    Usd.ModelAPI(stage.GetPrimAtPath("/World")).SetKind("assembly")
    UsdGeom.Scope.Define(stage, "/World/Environment")
    grp = UsdGeom.Xform.Define(stage, "/World/Environment/Infrastructure")
    Usd.ModelAPI(grp.GetPrim()).SetKind("group")
    set_xform(grp.GetPrim())
    hx, hy = BAY_X / 2, BAY_Y / 2
    stats = {}

    # ---- sectional dock doors (closed), levellers and bumpers
    door_boxes, bumper_boxes = [], []
    for dy in DOCK_DOOR_Y:
        for k in range(6):                       # 6 horizontal slats
            z0 = k * (DOCK_DOOR_H / 6)
            door_boxes.append((0.06, DOCK_DOOR_W, DOCK_DOOR_H / 6 - 0.01,
                               (hx - 0.05, dy, z0 + DOCK_DOOR_H / 12)))
        for s in (-1, 1):                        # rubber dock bumpers
            bumper_boxes.append((0.20, 0.25, 0.45,
                                 (hx - 0.10, dy + s * (DOCK_DOOR_W / 2 + 0.2), 0.35)))
    d = merge_meshes(stage, "/World/Environment/Infrastructure/dock_doors", door_boxes)
    set_xform(d.GetPrim()); make_static_collider(d.GetPrim(), "none")
    b = merge_meshes(stage, "/World/Environment/Infrastructure/dock_bumpers", bumper_boxes)
    set_xform(b.GetPrim()); make_static_collider(b.GetPrim(), "none")
    stats["dock_doors"] = len(DOCK_DOOR_Y)

    # ---- opportunity chargers on the west wall
    ch_boxes = []
    for cy in CHARGER_Y:
        ch_boxes.append((0.40, 0.60, 1.20, (-hx + 0.20, cy, 0.60)))      # cabinet
        ch_boxes.append((0.06, 0.30, 0.12, (-hx + 0.43, cy, 0.22)))      # contact plate
    c = merge_meshes(stage, "/World/Environment/Infrastructure/chargers", ch_boxes)
    set_xform(c.GetPrim()); make_static_collider(c.GetPrim(), "none")
    stats["chargers"] = len(CHARGER_Y)

    # ---- bollards: charger bank + cross-aisle blind corners
    bl = UsdGeom.Scope.Define(stage, "/World/Environment/Infrastructure/Bollards")
    n_bol = 0
    spots = [(-hx + 0.9, cy + s * 0.55) for cy in CHARGER_Y for s in (-1, 1)]
    for ry in RACK_RUN_Y:
        for cx in (-2.25 - 0.5, 2.25 + 0.5):
            spots.append((cx, ry))
    for i, (bx, by) in enumerate(spots):
        cyl = UsdGeom.Cylinder.Define(
            stage, f"/World/Environment/Infrastructure/Bollards/bollard_{i:02d}")
        cyl.CreateRadiusAttr(0.09); cyl.CreateHeightAttr(1.0); cyl.CreateAxisAttr("Z")
        cyl.CreateExtentAttr([Gf.Vec3f(-0.09, -0.09, -0.5), Gf.Vec3f(0.09, 0.09, 0.5)])
        p = cyl.GetPrim()
        set_xform(p, (bx, by, floor_z(bx, by) + 0.5))
        UsdPhysics.CollisionAPI.Apply(p)
        n_bol += 1
    stats["bollards"] = n_bol

    # ---- floor markings, conformed to the slab
    mk = UsdGeom.Scope.Define(stage, "/World/Environment/Infrastructure/FloorMarkings")
    n_mk = 0
    for i, ay in enumerate(AISLE_Y):                       # aisle edge lines
        for s in (-1, 1):
            floor_strip(stage, f"/World/Environment/Infrastructure/FloorMarkings/aisle_{i}_{'p' if s>0 else 'm'}",
                        -28.0, 28.0, ay + s * 1.5, 0.10)
            n_mk += 1
    for s in (-1, 1):                                       # walkway boundary
        floor_strip(stage, f"/World/Environment/Infrastructure/FloorMarkings/walkway_{'n' if s>0 else 's'}",
                    -28.0, 28.0, s * 9.25, 0.10)
        n_mk += 1
    floor_strip(stage, "/World/Environment/Infrastructure/FloorMarkings/dock_keepout",
                26.0, 26.0 + 0.1, 0.0, 20.0)                # dock apron edge
    n_mk += 1
    for i, ay in enumerate(AISLE_Y):                        # cross-aisle give-way bars
        for s in (-1, 1):
            floor_strip(stage, f"/World/Environment/Infrastructure/FloorMarkings/giveway_{i}_{'p' if s>0 else 'm'}",
                        s * 2.6, s * 2.7, ay, 3.0)
            n_mk += 1
    stats["floor_markings"] = n_mk

    # ---- racked inventory -------------------------------------------------
    rng = random.Random(SEED)
    slots = pallet_slots()
    filled = [s for s in slots if rng.random() < RACK_OCCUPANCY]
    lvl0 = [s for s in filled if s[3] == 0]
    high = [s for s in filled if s[3] > 0]

    # Levels 1-4: PointInstancer, render-only. These sit 1.8 m and higher, well
    # above the AMR deck (0.35 m) and its lidar plane (0.20 m) -- the robot can
    # never reach them, so per-instance colliders would be pure solver cost.
    inst = UsdGeom.PointInstancer.Define(
        stage, "/World/Environment/Infrastructure/RackedPallets")
    ip = inst.GetPrim()
    set_xform(ip)
    protos = []
    for name, variant in (("pallet_full", "full"), ("pallet_half", "half")):
        pp = "/World/Environment/Infrastructure/RackedPallets/Prototypes/" + name
        x = UsdGeom.Xform.Define(stage, pp)
        x.GetPrim().GetReferences().AddReference("../assets/props/pallet/pallet.usda")
        x.GetPrim().GetVariantSets().GetVariantSet("load").SetVariantSelection(variant)
        set_xform(x.GetPrim())
        # kill collision on the prototype -- stable path, no payload load needed
        over = stage.OverridePrim(pp + "/Collisions/hull")
        over.CreateAttribute("physics:collisionEnabled",
                             Sdf.ValueTypeNames.Bool, custom=False).Set(False)
        protos.append(Sdf.Path(pp))
    inst.CreatePrototypesRel().SetTargets(protos)

    pos, ori, scl, pid = [], [], [], []
    for (px, py, pz, lvl, fy) in high:
        jx = rng.uniform(-0.02, 0.02)
        jy = rng.uniform(-0.03, 0.03)
        # +90 deg: an 800 mm EUR pallet face goes toward the aisle, 1200 mm into
        # the rack depth. Without this the 1.2 m length lies along the beam and
        # adjacent pallets overlap by 0.3 m -- caught by the aabb_overlap check.
        yaw = 90.0 + rng.uniform(-2.0, 2.0)
        q = quat_from_axis_angle((0, 0, 1), yaw)
        pos.append(Gf.Vec3f(px + jx, py + jy, pz))
        ori.append(Gf.Quath(float(q.GetReal()), *[float(v) for v in q.GetImaginary()]))
        scl.append(Gf.Vec3f(1, 1, 1))
        pid.append(0 if rng.random() < 0.72 else 1)
    inst.CreatePositionsAttr(pos)
    inst.CreateOrientationsAttr(ori)
    inst.CreateScalesAttr(scl)
    inst.CreateProtoIndicesAttr(pid)
    inst.CreateExtentAttr([Gf.Vec3f(-hx, -hy, 0), Gf.Vec3f(hx, hy, RACK_H)])

    # Level 0: real prims. These ARE reachable, so they carry static collision.
    # instanceable=True is safe here precisely because every one of them needs
    # the identical static collider -- no per-instance physics variation.
    gi = UsdGeom.Scope.Define(stage, "/World/Environment/Infrastructure/GroundInventory")
    for i, (px, py, pz, lvl, fy) in enumerate(lvl0):
        p = UsdGeom.Xform.Define(
            stage, f"/World/Environment/Infrastructure/GroundInventory/pallet_{i:03d}").GetPrim()
        p.GetReferences().AddReference("../assets/props/pallet/pallet.usda")
        p.GetVariantSets().GetVariantSet("load").SetVariantSelection(
            "full" if rng.random() < 0.72 else "half")
        yaw = 90.0 + rng.uniform(-2.0, 2.0)      # 800 mm face to the aisle
        set_xform(p, (px + rng.uniform(-0.02, 0.02), py + rng.uniform(-0.03, 0.03), pz),
                  quat_from_axis_angle((0, 0, 1), yaw))
        p.SetInstanceable(True)
    stats.update(storage_positions=len(slots), filled=len(filled),
                 instanced_high=len(high), ground_pallets=len(lvl0))

    stage.GetRootLayer().Save()
    return stats
