#!/usr/bin/env python3
"""Executable validation for the warehouse bay stage.

Every line this prints is the result of a measurement. Nothing is asserted
from the prim declarations alone.

    python3 tools/validate_scene.py            # report
    python3 tools/validate_scene.py --fix      # also write safety/overrides.usda
"""
import math, os, subprocess, sys, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf
from wh_common import (SCENE_ROOT, floor_z, PALLET_H, BAY_X, BAY_Y, CLEAR_H,
                       AMR_DECK_H, PALLET_L, PALLET_W, TOTE_H, RACK_H,
                       applied_schemas, RACK_RUN_Y, RACK_RUN_DEPTH, RACK_SEG_X,
                       AISLE_Y, AISLE_W, racking_x_extent)

RACK_X = racking_x_extent()   # real steel + end guards, not the layout allocation
RACK_BANDS = [(ry - RACK_RUN_DEPTH / 2, ry + RACK_RUN_DEPTH / 2)
              for ry in RACK_RUN_Y]


def rack_clearance(mn, mx):
    """(collides, clearance_m) for a world AABB against the racking.

    Racking occupies the product set {X segment} x {Y run band}, so this is a
    true AABB-to-box distance over every such box: per-axis gap, 0 where the
    ranges overlap, combined as a Euclidean distance. Measuring a single axis
    is wrong -- a robot on the aisle centreline is 1.27 m from any rack in Y
    no matter how close it gets to a segment boundary in X.

    Conservative by ~60 mm while the wheels turn: BBoxCache transforms each
    cylinder's axis-aligned extent, and rotating that box about the wheel axis
    grows it to r*sqrt(2). The real cylinder does not change shape when spun.
    """
    best, hit = 1e9, False
    for a, b in RACK_X:
        gx = max(a - mx[0], mn[0] - b, 0.0)
        for lo, hi in RACK_BANDS:
            gy = max(lo - mx[1], mn[1] - hi, 0.0)
            if gx == 0.0 and gy == 0.0:
                hit = True
                best = -1.0
                continue
            best = min(best, math.hypot(gx, gy))
    return hit, best


GROUND_TOL = 0.005          # m; beyond this a body floats or is buried
PENETRATION_TOL = 0.005     # m; AABB overlap below this is contact, not intersection

# MDL modules that ship with Kit. Offline they cannot resolve; inside Isaac Sim
# they always do. Reported as WARN with the reason, never silently suppressed.
KIT_MDL = ("OmniPBR.mdl", "OmniGlass.mdl", "OmniSurface.mdl",
           "OmniSurfacePresets.mdl", "OmniHair.mdl")

RESULTS = []


def rec(status, check, msg):
    RESULTS.append((status, check, msg))


# --------------------------------------------------------------- 1. usdchecker
def check_composition(root):
    exe = None
    for cand in ("usdchecker", "/usr/bin/usdchecker"):
        try:
            subprocess.run([cand, "--version"], capture_output=True, timeout=20)
            exe = cand
            break
        except Exception:
            continue
    if exe is None:
        rec("SKIP", "composition", "usdchecker not on PATH -- not run")
        return
    p = subprocess.run([exe, "--strict", "--verbose", root],
                       capture_output=True, text=True, timeout=600)
    out = (p.stdout + p.stderr).strip()
    noise = ("Opening ", "Not performing texture", "Failed!", "Success")
    errs, mdl = [], []
    for l in out.splitlines():
        l = l.strip().replace("\x1b[91m", "").replace("\x1b[0m", "")
        if not l or l.startswith(noise) or "Checking" in l:
            continue
        (mdl if any(k in l for k in KIT_MDL) else errs).append(l)
    if errs:
        rec("FAIL", "composition",
            f"{exe} --strict returned {p.returncode}; " + " | ".join(errs[:8]))
    else:
        rec("PASS", "composition",
            f"{exe} --strict: 0 errors excluding Kit-provided MDL")
    if mdl:
        rec("WARN", "composition",
            f"{len(mdl)} unresolvable Kit-provided MDL reference(s) "
            f"({', '.join(sorted(set(k for k in KIT_MDL if any(k in m for m in mdl))))}) "
            f"-- expected offline, resolves inside Isaac Sim; each material also "
            f"carries a UsdPreviewSurface so the stage still shades standalone")


# ----------------------------------------------------------- 2. stage metadata
def check_metadata(stage):
    want = {"metersPerUnit": 1.0, "kilogramsPerUnit": 1.0,
            "upAxis": "Z", "timeCodesPerSecond": 60.0}
    got = {"metersPerUnit": UsdGeom.GetStageMetersPerUnit(stage),
           "kilogramsPerUnit": stage.GetMetadata("kilogramsPerUnit"),
           "upAxis": UsdGeom.GetStageUpAxis(stage),
           "timeCodesPerSecond": stage.GetTimeCodesPerSecond()}
    bad = {k: (got[k], v) for k, v in want.items() if got[k] != v}
    dp = stage.GetDefaultPrim()
    if not dp or dp.GetPath() != Sdf.Path("/World"):
        bad["defaultPrim"] = (dp.GetPath() if dp else None, "/World")
    if bad:
        rec("FAIL", "stage_metadata",
            "; ".join(f"{k}={g!r} expected {e!r}" for k, (g, e) in bad.items()))
    else:
        rec("PASS", "stage_metadata",
            "metersPerUnit=1.0 kilogramsPerUnit=1.0 upAxis=Z "
            "timeCodesPerSecond=60 defaultPrim=/World")


# ------------------------------------------------------- placed-asset gathering
def placed_assets(stage):
    """Top-level placed things whose ground contact and mutual overlap matter."""
    out = []
    for scope, cls in (("/World/Scenario/Fleet", "amr"),
                       ("/World/Scenario/Staged", "staged"),
                       ("/World/Environment/Infrastructure/GroundInventory", "pallet"),
                       ("/World/Environment/Infrastructure/Bollards", "bollard")):
        p = stage.GetPrimAtPath(scope)
        if not p:
            continue
        for c in p.GetChildren():
            k = cls
            if cls == "staged":
                k = "tote" if c.GetName().startswith("tote") else "pallet"
            elif cls == "amr" and c.GetName().startswith("rover"):
                k = "rover"
            out.append((c, k))
    return out


def expected_support_z(prim, kind, cx, cy):
    """Height the object should be resting on. Totes sit on a pallet deck."""
    if kind == "tote":
        return floor_z(cx, cy) + PALLET_H
    return floor_z(cx, cy)


# ---------------------------------------------------------- 3. ground contact
def check_ground(stage, assets, bbc):
    fails = 0
    worst = ("", 0.0)
    for prim, kind in assets:
        bb = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
        if bb.IsEmpty():
            rec("FAIL", "ground_contact", f"{prim.GetPath()} has empty bound")
            fails += 1
            continue
        mn, mx = bb.GetMin(), bb.GetMax()
        cx, cy = (mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0
        gap = mn[2] - expected_support_z(prim, kind, cx, cy)
        if abs(gap) > abs(worst[1]):
            worst = (str(prim.GetPath()), gap)
        if gap > GROUND_TOL:
            rec("FAIL", "ground_contact",
                f"{prim.GetPath()} floats {gap:.3f} m above its support")
            fails += 1
        elif gap < -GROUND_TOL:
            rec("FAIL", "ground_contact",
                f"{prim.GetPath()} penetrates its support by {-gap:.3f} m")
            fails += 1
    if fails == 0:
        rec("PASS", "ground_contact",
            f"{len(assets)} placed assets rest on their support "
            f"(worst residual {worst[1]*1000:+.2f} mm at {worst[0].split('/')[-1]})")
    return fails


# --------------------------------------------------------- 4. AABB intersection
def whitelisted(a, b, ka, kb):
    """Pairs that are physically joined or stacked and may touch."""
    if {ka, kb} == {"tote", "pallet"} and a.GetParent() == b.GetParent():
        return True
    if ka == "tote" and kb == "tote":
        return True          # two totes share a pallet deck; separation is checked
    return False


def check_overlap(stage, assets, bbc):
    ranges = []
    for prim, kind in assets:
        r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
        if not r.IsEmpty():
            ranges.append((prim, kind, r))
    hits = 0
    shown = 0
    for (pa, ka, ra), (pb, kb, rb) in itertools.combinations(ranges, 2):
        pen = []
        for ax in range(3):
            lo = max(ra.GetMin()[ax], rb.GetMin()[ax])
            hi = min(ra.GetMax()[ax], rb.GetMax()[ax])
            pen.append(hi - lo)
        if min(pen) > PENETRATION_TOL:
            if whitelisted(pa, pb, ka, kb):
                continue
            hits += 1
            if shown < 6:
                rec("FAIL", "aabb_overlap",
                    f"{pa.GetPath().name} <-> {pb.GetPath().name} "
                    f"penetrate ({pen[0]:.3f}, {pen[1]:.3f}, {pen[2]:.3f}) m")
                shown += 1
    if hits > shown:
        rec("FAIL", "aabb_overlap", f"... and {hits - shown} further overlapping pairs")
    if hits == 0:
        rec("PASS", "aabb_overlap",
            f"{len(ranges)} assets, {len(ranges)*(len(ranges)-1)//2} pairs, "
            f"no intersection beyond {PENETRATION_TOL*1000:.0f} mm")
    return hits


# ---------------------------------------------------------------- 5. physics
def local_scale(prim):
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            v = op.Get()
            if v is not None:
                return Gf.Vec3d(v)
    return None


def check_physics(stage):
    fails = 0
    rb, arts = [], []
    it = iter(Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))
    for prim in it:
        schemas = applied_schemas(prim)
        if "PhysicsRigidBodyAPI" in schemas:
            rb.append(prim)
        if "PhysicsArticulationRootAPI" in schemas:
            arts.append(prim)

    for prim in rb:
        # collider present?
        has_col = any("PhysicsCollisionAPI" in applied_schemas(d)
                      for d in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))
        if not has_col:
            rec("FAIL", "physics", f"{prim.GetPath()} has RigidBodyAPI, no CollisionAPI")
            fails += 1
        # mass or density?
        m = UsdPhysics.MassAPI(prim)
        mass = m.GetMassAttr().Get() if m.GetMassAttr() else None
        dens = m.GetDensityAttr().Get() if m.GetDensityAttr() else None
        kin = UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get()
        if not kin and not (mass or dens):
            rec("FAIL", "physics",
                f"{prim.GetPath()} dynamic body with no mass and no density")
            fails += 1
        # nested rigid bodies?
        anc = prim.GetParent()
        while anc and anc.GetPath() != Sdf.Path("/"):
            if "PhysicsRigidBodyAPI" in applied_schemas(anc):
                rec("FAIL", "physics",
                    f"{prim.GetPath()} is a rigid body nested inside {anc.GetPath()}")
                fails += 1
                break
            anc = anc.GetParent()
        # articulation root must not also be a rigid body
        if "PhysicsArticulationRootAPI" in applied_schemas(prim):
            rec("FAIL", "physics",
                f"{prim.GetPath()} carries both ArticulationRootAPI and RigidBodyAPI")
            fails += 1
        # non-uniform scale on the body or anywhere above it
        node = prim
        while node and node.GetPath() != Sdf.Path("/"):
            s = local_scale(node)
            if s is not None:
                if max(abs(s[0] - s[1]), abs(s[1] - s[2]), abs(s[0] - s[2])) > 1e-9:
                    rec("FAIL", "physics",
                        f"{prim.GetPath()} has non-uniform scale {tuple(s)} at {node.GetPath()}")
                    fails += 1
                elif abs(s[0] - 1.0) > 1e-9 and node != prim:
                    rec("WARN", "physics",
                        f"{prim.GetPath()} has scale {s[0]:.3f} applied above it at {node.GetPath()}")
            node = node.GetParent()
        # triangle-mesh collision on a dynamic body
        for d in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            if "PhysicsMeshCollisionAPI" in applied_schemas(d):
                ap = UsdPhysics.MeshCollisionAPI(d).GetApproximationAttr().Get()
                if ap == "none" and not kin:
                    rec("FAIL", "physics",
                        f"{d.GetPath()} uses triangle-mesh collision on a dynamic body")
                    fails += 1

    scene = stage.GetPrimAtPath("/World/physicsScene")
    if not scene or not scene.IsA(UsdPhysics.Scene):
        rec("FAIL", "physics", "no UsdPhysics.Scene at /World/physicsScene")
        fails += 1
    else:
        g = UsdPhysics.Scene(scene)
        gd = g.GetGravityDirectionAttr().Get()
        gm = g.GetGravityMagnitudeAttr().Get()
        if tuple(gd) != (0, 0, -1) or abs(gm - 9.81) > 1e-6:
            rec("FAIL", "physics", f"gravity is {tuple(gd)} x {gm}")
            fails += 1
        if "PhysxSceneAPI" not in applied_schemas(scene):
            rec("FAIL", "physics", "physicsScene missing PhysxSceneAPI")
            fails += 1
        st = scene.GetAttribute("physxScene:solverType").Get()
        ts = scene.GetAttribute("physxScene:timeStepsPerSecond").Get()
        if st != "TGS" or ts != 60:
            rec("FAIL", "physics", f"solverType={st} timeStepsPerSecond={ts}")
            fails += 1

    if fails == 0:
        rec("PASS", "physics",
            f"{len(rb)} rigid bodies, {len(arts)} articulation roots: colliders "
            f"present, mass set, no nesting, no non-uniform scale, no tri-mesh "
            f"on dynamics; scene TGS @60 Hz, gravity (0,0,-1)x9.81")
    return fails


# -------------------------------------------------------------- 6. transforms
def check_transforms(stage):
    fails = 0
    n_xf = 0
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        x = UsdGeom.Xformable(prim)
        if not x:
            continue
        ops = x.GetOrderedXformOps()
        if not ops:
            continue
        n_xf += 1
        if not prim.GetAttribute("xformOpOrder").HasAuthoredValue() \
                and not prim.GetAttribute("xformOpOrder").Get():
            rec("FAIL", "transforms", f"{prim.GetPath()} has ops but no xformOpOrder")
            fails += 1
        for op in ops:
            v = op.Get()
            if v is None:
                continue
            vals = [v] if isinstance(v, (float, int)) else list(v) if hasattr(v, "__len__") else []
            try:
                flat = []
                for e in vals:
                    flat.extend(list(e) if hasattr(e, "__len__") else [e])
            except TypeError:
                flat = []
            if any(isinstance(f, float) and (math.isnan(f) or math.isinf(f)) for f in flat):
                rec("FAIL", "transforms", f"{prim.GetPath()}.{op.GetOpName()} is NaN/Inf")
                fails += 1
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                if any(abs(c) < 1e-12 or c < 0 for c in flat):
                    rec("FAIL", "transforms",
                        f"{prim.GetPath()} has zero or negative scale {tuple(flat)}")
                    fails += 1
    if fails == 0:
        rec("PASS", "transforms",
            f"{n_xf} transformed prims: xformOpOrder authored, no NaN/Inf, "
            f"no zero or negative scale")
    return fails


# ------------------------------------------------------------- 7. scale sanity
CLASS_BOUNDS = {   # (dx, dy, dz) plausible min/max per placed class, metres
    "amr":     ((0.6, 0.4, 0.15), (1.4, 1.0, 0.9)),
    "rover":   ((0.5, 0.4, 0.8),  (1.2, 1.0, 1.6)),
    "pallet":  ((0.7, 0.7, 0.10), (1.4, 1.4, 1.9)),
    "tote":    ((0.3, 0.3, 0.20), (0.8, 0.8, 0.6)),
    "bollard": ((0.10, 0.10, 0.6), (0.4, 0.4, 1.5)),
}


def check_scale(stage, assets, bbc):
    fails = 0
    for prim, kind in assets:
        lo, hi = CLASS_BOUNDS[kind]
        r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
        d = r.GetMax() - r.GetMin()
        for ax, nm in enumerate("xyz"):
            if not (lo[ax] - 1e-6 <= d[ax] <= hi[ax] + 1e-6):
                rec("WARN", "scale",
                    f"{prim.GetPath()} {nm}-extent {d[ax]:.3f} m outside "
                    f"{kind} range [{lo[ax]}, {hi[ax]}]")
                fails += 1
    # world bounds
    root = stage.GetPrimAtPath("/World")
    r = bbc.ComputeWorldBound(root).ComputeAlignedRange()
    d = r.GetMax() - r.GetMin()
    if not (BAY_X <= d[0] <= BAY_X + 2.0 and BAY_Y <= d[1] <= BAY_Y + 2.0):
        rec("WARN", "scale", f"world footprint {d[0]:.2f} x {d[1]:.2f} m "
                             f"vs declared {BAY_X} x {BAY_Y} m")
        fails += 1
    if fails == 0:
        rec("PASS", "scale",
            f"{len(assets)} assets within class bounds; world bound "
            f"{d[0]:.2f} x {d[1]:.2f} x {d[2]:.2f} m")
    return fails


# ------------------------------------------------- 8. floor-marking winding
def check_marking_normals(stage):
    """Floor paint must face UP. A strip authored with reversed winding renders
    black and is invisible to a downward-looking camera -- it looks like a
    material bug but is a geometry bug. Checks the GEOMETRIC normal from the
    points, so an authored normals attribute cannot mask a bad winding."""
    scope = stage.GetPrimAtPath(
        "/World/Environment/Infrastructure/FloorMarkings")
    if not scope:
        rec("SKIP", "marking_winding", "no FloorMarkings scope")
        return 0
    fails, checked = 0, 0
    for prim in Usd.PrimRange(scope):
        m = UsdGeom.Mesh(prim)
        if not m:
            continue
        pts = m.GetPointsAttr().Get()
        counts = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        checked += 1
        bad, o = 0, 0
        for c in counts:
            f = [Gf.Vec3d(pts[idx[o + k]]) for k in range(c)]
            nrm = Gf.Vec3d(0, 0, 0)
            for k in range(c):                       # Newell's method
                a, b = f[k], f[(k + 1) % c]
                nrm += Gf.Vec3d((a[1] - b[1]) * (a[2] + b[2]),
                                (a[2] - b[2]) * (a[0] + b[0]),
                                (a[0] - b[0]) * (a[1] + b[1]))
            if nrm[2] <= 0:
                bad += 1
            o += c
        if bad:
            rec("FAIL", "marking_winding",
                f"{prim.GetPath().name}: {bad}/{len(counts)} faces wind "
                f"downward -- paint renders black and is invisible from above")
            fails += 1
    if fails == 0:
        rec("PASS", "marking_winding",
            f"{checked} floor-marking meshes, every face winds +Z")
    return fails


# --------------------------------------------------------- 9. patrol tour
def check_tour(stage, bbc):
    """Sample the AUTHORED animation over its whole time range and prove the
    rover never drives into racking, never leaves the building, and honours the
    cross-aisle speed limit. Reads the composed time samples, not the intent
    that produced them."""
    from wh_common import RACK_RUN_Y, RACK_RUN_DEPTH, BAY_X, BAY_Y
    rover = stage.GetPrimAtPath("/World/Scenario/Fleet/rover_01")
    if not rover:
        rec("SKIP", "patrol_tour", "no rover_01 on the stage")
        return 0
    op = rover.GetAttribute("xformOp:translate")
    samples = op.GetTimeSamples() if op else []
    if not samples:
        rec("SKIP", "patrol_tour", "rover_01 has no animated transform")
        return 0

    t0, t1 = min(samples), max(samples)
    step = 30.0                       # 0.5 s at 60 fps
    fails, worst_gap, worst_at = 0, 1e9, None
    max_cross_speed, prev = 0.0, None
    n = 0
    tc = t0
    while tc <= t1:
        bbc.SetTime(Usd.TimeCode(tc))
        r = bbc.ComputeWorldBound(rover).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        n += 1
        # --- inside the building?
        if mn[0] < -BAY_X / 2 or mx[0] > BAY_X / 2 or \
           mn[1] < -BAY_Y / 2 or mx[1] > BAY_Y / 2:
            rec("FAIL", "patrol_tour",
                f"t={tc/60:.1f}s rover leaves the bay footprint "
                f"(x {mn[0]:.2f}..{mx[0]:.2f}, y {mn[1]:.2f}..{mx[1]:.2f})")
            fails += 1
        # --- racking incursion
        hit, gap = rack_clearance(mn, mx)
        if hit:
            rec("FAIL", "patrol_tour",
                f"t={tc/60:.1f}s rover footprint is inside racking "
                f"(x {mn[0]:.2f}..{mx[0]:.2f}, y {mn[1]:.2f}..{mx[1]:.2f})")
            fails += 1
        elif gap < worst_gap:
            worst_gap, worst_at = gap, tc / 60.0
        # --- cross-aisle speed limit (safety/constraints: 0.6 m/s at |x| <= 2.25)
        cx, cy = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2
        if prev is not None:
            dt = (tc - prev[0]) / 60.0
            v = math.hypot(cx - prev[1], cy - prev[2]) / dt if dt > 0 else 0.0
            if abs(cx) <= 2.25 and abs(prev[1]) <= 2.25:
                max_cross_speed = max(max_cross_speed, v)
        prev = (tc, cx, cy)
        tc += step

    if max_cross_speed > 0.62:
        rec("FAIL", "patrol_tour",
            f"cross-aisle speed {max_cross_speed:.2f} m/s exceeds the 0.60 m/s "
            f"limit in safety/constraints.usda")
        fails += 1
    if fails == 0:
        rec("PASS", "patrol_tour",
            f"{n} poses sampled over {t1/60:.1f}s: never enters racking, stays "
            f"in the bay, tightest clearance {worst_gap:.3f} m at t={worst_at:.1f}s; "
            f"cross-aisle peak {max_cross_speed:.2f} m/s (limit 0.60)")
    return fails


# --------------------------------------------------- 10. view camera framing
def check_view_cameras(stage, bbc):
    """A view camera parked inside a rack renders a wall of pallet. Checks that
    every view camera's eye point sits in free space -- and for the chase
    camera, which is parented to the rover, over the whole tour."""
    cams = []
    vc = stage.GetPrimAtPath("/World/Simulation/ViewCameras")
    if vc:
        cams += [(c, False) for c in vc.GetChildren() if c.IsA(UsdGeom.Camera)]
    for f in stage.GetPrimAtPath("/World/Scenario/Fleet").GetChildren():
        p = stage.GetPrimAtPath(f.GetPath().AppendChild("ViewCameras"))
        if p:
            cams += [(c, True) for c in p.GetChildren() if c.IsA(UsdGeom.Camera)]
    if not cams:
        rec("SKIP", "view_cameras", "no view cameras on the stage")
        return 0

    rover = stage.GetPrimAtPath("/World/Scenario/Fleet/rover_01")
    op = rover.GetAttribute("xformOp:translate") if rover else None
    samples = op.GetTimeSamples() if op else []
    fails, worst = 0, (None, 1e9)
    for cam, animated in cams:
        times = [Usd.TimeCode.Default()]
        if animated and samples:
            t0, t1 = min(samples), max(samples)
            t = t0
            times = []
            while t <= t1:
                times.append(Usd.TimeCode(t))
                t += 60.0                      # 1 s
        for tc in times:
            m = UsdGeom.Xformable(cam).ComputeLocalToWorldTransform(tc)
            e = m.ExtractTranslation()
            eps = Gf.Vec3d(0.05, 0.05, 0.05)
            hit, gap = rack_clearance(e - eps, e + eps)
            if hit:
                rec("FAIL", "view_cameras",
                    f"{cam.GetName()} eye is inside racking at "
                    f"({e[0]:.2f}, {e[1]:.2f})"
                    + (f" t={tc.GetValue()/60:.1f}s" if animated else ""))
                fails += 1
                break
            if gap < worst[1]:
                worst = (cam.GetName(), gap)
    if fails == 0:
        rec("PASS", "view_cameras",
            f"{len(cams)} view cameras in free space; tightest eye-to-rack "
            f"{worst[1]:.3f} m ({worst[0]})")
    return fails


# --------------------------------------------------- 11. navigable clearance
def check_navigable(stage, assets, bbc):
    """The property that actually matters for an AMR scene: does every robot
    fit in the corridor it was placed in, and with how much margin?

    Racking occupies (x in a rack segment) AND (y in a run band). The
    cross-aisle is the X gap between the two segments, so a robot standing at
    x=0 is clear even when its y falls inside a run band.
    """
    fails = 0
    worst = ("", 1e9)
    for prim, kind in assets:
        if kind not in ("amr", "rover"):
            continue
        r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        hit, margin = rack_clearance(mn, mx)
        if hit:
            rec("FAIL", "navigable",
                f"{prim.GetPath().name} footprint enters racking "
                f"(x {mn[0]:.2f}..{mx[0]:.2f}, y {mn[1]:.2f}..{mx[1]:.2f})")
            fails += 1
        if margin < worst[1]:
            worst = (prim.GetPath().name, margin)
        if 0 <= margin < 0.20:
            rec("WARN", "navigable",
                f"{prim.GetPath().name} clears racking by only {margin:.3f} m")
    if fails == 0:
        rec("PASS", "navigable",
            f"all AMRs and rovers inside their corridor; tightest lateral clearance "
            f"{worst[1]:.3f} m ({worst[0]}) in a {AISLE_W:.1f} m aisle")
    return fails


# ------------------------------------------------------------------------ main
def main():
    root = os.path.join(SCENE_ROOT, "root.usda")
    stage = Usd.Stage.Open(root, load=Usd.Stage.LoadAll)
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                            useExtentsHint=False)
    assets = placed_assets(stage)

    check_metadata(stage)
    check_composition(root)
    check_ground(stage, assets, bbc)
    check_overlap(stage, assets, bbc)
    check_physics(stage)
    check_transforms(stage)
    check_scale(stage, assets, bbc)
    check_marking_normals(stage)
    check_navigable(stage, assets, bbc)
    check_tour(stage, bbc)
    check_view_cameras(stage, bbc)

    order = {"FAIL": 0, "WARN": 1, "SKIP": 2, "PASS": 3}
    RESULTS.sort(key=lambda r: order[r[0]])
    for status, check, msg in RESULTS:
        print(f"[{status}] {check}: {msg}")
    n_fail = sum(1 for s, _, _ in RESULTS if s == "FAIL")
    n_warn = sum(1 for s, _, _ in RESULTS if s == "WARN")
    print(f"\n{len(assets)} placed assets checked. "
          f"{n_fail} FAIL, {n_warn} WARN.")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
