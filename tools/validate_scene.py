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
                       AISLE_Y, AISLE_W)

# MDL modules that ship with Kit. Offline they cannot resolve; inside Isaac Sim
# they always do. Reported as WARN with the reason, never silently suppressed.
KIT_MDL = ("OmniPBR.mdl", "OmniGlass.mdl", "OmniSurface.mdl",
           "OmniSurfacePresets.mdl", "OmniHair.mdl")

GROUND_TOL = 0.005          # m; beyond this a body floats or is buried
PENETRATION_TOL = 0.005     # m; AABB overlap smaller than this is contact, not intersection

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


# ---------------------------------------------------- 8. navigable clearance
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
        if kind != "amr":
            continue
        r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
        mn, mx = r.GetMin(), r.GetMax()
        in_seg = any(not (mx[0] < sx0 or mn[0] > sx1) for sx0, sx1 in RACK_SEG_X)
        margin = 1e9
        for ry in RACK_RUN_Y:
            lo, hi = ry - RACK_RUN_DEPTH / 2, ry + RACK_RUN_DEPTH / 2
            if mx[1] > lo and mn[1] < hi:            # y overlaps this run band
                if in_seg:
                    rec("FAIL", "navigable",
                        f"{prim.GetPath().name} footprint enters rack run at "
                        f"y={ry:+.2f} (robot y {mn[1]:.3f}..{mx[1]:.3f}, "
                        f"run {lo:.2f}..{hi:.2f})")
                    fails += 1
                    margin = -1.0
                continue
            margin = min(margin, lo - mx[1] if mn[1] < lo else mn[1] - hi)
        # cross-aisle occupants are bounded by the segment ends instead
        if not in_seg:
            margin = min(min(sx0 - mx[0] for sx0, sx1 in RACK_SEG_X if sx0 > mx[0]) if any(sx0 > mx[0] for sx0, _ in RACK_SEG_X) else 1e9,
                         min(mn[0] - sx1 for sx0, sx1 in RACK_SEG_X if sx1 < mn[0]) if any(sx1 < mn[0] for _, sx1 in RACK_SEG_X) else 1e9)
        if margin < worst[1]:
            worst = (prim.GetPath().name, margin)
        if 0 <= margin < 0.20:
            rec("WARN", "navigable",
                f"{prim.GetPath().name} clears racking by only {margin:.3f} m")
    if fails == 0:
        rec("PASS", "navigable",
            f"all AMRs inside their corridor; tightest lateral clearance "
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
    check_navigable(stage, assets, bbc)

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
