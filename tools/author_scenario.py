"""scenario/placements.usda, routes.usda, timeline.usda.
Transforms and references only -- no geometry is defined here."""
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
from wh_common import *

# (name, x, y, heading_deg)  heading 0 = +X. Every pose is on an aisle
# centreline and aligned to the lane direction.
FLEET = [
    ("amr_tote_01", -18.0, AISLE_Y[1],   0.0),    # centre aisle, outbound
    ("amr_tote_02",   8.0, AISLE_Y[0], 180.0),    # south aisle, inbound
    ("amr_tote_03",  -6.0, AISLE_Y[2],   0.0),    # north aisle, outbound
    ("amr_tote_04",   0.0,   8.0,      -90.0),    # cross-aisle, heading -Y
]
# Inspection rover. Centre aisle, outbound, 10 m clear of amr_tote_01.
ROVERS = [("rover_01", -8.0, AISLE_Y[1], 0.0)]
STAGED_PALLET_Y = [-8.5, -7.2, -5.9, 5.9, 7.2, 8.5]
STAGED_PALLET_X = -28.2
EMPTY_PALLET_MASS = 25.0


def author_placements():
    stage = new_layer("scenario/placements.usda",
                      "Scenario placements: AMR fleet and staged dynamic "
                      "payloads. References into assets/, transforms only.")
    UsdGeom.Xform.Define(stage, "/World")
    Usd.ModelAPI(stage.GetPrimAtPath("/World")).SetKind("assembly")
    sc = UsdGeom.Scope.Define(stage, "/World/Scenario")
    Usd.ModelAPI(sc.GetPrim()).SetKind("group")

    # ---- fleet -----------------------------------------------------------
    fleet = UsdGeom.Scope.Define(stage, "/World/Scenario/Fleet")
    for name, x, y, hdg in FLEET:
        p = UsdGeom.Xform.Define(stage, f"/World/Scenario/Fleet/{name}").GetPrim()
        p.GetReferences().AddReference("../assets/robots/amr_tote/amr_tote.usda")
        # asset origin is the ground-contact plane, so z is simply the slab height
        set_xform(p, (x, y, floor_z(x, y)), quat_from_axis_angle((0, 0, 1), hdg))
        # NOT instanceable: each robot needs its own articulation state and its
        # own sensor prims. Instance proxies are read-only.
        p.SetInstanceable(False)

    for name, x, y, hdg in ROVERS:
        p = UsdGeom.Xform.Define(stage, f"/World/Scenario/Fleet/{name}").GetPrim()
        p.GetReferences().AddReference("../assets/robots/rover/rover.usda")
        set_xform(p, (x, y, floor_z(x, y)), quat_from_axis_angle((0, 0, 1), hdg))
        p.SetInstanceable(False)

    # ---- staged dynamic pallets at the west pick/drop station -------------
    st = UsdGeom.Scope.Define(stage, "/World/Scenario/Staged")
    for i, py in enumerate(STAGED_PALLET_Y):
        px = STAGED_PALLET_X
        p = UsdGeom.Xform.Define(
            stage, f"/World/Scenario/Staged/pallet_{i:02d}").GetPrim()
        p.GetReferences().AddReference("../assets/props/pallet/pallet.usda")
        p.GetVariantSets().GetVariantSet("load").SetVariantSelection("empty")
        set_xform(p, (px, py, floor_z(px, py)),
                  quat_from_axis_angle((0, 0, 1), 0.0))
        # Dynamic body on the referencing xform; the convexHull collider comes
        # from the asset's /Collisions/hull. Deliberately NOT instanceable --
        # instance proxies are read-only and could not carry per-body physics.
        UsdPhysics.RigidBodyAPI.Apply(p)
        UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(EMPTY_PALLET_MASS)

    # ---- totes resting on two of the staged pallets ----------------------
    n_tote = 0
    for i in (0, 3):
        py = STAGED_PALLET_Y[i]
        for k, dx in enumerate((-0.25, 0.25)):
            px = STAGED_PALLET_X + dx
            p = UsdGeom.Xform.Define(
                stage, f"/World/Scenario/Staged/tote_{n_tote:02d}").GetPrim()
            p.GetReferences().AddReference("../assets/props/tote/tote.usda")
            # rotated 90deg: 0.4 m across X, 0.6 m across Y -> two fit the deck
            set_xform(p, (px, py, floor_z(px, py) + PALLET_H),
                      quat_from_axis_angle((0, 0, 1), 90.0))
            UsdPhysics.RigidBodyAPI.Apply(p)
            UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(TOTE_MASS)
            n_tote += 1

    stage.GetRootLayer().Save()
    return dict(fleet=len(FLEET), rovers=len(ROVERS),
                staged_pallets=len(STAGED_PALLET_Y),
                totes=n_tote,
                dynamic_bodies=len(STAGED_PALLET_Y) + n_tote,
                articulation_links=len(FLEET) * 7 + len(ROVERS) * 8)


def author_routes():
    stage = new_layer("scenario/routes.usda",
                      "Navigation graph: aisle and cross-aisle centrelines as "
                      "BasisCurves, plus named waypoints. Guide purpose -- "
                      "never rendered, never collided.")
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Scope.Define(stage, "/World/Scenario")
    rt = UsdGeom.Scope.Define(stage, "/World/Scenario/Routes")

    pts, counts, n_seg = [], [], 0
    for ay in AISLE_Y:                       # three aisles, along X
        n = 25
        for i in range(n):
            x = -28.0 + 56.0 * i / (n - 1)
            pts.append(Gf.Vec3f(x, ay, floor_z(x, ay) + 0.05))
        counts.append(n); n_seg += 1
    n = 13                                   # cross-aisle, along Y
    for i in range(n):
        y = -10.5 + 21.0 * i / (n - 1)
        pts.append(Gf.Vec3f(0.0, y, floor_z(0.0, y) + 0.05))
    counts.append(n); n_seg += 1

    cv = UsdGeom.BasisCurves.Define(stage, "/World/Scenario/Routes/centrelines")
    cv.CreateTypeAttr("linear")
    cv.CreateWrapAttr("nonperiodic")
    cv.CreatePointsAttr(pts)
    cv.CreateCurveVertexCountsAttr(counts)
    cv.CreateWidthsAttr([0.05] * len(pts))
    cv.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
    cv.CreateExtentAttr(UsdGeom.PointBased(cv).ComputeExtent(pts))
    set_xform(cv.GetPrim())
    UsdGeom.Imageable(cv.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)

    wp = UsdGeom.Scope.Define(stage, "/World/Scenario/Routes/Waypoints")
    named = [("pick_drop_west", -27.0, 0.0, 180.0),
             ("dock_east_a", 25.5, -6.75, 0.0),
             ("dock_east_b", 25.5, -2.25, 0.0),
             ("dock_east_c", 25.5, 2.25, 0.0),
             ("dock_east_d", 25.5, 6.75, 0.0),
             ("charge_00", -27.6, -2.4, 180.0),
             ("charge_01", -27.6, 0.0, 180.0),
             ("charge_02", -27.6, 2.4, 180.0)]
    for nm, x, y, hdg in named:
        p = UsdGeom.Xform.Define(
            stage, f"/World/Scenario/Routes/Waypoints/{nm}").GetPrim()
        set_xform(p, (x, y, floor_z(x, y)), quat_from_axis_angle((0, 0, 1), hdg))
        UsdGeom.Imageable(p).CreatePurposeAttr(UsdGeom.Tokens.guide)
    stage.GetRootLayer().Save()
    return dict(route_segments=n_seg, route_points=len(pts), waypoints=len(named))


def author_timeline():
    stage = new_layer("scenario/timeline.usda",
                      "Mission schedule metadata. No animation is authored: the "
                      "AMRs are driven by joint drives from the controller, and "
                      "time-sampled joint targets would fight it.")
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Scope.Define(stage, "/World/Scenario")
    tl = UsdGeom.Scope.Define(stage, "/World/Scenario/Timeline").GetPrim()
    tl.CreateAttribute("mission:durationSeconds", Sdf.ValueTypeNames.Double,
                       custom=True).Set(30.0)
    tl.CreateAttribute("mission:tasks", Sdf.ValueTypeNames.StringArray,
                       custom=True).Set([
        "amr_tote_01: pick_drop_west -> dock_east_c",
        "amr_tote_02: dock_east_a -> pick_drop_west",
        "amr_tote_03: charge_02 -> dock_east_d",
        "amr_tote_04: cross-aisle transit, north to south",
    ])
    stage.GetRootLayer().Save()
    return dict(tasks=4, duration_s=30.0)
