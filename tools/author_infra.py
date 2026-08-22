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

    # ---- side-transfer stations ------------------------------------------
    st_boxes = []
    for (sx, sy) in (PICK_STATION, DROP_STATION):
        z0 = floor_z(sx, sy)
        top = STATION_DECK_Z - 0.04
        st_boxes.append((STATION_L, STATION_W, 0.04,
                         (sx, sy, z0 + STATION_DECK_Z - 0.02)))     # roller deck
        for lx in (-STATION_L / 2 + 0.06, STATION_L / 2 - 0.06):
            for ly in (-STATION_W / 2 + 0.06, STATION_W / 2 - 0.06):
                st_boxes.append((0.06, 0.06, top, (sx + lx, sy + ly, z0 + top / 2)))
    st = merge_meshes(stage, "/World/Environment/Infrastructure/transfer_stations",
                      st_boxes)
    set_xform(st.GetPrim())
    make_static_collider(st.GetPrim(), "none")
    stats["transfer_stations"] = 2

    # ---- realistic details: exit signs, fire extinguishers, drain grates ---
    details = UsdGeom.Scope.Define(stage, "/World/Environment/Infrastructure/Details")
    set_xform(details.GetPrim())
    # Exit signs on east and west walls
    for i, (ex, ey) in enumerate([(-29.0, -5.9), (-29.0, 5.9),
                                   (29.0, -5.9), (29.0, 5.9)]):
        sign = define_box_mesh(stage,
            f"/World/Environment/Infrastructure/Details/exit_sign_{i:02d}",
            0.60, 0.02, 0.20, center=(ex, ey, 3.2))
        set_xform(sign.GetPrim())
    # Fire extinguisher cabinets on columns
    for i, cx in enumerate(COLUMN_X[:2]):
        cab = define_box_mesh(stage,
            f"/World/Environment/Infrastructure/Details/fire_ext_{i:02d}",
            0.20, 0.15, 0.60, center=(cx, -11.5, 1.2))
        set_xform(cab.GetPrim())
    # Floor drain grates along the south drainage line
    for i in range(6):
        gx = -20.0 + 8.0 * i
        grate = define_box_mesh(stage,
            f"/World/Environment/Infrastructure/Details/drain_{i:02d}",
            0.30, 0.30, 0.005, center=(gx, -11.8, floor_z(gx, -11.8) + 0.001))
        set_xform(grate.GetPrim())
    # Safety striping on column bases
    for i, cx in enumerate(COLUMN_X):
        for j, ry in enumerate(RACK_RUN_Y):
            stripe = define_box_mesh(stage,
                f"/World/Environment/Infrastructure/Details/col_stripe_{i}_{j}",
                COLUMN_W + 0.02, COLUMN_W + 0.02, 0.10,
                center=(cx, ry, 0.55))
            set_xform(stripe.GetPrim())

    # ---- obstacles and human figures in the aisles -----------------------
    obs = UsdGeom.Scope.Define(stage, "/World/Environment/Infrastructure/Obstacles")
    set_xform(obs.GetPrim())
    # Parked forklift in centre aisle
    forklift = merge_meshes(stage,
        "/World/Environment/Infrastructure/Obstacles/forklift", [
        (1.8, 0.9, 0.3, (12.0, 0.0, 0.15)),
        (0.6, 0.7, 1.2, (12.6, 0.0, 0.9)),
        (0.15, 0.9, 1.8, (11.0, 0.0, 0.9)),
    ])
    set_xform(forklift.GetPrim())
    make_static_collider(forklift.GetPrim(), "convexHull")
    # Stacked crates blocking part of south aisle
    for ci in range(3):
        crate = define_box_mesh(stage,
            f"/World/Environment/Infrastructure/Obstacles/crate_{ci:02d}",
            0.50, 0.50, 0.50,
            center=(-15.0 + ci * 0.55, -5.9 + 1.2, 0.25 + (ci % 2) * 0.50))
        set_xform(crate.GetPrim())
        make_static_collider(crate.GetPrim(), "convexHull")
    # Cart in north aisle
    cart = merge_meshes(stage,
        "/World/Environment/Infrastructure/Obstacles/cart", [
        (1.0, 0.6, 0.04, (8.0, 5.9 - 1.0, 0.80)),
        (0.04, 0.04, 0.78, (7.55, 5.9 - 1.3, 0.40)),
        (0.04, 0.04, 0.78, (7.55, 5.9 - 0.7, 0.40)),
        (0.04, 0.04, 0.78, (8.45, 5.9 - 1.3, 0.40)),
        (0.04, 0.04, 0.78, (8.45, 5.9 - 0.7, 0.40)),
    ])
    set_xform(cart.GetPrim())
    make_static_collider(cart.GetPrim(), "convexHull")
    # Human figures (simplified box mannequins)
    humans = UsdGeom.Scope.Define(stage, "/World/Environment/Infrastructure/Humans")
    set_xform(humans.GetPrim())
    human_poses = [
        (5.0, 0.0, "worker near cross-aisle"),
        (-10.0, -5.9, "worker in south aisle"),
        (15.0, 5.9, "worker in north aisle"),
    ]
    for hi, (hx, hy, _) in enumerate(human_poses):
        hz = floor_z(hx, hy)
        human = merge_meshes(stage,
            f"/World/Environment/Infrastructure/Humans/person_{hi:02d}", [
            (0.35, 0.25, 0.90, (hx, hy, hz + 0.45)),
            (0.20, 0.20, 0.20, (hx, hy, hz + 1.00)),
            (0.15, 0.15, 0.45, (hx - 0.15, hy, hz + 0.70)),
            (0.15, 0.15, 0.45, (hx + 0.15, hy, hz + 0.70)),
        ])
        set_xform(human.GetPrim())
        make_static_collider(human.GetPrim(), "convexHull")

    # ---- building services, overhead --------------------------------------
    # Everything here lives in the 1.7 m of airspace between the rack top
    # (8.8 m) and the roof deck (10.5 m), threading past the light plane at
    # 9.6 m. NO colliders on any of it: the tallest thing on the floor is a
    # 1.2 m rover mast, so nothing can ever reach these -- same reasoning the
    # RackedPallets instancer uses for levels 1-4.
    svc = UsdGeom.Scope.Define(stage,
                               "/World/Environment/Infrastructure/BuildingServices")
    set_xform(svc.GetPrim())
    X0, X1 = -28.0, 28.0
    SPAN = X1 - X0

    # Wet sprinkler system: one riser main along the cross-aisle, branch lines
    # over every aisle and rack run, pendent heads at 3 m centres.
    Z_MAIN, Z_BRANCH = 10.20, 10.08
    sprink = [(0.14, 22.0, 0.14, (0.0, 0.0, Z_MAIN))]          # riser main
    n_heads = 0
    for ly in sorted(AISLE_Y + RACK_RUN_Y):
        sprink.append((SPAN, 0.09, 0.09, ((X0 + X1) / 2, ly, Z_BRANCH)))
        n = int(SPAN / 3.0)
        for i in range(n + 1):
            hx_ = X0 + SPAN * i / n
            sprink.append((0.05, 0.05, 0.14, (hx_, ly, Z_BRANCH - 0.11)))
            sprink.append((0.11, 0.11, 0.02, (hx_, ly, Z_BRANCH - 0.19)))  # deflector
            n_heads += 1
    sp_m = merge_meshes(stage,
                        "/World/Environment/Infrastructure/BuildingServices/sprinklers",
                        sprink)
    set_xform(sp_m.GetPrim())

    # HVAC: two supply ducts over the walkways, outboard of the 11.1 m light
    # line so they never shadow an aisle, with diffusers every 6 m.
    duct = []
    n_diff = 0
    for s in (-1, 1):
        dy = s * 11.5
        duct.append((SPAN, 0.50, 0.45, ((X0 + X1) / 2, dy, 9.90)))
        n = int(SPAN / 6.0)
        for i in range(n + 1):
            dx = X0 + SPAN * i / n
            duct.append((0.40, 0.40, 0.10, (dx, dy, 9.62)))
            n_diff += 1
    dm = merge_meshes(stage,
                      "/World/Environment/Infrastructure/BuildingServices/hvac_ducts",
                      duct)
    set_xform(dm.GetPrim())

    # Cable containment along both walls, under the ducts.
    tray = [(SPAN, 0.28, 0.09, ((X0 + X1) / 2, s * 11.82, 8.50)) for s in (-1, 1)]
    tm = merge_meshes(stage,
                      "/World/Environment/Infrastructure/BuildingServices/cable_trays",
                      tray)
    set_xform(tm.GetPrim())
    stats.update(sprinkler_heads=n_heads, hvac_diffusers=n_diff)

    # ---- hanging aisle identification signs --------------------------------
    # At both cross-aisle mouths of every aisle, on drop rods up to rack top.
    # Face height 4.6 m clears the 1.2 m rover mast with room to spare.
    sign_boxes = []
    n_sign = 0
    for ay in AISLE_Y:
        for sx in (-2.90, 2.90):
            sign_boxes.append((0.05, 1.20, 0.40, (sx, ay, 4.60)))     # face
            for oy in (-0.50, 0.50):                                   # drop rods
                sign_boxes.append((0.03, 0.03, RACK_H - 4.80,
                                   (sx, ay + oy, 4.80 + (RACK_H - 4.80) / 2)))
            n_sign += 1
    sg = merge_meshes(stage, "/World/Environment/Infrastructure/aisle_signs",
                      sign_boxes)
    set_xform(sg.GetPrim())
    stats["aisle_signs"] = n_sign

    # ---- dock apron staging lanes -----------------------------------------
    # Painted outbound lanes on the apron. The apron is the keep-out east of
    # x=26 (safety/constraints.usda), so this is paint only -- no obstacles
    # that could invalidate the rover's clearance margins.
    n_lane = 0
    for i, dy in enumerate(DOCK_DOOR_Y):
        for s in (-1, 1):
            floor_strip(
                stage,
                f"/World/Environment/Infrastructure/FloorMarkings/dock_lane_{i}_"
                f"{'p' if s > 0 else 'm'}",
                26.4, 29.4, dy + s * 1.5, 0.10)
            n_lane += 1
    stats["dock_lanes"] = n_lane

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
