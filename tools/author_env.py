"""Authors environment/shell.usdc, racking.usdc, infrastructure.usdc."""
import math
from pxr import Usd, UsdGeom, Sdf, Gf
from wh_common import *


def _root(stage, kind_world="assembly"):
    w = UsdGeom.Xform.Define(stage, "/World")
    Usd.ModelAPI(w.GetPrim()).SetKind(kind_world)
    set_xform(w.GetPrim())
    env = UsdGeom.Scope.Define(stage, "/World/Environment")
    Usd.ModelAPI(env.GetPrim()).SetKind("group")
    return w, env


# ------------------------------------------------------------------ shell
def author_shell():
    stage = new_layer("environment/shell.usdc",
                      "Building shell: slab, tilt-up walls, roof deck, columns. "
                      "All static colliders (CollisionAPI only, triangle mesh).")
    _root(stage)
    grp = UsdGeom.Xform.Define(stage, "/World/Environment/Shell")
    Usd.ModelAPI(grp.GetPrim()).SetKind("group")
    set_xform(grp.GetPrim())

    # ---- floor slab: displaced grid, 0.5% fall to the drain line at Y=-12
    hx0, hy0 = BAY_X / 2, BAY_Y / 2
    nx = int(BAY_X / FLOOR_RES) + 1
    ny = int(BAY_Y / FLOOR_RES) + 1
    pts, sts = [], []
    for j in range(ny):
        y = -BAY_Y / 2 + j * FLOOR_RES
        for i in range(nx):
            x = -BAY_X / 2 + i * FLOOR_RES
            pts.append(Gf.Vec3f(x, y, floor_z(x, y)))
    counts, idx = [], []
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = j * nx + i
            counts.append(4)
            idx.extend([a, a + 1, a + nx + 1, a + nx])
    floor = UsdGeom.Mesh.Define(stage, "/World/Environment/Shell/floor_slab")
    floor.CreatePointsAttr(pts)
    floor.CreateFaceVertexCountsAttr(counts)
    floor.CreateFaceVertexIndicesAttr(idx)
    floor.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    floor.CreateExtentAttr(UsdGeom.PointBased(floor).ComputeExtent(pts))
    papi = UsdGeom.PrimvarsAPI(floor.GetPrim())
    pv = papi.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                            UsdGeom.Tokens.vertex)
    pv.Set([Gf.Vec2f((p[0] + 30) / 6.0, (p[1] + 12) / 6.0) for p in pts])

    # Controlled imperfection, driven by where the work actually happens:
    #   wear  -- tyre polish along aisle and cross-aisle running lines
    #   grime -- dirt accumulation against wall bases and under rack runs
    # Consumed as roughness / base-colour modulation by the floor MDL. One
    # material, varied per-vertex -- not a second material.
    wear, grime = [], []
    for p in pts:
        x, y = float(p[0]), float(p[1])
        w = max([math.exp(-((y - ay) / 1.6) ** 2) for ay in AISLE_Y])
        w = max(w, math.exp(-(x / 2.6) ** 2))
        wear.append(min(1.0, w))
        dw = min(abs(x + hx0), abs(x - hx0), abs(y + hy0), abs(y - hy0))
        g = math.exp(-(dw / 0.6) ** 2)
        g = max(g, 0.55 * max([math.exp(-((y - ry) / 1.2) ** 2) for ry in RACK_RUN_Y]))
        grime.append(min(1.0, g))
    papi.CreatePrimvar("wear", Sdf.ValueTypeNames.FloatArray,
                       UsdGeom.Tokens.vertex).Set(wear)
    papi.CreatePrimvar("grime", Sdf.ValueTypeNames.FloatArray,
                       UsdGeom.Tokens.vertex).Set(grime)
    set_xform(floor.GetPrim())
    make_static_collider(floor.GetPrim(), "none")

    # ---- walls. Solid on N/S/W; E wall carries four dock door openings.
    hx, hy = BAY_X / 2, BAY_Y / 2
    walls = []
    walls.append((BAY_X + 2 * WALL_T, WALL_T, CLEAR_H,
                  (0, hy + WALL_T / 2, CLEAR_H / 2)))          # north  Y=+12
    walls.append((BAY_X + 2 * WALL_T, WALL_T, CLEAR_H,
                  (0, -hy - WALL_T / 2, CLEAR_H / 2)))         # south  Y=-12
    merge_meshes(stage, "/World/Environment/Shell/wall_north",
                 [walls[0]]).GetPrim()
    merge_meshes(stage, "/World/Environment/Shell/wall_south", [walls[1]])

    # west wall with one personnel door (0.9 x 2.1 m at Y=+10)
    pd_y, pd_w, pd_h = 10.0, 0.9, 2.1
    west = []
    segs = [(-hy, pd_y - pd_w / 2), (pd_y + pd_w / 2, hy)]
    for y0, y1 in segs:
        west.append((WALL_T, y1 - y0, pd_h, (-hx - WALL_T / 2, (y0 + y1) / 2, pd_h / 2)))
    west.append((WALL_T, BAY_Y, CLEAR_H - pd_h,
                 (-hx - WALL_T / 2, 0, pd_h + (CLEAR_H - pd_h) / 2)))
    merge_meshes(stage, "/World/Environment/Shell/wall_west", west)

    # east wall: 4 dock openings, header spanning full width above them
    door_ys = [-6.75, -2.25, 2.25, 6.75]
    east = []
    edges = [-hy]
    for dy in door_ys:
        edges += [dy - DOCK_DOOR_W / 2, dy + DOCK_DOOR_W / 2]
    edges.append(hy)
    for k in range(0, len(edges), 2):
        y0, y1 = edges[k], edges[k + 1]
        if y1 - y0 > 1e-6:
            east.append((WALL_T, y1 - y0, DOCK_DOOR_H,
                         (hx + WALL_T / 2, (y0 + y1) / 2, DOCK_DOOR_H / 2)))
    east.append((WALL_T, BAY_Y, CLEAR_H - DOCK_DOOR_H,
                 (hx + WALL_T / 2, 0, DOCK_DOOR_H + (CLEAR_H - DOCK_DOOR_H) / 2)))
    merge_meshes(stage, "/World/Environment/Shell/wall_east", east)

    for n in ("wall_north", "wall_south", "wall_east", "wall_west"):
        p = stage.GetPrimAtPath(f"/World/Environment/Shell/{n}")
        set_xform(p)
        make_static_collider(p, "none")

    # ---- roof deck at 10.5 m, panelised so skylights can be cut out
    sky = []
    for ay in AISLE_Y:
        for ax in (-22.5, -7.5, 7.5, 22.5):
            sky.append((ax - 1.5, ax + 1.5, ay - 0.75, ay + 0.75))
    cell = 1.5
    panels = []
    n_sky = 0
    for j in range(int(BAY_Y / cell)):
        cy = -hy + (j + 0.5) * cell
        for i in range(int(BAY_X / cell)):
            cx = -hx + (i + 0.5) * cell
            if any(x0 <= cx <= x1 and y0 <= cy <= y1 for x0, x1, y0, y1 in sky):
                n_sky += 1
                continue
            panels.append((cell, cell, 0.15, (cx, cy, CLEAR_H + 0.075)))
    roof = merge_meshes(stage, "/World/Environment/Shell/roof_deck", panels)
    set_xform(roof.GetPrim())
    make_static_collider(roof.GetPrim(), "none")
    UsdGeom.Imageable(roof.GetPrim()).CreateVisibilityAttr().Set("invisible")

    # ---- structural columns, absorbed into the rack runs (never in an aisle)
    cols = []
    for ry in RACK_RUN_Y:
        for cx in COLUMN_X:
            cols.append((COLUMN_W, COLUMN_W, CLEAR_H, (cx, ry, CLEAR_H / 2)))
    colm = merge_meshes(stage, "/World/Environment/Shell/columns", cols)
    set_xform(colm.GetPrim())
    make_static_collider(colm.GetPrim(), "none")

    stage.GetRootLayer().Save()
    return dict(floor_points=len(pts), floor_faces=len(counts),
                wear_mean=round(sum(wear) / len(wear), 3),
                grime_mean=round(sum(grime) / len(grime), 3),
                roof_panels=len(panels), skylight_cells=n_sky, columns=len(cols))


# ---------------------------------------------------------------- racking
def rack_faces():
    """Yield (face_y, front_upright_y, back_upright_y, aisle_y) per rack face."""
    for ry in RACK_RUN_Y:
        for side in (-1, 1):
            fy = ry + side * FACE_OFFSET
            yield fy, fy - FRAME_DEPTH_CLEAR / 2, fy + FRAME_DEPTH_CLEAR / 2


def pallet_slots():
    """Every storage position in the bay: (x, y, z_base, level, face_y)."""
    out = []
    for fy, uy0, uy1 in rack_faces():
        for (sx0, sx1) in RACK_SEG_X:
            x0 = sx0 + RACK_END_CLEAR + UPRIGHT_W / 2
            for b in range(N_BAYS_PER_SEG):
                bay_left = x0 + b * BAY_PITCH + UPRIGHT_W / 2
                for j in range(3):
                    px = bay_left + PALLET_GAP + PALLET_W / 2 + j * (PALLET_W + PALLET_GAP)
                    if any(abs(px - cx) < COLUMN_CLEAR for cx in COLUMN_X):
                        continue          # this position is taken by a column
                    for lvl in range(N_LEVELS):
                        z = floor_z(px, fy) if lvl == 0 else BEAM_LEVEL_Z[lvl - 1]
                        out.append((px, fy, z, lvl, fy))
    return out


def author_racking():
    stage = new_layer("environment/racking.usdc",
                      "Selective pallet racking: uprights, braces, beams, end guards. "
                      "One merged mesh per rack run. Static triangle-mesh colliders.")
    _root(stage)
    grp = UsdGeom.Xform.Define(stage, "/World/Environment/Racking")
    Usd.ModelAPI(grp.GetPrim()).SetKind("group")
    set_xform(grp.GetPrim())

    n_box = 0
    for r, ry in enumerate(RACK_RUN_Y):
        boxes = []
        for side in (-1, 1):
            fy = ry + side * FACE_OFFSET
            uy = [fy - FRAME_DEPTH_CLEAR / 2, fy + FRAME_DEPTH_CLEAR / 2]
            for (sx0, sx1) in RACK_SEG_X:
                x0 = sx0 + RACK_END_CLEAR + UPRIGHT_W / 2
                # frames
                for f in range(N_BAYS_PER_SEG + 1):
                    fx = x0 + f * BAY_PITCH
                    for y in uy:
                        boxes.append((UPRIGHT_W, UPRIGHT_W, RACK_H, (fx, y, RACK_H / 2)))
                    for bz in [0.4 + k * 1.2 for k in range(7)]:
                        boxes.append((0.05, FRAME_DEPTH_CLEAR, 0.05, (fx, fy, bz)))
                # beams
                for b in range(N_BAYS_PER_SEG):
                    bcx = x0 + b * BAY_PITCH + BAY_PITCH / 2
                    for bz in BEAM_LEVEL_Z:
                        for y in uy:
                            boxes.append((BEAM_CLEAR, 0.05, BEAM_H,
                                          (bcx, y, bz - BEAM_H / 2)))
                # rack-end guards -- steel bollard pairs protecting each run end
                for ex in (sx0 + RACK_END_CLEAR / 2, sx1 - RACK_END_CLEAR / 2):
                    boxes.append((0.12, 0.12, 0.45, (ex, fy, 0.225)))
        n_box += len(boxes)
        m = merge_meshes(stage, f"/World/Environment/Racking/rack_run_{r:02d}", boxes)
        set_xform(m.GetPrim())
        make_static_collider(m.GetPrim(), "none")

    stage.GetRootLayer().Save()
    return dict(rack_runs=len(RACK_RUN_Y), steel_boxes=n_box,
                storage_positions=len(pallet_slots()))
