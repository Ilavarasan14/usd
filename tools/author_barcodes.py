"""Author barcode label prims on rack positions, ground pallets, totes,
and transfer stations.

Each barcode is a visible planar label (200x80 mm white backing + black bars)
attached to the front face of the storage position or asset. Barcodes carry:
  - a unique `barcode:id` string (e.g. "BC-R02-L01-0042")
  - a `barcode:type` token ("rack_position" | "pallet" | "tote" | "station")

The barcode prims are authored as a separate USD layer so they compose
non-destructively over the existing scene.  Material bindings reference
/World/Looks/barcode_white and barcode_black from simulation/materials.usda.
"""
import os, random
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
from wh_common import (
    SCENE_ROOT, new_layer, set_xform, floor_z,
    RACK_RUN_Y, RACK_SEG_X, RACK_END_CLEAR, UPRIGHT_W,
    BAY_PITCH, N_BAYS_PER_SEG, N_LEVELS, BEAM_LEVEL_Z,
    PALLET_GAP, PALLET_W, PALLET_H, COLUMN_X, COLUMN_CLEAR,
    FACE_OFFSET, TOTE_H, PICK_STATION, DROP_STATION,
    SEMANTIC_CLASSES, bind_visual_material,
)
from author_env import pallet_slots, rack_faces

# Visible barcode dimensions
BARCODE_W = 0.200   # 200 mm wide — visible at warehouse scale
BARCODE_H = 0.080   # 80 mm tall
BAR_COUNT = 8       # number of black bars per label
BARCODE_LIFT = 0.004  # offset from surface to prevent z-fighting

SEED = 20260822


def _barcode_quad(stage, path, center, sx, sz, normal_axis="y", normal_sign=1):
    """A single-face quad of size sx x sz.
    normal_axis: which world axis the quad faces ('x' or 'y').
    normal_sign: +1 or -1 for the facing direction.
    """
    cx, cy, cz = center
    hw, hh = sx / 2, sz / 2

    if normal_axis == "y":
        off = normal_sign * BARCODE_LIFT
        pts = [
            Gf.Vec3f(cx - hw, cy + off, cz - hh),
            Gf.Vec3f(cx + hw, cy + off, cz - hh),
            Gf.Vec3f(cx + hw, cy + off, cz + hh),
            Gf.Vec3f(cx - hw, cy + off, cz + hh),
        ]
        nrm = Gf.Vec3f(0, normal_sign, 0)
    else:
        off = normal_sign * BARCODE_LIFT
        pts = [
            Gf.Vec3f(cx + off, cy - hw, cz - hh),
            Gf.Vec3f(cx + off, cy + hw, cz - hh),
            Gf.Vec3f(cx + off, cy + hw, cz + hh),
            Gf.Vec3f(cx + off, cy - hw, cz + hh),
        ]
        nrm = Gf.Vec3f(normal_sign, 0, 0)

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([nrm] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(pts))
    set_xform(mesh.GetPrim())
    return mesh


def _author_barcode_label(stage, parent_path, center, normal_axis, normal_sign,
                          bc_id, bc_type, white_mat, black_mat):
    """Author a full barcode label: white backing + black bars + metadata."""
    grp = UsdGeom.Xform.Define(stage, parent_path)
    set_xform(grp.GetPrim())
    grp.GetPrim().CreateAttribute("barcode:id", Sdf.ValueTypeNames.String,
                                  custom=True).Set(bc_id)
    grp.GetPrim().CreateAttribute("barcode:type", Sdf.ValueTypeNames.Token,
                                  custom=True).Set(bc_type)

    # White backing
    backing = _barcode_quad(stage, parent_path + "/backing", center,
                            BARCODE_W, BARCODE_H, normal_axis, normal_sign)
    UsdShade.MaterialBindingAPI.Apply(backing.GetPrim())
    UsdShade.MaterialBindingAPI(backing.GetPrim()).Bind(
        white_mat, UsdShade.Tokens.weakerThanDescendants)

    # Black bars across the label
    cx, cy, cz = center
    bar_w = BARCODE_W * 0.06   # each bar is 6% of label width
    margin = BARCODE_W * 0.08
    usable = BARCODE_W - 2 * margin
    for bi in range(BAR_COUNT):
        bx_off = -BARCODE_W / 2 + margin + usable * bi / (BAR_COUNT - 1)
        if normal_axis == "y":
            bar_center = (cx + bx_off, cy, cz)
            extra_lift = normal_sign * 0.001
            bar_center = (cx + bx_off, cy + extra_lift, cz)
        else:
            bar_center = (cx, cy + bx_off, cz)
            extra_lift = normal_sign * 0.001
            bar_center = (cx + extra_lift, cy + bx_off, cz)
        bar = _barcode_quad(stage, f"{parent_path}/bar_{bi:02d}",
                            bar_center, bar_w, BARCODE_H * 0.75,
                            normal_axis, normal_sign)
        UsdShade.MaterialBindingAPI.Apply(bar.GetPrim())
        UsdShade.MaterialBindingAPI(bar.GetPrim()).Bind(
            black_mat, UsdShade.Tokens.weakerThanDescendants)

    return grp


def author_barcodes():
    stage = new_layer(
        "simulation/barcodes.usda",
        "Barcode labels on rack positions, ground pallets, totes, and "
        "transfer stations. Each label is a white backing + black bars, "
        "200x80 mm, visible at warehouse scale. Materials bind to "
        "/World/Looks/barcode_white and barcode_black from materials.usda.")

    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Scope.Define(stage, "/World/Simulation")
    UsdGeom.Scope.Define(stage, "/World/Simulation/Barcodes")

    # Reference materials from materials.usda (already authored)
    white_mat = UsdShade.Material.Define(stage, "/World/Looks/barcode_white")
    black_mat = UsdShade.Material.Define(stage, "/World/Looks/barcode_black")

    rng = random.Random(SEED)
    stats = {"rack_barcodes": 0, "ground_barcodes": 0,
             "tote_barcodes": 0, "station_barcodes": 0}

    # --- Rack position barcodes ---
    UsdGeom.Scope.Define(stage, "/World/Simulation/Barcodes/RackPositions")
    slots = pallet_slots()

    bc_idx = 0
    for (px, py, pz, lvl, fy) in slots:
        run_idx = None
        for ri, ry in enumerate(RACK_RUN_Y):
            if abs(fy - ry) < 2.0:
                run_idx = ri
                break
        if run_idx is None:
            continue

        normal_sign = 1 if fy > RACK_RUN_Y[run_idx] else -1
        bc_z = pz + 0.12 if lvl == 0 else pz + 0.06
        bc_id = f"BC-R{run_idx:02d}-L{lvl}-{bc_idx:04d}"
        bc_path = (f"/World/Simulation/Barcodes/RackPositions/"
                   f"bc_rack_{bc_idx:04d}")

        _author_barcode_label(
            stage, bc_path,
            (px, fy + normal_sign * 0.02, bc_z),
            "y", normal_sign, bc_id, "rack_position",
            white_mat, black_mat)
        bc_idx += 1
        stats["rack_barcodes"] += 1

    # --- Ground pallet barcodes ---
    UsdGeom.Scope.Define(stage, "/World/Simulation/Barcodes/GroundPallets")
    rng2 = random.Random(SEED)
    from wh_common import RACK_OCCUPANCY
    ground_slots = [s for s in slots if s[3] == 0]
    filled_ground = [s for s in ground_slots if rng2.random() < RACK_OCCUPANCY]

    for gi, (px, py, pz, lvl, fy) in enumerate(filled_ground):
        run_idx = 0
        for ri, ry in enumerate(RACK_RUN_Y):
            if abs(fy - ry) < 2.0:
                run_idx = ri
                break
        normal_sign = 1 if fy > RACK_RUN_Y[run_idx] else -1
        bc_id = f"BC-GP-{gi:04d}"
        bc_path = (f"/World/Simulation/Barcodes/GroundPallets/"
                   f"bc_ground_{gi:04d}")
        _author_barcode_label(
            stage, bc_path,
            (px, fy + normal_sign * 0.45, pz + PALLET_H * 0.7),
            "y", normal_sign, bc_id, "pallet",
            white_mat, black_mat)
        stats["ground_barcodes"] += 1

    # --- Tote barcodes ---
    UsdGeom.Scope.Define(stage, "/World/Simulation/Barcodes/Totes")
    from author_scenario import STAGED_PALLET_Y, STAGED_PALLET_X
    tote_idx = 0
    for i in (0, 3):
        py = STAGED_PALLET_Y[i]
        for k, dx in enumerate((-0.25, 0.25)):
            px = STAGED_PALLET_X + dx
            bc_id = f"BC-TOTE-{tote_idx:03d}"
            bc_path = (f"/World/Simulation/Barcodes/Totes/"
                       f"bc_tote_{tote_idx:03d}")
            bc_z = floor_z(px, py) + PALLET_H + TOTE_H * 0.5
            _author_barcode_label(
                stage, bc_path, (px + 0.21, py, bc_z),
                "x", 1, bc_id, "tote", white_mat, black_mat)
            tote_idx += 1
            stats["tote_barcodes"] += 1

    # Payload tote barcode
    px, py = PICK_STATION
    from wh_common import STATION_DECK_Z
    bc_id = "BC-TOTE-PAYLOAD"
    bc_path = "/World/Simulation/Barcodes/Totes/bc_tote_payload"
    bc_z = floor_z(px, py) + STATION_DECK_Z + TOTE_H * 0.5
    _author_barcode_label(
        stage, bc_path, (px + 0.21, py, bc_z),
        "x", 1, bc_id, "tote", white_mat, black_mat)
    stats["tote_barcodes"] += 1

    # --- Transfer station barcodes ---
    UsdGeom.Scope.Define(stage, "/World/Simulation/Barcodes/Stations")
    for si, (sx, sy) in enumerate((PICK_STATION, DROP_STATION)):
        bc_id = f"BC-STN-{si:02d}"
        bc_path = (f"/World/Simulation/Barcodes/Stations/"
                   f"bc_station_{si:02d}")
        bc_z = floor_z(sx, sy) + STATION_DECK_Z - 0.05
        _author_barcode_label(
            stage, bc_path, (sx + 0.46, sy, bc_z),
            "x", 1, bc_id, "station", white_mat, black_mat)
        stats["station_barcodes"] += 1

    stage.GetRootLayer().Save()
    total = sum(stats.values())
    stats["total_barcodes"] = total
    return stats


if __name__ == "__main__":
    result = author_barcodes()
    for k, v in result.items():
        print(f"  {k}: {v}")
