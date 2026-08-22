"""Author barcode label prims on rack positions, ground pallets, totes,
and transfer stations.

Each barcode is a small planar quad (40x20 mm) attached to the front face
of the storage position or asset. Barcodes carry:
  - a unique `barcode:id` string (e.g. "BC-R02-B03-L01-P1")
  - a `barcode:type` token ("rack_position" | "pallet" | "tote" | "station")
  - SDG semantic label "barcode" for perception training

The barcode prims are authored as a separate USD layer so they compose
non-destructively over the existing scene.
"""
import os, random
from pxr import Usd, UsdGeom, Sdf, Gf
from wh_common import (
    SCENE_ROOT, new_layer, set_xform, floor_z,
    RACK_RUN_Y, RACK_SEG_X, RACK_END_CLEAR, UPRIGHT_W,
    BAY_PITCH, N_BAYS_PER_SEG, N_LEVELS, BEAM_LEVEL_Z,
    PALLET_GAP, PALLET_W, PALLET_H, COLUMN_X, COLUMN_CLEAR,
    FACE_OFFSET, TOTE_H, PICK_STATION, DROP_STATION,
    SEMANTIC_CLASSES, add_semantics,
)
from author_env import pallet_slots, rack_faces

BARCODE_W = 0.040   # 40 mm wide
BARCODE_H = 0.020   # 20 mm tall
BARCODE_LIFT = 0.003  # slight offset from the surface to prevent z-fighting

SEED = 20260822


def _barcode_quad(stage, path, center, normal_axis="y", normal_sign=1):
    """A single-face quad representing a barcode label.
    normal_axis: which axis the quad faces ('x', 'y', or 'z').
    normal_sign: +1 or -1 for the facing direction.
    """
    cx, cy, cz = center
    hw, hh = BARCODE_W / 2, BARCODE_H / 2

    if normal_axis == "y":
        off = normal_sign * BARCODE_LIFT
        pts = [
            Gf.Vec3f(cx - hw, cy + off, cz - hh),
            Gf.Vec3f(cx + hw, cy + off, cz - hh),
            Gf.Vec3f(cx + hw, cy + off, cz + hh),
            Gf.Vec3f(cx - hw, cy + off, cz + hh),
        ]
        nrm = Gf.Vec3f(0, normal_sign, 0)
    elif normal_axis == "x":
        off = normal_sign * BARCODE_LIFT
        pts = [
            Gf.Vec3f(cx + off, cy - hw, cz - hh),
            Gf.Vec3f(cx + off, cy + hw, cz - hh),
            Gf.Vec3f(cx + off, cy + hw, cz + hh),
            Gf.Vec3f(cx + off, cy - hw, cz + hh),
        ]
        nrm = Gf.Vec3f(normal_sign, 0, 0)
    else:
        off = normal_sign * BARCODE_LIFT
        pts = [
            Gf.Vec3f(cx - hw, cy - hh, cz + off),
            Gf.Vec3f(cx + hw, cy - hh, cz + off),
            Gf.Vec3f(cx + hw, cy + hh, cz + off),
            Gf.Vec3f(cx - hw, cy + hh, cz + off),
        ]
        nrm = Gf.Vec3f(0, 0, normal_sign)

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


def _set_barcode_attrs(prim, bc_id, bc_type):
    prim.CreateAttribute("barcode:id", Sdf.ValueTypeNames.String,
                         custom=True).Set(bc_id)
    prim.CreateAttribute("barcode:type", Sdf.ValueTypeNames.Token,
                         custom=True).Set(bc_type)


def author_barcodes():
    stage = new_layer(
        "simulation/barcodes.usda",
        "Barcode labels on rack positions, ground pallets, totes, and "
        "transfer stations. Each barcode carries a unique barcode:id and "
        "barcode:type for perception and inventory tracking.")

    UsdGeom.Xform.Define(stage, "/World")
    bc_scope = UsdGeom.Scope.Define(stage, "/World/Simulation")
    bc_root = UsdGeom.Scope.Define(stage, "/World/Simulation/Barcodes")

    rng = random.Random(SEED)
    stats = {"rack_barcodes": 0, "ground_barcodes": 0,
             "tote_barcodes": 0, "station_barcodes": 0}

    # --- Rack position barcodes ---
    rack_bc = UsdGeom.Scope.Define(
        stage, "/World/Simulation/Barcodes/RackPositions")
    faces = list(rack_faces())
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

        # Determine which side of the run this face is on
        normal_sign = 1 if fy > RACK_RUN_Y[run_idx] else -1

        # Barcode sits at the beam level, facing the aisle
        bc_z = pz + 0.10 if lvl == 0 else pz + 0.05
        bc_id = f"BC-R{run_idx:02d}-L{lvl}-{bc_idx:04d}"

        bc_path = (f"/World/Simulation/Barcodes/RackPositions/"
                   f"bc_rack_{bc_idx:04d}")
        quad = _barcode_quad(stage, bc_path,
                             (px, fy + normal_sign * 0.02, bc_z),
                             "y", normal_sign)
        _set_barcode_attrs(quad.GetPrim(), bc_id, "rack_position")
        bc_idx += 1
        stats["rack_barcodes"] += 1

    # --- Ground pallet barcodes (on the pallet face toward the aisle) ---
    ground_bc = UsdGeom.Scope.Define(
        stage, "/World/Simulation/Barcodes/GroundPallets")
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
        quad = _barcode_quad(stage, bc_path,
                             (px, fy + normal_sign * 0.45, pz + PALLET_H * 0.7),
                             "y", normal_sign)
        _set_barcode_attrs(quad.GetPrim(), bc_id, "pallet")
        stats["ground_barcodes"] += 1

    # --- Tote barcodes (on staged totes) ---
    tote_bc = UsdGeom.Scope.Define(
        stage, "/World/Simulation/Barcodes/Totes")
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
            quad = _barcode_quad(stage, bc_path,
                                 (px + 0.21, py, bc_z), "x", 1)
            _set_barcode_attrs(quad.GetPrim(), bc_id, "tote")
            tote_idx += 1
            stats["tote_barcodes"] += 1

    # Payload tote barcode
    px, py = PICK_STATION
    from wh_common import STATION_DECK_Z
    bc_id = "BC-TOTE-PAYLOAD"
    bc_path = "/World/Simulation/Barcodes/Totes/bc_tote_payload"
    bc_z = floor_z(px, py) + STATION_DECK_Z + TOTE_H * 0.5
    quad = _barcode_quad(stage, bc_path, (px + 0.21, py, bc_z), "x", 1)
    _set_barcode_attrs(quad.GetPrim(), bc_id, "tote")
    stats["tote_barcodes"] += 1

    # --- Transfer station barcodes ---
    stn_bc = UsdGeom.Scope.Define(
        stage, "/World/Simulation/Barcodes/Stations")
    for si, (sx, sy) in enumerate((PICK_STATION, DROP_STATION)):
        bc_id = f"BC-STN-{si:02d}"
        bc_path = (f"/World/Simulation/Barcodes/Stations/"
                   f"bc_station_{si:02d}")
        bc_z = floor_z(sx, sy) + STATION_DECK_Z - 0.05
        quad = _barcode_quad(stage, bc_path,
                             (sx + 0.46, sy, bc_z), "x", 1)
        _set_barcode_attrs(quad.GetPrim(), bc_id, "station")
        stats["station_barcodes"] += 1

    stage.GetRootLayer().Save()
    total = sum(stats.values())
    stats["total_barcodes"] = total
    return stats


if __name__ == "__main__":
    result = author_barcodes()
    for k, v in result.items():
        print(f"  {k}: {v}")
