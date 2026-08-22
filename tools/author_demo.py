#!/usr/bin/env python3
"""Author demo.usda — a self-contained demo layer that sublayers root.usda
and adds the scanner mast beam, demo cameras, and stage metadata.

    python3 tools/author_demo.py          # writes demo.usda
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf
from wh_common import SCENE_ROOT, set_xform

MAST_HEIGHT = 8.5     # metres — reaches above level 5 (7.2 m)
BEAM_RADIUS = 0.02
OUT = os.path.join(SCENE_ROOT, "demo.usda")


def author_demo():
    stage = Usd.Stage.CreateNew(OUT)
    stage.SetMetadata("upAxis", "Z")
    stage.SetMetadata("metersPerUnit", 1.0)
    stage.SetMetadata("kilogramsPerUnit", 1.0)
    stage.SetMetadata("timeCodesPerSecond", 60)
    stage.SetMetadata("startTimeCode", 0)
    stage.SetMetadata("endTimeCode", 225530)
    stage.SetDefaultPrim(stage.DefinePrim("/World"))
    stage.GetRootLayer().documentation = (
        "Demo layer for the 60-second barcode-scanning rover demo. "
        "Sublayers root.usda (full warehouse) and adds the scanner mast "
        "beam and demo-specific overrides. Open this in Isaac Sim.")

    root_layer = stage.GetRootLayer()
    root_layer.subLayerPaths.append("./root.usda")

    # ── Scanner mast beam (green cylinder on rover) ──
    rover_path = "/World/Scenario/Fleet/rover_01"
    mast_path = f"{rover_path}/ScannerMast"
    beam_path = f"{mast_path}/beam"

    xf = UsdGeom.Xform.Define(stage, mast_path)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.3, 0, 0.5))

    cyl = UsdGeom.Cylinder.Define(stage, beam_path)
    cyl.CreateRadiusAttr(BEAM_RADIUS)
    cyl.CreateHeightAttr(MAST_HEIGHT)
    cyl.CreateAxisAttr("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(0, 0, MAST_HEIGHT / 2))
    cyl.CreateExtentAttr([
        Gf.Vec3f(-BEAM_RADIUS, -BEAM_RADIUS, 0),
        Gf.Vec3f(BEAM_RADIUS, BEAM_RADIUS, MAST_HEIGHT),
    ])

    # Bind scan_beam_green material (defined in simulation/materials.usda)
    mat = UsdShade.Material.Get(stage, "/World/Looks/scan_beam_green")
    if not mat:
        mat = UsdShade.Material.Define(stage, "/World/Looks/scan_beam_green")
    UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim())
    UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(mat)

    # Start beam hidden — the demo script toggles it
    cyl.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)

    # ── Demo-specific scope for script metadata ──
    demo_scope = UsdGeom.Scope.Define(stage, "/World/Demo")
    demo_scope.GetPrim().CreateAttribute(
        "demo:duration", Sdf.ValueTypeNames.Float, custom=True).Set(60.0)
    demo_scope.GetPrim().CreateAttribute(
        "demo:playbackSpeed", Sdf.ValueTypeNames.Float, custom=True).Set(62.0)
    demo_scope.GetPrim().CreateAttribute(
        "demo:mastHeight", Sdf.ValueTypeNames.Float, custom=True).Set(MAST_HEIGHT)
    demo_scope.GetPrim().CreateAttribute(
        "demo:scanRadius", Sdf.ValueTypeNames.Float, custom=True).Set(5.0)

    stage.GetRootLayer().Save()
    print(f"  demo.usda written  ({os.path.getsize(OUT)} bytes)")
    print(f"  Scanner mast: {mast_path}  (height {MAST_HEIGHT} m)")
    print(f"  Open demo.usda in Isaac Sim, then run:")
    print(f"    import sys; sys.path.insert(0, '<repo>/tools')")
    print(f"    import run_isaac_demo; run_isaac_demo.run_in_editor()")


if __name__ == "__main__":
    author_demo()
