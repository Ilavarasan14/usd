#!/usr/bin/env python3
"""Offline preview renders via Hydra Storm (/usr/bin/usdrecord).

    python3 tools/preview_render.py --camera <prim> --out shot.png
    python3 tools/preview_render.py --camera <prim> --out shot.#.png \
                                    --frames 0:56339x3000

This is a GEOMETRY, MOTION and FRAMING check -- not a lighting check. Storm is
not RTX: it ignores MDL (falling back to the UsdPreviewSurface each material
also carries), caps at 16 lights, and scales UsdLux intensity differently. So
it renders through a throwaway `over` layer that dials the lights down to
Storm-legible levels. The shipped lighting layers are never modified, and
nothing here says anything about how the scene exposes under RTX.

The override layer is built by ENUMERATING every UsdLux prim on the composed
stage, not from a hardcoded name list. An earlier version hardcoded the names
and silently missed 71 fixtures added later, blowing out half the frames.
"""
import argparse, glob, os, subprocess, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pxr import Usd, UsdGeom, UsdLux, Sdf
from wh_common import SCENE_ROOT

SUN_INTENSITY, SKY_INTENSITY = 2.5, 0.45


def build_preview_root(tmpdir, fixture_intensity):
    real = os.path.join(SCENE_ROOT, "root.usda")
    src = Usd.Stage.Open(real)

    lit_path = os.path.join(tmpdir, "_preview_lighting.usda")
    lit = Usd.Stage.CreateNew(lit_path)
    UsdGeom.SetStageMetersPerUnit(lit, 1.0)
    UsdGeom.SetStageUpAxis(lit, UsdGeom.Tokens.z)
    lit.SetMetadata("kilogramsPerUnit", 1.0)
    lit.GetRootLayer().documentation = (
        "Hydra Storm preview only. Never sublayered into root.usda.")

    n = {"DistantLight": 0, "DomeLight": 0, "RectLight": 0, "other": 0}
    for prim in src.Traverse():
        if not prim.HasAPI(UsdLux.LightAPI):
            continue
        ty = prim.GetTypeName()
        if ty == "DistantLight":
            v = SUN_INTENSITY
        elif ty == "DomeLight":
            v = SKY_INTENSITY
        elif ty == "RectLight":
            v = fixture_intensity
        else:
            v = fixture_intensity
            ty = "other"
        n[ty if ty in n else "other"] += 1
        o = lit.OverridePrim(prim.GetPath())
        o.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float,
                          custom=False).Set(float(v))
        o.CreateAttribute("inputs:exposure", Sdf.ValueTypeNames.Float,
                          custom=False).Set(0.0)
    lit.GetRootLayer().Save()

    root = os.path.join(tmpdir, "_preview_root.usda")
    with open(root, "w") as f:
        f.write(f'''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1.0
    kilogramsPerUnit = 1.0
    upAxis = "Z"
    timeCodesPerSecond = 60
    subLayers = [
        @{lit_path}@,
        @{real}@
    ]
)
over "World" {{}}
''')
    return root, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--fixture-intensity", type=float, default=60.0)
    ap.add_argument("--frames", default=None,
                    help="usdrecord FrameSpec, e.g. '0:56339x3000'. The output "
                         "path must then contain a '#' frame placeholder.")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

    with tempfile.TemporaryDirectory() as td:
        root, n = build_preview_root(td, a.fixture_intensity)
        if not a.quiet:
            print(f"  dimmed {sum(n.values())} lights "
                  f"({n['RectLight']} rect, {n['DistantLight']} distant, "
                  f"{n['DomeLight']} dome)")
        cmd = ["/usr/bin/usdrecord", "--camera", a.camera,
               "--imageWidth", str(a.width)]
        if a.frames:
            cmd += ["--frames", a.frames]
        cmd += [root, a.out]
        p = subprocess.run(cmd, capture_output=True, text=True)
        out = p.stdout + p.stderr
        if "Could not load sublayer" in out or "syntax error" in out:
            print("ERROR: preview lighting layer failed to compose -- the render "
                  "would silently use the scene's real light levels.")
            for line in out.splitlines():
                if "syntax error" in line or "Could not load" in line:
                    print("  " + line.strip())
            return 1
        if not a.quiet:
            for line in out.splitlines():
                if "Error" in line or "Traceback" in line:
                    print("  " + line.strip())

    hits = sorted(glob.glob(a.out.replace("#", "*"))) if "#" in a.out \
        else ([a.out] if os.path.exists(a.out) else [])
    if not hits:
        print(f"FAILED {a.out}")
        return 1
    if not a.quiet:
        print(f"wrote {len(hits)} image(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
