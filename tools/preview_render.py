#!/usr/bin/env python3
"""Offline preview renders via Hydra Storm (/usr/bin/usdrecord).

    python3 tools/preview_render.py --camera /World/Simulation/ViewCameras/rover_closeup \
                                    --out /tmp/shot.png [--width 1280]

This is a GEOMETRY AND FRAMING check, not a lighting check. Storm is not RTX:
it ignores MDL (falling back to the UsdPreviewSurface each material also
carries), caps at 16 lights, and scales UsdLux intensity differently. So it
renders through a throwaway `over` layer that dials the lights down to
Storm-legible levels. The shipped lighting layers are never modified, and
nothing here says anything about how the scene will expose under RTX.
"""
import argparse, os, subprocess, sys, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wh_common import SCENE_ROOT

PREVIEW_LIGHTING = '''#usda 1.0
(
    doc = """Storm preview only. Never sublayered into root.usda."""
    metersPerUnit = 1.0
    kilogramsPerUnit = 1.0
    upAxis = "Z"
    timeCodesPerSecond = 60
)

over "World"
{
    over "Lighting"
    {
        over "sun"
        {
            float inputs:intensity = 2.5
            float inputs:exposure = 0
        }
        over "sky"
        {
            float inputs:intensity = 0.45
            float inputs:exposure = 0
        }
        over "HighBay"
        {
%s
        }
    }
}
'''


def build_preview_root(tmpdir, intensity):
    names = []
    for i in range(3):
        for k in range(10):
            names.append(f"aisle_{i}_{k:02d}")
    names += [f"cross_{k:02d}" for k in range(4)]
    names += [f"dock_{j:02d}" for j in range(4)]
    body = "\n".join(
        f'            over "{n}"\n'
        f'            {{\n'
        f'                float inputs:intensity = {intensity}\n'
        f'                float inputs:exposure = 0\n'
        f'            }}' for n in names)
    lit = os.path.join(tmpdir, "_preview_lighting.usda")
    with open(lit, "w") as f:
        f.write(PREVIEW_LIGHTING % body)
    root = os.path.join(tmpdir, "_preview_root.usda")
    real = os.path.join(SCENE_ROOT, "root.usda")
    with open(root, "w") as f:
        f.write(f'''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1.0
    kilogramsPerUnit = 1.0
    upAxis = "Z"
    timeCodesPerSecond = 60
    subLayers = [
        @{lit}@,
        @{real}@
    ]
)
over "World" {{}}
''')
    return root


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--fixture-intensity", type=float, default=90.0)
    ap.add_argument("--frames", default=None,
                    help="usdrecord FrameSpec, e.g. '0:21771x2000'. Output path "
                         "must then contain a frame placeholder like out.#.png")
    a = ap.parse_args()

    with tempfile.TemporaryDirectory() as td:
        root = build_preview_root(td, a.fixture_intensity)
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
        for line in out.splitlines():
            if "Warning" in line or "Error" in line or "Traceback" in line:
                print("  " + line.strip())
    import glob
    hits = sorted(glob.glob(a.out.replace("#", "*"))) if "#" in a.out \
        else ([a.out] if os.path.exists(a.out) else [])
    if not hits:
        print(f"FAILED {a.out}")
        return 1
    for h in hits:
        print(f"wrote {h}  ({os.path.getsize(h)//1024} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
