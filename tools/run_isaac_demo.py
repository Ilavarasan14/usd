#!/usr/bin/env python3
"""
Run the barcode-scanning rover demo INSIDE Isaac Sim.

The rover has a telescoping mast scanner (visible green beam) that extends
vertically to scan ALL 5 rack levels from ground level. Barcodes glow
(emissive material) so they're visible at warehouse scale.

Usage — standalone:
    <isaac-sim-python>  tools/run_isaac_demo.py

Usage — Script Editor (scene already open):
    import sys; sys.path.insert(0, "/Users/karthikeyan/hackathon/usd/tools")
    import run_isaac_demo
    run_isaac_demo.run_in_editor()
"""
import math, os, sys, time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
USD_PATH = os.path.join(SCENE_ROOT, "root.usda")

# Camera schedule: (wall_seconds, camera_prim, label)
CAMERA_CUTS = [
    ( 0, "/World/Simulation/OverviewCameras/warehouse_iso",     "WAREHOUSE OVERVIEW"),
    ( 8, "/World/Scenario/Fleet/rover_01/ViewCameras/chase",    "ROVER CHASE CAM"),
    (18, "/World/Simulation/ViewCameras/rover_closeup",         "BARCODE SCANNING — MAST EXTENDED"),
    (28, "/World/Simulation/DroneCameras/drone_center",         "OVERHEAD DRONE"),
    (38, "/World/Scenario/Fleet/rover_01/ViewCameras/chase_top","ROVER TOP-DOWN FOLLOW"),
    (48, "/World/Simulation/OverviewCameras/warehouse_side",    "SIDE VIEW — MULTI-LEVEL SCAN"),
    (55, "/World/Simulation/OverviewCameras/warehouse_iso2",    "MISSION COMPLETE"),
]

DEMO_DURATION = 60
PLAYBACK_SPEED = 62.0       # compress 62-min patrol into 60s
FPS = 60
MAST_HEIGHT = 8.5           # scanner mast reaches 8.5m (above level 5 at 7.2m)
SCAN_RADIUS_XY = 5.0        # horizontal scan range in metres

# Rack level heights (from wh_common.py)
RACK_LEVELS = [0.0, 1.8, 3.6, 5.4, 7.2]


def _launch_app():
    config = {
        "headless": False,
        "width": 1920,
        "height": 1080,
        "anti_aliasing": "FXAA",
        "renderer": "RayTracedLighting",
        "window_title": "Warehouse Barcode Scanner Demo",
    }
    for mod in ("isaacsim", "omni.isaac.kit"):
        try:
            SimulationApp = __import__(mod, fromlist=["SimulationApp"]).SimulationApp
            return SimulationApp(config)
        except (ImportError, AttributeError):
            continue
    print("ERROR: SimulationApp not found. Run with Isaac Sim's Python, or\n"
          "  paste run_in_editor() into Isaac Sim's Script Editor.")
    sys.exit(1)


def _open_stage(usd_path):
    import omni.usd
    import omni.kit.app
    ctx = omni.usd.get_context()
    if not ctx.open_stage(usd_path):
        print(f"ERROR: Could not open {usd_path}")
        sys.exit(1)
    for _ in range(180):
        omni.kit.app.get_app().update()
    return ctx.get_stage()


def _set_camera(cam_path):
    try:
        from omni.kit.viewport.utility import get_active_viewport
        vp = get_active_viewport()
        if vp:
            vp.camera_path = cam_path
            return True
    except (ImportError, AttributeError):
        pass
    try:
        import omni.kit.viewport_legacy as vl
        vp = vl.get_viewport_interface().get_viewport_window()
        if vp:
            vp.set_active_camera(cam_path)
            return True
    except (ImportError, AttributeError):
        pass
    return False


def _get_timeline():
    import omni.timeline
    return omni.timeline.get_timeline_interface()


# ── Scanner mast beam (visible green cylinder on the rover) ───────────────

def _create_scan_beam(stage):
    """Create a tall thin cylinder parented to the rover — the scanning mast."""
    from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

    # Author on the session layer so we never modify the USD files on disk
    prev_target = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())

    rover_path = "/World/Scenario/Fleet/rover_01"
    beam_path = f"{rover_path}/ScannerMast"
    beam_viz_path = f"{beam_path}/beam"

    xf = UsdGeom.Xform.Define(stage, beam_path)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.3, 0, 0.5))

    cyl = UsdGeom.Cylinder.Define(stage, beam_viz_path)
    cyl.CreateRadiusAttr(0.02)
    cyl.CreateHeightAttr(MAST_HEIGHT)
    cyl.CreateAxisAttr("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(0, 0, MAST_HEIGHT / 2))

    # Bind scan_beam_green material
    mat_path = "/World/Looks/scan_beam_green"
    mat_prim = stage.GetPrimAtPath(mat_path)
    if mat_prim:
        mat = UsdShade.Material(mat_prim)
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim())
        UsdShade.MaterialBindingAPI(cyl.GetPrim()).Bind(mat)

    # Start hidden
    cyl.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)

    stage.SetEditTarget(prev_target)
    return cyl


def _set_beam_visible(beam_cyl, visible, stage=None):
    if not beam_cyl:
        return
    from pxr import UsdGeom
    if stage:
        prev = stage.GetEditTarget()
        stage.SetEditTarget(stage.GetSessionLayer())
    beam_cyl.GetVisibilityAttr().Set(
        UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)
    if stage:
        stage.SetEditTarget(prev)


# ── Barcode loading ───────────────────────────────────────────────────────

def _load_barcodes(stage):
    from pxr import UsdGeom, Usd
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                            useExtentsHint=False)
    root = stage.GetPrimAtPath("/World/Simulation/Barcodes")
    if not root:
        return []
    out = []
    for p in Usd.PrimRange(root):
        a = p.GetAttribute("barcode:id")
        if not a or not a.Get():
            continue
        r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        mid = r.GetMidpoint()
        out.append({
            "id": a.Get(),
            "type": (p.GetAttribute("barcode:type").Get()
                     if p.GetAttribute("barcode:type") else "?"),
            "x": mid[0], "y": mid[1], "z": mid[2],
            "level": _z_to_level(mid[2]),
        })
    return out


def _z_to_level(z):
    for i, lz in enumerate(RACK_LEVELS):
        if z < lz + 0.9:
            return i
    return len(RACK_LEVELS) - 1


# ── HUD overlay ──────────────────────────────────────────────────────────

def _build_hud():
    try:
        import omni.ui as ui
    except ImportError:
        return None

    # Flag constants vary across Kit versions
    def _flag(name, fallback=0):
        for attr in (name, name.replace("WINDOW_FLAGS_", "")):
            if hasattr(ui, attr):
                return getattr(ui, attr)
        return fallback

    flags = (_flag("WINDOW_FLAGS_NO_TITLE_BAR")
             | _flag("WINDOW_FLAGS_NO_RESIZE")
             | _flag("WINDOW_FLAGS_NO_SCROLLBAR")
             | _flag("WINDOW_FLAGS_NO_MOVE"))

    class HUD:
        def __init__(self):
            self.win = ui.Window("Barcode Scanner HUD", width=460, height=230,
                                 flags=flags)
            self.win.frame.set_style({
                "Window": {"background_color": 0xCC111111,
                           "border_radius": 8}
            })
            with self.win.frame:
                with ui.VStack(spacing=6):
                    ui.Spacer(height=4)
                    self.title = ui.Label(
                        "  WAREHOUSE BARCODE SCANNER",
                        style={"font_size": 20, "color": 0xFF00EEFF})
                    ui.Spacer(height=2)
                    self.phase = ui.Label(
                        "  Initializing...",
                        style={"font_size": 15, "color": 0xFFCCCCCC})
                    self.scan_text = ui.Label(
                        "  Scanned: 0 / 1912  (0.0%)",
                        style={"font_size": 16, "color": 0xFF44FF66})
                    self.level_text = ui.Label(
                        "  Levels: L0 ░░  L1 ░░  L2 ░░  L3 ░░  L4 ░░",
                        style={"font_size": 13, "color": 0xFFAAAAFF})
                    with ui.HStack(height=16, spacing=4):
                        ui.Spacer(width=8)
                        self._bar_bg = ui.Rectangle(
                            style={"background_color": 0xFF333333,
                                   "border_radius": 4})
                    self.time_text = ui.Label(
                        "  00:00 / 01:00",
                        style={"font_size": 13, "color": 0xFF888888})
                    ui.Spacer(height=4)

        def update(self, phase, scanned, total, by_level, elapsed, beam_on):
            pct = 100.0 * scanned / total if total else 0
            self.phase.text = f"  {phase}"
            scan_icon = "🔍" if beam_on else "📷"
            self.scan_text.text = (f"  {scan_icon} Scanned: {scanned} / "
                                   f"{total}  ({pct:.1f}%)")
            parts = []
            for i in range(5):
                s = by_level.get(i, 0)
                bar = "██" if s > 50 else "▓▓" if s > 10 else "░░"
                parts.append(f"L{i} {bar}{s}")
            self.level_text.text = "  Levels: " + "  ".join(parts)
            m, s = divmod(int(elapsed), 60)
            self.time_text.text = f"  {m:02d}:{s:02d} / 01:00"

        def final(self, scanned, total):
            pct = 100.0 * scanned / total if total else 0
            self.title.text = "  ✅ SCAN COMPLETE"
            self.phase.text = f"  {scanned}/{total} barcodes ({pct:.1f}%)"
            self.scan_text.text = "  All 5 rack levels scanned via mast scanner"

        def destroy(self):
            self.win.visible = False

    return HUD()


# ── Core demo loop ───────────────────────────────────────────────────────

def run_demo(stage, app_update):
    from pxr import Usd, UsdGeom

    timeline = _get_timeline()

    # Load scene data
    barcodes = _load_barcodes(stage)
    total_bc = len(barcodes) or 1912
    scanned_ids = set()
    by_level = {i: 0 for i in range(5)}
    print(f"  📦 {len(barcodes)} barcodes across {len(RACK_LEVELS)} levels")

    # Rover
    rover = stage.GetPrimAtPath("/World/Scenario/Fleet/rover_01")
    t_attr = rover.GetAttribute("xformOp:translate") if rover else None

    # Scanner mast beam
    beam_cyl = _create_scan_beam(stage)
    beam_on = False

    # HUD
    hud = _build_hud()

    # Camera
    _set_camera(CAMERA_CUTS[0][1])

    # Timeline — scrub only, don't play (avoids starting physics)
    timeline.set_start_time(0)
    timeline.set_end_time(225530 / FPS)
    timeline.set_time_codes_per_second(FPS)
    print("  ▶  Demo started (62x scrub, 60-second demo)")

    t0 = time.time()
    cam_idx = 0
    last_scan = 0
    scan_interval = 0.4

    # Aisle Y positions
    AISLE_Y = [-5.9, 0.0, 5.9]
    AISLE_HALF = 1.6

    while True:
        elapsed = time.time() - t0
        if elapsed >= DEMO_DURATION:
            break

        sim_t = elapsed * PLAYBACK_SPEED
        timeline.set_current_time(sim_t)

        # Camera cuts
        if cam_idx + 1 < len(CAMERA_CUTS):
            if elapsed >= CAMERA_CUTS[cam_idx + 1][0]:
                cam_idx += 1
                _set_camera(CAMERA_CUTS[cam_idx][1])
                print(f"  [{elapsed:5.1f}s] 🎬 {CAMERA_CUTS[cam_idx][2]}")

        # Barcode scanning with vertical mast
        if t_attr and elapsed - last_scan > scan_interval:
            tc = sim_t * FPS
            pos = t_attr.Get(Usd.TimeCode(tc))
            if pos:
                rx, ry = pos[0], pos[1]
                in_aisle = any(abs(ry - ay) < AISLE_HALF for ay in AISLE_Y)

                if in_aisle:
                    if not beam_on:
                        _set_beam_visible(beam_cyl, True, stage)
                        beam_on = True

                    found = [bc for bc in barcodes
                             if bc["id"] not in scanned_ids
                             and math.hypot(bc["x"] - rx, bc["y"] - ry)
                                < SCAN_RADIUS_XY]
                    for bc in found:
                        scanned_ids.add(bc["id"])
                        by_level[bc["level"]] = by_level.get(bc["level"], 0) + 1
                else:
                    if beam_on:
                        _set_beam_visible(beam_cyl, False, stage)
                        beam_on = False

            last_scan = elapsed

        # Update HUD
        phase = CAMERA_CUTS[cam_idx][2]
        if hud:
            hud.update(phase, len(scanned_ids), total_bc,
                       by_level, elapsed, beam_on)

        app_update()

    # Finish
    _set_beam_visible(beam_cyl, False, stage)

    # Clean up session-layer beam prim
    prev_target = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    beam_prim = stage.GetPrimAtPath(
        "/World/Scenario/Fleet/rover_01/ScannerMast")
    if beam_prim:
        stage.RemovePrim(beam_prim.GetPath())
    stage.SetEditTarget(prev_target)

    hit = len(scanned_ids)
    pct = 100.0 * hit / total_bc
    print(f"\n  ── RESULTS ──")
    print(f"  Barcodes scanned : {hit} / {total_bc}  ({pct:.1f}%)")
    for i in range(5):
        z = RACK_LEVELS[i] if i < len(RACK_LEVELS) else 0
        print(f"    Level {i} (z={z:.1f}m) : {by_level.get(i, 0)} scanned")
    print(f"  Demo duration    : {DEMO_DURATION}s")

    if hud:
        hud.final(hit, total_bc)

    for _ in range(240):
        app_update()

    if hud:
        hud.destroy()


def run_standalone():
    print("\n  ╔════════════════════════════════════════════════════════════╗")
    print("  ║  WAREHOUSE BARCODE SCANNER — Isaac Sim Demo              ║")
    print("  ║  Telescoping mast scans ALL 5 rack levels                ║")
    print("  ╚════════════════════════════════════════════════════════════╝\n")

    app = _launch_app()
    import omni.kit.app

    print(f"  Opening {USD_PATH} ...")
    stage = _open_stage(USD_PATH)
    print("  ✓ Stage loaded\n")

    run_demo(stage, omni.kit.app.get_app().update)

    print("\n  Press Ctrl+C or close Isaac Sim to exit.\n")
    try:
        while app.is_running():
            app.update()
    except KeyboardInterrupt:
        pass
    app.close()


def run_in_editor():
    """Call from Isaac Sim's Script Editor when root.usda is already open."""
    import omni.usd
    import omni.kit.app
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("ERROR: No stage open. Open root.usda first.")
        return
    run_demo(stage, omni.kit.app.get_app().update)


if __name__ == "__main__":
    run_standalone()
