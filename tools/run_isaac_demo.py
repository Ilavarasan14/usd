#!/usr/bin/env python3
"""
Run the barcode-scanning rover demo INSIDE Isaac Sim.

Usage (from repo root):
    <isaac-sim-python>  tools/run_isaac_demo.py
    # or, if Isaac Sim's python is on PATH:
    python  tools/run_isaac_demo.py

Alternatively, paste into Isaac Sim's Script Editor (Window > Script Editor):
    import sys, os
    sys.path.insert(0, "/path/to/hackathon/usd/tools")
    import run_isaac_demo
    run_isaac_demo.run_in_editor()
"""
import math, os, sys, time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
USD_PATH = os.path.join(SCENE_ROOT, "root.usda")

# ── Camera schedule: (seconds_into_demo, camera_prim_path, label) ─────────
CAMERA_CUTS = [
    ( 0, "/World/Simulation/OverviewCameras/warehouse_iso",  "Warehouse Overview"),
    ( 8, "/World/Scenario/Fleet/rover_01/ViewCameras/chase",  "Rover Chase Cam"),
    (20, "/World/Simulation/ViewCameras/rover_closeup",       "Barcode Scanning"),
    (30, "/World/Simulation/DroneCameras/drone_center",       "Overhead Drone"),
    (40, "/World/Scenario/Fleet/rover_01/ViewCameras/chase_top", "Rover Top-Down"),
    (50, "/World/Simulation/OverviewCameras/warehouse_iso2",  "Mission Complete"),
]

DEMO_DURATION = 60          # wall-clock seconds
PLAYBACK_SPEED = 60.0       # x real-time  (62 min patrol → 62 s)
FPS = 60                    # stage timeCodesPerSecond


def _launch_app():
    """Create the Isaac Sim application (standalone mode)."""
    config = {
        "headless": False,
        "width": 1920,
        "height": 1080,
        "anti_aliasing": "FXAA",
        "renderer": "RayTracedLighting",
        "window_title": "Warehouse Barcode Demo",
    }
    try:
        from isaacsim import SimulationApp
        return SimulationApp(config)
    except ImportError:
        pass
    try:
        from omni.isaac.kit import SimulationApp
        return SimulationApp(config)
    except ImportError:
        pass
    print("ERROR: Cannot import SimulationApp. Run this script with Isaac "
          "Sim's Python interpreter, e.g.:\n"
          "  ~/.local/share/ov/pkg/isaac-sim-*/python.sh tools/run_isaac_demo.py\n"
          "Or paste run_in_editor() into Isaac Sim's Script Editor.")
    sys.exit(1)


def _open_stage(usd_path):
    import omni.usd
    ctx = omni.usd.get_context()
    result = ctx.open_stage(usd_path)
    if not result:
        print(f"ERROR: Could not open {usd_path}")
        sys.exit(1)
    # Wait for stage to be fully loaded
    import omni.kit.app
    for _ in range(120):
        omni.kit.app.get_app().update()
    return ctx.get_stage()


def _set_camera(cam_path):
    """Switch the active viewport to the given camera prim."""
    try:
        from omni.kit.viewport.utility import get_active_viewport
        vp = get_active_viewport()
        if vp:
            vp.camera_path = cam_path
            return
    except (ImportError, AttributeError):
        pass
    try:
        import omni.kit.viewport_legacy as vp_legacy
        vpi = vp_legacy.get_viewport_interface()
        vp = vpi.get_viewport_window()
        if vp:
            vp.set_active_camera(cam_path)
            return
    except (ImportError, AttributeError):
        pass
    print(f"  [cam] viewport API not found — manually set camera to {cam_path}")


def _get_timeline():
    import omni.timeline
    return omni.timeline.get_timeline_interface()


def _build_overlay():
    """Create an on-screen HUD using omni.ui."""
    try:
        import omni.ui as ui
    except ImportError:
        return None, None, None, None

    win = ui.Window("Demo HUD", width=420, height=200,
                    flags=ui.WINDOW_FLAGS_NO_TITLE_BAR
                    | ui.WINDOW_FLAGS_NO_RESIZE
                    | ui.WINDOW_FLAGS_NO_SCROLLBAR)
    win.frame.set_style({"Window": {"background_color": 0x99000000}})

    with win.frame:
        with ui.VStack(spacing=4):
            phase_label = ui.Label("WAREHOUSE BARCODE DEMO",
                                   style={"font_size": 22,
                                          "color": 0xFF00DDFF})
            scan_label = ui.Label("Barcodes scanned: 0 / 1912",
                                  style={"font_size": 16,
                                         "color": 0xFFFFFFFF})
            bar_model = ui.FloatDrag(min=0, max=100, step=0.1)
            bar_model.model.set_value(0.0)
            progress = ui.ProgressBar(model=bar_model.model)
            progress.height = ui.Pixel(14)
            bar_model.visible = False
            status_label = ui.Label("",
                                    style={"font_size": 14,
                                           "color": 0xFF88FF88})

    return phase_label, scan_label, bar_model, status_label


def _scan_barcodes_at(stage, rx, ry, scanned_ids, all_barcodes):
    """Check which barcodes are within scanning range of rover position."""
    found = []
    for bc in all_barcodes:
        if bc["id"] in scanned_ids:
            continue
        d = math.hypot(bc["x"] - rx, bc["y"] - ry)
        if d < 5.0:
            scanned_ids.add(bc["id"])
            found.append(bc)
    return found


def _load_barcodes_from_stage(stage):
    """Load barcode metadata from the composed stage."""
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
        out.append({
            "id": a.Get(),
            "type": (p.GetAttribute("barcode:type").Get()
                     if p.GetAttribute("barcode:type") else "?"),
            "x": (r.GetMin()[0] + r.GetMax()[0]) / 2,
            "y": (r.GetMin()[1] + r.GetMax()[1]) / 2,
        })
    return out


def run_demo(stage, app_update_fn):
    """Core demo loop — works in both standalone and editor mode."""
    from pxr import Usd

    timeline = _get_timeline()

    # Load barcodes
    barcodes = _load_barcodes_from_stage(stage)
    total_bc = len(barcodes) or 1912
    scanned_ids = set()
    print(f"  Loaded {len(barcodes)} barcodes from scene")

    # Rover translate attribute (for scanning)
    rover = stage.GetPrimAtPath("/World/Scenario/Fleet/rover_01")
    t_attr = rover.GetAttribute("xformOp:translate") if rover else None

    # HUD overlay
    phase_label, scan_label, bar_model, status_label = _build_overlay()

    # Set initial camera
    _set_camera(CAMERA_CUTS[0][1])
    if phase_label:
        phase_label.text = CAMERA_CUTS[0][2]

    # Configure timeline
    timeline.set_start_time(0)
    timeline.set_end_time(225530 / FPS)  # full patrol
    timeline.set_time_codes_per_second(FPS)
    timeline.set_target_framerate(FPS)

    # Start playback
    timeline.play()
    print("  ▶ Playback started")

    t_start = time.time()
    cam_idx = 0
    last_scan_check = 0

    while True:
        elapsed = time.time() - t_start
        if elapsed >= DEMO_DURATION:
            break

        # Advance the timeline faster than real-time
        sim_time = elapsed * PLAYBACK_SPEED
        timeline.set_current_time(sim_time)

        # Camera cuts
        if cam_idx + 1 < len(CAMERA_CUTS):
            if elapsed >= CAMERA_CUTS[cam_idx + 1][0]:
                cam_idx += 1
                _set_camera(CAMERA_CUTS[cam_idx][1])
                label = CAMERA_CUTS[cam_idx][2]
                print(f"  [{elapsed:5.1f}s] 🎬 {label}")
                if phase_label:
                    phase_label.text = label

        # Barcode scanning every 0.5s wall-clock
        if t_attr and elapsed - last_scan_check > 0.5:
            tc = sim_time * FPS
            pos = t_attr.Get(Usd.TimeCode(tc))
            if pos:
                found = _scan_barcodes_at(stage, pos[0], pos[1],
                                          scanned_ids, barcodes)
                if found:
                    pct = 100.0 * len(scanned_ids) / total_bc
                    if scan_label:
                        scan_label.text = (f"Barcodes scanned: "
                                           f"{len(scanned_ids)} / {total_bc}"
                                           f"  ({pct:.1f}%)")
                    if bar_model:
                        bar_model.model.set_value(pct)
            last_scan_check = elapsed

        # Status line
        if status_label:
            remaining = DEMO_DURATION - elapsed
            status_label.text = (f"Time: {elapsed:.0f}s / {DEMO_DURATION}s  "
                                 f"  Sim: {sim_time:.0f}s  "
                                 f"  [{remaining:.0f}s remaining]")

        # Let the app render a frame
        app_update_fn()

    timeline.pause()

    # Final results
    hit = len(scanned_ids)
    pct = 100.0 * hit / total_bc
    print(f"\n  ── RESULTS ──")
    print(f"  Barcodes scanned: {hit} / {total_bc}  ({pct:.1f}%)")
    print(f"  Demo duration:    {DEMO_DURATION}s")

    if phase_label:
        phase_label.text = f"COMPLETE — {hit}/{total_bc} scanned ({pct:.1f}%)"
    if status_label:
        status_label.text = "Demo finished. Close this window or press Stop."

    # Hold the final frame for a few seconds
    for _ in range(180):
        app_update_fn()


def run_standalone():
    """Launch Isaac Sim, open the scene, and run the demo."""
    print("\n  ╔════════════════════════════════════════════════════════════╗")
    print("  ║   WAREHOUSE BARCODE DEMO — Isaac Sim                     ║")
    print("  ╚════════════════════════════════════════════════════════════╝\n")

    app = _launch_app()
    import omni.kit.app

    print(f"  Opening {USD_PATH} ...")
    stage = _open_stage(USD_PATH)
    print("  Stage loaded")

    update = omni.kit.app.get_app().update
    run_demo(stage, update)

    # Keep the window open until user closes it
    print("\n  Press Ctrl+C or close the Isaac Sim window to exit.\n")
    try:
        while app.is_running():
            app.update()
    except KeyboardInterrupt:
        pass
    app.close()


def run_in_editor():
    """Call from Isaac Sim's Script Editor when the scene is already open."""
    import omni.usd
    import omni.kit.app

    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("ERROR: No stage open. Open root.usda first.")
        return

    update = omni.kit.app.get_app().update
    run_demo(stage, update)


if __name__ == "__main__":
    run_standalone()
