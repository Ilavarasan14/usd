#!/usr/bin/env python3
"""
=============================================================================
  WAREHOUSE BARCODE-SCANNING ROVER — LIVE DEMO SIMULATOR
=============================================================================

  USE CASE:  Autonomous inventory audit via barcode scanning
  ─────────────────────────────────────────────────────────

  A distribution warehouse has 1,760 rack positions across 4 rack runs and
  5 levels.  Every position carries a barcode label.  Manually auditing
  them takes a team of 3 workers an entire shift (8 hours).

  Our rover ("rover_01") autonomously patrols every aisle, stops every 4 m,
  turns to face each rack wall, and sweeps its barcode scanner across the
  storage face.  It detects obstacles (forklifts, crates, people), reroutes
  around them, and flags risks — all while collecting a complete inventory.

  MISSION PHASES:
    1. DEPLOY   → Rover starts in the centre aisle
    2. PATROL   → Sweeps south aisle, centre aisle, north aisle
    3. SCAN     → At each stop: turn +90°, scan wall, turn -90°, scan wall
    4. DETECT   → Lidar senses obstacles ahead; rover stops and reroutes
    5. ANALYZE  → After patrol: risk report with barcode coverage stats
    6. REPORT   → Final scorecard: scanned / missed / risks / grade

  This simulator replays the rover's authored mission from the USD stage,
  sampling its position every second, and prints a live ticker showing
  exactly what is happening and why.

    python3 tools/demo_simulator.py              # run the demo
    python3 tools/demo_simulator.py --fast       # 10x speed (no delays)

=============================================================================
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pxr import Usd, UsdGeom, Sdf, Gf
from wh_common import (
    SCENE_ROOT, BAY_X, BAY_Y, AISLE_Y, AISLE_W, RACK_RUN_Y,
    RACK_RUN_DEPTH, RACK_SEG_X, floor_z,
    PICK_STATION, DROP_STATION, STATION_DECK_Z,
)
from author_tour import (
    PATHS, OBSTACLES, SPEED_NORMAL, SPEED_CROSS, SCAN_DWELL, SCAN_SPACING,
    SENSE_RANGE, build_schedule,
)
from validate_scene import rack_clearance

# ── ANSI colours ──────────────────────────────────────────────────────────
R = "\033[91m"; G = "\033[92m"; Y = "\033[93m"; B = "\033[94m"
C = "\033[96m"; W = "\033[97m"; D = "\033[90m"; BOLD = "\033[1m"
RST = "\033[0m"

FAST = "--fast" in sys.argv


def pause(secs):
    if not FAST:
        time.sleep(secs)


def banner(text, color=C):
    w = 72
    print(f"\n{color}{'━'*w}")
    print(f"  {BOLD}{text}{RST}{color}")
    print(f"{'━'*w}{RST}")


def step_msg(icon, msg, color=W):
    print(f"  {color}{icon}  {msg}{RST}")


def progress_bar(pct, width=40):
    filled = int(width * pct / 100)
    bar = f"{'█' * filled}{'░' * (width - filled)}"
    return f"[{bar}] {pct:5.1f}%"


# ── MAP RENDERER ──────────────────────────────────────────────────────────
def render_map(rover_x, rover_y, scanned_aisles, obstacles_hit, phase_name):
    """Draw a top-down ASCII map of the warehouse bay."""
    W_CELLS = 60
    H_CELLS = 24
    grid = [[' ' for _ in range(W_CELLS)] for _ in range(H_CELLS)]

    def to_cell(wx, wy):
        cx = int((wx + BAY_X / 2) / BAY_X * (W_CELLS - 1))
        cy = int((BAY_Y / 2 - wy) / BAY_Y * (H_CELLS - 1))
        return max(0, min(cx, W_CELLS - 1)), max(0, min(cy, H_CELLS - 1))

    # Draw walls
    for c in range(W_CELLS):
        grid[0][c] = '─'
        grid[H_CELLS - 1][c] = '─'
    for r in range(H_CELLS):
        grid[r][0] = '│'
        grid[r][W_CELLS - 1] = '│'
    grid[0][0] = '┌'; grid[0][W_CELLS-1] = '┐'
    grid[H_CELLS-1][0] = '└'; grid[H_CELLS-1][W_CELLS-1] = '┘'

    # Draw rack runs (horizontal bands)
    for ry in RACK_RUN_Y:
        _, cy = to_cell(0, ry)
        for cx in range(2, W_CELLS - 2):
            # Gap for cross-aisle at x=0
            wx = (cx / (W_CELLS - 1)) * BAY_X - BAY_X / 2
            if abs(wx) < 3:
                continue
            if grid[cy][cx] == ' ':
                grid[cy][cx] = '▒'

    # Draw aisle labels
    for i, ay in enumerate(AISLE_Y):
        _, cy = to_cell(0, ay)
        label = f"A{i+1}"
        cx_l = 1
        for k, ch in enumerate(label):
            if cx_l + k < W_CELLS:
                grid[cy][cx_l + k] = ch

    # Draw obstacles
    for ox, oy, _ in OBSTACLES:
        cx, cy = to_cell(ox, oy)
        grid[cy][cx] = '✖'

    # Draw scanned trail
    for (sx, sy) in scanned_aisles:
        cx, cy = to_cell(sx, sy)
        if grid[cy][cx] == ' ':
            grid[cy][cx] = '·'

    # Draw rover
    rx, ry_cell = to_cell(rover_x, rover_y)
    grid[ry_cell][rx] = '◉'

    # Render
    print(f"\n  {D}{'─'*(W_CELLS+4)}{RST}")
    print(f"  {D}│{RST} {C}{BOLD}{phase_name:^{W_CELLS}s}{RST} {D}│{RST}")
    print(f"  {D}{'─'*(W_CELLS+4)}{RST}")
    for row in grid:
        line = ''.join(row)
        # Colorize
        colored = ""
        for ch in line:
            if ch == '◉':
                colored += G + BOLD + ch + RST
            elif ch == '▒':
                colored += Y + ch + RST
            elif ch == '✖':
                colored += R + ch + RST
            elif ch == '·':
                colored += B + ch + RST
            elif ch in ('│', '─', '┌', '┐', '└', '┘'):
                colored += D + ch + RST
            else:
                colored += ch
        print(f"  {D}│{RST} {colored} {D}│{RST}")
    print(f"  {D}{'─'*(W_CELLS+4)}{RST}")
    print(f"  {D}  {G}◉{RST}{D}=rover  {Y}▒{RST}{D}=rack  "
          f"{R}✖{RST}{D}=obstacle  {B}·{RST}{D}=scanned{RST}")


# ── BARCODE SCANNER ───────────────────────────────────────────────────────
def load_barcodes(stage):
    """Load all barcode positions from the composed stage."""
    bc_root = stage.GetPrimAtPath("/World/Simulation/Barcodes")
    if not bc_root:
        return []
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                            useExtentsHint=False)
    barcodes = []
    for prim in Usd.PrimRange(bc_root):
        bc_id_attr = prim.GetAttribute("barcode:id")
        if not bc_id_attr or not bc_id_attr.Get():
            continue
        bc_id = bc_id_attr.Get()
        bc_type = prim.GetAttribute("barcode:type")
        r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        cx = (r.GetMin()[0] + r.GetMax()[0]) / 2
        cy = (r.GetMin()[1] + r.GetMax()[1]) / 2
        cz = (r.GetMin()[2] + r.GetMax()[2]) / 2
        barcodes.append({
            "id": bc_id,
            "type": bc_type.Get() if bc_type else "unknown",
            "x": cx, "y": cy, "z": cz,
        })
    return barcodes


def barcodes_in_range(barcodes, rover_x, rover_y, scan_radius=6.0):
    """Find barcodes the scanner can read from this position."""
    found = []
    for bc in barcodes:
        dist = math.hypot(bc["x"] - rover_x, bc["y"] - rover_y)
        if dist <= scan_radius:
            found.append(bc)
    return found


# ── OBSTACLE DETECTOR ─────────────────────────────────────────────────────
def check_obstacles_nearby(rover_x, rover_y, radius=5.0):
    """Check for obstacles within sensing range."""
    nearby = []
    names = ["forklift", "worker", "crates", "worker", "cart", "worker"]
    for i, (ox, oy, orad) in enumerate(OBSTACLES):
        dist = math.hypot(ox - rover_x, oy - rover_y)
        if dist < radius + orad:
            nearby.append((names[i] if i < len(names) else f"obstacle_{i}",
                           dist, ox, oy))
    return nearby


# ── MAIN DEMO ─────────────────────────────────────────────────────────────
def main():
    # ── TITLE ──
    print(f"\n{C}{BOLD}")
    print(f"  ╔══════════════════════════════════════════════════════════════╗")
    print(f"  ║     WAREHOUSE AUTONOMOUS BARCODE-SCANNING ROVER DEMO       ║")
    print(f"  ╚══════════════════════════════════════════════════════════════╝{RST}")
    pause(1)

    # ── USE CASE ──
    banner("USE CASE: Autonomous Inventory Audit")
    step_msg("📋", f"Warehouse: {BAY_X:.0f} x {BAY_Y:.0f} m, "
             f"{len(RACK_RUN_Y)} rack runs, 5 levels")
    step_msg("📋", f"1,760 storage positions, each with a barcode label")
    step_msg("📋", f"Manual audit: 3 workers x 8 hours = 24 person-hours")
    step_msg("🤖", f"Rover audit: 1 robot x ~19 minutes = fully autonomous")
    step_msg("🎯", f"Goal: scan every reachable barcode, flag risks")
    pause(2)

    # ── LOAD SCENE ──
    banner("PHASE 1: Loading Warehouse Scene", B)
    root = os.path.join(SCENE_ROOT, "root.usda")
    step_msg("📂", f"Opening {root}")
    stage = Usd.Stage.Open(root, load=Usd.Stage.LoadAll)
    step_msg("✓", "USD stage composed — all layers loaded", G)
    pause(0.5)

    # Load barcodes
    step_msg("🏷️ ", "Loading barcode inventory from simulation/barcodes.usda")
    barcodes = load_barcodes(stage)
    step_msg("✓", f"{len(barcodes)} barcodes loaded", G)
    by_type = {}
    for bc in barcodes:
        by_type[bc["type"]] = by_type.get(bc["type"], 0) + 1
    for t, c in sorted(by_type.items()):
        step_msg("  ", f"  {t}: {c}")
    pause(1)

    # ── BUILD SCHEDULE ──
    banner("PHASE 2: Building Patrol Schedule", B)
    sch, phases = build_schedule()
    step_msg("🗺️ ", f"Mission: {len(phases)} patrol phases, "
             f"{sch.distance:.0f} m total distance")
    step_msg("⏱️ ", f"Duration: {sch.t:.0f} seconds "
             f"({sch.t/60:.1f} minutes)")
    step_msg("🚗", f"Cruise: {SPEED_NORMAL} m/s, "
             f"cross-aisle: {SPEED_CROSS} m/s")
    step_msg("📷", f"Scan stops every {SCAN_SPACING:.0f} m, "
             f"{SCAN_DWELL:.0f}s dwell per wall")
    print()
    for name, t0, t1, dist in phases:
        dur = t1 - t0
        step_msg("  ", f"  {name:20s}  {dist:6.1f} m  {dur:6.0f}s")
    pause(2)

    # ── SIMULATE PATROL ──
    banner("PHASE 3: Rover Patrol — LIVE", G)
    step_msg("🟢", "Mission start — rover deploying from centre aisle", G)
    pause(1)

    scanned_ids = set()
    scanned_trail = []
    obstacles_detected = []
    events_log = []
    total_duration = sch.t
    current_phase_idx = 0

    # Sample schedule at coarser intervals for the demo
    sample_interval = max(1, int(total_duration / 80))  # ~80 ticks for the demo
    tick = 0

    rover_op = stage.GetPrimAtPath("/World/Scenario/Fleet/rover_01")
    translate_attr = rover_op.GetAttribute("xformOp:translate") if rover_op else None
    time_samples = translate_attr.GetTimeSamples() if translate_attr else []

    prev_x, prev_y = None, None
    last_obstacle_report = -999
    last_scan_report = -999
    scans_this_phase = 0

    for sim_t in range(0, int(total_duration) + 1, sample_interval):
        # Get rover position from USD time samples
        tc = sim_t * 60.0  # convert seconds to time codes (60 fps)
        pos = translate_attr.Get(Usd.TimeCode(tc)) if translate_attr else None
        if not pos:
            continue

        rover_x, rover_y = pos[0], pos[1]
        tick += 1

        # Determine current phase
        phase_name = "transit"
        for pi, (pname, pt0, pt1, _) in enumerate(phases):
            if pt0 <= sim_t <= pt1:
                phase_name = pname
                if pi != current_phase_idx:
                    current_phase_idx = pi
                    print()
                    step_msg("📍", f"Entering phase: {BOLD}{pname}{RST}", C)
                    scans_this_phase = 0
                    pause(0.5)
                break

        # Check for obstacles
        nearby_obs = check_obstacles_nearby(rover_x, rover_y, SENSE_RANGE)
        for oname, odist, ox, oy in nearby_obs:
            if odist < 4.0 and sim_t - last_obstacle_report > 10:
                step_msg("⚠️ ", f"t={sim_t:4.0f}s  OBSTACLE DETECTED: {oname} "
                         f"at {odist:.1f}m — stopping, analyzing, rerouting", Y)
                obstacles_detected.append((sim_t, oname, odist))
                events_log.append(f"t={sim_t:.0f}s: Obstacle '{oname}' at "
                                  f"{odist:.1f}m, rerouted")
                last_obstacle_report = sim_t
                pause(0.3)

        # Scan barcodes when the rover is near a scan point
        in_aisle = any(abs(rover_y - ay) < AISLE_W / 2 for ay in AISLE_Y)
        if in_aisle and sim_t - last_scan_report >= SCAN_SPACING / SPEED_NORMAL:
            found = barcodes_in_range(barcodes, rover_x, rover_y, scan_radius=5.0)
            new_scans = [bc for bc in found if bc["id"] not in scanned_ids]
            if new_scans:
                for bc in new_scans:
                    scanned_ids.add(bc["id"])
                scans_this_phase += len(new_scans)
                scanned_trail.append((rover_x, rover_y))
                if len(new_scans) > 0 and sim_t - last_scan_report > 5:
                    pct = 100.0 * len(scanned_ids) / len(barcodes) if barcodes else 0
                    step_msg("📷", f"t={sim_t:4.0f}s  SCAN at "
                             f"({rover_x:+6.1f}, {rover_y:+5.1f})  "
                             f"+{len(new_scans):3d} barcodes  "
                             f"{progress_bar(pct)}", B)
                    last_scan_report = sim_t
                    pause(0.05)

        # Render map at key moments
        if tick % 20 == 1 or tick == 1:
            render_map(rover_x, rover_y, scanned_trail, obstacles_detected,
                       phase_name)
            pause(0.3)

        prev_x, prev_y = rover_x, rover_y

    # Final map
    render_map(prev_x or 0, prev_y or 0, scanned_trail, obstacles_detected,
               "MISSION COMPLETE")

    # ── RESULTS ──
    banner("PHASE 4: Barcode Scan Results", B)
    total_bc = len(barcodes)
    scanned = len(scanned_ids)
    missed = total_bc - scanned
    pct = 100.0 * scanned / total_bc if total_bc else 0

    step_msg("📊", f"Total barcodes in warehouse:    {total_bc:>6d}")
    step_msg("✅", f"Barcodes scanned by rover:      {scanned:>6d}  "
             f"({pct:.1f}%)", G)
    step_msg("❌", f"Barcodes not reached:           {missed:>6d}  "
             f"({100-pct:.1f}%)", R if missed > 0 else D)
    step_msg("⚠️ ", f"Obstacles detected & avoided:   {len(obstacles_detected):>6d}", Y)
    step_msg("⏱️ ", f"Mission duration:               {total_duration:>5.0f}s  "
             f"({total_duration/60:.1f} min)")
    step_msg("📏", f"Distance covered:               {sch.distance:>5.0f}m")
    pause(1)

    # Breakdown by barcode type
    scanned_by_type = {}
    total_by_type = {}
    for bc in barcodes:
        t = bc["type"]
        total_by_type[t] = total_by_type.get(t, 0) + 1
        if bc["id"] in scanned_ids:
            scanned_by_type[t] = scanned_by_type.get(t, 0) + 1

    print(f"\n  {'Type':<18} {'Scanned':>8} {'Total':>8} {'Coverage':>10}")
    print(f"  {'─'*48}")
    for t in sorted(total_by_type):
        s = scanned_by_type.get(t, 0)
        tot = total_by_type[t]
        p = 100.0 * s / tot if tot else 0
        color = G if p >= 90 else Y if p >= 50 else R
        print(f"  {t:<18} {s:>8} {tot:>8} {color}{p:>9.1f}%{RST}")
    pause(1)

    # ── RISK ANALYSIS ──
    banner("PHASE 5: Risk Analysis", Y)
    step_msg("🔍", "Running safety & operational risk analysis...", Y)
    pause(0.5)

    from risk_analysis import run_full_analysis
    risks = run_full_analysis(stage)

    sev_counts = {}
    for r in risks:
        sev_counts[r.severity] = sev_counts.get(r.severity, 0) + 1

    sev_colors = {"CRITICAL": R, "HIGH": R, "MEDIUM": Y, "LOW": C, "INFO": D}
    for r in risks:
        if r.severity in ("CRITICAL", "HIGH", "MEDIUM"):
            sc = sev_colors.get(r.severity, W)
            step_msg("⚡", f"[{r.severity:8s}] {r.category}: {r.title}", sc)
    print()
    for sev in ("CRITICAL", "HIGH", "MEDIUM", "LOW", "INFO"):
        if sev in sev_counts:
            sc = sev_colors.get(sev, W)
            step_msg("  ", f"  {sc}{sev:>10s}: {sev_counts[sev]}{RST}")
    pause(1)

    # ── FINAL SCORECARD ──
    banner("FINAL SCORECARD", G)

    critical = sev_counts.get("CRITICAL", 0)
    high = sev_counts.get("HIGH", 0)
    medium = sev_counts.get("MEDIUM", 0)
    low = sev_counts.get("LOW", 0)
    info = sev_counts.get("INFO", 0)
    total_risks = critical + high + medium + low + info
    # Weighted deduction: only critical/high matter for the grade
    deduction = critical * 30 + high * 8 + medium * 2
    score = max(0, min(100, 100 - deduction))
    grade = ("A" if score >= 90 else "B" if score >= 75 else
             "C" if score >= 60 else "D" if score >= 40 else "F")
    grade_color = G if grade in ("A", "B") else Y if grade == "C" else R

    print(f"""
  {BOLD}┌────────────────────────────────────────────────────────┐
  │                  MISSION SUMMARY                     │
  ├────────────────────────────────────────────────────────┤
  │  Barcode Coverage:  {pct:>5.1f}%  ({scanned}/{total_bc})            │
  │  Obstacles Avoided:   {len(obstacles_detected):>3d}                               │
  │  Mission Time:      {total_duration/60:>5.1f} min  ({sch.distance:.0f} m)           │
  │  Risk Findings:       {len(risks):>3d}  ({critical} critical, {high} high)     │
  │                                                        │
  │  Safety Score:  {grade_color}{BOLD}{score:>3d}/100  (Grade: {grade}){RST}{BOLD}                    │
  └────────────────────────────────────────────────────────┘{RST}
""")

    if grade in ("A", "B"):
        step_msg("✅", f"Warehouse passes safety baseline. "
                 f"Ready for autonomous operation.", G)
    elif grade == "C":
        step_msg("⚠️ ", f"Some risks need attention before production.", Y)
    else:
        step_msg("❌", f"Critical risks detected. Address before operating.", R)

    print(f"\n  {D}Full report: python3 tools/generate_report.py --out report.md{RST}")
    print(f"  {D}USD scene:   root.usda (open in Isaac Sim / usdview){RST}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
