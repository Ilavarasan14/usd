#!/usr/bin/env python3
"""
  WAREHOUSE BARCODE-SCANNING ROVER — 60-SECOND DEMO
  ──────────────────────────────────────────────────
  Shows the rover patrolling aisles, scanning barcode labels on rack
  positions, detecting obstacles, rerouting, then printing results + risk
  analysis.  ~60 seconds with pauses, instant with --fast.

    python3 tools/demo_simulator.py          # live demo (~60s)
    python3 tools/demo_simulator.py --fast   # no pauses
"""
import math, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pxr import Usd, UsdGeom, Sdf, Gf
from wh_common import (
    SCENE_ROOT, BAY_X, BAY_Y, AISLE_Y, AISLE_W, RACK_RUN_Y,
    RACK_RUN_DEPTH,
)
from author_tour import OBSTACLES, SPEED_NORMAL, build_schedule

# ── ANSI ──────────────────────────────────────────────────────────────────
R  = "\033[91m"; G  = "\033[92m"; Y  = "\033[93m"; B  = "\033[94m"
C  = "\033[96m"; W  = "\033[97m"; D  = "\033[90m"; BOLD = "\033[1m"
RST = "\033[0m"
FAST = "--fast" in sys.argv
def pause(s):
    if not FAST: time.sleep(s)
def hdr(t, c=C):
    print(f"\n{c}{'━'*70}\n  {BOLD}{t}{RST}{c}\n{'━'*70}{RST}")
def msg(icon, t, c=W):
    print(f"  {c}{icon}  {t}{RST}")
def bar(pct, w=36):
    f = int(w * pct / 100)
    return f"[{'█'*f}{'░'*(w-f)}] {pct:5.1f}%"


# ── MAP ───────────────────────────────────────────────────────────────────
def draw_map(rx, ry, trail, phase):
    CW, CH = 60, 24
    g = [[' ']*CW for _ in range(CH)]
    def cell(wx, wy):
        return (max(0,min(int((wx+BAY_X/2)/BAY_X*(CW-1)),CW-1)),
                max(0,min(int((BAY_Y/2-wy)/BAY_Y*(CH-1)),CH-1)))
    for c in range(CW): g[0][c]='─'; g[CH-1][c]='─'
    for r in range(CH):  g[r][0]='│'; g[r][CW-1]='│'
    g[0][0]='┌'; g[0][CW-1]='┐'; g[CH-1][0]='└'; g[CH-1][CW-1]='┘'
    # Racks
    for ry_ in RACK_RUN_Y:
        _, cy = cell(0, ry_)
        for cx in range(2, CW-2):
            wx = (cx/(CW-1))*BAY_X - BAY_X/2
            if abs(wx) < 3: continue
            if g[cy][cx]==' ': g[cy][cx]='▒'
    # Barcode ticks on rack faces
    for ry_ in RACK_RUN_Y:
        for sign in (-1, 1):
            face_y = ry_ + sign * 0.75
            _, fcy = cell(0, face_y)
            for cx in range(3, CW-3, 3):
                wx = (cx/(CW-1))*BAY_X - BAY_X/2
                if abs(wx) < 3: continue
                if g[fcy][cx]==' ': g[fcy][cx]='┊'
    # Aisle labels
    for i, ay in enumerate(AISLE_Y):
        _, cy = cell(0, ay)
        for k, ch in enumerate(f"A{i+1}"): g[cy][1+k]=ch
    # Obstacles
    for i,(ox,oy,_) in enumerate(OBSTACLES):
        cx,cy = cell(ox,oy); g[cy][cx]='✖'
    # Trail
    for (sx,sy) in trail:
        cx,cy = cell(sx,sy)
        if g[cy][cx]==' ': g[cy][cx]='·'
    # Rover
    rcx,rcy = cell(rx,ry); g[rcy][rcx]='◉'
    # Render
    print(f"\n  {D}┌{'─'*(CW+2)}┐{RST}")
    print(f"  {D}│{RST} {C}{BOLD}{phase:^{CW}s}{RST} {D}│{RST}")
    print(f"  {D}├{'─'*(CW+2)}┤{RST}")
    for row in g:
        ln = ""
        for ch in row:
            if   ch=='◉': ln += G+BOLD+ch+RST
            elif ch=='▒': ln += Y+ch+RST
            elif ch=='┊': ln += W+BOLD+ch+RST
            elif ch=='✖': ln += R+ch+RST
            elif ch=='·': ln += B+ch+RST
            elif ch in '│─┌┐└┘': ln += D+ch+RST
            else: ln += ch
        print(f"  {D}│{RST} {ln} {D}│{RST}")
    print(f"  {D}└{'─'*(CW+2)}┘{RST}")
    print(f"  {D}  {G}◉{RST}{D}=rover  {Y}▒{RST}{D}=rack  "
          f"{W}{BOLD}┊{RST}{D}=barcode  {R}✖{RST}{D}=obstacle  "
          f"{B}·{RST}{D}=scanned{RST}")


# ── BARCODE LOADER ────────────────────────────────────────────────────────
def load_barcodes(stage):
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                            useExtentsHint=False)
    bc_root = stage.GetPrimAtPath("/World/Simulation/Barcodes")
    if not bc_root: return []
    out = []
    for p in Usd.PrimRange(bc_root):
        a = p.GetAttribute("barcode:id")
        if not a or not a.Get(): continue
        r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
        if r.IsEmpty(): continue
        out.append({
            "id": a.Get(),
            "type": (p.GetAttribute("barcode:type").Get()
                     if p.GetAttribute("barcode:type") else "?"),
            "x": (r.GetMin()[0]+r.GetMax()[0])/2,
            "y": (r.GetMin()[1]+r.GetMax()[1])/2,
            "z": (r.GetMin()[2]+r.GetMax()[2])/2,
        })
    return out


# ── MAIN ──────────────────────────────────────────────────────────────────
def main():
    t_wall_start = time.time()

    # ── TITLE ──
    print(f"\n{C}{BOLD}"
          f"  ╔════════════════════════════════════════════════════════════╗\n"
          f"  ║   WAREHOUSE BARCODE-SCANNING ROVER  ·  60-SECOND DEMO    ║\n"
          f"  ╚════════════════════════════════════════════════════════════╝{RST}")
    pause(3)

    # ── USE CASE ──
    hdr("WHAT THE ROBOT DOES")
    msg("📦", f"Warehouse: {BAY_X:.0f}x{BAY_Y:.0f} m, "
             f"4 rack runs, 5 levels, 1,760 storage positions")
    msg("🏷️ ", "Every position has a barcode label (white backing + black bars)")
    msg("🤖", "Rover patrols each aisle at 0.8 m/s, stops every 4 m")
    msg("📷", "Turns ±90° to face each rack wall → sweeps barcode scanner")
    msg("⚠️ ", "Lidar detects obstacles → stops, analyzes, reroutes")
    msg("📊", "Output: full barcode inventory + safety risk scorecard")
    pause(6)

    # ── LOAD ──
    hdr("STEP 1 ─ Load Scene + Barcodes", B)
    stage = Usd.Stage.Open(os.path.join(SCENE_ROOT, "root.usda"),
                           load=Usd.Stage.LoadAll)
    msg("✓", "USD stage composed (17 layers)", G)
    barcodes = load_barcodes(stage)
    by_type = {}
    for bc in barcodes:
        by_type[bc["type"]] = by_type.get(bc["type"], 0) + 1
    msg("🏷️ ", f"{len(barcodes)} barcodes loaded: "
             + ", ".join(f"{c} {t}" for t,c in sorted(by_type.items())), G)
    pause(3)

    # ── PATROL ──
    hdr("STEP 2 ─ Rover Patrol", G)

    rover = stage.GetPrimAtPath("/World/Scenario/Fleet/rover_01")
    t_attr = rover.GetAttribute("xformOp:translate")
    sch, phases = build_schedule()
    total_t = sch.t

    scanned_ids = set()
    trail = []
    obs_log = []
    last_scan_t = -999
    last_obs_t  = -999
    obs_names = ["forklift","worker","crates","worker","cart","worker"]

    pos0 = t_attr.Get(Usd.TimeCode(0))
    draw_map(pos0[0], pos0[1], trail, "DEPLOYING")
    pause(5)

    step_s = max(1, int(total_t / 35))
    map_every = step_s * 12
    prev_phase = ""

    for sim_t in range(0, int(total_t)+1, step_s):
        tc = sim_t * 60.0
        pos = t_attr.Get(Usd.TimeCode(tc))
        if not pos: continue
        rx, ry = pos[0], pos[1]

        phase = "transit"
        for pn, pt0, pt1, _ in phases:
            if pt0 <= sim_t <= pt1: phase = pn; break
        if phase != prev_phase:
            msg("📍", f"Phase: {BOLD}{phase}{RST}", C)
            prev_phase = phase
            pause(0.3)

        # Obstacle detection
        for i,(ox,oy,orad) in enumerate(OBSTACLES):
            d = math.hypot(ox-rx, oy-ry)
            if d < 4.0 and sim_t - last_obs_t > 20:
                nm = obs_names[i] if i < len(obs_names) else "obstacle"
                msg("⚠️ ", f"t={sim_t:4d}s  OBSTACLE: {nm} at {d:.1f}m "
                         f"→ stop → analyze → reroute", Y)
                obs_log.append(nm)
                last_obs_t = sim_t
                pause(2)

        # Barcode scanning
        in_aisle = any(abs(ry - ay) < AISLE_W/2 for ay in AISLE_Y)
        if in_aisle and sim_t - last_scan_t >= 5:
            found = [bc for bc in barcodes
                     if math.hypot(bc["x"]-rx, bc["y"]-ry) < 5.0
                     and bc["id"] not in scanned_ids]
            if found:
                for bc in found: scanned_ids.add(bc["id"])
                trail.append((rx, ry))
                pct = 100.0 * len(scanned_ids) / len(barcodes)
                sample = found[:3]
                ids_str = ", ".join(bc["id"] for bc in sample)
                if len(found) > 3: ids_str += f" +{len(found)-3} more"
                msg("📷", f"t={sim_t:4d}s  SCAN +{len(found):3d}  "
                         f"{bar(pct)}  {D}[{ids_str}]{RST}", B)
                last_scan_t = sim_t
                pause(1.0)

        if sim_t % map_every == 0 and sim_t > 0:
            draw_map(rx, ry, trail, phase)
            pause(3)

    last_pos = t_attr.Get(Usd.TimeCode(total_t * 60))
    draw_map(last_pos[0], last_pos[1], trail, "PATROL COMPLETE")
    pause(5)

    # ── RESULTS ──
    hdr("STEP 3 ─ Scan Results", B)
    total = len(barcodes)
    hit = len(scanned_ids)
    pct = 100.0 * hit / total if total else 0
    msg("📊", f"Barcodes scanned:  {hit:>5d} / {total}  ({pct:.1f}%)", G)
    msg("⚠️ ", f"Obstacles avoided: {len(obs_log):>5d}  "
             f"({', '.join(obs_log) if obs_log else 'none'})", Y)
    msg("⏱️ ", f"Mission time:      {total_t/60:>5.1f} min  "
             f"({sch.distance:.0f} m)")

    scanned_by = {}; total_by = {}
    for bc in barcodes:
        t = bc["type"]; total_by[t] = total_by.get(t,0)+1
        if bc["id"] in scanned_ids: scanned_by[t] = scanned_by.get(t,0)+1
    print(f"\n  {'Type':<16} {'Scanned':>8} {'Total':>7} {'Coverage':>10}")
    print(f"  {'─'*44}")
    for t in sorted(total_by):
        s = scanned_by.get(t,0); tot = total_by[t]
        p = 100.0*s/tot if tot else 0
        co = G if p>=80 else Y if p>=50 else R
        print(f"  {t:<16} {s:>8} {tot:>7} {co}{p:>9.1f}%{RST}")
    pause(6)

    # ── RISK ──
    hdr("STEP 4 ─ Risk Analysis + Score", Y)
    from risk_analysis import run_full_analysis
    risks = run_full_analysis(stage)
    sc_map = {"CRITICAL":R,"HIGH":R,"MEDIUM":Y,"LOW":C,"INFO":D}
    sev_c = {}
    for r in risks: sev_c[r.severity] = sev_c.get(r.severity,0)+1
    for r in risks:
        if r.severity in ("CRITICAL","HIGH","MEDIUM"):
            msg("⚡", f"[{r.severity:8s}] {r.category}: {r.title}",
                sc_map.get(r.severity, W))

    cr = sev_c.get("CRITICAL",0)
    hi = sev_c.get("HIGH",0)
    md = sev_c.get("MEDIUM",0)
    score = max(0, min(100, 100 - cr*30 - hi*8 - md*2))
    grade = ("A" if score>=90 else "B" if score>=75 else
             "C" if score>=60 else "D" if score>=40 else "F")
    gc = G if grade in "AB" else Y if grade=="C" else R

    print(f"""
  {BOLD}┌──────────────────────────────────────────────────┐
  │             MISSION SUMMARY                      │
  ├──────────────────────────────────────────────────┤
  │  Barcode Coverage : {pct:>5.1f}%  ({hit}/{total})          │
  │  Obstacles Avoided:   {len(obs_log):>3d}                         │
  │  Mission Time     : {total_t/60:>5.1f} min ({sch.distance:.0f} m)         │
  │  Risk Findings    :   {len(risks):>3d}  ({cr} crit, {hi} high)    │
  │                                                    │
  │  Safety Score: {gc}{BOLD}{score:>3d}/100  Grade {grade}{RST}{BOLD}                  │
  └──────────────────────────────────────────────────┘{RST}
""")
    elapsed = time.time() - t_wall_start
    msg("⏱️ ", f"Demo completed in {elapsed:.0f}s", D)
    if grade in "AB":
        msg("✅", "Ready for autonomous operation.", G)
    elif grade == "C":
        msg("⚠️ ", "Some risks to address before production.", Y)
    else:
        msg("❌", "Address high-priority findings first.", R)
    print(f"\n  {D}Open root.usda in usdview / Isaac Sim "
          f"to see barcode labels on racks{RST}\n")
    return 0

if __name__ == "__main__":
    sys.exit(main())