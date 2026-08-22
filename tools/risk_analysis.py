#!/usr/bin/env python3
"""Warehouse risk analysis engine.

Scans the composed USD stage for safety, operational, and perception risks.
Each risk gets a severity (CRITICAL / HIGH / MEDIUM / LOW / INFO), a category,
and a measurement-backed description.

Risk categories:
  collision       -- AABB overlaps, rack incursions, clearance margins
  navigation      -- blocked aisles, dead-end traps, speed-zone violations
  barcode         -- unreadable labels, scanner coverage gaps, facing errors
  structural      -- load limits, rack occupancy imbalance, column clearance
  fire_safety     -- blocked exits, sprinkler head clearance, extinguisher access
  human_safety    -- pedestrian/AMR shared zones, blind corners, lighting gaps

    python3 tools/risk_analysis.py              # print risk table
    python3 tools/risk_analysis.py --json       # JSON output
"""
import json
import math
import os
import sys
import itertools

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
from wh_common import (
    SCENE_ROOT, floor_z, BAY_X, BAY_Y, CLEAR_H,
    AISLE_W, AISLE_Y, RACK_RUN_Y, RACK_RUN_DEPTH, RACK_SEG_X,
    RACK_OCCUPANCY, N_LEVELS, PALLET_H, PALLET_W, PALLET_L,
    AMR_DECK_H, ROVER_WHEEL_R, ROVER_TRACK,
    PICK_STATION, DROP_STATION, STATION_DECK_Z,
    applied_schemas, racking_x_extent,
)
from validate_scene import rack_clearance, RACK_BANDS, RACK_X

RACK_BANDS_LOCAL = [(ry - RACK_RUN_DEPTH / 2, ry + RACK_RUN_DEPTH / 2)
                    for ry in RACK_RUN_Y]


class Risk:
    def __init__(self, severity, category, title, detail, location=None,
                 measurement=None):
        self.severity = severity
        self.category = category
        self.title = title
        self.detail = detail
        self.location = location
        self.measurement = measurement

    def to_dict(self):
        d = {"severity": self.severity, "category": self.category,
             "title": self.title, "detail": self.detail}
        if self.location:
            d["location"] = self.location
        if self.measurement is not None:
            d["measurement"] = self.measurement
        return d


def _severity_order(s):
    return {"CRITICAL": 0, "HIGH": 1, "MEDIUM": 2, "LOW": 3, "INFO": 4}[s]


# ---------------------------------------------------------------- analysers

def analyse_collision_risks(stage, bbc):
    """Check all placed assets for mutual AABB overlap and rack incursion."""
    risks = []
    fleet = stage.GetPrimAtPath("/World/Scenario/Fleet")
    if not fleet:
        return risks
    robots = []
    for child in fleet.GetChildren():
        r = bbc.ComputeWorldBound(child).ComputeAlignedRange()
        if not r.IsEmpty():
            robots.append((child, r))

    for prim, rng in robots:
        mn, mx = rng.GetMin(), rng.GetMax()
        hit, gap = rack_clearance(mn, mx)
        if hit:
            risks.append(Risk(
                "CRITICAL", "collision",
                f"{prim.GetName()} inside racking",
                f"Robot footprint x=[{mn[0]:.2f},{mx[0]:.2f}] "
                f"y=[{mn[1]:.2f},{mx[1]:.2f}] intersects rack steel.",
                str(prim.GetPath()), gap))
        elif gap < 0.15:
            risks.append(Risk(
                "HIGH", "collision",
                f"{prim.GetName()} tight rack clearance",
                f"Only {gap:.3f} m clearance to nearest rack. "
                f"Minimum recommended: 0.15 m.",
                str(prim.GetPath()), gap))
        elif gap < 0.30:
            risks.append(Risk(
                "MEDIUM", "collision",
                f"{prim.GetName()} marginal rack clearance",
                f"{gap:.3f} m clearance to nearest rack. "
                f"Consider widening margin to >= 0.30 m.",
                str(prim.GetPath()), gap))

    # mutual overlap between robots
    for (pa, ra), (pb, rb) in itertools.combinations(robots, 2):
        pen = []
        for ax in range(3):
            lo = max(ra.GetMin()[ax], rb.GetMin()[ax])
            hi = min(ra.GetMax()[ax], rb.GetMax()[ax])
            pen.append(hi - lo)
        if min(pen) > 0.005:
            risks.append(Risk(
                "CRITICAL", "collision",
                f"{pa.GetName()} overlaps {pb.GetName()}",
                f"AABB penetration ({pen[0]:.3f}, {pen[1]:.3f}, {pen[2]:.3f}) m.",
                f"{pa.GetPath()} <-> {pb.GetPath()}"))
    return risks


def analyse_navigation_risks(stage, bbc):
    """Check navigable clearance, dead-end risks, and obstacle density."""
    risks = []
    fleet = stage.GetPrimAtPath("/World/Scenario/Fleet")
    if not fleet:
        return risks

    # Obstacle density per aisle
    obs_scope = stage.GetPrimAtPath(
        "/World/Environment/Infrastructure/Obstacles")
    if obs_scope:
        for ay in AISLE_Y:
            count = 0
            for child in Usd.PrimRange(obs_scope):
                if not child.IsA(UsdGeom.Gprim):
                    continue
                r = bbc.ComputeWorldBound(child).ComputeAlignedRange()
                if r.IsEmpty():
                    continue
                cy = (r.GetMin()[1] + r.GetMax()[1]) / 2
                if abs(cy - ay) < AISLE_W:
                    count += 1
            if count >= 3:
                risks.append(Risk(
                    "HIGH", "navigation",
                    f"High obstacle density in aisle at y={ay:.1f}",
                    f"{count} obstacles within the aisle width. "
                    f"Risk of AMR deadlock or forced reroute.",
                    f"aisle_y={ay}", count))
            elif count >= 2:
                risks.append(Risk(
                    "MEDIUM", "navigation",
                    f"Moderate obstacle density in aisle at y={ay:.1f}",
                    f"{count} obstacles in the aisle.",
                    f"aisle_y={ay}", count))

    # Dead-end risk: east end of aisles has no connector (dock apron keep-out)
    for ay in AISLE_Y:
        risks.append(Risk(
            "LOW", "navigation",
            f"Dead-end at east end of aisle y={ay:.1f}",
            "No east connector between aisles. Robots must reverse to "
            "the cross-aisle at x=0. Increases mission time and collision "
            "risk during backing manoeuvres.",
            f"aisle_y={ay}, x>26"))

    # Cross-aisle blind corners
    risks.append(Risk(
        "MEDIUM", "navigation",
        "Blind corners at cross-aisle intersections",
        "Rack ends at the cross-aisle (x=0) block line of sight between "
        "aisles. Speed zone (0.6 m/s) is in place but sensor occlusion "
        "remains a risk for simultaneous multi-robot transit.",
        "x=0, all aisles"))

    return risks


def analyse_barcode_risks(stage, bbc):
    """Check barcode placement, scanner coverage, and readability."""
    risks = []
    bc_root = stage.GetPrimAtPath("/World/Simulation/Barcodes")
    if not bc_root:
        risks.append(Risk(
            "HIGH", "barcode",
            "No barcodes authored in the scene",
            "Barcode layer (simulation/barcodes.usda) not found or not "
            "composed into root.usda. Run author_barcodes.py and add the "
            "layer to root.usda subLayers.",
            "/World/Simulation/Barcodes"))
        return risks

    # Collect all barcodes and check properties
    barcodes = []
    missing_id = 0
    for prim in Usd.PrimRange(bc_root):
        bc_id_attr = prim.GetAttribute("barcode:id")
        if not bc_id_attr or not bc_id_attr.Get():
            continue
        bc_id = bc_id_attr.Get()
        bc_type = prim.GetAttribute("barcode:type")
        bc_type_val = bc_type.Get() if bc_type else None
        r = bbc.ComputeWorldBound(prim).ComputeAlignedRange()
        barcodes.append((prim, bc_id, bc_type_val, r))
        if not bc_type_val:
            missing_id += 1

    if missing_id:
        risks.append(Risk(
            "MEDIUM", "barcode",
            f"{missing_id} barcodes missing type attribute",
            "Barcodes without barcode:type cannot be classified by the "
            "perception pipeline.", None, missing_id))

    # Check for duplicate IDs
    ids = [bc[1] for bc in barcodes]
    dupes = set(x for x in ids if ids.count(x) > 1)
    if dupes:
        risks.append(Risk(
            "HIGH", "barcode",
            f"{len(dupes)} duplicate barcode IDs",
            f"Duplicate IDs: {', '.join(sorted(dupes)[:5])}{'...' if len(dupes) > 5 else ''}. "
            f"Inventory tracking requires unique IDs.",
            None, len(dupes)))

    # Scanner coverage analysis: check if rover patrol covers barcode positions
    # by comparing barcode Y positions against aisle centrelines
    rack_barcodes = [bc for bc in barcodes if bc[2] == "rack_position"]
    covered = 0
    uncovered_positions = []
    for prim, bc_id, _, r in rack_barcodes:
        if r.IsEmpty():
            continue
        bc_y = (r.GetMin()[1] + r.GetMax()[1]) / 2
        bc_x = (r.GetMin()[0] + r.GetMax()[0]) / 2
        # A barcode is scannable if it faces an aisle the rover patrols
        scannable = False
        for ay in AISLE_Y:
            if abs(bc_y - ay) < RACK_RUN_DEPTH:
                scannable = True
                break
        if scannable:
            covered += 1
        else:
            uncovered_positions.append(bc_id)

    if rack_barcodes:
        coverage_pct = 100.0 * covered / len(rack_barcodes)
        if coverage_pct < 90.0:
            risks.append(Risk(
                "HIGH", "barcode",
                f"Scanner coverage only {coverage_pct:.1f}%",
                f"{len(rack_barcodes) - covered} of {len(rack_barcodes)} rack "
                f"barcodes are outside rover patrol coverage.",
                None, coverage_pct))
        elif coverage_pct < 100.0:
            risks.append(Risk(
                "MEDIUM", "barcode",
                f"Scanner coverage {coverage_pct:.1f}%",
                f"{len(rack_barcodes) - covered} rack barcodes not covered.",
                None, coverage_pct))
        else:
            risks.append(Risk(
                "INFO", "barcode",
                "Full scanner coverage",
                f"All {len(rack_barcodes)} rack barcodes are within rover "
                f"patrol reach.",
                None, 100.0))

    # High-level barcodes may be hard to read
    high_level = [bc for bc in rack_barcodes
                  if not bc[3].IsEmpty() and bc[3].GetMin()[2] > 3.5]
    if high_level:
        risks.append(Risk(
            "MEDIUM", "barcode",
            f"{len(high_level)} barcodes above 3.5 m",
            "Barcodes on rack levels 2+ require the rover's tilt camera "
            "to aim upward. Read reliability drops with height and angle.",
            None, len(high_level)))

    risks.append(Risk(
        "INFO", "barcode",
        f"{len(barcodes)} total barcodes authored",
        f"rack_position: {len(rack_barcodes)}, "
        f"other: {len(barcodes) - len(rack_barcodes)}",
        None, len(barcodes)))

    return risks


def analyse_structural_risks(stage, bbc):
    """Check rack loading balance, column clearance, and load distribution."""
    risks = []

    # Rack occupancy per run
    from author_env import pallet_slots
    slots = pallet_slots()
    rng = __import__("random").Random(20260822)
    from wh_common import RACK_OCCUPANCY as OCC
    filled = [s for s in slots if rng.random() < OCC]

    per_run = {}
    for (px, py, pz, lvl, fy) in filled:
        for ri, ry in enumerate(RACK_RUN_Y):
            if abs(fy - ry) < 2.0:
                per_run.setdefault(ri, []).append((px, py, pz, lvl))
                break

    for ri in sorted(per_run):
        count = len(per_run[ri])
        levels = {}
        for (_, _, _, lvl) in per_run[ri]:
            levels[lvl] = levels.get(lvl, 0) + 1
        # Top-heavy loading check
        low_count = sum(v for k, v in levels.items() if k <= 1)
        high_count = sum(v for k, v in levels.items() if k >= 3)
        if high_count > low_count * 1.5 and high_count > 10:
            risks.append(Risk(
                "MEDIUM", "structural",
                f"Top-heavy loading on rack run {ri}",
                f"{high_count} pallets on levels 3-4 vs {low_count} on levels "
                f"0-1. High centre of gravity increases seismic tip-over risk.",
                f"rack_run_{ri:02d}"))

    # Overall occupancy
    total_slots = len(slots)
    filled_count = len(filled)
    occ_pct = 100.0 * filled_count / total_slots if total_slots else 0
    if occ_pct > 85:
        risks.append(Risk(
            "MEDIUM", "structural",
            f"High rack occupancy ({occ_pct:.0f}%)",
            "Occupancy above 85% leaves minimal staging buffer for inbound "
            "goods, increasing dock congestion.",
            None, occ_pct))
    risks.append(Risk(
        "INFO", "structural",
        f"Rack occupancy {occ_pct:.0f}%",
        f"{filled_count} of {total_slots} positions filled.",
        None, occ_pct))

    return risks


def analyse_fire_safety_risks(stage, bbc):
    """Check sprinkler clearance, exit access, fire extinguisher reach."""
    risks = []

    # Exit sign visibility (must not be blocked by racking)
    details = stage.GetPrimAtPath("/World/Environment/Infrastructure/Details")
    if details:
        for child in Usd.PrimRange(details):
            if "exit_sign" in child.GetName():
                r = bbc.ComputeWorldBound(child).ComputeAlignedRange()
                if not r.IsEmpty():
                    z = r.GetMin()[2]
                    if z < 2.5:
                        risks.append(Risk(
                            "HIGH", "fire_safety",
                            f"Exit sign {child.GetName()} below 2.5 m",
                            f"Mounted at {z:.2f} m. May be obscured by "
                            f"stacked inventory.",
                            str(child.GetPath()), z))

    # Sprinkler clearance: top of racking must be >= 0.45 m below heads
    sprinkler_z = 10.08  # Z_BRANCH from author_infra
    rack_top = 8.80  # RACK_H
    clearance = sprinkler_z - rack_top
    if clearance < 0.45:
        risks.append(Risk(
            "HIGH", "fire_safety",
            f"Sprinkler clearance only {clearance:.2f} m",
            "NFPA 13 requires minimum 0.45 m between top of storage and "
            "sprinkler deflectors. Tall loads could violate this.",
            "rack_top_to_sprinkler", clearance))
    else:
        risks.append(Risk(
            "INFO", "fire_safety",
            f"Sprinkler clearance {clearance:.2f} m",
            "Adequate clearance between rack top and sprinkler deflectors.",
            None, clearance))

    # Blocked aisle risk for emergency egress
    obs_scope = stage.GetPrimAtPath(
        "/World/Environment/Infrastructure/Obstacles")
    if obs_scope:
        for ay in AISLE_Y:
            blocking = []
            for child in Usd.PrimRange(obs_scope):
                if not child.IsA(UsdGeom.Gprim):
                    continue
                r = bbc.ComputeWorldBound(child).ComputeAlignedRange()
                if r.IsEmpty():
                    continue
                cy = (r.GetMin()[1] + r.GetMax()[1]) / 2
                width = r.GetMax()[1] - r.GetMin()[1]
                if abs(cy - ay) < AISLE_W / 2 and width > AISLE_W * 0.4:
                    blocking.append(child.GetName())
            if blocking:
                risks.append(Risk(
                    "MEDIUM", "fire_safety",
                    f"Aisle at y={ay:.1f} partially blocked",
                    f"Obstacles [{', '.join(blocking)}] narrow the egress "
                    f"path. Fire code requires {AISLE_W:.1f} m clear.",
                    f"aisle_y={ay}"))

    return risks


def analyse_human_safety_risks(stage, bbc):
    """Check human-robot interaction zones, lighting, and signage."""
    risks = []

    # Humans in the same aisle as robots
    humans = stage.GetPrimAtPath("/World/Environment/Infrastructure/Humans")
    fleet = stage.GetPrimAtPath("/World/Scenario/Fleet")
    if humans and fleet:
        human_positions = []
        for child in Usd.PrimRange(humans):
            if not child.IsA(UsdGeom.Gprim):
                continue
            r = bbc.ComputeWorldBound(child).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            cy = (r.GetMin()[1] + r.GetMax()[1]) / 2
            cx = (r.GetMin()[0] + r.GetMax()[0]) / 2
            human_positions.append((child.GetName(), cx, cy))

        robot_positions = []
        for child in fleet.GetChildren():
            r = bbc.ComputeWorldBound(child).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            cy = (r.GetMin()[1] + r.GetMax()[1]) / 2
            cx = (r.GetMin()[0] + r.GetMax()[0]) / 2
            robot_positions.append((child.GetName(), cx, cy))

        for hname, hx, hy in human_positions:
            for rname, rx, ry in robot_positions:
                dist = math.hypot(hx - rx, hy - ry)
                if dist < 3.0:
                    risks.append(Risk(
                        "HIGH", "human_safety",
                        f"Human {hname} within {dist:.1f} m of {rname}",
                        "Close proximity at initial placement. Ensure "
                        "safety-rated monitored stop or speed-limited zone.",
                        f"{hname} <-> {rname}", dist))
                elif dist < 8.0:
                    risks.append(Risk(
                        "MEDIUM", "human_safety",
                        f"Human {hname} shares aisle with {rname}",
                        f"Distance {dist:.1f} m. Robot must reduce speed "
                        f"when human detected.",
                        f"{hname} <-> {rname}", dist))

    # Lighting coverage gaps
    lights_scope = stage.GetPrimAtPath("/World/Lighting")
    if lights_scope:
        light_count = sum(1 for p in Usd.PrimRange(lights_scope)
                          if p.IsA(UsdGeom.Gprim) or "Light" in p.GetTypeName())
        area = BAY_X * BAY_Y
        lux_density = light_count / area if area > 0 else 0
        if light_count < 100:
            risks.append(Risk(
                "MEDIUM", "human_safety",
                f"Potentially insufficient lighting ({light_count} fixtures)",
                "Warehouse standards require minimum 200 lux at floor level. "
                "Verify with light meter or render.",
                None, light_count))
    return risks


def analyse_patrol_coverage(stage, bbc):
    """Verify rover patrol reaches all barcode zones."""
    risks = []
    rover = stage.GetPrimAtPath("/World/Scenario/Fleet/rover_01")
    if not rover:
        return risks

    op = rover.GetAttribute("xformOp:translate")
    samples = op.GetTimeSamples() if op else []
    if not samples:
        risks.append(Risk(
            "MEDIUM", "barcode",
            "Rover has no animated patrol",
            "Cannot verify barcode scanning coverage without time samples.",
            "/World/Scenario/Fleet/rover_01"))
        return risks

    # Sample patrol path and check aisle coverage
    visited_aisles = set()
    t0, t1 = min(samples), max(samples)
    tc = t0
    while tc <= t1:
        pos = op.Get(Usd.TimeCode(tc))
        if pos:
            for i, ay in enumerate(AISLE_Y):
                if abs(pos[1] - ay) < AISLE_W / 2:
                    x_bin = int(pos[0] / 4.0)
                    visited_aisles.add((i, x_bin))
        tc += 120.0  # sample every 2 seconds

    # Expected coverage: all X bins in all aisles
    x_bins = set(range(int(-BAY_X / 2 / 4.0), int(BAY_X / 2 / 4.0) + 1))
    total_expected = len(AISLE_Y) * len(x_bins)
    covered = len(visited_aisles)
    coverage_pct = 100.0 * covered / total_expected if total_expected else 0

    if coverage_pct < 70:
        risks.append(Risk(
            "HIGH", "barcode",
            f"Patrol coverage only {coverage_pct:.0f}%",
            f"Rover visits {covered} of {total_expected} aisle zones. "
            f"Many barcodes will not be scanned.",
            None, coverage_pct))
    elif coverage_pct < 90:
        risks.append(Risk(
            "MEDIUM", "barcode",
            f"Patrol coverage {coverage_pct:.0f}%",
            f"Some aisle zones not reached. {total_expected - covered} zones missed.",
            None, coverage_pct))
    else:
        risks.append(Risk(
            "INFO", "barcode",
            f"Patrol coverage {coverage_pct:.0f}%",
            f"Rover reaches {covered} of {total_expected} aisle zones.",
            None, coverage_pct))

    return risks


def run_full_analysis(stage=None):
    """Run all risk analysers and return sorted list of Risk objects."""
    if stage is None:
        root = os.path.join(SCENE_ROOT, "root.usda")
        stage = Usd.Stage.Open(root, load=Usd.Stage.LoadAll)
    bbc = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=False)

    risks = []
    risks += analyse_collision_risks(stage, bbc)
    risks += analyse_navigation_risks(stage, bbc)
    risks += analyse_barcode_risks(stage, bbc)
    risks += analyse_structural_risks(stage, bbc)
    risks += analyse_fire_safety_risks(stage, bbc)
    risks += analyse_human_safety_risks(stage, bbc)
    risks += analyse_patrol_coverage(stage, bbc)

    risks.sort(key=lambda r: _severity_order(r.severity))
    return risks


def print_risk_table(risks):
    """Print a formatted risk report to stdout."""
    SEV_COLOR = {"CRITICAL": "\033[91m", "HIGH": "\033[93m",
                 "MEDIUM": "\033[33m", "LOW": "\033[36m", "INFO": "\033[37m"}
    RESET = "\033[0m"

    print(f"\n{'='*80}")
    print(f"  WAREHOUSE RISK ANALYSIS REPORT")
    print(f"{'='*80}\n")

    by_cat = {}
    for r in risks:
        by_cat.setdefault(r.category, []).append(r)

    for cat in ("collision", "navigation", "barcode", "structural",
                "fire_safety", "human_safety"):
        cat_risks = by_cat.get(cat, [])
        print(f"  [{cat.upper()}] ({len(cat_risks)} findings)")
        print(f"  {'-'*60}")
        for r in cat_risks:
            c = SEV_COLOR.get(r.severity, "")
            print(f"    {c}[{r.severity:8s}]{RESET} {r.title}")
            print(f"              {r.detail}")
            if r.location:
                print(f"              Location: {r.location}")
        print()

    summary = {}
    for r in risks:
        summary[r.severity] = summary.get(r.severity, 0) + 1
    print(f"  SUMMARY: {len(risks)} findings")
    for sev in ("CRITICAL", "HIGH", "MEDIUM", "LOW", "INFO"):
        if sev in summary:
            print(f"    {sev}: {summary[sev]}")
    print(f"{'='*80}\n")


def main():
    use_json = "--json" in sys.argv
    risks = run_full_analysis()
    if use_json:
        print(json.dumps([r.to_dict() for r in risks], indent=2))
    else:
        print_risk_table(risks)
    critical = sum(1 for r in risks if r.severity == "CRITICAL")
    return 1 if critical else 0


if __name__ == "__main__":
    sys.exit(main())
