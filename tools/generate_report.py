#!/usr/bin/env python3
"""Generate a complete warehouse risk report.

Runs barcode authoring, validation, and risk analysis, then produces a
consolidated Markdown report with test results, risk findings, and
recommendations.

    python3 tools/generate_report.py                    # Markdown to stdout
    python3 tools/generate_report.py --out report.md    # write to file
    python3 tools/generate_report.py --json             # JSON output
"""
import datetime
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pxr import Usd, UsdGeom, Sdf, Gf
from wh_common import (
    SCENE_ROOT, BAY_X, BAY_Y, CLEAR_H, AISLE_Y, RACK_RUN_Y, N_LEVELS,
    RACK_OCCUPANCY,
)


def _safe_import():
    """Import modules that may fail if barcode layer isn't authored yet."""
    try:
        from author_barcodes import author_barcodes
        return author_barcodes
    except ImportError:
        return None


def _run_validation():
    """Run validate_scene checks and capture results."""
    from validate_scene import (
        check_metadata, check_ground, check_overlap, check_physics,
        check_transforms, check_scale, check_marking_normals,
        check_navigable, check_tour, check_payload, check_chase_speed,
        check_view_cameras, check_composition, placed_assets, RESULTS
    )
    RESULTS.clear()

    root_path = os.path.join(SCENE_ROOT, "root.usda")
    stage = Usd.Stage.Open(root_path, load=Usd.Stage.LoadAll)
    bbc = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=False)
    assets = placed_assets(stage)

    check_metadata(stage)
    check_composition(root_path)
    check_ground(stage, assets, bbc)
    check_overlap(stage, assets, bbc)
    check_physics(stage)
    check_transforms(stage)
    check_scale(stage, assets, bbc)
    check_marking_normals(stage)
    check_navigable(stage, assets, bbc)
    check_tour(stage, bbc)
    check_view_cameras(stage, bbc)
    check_payload(stage, bbc)
    check_chase_speed(stage)

    return list(RESULTS), stage


def _run_risk_analysis(stage):
    """Run the risk analysis engine."""
    from risk_analysis import run_full_analysis
    return run_full_analysis(stage)


def _barcode_stats(stage):
    """Count barcodes in the composed stage."""
    bc_root = stage.GetPrimAtPath("/World/Simulation/Barcodes")
    if not bc_root:
        return None
    stats = {"total": 0, "rack_position": 0, "pallet": 0, "tote": 0, "station": 0}
    for prim in Usd.PrimRange(bc_root):
        bc_id = prim.GetAttribute("barcode:id")
        if not bc_id or not bc_id.Get():
            continue
        stats["total"] += 1
        bc_type = prim.GetAttribute("barcode:type")
        if bc_type:
            t = bc_type.Get()
            if t in stats:
                stats[t] += 1
    return stats


def generate_markdown_report(validation_results, risks, barcode_stats):
    """Build a Markdown report string."""
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = []
    lines.append(f"# Warehouse Safety & Risk Analysis Report")
    lines.append(f"")
    lines.append(f"**Generated:** {now}  ")
    lines.append(f"**Bay dimensions:** {BAY_X} x {BAY_Y} m, "
                 f"clear height {CLEAR_H} m  ")
    lines.append(f"**Aisles:** {len(AISLE_Y)} at y = "
                 f"{', '.join(f'{y:.1f}' for y in AISLE_Y)} m  ")
    lines.append(f"**Rack runs:** {len(RACK_RUN_Y)} at y = "
                 f"{', '.join(f'{y:.1f}' for y in RACK_RUN_Y)} m  ")
    lines.append(f"**Rack levels:** {N_LEVELS}  ")
    lines.append(f"**Target occupancy:** {RACK_OCCUPANCY*100:.0f}%  ")
    lines.append(f"")

    # Barcode summary
    lines.append(f"## 1. Barcode Inventory")
    lines.append(f"")
    if barcode_stats:
        lines.append(f"| Type | Count |")
        lines.append(f"|------|-------|")
        for k in ("rack_position", "pallet", "tote", "station"):
            lines.append(f"| {k} | {barcode_stats[k]} |")
        lines.append(f"| **Total** | **{barcode_stats['total']}** |")
    else:
        lines.append(f"*No barcodes found in the scene. Run "
                     f"`python3 tools/author_barcodes.py` to generate them.*")
    lines.append(f"")

    # Validation results
    lines.append(f"## 2. Scene Validation Tests")
    lines.append(f"")
    lines.append(f"| Status | Check | Detail |")
    lines.append(f"|--------|-------|--------|")
    for status, check, msg in validation_results:
        icon = {"PASS": "PASS", "FAIL": "FAIL", "WARN": "WARN",
                "SKIP": "SKIP"}.get(status, status)
        # Truncate long messages for table readability
        short = msg[:120] + "..." if len(msg) > 120 else msg
        lines.append(f"| {icon} | {check} | {short} |")
    n_pass = sum(1 for s, _, _ in validation_results if s == "PASS")
    n_fail = sum(1 for s, _, _ in validation_results if s == "FAIL")
    n_warn = sum(1 for s, _, _ in validation_results if s == "WARN")
    n_skip = sum(1 for s, _, _ in validation_results if s == "SKIP")
    lines.append(f"")
    lines.append(f"**Summary:** {n_pass} PASS, {n_fail} FAIL, "
                 f"{n_warn} WARN, {n_skip} SKIP")
    lines.append(f"")

    # Risk analysis
    lines.append(f"## 3. Risk Analysis")
    lines.append(f"")

    sev_counts = {}
    for r in risks:
        sev_counts[r.severity] = sev_counts.get(r.severity, 0) + 1

    lines.append(f"### Risk Summary")
    lines.append(f"")
    lines.append(f"| Severity | Count |")
    lines.append(f"|----------|-------|")
    for sev in ("CRITICAL", "HIGH", "MEDIUM", "LOW", "INFO"):
        if sev in sev_counts:
            lines.append(f"| {sev} | {sev_counts[sev]} |")
    lines.append(f"| **Total** | **{len(risks)}** |")
    lines.append(f"")

    # Detailed findings by category
    categories = {}
    for r in risks:
        categories.setdefault(r.category, []).append(r)

    cat_labels = {
        "collision": "Collision Risks",
        "navigation": "Navigation Risks",
        "barcode": "Barcode & Scanner Risks",
        "structural": "Structural Risks",
        "fire_safety": "Fire Safety Risks",
        "human_safety": "Human Safety Risks",
    }

    for cat_key in ("collision", "navigation", "barcode", "structural",
                    "fire_safety", "human_safety"):
        cat_risks = categories.get(cat_key, [])
        label = cat_labels.get(cat_key, cat_key)
        lines.append(f"### 3.{list(cat_labels.keys()).index(cat_key)+1}. {label}")
        lines.append(f"")
        if not cat_risks:
            lines.append(f"*No findings.*")
            lines.append(f"")
            continue

        lines.append(f"| # | Severity | Finding | Detail |")
        lines.append(f"|---|----------|---------|--------|")
        for i, r in enumerate(cat_risks, 1):
            detail = r.detail[:100] + "..." if len(r.detail) > 100 else r.detail
            loc = f" ({r.location})" if r.location else ""
            lines.append(f"| {i} | {r.severity} | {r.title}{loc} | {detail} |")
        lines.append(f"")

    # Recommendations
    lines.append(f"## 4. Recommendations")
    lines.append(f"")

    critical = [r for r in risks if r.severity == "CRITICAL"]
    high = [r for r in risks if r.severity == "HIGH"]
    medium = [r for r in risks if r.severity == "MEDIUM"]

    if critical:
        lines.append(f"### Immediate Actions Required")
        lines.append(f"")
        for r in critical:
            lines.append(f"- **{r.title}**: {r.detail}")
        lines.append(f"")

    if high:
        lines.append(f"### High Priority")
        lines.append(f"")
        for r in high:
            lines.append(f"- **{r.title}**: {r.detail}")
        lines.append(f"")

    if medium:
        lines.append(f"### Medium Priority")
        lines.append(f"")
        for r in medium:
            lines.append(f"- **{r.title}**: {r.detail}")
        lines.append(f"")

    if not critical and not high:
        lines.append(f"No critical or high-severity risks detected. "
                     f"The warehouse layout meets safety baselines.")
        lines.append(f"")

    # Risk score
    lines.append(f"## 5. Overall Risk Score")
    lines.append(f"")
    score = (len(critical) * 30 + len(high) * 8 +
             len(medium) * 2)
    normalized = max(0, min(100, 100 - score))
    if normalized >= 90:
        grade = "A"
    elif normalized >= 75:
        grade = "B"
    elif normalized >= 60:
        grade = "C"
    elif normalized >= 40:
        grade = "D"
    else:
        grade = "F"

    lines.append(f"**Score:** {normalized:.0f}/100 (Grade: **{grade}**)  ")
    lines.append(f"")
    if grade in ("A", "B"):
        lines.append(f"The warehouse scene is well-configured with acceptable "
                     f"risk levels.")
    elif grade == "C":
        lines.append(f"The warehouse has some risks that should be addressed "
                     f"before production deployment.")
    else:
        lines.append(f"Significant risks detected. Address critical and "
                     f"high-priority findings before operating.")
    lines.append(f"")
    lines.append(f"---")
    lines.append(f"*Report generated by `tools/generate_report.py`*")

    return "\n".join(lines)


def generate_json_report(validation_results, risks, barcode_stats):
    """Build a JSON report dict."""
    return {
        "generated": datetime.datetime.now().isoformat(),
        "bay": {"x": BAY_X, "y": BAY_Y, "clear_height": CLEAR_H},
        "barcodes": barcode_stats,
        "validation": [
            {"status": s, "check": c, "message": m}
            for s, c, m in validation_results
        ],
        "risks": [r.to_dict() for r in risks],
        "summary": {
            "total_risks": len(risks),
            "critical": sum(1 for r in risks if r.severity == "CRITICAL"),
            "high": sum(1 for r in risks if r.severity == "HIGH"),
            "medium": sum(1 for r in risks if r.severity == "MEDIUM"),
            "low": sum(1 for r in risks if r.severity == "LOW"),
            "info": sum(1 for r in risks if r.severity == "INFO"),
        },
    }


def main():
    use_json = "--json" in sys.argv
    out_file = None
    if "--out" in sys.argv:
        idx = sys.argv.index("--out")
        if idx + 1 < len(sys.argv):
            out_file = sys.argv[idx + 1]

    print("Running scene validation...")
    validation_results, stage = _run_validation()

    print("Running risk analysis...")
    risks = _run_risk_analysis(stage)

    print("Collecting barcode inventory...")
    barcode_stats = _barcode_stats(stage)

    if use_json:
        report = json.dumps(
            generate_json_report(validation_results, risks, barcode_stats),
            indent=2)
    else:
        report = generate_markdown_report(validation_results, risks,
                                          barcode_stats)

    if out_file:
        path = os.path.join(SCENE_ROOT, out_file)
        with open(path, "w") as f:
            f.write(report)
        print(f"Report written to {path}")
    else:
        print(report)

    critical = sum(1 for r in risks if r.severity == "CRITICAL")
    n_fail = sum(1 for s, _, _ in validation_results if s == "FAIL")
    return 1 if (critical or n_fail) else 0


if __name__ == "__main__":
    sys.exit(main())
