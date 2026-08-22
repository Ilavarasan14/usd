# Warehouse Safety & Risk Analysis Report

**Generated:** 2026-08-22 09:36:58  
**Bay dimensions:** 60.0 x 24.0 m, clear height 10.5 m  
**Aisles:** 3 at y = -5.9, 0.0, 5.9 m  
**Rack runs:** 4 at y = -8.8, -3.0, 3.0, 8.8 m  
**Rack levels:** 5  
**Target occupancy:** 45%  

## 1. Barcode Inventory

| Type | Count |
|------|-------|
| rack_position | 1760 |
| pallet | 145 |
| tote | 5 |
| station | 2 |
| **Total** | **1912** |

## 2. Scene Validation Tests

| Status | Check | Detail |
|--------|-------|--------|
| PASS | stage_metadata | metersPerUnit=1.0 kilogramsPerUnit=1.0 upAxis=Z timeCodesPerSecond=60 defaultPrim=/World |
| PASS | composition | usdchecker --strict: 0 errors excluding Kit-provided MDL |
| WARN | composition | 1 unresolvable Kit-provided MDL reference(s) (OmniPBR.mdl) -- expected offline, resolves inside Isaac Sim; each material... |
| PASS | ground_contact | 181 placed assets rest on their support (worst residual +0.31 mm at rover_01) |
| PASS | aabb_overlap | 181 assets, 16290 pairs, no intersection beyond 5 mm |
| PASS | physics | 40 rigid bodies, 4 articulation roots: colliders present, mass set, no nesting, no non-uniform scale, no tri-mesh on dyn... |
| PASS | transforms | 3122 transformed prims: xformOpOrder authored, no NaN/Inf, no zero or negative scale |
| PASS | scale | 181 assets within class bounds; world bound 60.40 x 24.40 x 10.51 m |
| PASS | marking_winding | 23 floor-marking meshes, every face winds +Z |
| PASS | navigable | all AMRs and rovers inside their corridor; tightest lateral clearance 1.270 m (rover_01) in a 3.2 m aisle |
| PASS | patrol_tour | 1149 poses sampled over 1148.7s: never enters racking, stays in the bay, tightest clearance 0.871 m at t=816.0s; cross-a... |
| PASS | view_cameras | 4 view cameras in free space; tightest eye-to-rack 0.300 m (rover_closeup) |
| SKIP | payload | tote_payload is not animated |
| PASS | chase_speed | rover peak 0.80 m/s (cruise 0.8), chase camera peak 0.80 m/s (1.00x, the boom swinging through turns), peak camera accel... |

**Summary:** 12 PASS, 0 FAIL, 1 WARN, 1 SKIP

## 3. Risk Analysis

### Risk Summary

| Severity | Count |
|----------|-------|
| HIGH | 3 |
| MEDIUM | 4 |
| LOW | 3 |
| INFO | 3 |
| **Total** | **13** |

### 3.1. Collision Risks

*No findings.*

### 3.2. Navigation Risks

| # | Severity | Finding | Detail |
|---|----------|---------|--------|
| 1 | HIGH | High obstacle density in aisle at y=-5.9 (aisle_y=-5.9) | 3 obstacles within the aisle width. Risk of AMR deadlock or forced reroute. |
| 2 | MEDIUM | Blind corners at cross-aisle intersections (x=0, all aisles) | Rack ends at the cross-aisle (x=0) block line of sight between aisles. Speed zone (0.6 m/s) is in pl... |
| 3 | LOW | Dead-end at east end of aisle y=-5.9 (aisle_y=-5.9, x>26) | No east connector between aisles. Robots must reverse to the cross-aisle at x=0. Increases mission t... |
| 4 | LOW | Dead-end at east end of aisle y=0.0 (aisle_y=0.0, x>26) | No east connector between aisles. Robots must reverse to the cross-aisle at x=0. Increases mission t... |
| 5 | LOW | Dead-end at east end of aisle y=5.9 (aisle_y=5.9, x>26) | No east connector between aisles. Robots must reverse to the cross-aisle at x=0. Increases mission t... |

### 3.3. Barcode & Scanner Risks

| # | Severity | Finding | Detail |
|---|----------|---------|--------|
| 1 | HIGH | Scanner coverage only 75.0% | 440 of 1760 rack barcodes are outside rover patrol coverage. |
| 2 | HIGH | Patrol coverage only 42% | Rover visits 19 of 45 aisle zones. Many barcodes will not be scanned. |
| 3 | MEDIUM | 1056 barcodes above 3.5 m | Barcodes on rack levels 2+ require the rover's tilt camera to aim upward. Read reliability drops wit... |
| 4 | INFO | 1912 total barcodes authored | rack_position: 1760, other: 152 |

### 3.4. Structural Risks

| # | Severity | Finding | Detail |
|---|----------|---------|--------|
| 1 | INFO | Rack occupancy 45% | 794 of 1760 positions filled. |

### 3.5. Fire Safety Risks

| # | Severity | Finding | Detail |
|---|----------|---------|--------|
| 1 | INFO | Sprinkler clearance 1.28 m | Adequate clearance between rack top and sprinkler deflectors. |

### 3.6. Human Safety Risks

| # | Severity | Finding | Detail |
|---|----------|---------|--------|
| 1 | MEDIUM | Human person_00 shares aisle with amr_tote_02 (person_00 <-> amr_tote_02) | Distance 6.6 m. Robot must reduce speed when human detected. |
| 2 | MEDIUM | Human person_01 shares aisle with rover_01 (person_01 <-> rover_01) | Distance 6.3 m. Robot must reduce speed when human detected. |

## 4. Recommendations

### High Priority

- **High obstacle density in aisle at y=-5.9**: 3 obstacles within the aisle width. Risk of AMR deadlock or forced reroute.
- **Scanner coverage only 75.0%**: 440 of 1760 rack barcodes are outside rover patrol coverage.
- **Patrol coverage only 42%**: Rover visits 19 of 45 aisle zones. Many barcodes will not be scanned.

### Medium Priority

- **Blind corners at cross-aisle intersections**: Rack ends at the cross-aisle (x=0) block line of sight between aisles. Speed zone (0.6 m/s) is in place but sensor occlusion remains a risk for simultaneous multi-robot transit.
- **1056 barcodes above 3.5 m**: Barcodes on rack levels 2+ require the rover's tilt camera to aim upward. Read reliability drops with height and angle.
- **Human person_00 shares aisle with amr_tote_02**: Distance 6.6 m. Robot must reduce speed when human detected.
- **Human person_01 shares aisle with rover_01**: Distance 6.3 m. Robot must reduce speed when human detected.

## 5. Overall Risk Score

**Score:** 84/100 (Grade: **B**)  

The warehouse scene is well-configured with acceptable risk levels.

---
*Report generated by `tools/generate_report.py`*