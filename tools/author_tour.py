"""scenario/timeline.usda -- kinematic patrol tour for rover_01.

Corridor topology of this bay (why the route looks the way it does):
  aisles      A1/A2/A3 at y = -5.9 / 0.0 / +5.9, running east-west
  walkways    y = +/-11.1, running east-west
  cross-aisle x = 0, the ONLY full-width north-south connector between racks
  west lane   x = -26.9, the clear strip between the rack ends and the
              staged pallets -- the second north-south connector
There is deliberately NO east-end connector: x 26..30 is the dock apron
keep-out authored in safety/constraints.usda, and you do not drive a robot
across an active dock face. The eastern half of each aisle is therefore a
dead end, traversed out-and-back -- which is what a real patrol does anyway,
since it has to inspect both rack faces.
"""
import math
from pxr import Usd, UsdGeom, Sdf, Gf
from wh_common import *

A1, A2, A3 = AISLE_Y[0], AISLE_Y[1], AISLE_Y[2]
WALK_N, WALK_S = 11.1, -11.1
X_EAST = 25.8          # east limit: clear of racking, outside the apron keep-out
X_WEST = -27.35        # west lane, centred between staged pallets and rack-end guards
X_CROSS = 0.0

TOUR = [
    (-8.0,   A2),      # start -- matches the pose in scenario/placements.usda
    (X_EAST, A2),      # east along the centre aisle
    (X_CROSS, A2),     # back west to the cross-aisle
    (X_CROSS, WALK_N), # north up the cross-aisle
    (X_WEST, WALK_N),  # west along the north walkway
    (X_WEST, A3),      # south down the west lane
    (X_EAST, A3),      # east along aisle 3
    (X_WEST, A3),      # back west
    (X_WEST, A1),      # south past aisle 2 to aisle 1
    (X_EAST, A1),      # east along aisle 1
    (X_CROSS, A1),     # back to the cross-aisle
    (X_CROSS, WALK_S), # south down the cross-aisle
    (X_WEST, WALK_S),  # west along the south walkway
    (X_WEST, A2),      # north up the west lane
    (-8.0,   A2),      # east back to the start -- closed loop
]

SPEED_NORMAL = 1.2         # m/s in aisles and walkways
SPEED_CROSS = 0.6          # m/s inside the cross-aisle (safety/constraints)
CROSS_HALF_WIDTH = 2.25
TURN_RATE = 60.0           # deg/s, skid steer turning in place
FPS = 60.0


def _in_cross(x):
    return abs(x) <= CROSS_HALF_WIDTH


def _split_leg(x0, y0, x1, y1):
    """Split a straight leg at the cross-aisle boundary so the 0.6 m/s limit
    applies ONLY while the robot is actually inside |x| <= 2.25 -- not to the
    whole 52 m aisle run merely because it passes through."""
    ts = [0.0, 1.0]
    dx = x1 - x0
    if abs(dx) > 1e-9:
        for bx in (-CROSS_HALF_WIDTH, CROSS_HALF_WIDTH):
            u = (bx - x0) / dx
            if 1e-9 < u < 1.0 - 1e-9:
                ts.append(u)
    ts = sorted(set(ts))
    out = []
    for a, b in zip(ts[:-1], ts[1:]):
        ax, ay = x0 + dx * a, y0 + (y1 - y0) * a
        bx_, by_ = x0 + dx * b, y0 + (y1 - y0) * b
        mid_x = (ax + bx_) / 2.0
        v = SPEED_CROSS if _in_cross(mid_x) else SPEED_NORMAL
        out.append((ax, ay, bx_, by_, v))
    return out


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def build_schedule():
    """(time_s, x, y, heading_deg, wheel_left_rad, wheel_right_rad) keyframes.
    Straight runs and in-place rotations alternate; wheel angles integrate the
    rolling-without-slipping condition so the wheels match the ground speed."""
    t = 0.0
    hdg = 0.0                       # placement heading: +X
    wl = wr = 0.0
    keys = [(t, TOUR[0][0], TOUR[0][1], hdg, wl, wr)]
    x, y = TOUR[0]
    dist_total = 0.0
    for nx, ny in TOUR[1:]:
        dx, dy = nx - x, ny - y
        seg = math.hypot(dx, dy)
        if seg < 1e-9:
            continue
        target = math.degrees(math.atan2(dy, dx))
        # --- rotate in place onto the new heading
        turn = _wrap180(target - hdg)
        if abs(turn) > 1e-6:
            dt = abs(turn) / TURN_RATE
            t += dt
            # skid steer: wheels counter-rotate through the arc each one travels
            arc = math.radians(turn) * (ROVER_TRACK / 2.0) / ROVER_WHEEL_R
            wl -= arc
            wr += arc
            hdg = target
            keys.append((t, x, y, hdg, wl, wr))
        # --- drive straight, one keyframe per speed-zone sub-segment
        for (ax, ay, bx_, by_, v) in _split_leg(x, y, nx, ny):
            sub = math.hypot(bx_ - ax, by_ - ay)
            if sub < 1e-9:
                continue
            t += sub / v
            spin = sub / ROVER_WHEEL_R
            wl += spin
            wr += spin
            dist_total += sub
            keys.append((t, bx_, by_, hdg, wl, wr))
        x, y = nx, ny
    return keys, dist_total


def author_timeline():
    keys, dist = build_schedule()
    duration = keys[-1][0]
    end_tc = math.ceil(duration * FPS)

    stage = new_layer(
        "scenario/timeline.usda",
        f"Kinematic patrol tour for rover_01: {len(TOUR)-1} legs, "
        f"{dist:.1f} m of travel, {duration:.1f} s.\n\n"
        "IMPORTANT: these are time-sampled TRANSFORMS. Time-sampled transforms "
        "and PhysX articulation control are mutually exclusive -- if you press "
        "Play with the articulation active, PhysX drives the robot and this "
        "animation is ignored. Use this layer for SDG capture, flythroughs and "
        "deterministic replay (scrub or render the timeline without simulating). "
        "To drive the SAME route under physics instead, feed the waypoints in "
        "scenario/routes.usda to your controller and mute this layer.")
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Scope.Define(stage, "/World/Scenario")

    rover = stage.OverridePrim("/World/Scenario/Fleet/rover_01")
    xf = UsdGeom.Xformable(rover)
    xf.ClearXformOpOrder()
    t_op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    o_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    xf.SetXformOpOrder([t_op, o_op])

    wheels = {}
    for nm in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
        w = stage.OverridePrim(f"/World/Scenario/Fleet/rover_01/{nm}")
        wx = UsdGeom.Xformable(w)
        wx.ClearXformOpOrder()
        side = 1 if nm.endswith("l") else -1
        pos = Gf.Vec3d(ROVER_WHEELBASE / 2 * (1 if "f" in nm.split("_")[1] else -1),
                       ROVER_TRACK / 2 * side, ROVER_WHEEL_R)
        wt = wx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        wt.Set(pos)
        wo = wx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        wx.SetXformOpOrder([wt, wo])
        wheels[nm] = wo

    for (t, x, y, hdg, wl, wr) in keys:
        tc = t * FPS
        t_op.Set(Gf.Vec3d(x, y, floor_z(x, y)), tc)
        o_op.Set(quat_from_axis_angle((0, 0, 1), hdg), tc)
        for nm, op in wheels.items():
            ang = math.degrees(wl if nm.endswith("l") else wr)
            op.Set(quat_from_axis_angle((0, 1, 0), ang), tc)

    tl = UsdGeom.Scope.Define(stage, "/World/Scenario/Timeline").GetPrim()
    tl.CreateAttribute("mission:durationSeconds", Sdf.ValueTypeNames.Double,
                       custom=True).Set(duration)
    tl.CreateAttribute("mission:tourDistanceMeters", Sdf.ValueTypeNames.Double,
                       custom=True).Set(dist)
    tl.CreateAttribute("mission:tasks", Sdf.ValueTypeNames.StringArray,
                       custom=True).Set([
        f"rover_01: closed patrol loop, {len(TOUR)-1} legs, {dist:.1f} m, "
        f"{duration:.1f} s at {SPEED_NORMAL} m/s ({SPEED_CROSS} m/s in cross-aisle)",
        "amr_tote_01: pick_drop_west -> dock_east_c",
        "amr_tote_02: dock_east_a -> pick_drop_west",
        "amr_tote_03: charge_02 -> dock_east_d",
    ])
    stage.GetRootLayer().Save()
    return dict(legs=len(TOUR) - 1, distance_m=round(dist, 1),
                duration_s=round(duration, 1), keyframes=len(keys),
                end_time_code=end_tc)
