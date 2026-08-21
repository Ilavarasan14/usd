"""scenario/timeline.usda -- rover_01 mission: five named paths plus a
pick-and-place, authored as kinematic time samples.

Corridor topology of this bay (why the route looks the way it does):
  aisles      A1/A2/A3 at y = -5.9 / 0.0 / +5.9, running east-west
  walkways    y = +/-11.1, running east-west
  cross-aisle x = 0, the ONLY full-width north-south connector between racks
  west lane   x = -27.35, between the rack ends and the staged pallets
There is NO east-end connector: x 26..30 is the dock apron keep-out authored in
safety/constraints.usda. The eastern half of each aisle is a dead end, run
out-and-back -- which is what a real patrol does, since it must inspect both
rack faces.

Motion uses a forward/backward velocity-profile pass, so the rover accelerates
and decelerates instead of stepping instantly between speeds. That matters for
the chase camera: it is rigidly parented, so any jerk in the rover is a jerk in
the shot.
"""
import math
from pxr import Usd, UsdGeom, Sdf, Gf
from wh_common import *

A1, A2, A3 = AISLE_Y[0], AISLE_Y[1], AISLE_Y[2]
WALK_N, WALK_S = 11.1, -11.1
X_EAST, X_WEST, X_CROSS = 23.0, -23.0, 0.0
PICK_X, PICK_Y = PICK_STATION
DROP_X, DROP_Y = DROP_STATION

# Inspection pace: slow enough for barcode capture and stable chase footage.
SPEED_NORMAL = 0.30            # m/s
SPEED_CROSS = 0.15             # m/s inside |x| <= 2.25 (safety/constraints)
ACCEL = 0.15                   # m/s^2
TURN_RATE = 20.0               # deg/s
TURN_ACCEL = 20.0              # deg/s^2
CROSS_HALF_WIDTH = 2.25
TRANSFER_SECONDS = 6.0         # roller transfer dwell at a station
FPS = 60.0
DT_STRAIGHT, DT_TURN = 0.5, 0.2

# Scan interval: every SCAN_SPACING metres the rover pauses and turns to face
# each rack wall (90 deg left then 90 deg right) so the POV camera captures
# barcodes on both sides of the aisle.
SCAN_SPACING = 4.0
SCAN_DWELL = 4.0              # seconds facing each rack wall

def _aisle_sweep(aisle_y, x_start, x_end):
    """Drive an aisle with periodic stops to turn and scan both rack faces."""
    legs = []
    direction = 1 if x_end > x_start else -1
    dist = abs(x_end - x_start)
    n_stops = max(1, int(dist / SCAN_SPACING))
    step = dist / n_stops
    for i in range(n_stops + 1):
        x = x_start + direction * step * i
        legs.append((x, aisle_y))
        legs.append("SCAN")
    return legs

PATHS = [
    ("patrol_south",  [(X_CROSS, A2), (X_CROSS, A1)]
                      + _aisle_sweep(A1, X_CROSS, X_EAST)
                      + [(X_CROSS, A1), (X_CROSS, A2)]),
    ("patrol_centre", _aisle_sweep(A2, X_CROSS, X_EAST)
                      + _aisle_sweep(A2, X_CROSS, X_WEST)
                      + [(X_CROSS, A2)]),
    ("patrol_north",  [(X_CROSS, A3)]
                      + _aisle_sweep(A3, X_CROSS, X_EAST)
                      + _aisle_sweep(A3, X_CROSS, X_WEST)
                      + [(X_CROSS, A3), (X_CROSS, A2)]),
    ("pickup_run",    [(X_WEST, A2), (PICK_X, A2), "PICK"]),
    ("transport",     [(X_CROSS, A2), (X_CROSS, A1), (DROP_X, A1), "PLACE"]),
]
START = (-8.0, A2)


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def _profile(length, v_caps, ds, accel):
    """Forward/backward pass giving a velocity profile that starts and ends at
    rest, never exceeds the local cap, and respects the acceleration limit.
    Returns (arc_lengths, times)."""
    n = max(2, int(round(length / ds)) + 1)
    s = [length * i / (n - 1) for i in range(n)]
    v = [min(v_caps(si), 1e9) for si in s]
    v[0] = v[-1] = 0.0
    for i in range(1, n):
        step = s[i] - s[i - 1]
        v[i] = min(v[i], math.sqrt(max(0.0, v[i - 1] ** 2 + 2 * accel * step)))
    for i in range(n - 2, -1, -1):
        step = s[i + 1] - s[i]
        v[i] = min(v[i], math.sqrt(max(0.0, v[i + 1] ** 2 + 2 * accel * step)))
    t = [0.0]
    for i in range(1, n):
        step = s[i] - s[i - 1]
        vm = (v[i] + v[i - 1]) / 2.0
        t.append(t[-1] + (step / vm if vm > 1e-6 else 0.0))
    return s, t


def _resample(s, t, dt):
    """Uniform-in-time samples of arc length, endpoints preserved."""
    out, k = [], 0
    total = t[-1]
    steps = max(1, int(math.ceil(total / dt)))
    for i in range(steps + 1):
        tt = min(total, total * i / steps)
        while k < len(t) - 2 and t[k + 1] < tt:
            k += 1
        span = t[k + 1] - t[k]
        u = 0.0 if span <= 1e-12 else (tt - t[k]) / span
        out.append((tt, s[k] + (s[k + 1] - s[k]) * u))
    return out


class Schedule:
    def __init__(self):
        self.t = 0.0
        self.x, self.y = START
        self.hdg = 0.0
        self.wl = self.wr = 0.0
        self.keys = []                      # (t, x, y, hdg, wl, wr)
        self.events = []                    # (kind, t_start, t_end)
        self.distance = 0.0
        self._emit()

    def _emit(self):
        self.keys.append((self.t, self.x, self.y, self.hdg, self.wl, self.wr))

    def turn_to(self, target):
        d = _wrap180(target - self.hdg)
        if abs(d) < 1e-6:
            return
        s, t = _profile(abs(d), lambda _: TURN_RATE, 1.0, TURN_ACCEL)
        h0, sign = self.hdg, (1.0 if d > 0 else -1.0)
        base_t, wl0, wr0 = self.t, self.wl, self.wr
        for (tt, ss) in _resample(s, t, DT_TURN)[1:]:
            self.t = base_t + tt
            self.hdg = h0 + sign * ss
            arc = math.radians(sign * ss) * (ROVER_TRACK / 2.0) / ROVER_WHEEL_R
            self.wl, self.wr = wl0 - arc, wr0 + arc
            self._emit()
        self.hdg = target

    def drive_to(self, nx, ny):
        dx, dy = nx - self.x, ny - self.y
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return
        self.turn_to(math.degrees(math.atan2(dy, dx)))
        x0, y0 = self.x, self.y

        def cap(si):
            u = si / length
            return SPEED_CROSS if abs(x0 + dx * u) <= CROSS_HALF_WIDTH \
                else SPEED_NORMAL

        s, t = _profile(length, cap, 0.05, ACCEL)
        base_t, wl0, wr0 = self.t, self.wl, self.wr
        for (tt, ss) in _resample(s, t, DT_STRAIGHT)[1:]:
            self.t = base_t + tt
            u = ss / length
            self.x, self.y = x0 + dx * u, y0 + dy * u
            spin = ss / ROVER_WHEEL_R
            self.wl, self.wr = wl0 + spin, wr0 + spin
            self._emit()
        self.x, self.y = nx, ny
        self.distance += length

    def dwell(self, secs, kind):
        t0 = self.t
        self.events.append((kind, t0, t0 + secs))
        self.t += secs
        self._emit()


def build_schedule():
    sch = Schedule()
    phases = []
    for name, legs in PATHS:
        t0, d0 = sch.t, sch.distance
        for leg in legs:
            if leg == "PICK":
                sch.dwell(TRANSFER_SECONDS, "pick")
            elif leg == "PLACE":
                sch.dwell(TRANSFER_SECONDS, "place")
            elif leg == "SCAN":
                # Turn 90 deg left (face south rack), dwell, turn 180 deg right
                # (face north rack), dwell, turn back to original heading.
                orig = sch.hdg
                sch.turn_to(_wrap180(orig + 90.0))
                sch.dwell(SCAN_DWELL, "scan")
                sch.turn_to(_wrap180(orig - 90.0))
                sch.dwell(SCAN_DWELL, "scan")
                sch.turn_to(orig)
            else:
                sch.drive_to(*leg)
        phases.append((name, t0, sch.t, sch.distance - d0))
    return sch, phases


def author_timeline():
    sch, phases = build_schedule()
    duration = sch.t
    end_tc = math.ceil(duration * FPS)
    pick = next(e for e in sch.events if e[0] == "pick")
    place = next(e for e in sch.events if e[0] == "place")

    stage = new_layer(
        "scenario/timeline.usda",
        f"rover_01 mission: {len(PATHS)} named paths, {sch.distance:.1f} m, "
        f"{duration:.1f} s, with a pick-and-place transfer.\n\n"
        f"Cruise {SPEED_NORMAL} m/s ({SPEED_CROSS} m/s in the cross-aisle), "
        f"accel {ACCEL} m/s^2, turns {TURN_RATE} deg/s. Velocity is profiled "
        f"(forward/backward pass) so there are no instantaneous speed steps -- "
        f"the chase camera is rigidly parented, so rover jerk is camera jerk.\n\n"
        "IMPORTANT: these are time-sampled TRANSFORMS. Time-sampled transforms "
        "and PhysX articulation control are mutually exclusive -- press Play "
        "with the articulation active and PhysX drives the robot, ignoring "
        "this. Use this layer for SDG capture, flythroughs and deterministic "
        "replay. To drive the same route under physics, feed "
        "scenario/routes.usda to a controller and mute this layer. "
        "tote_payload is authored KINEMATIC so its scripted motion is "
        "authoritative while it still collides.")
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
        fx = ROVER_WHEELBASE / 2 * (1 if nm.split("_")[1][0] == "f" else -1)
        fy = ROVER_TRACK / 2 * (1 if nm.endswith("l") else -1)
        wt = wx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        wt.Set(Gf.Vec3d(fx, fy, ROVER_WHEEL_R))
        wo = wx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
        wx.SetXformOpOrder([wt, wo])
        wheels[nm] = wo

    tote = stage.OverridePrim("/World/Scenario/Staged/tote_payload")
    tx = UsdGeom.Xformable(tote)
    tx.ClearXformOpOrder()
    tt_op = tx.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    to_op = tx.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    tx.SetXformOpOrder([tt_op, to_op])

    pick_pos = Gf.Vec3d(PICK_X, PICK_Y, floor_z(PICK_X, PICK_Y) + STATION_DECK_Z)
    drop_pos = Gf.Vec3d(DROP_X, DROP_Y, floor_z(DROP_X, DROP_Y) + STATION_DECK_Z)

    def deck_pose(x, y, hdg):
        """Tote riding the rover deck: same yaw, 0.47 m above the ground plane."""
        return Gf.Vec3d(x, y, floor_z(x, y) + ROVER_DECK_Z), \
            quat_from_axis_angle((0, 0, 1), hdg)

    for (t, x, y, hdg, wl, wr) in sch.keys:
        tc = t * FPS
        t_op.Set(Gf.Vec3d(x, y, floor_z(x, y)), tc)
        o_op.Set(quat_from_axis_angle((0, 0, 1), hdg), tc)
        for nm, op in wheels.items():
            op.Set(quat_from_axis_angle(
                (0, 1, 0), math.degrees(wl if nm.endswith("l") else wr)), tc)
        # --- payload state machine
        if t <= pick[1]:
            tt_op.Set(pick_pos, tc)
            to_op.Set(Gf.Quatd(1, 0, 0, 0), tc)
        elif t >= place[2]:
            tt_op.Set(drop_pos, tc)
            to_op.Set(Gf.Quatd(1, 0, 0, 0), tc)
        else:
            p, q = deck_pose(x, y, hdg)
            tt_op.Set(p, tc)
            to_op.Set(q, tc)

    # --- roller transfers: interpolate across the dwell so the tote slides
    #     between station deck and rover deck instead of teleporting
    for kind, t0, t1 in (pick, place):
        on_deck, _ = deck_pose(PICK_X if kind == "pick" else DROP_X,
                               A2 if kind == "pick" else A1,
                               0.0 if kind == "pick" else 0.0)
        a, b = (pick_pos, on_deck) if kind == "pick" else (on_deck, drop_pos)
        steps = 24
        for i in range(steps + 1):
            u = i / steps
            tc = (t0 + (t1 - t0) * u) * FPS
            tt_op.Set(a + (b - a) * u, tc)
            to_op.Set(Gf.Quatd(1, 0, 0, 0), tc)

    tl = UsdGeom.Scope.Define(stage, "/World/Scenario/Timeline").GetPrim()
    tl.CreateAttribute("mission:durationSeconds", Sdf.ValueTypeNames.Double,
                       custom=True).Set(duration)
    tl.CreateAttribute("mission:distanceMeters", Sdf.ValueTypeNames.Double,
                       custom=True).Set(sch.distance)
    tl.CreateAttribute("mission:paths", Sdf.ValueTypeNames.StringArray,
                       custom=True).Set(
        [f"{n}: t {a/1:.1f}..{b:.1f}s, {d:.1f} m" for n, a, b, d in phases])
    tl.CreateAttribute("mission:pickSeconds", Sdf.ValueTypeNames.Double2,
                       custom=True).Set(Gf.Vec2d(pick[1], pick[2]))
    tl.CreateAttribute("mission:placeSeconds", Sdf.ValueTypeNames.Double2,
                       custom=True).Set(Gf.Vec2d(place[1], place[2]))
    stage.GetRootLayer().Save()
    return dict(paths=len(PATHS), distance_m=round(sch.distance, 1),
                duration_s=round(duration, 1), keyframes=len(sch.keys),
                end_time_code=end_tc,
                pick_at_s=round(pick[1], 1), place_at_s=round(place[1], 1),
                phases=[(n, round(b - a, 1)) for n, a, b, _ in phases])
