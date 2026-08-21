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

# Tote-scanning pace: crawl between stops so the barcode reader can capture.
SPEED_NORMAL = 0.25            # m/s
SPEED_CROSS = 0.12             # m/s inside |x| <= 2.25 (safety/constraints)
ACCEL = 0.20                   # m/s^2
TURN_RATE = 25.0               # deg/s
TURN_ACCEL = 25.0              # deg/s^2
CROSS_HALF_WIDTH = 2.25
FPS = 60.0
DT_STRAIGHT, DT_TURN = 1.0, 0.4

SCAN_SPACING = 4.0            # stop every 4 m to scan tote barcodes
SCAN_DWELL = 3.0              # seconds facing each rack wall (barcode read time)
SENSE_RANGE = 10.0            # lidar look-ahead distance (metres)
SENSE_PAUSE = 1.0             # seconds the rover pauses while sensing ahead


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

# Obstacle map: walls and rack ends the rover can "see" ahead.
_WALL_X_MIN, _WALL_X_MAX = -BAY_X / 2 + 0.5, BAY_X / 2 - 0.5  # inside walls
_RACK_SEGS = RACK_SEG_X      # [(x_min, x_max), ...]
_RACK_Y_BANDS = [(ry - 1.35, ry + 1.35) for ry in RACK_RUN_Y]


def _blocked_ahead(x, y, hdg_deg, distance=SENSE_RANGE):
    """True if driving `distance` m in the current heading hits racking or wall."""
    rad = math.radians(hdg_deg)
    ex = x + distance * math.cos(rad)
    ey = y + distance * math.sin(rad)
    # Wall check
    if ex <= _WALL_X_MIN or ex >= _WALL_X_MAX:
        return True
    if ey <= -BAY_Y / 2 + 0.5 or ey >= BAY_Y / 2 - 0.5:
        return True
    # Rack band check: would the path cross into a rack run?
    for (rx0, rx1) in _RACK_SEGS:
        for (ry0, ry1) in _RACK_Y_BANDS:
            # Check intermediate sample points along the path
            for u in (0.3, 0.5, 0.7, 1.0):
                px = x + u * distance * math.cos(rad)
                py = y + u * distance * math.sin(rad)
                if rx0 <= px <= rx1 and ry0 <= py <= ry1:
                    return True
    return False


def _choose_turn(x, y, hdg_deg):
    """When blocked ahead, return the open direction: +90 (left) or -90 (right)."""
    left = _wrap180(hdg_deg + 90.0)
    right = _wrap180(hdg_deg - 90.0)
    left_ok = not _blocked_ahead(x, y, left)
    right_ok = not _blocked_ahead(x, y, right)
    if left_ok and not right_ok:
        return left
    if right_ok and not left_ok:
        return right
    # Both open or both blocked: prefer toward the cross-aisle
    if abs(x) > 1.0:
        # Turn toward x=0
        toward_cross = math.degrees(math.atan2(0, -x))
        if abs(_wrap180(left - toward_cross)) < abs(_wrap180(right - toward_cross)):
            return left
    return right


def _reactive_aisle_sweep(aisle_y, x_start, x_end):
    """Sweep an aisle with scan stops; at each stop sense ahead and turn if blocked."""
    legs = []
    direction = 1 if x_end > x_start else -1
    dist = abs(x_end - x_start)
    n_stops = max(1, int(dist / SCAN_SPACING))
    step = dist / n_stops
    for i in range(n_stops + 1):
        x = x_start + direction * step * i
        legs.append((x, aisle_y))
        legs.append("SCAN")
        # At the last stop, sense ahead and turn toward open space
        if i == n_stops:
            hdg = 0.0 if direction > 0 else 180.0
            if _blocked_ahead(x, aisle_y, hdg, SENSE_RANGE):
                new_hdg = _choose_turn(x, aisle_y, hdg)
                legs.append(("SENSE_TURN", new_hdg))
    return legs


PATHS = [
    ("patrol_south",  [(X_CROSS, A2), (X_CROSS, A1)]
                      + _reactive_aisle_sweep(A1, X_CROSS, X_EAST)
                      + [(X_CROSS, A1), (X_CROSS, A2)]),
    ("patrol_centre", _reactive_aisle_sweep(A2, X_CROSS, X_EAST)
                      + _reactive_aisle_sweep(A2, X_CROSS, X_WEST)
                      + [(X_CROSS, A2)]),
    ("patrol_north",  [(X_CROSS, A3)]
                      + _reactive_aisle_sweep(A3, X_CROSS, X_EAST)
                      + _reactive_aisle_sweep(A3, X_CROSS, X_WEST)
                      + [(X_CROSS, A3), (X_CROSS, A2)]),
]
START = (-8.0, A2)


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
            if leg == "SCAN":
                orig = sch.hdg
                sch.turn_to(_wrap180(orig + 90.0))
                sch.dwell(SCAN_DWELL, "scan")
                sch.turn_to(_wrap180(orig - 90.0))
                sch.dwell(SCAN_DWELL, "scan")
                sch.turn_to(orig)
            elif isinstance(leg, tuple) and leg[0] == "SENSE_TURN":
                sch.dwell(SENSE_PAUSE, "sense")
                sch.turn_to(leg[1])
            else:
                # Break long legs into SCAN_SPACING chunks, sensing before each
                nx, ny = leg
                total = math.hypot(nx - sch.x, ny - sch.y)
                if total < 0.1:
                    continue
                hdg_to_target = math.degrees(math.atan2(ny - sch.y, nx - sch.x))
                n_chunks = max(1, int(math.ceil(total / SCAN_SPACING)))
                dx = (nx - sch.x) / n_chunks
                dy = (ny - sch.y) / n_chunks
                for ci in range(n_chunks):
                    cx = sch.x + dx
                    cy = sch.y + dy
                    chunk_hdg = math.degrees(math.atan2(cy - sch.y, cx - sch.x))
                    chunk_dist = math.hypot(cx - sch.x, cy - sch.y)
                    # Sense ahead before every chunk
                    sch.dwell(SENSE_PAUSE, "sense")
                    if _blocked_ahead(sch.x, sch.y, chunk_hdg,
                                      min(SENSE_RANGE, chunk_dist + 1.0)):
                        new_hdg = _choose_turn(sch.x, sch.y, chunk_hdg)
                        sch.turn_to(new_hdg)
                        sch.dwell(SENSE_PAUSE, "sense")
                    sch.drive_to(cx, cy)
        phases.append((name, t0, sch.t, sch.distance - d0))
    return sch, phases


def author_timeline():
    sch, phases = build_schedule()
    duration = sch.t
    end_tc = math.ceil(duration * FPS)

    stage = new_layer(
        "scenario/timeline.usda",
        f"rover_01 mission: {len(PATHS)} named paths, {sch.distance:.1f} m, "
        f"{duration:.1f} s, barcode-scanning patrol only.\n\n"
        f"Cruise {SPEED_NORMAL} m/s ({SPEED_CROSS} m/s in the cross-aisle), "
        f"accel {ACCEL} m/s^2, turns {TURN_RATE} deg/s. Velocity is profiled "
        f"(forward/backward pass) so there are no instantaneous speed steps -- "
        f"the chase camera is rigidly parented, so rover jerk is camera jerk.\n\n"
        "IMPORTANT: these are time-sampled TRANSFORMS. Time-sampled transforms "
        "and PhysX articulation control are mutually exclusive -- press Play "
        "with the articulation active and PhysX drives the robot, ignoring "
        "this. Use this layer for SDG capture, flythroughs and deterministic "
        "replay. To drive the same route under physics, feed "
        "scenario/routes.usda to a controller and mute this layer.")
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

    for (t, x, y, hdg, wl, wr) in sch.keys:
        tc = t * FPS
        t_op.Set(Gf.Vec3d(x, y, floor_z(x, y)), tc)
        o_op.Set(quat_from_axis_angle((0, 0, 1), hdg), tc)
        for nm, op in wheels.items():
            op.Set(quat_from_axis_angle(
                (0, 1, 0), math.degrees(wl if nm.endswith("l") else wr)), tc)

    tl = UsdGeom.Scope.Define(stage, "/World/Scenario/Timeline").GetPrim()
    tl.CreateAttribute("mission:durationSeconds", Sdf.ValueTypeNames.Double,
                       custom=True).Set(duration)
    tl.CreateAttribute("mission:distanceMeters", Sdf.ValueTypeNames.Double,
                       custom=True).Set(sch.distance)
    tl.CreateAttribute("mission:paths", Sdf.ValueTypeNames.StringArray,
                       custom=True).Set(
        [f"{n}: t {a/1:.1f}..{b:.1f}s, {d:.1f} m" for n, a, b, d in phases])
    stage.GetRootLayer().Save()
    return dict(paths=len(PATHS), distance_m=round(sch.distance, 1),
                duration_s=round(duration, 1), keyframes=len(sch.keys),
                end_time_code=end_tc,
                phases=[(n, round(b - a, 1)) for n, a, b, _ in phases])
