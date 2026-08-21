"""Live, sensor-reactive skid-steer controller for rover_01.

Runs INSIDE Isaac Sim (Kit) against a loaded, playing stage -- unlike the rest
of tools/*.py, which author USD standalone against usd-core and never touch
Kit at all.

Why this file exists
---------------------
scenario/timeline.usda (tools/author_tour.py) bakes an entire "sense ahead,
turn left/right, patrol the aisle" route as OFFLINE keyframed transforms,
computed once against static rack geometry. That's the right tool for
flythroughs and SDG capture with physics OFF. But the rover's payload root
(assets/robots/rover/rover_geom.usdc, prim /rover) also carries
PhysicsArticulationRootAPI with 4 wheel RevoluteJoints, each a velocity drive
pinned at targetVelocity = 0. The moment you press Play, PhysX owns that
prim's transform -- the baked animation is silently ignored (this is
documented in timeline.usda's own doc string) -- and since nothing was ever
writing a non-zero velocity to the wheel drives, the rover just sits still.
That combination is the "robot doesn't move / doesn't turn left-right" bug.

This script is the missing live command source. It re-implements the same
"sense range ahead, turn toward the more open side" behaviour that
tools/author_tour.py computes OFFLINE (_blocked_ahead / _choose_turn), but
evaluates it every physics step against the rover's own RTX Lidar
(simulation/sensors.usda,
/World/Scenario/Fleet/rover_01/Sensors/lidar_mount/obstacle_lidar). The
robot now actually reacts to what it "sees" in the running simulation --
driving back and forth along the aisle, steering away from whatever the
lidar reports ahead -- instead of replaying a pre-computed path. Once real
wheel commands are flowing, the offline/physics conflict above stops
mattering: PhysX was always going to win the argument over that prim, so we
just make sure something correct is now driving it.

Run inside Isaac Sim
---------------------
Open root.usda, then in Window > Script Editor:

    import sys; sys.path.append("<repo>/tools")
    import rover_controller
    rover_controller.attach()

then press Play. `rover_controller.detach()` removes the callback and parks
the drives back at zero. Re-running `attach()` after an edit hot-reloads it
(it detaches any previous instance first).

API-drift note
---------------
The two Isaac Sim entry points below (the isaacsim.core Articulation wrapper,
and the RtxSensorCpuIsaacComputeRTXLidarFlatScan replicator annotator) are the
current Isaac Sim 6.0.1 names as far as this repo can verify without a live
Kit session -- the same caveat simulation/sensors.usda already carries for the
RTX lidar prim itself (see its "I have NOT verified the exact 6.0.1 call
signature" note). If your build renamed either, the fix is a one-line import
or annotator-name swap in _get_articulation() / _get_annotator(); the control
logic beneath does not care which import path resolved it.
"""
import math

ROVER_PATH = "/World/Scenario/Fleet/rover_01"
LIDAR_PATH = f"{ROVER_PATH}/Sensors/lidar_mount/obstacle_lidar"

# Matches tools/author_rover.py's WHEELS list -- left/right pairing for
# skid-steer: same-side wheels always get the same commanded velocity.
LEFT_WHEELS = ["joint_wheel_fl", "joint_wheel_rl"]
RIGHT_WHEELS = ["joint_wheel_fr", "joint_wheel_rr"]

# Mirrors tools/author_tour.py's constants so live behaviour matches the
# baked-animation behaviour it replaces, rather than inventing new numbers.
ROVER_WHEEL_R = 0.15           # tools/wh_common.py ROVER_WHEEL_R
CRUISE_SPEED = 0.25            # m/s,  author_tour.SPEED_NORMAL
TURN_SURFACE_SPEED = 0.12      # m/s wheel-rim speed while pivoting in place
SENSE_RANGE = 3.0              # m -- brake/turn threshold. Tighter than the
                                # 10 m look-ahead author_tour.py uses offline:
                                # that one only chose a heading at a scan stop,
                                # this one has to physically stop in time.
FORWARD_HALF_ANGLE = 25.0      # deg either side of +X counted as "ahead"
SIDE_CENTER = 70.0             # deg off +X for the left/right comparison cones
SIDE_HALF_ANGLE = 30.0         # -> samples 40..100 deg either side, safely
                                # inside the rover lidar's +/-110 deg FOV
                                # (simulation/sensors.usda horizontalFov=220)
X_WEST, X_EAST = -23.0, 23.0   # aisle turnaround bounds, tools/author_tour.py
TURN_RATE = 25.0               # deg/s, tools/author_tour.py TURN_RATE

CRUISE_OMEGA = CRUISE_SPEED / ROVER_WHEEL_R
TURN_OMEGA = TURN_SURFACE_SPEED / ROVER_WHEEL_R

_state = {}   # module-level so attach()/detach() survive Script Editor re-runs


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def _get_articulation():
    """Isaac Sim 6.0's core prim wrapper, with a fallback to the pre-6.0
    location in case this is running on an older extension set."""
    try:
        from isaacsim.core.prims import Articulation
    except ImportError:
        from omni.isaac.core.articulations import Articulation
    art = Articulation(ROVER_PATH)
    art.initialize()
    return art


def _get_annotator():
    """Attaches the RTX-lidar flat-scan annotator to the obstacle_lidar prim
    authored in simulation/sensors.usda and returns it, ready for get_data()."""
    import omni.replicator.core as rep
    hydra_texture = rep.create.render_product(LIDAR_PATH, [1, 1], name="rover_obstacle_lidar")
    annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacComputeRTXLidarFlatScan")
    annotator.attach([hydra_texture])
    return annotator, hydra_texture


def _min_range_in_cone(scan, center_deg, half_angle_deg):
    """scan: dict from the flat-scan annotator. Handles either a per-beam
    azimuth array or a uniform (start, end) FOV -- see the API-drift note."""
    depth = scan.get("linearDepthData")
    if depth is None or len(depth) == 0:
        return math.inf
    az = scan.get("azimuthRange")   # (start_deg, end_deg) in most builds
    n = len(depth)
    if az is not None and len(az) == 2:
        start, end = az
        azimuths = [start + (end - start) * i / max(1, n - 1) for i in range(n)]
    else:
        # No azimuth metadata: fall back to the lidar's own authored FOV,
        # centred on +X (the mount's forward axis).
        fov = 220.0   # simulation/sensors.usda: rover lidar inputs:horizontalFov
        azimuths = [-fov / 2 + fov * i / max(1, n - 1) for i in range(n)]

    best = math.inf
    for d, a in zip(depth, azimuths):
        if d <= 0.0:
            continue
        if abs(_wrap180(a - center_deg)) <= half_angle_deg:
            best = min(best, float(d))
    return best


def _drive(art, left_omega, right_omega):
    from omni.isaac.core.utils.types import ArticulationAction
    dof_names = art.dof_names
    targets = [0.0] * len(dof_names)
    for name in LEFT_WHEELS:
        targets[dof_names.index(name)] = left_omega
    for name in RIGHT_WHEELS:
        targets[dof_names.index(name)] = right_omega
    art.apply_action(ArticulationAction(joint_velocities=targets))


def _on_physics_step(dt):
    s = _state
    art = s["art"]
    annotator = s["annotator"]

    scan = annotator.get_data()
    if not scan or "linearDepthData" not in scan or len(scan["linearDepthData"]) == 0:
        return  # no data yet (first few frames after Play) -- hold still

    forward = _min_range_in_cone(scan, 0.0, FORWARD_HALF_ANGLE)
    left = _min_range_in_cone(scan, SIDE_CENTER, SIDE_HALF_ANGLE)
    right = _min_range_in_cone(scan, -SIDE_CENTER, SIDE_HALF_ANGLE)

    now = s["t"] = s.get("t", 0.0) + dt

    if now < s["turning_until"]:
        omega = TURN_OMEGA * s["turn_bias"]
        _drive(art, -omega, omega)
        return

    # Reached the far end of the aisle the rover is currently heading
    # toward: pivot 180 deg and head back the other way -- the "back and
    # forth" half of the patrol. `dir` only tracks which end is next; the
    # rover always drives nose-first, so wheel omega is never sign-flipped
    # here -- only the pivot below changes which way "forward" points.
    pos, _ = art.get_world_pose()
    x = float(pos[0])
    if (s["dir"] > 0 and x >= X_EAST) or (s["dir"] < 0 and x <= X_WEST):
        s["dir"] *= -1
        s["turning_until"] = now + 180.0 / TURN_RATE
        s["turn_bias"] = 1
        return

    if forward < SENSE_RANGE:
        # Blocked ahead: pivot toward whichever side has more clearance --
        # the live analogue of author_tour.py's _choose_turn.
        s["turning_until"] = now + 90.0 / TURN_RATE
        s["turn_bias"] = 1 if left >= right else -1
        return

    _drive(art, CRUISE_OMEGA, CRUISE_OMEGA)


def attach():
    """Idempotent: detaches any previous instance first, so re-running this
    after an edit in Script Editor just hot-reloads the controller."""
    detach()
    import omni.physx
    art = _get_articulation()
    annotator, hydra_texture = _get_annotator()
    sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
        lambda step_size: _on_physics_step(step_size))
    _state.update(art=art, annotator=annotator, hydra_texture=hydra_texture,
                   sub=sub, t=0.0, dir=1, turning_until=0.0, turn_bias=1)
    print("rover_controller: attached, live lidar-reactive patrol driving "
          f"{ROVER_PATH}")


def detach():
    sub = _state.pop("sub", None)
    if sub is not None:
        sub.unsubscribe()
    art = _state.pop("art", None)
    if art is not None:
        try:
            _drive(art, 0.0, 0.0)
        except Exception:
            pass
    _state.clear()


if __name__ == "__main__":
    print(__doc__)
