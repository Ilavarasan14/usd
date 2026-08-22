# usd

Warehouse bay scene for Isaac Sim, authored as USD via `tools/author_scene.py`
(runs standalone against usd-core, no Kit required).

## The bay

60 x 24 m, 10.5 m clear height, Z-up, metres/kilograms. Open `root.usda`.

| Layer | Contents |
|---|---|
| `environment/shell.usdc` | floor slab (0.5% drainage fall, +/-6 mm flatness), 4 tilt-up walls, roof deck, structural columns |
| `environment/racking.usdc` | 4 rack runs, 5 levels, 1760 storage positions |
| `environment/infrastructure.usdc` | dock doors + bumpers, chargers, bollards, floor markings, transfer stations, overhead building services, aisle signage, 643 instanced racked pallets + 151 ground pallets |
| `lighting/` | 145 high-bay RectLights at 9.6 m, plus sun and sky |
| `scenario/` | 4 AMRs + rover_01, staged pallets/totes, route graph, baked patrol |

Y budget across the 24 m span is derived, not guessed: `1.8 walkway +
4 x 2.70 rack run + 3 x 3.2 aisle + 1.8 walkway = 24.00`. The only full-width
north-south connector is the cross-aisle at x=0; east of x=26 is the dock
apron keep-out. `tools/author_tour.py` derives the patrol from those
coordinates, so **changing the bay layout invalidates the patrol** -- rerun
`author_scene.py` and check `validate_scene.py` after any layout edit.

Overhead services (sprinkler mains, branch lines and 133 pendent heads at
10.1 m; HVAC ducts and 20 diffusers at 9.9 m; wall cable trays at 8.5 m) and
the hanging aisle signs at 4.6 m carry **no colliders**. Nothing on the floor
is taller than the rover's 1.22 m mast, so per-prim colliders up there would
be pure solver cost -- the same reasoning the RackedPallets instancer uses
for levels 1-4.

Validate any change with `python3 tools/validate_scene.py` (needs
`pip install usd-core`): 14 checks covering composition, ground contact,
AABB overlap, physics, scale, navigable clearance and the patrol route.

## rover_01: physics diff-drive rig

`tools/author_rover.py` builds the same rig as the "build a robot from
scratch" Isaac Sim walkthrough: chassis + 4 wheels as separate rigid bodies,
Y-axis RevoluteJoints, `PhysicsArticulationRootAPI` on the robot container,
and an angular velocity Drive (stiffness 0, damping 10000) on only the two
**rear** wheel joints -- the front two are jointed but undriven idlers. That
part is fully scripted and verified (`python3 tools/validate_scene.py`
passes: 5 articulation roots, 44 rigid bodies).

**What's not scripted:** wiring the driven joints to a live command source.
The walkthrough does this with Isaac Sim's built-in wheeled-robot controller
wizard, which only runs inside Kit and auto-generates its own OmniGraph --
there's no way to invoke or verify that from outside a running Isaac Sim
session, so guessing the node names here would just risk another silently
broken graph. Run the wizard yourself with the rover's real numbers:

1. Open `root.usda` in Isaac Sim, select `/World/Scenario/Fleet/rover_01`
   (carries the articulation root).
2. **Tools > (Robot) Wheeled Robot > Controller Wizard** (menu wording varies
   by build -- search "differential" or "wheeled robot" in the tool search if
   it's not under Tools directly).
3. Wheel radius: **0.15 m**. Wheel distance: **0.58 m**
   (`ROVER_WHEEL_R` / `ROVER_TRACK` in `tools/wh_common.py`).
4. Control joints: `.../rover_01/Joints/joint_wheel_rl` and
   `.../rover_01/Joints/joint_wheel_rr`.
5. Enable keyboard control, click OK, press Play, drive with WASD.

If A/D turn the wrong way, swap the two wheel inputs in the generated graph
-- the walkthrough calls this out as a normal, wheel-axis-dependent fix, not
a bug.

`scenario/timeline.usda` still holds the baked flythrough/SDG-capture patrol
for physics-off scrubbing. Once the articulation is being commanded (by the
wizard's graph, or anything else), PhysX owns `rover_01`'s transform during
Play and that layer goes inert by design -- same as it always would once
something is actually driving the wheels.

## rover_01: ROS2 LaserScan streaming (perception, not control)

The walkthrough's second half (00:11:11-00:13:36) reads the lidar every tick
and publishes it to ROS2 for RViz -- purely for visualization, never wired to
the wheels. The sensors involved, in `simulation/sensors.usda`:

- `Sensors/lidar_mount/obstacle_lidar` -- the project's **RTX** lidar, a
  render-pipeline sensor, so it follows the composed USD transform and rides
  along with the rover. This is what gets published.
- `Sensors/camera_mount/rgb` -- forward RGB camera, level (no downward tilt),
  same pattern as the AMR fleet's perception camera.

**Do not add a legacy physics Lidar here**, even though the walkthrough uses
one. A `Lidar` (`omni.isaac.range_sensor`) prim is a PhysX raycast sensor:
its beams, and its `drawLines`/`drawPoints` visualisation, come from the pose
**PhysX** holds for it. `rover_01` is driven by time samples in
`scenario/timeline.usda`, not by PhysX, so the sensor never moves -- the
beams hang in the air at the spawn point while the rover drives away. Making
the rover root a kinematic rigid body does not bridge that gap. The
walkthrough gets away with it only because its robot is genuinely PhysX-driven
via a differential controller. This was tried here and reverted; see git
history.

The publish graph itself is `tools/ros_bridge_setup.py`, run the same way as
the (now-removed) live controller used to be -- inside Isaac Sim's Script
Editor, since it's built with `omni.graph.core.Controller.edit()` rather than
hand-authored USD (a wrong pin name there raises a traceback instead of
silently producing a dead graph):

    import sys; sys.path.append("<repo>/tools")
    import ros_bridge_setup
    ros_bridge_setup.build()

Then, matching the walkthrough's own test: enable the ROS2 Bridge extension,
make sure ROS2 is sourced, press Play, `ros2 topic list` should show
`/rover_01/scan`, and RViz with Fixed Frame `rover_01_lidar_frame` plus a
LaserScan display on that topic should show live returns as the rover moves.

The node type token (`ROS2RtxLidarHelper`) and its pin names are this repo's
best-documented guess at the 6.0.1 names -- not verified against a live Kit
session. See the script's doc string for what to do if a token turns out
wrong; wiring it by hand in the Action Graph editor is a two-minute fallback.
