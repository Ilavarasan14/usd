# usd

Warehouse bay scene for Isaac Sim, authored as USD via `tools/author_scene.py`
(runs standalone against usd-core, no Kit required).

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
