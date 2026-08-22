"""assets/robots/rover/ -- kinematic inspection rover.

Driven by time-sampled transforms in scenario/timeline.usda.
No physics articulation -- pressing Play uses the scripted path.

The root is a KINEMATIC rigid body, not a bare Xform with a static collider.
Same reasoning as tote_payload in author_scenario.py: a kinematic body
follows its authored transform while staying visible to PhysX, whereas a
static collider is baked at its start pose the moment simulation begins.
That distinction is what keeps physics-derived things attached to the robot
-- notably the physics Lidar under Sensors/lidar_mount, whose beams are cast
from the PhysX pose, not the USD one. With no body here, the rover drove off
on its time samples while its lidar stayed behind at the spawn point.
"""
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
from wh_common import *
from author_assets import _mk_interface, _collision_hull, camera_orient

WHEELS = [("wheel_fl",  ROVER_WHEELBASE / 2,  ROVER_TRACK / 2),
          ("wheel_fr",  ROVER_WHEELBASE / 2, -ROVER_TRACK / 2),
          ("wheel_rl", -ROVER_WHEELBASE / 2,  ROVER_TRACK / 2),
          ("wheel_rr", -ROVER_WHEELBASE / 2, -ROVER_TRACK / 2)]


def author_rover():
    stage = new_layer("assets/robots/rover/rover_geom.usdc",
                      "Kinematic inspection rover. Driven by timeline.")
    root = UsdGeom.Xform.Define(stage, "/rover")
    rp = root.GetPrim()
    set_xform(rp)
    stage.SetDefaultPrim(rp)
    # One kinematic body for the whole robot: descendant colliders (chassis)
    # belong to it, and every child -- sensor mounts, scanner arm, laser --
    # rides its PhysX pose instead of being frozen at the spawn transform.
    rb = UsdPhysics.RigidBodyAPI.Apply(rp)
    rb.CreateKinematicEnabledAttr().Set(True)
    UsdPhysics.MassAPI.Apply(rp).CreateMassAttr().Set(ROVER_CHASSIS_MASS)
    add_semantics(rp, "rover")

    cz = ROVER_CLEARANCE + ROVER_BODY_H / 2
    chassis_pos = Gf.Vec3d(0, 0, cz)

    ch = UsdGeom.Xform.Define(stage, "/rover/chassis")
    chp = ch.GetPrim()
    set_xform(chp, tuple(chassis_pos))
    render = merge_meshes(stage, "/rover/chassis/render", [
        (ROVER_L, ROVER_W, ROVER_BODY_H, (0, 0, 0)),
        (0.66, 0.46, 0.02, (0, 0, ROVER_BODY_H / 2 + 0.01)),
    ])
    set_xform(render.GetPrim())
    UsdGeom.Scope.Define(stage, "/rover/chassis/Collisions")
    _collision_hull(stage, "/rover/chassis/Collisions/body",
                    ROVER_L, ROVER_W, ROVER_BODY_H, (0, 0, 0))
    UsdPhysics.CollisionAPI.Apply(chp)

    for name, wx, wy in WHEELS:
        c = UsdGeom.Cylinder.Define(stage, f"/rover/{name}")
        c.CreateRadiusAttr(ROVER_WHEEL_R)
        c.CreateHeightAttr(ROVER_WHEEL_T)
        c.CreateAxisAttr("Y")
        c.CreateExtentAttr([Gf.Vec3f(-ROVER_WHEEL_R, -ROVER_WHEEL_T / 2, -ROVER_WHEEL_R),
                            Gf.Vec3f(ROVER_WHEEL_R, ROVER_WHEEL_T / 2, ROVER_WHEEL_R)])
        set_xform(c.GetPrim(), (wx, wy, ROVER_WHEEL_R))
        add_semantics(c.GetPrim(), "rover_wheel")

    # Scanner arm with red laser beam
    arm = UsdGeom.Xform.Define(stage, "/rover/scanner_arm")
    set_xform(arm.GetPrim())
    arm_render = merge_meshes(stage, "/rover/scanner_arm/render", [
        (0.04, 0.04, 0.40, (0.10, 0.0, cz + ROVER_BODY_H / 2 + 0.20)),
        (0.25, 0.04, 0.04, (0.22, 0.0, cz + ROVER_BODY_H / 2 + 0.38)),
        (0.08, 0.06, 0.06, (0.30, 0.0, cz + ROVER_BODY_H / 2 + 0.38)),
    ])
    set_xform(arm_render.GetPrim())
    add_semantics(arm_render.GetPrim(), "rover_sensor")

    laser = define_box_mesh(stage, "/rover/scanner_arm/laser_line",
                            0.01, 0.40, 0.005,
                            center=(0.34, 0.0, cz + ROVER_BODY_H / 2 + 0.38))
    set_xform(laser.GetPrim())

    UsdGeom.Scope.Define(stage, "/rover/Sensors")

    lm = UsdGeom.Xform.Define(stage, "/rover/Sensors/lidar_mount")
    set_xform(lm.GetPrim(), (0.35, 0.0, 0.30))
    add_semantics(lm.GetPrim(), "rover_sensor")

    # Forward RGB perception camera -- the walkthrough's "attach an RGB
    # camera to the sensor mount". Level (no downward tilt), unlike the
    # barcode scanner below which is aimed at rack faces.
    cm = UsdGeom.Xform.Define(stage, "/rover/Sensors/camera_mount")
    set_xform(cm.GetPrim(), (0.35, 0.0, 0.40), camera_orient(0.0))
    add_semantics(cm.GetPrim(), "rover_sensor")

    sm = UsdGeom.Xform.Define(stage, "/rover/scanner_arm/scanner_mount")
    set_xform(sm.GetPrim(), (0.30, 0.0, 0.45), camera_orient(15.0))
    add_semantics(sm.GetPrim(), "rover_sensor")

    im = UsdGeom.Xform.Define(stage, "/rover/Sensors/imu_mount")
    set_xform(im.GetPrim(), (0.0, 0.0, 0.325))
    add_semantics(im.GetPrim(), "rover_sensor")

    rm = UsdGeom.Xform.Define(stage, "/rover/Sensors/rear_proximity")
    set_xform(rm.GetPrim(), (-0.35, 0.0, 0.30),
              quat_from_axis_angle((0, 0, 1), 180.0))
    add_semantics(rm.GetPrim(), "rover_sensor")

    UsdGeom.Scope.Define(stage, "/rover/ViewCameras")

    stage.GetRootLayer().Save()
    _mk_interface("assets/robots/rover/rover.usda", "rover_geom.usdc", "rover",
                  ((-0.35, -0.33, 0.0), (0.35, 0.33, 0.50)),
                  "Kinematic inspection rover. Driven by timeline, not PhysX.")
    return dict(steering="kinematic", wheel_r=ROVER_WHEEL_R,
                width_over_wheels=round(ROVER_TRACK + ROVER_WHEEL_T, 3))
