"""assets/robots/rover/ -- four-wheel inspection rover, physics diff-drive.

Rig matches the "build a robot from scratch" recipe exactly (chassis + 4
wheels as separate rigid bodies, Y-axis RevoluteJoints, ArticulationRootAPI
on the robot container, an angular velocity Drive on only the two REAR wheel
joints -- the front two are revolute-jointed but undriven idlers, same as a
real differential-drive rig with caster/idler front wheels):
  - robot root (/rover) carries PhysicsArticulationRootAPI
  - chassis + all 4 wheels carry RigidBodyAPI + CollisionAPI
  - RevoluteJoint (axis Y) chassis -> each wheel
  - DriveAPI (stiffness 0, damping 10000, velocity-mode) on wheel_rl/wheel_rr
    only -- wheel_fl/wheel_fr get the joint but no drive
Pressing Play now drives the rover via whatever writes those two joints'
targetVelocity (e.g. Isaac Sim's built-in differential-controller wizard --
see README.md). scenario/timeline.usda's baked flythrough path still exists
for physics-off scrubbing/SDG capture, but PhysX owns this prim once
simulating, so that layer is inert during Play by design (see its own doc
string) -- the same way it always was before anything wrote real wheel
commands.
"""
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
from wh_common import *
from author_assets import _mk_interface, _collision_hull, camera_orient

WHEELS = [("wheel_fl",  ROVER_WHEELBASE / 2,  ROVER_TRACK / 2),
          ("wheel_fr",  ROVER_WHEELBASE / 2, -ROVER_TRACK / 2),
          ("wheel_rl", -ROVER_WHEELBASE / 2,  ROVER_TRACK / 2),
          ("wheel_rr", -ROVER_WHEELBASE / 2, -ROVER_TRACK / 2)]
DRIVEN_WHEELS = {"wheel_rl", "wheel_rr"}   # rear pair only, per the video

# Video: "set the stiffness to zero and use damping ... I use a damping
# value of 10,000". maxForce isn't stated there -- kept as an explicit cap
# (matching this repo's existing convention of never leaving a drive
# unbounded) rather than inventing a video-sourced number.
DRIVE_STIFFNESS, DRIVE_DAMPING, DRIVE_MAX_FORCE = 0.0, 10000.0, 500.0


def author_rover():
    stage = new_layer("assets/robots/rover/rover_geom.usdc",
                      "Physics diff-drive inspection rover. 5 links (chassis "
                      "+ 4 wheels), 4 wheel joints, 2 driven (rear). "
                      "ArticulationRootAPI on /rover.")
    root = UsdGeom.Xform.Define(stage, "/rover")
    rp = root.GetPrim()
    set_xform(rp)
    stage.SetDefaultPrim(rp)
    apply_api(rp, "PhysicsArticulationRootAPI")
    apply_api(rp, "PhysxArticulationAPI")
    set_attr(rp, "physxArticulation:enabledSelfCollisions", "bool", False)
    set_attr(rp, "physxArticulation:solverPositionIterationCount", "int", 32)
    set_attr(rp, "physxArticulation:solverVelocityIterationCount", "int", 4)
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
    UsdPhysics.RigidBodyAPI.Apply(chp)
    UsdPhysics.CollisionAPI.Apply(chp)
    UsdPhysics.MassAPI.Apply(chp).CreateMassAttr().Set(ROVER_CHASSIS_MASS)

    wheel_pos = {}
    for name, wx, wy in WHEELS:
        c = UsdGeom.Cylinder.Define(stage, f"/rover/{name}")
        c.CreateRadiusAttr(ROVER_WHEEL_R)
        c.CreateHeightAttr(ROVER_WHEEL_T)
        c.CreateAxisAttr("Y")
        c.CreateExtentAttr([Gf.Vec3f(-ROVER_WHEEL_R, -ROVER_WHEEL_T / 2, -ROVER_WHEEL_R),
                            Gf.Vec3f(ROVER_WHEEL_R, ROVER_WHEEL_T / 2, ROVER_WHEEL_R)])
        p = c.GetPrim()
        pos = Gf.Vec3d(wx, wy, ROVER_WHEEL_R)
        set_xform(p, tuple(pos))
        UsdPhysics.RigidBodyAPI.Apply(p)
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(ROVER_WHEEL_MASS)
        add_semantics(p, "rover_wheel")
        wheel_pos[name] = pos

    UsdGeom.Scope.Define(stage, "/rover/Joints")

    def revolute(path, b0, p0, b1, p1, axis, drive=None):
        j = UsdPhysics.RevoluteJoint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([b0.GetPath()])
        j.CreateBody1Rel().SetTargets([b1.GetPath()])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*p0))
        j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*p1))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        j.CreateAxisAttr(axis)
        jp = j.GetPrim()
        apply_api(jp, "PhysxJointAPI")
        set_attr(jp, "physxJoint:maxJointVelocity", "float", 200.0)
        if drive:
            d = UsdPhysics.DriveAPI.Apply(jp, "angular")
            d.CreateTypeAttr().Set("force")
            d.CreateStiffnessAttr().Set(drive["stiffness"])
            d.CreateDampingAttr().Set(drive["damping"])
            d.CreateTargetPositionAttr().Set(0.0)
            d.CreateTargetVelocityAttr().Set(0.0)
            d.CreateMaxForceAttr().Set(drive["maxForce"])
        return j

    for name, _, _ in WHEELS:
        drive = ({"stiffness": DRIVE_STIFFNESS, "damping": DRIVE_DAMPING,
                  "maxForce": DRIVE_MAX_FORCE} if name in DRIVEN_WHEELS else None)
        revolute(f"/rover/Joints/joint_{name}", chp,
                 wheel_pos[name] - chassis_pos, stage.GetPrimAtPath(f"/rover/{name}"),
                 Gf.Vec3d(0, 0, 0), "Y", drive=drive)

    # Scanner arm: vertical post + horizontal boom + scanner head
    arm = UsdGeom.Xform.Define(stage, "/rover/scanner_arm")
    set_xform(arm.GetPrim())
    arm_render = merge_meshes(stage, "/rover/scanner_arm/render", [
        (0.04, 0.04, 0.40, (0.10, 0.0, cz + ROVER_BODY_H / 2 + 0.20)),
        (0.25, 0.04, 0.04, (0.22, 0.0, cz + ROVER_BODY_H / 2 + 0.38)),
        (0.08, 0.06, 0.06, (0.30, 0.0, cz + ROVER_BODY_H / 2 + 0.38)),
    ])
    set_xform(arm_render.GetPrim())
    add_semantics(arm_render.GetPrim(), "rover_sensor")

    UsdGeom.Scope.Define(stage, "/rover/Sensors")

    lm = UsdGeom.Xform.Define(stage, "/rover/Sensors/lidar_mount")
    set_xform(lm.GetPrim(), (0.35, 0.0, 0.30))
    add_semantics(lm.GetPrim(), "rover_sensor")

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
                  "Physics diff-drive inspection rover. ArticulationRootAPI "
                  "is on the payload root prim; rear wheel joints are driven.")
    return dict(links=5, joints=4, driven_joints=len(DRIVEN_WHEELS),
                steering="differential", wheel_r=ROVER_WHEEL_R,
                width_over_wheels=round(ROVER_TRACK + ROVER_WHEEL_T, 3))
