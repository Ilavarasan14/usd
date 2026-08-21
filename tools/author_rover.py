"""assets/robots/rover/ -- four-wheel skid-steer inspection rover.

Distinct from amr_tote: skid steer (all four wheels driven, no casters),
higher ground clearance, and a pan/tilt camera mast at 1.22 m carrying the
first-person view. Origin at the ground-contact plane, same as amr_tote.
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
                      "Skid-steer inspection rover. 5 links, 4 joints "
                      "(4 driven wheels). Origin at ground contact.")
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

    cz = ROVER_CLEARANCE + ROVER_BODY_H / 2          # 0.325 m
    chassis_pos = Gf.Vec3d(0, 0, cz)

    # ---- chassis
    ch = UsdGeom.Xform.Define(stage, "/rover/chassis")
    chp = ch.GetPrim()
    set_xform(chp, tuple(chassis_pos))
    render = merge_meshes(stage, "/rover/chassis/render", [
        (ROVER_L, ROVER_W, ROVER_BODY_H, (0, 0, 0)),
        (0.66, 0.46, 0.02, (0, 0, ROVER_BODY_H / 2 + 0.01)),   # deck plate
    ])
    set_xform(render.GetPrim())
    UsdGeom.Scope.Define(stage, "/rover/chassis/Collisions")
    _collision_hull(stage, "/rover/chassis/Collisions/body",
                    ROVER_L, ROVER_W, ROVER_BODY_H, (0, 0, 0))
    UsdPhysics.RigidBodyAPI.Apply(chp)
    UsdPhysics.MassAPI.Apply(chp).CreateMassAttr().Set(ROVER_CHASSIS_MASS)

    # ---- wheels
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

    # ---- joints: all four wheels driven (skid steer)
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
        revolute(f"/rover/Joints/joint_{name}", chp,
                 wheel_pos[name] - chassis_pos, stage.GetPrimAtPath(f"/rover/{name}"),
                 Gf.Vec3d(0, 0, 0), "Y",
                 drive={"stiffness": 0.0, "damping": 1.5e4, "maxForce": 200.0})

    # ---- sensor mounts
    UsdGeom.Scope.Define(stage, "/rover/Sensors")

    # Front lidar
    lm = UsdGeom.Xform.Define(stage, "/rover/Sensors/lidar_mount")
    set_xform(lm.GetPrim(), (0.35, 0.0, 0.30))
    add_semantics(lm.GetPrim(), "rover_sensor")

    # Barcode scanner (front, angled down 15 deg)
    sm = UsdGeom.Xform.Define(stage, "/rover/Sensors/scanner_mount")
    set_xform(sm.GetPrim(), (0.30, 0.0, 0.45), camera_orient(15.0))
    add_semantics(sm.GetPrim(), "rover_sensor")

    # IMU at chassis centre
    im = UsdGeom.Xform.Define(stage, "/rover/Sensors/imu_mount")
    set_xform(im.GetPrim(), (0.0, 0.0, 0.325))
    add_semantics(im.GetPrim(), "rover_sensor")

    # Rear proximity
    rm = UsdGeom.Xform.Define(stage, "/rover/Sensors/rear_proximity")
    set_xform(rm.GetPrim(), (-0.35, 0.0, 0.30),
              quat_from_axis_angle((0, 0, 1), 180.0))
    add_semantics(rm.GetPrim(), "rover_sensor")

    stage.GetRootLayer().Save()
    _mk_interface("assets/robots/rover/rover.usda", "rover_geom.usdc", "rover",
                  ((-0.35, -0.33, 0.0), (0.35, 0.33, 0.50)),
                  "Inspection rover. 4WD skid-steer, lidar + barcode scanner. "
                  "ArticulationRootAPI is on the payload root prim.")
    return dict(links=5, joints=4, driven_joints=4,
                steering="skid", wheel_r=ROVER_WHEEL_R,
                width_over_wheels=round(ROVER_TRACK + ROVER_WHEEL_T, 3))
