"""Reusable, scenario-agnostic assets. Each is an interface layer (.usda) that
declares kind + extentsHint + a payload arc, targeting a binary geom layer."""
import math, os
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
from wh_common import *


def _mk_interface(rel_iface, rel_geom, prim_name, extent, doc, variant_defaults=None):
    stage = new_layer(rel_iface, doc)
    x = UsdGeom.Xform.Define(stage, f"/{prim_name}")
    p = x.GetPrim()
    Usd.ModelAPI(p).SetKind("component")
    p.GetPayloads().AddPayload(Sdf.Payload(f"./{os.path.basename(rel_geom)}",
                                           Sdf.Path(f"/{prim_name}")))
    UsdGeom.ModelAPI(p).SetExtentsHint(
        [Gf.Vec3f(*extent[0]), Gf.Vec3f(*extent[1])])
    stage.SetDefaultPrim(p)
    if variant_defaults:
        for vs, sel in variant_defaults.items():
            p.GetVariantSets().GetVariantSet(vs).SetVariantSelection(sel)
    stage.GetRootLayer().Save()
    return stage


# ------------------------------------------------------------------- pallet
def _collision_hull(stage, path, sx, sy, sz, center):
    """Convex hull collider on a known, stable path so consumers (e.g. a
    PointInstancer prototype) can disable it with a single `over`."""
    h = define_box_mesh(stage, path, sx, sy, sz, center=center)
    set_xform(h.GetPrim())
    UsdGeom.Imageable(h.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(h.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(h.GetPrim()).CreateApproximationAttr().Set("convexHull")
    return h


def author_pallet():
    stage = new_layer("assets/props/pallet/pallet_geom.usdc",
                      "EUR pallet 1200x800x144 mm, origin at base. "
                      "variantSet 'load': empty | half | full. "
                      "Collider at /pallet/Collisions/hull, convexHull "
                      "(legal for static AND dynamic use).")
    root = UsdGeom.Xform.Define(stage, "/pallet")
    set_xform(root.GetPrim())
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.Scope.Define(stage, "/pallet/Collisions")

    bt, bb = 0.022, 0.022
    blk = PALLET_H - bt - bb
    boxes = []
    for by in (-0.34, 0.0, 0.34):
        boxes.append((PALLET_L, 0.10, bb, (0, by, bb / 2)))
    for bx in (-0.55, 0.0, 0.55):
        for by in (-0.34, 0.0, 0.34):
            boxes.append((0.10, 0.10, blk, (bx, by, bb + blk / 2)))
    for bx in (-0.55, -0.275, 0.0, 0.275, 0.55):
        boxes.append((0.10, PALLET_W, bt, (bx, 0, bb + blk + bt / 2)))
    deck = merge_meshes(stage, "/pallet/deck", boxes)
    set_xform(deck.GetPrim())
    add_semantics(deck.GetPrim(), "pallet")

    vs = root.GetPrim().GetVariantSets().AddVariantSet("load")
    for name, h in (("empty", 0.0), ("half", 0.70), ("full", PALLET_LOAD_H)):
        vs.AddVariant(name)
        vs.SetVariantSelection(name)
        with vs.GetVariantEditContext():
            if h > 0:
                m = define_box_mesh(stage, "/pallet/load", 1.15, 0.75, h,
                                    center=(0, 0, PALLET_H + h / 2))
                set_xform(m.GetPrim())
                add_semantics(m.GetPrim(), "pallet")
            tot = PALLET_H + h
            _collision_hull(stage, "/pallet/Collisions/hull",
                            PALLET_L, PALLET_W, tot, (0, 0, tot / 2))
    vs.SetVariantSelection("full")
    stage.GetRootLayer().Save()

    _mk_interface("assets/props/pallet/pallet.usda",
                  "pallet_geom.usdc", "pallet",
                  ((-0.6, -0.4, 0.0), (0.6, 0.4, PALLET_H + PALLET_LOAD_H)),
                  "Pallet asset interface. Payload = pallet_geom.usdc.",
                  {"load": "full"})
    return dict(boxes=len(boxes), variants=["empty", "half", "full"])


# --------------------------------------------------------------------- tote
def author_tote():
    stage = new_layer("assets/props/tote/tote_geom.usdc",
                      "Stackable plastic tote 600x400x320 mm, origin at base. "
                      "Collider at /tote/Collisions/hull, convexHull.")
    root = UsdGeom.Xform.Define(stage, "/tote")
    set_xform(root.GetPrim())
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.Scope.Define(stage, "/tote/Collisions")
    t = 0.012
    boxes = [(TOTE_L, TOTE_W, t, (0, 0, t / 2)),
             (TOTE_L, t, TOTE_H, (0, -TOTE_W / 2 + t / 2, TOTE_H / 2)),
             (TOTE_L, t, TOTE_H, (0, TOTE_W / 2 - t / 2, TOTE_H / 2)),
             (t, TOTE_W, TOTE_H, (-TOTE_L / 2 + t / 2, 0, TOTE_H / 2)),
             (t, TOTE_W, TOTE_H, (TOTE_L / 2 - t / 2, 0, TOTE_H / 2))]
    m = merge_meshes(stage, "/tote/shell", boxes)
    set_xform(m.GetPrim())
    add_semantics(m.GetPrim(), "tote")
    _collision_hull(stage, "/tote/Collisions/hull",
                    TOTE_L, TOTE_W, TOTE_H, (0, 0, TOTE_H / 2))
    stage.GetRootLayer().Save()
    _mk_interface("assets/props/tote/tote.usda", "tote_geom.usdc", "tote",
                  ((-0.3, -0.2, 0.0), (0.3, 0.2, TOTE_H)),
                  "Tote asset interface. Payload = tote_geom.usdc.")
    return dict(boxes=len(boxes))


# ---------------------------------------------------------------- AMR robot
def camera_orient(pitch_down_deg):
    """USD cameras look down local -Z with +Y up. Robot forward is +X, up +Z.
    Built from explicit basis vectors (Euler composition got the heading wrong):
        right = up x back = (0,0,1) x (-1,0,0) = (0,-1,0)
    then pitched about world +Y. Asserted below, not assumed."""
    base = Gf.Matrix3d(0, -1, 0,
                       0, 0, 1,
                       -1, 0, 0)
    m3 = base * Gf.Matrix3d(Gf.Rotation(Gf.Vec3d(0, 1, 0), pitch_down_deg))
    q = Gf.Matrix4d(m3.GetOrthonormalized(), Gf.Vec3d(0, 0, 0)).ExtractRotationQuat()
    return Gf.Quatd(q.GetReal(), Gf.Vec3d(*q.GetImaginary()))


def _check_camera_orient(pitch_down_deg):
    m = Gf.Matrix4d().SetRotate(camera_orient(pitch_down_deg))
    f = m.TransformDir(Gf.Vec3d(0, 0, -1))
    u = m.TransformDir(Gf.Vec3d(0, 1, 0))
    pitch = -math.degrees(math.asin(f[2]))
    assert abs(f[1]) < 1e-9 and f[0] > 0, f"camera not facing robot +X: {f}"
    assert u[2] > 0, f"camera up is not world-up: {u}"
    assert abs(pitch - pitch_down_deg) < 1e-6, f"pitch {pitch} != {pitch_down_deg}"
    return f, u, pitch


def author_amr():
    stage = new_layer("assets/robots/amr_tote/amr_tote_geom.usdc",
                      "Differential-drive tote AMR. Origin at ground contact "
                      "plane (Z=0 touches the floor). Floating-base articulation.")
    root = UsdGeom.Xform.Define(stage, "/amr_tote")
    rp = root.GetPrim()
    set_xform(rp)
    stage.SetDefaultPrim(rp)
    apply_api(rp, "PhysicsArticulationRootAPI")
    apply_api(rp, "PhysxArticulationAPI")
    set_attr(rp, "physxArticulation:enabledSelfCollisions", "bool", False)
    set_attr(rp, "physxArticulation:solverPositionIterationCount", "int", 32)
    set_attr(rp, "physxArticulation:solverVelocityIterationCount", "int", 4)
    add_semantics(rp, "amr")

    cz = AMR_BODY_H / 2 + 0.13                      # chassis centre = 0.24 m
    # ---- chassis: render mesh separate from a low-poly convex collider
    ch = UsdGeom.Xform.Define(stage, "/amr_tote/chassis")
    chp = ch.GetPrim()
    set_xform(chp, (0, 0, cz))
    render = merge_meshes(stage, "/amr_tote/chassis/render", [
        (AMR_L, AMR_W, AMR_BODY_H, (0, 0, 0)),                       # body
        (0.86, 0.56, 0.02, (0, 0, AMR_BODY_H / 2 + 0.01)),           # deck plate
        (0.10, 0.22, 0.10, (0.43, 0, -0.04)),                        # sensor cowl
    ])
    set_xform(render.GetPrim())
    coll = UsdGeom.Scope.Define(stage, "/amr_tote/chassis/Collisions")
    hull = define_box_mesh(stage, "/amr_tote/chassis/Collisions/hull",
                           AMR_L, AMR_W, AMR_BODY_H)
    set_xform(hull.GetPrim())
    UsdGeom.Imageable(hull.GetPrim()).CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.RigidBodyAPI.Apply(chp)
    UsdPhysics.MassAPI.Apply(chp).CreateMassAttr().Set(AMR_CHASSIS_MASS)
    UsdPhysics.CollisionAPI.Apply(hull.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(hull.GetPrim()).CreateApproximationAttr().Set("convexHull")

    # ---- wheels: analytic cylinders, no scale op anywhere on a body
    wheels = {}
    for name, y in (("wheel_left", AMR_TRACK / 2), ("wheel_right", -AMR_TRACK / 2)):
        c = UsdGeom.Cylinder.Define(stage, f"/amr_tote/{name}")
        c.CreateRadiusAttr(AMR_WHEEL_R)
        c.CreateHeightAttr(AMR_WHEEL_T)
        c.CreateAxisAttr("Y")
        c.CreateExtentAttr([Gf.Vec3f(-AMR_WHEEL_R, -AMR_WHEEL_T / 2, -AMR_WHEEL_R),
                            Gf.Vec3f(AMR_WHEEL_R, AMR_WHEEL_T / 2, AMR_WHEEL_R)])
        p = c.GetPrim()
        set_xform(p, (0, y, AMR_WHEEL_R))
        UsdPhysics.RigidBodyAPI.Apply(p)
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(AMR_WHEEL_MASS)
        add_semantics(p, "amr_wheel")
        wheels[name] = (p, Gf.Vec3d(0, y, AMR_WHEEL_R))

    # ---- casters: swivel bracket + trailing roll wheel
    casters = {}
    for tag, sx in (("f", AMR_CASTER_X), ("r", -AMR_CASTER_X)):
        bpos = Gf.Vec3d(sx, 0, 0.09)
        b = define_box_mesh(stage, f"/amr_tote/caster_{tag}_bracket", 0.06, 0.08, 0.08)
        bp = b.GetPrim()
        set_xform(bp, tuple(bpos))
        UsdPhysics.RigidBodyAPI.Apply(bp)
        UsdPhysics.CollisionAPI.Apply(bp)
        UsdPhysics.MeshCollisionAPI.Apply(bp).CreateApproximationAttr().Set("convexHull")
        UsdPhysics.MassAPI.Apply(bp).CreateMassAttr().Set(AMR_BRACKET_MASS)

        wpos = Gf.Vec3d(sx - AMR_CASTER_TRAIL, 0, AMR_CASTER_R)
        c = UsdGeom.Cylinder.Define(stage, f"/amr_tote/caster_{tag}_wheel")
        c.CreateRadiusAttr(AMR_CASTER_R)
        c.CreateHeightAttr(AMR_CASTER_T)
        c.CreateAxisAttr("Y")
        c.CreateExtentAttr([Gf.Vec3f(-AMR_CASTER_R, -AMR_CASTER_T / 2, -AMR_CASTER_R),
                            Gf.Vec3f(AMR_CASTER_R, AMR_CASTER_T / 2, AMR_CASTER_R)])
        cp = c.GetPrim()
        set_xform(cp, tuple(wpos))
        UsdPhysics.RigidBodyAPI.Apply(cp)
        UsdPhysics.CollisionAPI.Apply(cp)
        UsdPhysics.MassAPI.Apply(cp).CreateMassAttr().Set(AMR_CASTER_MASS)
        add_semantics(cp, "amr_wheel")
        casters[tag] = (bp, bpos, cp, wpos)

    # ---- joints
    UsdGeom.Scope.Define(stage, "/amr_tote/Joints")
    chassis_pos = Gf.Vec3d(0, 0, cz)

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
            d.CreateStiffnessAttr().Set(0.0)        # velocity control
            d.CreateDampingAttr().Set(drive["damping"])
            d.CreateTargetVelocityAttr().Set(0.0)
            d.CreateMaxForceAttr().Set(drive["maxForce"])
        return j

    for name, (wp, wpos) in wheels.items():
        revolute(f"/amr_tote/Joints/joint_{name}", chp, wpos - chassis_pos,
                 wp, Gf.Vec3d(0, 0, 0), "Y",
                 drive={"damping": 1.0e4, "maxForce": 120.0})
    for tag, (bp, bpos, cp, wpos) in casters.items():
        revolute(f"/amr_tote/Joints/joint_caster_{tag}_swivel", chp,
                 bpos - chassis_pos, bp, Gf.Vec3d(0, 0, 0), "Z")
        revolute(f"/amr_tote/Joints/joint_caster_{tag}_roll", bp,
                 wpos - bpos, cp, Gf.Vec3d(0, 0, 0), "Y")

    # ---- sensor mounts (transforms only; sensor prims live in simulation/sensors)
    UsdGeom.Scope.Define(stage, "/amr_tote/Sensors")
    lm = UsdGeom.Xform.Define(stage, "/amr_tote/Sensors/lidar_mount")
    set_xform(lm.GetPrim(), (0.42, 0.0, 0.20))
    add_semantics(lm.GetPrim(), "amr_sensor")
    cm = UsdGeom.Xform.Define(stage, "/amr_tote/Sensors/camera_mount")
    set_xform(cm.GetPrim(), (0.44, 0.0, 0.32), camera_orient(15.0))
    add_semantics(cm.GetPrim(), "amr_sensor")
    im = UsdGeom.Xform.Define(stage, "/amr_tote/Sensors/imu_mount")
    set_xform(im.GetPrim(), (0.0, 0.0, 0.24))
    add_semantics(im.GetPrim(), "amr_sensor")

    stage.GetRootLayer().Save()
    _mk_interface("assets/robots/amr_tote/amr_tote.usda", "amr_tote_geom.usdc",
                  "amr_tote",
                  ((-0.45, -0.31, 0.0), (0.45, 0.31, AMR_DECK_H)),
                  "AMR asset interface. Payload = amr_tote_geom.usdc. "
                  "ArticulationRootAPI is on the payload root prim.")

    fwd, up, pitch = _check_camera_orient(15.0)
    return dict(links=7, joints=6, driven_joints=2,
                cam_forward=tuple(round(v, 4) for v in fwd),
                cam_up=tuple(round(v, 4) for v in up),
                cam_pitch_deg=round(pitch, 2))
