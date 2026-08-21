"""Shared constants and USD authoring helpers for the warehouse bay scene.

Targets Isaac Sim 6.0.1 (Kit 110.1.2) but runs standalone against usd-core.
PhysxSchema is an Omniverse-only plugin and is NOT present in usd-core, so
every PhysX opinion goes through `apply_api` / `set_attr`, which fall back to
authoring the `apiSchemas` token listop plus explicit attributes directly.
The resulting USD is identical to what PhysxSchema.*API.Apply() emits.
"""
import math
import os
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf

try:
    from pxr import PhysxSchema  # noqa: F401  (present only inside Kit)
    HAVE_PHYSX_SCHEMA = True
except ImportError:
    HAVE_PHYSX_SCHEMA = False

SCENE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# ---------------------------------------------------------------- scale table
BAY_X, BAY_Y, CLEAR_H = 60.0, 24.0, 10.5     # bay envelope, metres
WALL_T = 0.20                                 # tilt-up panel thickness
FLOOR_FALL = 0.005                            # 0.5% drainage fall toward Y=-12
FLOOR_WAVE = 0.006                            # +/-6 mm slab flatness variation
FLOOR_RES = 0.5                               # floor mesh grid spacing

AISLE_W = 3.2                                 # navigable aisle, clear
CROSS_AISLE_W = 4.5
WALKWAY_W = 1.8                               # perimeter pedestrian walkway
RACK_DEPTH = 1.1                              # single-deep frame depth
FACE_OFFSET = 0.75                            # rack face centre from run centre
FLUE_SPACE = 2 * FACE_OFFSET - 1.2            # 0.30 m transverse flue, back-to-back
RACK_RUN_DEPTH = 2 * FACE_OFFSET + 1.2        # 2.70 m over the pallet faces
RACK_H = 8.80                                 # upright height (clears top load)
BAY_PITCH = 2.90                              # upright centre-to-centre
UPRIGHT_W = 0.10
BEAM_CLEAR = BAY_PITCH - UPRIGHT_W            # 2.80 m = 3x0.8 pallet + 4x0.1 gap
BEAM_H = 0.10
FRAME_DEPTH_CLEAR = 1.00                      # between front/back uprights
LEVEL_PITCH = 1.8
BEAM_LEVEL_Z = [1.8, 3.6, 5.4, 7.2]           # beam top surfaces
N_LEVELS = 1 + len(BEAM_LEVEL_Z)              # floor + 4 beam levels
N_BAYS_PER_SEG = 8
RACK_STEEL_LEN = N_BAYS_PER_SEG * BAY_PITCH + UPRIGHT_W   # 23.30 m
RACK_END_CLEAR = 0.725                        # each end of the 24.75 m segment
PALLET_GAP = 0.10                             # side gap between racked pallets
RACK_OCCUPANCY = 0.45                         # fraction of positions filled

PALLET_L, PALLET_W, PALLET_H = 1.2, 0.8, 0.144   # EUR pallet
PALLET_LOAD_H = 1.40
PALLET_MASS = 25.0                            # pallet + light load, kg
TOTE_L, TOTE_W, TOTE_H = 0.60, 0.40, 0.32
TOTE_MASS = 8.0

AMR_L, AMR_W, AMR_DECK_H = 0.90, 0.60, 0.35
AMR_BODY_H = 0.22
AMR_WHEEL_R, AMR_WHEEL_T = 0.10, 0.05
AMR_TRACK = 0.56                              # drive wheel centre-to-centre
AMR_CASTER_R, AMR_CASTER_T = 0.05, 0.03
AMR_CASTER_X = 0.33
AMR_CASTER_TRAIL = 0.03
AMR_CHASSIS_MASS = 45.0
AMR_WHEEL_MASS = 1.2
AMR_CASTER_MASS = 0.4
AMR_BRACKET_MASS = 0.5

# --- inspection rover: 4-wheel skid steer with a pan/tilt camera mast -------
ROVER_L, ROVER_W, ROVER_BODY_H = 0.70, 0.50, 0.25
ROVER_WHEEL_R, ROVER_WHEEL_T = 0.15, 0.08
ROVER_WHEELBASE, ROVER_TRACK = 0.50, 0.58     # centre-to-centre
ROVER_CLEARANCE = 0.20                        # chassis underside above ground
ROVER_MAST_TOP = 1.10
ROVER_PAN_Z, ROVER_TILT_Z = 1.15, 1.22
ROVER_CHASSIS_MASS = 55.0
ROVER_WHEEL_MASS = 2.5
ROVER_PAN_MASS, ROVER_TILT_MASS = 1.5, 2.0
ROVER_TILT_LIMITS = (-45.0, 45.0)             # degrees
ROVER_PAN_LIMITS = (-175.0, 175.0)

DOCK_DOOR_W, DOCK_DOOR_H = 2.75, 3.00
HUMAN_REF = 1.70                              # 1.7 m human, sanity reference

# Y budget across the 24 m bay, derived not guessed:
#   1.8 walkway + 4 x 2.70 rack run + 3 x 3.2 aisle + 1.8 walkway = 24.00 m
RACK_RUN_Y = [-8.85, -2.95, 2.95, 8.85]
AISLE_Y = [-5.9, 0.0, 5.9]
# Structural columns sit in the flue space at BAY CENTRES, never on a frame
# line, and the pallet position they occupy is left empty -- which is exactly
# how a real layout absorbs a column into a rack run.
COLUMN_X = [-21.875, -10.275, 10.275, 21.875]
COLUMN_W = 0.35
COLUMN_CLEAR = 0.70                           # pallet exclusion radius in X
# Rack segments in X, split by the central cross-aisle
RACK_SEG_X = [(-27.0, -2.25), (2.25, 27.0)]

TIME_STEPS_PER_SECOND = 60

# Flat semantic taxonomy -- every labelled prim uses exactly one of these.
SEMANTIC_CLASSES = [
    "floor", "wall", "ceiling", "column",
    "rack_upright", "rack_beam", "rack_brace", "rack_guard",
    "pallet", "tote", "bollard", "floor_marking",
    "dock_door", "charger", "light_fixture",
    "amr", "amr_wheel", "amr_sensor",
    "rover", "rover_wheel", "rover_sensor",
]


# ------------------------------------------------------------------- geometry
def floor_z(x, y):
    """Slab surface height. 0.5% fall toward the drain line at Y = -12, plus
    +/-6 mm pour undulation (realistic FF/FL flatness for a warehouse slab)."""
    z = FLOOR_FALL * (y + BAY_Y / 2.0)
    z += FLOOR_WAVE * math.sin(x * math.pi / 7.5) * math.cos(y * math.pi / 6.0)
    return z


# --------------------------------------------------------------- schema plumbing
def apply_api(prim, schema_name):
    """Apply an API schema by name, whether or not its plugin is loaded.

    Always goes through the apiSchemas listop rather than Usd.Prim.ApplyAPI.
    Reason: ApplyAPI (and every typed *API.Apply()) writes to explicitItems,
    so a fallback that read/wrote prependedItems silently DISCARDED anything
    ApplyAPI had already authored. Merging all three fields into one explicit
    list composes correctly with typed Apply() calls in any order.
    """
    if not HAVE_PHYSX_SCHEMA and schema_name.startswith("Physx"):
        pass  # plugin absent; the listop below is the only way to author it
    existing = prim.GetMetadata("apiSchemas")
    items = []
    if existing:
        for field in (existing.explicitItems, existing.prependedItems,
                      existing.appendedItems):
            for i in field:
                if i not in items:
                    items.append(i)
    if schema_name not in items:
        items.append(schema_name)
    listop = Sdf.TokenListOp()
    listop.explicitItems = items
    prim.SetMetadata("apiSchemas", listop)
    return prim


def applied_schemas(prim):
    """Composed API schema names, INCLUDING ones whose plugin is not loaded.
    Usd.Prim.GetAppliedSchemas() silently drops those, which would make a
    validator report a correctly-authored PhysX opinion as missing."""
    out = list(prim.GetAppliedSchemas())
    md = prim.GetMetadata("apiSchemas")
    if md:
        for field in (md.explicitItems, md.prependedItems, md.appendedItems):
            for i in field:
                if i not in out:
                    out.append(i)
    return out


_TYPE_MAP = {
    "token": Sdf.ValueTypeNames.Token,
    "bool": Sdf.ValueTypeNames.Bool,
    "float": Sdf.ValueTypeNames.Float,
    "double": Sdf.ValueTypeNames.Double,
    "int": Sdf.ValueTypeNames.Int,
    "uint": Sdf.ValueTypeNames.UInt,
    "float3": Sdf.ValueTypeNames.Float3,
}


def set_attr(prim, name, type_name, value):
    """Author a (possibly plugin-defined) attribute without needing its schema."""
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, _TYPE_MAP[type_name], custom=False)
    attr.Set(value)
    return attr


def set_xform(prim, translate=(0, 0, 0), orient=None, scale=None):
    """Author translate / orient(quatd) / scale with an explicit xformOpOrder.
    `orient` is preferred over rotateXYZ on anything that becomes a body."""
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    ops = []
    op = x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    op.Set(Gf.Vec3d(*translate))
    ops.append(op)
    op = x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    op.Set(orient if orient is not None else Gf.Quatd(1, 0, 0, 0))
    ops.append(op)
    if scale is not None:
        op = x.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)
        op.Set(Gf.Vec3d(*scale))
        ops.append(op)
    x.SetXformOpOrder(ops)
    return prim


def look_at_orient(eye, target, up=(0, 0, 1)):
    """Quatd aiming a USD camera (local -Z forward, +Y up) from eye at target.
    SetLookAt builds a VIEW matrix, so the camera transform is its inverse."""
    view = Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
    return view.GetInverse().ExtractRotationQuat()


def quat_from_axis_angle(axis, degrees):
    r = Gf.Rotation(Gf.Vec3d(*axis), degrees)
    q = r.GetQuat()
    return Gf.Quatd(q.GetReal(), Gf.Vec3d(*q.GetImaginary()))


# ------------------------------------------------------------------ box meshes
_BOX_FACES = [
    ((0, 1, 2, 3), (0, 0, -1)),   # -Z
    ((7, 6, 5, 4), (0, 0, 1)),    # +Z
    ((0, 4, 5, 1), (0, -1, 0)),   # -Y
    ((3, 2, 6, 7), (0, 1, 0)),    # +Y
    ((0, 3, 7, 4), (-1, 0, 0)),   # -X
    ((1, 5, 6, 2), (1, 0, 0)),    # +X
]


def box_points(sx, sy, sz, cx=0.0, cy=0.0, cz=0.0):
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    return [Gf.Vec3f(cx + dx * hx, cy + dy * hy, cz + dz * hz)
            for dx, dy, dz in ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                               (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1))]


def define_box_mesh(stage, path, sx, sy, sz, center=(0, 0, 0)):
    """A box baked to real dimensions -- no scale op. Required for anything that
    becomes a rigid body or articulation link (PhysX mishandles non-uniform scale)."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = box_points(sx, sy, sz, *center)
    counts, indices, normals, sts = [], [], [], []
    for face, n in _BOX_FACES:
        counts.append(4)
        indices.extend(face)
        normals.extend([Gf.Vec3f(*n)] * 4)
        sts.extend([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateNormalsAttr(normals)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(pts))
    api = UsdGeom.PrimvarsAPI(mesh.GetPrim())
    pv = api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                           UsdGeom.Tokens.faceVarying)
    pv.Set(sts)
    return mesh


def merge_meshes(stage, path, boxes):
    """Merge many boxes into one Mesh. Keeps prim count sane for static steel:
    one rack run becomes a single mesh, not 400 prims."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts, counts, indices, normals, sts = [], [], [], [], []
    for (sx, sy, sz, c) in boxes:
        base = len(pts)
        pts.extend(box_points(sx, sy, sz, *c))
        for face, n in _BOX_FACES:
            counts.append(4)
            indices.extend([base + i for i in face])
            normals.extend([Gf.Vec3f(*n)] * 4)
            sts.extend([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateNormalsAttr(normals)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(pts))
    api = UsdGeom.PrimvarsAPI(mesh.GetPrim())
    pv = api.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                           UsdGeom.Tokens.faceVarying)
    pv.Set(sts)
    return mesh


# -------------------------------------------------------------------- physics
def make_static_collider(prim, approximation="none"):
    """Static collider: CollisionAPI only, never RigidBodyAPI.
    approximation='none' means triangle mesh -- legal for static bodies only."""
    UsdPhysics.CollisionAPI.Apply(prim)
    if prim.IsA(UsdGeom.Mesh):
        mca = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mca.CreateApproximationAttr().Set(approximation)
    return prim


def make_rigid_body(prim, mass=None, density=None, approximation="convexHull",
                    kinematic=False):
    """Dynamic (or kinematic) rigid body. Mass or density is mandatory --
    a dynamic body with neither is an illegal configuration."""
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    if kinematic:
        rb.CreateKinematicEnabledAttr().Set(True)
    UsdPhysics.CollisionAPI.Apply(prim)
    if prim.IsA(UsdGeom.Mesh):
        mca = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mca.CreateApproximationAttr().Set(approximation)
    m = UsdPhysics.MassAPI.Apply(prim)
    if mass is not None:
        m.CreateMassAttr().Set(float(mass))
    elif density is not None:
        m.CreateDensityAttr().Set(float(density))
    else:
        raise ValueError(f"{prim.GetPath()}: dynamic body needs mass or density")
    return prim


def bind_physics_material(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics")


def bind_visual_material(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim)
    UsdShade.MaterialBindingAPI(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants)


# ------------------------------------------------------------------ semantics
def add_semantics(prim, label, taxonomy="class"):
    """Isaac Sim 6.0 semantics: UsdSemantics.LabelsAPI (multiple-apply, keyed by
    taxonomy). The legacy Semantics.SemanticsAPI is deprecated in 6.0."""
    from pxr import UsdSemantics
    assert label in SEMANTIC_CLASSES, f"{label} not in taxonomy"
    api = UsdSemantics.LabelsAPI.Apply(prim, taxonomy)
    api.CreateLabelsAttr().Set([label])
    return prim


# ---------------------------------------------------------------------- layer
def new_layer(rel_path, doc):
    """Create (or truncate) a layer with the mandatory stage metadata."""
    path = os.path.join(SCENE_ROOT, rel_path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    layer = Sdf.Layer.CreateNew(path) if not os.path.exists(path) else Sdf.Layer.FindOrOpen(path)
    layer.Clear()
    layer.documentation = doc
    stage = Usd.Stage.Open(layer)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    stage.SetTimeCodesPerSecond(60)
    stage.SetMetadata("kilogramsPerUnit", 1.0)
    return stage
