"""simulation/{physics,materials,sensors,semantics}.usda"""
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, UsdSemantics, Sdf, Gf
from wh_common import *
from wh_common import look_at_orient
from author_scenario import FLEET, ROVERS, STAGED_PALLET_Y

ENV = "/World/Environment"
SHELL = ENV + "/Shell"
INFRA = ENV + "/Infrastructure"


def _skeleton(stage):
    UsdGeom.Xform.Define(stage, "/World")
    return stage


# ------------------------------------------------------------------- physics
PHYS_MATERIALS = {
    #  name            static  dynamic  restitution
    "concrete_floor": (0.75, 0.65, 0.05),
    "rubber_wheel":   (0.95, 0.85, 0.05),
    "steel_rack":     (0.40, 0.35, 0.20),
    "wood_pallet":    (0.55, 0.45, 0.10),
    "plastic_tote":   (0.35, 0.30, 0.15),
}


def author_physics():
    stage = new_layer("simulation/physics.usda",
                      "PhysicsScene, physics materials and their bindings. "
                      "Bindings are authored as `over` -- the prims themselves "
                      "are defined in the environment and scenario layers.")
    _skeleton(stage)

    scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    sp = scene.GetPrim()
    apply_api(sp, "PhysxSceneAPI")
    set_attr(sp, "physxScene:solverType", "token", "TGS")
    set_attr(sp, "physxScene:timeStepsPerSecond", "uint", TIME_STEPS_PER_SECOND)
    set_attr(sp, "physxScene:enableCCD", "bool", True)
    set_attr(sp, "physxScene:enableStabilization", "bool", True)
    # GPU dynamics OFF: this scene has 10 dynamic prop bodies plus 4 x 7 = 28
    # articulation links = 38 dynamic actors. Everything else -- slab, walls,
    # roof, racking, 168 ground pallets, 699 instanced pallets -- is static and
    # costs the solver nothing. Below ~200 dynamic actors the host<->device
    # transfer costs more than the parallel solve saves.
    set_attr(sp, "physxScene:enableGPUDynamics", "bool", False)

    mats = {}
    UsdGeom.Scope.Define(stage, "/World/PhysicsMaterials")
    for name, (sf, df, r) in PHYS_MATERIALS.items():
        m = UsdShade.Material.Define(stage, f"/World/PhysicsMaterials/{name}")
        api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        api.CreateStaticFrictionAttr().Set(sf)
        api.CreateDynamicFrictionAttr().Set(df)
        api.CreateRestitutionAttr().Set(r)
        mats[name] = m

    def bind(path, mat):
        p = stage.OverridePrim(path)
        bind_physics_material(p, mats[mat])

    bind(SHELL + "/floor_slab", "concrete_floor")
    for w in ("wall_north", "wall_south", "wall_east", "wall_west",
              "roof_deck", "columns"):
        bind(f"{SHELL}/{w}", "concrete_floor")
    for r in range(len(RACK_RUN_Y)):
        bind(f"{ENV}/Racking/rack_run_{r:02d}", "steel_rack")
    for n in ("dock_doors", "dock_bumpers", "chargers"):
        bind(f"{INFRA}/{n}", "steel_rack")
    bind(INFRA + "/GroundInventory", "wood_pallet")
    for i in range(len(STAGED_PALLET_Y)):
        bind(f"/World/Scenario/Staged/pallet_{i:02d}", "wood_pallet")
    for i in range(4):
        bind(f"/World/Scenario/Staged/tote_{i:02d}", "plastic_tote")
    for name, *_ in FLEET:
        base = f"/World/Scenario/Fleet/{name}"
        bind(base + "/chassis", "steel_rack")
        for w in ("wheel_left", "wheel_right", "caster_f_wheel", "caster_r_wheel"):
            bind(f"{base}/{w}", "rubber_wheel")
    for name, *_ in ROVERS:
        base = f"/World/Scenario/Fleet/{name}"
        for part in ("chassis", "pan_head", "tilt_head"):
            bind(f"{base}/{part}", "steel_rack")
        for w in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
            bind(f"{base}/{w}", "rubber_wheel")

    stage.GetRootLayer().Save()
    return dict(materials=len(mats), solver="TGS",
                timeStepsPerSecond=TIME_STEPS_PER_SECOND,
                gpu_dynamics=False,
                dynamic_actors=10 + len(FLEET) * 7 + len(ROVERS) * 8)


# ----------------------------------------------------------------- materials
VIS_MATERIALS = {
    #  name                base colour        rough  metal
    "concrete_floor":     ((0.42, 0.42, 0.44), 0.55, 0.0),
    "wall_panel":         ((0.72, 0.71, 0.68), 0.75, 0.0),
    "roof_deck":          ((0.55, 0.56, 0.58), 0.65, 0.6),
    "column_grey":        ((0.60, 0.60, 0.62), 0.70, 0.0),
    "rack_upright_blue":  ((0.10, 0.20, 0.45), 0.45, 0.4),
    "rack_beam_orange":   ((0.85, 0.35, 0.05), 0.42, 0.4),
    "pallet_wood":        ((0.58, 0.44, 0.28), 0.80, 0.0),
    "carton_tan":         ((0.72, 0.60, 0.44), 0.85, 0.0),
    "tote_plastic":       ((0.15, 0.30, 0.55), 0.35, 0.0),
    "amr_shell":          ((0.18, 0.19, 0.21), 0.40, 0.2),
    "amr_rubber":         ((0.05, 0.05, 0.05), 0.85, 0.0),
    "dock_door_steel":    ((0.66, 0.67, 0.69), 0.50, 0.7),
    "floor_paint_yellow": ((0.85, 0.68, 0.05), 0.60, 0.0),
    "safety_yellow":      ((0.90, 0.72, 0.02), 0.55, 0.0),
    "charger_housing":    ((0.88, 0.88, 0.86), 0.45, 0.1),
}


def author_materials():
    stage = new_layer("simulation/materials.usda",
                      "Visual materials. Each carries BOTH an MDL surface "
                      "(OmniPBR -- what the RTX renderer consumes) and a "
                      "UsdPreviewSurface, so the stage still shades correctly "
                      "when opened outside Omniverse.")
    _skeleton(stage)
    UsdGeom.Scope.Define(stage, "/World/Looks")
    made = {}
    for name, (col, rough, metal) in VIS_MATERIALS.items():
        m = UsdShade.Material.Define(stage, f"/World/Looks/{name}")

        mdl = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/mdl_shader")
        mdl.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
        mdl.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        mdl.CreateInput("diffuse_color_constant",
                        Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*col))
        mdl.CreateInput("reflection_roughness_constant",
                        Sdf.ValueTypeNames.Float).Set(rough)
        mdl.CreateInput("metallic_constant",
                        Sdf.ValueTypeNames.Float).Set(metal)
        m.CreateSurfaceOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")
        m.CreateDisplacementOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")
        m.CreateVolumeOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")

        prev = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/preview_shader")
        prev.CreateIdAttr("UsdPreviewSurface")
        prev.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*col))
        prev.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        prev.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        m.CreateSurfaceOutput().ConnectToSource(prev.ConnectableAPI(), "surface")
        made[name] = m

    def bind(path, mat):
        bind_visual_material(stage.OverridePrim(path), made[mat])

    bind(SHELL + "/floor_slab", "concrete_floor")
    for w in ("wall_north", "wall_south", "wall_east", "wall_west"):
        bind(f"{SHELL}/{w}", "wall_panel")
    bind(SHELL + "/roof_deck", "roof_deck")
    bind(SHELL + "/columns", "column_grey")
    for r in range(len(RACK_RUN_Y)):
        bind(f"{ENV}/Racking/rack_run_{r:02d}", "rack_beam_orange")
    bind(INFRA + "/dock_doors", "dock_door_steel")
    bind(INFRA + "/dock_bumpers", "amr_rubber")
    bind(INFRA + "/chargers", "charger_housing")
    bind(INFRA + "/Bollards", "safety_yellow")
    bind(INFRA + "/FloorMarkings", "floor_paint_yellow")
    bind(INFRA + "/GroundInventory", "pallet_wood")
    bind(INFRA + "/RackedPallets", "pallet_wood")
    for i in range(len(STAGED_PALLET_Y)):
        bind(f"/World/Scenario/Staged/pallet_{i:02d}", "pallet_wood")
    for i in range(4):
        bind(f"/World/Scenario/Staged/tote_{i:02d}", "tote_plastic")
    for name, *_ in FLEET:
        base = f"/World/Scenario/Fleet/{name}"
        bind(base + "/chassis", "amr_shell")
        for w in ("wheel_left", "wheel_right", "caster_f_wheel", "caster_r_wheel"):
            bind(f"{base}/{w}", "amr_rubber")
    for name, *_ in ROVERS:
        base = f"/World/Scenario/Fleet/{name}"
        for part in ("chassis", "pan_head", "tilt_head"):
            bind(f"{base}/{part}", "amr_shell")
        for w in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
            bind(f"{base}/{w}", "amr_rubber")

    stage.GetRootLayer().Save()
    return dict(materials=len(made), mdl="OmniPBR.mdl",
                preview_surface=True)


# ------------------------------------------------------------------- sensors
# Approximating an Intel RealSense D435 RGB module: HFOV 69.4 deg, VFOV 42.6 deg,
# 16:9. USD expresses focalLength and aperture in tenths of a scene unit, but the
# projection depends only on their RATIO, so the absolute unit does not change
# the FOV. Values below reproduce the D435 numbers exactly.
CAM_FOCAL = 1.88
CAM_HAPERTURE = 2.604
CAM_VAPERTURE = 1.4648
CAM_CLIP = (0.105, 10.0)

# Fixed ceiling cameras for site-wide SDG. 6 mm lens on a 1/1.8" sensor
# (7.18 x 5.32 mm) -> HFOV 61.6 deg.
FIXED_FOCAL, FIXED_HAP, FIXED_VAP = 6.0, 7.18, 5.32

# Rover first-person view. Wide-angle inspection lens: 2.8 mm on a 1/2.3"
# sensor (5.6 x 3.15 mm active, 16:9) -> HFOV 90.0 deg, VFOV 58.7 deg.
# Near clip 0.05 m so the rover's own mast stays visible at the frame edge.
ROVER_FOCAL, ROVER_HAP, ROVER_VAP = 2.8, 5.6, 3.15
ROVER_CLIP = (0.05, 40.0)

# Third-person VIEW cameras -- for looking AT the rover, not for perception.
# Full-frame equivalents (36 x 20.25 mm gate, 16:9) so the focal lengths read
# the way a photographer expects: 35 mm wide, 50 mm normal.
VIEW_HAP, VIEW_VAP, VIEW_CLIP = 36.0, 20.25, (0.1, 80.0)
VIEW_CAMS = [
    # name,            eye,                    target,            focal
    ("rover_aisle",    (1.00, -1.10, 2.30), (-8.00, 0.00, 0.65), 35.0),
    ("rover_closeup",  (-4.60, -1.25, 1.75), (-8.00, 0.00, 0.62), 50.0),
]
# Chase camera rides the rover root, so it follows the robot as it drives.
# Chase boom RADIUS is the binding constraint, not lateral offset. The camera is
# rigidly parented, so during an in-place 180 deg turn it sweeps a circle of that
# radius around the rover. The east-end turns happen deep inside a rack segment
# where the aisle is only 3.2 m wide, so the boom must stay under 1.6 m or the
# camera swings into the racking mid-turn. sqrt(1.20^2 + 0.35^2) = 1.25 m leaves
# 0.35 m of margin. A short boom needs a wide lens, hence 24 mm.
CHASE_EYE, CHASE_TARGET, CHASE_FOCAL = (-1.20, -0.35, 1.85), (0.20, 0.0, 0.70), 24.0
# Overhead follow: zero boom radius, so it is free everywhere by construction.
# The clearest way to actually watch a patrol.
CHASE_TOP_EYE, CHASE_TOP_FOCAL = (0.0, 0.0, 6.0), 35.0


def author_sensors():
    import math
    stage = new_layer("simulation/sensors.usda",
                      "Cameras. RTX Lidar and IMU prims are NOT authored here "
                      "-- see the note attribute on each mount.")
    _skeleton(stage)

    def camera(path, focal, hap, vap, clip):
        c = UsdGeom.Camera.Define(stage, path)
        c.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
        c.CreateFocalLengthAttr().Set(focal)
        c.CreateHorizontalApertureAttr().Set(hap)
        c.CreateVerticalApertureAttr().Set(vap)
        c.CreateClippingRangeAttr().Set(Gf.Vec2f(*clip))
        return c

    n_cam = 0
    for name, *_ in FLEET:
        base = f"/World/Scenario/Fleet/{name}"
        stage.OverridePrim(base)
        stage.OverridePrim(base + "/Sensors")
        mount = stage.OverridePrim(base + "/Sensors/camera_mount")
        c = camera(base + "/Sensors/camera_mount/rgb",
                   CAM_FOCAL, CAM_HAPERTURE, CAM_VAPERTURE, CAM_CLIP)
        set_xform(c.GetPrim())      # inherits the mount's pose exactly
        n_cam += 1

        # RTX Lidar: Isaac Sim 6.0 replaced the JSON-config-on-a-Camera-prim
        # route with a new experimental RTX sensor API. I have NOT verified the
        # exact 6.0.1 call signature, so no lidar prim is invented here. The
        # mount transform is correct and ready; create the sensor in Kit.
        lm = stage.OverridePrim(base + "/Sensors/lidar_mount")
        lm.CreateAttribute("isaac:note", Sdf.ValueTypeNames.String,
                           custom=True).Set(
            "RTX Lidar not authored offline. Create in Kit under this prim. "
            "Mount pose is final: (0.42, 0, 0.20) m in robot frame, +X forward. "
            "Modelled sensor: SICK nanoScan3, 275 deg aperture, scan plane at "
            "0.20 m AGL.")
        im = stage.OverridePrim(base + "/Sensors/imu_mount")
        im.CreateAttribute("isaac:note", Sdf.ValueTypeNames.String,
                           custom=True).Set(
            "IMU not authored offline -- isaacsim.sensors.physics is a Kit "
            "extension. Mount pose is final: (0, 0, 0.24) m, coincident with "
            "the chassis body origin.")

    # ---- rover first-person view -------------------------------------
    # Parented under tilt_head/camera_mount so the pan and tilt joints
    # genuinely aim it -- driving joint_pan / joint_tilt moves the shot.
    for name, *_ in ROVERS:
        base = f"/World/Scenario/Fleet/{name}"
        stage.OverridePrim(base)
        stage.OverridePrim(base + "/tilt_head")
        stage.OverridePrim(base + "/tilt_head/camera_mount")
        c = camera(base + "/tilt_head/camera_mount/pov",
                   ROVER_FOCAL, ROVER_HAP, ROVER_VAP, ROVER_CLIP)
        set_xform(c.GetPrim())      # inherits the mount pose exactly
        c.GetPrim().CreateAttribute("isaac:note", Sdf.ValueTypeNames.String,
                                    custom=True).Set(
            "Rover first-person view. Select this prim as the active viewport "
            "camera, or bind it to a Replicator render product. Aimed by "
            "joint_pan (Z, +/-175 deg) and joint_tilt (Y, +/-45 deg).")
        n_cam += 1

    # ---- third-person views of the rover ------------------------------
    UsdGeom.Scope.Define(stage, "/World/Simulation")
    UsdGeom.Scope.Define(stage, "/World/Simulation/ViewCameras")
    for nm, eye, tgt, focal in VIEW_CAMS:
        c = camera(f"/World/Simulation/ViewCameras/{nm}",
                   focal, VIEW_HAP, VIEW_VAP, VIEW_CLIP)
        set_xform(c.GetPrim(), eye, look_at_orient(eye, tgt))
        c.GetPrim().CreateAttribute("isaac:note", Sdf.ValueTypeNames.String,
                                    custom=True).Set(
            f"Third-person view of rover_01. Static, world-space. "
            f"{focal:.0f} mm equivalent. Both eye and target sit inside the "
            f"3.2 m aisle, so nothing clips through racking.")
        n_cam += 1
    for name, *_ in ROVERS:
        base = f"/World/Scenario/Fleet/{name}"
        c = camera(f"{base}/ViewCameras/chase",
                   CHASE_FOCAL, VIEW_HAP, VIEW_VAP, VIEW_CLIP)
        set_xform(c.GetPrim(), CHASE_EYE,
                  look_at_orient(CHASE_EYE, CHASE_TARGET))
        c.GetPrim().CreateAttribute("isaac:note", Sdf.ValueTypeNames.String,
                                    custom=True).Set(
            "Chase view. Parented to the rover root, so it FOLLOWS the robot. "
            "Rear-left 3/4 on a 1.25 m boom -- short enough to stay clear of "
            "racking while the rover turns in place.")
        n_cam += 1
        c = camera(f"{base}/ViewCameras/chase_top",
                   CHASE_TOP_FOCAL, VIEW_HAP, VIEW_VAP, (0.5, 40.0))
        set_xform(c.GetPrim(), CHASE_TOP_EYE,
                  look_at_orient(CHASE_TOP_EYE, (0.0, 0.0, 0.0), up=(1, 0, 0)))
        c.GetPrim().CreateAttribute("isaac:note", Sdf.ValueTypeNames.String,
                                    custom=True).Set(
            "Overhead follow, 6 m above the rover, image-up = robot forward. "
            "Zero boom radius, so it never clips racking regardless of heading.")
        n_cam += 1

    # fixed ceiling cameras, one per aisle, looking straight down
    UsdGeom.Scope.Define(stage, "/World/Simulation")
    fx = UsdGeom.Scope.Define(stage, "/World/Simulation/FixedCameras")
    down = Gf.Matrix4d(1).SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0))
    q = down.ExtractRotationQuat()
    for i, ay in enumerate(AISLE_Y):
        c = camera(f"/World/Simulation/FixedCameras/ceiling_{i:02d}",
                   FIXED_FOCAL, FIXED_HAP, FIXED_VAP, (0.5, 30.0))
        set_xform(c.GetPrim(), (0.0, ay, CLEAR_H - 0.4),
                  Gf.Quatd(q.GetReal(), Gf.Vec3d(*q.GetImaginary())))
        n_cam += 1

    stage.GetRootLayer().Save()
    hfov = 2 * math.degrees(math.atan(CAM_HAPERTURE / (2 * CAM_FOCAL)))
    vfov = 2 * math.degrees(math.atan(CAM_VAPERTURE / (2 * CAM_FOCAL)))
    ffov = 2 * math.degrees(math.atan(FIXED_HAP / (2 * FIXED_FOCAL)))
    rhfov = 2 * math.degrees(math.atan(ROVER_HAP / (2 * ROVER_FOCAL)))
    rvfov = 2 * math.degrees(math.atan(ROVER_VAP / (2 * ROVER_FOCAL)))
    return dict(cameras=n_cam, view_cams=len(VIEW_CAMS) + 2 * len(ROVERS),
                amr_hfov=round(hfov, 1), amr_vfov=round(vfov, 1),
                rover_pov_hfov=round(rhfov, 1), rover_pov_vfov=round(rvfov, 1),
                fixed_hfov=round(ffov, 1), rtx_lidar="mount only, not authored",
                imu="mount only, not authored")


# ----------------------------------------------------------------- semantics
# Flat taxonomy, one label per prim, taxonomy name "class".
SEMANTIC_MAP = [
    (SHELL + "/floor_slab", "floor"),
    (SHELL + "/wall_north", "wall"), (SHELL + "/wall_south", "wall"),
    (SHELL + "/wall_east", "wall"), (SHELL + "/wall_west", "wall"),
    (SHELL + "/roof_deck", "ceiling"),
    (SHELL + "/columns", "column"),
    (INFRA + "/dock_doors", "dock_door"), (INFRA + "/dock_bumpers", "dock_door"),
    (INFRA + "/chargers", "charger"),
    (INFRA + "/Bollards", "bollard"),
    (INFRA + "/FloorMarkings", "floor_marking"),
    (INFRA + "/RackedPallets", "pallet"),
    (INFRA + "/GroundInventory", "pallet"),
]


def author_semantics():
    stage = new_layer("simulation/semantics.usda",
                      "SDG labels for Isaac Sim 6.0: UsdSemantics.LabelsAPI, "
                      "taxonomy 'class'. The legacy Semantics.SemanticsAPI "
                      "(semanticType/semanticData) is deprecated in 6.0 -- see "
                      "isaacsim.core.utils.semantics.upgrade_prim_semantics_to_labels. "
                      "`over` prims only; nothing is defined here.")
    _skeleton(stage)
    n = 0
    for path, label in SEMANTIC_MAP:
        p = stage.OverridePrim(path)
        add_semantics(p, label)
        n += 1
    for r in range(len(RACK_RUN_Y)):
        add_semantics(stage.OverridePrim(f"{ENV}/Racking/rack_run_{r:02d}"),
                      "rack_upright")
        n += 1
    for i in range(len(STAGED_PALLET_Y)):
        add_semantics(stage.OverridePrim(f"/World/Scenario/Staged/pallet_{i:02d}"),
                      "pallet")
        n += 1
    for i in range(4):
        add_semantics(stage.OverridePrim(f"/World/Scenario/Staged/tote_{i:02d}"),
                      "tote")
        n += 1
    for name, *_ in FLEET:
        add_semantics(stage.OverridePrim(f"/World/Scenario/Fleet/{name}"), "amr")
        n += 1
    for name, *_ in ROVERS:
        add_semantics(stage.OverridePrim(f"/World/Scenario/Fleet/{name}"), "rover")
        n += 1
    for i in range(len(AISLE_Y)):
        add_semantics(
            stage.OverridePrim(f"/World/Simulation/FixedCameras/ceiling_{i:02d}"),
            "amr_sensor")
        n += 1
    stage.GetRootLayer().Save()
    used = sorted({l for _, l in SEMANTIC_MAP} |
                  {"rack_upright", "pallet", "tote", "amr", "amr_sensor",
                   "rover", "rover_sensor"})
    return dict(labelled_prims=n, taxonomy="class", classes_used=used)
