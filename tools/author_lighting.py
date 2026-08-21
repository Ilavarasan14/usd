"""lighting/{sun,sky,artificial_lights}.usda

Site: distribution center, Reno NV.  39.5296 N, 119.8138 W.
Time: 2026-06-21 10:00 local (PDT, UTC-7) -> 17:00 UTC.
Solar azimuth/elevation are computed with the NOAA algorithm below, not guessed.
"""
import math
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf
from wh_common import *

SITE_LAT, SITE_LON = 39.5296, -119.8138
SITE_DATE = (2026, 6, 21)
SITE_LOCAL_HOUR, SITE_UTC_OFFSET = 10.0, -7.0
FIXTURE_Z = 9.6


def solar_position(year, month, day, utc_hour, lat, lon):
    """NOAA solar position. Returns (azimuth_deg_from_north_CW, elevation_deg)."""
    if month <= 2:
        year, month = year - 1, month + 12
    A = year // 100
    B = 2 - A + A // 4
    jd = (math.floor(365.25 * (year + 4716)) + math.floor(30.6001 * (month + 1))
          + day + B - 1524.5 + utc_hour / 24.0)
    t = (jd - 2451545.0) / 36525.0
    L0 = (280.46646 + t * (36000.76983 + t * 0.0003032)) % 360.0
    M = 357.52911 + t * (35999.05029 - 0.0001537 * t)
    e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t)
    Mr = math.radians(M)
    C = (math.sin(Mr) * (1.914602 - t * (0.004817 + 0.000014 * t))
         + math.sin(2 * Mr) * (0.019993 - 0.000101 * t)
         + math.sin(3 * Mr) * 0.000289)
    true_long = L0 + C
    omega = 125.04 - 1934.136 * t
    app_long = true_long - 0.00569 - 0.00478 * math.sin(math.radians(omega))
    seconds = 21.448 - t * (46.8150 + t * (0.00059 - t * 0.001813))
    e0 = 23.0 + (26.0 + seconds / 60.0) / 60.0
    ec = e0 + 0.00256 * math.cos(math.radians(omega))
    decl = math.degrees(math.asin(math.sin(math.radians(ec))
                                 * math.sin(math.radians(app_long))))
    y = math.tan(math.radians(ec / 2.0)) ** 2
    L0r = math.radians(L0)
    eot = 4.0 * math.degrees(
        y * math.sin(2 * L0r) - 2 * e * math.sin(Mr)
        + 4 * e * y * math.sin(Mr) * math.cos(2 * L0r)
        - 0.5 * y * y * math.sin(4 * L0r) - 1.25 * e * e * math.sin(2 * Mr))
    tst = (utc_hour * 60.0 + eot + 4.0 * lon) % 1440.0
    ha = tst / 4.0 - 180.0 if tst / 4.0 >= 0 else tst / 4.0 + 180.0
    latr, declr, har = math.radians(lat), math.radians(decl), math.radians(ha)
    cz = (math.sin(latr) * math.sin(declr)
          + math.cos(latr) * math.cos(declr) * math.cos(har))
    cz = max(-1.0, min(1.0, cz))
    zenith = math.degrees(math.acos(cz))
    elevation = 90.0 - zenith
    den = math.cos(latr) * math.sin(math.radians(zenith))
    if abs(den) < 1e-9:
        az = 180.0
    else:
        ca = (math.sin(latr) * cz - math.sin(declr)) / den
        ca = max(-1.0, min(1.0, ca))
        az = math.degrees(math.acos(ca))
        az = (az + 180.0) if ha > 0 else (180.0 - az)
    return az % 360.0, elevation


def sun_vector(az_deg, el_deg):
    """Unit vector from the site toward the sun. +X = east, +Y = north, +Z = up."""
    az, el = math.radians(az_deg), math.radians(el_deg)
    return Gf.Vec3d(math.cos(el) * math.sin(az),
                    math.cos(el) * math.cos(az),
                    math.sin(el))


def author_sun():
    az, el = solar_position(*SITE_DATE, SITE_LOCAL_HOUR - SITE_UTC_OFFSET,
                            SITE_LAT, SITE_LON)
    stage = new_layer(
        "lighting/sun.usda",
        f"Solar DistantLight. Site {SITE_LAT}N {abs(SITE_LON)}W, "
        f"{SITE_DATE[0]}-{SITE_DATE[1]:02d}-{SITE_DATE[2]:02d} "
        f"{SITE_LOCAL_HOUR:.0f}:00 PDT. Computed azimuth {az:.2f} deg from north, "
        f"elevation {el:.2f} deg. Indoors the roof occludes this light almost "
        f"entirely -- it reaches the floor through the 32 skylight cells, which "
        f"is the intended behaviour.")
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Scope.Define(stage, "/World/Lighting")

    d = UsdLux.DistantLight.Define(stage, "/World/Lighting/sun")
    d.CreateAngleAttr().Set(0.53)          # true angular diameter of the solar disc
    d.CreateIntensityAttr().Set(3.2)
    d.CreateExposureAttr().Set(9.5)
    d.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.955, 0.90))   # ~5200 K at this elevation
    # DistantLight emits along local -Z, so aim -Z down the incoming ray:
    # rotate +Z onto the vector pointing at the sun.
    v = sun_vector(az, el)
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), v)
    q = rot.GetQuat()
    set_xform(d.GetPrim(), (0, 0, CLEAR_H + 5.0),
              Gf.Quatd(q.GetReal(), Gf.Vec3d(*q.GetImaginary())))
    stage.GetRootLayer().Save()

    m = Gf.Matrix4d(1).SetRotate(rot)
    emit = m.TransformDir(Gf.Vec3d(0, 0, -1))
    return dict(azimuth_deg=round(az, 2), elevation_deg=round(el, 2),
                sun_vector=tuple(round(c, 4) for c in v),
                emit_dir=tuple(round(c, 4) for c in emit),
                angle_deg=0.53)


def author_sky():
    stage = new_layer(
        "lighting/sky.usda",
        "Sky DomeLight. No HDRI is shipped with this scene, so a physically "
        "plausible clear-sky colour and intensity are used instead; swap "
        "inputs:texture:file for a measured HDRI when one is available. "
        "Indoors this mainly lights the floor through the skylights and "
        "provides the ambient bounce term.")
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Scope.Define(stage, "/World/Lighting")
    dome = UsdLux.DomeLight.Define(stage, "/World/Lighting/sky")
    dome.CreateIntensityAttr().Set(1.0)
    dome.CreateExposureAttr().Set(6.0)
    dome.CreateColorAttr().Set(Gf.Vec3f(0.62, 0.76, 1.0))    # clear zenith blue
    set_xform(dome.GetPrim())
    stage.GetRootLayer().Save()
    return dict(hdri=None, color=(0.62, 0.76, 1.0), exposure=6.0)


def author_artificial():
    stage = new_layer(
        "lighting/artificial_lights.usda",
        "LED high-bay array, 1.2 x 0.3 m linear fixtures at 9.6 m, on aisle and "
        "cross-aisle centrelines (never over a rack run -- the racking would "
        "shadow the working plane). ~5000 K. NOTE: Kit's UsdLux intensity is "
        "not photometric by default, so these values are tuned for "
        "sensor-plausible exposure, not asserted as lumen-accurate. They have "
        "NOT been render-verified -- no RTX renderer is available offline.")
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Scope.Define(stage, "/World/Lighting")
    grp = UsdGeom.Scope.Define(stage, "/World/Lighting/HighBay")

    n = 0
    def fixture(path, x, y, yaw):
        nonlocal n
        r = UsdLux.RectLight.Define(stage, path)
        r.CreateWidthAttr(1.2)
        r.CreateHeightAttr(0.3)
        r.CreateIntensityAttr(1800.0)
        r.CreateExposureAttr(0.0)
        r.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.94))     # ~5000 K
        r.CreateNormalizeAttr(True)
        # RectLight emits along local -Z; identity orientation already points down
        set_xform(r.GetPrim(), (x, y, FIXTURE_Z),
                  quat_from_axis_angle((0, 0, 1), yaw))
        n += 1

    for i, ay in enumerate(AISLE_Y):
        for k in range(10):
            x = -27.0 + 54.0 * k / 9.0
            fixture(f"/World/Lighting/HighBay/aisle_{i}_{k:02d}", x, ay, 0.0)
    for k in range(4):
        y = -9.0 + 18.0 * k / 3.0
        fixture(f"/World/Lighting/HighBay/cross_{k:02d}", 0.0, y, 90.0)

    # dock apron task lighting, aimed down at the door thresholds
    for j, dy in enumerate([-6.75, -2.25, 2.25, 6.75]):
        fixture(f"/World/Lighting/HighBay/dock_{j:02d}", 27.5, dy, 90.0)

    stage.GetRootLayer().Save()
    return dict(fixtures=n, mount_height_m=FIXTURE_Z, cct_k=5000,
                render_verified=False)
