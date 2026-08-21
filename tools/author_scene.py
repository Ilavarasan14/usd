#!/usr/bin/env python3
"""Author the entire warehouse bay stage from scratch, in dependency order.

    python3 tools/author_scene.py

Runs standalone against usd-core -- the Kit app is NOT required. PhysX
opinions are authored through the apiSchemas listop when the PhysxSchema
plugin is absent, which produces byte-identical USD to PhysxSchema.*API.Apply().

Does NOT author: RTX Lidar prims, IMU prims, MDL texture resolution. Those
need Isaac Sim. Mount transforms for the sensors are final and already in place.
"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import wh_common
import author_env, author_assets, author_rover, author_infra, author_scenario
import author_sim, author_lighting

STEPS = [
    ("environment/shell.usdc",            author_env.author_shell),
    ("environment/racking.usdc",          author_env.author_racking),
    ("assets/props/pallet/*",             author_assets.author_pallet),
    ("assets/props/tote/*",               author_assets.author_tote),
    ("assets/robots/amr_tote/*",          author_assets.author_amr),
    ("assets/robots/rover/*",             author_rover.author_rover),
    ("environment/infrastructure.usdc",   author_infra.author_infrastructure),
    ("scenario/placements.usda",          author_scenario.author_placements),
    ("scenario/routes.usda",              author_scenario.author_routes),
    ("scenario/timeline.usda",            author_scenario.author_timeline),
    ("simulation/physics.usda",           author_sim.author_physics),
    ("simulation/materials.usda",         author_sim.author_materials),
    ("simulation/sensors.usda",           author_sim.author_sensors),
    ("simulation/semantics.usda",         author_sim.author_semantics),
    ("lighting/sun.usda",                 author_lighting.author_sun),
    ("lighting/sky.usda",                 author_lighting.author_sky),
    ("lighting/artificial_lights.usda",   author_lighting.author_artificial),
]

# root.usda, safety/constraints.usda and safety/overrides.usda are hand-authored
# composition layers and are deliberately NOT regenerated here.
HAND_AUTHORED = ["root.usda", "safety/constraints.usda", "safety/overrides.usda"]


def main():
    print(f"SCENE_ROOT = {wh_common.SCENE_ROOT}")
    print(f"PhysxSchema plugin present: {wh_common.HAVE_PHYSX_SCHEMA}"
          f"{'' if wh_common.HAVE_PHYSX_SCHEMA else '  (using apiSchemas listop path)'}\n")
    for rel in HAND_AUTHORED:
        ok = os.path.exists(os.path.join(wh_common.SCENE_ROOT, rel))
        print(f"  {'found  ' if ok else 'MISSING'} {rel}  (hand-authored, not regenerated)")
    print()
    t0 = time.time()
    for label, fn in STEPS:
        t = time.time()
        info = fn()
        print(f"  {label:<36} {time.time()-t:5.2f}s  {info}")
    print(f"\nauthored {len(STEPS)} layers in {time.time()-t0:.1f}s")
    print("next: python3 tools/validate_scene.py")


if __name__ == "__main__":
    main()
