"""Rebuild the Pragyaan robot USD from its source copy (docs/todo_pragyaan_physics.md).

nograph_pragyaan_source.usd is the asset as exported from Blender, kept unchanged. This script copies it to
nograph_pragyaan.usd, the file that cfg/robot/pragyaan.yaml loads, and applies the fix steps to the copy. Each run starts
again from the source copy, so the script is safe to run twice.

    pixi run -e yamcs python scripts/assets/rebuild_pragyaan.py

Steps:
  1. Mass properties. The real rover weighs 26 kg. Without authored masses, PhysX used its default density of
     1000 kg/m3 on the collider volumes (675 kg in total), and base_link, which has no collider, got a 1 kg fallback
     and the "invalid inertia tensor ... negative mass" warning. Each rigid body gets a mass. PhysX computes the
     inertia and the center of mass from the colliders of the body (also from disabled colliders, as on the solar
     panel). base_link gets an explicit small inertia and center of mass.
"""

import shutil
import sys
from pathlib import Path

from pxr import Gf, Usd, UsdPhysics

REPO_ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = REPO_ROOT / "assets" / "USD_Assets" / "robots" / "pragyaan"
SOURCE = ASSET_DIR / "nograph_pragyaan_source.usd"
OUTPUT = ASSET_DIR / "nograph_pragyaan.usd"
ROOT = "/Pragyaan"

TOTAL_MASS_KG = 26.0
# kg per rigid body, relative to ROOT. Split agreed in docs/todo_pragyaan_physics.md (D2).
MASSES_KG = {
    "Body": 16.99,
    "solarPanel": 1.5,
    "Mobility/Wheel_front_left": 0.75,
    "Mobility/Wheel_middle_left": 0.75,
    "Mobility/Wheel_back_left": 0.75,
    "Mobility/Wheel_front_right": 0.75,
    "Mobility/Wheel_middle_right": 0.75,
    "Mobility/Wheel_back_right": 0.75,
    "rocker_bogie/rocker_left": 0.9,
    "rocker_bogie/rocker_right": 0.9,
    "rocker_bogie/bogie_left": 0.6,
    "rocker_bogie/bogie_right": 0.6,
    "base_link": 0.01,
}
# base_link (IMU mount) has no collider, so PhysX cannot compute its inertia: a 1 cm sphere-like value is authored.
BASE_LINK_INERTIA_KGM2 = 1e-6


def apply_masses(stage: Usd.Stage) -> None:
    rigid_bodies = {
        p.GetPath().pathString for p in Usd.PrimRange(stage.GetPrimAtPath(ROOT)) if p.HasAPI(UsdPhysics.RigidBodyAPI)
    }
    configured = {f"{ROOT}/{rel}" for rel in MASSES_KG}
    if rigid_bodies != configured:
        sys.exit(f"Rigid bodies and MASSES_KG differ: {sorted(rigid_bodies ^ configured)}")
    if abs(sum(MASSES_KG.values()) - TOTAL_MASS_KG) > 1e-6:
        sys.exit(f"MASSES_KG sums to {sum(MASSES_KG.values())} kg, expected {TOTAL_MASS_KG} kg")

    for rel, mass in MASSES_KG.items():
        mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(f"{ROOT}/{rel}"))
        mass_api.CreateMassAttr().Set(mass)
    base_link = UsdPhysics.MassAPI(stage.GetPrimAtPath(f"{ROOT}/base_link"))
    base_link.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(BASE_LINK_INERTIA_KGM2))
    base_link.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0))
    base_link.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0))


def main() -> None:
    shutil.copyfile(SOURCE, OUTPUT)
    # The copy sits next to the source, so its relative references (./rocker_bogie.usd) still resolve.
    stage = Usd.Stage.Open(str(OUTPUT))
    apply_masses(stage)
    stage.GetRootLayer().Save()

    stage = Usd.Stage.Open(str(OUTPUT))
    total = 0.0
    for p in Usd.PrimRange(stage.GetPrimAtPath(ROOT)):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            mass = UsdPhysics.MassAPI(p).GetMassAttr().Get()
            total += mass
            print(f"  {p.GetPath().pathString:40s} {mass:6.2f} kg")
    print(f"Wrote {OUTPUT.relative_to(REPO_ROOT)}: total mass {total:.2f} kg")


if __name__ == "__main__":
    main()
