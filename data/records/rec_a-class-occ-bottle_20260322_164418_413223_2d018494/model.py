from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    Material,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
MESH_DIR = ASSETS.mesh_dir


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_bottle", assets=ASSETS)

    # Materials
    glass_mat = model.material("glass", rgba=(0.7, 0.8, 0.9, 0.4))
    liquid_mat = model.material("cola", rgba=(0.2, 0.1, 0.05, 0.95))
    metal_mat = model.material("metal", rgba=(0.8, 0.8, 0.8, 1.0))
    label_mat = model.material("label", rgba=(0.9, 0.1, 0.1, 1.0))  # Red label

    # Bottle Body
    bottle_profile = [
        (0.0, 0.0),
        (0.026, 0.0),
        (0.028, 0.01),
        (0.022, 0.05),  # Pronounced waist
        (0.030, 0.10),  # Shoulder
        (0.013, 0.18),  # Neck
        (0.015, 0.185), # Lip bottom
        (0.015, 0.195), # Lip top
        (0.012, 0.195), # Mouth inner top
        (0.010, 0.18),  # Neck inner
        (0.027, 0.10),  # Shoulder inner
        (0.019, 0.05),  # Waist inner
        (0.025, 0.01),  # Heel inner
        (0.0, 0.005),   # Bottom inner center
    ]
    bottle_geom = LatheGeometry(bottle_profile, segments=64)
    bottle_mesh = mesh_from_geometry(bottle_geom, MESH_DIR / "bottle_body.obj")

    bottle_part = model.part("bottle")
    bottle_part.visual(bottle_mesh, material=glass_mat, name="glass_shell")
    bottle_part.inertial = Inertial.from_geometry(Cylinder(0.03, 0.2), mass=0.5, origin=Origin(xyz=(0.0, 0.0, 0.1)))

    # Liquid
    liquid_profile = [
        (0.0, 0.005),
        (0.0245, 0.01),
        (0.0185, 0.05), # Inside waist
        (0.0265, 0.10), # Inside shoulder
        (0.011, 0.16),  # Fill level
        (0.0, 0.16),
    ]
    liquid_geom = LatheGeometry(liquid_profile, segments=32)
    liquid_mesh = mesh_from_geometry(liquid_geom, MESH_DIR / "liquid.obj")
    bottle_part.visual(liquid_mesh, material=liquid_mat, name="liquid_content")

    # Label (curved to follow the bottle's shoulder)
    label_profile_closed = [
        (0.0275, 0.075), (0.0305, 0.10), (0.0275, 0.125),
        (0.0277, 0.125), (0.0307, 0.10), (0.0277, 0.075)
    ]
    label_geom = LatheGeometry(label_profile_closed, segments=64)
    label_mesh = mesh_from_geometry(label_geom, MESH_DIR / "label.obj")
    bottle_part.visual(label_mesh, material=label_mat, name="main_label")

    # Cap
    cap_part = model.part("cap")
    # Crown cap shape: slightly flared bottom with a bit more detail
    cap_profile = [
        (0.0, 0.0),
        (0.016, 0.0),
        (0.0165, 0.002),
        (0.016, 0.008),
        (0.015, 0.010),
        (0.0, 0.010),
    ]
    cap_geom = LatheGeometry(cap_profile, segments=32)
    cap_mesh = mesh_from_geometry(cap_geom, MESH_DIR / "cap.obj")
    cap_part.visual(cap_mesh, material=metal_mat, name="crown_cap")
    cap_part.inertial = Inertial.from_geometry(Cylinder(0.016, 0.01), mass=0.01, origin=Origin(xyz=(0.0, 0.0, 0.005)))

    # Articulation
    # Cap sits at z=0.195 (lip top)
    model.articulation(
        name="bottle_to_cap",
        articulation_type=ArticulationType.PRISMATIC,
        parent="bottle",
        child="cap",
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Intent checks
    # Cap should be on top of the bottle
    ctx.expect_aabb_gap("cap", "bottle", axis="z", max_gap=0.001, max_penetration=0.002)
    
    # Check motion
    with ctx.pose(bottle_to_cap=0.05):
        # Cap should be 5cm above the lip
        ctx.expect_aabb_gap("cap", "bottle", axis="z", min_gap=0.045, max_gap=0.055)

    return ctx.report()


object_model = build_object_model()


# >>> USER_CODE_END

object_model = build_object_model()
