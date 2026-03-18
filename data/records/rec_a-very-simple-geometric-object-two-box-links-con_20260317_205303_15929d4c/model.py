from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_revolute_boxes", assets=ASSETS)

    # Material definitions for better visual distinction
    wood = model.material("wood", rgba=(0.6, 0.4, 0.2, 1.0))
    metal = model.material("metal", rgba=(0.7, 0.7, 0.7, 1.0))

    # Base Link: A horizontal box
    base = model.part("base")
    base_size = (0.2, 0.05, 0.05)
    # Origin at (0,0,0.025) so it sits on the ground and its top face is at z=0.05
    base.visual(Box(base_size), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=wood)
    base.inertial = Inertial.from_geometry(
        Box(base_size), mass=1.0, origin=Origin(xyz=(0.0, 0.0, 0.025))
    )

    # Pin: A small cylinder connecting them
    pin_radius = 0.01
    pin_height = 0.06
    base.visual(
        Cylinder(radius=pin_radius, length=pin_height),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # Centered on the interface
        material=metal,
        name="pin",
    )

    # Moving Link: Another horizontal box, initially on top of the base
    moving = model.part("moving")
    moving_size = (0.2, 0.05, 0.05)
    # Visual origin at (0,0,0.025) relative to the joint origin at (0,0,0.05)
    # This puts the bottom of the moving box at z=0.05 (flush with base top)
    moving_visual_origin = Origin(xyz=(0.0, 0.0, 0.025))
    moving.visual(Box(moving_size), origin=moving_visual_origin, material=metal)
    moving.inertial = Inertial.from_geometry(
        Box(moving_size), mass=0.5, origin=moving_visual_origin
    )

    # Revolute joint at the center interface
    model.articulation(
        "joint",
        ArticulationType.REVOLUTE,
        parent="base",
        child="moving",
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # Exactly at the interface
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=-1.57, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Check that the articulation origin is near the geometry
    ctx.check_articulation_origin_near_geometry(tol=0.01)

    # Check that parts are connected
    ctx.check_part_geometry_connected(use="visual")

    # Check for overlaps, ignoring adjacent parts (base and moving are adjacent)
    ctx.allow_overlap("base", "moving", reason="pin insertion")
    ctx.check_no_overlaps(
        max_pose_samples=64,
        overlap_tol=0.001,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    # Attachment checks: moving link should be right above base
    # Note: gap is negative because the pin overlaps. Using max_penetration to allow it.
    ctx.expect_aabb_gap("moving", "base", axis="z", max_gap=0.001, max_penetration=0.04)
    ctx.expect_aabb_overlap("moving", "base", axes="xy", min_overlap=0.04)

    # Test motion: at rest, they are aligned
    ctx.expect_origin_distance("moving", "base", axes="xy", max_dist=0.001)

    # At non-zero pose, the AABB center remains centered in XY relative to the base origin.
    with ctx.pose(joint=1.0):
        ctx.expect_origin_distance("moving", "base", axes="xy", max_dist=0.001)
        ctx.expect_aabb_gap("moving", "base", axis="z", max_gap=0.001, max_penetration=0.04)

    return ctx.report()


object_model = build_object_model()
# >>> USER_CODE_END

object_model = build_object_model()
