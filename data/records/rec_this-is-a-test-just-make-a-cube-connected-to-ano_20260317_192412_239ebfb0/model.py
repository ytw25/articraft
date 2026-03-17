from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from pathlib import Path
from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_box", assets=ASSETS)

    # Materials
    base_mat = model.material("dark_plastic", rgba=(0.2, 0.2, 0.2, 1.0))
    lid_mat = model.material("brushed_metal", rgba=(0.7, 0.7, 0.75, 1.0))

    # Base cube
    base = model.part("base")
    base_shape = Box((0.1, 0.1, 0.1))
    base.visual(base_shape, origin=Origin(xyz=(0, 0, 0.05)), material=base_mat)
    base.inertial = Inertial.from_geometry(base_shape, mass=1.0, origin=Origin(xyz=(0, 0, 0.05)))

    # Lid (thin box)
    top = model.part("top")
    top_shape = Box((0.1, 0.1, 0.02))
    # We'll place the joint at the top-back edge of the base: (0, 0.05, 0.1)
    # The lid sits on top of the base, so it spans z=[0.1, 0.12].
    # Relative to the joint at (0, 0.05, 0.1):
    # The lid's center in world is (0, 0, 0.11).
    # Thus the lid visual origin relative to the joint is (0, -0.05, 0.01).
    top.visual(top_shape, origin=Origin(xyz=(0, -0.05, 0.01)), material=lid_mat)
    top.inertial = Inertial.from_geometry(top_shape, mass=0.2, origin=Origin(xyz=(0, -0.05, 0.01)))

    # Revolute joint connecting base and lid
    # Axis (-1, 0, 0) with positive rotation follows RHR around -X:
    # Curl from -Y to +Z. This swings the lid UP and BACK (positive Y).
    model.articulation(
        name="lid_hinge",
        articulation_type=ArticulationType.REVOLUTE,
        parent="base",
        child="top",
        origin=Origin(xyz=(0, 0.05, 0.1)),
        axis=(-1, 0, 0),
        motion_limits=MotionLimits(lower=0.0, upper=1.57),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    
    # Check rest pose: lid should be on top of base
    ctx.expect_aabb_gap("top", "base", axis="z", max_gap=0.001, max_penetration=0.001)
    ctx.expect_aabb_overlap("top", "base", axes="xy", min_overlap=0.09)

    # Check motion: lid opens UP and BACK
    with ctx.pose(lid_hinge=1.57):
        # Center moves from (0, 0, 0.11) to (0, 0.06, 0.15)
        # delta_z = 0.04 (positive)
        # delta_y = 0.06 (positive)
        ctx.expect_joint_motion_axis("lid_hinge", "top", world_axis="z", direction="positive")
        ctx.expect_joint_motion_axis("lid_hinge", "top", world_axis="y", direction="positive")

    ctx.check_no_overlaps(ignore_adjacent=True)
    
    return ctx.report()

object_model = build_object_model()
# >>> USER_CODE_END

object_model = build_object_model()
