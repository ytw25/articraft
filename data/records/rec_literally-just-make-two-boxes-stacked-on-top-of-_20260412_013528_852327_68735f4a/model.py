from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacked_boxes")

    # Base box
    base = model.part("base")
    base.visual(
        Box((0.2, 0.2, 0.1)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="base_box",
    )

    # Top box (lid)
    top = model.part("top")
    top.visual(
        Box((0.2, 0.2, 0.05)),
        # The top box part frame will be at the hinge line (rear edge).
        # Rear edge is at y = 0.1 in world coords if centered.
        # But let's define the top box relative to its own frame at the hinge.
        # If the hinge is at (0, 0.1, 0.1), then the top box center is at (0, 0, 0.025)
        # relative to that if we want it to extend forward (along -Y).
        origin=Origin(xyz=(0.0, -0.1, 0.025)),
        name="top_box",
    )

    # Articulation: Revolute hinge at the top rear edge of the base.
    model.articulation(
        "base_to_top",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top,
        origin=Origin(xyz=(0.0, 0.1, 0.1)),
        axis=(1.0, 0.0, 0.0),  # Rotate around X axis
        motion_limits=MotionLimits(lower=0.0, upper=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top = object_model.get_part("top")
    hinge = object_model.get_articulation("base_to_top")

    # Check rest pose: top should be on top of base
    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(top, base, axis="z", min_gap=-0.001, max_gap=0.001, name="top_rests_on_base")
        ctx.expect_overlap(top, base, axes="xy", min_overlap=0.1, name="top_aligned_with_base")

    return ctx.report()


object_model = build_object_model()
