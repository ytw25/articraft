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

    # Base Box
    base = model.part("base")
    base.visual(
        Box((0.2, 0.2, 0.1)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="base_box",
    )

    # Top Box
    top = model.part("top")
    top.visual(
        Box((0.2, 0.2, 0.1)),
        # The top part frame will be at the hinge: (0.1, 0, 0.1)
        # So the visual origin relative to the part frame is (-0.1, 0, 0.05)
        origin=Origin(xyz=(-0.1, 0.0, 0.05)),
        name="top_box",
    )

    # Articulation
    model.articulation(
        "hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top,
        origin=Origin(xyz=(0.1, 0.0, 0.1)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.57),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top = object_model.get_part("top")

    # Check contact at rest pose
    ctx.expect_contact(base, top, name="boxes_touch_at_rest")

    return ctx.report()


object_model = build_object_model()
