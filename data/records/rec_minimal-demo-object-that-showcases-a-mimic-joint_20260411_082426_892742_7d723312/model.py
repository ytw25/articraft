from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mimic_gripper_demo")

    palm = model.part("palm")
    palm.visual(
        Box((0.12, 0.08, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        name="palm_body",
    )

    left_jaw = model.part("left_jaw")
    left_jaw.visual(
        Box((0.08, 0.02, 0.025)),
        origin=Origin(xyz=(0.04, 0.0, 0.0125)),
        name="left_jaw_body",
    )

    right_jaw = model.part("right_jaw")
    right_jaw.visual(
        Box((0.08, 0.02, 0.025)),
        origin=Origin(xyz=(0.04, 0.0, 0.0125)),
        name="right_jaw_body",
    )

    jaw_limits = MotionLimits(effort=8.0, velocity=0.04, lower=0.0, upper=0.02)

    model.articulation(
        "palm_to_left_jaw",
        ArticulationType.PRISMATIC,
        parent=palm,
        child=left_jaw,
        origin=Origin(xyz=(0.0, 0.03, 0.03)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=jaw_limits,
    )
    model.articulation(
        "palm_to_right_jaw",
        ArticulationType.PRISMATIC,
        parent=palm,
        child=right_jaw,
        origin=Origin(xyz=(0.0, -0.03, 0.03)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=jaw_limits,
        mimic=Mimic(joint="palm_to_left_jaw", multiplier=-1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    driver = object_model.get_articulation("palm_to_left_jaw")

    left_rest = ctx.part_world_position(left_jaw)
    right_rest = ctx.part_world_position(right_jaw)

    with ctx.pose({driver: 0.015}):
        left_open = ctx.part_world_position(left_jaw)
        right_open = ctx.part_world_position(right_jaw)

    ctx.check(
        "mimic jaws open symmetrically",
        left_rest is not None
        and right_rest is not None
        and left_open is not None
        and right_open is not None
        and left_open[1] > left_rest[1] + 0.01
        and right_open[1] < right_rest[1] - 0.01,
        details=(
            f"left_rest={left_rest}, left_open={left_open}, "
            f"right_rest={right_rest}, right_open={right_open}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
