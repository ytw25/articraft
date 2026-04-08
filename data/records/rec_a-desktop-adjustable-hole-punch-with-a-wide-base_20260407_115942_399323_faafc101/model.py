from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_adjustable_hole_punch")

    frame_metal = model.material("frame_metal", rgba=(0.21, 0.22, 0.24, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.14, 0.15, 0.16, 1.0))
    plastic_stop = model.material("plastic_stop", rgba=(0.72, 0.74, 0.76, 1.0))
    accent = model.material("accent", rgba=(0.88, 0.12, 0.10, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.32, 0.17, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=frame_metal,
        name="base_plate",
    )
    frame.visual(
        Box((0.25, 0.08, 0.038)),
        origin=Origin(xyz=(0.0, 0.010, 0.037)),
        material=frame_metal,
        name="punch_body",
    )
    frame.visual(
        Box((0.032, 0.055, 0.060)),
        origin=Origin(xyz=(-0.123, 0.033, 0.048)),
        material=frame_metal,
        name="left_hinge_cheek",
    )
    frame.visual(
        Box((0.032, 0.055, 0.060)),
        origin=Origin(xyz=(0.123, 0.033, 0.048)),
        material=frame_metal,
        name="right_hinge_cheek",
    )
    frame.visual(
        Box((0.24, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.035, 0.021)),
        material=frame_metal,
        name="paper_shelf",
    )
    frame.visual(
        Box((0.26, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, -0.041, 0.030)),
        material=frame_metal,
        name="paper_fence",
    )
    frame.visual(
        Cylinder(radius=0.0065, length=0.24),
        origin=Origin(xyz=(0.0, -0.070, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="guide_rail",
    )
    frame.visual(
        Box((0.026, 0.020, 0.010)),
        origin=Origin(xyz=(-0.095, -0.070, 0.023)),
        material=frame_metal,
        name="left_rail_support",
    )
    frame.visual(
        Box((0.026, 0.020, 0.010)),
        origin=Origin(xyz=(0.095, -0.070, 0.023)),
        material=frame_metal,
        name="right_rail_support",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.040, -0.002, 0.059)),
        material=accent,
        name="left_punch_die",
    )
    frame.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.040, -0.002, 0.059)),
        material=accent,
        name="right_punch_die",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.32, 0.17, 0.090)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.009, length=0.206),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.20, 0.024, 0.016)),
        origin=Origin(xyz=(0.0, -0.018, -0.003)),
        material=handle_metal,
        name="rear_web",
    )
    handle.visual(
        Box((0.21, 0.076, 0.014)),
        origin=Origin(xyz=(0.0, -0.055, -0.003)),
        material=handle_metal,
        name="handle_body",
    )
    handle.visual(
        Box((0.028, 0.050, 0.024)),
        origin=Origin(xyz=(-0.094, -0.030, -0.002)),
        material=handle_metal,
        name="left_handle_arm",
    )
    handle.visual(
        Box((0.028, 0.050, 0.024)),
        origin=Origin(xyz=(0.094, -0.030, -0.002)),
        material=handle_metal,
        name="right_handle_arm",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.214),
        origin=Origin(xyz=(0.0, -0.091, -0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_metal,
        name="front_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.22, 0.11, 0.040)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.045, -0.002)),
    )

    left_stop = model.part("left_stop")
    left_stop.visual(
        Box((0.026, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, -0.009, 0.005)),
        material=plastic_stop,
        name="clamp_front",
    )
    left_stop.visual(
        Box((0.026, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.009, 0.005)),
        material=plastic_stop,
        name="clamp_rear",
    )
    left_stop.visual(
        Box((0.026, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=plastic_stop,
        name="clamp_bridge",
    )
    left_stop.visual(
        Box((0.014, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.010, 0.014)),
        material=plastic_stop,
        name="stop_connector",
    )
    left_stop.visual(
        Box((0.014, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.020, 0.024)),
        material=plastic_stop,
        name="stop_fin",
    )
    left_stop.inertial = Inertial.from_geometry(
        Box((0.03, 0.032, 0.040)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.008, 0.018)),
    )

    right_stop = model.part("right_stop")
    right_stop.visual(
        Box((0.026, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, -0.009, 0.005)),
        material=plastic_stop,
        name="clamp_front",
    )
    right_stop.visual(
        Box((0.026, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.009, 0.005)),
        material=plastic_stop,
        name="clamp_rear",
    )
    right_stop.visual(
        Box((0.026, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=plastic_stop,
        name="clamp_bridge",
    )
    right_stop.visual(
        Box((0.014, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.010, 0.014)),
        material=plastic_stop,
        name="stop_connector",
    )
    right_stop.visual(
        Box((0.014, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.020, 0.024)),
        material=plastic_stop,
        name="stop_fin",
    )
    right_stop.inertial = Inertial.from_geometry(
        Box((0.03, 0.032, 0.040)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.008, 0.018)),
    )

    model.articulation(
        "frame_to_handle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(0.0, 0.058, 0.072)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "frame_to_left_stop",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=left_stop,
        origin=Origin(xyz=(-0.070, -0.070, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.10,
            lower=0.0,
            upper=0.060,
        ),
    )
    model.articulation(
        "frame_to_right_stop",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=right_stop,
        origin=Origin(xyz=(0.070, -0.070, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.10,
            lower=-0.060,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    handle = object_model.get_part("handle")
    left_stop = object_model.get_part("left_stop")
    right_stop = object_model.get_part("right_stop")
    handle_joint = object_model.get_articulation("frame_to_handle")
    left_stop_joint = object_model.get_articulation("frame_to_left_stop")
    right_stop_joint = object_model.get_articulation("frame_to_right_stop")

    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_gap(
            handle,
            frame,
            axis="z",
            positive_elem="handle_body",
            negative_elem="punch_body",
            min_gap=0.004,
            max_gap=0.010,
            name="closed handle sits just above the punch body",
        )
        ctx.expect_gap(
            handle,
            frame,
            axis="z",
            positive_elem="front_grip",
            negative_elem="base_plate",
            min_gap=0.030,
            max_gap=0.045,
            name="front grip clears the base at rest",
        )
        rest_grip = ctx.part_element_world_aabb(handle, elem="front_grip")

    with ctx.pose({handle_joint: math.radians(55.0)}):
        open_grip = ctx.part_element_world_aabb(handle, elem="front_grip")

    ctx.check(
        "handle opens upward around the rear hinge",
        rest_grip is not None
        and open_grip is not None
        and open_grip[0][2] > rest_grip[0][2] + 0.060,
        details=f"rest_grip={rest_grip}, open_grip={open_grip}",
    )

    left_rest = ctx.part_world_position(left_stop)
    with ctx.pose({left_stop_joint: 0.050}):
        left_shifted = ctx.part_world_position(left_stop)
    ctx.check(
        "left stop slides rightward along the guide rail",
        left_rest is not None
        and left_shifted is not None
        and left_shifted[0] > left_rest[0] + 0.040
        and abs(left_shifted[1] - left_rest[1]) < 1e-6
        and abs(left_shifted[2] - left_rest[2]) < 1e-6,
        details=f"left_rest={left_rest}, left_shifted={left_shifted}",
    )

    right_rest = ctx.part_world_position(right_stop)
    with ctx.pose({right_stop_joint: -0.050}):
        right_shifted = ctx.part_world_position(right_stop)
    ctx.check(
        "right stop slides leftward along the guide rail",
        right_rest is not None
        and right_shifted is not None
        and right_shifted[0] < right_rest[0] - 0.040
        and abs(right_shifted[1] - right_rest[1]) < 1e-6
        and abs(right_shifted[2] - right_rest[2]) < 1e-6,
        details=f"right_rest={right_rest}, right_shifted={right_shifted}",
    )

    ctx.expect_gap(
        frame,
        left_stop,
        axis="y",
        positive_elem="paper_fence",
        negative_elem="stop_fin",
        min_gap=0.0005,
        max_gap=0.003,
        name="left stop fin sits just in front of the paper fence",
    )
    ctx.expect_gap(
        frame,
        right_stop,
        axis="y",
        positive_elem="paper_fence",
        negative_elem="stop_fin",
        min_gap=0.0005,
        max_gap=0.003,
        name="right stop fin sits just in front of the paper fence",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
