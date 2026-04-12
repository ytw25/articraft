from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_grinder")

    cast_iron = model.material("cast_iron", rgba=(0.33, 0.36, 0.40, 1.0))
    machine_blue = model.material("machine_blue", rgba=(0.22, 0.39, 0.56, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    stone_gray = model.material("stone_gray", rgba=(0.66, 0.64, 0.60, 1.0))
    shield_clear = model.material("shield_clear", rgba=(0.84, 0.92, 0.98, 0.34))
    switch_black = model.material("switch_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.205, 0.240, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=cast_iron,
        name="base_plate",
    )
    housing.visual(
        Box((0.106, 0.106, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=machine_blue,
        name="pedestal",
    )
    housing.visual(
        Cylinder(radius=0.074, length=0.188),
        origin=Origin(xyz=(0.0, 0.0, 0.128), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machine_blue,
        name="motor_body",
    )
    housing.visual(
        Box((0.086, 0.050, 0.040)),
        origin=Origin(xyz=(0.033, 0.0, 0.118)),
        material=machine_blue,
        name="front_motor_bulge",
    )
    housing.visual(
        Cylinder(radius=0.080, length=0.032),
        origin=Origin(xyz=(0.0, 0.108, 0.128), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machine_blue,
        name="left_endbell",
    )
    housing.visual(
        Cylinder(radius=0.080, length=0.032),
        origin=Origin(xyz=(0.0, -0.108, 0.128), rpy=(pi / 2.0, 0.0, 0.0)),
        material=machine_blue,
        name="right_endbell",
    )
    housing.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, 0.132, 0.128), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_spindle_stub",
    )
    housing.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, -0.132, 0.128), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_spindle_stub",
    )
    housing.visual(
        Box((0.078, 0.022, 0.024)),
        origin=Origin(xyz=(0.029, 0.126, 0.060)),
        material=steel,
        name="left_rest_bracket",
    )
    housing.visual(
        Box((0.078, 0.022, 0.024)),
        origin=Origin(xyz=(0.029, -0.126, 0.060)),
        material=steel,
        name="right_rest_bracket",
    )
    housing.visual(
        Box((0.018, 0.018, 0.076)),
        origin=Origin(xyz=(0.010, 0.126, 0.173)),
        material=steel,
        name="left_shield_post",
    )
    housing.visual(
        Box((0.018, 0.018, 0.076)),
        origin=Origin(xyz=(0.010, -0.126, 0.173)),
        material=steel,
        name="right_shield_post",
    )
    housing.visual(
        Box((0.076, 0.018, 0.018)),
        origin=Origin(xyz=(0.047, 0.126, 0.205)),
        material=steel,
        name="left_shield_bracket",
    )
    housing.visual(
        Box((0.076, 0.018, 0.018)),
        origin=Origin(xyz=(0.047, -0.126, 0.205)),
        material=steel,
        name="right_shield_bracket",
    )
    housing.visual(
        Box((0.010, 0.050, 0.030)),
        origin=Origin(xyz=(0.052, 0.0, 0.056)),
        material=rubber,
        name="switch_bezel",
    )

    left_stone = model.part("left_stone")
    left_stone.visual(
        Cylinder(radius=0.088, length=0.028),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=stone_gray,
        name="stone",
    )
    left_stone.visual(
        Cylinder(radius=0.031, length=0.015),
        origin=Origin(xyz=(0.0, -0.0215, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="inner_flange",
    )
    left_stone.visual(
        Cylinder(radius=0.026, length=0.009),
        origin=Origin(xyz=(0.0, 0.0185, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="outer_flange",
    )

    right_stone = model.part("right_stone")
    right_stone.visual(
        Cylinder(radius=0.088, length=0.028),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=stone_gray,
        name="stone",
    )
    right_stone.visual(
        Cylinder(radius=0.031, length=0.015),
        origin=Origin(xyz=(0.0, 0.0215, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="inner_flange",
    )
    right_stone.visual(
        Cylinder(radius=0.026, length=0.009),
        origin=Origin(xyz=(0.0, -0.0185, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="outer_flange",
    )

    left_rest = model.part("left_rest")
    left_rest.visual(
        Cylinder(radius=0.007, length=0.048),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    left_rest.visual(
        Box((0.044, 0.036, 0.014)),
        origin=Origin(xyz=(0.024, 0.016, -0.008)),
        material=steel,
        name="arm",
    )
    left_rest.visual(
        Box((0.082, 0.068, 0.007)),
        origin=Origin(xyz=(0.059, 0.030, -0.014)),
        material=cast_iron,
        name="tray",
    )

    right_rest = model.part("right_rest")
    right_rest.visual(
        Cylinder(radius=0.007, length=0.048),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    right_rest.visual(
        Box((0.044, 0.036, 0.014)),
        origin=Origin(xyz=(0.024, -0.016, -0.008)),
        material=steel,
        name="arm",
    )
    right_rest.visual(
        Box((0.082, 0.068, 0.007)),
        origin=Origin(xyz=(0.059, -0.030, -0.014)),
        material=cast_iron,
        name="tray",
    )

    left_shield = model.part("left_shield")
    left_shield.visual(
        Box((0.022, 0.116, 0.006)),
        origin=Origin(xyz=(0.009, 0.0, -0.003)),
        material=steel,
        name="top_clamp",
    )
    left_shield.visual(
        Box((0.004, 0.112, 0.082)),
        origin=Origin(xyz=(0.011, 0.0, -0.046)),
        material=shield_clear,
        name="screen",
    )

    right_shield = model.part("right_shield")
    right_shield.visual(
        Box((0.022, 0.116, 0.006)),
        origin=Origin(xyz=(0.009, 0.0, -0.003)),
        material=steel,
        name="top_clamp",
    )
    right_shield.visual(
        Box((0.004, 0.112, 0.082)),
        origin=Origin(xyz=(0.011, 0.0, -0.046)),
        material=shield_clear,
        name="screen",
    )

    rocker_switch = model.part("rocker_switch")
    rocker_switch.visual(
        Box((0.024, 0.034, 0.014)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=switch_black,
        name="body",
    )
    rocker_switch.visual(
        Box((0.010, 0.034, 0.005)),
        origin=Origin(xyz=(0.015, 0.0, 0.006)),
        material=switch_black,
        name="crown",
    )

    model.articulation(
        "housing_to_left_stone",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_stone,
        origin=Origin(xyz=(0.0, 0.174, 0.128)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )
    model.articulation(
        "housing_to_right_stone",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_stone,
        origin=Origin(xyz=(0.0, -0.174, 0.128)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=20.0),
    )
    model.articulation(
        "housing_to_left_rest",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=left_rest,
        origin=Origin(xyz=(0.075, 0.126, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.45,
        ),
    )
    model.articulation(
        "housing_to_right_rest",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=right_rest,
        origin=Origin(xyz=(0.075, -0.126, 0.060)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.45,
        ),
    )
    model.articulation(
        "housing_to_left_shield",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=left_shield,
        origin=Origin(xyz=(0.082, 0.126, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "housing_to_right_shield",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=right_shield,
        origin=Origin(xyz=(0.082, -0.126, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "housing_to_rocker_switch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=rocker_switch,
        origin=Origin(xyz=(0.061, 0.0, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-0.35,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    left_stone = object_model.get_part("left_stone")
    right_stone = object_model.get_part("right_stone")
    left_rest = object_model.get_part("left_rest")
    right_rest = object_model.get_part("right_rest")
    left_shield = object_model.get_part("left_shield")
    right_shield = object_model.get_part("right_shield")
    rocker_switch = object_model.get_part("rocker_switch")

    left_rest_joint = object_model.get_articulation("housing_to_left_rest")
    right_rest_joint = object_model.get_articulation("housing_to_right_rest")
    left_shield_joint = object_model.get_articulation("housing_to_left_shield")
    right_shield_joint = object_model.get_articulation("housing_to_right_shield")
    switch_joint = object_model.get_articulation("housing_to_rocker_switch")

    ctx.expect_origin_gap(
        left_stone,
        housing,
        axis="y",
        min_gap=0.15,
        name="left stone sits outboard of housing",
    )
    ctx.expect_origin_gap(
        housing,
        right_stone,
        axis="y",
        min_gap=0.15,
        name="right stone sits outboard of housing",
    )

    ctx.expect_gap(
        left_rest,
        left_stone,
        axis="x",
        positive_elem="tray",
        negative_elem="stone",
        min_gap=0.0,
        max_gap=0.02,
        name="left rest sits just ahead of the stone",
    )
    ctx.expect_gap(
        right_rest,
        right_stone,
        axis="x",
        positive_elem="tray",
        negative_elem="stone",
        min_gap=0.0,
        max_gap=0.02,
        name="right rest sits just ahead of the stone",
    )
    ctx.expect_overlap(
        left_rest,
        left_stone,
        axes="y",
        elem_a="tray",
        elem_b="stone",
        min_overlap=0.02,
        name="left rest spans the wheel working zone",
    )
    ctx.expect_overlap(
        right_rest,
        right_stone,
        axes="y",
        elem_a="tray",
        elem_b="stone",
        min_overlap=0.02,
        name="right rest spans the wheel working zone",
    )

    ctx.expect_gap(
        left_shield,
        left_stone,
        axis="x",
        positive_elem="screen",
        negative_elem="stone",
        min_gap=0.001,
        max_gap=0.010,
        name="left shield hangs just ahead of the wheel",
    )
    ctx.expect_gap(
        right_shield,
        right_stone,
        axis="x",
        positive_elem="screen",
        negative_elem="stone",
        min_gap=0.001,
        max_gap=0.010,
        name="right shield hangs just ahead of the wheel",
    )
    ctx.expect_overlap(
        left_shield,
        left_stone,
        axes="yz",
        elem_a="screen",
        elem_b="stone",
        min_overlap=0.02,
        name="left shield covers the upper wheel face",
    )
    ctx.expect_overlap(
        right_shield,
        right_stone,
        axes="yz",
        elem_a="screen",
        elem_b="stone",
        min_overlap=0.02,
        name="right shield covers the upper wheel face",
    )

    with ctx.pose({left_rest_joint: 0.0}):
        left_rest_closed = ctx.part_world_aabb(left_rest)
    with ctx.pose({left_rest_joint: 0.35}):
        left_rest_open = ctx.part_world_aabb(left_rest)
    ctx.check(
        "left rest pitches upward",
        left_rest_closed is not None
        and left_rest_open is not None
        and left_rest_open[1][2] > left_rest_closed[1][2] + 0.015,
        details=f"closed={left_rest_closed}, open={left_rest_open}",
    )

    with ctx.pose({right_rest_joint: 0.0}):
        right_rest_closed = ctx.part_world_aabb(right_rest)
    with ctx.pose({right_rest_joint: 0.35}):
        right_rest_open = ctx.part_world_aabb(right_rest)
    ctx.check(
        "right rest pitches upward",
        right_rest_closed is not None
        and right_rest_open is not None
        and right_rest_open[1][2] > right_rest_closed[1][2] + 0.015,
        details=f"closed={right_rest_closed}, open={right_rest_open}",
    )

    with ctx.pose({left_shield_joint: 0.0}):
        left_shield_closed = ctx.part_world_aabb(left_shield)
    with ctx.pose({left_shield_joint: 1.0}):
        left_shield_open = ctx.part_world_aabb(left_shield)
    ctx.check(
        "left shield flips upward",
        left_shield_closed is not None
        and left_shield_open is not None
        and left_shield_open[0][2] > left_shield_closed[0][2] + 0.035,
        details=f"closed={left_shield_closed}, open={left_shield_open}",
    )

    with ctx.pose({right_shield_joint: 0.0}):
        right_shield_closed = ctx.part_world_aabb(right_shield)
    with ctx.pose({right_shield_joint: 1.0}):
        right_shield_open = ctx.part_world_aabb(right_shield)
    ctx.check(
        "right shield flips upward",
        right_shield_closed is not None
        and right_shield_open is not None
        and right_shield_open[0][2] > right_shield_closed[0][2] + 0.035,
        details=f"closed={right_shield_closed}, open={right_shield_open}",
    )

    with ctx.pose({switch_joint: -0.30}):
        switch_down = ctx.part_world_aabb(rocker_switch)
    with ctx.pose({switch_joint: 0.30}):
        switch_up = ctx.part_world_aabb(rocker_switch)
    ctx.check(
        "rocker switch swings through its pivot",
        switch_down is not None
        and switch_up is not None
        and abs(switch_up[1][2] - switch_down[1][2]) > 0.004,
        details=f"down={switch_down}, up={switch_up}",
    )

    return ctx.report()


object_model = build_object_model()
