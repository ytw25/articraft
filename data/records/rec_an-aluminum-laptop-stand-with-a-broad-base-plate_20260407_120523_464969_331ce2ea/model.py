from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.330
BASE_WIDTH = 0.230
BASE_THICKNESS = 0.008
BASE_BRACKET_THICKNESS = 0.008
BASE_BRACKET_HEIGHT = 0.026
BASE_BRACKET_LENGTH = 0.210

LOWER_FRONT_X = -0.120
LOWER_REAR_X = 0.030
LEFT_PIVOT_Y = -(BASE_WIDTH / 2.0 + BASE_BRACKET_THICKNESS)
RIGHT_PIVOT_Y = -LEFT_PIVOT_Y
LOWER_PIVOT_Z = BASE_THICKNESS + 0.016

BAR_DX = 0.100
BAR_DZ = 0.125
BAR_THICKNESS = 0.006
BAR_CAP_RADIUS = 0.011
BAR_NECK_WIDTH = 0.015
BAR_CENTER_DISTANCE = math.hypot(BAR_DX, BAR_DZ)
BAR_ANGLE_DEG = math.degrees(math.atan2(BAR_DZ, BAR_DX))

PIVOT_SPAN_X = LOWER_REAR_X - LOWER_FRONT_X
PLATFORM_LENGTH = 0.285
PLATFORM_TOP_WIDTH = 0.220
PLATFORM_OUTER_WIDTH = RIGHT_PIVOT_Y - LEFT_PIVOT_Y
PLATFORM_TOP_THICKNESS = 0.008
PLATFORM_TOP_CENTER_Z = 0.020
PLATFORM_FLANGE_HEIGHT = 0.030
PLATFORM_FLANGE_CENTER_Z = 0.005
PLATFORM_FLANGE_THICKNESS = (PLATFORM_OUTER_WIDTH - PLATFORM_TOP_WIDTH) / 2.0
PLATFORM_FRONT_EDGE_X = -0.035
PLATFORM_CENTER_X = PLATFORM_FRONT_EDGE_X + PLATFORM_LENGTH / 2.0
PLATFORM_TAB_HEIGHT = 0.012
PLATFORM_TAB_LENGTH = 0.028
PLATFORM_TAB_WIDTH = 0.016
PLATFORM_TAB_CENTER_Z = PLATFORM_TOP_CENTER_Z + PLATFORM_TOP_THICKNESS / 2.0 + PLATFORM_TAB_HEIGHT / 2.0 - 0.002
COOLING_CUTOUT_LENGTH = 0.150
COOLING_CUTOUT_WIDTH = 0.092


def _rounded_box(length: float, width: float, height: float, fillet_radius: float) -> cq.Shape:
    return (
        cq.Workplane("XY")
        .sketch()
        .rect(length, width)
        .vertices()
        .fillet(fillet_radius)
        .finalize()
        .extrude(height)
        .val()
    )


def _combine_shapes(*shapes: cq.Shape) -> cq.Shape:
    wp = cq.Workplane(obj=shapes[0])
    for shape in shapes[1:]:
        wp = wp.add(shape)
    return wp.combine().val()


def _make_base_shape() -> cq.Shape:
    plate = _rounded_box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, 0.020).translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    bracket_center_x = (LOWER_FRONT_X + LOWER_REAR_X) / 2.0
    bracket_center_z = BASE_THICKNESS + BASE_BRACKET_HEIGHT / 2.0 - 0.002

    left_bracket = (
        cq.Workplane("XY")
        .box(BASE_BRACKET_LENGTH, BASE_BRACKET_THICKNESS, BASE_BRACKET_HEIGHT)
        .translate((bracket_center_x, LEFT_PIVOT_Y + BASE_BRACKET_THICKNESS / 2.0, bracket_center_z))
        .val()
    )
    right_bracket = (
        cq.Workplane("XY")
        .box(BASE_BRACKET_LENGTH, BASE_BRACKET_THICKNESS, BASE_BRACKET_HEIGHT)
        .translate((bracket_center_x, RIGHT_PIVOT_Y - BASE_BRACKET_THICKNESS / 2.0, bracket_center_z))
        .val()
    )

    return _combine_shapes(plate, left_bracket, right_bracket)


def _make_platform_shape() -> cq.Shape:
    top_plate = _rounded_box(PLATFORM_LENGTH, PLATFORM_TOP_WIDTH, PLATFORM_TOP_THICKNESS, 0.014).translate(
        (
            PLATFORM_CENTER_X,
            PLATFORM_OUTER_WIDTH / 2.0,
            PLATFORM_TOP_CENTER_Z,
        )
    )
    cutout = _rounded_box(COOLING_CUTOUT_LENGTH, COOLING_CUTOUT_WIDTH, 0.040, 0.012).translate(
        (PLATFORM_CENTER_X + 0.006, PLATFORM_OUTER_WIDTH / 2.0, PLATFORM_TOP_CENTER_Z)
    )
    top_plate = top_plate.cut(cutout)

    flange_length = 0.220
    flange_center_x = 0.090
    left_flange = (
        cq.Workplane("XY")
        .box(flange_length, PLATFORM_FLANGE_THICKNESS, PLATFORM_FLANGE_HEIGHT)
        .translate((flange_center_x, PLATFORM_FLANGE_THICKNESS / 2.0, PLATFORM_FLANGE_CENTER_Z))
        .val()
    )
    right_flange = (
        cq.Workplane("XY")
        .box(flange_length, PLATFORM_FLANGE_THICKNESS, PLATFORM_FLANGE_HEIGHT)
        .translate(
            (
                flange_center_x,
                PLATFORM_OUTER_WIDTH - PLATFORM_FLANGE_THICKNESS / 2.0,
                PLATFORM_FLANGE_CENTER_Z,
            )
        )
        .val()
    )

    tab_x = PLATFORM_FRONT_EDGE_X + 0.018
    left_tab = (
        cq.Workplane("XY")
        .box(PLATFORM_TAB_LENGTH, PLATFORM_TAB_WIDTH, PLATFORM_TAB_HEIGHT)
        .translate((tab_x, 0.060, PLATFORM_TAB_CENTER_Z))
        .val()
    )
    right_tab = (
        cq.Workplane("XY")
        .box(PLATFORM_TAB_LENGTH, PLATFORM_TAB_WIDTH, PLATFORM_TAB_HEIGHT)
        .translate(
            (
                tab_x,
                PLATFORM_OUTER_WIDTH - 0.058,
                PLATFORM_TAB_CENTER_Z,
            )
        )
        .val()
    )

    return _combine_shapes(top_plate, left_flange, right_flange, left_tab, right_tab)


def _make_bar_shape() -> cq.Shape:
    bridge = (
        cq.Workplane("XZ")
        .center(BAR_DX / 2.0, BAR_DZ / 2.0)
        .slot2D(BAR_CENTER_DISTANCE, BAR_NECK_WIDTH, angle=BAR_ANGLE_DEG)
        .extrude(BAR_THICKNESS, both=True)
        .val()
    )
    lower_cap = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(BAR_CAP_RADIUS)
        .extrude(BAR_THICKNESS, both=True)
        .val()
    )
    upper_cap = (
        cq.Workplane("XZ")
        .center(BAR_DX, BAR_DZ)
        .circle(BAR_CAP_RADIUS)
        .extrude(BAR_THICKNESS, both=True)
        .val()
    )
    return _combine_shapes(bridge, lower_cap, upper_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aluminum_laptop_stand")
    model.material("anodized_aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    model.material("soft_gray", rgba=(0.62, 0.65, 0.69, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shape(), "base_plate"), material="anodized_aluminum", name="base_body")
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, PLATFORM_OUTER_WIDTH, BASE_THICKNESS + BASE_BRACKET_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + BASE_BRACKET_HEIGHT) / 2.0)),
    )

    left_front_bar = model.part("left_front_bar")
    left_front_bar.visual(
        mesh_from_cadquery(_make_bar_shape(), "left_front_bar"),
        origin=Origin(xyz=(0.0, -BAR_THICKNESS / 2.0, 0.0)),
        material="soft_gray",
        name="left_front_link",
    )
    left_front_bar.inertial = Inertial.from_geometry(
        Box((BAR_DX + 2.0 * BAR_CAP_RADIUS, BAR_THICKNESS, BAR_DZ + 2.0 * BAR_CAP_RADIUS)),
        mass=0.10,
        origin=Origin(xyz=(BAR_DX / 2.0, -BAR_THICKNESS / 2.0, BAR_DZ / 2.0)),
    )

    left_rear_bar = model.part("left_rear_bar")
    left_rear_bar.visual(
        mesh_from_cadquery(_make_bar_shape(), "left_rear_bar"),
        origin=Origin(xyz=(0.0, -BAR_THICKNESS / 2.0, 0.0)),
        material="soft_gray",
        name="left_rear_link",
    )
    left_rear_bar.inertial = Inertial.from_geometry(
        Box((BAR_DX + 2.0 * BAR_CAP_RADIUS, BAR_THICKNESS, BAR_DZ + 2.0 * BAR_CAP_RADIUS)),
        mass=0.10,
        origin=Origin(xyz=(BAR_DX / 2.0, -BAR_THICKNESS / 2.0, BAR_DZ / 2.0)),
    )

    right_front_bar = model.part("right_front_bar")
    right_front_bar.visual(
        mesh_from_cadquery(_make_bar_shape(), "right_front_bar"),
        origin=Origin(xyz=(0.0, BAR_THICKNESS / 2.0, 0.0)),
        material="soft_gray",
        name="right_front_link",
    )
    right_front_bar.inertial = Inertial.from_geometry(
        Box((BAR_DX + 2.0 * BAR_CAP_RADIUS, BAR_THICKNESS, BAR_DZ + 2.0 * BAR_CAP_RADIUS)),
        mass=0.10,
        origin=Origin(xyz=(BAR_DX / 2.0, BAR_THICKNESS / 2.0, BAR_DZ / 2.0)),
    )

    right_rear_bar = model.part("right_rear_bar")
    right_rear_bar.visual(
        mesh_from_cadquery(_make_bar_shape(), "right_rear_bar"),
        origin=Origin(xyz=(0.0, BAR_THICKNESS / 2.0, 0.0)),
        material="soft_gray",
        name="right_rear_link",
    )
    right_rear_bar.inertial = Inertial.from_geometry(
        Box((BAR_DX + 2.0 * BAR_CAP_RADIUS, BAR_THICKNESS, BAR_DZ + 2.0 * BAR_CAP_RADIUS)),
        mass=0.10,
        origin=Origin(xyz=(BAR_DX / 2.0, BAR_THICKNESS / 2.0, BAR_DZ / 2.0)),
    )

    platform = model.part("platform")
    platform.visual(mesh_from_cadquery(_make_platform_shape(), "platform"), material="anodized_aluminum", name="platform_body")
    platform.inertial = Inertial.from_geometry(
        Box((PLATFORM_LENGTH, PLATFORM_OUTER_WIDTH, PLATFORM_FLANGE_HEIGHT + PLATFORM_TOP_THICKNESS + PLATFORM_TAB_HEIGHT)),
        mass=0.9,
        origin=Origin(
            xyz=(
                PLATFORM_CENTER_X,
                PLATFORM_OUTER_WIDTH / 2.0,
                (PLATFORM_FLANGE_HEIGHT + PLATFORM_TOP_THICKNESS + PLATFORM_TAB_HEIGHT) / 2.0 - 0.004,
            )
        ),
    )

    model.articulation(
        "base_to_left_front_bar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_front_bar,
        origin=Origin(xyz=(LOWER_FRONT_X, LEFT_PIVOT_Y, LOWER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.60, effort=25.0, velocity=1.2),
    )
    model.articulation(
        "base_to_left_rear_bar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_rear_bar,
        origin=Origin(xyz=(LOWER_REAR_X, LEFT_PIVOT_Y, LOWER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.60, effort=25.0, velocity=1.2),
    )
    model.articulation(
        "base_to_right_front_bar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_front_bar,
        origin=Origin(xyz=(LOWER_FRONT_X, RIGHT_PIVOT_Y, LOWER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.60, effort=25.0, velocity=1.2),
    )
    model.articulation(
        "base_to_right_rear_bar",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_rear_bar,
        origin=Origin(xyz=(LOWER_REAR_X, RIGHT_PIVOT_Y, LOWER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.60, effort=25.0, velocity=1.2),
    )
    model.articulation(
        "left_front_bar_to_platform",
        ArticulationType.REVOLUTE,
        parent=left_front_bar,
        child=platform,
        origin=Origin(xyz=(BAR_DX, 0.0, BAR_DZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.70, upper=0.10, effort=25.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    platform = object_model.get_part("platform")
    left_front_bar = object_model.get_part("left_front_bar")
    left_rear_bar = object_model.get_part("left_rear_bar")
    right_front_bar = object_model.get_part("right_front_bar")
    right_rear_bar = object_model.get_part("right_rear_bar")

    left_front_joint = object_model.get_articulation("base_to_left_front_bar")
    left_rear_joint = object_model.get_articulation("base_to_left_rear_bar")
    right_front_joint = object_model.get_articulation("base_to_right_front_bar")
    right_rear_joint = object_model.get_articulation("base_to_right_rear_bar")
    platform_joint = object_model.get_articulation("left_front_bar_to_platform")

    ctx.expect_gap(
        platform,
        base,
        axis="z",
        min_gap=0.10,
        name="platform clears the base plate",
    )

    for bar, name in (
        (left_front_bar, "left front"),
        (left_rear_bar, "left rear"),
        (right_front_bar, "right front"),
        (right_rear_bar, "right rear"),
    ):
        ctx.expect_contact(bar, base, name=f"{name} bar stays mounted to the base")
        ctx.expect_contact(bar, platform, name=f"{name} bar meets the platform mounts")

    rest_pos = ctx.part_world_position(platform)
    raised_pose = {
        left_front_joint: 0.52,
        left_rear_joint: 0.52,
        right_front_joint: 0.52,
        right_rear_joint: 0.52,
        platform_joint: -0.52,
    }
    with ctx.pose(raised_pose):
        raised_pos = ctx.part_world_position(platform)
        ctx.check(
            "platform raises when linkage opens",
            rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.020,
            details=f"rest={rest_pos}, raised={raised_pos}",
        )
        ctx.expect_gap(
            platform,
            base,
            axis="z",
            min_gap=0.12,
            name="raised platform stays above the base",
        )
        for bar, name in (
            (left_rear_bar, "left rear"),
            (right_front_bar, "right front"),
            (right_rear_bar, "right rear"),
        ):
            ctx.expect_contact(bar, platform, name=f"{name} bar remains engaged in the raised pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
