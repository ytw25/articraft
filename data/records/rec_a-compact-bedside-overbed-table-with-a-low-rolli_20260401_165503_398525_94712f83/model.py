from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_LENGTH = 0.78
BASE_RAIL_OFFSET_Y = 0.155
BASE_RAIL_WIDTH = 0.050
BASE_RAIL_HEIGHT = 0.016
BASE_RAIL_CENTER_Z = 0.030
BASE_REAR_CROSS_X = -0.235
BASE_REAR_CROSS_LENGTH = 0.120
BASE_FRONT_X = 0.305
BASE_COLUMN_X = -0.080

BASE_BOTTOM_Z = BASE_RAIL_CENTER_Z - (BASE_RAIL_HEIGHT / 2.0)
BASE_TOP_Z = BASE_RAIL_CENTER_Z + (BASE_RAIL_HEIGHT / 2.0)

COLUMN_PLATE_X = BASE_COLUMN_X
COLUMN_PLATE_Y = BASE_RAIL_OFFSET_Y
COLUMN_PLATE_SIZE = (0.095, 0.060, 0.008)
COLUMN_PLATE_CENTER_Z = BASE_TOP_Z + (COLUMN_PLATE_SIZE[2] / 2.0)
COLUMN_PLATE_TOP_Z = COLUMN_PLATE_CENTER_Z + (COLUMN_PLATE_SIZE[2] / 2.0)

OUTER_SHOE_SIZE = (0.090, 0.058, 0.008)
OUTER_COLUMN_W = 0.062
OUTER_COLUMN_D = 0.040
OUTER_COLUMN_H = 0.390
OUTER_WALL = 0.004

INNER_COLUMN_W = OUTER_COLUMN_W - (2.0 * OUTER_WALL)
INNER_COLUMN_D = OUTER_COLUMN_D - (2.0 * OUTER_WALL)
INNER_COLUMN_H = 0.540
INNER_INSERTION = 0.270
INNER_SLIDE_MAX = 0.200

HINGE_X = 0.450
HINGE_Z = 0.278
HINGE_SUPPORT_Y = 0.018
TOP_TILT_MAX = 1.08

TOP_LENGTH = 0.720
TOP_WIDTH = 0.320
TOP_THICKNESS = 0.016
TOP_BARREL_RADIUS = 0.012
TOP_BARREL_LENGTH = 0.070
TOP_PANEL_CENTER = (
    ((TOP_LENGTH / 2.0) - 0.050),
    -(TOP_WIDTH / 2.0),
    -(TOP_BARREL_RADIUS + (TOP_THICKNESS / 2.0)),
)
TOP_LIP_SIZE = (0.660, 0.010, 0.018)

CASTER_COLLAR_RADIUS = 0.010
CASTER_COLLAR_LENGTH = 0.006
CASTER_STEM_RADIUS = 0.005
CASTER_STEM_LENGTH = 0.018
CASTER_CROWN_SIZE = (0.024, 0.014, 0.006)
CASTER_ARM_SIZE = (0.003, 0.012, 0.016)
CASTER_ARM_OFFSET_X = 0.0085
CASTER_WHEEL_RADIUS = 0.016
CASTER_WHEEL_WIDTH = 0.010
CASTER_WHEEL_CENTER_Z = -0.031
CASTER_AXLE_RADIUS = 0.0025
CASTER_AXLE_LENGTH = 0.018
REAR_WHEEL_RADIUS = 0.016
REAR_WHEEL_WIDTH = 0.012
REAR_WHEEL_X = -0.285
REAR_WHEEL_CENTER_Z = 0.018


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bedside_overbed_table")

    model.material("frame_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("tube_silver", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("laminate", rgba=(0.82, 0.77, 0.68, 1.0))
    model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("dark_trim", rgba=(0.18, 0.19, 0.21, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_RAIL_WIDTH, BASE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, BASE_RAIL_OFFSET_Y, BASE_RAIL_CENTER_Z)),
        material="frame_gray",
        name="outer_rail",
    )
    base.visual(
        Box((BASE_LENGTH, BASE_RAIL_WIDTH, BASE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -BASE_RAIL_OFFSET_Y, BASE_RAIL_CENTER_Z)),
        material="frame_gray",
        name="inner_rail",
    )
    base.visual(
        Box((BASE_REAR_CROSS_LENGTH, (2.0 * BASE_RAIL_OFFSET_Y) + BASE_RAIL_WIDTH, BASE_RAIL_HEIGHT)),
        origin=Origin(xyz=(BASE_REAR_CROSS_X, 0.0, BASE_RAIL_CENTER_Z)),
        material="frame_gray",
        name="rear_crossbar",
    )
    base.visual(
        Box(COLUMN_PLATE_SIZE),
        origin=Origin(xyz=(COLUMN_PLATE_X, COLUMN_PLATE_Y, COLUMN_PLATE_CENTER_Z)),
        material="frame_gray",
        name="column_plate",
    )
    base.visual(
        Box((0.016, 0.014, 0.014)),
        origin=Origin(xyz=(REAR_WHEEL_X, BASE_RAIL_OFFSET_Y, 0.029)),
        material="frame_gray",
        name="rear_outer_bracket",
    )
    base.visual(
        Box((0.016, 0.014, 0.014)),
        origin=Origin(xyz=(REAR_WHEEL_X, -BASE_RAIL_OFFSET_Y, 0.029)),
        material="frame_gray",
        name="rear_inner_bracket",
    )
    base.visual(
        Cylinder(radius=REAR_WHEEL_RADIUS, length=REAR_WHEEL_WIDTH),
        origin=Origin(xyz=(REAR_WHEEL_X, BASE_RAIL_OFFSET_Y, REAR_WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="rubber_black",
        name="rear_outer_wheel",
    )
    base.visual(
        Cylinder(radius=REAR_WHEEL_RADIUS, length=REAR_WHEEL_WIDTH),
        origin=Origin(xyz=(REAR_WHEEL_X, -BASE_RAIL_OFFSET_Y, REAR_WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="rubber_black",
        name="rear_inner_wheel",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, (2.0 * BASE_RAIL_OFFSET_Y) + BASE_RAIL_WIDTH, 0.060)),
        mass=9.8,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    outer_column = model.part("outer_column")
    outer_column.visual(
        Box(OUTER_SHOE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, OUTER_SHOE_SIZE[2] / 2.0)),
        material="frame_gray",
        name="shoe",
    )
    wall_center_z = OUTER_SHOE_SIZE[2] + (OUTER_COLUMN_H / 2.0)
    outer_column.visual(
        Box((OUTER_WALL, OUTER_COLUMN_D, OUTER_COLUMN_H)),
        origin=Origin(xyz=(-((OUTER_COLUMN_W - OUTER_WALL) / 2.0), 0.0, wall_center_z)),
        material="tube_silver",
        name="left_wall",
    )
    outer_column.visual(
        Box((OUTER_WALL, OUTER_COLUMN_D, OUTER_COLUMN_H)),
        origin=Origin(xyz=(((OUTER_COLUMN_W - OUTER_WALL) / 2.0), 0.0, wall_center_z)),
        material="tube_silver",
        name="right_wall",
    )
    outer_column.visual(
        Box((OUTER_COLUMN_W - (2.0 * OUTER_WALL), OUTER_WALL, OUTER_COLUMN_H)),
        origin=Origin(xyz=(0.0, -((OUTER_COLUMN_D - OUTER_WALL) / 2.0), wall_center_z)),
        material="tube_silver",
        name="rear_wall",
    )
    outer_column.visual(
        Box((OUTER_COLUMN_W - (2.0 * OUTER_WALL), OUTER_WALL, OUTER_COLUMN_H)),
        origin=Origin(xyz=(0.0, ((OUTER_COLUMN_D - OUTER_WALL) / 2.0), wall_center_z)),
        material="tube_silver",
        name="front_wall",
    )
    outer_column.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, -(OUTER_COLUMN_D / 2.0) - 0.008, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_trim",
        name="height_lock_knob",
    )
    outer_column.inertial = Inertial.from_geometry(
        Box((OUTER_SHOE_SIZE[0], OUTER_SHOE_SIZE[1], OUTER_SHOE_SIZE[2] + OUTER_COLUMN_H)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, (OUTER_SHOE_SIZE[2] + OUTER_COLUMN_H) / 2.0)),
    )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Box((INNER_COLUMN_W, INNER_COLUMN_D, INNER_COLUMN_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="tube_silver",
        name="mast",
    )
    inner_column.visual(
        Box((0.060, 0.032, 0.070)),
        origin=Origin(xyz=(0.0, 0.026, 0.240)),
        material="frame_gray",
        name="head_block",
    )
    inner_column.visual(
        Box((HINGE_X, 0.012, 0.012)),
        origin=Origin(xyz=(HINGE_X / 2.0, HINGE_SUPPORT_Y, HINGE_Z)),
        material="frame_gray",
        name="support_arm",
    )
    inner_column.visual(
        Box((0.018, 0.012, 0.060)),
        origin=Origin(xyz=(HINGE_X, HINGE_SUPPORT_Y, HINGE_Z)),
        material="frame_gray",
        name="hinge_cheek",
    )
    inner_column.visual(
        Box((0.220, 0.012, 0.024)),
        origin=Origin(xyz=(0.110, 0.022, 0.266)),
        material="frame_gray",
        name="brace_block",
    )
    inner_column.inertial = Inertial.from_geometry(
        Box((0.330, 0.050, INNER_COLUMN_H)),
        mass=2.5,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
    )

    top = model.part("top")
    top.visual(
        Box((TOP_LENGTH, TOP_WIDTH, TOP_THICKNESS)),
        origin=Origin(xyz=TOP_PANEL_CENTER),
        material="laminate",
        name="panel_body",
    )
    top.visual(
        Cylinder(radius=TOP_BARREL_RADIUS, length=TOP_BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_trim",
        name="hinge_barrel",
    )
    top.visual(
        Box((0.100, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, -0.017)),
        material="dark_trim",
        name="hinge_leaf",
    )
    panel_top_z = TOP_PANEL_CENTER[2] + (TOP_THICKNESS / 2.0)
    top.visual(
        Box(TOP_LIP_SIZE),
        origin=Origin(
            xyz=(
                TOP_PANEL_CENTER[0],
                -(TOP_WIDTH - (TOP_LIP_SIZE[1] / 2.0)),
                panel_top_z + (TOP_LIP_SIZE[2] / 2.0),
            )
        ),
        material="laminate",
        name="retaining_lip",
    )
    top.inertial = Inertial.from_geometry(
        Box((TOP_LENGTH, TOP_WIDTH, 0.050)),
        mass=4.0,
        origin=Origin(xyz=(TOP_PANEL_CENTER[0], TOP_PANEL_CENTER[1], TOP_PANEL_CENTER[2])),
    )

    front_outer_caster = model.part("front_outer_caster")
    front_outer_caster.visual(
        Cylinder(radius=CASTER_COLLAR_RADIUS, length=CASTER_COLLAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -(CASTER_COLLAR_LENGTH / 2.0))),
        material="tube_silver",
        name="swivel_collar",
    )
    front_outer_caster.visual(
        Cylinder(radius=CASTER_STEM_RADIUS, length=CASTER_STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -(CASTER_STEM_LENGTH / 2.0))),
        material="tube_silver",
        name="stem",
    )
    front_outer_caster.visual(
        Box(CASTER_CROWN_SIZE),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material="tube_silver",
        name="crown",
    )
    front_outer_caster.visual(
        Box(CASTER_ARM_SIZE),
        origin=Origin(xyz=(-CASTER_ARM_OFFSET_X, 0.0, CASTER_WHEEL_CENTER_Z)),
        material="tube_silver",
        name="left_fork_arm",
    )
    front_outer_caster.visual(
        Box(CASTER_ARM_SIZE),
        origin=Origin(xyz=(CASTER_ARM_OFFSET_X, 0.0, CASTER_WHEEL_CENTER_Z)),
        material="tube_silver",
        name="right_fork_arm",
    )
    front_outer_caster.visual(
        Cylinder(radius=CASTER_AXLE_RADIUS, length=CASTER_AXLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, CASTER_WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_trim",
        name="axle_pin",
    )

    front_inner_caster = model.part("front_inner_caster")
    front_inner_caster.visual(
        Cylinder(radius=CASTER_COLLAR_RADIUS, length=CASTER_COLLAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -(CASTER_COLLAR_LENGTH / 2.0))),
        material="tube_silver",
        name="swivel_collar",
    )
    front_inner_caster.visual(
        Cylinder(radius=CASTER_STEM_RADIUS, length=CASTER_STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -(CASTER_STEM_LENGTH / 2.0))),
        material="tube_silver",
        name="stem",
    )
    front_inner_caster.visual(
        Box(CASTER_CROWN_SIZE),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material="tube_silver",
        name="crown",
    )
    front_inner_caster.visual(
        Box(CASTER_ARM_SIZE),
        origin=Origin(xyz=(-CASTER_ARM_OFFSET_X, 0.0, CASTER_WHEEL_CENTER_Z)),
        material="tube_silver",
        name="left_fork_arm",
    )
    front_inner_caster.visual(
        Box(CASTER_ARM_SIZE),
        origin=Origin(xyz=(CASTER_ARM_OFFSET_X, 0.0, CASTER_WHEEL_CENTER_Z)),
        material="tube_silver",
        name="right_fork_arm",
    )
    front_inner_caster.visual(
        Cylinder(radius=CASTER_AXLE_RADIUS, length=CASTER_AXLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, CASTER_WHEEL_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_trim",
        name="axle_pin",
    )

    front_outer_wheel = model.part("front_outer_wheel")
    front_outer_wheel.visual(
        Cylinder(radius=CASTER_WHEEL_RADIUS, length=CASTER_WHEEL_WIDTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="rubber_black",
        name="wheel_tire",
    )
    front_outer_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=CASTER_WHEEL_RADIUS, length=CASTER_WHEEL_WIDTH),
        mass=0.10,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    front_inner_wheel = model.part("front_inner_wheel")
    front_inner_wheel.visual(
        Cylinder(radius=CASTER_WHEEL_RADIUS, length=CASTER_WHEEL_WIDTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="rubber_black",
        name="wheel_tire",
    )
    front_inner_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=CASTER_WHEEL_RADIUS, length=CASTER_WHEEL_WIDTH),
        mass=0.10,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "base_to_outer_column",
        ArticulationType.FIXED,
        parent=base,
        child=outer_column,
        origin=Origin(xyz=(COLUMN_PLATE_X, COLUMN_PLATE_Y, COLUMN_PLATE_TOP_Z)),
    )
    model.articulation(
        "outer_to_inner_column",
        ArticulationType.PRISMATIC,
        parent=outer_column,
        child=inner_column,
        origin=Origin(xyz=(0.0, 0.0, OUTER_SHOE_SIZE[2] + OUTER_COLUMN_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=INNER_SLIDE_MAX, effort=80.0, velocity=0.18),
    )
    model.articulation(
        "inner_column_to_top",
        ArticulationType.REVOLUTE,
        parent=inner_column,
        child=top,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TOP_TILT_MAX, effort=10.0, velocity=1.2),
    )
    model.articulation(
        "base_to_front_outer_caster",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=front_outer_caster,
        origin=Origin(xyz=(BASE_FRONT_X, BASE_RAIL_OFFSET_Y, BASE_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "base_to_front_inner_caster",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=front_inner_caster,
        origin=Origin(xyz=(BASE_FRONT_X, -BASE_RAIL_OFFSET_Y, BASE_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )
    model.articulation(
        "front_outer_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_outer_caster,
        child=front_outer_wheel,
        origin=Origin(xyz=(0.0, 0.0, CASTER_WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )
    model.articulation(
        "front_inner_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_inner_caster,
        child=front_inner_wheel,
        origin=Origin(xyz=(0.0, 0.0, CASTER_WHEEL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer_column = object_model.get_part("outer_column")
    inner_column = object_model.get_part("inner_column")
    top = object_model.get_part("top")
    front_outer_caster = object_model.get_part("front_outer_caster")
    front_inner_caster = object_model.get_part("front_inner_caster")
    front_outer_wheel = object_model.get_part("front_outer_wheel")
    front_inner_wheel = object_model.get_part("front_inner_wheel")

    column_slide = object_model.get_articulation("outer_to_inner_column")
    top_tilt = object_model.get_articulation("inner_column_to_top")
    outer_swivel = object_model.get_articulation("base_to_front_outer_caster")
    inner_swivel = object_model.get_articulation("base_to_front_inner_caster")
    outer_roll = object_model.get_articulation("front_outer_caster_to_wheel")
    inner_roll = object_model.get_articulation("front_inner_caster_to_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        front_outer_caster,
        front_outer_wheel,
        reason="The caster axle pin intentionally passes through the simplified solid wheel hub.",
    )
    ctx.allow_overlap(
        front_inner_caster,
        front_inner_wheel,
        reason="The caster axle pin intentionally passes through the simplified solid wheel hub.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "column slide is vertical prismatic travel",
        column_slide.axis == (0.0, 0.0, 1.0)
        and column_slide.motion_limits is not None
        and column_slide.motion_limits.lower == 0.0
        and column_slide.motion_limits.upper is not None
        and column_slide.motion_limits.upper >= 0.18,
        details=f"axis={column_slide.axis}, limits={column_slide.motion_limits}",
    )
    ctx.check(
        "top tilts upward about a side hinge",
        top_tilt.axis == (-1.0, 0.0, 0.0)
        and top_tilt.motion_limits is not None
        and top_tilt.motion_limits.lower == 0.0
        and top_tilt.motion_limits.upper is not None
        and top_tilt.motion_limits.upper >= 1.0,
        details=f"axis={top_tilt.axis}, limits={top_tilt.motion_limits}",
    )
    ctx.check(
        "caster stems swivel vertically and wheels roll horizontally",
        outer_swivel.axis == (0.0, 0.0, 1.0)
        and inner_swivel.axis == (0.0, 0.0, 1.0)
        and outer_roll.axis == (1.0, 0.0, 0.0)
        and inner_roll.axis == (1.0, 0.0, 0.0),
        details=(
            f"outer_swivel={outer_swivel.axis}, inner_swivel={inner_swivel.axis}, "
            f"outer_roll={outer_roll.axis}, inner_roll={inner_roll.axis}"
        ),
    )

    ctx.expect_contact(
        outer_column,
        base,
        elem_a="shoe",
        elem_b="column_plate",
        name="outer column shoe seats on the column plate",
    )
    ctx.expect_within(
        inner_column,
        outer_column,
        axes="xy",
        inner_elem="mast",
        margin=0.0,
        name="inner mast stays nested in the outer sleeve",
    )
    ctx.expect_overlap(
        inner_column,
        outer_column,
        axes="z",
        elem_a="mast",
        min_overlap=0.12,
        name="inner mast keeps insertion at rest",
    )
    ctx.expect_origin_gap(
        front_outer_caster,
        front_outer_wheel,
        axis="z",
        min_gap=0.020,
        name="outer front wheel hangs below the caster stem",
    )
    ctx.expect_origin_gap(
        front_inner_caster,
        front_inner_wheel,
        axis="z",
        min_gap=0.020,
        name="inner front wheel hangs below the caster stem",
    )

    rest_inner_pos = ctx.part_world_position(inner_column)
    lip_rest = ctx.part_element_world_aabb(top, elem="retaining_lip")

    with ctx.pose({column_slide: INNER_SLIDE_MAX}):
        extended_inner_pos = ctx.part_world_position(inner_column)
        ctx.expect_within(
            inner_column,
            outer_column,
            axes="xy",
            inner_elem="mast",
            margin=0.0,
            name="inner mast remains guided at full height",
        )
        ctx.expect_overlap(
            inner_column,
            outer_column,
            axes="z",
            elem_a="mast",
            min_overlap=0.06,
            name="inner mast retains engagement at full height",
        )
        ctx.check(
            "height adjustment raises the top support",
            rest_inner_pos is not None
            and extended_inner_pos is not None
            and extended_inner_pos[2] > rest_inner_pos[2] + 0.18,
            details=f"rest={rest_inner_pos}, extended={extended_inner_pos}",
        )

    with ctx.pose({top_tilt: TOP_TILT_MAX}):
        lip_open = ctx.part_element_world_aabb(top, elem="retaining_lip")
        ctx.check(
            "retaining lip rises with the tilted reading top",
            lip_rest is not None and lip_open is not None and lip_open[0][2] > lip_rest[0][2] + 0.10,
            details=f"rest={lip_rest}, open={lip_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
