from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RAIL_TOP_Z = 0.111
BASE_RAIL_H = 0.028
BASE_RAIL_CENTER_Z = BASE_RAIL_TOP_Z - (BASE_RAIL_H / 2.0)
BASE_HALF_WIDTH = 0.190
BASE_LEG_LENGTH = 0.620
BASE_REAR_CROSSBAR_X = 0.0

WHEEL_RADIUS = 0.038
WHEEL_WIDTH = 0.022
AXLE_RADIUS = 0.0045
FORK_HEAD_Z = 0.083
WHEEL_CENTER_DROP = 0.050
CASTER_TRAIL = 0.032
FRONT_CASTER_X = 0.580
REAR_CASTER_X = -0.030

LOWER_MAST_BASE_Z = BASE_RAIL_TOP_Z - 0.002
LOWER_MAST_HEIGHT = 0.400
LOWER_MAST_OUTER_X = 0.085
LOWER_MAST_OUTER_Y = 0.055
LOWER_MAST_WALL = 0.012
LOWER_MAST_TOP_Z = LOWER_MAST_BASE_Z + LOWER_MAST_HEIGHT

UPPER_TUBE_X = 0.058
UPPER_TUBE_Y = 0.028
UPPER_TUBE_Z = 0.660
MAST_SLIDE_TRAVEL = 0.220

TABLE_LENGTH = 0.760
FIXED_TOP_WIDTH = 0.110
TILT_TOP_WIDTH = 0.310
TOP_THICKNESS = 0.018
HINGE_PIN_R = 0.0047
HINGE_KNUCKLE_R = 0.009
HINGE_SEGMENT_L = 0.120
HINGE_FIXED_X = (-0.240, 0.000, 0.240)
HINGE_TILT_X = (-0.120, 0.120)


def _make_lower_mast_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(LOWER_MAST_OUTER_X, LOWER_MAST_OUTER_Y, LOWER_MAST_HEIGHT)
        .translate((0.0, 0.0, LOWER_MAST_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.004)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            LOWER_MAST_OUTER_X - (2.0 * LOWER_MAST_WALL),
            LOWER_MAST_OUTER_Y - (2.0 * LOWER_MAST_WALL),
            LOWER_MAST_HEIGHT + 0.004,
        )
        .translate((0.0, 0.0, LOWER_MAST_HEIGHT / 2.0))
    )
    return outer.cut(inner)


def _make_rounded_board(length: float, width: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .edges("|Z")
        .fillet(0.012)
    )


def _make_hinge_ring(length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(outer_radius).extrude(length).translate((-length / 2.0, 0.0, 0.0))
    inner = (
        cq.Workplane("YZ")
        .circle(inner_radius)
        .extrude(length + 0.004)
        .translate((-(length + 0.004) / 2.0, 0.0, 0.0))
    )
    return outer.cut(inner)


def _make_caster_fork_shape() -> cq.Workplane:
    arm_y = 0.017
    head = cq.Workplane("XY").circle(0.018).extrude(0.008).translate((0.0, 0.0, -0.008))
    neck = cq.Workplane("XY").box(0.012, 0.014, 0.020).translate((0.0, 0.0, -0.018))
    brace_left = cq.Workplane("XY").box(CASTER_TRAIL, 0.006, 0.008).translate((CASTER_TRAIL / 2.0, arm_y, -0.018))
    brace_right = cq.Workplane("XY").box(CASTER_TRAIL, 0.006, 0.008).translate((CASTER_TRAIL / 2.0, -arm_y, -0.018))
    arm_left = cq.Workplane("XY").box(0.010, 0.006, 0.032).translate((CASTER_TRAIL, arm_y, -0.034))
    arm_right = cq.Workplane("XY").box(0.010, 0.006, 0.032).translate((CASTER_TRAIL, -arm_y, -0.034))
    return head.union(neck).union(brace_left).union(brace_right).union(arm_left).union(arm_right)


def _make_caster_wheel_shape() -> cq.Workplane:
    tread = cq.Workplane("XZ").circle(WHEEL_RADIUS).extrude(WHEEL_WIDTH / 2.0, both=True)
    hub = cq.Workplane("XZ").circle(0.014).extrude(0.028 / 2.0, both=True)
    return tread.union(hub)


def _axis_matches(axis: tuple[float, float, float], target: tuple[float, float, float]) -> bool:
    return tuple(round(v, 3) for v in axis) == tuple(round(v, 3) for v in target)


def _add_caster(
    model: ArticulatedObject,
    base_frame,
    *,
    label: str,
    x_pos: float,
    y_pos: float,
) -> None:
    fork = model.part(f"{label}_fork")
    fork.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material="caster_metal",
        name="swivel_head",
    )
    fork.visual(
        Box((CASTER_TRAIL, 0.006, 0.006)),
        origin=Origin(xyz=(CASTER_TRAIL / 2.0, 0.016, -0.007)),
        material="caster_metal",
        name="left_brace",
    )
    fork.visual(
        Box((CASTER_TRAIL, 0.006, 0.006)),
        origin=Origin(xyz=(CASTER_TRAIL / 2.0, -0.016, -0.007)),
        material="caster_metal",
        name="right_brace",
    )
    fork.visual(
        Box((0.010, 0.006, 0.036)),
        origin=Origin(xyz=(CASTER_TRAIL, 0.016, -0.028)),
        material="caster_metal",
        name="left_arm",
    )
    fork.visual(
        Box((0.010, 0.006, 0.036)),
        origin=Origin(xyz=(CASTER_TRAIL, -0.016, -0.028)),
        material="caster_metal",
        name="right_arm",
    )

    wheel = model.part(f"{label}_wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="wheel_black",
        name="wheel_body",
    )
    wheel.visual(
        Cylinder(radius=0.014, length=0.028),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="caster_metal",
        name="wheel_hub",
    )

    model.articulation(
        f"{label}_swivel",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=fork,
        origin=Origin(xyz=(x_pos, y_pos, FORK_HEAD_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        f"{label}_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(CASTER_TRAIL, 0.0, -WHEEL_CENTER_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_overbed_table")

    model.material("powder_charcoal", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("satin_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("laminate_fixed", rgba=(0.90, 0.88, 0.82, 1.0))
    model.material("laminate_tilt", rgba=(0.86, 0.84, 0.78, 1.0))
    model.material("caster_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("wheel_black", rgba=(0.11, 0.11, 0.12, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((0.080, 0.440, BASE_RAIL_H)),
        origin=Origin(xyz=(BASE_REAR_CROSSBAR_X, 0.0, BASE_RAIL_CENTER_Z)),
        material="powder_charcoal",
        name="rear_crossbar",
    )
    for y_pos, name in ((BASE_HALF_WIDTH, "left_leg"), (-BASE_HALF_WIDTH, "right_leg")):
        base_frame.visual(
            Box((BASE_LEG_LENGTH, 0.060, BASE_RAIL_H)),
            origin=Origin(xyz=(BASE_LEG_LENGTH / 2.0, y_pos, BASE_RAIL_CENTER_Z)),
            material="powder_charcoal",
            name=name,
        )
    base_frame.visual(
        mesh_from_cadquery(_make_lower_mast_shape(), "lower_mast_shell"),
        origin=Origin(xyz=(0.0, 0.0, LOWER_MAST_BASE_Z)),
        material="powder_charcoal",
        name="lower_mast",
    )

    caster_points = {
        "rear_left": (REAR_CASTER_X, BASE_HALF_WIDTH),
        "rear_right": (REAR_CASTER_X, -BASE_HALF_WIDTH),
        "front_left": (FRONT_CASTER_X, BASE_HALF_WIDTH),
        "front_right": (FRONT_CASTER_X, -BASE_HALF_WIDTH),
    }
    upper_column = model.part("upper_column")
    upper_column.visual(
        Box((UPPER_TUBE_X, UPPER_TUBE_Y, UPPER_TUBE_Z)),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material="satin_aluminum",
        name="inner_tube",
    )
    upper_column.visual(
        Box((0.090, 0.054, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="satin_aluminum",
        name="slide_collar",
    )
    upper_column.visual(
        Box((0.320, 0.054, 0.028)),
        origin=Origin(xyz=(0.160, 0.0, 0.286)),
        material="satin_aluminum",
        name="support_arm",
    )
    upper_column.visual(
        Box((0.140, 0.060, 0.010)),
        origin=Origin(xyz=(0.300, 0.0, 0.297)),
        material="satin_aluminum",
        name="mount_plate",
    )
    upper_column.visual(
        Box((0.260, 0.022, 0.022)),
        origin=Origin(xyz=(0.130, 0.0, 0.205), rpy=(0.0, -0.72, 0.0)),
        material="satin_aluminum",
        name="arm_brace",
    )

    fixed_top = model.part("fixed_top")
    fixed_top.visual(
        mesh_from_cadquery(_make_rounded_board(TABLE_LENGTH, FIXED_TOP_WIDTH, TOP_THICKNESS), "fixed_top_surface"),
        origin=Origin(xyz=(0.0, -FIXED_TOP_WIDTH / 2.0, 0.014)),
        material="laminate_fixed",
        name="fixed_surface",
    )
    fixed_top.visual(
        Box((0.120, 0.070, 0.020)),
        origin=Origin(xyz=(-0.040, -0.055, -0.005)),
        material="powder_charcoal",
        name="mount_block",
    )
    fixed_top.visual(
        Cylinder(radius=HINGE_PIN_R, length=0.680),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="caster_metal",
        name="hinge_pin",
    )
    for idx, x_pos in enumerate(HINGE_FIXED_X):
        fixed_top.visual(
            Cylinder(radius=HINGE_KNUCKLE_R, length=HINGE_SEGMENT_L),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="caster_metal",
            name=f"fixed_knuckle_{idx}",
        )

    tilt_panel = model.part("tilt_panel")
    tilt_panel.visual(
        mesh_from_cadquery(_make_rounded_board(TABLE_LENGTH, TILT_TOP_WIDTH, TOP_THICKNESS), "tilt_top_surface"),
        origin=Origin(xyz=(0.0, TILT_TOP_WIDTH / 2.0, 0.014)),
        material="laminate_tilt",
        name="main_surface",
    )
    for side_name, x_pos in (("left", HINGE_TILT_X[0]), ("right", HINGE_TILT_X[1])):
        tilt_panel.visual(
            mesh_from_cadquery(_make_hinge_ring(HINGE_SEGMENT_L, HINGE_KNUCKLE_R, 0.0055), f"tilt_knuckle_{side_name}"),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material="caster_metal",
            name=f"tilt_knuckle_{side_name}",
        )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, LOWER_MAST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=MAST_SLIDE_TRAVEL, effort=120.0, velocity=0.22),
    )
    model.articulation(
        "column_to_fixed_top",
        ArticulationType.FIXED,
        parent=upper_column,
        child=fixed_top,
        origin=Origin(xyz=(0.340, 0.0, 0.317)),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=fixed_top,
        child=tilt_panel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=20.0, velocity=1.2),
    )

    for label, (x_pos, y_pos) in caster_points.items():
        _add_caster(model, base_frame, label=label, x_pos=x_pos, y_pos=y_pos)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    upper_column = object_model.get_part("upper_column")
    fixed_top = object_model.get_part("fixed_top")
    tilt_panel = object_model.get_part("tilt_panel")
    mast_slide = object_model.get_articulation("mast_slide")
    table_tilt = object_model.get_articulation("table_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "mast slide is vertical prismatic motion",
        mast_slide.articulation_type == ArticulationType.PRISMATIC and _axis_matches(mast_slide.axis, (0.0, 0.0, 1.0)),
        details=f"type={mast_slide.articulation_type}, axis={mast_slide.axis}",
    )
    ctx.check(
        "table tilt uses a longitudinal hinge axis",
        table_tilt.articulation_type == ArticulationType.REVOLUTE and _axis_matches(table_tilt.axis, (1.0, 0.0, 0.0)),
        details=f"type={table_tilt.articulation_type}, axis={table_tilt.axis}",
    )

    ctx.expect_origin_gap(
        fixed_top,
        base_frame,
        axis="z",
        min_gap=0.78,
        max_gap=0.92,
        name="tabletop starts at a realistic hospital working height",
    )
    ctx.expect_contact(
        upper_column,
        base_frame,
        elem_a="slide_collar",
        elem_b="lower_mast",
        name="upper column collar rests on the lower mast",
    )
    ctx.expect_within(
        upper_column,
        base_frame,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="lower_mast",
        margin=0.020,
        name="upper column remains centered within the lower mast",
    )
    ctx.expect_overlap(
        upper_column,
        base_frame,
        axes="z",
        elem_a="inner_tube",
        elem_b="lower_mast",
        min_overlap=0.150,
        name="upper column retains insertion in the lower mast at rest",
    )
    ctx.expect_contact(
        fixed_top,
        upper_column,
        elem_a="mount_block",
        elem_b="mount_plate",
        name="fixed tabletop section is mounted to the cantilever arm",
    )
    ctx.expect_gap(
        tilt_panel,
        fixed_top,
        axis="y",
        positive_elem="main_surface",
        negative_elem="fixed_surface",
        max_gap=0.002,
        max_penetration=0.0,
        name="split tabletop panels meet cleanly at the hinge line",
    )
    ctx.expect_overlap(
        tilt_panel,
        fixed_top,
        axes="x",
        elem_a="main_surface",
        elem_b="fixed_surface",
        min_overlap=0.700,
        name="fixed and tilting tabletop panels stay aligned along the table length",
    )
    ctx.expect_contact(
        tilt_panel,
        fixed_top,
        name="tilting panel is physically hinged to the fixed panel",
    )

    rest_column_pos = ctx.part_world_position(upper_column)
    slide_upper = mast_slide.motion_limits.upper if mast_slide.motion_limits is not None else None
    with ctx.pose({mast_slide: slide_upper if slide_upper is not None else 0.0}):
        raised_column_pos = ctx.part_world_position(upper_column)
        ctx.expect_within(
            upper_column,
            base_frame,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="lower_mast",
            margin=0.020,
            name="raised upper column stays centered in the mast",
        )
        ctx.expect_overlap(
            upper_column,
            base_frame,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_mast",
            min_overlap=0.150,
            name="raised upper column still retains insertion",
        )
    ctx.check(
        "mast extension moves the tabletop upward",
        rest_column_pos is not None
        and raised_column_pos is not None
        and raised_column_pos[2] > rest_column_pos[2] + 0.18,
        details=f"rest={rest_column_pos}, raised={raised_column_pos}",
    )

    closed_main_aabb = ctx.part_element_world_aabb(tilt_panel, elem="main_surface")
    tilt_upper = table_tilt.motion_limits.upper if table_tilt.motion_limits is not None else None
    with ctx.pose({table_tilt: tilt_upper if tilt_upper is not None else 0.0}):
        opened_main_aabb = ctx.part_element_world_aabb(tilt_panel, elem="main_surface")
    ctx.check(
        "tilting panel lifts its free edge upward",
        closed_main_aabb is not None
        and opened_main_aabb is not None
        and opened_main_aabb[1][2] > closed_main_aabb[1][2] + 0.12,
        details=f"closed={closed_main_aabb}, opened={opened_main_aabb}",
    )

    for label in ("rear_left", "rear_right", "front_left", "front_right"):
        fork = object_model.get_part(f"{label}_fork")
        wheel = object_model.get_part(f"{label}_wheel")
        swivel = object_model.get_articulation(f"{label}_swivel")
        spin = object_model.get_articulation(f"{label}_spin")

        ctx.expect_contact(
            fork,
            base_frame,
            name=f"{label} caster fork mounts to the base",
        )
        ctx.expect_gap(
            wheel,
            fork,
            axis="y",
            positive_elem="wheel_body",
            negative_elem="right_arm",
            min_gap=0.001,
            max_gap=0.006,
            name=f"{label} wheel clears the right fork arm",
        )
        ctx.expect_gap(
            fork,
            wheel,
            axis="y",
            positive_elem="left_arm",
            negative_elem="wheel_body",
            min_gap=0.001,
            max_gap=0.006,
            name=f"{label} wheel clears the left fork arm",
        )
        ctx.expect_overlap(
            wheel,
            fork,
            axes="z",
            elem_a="wheel_body",
            elem_b="left_arm",
            min_overlap=0.020,
            name=f"{label} wheel sits within the fork arm drop",
        )
        ctx.check(
            f"{label} caster fork swivels about a vertical pivot",
            swivel.articulation_type == ArticulationType.CONTINUOUS and _axis_matches(swivel.axis, (0.0, 0.0, 1.0)),
            details=f"type={swivel.articulation_type}, axis={swivel.axis}",
        )
        ctx.check(
            f"{label} wheel spins on a horizontal axle",
            spin.articulation_type == ArticulationType.CONTINUOUS and _axis_matches(spin.axis, (0.0, 1.0, 0.0)),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
