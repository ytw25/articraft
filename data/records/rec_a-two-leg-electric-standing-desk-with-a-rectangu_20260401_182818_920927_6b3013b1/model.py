from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOP_LENGTH = 1.40
TOP_DEPTH = 0.75
TOP_THICKNESS = 0.028

LEG_CENTER_X = 0.46

FRAME_DROP = 0.080
FRAME_TOP_PLATE_THICKNESS = 0.006
FRAME_HEAD_BLOCK_HEIGHT = FRAME_DROP - FRAME_TOP_PLATE_THICKNESS

OUTER_SLEEVE_WIDTH = 0.090
OUTER_SLEEVE_DEPTH = 0.070
OUTER_SLEEVE_HEIGHT = 0.500
OUTER_SIDE_WALL = 0.003
OUTER_FRONT_BACK_WALL = 0.006

INNER_STAGE_WIDTH = 0.084
INNER_STAGE_DEPTH = 0.058
INNER_STAGE_LENGTH = 0.620
LEG_TRAVEL = 0.220

FOOT_LENGTH = 0.700
FOOT_WIDTH = 0.090
FOOT_THICKNESS = 0.028
FOOT_RISER_HEIGHT = 0.045

KEYPAD_WIDTH = 0.110
KEYPAD_DEPTH = 0.045
KEYPAD_HEIGHT = 0.020
KEYPAD_WALL = 0.005
KEYPAD_ROOF = 0.004
KEYPAD_BOTTOM_WALL = 0.0035
BUTTON_SPACING = 0.026
BUTTON_STEM_RADIUS = 0.0048
BUTTON_HEAD_RADIUS = 0.0070
BUTTON_HEAD_HEIGHT = 0.0022
BUTTON_PROTRUSION = 0.0048
BUTTON_TRAVEL = 0.0028
KEYPAD_X = TOP_LENGTH / 2.0 - 0.175
KEYPAD_Y = -(TOP_DEPTH / 2.0 - 0.050)


def _worktop_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(TOP_LENGTH, TOP_DEPTH, TOP_THICKNESS)
        .translate((0.0, 0.0, -TOP_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )


def _underframe_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(0.780, 0.120, 0.050).translate((0.0, 0.0, -0.055))

    for sign in (-1.0, 1.0):
        x = sign * LEG_CENTER_X
        top_plate = (
            cq.Workplane("XY")
            .box(0.240, 0.180, FRAME_TOP_PLATE_THICKNESS)
            .translate((x, 0.0, -FRAME_TOP_PLATE_THICKNESS / 2.0))
        )
        head_block = (
            cq.Workplane("XY")
            .box(0.140, 0.220, FRAME_HEAD_BLOCK_HEIGHT)
            .translate((x, 0.0, -(FRAME_TOP_PLATE_THICKNESS + FRAME_HEAD_BLOCK_HEIGHT / 2.0)))
        )
        side_rail = cq.Workplane("XY").box(0.090, 0.520, 0.030).translate((x, 0.0, -0.021))
        frame = frame.union(top_plate).union(head_block).union(side_rail)

    frame = frame.union(cq.Workplane("XY").box(0.980, 0.040, 0.030).translate((0.0, -0.165, -0.021)))
    frame = frame.union(cq.Workplane("XY").box(0.980, 0.040, 0.030).translate((0.0, 0.165, -0.021)))
    return frame.edges("|Z").fillet(0.004)


def _outer_sleeve_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(OUTER_SLEEVE_WIDTH, OUTER_SLEEVE_DEPTH, OUTER_SLEEVE_HEIGHT)
        .translate((0.0, 0.0, -OUTER_SLEEVE_HEIGHT / 2.0))
    )
    cavity = (
        cq.Workplane("XY")
        .box(INNER_STAGE_WIDTH, 0.060, OUTER_SLEEVE_HEIGHT + 0.020)
        .translate((0.0, 0.0, -OUTER_SLEEVE_HEIGHT / 2.0))
    )
    return outer.cut(cavity).edges("|Z").fillet(0.004)


def _add_outer_sleeve_visuals(part, prefix: str, material: str) -> None:
    z_center = -OUTER_SLEEVE_HEIGHT / 2.0
    part.visual(
        Box((OUTER_SLEEVE_WIDTH, OUTER_FRONT_BACK_WALL, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(0.0, OUTER_SLEEVE_DEPTH / 2.0 - OUTER_FRONT_BACK_WALL / 2.0, z_center)),
        material=material,
        name=f"{prefix}_front_wall",
    )
    part.visual(
        Box((OUTER_SLEEVE_WIDTH, OUTER_FRONT_BACK_WALL, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(0.0, -(OUTER_SLEEVE_DEPTH / 2.0 - OUTER_FRONT_BACK_WALL / 2.0), z_center)),
        material=material,
        name=f"{prefix}_back_wall",
    )
    part.visual(
        Box((OUTER_SIDE_WALL, OUTER_SLEEVE_DEPTH, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(OUTER_SLEEVE_WIDTH / 2.0 - OUTER_SIDE_WALL / 2.0, 0.0, z_center)),
        material=material,
        name=f"{prefix}_right_wall",
    )
    part.visual(
        Box((OUTER_SIDE_WALL, OUTER_SLEEVE_DEPTH, OUTER_SLEEVE_HEIGHT)),
        origin=Origin(xyz=(-(OUTER_SLEEVE_WIDTH / 2.0 - OUTER_SIDE_WALL / 2.0), 0.0, z_center)),
        material=material,
        name=f"{prefix}_left_wall",
    )


def _inner_stage_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(INNER_STAGE_WIDTH, INNER_STAGE_DEPTH, INNER_STAGE_LENGTH)
        .translate((0.0, 0.0, -INNER_STAGE_LENGTH / 2.0))
        .edges("|Z")
        .fillet(0.0035)
    )


def _foot_shape() -> cq.Workplane:
    foot_beam = (
        cq.Workplane("XY")
        .slot2D(FOOT_LENGTH - FOOT_WIDTH, FOOT_WIDTH)
        .extrude(FOOT_THICKNESS)
        .translate((0.0, 0.0, -(FOOT_RISER_HEIGHT + FOOT_THICKNESS)))
    )
    riser = (
        cq.Workplane("XY")
        .box(0.120, 0.080, FOOT_RISER_HEIGHT)
        .translate((0.0, 0.0, -FOOT_RISER_HEIGHT / 2.0))
    )
    return foot_beam.union(riser).edges("|Z").fillet(0.006)


def _keypad_housing_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(KEYPAD_WIDTH, KEYPAD_DEPTH, KEYPAD_HEIGHT)
        .translate((0.0, 0.0, -KEYPAD_HEIGHT / 2.0))
    )
    cavity_height = KEYPAD_HEIGHT - KEYPAD_ROOF - KEYPAD_BOTTOM_WALL
    cavity_center_z = -(KEYPAD_ROOF + cavity_height / 2.0)
    cavity = (
        cq.Workplane("XY")
        .box(KEYPAD_WIDTH - 2.0 * KEYPAD_WALL, KEYPAD_DEPTH - 2.0 * KEYPAD_WALL, cavity_height)
        .translate((0.0, 0.0, cavity_center_z))
    )
    housing = outer.cut(cavity)

    for x in (-BUTTON_SPACING / 2.0, BUTTON_SPACING / 2.0):
        guide_hole = (
            cq.Workplane("XY")
            .circle(BUTTON_STEM_RADIUS + 0.0003)
            .extrude(KEYPAD_BOTTOM_WALL + 0.001)
            .translate((x, 0.0, -KEYPAD_HEIGHT))
        )
        housing = housing.cut(guide_hole)

    return housing.edges("|Z").fillet(0.003)


def _button_shape() -> cq.Workplane:
    head = cq.Workplane("XY").circle(BUTTON_HEAD_RADIUS).extrude(BUTTON_HEAD_HEIGHT)
    stem = (
        cq.Workplane("XY")
        .circle(BUTTON_STEM_RADIUS)
        .extrude(KEYPAD_BOTTOM_WALL + BUTTON_PROTRUSION)
        .translate((0.0, 0.0, -(KEYPAD_BOTTOM_WALL + BUTTON_PROTRUSION)))
    )
    return head.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_standing_desk")

    model.material("oak_top", rgba=(0.77, 0.64, 0.46, 1.0))
    model.material("powder_coat", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("column_steel", rgba=(0.54, 0.56, 0.59, 1.0))
    model.material("dark_plastic", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("button_gray", rgba=(0.82, 0.84, 0.86, 1.0))

    worktop = model.part("worktop")
    worktop.visual(
        mesh_from_cadquery(_worktop_shape(), "standing_desk_worktop"),
        material="oak_top",
        name="top_panel",
    )

    underframe = model.part("underframe")
    underframe.visual(
        mesh_from_cadquery(_underframe_shape(), "standing_desk_underframe"),
        material="powder_coat",
        name="frame",
    )

    left_outer = model.part("left_outer_sleeve")
    _add_outer_sleeve_visuals(left_outer, "left_outer", "powder_coat")

    right_outer = model.part("right_outer_sleeve")
    _add_outer_sleeve_visuals(right_outer, "right_outer", "powder_coat")

    left_inner = model.part("left_inner_stage")
    left_inner.visual(
        mesh_from_cadquery(_inner_stage_shape(), "standing_desk_left_inner_stage"),
        material="column_steel",
        name="left_inner_stage",
    )

    right_inner = model.part("right_inner_stage")
    right_inner.visual(
        mesh_from_cadquery(_inner_stage_shape(), "standing_desk_right_inner_stage"),
        material="column_steel",
        name="right_inner_stage",
    )

    left_foot = model.part("left_foot")
    left_foot.visual(
        mesh_from_cadquery(_foot_shape(), "standing_desk_left_foot"),
        material="powder_coat",
        name="left_foot",
    )

    right_foot = model.part("right_foot")
    right_foot.visual(
        mesh_from_cadquery(_foot_shape(), "standing_desk_right_foot"),
        material="powder_coat",
        name="right_foot",
    )

    keypad_housing = model.part("keypad_housing")
    keypad_housing.visual(
        mesh_from_cadquery(_keypad_housing_shape(), "standing_desk_keypad_housing"),
        material="dark_plastic",
        name="housing_shell",
    )

    up_button = model.part("up_button")
    up_button.visual(
        mesh_from_cadquery(_button_shape(), "standing_desk_up_button"),
        material="button_gray",
        name="up_button",
    )

    down_button = model.part("down_button")
    down_button.visual(
        mesh_from_cadquery(_button_shape(), "standing_desk_down_button"),
        material="button_gray",
        name="down_button",
    )

    model.articulation(
        "worktop_to_underframe",
        ArticulationType.FIXED,
        parent=worktop,
        child=underframe,
        origin=Origin(xyz=(0.0, 0.0, -TOP_THICKNESS)),
    )

    model.articulation(
        "underframe_to_left_outer",
        ArticulationType.FIXED,
        parent=underframe,
        child=left_outer,
        origin=Origin(xyz=(-LEG_CENTER_X, 0.0, -FRAME_DROP)),
    )
    model.articulation(
        "underframe_to_right_outer",
        ArticulationType.FIXED,
        parent=underframe,
        child=right_outer,
        origin=Origin(xyz=(LEG_CENTER_X, 0.0, -FRAME_DROP)),
    )

    model.articulation(
        "left_outer_to_left_inner",
        ArticulationType.PRISMATIC,
        parent=left_outer,
        child=left_inner,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.060, lower=0.0, upper=LEG_TRAVEL),
    )
    model.articulation(
        "right_outer_to_right_inner",
        ArticulationType.PRISMATIC,
        parent=right_outer,
        child=right_inner,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.060, lower=0.0, upper=LEG_TRAVEL),
    )

    model.articulation(
        "left_inner_to_left_foot",
        ArticulationType.FIXED,
        parent=left_inner,
        child=left_foot,
        origin=Origin(xyz=(0.0, 0.0, -INNER_STAGE_LENGTH)),
    )
    model.articulation(
        "right_inner_to_right_foot",
        ArticulationType.FIXED,
        parent=right_inner,
        child=right_foot,
        origin=Origin(xyz=(0.0, 0.0, -INNER_STAGE_LENGTH)),
    )

    model.articulation(
        "worktop_to_keypad_housing",
        ArticulationType.FIXED,
        parent=worktop,
        child=keypad_housing,
        origin=Origin(xyz=(KEYPAD_X, KEYPAD_Y, -TOP_THICKNESS)),
    )

    button_origin_z = -KEYPAD_HEIGHT + KEYPAD_BOTTOM_WALL
    model.articulation(
        "keypad_to_up_button",
        ArticulationType.PRISMATIC,
        parent=keypad_housing,
        child=up_button,
        origin=Origin(xyz=(-BUTTON_SPACING / 2.0, 0.0, button_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.030, lower=0.0, upper=BUTTON_TRAVEL),
    )
    model.articulation(
        "keypad_to_down_button",
        ArticulationType.PRISMATIC,
        parent=keypad_housing,
        child=down_button,
        origin=Origin(xyz=(BUTTON_SPACING / 2.0, 0.0, button_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.030, lower=0.0, upper=BUTTON_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    worktop = object_model.get_part("worktop")
    underframe = object_model.get_part("underframe")
    left_outer = object_model.get_part("left_outer_sleeve")
    right_outer = object_model.get_part("right_outer_sleeve")
    left_inner = object_model.get_part("left_inner_stage")
    right_inner = object_model.get_part("right_inner_stage")
    left_foot = object_model.get_part("left_foot")
    right_foot = object_model.get_part("right_foot")
    keypad_housing = object_model.get_part("keypad_housing")
    up_button = object_model.get_part("up_button")
    down_button = object_model.get_part("down_button")

    left_lift = object_model.get_articulation("left_outer_to_left_inner")
    right_lift = object_model.get_articulation("right_outer_to_right_inner")
    up_button_slide = object_model.get_articulation("keypad_to_up_button")
    down_button_slide = object_model.get_articulation("keypad_to_down_button")

    ctx.expect_contact(
        underframe,
        worktop,
        elem_a="frame",
        elem_b="top_panel",
        name="underframe is mounted hard against the underside of the worktop",
    )
    ctx.expect_contact(
        keypad_housing,
        worktop,
        elem_a="housing_shell",
        elem_b="top_panel",
        name="keypad housing is mounted under the front edge of the worktop",
    )
    ctx.expect_contact(
        left_outer,
        underframe,
        elem_b="frame",
        name="left outer sleeve is bolted to the steel underframe",
    )
    ctx.expect_contact(
        right_outer,
        underframe,
        elem_b="frame",
        name="right outer sleeve is bolted to the steel underframe",
    )
    ctx.expect_contact(
        left_inner,
        left_outer,
        elem_a="left_inner_stage",
        name="left inner stage bears on the left sleeve guides",
    )
    ctx.expect_contact(
        right_inner,
        right_outer,
        elem_a="right_inner_stage",
        name="right inner stage bears on the right sleeve guides",
    )
    ctx.expect_contact(
        left_foot,
        left_inner,
        elem_a="left_foot",
        elem_b="left_inner_stage",
        name="left foot is fixed to the left lifting stage",
    )
    ctx.expect_contact(
        right_foot,
        right_inner,
        elem_a="right_foot",
        elem_b="right_inner_stage",
        name="right foot is fixed to the right lifting stage",
    )
    ctx.expect_contact(
        up_button,
        keypad_housing,
        elem_a="up_button",
        elem_b="housing_shell",
        name="up button is retained by the keypad housing guide",
    )
    ctx.expect_contact(
        down_button,
        keypad_housing,
        elem_a="down_button",
        elem_b="housing_shell",
        name="down button is retained by the keypad housing guide",
    )

    ctx.expect_within(
        left_inner,
        left_outer,
        axes="xy",
        inner_elem="left_inner_stage",
        margin=0.0015,
        name="left telescoping stage stays centered inside its sleeve",
    )
    ctx.expect_within(
        right_inner,
        right_outer,
        axes="xy",
        inner_elem="right_inner_stage",
        margin=0.0015,
        name="right telescoping stage stays centered inside its sleeve",
    )
    ctx.expect_overlap(
        left_inner,
        left_outer,
        axes="z",
        elem_a="left_inner_stage",
        min_overlap=0.48,
        name="left inner stage remains deeply inserted at the low pose",
    )
    ctx.expect_overlap(
        right_inner,
        right_outer,
        axes="z",
        elem_a="right_inner_stage",
        min_overlap=0.48,
        name="right inner stage remains deeply inserted at the low pose",
    )

    left_foot_rest = ctx.part_world_position(left_foot)
    right_foot_rest = ctx.part_world_position(right_foot)
    up_button_rest = ctx.part_world_position(up_button)
    down_button_rest = ctx.part_world_position(down_button)

    with ctx.pose({left_lift: LEG_TRAVEL, right_lift: LEG_TRAVEL}):
        ctx.expect_within(
            left_inner,
            left_outer,
            axes="xy",
            inner_elem="left_inner_stage",
            margin=0.0015,
            name="left telescoping stage stays centered at maximum height",
        )
        ctx.expect_within(
            right_inner,
            right_outer,
            axes="xy",
            inner_elem="right_inner_stage",
            margin=0.0015,
            name="right telescoping stage stays centered at maximum height",
        )
        ctx.expect_overlap(
            left_inner,
            left_outer,
            axes="z",
            elem_a="left_inner_stage",
            min_overlap=0.26,
            name="left stage retains insertion at maximum height",
        )
        ctx.expect_overlap(
            right_inner,
            right_outer,
            axes="z",
            elem_a="right_inner_stage",
            min_overlap=0.26,
            name="right stage retains insertion at maximum height",
        )
        left_foot_extended = ctx.part_world_position(left_foot)
        right_foot_extended = ctx.part_world_position(right_foot)

    with ctx.pose({up_button_slide: BUTTON_TRAVEL, down_button_slide: BUTTON_TRAVEL}):
        up_button_pressed = ctx.part_world_position(up_button)
        down_button_pressed = ctx.part_world_position(down_button)

    ctx.check(
        "both lifting columns extend downward from the frame when driven upward in travel",
        left_foot_rest is not None
        and right_foot_rest is not None
        and left_foot_extended is not None
        and right_foot_extended is not None
        and left_foot_extended[2] < left_foot_rest[2] - 0.18
        and right_foot_extended[2] < right_foot_rest[2] - 0.18,
        details=(
            f"left_rest={left_foot_rest}, left_extended={left_foot_extended}, "
            f"right_rest={right_foot_rest}, right_extended={right_foot_extended}"
        ),
    )
    ctx.check(
        "both keypad buttons travel upward on short vertical guides",
        up_button_rest is not None
        and down_button_rest is not None
        and up_button_pressed is not None
        and down_button_pressed is not None
        and up_button_pressed[2] > up_button_rest[2] + 0.002
        and down_button_pressed[2] > down_button_rest[2] + 0.002,
        details=(
            f"up_rest={up_button_rest}, up_pressed={up_button_pressed}, "
            f"down_rest={down_button_rest}, down_pressed={down_button_pressed}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
