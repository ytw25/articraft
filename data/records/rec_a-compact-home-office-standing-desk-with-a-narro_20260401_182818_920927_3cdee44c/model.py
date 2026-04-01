from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


DESK_WIDTH = 1.00
DESK_DEPTH = 0.55
TOP_THICKNESS = 0.026

LEG_CENTER_SPACING = 0.68

FOOT_WIDTH = 0.08
FOOT_LENGTH = 0.64
FOOT_THICKNESS = 0.02
FOOT_PAD_WIDTH = 0.11
FOOT_PAD_DEPTH = 0.10
FOOT_PAD_THICKNESS = 0.012

OUTER_X = 0.08
OUTER_Y = 0.055
OUTER_HEIGHT = 0.53
OUTER_WALL = 0.004
OUTER_BOTTOM_WALL = 0.02

STAGE_CLEARANCE = 0.006
INNER_X = OUTER_X - 2.0 * OUTER_WALL - STAGE_CLEARANCE
INNER_Y = OUTER_Y - 2.0 * OUTER_WALL - STAGE_CLEARANCE
INNER_HEIGHT = 0.56
INNER_CENTER_Z = -0.09
INNER_VISIBLE_ABOVE_JOINT = 0.19
LEG_TRAVEL = 0.28

MOUNT_BLOCK_X = 0.09
MOUNT_BLOCK_Y = 0.06
MOUNT_BLOCK_T = 0.028
MOUNT_BLOCK_CENTER_Z = INNER_VISIBLE_ABOVE_JOINT - MOUNT_BLOCK_T / 2.0

MOUNT_PLATE_X = 0.14
MOUNT_PLATE_Y = 0.10
MOUNT_PLATE_T = 0.008
MOUNT_PLATE_CENTER_Z = INNER_VISIBLE_ABOVE_JOINT + MOUNT_PLATE_T / 2.0
MOUNT_TOP_Z = INNER_VISIBLE_ABOVE_JOINT + MOUNT_PLATE_T

CROSSBAR_LENGTH = LEG_CENTER_SPACING - OUTER_X
CROSSBAR_Y = 0.05
CROSSBAR_Z = 0.04
CROSSBAR_CENTER_Z = 0.145

UNDERFRAME_LENGTH = 0.76
UNDERFRAME_WIDTH = 0.12
UNDERFRAME_THICKNESS = 0.018

CONTROL_PANEL_X = 0.118
CONTROL_PANEL_Y = 0.045
CONTROL_PANEL_Z = 0.018
CONTROL_PANEL_WALL = 0.002
CONTROL_PANEL_BOTTOM_T = 0.0025
CONTROL_PANEL_MOUNT = (0.77, -0.225, 0.0)

BUTTON_CAP_X = 0.013
BUTTON_CAP_Y = 0.009
BUTTON_CAP_Z = 0.003
BUTTON_PLUNGER_X = 0.007
BUTTON_PLUNGER_Y = 0.005
BUTTON_PLUNGER_Z = 0.009
BUTTON_TRAVEL = 0.0025
BUTTON_SPACING = 0.03
BUTTON_X_OFFSETS = (-BUTTON_SPACING, 0.0, BUTTON_SPACING)
BUTTON_COLLAR_X = 0.011
BUTTON_COLLAR_Y = 0.007
BUTTON_COLLAR_Z = 0.0015

GUIDE_PAD_T = STAGE_CLEARANCE / 2.0
GUIDE_PAD_LENGTH = 0.40
GUIDE_PAD_WIDTH = 0.018
GUIDE_PAD_CENTER_Z = -0.19
CONTROL_PANEL_TOP_T = 0.003


def _rounded_box(
    size: tuple[float, float, float],
    *,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    fillet: float = 0.0,
):
    sx, sy, sz = size
    solid = cq.Workplane("XY").box(sx, sy, sz)
    if fillet > 0.0:
        solid = solid.edges("|Z").fillet(min(fillet, sx * 0.49, sy * 0.49, sz * 0.49))
    return solid.translate(center)


def _add_visual(part, shape, *, mesh_name: str, visual_name: str, material: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        origin=Origin(),
        material=material,
        name=visual_name,
    )


def _foot_shape():
    foot = _rounded_box(
        (FOOT_WIDTH, FOOT_LENGTH, FOOT_THICKNESS),
        center=(0.0, 0.0, -FOOT_THICKNESS / 2.0),
        fillet=0.009,
    )
    pad = _rounded_box(
        (FOOT_PAD_WIDTH, FOOT_PAD_DEPTH, FOOT_PAD_THICKNESS),
        center=(0.0, 0.0, FOOT_PAD_THICKNESS / 2.0),
        fillet=0.004,
    )
    return foot.union(pad)


def _outer_sleeve_shape():
    outer = _rounded_box(
        (OUTER_X, OUTER_Y, OUTER_HEIGHT),
        center=(0.0, 0.0, OUTER_HEIGHT / 2.0),
        fillet=0.006,
    )
    inner = _rounded_box(
        (
            OUTER_X - 2.0 * OUTER_WALL,
            OUTER_Y - 2.0 * OUTER_WALL,
            OUTER_HEIGHT - OUTER_BOTTOM_WALL,
        ),
        center=(
            0.0,
            0.0,
            OUTER_BOTTOM_WALL + (OUTER_HEIGHT - OUTER_BOTTOM_WALL) / 2.0,
        ),
    )
    return outer.cut(inner)


def _crossbar_shape():
    return _rounded_box(
        (CROSSBAR_LENGTH, CROSSBAR_Y, CROSSBAR_Z),
        center=(0.0, 0.0, CROSSBAR_CENTER_Z),
        fillet=0.005,
    )


def _stage_post_shape():
    return _rounded_box(
        (INNER_X, INNER_Y, INNER_HEIGHT),
        center=(0.0, 0.0, INNER_CENTER_Z),
        fillet=0.004,
    )


def _stage_head_shape():
    return _rounded_box(
        (MOUNT_BLOCK_X, MOUNT_BLOCK_Y, MOUNT_BLOCK_T),
        center=(0.0, 0.0, MOUNT_BLOCK_CENTER_Z),
        fillet=0.003,
    )


def _stage_guides_shape():
    guides = _rounded_box(
        (GUIDE_PAD_T, GUIDE_PAD_WIDTH, GUIDE_PAD_LENGTH),
        center=(INNER_X / 2.0 + GUIDE_PAD_T / 2.0, 0.0, GUIDE_PAD_CENTER_Z),
        fillet=0.0006,
    )
    guides = guides.union(
        _rounded_box(
            (GUIDE_PAD_T, GUIDE_PAD_WIDTH, GUIDE_PAD_LENGTH),
            center=(-(INNER_X / 2.0 + GUIDE_PAD_T / 2.0), 0.0, GUIDE_PAD_CENTER_Z),
            fillet=0.0006,
        )
    )
    guides = guides.union(
        _rounded_box(
            (GUIDE_PAD_WIDTH, GUIDE_PAD_T, GUIDE_PAD_LENGTH),
            center=(0.0, INNER_Y / 2.0 + GUIDE_PAD_T / 2.0, GUIDE_PAD_CENTER_Z),
            fillet=0.0006,
        )
    )
    guides = guides.union(
        _rounded_box(
            (GUIDE_PAD_WIDTH, GUIDE_PAD_T, GUIDE_PAD_LENGTH),
            center=(0.0, -(INNER_Y / 2.0 + GUIDE_PAD_T / 2.0), GUIDE_PAD_CENTER_Z),
            fillet=0.0006,
        )
    )
    return guides


def _mount_plate_shape():
    return _rounded_box(
        (MOUNT_PLATE_X, MOUNT_PLATE_Y, MOUNT_PLATE_T),
        center=(0.0, 0.0, MOUNT_PLATE_CENTER_Z),
        fillet=0.004,
    )


def _desktop_board_shape():
    return _rounded_box(
        (DESK_WIDTH, DESK_DEPTH, TOP_THICKNESS),
        center=(LEG_CENTER_SPACING / 2.0, 0.0, TOP_THICKNESS / 2.0),
        fillet=0.012,
    )


def _underframe_shape():
    return _rounded_box(
        (UNDERFRAME_LENGTH, UNDERFRAME_WIDTH, UNDERFRAME_THICKNESS),
        center=(LEG_CENTER_SPACING / 2.0, 0.0, -UNDERFRAME_THICKNESS / 2.0),
        fillet=0.004,
    )


def _control_panel_shape():
    top_plate = _rounded_box(
        (CONTROL_PANEL_X, CONTROL_PANEL_Y, CONTROL_PANEL_TOP_T),
        center=(0.0, 0.0, -CONTROL_PANEL_TOP_T / 2.0),
        fillet=0.002,
    )
    left_wall = _rounded_box(
        (CONTROL_PANEL_WALL, CONTROL_PANEL_Y, CONTROL_PANEL_Z - CONTROL_PANEL_TOP_T),
        center=(
            -CONTROL_PANEL_X / 2.0 + CONTROL_PANEL_WALL / 2.0,
            0.0,
            -(CONTROL_PANEL_Z + CONTROL_PANEL_TOP_T) / 2.0,
        ),
        fillet=0.001,
    )
    right_wall = _rounded_box(
        (CONTROL_PANEL_WALL, CONTROL_PANEL_Y, CONTROL_PANEL_Z - CONTROL_PANEL_TOP_T),
        center=(
            CONTROL_PANEL_X / 2.0 - CONTROL_PANEL_WALL / 2.0,
            0.0,
            -(CONTROL_PANEL_Z + CONTROL_PANEL_TOP_T) / 2.0,
        ),
        fillet=0.001,
    )
    back_wall = _rounded_box(
        (
            CONTROL_PANEL_X - 2.0 * CONTROL_PANEL_WALL,
            CONTROL_PANEL_WALL,
            CONTROL_PANEL_Z - CONTROL_PANEL_TOP_T,
        ),
        center=(
            0.0,
            CONTROL_PANEL_Y / 2.0 - CONTROL_PANEL_WALL / 2.0,
            -(CONTROL_PANEL_Z + CONTROL_PANEL_TOP_T) / 2.0,
        ),
        fillet=0.001,
    )
    bottom_plate = _rounded_box(
        (
            CONTROL_PANEL_X - 2.0 * CONTROL_PANEL_WALL,
            CONTROL_PANEL_Y - 2.0 * CONTROL_PANEL_WALL,
            CONTROL_PANEL_BOTTOM_T,
        ),
        center=(
            0.0,
            0.0,
            -CONTROL_PANEL_Z + CONTROL_PANEL_BOTTOM_T / 2.0,
        ),
        fillet=0.001,
    )
    for x_off in BUTTON_X_OFFSETS:
        plunger_hole = _rounded_box(
            (
                BUTTON_PLUNGER_X + 0.0015,
                BUTTON_PLUNGER_Y + 0.0015,
                CONTROL_PANEL_BOTTOM_T + 0.002,
            ),
            center=(
                x_off,
                0.0,
                -CONTROL_PANEL_Z + CONTROL_PANEL_BOTTOM_T / 2.0,
            ),
        )
        bottom_plate = bottom_plate.cut(plunger_hole)
    return top_plate.union(left_wall).union(right_wall).union(back_wall).union(bottom_plate)


def _button_shape():
    cap = _rounded_box(
        (BUTTON_CAP_X, BUTTON_CAP_Y, BUTTON_CAP_Z),
        center=(0.0, 0.0, -BUTTON_PLUNGER_Z - BUTTON_CAP_Z / 2.0),
        fillet=0.001,
    )
    plunger = _rounded_box(
        (BUTTON_PLUNGER_X, BUTTON_PLUNGER_Y, BUTTON_PLUNGER_Z),
        center=(0.0, 0.0, -BUTTON_PLUNGER_Z / 2.0),
        fillet=0.0008,
    )
    collar = _rounded_box(
        (BUTTON_COLLAR_X, BUTTON_COLLAR_Y, BUTTON_COLLAR_Z),
        center=(0.0, 0.0, BUTTON_COLLAR_Z / 2.0),
        fillet=0.0006,
    )
    return cap.union(plunger).union(collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standing_desk")

    model.material("powder_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("charcoal_frame", rgba=(0.20, 0.21, 0.24, 1.0))
    model.material("warm_oak", rgba=(0.73, 0.59, 0.40, 1.0))
    model.material("dark_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("button_gray", rgba=(0.33, 0.35, 0.38, 1.0))

    crossbar = model.part("crossbar")
    _add_visual(
        crossbar,
        _crossbar_shape(),
        mesh_name="desk_crossbar",
        visual_name="crossbar_beam",
        material="charcoal_frame",
    )

    left_base = model.part("left_base_leg")
    _add_visual(
        left_base,
        _foot_shape(),
        mesh_name="left_leg_foot",
        visual_name="foot",
        material="powder_black",
    )
    _add_visual(
        left_base,
        _outer_sleeve_shape(),
        mesh_name="left_outer_sleeve",
        visual_name="outer_sleeve",
        material="charcoal_frame",
    )

    right_base = model.part("right_base_leg")
    _add_visual(
        right_base,
        _foot_shape(),
        mesh_name="right_leg_foot",
        visual_name="foot",
        material="powder_black",
    )
    _add_visual(
        right_base,
        _outer_sleeve_shape(),
        mesh_name="right_outer_sleeve",
        visual_name="outer_sleeve",
        material="charcoal_frame",
    )

    left_inner = model.part("left_inner_stage")
    _add_visual(
        left_inner,
        _stage_post_shape(),
        mesh_name="left_inner_stage",
        visual_name="stage_post",
        material="charcoal_frame",
    )
    _add_visual(
        left_inner,
        _stage_head_shape(),
        mesh_name="left_stage_head",
        visual_name="stage_head",
        material="charcoal_frame",
    )
    _add_visual(
        left_inner,
        _stage_guides_shape(),
        mesh_name="left_stage_guides",
        visual_name="stage_guides",
        material="powder_black",
    )
    _add_visual(
        left_inner,
        _mount_plate_shape(),
        mesh_name="left_mount_plate",
        visual_name="mount_plate",
        material="powder_black",
    )

    right_inner = model.part("right_inner_stage")
    _add_visual(
        right_inner,
        _stage_post_shape(),
        mesh_name="right_inner_stage",
        visual_name="stage_post",
        material="charcoal_frame",
    )
    _add_visual(
        right_inner,
        _stage_head_shape(),
        mesh_name="right_stage_head",
        visual_name="stage_head",
        material="charcoal_frame",
    )
    _add_visual(
        right_inner,
        _stage_guides_shape(),
        mesh_name="right_stage_guides",
        visual_name="stage_guides",
        material="powder_black",
    )
    _add_visual(
        right_inner,
        _mount_plate_shape(),
        mesh_name="right_mount_plate",
        visual_name="mount_plate",
        material="powder_black",
    )

    desktop = model.part("desktop")
    _add_visual(
        desktop,
        _desktop_board_shape(),
        mesh_name="desktop_board",
        visual_name="top_board",
        material="warm_oak",
    )
    _add_visual(
        desktop,
        _underframe_shape(),
        mesh_name="desktop_underframe",
        visual_name="underframe",
        material="powder_black",
    )

    control_panel = model.part("control_panel")
    _add_visual(
        control_panel,
        _control_panel_shape(),
        mesh_name="control_panel_housing",
        visual_name="housing",
        material="dark_plastic",
    )

    for button_name in ("button_1", "button_2", "button_3"):
        button = model.part(button_name)
        _add_visual(
            button,
            _button_shape(),
            mesh_name=f"{button_name}_mesh",
            visual_name="button",
            material="button_gray",
        )

    model.articulation(
        "crossbar_to_left_base",
        ArticulationType.FIXED,
        parent=crossbar,
        child=left_base,
        origin=Origin(xyz=(-LEG_CENTER_SPACING / 2.0, 0.0, FOOT_THICKNESS)),
    )
    model.articulation(
        "crossbar_to_right_base",
        ArticulationType.FIXED,
        parent=crossbar,
        child=right_base,
        origin=Origin(xyz=(LEG_CENTER_SPACING / 2.0, 0.0, FOOT_THICKNESS)),
    )
    model.articulation(
        "left_outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=left_base,
        child=left_inner,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.08,
            lower=0.0,
            upper=LEG_TRAVEL,
        ),
    )
    model.articulation(
        "right_outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=right_base,
        child=right_inner,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.08,
            lower=0.0,
            upper=LEG_TRAVEL,
        ),
    )
    model.articulation(
        "left_inner_to_desktop",
        ArticulationType.FIXED,
        parent=left_inner,
        child=desktop,
        origin=Origin(xyz=(0.0, 0.0, MOUNT_TOP_Z + UNDERFRAME_THICKNESS)),
    )
    model.articulation(
        "desktop_to_control_panel",
        ArticulationType.FIXED,
        parent=desktop,
        child=control_panel,
        origin=Origin(xyz=CONTROL_PANEL_MOUNT),
    )
    for button_name, x_off in zip(("button_1", "button_2", "button_3"), BUTTON_X_OFFSETS):
        model.articulation(
            f"panel_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button_name,
            origin=Origin(xyz=(x_off, 0.0, -CONTROL_PANEL_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crossbar = object_model.get_part("crossbar")
    left_base = object_model.get_part("left_base_leg")
    right_base = object_model.get_part("right_base_leg")
    left_inner = object_model.get_part("left_inner_stage")
    right_inner = object_model.get_part("right_inner_stage")
    desktop = object_model.get_part("desktop")
    control_panel = object_model.get_part("control_panel")
    button_1 = object_model.get_part("button_1")
    button_2 = object_model.get_part("button_2")
    button_3 = object_model.get_part("button_3")

    left_lift = object_model.get_articulation("left_outer_to_inner")
    right_lift = object_model.get_articulation("right_outer_to_inner")
    button_1_joint = object_model.get_articulation("panel_to_button_1")
    button_2_joint = object_model.get_articulation("panel_to_button_2")
    button_3_joint = object_model.get_articulation("panel_to_button_3")

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
        "lifting joints move upward",
        left_lift.axis == (0.0, 0.0, 1.0)
        and right_lift.axis == (0.0, 0.0, 1.0)
        and left_lift.motion_limits is not None
        and right_lift.motion_limits is not None
        and left_lift.motion_limits.upper == LEG_TRAVEL
        and right_lift.motion_limits.upper == LEG_TRAVEL,
        details=f"left={left_lift.axis}, right={right_lift.axis}",
    )
    ctx.check(
        "button plungers press upward into the panel",
        button_1_joint.axis == (0.0, 0.0, 1.0)
        and button_2_joint.axis == (0.0, 0.0, 1.0)
        and button_3_joint.axis == (0.0, 0.0, 1.0),
        details=(
            f"axes={(button_1_joint.axis, button_2_joint.axis, button_3_joint.axis)}"
        ),
    )

    ctx.expect_contact(
        crossbar,
        left_base,
        elem_a="crossbar_beam",
        elem_b="outer_sleeve",
        name="crossbar meets left outer column",
    )
    ctx.expect_contact(
        crossbar,
        right_base,
        elem_a="crossbar_beam",
        elem_b="outer_sleeve",
        name="crossbar meets right outer column",
    )

    ctx.expect_within(
        left_inner,
        left_base,
        axes="xy",
        inner_elem="stage_post",
        outer_elem="outer_sleeve",
        name="left inner stage stays centered in its sleeve",
    )
    ctx.expect_within(
        right_inner,
        right_base,
        axes="xy",
        inner_elem="stage_post",
        outer_elem="outer_sleeve",
        name="right inner stage stays centered in its sleeve",
    )
    ctx.expect_overlap(
        left_inner,
        left_base,
        axes="z",
        elem_a="stage_post",
        elem_b="outer_sleeve",
        min_overlap=0.30,
        name="left stage remains deeply inserted at seated height",
    )
    ctx.expect_overlap(
        right_inner,
        right_base,
        axes="z",
        elem_a="stage_post",
        elem_b="outer_sleeve",
        min_overlap=0.30,
        name="right stage remains deeply inserted at seated height",
    )

    ctx.expect_contact(
        left_inner,
        desktop,
        elem_a="mount_plate",
        elem_b="underframe",
        name="left lifting stage supports the desktop frame",
    )
    ctx.expect_contact(
        right_inner,
        desktop,
        elem_a="mount_plate",
        elem_b="underframe",
        name="right lifting stage supports the desktop frame",
    )
    ctx.expect_contact(
        control_panel,
        desktop,
        elem_a="housing",
        elem_b="top_board",
        name="control panel is mounted directly under the desktop",
    )

    desktop_rest = ctx.part_world_position(desktop)
    button_rest_positions = [
        ctx.part_world_position(button_1),
        ctx.part_world_position(button_2),
        ctx.part_world_position(button_3),
    ]
    with ctx.pose({left_lift: LEG_TRAVEL, right_lift: LEG_TRAVEL}):
        ctx.expect_within(
            left_inner,
            left_base,
            axes="xy",
            inner_elem="stage_post",
            outer_elem="outer_sleeve",
            name="left stage stays centered when fully raised",
        )
        ctx.expect_within(
            right_inner,
            right_base,
            axes="xy",
            inner_elem="stage_post",
            outer_elem="outer_sleeve",
            name="right stage stays centered when fully raised",
        )
        ctx.expect_overlap(
            left_inner,
            left_base,
            axes="z",
            elem_a="stage_post",
            elem_b="outer_sleeve",
            min_overlap=0.08,
            name="left stage retains insertion at full height",
        )
        ctx.expect_overlap(
            right_inner,
            right_base,
            axes="z",
            elem_a="stage_post",
            elem_b="outer_sleeve",
            min_overlap=0.08,
            name="right stage retains insertion at full height",
        )
        ctx.expect_contact(
            left_inner,
            desktop,
            elem_a="mount_plate",
            elem_b="underframe",
            name="left stage still supports the desktop when raised",
        )
        ctx.expect_contact(
            right_inner,
            desktop,
            elem_a="mount_plate",
            elem_b="underframe",
            name="right stage still supports the desktop when raised",
        )
        desktop_raised = ctx.part_world_position(desktop)
    ctx.check(
        "desktop lifts upward with both columns",
        desktop_rest is not None
        and desktop_raised is not None
        and desktop_raised[2] > desktop_rest[2] + 0.20,
        details=f"rest={desktop_rest}, raised={desktop_raised}",
    )

    with ctx.pose(
        {
            button_1_joint: BUTTON_TRAVEL,
            button_2_joint: BUTTON_TRAVEL,
            button_3_joint: BUTTON_TRAVEL,
        }
    ):
        button_pressed_positions = [
            ctx.part_world_position(button_1),
            ctx.part_world_position(button_2),
            ctx.part_world_position(button_3),
        ]
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no part overlaps with all three buttons fully pressed"
        )
    for idx, (rest_pos, pressed_pos) in enumerate(
        zip(button_rest_positions, button_pressed_positions), start=1
    ):
        ctx.check(
            f"button {idx} moves upward on its plunger",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] > rest_pos[2] + 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
