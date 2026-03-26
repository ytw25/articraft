from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

OUTER_WIDTH = 0.48
OUTER_DEPTH = 0.36
OUTER_HEIGHT = 0.28

FRONT_CORNER_RADIUS = 0.028
REAR_CORNER_RADIUS = 0.012

CONTROL_COLUMN_WIDTH = 0.11
DIVIDER_WIDTH = 0.012
SIDE_WALL = 0.012

OPENING_LEFT = -OUTER_WIDTH / 2.0 + SIDE_WALL
OPENING_WIDTH = OUTER_WIDTH - (2.0 * SIDE_WALL) - CONTROL_COLUMN_WIDTH - DIVIDER_WIDTH
OPENING_BOTTOM = 0.038
OPENING_HEIGHT = 0.208
OPENING_CENTER_Z = OPENING_BOTTOM + (OPENING_HEIGHT / 2.0)

CAVITY_DEPTH = 0.32
CAVITY_CENTER_X = OPENING_LEFT + (OPENING_WIDTH / 2.0)
CAVITY_CENTER_Y = (OUTER_DEPTH / 2.0) - (CAVITY_DEPTH / 2.0) + 0.005
CAVITY_CENTER_Z = OPENING_CENTER_Z

DOOR_THICKNESS = 0.018
DOOR_WIDTH = OPENING_WIDTH + 0.004
DOOR_HEIGHT = OPENING_HEIGHT + 0.010

CONTROL_FACE_WIDTH = CONTROL_COLUMN_WIDTH - 0.012
CONTROL_FACE_HEIGHT = OUTER_HEIGHT - 0.036
CONTROL_FACE_THICKNESS = 0.018
CONTROL_FACE_CENTER_X = (
    OPENING_LEFT + OPENING_WIDTH + DIVIDER_WIDTH + (CONTROL_FACE_WIDTH / 2.0)
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _section_loop(
    profile: list[tuple[float, float]],
    *,
    y_pos: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [(x, y_pos, z + z_center) for x, z in profile]


def _build_body_shell_mesh():
    front_profile = rounded_rect_profile(
        OUTER_WIDTH,
        OUTER_HEIGHT,
        FRONT_CORNER_RADIUS,
        corner_segments=10,
    )
    back_profile = rounded_rect_profile(
        OUTER_WIDTH,
        OUTER_HEIGHT,
        REAR_CORNER_RADIUS,
        corner_segments=10,
    )

    outer_shell = repair_loft(
        section_loft(
            [
                _section_loop(
                    front_profile,
                    y_pos=OUTER_DEPTH / 2.0,
                    z_center=OUTER_HEIGHT / 2.0,
                ),
                _section_loop(
                    back_profile,
                    y_pos=-OUTER_DEPTH / 2.0,
                    z_center=OUTER_HEIGHT / 2.0,
                ),
            ]
        )
    )

    cavity_cut = BoxGeometry((OPENING_WIDTH, CAVITY_DEPTH, OPENING_HEIGHT)).translate(
        CAVITY_CENTER_X,
        CAVITY_CENTER_Y,
        CAVITY_CENTER_Z,
    )
    return boolean_difference(outer_shell, cavity_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_microwave", assets=ASSETS)

    body_white = model.material("body_white", rgba=(0.93, 0.93, 0.91, 1.0))
    panel_black = model.material("panel_black", rgba=(0.11, 0.12, 0.13, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.22, 0.23, 0.24, 1.0))
    cavity_gray = model.material("cavity_gray", rgba=(0.72, 0.73, 0.75, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.26, 0.34, 0.39, 0.32))
    button_gray = model.material("button_gray", rgba=(0.82, 0.84, 0.86, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.75, 0.77, 0.79, 1.0))

    body = model.part("body")
    body.visual(
        _save_mesh("microwave_body_shell.obj", _build_body_shell_mesh()),
        material=body_white,
        name="housing_shell",
    )
    body.visual(
        Box((OPENING_WIDTH - 0.026, CAVITY_DEPTH - 0.028, 0.002)),
        origin=Origin(
            xyz=(
                CAVITY_CENTER_X,
                CAVITY_CENTER_Y - 0.004,
                OPENING_BOTTOM + 0.001,
            )
        ),
        material=cavity_gray,
        name="cavity_floor_plate",
    )
    body.inertial = Inertial.from_geometry(
        Box((OUTER_WIDTH, OUTER_DEPTH, OUTER_HEIGHT)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, OUTER_HEIGHT / 2.0)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((CONTROL_FACE_WIDTH, CONTROL_FACE_THICKNESS, CONTROL_FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, CONTROL_FACE_THICKNESS / 2.0, 0.0)),
        material=panel_black,
        name="panel_face",
    )
    control_panel.visual(
        Box((CONTROL_FACE_WIDTH - 0.016, 0.003, CONTROL_FACE_HEIGHT - 0.030)),
        origin=Origin(xyz=(0.0, CONTROL_FACE_THICKNESS + 0.0015, 0.0)),
        material=trim_dark,
        name="panel_overlay",
    )
    control_panel.visual(
        Box((CONTROL_FACE_WIDTH * 0.62, 0.004, 0.036)),
        origin=Origin(xyz=(0.0, CONTROL_FACE_THICKNESS + 0.002, 0.078)),
        material=glass_dark,
        name="display_window",
    )
    control_panel.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(
            xyz=(0.0, CONTROL_FACE_THICKNESS + 0.005, -0.020),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=knob_silver,
        name="timer_dial",
    )
    for index, z_pos in enumerate((0.030, 0.006, -0.018, -0.042)):
        control_panel.visual(
            Box((CONTROL_FACE_WIDTH * 0.60, 0.006, 0.010)),
            origin=Origin(
                xyz=(0.0, CONTROL_FACE_THICKNESS + 0.003, z_pos),
            ),
            material=button_gray if index < 3 else trim_dark,
            name=f"button_strip_{index}",
        )
    control_panel.visual(
        Box((CONTROL_FACE_WIDTH * 0.44, 0.005, 0.016)),
        origin=Origin(xyz=(0.0, CONTROL_FACE_THICKNESS + 0.0025, -0.090)),
        material=button_gray,
        name="door_release_button",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((CONTROL_FACE_WIDTH, CONTROL_FACE_THICKNESS + 0.020, CONTROL_FACE_HEIGHT)),
        mass=1.1,
        origin=Origin(
            xyz=(0.0, (CONTROL_FACE_THICKNESS + 0.020) / 2.0, 0.0),
        ),
    )

    door = model.part("door")
    frame_width = 0.022
    rail_height = 0.022
    glass_width = DOOR_WIDTH - 0.042
    glass_height = DOOR_HEIGHT - 0.042
    door.visual(
        Box((frame_width, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(xyz=(frame_width / 2.0, DOOR_THICKNESS / 2.0, 0.0)),
        material=trim_dark,
        name="hinge_stile",
    )
    door.visual(
        Box((frame_width, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(DOOR_WIDTH - (frame_width / 2.0), DOOR_THICKNESS / 2.0, 0.0)
        ),
        material=trim_dark,
        name="handle_stile",
    )
    door.visual(
        Box((DOOR_WIDTH - (2.0 * frame_width), DOOR_THICKNESS, rail_height)),
        origin=Origin(
            xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, (DOOR_HEIGHT / 2.0) - (rail_height / 2.0))
        ),
        material=trim_dark,
        name="top_rail",
    )
    door.visual(
        Box((DOOR_WIDTH - (2.0 * frame_width), DOOR_THICKNESS, rail_height)),
        origin=Origin(
            xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS / 2.0, -(DOOR_HEIGHT / 2.0) + (rail_height / 2.0))
        ),
        material=trim_dark,
        name="bottom_rail",
    )
    door.visual(
        Box((glass_width, DOOR_THICKNESS * 0.56, glass_height)),
        origin=Origin(
            xyz=(DOOR_WIDTH / 2.0, DOOR_THICKNESS * 0.36, 0.0),
        ),
        material=glass_dark,
        name="door_glass",
    )
    handle_x = DOOR_WIDTH - 0.020
    handle_y = DOOR_THICKNESS + 0.013
    for index, z_pos in enumerate((0.038, -0.038)):
        door.visual(
            Cylinder(radius=0.0045, length=0.020),
            origin=Origin(
                xyz=(handle_x, DOOR_THICKNESS + 0.010, z_pos),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=knob_silver,
            name=f"door_handle_post_{index}",
        )
    door.visual(
        Cylinder(radius=0.0065, length=0.116),
        origin=Origin(
            xyz=(handle_x, handle_y, 0.0),
        ),
        material=knob_silver,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, DOOR_THICKNESS + 0.024, DOOR_HEIGHT)),
        mass=2.2,
        origin=Origin(
            xyz=(DOOR_WIDTH / 2.0, (DOOR_THICKNESS + 0.024) / 2.0, 0.0),
        ),
    )

    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(
            xyz=(
                CONTROL_FACE_CENTER_X,
                OUTER_DEPTH / 2.0,
                OUTER_HEIGHT / 2.0,
            )
        ),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(
            xyz=(
                OPENING_LEFT,
                OUTER_DEPTH / 2.0,
                OPENING_CENTER_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    control_panel = object_model.get_part("control_panel")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="door_sits_flush_over_opening",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.19,
        name="door_covers_cavity_opening",
    )
    ctx.expect_gap(
        control_panel,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="control_panel_mounts_to_front_column",
    )
    ctx.expect_overlap(
        control_panel,
        body,
        axes="xz",
        min_overlap=0.09,
        name="control_panel_overlaps_body_column",
    )
    ctx.expect_gap(
        control_panel,
        door,
        axis="x",
        min_gap=0.006,
        max_gap=0.020,
        name="door_and_control_panel_have_center_seam",
    )

    body_aabb = ctx.part_world_aabb(body)
    door_rest_aabb = ctx.part_world_aabb(door)
    control_aabb = ctx.part_world_aabb(control_panel)

    assert body_aabb is not None
    assert door_rest_aabb is not None
    assert control_aabb is not None

    body_dx = body_aabb[1][0] - body_aabb[0][0]
    body_dy = body_aabb[1][1] - body_aabb[0][1]
    body_dz = body_aabb[1][2] - body_aabb[0][2]
    assert 0.46 < body_dx < 0.50
    assert 0.34 < body_dy < 0.38
    assert 0.27 < body_dz < 0.29

    assert control_aabb[0][0] > door_rest_aabb[1][0]
    assert door_hinge.axis == (0.0, 0.0, 1.0)
    assert door_hinge.motion_limits is not None
    assert math.isclose(door_hinge.motion_limits.lower, 0.0, abs_tol=1e-9)
    assert 1.5 < door_hinge.motion_limits.upper < 1.7

    with ctx.pose({door_hinge: math.radians(90.0)}):
        door_open_aabb = ctx.part_world_aabb(door)
        assert door_open_aabb is not None
        assert door_open_aabb[1][1] > door_rest_aabb[1][1] + 0.26
        assert (door_open_aabb[1][0] - door_open_aabb[0][0]) < 0.05
        ctx.expect_gap(
            control_panel,
            door,
            axis="x",
            min_gap=0.25,
            name="open_door_swings_clear_of_controls",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            min_gap=0.0,
            max_penetration=0.0,
            name="open_door_does_not_cut_into_body",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
