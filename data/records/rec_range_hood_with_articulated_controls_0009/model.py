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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

CANOPY_WIDTH = 0.90
CANOPY_DEPTH = 0.50
CANOPY_HEIGHT = 0.20
SHELL_THICKNESS = 0.018
TOP_THICKNESS = 0.020
WALL_HEIGHT = 0.190

CONTROL_PANEL_WIDTH = 0.130
CONTROL_PANEL_HEIGHT = 0.080
CONTROL_PANEL_THICKNESS = 0.004
CONTROL_PANEL_X = 0.295
CONTROL_PANEL_Y = CANOPY_DEPTH / 2.0 - CONTROL_PANEL_THICKNESS / 2.0
CONTROL_PANEL_Z = 0.110

BUTTON_HOLE_SIZE = 0.022
BUTTON_CAP_SIZE = 0.020
BUTTON_CAP_DEPTH = 0.003
BUTTON_STEM_DEPTH = 0.022
BUTTON_TRAVEL = 0.0035
BUTTON_COLUMN_XS = (-0.032, 0.000, 0.032)
BUTTON_ROW_ZS = (0.016, -0.016)

CHIMNEY_OUTER_WIDTH = 0.320
CHIMNEY_OUTER_DEPTH = 0.260
CHIMNEY_WALL = 0.018
CHIMNEY_HEIGHT = 0.700

BUTTON_LAYOUT = (
    ("button_top_left", BUTTON_COLUMN_XS[0], BUTTON_ROW_ZS[0]),
    ("button_top_center", BUTTON_COLUMN_XS[1], BUTTON_ROW_ZS[0]),
    ("button_top_right", BUTTON_COLUMN_XS[2], BUTTON_ROW_ZS[0]),
    ("button_bottom_left", BUTTON_COLUMN_XS[0], BUTTON_ROW_ZS[1]),
    ("button_bottom_center", BUTTON_COLUMN_XS[1], BUTTON_ROW_ZS[1]),
    ("button_bottom_right", BUTTON_COLUMN_XS[2], BUTTON_ROW_ZS[1]),
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _control_panel_mesh():
    hole_profiles = []
    square = _rect_profile(BUTTON_HOLE_SIZE, BUTTON_HOLE_SIZE)
    for row_z in BUTTON_ROW_ZS:
        for col_x in BUTTON_COLUMN_XS:
            hole_profiles.append(_offset_profile(square, dx=col_x, dy=row_z))
    geometry = ExtrudeWithHolesGeometry(
        _rect_profile(CONTROL_PANEL_WIDTH, CONTROL_PANEL_HEIGHT),
        hole_profiles,
        CONTROL_PANEL_THICKNESS,
        center=True,
    )
    return mesh_from_geometry(
        geometry.rotate_x(-math.pi / 2.0),
        ASSETS.asset_root / "range_hood_control_panel.obj",
    )


def _chimney_shell_mesh():
    inner_width = CHIMNEY_OUTER_WIDTH - 2.0 * CHIMNEY_WALL
    inner_depth = CHIMNEY_OUTER_DEPTH - 2.0 * CHIMNEY_WALL
    geometry = ExtrudeWithHolesGeometry(
        _rect_profile(CHIMNEY_OUTER_WIDTH, CHIMNEY_OUTER_DEPTH),
        [_rect_profile(inner_width, inner_depth)],
        CHIMNEY_HEIGHT,
        center=False,
    )
    return mesh_from_geometry(
        geometry,
        ASSETS.asset_root / "range_hood_chimney_shell.obj",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.77, 0.78, 0.80, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))

    opening_x_min = CONTROL_PANEL_X - CONTROL_PANEL_WIDTH * 0.5
    opening_x_max = CONTROL_PANEL_X + CONTROL_PANEL_WIDTH * 0.5
    opening_z_min = CONTROL_PANEL_Z - CONTROL_PANEL_HEIGHT * 0.5
    opening_z_max = CONTROL_PANEL_Z + CONTROL_PANEL_HEIGHT * 0.5
    front_y_center = CANOPY_DEPTH * 0.5 - SHELL_THICKNESS * 0.5

    canopy = model.part("canopy")
    canopy.visual(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT - TOP_THICKNESS * 0.5)),
        material=stainless,
        name="top_panel",
    )
    canopy.visual(
        Box((SHELL_THICKNESS, CANOPY_DEPTH, WALL_HEIGHT)),
        origin=Origin(
            xyz=((CANOPY_WIDTH - SHELL_THICKNESS) * 0.5, 0.0, WALL_HEIGHT * 0.5)
        ),
        material=stainless,
        name="right_wall",
    )
    canopy.visual(
        Box((SHELL_THICKNESS, CANOPY_DEPTH, WALL_HEIGHT)),
        origin=Origin(
            xyz=(-(CANOPY_WIDTH - SHELL_THICKNESS) * 0.5, 0.0, WALL_HEIGHT * 0.5)
        ),
        material=stainless,
        name="left_wall",
    )
    canopy.visual(
        Box((CANOPY_WIDTH - 2.0 * SHELL_THICKNESS, SHELL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -(CANOPY_DEPTH - SHELL_THICKNESS) * 0.5, WALL_HEIGHT * 0.5)
        ),
        material=stainless,
        name="back_wall",
    )
    canopy.visual(
        Box((opening_x_min + CANOPY_WIDTH * 0.5, SHELL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                -CANOPY_WIDTH * 0.5 + (opening_x_min + CANOPY_WIDTH * 0.5) * 0.5,
                front_y_center,
                WALL_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="front_left_frame",
    )
    canopy.visual(
        Box((CANOPY_WIDTH * 0.5 - opening_x_max, SHELL_THICKNESS, WALL_HEIGHT)),
        origin=Origin(
            xyz=(
                opening_x_max + (CANOPY_WIDTH * 0.5 - opening_x_max) * 0.5,
                front_y_center,
                WALL_HEIGHT * 0.5,
            )
        ),
        material=stainless,
        name="front_right_frame",
    )
    canopy.visual(
        Box((CONTROL_PANEL_WIDTH, SHELL_THICKNESS, CANOPY_HEIGHT - opening_z_max)),
        origin=Origin(
            xyz=(
                CONTROL_PANEL_X,
                front_y_center,
                opening_z_max + (CANOPY_HEIGHT - opening_z_max) * 0.5,
            )
        ),
        material=stainless,
        name="front_top_frame",
    )
    canopy.visual(
        Box((CONTROL_PANEL_WIDTH, SHELL_THICKNESS, opening_z_min)),
        origin=Origin(
            xyz=(
                CONTROL_PANEL_X,
                front_y_center,
                opening_z_min * 0.5,
            )
        ),
        material=stainless,
        name="front_bottom_frame",
    )
    canopy.visual(
        Box((0.180, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT - 0.027)),
        material=graphite,
        name="chimney_mount_pad",
    )
    canopy.inertial = Inertial.from_geometry(
        Box((CANOPY_WIDTH, CANOPY_DEPTH, CANOPY_HEIGHT)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT * 0.5)),
    )

    chimney_stack = model.part("chimney_stack")
    chimney_stack.visual(
        _chimney_shell_mesh(),
        material=stainless,
        name="chimney_shell",
    )
    chimney_stack.inertial = Inertial.from_geometry(
        Box((CHIMNEY_OUTER_WIDTH, CHIMNEY_OUTER_DEPTH, CHIMNEY_HEIGHT)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, CHIMNEY_HEIGHT * 0.5)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        _control_panel_mesh(),
        material=satin_black,
        name="control_plate",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((CONTROL_PANEL_WIDTH, CONTROL_PANEL_THICKNESS, CONTROL_PANEL_HEIGHT)),
        mass=0.35,
        origin=Origin(),
    )

    model.articulation(
        "canopy_to_chimney_stack",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney_stack,
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT)),
    )
    model.articulation(
        "canopy_to_control_panel",
        ArticulationType.FIXED,
        parent=canopy,
        child=control_panel,
        origin=Origin(xyz=(CONTROL_PANEL_X, CONTROL_PANEL_Y, CONTROL_PANEL_Z)),
    )

    for button_name, x_offset, z_offset in BUTTON_LAYOUT:
        button = model.part(button_name)
        button.visual(
            Box((BUTTON_CAP_SIZE, BUTTON_CAP_DEPTH, BUTTON_CAP_SIZE)),
            origin=Origin(xyz=(0.0, BUTTON_CAP_DEPTH * 0.5, 0.0)),
            material=charcoal,
            name="cap",
        )
        button.visual(
            Box((BUTTON_HOLE_SIZE, BUTTON_STEM_DEPTH, BUTTON_HOLE_SIZE)),
            origin=Origin(xyz=(0.0, -BUTTON_STEM_DEPTH * 0.5, 0.0)),
            material=graphite,
            name="stem",
        )
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_HOLE_SIZE, BUTTON_STEM_DEPTH + BUTTON_CAP_DEPTH, BUTTON_HOLE_SIZE)),
            mass=0.025,
            origin=Origin(
                xyz=(0.0, -(BUTTON_STEM_DEPTH - BUTTON_CAP_DEPTH) * 0.5, 0.0)
            ),
        )
        model.articulation(
            f"control_panel_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(
                xyz=(x_offset, CONTROL_PANEL_THICKNESS * 0.5, z_offset)
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.04,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    canopy = object_model.get_part("canopy")
    chimney_stack = object_model.get_part("chimney_stack")
    control_panel = object_model.get_part("control_panel")
    chimney_joint = object_model.get_articulation("canopy_to_chimney_stack")
    control_panel_joint = object_model.get_articulation("canopy_to_control_panel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "non_button_articulations_fixed",
        chimney_joint.articulation_type == ArticulationType.FIXED
        and control_panel_joint.articulation_type == ArticulationType.FIXED,
        "Chimney stack and control panel should be rigidly mounted.",
    )
    ctx.expect_contact(chimney_stack, canopy)
    ctx.expect_contact(control_panel, canopy)
    ctx.expect_origin_distance(
        chimney_stack,
        canopy,
        axes="xy",
        max_dist=0.001,
        name="chimney_centered_on_canopy",
    )
    ctx.expect_overlap(
        chimney_stack,
        canopy,
        axes="xy",
        min_overlap=0.20,
        name="chimney_over_canopy_footprint",
    )

    canopy_aabb = ctx.part_world_aabb(canopy)
    chimney_aabb = ctx.part_world_aabb(chimney_stack)
    control_panel_pos = ctx.part_world_position(control_panel)
    if canopy_aabb is not None:
        canopy_width = canopy_aabb[1][0] - canopy_aabb[0][0]
        canopy_depth = canopy_aabb[1][1] - canopy_aabb[0][1]
        canopy_height = canopy_aabb[1][2] - canopy_aabb[0][2]
        ctx.check(
            "canopy_proportions_realistic",
            0.84 <= canopy_width <= 0.96
            and 0.46 <= canopy_depth <= 0.54
            and 0.18 <= canopy_height <= 0.22,
            (
                f"canopy dims were {(canopy_width, canopy_depth, canopy_height)} "
                "but should read as a full-size kitchen hood."
            ),
        )
    if chimney_aabb is not None:
        total_height = chimney_aabb[1][2] - (canopy_aabb[0][2] if canopy_aabb is not None else 0.0)
        ctx.check(
            "chimney_height_realistic",
            0.82 <= total_height <= 0.98,
            f"overall hood height was {total_height:.3f} m.",
        )
    if control_panel_pos is not None:
        ctx.check(
            "control_panel_front_right_location",
            control_panel_pos[0] > 0.24 and control_panel_pos[1] > 0.24,
            f"control panel position was {control_panel_pos}.",
        )

    button_positions: list[tuple[float, float, float]] = []
    for button_name, x_offset, z_offset in BUTTON_LAYOUT:
        button = object_model.get_part(button_name)
        joint = object_model.get_articulation(f"control_panel_to_{button_name}")
        limits = joint.motion_limits

        ctx.check(
            f"{joint.name}_is_prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            "Each push-button should be a prismatic plunger.",
        )
        ctx.check(
            f"{joint.name}_axis",
            tuple(joint.axis) == (0.0, -1.0, 0.0),
            f"Expected inward button travel along -Y, got {joint.axis}.",
        )
        ctx.check(
            f"{joint.name}_travel",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.003 <= limits.upper <= 0.004,
            f"Button travel should be a short plunger stroke, got {limits}.",
        )

        rest_pos = ctx.part_world_position(button)
        if rest_pos is not None:
            button_positions.append(rest_pos)
            expected = (
                CONTROL_PANEL_X + x_offset,
                CONTROL_PANEL_Y + CONTROL_PANEL_THICKNESS * 0.5,
                CONTROL_PANEL_Z + z_offset,
            )
            ctx.check(
                f"{button_name}_rest_pose_location",
                abs(rest_pos[0] - expected[0]) < 0.001
                and abs(rest_pos[1] - expected[1]) < 0.001
                and abs(rest_pos[2] - expected[2]) < 0.001,
                f"Expected {expected}, got {rest_pos}.",
            )

        ctx.expect_contact(button, control_panel, name=f"{button_name}_rest_contact")
        ctx.expect_within(
            button,
            control_panel,
            axes="xz",
            margin=0.001,
            name=f"{button_name}_within_control_panel_face",
        )

        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                pressed_pos = ctx.part_world_position(button)
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{button_name}_pressed_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{button_name}_pressed_no_floating")
                ctx.expect_contact(
                    button,
                    control_panel,
                    name=f"{button_name}_pressed_contact",
                )
                ctx.check(
                    f"{button_name}_presses_inward",
                    rest_pos is not None
                    and pressed_pos is not None
                    and pressed_pos[1] < rest_pos[1] - 0.003,
                    f"Rest {rest_pos}, pressed {pressed_pos}.",
                )

    if button_positions:
        unique_xs = sorted({round(position[0], 3) for position in button_positions})
        unique_zs = sorted({round(position[2], 3) for position in button_positions})
        ctx.check(
            "button_layout_has_three_columns",
            len(unique_xs) == 3,
            f"Expected 3 columns, got x positions {unique_xs}.",
        )
        ctx.check(
            "button_layout_has_two_rows",
            len(unique_zs) == 2,
            f"Expected 2 rows, got z positions {unique_zs}.",
        )
        ctx.check(
            "buttons_are_on_front_right",
            min(unique_xs) > 0.24 and max(unique_zs) > 0.10,
            f"Button positions were xs={unique_xs}, zs={unique_zs}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
