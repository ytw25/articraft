from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

HOOD_WIDTH = 0.76
HOOD_DEPTH = 0.50
HOOD_HEIGHT = 0.14
PANEL_THICKNESS = 0.012
BODY_HEIGHT = HOOD_HEIGHT - PANEL_THICKNESS
LIP_DEPTH = 0.022

BOTTOM_STRIP_HEIGHT = 0.020
BUTTON_HEIGHT = 0.018
UPPER_STRIP_HEIGHT = BODY_HEIGHT - BOTTOM_STRIP_HEIGHT - BUTTON_HEIGHT

INNER_WIDTH = HOOD_WIDTH - 2.0 * PANEL_THICKNESS
BUTTON_ARRAY_SPAN = 0.64
BUTTON_DIVIDER_WIDTH = 0.006
BUTTON_WIDTH = (BUTTON_ARRAY_SPAN - 4.0 * BUTTON_DIVIDER_WIDTH) / 5.0
BUTTON_DEPTH = 0.015
BUTTON_TRAVEL = 0.007
SIDE_FRAME_WIDTH = (INNER_WIDTH - BUTTON_ARRAY_SPAN) / 2.0
FILTER_THICKNESS = 0.006
CENTER_RIB_WIDTH = 0.020
FILTER_DEPTH = HOOD_DEPTH - PANEL_THICKNESS - LIP_DEPTH
FILTER_WIDTH = (INNER_WIDTH - CENTER_RIB_WIDTH) / 2.0
FILTER_SLOT_COUNT = 6
FILTER_SLOT_GAP = 0.010
FILTER_HOLE_MARGIN_X = 0.018
FILTER_HOLE_MARGIN_Y = 0.040

BUTTON_CENTER_Y = BUTTON_DEPTH / 2.0
BUTTON_CENTER_Z = BOTTOM_STRIP_HEIGHT + BUTTON_HEIGHT / 2.0
BUTTON_PITCH = BUTTON_WIDTH + BUTTON_DIVIDER_WIDTH
BUTTON_CENTER_XS = tuple(
    (-BUTTON_ARRAY_SPAN / 2.0) + (BUTTON_WIDTH / 2.0) + (index * BUTTON_PITCH)
    for index in range(5)
)
BUTTON_PART_NAMES = tuple(f"button_{index + 1}" for index in range(5))
BUTTON_JOINT_NAMES = tuple(f"{name}_press" for name in BUTTON_PART_NAMES)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_width = width / 2.0
    half_height = height / 2.0
    return [
        (-half_width, -half_height),
        (half_width, -half_height),
        (half_width, half_height),
        (-half_width, half_height),
    ]


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _slotted_filter_mesh(filename: str):
    usable_width = FILTER_WIDTH - (2.0 * FILTER_HOLE_MARGIN_X)
    slot_width = (usable_width - ((FILTER_SLOT_COUNT - 1) * FILTER_SLOT_GAP)) / FILTER_SLOT_COUNT
    slot_length = FILTER_DEPTH - (2.0 * FILTER_HOLE_MARGIN_Y)
    first_slot_center_x = (-usable_width / 2.0) + (slot_width / 2.0)

    hole_profiles = []
    for index in range(FILTER_SLOT_COUNT):
        slot_center_x = first_slot_center_x + index * (slot_width + FILTER_SLOT_GAP)
        hole_profiles.append(
            _translate_profile(
                _rect_profile(slot_width, slot_length),
                slot_center_x,
                0.0,
            )
        )

    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(FILTER_WIDTH, FILTER_DEPTH),
            hole_profiles,
            FILTER_THICKNESS,
            cap=True,
            center=True,
        ),
        ASSETS.asset_root / filename,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood", assets=ASSETS)

    shell_material = model.material(
        "brushed_steel",
        rgba=(0.80, 0.81, 0.83, 1.0),
    )
    button_material = model.material(
        "dark_button",
        rgba=(0.18, 0.19, 0.21, 1.0),
    )
    filter_material = model.material(
        "filter_aluminum",
        rgba=(0.64, 0.66, 0.69, 1.0),
    )

    shell = model.part("hood_shell")
    shell.visual(
        Box((HOOD_WIDTH, HOOD_DEPTH, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, HOOD_DEPTH / 2.0, BODY_HEIGHT + (PANEL_THICKNESS / 2.0))),
        material=shell_material,
        name="top_panel",
    )
    shell.visual(
        Box((PANEL_THICKNESS, HOOD_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                (-HOOD_WIDTH / 2.0) + (PANEL_THICKNESS / 2.0),
                HOOD_DEPTH / 2.0,
                BODY_HEIGHT / 2.0,
            )
        ),
        material=shell_material,
        name="left_side_panel",
    )
    shell.visual(
        Box((PANEL_THICKNESS, HOOD_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                (HOOD_WIDTH / 2.0) - (PANEL_THICKNESS / 2.0),
                HOOD_DEPTH / 2.0,
                BODY_HEIGHT / 2.0,
            )
        ),
        material=shell_material,
        name="right_side_panel",
    )
    shell.visual(
        Box((INNER_WIDTH, PANEL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                HOOD_DEPTH - (PANEL_THICKNESS / 2.0),
                BODY_HEIGHT / 2.0,
            )
        ),
        material=shell_material,
        name="back_panel",
    )
    shell.visual(
        Box((INNER_WIDTH, LIP_DEPTH, BOTTOM_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                LIP_DEPTH / 2.0,
                BOTTOM_STRIP_HEIGHT / 2.0,
            )
        ),
        material=shell_material,
        name="front_lower_strip",
    )
    shell.visual(
        Box((INNER_WIDTH, LIP_DEPTH, UPPER_STRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                LIP_DEPTH / 2.0,
                BOTTOM_STRIP_HEIGHT + BUTTON_HEIGHT + (UPPER_STRIP_HEIGHT / 2.0),
            )
        ),
        material=shell_material,
        name="front_upper_strip",
    )
    shell.visual(
        Box((SIDE_FRAME_WIDTH, LIP_DEPTH, BUTTON_HEIGHT)),
        origin=Origin(
            xyz=(
                (-INNER_WIDTH / 2.0) + (SIDE_FRAME_WIDTH / 2.0),
                LIP_DEPTH / 2.0,
                BUTTON_CENTER_Z,
            )
        ),
        material=shell_material,
        name="front_left_frame",
    )
    shell.visual(
        Box((SIDE_FRAME_WIDTH, LIP_DEPTH, BUTTON_HEIGHT)),
        origin=Origin(
            xyz=(
                (INNER_WIDTH / 2.0) - (SIDE_FRAME_WIDTH / 2.0),
                LIP_DEPTH / 2.0,
                BUTTON_CENTER_Z,
            )
        ),
        material=shell_material,
        name="front_right_frame",
    )
    shell.visual(
        Box((CENTER_RIB_WIDTH, FILTER_DEPTH, FILTER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                LIP_DEPTH + (FILTER_DEPTH / 2.0),
                FILTER_THICKNESS / 2.0,
            )
        ),
        material=shell_material,
        name="center_filter_support",
    )
    shell.visual(
        _slotted_filter_mesh("hood_left_filter.obj"),
        origin=Origin(
            xyz=(
                (-INNER_WIDTH / 2.0) + (FILTER_WIDTH / 2.0),
                LIP_DEPTH + (FILTER_DEPTH / 2.0),
                FILTER_THICKNESS / 2.0,
            )
        ),
        material=filter_material,
        name="left_filter",
    )
    shell.visual(
        _slotted_filter_mesh("hood_right_filter.obj"),
        origin=Origin(
            xyz=(
                (INNER_WIDTH / 2.0) - (FILTER_WIDTH / 2.0),
                LIP_DEPTH + (FILTER_DEPTH / 2.0),
                FILTER_THICKNESS / 2.0,
            )
        ),
        material=filter_material,
        name="right_filter",
    )

    for index in range(4):
        divider_center_x = (
            (-BUTTON_ARRAY_SPAN / 2.0)
            + BUTTON_WIDTH
            + (BUTTON_DIVIDER_WIDTH / 2.0)
            + (index * BUTTON_PITCH)
        )
        shell.visual(
            Box((BUTTON_DIVIDER_WIDTH, LIP_DEPTH, BUTTON_HEIGHT)),
            origin=Origin(
                xyz=(
                    divider_center_x,
                    LIP_DEPTH / 2.0,
                    BUTTON_CENTER_Z,
                )
            ),
            material=shell_material,
            name=f"button_divider_{index + 1}",
        )

    for button_name, button_center_x in zip(BUTTON_PART_NAMES, BUTTON_CENTER_XS):
        button = model.part(button_name)
        button.visual(
            Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_HEIGHT)),
            material=button_material,
            name="button_cap",
        )
        model.articulation(
            f"{button_name}_press",
            ArticulationType.PRISMATIC,
            parent=shell,
            child=button,
            origin=Origin(
                xyz=(
                    button_center_x,
                    BUTTON_CENTER_Y,
                    BUTTON_CENTER_Z,
                )
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    shell = object_model.get_part("hood_shell")
    buttons = [object_model.get_part(name) for name in BUTTON_PART_NAMES]
    joints = [object_model.get_articulation(name) for name in BUTTON_JOINT_NAMES]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    shell_aabb = ctx.part_world_aabb(shell)
    if shell_aabb is not None:
        shell_dims = tuple(
            shell_aabb[1][axis] - shell_aabb[0][axis]
            for axis in range(3)
        )
        ctx.check(
            "hood_realistic_proportions",
            0.72 <= shell_dims[0] <= 0.80
            and 0.48 <= shell_dims[1] <= 0.52
            and 0.13 <= shell_dims[2] <= 0.15,
            f"unexpected hood dimensions {shell_dims}",
        )
    else:
        ctx.fail("hood_shell_aabb_resolved", "hood shell AABB did not resolve")

    left_filter_aabb = ctx.part_element_world_aabb(shell, elem="left_filter")
    right_filter_aabb = ctx.part_element_world_aabb(shell, elem="right_filter")
    if left_filter_aabb is not None and right_filter_aabb is not None:
        combined_filter_span = right_filter_aabb[1][0] - left_filter_aabb[0][0]
        filter_depth = left_filter_aabb[1][1] - left_filter_aabb[0][1]
        ctx.check(
            "underside_filters_read_as_prominent_intake",
            combined_filter_span >= (INNER_WIDTH - 0.03) and filter_depth >= 0.44,
            (
                "underside filters should occupy most of the bottom opening, "
                f"got span={combined_filter_span} m depth={filter_depth} m"
            ),
        )
    else:
        ctx.fail("underside_filters_resolved", "underside filter visuals did not resolve")

    rest_positions: dict[str, tuple[float, float, float] | None] = {}
    for button, joint in zip(buttons, joints):
        limits = joint.motion_limits
        axis_ok = tuple(round(value, 6) for value in joint.axis) == (0.0, 1.0, 0.0)
        limits_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower) <= 1e-9
            and abs(limits.upper - BUTTON_TRAVEL) <= 1e-9
        )
        ctx.check(
            f"{joint.name}_type",
            joint.articulation_type == ArticulationType.PRISMATIC,
            f"{joint.name} should be prismatic",
        )
        ctx.check(
            f"{joint.name}_axis",
            axis_ok,
            f"{joint.name} axis was {joint.axis}, expected (0, 1, 0)",
        )
        ctx.check(
            f"{joint.name}_limits",
            limits_ok,
            f"{joint.name} limits should be [0.0, {BUTTON_TRAVEL}]",
        )

        rest_position = ctx.part_world_position(button)
        rest_positions[button.name] = rest_position
        ctx.check(
            f"{button.name}_rest_position_resolves",
            rest_position is not None,
            f"{button.name} world position did not resolve",
        )
        ctx.expect_contact(button, shell, name=f"{button.name}_mounted_at_rest")
        ctx.expect_within(button, shell, axes="xz", margin=0.0, name=f"{button.name}_within_shell_at_rest")

        if limits is None or limits.lower is None or limits.upper is None:
            continue

        with ctx.pose({joint: limits.lower}):
            ctx.expect_contact(button, shell, name=f"{button.name}_mounted_at_lower_limit")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{button.name}_lower_limit_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{button.name}_lower_limit_no_floating")

        with ctx.pose({joint: limits.upper}):
            ctx.expect_contact(button, shell, name=f"{button.name}_mounted_at_upper_limit")
            ctx.expect_within(button, shell, axes="xz", margin=0.0, name=f"{button.name}_within_shell_at_upper_limit")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{button.name}_upper_limit_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{button.name}_upper_limit_no_floating")

            pressed_position = ctx.part_world_position(button)
            motion_ok = (
                rest_position is not None
                and pressed_position is not None
                and abs(pressed_position[0] - rest_position[0]) <= 1e-6
                and abs(pressed_position[2] - rest_position[2]) <= 1e-6
                and abs((pressed_position[1] - rest_position[1]) - BUTTON_TRAVEL) <= 1e-6
            )
            ctx.check(
                f"{button.name}_short_inward_travel",
                motion_ok,
                f"{button.name} should move inward by {BUTTON_TRAVEL} m only along +Y",
            )

    leftmost = rest_positions[BUTTON_PART_NAMES[0]]
    rightmost = rest_positions[BUTTON_PART_NAMES[-1]]
    if leftmost is not None and rightmost is not None:
        span = (rightmost[0] - leftmost[0]) + BUTTON_WIDTH
        ctx.check(
            "button_bank_spans_most_of_front_lip",
            span >= (0.84 * INNER_WIDTH),
            f"button span {span} m was too short for inner width {INNER_WIDTH} m",
        )
    else:
        ctx.fail("button_bank_positions_resolved", "button bank positions did not resolve")

    with ctx.pose({joint: BUTTON_TRAVEL for joint in joints}):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_buttons_pressed_no_overlap")
        ctx.fail_if_isolated_parts(name="all_buttons_pressed_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
