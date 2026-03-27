from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _side_shell_mesh(
    *,
    filename: str,
    x_outer: float,
    x_inner: float,
    canopy_depth: float,
    canopy_height: float,
    lower_shell_bottom: float,
):
    profile = [
        (-canopy_depth / 2.0, lower_shell_bottom),
        (-canopy_depth / 2.0, canopy_height),
        (0.05, canopy_height),
        (canopy_depth / 2.0, 0.102),
        (canopy_depth / 2.0, 0.045),
        (-0.02, lower_shell_bottom),
    ]
    outer_section = [(x_outer, y, z) for y, z in profile]
    inner_section = [(x_inner, y, z) for y, z in profile]
    return mesh_from_geometry(
        section_loft([outer_section, inner_section]),
        ASSETS.asset_root / filename,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.78, 0.79, 0.80, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    black = model.material("control_black", rgba=(0.06, 0.06, 0.07, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.38, 0.39, 0.41, 1.0))

    canopy_width = 0.60
    canopy_depth = 0.50
    canopy_height = 0.12
    sheet = 0.004
    lower_shell_height = 0.105
    lower_shell_bottom = canopy_height - lower_shell_height

    canopy = model.part("canopy")
    canopy.visual(
        Box((canopy_width, canopy_depth, sheet)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - sheet / 2.0)),
        material=stainless,
        name="top_shell",
    )
    canopy.visual(
        _side_shell_mesh(
            filename="right_cheek_shell.obj",
            x_outer=canopy_width / 2.0,
            x_inner=canopy_width / 2.0 - sheet,
            canopy_depth=canopy_depth,
            canopy_height=canopy_height,
            lower_shell_bottom=lower_shell_bottom,
        ),
        material=stainless,
        name="right_cheek_shell",
    )
    canopy.visual(
        _side_shell_mesh(
            filename="left_cheek_shell.obj",
            x_outer=-canopy_width / 2.0,
            x_inner=-canopy_width / 2.0 + sheet,
            canopy_depth=canopy_depth,
            canopy_height=canopy_height,
            lower_shell_bottom=lower_shell_bottom,
        ),
        material=stainless,
        name="left_cheek_shell",
    )
    canopy.visual(
        Box((canopy_width - 2.0 * sheet, sheet, lower_shell_height)),
        origin=Origin(
            xyz=(0.0, -canopy_depth / 2.0 + sheet / 2.0, lower_shell_height / 2.0 + lower_shell_bottom)
        ),
        material=stainless,
        name="rear_shell",
    )
    canopy.visual(
        Box((canopy_width - 2.0 * sheet, sheet, 0.040)),
        origin=Origin(xyz=(0.0, canopy_depth / 2.0 - sheet / 2.0, 0.100)),
        material=stainless,
        name="front_fascia",
    )
    canopy.visual(
        Box((canopy_width - sheet, 0.38, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=filter_gray,
        name="underside_filter",
    )

    pod_width = 0.150
    pod_depth = 0.038
    pod_height = 0.030
    pod_center_x = -0.195
    pod_center_y = 0.239
    pod_center_z = 0.086

    canopy.visual(
        Box((pod_width, pod_depth, sheet)),
        origin=Origin(xyz=(pod_center_x, pod_center_y, pod_center_z + pod_height / 2.0 - sheet / 2.0)),
        material=charcoal,
        name="button_pod_top",
    )
    canopy.visual(
        Box((pod_width, pod_depth, sheet)),
        origin=Origin(xyz=(pod_center_x, pod_center_y, pod_center_z - pod_height / 2.0 + sheet / 2.0)),
        material=charcoal,
        name="button_pod_bottom",
    )
    canopy.visual(
        Box((sheet, pod_depth, pod_height)),
        origin=Origin(
            xyz=(pod_center_x - pod_width / 2.0 + sheet / 2.0, pod_center_y, pod_center_z)
        ),
        material=charcoal,
        name="button_pod_left",
    )
    canopy.visual(
        Box((sheet, pod_depth, pod_height)),
        origin=Origin(
            xyz=(pod_center_x + pod_width / 2.0 - sheet / 2.0, pod_center_y, pod_center_z)
        ),
        material=charcoal,
        name="button_pod_right",
    )
    canopy.visual(
        Box((pod_width - 2.0 * sheet, sheet, pod_height)),
        origin=Origin(
            xyz=(pod_center_x, pod_center_y - pod_depth / 2.0 + sheet / 2.0, pod_center_z)
        ),
        material=charcoal,
        name="button_pod_back",
    )

    button_radius = 0.009
    button_diameter = button_radius * 2.0
    button_length = 0.022
    button_travel = 0.008
    guide_wall = 0.004
    guide_length = 0.026
    front_open_y = pod_center_y + pod_depth / 2.0
    guide_center_y = front_open_y - guide_length / 2.0
    button_z = 0.086
    button_positions = {
        "left_button": -0.225,
        "right_button": -0.170,
    }
    guide_outer = button_diameter + 2.0 * guide_wall
    guide_offset = button_radius + guide_wall / 2.0

    for button_name, button_x in button_positions.items():
        guide_prefix = f"{button_name}_guide"
        canopy.visual(
            Box((guide_outer, guide_length, guide_wall)),
            origin=Origin(xyz=(button_x, guide_center_y, button_z + guide_offset)),
            material=black,
            name=f"{guide_prefix}_top",
        )
        canopy.visual(
            Box((guide_outer, guide_length, guide_wall)),
            origin=Origin(xyz=(button_x, guide_center_y, button_z - guide_offset)),
            material=black,
            name=f"{guide_prefix}_bottom",
        )
        canopy.visual(
            Box((guide_wall, guide_length, button_diameter)),
            origin=Origin(xyz=(button_x - guide_offset, guide_center_y, button_z)),
            material=black,
            name=f"{guide_prefix}_left",
        )
        canopy.visual(
            Box((guide_wall, guide_length, button_diameter)),
            origin=Origin(xyz=(button_x + guide_offset, guide_center_y, button_z)),
            material=black,
            name=f"{guide_prefix}_right",
        )

    chimney = model.part("chimney")
    chimney_width = 0.26
    chimney_depth = 0.18
    chimney_height = 0.78
    chimney_sheet = 0.004
    chimney.visual(
        Box((chimney_width, chimney_sheet, chimney_height)),
        origin=Origin(xyz=(0.0, chimney_depth / 2.0 - chimney_sheet / 2.0, chimney_height / 2.0)),
        material=stainless,
        name="chimney_front",
    )
    chimney.visual(
        Box((chimney_width, chimney_sheet, chimney_height)),
        origin=Origin(xyz=(0.0, -chimney_depth / 2.0 + chimney_sheet / 2.0, chimney_height / 2.0)),
        material=stainless,
        name="chimney_back",
    )
    chimney.visual(
        Box((chimney_sheet, chimney_depth, chimney_height)),
        origin=Origin(xyz=(chimney_width / 2.0 - chimney_sheet / 2.0, 0.0, chimney_height / 2.0)),
        material=stainless,
        name="chimney_right",
    )
    chimney.visual(
        Box((chimney_sheet, chimney_depth, chimney_height)),
        origin=Origin(xyz=(-chimney_width / 2.0 + chimney_sheet / 2.0, 0.0, chimney_height / 2.0)),
        material=stainless,
        name="chimney_left",
    )

    model.articulation(
        "canopy_to_chimney",
        ArticulationType.FIXED,
        parent=canopy,
        child=chimney,
        origin=Origin(xyz=(0.0, -0.100, canopy_height)),
    )

    knob_radius = 0.020
    knob_depth = 0.028
    knob_y = 0.205
    knob_zs = {
        "upper_knob": 0.092,
        "lower_knob": 0.040,
    }
    for knob_name, knob_z in knob_zs.items():
        knob = model.part(knob_name)
        knob.visual(
            Cylinder(radius=knob_radius, length=knob_depth),
            origin=Origin(xyz=(knob_depth / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name="body",
        )
        model.articulation(
            f"{knob_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=canopy,
            child=knob,
            origin=Origin(xyz=(canopy_width / 2.0, knob_y, knob_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=6.0),
        )

    for button_name, button_x in button_positions.items():
        button = model.part(button_name)
        button.visual(
            Cylinder(radius=button_radius, length=button_length),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=black,
            name="cap",
        )
        model.articulation(
            f"{button_name}_press",
            ArticulationType.PRISMATIC,
            parent=canopy,
            child=button,
            origin=Origin(xyz=(button_x, front_open_y, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=button_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
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

    canopy = object_model.get_part("canopy")
    chimney = object_model.get_part("chimney")
    upper_knob = object_model.get_part("upper_knob")
    lower_knob = object_model.get_part("lower_knob")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")

    canopy_to_chimney = object_model.get_articulation("canopy_to_chimney")
    upper_knob_spin = object_model.get_articulation("upper_knob_spin")
    lower_knob_spin = object_model.get_articulation("lower_knob_spin")
    left_button_press = object_model.get_articulation("left_button_press")
    right_button_press = object_model.get_articulation("right_button_press")

    def axis_is(actual: tuple[float, float, float], expected: tuple[float, float, float]) -> bool:
        return all(abs(a - b) < 1e-9 for a, b in zip(actual, expected))

    ctx.check(
        "chimney_fixed_joint",
        canopy_to_chimney.articulation_type == ArticulationType.FIXED,
        details="The chimney should be rigidly fixed to the canopy.",
    )
    ctx.check(
        "knob_joint_types",
        upper_knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and lower_knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="Both rotary knobs must use continuous rotation joints.",
    )
    ctx.check(
        "button_joint_types",
        left_button_press.articulation_type == ArticulationType.PRISMATIC
        and right_button_press.articulation_type == ArticulationType.PRISMATIC,
        details="Both push-buttons must use prismatic joints.",
    )
    ctx.check(
        "control_axes",
        axis_is(upper_knob_spin.axis, (1.0, 0.0, 0.0))
        and axis_is(lower_knob_spin.axis, (1.0, 0.0, 0.0))
        and axis_is(left_button_press.axis, (0.0, -1.0, 0.0))
        and axis_is(right_button_press.axis, (0.0, -1.0, 0.0)),
        details="Knobs should rotate about the right cheek normal and buttons should press inward along -Y.",
    )

    left_limits = left_button_press.motion_limits
    right_limits = right_button_press.motion_limits
    ctx.check(
        "button_press_travel",
        left_limits is not None
        and right_limits is not None
        and left_limits.lower == 0.0
        and right_limits.lower == 0.0
        and left_limits.upper == 0.008
        and right_limits.upper == 0.008,
        details="The two push-buttons should have a short 8 mm inward travel.",
    )

    ctx.expect_contact(chimney, canopy, contact_tol=1e-5, name="chimney_seated_on_canopy")
    ctx.expect_contact(upper_knob, canopy, contact_tol=1e-5, name="upper_knob_mounted")
    ctx.expect_contact(lower_knob, canopy, contact_tol=1e-5, name="lower_knob_mounted")
    ctx.expect_contact(left_button, canopy, contact_tol=1e-5, name="left_button_guided")
    ctx.expect_contact(right_button, canopy, contact_tol=1e-5, name="right_button_guided")

    ctx.expect_origin_gap(
        right_button,
        left_button,
        axis="x",
        min_gap=0.045,
        max_gap=0.060,
        name="buttons_horizontal_spacing",
    )
    ctx.expect_origin_gap(
        upper_knob,
        lower_knob,
        axis="z",
        min_gap=0.048,
        max_gap=0.056,
        name="knobs_vertical_spacing",
    )

    canopy_aabb = ctx.part_world_aabb(canopy)
    chimney_aabb = ctx.part_world_aabb(chimney)
    if canopy_aabb is not None and chimney_aabb is not None:
        canopy_dims = tuple(hi - lo for lo, hi in zip(canopy_aabb[0], canopy_aabb[1]))
        chimney_dims = tuple(hi - lo for lo, hi in zip(chimney_aabb[0], chimney_aabb[1]))
        ctx.check(
            "range_hood_proportions",
            0.58 <= canopy_dims[0] <= 0.62
            and 0.49 <= canopy_dims[1] <= 0.52
            and 0.10 <= canopy_dims[2] <= 0.13
            and 0.24 <= chimney_dims[0] <= 0.28
            and 0.16 <= chimney_dims[1] <= 0.20
            and 0.75 <= chimney_dims[2] <= 0.81
            and chimney_dims[2] > canopy_dims[2] * 6.5,
            details="The model should read as a compact canopy with a much taller rectangular chimney above it.",
        )

    upper_pos = ctx.part_world_position(upper_knob)
    lower_pos = ctx.part_world_position(lower_knob)
    left_pos = ctx.part_world_position(left_button)
    right_pos = ctx.part_world_position(right_button)
    if None not in (upper_pos, lower_pos, left_pos, right_pos):
        assert upper_pos is not None
        assert lower_pos is not None
        assert left_pos is not None
        assert right_pos is not None
        ctx.check(
            "knobs_on_right_front_cheek",
            upper_pos[0] > 0.29
            and lower_pos[0] > 0.29
            and upper_pos[1] > 0.18
            and lower_pos[1] > 0.18,
            details="The knobs should sit on the right-hand front cheek of the canopy.",
        )
        ctx.check(
            "buttons_on_left_front_lip",
            left_pos[0] < -0.19
            and right_pos[0] < -0.14
            and left_pos[1] > 0.25
            and right_pos[1] > 0.25
            and abs(left_pos[2] - right_pos[2]) < 0.002,
            details="The push-buttons should line up horizontally along the left front lip.",
        )

    if left_limits is not None and left_limits.upper is not None:
        left_rest = ctx.part_world_position(left_button)
        with ctx.pose({left_button_press: left_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="left_button_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name="left_button_pressed_no_floating")
            ctx.expect_contact(left_button, canopy, contact_tol=1e-5, name="left_button_stays_guided")
            left_pressed = ctx.part_world_position(left_button)
        if left_rest is not None and left_pressed is not None:
            ctx.check(
                "left_button_moves_inward",
                abs(left_rest[0] - left_pressed[0]) < 1e-6
                and abs(left_rest[2] - left_pressed[2]) < 1e-6
                and (left_rest[1] - left_pressed[1]) > 0.0075,
                details="The left push-button should translate inward along its short stroke.",
            )

    if right_limits is not None and right_limits.upper is not None:
        right_rest = ctx.part_world_position(right_button)
        with ctx.pose({right_button_press: right_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="right_button_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name="right_button_pressed_no_floating")
            ctx.expect_contact(right_button, canopy, contact_tol=1e-5, name="right_button_stays_guided")
            right_pressed = ctx.part_world_position(right_button)
        if right_rest is not None and right_pressed is not None:
            ctx.check(
                "right_button_moves_inward",
                abs(right_rest[0] - right_pressed[0]) < 1e-6
                and abs(right_rest[2] - right_pressed[2]) < 1e-6
                and (right_rest[1] - right_pressed[1]) > 0.0075,
                details="The right push-button should translate inward along its short stroke.",
            )

    with ctx.pose({upper_knob_spin: 1.3, lower_knob_spin: 4.2}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotary_controls_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="rotary_controls_rotated_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
