from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _shift_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_top_deck_mesh(
    *,
    width: float,
    depth: float,
    thickness: float,
    opening_width: float,
    opening_depth: float,
    opening_center_y: float,
):
    outer = rounded_rect_profile(width, depth, radius=0.024, corner_segments=8)
    opening = _shift_profile(
        rounded_rect_profile(opening_width, opening_depth, radius=0.050, corner_segments=10),
        dy=opening_center_y,
    )
    return ExtrudeWithHolesGeometry(
        outer,
        [opening],
        height=thickness,
        center=True,
    )


def _build_basket_shell_mesh(*, height: float):
    outer_profile = [
        (0.040, 0.000),
        (0.115, 0.030),
        (0.205, 0.095),
        (0.228, 0.210),
        (0.236, height - 0.028),
        (0.244, height),
    ]
    inner_profile = [
        (0.000, 0.014),
        (0.092, 0.040),
        (0.188, 0.100),
        (0.214, 0.210),
        (0.222, height - 0.010),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=56,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_load_washing_machine")

    cabinet_width = 0.68
    cabinet_depth = 0.71
    cabinet_height = 0.92
    wall = 0.028
    top_thickness = 0.030
    opening_width = 0.53
    opening_depth = 0.44
    opening_center_y = -0.035
    hinge_y = 0.190
    lid_width = 0.62
    lid_depth = 0.56
    lid_thickness = 0.022
    basket_height = 0.58
    basket_joint_z = 0.22

    enamel_white = model.material("enamel_white", rgba=(0.95, 0.96, 0.97, 1.0))
    console_gray = model.material("console_gray", rgba=(0.83, 0.85, 0.88, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    basket_steel = model.material("basket_steel", rgba=(0.78, 0.81, 0.84, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.33, 0.35, 0.38, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((wall, cabinet_depth, cabinet_height - top_thickness)),
        origin=Origin(
            xyz=(
                -(cabinet_width * 0.5) + (wall * 0.5),
                0.0,
                (cabinet_height - top_thickness) * 0.5,
            )
        ),
        material=enamel_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((wall, cabinet_depth, cabinet_height - top_thickness)),
        origin=Origin(
            xyz=(
                (cabinet_width * 0.5) - (wall * 0.5),
                0.0,
                (cabinet_height - top_thickness) * 0.5,
            )
        ),
        material=enamel_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((cabinet_width - (2.0 * wall), wall, cabinet_height - top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -(cabinet_depth * 0.5) + (wall * 0.5),
                (cabinet_height - top_thickness) * 0.5,
            )
        ),
        material=enamel_white,
        name="front_panel",
    )
    cabinet.visual(
        Box((cabinet_width - (2.0 * wall), wall, cabinet_height - top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                (cabinet_depth * 0.5) - (wall * 0.5),
                (cabinet_height - top_thickness) * 0.5,
            )
        ),
        material=enamel_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((cabinet_width - (2.0 * wall), cabinet_depth - (2.0 * wall), 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_trim,
        name="base_plinth",
    )
    cabinet.visual(
        mesh_from_geometry(
            _build_top_deck_mesh(
                width=cabinet_width,
                depth=cabinet_depth,
                thickness=top_thickness,
                opening_width=opening_width,
                opening_depth=opening_depth,
                opening_center_y=opening_center_y,
            ),
            "washer_top_deck",
        ),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - (top_thickness * 0.5))),
        material=enamel_white,
        name="top_deck",
    )
    cabinet.visual(
        Box((0.62, 0.020, 0.090)),
        origin=Origin(xyz=(0.0, 0.255, 0.965)),
        material=console_gray,
        name="console_face",
    )
    cabinet.visual(
        Box((0.64, 0.100, 0.130)),
        origin=Origin(xyz=(0.0, 0.315, 0.985)),
        material=console_gray,
        name="console_housing",
    )
    cabinet.visual(
        Box((0.58, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.266, 0.930)),
        material=dark_trim,
        name="console_lower_trim",
    )
    cabinet.visual(
        Cylinder(radius=0.090, length=0.140),
        origin=Origin(xyz=(0.0, opening_center_y, 0.150)),
        material=hub_gray,
        name="drive_pedestal",
    )
    cabinet.visual(
        Cylinder(radius=0.060, length=0.040),
        origin=Origin(xyz=(0.0, opening_center_y, 0.200)),
        material=hub_gray,
        name="drive_cap",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, 1.08)),
        mass=54.0,
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, -(lid_depth * 0.5), lid_thickness * 0.5)),
        material=enamel_white,
        name="lid_panel",
    )
    lid.visual(
        Box((0.56, 0.028, 0.046)),
        origin=Origin(xyz=(0.0, -0.573, 0.007)),
        material=console_gray,
        name="lid_front_edge",
    )
    lid.visual(
        Box((0.18, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, -0.490, 0.029)),
        material=dark_trim,
        name="lid_handle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, 0.050)),
        mass=4.8,
        origin=Origin(xyz=(0.0, -(lid_depth * 0.5), 0.010)),
    )

    basket = model.part("wash_basket")
    basket.visual(
        mesh_from_geometry(
            _build_basket_shell_mesh(height=basket_height),
            "washer_basket_shell",
        ),
        material=basket_steel,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=hub_gray,
        name="hub_pad",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.235, length=basket_height),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, basket_height * 0.5)),
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="knob_shaft",
    )
    timer_knob.visual(
        Cylinder(radius=0.033, length=0.028),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_silver,
        name="knob_body",
    )
    timer_knob.visual(
        Box((0.002, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, -0.030, 0.029)),
        material=dark_trim,
        name="knob_indicator",
    )
    timer_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=0.050),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, cabinet_height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(0.0, opening_center_y, basket_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=16.0,
        ),
    )
    model.articulation(
        "timer_knob_turn",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=timer_knob,
        origin=Origin(xyz=(0.180, 0.245, 0.975)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=5.0,
            lower=0.0,
            upper=5.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("wash_basket")
    timer_knob = object_model.get_part("timer_knob")
    lid_hinge = object_model.get_articulation("lid_hinge")
    basket_spin = object_model.get_articulation("basket_spin")
    knob_turn = object_model.get_articulation("timer_knob_turn")

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

    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="top_deck",
        max_gap=0.001,
        max_penetration=0.0,
        name="lid sits flush on the deck when closed",
    )
    ctx.expect_overlap(
        lid,
        cabinet,
        axes="xy",
        elem_a="lid_panel",
        elem_b="top_deck",
        min_overlap=0.50,
        name="lid spans the cabinet top opening footprint",
    )
    ctx.expect_gap(
        cabinet,
        basket,
        axis="z",
        positive_elem="top_deck",
        negative_elem="basket_shell",
        min_gap=0.05,
        max_gap=0.14,
        name="basket rim stays recessed below the top opening",
    )
    ctx.expect_contact(
        basket,
        cabinet,
        elem_a="hub_pad",
        elem_b="drive_cap",
        name="basket is physically supported on the drive cap",
    )
    ctx.expect_contact(
        timer_knob,
        cabinet,
        elem_a="knob_shaft",
        elem_b="console_face",
        name="timer knob is mounted on the control panel face",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="lid_front_edge")
    with ctx.pose({lid_hinge: math.radians(72.0)}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open lid pose stays clear")
        open_front = ctx.part_element_world_aabb(lid, elem="lid_front_edge")

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[0][2] + 0.20,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )
    ctx.check(
        "basket uses a vertical continuous spin joint",
        tuple(round(v, 6) for v in basket_spin.axis) == (0.0, 0.0, 1.0)
        and basket_spin.motion_limits is not None
        and basket_spin.motion_limits.lower is None
        and basket_spin.motion_limits.upper is None,
        details=f"axis={basket_spin.axis}, limits={basket_spin.motion_limits}",
    )
    ctx.check(
        "timer knob uses a forward-facing rotary joint with realistic travel",
        tuple(round(v, 6) for v in knob_turn.axis) == (0.0, -1.0, 0.0)
        and knob_turn.motion_limits is not None
        and knob_turn.motion_limits.lower is not None
        and knob_turn.motion_limits.upper is not None
        and knob_turn.motion_limits.upper - knob_turn.motion_limits.lower >= 4.5,
        details=f"axis={knob_turn.axis}, limits={knob_turn.motion_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
