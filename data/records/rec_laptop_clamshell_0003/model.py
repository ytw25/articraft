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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_laptop", assets=ASSETS)

    body = model.material("body", rgba=(0.24, 0.26, 0.28, 1.0))
    bumper = model.material("bumper", rgba=(0.08, 0.09, 0.10, 1.0))
    deck_panel = model.material("deck_panel", rgba=(0.36, 0.38, 0.40, 1.0))
    bezel = model.material("bezel", rgba=(0.04, 0.04, 0.05, 1.0))
    screen = model.material("screen", rgba=(0.08, 0.12, 0.16, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.320, 0.245, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=body,
        name="bottom_plate",
    )
    base.visual(
        Box((0.020, 0.245, 0.028)),
        origin=Origin(xyz=(-0.150, 0.0, 0.020)),
        material=body,
        name="left_rail",
    )
    base.visual(
        Box((0.020, 0.245, 0.028)),
        origin=Origin(xyz=(0.150, 0.0, 0.020)),
        material=body,
        name="right_rail",
    )
    base.visual(
        Box((0.280, 0.020, 0.028)),
        origin=Origin(xyz=(0.0, 0.1125, 0.020)),
        material=body,
        name="front_wall",
    )
    base.visual(
        Box((0.090, 0.020, 0.020)),
        origin=Origin(xyz=(-0.105, -0.1125, 0.016)),
        material=body,
        name="rear_bridge_left",
    )
    base.visual(
        Box((0.090, 0.020, 0.020)),
        origin=Origin(xyz=(0.105, -0.1125, 0.016)),
        material=body,
        name="rear_bridge_right",
    )
    base.visual(
        Box((0.118, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.1135, 0.026)),
        material=body,
        name="rear_center_mount",
    )
    base.visual(
        Box((0.120, 0.012, 0.007)),
        origin=Origin(xyz=(0.0, -0.1005, 0.032)),
        material=body,
        name="hinge_support_pad",
    )
    base.visual(
        Box((0.280, 0.205, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=deck_panel,
        name="deck",
    )
    base.visual(
        Box((0.184, 0.076, 0.0015)),
        origin=Origin(xyz=(0.0, 0.012, 0.03175)),
        material=bezel,
        name="keyboard_plate",
    )
    base.visual(
        Box((0.082, 0.052, 0.0012)),
        origin=Origin(xyz=(0.0, 0.076, 0.0316)),
        material=bezel,
        name="touchpad_plate",
    )
    for name, x_pos, y_pos in (
        ("corner_front_left", -0.144, 0.1065),
        ("corner_front_right", 0.144, 0.1065),
        ("corner_rear_left", -0.144, -0.1065),
        ("corner_rear_right", 0.144, -0.1065),
    ):
        base.visual(
            Box((0.032, 0.032, 0.034)),
            origin=Origin(xyz=(x_pos, y_pos, 0.017)),
            material=bumper,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.006, length=0.272),
        origin=Origin(xyz=(0.0, -0.121, 0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bumper,
        name="hinge_spine",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.320, 0.245, 0.034)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    display = model.part("display")
    display.visual(
        Box((0.300, 0.228, 0.006)),
        origin=Origin(xyz=(0.0, 0.124, 0.0245)),
        material=body,
        name="back_plate",
    )
    display.visual(
        Box((0.016, 0.228, 0.018)),
        origin=Origin(xyz=(-0.142, 0.124, 0.0125)),
        material=body,
        name="left_wall",
    )
    display.visual(
        Box((0.016, 0.228, 0.018)),
        origin=Origin(xyz=(0.142, 0.124, 0.0125)),
        material=body,
        name="right_wall",
    )
    display.visual(
        Box((0.268, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.230, 0.0125)),
        material=body,
        name="top_wall",
    )
    display.visual(
        Box((0.268, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.022, 0.0125)),
        material=body,
        name="bottom_wall",
    )
    display.visual(
        Box((0.020, 0.180, 0.007)),
        origin=Origin(xyz=(-0.124, 0.124, 0.007)),
        material=bezel,
        name="left_bezel",
    )
    display.visual(
        Box((0.020, 0.180, 0.007)),
        origin=Origin(xyz=(0.124, 0.124, 0.007)),
        material=bezel,
        name="right_bezel",
    )
    display.visual(
        Box((0.228, 0.024, 0.007)),
        origin=Origin(xyz=(0.0, 0.046, 0.007)),
        material=bezel,
        name="bottom_bezel",
    )
    display.visual(
        Box((0.228, 0.018, 0.007)),
        origin=Origin(xyz=(0.0, 0.205, 0.007)),
        material=bezel,
        name="top_bezel",
    )
    display.visual(
        Box((0.228, 0.138, 0.011)),
        origin=Origin(xyz=(0.0, 0.127, 0.016)),
        material=screen,
        name="screen_module",
    )
    for name, x_pos, y_pos in (
        ("rear_left_guard", -0.136, 0.024),
        ("rear_right_guard", 0.136, 0.024),
        ("front_left_guard", -0.136, 0.224),
        ("front_right_guard", 0.136, 0.224),
    ):
        display.visual(
            Box((0.028, 0.028, 0.024)),
            origin=Origin(xyz=(x_pos, y_pos, 0.0155)),
            material=bumper,
            name=name,
        )
    display.inertial = Inertial.from_geometry(
        Box((0.300, 0.228, 0.024)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.124, 0.0155)),
    )

    model.articulation(
        "base_to_display",
        ArticulationType.REVOLUTE,
        parent=base,
        child=display,
        origin=Origin(xyz=(0.0, -0.121, 0.032)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(150.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    display = object_model.get_part("display")
    hinge = object_model.get_articulation("base_to_display")

    deck = base.get_visual("deck")
    hinge_support_pad = base.get_visual("hinge_support_pad")
    hinge_spine = base.get_visual("hinge_spine")
    back_plate = display.get_visual("back_plate")
    bottom_wall = display.get_visual("bottom_wall")
    bottom_bezel = display.get_visual("bottom_bezel")
    screen_module = display.get_visual("screen_module")

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

    ctx.check(
        "hinge_is_revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"expected revolute hinge, got {hinge.articulation_type}",
    )
    ctx.check(
        "hinge_axis_runs_left_to_right",
        tuple(hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected hinge axis (1, 0, 0), got {hinge.axis}",
    )
    ctx.check(
        "hinge_motion_matches_clamshell_range",
        math.isclose(hinge.motion_limits.lower, 0.0)
        and math.isclose(hinge.motion_limits.upper, math.radians(150.0)),
        details=(
            f"expected hinge limits [0, {math.radians(150.0):.6f}], "
            f"got [{hinge.motion_limits.lower}, {hinge.motion_limits.upper}]"
        ),
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_origin_distance(
            display,
            base,
            axes="x",
            max_dist=0.001,
            name="lid_stays_centered_on_base",
        )
        ctx.expect_overlap(
            display,
            base,
            axes="xy",
            min_overlap=0.220,
            name="closed_lid_covers_base_footprint",
        )
        ctx.expect_gap(
            display,
            base,
            axis="z",
            positive_elem=bottom_bezel,
            negative_elem=deck,
            min_gap=0.003,
            max_gap=0.008,
            name="closed_lid_sits_just_above_deck",
        )
        ctx.expect_contact(
            display,
            base,
            elem_a=bottom_wall,
            elem_b=hinge_support_pad,
            name="display_shell_seats_on_hinge_support_pad",
        )
        ctx.expect_gap(
            display,
            base,
            axis="y",
            positive_elem=bottom_wall,
            negative_elem=hinge_spine,
            min_gap=0.002,
            max_gap=0.006,
            name="display_shell_sits_tight_to_hinge_spine",
        )
        ctx.expect_within(
            display,
            display,
            axes="xy",
            inner_elem=screen_module,
            outer_elem=back_plate,
            margin=0.0,
            name="screen_stays_inside_display_housing",
        )

    with ctx.pose({hinge: math.radians(110.0)}):
        ctx.expect_gap(
            display,
            base,
            axis="z",
            positive_elem=screen_module,
            negative_elem=deck,
            min_gap=0.040,
            name="open_screen_rises_well_above_base",
        )
        ctx.expect_overlap(
            display,
            base,
            axes="x",
            min_overlap=0.280,
            name="open_screen_remains_width_aligned",
        )

    with ctx.pose({hinge: math.radians(150.0)}):
        ctx.expect_gap(
            display,
            base,
            axis="z",
            positive_elem=screen_module,
            negative_elem=deck,
            min_gap=0.010,
            name="full_open_screen_clears_base",
        )

    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="lid_clears_base_through_full_motion",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
