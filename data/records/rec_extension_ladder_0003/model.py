from __future__ import annotations

import os

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platform_extension_ladder")

    aluminum = model.material("aluminum", rgba=(0.75, 0.77, 0.79, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.92, 0.77, 0.14, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.22, 0.23, 0.25, 1.0))
    grating = model.material("grating", rgba=(0.69, 0.71, 0.73, 1.0))
    rubber = model.material("rubber", rgba=(0.13, 0.13, 0.14, 1.0))

    base_stile_x = 0.205
    fly_stile_x = 0.165
    fly_y = 0.032
    platform_half_width = 0.168
    platform_rear_y = 0.042
    platform_front_y = 0.310
    platform_depth = platform_front_y - platform_rear_y
    platform_z = 1.610
    platform_top_z = platform_z + 0.014
    guard_pivot_x = 0.188
    guard_receiver_x = 0.190
    step_half_width = 0.132

    def add_cylinder(
        part,
        *,
        name: str,
        center: tuple[float, float, float],
        radius: float,
        length: float,
        axis: str,
        material,
    ) -> None:
        if axis == "x":
            rpy = (0.0, math.pi / 2.0, 0.0)
        elif axis == "y":
            rpy = (math.pi / 2.0, 0.0, 0.0)
        else:
            rpy = (0.0, 0.0, 0.0)
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=center, rpy=rpy),
            material=material,
            name=name,
        )

    base = model.part("base_section")
    base.visual(
        Box((0.048, 0.028, 2.38)),
        origin=Origin(xyz=(-base_stile_x, 0.0, 1.19)),
        material=aluminum,
        name="base_left_stile",
    )
    base.visual(
        Box((0.048, 0.028, 2.38)),
        origin=Origin(xyz=(base_stile_x, 0.0, 1.19)),
        material=aluminum,
        name="base_right_stile",
    )
    for index, z in enumerate((0.24, 0.46, 0.68, 0.90, 1.12, 1.34, 1.56, 1.78, 2.00)):
        add_cylinder(
            base,
            name=f"base_rung_{index}",
            center=(0.0, 0.0, z),
            length=0.364,
            radius=0.0115,
            axis="x",
            material=steel,
        )
    base.visual(
        Box((0.060, 0.045, 0.032)),
        origin=Origin(xyz=(-base_stile_x, 0.0, 0.016)),
        material=rubber,
        name="left_foot_pad",
    )
    base.visual(
        Box((0.060, 0.045, 0.032)),
        origin=Origin(xyz=(base_stile_x, 0.0, 0.016)),
        material=rubber,
        name="right_foot_pad",
    )
    base.visual(
        Box((0.070, 0.020, 0.140)),
        origin=Origin(xyz=(-base_stile_x, 0.004, 2.23)),
        material=dark_hardware,
        name="left_top_guide_cap",
    )
    base.visual(
        Box((0.070, 0.020, 0.140)),
        origin=Origin(xyz=(base_stile_x, 0.004, 2.23)),
        material=dark_hardware,
        name="right_top_guide_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.50, 0.07, 2.38)),
        mass=12.5,
        origin=Origin(xyz=(0.0, 0.0, 1.19)),
    )

    fly = model.part("fly_section")
    fly.visual(
        Box((0.042, 0.024, 1.60)),
        origin=Origin(xyz=(-fly_stile_x, fly_y, 0.80)),
        material=aluminum,
        name="fly_left_stile",
    )
    fly.visual(
        Box((0.042, 0.024, 1.60)),
        origin=Origin(xyz=(fly_stile_x, fly_y, 0.80)),
        material=aluminum,
        name="fly_right_stile",
    )
    for index, z in enumerate((0.20, 0.40, 0.60, 0.80, 1.00, 1.20, 1.40)):
        add_cylinder(
            fly,
            name=f"fly_rung_{index}",
            center=(0.0, fly_y, z),
            length=0.290,
            radius=0.0105,
            axis="x",
            material=steel,
        )
    for side_name, x in (("left", -fly_stile_x), ("right", fly_stile_x)):
        fly.visual(
            Box((0.050, 0.006, 0.160)),
            origin=Origin(xyz=(x, 0.017, 0.32)),
            material=dark_hardware,
            name=f"{side_name}_lower_guide_shoe",
        )
        fly.visual(
            Box((0.050, 0.006, 0.160)),
            origin=Origin(xyz=(x, 0.017, 0.96)),
            material=dark_hardware,
            name=f"{side_name}_upper_guide_shoe",
        )

    fly.visual(
        Box((0.336, 0.026, 0.028)),
        origin=Origin(xyz=(0.0, platform_rear_y, platform_z)),
        material=aluminum,
        name="platform_rear_bar",
    )
    fly.visual(
        Box((0.336, 0.026, 0.028)),
        origin=Origin(xyz=(0.0, platform_front_y, platform_z)),
        material=aluminum,
        name="platform_front_bar",
    )
    fly.visual(
        Box((0.026, platform_depth, 0.028)),
        origin=Origin(xyz=(-platform_half_width, 0.176, platform_z)),
        material=aluminum,
        name="platform_left_side",
    )
    fly.visual(
        Box((0.026, platform_depth, 0.028)),
        origin=Origin(xyz=(platform_half_width, 0.176, platform_z)),
        material=aluminum,
        name="platform_right_side",
    )
    fly.visual(
        Box((0.300, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.070, 1.587)),
        material=aluminum,
        name="platform_rear_tie",
    )
    fly.visual(
        Box((0.300, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.252, 1.587)),
        material=aluminum,
        name="platform_front_tie",
    )
    for side_name, x in (("left", -0.145), ("right", 0.145)):
        fly.visual(
            Box((0.018, 0.252, 0.018)),
            origin=Origin(xyz=(x, 0.144, 1.521), rpy=(0.612, 0.0, 0.0)),
            material=aluminum,
            name=f"{side_name}_platform_brace",
        )
    for index, y in enumerate((0.070, 0.102, 0.134, 0.166, 0.198, 0.230, 0.262, 0.294)):
        fly.visual(
            Box((0.302, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, y, 1.583)),
            material=grating,
            name=f"grate_slat_{index}",
        )
    for index, x in enumerate((-0.082, 0.0, 0.082)):
        fly.visual(
            Box((0.010, 0.250, 0.006)),
            origin=Origin(xyz=(x, 0.176, 1.583)),
            material=grating,
            name=f"grate_runner_{index}",
        )

    fly.visual(
        Box((0.006, 0.024, 0.180)),
        origin=Origin(xyz=(-0.170, platform_rear_y, 1.714)),
        material=dark_hardware,
        name="left_hinge_cheek",
    )
    fly.visual(
        Box((0.006, 0.024, 0.180)),
        origin=Origin(xyz=(0.170, platform_rear_y, 1.714)),
        material=dark_hardware,
        name="right_hinge_cheek",
    )
    add_cylinder(
        fly,
        name="left_guard_pivot_pin",
        center=(-0.1655, platform_rear_y, 1.714),
        radius=0.0015,
        length=0.180,
        axis="z",
        material=steel,
    )
    add_cylinder(
        fly,
        name="right_guard_pivot_pin",
        center=(0.1655, platform_rear_y, 1.714),
        radius=0.0015,
        length=0.180,
        axis="z",
        material=steel,
    )
    fly.visual(
        Box((0.040, 0.018, 0.036)),
        origin=Origin(xyz=(-0.186, 0.322, 1.606)),
        material=dark_hardware,
        name="left_guard_receiver",
    )
    fly.visual(
        Box((0.040, 0.018, 0.036)),
        origin=Origin(xyz=(0.186, 0.322, 1.606)),
        material=dark_hardware,
        name="right_guard_receiver",
    )
    fly.visual(
        Box((0.024, 0.012, 0.032)),
        origin=Origin(xyz=(-step_half_width, 0.314, 1.628)),
        material=dark_hardware,
        name="left_step_hinge_cheek",
    )
    fly.visual(
        Box((0.024, 0.012, 0.032)),
        origin=Origin(xyz=(step_half_width, 0.314, 1.628)),
        material=dark_hardware,
        name="right_step_hinge_cheek",
    )
    fly.inertial = Inertial.from_geometry(
        Box((0.48, 0.38, 2.00)),
        mass=10.8,
        origin=Origin(xyz=(0.0, 0.17, 1.00)),
    )

    def add_guard_rail(part_name: str, inward_sign: float) -> None:
        rail = model.part(part_name)
        add_cylinder(
            rail,
            name="rear_post",
            center=(0.0, 0.0, 0.450),
            radius=0.009,
            length=0.900,
            axis="z",
            material=safety_yellow,
        )
        add_cylinder(
            rail,
            name="front_post",
            center=(0.0, 0.268, 0.450),
            radius=0.009,
            length=0.900,
            axis="z",
            material=safety_yellow,
        )
        add_cylinder(
            rail,
            name="top_rail",
            center=(0.0, 0.134, 0.882),
            radius=0.009,
            length=0.258,
            axis="y",
            material=safety_yellow,
        )
        add_cylinder(
            rail,
            name="mid_rail",
            center=(0.0, 0.134, 0.470),
            radius=0.007,
            length=0.250,
            axis="y",
            material=safety_yellow,
        )
        rail.visual(
            Box((0.006, 0.018, 0.160)),
            origin=Origin(xyz=(-0.012, 0.0, 0.100)),
            material=dark_hardware,
            name="left_side_hinge_shoe",
        )
        rail.visual(
            Box((0.006, 0.018, 0.160)),
            origin=Origin(xyz=(0.012, 0.0, 0.100)),
            material=dark_hardware,
            name="right_side_hinge_shoe",
        )
        rail.visual(
            Box((0.014, 0.016, 0.120)),
            origin=Origin(xyz=(0.012 * inward_sign, 0.268, 0.060)),
            material=dark_hardware,
            name="front_latch_lug",
        )
        rail.inertial = Inertial.from_geometry(
            Box((0.06, 0.30, 0.92)),
            mass=1.7,
            origin=Origin(xyz=(0.0, 0.134, 0.460)),
        )

    add_guard_rail("left_guard_rail", inward_sign=-1.0)
    add_guard_rail("right_guard_rail", inward_sign=1.0)

    step_bar = model.part("step_over_bar")
    step_bar.visual(
        Box((0.018, 0.008, 0.016)),
        origin=Origin(xyz=(-step_half_width, 0.004, 0.008)),
        material=dark_hardware,
        name="left_hinge_foot",
    )
    step_bar.visual(
        Box((0.018, 0.008, 0.016)),
        origin=Origin(xyz=(step_half_width, 0.004, 0.008)),
        material=dark_hardware,
        name="right_hinge_foot",
    )
    step_bar.visual(
        Box((0.014, 0.014, 0.216)),
        origin=Origin(xyz=(-step_half_width, 0.004, 0.124)),
        material=safety_yellow,
        name="left_side_bar",
    )
    step_bar.visual(
        Box((0.014, 0.014, 0.216)),
        origin=Origin(xyz=(step_half_width, 0.004, 0.124)),
        material=safety_yellow,
        name="right_side_bar",
    )
    step_bar.visual(
        Box((0.264, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.004, 0.235)),
        material=safety_yellow,
        name="top_bar",
    )
    step_bar.inertial = Inertial.from_geometry(
        Box((0.30, 0.04, 0.25)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.004, 0.125)),
    )

    model.articulation(
        "base_to_fly",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.40,
            lower=0.0,
            upper=0.52,
        ),
    )
    model.articulation(
        "fly_to_left_guard",
        ArticulationType.REVOLUTE,
        parent=fly,
        child="left_guard_rail",
        origin=Origin(xyz=(-guard_pivot_x, platform_rear_y, platform_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=-0.72,
            upper=0.0,
        ),
    )
    model.articulation(
        "fly_to_right_guard",
        ArticulationType.REVOLUTE,
        parent=fly,
        child="right_guard_rail",
        origin=Origin(xyz=(guard_pivot_x, platform_rear_y, platform_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=0.0,
            upper=0.72,
        ),
    )
    model.articulation(
        "fly_to_step_bar",
        ArticulationType.REVOLUTE,
        parent=fly,
        child=step_bar,
        origin=Origin(xyz=(0.0, platform_front_y, 1.630)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=0.0,
            upper=1.12,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    left_guard = object_model.get_part("left_guard_rail")
    right_guard = object_model.get_part("right_guard_rail")
    step_bar = object_model.get_part("step_over_bar")

    fly_extension = object_model.get_articulation("base_to_fly")
    left_hinge = object_model.get_articulation("fly_to_left_guard")
    right_hinge = object_model.get_articulation("fly_to_right_guard")
    step_hinge = object_model.get_articulation("fly_to_step_bar")

    base_left_stile = base.get_visual("base_left_stile")
    base_right_stile = base.get_visual("base_right_stile")
    left_lower_guide_shoe = fly.get_visual("left_lower_guide_shoe")
    right_lower_guide_shoe = fly.get_visual("right_lower_guide_shoe")
    left_upper_guide_shoe = fly.get_visual("left_upper_guide_shoe")
    right_upper_guide_shoe = fly.get_visual("right_upper_guide_shoe")
    platform_front_bar = fly.get_visual("platform_front_bar")
    platform_left_side = fly.get_visual("platform_left_side")
    platform_grate = fly.get_visual("grate_runner_1")
    left_hinge_cheek = fly.get_visual("left_hinge_cheek")
    right_hinge_cheek = fly.get_visual("right_hinge_cheek")
    left_guard_receiver = fly.get_visual("left_guard_receiver")
    right_guard_receiver = fly.get_visual("right_guard_receiver")
    left_step_hinge_cheek = fly.get_visual("left_step_hinge_cheek")
    right_step_hinge_cheek = fly.get_visual("right_step_hinge_cheek")

    left_hinge_shoe = left_guard.get_visual("right_side_hinge_shoe")
    right_hinge_shoe = right_guard.get_visual("left_side_hinge_shoe")
    left_latch_lug = left_guard.get_visual("front_latch_lug")
    right_latch_lug = right_guard.get_visual("front_latch_lug")
    left_top_rail = left_guard.get_visual("top_rail")
    right_top_rail = right_guard.get_visual("top_rail")

    step_left_hinge_foot = step_bar.get_visual("left_hinge_foot")
    step_right_hinge_foot = step_bar.get_visual("right_hinge_foot")
    step_top_bar = step_bar.get_visual("top_bar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        step_bar,
        fly,
        reason="step-over bar hinge feet are intentionally captured between the front pivot cheeks",
    )
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(fly, base, axes="xz", min_overlap=0.20)
    ctx.expect_origin_distance(fly, base, axes="x", max_dist=0.03)
    ctx.expect_gap(
        fly,
        base,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_lower_guide_shoe,
        negative_elem=base_left_stile,
        name="left_lower_guide_shoe_runs_on_base_stile",
    )
    ctx.expect_gap(
        fly,
        base,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_lower_guide_shoe,
        negative_elem=base_right_stile,
        name="right_lower_guide_shoe_runs_on_base_stile",
    )

    ctx.expect_gap(
        fly,
        fly,
        axis="z",
        min_gap=0.010,
        positive_elem=platform_front_bar,
        negative_elem=platform_grate,
        name="grating_sits_recessed_below_platform_frame",
    )
    ctx.expect_gap(
        fly,
        fly,
        axis="z",
        min_gap=0.010,
        positive_elem=platform_left_side,
        negative_elem=platform_grate,
        name="grating_sits_below_platform_side_tube",
    )

    ctx.expect_contact(left_guard, fly, elem_a=left_hinge_shoe, elem_b=left_hinge_cheek)
    ctx.expect_contact(right_guard, fly, elem_a=right_hinge_shoe, elem_b=right_hinge_cheek)
    ctx.expect_contact(left_guard, fly, elem_a=left_latch_lug, elem_b=left_guard_receiver)
    ctx.expect_contact(right_guard, fly, elem_a=right_latch_lug, elem_b=right_guard_receiver)
    ctx.expect_gap(
        left_guard,
        fly,
        axis="z",
        min_gap=0.78,
        positive_elem=left_top_rail,
        negative_elem=platform_grate,
        name="left_guard_rail_stands_well_above_platform",
    )
    ctx.expect_gap(
        right_guard,
        fly,
        axis="z",
        min_gap=0.78,
        positive_elem=right_top_rail,
        negative_elem=platform_grate,
        name="right_guard_rail_stands_well_above_platform",
    )

    ctx.expect_contact(step_bar, fly, elem_a=step_left_hinge_foot, elem_b=left_step_hinge_cheek)
    ctx.expect_contact(step_bar, fly, elem_a=step_right_hinge_foot, elem_b=right_step_hinge_cheek)
    ctx.expect_gap(
        step_bar,
        fly,
        axis="z",
        min_gap=0.25,
        positive_elem=step_top_bar,
        negative_elem=platform_grate,
        name="step_over_bar_rises_above_platform_front_edge",
    )

    with ctx.pose({fly_extension: 0.46}):
        ctx.expect_overlap(fly, base, axes="xz", min_overlap=0.12)
        ctx.expect_gap(
            fly,
            base,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=left_upper_guide_shoe,
            negative_elem=base_left_stile,
            name="left_upper_guide_shoe_stays_on_base_when_extended",
        )
        ctx.expect_gap(
            fly,
            base,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=right_upper_guide_shoe,
            negative_elem=base_right_stile,
            name="right_upper_guide_shoe_stays_on_base_when_extended",
        )

    with ctx.pose({left_hinge: -0.68}):
        ctx.expect_within(
            left_guard,
            fly,
            axes="xy",
            inner_elem=left_top_rail,
            name="left_guard_rail_folds_inboard_over_platform",
        )
    with ctx.pose({right_hinge: 0.68}):
        ctx.expect_within(
            right_guard,
            fly,
            axes="xy",
            inner_elem=right_top_rail,
            name="right_guard_rail_folds_inboard_over_platform",
        )
    with ctx.pose({step_hinge: 1.08}):
        ctx.expect_within(
            step_bar,
            fly,
            axes="xy",
            inner_elem=step_top_bar,
            name="step_over_bar_folds_back_inside_platform_footprint",
        )
        ctx.expect_gap(
            step_bar,
            fly,
            axis="z",
            min_gap=0.08,
            max_gap=0.19,
            positive_elem=step_top_bar,
            negative_elem=platform_grate,
            name="folded_step_bar_stows_just_above_grating",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
