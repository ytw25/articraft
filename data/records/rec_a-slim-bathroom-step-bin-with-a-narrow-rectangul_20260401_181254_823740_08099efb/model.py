from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="slim_bathroom_step_bin")

    outer_depth = 0.170
    outer_width = 0.220
    shell_height = 0.400
    wall_t = 0.005
    bottom_t = 0.006

    lid_depth = 0.178
    lid_width = 0.236
    lid_t = 0.008
    lid_skirt_t = 0.006
    lid_skirt_h = 0.020

    pedal_pivot_x = outer_depth / 2.0 + 0.009
    pedal_pivot_y = outer_width / 2.0 - 0.042
    pedal_pivot_z = 0.042
    pedal_bracket_x = outer_depth / 2.0 + 0.002

    shell_white = model.material("shell_white", rgba=(0.93, 0.93, 0.91, 1.0))
    lid_white = model.material("lid_white", rgba=(0.96, 0.96, 0.95, 1.0))
    pedal_gray = model.material("pedal_gray", rgba=(0.35, 0.36, 0.38, 1.0))
    accent_gray = model.material("accent_gray", rgba=(0.58, 0.59, 0.61, 1.0))

    shell = model.part("shell")
    shell.visual(
        Box((wall_t, outer_width, shell_height)),
        origin=Origin(xyz=(outer_depth / 2.0 - wall_t / 2.0, 0.0, shell_height / 2.0)),
        material=shell_white,
        name="front_wall",
    )
    shell.visual(
        Box((wall_t, outer_width, shell_height)),
        origin=Origin(xyz=(-outer_depth / 2.0 + wall_t / 2.0, 0.0, shell_height / 2.0)),
        material=shell_white,
        name="back_wall",
    )
    shell.visual(
        Box((outer_depth, wall_t, shell_height)),
        origin=Origin(xyz=(0.0, outer_width / 2.0 - wall_t / 2.0, shell_height / 2.0)),
        material=shell_white,
        name="right_wall",
    )
    shell.visual(
        Box((outer_depth, wall_t, shell_height)),
        origin=Origin(xyz=(0.0, -outer_width / 2.0 + wall_t / 2.0, shell_height / 2.0)),
        material=shell_white,
        name="left_wall",
    )
    shell.visual(
        Box((outer_depth - 2.0 * wall_t, outer_width - 2.0 * wall_t, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=shell_white,
        name="bottom_panel",
    )

    shell.visual(
        Box((0.008, 0.070, 0.010)),
        origin=Origin(xyz=(pedal_bracket_x, pedal_pivot_y, 0.025)),
        material=accent_gray,
        name="pedal_bracket",
    )
    shell.visual(
        Box((0.005, 0.014, 0.015)),
        origin=Origin(xyz=(0.087, pedal_pivot_y - 0.018, 0.030)),
        material=accent_gray,
        name="pedal_bracket_inner_rib",
    )
    shell.visual(
        Box((0.005, 0.014, 0.015)),
        origin=Origin(xyz=(0.087, pedal_pivot_y + 0.018, 0.030)),
        material=accent_gray,
        name="pedal_bracket_outer_rib",
    )
    shell.visual(
        Box((0.004, 0.010, 0.024)),
        origin=Origin(xyz=(0.0875, pedal_pivot_y - 0.025, pedal_pivot_z)),
        material=accent_gray,
        name="pedal_lug_inner",
    )
    shell.visual(
        Box((0.004, 0.010, 0.024)),
        origin=Origin(xyz=(0.0875, pedal_pivot_y + 0.025, pedal_pivot_z)),
        material=accent_gray,
        name="pedal_lug_outer",
    )
    shell.inertial = Inertial.from_geometry(
        Box((outer_depth, outer_width, shell_height)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, shell_height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_depth, lid_width, lid_t)),
        origin=Origin(xyz=(lid_depth / 2.0, 0.0, lid_t / 2.0)),
        material=lid_white,
        name="lid_panel",
    )
    lid.visual(
        Box((lid_skirt_t, lid_width, lid_skirt_h)),
        origin=Origin(xyz=(lid_depth - lid_skirt_t / 2.0, 0.0, -0.006)),
        material=lid_white,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_depth - 0.014, lid_skirt_t, lid_skirt_h)),
        origin=Origin(xyz=(0.089, lid_width / 2.0 - lid_skirt_t / 2.0, -0.006)),
        material=lid_white,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_depth - 0.014, lid_skirt_t, lid_skirt_h)),
        origin=Origin(xyz=(0.089, -lid_width / 2.0 + lid_skirt_t / 2.0, -0.006)),
        material=lid_white,
        name="left_skirt",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_depth, lid_width, lid_t + lid_skirt_h)),
        mass=0.45,
        origin=Origin(xyz=(lid_depth / 2.0, 0.0, 0.002)),
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.0045, length=0.060),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=accent_gray,
        name="pedal_axle",
    )
    pedal.visual(
        Box((0.010, 0.020, 0.012)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=pedal_gray,
        name="lever_hub",
    )
    pedal.visual(
        Box((0.036, 0.018, 0.012)),
        origin=Origin(xyz=(0.026, 0.0, -0.004)),
        material=pedal_gray,
        name="lever_bar",
    )
    pedal.visual(
        Box((0.022, 0.056, 0.008)),
        origin=Origin(xyz=(0.045, 0.0, -0.004)),
        material=pedal_gray,
        name="tread",
    )
    pedal.inertial = Inertial.from_geometry(
        Box((0.056, 0.056, 0.016)),
        mass=0.12,
        origin=Origin(xyz=(0.030, 0.0, -0.002)),
    )

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(-outer_depth / 2.0, 0.0, shell_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=1.45),
    )
    model.articulation(
        "shell_to_pedal",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=pedal,
        origin=Origin(xyz=(pedal_pivot_x, pedal_pivot_y, pedal_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=0.0, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("shell_to_lid")
    pedal_hinge = object_model.get_articulation("shell_to_pedal")

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
        "lid hinge uses rear transverse axis",
        tuple(lid_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "pedal hinge uses transverse axis",
        tuple(pedal_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={pedal_hinge.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, pedal_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            shell,
            axis="z",
            positive_elem="lid_panel",
            max_gap=0.001,
            max_penetration=0.0,
            name="closed lid sits on shell top",
        )
        ctx.expect_overlap(
            lid,
            shell,
            axes="xy",
            min_overlap=0.150,
            elem_a="lid_panel",
            name="lid covers the shell opening footprint",
        )
        ctx.expect_contact(
            pedal,
            shell,
            elem_a="pedal_axle",
            elem_b="pedal_lug_inner",
            contact_tol=1e-6,
            name="pedal pivot is supported by the inner shell lug",
        )
        ctx.expect_contact(
            pedal,
            shell,
            elem_a="pedal_axle",
            elem_b="pedal_lug_outer",
            contact_tol=1e-6,
            name="pedal pivot is supported by the outer shell lug",
        )

    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits else None
    pedal_upper = pedal_hinge.motion_limits.upper if pedal_hinge.motion_limits else None

    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    pedal_rest_aabb = ctx.part_element_world_aabb(pedal, elem="tread")

    if lid_upper is not None:
        with ctx.pose({lid_hinge: lid_upper}):
            lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
        ctx.check(
            "lid opens upward",
            lid_rest_aabb is not None
            and lid_open_aabb is not None
            and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.060,
            details=f"rest={lid_rest_aabb}, open={lid_open_aabb}",
        )

    if pedal_upper is not None:
        with ctx.pose({pedal_hinge: pedal_upper}):
            pedal_pressed_aabb = ctx.part_element_world_aabb(pedal, elem="tread")
        ctx.check(
            "pedal rotates downward when pressed",
            pedal_rest_aabb is not None
            and pedal_pressed_aabb is not None
            and pedal_pressed_aabb[0][2] < pedal_rest_aabb[0][2] - 0.008,
            details=f"rest={pedal_rest_aabb}, pressed={pedal_pressed_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
