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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pilot_case_suitcase")

    shell_mat = model.material("shell", rgba=(0.20, 0.21, 0.23, 1.0))
    trim_mat = model.material("trim", rgba=(0.68, 0.70, 0.73, 1.0))
    plastic_mat = model.material("plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    wheel_mat = model.material("wheel", rgba=(0.07, 0.07, 0.08, 1.0))
    hub_mat = model.material("hub", rgba=(0.18, 0.19, 0.20, 1.0))

    case_width = 0.43
    case_depth = 0.23
    body_height = 0.37
    wall_t = 0.004
    bottom_t = 0.005
    lid_panel_t = 0.006
    lid_skirt_h = 0.050
    lid_skirt_t = 0.004
    lid_clearance = 0.0015

    wheel_radius = 0.035
    wheel_width = 0.018
    hub_radius = 0.018
    hub_length = 0.026
    wheel_x = -case_depth / 2.0 + 0.020
    wheel_y = case_width / 2.0 - 0.032
    wheel_z = 0.026
    axle_cheek_t = 0.004
    axle_cheek_x = 0.040
    axle_cheek_h = 0.070

    sleeve_outer_w = 0.032
    sleeve_depth = 0.021
    sleeve_wall = 0.003
    sleeve_length = 0.240
    sleeve_bottom_z = 0.110
    sleeve_top_z = sleeve_bottom_z + sleeve_length
    handle_y = case_width / 2.0 - 0.070
    sleeve_center_x = -case_depth / 2.0 - sleeve_depth / 2.0

    inner_tube_w = 0.019
    inner_tube_d = 0.013
    inner_tube_length = 0.300
    inner_tube_center_z = -0.069
    handle_stop_t = 0.006
    handle_travel = 0.160
    grip_bar_t = 0.018
    grip_bar_depth = 0.018
    grip_bar_width = 2.0 * handle_y + inner_tube_w
    grip_bar_z = inner_tube_center_z + inner_tube_length / 2.0 + grip_bar_t / 2.0
    grip_pad_t = 0.012

    lid_panel_w = case_width + 2.0 * (lid_clearance + lid_skirt_t)
    lid_panel_d = case_depth + lid_clearance + lid_skirt_t

    body = model.part("body")
    body.visual(
        Box((case_depth, case_width, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=shell_mat,
        name="bottom_panel",
    )
    wall_z = bottom_t + (body_height - bottom_t) / 2.0
    body.visual(
        Box((wall_t, case_width, body_height - bottom_t)),
        origin=Origin(xyz=(case_depth / 2.0 - wall_t / 2.0, 0.0, wall_z)),
        material=shell_mat,
        name="front_wall",
    )
    body.visual(
        Box((wall_t, case_width, body_height - bottom_t)),
        origin=Origin(xyz=(-case_depth / 2.0 + wall_t / 2.0, 0.0, wall_z)),
        material=shell_mat,
        name="back_wall",
    )
    side_wall_depth = case_depth - 2.0 * wall_t
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((side_wall_depth, wall_t, body_height - bottom_t)),
            origin=Origin(
                xyz=(0.0, side_sign * (case_width / 2.0 - wall_t / 2.0), wall_z)
            ),
            material=shell_mat,
            name=f"{side_name}_wall",
        )

    foot_size = 0.022
    foot_h = 0.010
    foot_x = case_depth / 2.0 - 0.030
    foot_y = case_width / 2.0 - 0.035
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        body.visual(
            Box((foot_size, foot_size, foot_h)),
            origin=Origin(xyz=(foot_x, side_sign * foot_y, -foot_h / 2.0)),
            material=plastic_mat,
            name=f"{side_name}_foot",
        )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        y_center = side_sign * wheel_y
        body.visual(
            Box((axle_cheek_x, axle_cheek_t, axle_cheek_h)),
            origin=Origin(
                xyz=(
                    wheel_x,
                    y_center - (hub_length / 2.0 + axle_cheek_t / 2.0),
                    axle_cheek_h / 2.0,
                )
            ),
            material=plastic_mat,
            name=f"{side_name}_axle_inner_cheek",
        )
        body.visual(
            Box((axle_cheek_x, axle_cheek_t, axle_cheek_h)),
            origin=Origin(
                xyz=(
                    wheel_x,
                    y_center + (hub_length / 2.0 + axle_cheek_t / 2.0),
                    axle_cheek_h / 2.0,
                )
            ),
            material=plastic_mat,
            name=f"{side_name}_axle_outer_cheek",
        )

    left_sleeve = model.part("left_sleeve")
    right_sleeve = model.part("right_sleeve")
    for sleeve in (left_sleeve, right_sleeve):
        sleeve.visual(
            Box((sleeve_wall, sleeve_outer_w, sleeve_length)),
            origin=Origin(
                xyz=(-sleeve_depth / 2.0 + sleeve_wall / 2.0, 0.0, sleeve_length / 2.0)
            ),
            material=trim_mat,
            name="back_plate",
        )
        sleeve.visual(
            Box((sleeve_depth, sleeve_wall, sleeve_length)),
            origin=Origin(
                xyz=(0.0, sleeve_outer_w / 2.0 - sleeve_wall / 2.0, sleeve_length / 2.0)
            ),
            material=trim_mat,
            name="outer_rail",
        )
        sleeve.visual(
            Box((sleeve_depth, sleeve_wall, sleeve_length)),
            origin=Origin(
                xyz=(0.0, -sleeve_outer_w / 2.0 + sleeve_wall / 2.0, sleeve_length / 2.0)
            ),
            material=trim_mat,
            name="inner_rail",
        )

    handle_stage = model.part("handle_stage")
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        handle_stage.visual(
            Box((inner_tube_d, inner_tube_w, inner_tube_length)),
            origin=Origin(
                xyz=(0.0, side_sign * handle_y, inner_tube_center_z)
            ),
            material=trim_mat,
            name=f"{side_name}_inner_tube",
        )
        handle_stage.visual(
            Box((0.016, 0.026, handle_stop_t)),
            origin=Origin(xyz=(0.0, side_sign * handle_y, handle_stop_t / 2.0)),
            material=plastic_mat,
            name=f"{side_name}_tube_stop",
        )

    handle_stage.visual(
        Box((grip_bar_depth, grip_bar_width, grip_bar_t)),
        origin=Origin(xyz=(0.0, 0.0, grip_bar_z)),
        material=trim_mat,
        name="grip_bar",
    )
    handle_stage.visual(
        Box((grip_bar_depth + 0.010, grip_bar_width - 0.040, grip_pad_t)),
        origin=Origin(
            xyz=(0.0, 0.0, grip_bar_z + grip_bar_t / 2.0 + grip_pad_t / 2.0)
        ),
        material=plastic_mat,
        name="grip_pad",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_panel_d, lid_panel_w, lid_panel_t)),
        origin=Origin(xyz=(lid_panel_d / 2.0, 0.0, lid_panel_t / 2.0)),
        material=shell_mat,
        name="lid_panel",
    )
    side_skirt_depth = lid_panel_d - 0.012
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        lid.visual(
            Box((side_skirt_depth, lid_skirt_t, lid_skirt_h)),
            origin=Origin(
                xyz=(
                    lid_panel_d / 2.0,
                    side_sign * (case_width / 2.0 + lid_clearance + lid_skirt_t / 2.0),
                    -lid_skirt_h / 2.0,
                )
            ),
            material=shell_mat,
            name=f"{side_name}_skirt",
        )
    lid.visual(
        Box((lid_skirt_t, lid_panel_w, lid_skirt_h)),
        origin=Origin(
            xyz=(lid_panel_d - lid_skirt_t / 2.0, 0.0, -lid_skirt_h / 2.0)
        ),
        material=shell_mat,
        name="front_skirt",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_mat,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_mat,
        name="hub",
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_mat,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hub_mat,
        name="hub",
    )

    model.articulation(
        "body_to_left_sleeve",
        ArticulationType.FIXED,
        parent=body,
        child=left_sleeve,
        origin=Origin(xyz=(sleeve_center_x, handle_y, sleeve_bottom_z)),
    )
    model.articulation(
        "body_to_right_sleeve",
        ArticulationType.FIXED,
        parent=body,
        child=right_sleeve,
        origin=Origin(xyz=(sleeve_center_x, -handle_y, sleeve_bottom_z)),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle_stage,
        origin=Origin(xyz=(sleeve_center_x, 0.0, sleeve_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=handle_travel,
        ),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-case_depth / 2.0, 0.0, body_height)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(wheel_x, wheel_y, wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(wheel_x, -wheel_y, wheel_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_sleeve = object_model.get_part("left_sleeve")
    right_sleeve = object_model.get_part("right_sleeve")
    handle_stage = object_model.get_part("handle_stage")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_slide = object_model.get_articulation("body_to_handle")
    left_wheel_joint = object_model.get_articulation("body_to_left_wheel")
    right_wheel_joint = object_model.get_articulation("body_to_right_wheel")

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
        "all major suitcase parts exist",
        all(
            part is not None
            for part in (
                body,
                lid,
                left_sleeve,
                right_sleeve,
                handle_stage,
                left_wheel,
                right_wheel,
            )
        ),
        details="One or more suitcase parts could not be resolved.",
    )
    ctx.check(
        "lid hinge opens upward from the rear edge",
        lid_hinge.axis == (0.0, -1.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper >= 1.2,
        details=f"axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "handle uses an upward prismatic slide",
        handle_slide.axis == (0.0, 0.0, 1.0)
        and handle_slide.motion_limits is not None
        and handle_slide.motion_limits.lower == 0.0
        and handle_slide.motion_limits.upper is not None
        and handle_slide.motion_limits.upper >= 0.15,
        details=f"axis={handle_slide.axis}, limits={handle_slide.motion_limits}",
    )

    for joint, label in (
        (left_wheel_joint, "left wheel"),
        (right_wheel_joint, "right wheel"),
    ):
        limits = joint.motion_limits
        ctx.check(
            f"{label} spins on a continuous axle joint",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (0.0, 1.0, 0.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    with ctx.pose({lid_hinge: 0.0, handle_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            max_gap=0.001,
            max_penetration=0.0,
            name="lid panel seats on the case rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            min_overlap=0.20,
            name="lid panel covers the top opening",
        )
        ctx.expect_within(
            handle_stage,
            left_sleeve,
            axes="xy",
            inner_elem="left_inner_tube",
            margin=0.006,
            name="left telescoping tube stays aligned in the left sleeve",
        )
        ctx.expect_within(
            handle_stage,
            right_sleeve,
            axes="xy",
            inner_elem="right_inner_tube",
            margin=0.006,
            name="right telescoping tube stays aligned in the right sleeve",
        )
        ctx.expect_overlap(
            handle_stage,
            left_sleeve,
            axes="z",
            elem_a="left_inner_tube",
            min_overlap=0.20,
            name="left telescoping tube is deeply inserted when retracted",
        )
        ctx.expect_overlap(
            handle_stage,
            right_sleeve,
            axes="z",
            elem_a="right_inner_tube",
            min_overlap=0.20,
            name="right telescoping tube is deeply inserted when retracted",
        )
        ctx.expect_contact(
            handle_stage,
            left_sleeve,
            elem_a="left_tube_stop",
            name="left handle stop rests on the sleeve lip",
        )
        ctx.expect_contact(
            handle_stage,
            right_sleeve,
            elem_a="right_tube_stop",
            name="right handle stop rests on the sleeve lip",
        )
        ctx.expect_contact(
            left_wheel,
            body,
            elem_a="hub",
            name="left wheel hub is carried by the rear fork",
        )
        ctx.expect_contact(
            right_wheel,
            body,
            elem_a="hub",
            name="right wheel hub is carried by the rear fork",
        )

    rest_handle_pos = ctx.part_world_position(handle_stage)
    with ctx.pose({handle_slide: 0.16}):
        extended_handle_pos = ctx.part_world_position(handle_stage)
        ctx.expect_within(
            handle_stage,
            left_sleeve,
            axes="xy",
            inner_elem="left_inner_tube",
            margin=0.006,
            name="left tube stays centered when the handle is extended",
        )
        ctx.expect_within(
            handle_stage,
            right_sleeve,
            axes="xy",
            inner_elem="right_inner_tube",
            margin=0.006,
            name="right tube stays centered when the handle is extended",
        )
        ctx.expect_overlap(
            handle_stage,
            left_sleeve,
            axes="z",
            elem_a="left_inner_tube",
            min_overlap=0.05,
            name="left tube keeps retained insertion at full extension",
        )
        ctx.expect_overlap(
            handle_stage,
            right_sleeve,
            axes="z",
            elem_a="right_inner_tube",
            min_overlap=0.05,
            name="right tube keeps retained insertion at full extension",
        )
    ctx.check(
        "handle extends upward",
        rest_handle_pos is not None
        and extended_handle_pos is not None
        and extended_handle_pos[2] > rest_handle_pos[2] + 0.12,
        details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        closed_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({lid_hinge: 1.10}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    ctx.check(
        "lid front edge lifts clear when opened",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.12,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
