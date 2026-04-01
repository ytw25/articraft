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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_rod(
    part,
    *,
    name: str,
    axis: str,
    center: tuple[float, float, float],
    radius: float,
    length: float,
    material,
):
    rotations = {
        "x": (0.0, math.pi / 2.0, 0.0),
        "y": (-math.pi / 2.0, 0.0, 0.0),
        "z": (0.0, 0.0, 0.0),
    }
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rotations[axis]),
        material=material,
        name=name,
    )


def _build_wire_rack(
    part,
    *,
    prefix: str,
    side_x: float,
    front_y: float,
    depth: float,
    runner_x: float,
    runner_length: float,
    runner_radius: float,
    frame_radius: float,
    top_z: float,
    material,
    tine_rows: tuple[tuple[float, float, tuple[float, ...]], ...] = (),
) -> None:
    bottom_z = 0.024
    rear_y = front_y + depth
    full_width = side_x * 2.0
    mid_y = front_y + depth / 2.0
    mid_z = (bottom_z + top_z) / 2.0
    support_overlap = 0.001

    _add_rod(
        part,
        name=f"{prefix}_left_runner",
        axis="y",
        center=(-runner_x, runner_length / 2.0, 0.0),
        radius=runner_radius,
        length=runner_length,
        material=material,
    )
    _add_rod(
        part,
        name=f"{prefix}_right_runner",
        axis="y",
        center=(runner_x, runner_length / 2.0, 0.0),
        radius=runner_radius,
        length=runner_length,
        material=material,
    )

    for x_pos, label in ((-runner_x, "left"), (runner_x, "right")):
        for y_pos, y_label in ((front_y, "front"), (rear_y, "rear")):
            _add_rod(
                part,
                name=f"{prefix}_{label}_runner_hanger_{y_label}",
                axis="z",
                center=(x_pos, y_pos, bottom_z / 2.0),
                radius=frame_radius,
                length=bottom_z + support_overlap * 2.0,
                material=material,
            )

    for y_pos, label in ((front_y, "front"), (rear_y, "rear")):
        _add_rod(
            part,
            name=f"{prefix}_{label}_bottom_bar",
            axis="x",
            center=(0.0, y_pos, bottom_z),
            radius=frame_radius,
            length=full_width,
            material=material,
        )
        _add_rod(
            part,
            name=f"{prefix}_{label}_top_bar",
            axis="x",
            center=(0.0, y_pos, top_z),
            radius=frame_radius,
            length=full_width,
            material=material,
        )

    for x_pos, label in ((-side_x, "left"), (side_x, "right")):
        _add_rod(
            part,
            name=f"{prefix}_{label}_bottom_side",
            axis="y",
            center=(x_pos, mid_y, bottom_z),
            radius=frame_radius,
            length=depth,
            material=material,
        )
        _add_rod(
            part,
            name=f"{prefix}_{label}_top_side",
            axis="y",
            center=(x_pos, mid_y, top_z),
            radius=frame_radius,
            length=depth,
            material=material,
        )
        for y_pos, y_label in ((front_y, "front"), (rear_y, "rear")):
            _add_rod(
                part,
                name=f"{prefix}_{label}_{y_label}_post",
                axis="z",
                center=(x_pos, y_pos, mid_z),
                radius=frame_radius,
                length=top_z - bottom_z,
                material=material,
            )

    for index, y_pos in enumerate(
        (
            front_y + depth * 0.22,
            front_y + depth * 0.42,
            front_y + depth * 0.62,
            front_y + depth * 0.82,
        )
    ):
        _add_rod(
            part,
            name=f"{prefix}_base_cross_{index}",
            axis="x",
            center=(0.0, y_pos, bottom_z),
            radius=frame_radius,
            length=full_width,
            material=material,
        )

    for index, x_pos in enumerate((-side_x * 0.5, 0.0, side_x * 0.5)):
        _add_rod(
            part,
            name=f"{prefix}_base_longitudinal_{index}",
            axis="y",
            center=(x_pos, mid_y, bottom_z),
            radius=frame_radius,
            length=depth,
            material=material,
        )

    for row_index, (y_pos, tine_height, x_positions) in enumerate(tine_rows):
        _add_rod(
            part,
            name=f"{prefix}_tine_support_{row_index}",
            axis="x",
            center=(0.0, y_pos, bottom_z),
            radius=frame_radius,
            length=full_width,
            material=material,
        )
        for tine_index, x_pos in enumerate(x_positions):
            _add_rod(
                part,
                name=f"{prefix}_tine_{row_index}_{tine_index}",
                axis="z",
                center=(x_pos, y_pos, bottom_z + tine_height / 2.0 - support_overlap / 2.0),
                radius=frame_radius,
                length=tine_height + support_overlap,
                material=material,
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_dishwasher")

    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    inner_gray = model.material("inner_gray", rgba=(0.83, 0.84, 0.86, 1.0))
    control_dark = model.material("control_dark", rgba=(0.16, 0.18, 0.20, 1.0))
    display_glass = model.material("display_glass", rgba=(0.09, 0.17, 0.22, 1.0))
    rack_steel = model.material("rack_steel", rgba=(0.67, 0.69, 0.71, 1.0))
    rail_gray = model.material("rail_gray", rgba=(0.58, 0.60, 0.63, 1.0))

    width = 0.60
    depth = 0.58
    height = 0.82
    wall = 0.018
    top_thickness = 0.015
    bottom_thickness = 0.020

    door_width = 0.556
    door_thickness = 0.035
    door_height = 0.685
    door_bottom = 0.020
    door_joint_y = -depth / 2.0 + door_thickness / 2.0

    body = model.part("body")
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, height / 2.0)),
        material=inner_gray,
        name="left_wall",
    )
    body.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, height / 2.0)),
        material=inner_gray,
        name="right_wall",
    )
    body.visual(
        Box((width - 2.0 * wall, wall, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, height / 2.0)),
        material=inner_gray,
        name="back_wall",
    )
    body.visual(
        Box((width - 2.0 * wall, depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness / 2.0)),
        material=inner_gray,
        name="bottom_tray",
    )
    body.visual(
        Box((width - 2.0 * wall, depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, height - top_thickness / 2.0)),
        material=inner_gray,
        name="top_panel",
    )

    lower_rail_length = 0.50
    lower_rail_x = 0.252
    lower_rail_width = 0.060
    lower_runner_radius = 0.005
    lower_rail_height = 0.012
    lower_runner_z = 0.122
    lower_rail_center_z = lower_runner_z - lower_runner_radius - lower_rail_height / 2.0
    body.visual(
        Box((lower_rail_width, lower_rail_length, lower_rail_height)),
        origin=Origin(xyz=(-lower_rail_x, 0.0, lower_rail_center_z)),
        material=rail_gray,
        name="lower_left_rail",
    )
    body.visual(
        Box((lower_rail_width, lower_rail_length, lower_rail_height)),
        origin=Origin(xyz=(lower_rail_x, 0.0, lower_rail_center_z)),
        material=rail_gray,
        name="lower_right_rail",
    )

    upper_rail_length = 0.45
    upper_rail_x = 0.257
    upper_rail_width = 0.050
    upper_runner_radius = 0.005
    upper_rail_height = 0.012
    upper_runner_z = 0.420
    upper_rail_center_z = upper_runner_z - upper_runner_radius - upper_rail_height / 2.0
    body.visual(
        Box((upper_rail_width, upper_rail_length, upper_rail_height)),
        origin=Origin(xyz=(-upper_rail_x, 0.0, upper_rail_center_z)),
        material=rail_gray,
        name="upper_left_rail",
    )
    body.visual(
        Box((upper_rail_width, upper_rail_length, upper_rail_height)),
        origin=Origin(xyz=(upper_rail_x, 0.0, upper_rail_center_z)),
        material=rail_gray,
        name="upper_right_rail",
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((0.556, 0.028, 0.095)),
        material=control_dark,
        name="console_shell",
    )
    control_strip.visual(
        Box((0.150, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, -0.012, 0.010)),
        material=display_glass,
        name="display_window",
    )
    for index, x_pos in enumerate((-0.16, -0.11, -0.06, 0.11, 0.16)):
        control_strip.visual(
            Box((0.026, 0.004, 0.016)),
            origin=Origin(xyz=(x_pos, -0.012, -0.010)),
            material=stainless,
            name=f"button_{index}",
        )

    door = model.part("door")
    door.visual(
        Box((door_width, 0.006, door_height)),
        origin=Origin(xyz=(0.0, -door_thickness / 2.0 + 0.003, door_height / 2.0)),
        material=stainless,
        name="front_skin",
    )
    door.visual(
        Box((door_width - 0.032, 0.008, door_height - 0.020)),
        origin=Origin(xyz=(0.0, door_thickness / 2.0 - 0.004, door_height / 2.0)),
        material=inner_gray,
        name="inner_liner",
    )
    door.visual(
        Box((0.018, door_thickness - 0.012, door_height)),
        origin=Origin(xyz=(-door_width / 2.0 + 0.009, 0.0, door_height / 2.0)),
        material=stainless,
        name="left_stile",
    )
    door.visual(
        Box((0.018, door_thickness - 0.012, door_height)),
        origin=Origin(xyz=(door_width / 2.0 - 0.009, 0.0, door_height / 2.0)),
        material=stainless,
        name="right_stile",
    )
    door.visual(
        Box((door_width - 0.036, door_thickness - 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stainless,
        name="bottom_rail",
    )
    door.visual(
        Box((door_width - 0.036, door_thickness - 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, door_height - 0.011)),
        material=stainless,
        name="top_rail",
    )

    lower_rack = model.part("lower_rack")
    _build_wire_rack(
        lower_rack,
        prefix="lower",
        side_x=0.252,
        front_y=0.040,
        depth=0.460,
        runner_x=0.252,
        runner_length=lower_rail_length,
        runner_radius=lower_runner_radius,
        frame_radius=0.0035,
        top_z=0.145,
        material=rack_steel,
        tine_rows=(
            (0.320, 0.090, (-0.180, -0.120, -0.060, 0.060, 0.120, 0.180)),
            (0.430, 0.085, (-0.180, -0.120, -0.060, 0.060, 0.120, 0.180)),
        ),
    )

    upper_rack = model.part("upper_rack")
    _build_wire_rack(
        upper_rack,
        prefix="upper",
        side_x=0.232,
        front_y=0.050,
        depth=0.400,
        runner_x=0.232,
        runner_length=upper_rail_length,
        runner_radius=upper_runner_radius,
        frame_radius=0.0033,
        top_z=0.120,
        material=rack_steel,
        tine_rows=(
            (0.270, 0.070, (-0.150, -0.090, -0.030, 0.030, 0.090, 0.150)),
            (0.360, 0.065, (-0.150, -0.090, -0.030, 0.030, 0.090, 0.150)),
        ),
    )

    model.articulation(
        "body_to_control_strip",
        ArticulationType.FIXED,
        parent=body,
        child=control_strip,
        origin=Origin(xyz=(0.0, -depth / 2.0 + 0.014, 0.7575)),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, door_joint_y, door_bottom)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "body_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.0, -lower_rail_length / 2.0, lower_runner_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "body_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.0, -upper_rail_length / 2.0, upper_runner_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.30,
            lower=0.0,
            upper=0.26,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    control_strip = object_model.get_part("control_strip")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")

    door_hinge = object_model.get_articulation("body_to_door")
    lower_slide = object_model.get_articulation("body_to_lower_rack")
    upper_slide = object_model.get_articulation("body_to_upper_rack")

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

    ctx.expect_contact(
        control_strip,
        body,
        name="control strip is mounted to the top front of the tub",
    )
    ctx.expect_gap(
        control_strip,
        door,
        axis="z",
        min_gap=0.0,
        max_gap=0.010,
        name="door closes just below the control strip",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="bottom_rail",
        elem_b="bottom_tray",
        name="door rests on the lower front sill when closed",
    )
    ctx.expect_contact(
        lower_rack,
        body,
        elem_a="lower_left_runner",
        elem_b="lower_left_rail",
        name="lower rack rides on the left lower rail",
    )
    ctx.expect_contact(
        lower_rack,
        body,
        elem_a="lower_right_runner",
        elem_b="lower_right_rail",
        name="lower rack rides on the right lower rail",
    )
    ctx.expect_contact(
        upper_rack,
        body,
        elem_a="upper_left_runner",
        elem_b="upper_left_rail",
        name="upper rack rides on the left upper rail",
    )
    ctx.expect_contact(
        upper_rack,
        body,
        elem_a="upper_right_runner",
        elem_b="upper_right_rail",
        name="upper rack rides on the right upper rail",
    )
    ctx.expect_gap(
        upper_rack,
        lower_rack,
        axis="z",
        min_gap=0.140,
        name="upper rack sits clearly above the lower rack",
    )

    door_rest_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        door_open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward and downward",
        door_rest_aabb is not None
        and door_open_aabb is not None
        and door_open_aabb[0][1] < door_rest_aabb[0][1] - 0.18
        and door_open_aabb[1][2] < door_rest_aabb[1][2] - 0.30,
        details=f"rest={door_rest_aabb}, open={door_open_aabb}",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    with ctx.pose({lower_slide: lower_slide.motion_limits.upper}):
        lower_extended = ctx.part_world_position(lower_rack)
        ctx.expect_overlap(
            lower_rack,
            body,
            axes="y",
            elem_a="lower_left_runner",
            elem_b="lower_left_rail",
            min_overlap=0.19,
            name="lower rack retains left-rail insertion when extended",
        )
        ctx.expect_overlap(
            lower_rack,
            body,
            axes="y",
            elem_a="lower_right_runner",
            elem_b="lower_right_rail",
            min_overlap=0.19,
            name="lower rack retains right-rail insertion when extended",
        )
    ctx.check(
        "lower rack slides outward toward the user",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[1] < lower_rest[1] - 0.20,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    upper_rest = ctx.part_world_position(upper_rack)
    with ctx.pose({upper_slide: upper_slide.motion_limits.upper}):
        upper_extended = ctx.part_world_position(upper_rack)
        ctx.expect_overlap(
            upper_rack,
            body,
            axes="y",
            elem_a="upper_left_runner",
            elem_b="upper_left_rail",
            min_overlap=0.18,
            name="upper rack retains left-rail insertion when extended",
        )
        ctx.expect_overlap(
            upper_rack,
            body,
            axes="y",
            elem_a="upper_right_runner",
            elem_b="upper_right_rail",
            min_overlap=0.18,
            name="upper rack retains right-rail insertion when extended",
        )
    ctx.check(
        "upper rack slides outward toward the user",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[1] < upper_rest[1] - 0.18,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
