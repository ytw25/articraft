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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_loop(width: float, depth: float, z: float, *, y_offset: float = 0.0, radius: float = 0.02):
    return [(x, y + y_offset, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanics_toolbox")

    body_red = model.material("body_red", rgba=(0.72, 0.11, 0.08, 1.0))
    body_dark = model.material("body_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.37, 0.40, 1.0))
    latch_black = model.material("latch_black", rgba=(0.11, 0.11, 0.12, 1.0))
    translucent = model.material("translucent", rgba=(0.75, 0.84, 0.90, 0.55))

    outer_w = 0.50
    outer_d = 0.26
    body_h = 0.18
    wall = 0.012
    organizer_w = 0.10
    lid_w = outer_w - organizer_w - wall * 0.7
    lid_center_x = -0.5 * organizer_w - 0.25 * wall
    lid_peak_h = 0.078
    door_w = 0.210
    door_h = 0.120
    door_t = 0.010
    latch_offsets = (-0.132, 0.132)

    box_body = model.part("box_body")
    box_body.inertial = Inertial.from_geometry(
        Box((outer_w, outer_d, body_h)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    box_body.visual(
        Box((outer_w, outer_d, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=body_red,
        name="floor",
    )
    box_body.visual(
        Box((outer_w, wall, body_h - wall)),
        origin=Origin(xyz=(0.0, outer_d * 0.5 - wall * 0.5, (body_h + wall) * 0.5)),
        material=body_red,
        name="front_wall",
    )
    box_body.visual(
        Box((outer_w, wall, body_h - wall)),
        origin=Origin(xyz=(0.0, -outer_d * 0.5 + wall * 0.5, (body_h + wall) * 0.5)),
        material=body_red,
        name="rear_wall",
    )
    box_body.visual(
        Box((wall, outer_d - 2.0 * wall, body_h - wall)),
        origin=Origin(xyz=(-outer_w * 0.5 + wall * 0.5, 0.0, (body_h + wall) * 0.5)),
        material=body_red,
        name="left_wall",
    )

    divider_x = outer_w * 0.5 - organizer_w - wall * 0.5
    box_body.visual(
        Box((wall, outer_d - 2.0 * wall, body_h - wall)),
        origin=Origin(xyz=(divider_x, 0.0, (body_h + wall) * 0.5)),
        material=body_red,
        name="organizer_divider",
    )

    organizer_center_x = 0.5 * (divider_x + wall * 0.5 + outer_w * 0.5)
    organizer_span_x = outer_w * 0.5 - divider_x - wall * 0.5
    box_body.visual(
        Box((organizer_span_x, outer_d - 2.0 * wall, wall)),
        origin=Origin(xyz=(organizer_center_x, 0.0, body_h - wall * 0.5)),
        material=body_red,
        name="organizer_roof",
    )

    frame_x = outer_w * 0.5 - wall * 0.5
    frame_post_h = body_h - 0.038
    frame_open_z = 0.020 + frame_post_h * 0.5
    box_body.visual(
        Box((wall, wall, frame_post_h)),
        origin=Origin(xyz=(frame_x, outer_d * 0.5 - wall * 1.5, frame_open_z)),
        material=body_red,
        name="organizer_front_post",
    )
    box_body.visual(
        Box((wall, wall, frame_post_h)),
        origin=Origin(xyz=(frame_x, -outer_d * 0.5 + wall * 1.5, frame_open_z)),
        material=body_red,
        name="organizer_rear_post",
    )
    box_body.visual(
        Box((wall, outer_d - 4.0 * wall, wall)),
        origin=Origin(xyz=(frame_x, 0.0, 0.020 + wall * 0.5)),
        material=body_red,
        name="organizer_bottom_rail",
    )
    box_body.visual(
        Box((wall, outer_d - 4.0 * wall, wall)),
        origin=Origin(xyz=(frame_x, 0.0, body_h - wall * 1.5)),
        material=body_red,
        name="organizer_top_rail",
    )

    main_opening_w = lid_w - 0.020
    box_body.visual(
        Box((main_opening_w, wall, wall)),
        origin=Origin(xyz=(lid_center_x, outer_d * 0.5 - wall * 0.5, body_h - wall * 0.5)),
        material=dark_steel,
        name="front_lip",
    )
    box_body.visual(
        Box((main_opening_w, wall, wall)),
        origin=Origin(xyz=(lid_center_x, -outer_d * 0.5 + wall * 0.5, body_h - wall * 0.5)),
        material=dark_steel,
        name="rear_lip",
    )
    box_body.visual(
        Box((wall, outer_d - 2.0 * wall, wall)),
        origin=Origin(xyz=(-outer_w * 0.5 + wall * 1.5, 0.0, body_h - wall * 0.5)),
        material=dark_steel,
        name="left_lip",
    )
    box_body.visual(
        Box((wall, outer_d - 2.0 * wall, wall)),
        origin=Origin(xyz=(divider_x - wall * 0.5, 0.0, body_h - wall * 0.5)),
        material=dark_steel,
        name="divider_lip",
    )

    for sign in (-1.0, 1.0):
        box_body.visual(
            Box((0.050, 0.038, 0.008)),
            origin=Origin(xyz=(sign * 0.17, 0.0, 0.004)),
            material=body_dark,
        )
    latch_origin_y = outer_d * 0.5 + 0.029
    latch_origin_z = body_h - 0.015
    for index, x_offset in enumerate(latch_offsets, start=1):
        box_body.visual(
            Box((0.032, 0.020, 0.018)),
            origin=Origin(xyz=(lid_center_x + x_offset, outer_d * 0.5 + 0.010, body_h - 0.018)),
            material=dark_steel,
            name=f"latch_mount_{index}",
        )

    lid = model.part("main_lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, outer_d, lid_peak_h)),
        mass=1.9,
        origin=Origin(xyz=(0.0, outer_d * 0.5, lid_peak_h * 0.45)),
    )
    lid_cap = section_loft(
        [
            _rounded_loop(lid_w, outer_d, 0.004, y_offset=outer_d * 0.5, radius=0.028),
            _rounded_loop(lid_w - 0.018, outer_d - 0.016, 0.032, y_offset=outer_d * 0.5, radius=0.030),
            _rounded_loop(lid_w - 0.080, outer_d - 0.060, lid_peak_h, y_offset=outer_d * 0.5, radius=0.045),
        ]
    )
    lid.visual(_mesh("toolbox_lid_cap", lid_cap), material=body_red, name="lid_cap")
    lid.visual(
        Box((lid_w - 0.010, 0.012, 0.024)),
        origin=Origin(xyz=(0.0, outer_d - 0.006, 0.012)),
        material=body_red,
        name="front_fascia",
    )
    lid.visual(
        Box((0.012, outer_d - 0.010, 0.024)),
        origin=Origin(xyz=(-lid_w * 0.5 + 0.006, outer_d * 0.5, 0.012)),
        material=body_red,
        name="left_fascia",
    )
    lid.visual(
        Box((0.012, outer_d - 0.010, 0.024)),
        origin=Origin(xyz=(lid_w * 0.5 - 0.006, outer_d * 0.5, 0.012)),
        material=body_red,
        name="right_fascia",
    )
    lid.visual(
        Cylinder(radius=0.010, length=lid_w - 0.050),
        origin=Origin(xyz=(0.0, 0.004, 0.006), rpy=(0.0, pi * 0.5, 0.0)),
        material=steel,
        name="rear_hinge_bar",
    )
    handle_pivot_y = 0.118
    handle_pivot_z = 0.094
    lid.visual(
        Box((0.026, 0.030, 0.018)),
        origin=Origin(xyz=(latch_offsets[0], outer_d - 0.006, 0.020)),
        material=steel,
        name="keeper_1",
    )
    lid.visual(
        Box((0.026, 0.030, 0.018)),
        origin=Origin(xyz=(latch_offsets[1], outer_d - 0.006, 0.020)),
        material=steel,
        name="keeper_2",
    )
    for index, x_pos in enumerate((-0.125, 0.125), start=1):
        lid.visual(
            Box((0.016, 0.026, 0.060)),
            origin=Origin(xyz=(x_pos, handle_pivot_y, 0.060)),
            material=dark_steel,
            name=f"handle_pedestal_{index}",
        )
        lid.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(
                xyz=(x_pos, handle_pivot_y, handle_pivot_z - 0.008),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=steel,
            name=f"handle_boss_{index}",
        )

    handle = model.part("carry_handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.30, 0.08, 0.05)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
    )
    handle_path = [
        (-0.140, 0.000, 0.000),
        (-0.138, 0.018, 0.000),
        (-0.110, 0.050, 0.000),
        (-0.060, 0.066, 0.000),
        (0.060, 0.066, 0.000),
        (0.110, 0.050, 0.000),
        (0.138, 0.018, 0.000),
        (0.140, 0.000, 0.000),
    ]
    handle.visual(
        _mesh(
            "toolbox_carry_handle",
            tube_from_spline_points(
                handle_path,
                radius=0.008,
                samples_per_segment=14,
                radial_segments=18,
            ),
        ),
        material=steel,
        name="handle_frame",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.150),
        origin=Origin(xyz=(0.0, 0.066, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=body_dark,
        name="handle_grip",
    )
    for index, x_pos in enumerate((-0.140, 0.140), start=1):
        handle.visual(
            Cylinder(radius=0.009, length=0.016),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
            material=dark_steel,
            name=f"handle_barrel_{index}",
        )

    organizer_door = model.part("organizer_door")
    organizer_door.inertial = Inertial.from_geometry(
        Box((door_t, door_w, door_h)),
        mass=0.55,
        origin=Origin(xyz=(door_t * 0.5, door_w * 0.5, door_h * 0.5)),
    )
    organizer_door.visual(
        Box((door_t, door_w, door_h)),
        origin=Origin(xyz=(door_t * 0.5, door_w * 0.5, door_h * 0.5)),
        material=body_red,
        name="door_panel",
    )
    organizer_door.visual(
        Box((0.004, door_w - 0.030, door_h - 0.026)),
        origin=Origin(xyz=(door_t + 0.002, door_w * 0.5, door_h * 0.5)),
        material=translucent,
        name="window_main",
    )
    organizer_door.visual(
        Box((0.016, 0.040, 0.020)),
        origin=Origin(xyz=(door_t + 0.008, door_w * 0.72, door_h * 0.52)),
        material=latch_black,
        name="door_pull",
    )
    organizer_door.visual(
        Cylinder(radius=0.005, length=door_h - 0.010),
        origin=Origin(xyz=(0.005, 0.0, door_h * 0.5), rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="door_hinge_pin",
    )

    left_latch = model.part("left_latch")
    left_latch.inertial = Inertial.from_geometry(
        Box((0.030, 0.026, 0.088)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.006, -0.022)),
    )
    for latch_part in (left_latch, model.part("right_latch")):
        latch_part.inertial = Inertial.from_geometry(
            Box((0.030, 0.026, 0.088)),
            mass=0.12,
            origin=Origin(xyz=(0.0, 0.006, -0.022)),
        )
        latch_part.visual(
            Cylinder(radius=0.006, length=0.028),
            origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
            material=steel,
            name="pivot_barrel",
        )
        latch_part.visual(
            Box((0.022, 0.012, 0.062)),
            origin=Origin(xyz=(0.0, 0.010, -0.034)),
            material=latch_black,
            name="strap",
        )
        latch_part.visual(
            Box((0.030, 0.020, 0.014)),
            origin=Origin(xyz=(0.0, 0.012, -0.072)),
            material=latch_black,
            name="grip",
        )
        latch_part.visual(
            Box((0.022, 0.022, 0.016)),
            origin=Origin(xyz=(0.0, -0.004, 0.014)),
            material=steel,
            name="hook_pad",
        )
        latch_part.visual(
            Box((0.022, 0.012, 0.012)),
            origin=Origin(xyz=(0.0, -0.012, 0.026)),
            material=steel,
            name="hook_tip",
        )

    model.articulation(
        "main_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=box_body,
        child=lid,
        origin=Origin(xyz=(lid_center_x, -outer_d * 0.5, body_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "carry_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, handle_pivot_y, handle_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=0.0, upper=1.52),
    )
    model.articulation(
        "organizer_door_hinge",
        ArticulationType.REVOLUTE,
        parent=box_body,
        child=organizer_door,
        origin=Origin(xyz=(outer_w * 0.5, -door_w * 0.5, 0.024)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.85),
    )
    model.articulation(
        "left_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=box_body,
        child=left_latch,
        origin=Origin(xyz=(lid_center_x + latch_offsets[0], latch_origin_y, latch_origin_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "right_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=box_body,
        child="right_latch",
        origin=Origin(xyz=(lid_center_x + latch_offsets[1], latch_origin_y, latch_origin_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    box_body = object_model.get_part("box_body")
    lid = object_model.get_part("main_lid")
    handle = object_model.get_part("carry_handle")
    door = object_model.get_part("organizer_door")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")
    lid_hinge = object_model.get_articulation("main_lid_hinge")
    handle_hinge = object_model.get_articulation("carry_handle_pivot")
    door_hinge = object_model.get_articulation("organizer_door_hinge")
    left_latch_hinge = object_model.get_articulation("left_latch_pivot")
    right_latch_hinge = object_model.get_articulation("right_latch_pivot")

    ctx.expect_gap(
        lid,
        box_body,
        axis="z",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem="front_fascia",
        name="lid sits on the toolbox rim when closed",
    )
    ctx.expect_overlap(
        lid,
        box_body,
        axes="xy",
        min_overlap=0.18,
        name="lid covers the main compartment footprint",
    )
    ctx.expect_gap(
        door,
        box_body,
        axis="x",
        max_gap=0.012,
        max_penetration=0.0,
        name="organizer door closes flush to the end frame",
    )
    ctx.expect_gap(
        handle,
        lid,
        axis="z",
        positive_elem="handle_grip",
        negative_elem="lid_cap",
        min_gap=0.003,
        max_gap=0.020,
        name="handle stows just above the domed lid",
    )
    ctx.expect_overlap(
        left_latch,
        lid,
        axes="xz",
        elem_a="hook_tip",
        elem_b="keeper_1",
        min_overlap=0.005,
        name="left latch hook aligns under its keeper",
    )
    ctx.expect_overlap(
        right_latch,
        lid,
        axes="xz",
        elem_a="hook_tip",
        elem_b="keeper_2",
        min_overlap=0.005,
        name="right latch hook aligns under its keeper",
    )

    closed_lid_front = ctx.part_element_world_aabb(lid, elem="front_fascia")
    with ctx.pose({lid_hinge: 1.15}):
        open_lid_front = ctx.part_element_world_aabb(lid, elem="front_fascia")
    ctx.check(
        "lid front edge lifts high when opened",
        closed_lid_front is not None
        and open_lid_front is not None
        and open_lid_front[0][2] > closed_lid_front[0][2] + 0.09,
        details=f"closed={closed_lid_front}, open={open_lid_front}",
    )

    closed_door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.2}):
        open_door_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "organizer door swings outward from the end wall",
        closed_door_panel is not None
        and open_door_panel is not None
        and open_door_panel[1][0] > closed_door_panel[1][0] + 0.07,
        details=f"closed={closed_door_panel}, open={open_door_panel}",
    )

    closed_handle_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({handle_hinge: 1.25}):
        open_handle_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    ctx.check(
        "carry handle swings up for lifting",
        closed_handle_grip is not None
        and open_handle_grip is not None
        and open_handle_grip[0][2] > closed_handle_grip[0][2] + 0.05,
        details=f"closed={closed_handle_grip}, open={open_handle_grip}",
    )

    left_grip_closed = ctx.part_element_world_aabb(left_latch, elem="grip")
    right_grip_closed = ctx.part_element_world_aabb(right_latch, elem="grip")
    with ctx.pose({left_latch_hinge: 1.0, right_latch_hinge: 1.0}):
        left_grip_open = ctx.part_element_world_aabb(left_latch, elem="grip")
        right_grip_open = ctx.part_element_world_aabb(right_latch, elem="grip")
    ctx.check(
        "left latch flips forward off the keeper",
        left_grip_closed is not None
        and left_grip_open is not None
        and left_grip_open[0][1] > left_grip_closed[0][1] + 0.04
        and left_grip_open[0][2] > left_grip_closed[0][2] + 0.02,
        details=f"closed={left_grip_closed}, open={left_grip_open}",
    )
    ctx.check(
        "right latch flips forward off the keeper",
        right_grip_closed is not None
        and right_grip_open is not None
        and right_grip_open[0][1] > right_grip_closed[0][1] + 0.04
        and right_grip_open[0][2] > right_grip_closed[0][2] + 0.02,
        details=f"closed={right_grip_closed}, open={right_grip_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
