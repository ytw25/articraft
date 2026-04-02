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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_box(part, size, center, *, material, name):
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_rod(part, *, center, length, radius, axis, material, name):
    if axis == "x":
        rpy = (0.0, math.pi / 2.0, 0.0)
    elif axis == "y":
        rpy = (math.pi / 2.0, 0.0, 0.0)
    elif axis == "z":
        rpy = (0.0, 0.0, 0.0)
    else:
        raise ValueError(f"unsupported axis {axis!r}")

    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_dishwasher")

    shell_white = model.material("shell_white", rgba=(0.93, 0.93, 0.94, 1.0))
    kick_gray = model.material("kick_gray", rgba=(0.18, 0.18, 0.19, 1.0))
    tub_steel = model.material("tub_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    rack_gray = model.material("rack_gray", rgba=(0.71, 0.73, 0.75, 1.0))
    basket_gray = model.material("basket_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.12, 0.12, 0.13, 1.0))

    body_width = 0.598
    body_depth = 0.610
    body_height = 0.850
    body_half_w = body_width / 2.0
    body_half_d = body_depth / 2.0

    body = model.part("body")

    # Exterior shell and frame.
    _add_box(
        body,
        (0.546, 0.018, body_height),
        (-0.032, -0.290, body_height / 2.0),
        material=shell_white,
        name="left_outer_panel",
    )
    _add_box(
        body,
        (0.546, 0.018, body_height),
        (-0.032, 0.290, body_height / 2.0),
        material=shell_white,
        name="right_outer_panel",
    )
    _add_box(
        body,
        (body_depth, body_width, 0.020),
        (0.0, 0.0, 0.840),
        material=shell_white,
        name="top_panel",
    )
    _add_box(
        body,
        (0.018, 0.562, 0.812),
        (-0.296, 0.0, 0.406),
        material=shell_white,
        name="back_panel",
    )
    _add_box(
        body,
        (0.546, 0.562, 0.018),
        (-0.032, 0.0, 0.109),
        material=shell_white,
        name="base_tray",
    )
    # Recessed lower service panel / toe kick, narrower than the full cabinet width
    # so the thick door side hems clear it when swung down.
    _add_box(
        body,
        (0.050, 0.530, 0.094),
        (0.185, 0.0, 0.047),
        material=kick_gray,
        name="toe_kick",
    )
    _add_box(
        body,
        (0.064, 0.562, 0.030),
        (0.273, 0.0, 0.835),
        material=shell_white,
        name="top_fascia",
    )

    # Inner tub shell.
    _add_box(
        body,
        (0.470, 0.008, 0.680),
        (-0.015, -0.277, 0.460),
        material=tub_steel,
        name="left_tub_wall",
    )
    _add_box(
        body,
        (0.470, 0.008, 0.680),
        (-0.015, 0.277, 0.460),
        material=tub_steel,
        name="right_tub_wall",
    )
    _add_box(
        body,
        (0.008, 0.554, 0.680),
        (-0.246, 0.0, 0.460),
        material=tub_steel,
        name="tub_back_wall",
    )
    _add_box(
        body,
        (0.466, 0.554, 0.010),
        (-0.013, 0.0, 0.125),
        material=tub_steel,
        name="tub_floor",
    )
    _add_box(
        body,
        (0.466, 0.554, 0.010),
        (-0.013, 0.0, 0.795),
        material=tub_steel,
        name="tub_ceiling",
    )

    # Rack tracks mounted to the tub side walls.
    _add_box(
        body,
        (0.440, 0.030, 0.020),
        (-0.030, -0.262, 0.146),
        material=tub_steel,
        name="lower_left_track",
    )
    _add_box(
        body,
        (0.440, 0.030, 0.020),
        (-0.030, 0.262, 0.146),
        material=tub_steel,
        name="lower_right_track",
    )
    _add_box(
        body,
        (0.420, 0.030, 0.020),
        (-0.040, -0.262, 0.520),
        material=tub_steel,
        name="upper_left_track",
    )
    _add_box(
        body,
        (0.420, 0.030, 0.020),
        (-0.040, 0.262, 0.520),
        material=tub_steel,
        name="upper_right_track",
    )

    # Feet to read as a freestanding appliance.
    for name, x, y in (
        ("front_left_foot", 0.210, -0.232),
        ("front_right_foot", 0.210, 0.232),
        ("rear_left_foot", -0.220, -0.232),
        ("rear_right_foot", -0.220, 0.232),
    ):
        body.visual(
            Cylinder(radius=0.016, length=0.100),
            origin=Origin(xyz=(x, y, 0.050)),
            material=kick_gray,
            name=name,
        )

    body.inertial = Inertial.from_geometry(
        Box((body_depth, body_width, body_height)),
        mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    door = model.part("door")
    door_height = 0.720
    door_width = 0.590
    door_thickness = 0.070
    _add_box(
        door,
        (0.014, door_width, door_height),
        (0.063, 0.0, door_height / 2.0),
        material=shell_white,
        name="front_panel",
    )
    _add_box(
        door,
        (0.012, 0.554, 0.680),
        (0.006, 0.0, 0.360),
        material=tub_steel,
        name="inner_liner",
    )
    _add_box(
        door,
        (0.058, 0.556, 0.024),
        (0.041, 0.0, 0.012),
        material=kick_gray,
        name="bottom_edge",
    )
    _add_box(
        door,
        (0.058, 0.556, 0.020),
        (0.041, 0.0, 0.710),
        material=kick_gray,
        name="top_edge",
    )
    _add_box(
        door,
        (0.058, 0.012, door_height),
        (0.041, -0.279, door_height / 2.0),
        material=kick_gray,
        name="left_edge",
    )
    _add_box(
        door,
        (0.058, 0.012, door_height),
        (0.041, 0.279, door_height / 2.0),
        material=kick_gray,
        name="right_edge",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_thickness, door_width, door_height)),
        mass=8.5,
        origin=Origin(xyz=(door_thickness / 2.0, 0.0, door_height / 2.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.235, 0.0, 0.100)),
        # Closed door geometry rises along +Z from the bottom hinge line.
        # +Y makes positive q swing the top edge outward along +X and downward.
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(87.0),
        ),
    )

    lower_rack = model.part("lower_rack")
    for name, center, length, axis, radius in (
        ("left_bottom_rail", (-0.245, -0.228, 0.035), 0.470, "x", 0.0042),
        ("right_bottom_rail", (-0.245, 0.228, 0.035), 0.470, "x", 0.0042),
        ("front_bottom_rail", (-0.010, 0.0, 0.035), 0.456, "y", 0.0042),
        ("rear_bottom_rail", (-0.480, 0.0, 0.035), 0.456, "y", 0.0042),
        ("left_top_rail", (-0.240, -0.228, 0.100), 0.460, "x", 0.0038),
        ("right_top_rail", (-0.240, 0.228, 0.100), 0.460, "x", 0.0038),
        ("front_top_rail", (-0.010, 0.0, 0.100), 0.450, "y", 0.0038),
        ("rear_top_rail", (-0.470, 0.0, 0.100), 0.450, "y", 0.0038),
    ):
        _add_rod(
            lower_rack,
            center=center,
            length=length,
            radius=radius,
            axis=axis,
            material=rack_gray,
            name=name,
        )
    for name, x, y in (
        ("front_left_post", -0.010, -0.228),
        ("front_right_post", -0.010, 0.228),
        ("rear_left_post", -0.480, -0.228),
        ("rear_right_post", -0.480, 0.228),
    ):
        _add_rod(
            lower_rack,
            center=(x, y, 0.0675),
            length=0.065,
            radius=0.0030,
            axis="z",
            material=rack_gray,
            name=name,
        )
    for index, x in enumerate((-0.435, -0.355, -0.275, -0.195, -0.115, -0.035), start=1):
        _add_rod(
            lower_rack,
            center=(x, 0.0, 0.038),
            length=0.452,
            radius=0.0023,
            axis="y",
            material=rack_gray,
            name=f"floor_wire_{index}",
        )
    _add_box(
        lower_rack,
        (0.390, 0.024, 0.012),
        (-0.205, -0.239, 0.018),
        material=rack_gray,
        name="left_runner",
    )
    _add_box(
        lower_rack,
        (0.390, 0.024, 0.012),
        (-0.205, 0.239, 0.018),
        material=rack_gray,
        name="right_runner",
    )
    for name, x, y in (
        ("left_front_runner_bracket", -0.060, -0.235),
        ("left_rear_runner_bracket", -0.350, -0.235),
        ("right_front_runner_bracket", -0.060, 0.235),
        ("right_rear_runner_bracket", -0.350, 0.235),
    ):
        _add_box(
            lower_rack,
            (0.022, 0.020, 0.024),
            (x, y, 0.031),
            material=rack_gray,
            name=name,
        )
    for name, x, y in (
        ("front_left_axle_block", -0.060, -0.255),
        ("rear_left_axle_block", -0.350, -0.255),
        ("front_right_axle_block", -0.060, 0.255),
        ("rear_right_axle_block", -0.350, 0.255),
    ):
        _add_box(
            lower_rack,
            (0.020, 0.012, 0.022),
            (x, y, 0.023),
            material=rack_gray,
            name=name,
        )
    for name, x, y in (
        ("front_left_wheel", -0.060, -0.255),
        ("rear_left_wheel", -0.350, -0.255),
        ("front_right_wheel", -0.060, 0.255),
        ("rear_right_wheel", -0.350, 0.255),
    ):
        _add_rod(
            lower_rack,
            center=(x, y, 0.023),
            length=0.010,
            radius=0.013,
            axis="y",
            material=wheel_dark,
            name=name,
        )
    lower_rack.inertial = Inertial.from_geometry(
        Box((0.490, 0.470, 0.120)),
        mass=2.2,
        origin=Origin(xyz=(-0.245, 0.0, 0.060)),
    )

    model.articulation(
        "body_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lower_rack,
        origin=Origin(xyz=(0.245, 0.0, 0.146)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=0.230,
        ),
    )

    upper_rack = model.part("upper_rack")
    for name, center, length, axis, radius in (
        ("left_bottom_rail", (-0.225, -0.223, -0.080), 0.430, "x", 0.0038),
        ("right_bottom_rail", (-0.225, 0.223, -0.080), 0.430, "x", 0.0038),
        ("front_bottom_rail", (-0.010, 0.0, -0.080), 0.446, "y", 0.0038),
        ("rear_bottom_rail", (-0.440, 0.0, -0.080), 0.446, "y", 0.0038),
        ("left_top_rail", (-0.225, -0.223, -0.020), 0.430, "x", 0.0035),
        ("right_top_rail", (-0.225, 0.223, -0.020), 0.430, "x", 0.0035),
        ("front_top_rail", (-0.010, 0.0, -0.020), 0.446, "y", 0.0035),
        ("rear_top_rail", (-0.440, 0.0, -0.020), 0.446, "y", 0.0035),
    ):
        _add_rod(
            upper_rack,
            center=center,
            length=length,
            radius=radius,
            axis=axis,
            material=rack_gray,
            name=name,
        )
    for name, x, y in (
        ("front_left_post", -0.010, -0.223),
        ("front_right_post", -0.010, 0.223),
        ("rear_left_post", -0.440, -0.223),
        ("rear_right_post", -0.440, 0.223),
    ):
        _add_rod(
            upper_rack,
            center=(x, y, -0.050),
            length=0.060,
            radius=0.0028,
            axis="z",
            material=rack_gray,
            name=name,
        )
    for index, x in enumerate((-0.405, -0.335, -0.265, -0.195, -0.125, -0.055), start=1):
        _add_rod(
            upper_rack,
            center=(x, 0.0, -0.077),
            length=0.444,
            radius=0.0021,
            axis="y",
            material=rack_gray,
            name=f"floor_wire_{index}",
        )
    _add_box(
        upper_rack,
        (0.300, 0.024, 0.012),
        (-0.190, -0.236, -0.024),
        material=rack_gray,
        name="left_runner",
    )
    _add_box(
        upper_rack,
        (0.300, 0.024, 0.012),
        (-0.190, 0.236, -0.024),
        material=rack_gray,
        name="right_runner",
    )
    for name, x, y in (
        ("left_front_hanger", -0.060, -0.243),
        ("left_rear_hanger", -0.320, -0.243),
        ("right_front_hanger", -0.060, 0.243),
        ("right_rear_hanger", -0.320, 0.243),
    ):
        _add_box(
            upper_rack,
            (0.022, 0.050, 0.020),
            (x, y, -0.020),
            material=rack_gray,
            name=name,
        )
    upper_rack.inertial = Inertial.from_geometry(
        Box((0.455, 0.458, 0.105)),
        mass=1.5,
        origin=Origin(xyz=(-0.225, 0.0, -0.050)),
    )

    model.articulation(
        "body_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_rack,
        origin=Origin(xyz=(0.200, 0.0, 0.520)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.30,
            lower=0.0,
            upper=0.180,
        ),
    )

    basket = model.part("silverware_basket")
    _add_box(
        basket,
        (0.150, 0.100, 0.010),
        (0.0, 0.0, -0.055),
        material=basket_gray,
        name="basket_floor",
    )
    _add_box(
        basket,
        (0.160, 0.010, 0.120),
        (0.0, -0.045, 0.000),
        material=basket_gray,
        name="left_wall",
    )
    _add_box(
        basket,
        (0.160, 0.010, 0.120),
        (0.0, 0.045, 0.000),
        material=basket_gray,
        name="right_wall",
    )
    _add_box(
        basket,
        (0.010, 0.100, 0.120),
        (-0.075, 0.0, 0.000),
        material=basket_gray,
        name="rear_wall",
    )
    _add_box(
        basket,
        (0.010, 0.100, 0.120),
        (0.075, 0.0, 0.000),
        material=basket_gray,
        name="front_wall",
    )
    _add_box(
        basket,
        (0.010, 0.090, 0.108),
        (0.000, 0.0, 0.000),
        material=basket_gray,
        name="center_divider",
    )
    _add_box(
        basket,
        (0.022, 0.024, 0.018),
        (-0.045, 0.0, -0.069),
        material=basket_gray,
        name="left_mount_foot",
    )
    _add_box(
        basket,
        (0.022, 0.024, 0.018),
        (0.045, 0.0, -0.069),
        material=basket_gray,
        name="right_mount_foot",
    )
    basket.inertial = Inertial.from_geometry(
        Box((0.160, 0.100, 0.140)),
        mass=0.45,
    )

    model.articulation(
        "lower_rack_to_basket",
        ArticulationType.FIXED,
        parent=lower_rack,
        child=basket,
        origin=Origin(xyz=(-0.165, 0.135, 0.103)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    basket = object_model.get_part("silverware_basket")

    door_joint = object_model.get_articulation("body_to_door")
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

    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        min_overlap=0.55,
        name="door covers the dishwasher front opening",
    )
    ctx.expect_contact(
        lower_rack,
        body,
        elem_a="front_left_wheel",
        elem_b="lower_left_track",
        contact_tol=0.0015,
        name="lower rack front wheel rides on the side track",
    )
    ctx.expect_contact(
        upper_rack,
        body,
        elem_a="left_front_hanger",
        elem_b="upper_left_track",
        contact_tol=0.0015,
        name="upper rack front hanger rides under the upper rail",
    )
    ctx.expect_contact(
        basket,
        lower_rack,
        contact_tol=0.002,
        name="silverware basket is mounted onto the lower rack",
    )
    ctx.expect_within(
        basket,
        lower_rack,
        axes="xy",
        margin=0.010,
        name="silverware basket stays within the lower rack footprint",
    )

    door_closed_aabb = ctx.part_world_aabb(door)
    lower_rest = ctx.part_world_position(lower_rack)
    upper_rest = ctx.part_world_position(upper_rack)

    door_open = door_joint.motion_limits.upper if door_joint.motion_limits is not None else None
    lower_open = lower_slide.motion_limits.upper if lower_slide.motion_limits is not None else None
    upper_open = upper_slide.motion_limits.upper if upper_slide.motion_limits is not None else None

    with ctx.pose({door_joint: door_open}):
        door_open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door rotates downward from the bottom hinge",
            door_closed_aabb is not None
            and door_open_aabb is not None
            and door_open_aabb[1][0] > door_closed_aabb[1][0] + 0.45
            and door_open_aabb[1][2] < door_closed_aabb[1][2] - 0.55,
            details=f"closed={door_closed_aabb}, open={door_open_aabb}",
        )

    with ctx.pose({door_joint: door_open, lower_slide: lower_open}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no overlaps with door open and lower rack extended"
        )
        ctx.expect_contact(
            lower_rack,
            body,
            elem_a="rear_left_wheel",
            elem_b="lower_left_track",
            contact_tol=0.0015,
            name="rear lower rack wheel stays on the side track when extended",
        )
        ctx.expect_overlap(
            lower_rack,
            body,
            axes="x",
            elem_a="rear_left_wheel",
            elem_b="lower_left_track",
            min_overlap=0.020,
            name="lower rack keeps a wheel engaged on the lower track when extended",
        )
        lower_extended = ctx.part_world_position(lower_rack)
        ctx.check(
            "lower rack slides outward on the lower tracks",
            lower_rest is not None
            and lower_extended is not None
            and lower_extended[0] > lower_rest[0] + 0.18,
            details=f"rest={lower_rest}, extended={lower_extended}",
        )

    with ctx.pose({door_joint: door_open, upper_slide: upper_open}):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no overlaps with door open and upper rack extended"
        )
        ctx.expect_contact(
            upper_rack,
            body,
            elem_a="left_rear_hanger",
            elem_b="upper_left_track",
            contact_tol=0.0015,
            name="upper rack rear hanger stays under the upper track when extended",
        )
        ctx.expect_overlap(
            upper_rack,
            body,
            axes="x",
            elem_a="left_rear_hanger",
            elem_b="upper_left_track",
            min_overlap=0.020,
            name="upper rack retains insertion on the upper rail pair",
        )
        upper_extended = ctx.part_world_position(upper_rack)
        ctx.check(
            "upper rack slides outward under the tub ceiling rails",
            upper_rest is not None
            and upper_extended is not None
            and upper_extended[0] > upper_rest[0] + 0.13,
            details=f"rest={upper_rest}, extended={upper_extended}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
