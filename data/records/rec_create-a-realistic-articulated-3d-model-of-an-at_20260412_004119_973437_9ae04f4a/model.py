from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


STEERING_LIMIT = 0.52


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple((a[i] + b[i]) * 0.5 for i in range(3))


def _bar_rpy(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    planar = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, planar)
    return (0.0, pitch, yaw)


def _cylinder_rpy(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    planar = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(planar, dz)
    return (0.0, pitch, yaw)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def add_bar(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    width: float,
    height: float,
    material: Material,
    name: str,
) -> None:
    part.visual(
        Box((_distance(start, end), width, height)),
        origin=Origin(xyz=_midpoint(start, end), rpy=_bar_rpy(start, end)),
        material=material,
        name=name,
    )


def add_cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(start, end)),
        origin=Origin(xyz=_midpoint(start, end), rpy=_cylinder_rpy(start, end)),
        material=material,
        name=name,
    )


def add_wheel_pair(
    part,
    *,
    mesh_wheel,
    mesh_tire,
    center: tuple[float, float, float],
    material_wheel: Material,
    material_tire: Material,
    wheel_name: str,
    tire_name: str,
) -> None:
    rotation = (0.0, 0.0, math.pi * 0.5)
    part.visual(
        mesh_wheel,
        origin=Origin(xyz=center, rpy=rotation),
        material=material_wheel,
        name=wheel_name,
    )
    part.visual(
        mesh_tire,
        origin=Origin(xyz=center, rpy=rotation),
        material=material_tire,
        name=tire_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="atv_quad_bike")

    frame_black = model.material("frame_black", rgba=(0.12, 0.12, 0.12, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.18, 1.0))
    body_red = model.material("body_red", rgba=(0.78, 0.10, 0.08, 1.0))
    seat_black = model.material("seat_black", rgba=(0.08, 0.08, 0.08, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    shock_silver = model.material("shock_silver", rgba=(0.72, 0.72, 0.72, 1.0))
    fork_gray = model.material("fork_gray", rgba=(0.34, 0.36, 0.38, 1.0))
    footrest_black = model.material("footrest_black", rgba=(0.10, 0.10, 0.10, 1.0))

    front_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.185,
            0.130,
            rim=WheelRim(
                inner_radius=0.132,
                flange_height=0.010,
                flange_thickness=0.004,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.046,
                width=0.080,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=4,
                    circle_diameter=0.046,
                    hole_diameter=0.006,
                ),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="split_y", count=6, thickness=0.004, window_radius=0.016),
            bore=WheelBore(style="round", diameter=0.016),
        ),
        "front_wheel",
    )
    front_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.300,
            0.180,
            inner_radius=0.182,
            tread=TireTread(style="block", depth=0.010, count=20, land_ratio=0.54),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.010, radius=0.003),
        ),
        "front_tire",
    )
    rear_wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.195,
            0.145,
            rim=WheelRim(
                inner_radius=0.140,
                flange_height=0.012,
                flange_thickness=0.004,
                bead_seat_depth=0.004,
            ),
            hub=WheelHub(
                radius=0.050,
                width=0.090,
                cap_style="domed",
                bolt_pattern=BoltPattern(
                    count=4,
                    circle_diameter=0.050,
                    hole_diameter=0.006,
                ),
            ),
            face=WheelFace(dish_depth=0.012, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="split_y", count=6, thickness=0.005, window_radius=0.018),
            bore=WheelBore(style="round", diameter=0.018),
        ),
        "rear_wheel",
    )
    rear_tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.330,
            0.220,
            inner_radius=0.192,
            tread=TireTread(style="block", depth=0.012, count=22, land_ratio=0.56),
            sidewall=TireSidewall(style="square", bulge=0.028),
            shoulder=TireShoulder(width=0.012, radius=0.004),
        ),
        "rear_tire",
    )

    front_fender_shape = (
        cq.Workplane("XY")
        .box(0.340, 0.240, 0.060)
        .faces(">Z")
        .edges()
        .fillet(0.022)
    )
    rear_fender_shape = (
        cq.Workplane("XY")
        .box(0.760, 0.820, 0.060)
        .faces(">Z")
        .edges()
        .fillet(0.030)
    )
    seat_shape = (
        cq.Workplane("XY")
        .box(0.520, 0.260, 0.100)
        .faces(">Z")
        .edges()
        .fillet(0.040)
    )

    chassis = model.part("chassis")

    chassis.visual(
        Box((0.960, 0.050, 0.050)),
        origin=Origin(xyz=(-0.010, 0.180, 0.340)),
        material=frame_black,
        name="left_rail",
    )
    chassis.visual(
        Box((0.960, 0.050, 0.050)),
        origin=Origin(xyz=(-0.010, -0.180, 0.340)),
        material=frame_black,
        name="right_rail",
    )
    chassis.visual(
        Box((0.160, 0.500, 0.050)),
        origin=Origin(xyz=(0.360, 0.000, 0.345)),
        material=frame_black,
        name="front_crossmember",
    )
    chassis.visual(
        Box((0.180, 0.580, 0.050)),
        origin=Origin(xyz=(-0.430, 0.000, 0.345)),
        material=frame_black,
        name="rear_crossmember",
    )
    chassis.visual(
        Box((0.560, 0.300, 0.030)),
        origin=Origin(xyz=(-0.030, 0.000, 0.260)),
        material=satin_black,
        name="skid_plate",
    )
    chassis.visual(
        Box((0.240, 0.220, 0.170)),
        origin=Origin(xyz=(0.020, 0.000, 0.430)),
        material=frame_black,
        name="engine_block",
    )
    chassis.visual(
        Box((0.520, 0.160, 0.090)),
        origin=Origin(xyz=(-0.030, 0.000, 0.310)),
        material=frame_black,
        name="center_tunnel",
    )
    chassis.visual(
        Box((0.420, 0.240, 0.300)),
        origin=Origin(xyz=(0.090, 0.000, 0.500)),
        material=body_red,
        name="console",
    )
    chassis.visual(
        Box((0.540, 0.360, 0.280)),
        origin=Origin(xyz=(-0.280, 0.000, 0.530)),
        material=body_red,
        name="rear_body_support",
    )
    chassis.visual(
        Box((0.360, 0.700, 0.060)),
        origin=Origin(xyz=(0.560, 0.000, 0.620)),
        material=body_red,
        name="front_bridge",
    )
    chassis.visual(
        Box((0.300, 0.220, 0.120)),
        origin=Origin(xyz=(0.380, 0.000, 0.600)),
        material=body_red,
        name="nose",
    )
    chassis.visual(
        mesh_from_cadquery(front_fender_shape, "front_left_fender"),
        origin=Origin(xyz=(0.620, 0.470, 0.670)),
        material=body_red,
        name="front_left_fender",
    )
    chassis.visual(
        mesh_from_cadquery(front_fender_shape, "front_right_fender"),
        origin=Origin(xyz=(0.620, -0.470, 0.670)),
        material=body_red,
        name="front_right_fender",
    )
    chassis.visual(
        mesh_from_cadquery(rear_fender_shape, "rear_fender_deck"),
        origin=Origin(xyz=(-0.300, 0.000, 0.700)),
        material=body_red,
        name="rear_deck",
    )
    chassis.visual(
        mesh_from_cadquery(seat_shape, "seat_cushion"),
        origin=Origin(xyz=(-0.200, 0.000, 0.780)),
        material=seat_black,
        name="seat",
    )
    chassis.visual(
        Box((0.380, 0.160, 0.030)),
        origin=Origin(xyz=(-0.030, 0.310, 0.280)),
        material=footrest_black,
        name="left_footrest",
    )
    chassis.visual(
        Box((0.380, 0.160, 0.030)),
        origin=Origin(xyz=(-0.030, -0.310, 0.280)),
        material=footrest_black,
        name="right_footrest",
    )
    chassis.visual(
        Box((0.260, 0.050, 0.080)),
        origin=Origin(xyz=(-0.030, 0.220, 0.315)),
        material=frame_black,
        name="left_footrest_support",
    )
    chassis.visual(
        Box((0.260, 0.050, 0.080)),
        origin=Origin(xyz=(-0.030, -0.220, 0.315)),
        material=frame_black,
        name="right_footrest_support",
    )
    chassis.visual(
        Box((0.200, 0.120, 0.030)),
        origin=Origin(xyz=(-0.040, 0.270, 0.295)),
        material=frame_black,
        name="left_footrest_bridge",
    )
    chassis.visual(
        Box((0.200, 0.120, 0.030)),
        origin=Origin(xyz=(-0.040, -0.270, 0.295)),
        material=frame_black,
        name="right_footrest_bridge",
    )
    chassis.visual(
        Box((0.120, 0.600, 0.040)),
        origin=Origin(xyz=(0.760, 0.000, 0.360)),
        material=frame_black,
        name="front_bumper",
    )
    chassis.visual(
        Cylinder(radius=0.024, length=0.060),
        origin=Origin(xyz=(0.180, 0.000, 0.620)),
        material=frame_black,
        name="steering_head",
    )
    add_bar(
        chassis,
        (0.705, 0.240, 0.380),
        (0.430, 0.220, 0.520),
        width=0.028,
        height=0.028,
        material=frame_black,
        name="front_bumper_left_support",
    )
    add_bar(
        chassis,
        (0.705, -0.240, 0.380),
        (0.430, -0.220, 0.520),
        width=0.028,
        height=0.028,
        material=frame_black,
        name="front_bumper_right_support",
    )
    chassis.visual(
        Box((0.220, 0.060, 0.070)),
        origin=Origin(xyz=(0.270, 0.170, 0.295)),
        material=frame_black,
        name="left_lower_arm_pivot",
    )
    chassis.visual(
        Box((0.220, 0.060, 0.070)),
        origin=Origin(xyz=(0.270, -0.170, 0.295)),
        material=frame_black,
        name="right_lower_arm_pivot",
    )
    chassis.visual(
        Box((0.180, 0.060, 0.060)),
        origin=Origin(xyz=(0.265, 0.160, 0.415)),
        material=frame_black,
        name="left_upper_arm_pivot",
    )
    chassis.visual(
        Box((0.180, 0.060, 0.060)),
        origin=Origin(xyz=(0.265, -0.160, 0.415)),
        material=frame_black,
        name="right_upper_arm_pivot",
    )
    chassis.visual(
        Box((0.120, 0.040, 0.028)),
        origin=Origin(xyz=(0.560, 0.332, 0.320)),
        material=fork_gray,
        name="left_steering_support",
    )
    chassis.visual(
        Box((0.120, 0.040, 0.028)),
        origin=Origin(xyz=(0.560, -0.332, 0.320)),
        material=fork_gray,
        name="right_steering_support",
    )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        lower_outer = (0.545, 0.335 * side_sign, 0.295)
        upper_outer = (0.535, 0.325 * side_sign, 0.415)
        add_bar(
            chassis,
            (0.190, 0.165 * side_sign, 0.295),
            lower_outer,
            width=0.022,
            height=0.022,
            material=frame_black,
            name=f"{side_name}_lower_arm_front",
        )
        add_bar(
            chassis,
            (0.340, 0.165 * side_sign, 0.295),
            lower_outer,
            width=0.022,
            height=0.022,
            material=frame_black,
            name=f"{side_name}_lower_arm_rear",
        )
        add_bar(
            chassis,
            (0.210, 0.155 * side_sign, 0.415),
            upper_outer,
            width=0.020,
            height=0.020,
            material=frame_black,
            name=f"{side_name}_upper_arm_front",
        )
        add_bar(
            chassis,
            (0.330, 0.155 * side_sign, 0.415),
            upper_outer,
            width=0.020,
            height=0.020,
            material=frame_black,
            name=f"{side_name}_upper_arm_rear",
        )
        add_cylinder_between(
            chassis,
            (0.435, 0.235 * side_sign, 0.325),
            (0.430, 0.210 * side_sign, 0.575),
            radius=0.022,
            material=shock_silver,
            name=f"{side_name}_front_shock",
        )

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        wheel_center = (-0.580, 0.450 * side_sign, 0.330)
        add_wheel_pair(
            chassis,
            mesh_wheel=rear_wheel_mesh,
            mesh_tire=rear_tire_mesh,
            center=wheel_center,
            material_wheel=rim_silver,
            material_tire=satin_black,
            wheel_name=f"{side_name}_rear_wheel",
            tire_name=f"{side_name}_rear_tire",
        )
        chassis.visual(
            Box((0.180, 0.110, 0.140)),
            origin=Origin(xyz=(-0.500, 0.390 * side_sign, 0.340)),
            material=fork_gray,
            name=f"{side_name}_rear_hub",
        )
        add_bar(
            chassis,
            (-0.190, 0.185 * side_sign, 0.345),
            (-0.470, 0.335 * side_sign, 0.345),
            width=0.030,
            height=0.030,
            material=frame_black,
            name=f"{side_name}_swingarm_upper",
        )
        add_bar(
            chassis,
            (-0.210, 0.135 * side_sign, 0.300),
            (-0.470, 0.320 * side_sign, 0.300),
            width=0.030,
            height=0.030,
            material=frame_black,
            name=f"{side_name}_swingarm_lower",
        )
        add_cylinder_between(
            chassis,
            (-0.280, 0.225 * side_sign, 0.365),
            (-0.360, 0.270 * side_sign, 0.575),
            radius=0.025,
            material=shock_silver,
            name=f"{side_name}_rear_shock",
        )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.020, length=0.170),
        origin=Origin(xyz=(0.000, 0.000, 0.085)),
        material=frame_black,
        name="stem",
    )
    handlebar.visual(
        Cylinder(radius=0.016, length=0.620),
        origin=Origin(xyz=(0.020, 0.000, 0.180), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=frame_black,
        name="bar",
    )
    handlebar.visual(
        Cylinder(radius=0.012, length=0.180),
        origin=Origin(xyz=(0.030, 0.265, 0.205), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=seat_black,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.012, length=0.180),
        origin=Origin(xyz=(0.030, -0.265, 0.205), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=seat_black,
        name="right_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.009, length=0.260),
        origin=Origin(xyz=(0.015, 0.000, 0.130), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=fork_gray,
        name="brace",
    )
    handlebar.visual(
        Box((0.080, 0.110, 0.060)),
        origin=Origin(xyz=(0.055, 0.000, 0.190)),
        material=satin_black,
        name="pod",
    )

    for side_name, side_sign in (("front_left_corner", 1.0), ("front_right_corner", -1.0)):
        corner = model.part(side_name)
        wheel_offset = 0.105 * side_sign
        corner.visual(
            Box((0.060, 0.150, 0.180)),
            origin=Origin(xyz=(0.000, 0.045 * side_sign, 0.110)),
            material=fork_gray,
            name="upright",
        )
        corner.visual(
            Box((0.120, 0.040, 0.028)),
            origin=Origin(xyz=(-0.060, -0.021 * side_sign, 0.028)),
            material=fork_gray,
            name="bearing_shoe",
        )
        corner.visual(
            Cylinder(radius=0.040, length=0.140),
            origin=Origin(xyz=(0.000, 0.070 * side_sign, 0.000), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=fork_gray,
            name="hub",
        )
        corner.visual(
            Box((0.110, 0.032, 0.032)),
            origin=Origin(xyz=(0.015, 0.018 * side_sign, 0.090), rpy=(0.0, 0.0, 0.35 * side_sign)),
            material=fork_gray,
            name="steering_arm",
        )
        add_wheel_pair(
            corner,
            mesh_wheel=front_wheel_mesh,
            mesh_tire=front_tire_mesh,
            center=(0.000, wheel_offset, 0.000),
            material_wheel=rim_silver,
            material_tire=satin_black,
            wheel_name="wheel",
            tire_name="tire",
        )

    steering_limits = MotionLimits(
        effort=18.0,
        velocity=2.5,
        lower=-STEERING_LIMIT,
        upper=STEERING_LIMIT,
    )
    steering = model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=handlebar,
        origin=Origin(xyz=(0.180, 0.000, 0.650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=steering_limits,
    )
    model.articulation(
        "front_left_steer",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child="front_left_corner",
        origin=Origin(xyz=(0.620, 0.353, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-STEERING_LIMIT,
            upper=STEERING_LIMIT,
        ),
        mimic=Mimic(joint="steering"),
    )
    model.articulation(
        "front_right_steer",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child="front_right_corner",
        origin=Origin(xyz=(0.620, -0.353, 0.320)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=-STEERING_LIMIT,
            upper=STEERING_LIMIT,
        ),
        mimic=Mimic(joint="steering"),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    handlebar = object_model.get_part("handlebar")
    front_left = object_model.get_part("front_left_corner")
    front_right = object_model.get_part("front_right_corner")
    steering = object_model.get_articulation("steering")

    ctx.expect_origin_distance(
        front_left,
        front_right,
        axes="y",
        min_dist=0.70,
        name="front track is wide enough for an ATV stance",
    )
    ctx.expect_origin_distance(
        handlebar,
        chassis,
        axes="x",
        min_dist=0.15,
        max_dist=0.40,
        name="handlebar mount sits ahead of the rider position",
    )

    seat_aabb = ctx.part_element_world_aabb(chassis, elem="seat")
    footrest_aabb = ctx.part_element_world_aabb(chassis, elem="left_footrest")
    ctx.check(
        "seat sits well above the footrests",
        seat_aabb is not None
        and footrest_aabb is not None
        and seat_aabb[0][2] > footrest_aabb[1][2] + 0.40,
        details=f"seat={seat_aabb}, footrest={footrest_aabb}",
    )

    left_tire_rest = ctx.part_element_world_aabb(front_left, elem="tire")
    right_tire_rest = ctx.part_element_world_aabb(front_right, elem="tire")

    def aabb_center_x(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0]) if aabb is not None else None

    with ctx.pose({steering: STEERING_LIMIT}):
        left_tire_turn = ctx.part_element_world_aabb(front_left, elem="tire")
        right_tire_turn = ctx.part_element_world_aabb(front_right, elem="tire")

    left_rest_x = aabb_center_x(left_tire_rest)
    right_rest_x = aabb_center_x(right_tire_rest)
    left_turn_x = aabb_center_x(left_tire_turn)
    right_turn_x = aabb_center_x(right_tire_turn)

    ctx.check(
        "front steering yaws both wheels together",
        left_rest_x is not None
        and right_rest_x is not None
        and left_turn_x is not None
        and right_turn_x is not None
        and left_turn_x < left_rest_x - 0.01
        and right_turn_x > right_rest_x + 0.01,
        details=(
            f"left_rest_x={left_rest_x}, left_turn_x={left_turn_x}, "
            f"right_rest_x={right_rest_x}, right_turn_x={right_turn_x}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
