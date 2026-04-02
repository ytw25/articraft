from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, hypot, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = hypot(dx, dy)
    yaw = atan2(dy, dx)
    pitch = atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_node(part, point, radius: float, material, *, name: str | None = None) -> None:
    part.visual(Sphere(radius=radius), origin=Origin(xyz=point), material=material, name=name)


def _add_wheel_visuals(
    part,
    *,
    tire_radius: float,
    tire_width: float,
    rim_radius: float,
    rim_width: float,
    hub_radius: float,
    hub_width: float,
    rubber,
    rim_material,
    hub_material,
    tire_name: str,
) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=rubber,
        name=tire_name,
    )
    part.visual(
        Cylinder(radius=rim_radius, length=rim_width),
        origin=spin_origin,
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=spin_origin,
        material=hub_material,
        name="hub",
    )
    cap_offset = rim_width * 0.22
    part.visual(
        Cylinder(radius=hub_radius * 0.78, length=hub_width * 0.22),
        origin=Origin(xyz=(cap_offset, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_material,
        name="outer_cap",
    )
    part.visual(
        Cylinder(radius=hub_radius * 0.78, length=hub_width * 0.22),
        origin=Origin(xyz=(-cap_offset, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rim_material,
        name="inner_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_stroller")

    frame_paint = model.material("frame_paint", rgba=(0.34, 0.36, 0.38, 1.0))
    frame_dark = model.material("frame_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    fabric = model.material("fabric", rgba=(0.24, 0.25, 0.27, 1.0))
    canopy_fabric = model.material("canopy_fabric", rgba=(0.30, 0.33, 0.35, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    grip = model.material("grip", rgba=(0.12, 0.12, 0.13, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.48, 0.72, 1.02)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.04, 0.51)),
    )

    tube_radius = 0.012
    node_radius = 0.013
    side_x = 0.152
    seat_front = (side_x - 0.014, 0.15, 0.38)
    rear_hub_upper = (0.170, -0.16, 0.20)
    canopy_hinge_side = (side_x, -0.04, 0.68)
    handle_top = (0.150, -0.24, 0.93)
    front_pivot = (side_x, 0.32, 0.16)

    for sign in (1.0, -1.0):
        fp = (sign * front_pivot[0], front_pivot[1], front_pivot[2])
        sf = (sign * seat_front[0], seat_front[1], seat_front[2])
        rh = (sign * rear_hub_upper[0], rear_hub_upper[1], rear_hub_upper[2])
        ch = (sign * canopy_hinge_side[0], canopy_hinge_side[1], canopy_hinge_side[2])
        ht = (sign * handle_top[0], handle_top[1], handle_top[2])

        _add_member(frame, fp, sf, tube_radius, frame_paint)
        _add_member(frame, sf, rh, tube_radius, frame_paint)
        _add_member(frame, sf, ch, tube_radius, frame_paint)
        _add_member(frame, rh, ht, tube_radius, frame_paint)
        _add_member(frame, ch, ht, tube_radius * 0.95, frame_paint)

        _add_node(frame, sf, node_radius, frame_paint)
        _add_node(frame, rh, node_radius, frame_paint)
        _add_node(frame, ht, node_radius, frame_paint)

    _add_member(
        frame,
        (side_x - 0.014, 0.15, 0.38),
        (-(side_x - 0.014), 0.15, 0.38),
        0.010,
        frame_paint,
        name="seat_front_crossbar",
    )
    _add_member(
        frame,
        (side_x, -0.04, 0.68),
        (-side_x, -0.04, 0.68),
        0.009,
        frame_paint,
        name="upper_back_crossbar",
    )
    _add_member(
        frame,
        (0.170, -0.16, 0.11),
        (-0.170, -0.16, 0.11),
        0.008,
        frame_paint,
        name="rear_axle_bar",
    )
    _add_member(
        frame,
        (0.152, 0.24, 0.26),
        (-0.152, 0.24, 0.26),
        0.010,
        frame_paint,
        name="footrest_bar",
    )

    handle_geom = tube_from_spline_points(
        [
            (-0.150, -0.24, 0.93),
            (-0.080, -0.27, 0.96),
            (0.0, -0.285, 0.975),
            (0.080, -0.27, 0.96),
            (0.150, -0.24, 0.93),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    frame.visual(mesh_from_geometry(handle_geom, "stroller_handlebar"), material=grip, name="handlebar")

    frame.visual(
        Box((0.300, 0.220, 0.005)),
        origin=Origin(xyz=(0.0, 0.10, 0.31), rpy=(0.25, 0.0, 0.0)),
        material=fabric,
        name="seat_sling",
    )
    frame.visual(
        Box((0.300, 0.300, 0.005)),
        origin=Origin(xyz=(0.0, 0.00, 0.46), rpy=(-0.78, 0.0, 0.0)),
        material=fabric,
        name="back_sling",
    )
    frame.visual(
        Box((0.300, 0.110, 0.006)),
        origin=Origin(xyz=(0.0, 0.24, 0.23), rpy=(0.34, 0.0, 0.0)),
        material=fabric,
        name="footrest_panel",
    )

    frame.visual(
        Cylinder(radius=0.011, length=0.024),
        origin=Origin(xyz=(0.152, 0.32, 0.160)),
        material=frame_dark,
        name="left_caster_socket",
    )
    frame.visual(
        Cylinder(radius=0.011, length=0.024),
        origin=Origin(xyz=(-0.152, 0.32, 0.160)),
        material=frame_dark,
        name="right_caster_socket",
    )

    for sign, side_name in ((1.0, "left"), (-1.0, "right")):
        frame.visual(
            Box((0.020, 0.018, 0.090)),
            origin=Origin(xyz=(sign * 0.178, -0.16, 0.155)),
            material=frame_paint,
            name=f"{side_name}_rear_bracket",
        )
        frame.visual(
            Cylinder(radius=0.010, length=0.036),
            origin=Origin(xyz=(sign * 0.190, -0.16, 0.11), rpy=(0.0, pi / 2.0, 0.0)),
            material=frame_dark,
            name=f"{side_name}_rear_stub",
        )
        frame.visual(
            Box((0.030, 0.028, 0.060)),
            origin=Origin(xyz=(sign * 0.167, -0.054, 0.68)),
            material=frame_dark,
            name=f"{side_name}_canopy_support",
        )

    left_front_caster = model.part("left_front_caster_yoke")
    left_front_caster.inertial = Inertial.from_geometry(
        Box((0.05, 0.05, 0.11)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.006, -0.05)),
    )
    left_front_caster.visual(
        Cylinder(radius=0.009, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=frame_dark,
        name="stem",
    )
    left_front_caster.visual(
        Box((0.030, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.003, -0.041)),
        material=frame_dark,
        name="crown",
    )
    left_front_caster.visual(
        Box((0.010, 0.040, 0.062)),
        origin=Origin(xyz=(0.0, -0.006, -0.072)),
        material=frame_dark,
        name="fork_plate",
    )

    right_front_caster = model.part("right_front_caster_yoke")
    right_front_caster.inertial = Inertial.from_geometry(
        Box((0.05, 0.05, 0.11)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.006, -0.05)),
    )
    right_front_caster.visual(
        Cylinder(radius=0.009, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=frame_dark,
        name="stem",
    )
    right_front_caster.visual(
        Box((0.030, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.003, -0.041)),
        material=frame_dark,
        name="crown",
    )
    right_front_caster.visual(
        Box((0.010, 0.040, 0.062)),
        origin=Origin(xyz=(0.0, -0.006, -0.072)),
        material=frame_dark,
        name="fork_plate",
    )

    left_front_inner = model.part("left_front_inner_wheel")
    left_front_inner.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.018),
        mass=0.08,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        left_front_inner,
        tire_radius=0.055,
        tire_width=0.018,
        rim_radius=0.036,
        rim_width=0.012,
        hub_radius=0.014,
        hub_width=0.014,
        rubber=tire_rubber,
        rim_material=rim_silver,
        hub_material=frame_dark,
        tire_name="tire",
    )

    left_front_outer = model.part("left_front_outer_wheel")
    left_front_outer.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.018),
        mass=0.08,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        left_front_outer,
        tire_radius=0.055,
        tire_width=0.018,
        rim_radius=0.036,
        rim_width=0.012,
        hub_radius=0.014,
        hub_width=0.014,
        rubber=tire_rubber,
        rim_material=rim_silver,
        hub_material=frame_dark,
        tire_name="tire",
    )

    right_front_inner = model.part("right_front_inner_wheel")
    right_front_inner.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.018),
        mass=0.08,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        right_front_inner,
        tire_radius=0.055,
        tire_width=0.018,
        rim_radius=0.036,
        rim_width=0.012,
        hub_radius=0.014,
        hub_width=0.014,
        rubber=tire_rubber,
        rim_material=rim_silver,
        hub_material=frame_dark,
        tire_name="tire",
    )

    right_front_outer = model.part("right_front_outer_wheel")
    right_front_outer.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.018),
        mass=0.08,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        right_front_outer,
        tire_radius=0.055,
        tire_width=0.018,
        rim_radius=0.036,
        rim_width=0.012,
        hub_radius=0.014,
        hub_width=0.014,
        rubber=tire_rubber,
        rim_material=rim_silver,
        hub_material=frame_dark,
        tire_name="tire",
    )

    left_rear_wheel = model.part("left_rear_wheel")
    left_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.028),
        mass=0.20,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        left_rear_wheel,
        tire_radius=0.095,
        tire_width=0.028,
        rim_radius=0.066,
        rim_width=0.020,
        hub_radius=0.021,
        hub_width=0.018,
        rubber=tire_rubber,
        rim_material=rim_silver,
        hub_material=frame_dark,
        tire_name="tire",
    )

    right_rear_wheel = model.part("right_rear_wheel")
    right_rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=0.028),
        mass=0.20,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        right_rear_wheel,
        tire_radius=0.095,
        tire_width=0.028,
        rim_radius=0.066,
        rim_width=0.020,
        hub_radius=0.021,
        hub_width=0.018,
        rubber=tire_rubber,
        rim_material=rim_silver,
        hub_material=frame_dark,
        tire_name="tire",
    )

    rear_canopy = model.part("rear_canopy_segment")
    rear_canopy.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.12)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.10, 0.06)),
    )
    rear_canopy.visual(
        Box((0.012, 0.016, 0.050)),
        origin=Origin(xyz=(0.188, 0.008, 0.0)),
        material=frame_dark,
        name="left_hinge_pad",
    )
    rear_canopy.visual(
        Box((0.012, 0.016, 0.050)),
        origin=Origin(xyz=(-0.188, 0.008, 0.0)),
        material=frame_dark,
        name="right_hinge_pad",
    )
    _add_member(rear_canopy, (0.188, 0.008, 0.0), (0.160, 0.086, 0.076), 0.005, frame_dark)
    _add_member(rear_canopy, (0.160, 0.086, 0.076), (0.150, 0.138, 0.044), 0.005, frame_dark)
    _add_member(rear_canopy, (-0.188, 0.008, 0.0), (-0.160, 0.086, 0.076), 0.005, frame_dark)
    _add_member(rear_canopy, (-0.160, 0.086, 0.076), (-0.150, 0.138, 0.044), 0.005, frame_dark)
    rear_canopy.visual(
        Cylinder(radius=0.005, length=0.320),
        origin=Origin(xyz=(0.0, 0.086, 0.076), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="middle_bow",
    )
    rear_canopy.visual(
        Cylinder(radius=0.005, length=0.300),
        origin=Origin(xyz=(0.0, 0.138, 0.044), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="front_bow",
    )
    rear_canopy.visual(
        Box((0.284, 0.128, 0.004)),
        origin=Origin(xyz=(0.0, 0.082, 0.066), rpy=(-0.26, 0.0, 0.0)),
        material=canopy_fabric,
        name="panel",
    )
    rear_canopy.visual(
        Box((0.032, 0.012, 0.036)),
        origin=Origin(xyz=(0.150, 0.138, 0.044)),
        material=frame_dark,
        name="left_front_hinge_support",
    )
    rear_canopy.visual(
        Box((0.032, 0.012, 0.036)),
        origin=Origin(xyz=(-0.150, 0.138, 0.044)),
        material=frame_dark,
        name="right_front_hinge_support",
    )

    front_canopy = model.part("front_canopy_segment")
    front_canopy.inertial = Inertial.from_geometry(
        Box((0.31, 0.15, 0.08)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.07, 0.02)),
    )
    front_canopy.visual(
        Box((0.012, 0.012, 0.034)),
        origin=Origin(xyz=(0.172, 0.006, 0.0)),
        material=frame_dark,
        name="left_hinge_pad",
    )
    front_canopy.visual(
        Box((0.012, 0.012, 0.034)),
        origin=Origin(xyz=(-0.172, 0.006, 0.0)),
        material=frame_dark,
        name="right_hinge_pad",
    )
    _add_member(front_canopy, (0.172, 0.006, 0.0), (0.142, 0.056, 0.028), 0.0045, frame_dark)
    _add_member(front_canopy, (0.142, 0.056, 0.028), (0.128, 0.108, 0.014), 0.0045, frame_dark)
    _add_member(front_canopy, (-0.172, 0.006, 0.0), (-0.142, 0.056, 0.028), 0.0045, frame_dark)
    _add_member(front_canopy, (-0.142, 0.056, 0.028), (-0.128, 0.108, 0.014), 0.0045, frame_dark)
    front_canopy.visual(
        Cylinder(radius=0.0045, length=0.284),
        origin=Origin(xyz=(0.0, 0.056, 0.028), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="middle_bow",
    )
    front_canopy.visual(
        Cylinder(radius=0.0045, length=0.256),
        origin=Origin(xyz=(0.0, 0.108, 0.014), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="front_bow",
    )
    front_canopy.visual(
        Box((0.252, 0.090, 0.004)),
        origin=Origin(xyz=(0.0, 0.060, 0.020), rpy=(-0.32, 0.0, 0.0)),
        material=canopy_fabric,
        name="panel",
    )

    model.articulation(
        "left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_front_caster,
        origin=Origin(xyz=(0.152, 0.32, 0.148)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )
    model.articulation(
        "right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_front_caster,
        origin=Origin(xyz=(-0.152, 0.32, 0.148)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )

    model.articulation(
        "left_front_inner_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_front_caster,
        child=left_front_inner,
        origin=Origin(xyz=(-0.014, -0.006, -0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )
    model.articulation(
        "left_front_outer_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_front_caster,
        child=left_front_outer,
        origin=Origin(xyz=(0.014, -0.006, -0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )
    model.articulation(
        "right_front_inner_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_front_caster,
        child=right_front_inner,
        origin=Origin(xyz=(0.014, -0.006, -0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )
    model.articulation(
        "right_front_outer_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=right_front_caster,
        child=right_front_outer,
        origin=Origin(xyz=(-0.014, -0.006, -0.100)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=18.0),
    )

    model.articulation(
        "left_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_rear_wheel,
        origin=Origin(xyz=(0.222, -0.16, 0.11)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=14.0),
    )
    model.articulation(
        "right_rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_rear_wheel,
        origin=Origin(xyz=(-0.222, -0.16, 0.11)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=14.0),
    )

    model.articulation(
        "rear_canopy_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_canopy,
        origin=Origin(xyz=(0.0, -0.04, 0.68)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.10),
    )
    model.articulation(
        "front_canopy_hinge",
        ArticulationType.REVOLUTE,
        parent=rear_canopy,
        child=front_canopy,
        origin=Origin(xyz=(0.0, 0.138, 0.044)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.4, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_caster = object_model.get_part("left_front_caster_yoke")
    right_caster = object_model.get_part("right_front_caster_yoke")
    left_front_inner = object_model.get_part("left_front_inner_wheel")
    left_front_outer = object_model.get_part("left_front_outer_wheel")
    right_front_inner = object_model.get_part("right_front_inner_wheel")
    right_front_outer = object_model.get_part("right_front_outer_wheel")
    left_rear = object_model.get_part("left_rear_wheel")
    right_rear = object_model.get_part("right_rear_wheel")
    rear_canopy = object_model.get_part("rear_canopy_segment")
    front_canopy = object_model.get_part("front_canopy_segment")

    left_caster_swivel = object_model.get_articulation("left_caster_swivel")
    right_caster_swivel = object_model.get_articulation("right_caster_swivel")
    left_front_inner_spin = object_model.get_articulation("left_front_inner_wheel_spin")
    left_front_outer_spin = object_model.get_articulation("left_front_outer_wheel_spin")
    right_front_inner_spin = object_model.get_articulation("right_front_inner_wheel_spin")
    right_front_outer_spin = object_model.get_articulation("right_front_outer_wheel_spin")
    left_rear_spin = object_model.get_articulation("left_rear_wheel_spin")
    right_rear_spin = object_model.get_articulation("right_rear_wheel_spin")
    rear_canopy_hinge = object_model.get_articulation("rear_canopy_hinge")
    front_canopy_hinge = object_model.get_articulation("front_canopy_hinge")

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

    for joint_obj, expected_axis in (
        (left_caster_swivel, (0.0, 0.0, 1.0)),
        (right_caster_swivel, (0.0, 0.0, 1.0)),
        (left_front_inner_spin, (1.0, 0.0, 0.0)),
        (left_front_outer_spin, (1.0, 0.0, 0.0)),
        (right_front_inner_spin, (1.0, 0.0, 0.0)),
        (right_front_outer_spin, (1.0, 0.0, 0.0)),
        (left_rear_spin, (1.0, 0.0, 0.0)),
        (right_rear_spin, (1.0, 0.0, 0.0)),
        (rear_canopy_hinge, (1.0, 0.0, 0.0)),
        (front_canopy_hinge, (1.0, 0.0, 0.0)),
    ):
        axis = tuple(float(v) for v in joint_obj.axis)
        ctx.check(
            f"{joint_obj.name} uses the intended axis",
            axis == expected_axis,
            details=f"axis={axis}, expected={expected_axis}",
        )

    ctx.expect_contact(
        left_caster,
        frame,
        elem_a="stem",
        elem_b="left_caster_socket",
        name="left caster yoke mounts to the frame socket",
    )
    ctx.expect_contact(
        right_caster,
        frame,
        elem_a="stem",
        elem_b="right_caster_socket",
        name="right caster yoke mounts to the frame socket",
    )
    ctx.expect_contact(
        left_front_inner,
        left_caster,
        elem_b="fork_plate",
        name="left inner front wheel is supported by the left caster fork",
    )
    ctx.expect_contact(
        left_front_outer,
        left_caster,
        elem_b="fork_plate",
        name="left outer front wheel is supported by the left caster fork",
    )
    ctx.expect_contact(
        right_front_inner,
        right_caster,
        elem_b="fork_plate",
        name="right inner front wheel is supported by the right caster fork",
    )
    ctx.expect_contact(
        right_front_outer,
        right_caster,
        elem_b="fork_plate",
        name="right outer front wheel is supported by the right caster fork",
    )
    ctx.expect_contact(
        left_rear,
        frame,
        elem_b="left_rear_stub",
        name="left rear wheel is carried on its axle stub",
    )
    ctx.expect_contact(
        right_rear,
        frame,
        elem_b="right_rear_stub",
        name="right rear wheel is carried on its axle stub",
    )

    ctx.expect_contact(
        rear_canopy,
        frame,
        elem_a="left_hinge_pad",
        elem_b="left_canopy_support",
        name="rear canopy segment has a left side hinge support",
    )
    ctx.expect_contact(
        rear_canopy,
        frame,
        elem_a="right_hinge_pad",
        elem_b="right_canopy_support",
        name="rear canopy segment has a right side hinge support",
    )
    ctx.expect_contact(
        front_canopy,
        rear_canopy,
        elem_a="left_hinge_pad",
        elem_b="left_front_hinge_support",
        name="front canopy segment has a left side hinge support",
    )
    ctx.expect_contact(
        front_canopy,
        rear_canopy,
        elem_a="right_hinge_pad",
        elem_b="right_front_hinge_support",
        name="front canopy segment has a right side hinge support",
    )

    ctx.expect_origin_distance(
        left_front_inner,
        left_front_outer,
        axes="x",
        min_dist=0.025,
        max_dist=0.031,
        name="left twin caster wheels sit side-by-side",
    )
    ctx.expect_origin_distance(
        right_front_inner,
        right_front_outer,
        axes="x",
        min_dist=0.025,
        max_dist=0.031,
        name="right twin caster wheels sit side-by-side",
    )

    rest_inner = ctx.part_world_position(left_front_inner)
    rest_outer = ctx.part_world_position(left_front_outer)
    with ctx.pose({left_caster_swivel: pi / 2.0}):
        turned_inner = ctx.part_world_position(left_front_inner)
        turned_outer = ctx.part_world_position(left_front_outer)
    ctx.check(
        "left caster yoke swivels about a vertical pivot",
        rest_inner is not None
        and rest_outer is not None
        and turned_inner is not None
        and turned_outer is not None
        and abs(rest_inner[0] - rest_outer[0]) > 0.025
        and abs(turned_inner[0] - turned_outer[0]) < 0.004
        and abs(turned_inner[1] - turned_outer[1]) > 0.025,
        details=f"rest_inner={rest_inner}, rest_outer={rest_outer}, turned_inner={turned_inner}, turned_outer={turned_outer}",
    )

    rear_bow_rest = ctx.part_element_world_aabb(rear_canopy, elem="front_bow")
    with ctx.pose({rear_canopy_hinge: 0.80}):
        rear_bow_folded = ctx.part_element_world_aabb(rear_canopy, elem="front_bow")
    ctx.check(
        "rear canopy segment folds upward and back",
        rear_bow_rest is not None
        and rear_bow_folded is not None
        and rear_bow_folded[1][2] > rear_bow_rest[1][2] + 0.07
        and rear_bow_folded[1][1] < rear_bow_rest[1][1] - 0.05,
        details=f"rest={rear_bow_rest}, folded={rear_bow_folded}",
    )

    front_bow_rest = ctx.part_element_world_aabb(front_canopy, elem="front_bow")
    with ctx.pose({front_canopy_hinge: 0.90}):
        front_bow_folded = ctx.part_element_world_aabb(front_canopy, elem="front_bow")
    ctx.check(
        "front canopy segment folds upward relative to the rear segment",
        front_bow_rest is not None
        and front_bow_folded is not None
        and front_bow_folded[1][2] > front_bow_rest[1][2] + 0.05
        and front_bow_folded[1][1] < front_bow_rest[1][1] - 0.03,
        details=f"rest={front_bow_rest}, folded={front_bow_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
