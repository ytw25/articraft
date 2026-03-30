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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _section(
    width: float,
    depth: float,
    z: float,
    *,
    y_offset: float = 0.0,
    x_offset: float = 0.0,
    radius: float | None = None,
):
    corner = radius if radius is not None else min(width, depth) * 0.22
    corner = min(corner, width * 0.5, depth * 0.5)
    profile = rounded_rect_profile(width, depth, corner)
    return [(x + x_offset, y + y_offset, z) for x, y in profile]


def _seat_mesh():
    return section_loft(
        [
            _section(0.50, 0.47, 0.000, y_offset=0.000, radius=0.070),
            _section(0.50, 0.48, 0.020, y_offset=0.008, radius=0.068),
            _section(0.48, 0.45, 0.048, y_offset=-0.006, radius=0.062),
            _section(0.46, 0.40, 0.070, y_offset=-0.018, radius=0.055),
        ]
    )


def _backrest_mesh():
    return section_loft(
        [
            _section(0.40, 0.065, 0.020, y_offset=0.014, radius=0.030),
            _section(0.48, 0.075, 0.210, y_offset=0.002, radius=0.038),
            _section(0.46, 0.070, 0.430, y_offset=-0.032, radius=0.036),
            _section(0.35, 0.055, 0.640, y_offset=-0.075, radius=0.028),
        ]
    )


def _arm_pad_mesh():
    return ExtrudeGeometry(rounded_rect_profile(0.255, 0.095, 0.028), 0.030, center=True)


def _spring_mesh(
    *,
    coil_radius: float,
    wire_radius: float,
    length: float,
    turns: float,
    samples: int,
):
    points = []
    for index in range(samples + 1):
        t = index / samples
        angle = 2.0 * math.pi * turns * t
        y = -length * 0.5 + length * t
        points.append((coil_radius * math.cos(angle), y, coil_radius * math.sin(angle)))
    return tube_from_spline_points(
        points,
        radius=wire_radius,
        samples_per_segment=4,
        radial_segments=12,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ergonomic_office_chair")

    polished_aluminum = model.material("polished_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.19, 0.20, 0.22, 1.0))
    charcoal = model.material("charcoal", rgba=(0.13, 0.14, 0.15, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.09, 0.10, 1.0))
    cushion_gray = model.material("cushion_gray", rgba=(0.26, 0.29, 0.31, 1.0))
    mesh_black = model.material("mesh_black", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.63, 0.65, 0.68, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    accent = model.material("accent", rgba=(0.46, 0.48, 0.50, 1.0))

    base = model.part("base")
    base.inertial = Inertial.from_geometry(
        Box((0.74, 0.74, 0.12)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )
    base.visual(
        Cylinder(radius=0.078, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=polished_aluminum,
        name="hub",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=polished_aluminum,
        name="hub_taper_socket",
    )

    arm_profile = rounded_rect_profile(0.058, 0.016, 0.007)
    arm_geom = sweep_profile_along_spline(
        [
            (0.055, 0.000, 0.086),
            (0.180, 0.000, 0.082),
            (0.320, 0.000, 0.090),
        ],
        profile=arm_profile,
        samples_per_segment=18,
        cap_profile=True,
    )
    arm_mesh = _mesh("chair_base_arm", arm_geom)

    caster_angles = [math.pi / 2.0 + index * (2.0 * math.pi / 5.0) for index in range(5)]
    caster_radius = 0.320
    for index, angle in enumerate(caster_angles):
        socket_x = caster_radius * math.cos(angle)
        socket_y = caster_radius * math.sin(angle)
        base.visual(
            arm_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=polished_aluminum,
            name=f"base_arm_{index}",
        )
        base.visual(
            Cylinder(radius=0.015, length=0.012),
            origin=Origin(xyz=(socket_x, socket_y, 0.096)),
            material=satin_silver,
            name=f"caster_socket_{index}",
        )

    gas_lift_outer = model.part("gas_lift_outer")
    outer_shell = LatheGeometry.from_shell_profiles(
        [(0.032, 0.000), (0.029, 0.060), (0.026, 0.230)],
        [(0.024, 0.004), (0.021, 0.060), (0.018, 0.226)],
        segments=48,
    )
    gas_lift_outer.visual(
        _mesh("gas_lift_outer_shell", outer_shell),
        material=dark_gray,
        name="outer_shell",
    )
    gas_lift_outer.visual(
        Cylinder(radius=0.033, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_gray,
        name="outer_base_ring",
    )
    gas_lift_outer.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=0.242),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
    )
    model.articulation(
        "base_to_gas_lift_outer",
        ArticulationType.FIXED,
        parent=base,
        child=gas_lift_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    gas_lift_piston = model.part("gas_lift_piston")
    gas_lift_piston.visual(
        Cylinder(radius=0.018, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=satin_silver,
        name="piston_rod",
    )
    gas_lift_piston.visual(
        Cylinder(radius=0.024, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.291)),
        material=dark_gray,
        name="top_adapter",
    )
    gas_lift_piston.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.309),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.1545)),
    )
    model.articulation(
        "gas_lift_travel",
        ArticulationType.PRISMATIC,
        parent=gas_lift_outer,
        child=gas_lift_piston,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.18,
            lower=0.0,
            upper=0.120,
        ),
    )

    control_chassis = model.part("control_chassis")
    control_chassis.visual(
        Box((0.540, 0.090, 0.016)),
        origin=Origin(xyz=(0.0, 0.000, 0.008)),
        material=charcoal,
        name="underseat_cross_plate",
    )
    control_chassis.visual(
        Box((0.180, 0.240, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=black_plastic,
        name="center_housing",
    )
    control_chassis.visual(
        Box((0.220, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.150, 0.024)),
        material=charcoal,
        name="front_tension_bracket",
    )
    control_chassis.visual(
        Box((0.180, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, -0.125, 0.020)),
        material=charcoal,
        name="backrest_hinge_bridge",
    )
    control_chassis.visual(
        Box((0.032, 0.050, 0.100)),
        origin=Origin(xyz=(0.074, -0.145, 0.050)),
        material=charcoal,
        name="right_hinge_cheek",
    )
    control_chassis.visual(
        Box((0.032, 0.050, 0.100)),
        origin=Origin(xyz=(-0.074, -0.145, 0.050)),
        material=charcoal,
        name="left_hinge_cheek",
    )
    control_chassis.visual(
        Box((0.020, 0.040, 0.034)),
        origin=Origin(xyz=(0.120, 0.0, 0.033)),
        material=charcoal,
        name="right_track_bridge",
    )
    control_chassis.visual(
        Box((0.020, 0.040, 0.034)),
        origin=Origin(xyz=(-0.120, 0.0, 0.033)),
        material=charcoal,
        name="left_track_bridge",
    )
    control_chassis.visual(
        Box((0.100, 0.055, 0.006)),
        origin=Origin(xyz=(0.220, 0.045, 0.019)),
        material=charcoal,
        name="left_width_track",
    )
    control_chassis.visual(
        Box((0.100, 0.055, 0.006)),
        origin=Origin(xyz=(-0.220, 0.045, 0.019)),
        material=charcoal,
        name="right_width_track",
    )

    spring_mesh = _mesh(
        "chair_tension_spring",
        _spring_mesh(
            coil_radius=0.018,
            wire_radius=0.0024,
            length=0.080,
            turns=6.5,
            samples=72,
        ),
    )
    control_chassis.visual(
        spring_mesh,
        origin=Origin(xyz=(0.0, 0.130, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="tension_spring",
    )
    control_chassis.visual(
        Cylinder(radius=0.007, length=0.120),
        origin=Origin(xyz=(0.0, 0.130, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=accent,
        name="tension_bolt",
    )
    control_chassis.visual(
        Cylinder(radius=0.023, length=0.016),
        origin=Origin(xyz=(0.0, 0.188, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="tension_knob",
    )
    control_chassis.inertial = Inertial.from_geometry(
        Box((0.320, 0.280, 0.100)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )
    model.articulation(
        "piston_to_control_chassis",
        ArticulationType.FIXED,
        parent=gas_lift_piston,
        child=control_chassis,
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
    )

    seat_pan = model.part("seat_pan")
    seat_pan.visual(
        _mesh("chair_seat_shell", _seat_mesh()),
        origin=Origin(xyz=(0.0, 0.035, 0.0)),
        material=cushion_gray,
        name="seat_cushion",
    )
    seat_pan.visual(
        Box((0.410, 0.280, 0.024)),
        origin=Origin(xyz=(0.0, 0.025, 0.012)),
        material=black_plastic,
        name="seat_undertray",
    )
    seat_pan.visual(
        Cylinder(radius=0.026, length=0.360),
        origin=Origin(xyz=(0.0, 0.205, 0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cushion_gray,
        name="front_roll",
    )
    seat_pan.inertial = Inertial.from_geometry(
        Box((0.500, 0.480, 0.080)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )
    model.articulation(
        "control_chassis_to_seat_pan",
        ArticulationType.FIXED,
        parent=control_chassis,
        child=seat_pan,
        origin=Origin(xyz=(0.0, 0.100, 0.050)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        _mesh("chair_backrest_shell", _backrest_mesh()),
        origin=Origin(xyz=(0.0, -0.125, 0.080)),
        material=mesh_black,
        name="backrest_shell",
    )
    backrest.visual(
        Box((0.085, 0.040, 0.500)),
        origin=Origin(xyz=(0.0, -0.100, 0.330)),
        material=black_plastic,
        name="back_spine",
    )
    backrest.visual(
        Box((0.024, 0.090, 0.170)),
        origin=Origin(xyz=(0.030, -0.055, 0.079)),
        material=charcoal,
        name="right_support_arm",
    )
    backrest.visual(
        Box((0.024, 0.090, 0.170)),
        origin=Origin(xyz=(-0.030, -0.055, 0.079)),
        material=charcoal,
        name="left_support_arm",
    )
    backrest.visual(
        Box((0.070, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=charcoal,
        name="lower_mount_block",
    )
    backrest.visual(
        Cylinder(radius=0.008, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_silver,
        name="hinge_pin_barrel",
    )
    backrest.visual(
        Box((0.180, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, -0.072, 0.150)),
        material=black_plastic,
        name="lumbar_lower_stop",
    )
    backrest.visual(
        Box((0.180, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, -0.072, 0.430)),
        material=black_plastic,
        name="lumbar_upper_stop",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.500, 0.090, 0.660)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
    )
    model.articulation(
        "backrest_recline",
        ArticulationType.REVOLUTE,
        parent=control_chassis,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.145, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(45.0),
        ),
    )

    lumbar_support = model.part("lumbar_support")
    lumbar_support.visual(
        Box((0.250, 0.026, 0.080)),
        origin=Origin(xyz=(0.0, -0.051, 0.060)),
        material=cushion_gray,
        name="lumbar_pad",
    )
    lumbar_support.visual(
        Box((0.050, 0.014, 0.120)),
        origin=Origin(xyz=(0.0, -0.070, 0.060)),
        material=black_plastic,
        name="lumbar_slider",
    )
    lumbar_support.inertial = Inertial.from_geometry(
        Box((0.260, 0.060, 0.120)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )
    model.articulation(
        "lumbar_height",
        ArticulationType.PRISMATIC,
        parent=backrest,
        child=lumbar_support,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.090,
        ),
    )

    arm_pad_mesh = _mesh("chair_arm_pad", _arm_pad_mesh())

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        width_carriage = model.part(f"{side_name}_arm_width_carriage")
        width_carriage.visual(
            Box((0.070, 0.050, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=charcoal,
            name="width_slider_shoe",
        )
        width_carriage.visual(
            Box((0.070, 0.050, 0.022)),
            origin=Origin(xyz=(0.0, 0.0, 0.019)),
            material=black_plastic,
            name="width_support_block",
        )
        width_carriage.visual(
            Box((0.010, 0.055, 0.016)),
            origin=Origin(xyz=(0.030 * side_sign, 0.0, 0.010)),
            material=accent,
            name="width_stop_plate",
        )
        width_carriage.inertial = Inertial.from_geometry(
            Box((0.080, 0.055, 0.030)),
            mass=0.35,
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
        )
        model.articulation(
            f"{side_name}_arm_width_slide",
            ArticulationType.PRISMATIC,
            parent=control_chassis,
            child=width_carriage,
            origin=Origin(xyz=(0.300 * side_sign, 0.045, 0.016)),
            axis=(side_sign, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=0.10,
                lower=-0.030,
                upper=0.030,
            ),
        )

        depth_carriage = model.part(f"{side_name}_arm_depth_carriage")
        depth_carriage.visual(
            Box((0.055, 0.070, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=charcoal,
            name="depth_slider_shoe",
        )
        depth_carriage.visual(
            Box((0.055, 0.070, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.017)),
            material=black_plastic,
            name="depth_support_block",
        )
        depth_carriage.visual(
            Box((0.060, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, 0.030, 0.009)),
            material=accent,
            name="depth_stop_plate",
        )
        depth_carriage.inertial = Inertial.from_geometry(
            Box((0.060, 0.075, 0.026)),
            mass=0.28,
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
        )
        model.articulation(
            f"{side_name}_arm_depth_slide",
            ArticulationType.PRISMATIC,
            parent=width_carriage,
            child=depth_carriage,
            origin=Origin(xyz=(0.0, 0.0, 0.030)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=0.10,
                lower=-0.030,
                upper=0.030,
            ),
        )

        arm_post = model.part(f"{side_name}_arm_post")
        arm_post.visual(
            Box((0.042, 0.050, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=charcoal,
            name="post_base_plate",
        )
        arm_post.visual(
            Box((0.036, 0.045, 0.150)),
            origin=Origin(xyz=(0.0, 0.0, 0.080)),
            material=black_plastic,
            name="post_column",
        )
        arm_post.inertial = Inertial.from_geometry(
            Box((0.048, 0.055, 0.160)),
            mass=0.55,
            origin=Origin(xyz=(0.0, 0.0, 0.080)),
        )
        model.articulation(
            f"{side_name}_arm_height",
            ArticulationType.PRISMATIC,
            parent=depth_carriage,
            child=arm_post,
            origin=Origin(xyz=(0.0, 0.0, 0.026)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=45.0,
                velocity=0.12,
                lower=0.0,
                upper=0.080,
            ),
        )

        arm_pad = model.part(f"{side_name}_arm_pad")
        arm_pad.visual(
            arm_pad_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
            material=cushion_gray,
            name="pad_shell",
        )
        arm_pad.visual(
            Box((0.065, 0.060, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=black_plastic,
            name="pad_mount_plate",
        )
        arm_pad.inertial = Inertial.from_geometry(
            Box((0.255, 0.095, 0.030)),
            mass=0.40,
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
        )
        model.articulation(
            f"{side_name}_arm_pad_mount",
            ArticulationType.FIXED,
            parent=arm_post,
            child=arm_pad,
            origin=Origin(xyz=(0.0, 0.0, 0.155)),
        )

    def add_caster(caster_index: int, angle: float) -> None:
        socket_x = caster_radius * math.cos(angle)
        socket_y = caster_radius * math.sin(angle)

        caster_body = model.part(f"caster_{caster_index}_body")
        caster_body.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=satin_silver,
            name="stem_collar",
        )
        caster_body.visual(
            Cylinder(radius=0.007, length=0.046),
            origin=Origin(xyz=(0.0, 0.0, -0.027)),
            material=satin_silver,
            name="stem",
        )
        caster_body.visual(
            Box((0.026, 0.036, 0.008)),
            origin=Origin(xyz=(0.018, 0.0, -0.018)),
            material=black_plastic,
            name="fork_bridge",
        )
        caster_body.visual(
            Box((0.028, 0.003, 0.048)),
            origin=Origin(xyz=(0.018, 0.0185, -0.044)),
            material=black_plastic,
            name="fork_left_wall",
        )
        caster_body.visual(
            Box((0.028, 0.003, 0.048)),
            origin=Origin(xyz=(0.018, -0.0185, -0.044)),
            material=black_plastic,
            name="fork_right_wall",
        )
        caster_body.inertial = Inertial.from_geometry(
            Box((0.040, 0.040, 0.085)),
            mass=0.18,
            origin=Origin(xyz=(0.010, 0.0, -0.045)),
        )
        model.articulation(
            f"base_to_caster_{caster_index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster_body,
            origin=Origin(xyz=(socket_x, socket_y, 0.092), rpy=(0.0, 0.0, angle)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=5.0, velocity=8.0),
        )

        wheelset = model.part(f"caster_{caster_index}_wheelset")
        wheelset.visual(
            Cylinder(radius=0.030, length=0.016),
            origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="outer_wheel",
        )
        wheelset.visual(
            Cylinder(radius=0.030, length=0.016),
            origin=Origin(xyz=(0.0, -0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name="inner_wheel",
        )
        wheelset.visual(
            Cylinder(radius=0.009, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="axle_spacer",
        )
        wheelset.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(0.0, 0.0155, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=accent,
            name="left_bushing",
        )
        wheelset.visual(
            Cylinder(radius=0.004, length=0.003),
            origin=Origin(xyz=(0.0, -0.0155, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=accent,
            name="right_bushing",
        )
        wheelset.inertial = Inertial.from_geometry(
            Cylinder(radius=0.030, length=0.038),
            mass=0.24,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"caster_{caster_index}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster_body,
            child=wheelset,
            origin=Origin(xyz=(0.018, 0.0, -0.062)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    for index, angle in enumerate(caster_angles):
        add_caster(index, angle)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    gas_lift_outer = object_model.get_part("gas_lift_outer")
    gas_lift_piston = object_model.get_part("gas_lift_piston")
    control_chassis = object_model.get_part("control_chassis")
    seat_pan = object_model.get_part("seat_pan")
    backrest = object_model.get_part("backrest")
    lumbar_support = object_model.get_part("lumbar_support")
    left_width = object_model.get_part("left_arm_width_carriage")
    left_depth = object_model.get_part("left_arm_depth_carriage")
    left_post = object_model.get_part("left_arm_post")
    left_pad = object_model.get_part("left_arm_pad")
    right_width = object_model.get_part("right_arm_width_carriage")
    right_depth = object_model.get_part("right_arm_depth_carriage")
    right_post = object_model.get_part("right_arm_post")
    right_pad = object_model.get_part("right_arm_pad")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        base,
        gas_lift_outer,
        reason="The gas-lift outer taper is press-fit into the base hub socket.",
    )
    ctx.allow_overlap(
        gas_lift_outer,
        gas_lift_piston,
        reason="The telescoping piston travels inside the gas-lift outer cylinder.",
    )
    ctx.allow_overlap(
        control_chassis,
        seat_pan,
        reason="The synchronous mechanism nests into the molded underside cavity of the seat pan.",
    )
    ctx.allow_overlap(
        backrest,
        lumbar_support,
        reason="The lumbar support slider runs inside the backrest's captured guide track.",
    )
    ctx.allow_overlap(
        control_chassis,
        left_width,
        reason="The left armrest width slider shoe is intentionally captured inside the chassis track.",
    )
    ctx.allow_overlap(
        control_chassis,
        right_width,
        reason="The right armrest width slider shoe is intentionally captured inside the chassis track.",
    )
    ctx.allow_overlap(
        control_chassis,
        backrest,
        elem_a="left_hinge_cheek",
        elem_b="left_support_arm",
        reason="The left hinge cheek closely envelopes the recline pivot arm.",
    )
    ctx.allow_overlap(
        control_chassis,
        backrest,
        elem_a="right_hinge_cheek",
        elem_b="right_support_arm",
        reason="The right hinge cheek closely envelopes the recline pivot arm.",
    )
    for caster_index in range(5):
        caster_body = object_model.get_part(f"caster_{caster_index}_body")
        wheelset = object_model.get_part(f"caster_{caster_index}_wheelset")
        ctx.allow_overlap(
            base,
            caster_body,
            reason="Each caster stem is retained inside a base socket with a shallow inserted collar.",
        )
        ctx.allow_overlap(
            caster_body,
            wheelset,
            reason="The twin-wheel set sits inside the caster fork around the axle line.",
        )

    ctx.fail_if_parts_overlap_in_current_pose()

    lift = object_model.get_articulation("gas_lift_travel")
    recline = object_model.get_articulation("backrest_recline")
    lumbar = object_model.get_articulation("lumbar_height")
    left_width_slide = object_model.get_articulation("left_arm_width_slide")
    left_depth_slide = object_model.get_articulation("left_arm_depth_slide")
    left_height = object_model.get_articulation("left_arm_height")
    right_width_slide = object_model.get_articulation("right_arm_width_slide")
    right_depth_slide = object_model.get_articulation("right_arm_depth_slide")
    right_height = object_model.get_articulation("right_arm_height")

    ctx.expect_contact(gas_lift_outer, base, name="gas_lift_outer_seated_in_base")
    ctx.expect_contact(gas_lift_piston, control_chassis, name="piston_supports_control_chassis")
    ctx.expect_contact(control_chassis, seat_pan, name="seat_pan_supported_by_control")
    ctx.expect_overlap(
        backrest,
        control_chassis,
        axes="x",
        min_overlap=0.05,
        elem_a="lower_mount_block",
        elem_b="backrest_hinge_bridge",
        name="backrest_hinge_footprint_alignment",
    )
    ctx.expect_contact(
        backrest,
        control_chassis,
        elem_a="lower_mount_block",
        elem_b="backrest_hinge_bridge",
        name="backrest_hinges_on_control",
    )
    ctx.expect_contact(backrest, lumbar_support, name="lumbar_contacts_backrest_track")

    for side_parent, side_child, check_name in [
        (control_chassis, left_width, "left_width_carriage_contacts_track"),
        (left_width, left_depth, "left_depth_carriage_contacts_width_stage"),
        (left_post, left_pad, "left_pad_contacts_post"),
        (control_chassis, right_width, "right_width_carriage_contacts_track"),
        (right_width, right_depth, "right_depth_carriage_contacts_width_stage"),
        (right_post, right_pad, "right_pad_contacts_post"),
    ]:
        ctx.expect_contact(side_parent, side_child, name=check_name)

    ctx.expect_overlap(left_depth, left_post, axes="xy", min_overlap=0.03, name="left_post_aligned_with_depth_stage")
    ctx.expect_gap(
        left_post,
        left_depth,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="left_post_contacts_depth_stage",
    )
    ctx.expect_overlap(right_depth, right_post, axes="xy", min_overlap=0.03, name="right_post_aligned_with_depth_stage")
    ctx.expect_gap(
        right_post,
        right_depth,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        name="right_post_contacts_depth_stage",
    )

    for caster_index in range(5):
        caster_body = object_model.get_part(f"caster_{caster_index}_body")
        wheelset = object_model.get_part(f"caster_{caster_index}_wheelset")
        ctx.expect_contact(base, caster_body, name=f"caster_{caster_index}_stem_contacts_socket")
        ctx.expect_contact(caster_body, wheelset, name=f"caster_{caster_index}_wheelset_contacts_fork")

    ctx.check("gas_lift_axis_is_vertical", lift.axis == (0.0, 0.0, 1.0), f"Unexpected gas lift axis: {lift.axis}")
    ctx.check("backrest_axis_is_lateral", recline.axis == (1.0, 0.0, 0.0), f"Unexpected backrest axis: {recline.axis}")
    ctx.check("lumbar_axis_is_vertical", lumbar.axis == (0.0, 0.0, 1.0), f"Unexpected lumbar axis: {lumbar.axis}")
    ctx.check(
        "left_arm_axes",
        left_width_slide.axis == (1.0, 0.0, 0.0)
        and left_depth_slide.axis == (0.0, 1.0, 0.0)
        and left_height.axis == (0.0, 0.0, 1.0),
        "Left armrest axes must be width/depth/height = x/y/z.",
    )
    ctx.check(
        "right_arm_axes",
        right_width_slide.axis == (-1.0, 0.0, 0.0)
        and right_depth_slide.axis == (0.0, 1.0, 0.0)
        and right_height.axis == (0.0, 0.0, 1.0),
        "Right armrest axes must be mirrored in width and z-aligned in height.",
    )

    rest_control = ctx.part_world_position(control_chassis)
    assert rest_control is not None
    with ctx.pose({lift: 0.120}):
        high_control = ctx.part_world_position(control_chassis)
        assert high_control is not None
        ctx.check(
            "gas_lift_travel_matches_120mm",
            abs((high_control[2] - rest_control[2]) - 0.120) <= 0.003,
            f"Expected 0.120 m travel, got {high_control[2] - rest_control[2]:.4f} m.",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_height")

    rest_backrest_aabb = ctx.part_world_aabb(backrest)
    assert rest_backrest_aabb is not None
    with ctx.pose({recline: math.radians(45.0)}):
        reclined_backrest_aabb = ctx.part_world_aabb(backrest)
        assert reclined_backrest_aabb is not None
        rear_shift = reclined_backrest_aabb[0][1] - rest_backrest_aabb[0][1]
        height_drop = rest_backrest_aabb[1][2] - reclined_backrest_aabb[1][2]
        ctx.check(
            "backrest_reclines_rearward",
            rear_shift <= -0.140 and height_drop >= 0.180,
            (
                "Backrest should move rearward and lower at full recline; "
                f"rear shift={rear_shift:.4f}, height drop={height_drop:.4f}."
            ),
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_fully_reclined")

    rest_left_pad = ctx.part_world_position(left_pad)
    assert rest_left_pad is not None
    with ctx.pose(
        {
            left_width_slide: 0.030,
            left_depth_slide: 0.030,
            left_height: 0.080,
        }
    ):
        moved_left_pad = ctx.part_world_position(left_pad)
        assert moved_left_pad is not None
        dx = moved_left_pad[0] - rest_left_pad[0]
        dy = moved_left_pad[1] - rest_left_pad[1]
        dz = moved_left_pad[2] - rest_left_pad[2]
        ctx.check(
            "left_armrest_4d_travel",
            dx >= 0.028 and dy >= 0.028 and dz >= 0.078,
            f"Unexpected left armrest travel dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}.",
        )

    rest_right_pad = ctx.part_world_position(right_pad)
    assert rest_right_pad is not None
    with ctx.pose(
        {
            right_width_slide: 0.030,
            right_depth_slide: -0.030,
            right_height: 0.080,
        }
    ):
        moved_right_pad = ctx.part_world_position(right_pad)
        assert moved_right_pad is not None
        dx = moved_right_pad[0] - rest_right_pad[0]
        dy = moved_right_pad[1] - rest_right_pad[1]
        dz = moved_right_pad[2] - rest_right_pad[2]
        ctx.check(
            "right_armrest_mirrored_travel",
            dx <= -0.028 and dy <= -0.028 and dz >= 0.078,
            f"Unexpected right armrest travel dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}.",
        )

    rest_lumbar = ctx.part_world_position(lumbar_support)
    assert rest_lumbar is not None
    with ctx.pose({lumbar: 0.090}):
        high_lumbar = ctx.part_world_position(lumbar_support)
        assert high_lumbar is not None
        ctx.check(
            "lumbar_support_vertical_travel",
            abs((high_lumbar[2] - rest_lumbar[2]) - 0.090) <= 0.003,
            f"Expected 0.090 m lumbar travel, got {high_lumbar[2] - rest_lumbar[2]:.4f} m.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
