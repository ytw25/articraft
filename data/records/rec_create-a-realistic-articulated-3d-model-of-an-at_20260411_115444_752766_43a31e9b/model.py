from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    superellipse_side_loft,
    tube_from_spline_points,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _transform_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    angle: float = 0.0,
) -> list[tuple[float, float]]:
    c = cos(angle)
    s = sin(angle)
    return [(c * x - s * y + dx, s * x + c * y + dy) for x, y in profile]


def _add_wheel_visuals(
    part,
    mesh_prefix: str,
    *,
    tire_radius: float,
    tire_width: float,
    hub_width: float,
    wheel_steel,
    dark_steel,
    rubber,
    side_sign: float = 0.0,
    add_knuckle: bool = False,
) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    half_width = tire_width * 0.5

    tire_profile = [
        (tire_radius * 0.50, -half_width * 0.96),
        (tire_radius * 0.72, -half_width * 0.98),
        (tire_radius * 0.88, -half_width * 0.84),
        (tire_radius * 0.97, -half_width * 0.52),
        (tire_radius, -half_width * 0.16),
        (tire_radius, half_width * 0.16),
        (tire_radius * 0.97, half_width * 0.52),
        (tire_radius * 0.88, half_width * 0.84),
        (tire_radius * 0.72, half_width * 0.98),
        (tire_radius * 0.50, half_width * 0.96),
        (tire_radius * 0.42, half_width * 0.34),
        (tire_radius * 0.39, 0.0),
        (tire_radius * 0.42, -half_width * 0.34),
        (tire_radius * 0.50, -half_width * 0.96),
    ]
    tire_mesh = _save_mesh(
        f"{mesh_prefix}_tire",
        LatheGeometry(tire_profile, segments=64).rotate_y(pi / 2.0),
    )
    part.visual(tire_mesh, material=rubber, name="tire")

    rim_outer = superellipse_profile(
        tire_radius * 1.08,
        tire_radius * 1.08,
        exponent=2.0,
        segments=48,
    )
    window_profile = rounded_rect_profile(
        tire_radius * 0.16,
        tire_radius * 0.28,
        tire_radius * 0.040,
        corner_segments=6,
    )
    hole_profiles = [
        superellipse_profile(
            tire_radius * 0.30,
            tire_radius * 0.30,
            exponent=2.0,
            segments=24,
        )
    ]
    for spoke_index in range(5):
        angle = 2.0 * pi * spoke_index / 5.0
        hole_profiles.append(
            _transform_profile(
                window_profile,
                dx=cos(angle) * tire_radius * 0.29,
                dy=sin(angle) * tire_radius * 0.29,
                angle=angle,
            )
        )

    rim_face = _save_mesh(
        f"{mesh_prefix}_rim_face",
        ExtrudeWithHolesGeometry(
            rim_outer,
            hole_profiles,
            height=tire_width * 0.16,
            center=True,
        ).rotate_y(pi / 2.0),
    )
    part.visual(
        rim_face,
        origin=Origin(xyz=(tire_width * 0.20, 0.0, 0.0)),
        material=wheel_steel,
        name="rim_face_outer",
    )
    part.visual(
        rim_face,
        origin=Origin(xyz=(-tire_width * 0.20, 0.0, 0.0)),
        material=wheel_steel,
        name="rim_face_inner",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.50, length=tire_width * 0.44),
        origin=spin_origin,
        material=dark_steel,
        name="wheel_drum",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.28, length=hub_width),
        origin=spin_origin,
        material=dark_steel,
        name="hub_barrel",
    )
    part.visual(
        Cylinder(radius=tire_radius * 0.11, length=hub_width * 0.92),
        origin=spin_origin,
        material=wheel_steel,
        name="hub_cap",
    )

    if add_knuckle:
        arm_x = -0.075 * side_sign
        part.visual(
            Cylinder(radius=0.030, length=0.24),
            origin=Origin(xyz=(0.0, 0.0, 0.05)),
            material=dark_steel,
            name="knuckle_upright",
        )
        part.visual(
            Box((0.14, 0.055, 0.055)),
            origin=Origin(xyz=(arm_x, 0.0, 0.0)),
            material=dark_steel,
            name="knuckle_arm",
        )
        part.visual(
            Box((0.065, 0.13, 0.04)),
            origin=Origin(xyz=(arm_x * 0.70, -0.05, 0.085)),
            material=dark_steel,
            name="steering_tab",
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.42, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.06), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_steel,
            name="brake_disk",
        )
        part.visual(
            Cylinder(radius=tire_radius * 0.38, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, -0.075), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name="brake_hat",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="atv_quad_bike")

    body_red = model.material("body_red", rgba=(0.66, 0.10, 0.09, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.18, 0.19, 0.18, 1.0))
    wheel_steel = model.material("wheel_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    shock_silver = model.material("shock_silver", rgba=(0.78, 0.79, 0.82, 1.0))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.14, 0.14, 0.15, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        _save_mesh(
            "body_shell",
            superellipse_side_loft(
                [
                    (0.58, 0.50, 0.66, 0.22),
                    (0.42, 0.44, 0.58, 0.38),
                    (0.26, 0.43, 0.64, 0.40),
                    (0.08, 0.44, 0.70, 0.34),
                    (-0.10, 0.44, 0.67, 0.25),
                    (-0.28, 0.43, 0.57, 0.22),
                ],
                exponents=2.6,
                segments=56,
            ),
        ),
        material=body_red,
        name="body_shell",
    )
    chassis.visual(Box((0.14, 0.58, 0.08)), origin=Origin(xyz=(0.0, -0.10, 0.44)), material=black_plastic, name="seat_base")
    chassis.visual(Box((0.22, 0.30, 0.06)), origin=Origin(xyz=(0.30, -0.02, 0.40)), material=black_plastic, name="left_floor_panel")
    chassis.visual(Box((0.22, 0.30, 0.06)), origin=Origin(xyz=(-0.30, -0.02, 0.40)), material=black_plastic, name="right_floor_panel")
    chassis.visual(Box((0.44, 0.12, 0.08)), origin=Origin(xyz=(0.0, 0.37, 0.41)), material=frame_paint, name="front_crossmember")
    chassis.visual(Box((0.54, 0.18, 0.10)), origin=Origin(xyz=(0.0, -0.26, 0.42)), material=frame_paint, name="engine_block")
    chassis.visual(Cylinder(radius=0.05, length=0.18), origin=Origin(xyz=(0.0, 0.18, 0.71)), material=dark_steel, name="steering_head")
    chassis.visual(Box((0.22, 0.12, 0.06)), origin=Origin(xyz=(0.0, 0.55, 0.63)), material=frame_paint, name="front_rack_mount")
    chassis.visual(Box((0.26, 0.30, 0.14)), origin=Origin(xyz=(0.0, -0.18, 0.67)), material=black_plastic, name="tank_cover")

    chassis.visual(
        _save_mesh(
            "left_lower_rail",
            tube_from_spline_points(
                [(0.18, 0.57, 0.39), (0.24, 0.25, 0.40), (0.27, -0.02, 0.39), (0.31, -0.32, 0.38), (0.35, -0.52, 0.34)],
                radius=0.030,
                samples_per_segment=12,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="left_lower_rail",
    )
    chassis.visual(
        _save_mesh(
            "right_lower_rail",
            tube_from_spline_points(
                _mirror_x([(0.18, 0.57, 0.39), (0.24, 0.25, 0.40), (0.27, -0.02, 0.39), (0.31, -0.32, 0.38), (0.35, -0.52, 0.34)]),
                radius=0.030,
                samples_per_segment=12,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="right_lower_rail",
    )
    chassis.visual(
        _save_mesh(
            "center_spine",
            tube_from_spline_points(
                [(0.0, 0.50, 0.50), (0.0, 0.22, 0.58), (0.0, -0.04, 0.63), (0.0, -0.28, 0.60)],
                radius=0.045,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        material=frame_paint,
        name="center_spine",
    )

    for name, points in [
        (
            "left_lower_arm",
            [(0.16, 0.43, 0.40), (0.30, 0.48, 0.37), (0.46, 0.56, 0.33), (0.30, 0.62, 0.37), (0.16, 0.58, 0.40)],
        ),
        (
            "right_lower_arm",
            _mirror_x([(0.16, 0.43, 0.40), (0.30, 0.48, 0.37), (0.46, 0.56, 0.33), (0.30, 0.62, 0.37), (0.16, 0.58, 0.40)]),
        ),
        (
            "left_upper_arm",
            [(0.18, 0.46, 0.54), (0.30, 0.50, 0.52), (0.46, 0.56, 0.50), (0.30, 0.61, 0.52), (0.18, 0.57, 0.54)],
        ),
        (
            "right_upper_arm",
            _mirror_x([(0.18, 0.46, 0.54), (0.30, 0.50, 0.52), (0.46, 0.56, 0.50), (0.30, 0.61, 0.52), (0.18, 0.57, 0.54)]),
        ),
    ]:
        chassis.visual(
            _save_mesh(
                name,
                wire_from_points(
                    points,
                    radius=0.016 if "lower" in name else 0.013,
                    radial_segments=14,
                    cap_ends=True,
                    corner_mode="fillet",
                    corner_radius=0.03,
                    corner_segments=8,
                ),
            ),
            material=frame_paint,
            name=name,
        )

    for name, points in [
        ("left_front_shock_body", [(0.27, 0.46, 0.61), (0.43, 0.55, 0.39)]),
        ("right_front_shock_body", _mirror_x([(0.27, 0.46, 0.61), (0.43, 0.55, 0.39)])),
    ]:
        chassis.visual(
            _save_mesh(
                name,
                tube_from_spline_points(points, radius=0.030, samples_per_segment=2, radial_segments=16),
            ),
            material=shock_silver,
            name=name,
        )
        chassis.visual(
            _save_mesh(
                name.replace("_body", "_rod"),
                tube_from_spline_points(
                    [
                        (points[0][0] * 0.96, points[0][1] * 0.98, points[0][2] - 0.01),
                        (points[1][0] * 0.98, points[1][1], points[1][2] + 0.01),
                    ],
                    radius=0.013,
                    samples_per_segment=2,
                    radial_segments=14,
                ),
            ),
            material=dark_steel,
            name=name.replace("_body", "_rod"),
        )

    for name, points in [
        ("left_swingarm", [(0.17, -0.20, 0.44), (0.27, -0.35, 0.39), (0.36, -0.52, 0.34), (0.46, -0.55, 0.32)]),
        ("right_swingarm", _mirror_x([(0.17, -0.20, 0.44), (0.27, -0.35, 0.39), (0.36, -0.52, 0.34), (0.46, -0.55, 0.32)])),
        ("left_rear_shock", [(0.18, -0.12, 0.64), (0.33, -0.46, 0.41)]),
        ("right_rear_shock", _mirror_x([(0.18, -0.12, 0.64), (0.33, -0.46, 0.41)])),
    ]:
        chassis.visual(
            _save_mesh(
                name,
                tube_from_spline_points(
                    points,
                    radius=0.028 if "swingarm" in name else 0.030,
                    samples_per_segment=10,
                    radial_segments=16,
                ),
            ),
            material=frame_paint if "swingarm" in name else shock_silver,
            name=name,
        )

    chassis.visual(Box((0.06, 0.07, 0.26)), origin=Origin(xyz=(0.46, 0.56, 0.43)), material=dark_steel, name="left_upright")
    chassis.visual(Box((0.06, 0.07, 0.26)), origin=Origin(xyz=(-0.46, 0.56, 0.43)), material=dark_steel, name="right_upright")
    chassis.visual(Box((0.18, 0.12, 0.10)), origin=Origin(xyz=(0.40, -0.54, 0.32)), material=dark_steel, name="left_rear_hub_carrier")
    chassis.visual(Box((0.18, 0.12, 0.10)), origin=Origin(xyz=(-0.40, -0.54, 0.32)), material=dark_steel, name="right_rear_hub_carrier")

    front_body = model.part("front_body")
    front_body.visual(
        _save_mesh(
            "front_left_fender",
            superellipse_side_loft(
                [(0.44, 0.63, 0.70, 0.18), (0.58, 0.64, 0.75, 0.27), (0.74, 0.63, 0.69, 0.18)],
                exponents=2.2,
                segments=42,
            ).translate(0.44, -0.58, -0.70),
        ),
        material=body_red,
        name="front_left_fender",
    )
    front_body.visual(
        _save_mesh(
            "front_right_fender",
            superellipse_side_loft(
                [(0.44, 0.63, 0.70, 0.18), (0.58, 0.64, 0.75, 0.27), (0.74, 0.63, 0.69, 0.18)],
                exponents=2.2,
                segments=42,
            ).translate(-0.44, -0.58, -0.70),
        ),
        material=body_red,
        name="front_right_fender",
    )
    front_body.visual(Box((0.34, 0.14, 0.06)), origin=Origin(xyz=(0.0, 0.02, 0.00)), material=body_red, name="front_nose")
    front_body.visual(Box((0.24, 0.12, 0.06)), origin=Origin(xyz=(0.22, 0.04, 0.01)), material=body_red, name="front_left_shroud")
    front_body.visual(Box((0.24, 0.12, 0.06)), origin=Origin(xyz=(-0.22, 0.04, 0.01)), material=body_red, name="front_right_shroud")
    front_body.visual(Box((0.18, 0.10, 0.06)), origin=Origin(xyz=(0.0, -0.02, 0.01)), material=dark_steel, name="bumper_block")
    front_body.visual(Box((0.26, 0.08, 0.03)), origin=Origin(xyz=(0.0, -0.06, 0.03)), material=black_plastic, name="front_bumper")
    front_body.visual(Box((0.10, 0.06, 0.08)), origin=Origin(xyz=(0.0, 0.0, -0.03)), material=dark_steel, name="front_body_bracket")

    seat = model.part("seat")
    seat.visual(
        _save_mesh(
            "seat_shell",
            superellipse_side_loft(
                [(-0.02, 0.69, 0.79, 0.29), (-0.18, 0.68, 0.82, 0.34), (-0.34, 0.67, 0.78, 0.30)],
                exponents=2.0,
                segments=44,
            ).translate(0.0, 0.18, -0.74),
        ),
        material=seat_vinyl,
        name="seat_pad",
    )
    seat.visual(Box((0.28, 0.26, 0.04)), origin=Origin(xyz=(0.0, -0.02, -0.01)), material=black_plastic, name="seat_pan")
    seat.visual(Box((0.18, 0.16, 0.06)), origin=Origin(xyz=(0.0, -0.02, -0.055)), material=dark_steel, name="seat_bracket")

    rear_body = model.part("rear_body")
    rear_body.visual(
        _save_mesh(
            "rear_left_fender",
            superellipse_side_loft(
                [(-0.74, 0.60, 0.69, 0.22), (-0.55, 0.61, 0.76, 0.35), (-0.34, 0.59, 0.72, 0.26)],
                exponents=2.3,
                segments=42,
            ).translate(0.46, 0.50, -0.64),
        ),
        material=body_red,
        name="rear_left_fender",
    )
    rear_body.visual(
        _save_mesh(
            "rear_right_fender",
            superellipse_side_loft(
                [(-0.74, 0.60, 0.69, 0.22), (-0.55, 0.61, 0.76, 0.35), (-0.34, 0.59, 0.72, 0.26)],
                exponents=2.3,
                segments=42,
            ).translate(-0.46, 0.50, -0.64),
        ),
        material=body_red,
        name="rear_right_fender",
    )
    rear_body.visual(Box((0.66, 0.40, 0.05)), origin=Origin(xyz=(0.0, 0.04, -0.07)), material=body_red, name="rear_deck")
    rear_body.visual(Box((0.30, 0.16, 0.07)), origin=Origin(xyz=(0.0, 0.00, 0.00)), material=dark_steel, name="tail_center")
    rear_body.visual(Box((0.78, 0.16, 0.04)), origin=Origin(xyz=(0.0, 0.05, -0.03)), material=body_red, name="rear_spreader")
    rear_body.visual(Box((0.18, 0.18, 0.08)), origin=Origin(xyz=(0.32, 0.10, 0.00)), material=body_red, name="rear_left_pod")
    rear_body.visual(Box((0.18, 0.18, 0.08)), origin=Origin(xyz=(-0.32, 0.10, 0.00)), material=body_red, name="rear_right_pod")
    rear_body.visual(Box((0.24, 0.18, 0.07)), origin=Origin(xyz=(0.0, 0.00, -0.11)), material=dark_steel, name="rear_body_bracket")

    left_footrest = model.part("left_footrest")
    left_footrest.visual(Box((0.18, 0.30, 0.05)), origin=Origin(xyz=(0.04, 0.0, 0.0)), material=black_plastic, name="left_footrest_pad")
    left_footrest.visual(
        _save_mesh(
            "left_footrest_loop",
            tube_from_spline_points(
                [(0.0, 0.12, 0.0), (0.08, 0.08, 0.0), (0.10, 0.0, 0.0), (0.08, -0.12, 0.0), (0.0, -0.16, 0.0)],
                radius=0.013,
                samples_per_segment=12,
                radial_segments=14,
            ),
        ),
        material=frame_paint,
        name="left_footrest_loop",
    )
    left_footrest.visual(Box((0.06, 0.12, 0.06)), origin=Origin(xyz=(-0.04, 0.0, 0.03)), material=frame_paint, name="left_footrest_bracket")

    right_footrest = model.part("right_footrest")
    right_footrest.visual(Box((0.18, 0.30, 0.05)), origin=Origin(xyz=(-0.04, 0.0, 0.0)), material=black_plastic, name="right_footrest_pad")
    right_footrest.visual(
        _save_mesh(
            "right_footrest_loop",
            tube_from_spline_points(
                [(0.0, 0.12, 0.0), (-0.08, 0.08, 0.0), (-0.10, 0.0, 0.0), (-0.08, -0.12, 0.0), (0.0, -0.16, 0.0)],
                radius=0.013,
                samples_per_segment=12,
                radial_segments=14,
            ),
        ),
        material=frame_paint,
        name="right_footrest_loop",
    )
    right_footrest.visual(Box((0.06, 0.12, 0.06)), origin=Origin(xyz=(0.04, 0.0, 0.03)), material=frame_paint, name="right_footrest_bracket")

    handlebar = model.part("handlebar")
    handlebar.visual(Cylinder(radius=0.023, length=0.12), origin=Origin(xyz=(0.0, 0.0, 0.06)), material=dark_steel, name="steering_stem")
    handlebar.visual(Box((0.08, 0.05, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.11)), material=dark_steel, name="bar_clamp")
    handlebar.visual(
        _save_mesh(
            "handlebar_bar",
            tube_from_spline_points(
                [(-0.31, -0.02, 0.09), (-0.20, -0.01, 0.12), (-0.08, 0.0, 0.14), (0.08, 0.0, 0.14), (0.20, -0.01, 0.12), (0.31, -0.02, 0.09)],
                radius=0.016,
                samples_per_segment=16,
                radial_segments=18,
            ),
        ),
        material=dark_steel,
        name="handlebar_bar",
    )
    handlebar.visual(
        Cylinder(radius=0.020, length=0.11),
        origin=Origin(xyz=(0.27, -0.02, 0.09), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.020, length=0.11),
        origin=Origin(xyz=(-0.27, -0.02, 0.09), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.010, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.10), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="cross_brace",
    )
    handlebar.visual(Box((0.10, 0.04, 0.05)), origin=Origin(xyz=(0.0, -0.01, 0.13)), material=black_plastic, name="center_pad")

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(
        front_left_wheel,
        "front_left_wheel",
        tire_radius=0.30,
        tire_width=0.24,
        hub_width=0.30,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
        rubber=rubber,
        side_sign=1.0,
        add_knuckle=True,
    )

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(
        front_right_wheel,
        "front_right_wheel",
        tire_radius=0.30,
        tire_width=0.24,
        hub_width=0.30,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
        rubber=rubber,
        side_sign=-1.0,
        add_knuckle=True,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(
        rear_left_wheel,
        "rear_left_wheel",
        tire_radius=0.31,
        tire_width=0.30,
        hub_width=0.36,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
        rubber=rubber,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(
        rear_right_wheel,
        "rear_right_wheel",
        tire_radius=0.31,
        tire_width=0.30,
        hub_width=0.36,
        wheel_steel=wheel_steel,
        dark_steel=dark_steel,
        rubber=rubber,
    )

    model.articulation("front_body_mount", ArticulationType.FIXED, parent=chassis, child=front_body, origin=Origin(xyz=(0.0, 0.64, 0.73)))
    model.articulation("seat_mount", ArticulationType.FIXED, parent=chassis, child=seat, origin=Origin(xyz=(0.0, -0.18, 0.825)))
    model.articulation("rear_body_mount", ArticulationType.FIXED, parent=chassis, child=rear_body, origin=Origin(xyz=(0.0, -0.57, 0.79)))
    model.articulation("left_footrest_mount", ArticulationType.FIXED, parent=chassis, child=left_footrest, origin=Origin(xyz=(0.46, -0.02, 0.34)))
    model.articulation("right_footrest_mount", ArticulationType.FIXED, parent=chassis, child=right_footrest, origin=Origin(xyz=(-0.46, -0.02, 0.34)))
    model.articulation(
        "handlebar_steer",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.18, 0.74)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "front_left_steer",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_left_wheel,
        origin=Origin(xyz=(0.64, 0.60, 0.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.55, upper=0.55),
        mimic=Mimic("handlebar_steer"),
    )
    model.articulation(
        "front_right_steer",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_right_wheel,
        origin=Origin(xyz=(-0.64, 0.60, 0.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.55, upper=0.55),
        mimic=Mimic("handlebar_steer"),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.67, -0.54, 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=20.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.67, -0.54, 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    seat = object_model.get_part("seat")
    left_footrest = object_model.get_part("left_footrest")
    right_footrest = object_model.get_part("right_footrest")
    front_body = object_model.get_part("front_body")
    handlebar_steer = object_model.get_articulation("handlebar_steer")
    front_left_steer = object_model.get_articulation("front_left_steer")
    front_right_steer = object_model.get_articulation("front_right_steer")

    ctx.expect_origin_distance(
        front_left_wheel,
        front_right_wheel,
        axes="x",
        min_dist=1.24,
        max_dist=1.32,
        name="front track width is ATV-like",
    )
    ctx.expect_origin_distance(
        rear_left_wheel,
        rear_right_wheel,
        axes="x",
        min_dist=1.30,
        max_dist=1.38,
        name="rear track width is ATV-like",
    )
    ctx.expect_origin_gap(
        front_left_wheel,
        rear_left_wheel,
        axis="y",
        min_gap=1.00,
        max_gap=1.20,
        name="wheelbase stays mechanically plausible",
    )
    ctx.expect_origin_gap(
        seat,
        left_footrest,
        axis="z",
        min_gap=0.42,
        max_gap=0.56,
        name="seat is clearly above left footrest",
    )
    ctx.expect_origin_gap(
        seat,
        right_footrest,
        axis="z",
        min_gap=0.42,
        max_gap=0.56,
        name="seat is clearly above right footrest",
    )
    ctx.expect_origin_gap(
        front_body,
        front_left_wheel,
        axis="z",
        min_gap=0.32,
        max_gap=0.50,
        name="front fender bodywork sits above the front wheel centerline",
    )

    left_mimic_ok = (
        front_left_steer.mimic is not None
        and front_left_steer.mimic.joint == handlebar_steer.name
        and abs(front_left_steer.mimic.multiplier - 1.0) < 1e-9
    )
    right_mimic_ok = (
        front_right_steer.mimic is not None
        and front_right_steer.mimic.joint == handlebar_steer.name
        and abs(front_right_steer.mimic.multiplier - 1.0) < 1e-9
    )
    ctx.check(
        "front wheels are linked to handlebar steering",
        left_mimic_ok and right_mimic_ok,
        details=f"left_mimic={front_left_steer.mimic}, right_mimic={front_right_steer.mimic}",
    )

    chassis = object_model.get_part("chassis")
    handlebar = object_model.get_part("handlebar")
    ctx.allow_overlap(
        chassis,
        handlebar,
        elem_a="steering_head",
        elem_b="steering_stem",
        reason="The steering stem is intentionally represented as passing through a simplified solid steering head tube.",
    )
    ctx.allow_overlap(
        chassis,
        front_left_wheel,
        elem_a="left_upright",
        elem_b="hub_barrel",
        reason="The front hub barrel intentionally passes through the simplified upright carrier at the steering knuckle.",
    )
    ctx.allow_overlap(
        chassis,
        front_right_wheel,
        elem_a="right_upright",
        elem_b="hub_barrel",
        reason="The front hub barrel intentionally passes through the simplified upright carrier at the steering knuckle.",
    )
    ctx.allow_overlap(
        chassis,
        rear_left_wheel,
        elem_a="left_rear_hub_carrier",
        elem_b="hub_barrel",
        reason="The rear hub barrel is intentionally represented as seated through the simplified hub carrier.",
    )
    ctx.allow_overlap(
        chassis,
        rear_right_wheel,
        elem_a="right_rear_hub_carrier",
        elem_b="hub_barrel",
        reason="The rear hub barrel is intentionally represented as seated through the simplified hub carrier.",
    )
    ctx.allow_overlap(
        chassis,
        front_left_wheel,
        elem_a="left_upright",
        elem_b="hub_cap",
        reason="The front wheel hub cap shares the same simplified pass-through hub carrier envelope as the hub barrel.",
    )
    ctx.allow_overlap(
        chassis,
        front_right_wheel,
        elem_a="right_upright",
        elem_b="hub_cap",
        reason="The front wheel hub cap shares the same simplified pass-through hub carrier envelope as the hub barrel.",
    )
    ctx.allow_overlap(
        chassis,
        rear_left_wheel,
        elem_a="left_rear_hub_carrier",
        elem_b="hub_cap",
        reason="The rear hub cap is intentionally nested within the simplified carrier around the axle center.",
    )
    ctx.allow_overlap(
        chassis,
        rear_right_wheel,
        elem_a="right_rear_hub_carrier",
        elem_b="hub_cap",
        reason="The rear hub cap is intentionally nested within the simplified carrier around the axle center.",
    )
    ctx.allow_overlap(
        chassis,
        left_footrest,
        elem_a="left_floor_panel",
        elem_b="left_footrest_bracket",
        reason="The left footrest bracket is simplified as a nested mounting tongue under the floor panel.",
    )
    ctx.allow_overlap(
        chassis,
        right_footrest,
        elem_a="right_floor_panel",
        elem_b="right_footrest_bracket",
        reason="The right footrest bracket is simplified as a nested mounting tongue under the floor panel.",
    )
    ctx.allow_overlap(
        chassis,
        front_left_wheel,
        elem_a="left_upright",
        elem_b="knuckle_arm",
        reason="The steer knuckle arm is simplified as seated within the upright carrier envelope.",
    )
    ctx.allow_overlap(
        chassis,
        front_right_wheel,
        elem_a="right_upright",
        elem_b="knuckle_arm",
        reason="The steer knuckle arm is simplified as seated within the upright carrier envelope.",
    )
    ctx.allow_overlap(
        chassis,
        seat,
        elem_a="tank_cover",
        elem_b="seat_bracket",
        reason="The seat bracket is simplified as a solid subframe block resting into the tank-cover support volume.",
    )

    rest_aabb = ctx.part_world_aabb(front_left_wheel)
    with ctx.pose({handlebar_steer: 0.45}):
        steered_aabb = ctx.part_world_aabb(front_left_wheel)

    rest_span_x = None if rest_aabb is None else rest_aabb[1][0] - rest_aabb[0][0]
    steered_span_x = None if steered_aabb is None else steered_aabb[1][0] - steered_aabb[0][0]
    ctx.check(
        "front steering changes wheel yaw in pose",
        rest_span_x is not None and steered_span_x is not None and steered_span_x > rest_span_x + 0.10,
        details=f"rest_span_x={rest_span_x}, steered_span_x={steered_span_x}",
    )

    return ctx.report()


object_model = build_object_model()
