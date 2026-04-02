from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _arch_section(
    *,
    y: float,
    half_width: float,
    edge_z: float,
    crown_z: float,
    samples: int = 13,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(samples):
        t = index / (samples - 1)
        x = -half_width + 2.0 * half_width * t
        arch = sin(pi * t)
        z = edge_z + (crown_z - edge_z) * arch
        points.append((x, y, z))
    return points


def _open_sheet_shell(
    sections: list[list[tuple[float, float, float]]],
    *,
    thickness: float,
) -> MeshGeometry:
    geom = MeshGeometry()
    outer_ids: list[list[int]] = []
    inner_ids: list[list[int]] = []

    for section in sections:
        outer_ids.append([geom.add_vertex(x, y, z) for x, y, z in section])
        inner_ids.append([geom.add_vertex(x, y, z - thickness) for x, y, z in section])

    for sec_index in range(len(sections) - 1):
        section_len = len(sections[sec_index])
        for point_index in range(section_len - 1):
            _add_quad(
                geom,
                outer_ids[sec_index][point_index],
                outer_ids[sec_index + 1][point_index],
                outer_ids[sec_index + 1][point_index + 1],
                outer_ids[sec_index][point_index + 1],
            )
            _add_quad(
                geom,
                inner_ids[sec_index][point_index + 1],
                inner_ids[sec_index + 1][point_index + 1],
                inner_ids[sec_index + 1][point_index],
                inner_ids[sec_index][point_index],
            )

    for sec_index in range(len(sections) - 1):
        _add_quad(
            geom,
            outer_ids[sec_index][0],
            outer_ids[sec_index + 1][0],
            inner_ids[sec_index + 1][0],
            inner_ids[sec_index][0],
        )
        _add_quad(
            geom,
            outer_ids[sec_index][-1],
            inner_ids[sec_index][-1],
            inner_ids[sec_index + 1][-1],
            outer_ids[sec_index + 1][-1],
        )

    first_len = len(sections[0])
    last_len = len(sections[-1])
    for point_index in range(first_len - 1):
        _add_quad(
            geom,
            outer_ids[0][point_index + 1],
            outer_ids[0][point_index],
            inner_ids[0][point_index],
            inner_ids[0][point_index + 1],
        )
    for point_index in range(last_len - 1):
        _add_quad(
            geom,
            outer_ids[-1][point_index],
            outer_ids[-1][point_index + 1],
            inner_ids[-1][point_index + 1],
            inner_ids[-1][point_index],
        )
    return geom


def _wheel_visuals(
    part,
    *,
    mesh_prefix: str,
    radius: float,
    width: float,
    hub_width: float,
    wheel_dark,
    wheel_metal,
    tire_rubber,
    inner_cap_sign: float | None = None,
    dual_axle_caps: bool = False,
) -> None:
    half_width = width * 0.5
    tire_profile = [
        (radius * 0.58, -half_width * 0.96),
        (radius * 0.79, -half_width),
        (radius * 0.93, -half_width * 0.72),
        (radius, -half_width * 0.26),
        (radius, half_width * 0.26),
        (radius * 0.93, half_width * 0.72),
        (radius * 0.79, half_width),
        (radius * 0.58, half_width * 0.96),
        (radius * 0.46, half_width * 0.36),
        (radius * 0.42, 0.0),
        (radius * 0.46, -half_width * 0.36),
        (radius * 0.58, -half_width * 0.96),
    ]
    part.visual(
        _mesh(mesh_prefix + "_tire", LatheGeometry(tire_profile, segments=64).rotate_y(pi / 2.0)),
        material=tire_rubber,
        name="tire",
    )
    axle_orientation = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=radius * 0.72, length=width * 0.52),
        origin=axle_orientation,
        material=wheel_dark,
        name="rim_barrel",
    )
    part.visual(
        Cylinder(radius=radius * 0.56, length=0.005),
        origin=Origin(xyz=(width * 0.12, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="outer_disc",
    )
    part.visual(
        Cylinder(radius=radius * 0.56, length=0.005),
        origin=Origin(xyz=(-width * 0.12, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_metal,
        name="inner_disc",
    )
    part.visual(
        Cylinder(radius=radius * 0.13, length=hub_width),
        origin=axle_orientation,
        material=wheel_metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=radius * 0.045, length=max(hub_width + 0.020, 0.054)),
        origin=axle_orientation,
        material=wheel_dark,
        name="axle_shaft",
    )
    if dual_axle_caps:
        part.visual(
            Cylinder(radius=radius * 0.08, length=0.008),
            origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_metal,
            name="axle_left",
        )
        part.visual(
            Cylinder(radius=radius * 0.08, length=0.008),
            origin=Origin(xyz=(-0.026, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_metal,
            name="axle_right",
        )
    if inner_cap_sign is not None:
        part.visual(
            Cylinder(radius=radius * 0.08, length=0.010),
            origin=Origin(xyz=(inner_cap_sign * 0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_metal,
            name="inner_axle_cap",
        )
        part.visual(
            Cylinder(radius=radius * 0.08, length=0.008),
            origin=Origin(xyz=(-inner_cap_sign * 0.024, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_metal,
            name="outer_axle_cap",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jogging_stroller")

    frame_gray = model.material("frame_gray", rgba=(0.33, 0.35, 0.37, 1.0))
    frame_dark = model.material("frame_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    fabric_charcoal = model.material("fabric_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    canopy_black = model.material("canopy_black", rgba=(0.09, 0.09, 0.10, 1.0))
    rim_silver = model.material("rim_silver", rgba=(0.78, 0.79, 0.80, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.04, 0.04, 0.04, 1.0))
    grip_foam = model.material("grip_foam", rgba=(0.11, 0.11, 0.12, 1.0))

    chassis = model.part("chassis")
    chassis.inertial = Inertial.from_geometry(
        Box((0.70, 1.10, 1.05)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.10, 0.50)),
    )

    left_lower_rail = tube_from_spline_points(
        [
            (0.25, -0.34, 0.23),
            (0.22, -0.10, 0.27),
            (0.18, 0.16, 0.31),
            (0.03, 0.56, 0.39),
        ],
        radius=0.014,
        samples_per_segment=14,
        radial_segments=18,
    )
    right_lower_rail = tube_from_spline_points(
        _mirror_x(
            [
                (0.25, -0.34, 0.23),
                (0.22, -0.10, 0.27),
                (0.18, 0.16, 0.31),
                (0.03, 0.56, 0.39),
            ]
        ),
        radius=0.014,
        samples_per_segment=14,
        radial_segments=18,
    )
    left_upper_rail = tube_from_spline_points(
        [
            (0.25, -0.34, 0.47),
            (0.25, -0.22, 0.63),
            (0.26, -0.12, 0.86),
            (0.29, -0.18, 1.02),
        ],
        radius=0.014,
        samples_per_segment=14,
        radial_segments=18,
    )
    right_upper_rail = tube_from_spline_points(
        _mirror_x(
            [
                (0.25, -0.34, 0.47),
                (0.25, -0.22, 0.63),
                (0.26, -0.12, 0.86),
                (0.29, -0.18, 1.02),
            ]
        ),
        radius=0.014,
        samples_per_segment=14,
        radial_segments=18,
    )
    head_tube = LatheGeometry.from_shell_profiles(
        [(0.024, -0.040), (0.024, 0.040)],
        [(0.018, -0.040), (0.018, 0.040)],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )
    seat_shell = _open_sheet_shell(
        [
            _arch_section(y=0.20, half_width=0.20, edge_z=0.46, crown_z=0.43),
            _arch_section(y=0.05, half_width=0.22, edge_z=0.56, crown_z=0.53),
            _arch_section(y=-0.12, half_width=0.23, edge_z=0.70, crown_z=0.66),
        ],
        thickness=0.004,
    )

    chassis.visual(_mesh("stroller_left_lower_rail", left_lower_rail), material=frame_gray, name="left_lower_rail")
    chassis.visual(_mesh("stroller_right_lower_rail", right_lower_rail), material=frame_gray, name="right_lower_rail")
    chassis.visual(_mesh("stroller_left_upper_rail", left_upper_rail), material=frame_gray, name="left_upper_rail")
    chassis.visual(_mesh("stroller_right_upper_rail", right_upper_rail), material=frame_gray, name="right_upper_rail")
    chassis.visual(
        _mesh("stroller_head_tube", head_tube),
        origin=Origin(xyz=(0.0, 0.56, 0.39)),
        material=frame_dark,
        name="head_tube",
    )
    chassis.visual(
        Cylinder(radius=0.013, length=0.58),
        origin=Origin(xyz=(0.0, -0.34, 0.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="rear_axle_tube",
    )
    chassis.visual(
        Cylinder(radius=0.013, length=0.24),
        origin=Origin(xyz=(0.25, -0.34, 0.35)),
        material=frame_gray,
        name="left_rear_stay",
    )
    chassis.visual(
        Cylinder(radius=0.013, length=0.24),
        origin=Origin(xyz=(-0.25, -0.34, 0.35)),
        material=frame_gray,
        name="right_rear_stay",
    )
    chassis.visual(
        Cylinder(radius=0.012, length=0.52),
        origin=Origin(xyz=(0.0, -0.12, 0.70), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="seat_back_bar",
    )
    chassis.visual(
        Cylinder(radius=0.012, length=0.46),
        origin=Origin(xyz=(0.0, 0.20, 0.46), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="seat_front_bar",
    )
    chassis.visual(
        Cylinder(radius=0.011, length=0.34),
        origin=Origin(xyz=(0.0, 0.30, 0.33), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="footrest_bar",
    )
    chassis.visual(
        Cylinder(radius=0.014, length=0.60),
        origin=Origin(xyz=(0.0, -0.18, 1.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="handlebar",
    )
    chassis.visual(
        Cylinder(radius=0.016, length=0.14),
        origin=Origin(xyz=(0.18, -0.18, 1.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_foam,
        name="left_grip",
    )
    chassis.visual(
        Cylinder(radius=0.016, length=0.14),
        origin=Origin(xyz=(-0.18, -0.18, 1.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_foam,
        name="right_grip",
    )
    chassis.visual(
        _mesh("stroller_seat_shell", seat_shell),
        material=fabric_charcoal,
        name="seat_shell",
    )
    chassis.visual(
        _mesh(
            "stroller_left_seat_edge",
            tube_from_spline_points(
                [(0.20, 0.20, 0.46), (0.22, 0.05, 0.56), (0.23, -0.12, 0.70)],
                radius=0.010,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=frame_gray,
        name="left_seat_edge",
    )
    chassis.visual(
        _mesh(
            "stroller_right_seat_edge",
            tube_from_spline_points(
                _mirror_x([(0.20, 0.20, 0.46), (0.22, 0.05, 0.56), (0.23, -0.12, 0.70)]),
                radius=0.010,
                samples_per_segment=10,
                radial_segments=14,
            ),
        ),
        material=frame_gray,
        name="right_seat_edge",
    )
    chassis.visual(
        _mesh(
            "stroller_left_front_seat_strut",
            tube_from_spline_points(
                [(0.18, 0.16, 0.31), (0.19, 0.18, 0.38), (0.20, 0.20, 0.46)],
                radius=0.010,
                samples_per_segment=8,
                radial_segments=14,
            ),
        ),
        material=frame_gray,
        name="left_front_seat_strut",
    )
    chassis.visual(
        _mesh(
            "stroller_right_front_seat_strut",
            tube_from_spline_points(
                _mirror_x([(0.18, 0.16, 0.31), (0.19, 0.18, 0.38), (0.20, 0.20, 0.46)]),
                radius=0.010,
                samples_per_segment=8,
                radial_segments=14,
            ),
        ),
        material=frame_gray,
        name="right_front_seat_strut",
    )
    chassis.visual(
        _mesh(
            "stroller_left_rear_seat_strut",
            tube_from_spline_points(
                [(0.25, -0.22, 0.63), (0.24, -0.18, 0.67), (0.23, -0.12, 0.70)],
                radius=0.010,
                samples_per_segment=8,
                radial_segments=14,
            ),
        ),
        material=frame_gray,
        name="left_rear_seat_strut",
    )
    chassis.visual(
        _mesh(
            "stroller_right_rear_seat_strut",
            tube_from_spline_points(
                _mirror_x([(0.25, -0.22, 0.63), (0.24, -0.18, 0.67), (0.23, -0.12, 0.70)]),
                radius=0.010,
                samples_per_segment=8,
                radial_segments=14,
            ),
        ),
        material=frame_gray,
        name="right_rear_seat_strut",
    )
    chassis.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(0.279, -0.34, 0.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="left_axle_mount",
    )
    chassis.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(xyz=(-0.279, -0.34, 0.23), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="right_axle_mount",
    )
    chassis.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.300, -0.12, 0.86), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="left_canopy_boss",
    )
    chassis.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(-0.300, -0.12, 0.86), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="right_canopy_boss",
    )
    chassis.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(xyz=(0.280, -0.12, 0.86), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="left_canopy_bracket",
    )
    chassis.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(xyz=(-0.280, -0.12, 0.86), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="right_canopy_bracket",
    )

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.10, 0.22, 0.22)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.05, -0.06)),
    )
    left_leg = tube_from_spline_points(
        [
            (0.040, 0.05, 0.00),
            (0.038, 0.15, -0.10),
            (0.034, 0.25, -0.22),
        ],
        radius=0.010,
        samples_per_segment=10,
        radial_segments=16,
    )
    right_leg = tube_from_spline_points(
        _mirror_x(
            [
                (0.040, 0.05, 0.00),
                (0.038, 0.15, -0.10),
                (0.034, 0.25, -0.22),
            ]
        ),
        radius=0.010,
        samples_per_segment=10,
        radial_segments=16,
    )
    fork.visual(
        Cylinder(radius=0.017, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=frame_dark,
        name="steerer",
    )
    fork.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=frame_dark,
        name="upper_bearing",
    )
    fork.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.042)),
        material=frame_dark,
        name="lower_bearing",
    )
    fork.visual(
        Cylinder(radius=0.013, length=0.10),
        origin=Origin(xyz=(0.0, 0.05, 0.00), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="fork_crown",
    )
    fork.visual(
        Cylinder(radius=0.004, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.05), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="top_bridge",
    )
    fork.visual(
        _mesh(
            "stroller_fork_left_upright",
            tube_from_spline_points(
                [(0.03, 0.0, 0.05), (0.028, 0.025, 0.035), (0.022, 0.05, 0.005)],
                radius=0.004,
                samples_per_segment=8,
                radial_segments=12,
            ),
        ),
        material=frame_dark,
        name="left_upright",
    )
    fork.visual(
        _mesh(
            "stroller_fork_right_upright",
            tube_from_spline_points(
                _mirror_x([(0.03, 0.0, 0.05), (0.028, 0.025, 0.035), (0.022, 0.05, 0.005)]),
                radius=0.004,
                samples_per_segment=8,
                radial_segments=12,
            ),
        ),
        material=frame_dark,
        name="right_upright",
    )
    fork.visual(_mesh("stroller_fork_left_leg", left_leg), material=frame_gray, name="left_leg")
    fork.visual(_mesh("stroller_fork_right_leg", right_leg), material=frame_gray, name="right_leg")
    fork.visual(
        Box((0.008, 0.024, 0.050)),
        origin=Origin(xyz=(0.034, 0.25, -0.22)),
        material=frame_dark,
        name="left_dropout",
    )
    fork.visual(
        Box((0.008, 0.024, 0.050)),
        origin=Origin(xyz=(-0.034, 0.25, -0.22)),
        material=frame_dark,
        name="right_dropout",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.21, length=0.05),
        mass=1.7,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        front_wheel,
        mesh_prefix="stroller_front_wheel",
        radius=0.21,
        width=0.05,
        hub_width=0.036,
        wheel_dark=frame_dark,
        wheel_metal=rim_silver,
        tire_rubber=tire_rubber,
        dual_axle_caps=False,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=0.05),
        mass=2.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        rear_left_wheel,
        mesh_prefix="stroller_rear_left_wheel",
        radius=0.23,
        width=0.05,
        hub_width=0.034,
        wheel_dark=frame_dark,
        wheel_metal=rim_silver,
        tire_rubber=tire_rubber,
        inner_cap_sign=-1.0,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=0.05),
        mass=2.0,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _wheel_visuals(
        rear_right_wheel,
        mesh_prefix="stroller_rear_right_wheel",
        radius=0.23,
        width=0.05,
        hub_width=0.034,
        wheel_dark=frame_dark,
        wheel_metal=rim_silver,
        tire_rubber=tire_rubber,
        inner_cap_sign=1.0,
    )

    canopy = model.part("canopy")
    canopy.inertial = Inertial.from_geometry(
        Box((0.58, 0.50, 0.18)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.24, 0.05)),
    )
    left_canopy_arm = tube_from_spline_points(
        [
            (0.330, 0.00, 0.00),
            (0.324, 0.04, 0.04),
            (0.300, 0.22, 0.03),
            (0.280, 0.47, -0.02),
        ],
        radius=0.009,
        samples_per_segment=14,
        radial_segments=16,
    )
    right_canopy_arm = tube_from_spline_points(
        _mirror_x(
            [
                (0.330, 0.00, 0.00),
                (0.324, 0.04, 0.04),
                (0.300, 0.22, 0.03),
                (0.280, 0.47, -0.02),
            ]
        ),
        radius=0.009,
        samples_per_segment=14,
        radial_segments=16,
    )
    canopy_shell = _open_sheet_shell(
        [
            _arch_section(y=0.03, half_width=0.28, edge_z=0.04, crown_z=0.13),
            _arch_section(y=0.18, half_width=0.29, edge_z=0.02, crown_z=0.15),
            _arch_section(y=0.33, half_width=0.28, edge_z=-0.01, crown_z=0.11),
            _arch_section(y=0.47, half_width=0.26, edge_z=-0.02, crown_z=0.06),
        ],
        thickness=0.004,
    )
    canopy.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.318, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="left_hinge_washer",
    )
    canopy.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(-0.318, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_dark,
        name="right_hinge_washer",
    )
    canopy.visual(
        _mesh("stroller_canopy_left_arm", left_canopy_arm),
        material=frame_gray,
        name="left_support_arm",
    )
    canopy.visual(
        _mesh("stroller_canopy_right_arm", right_canopy_arm),
        material=frame_gray,
        name="right_support_arm",
    )
    canopy.visual(
        Cylinder(radius=0.010, length=0.66),
        origin=Origin(xyz=(0.0, 0.03, 0.04), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="rear_bow",
    )
    canopy.visual(
        Cylinder(radius=0.010, length=0.52),
        origin=Origin(xyz=(0.0, 0.47, -0.02), rpy=(0.0, pi / 2.0, 0.0)),
        material=frame_gray,
        name="front_bow",
    )
    canopy.visual(
        _mesh("stroller_canopy_shell", canopy_shell),
        material=canopy_black,
        name="canopy_shell",
    )

    model.articulation(
        "chassis_to_fork_steer",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=fork,
        origin=Origin(xyz=(0.0, 0.56, 0.39)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.2, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "fork_to_front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.25, -0.22)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "chassis_to_rear_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.315, -0.34, 0.23)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "chassis_to_rear_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.315, -0.34, 0.23)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "chassis_to_canopy",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=canopy,
        origin=Origin(xyz=(0.0, -0.12, 0.86)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.4, lower=-0.25, upper=0.80),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    chassis = object_model.get_part("chassis")
    fork = object_model.get_part("fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    canopy = object_model.get_part("canopy")

    steer = object_model.get_articulation("chassis_to_fork_steer")
    front_spin = object_model.get_articulation("fork_to_front_wheel_spin")
    rear_left_spin = object_model.get_articulation("chassis_to_rear_left_wheel")
    rear_right_spin = object_model.get_articulation("chassis_to_rear_right_wheel")
    canopy_joint = object_model.get_articulation("chassis_to_canopy")

    for part_name in ("chassis", "fork", "front_wheel", "rear_left_wheel", "rear_right_wheel", "canopy"):
        ctx.check(
            f"{part_name} present",
            object_model.get_part(part_name) is not None,
            details=f"missing {part_name}",
        )

    ctx.check("front steer axis is vertical", steer.axis == (0.0, 0.0, 1.0), details=f"axis={steer.axis}")
    ctx.check("front wheel spins on x axle", front_spin.axis == (1.0, 0.0, 0.0), details=f"axis={front_spin.axis}")
    ctx.check("rear left wheel spins on x axle", rear_left_spin.axis == (1.0, 0.0, 0.0), details=f"axis={rear_left_spin.axis}")
    ctx.check("rear right wheel spins on x axle", rear_right_spin.axis == (1.0, 0.0, 0.0), details=f"axis={rear_right_spin.axis}")
    ctx.check("canopy hinge axis runs side to side", canopy_joint.axis == (1.0, 0.0, 0.0), details=f"axis={canopy_joint.axis}")

    ctx.expect_gap(
        fork,
        front_wheel,
        axis="x",
        positive_elem="left_dropout",
        negative_elem="axle_shaft",
        min_gap=0.0,
        max_gap=0.003,
        name="front wheel left axle seats in fork dropout",
    )
    ctx.expect_gap(
        front_wheel,
        fork,
        axis="x",
        positive_elem="axle_shaft",
        negative_elem="right_dropout",
        min_gap=0.0,
        max_gap=0.003,
        name="front wheel right axle seats in fork dropout",
    )
    ctx.expect_contact(rear_left_wheel, chassis, elem_a="inner_axle_cap", elem_b="left_axle_mount", name="left rear wheel contacts axle mount")
    ctx.expect_contact(rear_right_wheel, chassis, elem_a="inner_axle_cap", elem_b="right_axle_mount", name="right rear wheel contacts axle mount")
    ctx.expect_contact(canopy, chassis, elem_a="left_hinge_washer", elem_b="left_canopy_boss", name="left canopy hinge is mounted")
    ctx.expect_contact(canopy, chassis, elem_a="right_hinge_washer", elem_b="right_canopy_boss", name="right canopy hinge is mounted")

    rest_tire_aabb = ctx.part_element_world_aabb(front_wheel, elem="tire")
    with ctx.pose({steer: 0.45}):
        turned_tire_aabb = ctx.part_element_world_aabb(front_wheel, elem="tire")
    rest_front_bow_aabb = ctx.part_element_world_aabb(canopy, elem="front_bow")
    with ctx.pose({canopy_joint: canopy_joint.motion_limits.upper}):
        raised_front_bow_aabb = ctx.part_element_world_aabb(canopy, elem="front_bow")

    ctx.check(
        "front wheel visibly yaws when steering",
        rest_tire_aabb is not None
        and turned_tire_aabb is not None
        and (turned_tire_aabb[1][0] - turned_tire_aabb[0][0]) > (rest_tire_aabb[1][0] - rest_tire_aabb[0][0]) + 0.10,
        details=f"rest={rest_tire_aabb}, turned={turned_tire_aabb}",
    )
    ctx.check(
        "canopy retracts upward and rearward",
        rest_front_bow_aabb is not None
        and raised_front_bow_aabb is not None
        and raised_front_bow_aabb[0][2] > rest_front_bow_aabb[0][2] + 0.15
        and raised_front_bow_aabb[1][1] < rest_front_bow_aabb[1][1] - 0.08,
        details=f"rest={rest_front_bow_aabb}, raised={raised_front_bow_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
