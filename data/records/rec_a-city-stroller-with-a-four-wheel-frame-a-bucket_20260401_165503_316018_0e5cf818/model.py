from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _arch_strip_profile(
    *,
    span: float,
    outer_rise: float,
    inner_rise: float,
    edge_height: float,
    inset: float,
) -> list[tuple[float, float]]:
    half = span * 0.5
    inner_half = half - inset
    outer = [
        (-half, edge_height),
        (-half * 0.88, outer_rise * 0.42),
        (-half * 0.55, outer_rise * 0.82),
        (0.0, outer_rise),
        (half * 0.55, outer_rise * 0.82),
        (half * 0.88, outer_rise * 0.42),
        (half, edge_height),
    ]
    inner = [
        (inner_half, edge_height + 0.025),
        (inner_half * 0.82, inner_rise * 0.52),
        (inner_half * 0.48, inner_rise * 0.86),
        (0.0, inner_rise),
        (-inner_half * 0.48, inner_rise * 0.86),
        (-inner_half * 0.82, inner_rise * 0.52),
        (-inner_half, edge_height + 0.025),
    ]
    return outer + inner


def _build_frame_mesh() -> MeshGeometry:
    frame = MeshGeometry()
    tube_radius = 0.0135

    left_lower = [
        (0.23, -0.21, 0.14),
        (0.21, -0.02, 0.29),
        (0.18, 0.08, 0.38),
        (0.17, 0.28, 0.145),
    ]
    left_rear = [
        (0.23, -0.21, 0.14),
        (0.21, -0.12, 0.50),
        (0.21, -0.20, 0.78),
        (0.20, -0.30, 0.96),
    ]
    left_front_brace = [
        (0.19, -0.11, 0.56),
        (0.18, 0.06, 0.46),
        (0.17, 0.28, 0.145),
    ]

    for points in (
        left_lower,
        _mirror_x(left_lower),
        left_rear,
        _mirror_x(left_rear),
        left_front_brace,
        _mirror_x(left_front_brace),
    ):
        frame.merge(
            tube_from_spline_points(
                points,
                radius=tube_radius,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            )
        )

    for length, xyz, radius in (
        (0.44, (0.0, -0.30, 0.96), 0.0135),
        (0.38, (0.0, -0.12, 0.54), 0.0125),
        (0.38, (0.0, 0.27, 0.145), 0.0115),
        (0.56, (0.0, -0.21, 0.14), 0.0145),
    ):
        frame.merge(
            CylinderGeometry(radius=radius, height=length, radial_segments=22)
            .rotate_y(pi / 2.0)
            .translate(*xyz)
        )

    return frame


def _build_seat_shell_mesh() -> MeshGeometry:
    shell = MeshGeometry()
    shell.merge(BoxGeometry((0.23, 0.18, 0.02)).rotate_x(0.12).translate(0.0, 0.09, 0.06))
    shell.merge(BoxGeometry((0.29, 0.025, 0.36)).rotate_x(0.56).translate(0.0, -0.03, 0.24))
    shell.merge(BoxGeometry((0.028, 0.20, 0.24)).rotate_x(0.24).translate(0.155, 0.06, 0.15))
    shell.merge(BoxGeometry((0.028, 0.20, 0.24)).rotate_x(0.24).translate(-0.155, 0.06, 0.15))
    shell.merge(BoxGeometry((0.030, 0.10, 0.16)).rotate_x(0.50).translate(0.155, -0.08, 0.28))
    shell.merge(BoxGeometry((0.030, 0.10, 0.16)).rotate_x(0.50).translate(-0.155, -0.08, 0.28))
    shell.merge(BoxGeometry((0.19, 0.025, 0.03)).translate(0.0, 0.17, 0.05))
    shell.merge(BoxGeometry((0.040, 0.050, 0.12)).translate(0.155, 0.02, 0.02))
    shell.merge(BoxGeometry((0.040, 0.050, 0.12)).translate(-0.155, 0.02, 0.02))
    shell.merge(BoxGeometry((0.040, 0.032, 0.060)).translate(0.165, 0.00, 0.30))
    shell.merge(BoxGeometry((0.040, 0.032, 0.060)).translate(-0.165, 0.00, 0.30))
    shell.merge(BoxGeometry((0.040, 0.032, 0.050)).translate(0.165, 0.10, 0.255))
    shell.merge(BoxGeometry((0.040, 0.032, 0.050)).translate(-0.165, 0.10, 0.255))
    return shell


def _build_bow_frame(points: list[tuple[float, float, float]], *, pivot_x: float) -> MeshGeometry:
    geom = MeshGeometry()
    geom.merge(
        tube_from_spline_points(
            points,
            radius=0.0075,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        )
    )
    for sign in (-1.0, 1.0):
        geom.merge(
            CylinderGeometry(radius=0.011, height=0.02, radial_segments=18)
            .rotate_y(pi / 2.0)
            .translate(sign * pivot_x, 0.0, 0.0)
        )
    return geom


def _build_canopy_panel(
    *,
    profile_span: float,
    outer_rise: float,
    inner_rise: float,
    edge_height: float,
    inset: float,
    depth: float,
    y_offset: float,
) -> MeshGeometry:
    profile = _arch_strip_profile(
        span=profile_span,
        outer_rise=outer_rise,
        inner_rise=inner_rise,
        edge_height=edge_height,
        inset=inset,
    )
    return (
        ExtrudeGeometry.centered(profile, depth)
        .rotate_x(pi / 2.0)
        .translate(0.0, y_offset, 0.0)
    )


def _add_wheel_visuals(part, *, radius: float, width: float, wheel_gray, rubber, axle_gray) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=radius * 0.76, length=width * 0.72),
        origin=spin_origin,
        material=wheel_gray,
        name="rim",
    )
    part.visual(
        Cylinder(radius=radius * 0.28, length=width * 1.02),
        origin=spin_origin,
        material=axle_gray,
        name="hub",
    )
    part.visual(
        Cylinder(radius=radius * 0.58, length=width * 0.10),
        origin=Origin(xyz=(width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_gray,
        name="outer_disc",
    )
    part.visual(
        Cylinder(radius=radius * 0.58, length=width * 0.10),
        origin=Origin(xyz=(-width * 0.18, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_gray,
        name="inner_disc",
    )


def _add_tube_visual(part, *, mesh_name: str, visual_name: str, points, radius: float, material) -> None:
    part.visual(
        _save_mesh(
            mesh_name,
            tube_from_spline_points(
                points,
                radius=radius,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=material,
        name=visual_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="city_stroller")

    frame_metal = model.material("frame_metal", rgba=(0.30, 0.31, 0.33, 1.0))
    shell_plastic = model.material("shell_plastic", rgba=(0.13, 0.14, 0.15, 1.0))
    seat_fabric = model.material("seat_fabric", rgba=(0.23, 0.24, 0.26, 1.0))
    canopy_fabric = model.material("canopy_fabric", rgba=(0.39, 0.41, 0.45, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    axle_gray = model.material("axle_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    grip_foam = model.material("grip_foam", rgba=(0.17, 0.18, 0.19, 1.0))

    frame = model.part("frame")
    for mesh_name, visual_name, points, radius in (
        (
            "stroller_left_lower_rail",
            "left_lower_rail",
            [(0.215, -0.21, 0.10), (0.180, -0.02, 0.48), (0.170, 0.24, 0.30)],
            0.0115,
        ),
        (
            "stroller_right_lower_rail",
            "right_lower_rail",
            [(-0.215, -0.21, 0.10), (-0.180, -0.02, 0.48), (-0.170, 0.24, 0.30)],
            0.0115,
        ),
        (
            "stroller_left_upper_rail",
            "left_upper_rail",
            [(0.170, -0.30, 0.96), (0.175, -0.18, 0.78), (0.180, -0.06, 0.58), (0.180, -0.02, 0.48)],
            0.0110,
        ),
        (
            "stroller_right_upper_rail",
            "right_upper_rail",
            [(-0.170, -0.30, 0.96), (-0.175, -0.18, 0.78), (-0.180, -0.06, 0.58), (-0.180, -0.02, 0.48)],
            0.0110,
        ),
    ):
        _add_tube_visual(
            frame,
            mesh_name=mesh_name,
            visual_name=visual_name,
            points=points,
            radius=radius,
            material=frame_metal,
        )
    for name, size, xyz, rpy, material in (
        ("rear_axle_crossbar", (0.46, 0.028, 0.028), (0.0, -0.21, 0.10), (0.0, 0.0, 0.0), frame_metal),
        ("seat_crossbar", (0.38, 0.030, 0.030), (0.0, -0.02, 0.48), (0.0, 0.0, 0.0), frame_metal),
        ("handle_crossbar", (0.34, 0.024, 0.024), (0.0, -0.30, 0.96), (0.0, 0.0, 0.0), frame_metal),
        ("lower_front_bar", (0.36, 0.022, 0.022), (0.0, 0.24, 0.30), (0.0, 0.0, 0.0), frame_metal),
    ):
        frame.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)
    frame.visual(
        Cylinder(radius=0.013, length=0.04),
        origin=Origin(xyz=(0.17, 0.24, 0.24)),
        material=frame_metal,
        name="front_left_socket",
    )
    frame.visual(
        Cylinder(radius=0.013, length=0.04),
        origin=Origin(xyz=(-0.17, 0.24, 0.24)),
        material=frame_metal,
        name="front_right_socket",
    )
    frame.visual(
        Box((0.018, 0.018, 0.030)),
        origin=Origin(xyz=(0.17, 0.24, 0.275)),
        material=frame_metal,
        name="front_left_socket_support",
    )
    frame.visual(
        Box((0.018, 0.018, 0.030)),
        origin=Origin(xyz=(-0.17, 0.24, 0.275)),
        material=frame_metal,
        name="front_right_socket_support",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.221, -0.21, 0.09), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="left_rear_axle_boss",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(-0.221, -0.21, 0.09), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_gray,
        name="right_rear_axle_boss",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(0.10, -0.30, 0.96), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_foam,
        name="left_handle_grip",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(-0.10, -0.30, 0.96), rpy=(0.0, pi / 2.0, 0.0)),
        material=grip_foam,
        name="right_handle_grip",
    )

    seat = model.part("seat")
    for name, size, xyz, rpy, material in (
        ("mount_plate", (0.32, 0.04, 0.04), (0.0, 0.00, 0.02), (0.0, 0.0, 0.0), shell_plastic),
        ("seat_shell", (0.24, 0.17, 0.032), (0.0, 0.10, 0.064), (0.12, 0.0, 0.0), shell_plastic),
        ("seat_back_shell", (0.28, 0.03, 0.36), (0.0, -0.02, 0.25), (0.54, 0.0, 0.0), shell_plastic),
        ("left_side_shell", (0.028, 0.18, 0.26), (0.145, 0.05, 0.16), (0.22, 0.0, 0.0), shell_plastic),
        ("right_side_shell", (0.028, 0.18, 0.26), (-0.145, 0.05, 0.16), (0.22, 0.0, 0.0), shell_plastic),
        ("left_upper_wing", (0.032, 0.09, 0.12), (0.140, -0.06, 0.31), (0.40, 0.0, 0.0), shell_plastic),
        ("right_upper_wing", (0.032, 0.09, 0.12), (-0.140, -0.06, 0.31), (0.40, 0.0, 0.0), shell_plastic),
        ("front_rim", (0.18, 0.024, 0.03), (0.0, 0.152, 0.060), (0.0, 0.0, 0.0), shell_plastic),
        ("seat_pan_pad", (0.21, 0.145, 0.014), (0.0, 0.10, 0.078), (0.12, 0.0, 0.0), seat_fabric),
        ("seat_back_pad", (0.24, 0.014, 0.27), (0.0, -0.015, 0.24), (0.54, 0.0, 0.0), seat_fabric),
        ("rear_left_hinge_plate", (0.010, 0.030, 0.060), (0.155, -0.02, 0.32), (0.0, 0.0, 0.0), frame_metal),
        ("rear_right_hinge_plate", (0.010, 0.030, 0.060), (-0.155, -0.02, 0.32), (0.0, 0.0, 0.0), frame_metal),
        ("front_left_hinge_plate", (0.010, 0.030, 0.060), (0.155, 0.125, 0.21), (0.0, 0.0, 0.0), frame_metal),
        ("front_right_hinge_plate", (0.010, 0.030, 0.060), (-0.155, 0.125, 0.21), (0.0, 0.0, 0.0), frame_metal),
    ):
        seat.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)

    front_left_caster = model.part("front_left_caster")
    front_left_caster.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=axle_gray,
        name="stem",
    )
    front_left_caster.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=frame_metal,
        name="crown",
    )
    front_left_caster.visual(
        Box((0.008, 0.018, 0.11)),
        origin=Origin(xyz=(0.01726, 0.0, -0.117)),
        material=frame_metal,
        name="left_fork_leg",
    )
    front_left_caster.visual(
        Box((0.008, 0.018, 0.11)),
        origin=Origin(xyz=(-0.01726, 0.0, -0.117)),
        material=frame_metal,
        name="right_fork_leg",
    )

    front_right_caster = model.part("front_right_caster")
    front_right_caster.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=axle_gray,
        name="stem",
    )
    front_right_caster.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=frame_metal,
        name="crown",
    )
    front_right_caster.visual(
        Box((0.008, 0.018, 0.11)),
        origin=Origin(xyz=(0.01726, 0.0, -0.117)),
        material=frame_metal,
        name="left_fork_leg",
    )
    front_right_caster.visual(
        Box((0.008, 0.018, 0.11)),
        origin=Origin(xyz=(-0.01726, 0.0, -0.117)),
        material=frame_metal,
        name="right_fork_leg",
    )

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(
        front_left_wheel,
        radius=0.070,
        width=0.026,
        wheel_gray=wheel_gray,
        rubber=rubber,
        axle_gray=axle_gray,
    )

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(
        front_right_wheel,
        radius=0.070,
        width=0.026,
        wheel_gray=wheel_gray,
        rubber=rubber,
        axle_gray=axle_gray,
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(
        rear_left_wheel,
        radius=0.090,
        width=0.026,
        wheel_gray=wheel_gray,
        rubber=rubber,
        axle_gray=axle_gray,
    )

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(
        rear_right_wheel,
        radius=0.090,
        width=0.026,
        wheel_gray=wheel_gray,
        rubber=rubber,
        axle_gray=axle_gray,
    )

    rear_bow = model.part("rear_bow")
    rear_bow.visual(
        Box((0.010, 0.030, 0.060)),
        origin=Origin(xyz=(0.165, 0.030, 0.030)),
        material=frame_metal,
        name="left_root",
    )
    rear_bow.visual(
        Box((0.010, 0.030, 0.060)),
        origin=Origin(xyz=(-0.165, 0.030, 0.030)),
        material=frame_metal,
        name="right_root",
    )
    rear_bow.visual(
        Box((0.024, 0.040, 0.040)),
        origin=Origin(xyz=(0.172, 0.055, 0.050)),
        material=frame_metal,
        name="left_connector",
    )
    rear_bow.visual(
        Box((0.024, 0.040, 0.040)),
        origin=Origin(xyz=(-0.172, 0.055, 0.050)),
        material=frame_metal,
        name="right_connector",
    )
    rear_bow.visual(
        Box((0.014, 0.080, 0.120)),
        origin=Origin(xyz=(0.178, 0.090, 0.085)),
        material=frame_metal,
        name="left_arm",
    )
    rear_bow.visual(
        Box((0.014, 0.080, 0.120)),
        origin=Origin(xyz=(-0.178, 0.090, 0.085)),
        material=frame_metal,
        name="right_arm",
    )
    rear_bow.visual(
        Box((0.36, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.125, 0.145)),
        material=frame_metal,
        name="crossbar",
    )
    rear_bow.visual(
        Box((0.24, 0.055, 0.095)),
        origin=Origin(xyz=(0.0, 0.107, 0.105)),
        material=canopy_fabric,
        name="panel",
    )

    front_bow = model.part("front_bow")
    front_bow.visual(
        Box((0.014, 0.160, 0.180)),
        origin=Origin(xyz=(0.167, 0.080, 0.090)),
        material=frame_metal,
        name="left_arm",
    )
    front_bow.visual(
        Box((0.014, 0.160, 0.180)),
        origin=Origin(xyz=(-0.167, 0.080, 0.090)),
        material=frame_metal,
        name="right_arm",
    )
    front_bow.visual(
        Box((0.34, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.165, 0.180)),
        material=frame_metal,
        name="crossbar",
    )
    front_bow.visual(
        Box((0.28, 0.085, 0.125)),
        origin=Origin(xyz=(0.0, 0.135, 0.120)),
        material=canopy_fabric,
        name="panel",
    )

    model.articulation(
        "frame_to_seat",
        ArticulationType.FIXED,
        parent=frame,
        child=seat,
        origin=Origin(xyz=(0.0, -0.02, 0.495)),
    )
    model.articulation(
        "front_left_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_left_caster,
        origin=Origin(xyz=(0.17, 0.24, 0.26)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0),
    )
    model.articulation(
        "front_right_swivel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=front_right_caster,
        origin=Origin(xyz=(-0.17, 0.24, 0.26)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=front_left_caster,
        child=front_left_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.158)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=front_right_caster,
        child=front_right_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.158)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "rear_left_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_left_wheel,
        origin=Origin(xyz=(0.248, -0.21, 0.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=25.0),
    )
    model.articulation(
        "rear_right_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.248, -0.21, 0.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=25.0),
    )
    model.articulation(
        "rear_bow_hinge",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=rear_bow,
        origin=Origin(xyz=(0.0, -0.02, 0.29)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.10),
    )
    model.articulation(
        "front_bow_hinge",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=front_bow,
        origin=Origin(xyz=(0.0, 0.125, 0.21)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    seat = object_model.get_part("seat")
    front_left_caster = object_model.get_part("front_left_caster")
    front_right_caster = object_model.get_part("front_right_caster")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    rear_bow = object_model.get_part("rear_bow")
    front_bow = object_model.get_part("front_bow")

    front_left_swivel = object_model.get_articulation("front_left_swivel")
    front_right_swivel = object_model.get_articulation("front_right_swivel")
    front_left_spin = object_model.get_articulation("front_left_spin")
    front_right_spin = object_model.get_articulation("front_right_spin")
    rear_left_spin = object_model.get_articulation("rear_left_spin")
    rear_right_spin = object_model.get_articulation("rear_right_spin")
    rear_bow_hinge = object_model.get_articulation("rear_bow_hinge")
    front_bow_hinge = object_model.get_articulation("front_bow_hinge")

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
    ctx.allow_overlap(
        frame,
        front_left_caster,
        elem_a="front_left_socket",
        elem_b="stem",
        reason="The front-left caster stem is intentionally retained inside the steering socket.",
    )
    ctx.allow_overlap(
        frame,
        front_right_caster,
        elem_a="front_right_socket",
        elem_b="stem",
        reason="The front-right caster stem is intentionally retained inside the steering socket.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(seat, frame, name="seat shell mounts onto the stroller frame")
    ctx.expect_contact(front_left_caster, frame, name="left caster stem seats in the frame socket")
    ctx.expect_contact(front_right_caster, frame, name="right caster stem seats in the frame socket")
    ctx.expect_contact(front_left_wheel, front_left_caster, name="left front wheel is carried by the caster fork")
    ctx.expect_contact(front_right_wheel, front_right_caster, name="right front wheel is carried by the caster fork")
    ctx.expect_contact(rear_left_wheel, frame, name="left rear wheel sits on the fixed rear axle")
    ctx.expect_contact(rear_right_wheel, frame, name="right rear wheel sits on the fixed rear axle")
    ctx.expect_contact(rear_bow, seat, name="rear canopy bow is hinged to the seat side plates")
    ctx.expect_contact(front_bow, seat, name="front canopy bow is hinged to the seat side plates")

    def _axis_matches(axis, expected) -> bool:
        return tuple(round(v, 6) for v in axis) == expected

    ctx.check(
        "front casters swivel on vertical pivots",
        front_left_swivel.articulation_type == ArticulationType.CONTINUOUS
        and front_right_swivel.articulation_type == ArticulationType.CONTINUOUS
        and _axis_matches(front_left_swivel.axis, (0.0, 0.0, 1.0))
        and _axis_matches(front_right_swivel.axis, (0.0, 0.0, 1.0)),
        details=f"left={front_left_swivel.axis}, right={front_right_swivel.axis}",
    )
    ctx.check(
        "all stroller wheels spin on lateral axles",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS and _axis_matches(joint.axis, (1.0, 0.0, 0.0))
            for joint in (front_left_spin, front_right_spin, rear_left_spin, rear_right_spin)
        ),
        details=str(
            {
                "front_left_spin": front_left_spin.axis,
                "front_right_spin": front_right_spin.axis,
                "rear_left_spin": rear_left_spin.axis,
                "rear_right_spin": rear_right_spin.axis,
            }
        ),
    )
    ctx.check(
        "canopy bows use lateral revolute hinges with fold-back limits",
        rear_bow_hinge.articulation_type == ArticulationType.REVOLUTE
        and front_bow_hinge.articulation_type == ArticulationType.REVOLUTE
        and _axis_matches(rear_bow_hinge.axis, (1.0, 0.0, 0.0))
        and _axis_matches(front_bow_hinge.axis, (1.0, 0.0, 0.0))
        and rear_bow_hinge.motion_limits is not None
        and front_bow_hinge.motion_limits is not None
        and rear_bow_hinge.motion_limits.lower == 0.0
        and front_bow_hinge.motion_limits.lower == 0.0
        and rear_bow_hinge.motion_limits.upper is not None
        and rear_bow_hinge.motion_limits.upper >= 1.0
        and front_bow_hinge.motion_limits.upper is not None
        and front_bow_hinge.motion_limits.upper >= 1.1,
        details=f"rear={rear_bow_hinge.axis}/{rear_bow_hinge.motion_limits}, front={front_bow_hinge.axis}/{front_bow_hinge.motion_limits}",
    )

    def _elem_center(part, elem: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        minimum, maximum = aabb
        return tuple((minimum[index] + maximum[index]) * 0.5 for index in range(3))

    rear_rest = _elem_center(rear_bow, "panel")
    front_rest = _elem_center(front_bow, "panel")
    with ctx.pose({rear_bow_hinge: 0.90, front_bow_hinge: 1.00}):
        rear_folded = _elem_center(rear_bow, "panel")
        front_folded = _elem_center(front_bow, "panel")

    ctx.check(
        "rear canopy bow folds back over the seat",
        rear_rest is not None
        and rear_folded is not None
        and rear_folded[1] < rear_rest[1] - 0.08,
        details=f"rest={rear_rest}, folded={rear_folded}",
    )
    ctx.check(
        "front canopy bow folds back farther than the deployed canopy nose",
        front_rest is not None
        and front_folded is not None
        and front_folded[1] < front_rest[1] - 0.14,
        details=f"rest={front_rest}, folded={front_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
