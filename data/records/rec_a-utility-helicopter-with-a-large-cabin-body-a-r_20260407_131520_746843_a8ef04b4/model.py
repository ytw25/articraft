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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _yz_section(
    x: float,
    width: float,
    height: float,
    *,
    corner: float,
    z_center: float,
    corner_segments: int = 8,
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(
            width,
            height,
            corner,
            corner_segments=corner_segments,
        )
    )


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_helicopter")

    fuselage_green = model.material("fuselage_green", rgba=(0.35, 0.41, 0.28, 1.0))
    darker_green = model.material("darker_green", rgba=(0.28, 0.33, 0.22, 1.0))
    rotor_grey = model.material("rotor_grey", rgba=(0.23, 0.25, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.59, 0.61, 1.0))
    skid_black = model.material("skid_black", rgba=(0.08, 0.08, 0.08, 1.0))
    glass = model.material("glass", rgba=(0.54, 0.69, 0.77, 0.42))
    warning_red = model.material("warning_red", rgba=(0.68, 0.16, 0.12, 1.0))

    airframe = model.part("airframe")

    fuselage_sections = [
        _yz_section(2.85, 0.18, 0.28, corner=0.07, z_center=1.50),
        _yz_section(2.20, 1.10, 1.20, corner=0.22, z_center=1.52),
        _yz_section(1.30, 2.05, 1.78, corner=0.32, z_center=1.58),
        _yz_section(0.10, 2.34, 2.08, corner=0.32, z_center=1.58),
        _yz_section(-1.35, 2.28, 1.96, corner=0.28, z_center=1.56),
        _yz_section(-2.55, 1.62, 1.28, corner=0.24, z_center=1.82),
        _yz_section(-3.15, 0.72, 0.68, corner=0.12, z_center=1.82),
    ]
    airframe.visual(
        _mesh("utility_helicopter_fuselage_shell", section_loft(fuselage_sections)),
        material=fuselage_green,
        name="fuselage_shell",
    )

    doghouse_sections = [
        _yz_section(-0.10, 1.14, 0.68, corner=0.16, z_center=2.45),
        _yz_section(-0.85, 1.42, 0.94, corner=0.18, z_center=2.38),
        _yz_section(-1.70, 1.18, 0.78, corner=0.14, z_center=2.34),
    ]
    airframe.visual(
        _mesh("utility_helicopter_engine_cowling", section_loft(doghouse_sections)),
        material=darker_green,
        name="engine_cowling",
    )

    tail_boom_sections = [
        _yz_section(-3.05, 0.64, 0.58, corner=0.10, z_center=1.82),
        _yz_section(-4.15, 0.52, 0.46, corner=0.08, z_center=1.75),
        _yz_section(-5.55, 0.40, 0.31, corner=0.06, z_center=1.68),
        _yz_section(-7.05, 0.28, 0.22, corner=0.04, z_center=1.62),
    ]
    airframe.visual(
        _mesh("utility_helicopter_tail_boom", section_loft(tail_boom_sections)),
        material=fuselage_green,
        name="tail_boom",
    )

    airframe.visual(
        Box((1.10, 0.16, 1.90)),
        origin=Origin(xyz=(-6.72, 0.0, 2.30)),
        material=fuselage_green,
        name="vertical_fin",
    )
    airframe.visual(
        Box((0.78, 0.10, 0.68)),
        origin=Origin(xyz=(-6.55, 0.0, 1.22)),
        material=fuselage_green,
        name="ventral_fin",
    )
    airframe.visual(
        Box((1.30, 0.30, 0.15)),
        origin=Origin(xyz=(-5.55, -0.33, 1.50)),
        material=fuselage_green,
        name="tailplane",
    )
    airframe.visual(
        Box((0.42, 0.38, 0.42)),
        origin=Origin(xyz=(-7.12, 0.19, 2.08)),
        material=darker_green,
        name="tail_rotor_mount",
    )

    airframe.visual(
        Cylinder(radius=0.20, length=0.26),
        origin=Origin(xyz=(-0.18, 0.0, 2.92)),
        material=darker_green,
        name="mast_fairing",
    )
    airframe.visual(
        Cylinder(radius=0.08, length=0.30),
        origin=Origin(xyz=(-0.18, 0.0, 3.20)),
        material=steel,
        name="mast_stub",
    )

    airframe.visual(
        Box((0.92, 0.03, 0.86)),
        origin=Origin(xyz=(1.88, -0.34, 1.80), rpy=(0.0, -0.34, 0.0)),
        material=glass,
        name="windshield_left",
    )
    airframe.visual(
        Box((0.92, 0.03, 0.86)),
        origin=Origin(xyz=(1.88, 0.34, 1.80), rpy=(0.0, -0.34, 0.0)),
        material=glass,
        name="windshield_right",
    )
    airframe.visual(
        Box((1.08, 0.03, 0.70)),
        origin=Origin(xyz=(0.95, -0.98, 1.82)),
        material=glass,
        name="side_window_left",
    )
    airframe.visual(
        Box((1.08, 0.03, 0.70)),
        origin=Origin(xyz=(0.95, 0.98, 1.82)),
        material=glass,
        name="side_window_right",
    )
    airframe.visual(
        Box((1.80, 0.10, 0.08)),
        origin=Origin(xyz=(0.10, 1.11, 2.26)),
        material=darker_green,
        name="door_upper_track",
    )
    airframe.visual(
        Box((2.60, 0.04, 0.04)),
        origin=Origin(xyz=(-0.26, 1.31, 2.22)),
        material=steel,
        name="door_outer_runner",
    )
    airframe.visual(
        Box((0.18, 0.24, 0.24)),
        origin=Origin(xyz=(1.06, 1.19, 2.13)),
        material=steel,
        name="door_runner_support_front",
    )
    airframe.visual(
        Box((0.22, 0.24, 0.18)),
        origin=Origin(xyz=(-0.92, 1.19, 2.16)),
        material=steel,
        name="door_runner_support_rear",
    )
    airframe.visual(
        Box((1.72, 0.08, 0.08)),
        origin=Origin(xyz=(0.10, 1.09, 0.94)),
        material=darker_green,
        name="door_lower_track",
    )
    airframe.visual(
        Box((0.22, 0.12, 0.16)),
        origin=Origin(xyz=(0.95, 1.07, 2.05)),
        material=darker_green,
        name="door_forward_stop",
    )
    airframe.visual(
        Box((0.28, 0.10, 0.10)),
        origin=Origin(xyz=(-0.81, 1.09, 2.18)),
        material=darker_green,
        name="door_rear_stop",
    )
    airframe.visual(
        Box((0.12, 0.16, 0.82)),
        origin=Origin(xyz=(-0.61, -1.10, 2.40)),
        material=darker_green,
        name="service_panel_hinge_post",
    )

    left_skid = tube_from_spline_points(
        [
            (2.35, -1.18, 0.33),
            (1.95, -1.18, 0.21),
            (1.05, -1.18, 0.15),
            (-1.55, -1.18, 0.15),
            (-2.35, -1.18, 0.20),
            (-2.90, -1.18, 0.31),
        ],
        radius=0.045,
        samples_per_segment=16,
        radial_segments=18,
        up_hint=(0.0, 0.0, 1.0),
    )
    right_skid = tube_from_spline_points(
        [
            (2.35, 1.18, 0.33),
            (1.95, 1.18, 0.21),
            (1.05, 1.18, 0.15),
            (-1.55, 1.18, 0.15),
            (-2.35, 1.18, 0.20),
            (-2.90, 1.18, 0.31),
        ],
        radius=0.045,
        samples_per_segment=16,
        radial_segments=18,
        up_hint=(0.0, 0.0, 1.0),
    )
    airframe.visual(_mesh("utility_helicopter_left_skid", left_skid), material=skid_black, name="left_skid")
    airframe.visual(_mesh("utility_helicopter_right_skid", right_skid), material=skid_black, name="right_skid")

    for a, b in [
        ((1.18, -0.62, 0.82), (1.05, -1.12, 0.18)),
        ((0.10, -0.78, 0.76), (0.20, -1.12, 0.16)),
        ((-1.10, -0.78, 0.84), (-1.20, -1.12, 0.16)),
        ((-2.20, -0.58, 0.98), (-2.30, -1.12, 0.20)),
        ((1.18, 0.62, 0.82), (1.05, 1.12, 0.18)),
        ((0.10, 0.78, 0.76), (0.20, 1.12, 0.16)),
        ((-1.10, 0.78, 0.84), (-1.20, 1.12, 0.16)),
        ((-2.20, 0.58, 0.98), (-2.30, 1.12, 0.20)),
    ]:
        _add_member(airframe, a, b, 0.038, steel)

    _add_member(airframe, (0.16, -0.92, 0.63), (0.16, 0.92, 0.63), 0.032, steel, name="forward_cross_tube")

    airframe.inertial = Inertial.from_geometry(
        Box((13.8, 3.2, 3.5)),
        mass=4200.0,
        origin=Origin(xyz=(-2.00, 0.0, 1.75)),
    )

    cargo_door = model.part("cargo_door")
    cargo_door.visual(
        Box((1.48, 0.06, 1.54)),
        material=fuselage_green,
        name="door_panel",
    )
    cargo_door.visual(
        Box((0.78, 0.025, 0.58)),
        origin=Origin(xyz=(0.12, 0.0, 0.26)),
        material=glass,
        name="door_window",
    )
    cargo_door.visual(
        Box((0.16, 0.03, 0.08)),
        origin=Origin(xyz=(0.46, 0.04, -0.06)),
        material=rotor_grey,
        name="door_handle",
    )
    cargo_door.visual(
        Box((0.18, 0.06, 0.16)),
        origin=Origin(xyz=(0.48, 0.06, 0.64)),
        material=rotor_grey,
        name="front_hanger",
    )
    cargo_door.visual(
        Box((0.18, 0.06, 0.16)),
        origin=Origin(xyz=(-0.48, 0.06, 0.64)),
        material=rotor_grey,
        name="rear_hanger",
    )
    cargo_door.inertial = Inertial.from_geometry(
        Box((1.50, 0.10, 1.58)),
        mass=85.0,
        origin=Origin(),
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.98, 0.06, 0.74)),
        origin=Origin(xyz=(-0.49, -0.03, 0.0)),
        material=darker_green,
        name="panel_shell",
    )
    for offset_z in (-0.18, 0.0, 0.18):
        service_panel.visual(
            Box((0.54, 0.02, 0.05)),
            origin=Origin(xyz=(-0.54, -0.055, offset_z)),
            material=rotor_grey,
            name=f"louver_{int((offset_z + 0.18) * 100):02d}",
        )
    service_panel.visual(
        Box((0.10, 0.025, 0.08)),
        origin=Origin(xyz=(-0.77, -0.055, -0.05)),
        material=warning_red,
        name="panel_handle",
    )
    service_panel.inertial = Inertial.from_geometry(
        Box((1.00, 0.10, 0.78)),
        mass=28.0,
        origin=Origin(xyz=(-0.50, -0.04, 0.0)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.12, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=rotor_grey,
        name="hub_core",
    )
    main_rotor.visual(
        Box((0.64, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=rotor_grey,
        name="hub_fairing",
    )
    for x_sign in (-1.0, 1.0):
        main_rotor.visual(
            Box((3.20, 0.30, 0.05)),
            origin=Origin(xyz=(x_sign * 1.75, 0.0, 0.08)),
            material=rotor_grey,
            name=f"blade_x_{'neg' if x_sign < 0 else 'pos'}",
        )
    for y_sign in (-1.0, 1.0):
        main_rotor.visual(
            Box((0.30, 3.20, 0.05)),
            origin=Origin(xyz=(0.0, y_sign * 1.75, 0.08)),
            material=rotor_grey,
            name=f"blade_y_{'neg' if y_sign < 0 else 'pos'}",
        )
    main_rotor.inertial = Inertial.from_geometry(
        Box((7.0, 7.0, 0.30)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    tail_rotor = model.part("tail_rotor")
    _add_member(tail_rotor, (0.0, 0.0, 0.0), (0.0, 0.14, 0.0), 0.045, rotor_grey, name="tail_shaft")
    tail_rotor.visual(
        Box((0.24, 0.16, 0.18)),
        origin=Origin(xyz=(0.0, 0.20, 0.0)),
        material=rotor_grey,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((1.08, 0.05, 0.13)),
        origin=Origin(xyz=(0.0, 0.20, 0.0)),
        material=rotor_grey,
        name="tail_blade_span_x",
    )
    tail_rotor.visual(
        Box((0.16, 0.05, 0.86)),
        origin=Origin(xyz=(0.0, 0.20, 0.0)),
        material=rotor_grey,
        name="tail_blade_span_z",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Box((1.20, 0.28, 1.00)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.18, 0.0)),
    )

    model.articulation(
        "airframe_to_cargo_door",
        ArticulationType.PRISMATIC,
        parent=airframe,
        child=cargo_door,
        origin=Origin(xyz=(0.20, 1.25, 1.48)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.80, lower=0.0, upper=1.22),
    )
    model.articulation(
        "airframe_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=service_panel,
        origin=Origin(xyz=(-0.55, -1.18, 2.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.20,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "airframe_to_main_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(-0.18, 0.0, 3.35)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=35.0),
    )
    model.articulation(
        "airframe_to_tail_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(-7.12, 0.38, 2.08)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=70.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def elem_center(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    airframe = object_model.get_part("airframe")
    cargo_door = object_model.get_part("cargo_door")
    service_panel = object_model.get_part("service_panel")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")

    door_slide = object_model.get_articulation("airframe_to_cargo_door")
    panel_hinge = object_model.get_articulation("airframe_to_service_panel")
    main_rotor_spin = object_model.get_articulation("airframe_to_main_rotor")
    tail_rotor_spin = object_model.get_articulation("airframe_to_tail_rotor")

    ctx.expect_overlap(
        cargo_door,
        airframe,
        axes="xz",
        elem_a="door_panel",
        elem_b="fuselage_shell",
        min_overlap=1.0,
        name="cargo door covers the cabin side opening",
    )

    door_rest = ctx.part_world_position(cargo_door)
    door_open = None
    with ctx.pose({door_slide: door_slide.motion_limits.upper}):
        door_open = ctx.part_world_position(cargo_door)

    ctx.check(
        "cargo door slides aft along the cabin side",
        door_rest is not None
        and door_open is not None
        and door_open[0] < door_rest[0] - 0.9
        and abs(door_open[1] - door_rest[1]) < 0.03
        and abs(door_open[2] - door_rest[2]) < 0.03,
        details=f"closed={door_rest}, open={door_open}",
    )

    panel_rest = elem_center(service_panel, "panel_handle")
    panel_open = None
    with ctx.pose({panel_hinge: panel_hinge.motion_limits.upper}):
        panel_open = elem_center(service_panel, "panel_handle")

    ctx.check(
        "service panel swings outward on its side hinge",
        panel_rest is not None
        and panel_open is not None
        and panel_open[1] < panel_rest[1] - 0.18
        and panel_open[0] > panel_rest[0] + 0.20,
        details=f"closed={panel_rest}, open={panel_open}",
    )

    ctx.check(
        "main rotor articulation is vertical continuous spin",
        main_rotor_spin.motion_limits is not None
        and tuple(main_rotor_spin.axis) == (0.0, 0.0, 1.0)
        and main_rotor_spin.motion_limits.lower is None
        and main_rotor_spin.motion_limits.upper is None
        and ctx.part_world_position(main_rotor) is not None,
        details=f"axis={main_rotor_spin.axis}, limits={main_rotor_spin.motion_limits}",
    )

    ctx.check(
        "tail rotor articulation is lateral continuous spin",
        tail_rotor_spin.motion_limits is not None
        and tuple(tail_rotor_spin.axis) == (0.0, 1.0, 0.0)
        and tail_rotor_spin.motion_limits.lower is None
        and tail_rotor_spin.motion_limits.upper is None
        and ctx.part_world_position(tail_rotor) is not None,
        details=f"axis={tail_rotor_spin.axis}, limits={tail_rotor_spin.motion_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
