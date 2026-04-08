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
    repair_loft,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


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


def _fuselage_section(
    x: float,
    width: float,
    height: float,
    z_center: float,
    *,
    exponent: float = 2.4,
    segments: int = 56,
) -> list[tuple[float, float, float]]:
    return [(x, y, z_center + z) for y, z in superellipse_profile(width, height, exponent=exponent, segments=segments)]


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_skid_assembly(part, side_sign: float, material) -> None:
    rail_y = side_sign * 0.78
    rail_geom = tube_from_spline_points(
        [
            (-1.10, rail_y, -1.20),
            (-0.86, rail_y, -1.28),
            (1.18, rail_y, -1.28),
            (1.42, rail_y, -1.20),
        ],
        radius=0.032,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    part.visual(
        _save_mesh(f"{part.name}_rail", rail_geom),
        material=material,
        name="skid_rail",
    )
    top_front = (-0.46, side_sign * 0.62, -0.62)
    top_rear = (0.56, side_sign * 0.48, -0.53)
    bottom_front = (-0.52, rail_y, -1.25)
    bottom_rear = (0.56, rail_y, -1.25)
    part.visual(
        Box((0.18, 0.10, 0.06)),
        origin=Origin(xyz=top_front),
        material=material,
        name="front_top_saddle",
    )
    part.visual(
        Box((0.18, 0.10, 0.06)),
        origin=Origin(xyz=top_rear),
        material=material,
        name="rear_top_saddle",
    )
    part.visual(
        Box((0.14, 0.10, 0.10)),
        origin=Origin(xyz=bottom_front),
        material=material,
        name="front_foot_mount",
    )
    part.visual(
        Box((0.14, 0.10, 0.10)),
        origin=Origin(xyz=bottom_rear),
        material=material,
        name="rear_foot_mount",
    )
    _add_member(part, top_front, bottom_front, 0.028, material, name="front_strut")
    _add_member(part, top_rear, bottom_rear, 0.028, material, name="rear_strut")


def _add_door_visuals(part, side_sign: float, shell_material, glass_material, hinge_material) -> None:
    thickness = 0.045
    part.visual(
        Cylinder(radius=0.025, length=0.92),
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        material=hinge_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.90, thickness, 0.92)),
        origin=Origin(xyz=(0.45, side_sign * (thickness * 0.5), 0.46)),
        material=shell_material,
        name="door_panel",
    )
    part.visual(
        Box((0.48, 0.016, 0.34)),
        origin=Origin(xyz=(0.40, side_sign * 0.030, 0.68)),
        material=glass_material,
        name="door_window",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="training_helicopter")

    body_white = model.material("body_white", rgba=(0.92, 0.94, 0.95, 1.0))
    stripe_blue = model.material("stripe_blue", rgba=(0.14, 0.29, 0.62, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.16, 0.17, 0.18, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.22, 0.24, 0.25, 1.0))
    cabin_glass = model.material("cabin_glass", rgba=(0.52, 0.72, 0.82, 0.45))

    airframe = model.part("airframe")

    fuselage_geom = repair_loft(
        section_loft(
        [
            _fuselage_section(-1.15, 0.12, 0.22, -0.16, exponent=2.0),
            _fuselage_section(-0.72, 1.08, 0.98, -0.02, exponent=2.2),
            _fuselage_section(-0.10, 1.34, 1.18, 0.08, exponent=2.45),
            _fuselage_section(0.58, 0.96, 0.88, 0.02, exponent=2.35),
            _fuselage_section(1.12, 0.28, 0.28, 0.06, exponent=2.1),
        ]
        )
    )
    airframe.visual(
        _save_mesh("helicopter_fuselage", fuselage_geom),
        material=body_white,
        name="fuselage_shell",
    )
    airframe.visual(
        Cylinder(radius=0.11, length=4.25),
        origin=Origin(xyz=(3.22, 0.0, 0.08), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_white,
        name="tail_boom",
    )
    airframe.visual(
        Box((0.52, 0.18, 0.20)),
        origin=Origin(xyz=(1.10, 0.0, 0.08)),
        material=stripe_blue,
        name="boom_fairing",
    )
    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        _add_member(
            airframe,
            (-0.36, side_sign * 0.42, -0.34),
            (-0.46, side_sign * 0.62, -0.54),
            0.034,
            dark_metal,
            name=f"{side_name}_front_pylon_brace",
        )
        airframe.visual(
            Box((0.20, 0.12, 0.10)),
            origin=Origin(xyz=(-0.46, side_sign * 0.62, -0.54)),
            material=dark_metal,
            name=f"{side_name}_front_pylon",
        )
        _add_member(
            airframe,
            (0.36, side_sign * 0.30, -0.24),
            (0.56, side_sign * 0.48, -0.45),
            0.034,
            dark_metal,
            name=f"{side_name}_rear_pylon_brace",
        )
        airframe.visual(
            Box((0.20, 0.12, 0.10)),
            origin=Origin(xyz=(0.56, side_sign * 0.48, -0.45)),
            material=dark_metal,
            name=f"{side_name}_rear_pylon",
        )
    airframe.visual(
        Cylinder(radius=0.16, length=0.10),
        origin=Origin(xyz=(0.10, 0.0, 0.63)),
        material=dark_metal,
        name="mast_fairing",
    )
    airframe.visual(
        Cylinder(radius=0.045, length=0.32),
        origin=Origin(xyz=(0.10, 0.0, 0.80)),
        material=dark_metal,
        name="mast_shaft",
    )
    airframe.visual(
        Box((0.20, 0.10, 1.02)),
        origin=Origin(xyz=(5.02, 0.0, 0.50)),
        material=body_white,
        name="tail_fin",
    )
    airframe.visual(
        Box((0.28, 0.18, 0.20)),
        origin=Origin(xyz=(5.16, 0.0, 0.24)),
        material=dark_metal,
        name="tail_gearbox_mount",
    )
    _add_member(
        airframe,
        (5.30, 0.0, 0.24),
        (5.36, 0.0, 0.24),
        0.035,
        dark_metal,
        name="tail_rotor_stub_shaft",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((6.8, 1.8, 2.6)),
        mass=420.0,
        origin=Origin(xyz=(1.85, 0.0, -0.06)),
    )

    left_skid = model.part("left_skid")
    _add_skid_assembly(left_skid, 1.0, dark_metal)
    left_skid.inertial = Inertial.from_geometry(
        Box((2.7, 0.5, 1.0)),
        mass=22.0,
        origin=Origin(xyz=(0.15, 0.61, -0.83)),
    )

    right_skid = model.part("right_skid")
    _add_skid_assembly(right_skid, -1.0, dark_metal)
    right_skid.inertial = Inertial.from_geometry(
        Box((2.7, 0.5, 1.0)),
        mass=22.0,
        origin=Origin(xyz=(0.15, -0.61, -0.83)),
    )

    left_door = model.part("left_door")
    _add_door_visuals(left_door, 1.0, body_white, cabin_glass, dark_metal)
    left_door.inertial = Inertial.from_geometry(
        Box((0.90, 0.08, 0.92)),
        mass=18.0,
        origin=Origin(xyz=(0.45, 0.03, 0.46)),
    )

    right_door = model.part("right_door")
    _add_door_visuals(right_door, -1.0, body_white, cabin_glass, dark_metal)
    right_door.inertial = Inertial.from_geometry(
        Box((0.90, 0.08, 0.92)),
        mass=18.0,
        origin=Origin(xyz=(0.45, -0.03, 0.46)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_metal,
        name="hub_disc",
    )
    main_rotor.visual(
        Cylinder(radius=0.05, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark_metal,
        name="rotor_cap",
    )
    for blade_index in range(3):
        main_rotor.visual(
            Box((3.55, 0.11, 0.03)),
            origin=Origin(
                xyz=(1.82, 0.0, 0.07),
                rpy=(0.0, 0.0, blade_index * (2.0 * math.pi / 3.0)),
            ),
            material=rotor_gray,
            name=f"blade_{blade_index}",
        )
    main_rotor.inertial = Inertial.from_geometry(
        Box((7.6, 7.6, 0.20)),
        mass=30.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    tail_rotor = model.part("tail_rotor")
    _add_member(
        tail_rotor,
        (0.0, -0.06, 0.0),
        (0.0, 0.06, 0.0),
        0.055,
        dark_metal,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((0.02, 0.04, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, 0.24)),
        material=rotor_gray,
        name="upper_blade",
    )
    tail_rotor.visual(
        Box((0.02, 0.04, 0.42)),
        origin=Origin(xyz=(0.0, 0.0, -0.24)),
        material=rotor_gray,
        name="lower_blade",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Box((0.95, 0.18, 0.95)),
        mass=6.0,
        origin=Origin(),
    )

    model.articulation(
        "airframe_to_left_skid",
        ArticulationType.FIXED,
        parent=airframe,
        child=left_skid,
        origin=Origin(),
    )
    model.articulation(
        "airframe_to_right_skid",
        ArticulationType.FIXED,
        parent=airframe,
        child=right_skid,
        origin=Origin(),
    )
    model.articulation(
        "airframe_to_left_door",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=left_door,
        origin=Origin(xyz=(-0.72, 0.672, -0.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=0.0,
            upper=1.2,
        ),
    )
    model.articulation(
        "airframe_to_right_door",
        ArticulationType.REVOLUTE,
        parent=airframe,
        child=right_door,
        origin=Origin(xyz=(-0.72, -0.672, -0.36)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=0.0,
            upper=1.2,
        ),
    )
    model.articulation(
        "airframe_to_main_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=main_rotor,
        origin=Origin(xyz=(0.10, 0.0, 0.96)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=40.0),
    )
    model.articulation(
        "airframe_to_tail_rotor",
        ArticulationType.CONTINUOUS,
        parent=airframe,
        child=tail_rotor,
        origin=Origin(xyz=(5.415, 0.0, 0.24)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=55.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    left_skid = object_model.get_part("left_skid")
    right_skid = object_model.get_part("right_skid")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    left_hinge = object_model.get_articulation("airframe_to_left_door")
    right_hinge = object_model.get_articulation("airframe_to_right_door")

    ctx.expect_origin_gap(
        main_rotor,
        airframe,
        axis="z",
        min_gap=0.85,
        max_gap=1.10,
        name="main rotor is mounted above the cabin roof",
    )
    ctx.expect_origin_gap(
        tail_rotor,
        airframe,
        axis="x",
        min_gap=4.8,
        max_gap=5.5,
        name="tail rotor sits at the tail boom tip",
    )
    ctx.expect_gap(
        airframe,
        left_skid,
        axis="z",
        min_gap=0.55,
        max_gap=0.90,
        positive_elem="fuselage_shell",
        negative_elem="skid_rail",
        name="left skid rail hangs well below the fuselage",
    )
    ctx.expect_gap(
        airframe,
        right_skid,
        axis="z",
        min_gap=0.55,
        max_gap=0.90,
        positive_elem="fuselage_shell",
        negative_elem="skid_rail",
        name="right skid rail hangs well below the fuselage",
    )
    ctx.expect_gap(
        left_door,
        airframe,
        axis="y",
        max_gap=0.020,
        max_penetration=0.003,
        positive_elem="door_panel",
        negative_elem="fuselage_shell",
        name="left door sits just proud of the left cabin side",
    )
    ctx.expect_gap(
        airframe,
        right_door,
        axis="y",
        max_gap=0.020,
        max_penetration=0.003,
        positive_elem="fuselage_shell",
        negative_elem="door_panel",
        name="right door sits just proud of the right cabin side",
    )
    ctx.expect_overlap(
        left_door,
        airframe,
        axes="xz",
        min_overlap=0.70,
        elem_a="door_panel",
        elem_b="fuselage_shell",
        name="left door covers the left cabin opening region",
    )
    ctx.expect_overlap(
        right_door,
        airframe,
        axes="xz",
        min_overlap=0.70,
        elem_a="door_panel",
        elem_b="fuselage_shell",
        name="right door covers the right cabin opening region",
    )

    closed_left_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
    closed_right_panel = ctx.part_element_world_aabb(right_door, elem="door_panel")
    with ctx.pose({left_hinge: 1.0, right_hinge: 1.0}):
        open_left_panel = ctx.part_element_world_aabb(left_door, elem="door_panel")
        open_right_panel = ctx.part_element_world_aabb(right_door, elem="door_panel")

    ctx.check(
        "front doors swing outward from their hinges",
        closed_left_panel is not None
        and closed_right_panel is not None
        and open_left_panel is not None
        and open_right_panel is not None
        and open_left_panel[1][1] > closed_left_panel[1][1] + 0.22
        and open_right_panel[0][1] < closed_right_panel[0][1] - 0.22,
        details=(
            f"closed_left={closed_left_panel}, open_left={open_left_panel}, "
            f"closed_right={closed_right_panel}, open_right={open_right_panel}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
