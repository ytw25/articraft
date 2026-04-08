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
    CylinderGeometry,
    Inertial,
    MeshGeometry,
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


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _yz_superellipse_section(
    x_pos: float,
    width: float,
    height: float,
    z_center: float,
    *,
    exponent: float = 2.4,
    segments: int = 28,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, z_center + z)
        for y, z in superellipse_profile(width, height, exponent=exponent, segments=segments)
    ]


def _rotor_blade_section(
    x_pos: float,
    chord: float,
    thickness: float,
    z_offset: float,
    *,
    chord_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    half_chord = chord * 0.5
    half_thickness = thickness * 0.5
    return [
        (x_pos, chord_shift - 0.50 * half_chord, z_offset + 0.05 * half_thickness),
        (x_pos, chord_shift - 0.26 * half_chord, z_offset + 0.92 * half_thickness),
        (x_pos, chord_shift + 0.05 * half_chord, z_offset + 1.00 * half_thickness),
        (x_pos, chord_shift + 0.45 * half_chord, z_offset + 0.24 * half_thickness),
        (x_pos, chord_shift + 0.50 * half_chord, z_offset + 0.00 * half_thickness),
        (x_pos, chord_shift + 0.18 * half_chord, z_offset - 0.68 * half_thickness),
        (x_pos, chord_shift - 0.16 * half_chord, z_offset - 0.74 * half_thickness),
        (x_pos, chord_shift - 0.45 * half_chord, z_offset - 0.22 * half_thickness),
    ]


def _build_rotor_blade(
    *,
    length: float,
    root_chord: float,
    tip_chord: float,
    root_thickness: float,
    tip_thickness: float,
    droop_tip: float,
) -> MeshGeometry:
    sections = [
        _rotor_blade_section(0.00, root_chord, root_thickness, 0.000, chord_shift=0.000),
        _rotor_blade_section(0.24 * length, 0.90 * root_chord, 0.86 * root_thickness, -0.008, chord_shift=0.006),
        _rotor_blade_section(0.58 * length, 0.62 * (root_chord + tip_chord), 0.56 * (root_thickness + tip_thickness), -0.040, chord_shift=0.014),
        _rotor_blade_section(length, tip_chord, tip_thickness, droop_tip, chord_shift=0.020),
    ]
    return repair_loft(section_loft(sections))


def _build_skid_gear() -> MeshGeometry:
    gear = MeshGeometry()

    left_skid_points = [
        (-1.78, 0.95, 0.10),
        (-1.40, 0.95, 0.045),
        (-1.20, 0.95, 0.045),
        (0.55, 0.95, 0.045),
        (0.82, 0.95, 0.045),
        (1.12, 0.95, 0.10),
    ]
    right_skid_points = [(x, -y, z) for x, y, z in left_skid_points]

    members = [
        tube_from_spline_points(
            left_skid_points,
            radius=0.040,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        tube_from_spline_points(
            right_skid_points,
            radius=0.040,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
        tube_from_spline_points(
            [(0.46, -0.72, 0.70), (0.46, 0.72, 0.70)],
            radius=0.030,
            samples_per_segment=2,
            radial_segments=16,
            cap_ends=True,
        ),
        tube_from_spline_points(
            [(-1.02, -0.70, 0.66), (-1.02, 0.70, 0.66)],
            radius=0.030,
            samples_per_segment=2,
            radial_segments=16,
            cap_ends=True,
        ),
        tube_from_spline_points(
            [(0.46, 0.72, 0.70), (0.55, 0.95, 0.045)],
            radius=0.027,
            samples_per_segment=2,
            radial_segments=14,
            cap_ends=True,
        ),
        tube_from_spline_points(
            [(0.46, -0.72, 0.70), (0.55, -0.95, 0.045)],
            radius=0.027,
            samples_per_segment=2,
            radial_segments=14,
            cap_ends=True,
        ),
        tube_from_spline_points(
            [(-1.02, 0.70, 0.66), (-1.20, 0.95, 0.045)],
            radius=0.027,
            samples_per_segment=2,
            radial_segments=14,
            cap_ends=True,
        ),
        tube_from_spline_points(
            [(-1.02, -0.70, 0.66), (-1.20, -0.95, 0.045)],
            radius=0.027,
            samples_per_segment=2,
            radial_segments=14,
            cap_ends=True,
        ),
    ]

    for member in members:
        gear.merge(member)
    return gear


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_helicopter")

    fuselage_paint = model.material("fuselage_paint", rgba=(0.88, 0.90, 0.93, 1.0))
    accent_red = model.material("accent_red", rgba=(0.66, 0.10, 0.10, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.16, 0.22, 0.28, 0.55))
    skid_gray = model.material("skid_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    hub_gray = model.material("hub_gray", rgba=(0.36, 0.38, 0.41, 1.0))

    fuselage = model.part("fuselage")

    fuselage_sections = [
        _yz_superellipse_section(1.80, 0.10, 0.14, 1.18, exponent=2.0),
        _yz_superellipse_section(1.18, 0.68, 0.92, 1.30, exponent=2.2),
        _yz_superellipse_section(0.34, 1.30, 1.52, 1.38, exponent=2.35),
        _yz_superellipse_section(-0.68, 1.46, 1.60, 1.40, exponent=2.55),
        _yz_superellipse_section(-1.62, 1.14, 1.28, 1.37, exponent=2.45),
        _yz_superellipse_section(-2.32, 0.56, 0.62, 1.27, exponent=2.2),
    ]
    fuselage.visual(
        _save_mesh("helicopter_fuselage_shell", repair_loft(section_loft(fuselage_sections))),
        material=fuselage_paint,
        name="fuselage_shell",
    )
    fuselage.visual(
        Box((2.00, 0.42, 0.22)),
        origin=Origin(xyz=(-0.28, 0.0, 0.76)),
        material=fuselage_paint,
        name="belly_keel",
    )
    fuselage.visual(
        Box((1.10, 0.70, 0.34)),
        origin=Origin(xyz=(-0.92, 0.0, 1.96)),
        material=accent_red,
        name="engine_cowling",
    )
    fuselage.visual(
        Box((0.54, 0.44, 0.12)),
        origin=Origin(xyz=(-0.28, 0.0, 1.93)),
        material=accent_red,
        name="roof_fairing",
    )
    fuselage.visual(
        Cylinder(radius=0.16, length=0.26),
        origin=Origin(xyz=(-0.28, 0.0, 2.10)),
        material=accent_red,
        name="mast_pylon",
    )
    fuselage.visual(
        Cylinder(radius=0.14, length=3.86),
        origin=Origin(xyz=(-4.16, 0.0, 1.26), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fuselage_paint,
        name="tail_boom",
    )
    fuselage.visual(
        Box((0.90, 0.12, 1.14)),
        origin=Origin(xyz=(-5.92, 0.0, 1.58)),
        material=accent_red,
        name="tail_fin",
    )
    fuselage.visual(
        Box((0.78, 1.34, 0.08)),
        origin=Origin(xyz=(-5.18, 0.0, 1.04)),
        material=accent_red,
        name="horizontal_stab",
    )
    fuselage.visual(
        Box((0.22, 0.46, 0.24)),
        origin=Origin(xyz=(-6.26, 0.23, 1.32)),
        material=hub_gray,
        name="tail_gearbox",
    )
    fuselage.visual(
        _save_mesh("helicopter_skid_gear", _build_skid_gear()),
        material=skid_gray,
        name="landing_skids",
    )

    fuselage.visual(
        Box((0.78, 0.02, 0.62)),
        origin=Origin(xyz=(0.98, 0.26, 1.56), rpy=(0.0, -0.52, 0.18)),
        material=tinted_glass,
        name="windshield_left",
    )
    fuselage.visual(
        Box((0.78, 0.02, 0.62)),
        origin=Origin(xyz=(0.98, -0.26, 1.56), rpy=(0.0, -0.52, -0.18)),
        material=tinted_glass,
        name="windshield_right",
    )
    fuselage.visual(
        Box((1.20, 0.02, 0.56)),
        origin=Origin(xyz=(0.10, 0.66, 1.50)),
        material=tinted_glass,
        name="side_window_left",
    )
    fuselage.visual(
        Box((1.20, 0.02, 0.56)),
        origin=Origin(xyz=(0.10, -0.66, 1.50)),
        material=tinted_glass,
        name="side_window_right",
    )
    fuselage.inertial = Inertial.from_geometry(
        Box((8.0, 2.4, 2.6)),
        mass=720.0,
        origin=Origin(xyz=(-2.0, 0.0, 1.30)),
    )

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.045, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=hub_gray,
        name="mast",
    )
    main_rotor.visual(
        Cylinder(radius=0.11, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=hub_gray,
        name="hub_shell",
    )
    main_rotor.visual(
        Cylinder(radius=0.032, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.32), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_gray,
        name="spindle_x",
    )
    main_rotor.visual(
        Cylinder(radius=0.032, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, 0.32), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="spindle_y",
    )

    main_blade_mesh = _save_mesh(
        "helicopter_main_blade",
        _build_rotor_blade(
            length=3.55,
            root_chord=0.26,
            tip_chord=0.12,
            root_thickness=0.040,
            tip_thickness=0.012,
            droop_tip=-0.12,
        ),
    )
    for name, origin in [
        ("blade_0", Origin(xyz=(0.16, 0.0, 0.34))),
        ("blade_1", Origin(xyz=(0.0, 0.16, 0.34), rpy=(0.0, 0.0, math.pi / 2.0))),
        ("blade_2", Origin(xyz=(-0.16, 0.0, 0.34), rpy=(0.0, 0.0, math.pi))),
        ("blade_3", Origin(xyz=(0.0, -0.16, 0.34), rpy=(0.0, 0.0, -math.pi / 2.0))),
    ]:
        main_rotor.visual(main_blade_mesh, origin=origin, material=rotor_gray, name=name)
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=3.75, length=0.38),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.050, length=0.12),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="hub_shaft",
    )
    tail_rotor.visual(
        Cylinder(radius=0.080, length=0.05),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_gray,
        name="hub_shell",
    )
    tail_blade_mesh = _save_mesh(
        "helicopter_tail_blade",
        _build_rotor_blade(
            length=0.62,
            root_chord=0.12,
            tip_chord=0.06,
            root_thickness=0.020,
            tip_thickness=0.008,
            droop_tip=0.0,
        ),
    )
    tail_rotor.visual(
        tail_blade_mesh,
        origin=Origin(xyz=(0.08, 0.0, 0.0)),
        material=rotor_gray,
        name="tail_blade_0",
    )
    tail_rotor.visual(
        tail_blade_mesh,
        origin=Origin(xyz=(-0.08, 0.0, 0.0), rpy=(0.0, math.pi, 0.0)),
        material=rotor_gray,
        name="tail_blade_1",
    )
    tail_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.72, length=0.16),
        mass=10.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(-0.28, 0.0, 2.23)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=45.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(-6.28, 0.52, 1.32)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=90.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fuselage = object_model.get_part("fuselage")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")
    main_spin = object_model.get_articulation("main_rotor_spin")
    tail_spin = object_model.get_articulation("tail_rotor_spin")

    ctx.check(
        "main rotor uses continuous vertical mast axis",
        main_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(main_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_spin.articulation_type}, axis={main_spin.axis}",
    )
    ctx.check(
        "tail rotor uses continuous transverse axis",
        tail_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(tail_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={tail_spin.articulation_type}, axis={tail_spin.axis}",
    )
    ctx.expect_gap(
        main_rotor,
        fuselage,
        axis="z",
        positive_elem="mast",
        negative_elem="mast_pylon",
        min_gap=0.0,
        max_gap=0.01,
        name="main rotor mast seats on pylon",
    )
    ctx.expect_gap(
        main_rotor,
        fuselage,
        axis="z",
        positive_elem="blade_0",
        negative_elem="engine_cowling",
        min_gap=0.14,
        name="main rotor blades clear engine cowling",
    )
    ctx.expect_gap(
        tail_rotor,
        fuselage,
        axis="y",
        positive_elem="hub_shaft",
        negative_elem="tail_fin",
        min_gap=0.28,
        name="tail rotor sits outboard of tail fin",
    )
    ctx.expect_gap(
        tail_rotor,
        fuselage,
        axis="y",
        positive_elem="hub_shaft",
        negative_elem="tail_gearbox",
        min_gap=0.0,
        max_gap=0.02,
        name="tail rotor hub is mounted directly off gearbox housing",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
