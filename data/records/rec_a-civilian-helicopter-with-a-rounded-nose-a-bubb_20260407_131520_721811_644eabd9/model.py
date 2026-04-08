from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="civilian_helicopter")

    body_white = model.material("body_white", rgba=(0.94, 0.95, 0.96, 1.0))
    canopy_tint = model.material("canopy_tint", rgba=(0.38, 0.55, 0.68, 0.42))
    accent_blue = model.material("accent_blue", rgba=(0.12, 0.32, 0.62, 1.0))
    blade_black = model.material("blade_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_mech = model.material("dark_mech", rgba=(0.22, 0.23, 0.25, 1.0))
    skid_grey = model.material("skid_grey", rgba=(0.46, 0.48, 0.50, 1.0))
    metal = model.material("metal", rgba=(0.74, 0.75, 0.77, 1.0))

    fuselage = model.part("fuselage")
    fuselage.inertial = Inertial.from_geometry(
        Box((2.1, 6.8, 2.2)),
        mass=820.0,
        origin=Origin(xyz=(0.0, -1.15, 1.12)),
    )

    body_shell = superellipse_side_loft(
        [
            (-1.45, 0.82, 1.28, 0.36),
            (-1.05, 0.66, 1.40, 0.92),
            (-0.55, 0.50, 1.56, 1.28),
            (0.05, 0.40, 1.64, 1.48),
            (0.72, 0.48, 1.54, 1.34),
            (1.24, 0.62, 1.30, 0.98),
            (1.62, 0.82, 1.16, 0.42),
        ],
        exponents=2.4,
        segments=60,
    )
    fuselage.visual(_mesh("fuselage_body_shell", body_shell), material=body_white, name="body_shell")

    canopy_shell = superellipse_side_loft(
        [
            (-0.46, 0.82, 1.72, 1.08),
            (0.08, 0.78, 1.90, 1.42),
            (0.62, 0.78, 1.86, 1.30),
            (1.12, 0.82, 1.58, 0.92),
        ],
        exponents=2.0,
        segments=56,
    )
    fuselage.visual(_mesh("fuselage_canopy_shell", canopy_shell), material=canopy_tint, name="canopy_shell")

    fuselage.visual(
        Box((0.50, 1.34, 0.16)),
        origin=Origin(xyz=(0.0, -0.10, 0.64)),
        material=body_white,
        name="belly_keel",
    )
    fuselage.visual(
        Box((0.22, 0.92, 0.68)),
        origin=Origin(xyz=(0.65, -0.46, 1.08)),
        material=body_white,
        name="baggage_pod",
    )
    fuselage.visual(
        Box((0.16, 2.10, 0.08)),
        origin=Origin(xyz=(0.0, -0.22, 1.08)),
        material=accent_blue,
        name="accent_stripe",
    )
    fuselage.visual(
        Box((0.54, 0.76, 0.26)),
        origin=Origin(xyz=(0.0, -0.54, 1.76)),
        material=body_white,
        name="engine_housing",
    )
    fuselage.visual(
        Cylinder(radius=0.11, length=0.24),
        origin=Origin(xyz=(0.0, -0.04, 1.88)),
        material=dark_mech,
        name="mast_pylon",
    )

    tail_boom = tube_from_spline_points(
        [
            (0.0, -1.38, 1.10),
            (0.0, -2.25, 1.14),
            (0.0, -3.20, 1.19),
            (0.0, -4.06, 1.24),
        ],
        radius=0.13,
        samples_per_segment=16,
        radial_segments=18,
    )
    fuselage.visual(_mesh("tail_boom", tail_boom), material=body_white, name="tail_boom")
    fuselage.visual(
        Box((0.06, 0.74, 0.76)),
        origin=Origin(xyz=(0.0, -3.96, 1.62)),
        material=body_white,
        name="vertical_fin",
    )
    fuselage.visual(
        Box((0.82, 0.22, 0.05)),
        origin=Origin(xyz=(0.0, -3.58, 1.36)),
        material=body_white,
        name="horizontal_stabilizer",
    )
    fuselage.visual(
        Box((0.20, 0.18, 0.16)),
        origin=Origin(xyz=(0.10, -4.22, 1.34)),
        material=dark_mech,
        name="tail_gearbox",
    )
    fuselage.visual(
        Box((0.16, 0.12, 0.16)),
        origin=Origin(xyz=(0.0, -4.29, 1.24)),
        material=body_white,
        name="tail_end_cap",
    )

    left_skid = tube_from_spline_points(
        [
            (0.74, -2.22, 0.24),
            (0.82, -1.78, 0.10),
            (0.86, -0.92, 0.09),
            (0.86, 0.42, 0.10),
            (0.78, 0.98, 0.25),
        ],
        radius=0.035,
        samples_per_segment=16,
        radial_segments=18,
    )
    right_skid = tube_from_spline_points(
        [
            (-0.74, -2.22, 0.24),
            (-0.82, -1.78, 0.10),
            (-0.86, -0.92, 0.09),
            (-0.86, 0.42, 0.10),
            (-0.78, 0.98, 0.25),
        ],
        radius=0.035,
        samples_per_segment=16,
        radial_segments=18,
    )
    front_cross_tube = tube_from_spline_points(
        [
            (-0.86, 0.42, 0.10),
            (-0.58, 0.44, 0.34),
            (0.0, 0.40, 0.66),
            (0.58, 0.44, 0.34),
            (0.86, 0.42, 0.10),
        ],
        radius=0.028,
        samples_per_segment=16,
        radial_segments=16,
    )
    rear_cross_tube = tube_from_spline_points(
        [
            (-0.82, -0.92, 0.09),
            (-0.50, -0.90, 0.33),
            (0.0, -0.88, 0.64),
            (0.50, -0.90, 0.33),
            (0.82, -0.92, 0.09),
        ],
        radius=0.028,
        samples_per_segment=16,
        radial_segments=16,
    )
    fuselage.visual(_mesh("left_skid", left_skid), material=skid_grey, name="left_skid")
    fuselage.visual(_mesh("right_skid", right_skid), material=skid_grey, name="right_skid")
    fuselage.visual(_mesh("front_cross_tube", front_cross_tube), material=skid_grey, name="front_cross_tube")
    fuselage.visual(_mesh("rear_cross_tube", rear_cross_tube), material=skid_grey, name="rear_cross_tube")

    for z in (0.86, 1.14, 1.42):
        fuselage.visual(
            Box((0.04, 0.02, 0.10)),
            origin=Origin(xyz=(0.75, -0.16, z)),
            material=dark_mech,
        )

    baggage_door = model.part("baggage_door")
    baggage_door.inertial = Inertial.from_geometry(
        Box((0.08, 0.64, 0.76)),
        mass=12.0,
        origin=Origin(xyz=(0.04, -0.32, 0.0)),
    )
    door_panel_geom = ExtrudeGeometry(
        rounded_rect_profile(0.72, 0.62, 0.08),
        0.024,
        center=True,
    ).rotate_y(pi / 2.0)
    baggage_door.visual(
        _mesh("baggage_door_panel", door_panel_geom),
        origin=Origin(xyz=(0.018, -0.31, 0.0)),
        material=body_white,
        name="door_panel",
    )
    baggage_door.visual(
        Box((0.01, 0.46, 0.56)),
        origin=Origin(xyz=(0.022, -0.31, 0.0)),
        material=accent_blue,
        name="door_accent",
    )
    for z in (-0.24, 0.0, 0.24):
        baggage_door.visual(
            Cylinder(radius=0.015, length=0.12),
            origin=Origin(xyz=(0.0, -0.01, z)),
            material=dark_mech,
        )
    baggage_door.visual(
        Cylinder(radius=0.012, length=0.05),
        origin=Origin(xyz=(0.038, -0.54, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="door_handle",
    )

    main_rotor = model.part("main_rotor")
    main_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.30),
        mass=45.0,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )
    main_rotor.visual(
        Cylinder(radius=0.032, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=metal,
        name="rotor_mast",
    )
    main_rotor.visual(
        Cylinder(radius=0.09, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=dark_mech,
        name="rotor_hub",
    )
    main_rotor.visual(
        Box((0.38, 0.12, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=dark_mech,
        name="hub_bar",
    )
    main_rotor.visual(
        Box((3.00, 0.14, 0.028)),
        origin=Origin(xyz=(1.68, 0.0, 0.215)),
        material=blade_black,
        name="right_main_blade",
    )
    main_rotor.visual(
        Box((3.00, 0.14, 0.028)),
        origin=Origin(xyz=(-1.68, 0.0, 0.215)),
        material=blade_black,
        name="left_main_blade",
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=0.18),
        mass=6.0,
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    tail_rotor.visual(
        Cylinder(radius=0.020, length=0.14),
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="tail_rotor_shaft",
    )
    tail_rotor.visual(
        Cylinder(radius=0.050, length=0.06),
        origin=Origin(xyz=(0.09, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_mech,
        name="tail_rotor_hub",
    )
    tail_rotor.visual(
        Box((0.04, 0.36, 0.07)),
        origin=Origin(xyz=(0.09, 0.19, 0.0)),
        material=blade_black,
        name="upper_tail_blade",
    )
    tail_rotor.visual(
        Box((0.04, 0.36, 0.07)),
        origin=Origin(xyz=(0.09, -0.19, 0.0)),
        material=blade_black,
        name="lower_tail_blade",
    )

    model.articulation(
        "baggage_door_hinge",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=baggage_door,
        origin=Origin(xyz=(0.775, -0.18, 1.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=1.30),
    )
    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=main_rotor,
        origin=Origin(xyz=(0.0, -0.04, 2.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=40.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=fuselage,
        child=tail_rotor,
        origin=Origin(xyz=(0.20, -4.28, 1.34)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=60.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    baggage_door = object_model.get_part("baggage_door")
    main_rotor = object_model.get_part("main_rotor")
    door_hinge = object_model.get_articulation("baggage_door_hinge")
    main_rotor_spin = object_model.get_articulation("main_rotor_spin")
    tail_rotor_spin = object_model.get_articulation("tail_rotor_spin")

    ctx.check(
        "main rotor articulation is continuous about vertical mast",
        main_rotor_spin.articulation_type == ArticulationType.CONTINUOUS and main_rotor_spin.axis == (0.0, 0.0, 1.0),
        details=f"type={main_rotor_spin.articulation_type}, axis={main_rotor_spin.axis}",
    )
    ctx.check(
        "tail rotor articulation is continuous about transverse x axis",
        tail_rotor_spin.articulation_type == ArticulationType.CONTINUOUS and tail_rotor_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={tail_rotor_spin.articulation_type}, axis={tail_rotor_spin.axis}",
    )
    ctx.check(
        "baggage door hinges on a vertical side axis",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and door_hinge.axis == (0.0, 0.0, 1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper >= 1.0,
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            baggage_door,
            fuselage,
            axis="x",
            positive_elem="door_panel",
            negative_elem="baggage_pod",
            max_gap=0.04,
            max_penetration=0.0,
            name="closed baggage door sits flush on pod side",
        )
        ctx.expect_overlap(
            baggage_door,
            fuselage,
            axes="yz",
            elem_a="door_panel",
            elem_b="baggage_pod",
            min_overlap=0.40,
            name="closed baggage door covers baggage pod opening area",
        )
        ctx.expect_gap(
            main_rotor,
            fuselage,
            axis="z",
            positive_elem="rotor_mast",
            negative_elem="mast_pylon",
            max_gap=0.01,
            max_penetration=1e-5,
            name="main rotor mast seats on roof pylon",
        )

        closed_aabb = ctx.part_element_world_aabb(baggage_door, elem="door_panel")

    with ctx.pose({door_hinge: 1.10}):
        open_aabb = ctx.part_element_world_aabb(baggage_door, elem="door_panel")

    ctx.check(
        "baggage door opens outward from fuselage side",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][0] > closed_aabb[1][0] + 0.22,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
