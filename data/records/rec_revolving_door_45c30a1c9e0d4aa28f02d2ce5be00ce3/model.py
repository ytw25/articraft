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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _annular_sector_geometry(
    *,
    inner_radius: float,
    outer_radius: float,
    height: float,
    start_angle: float,
    end_angle: float,
    segments: int = 48,
) -> MeshGeometry:
    geom = MeshGeometry()
    span = end_angle - start_angle
    full_circle = abs(abs(span) - 2.0 * math.pi) < 1e-5

    if full_circle:
        count = segments
        angles = [start_angle + span * i / count for i in range(count)]
    else:
        count = segments + 1
        angles = [start_angle + span * i / segments for i in range(count)]

    outer_bottom: list[int] = []
    outer_top: list[int] = []
    inner_bottom: list[int] = []
    inner_top: list[int] = []

    for angle in angles:
        ca = math.cos(angle)
        sa = math.sin(angle)
        outer_bottom.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, 0.0))
        outer_top.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, height))
        inner_bottom.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, 0.0))
        inner_top.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, height))

    face_count = len(angles) if full_circle else len(angles) - 1
    for i in range(face_count):
        j = (i + 1) % len(angles)
        _add_quad(geom, outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        _add_quad(geom, inner_bottom[i], inner_top[i], inner_top[j], inner_bottom[j])
        _add_quad(geom, inner_top[i], inner_top[j], outer_top[j], outer_top[i])
        _add_quad(geom, inner_bottom[i], outer_bottom[i], outer_bottom[j], inner_bottom[j])

    if not full_circle:
        _add_quad(geom, inner_bottom[0], outer_bottom[0], outer_top[0], inner_top[0])
        _add_quad(
            geom,
            inner_bottom[-1],
            inner_top[-1],
            outer_top[-1],
            outer_bottom[-1],
        )

    return geom


def _ring_mesh(
    name: str,
    *,
    inner_radius: float,
    outer_radius: float,
    height: float,
    start_angle: float,
    end_angle: float,
    segments: int = 56,
):
    return mesh_from_geometry(
        _annular_sector_geometry(
            inner_radius=inner_radius,
            outer_radius=outer_radius,
            height=height,
            start_angle=start_angle,
            end_angle=end_angle,
            segments=segments,
        ),
        name,
    )


def _add_radial_beam(
    part,
    *,
    angle: float,
    inner_radius: float,
    outer_radius: float,
    z_center: float,
    size_y: float,
    size_z: float,
    material,
    name: str,
) -> None:
    length = outer_radius - inner_radius
    mid_radius = 0.5 * (inner_radius + outer_radius)
    part.visual(
        Box((length, size_y, size_z)),
        origin=Origin(
            xyz=(mid_radius * math.cos(angle), mid_radius * math.sin(angle), z_center),
            rpy=(0.0, 0.0, angle),
        ),
        material=material,
        name=name,
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        0.5 * (lower[0] + upper[0]),
        0.5 * (lower[1] + upper[1]),
        0.5 * (lower[2] + upper[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wing_revolving_door")

    bronze = model.material("bronze", rgba=(0.34, 0.29, 0.23, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.19, 0.20, 0.21, 1.0))
    stainless = model.material("stainless", rgba=(0.69, 0.72, 0.74, 1.0))
    glass = model.material("glass", rgba=(0.62, 0.78, 0.86, 0.34))
    canopy_finish = model.material("canopy_finish", rgba=(0.46, 0.47, 0.49, 1.0))

    drum_structure = model.part("drum_structure")

    threshold_ring = _ring_mesh(
        "revolving_door_threshold",
        inner_radius=0.96,
        outer_radius=1.14,
        height=0.03,
        start_angle=0.0,
        end_angle=2.0 * math.pi,
        segments=72,
    )
    drum_structure.visual(threshold_ring, material=dark_metal, name="threshold_ring")
    drum_structure.visual(
        Cylinder(radius=0.095, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=stainless,
        name="lower_pivot_bearing",
    )
    for index, angle_deg in enumerate((30.0, 150.0, 270.0)):
        _add_radial_beam(
            drum_structure,
            angle=math.radians(angle_deg),
            inner_radius=0.095,
            outer_radius=0.97,
            z_center=0.015,
            size_y=0.08,
            size_z=0.03,
            material=dark_metal,
            name=f"floor_spoke_{index}",
        )

    wall_north = _ring_mesh(
        "revolving_door_wall_north",
        inner_radius=1.08,
        outer_radius=1.12,
        height=2.37,
        start_angle=math.radians(40.0),
        end_angle=math.radians(140.0),
        segments=36,
    )
    drum_structure.visual(
        wall_north,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=glass,
        name="wall_north",
    )

    wall_south = _ring_mesh(
        "revolving_door_wall_south",
        inner_radius=1.08,
        outer_radius=1.12,
        height=2.37,
        start_angle=math.radians(220.0),
        end_angle=math.radians(320.0),
        segments=36,
    )
    drum_structure.visual(
        wall_south,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=glass,
        name="wall_south",
    )

    header_north = _ring_mesh(
        "revolving_door_header_north",
        inner_radius=1.02,
        outer_radius=1.14,
        height=0.09,
        start_angle=math.radians(40.0),
        end_angle=math.radians(140.0),
        segments=36,
    )
    drum_structure.visual(
        header_north,
        origin=Origin(xyz=(0.0, 0.0, 2.40)),
        material=bronze,
        name="header_north",
    )

    header_south = _ring_mesh(
        "revolving_door_header_south",
        inner_radius=1.02,
        outer_radius=1.14,
        height=0.09,
        start_angle=math.radians(220.0),
        end_angle=math.radians(320.0),
        segments=36,
    )
    drum_structure.visual(
        header_south,
        origin=Origin(xyz=(0.0, 0.0, 2.40)),
        material=bronze,
        name="header_south",
    )

    canopy_north = _ring_mesh(
        "revolving_door_canopy_north",
        inner_radius=0.62,
        outer_radius=1.14,
        height=0.06,
        start_angle=math.radians(48.0),
        end_angle=math.radians(132.0),
        segments=36,
    )
    drum_structure.visual(
        canopy_north,
        origin=Origin(xyz=(0.0, 0.0, 2.49)),
        material=canopy_finish,
        name="canopy_north",
    )

    canopy_south = _ring_mesh(
        "revolving_door_canopy_south",
        inner_radius=0.62,
        outer_radius=1.14,
        height=0.06,
        start_angle=math.radians(228.0),
        end_angle=math.radians(312.0),
        segments=36,
    )
    drum_structure.visual(
        canopy_south,
        origin=Origin(xyz=(0.0, 0.0, 2.49)),
        material=canopy_finish,
        name="canopy_south",
    )

    for index, angle_deg in enumerate((40.0, 140.0, 220.0, 320.0)):
        angle = math.radians(angle_deg)
        radius = 1.10
        drum_structure.visual(
            Cylinder(radius=0.03, length=2.46),
            origin=Origin(xyz=(radius * math.cos(angle), radius * math.sin(angle), 1.23)),
            material=bronze,
            name=f"jamb_post_{index}",
        )

    for index, angle_deg in enumerate((68.0, 112.0, 248.0, 292.0)):
        _add_radial_beam(
            drum_structure,
            angle=math.radians(angle_deg),
            inner_radius=0.64,
            outer_radius=1.09,
            z_center=2.52,
            size_y=0.09,
            size_z=0.06,
            material=bronze,
            name=f"canopy_arm_{index}",
        )

    drum_structure.inertial = Inertial.from_geometry(
        Box((2.30, 2.30, 2.60)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.30)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.07, length=2.36),
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
        material=stainless,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_metal,
        name="bottom_hub",
    )
    rotor.visual(
        Cylinder(radius=0.16, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 2.39)),
        material=dark_metal,
        name="top_hub",
    )

    wing_height = 2.06
    wing_rail_height = 0.08
    wing_panel_length = 0.93
    wing_panel_center_x = 0.595
    wing_stile_length = 0.13
    wing_stile_center_x = 0.075

    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        yaw = angle
        rotor.visual(
            Box((wing_stile_length, 0.065, 2.20)),
            origin=Origin(xyz=(wing_stile_center_x, 0.0, 1.13), rpy=(0.0, 0.0, yaw)),
            material=bronze,
            name=f"wing_stile_{index}",
        )
        rotor.visual(
            Box((0.96, 0.065, wing_rail_height)),
            origin=Origin(xyz=(0.58, 0.0, 0.10), rpy=(0.0, 0.0, yaw)),
            material=bronze,
            name=f"wing_bottom_rail_{index}",
        )
        rotor.visual(
            Box((0.96, 0.065, wing_rail_height)),
            origin=Origin(xyz=(0.58, 0.0, 2.19), rpy=(0.0, 0.0, yaw)),
            material=bronze,
            name=f"wing_top_rail_{index}",
        )
        rotor.visual(
            Box((wing_panel_length, 0.024, wing_height)),
            origin=Origin(xyz=(wing_panel_center_x, 0.0, 1.13), rpy=(0.0, 0.0, yaw)),
            material=glass,
            name=f"wing_glass_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Box((2.12, 2.12, 2.44)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, 1.22)),
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=drum_structure,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum_structure = object_model.get_part("drum_structure")
    rotor = object_model.get_part("rotor")
    rotor_spin = object_model.get_articulation("rotor_spin")

    ctx.check(
        "rotor uses continuous articulation",
        rotor_spin.joint_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={rotor_spin.joint_type}",
    )
    ctx.check(
        "rotor spins about vertical axis",
        abs(rotor_spin.axis[0]) < 1e-9
        and abs(rotor_spin.axis[1]) < 1e-9
        and abs(rotor_spin.axis[2] - 1.0) < 1e-9,
        details=f"axis={rotor_spin.axis}",
    )
    ctx.expect_origin_distance(
        rotor,
        drum_structure,
        axes="xy",
        max_dist=0.001,
        name="rotor remains centered in drum",
    )
    ctx.expect_gap(
        drum_structure,
        rotor,
        axis="z",
        positive_elem="canopy_north",
        negative_elem="top_hub",
        min_gap=0.06,
        name="top hub clears canopy",
    )

    rest_panel_center = _aabb_center(ctx.part_element_world_aabb(rotor, elem="wing_glass_0"))
    with ctx.pose({rotor_spin: math.pi / 3.0}):
        ctx.expect_origin_distance(
            rotor,
            drum_structure,
            axes="xy",
            max_dist=0.001,
            name="rotor stays centered while turning",
        )
        turned_panel_center = _aabb_center(
            ctx.part_element_world_aabb(rotor, elem="wing_glass_0")
        )

    panel_rotates = (
        rest_panel_center is not None
        and turned_panel_center is not None
        and abs(rest_panel_center[0] - turned_panel_center[0]) > 0.18
        and abs(rest_panel_center[1] - turned_panel_center[1]) > 0.18
    )
    ctx.check(
        "wing panel sweeps around the central post",
        panel_rotates,
        details=f"rest={rest_panel_center}, turned={turned_panel_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
