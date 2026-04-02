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
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_face,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _yz_rounded_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_val, z_center + z_val)
        for y_val, z_val in rounded_rect_profile(
            width,
            height,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _yz_superellipse_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    z_center: float,
    exponent: float = 2.6,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_val, z_center + z_val)
        for y_val, z_val in superellipse_profile(
            width,
            height,
            exponent=exponent,
            segments=segments,
        )
    ]


def _build_bowl_shell() -> MeshGeometry:
    outer_sections = [
        _yz_rounded_section(0.014, width=0.34, height=0.11, radius=0.055, z_center=-0.010),
        _yz_rounded_section(0.120, width=0.35, height=0.18, radius=0.066, z_center=-0.045),
        _yz_rounded_section(0.250, width=0.36, height=0.22, radius=0.074, z_center=-0.058),
        _yz_rounded_section(0.390, width=0.33, height=0.20, radius=0.070, z_center=-0.050),
        _yz_rounded_section(0.500, width=0.20, height=0.11, radius=0.045, z_center=-0.010),
    ]
    inner_sections = [
        _yz_superellipse_section(0.030, width=0.18, height=0.046, z_center=0.021),
        _yz_superellipse_section(0.130, width=0.22, height=0.104, z_center=-0.008),
        _yz_superellipse_section(0.250, width=0.23, height=0.118, z_center=-0.015),
        _yz_superellipse_section(0.360, width=0.20, height=0.090, z_center=-0.001),
        _yz_superellipse_section(0.440, width=0.13, height=0.038, z_center=0.025),
    ]
    outer_shell = section_loft(outer_sections)
    inner_shell = section_loft(inner_sections)
    rim_ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.44, 0.36, 0.065, corner_segments=10),
        [superellipse_profile(0.32, 0.22, exponent=2.6, segments=48)],
        0.022,
        center=True,
    ).translate(0.22, 0.0, 0.055)
    outlet_stub = (
        CylinderGeometry(radius=0.045, height=0.075, radial_segments=28)
        .rotate_y(pi / 2.0)
        .translate(0.042, 0.0, -0.062)
    )
    return _merge_geometries(outer_shell, inner_shell, rim_ring, outlet_stub)


def _build_bowl_rear_mount() -> MeshGeometry:
    return _merge_geometries(
        BoxGeometry((0.094, 0.350, 0.100)).translate(0.047, 0.0, -0.002),
        BoxGeometry((0.046, 0.230, 0.018)).translate(0.024, 0.0, 0.055),
        BoxGeometry((0.030, 0.058, 0.020)).translate(0.020, 0.086, 0.064),
        BoxGeometry((0.030, 0.058, 0.020)).translate(0.020, -0.086, 0.064),
    )


def _build_seat_mesh() -> MeshGeometry:
    seat_ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.40, 0.35, 0.060, corner_segments=10),
        [superellipse_profile(0.30, 0.19, exponent=2.8, segments=48)],
        0.014,
        center=True,
    ).translate(0.214, 0.0, 0.007)
    rear_bridge = BoxGeometry((0.034, 0.200, 0.010)).translate(0.012, 0.0, 0.005)
    left_barrel = (
        CylinderGeometry(radius=0.010, height=0.042, radial_segments=20)
        .rotate_x(pi / 2.0)
        .translate(0.0, 0.088, 0.006)
    )
    right_barrel = (
        CylinderGeometry(radius=0.010, height=0.042, radial_segments=20)
        .rotate_x(pi / 2.0)
        .translate(0.0, -0.088, 0.006)
    )
    return _merge_geometries(seat_ring, rear_bridge, left_barrel, right_barrel)


def _build_lid_mesh() -> MeshGeometry:
    lid_panel = ExtrudeGeometry(
        rounded_rect_profile(0.44, 0.36, 0.066, corner_segments=10),
        0.010,
        center=True,
    ).translate(0.226, 0.0, 0.005)
    rear_bridge = BoxGeometry((0.040, 0.220, 0.010)).translate(0.012, 0.0, 0.005)
    left_barrel = (
        CylinderGeometry(radius=0.010, height=0.046, radial_segments=20)
        .rotate_x(pi / 2.0)
        .translate(0.0, 0.086, 0.006)
    )
    right_barrel = (
        CylinderGeometry(radius=0.010, height=0.046, radial_segments=20)
        .rotate_x(pi / 2.0)
        .translate(0.0, -0.086, 0.006)
    )
    return _merge_geometries(lid_panel, rear_bridge, left_barrel, right_barrel)


def _build_flush_paddle_mesh() -> MeshGeometry:
    return _merge_geometries(
        BoxGeometry((0.018, 0.076, 0.096)).translate(0.009, 0.0, 0.0),
        BoxGeometry((0.022, 0.040, 0.032)).translate(0.011, 0.0, -0.022),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_hung_toilet")

    wall_white = model.material("wall_white", rgba=(0.94, 0.94, 0.93, 1.0))
    ceramic_white = model.material("ceramic_white", rgba=(0.98, 0.98, 0.97, 1.0))
    seat_white = model.material("seat_white", rgba=(0.96, 0.96, 0.95, 1.0))
    plate_white = model.material("plate_white", rgba=(0.91, 0.91, 0.90, 1.0))
    chrome = model.material("chrome", rgba=(0.80, 0.81, 0.83, 1.0))

    service_wall = model.part("service_wall")
    service_wall.visual(
        Box((0.18, 0.54, 1.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=wall_white,
        name="service_wall_shell",
    )
    service_wall.inertial = Inertial.from_geometry(
        Box((0.18, 0.54, 1.12)),
        mass=60.0,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
    )

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.008, 0.22, 0.15)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=plate_white,
        name="plate_shell",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.008, 0.22, 0.15)),
        mass=0.25,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
    )

    flush_paddle = model.part("flush_paddle")
    flush_paddle.visual(
        Box((0.018, 0.076, 0.096)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material=chrome,
        name="paddle_face",
    )
    flush_paddle.visual(
        Box((0.022, 0.040, 0.030)),
        origin=Origin(xyz=(0.011, 0.0, -0.028)),
        material=chrome,
        name="paddle_thumb",
    )
    flush_paddle.inertial = Inertial.from_geometry(
        Box((0.022, 0.08, 0.10)),
        mass=0.08,
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        Box((0.10, 0.36, 0.12)),
        origin=Origin(xyz=(0.05, 0.0, -0.07)),
        material=ceramic_white,
        name="rear_shroud",
    )
    bowl.visual(
        Box((0.34, 0.030, 0.025)),
        origin=Origin(xyz=(0.21, 0.150, -0.0125)),
        material=ceramic_white,
        name="left_rim",
    )
    bowl.visual(
        Box((0.34, 0.030, 0.025)),
        origin=Origin(xyz=(0.21, -0.150, -0.0125)),
        material=ceramic_white,
        name="right_rim",
    )
    bowl.visual(
        Box((0.05, 0.33, 0.025)),
        origin=Origin(xyz=(0.405, 0.0, -0.0125)),
        material=ceramic_white,
        name="front_rim",
    )
    bowl.visual(
        Box((0.05, 0.18, 0.025)),
        origin=Origin(xyz=(0.045, 0.0, -0.0125)),
        material=ceramic_white,
        name="rear_rim",
    )
    bowl.visual(
        Box((0.24, 0.05, 0.15)),
        origin=Origin(xyz=(0.22, 0.125, -0.10)),
        material=ceramic_white,
        name="left_shell",
    )
    bowl.visual(
        Box((0.24, 0.05, 0.15)),
        origin=Origin(xyz=(0.22, -0.125, -0.10)),
        material=ceramic_white,
        name="right_shell",
    )
    bowl.visual(
        Cylinder(radius=0.075, length=0.25),
        origin=Origin(xyz=(0.39, 0.0, -0.105), rpy=(pi / 2.0, 0.0, 0.0)),
        material=ceramic_white,
        name="front_shell",
    )
    bowl.visual(
        Box((0.20, 0.18, 0.06)),
        origin=Origin(xyz=(0.28, 0.0, -0.15)),
        material=ceramic_white,
        name="underside_shell",
    )
    bowl.visual(
        Cylinder(radius=0.048, length=0.06),
        origin=Origin(xyz=(0.03, 0.0, -0.13), rpy=(0.0, pi / 2.0, 0.0)),
        material=ceramic_white,
        name="waste_outlet",
    )
    bowl.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.018, 0.115, -0.024), rpy=(0.0, pi / 2.0, 0.0)),
        material=ceramic_white,
        name="left_hinge_cap",
    )
    bowl.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.018, -0.115, -0.024), rpy=(0.0, pi / 2.0, 0.0)),
        material=ceramic_white,
        name="right_hinge_cap",
    )
    bowl.inertial = Inertial.from_geometry(
        Box((0.52, 0.36, 0.24)),
        mass=20.0,
        origin=Origin(xyz=(0.24, 0.0, -0.020)),
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.040, 0.19, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, -0.006)),
        material=seat_white,
        name="seat_rear_bridge",
    )
    seat.visual(
        Box((0.36, 0.070, 0.012)),
        origin=Origin(xyz=(0.21, 0.125, -0.006)),
        material=seat_white,
        name="seat_left_rail",
    )
    seat.visual(
        Box((0.36, 0.070, 0.012)),
        origin=Origin(xyz=(0.21, -0.125, -0.006)),
        material=seat_white,
        name="seat_right_rail",
    )
    seat.visual(
        Box((0.055, 0.27, 0.012)),
        origin=Origin(xyz=(0.3925, 0.0, -0.006)),
        material=seat_white,
        name="seat_front_bridge",
    )
    seat.visual(
        Cylinder(radius=0.010, length=0.042),
        origin=Origin(xyz=(0.0, 0.090, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=seat_white,
        name="seat_left_barrel",
    )
    seat.visual(
        Cylinder(radius=0.010, length=0.042),
        origin=Origin(xyz=(0.0, -0.090, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=seat_white,
        name="seat_right_barrel",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.40, 0.35, 0.026)),
        mass=1.8,
        origin=Origin(xyz=(0.20, 0.0, 0.010)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.050, 0.12, 0.010)),
        origin=Origin(xyz=(0.025, 0.0, -0.005)),
        material=seat_white,
        name="lid_rear_bridge",
    )
    lid.visual(
        Box((0.150, 0.050, 0.010)),
        origin=Origin(xyz=(0.125, 0.085, -0.005)),
        material=seat_white,
        name="lid_left_arm",
    )
    lid.visual(
        Box((0.150, 0.050, 0.010)),
        origin=Origin(xyz=(0.125, -0.085, -0.005)),
        material=seat_white,
        name="lid_right_arm",
    )
    lid.visual(
        Box((0.320, 0.36, 0.010)),
        origin=Origin(xyz=(0.300, 0.0, -0.005)),
        material=seat_white,
        name="lid_panel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.44, 0.36, 0.024)),
        mass=2.2,
        origin=Origin(xyz=(0.22, 0.0, 0.010)),
    )

    model.articulation(
        "service_wall_to_wall_plate",
        ArticulationType.FIXED,
        parent=service_wall,
        child=wall_plate,
        origin=Origin(xyz=(0.09, 0.0, 0.96)),
    )
    model.articulation(
        "wall_plate_to_flush_paddle",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=flush_paddle,
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-0.22, upper=0.22),
    )
    model.articulation(
        "service_wall_to_bowl",
        ArticulationType.FIXED,
        parent=service_wall,
        child=bowl,
        origin=Origin(xyz=(0.09, 0.0, 0.42)),
    )
    model.articulation(
        "bowl_to_seat",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=seat,
        origin=Origin(xyz=(0.018, 0.0, 0.012)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.6, lower=0.0, upper=1.45),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.012, 0.0, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.6, lower=0.0, upper=1.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    service_wall = object_model.get_part("service_wall")
    wall_plate = object_model.get_part("wall_plate")
    flush_paddle = object_model.get_part("flush_paddle")
    bowl = object_model.get_part("bowl")
    seat = object_model.get_part("seat")
    lid = object_model.get_part("lid")

    seat_hinge = object_model.get_articulation("bowl_to_seat")
    lid_hinge = object_model.get_articulation("bowl_to_lid")
    flush_joint = object_model.get_articulation("wall_plate_to_flush_paddle")

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

    ctx.expect_contact(
        bowl,
        service_wall,
        elem_a="rear_shroud",
        elem_b="service_wall_shell",
        name="bowl shroud bears on the service wall",
    )
    ctx.expect_contact(
        wall_plate,
        service_wall,
        elem_a="plate_shell",
        elem_b="service_wall_shell",
        name="wall plate mounts flush to the front panel",
    )
    ctx.expect_contact(
        flush_paddle,
        wall_plate,
        elem_a="paddle_face",
        elem_b="plate_shell",
        name="flush paddle sits on the wall plate",
    )
    ctx.expect_gap(
        seat,
        bowl,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="seat_left_rail",
        negative_elem="left_rim",
        name="seat left rail rests on the ceramic rim",
    )
    ctx.expect_overlap(
        seat,
        bowl,
        axes="xy",
        min_overlap=0.02,
        elem_a="seat_left_rail",
        elem_b="left_rim",
        name="seat left rail overlaps the left rim",
    )
    ctx.expect_gap(
        seat,
        bowl,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="seat_right_rail",
        negative_elem="right_rim",
        name="seat right rail rests on the ceramic rim",
    )
    ctx.expect_overlap(
        seat,
        bowl,
        axes="xy",
        min_overlap=0.02,
        elem_a="seat_right_rail",
        elem_b="right_rim",
        name="seat right rail overlaps the right rim",
    )
    ctx.expect_gap(
        seat,
        bowl,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="seat_front_bridge",
        negative_elem="front_rim",
        name="seat front bridge rests on the ceramic rim",
    )
    ctx.expect_overlap(
        seat,
        bowl,
        axes="xy",
        min_overlap=0.04,
        elem_a="seat_front_bridge",
        elem_b="front_rim",
        name="seat front bridge overlaps the front rim",
    )
    ctx.expect_gap(
        lid,
        seat,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="lid_panel",
        negative_elem="seat_front_bridge",
        name="lid closes onto the seat front bridge",
    )
    ctx.expect_overlap(
        lid,
        seat,
        axes="xy",
        min_overlap=0.25,
        name="lid footprint covers the seat opening",
    )

    ctx.check(
        "seat hinge opens upward about a horizontal rear axis",
        seat_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={seat_hinge.axis}",
    )
    ctx.check(
        "lid hinge opens upward about a horizontal rear axis",
        lid_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "flush paddle rotates around the front to back axis",
        flush_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={flush_joint.axis}",
    )

    seat_closed = ctx.part_element_world_aabb(seat, elem="seat_front_bridge")
    lid_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({seat_hinge: seat_hinge.motion_limits.upper}):
        seat_open = ctx.part_element_world_aabb(seat, elem="seat_front_bridge")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "seat front edge rises when the seat is opened",
        seat_closed is not None
        and seat_open is not None
        and seat_open[1][2] > seat_closed[1][2] + 0.22,
        details=f"closed={seat_closed}, open={seat_open}",
    )
    ctx.check(
        "lid front edge rises when the lid is opened",
        lid_closed is not None
        and lid_open is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.24,
        details=f"closed={lid_closed}, open={lid_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
