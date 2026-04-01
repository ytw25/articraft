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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_face,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _offset_profile(profile, *, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _rr_loop(
    size_x: float,
    size_y: float,
    radius: float,
    z: float,
    *,
    dx: float = 0.0,
    dy: float = 0.0,
    corner_segments: int = 10,
):
    return [
        (x + dx, y + dy, z)
        for x, y in rounded_rect_profile(
            size_x,
            size_y,
            radius,
            corner_segments=corner_segments,
        )
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_piece_toilet")

    ceramic = model.material("ceramic", rgba=(0.96, 0.96, 0.94, 1.0))
    seat_plastic = model.material("seat_plastic", rgba=(0.95, 0.94, 0.91, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.74, 0.74, 0.76, 1.0))
    chrome = model.material("chrome", rgba=(0.74, 0.77, 0.80, 1.0))

    bowl = model.part("bowl_base")
    bowl.inertial = Inertial.from_geometry(
        Box((0.72, 0.40, 0.78)),
        mass=34.0,
        origin=Origin(xyz=(0.00, 0.0, 0.39)),
    )

    pedestal_geom = section_loft(
        [
            _rr_loop(0.34, 0.28, 0.10, 0.00, dx=0.00, corner_segments=8),
            _rr_loop(0.27, 0.22, 0.08, 0.18, dx=-0.01, corner_segments=8),
            _rr_loop(0.24, 0.19, 0.07, 0.31, dx=-0.02, corner_segments=8),
        ]
    )
    bowl.visual(
        _save_mesh("toilet_pedestal", pedestal_geom),
        material=ceramic,
        name="pedestal_shell",
    )

    outer_bowl_geom = tube_from_spline_points(
        [
            (-0.10, 0.13, 0.293),
            (-0.01, 0.18, 0.290),
            (0.16, 0.17, 0.282),
            (0.27, 0.00, 0.272),
            (0.16, -0.17, 0.282),
            (-0.01, -0.18, 0.290),
            (-0.10, -0.13, 0.293),
        ],
        radius=0.085,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    bowl.visual(
        _save_mesh("toilet_outer_bowl", outer_bowl_geom),
        material=ceramic,
        name="outer_bowl",
    )

    rim_outer = rounded_rect_profile(0.48, 0.39, 0.12, corner_segments=10)
    opening_profile = rounded_rect_profile(0.31, 0.22, 0.09, corner_segments=10)
    drain_profile = rounded_rect_profile(0.10, 0.07, 0.03, corner_segments=8)

    rim_geom = ExtrudeWithHolesGeometry(
        _offset_profile(rim_outer, dx=0.07),
        [_offset_profile(opening_profile, dx=0.07)],
        height=0.030,
        center=True,
    )
    bowl.visual(
        _save_mesh("toilet_rim", rim_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.393)),
        material=ceramic,
        name="bowl_rim",
    )

    inner_bowl_geom = ExtrudeWithHolesGeometry(
        _offset_profile(opening_profile, dx=0.07),
        [_offset_profile(drain_profile, dx=0.11)],
        height=0.240,
        center=True,
    )
    bowl.visual(
        _save_mesh("toilet_inner_bowl", inner_bowl_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.258)),
        material=ceramic,
        name="inner_bowl",
    )

    bowl.visual(
        Box((0.20, 0.34, 0.060)),
        origin=Origin(xyz=(-0.19, 0.0, 0.378)),
        material=ceramic,
        name="rear_bridge",
    )
    bowl.visual(
        Box((0.16, 0.30, 0.030)),
        origin=Origin(xyz=(-0.24, 0.0, 0.393)),
        material=ceramic,
        name="tank_deck",
    )

    tank = model.part("tank")
    tank.inertial = Inertial.from_geometry(
        Box((0.22, 0.40, 0.40)),
        mass=14.0,
        origin=Origin(xyz=(-0.005, 0.0, 0.20)),
    )

    tank_body_geom = section_loft(
        [
            _rr_loop(0.20, 0.38, 0.035, 0.03, dx=-0.005, corner_segments=8),
            _rr_loop(0.21, 0.39, 0.040, 0.18, dx=-0.010, corner_segments=8),
            _rr_loop(0.19, 0.38, 0.032, 0.34, dx=-0.015, corner_segments=8),
        ]
    )
    tank.visual(
        Box((0.12, 0.24, 0.030)),
        origin=Origin(xyz=(-0.01, 0.0, 0.015)),
        material=ceramic,
        name="tank_foot",
    )
    tank.visual(
        _save_mesh("toilet_tank_body", tank_body_geom),
        material=ceramic,
        name="tank_body",
    )
    tank.visual(
        Box((0.23, 0.41, 0.024)),
        origin=Origin(xyz=(-0.015, 0.0, 0.352)),
        material=ceramic,
        name="tank_lid",
    )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.44, 0.36, 0.05)),
        mass=1.8,
        origin=Origin(xyz=(0.24, 0.0, -0.010)),
    )

    seat_panel_geom = ExtrudeWithHolesGeometry(
        _offset_profile(rounded_rect_profile(0.44, 0.36, 0.11, corner_segments=10), dx=0.24),
        [_offset_profile(rounded_rect_profile(0.30, 0.21, 0.08, corner_segments=10), dx=0.24)],
        height=0.024,
        center=True,
    )
    seat.visual(
        _save_mesh("toilet_seat_panel", seat_panel_geom),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=seat_plastic,
        name="seat_panel",
    )
    seat.visual(
        Cylinder(radius=0.013, length=0.055),
        origin=Origin(xyz=(0.0, 0.105, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_gray,
        name="seat_left_barrel",
    )
    seat.visual(
        Cylinder(radius=0.013, length=0.055),
        origin=Origin(xyz=(0.0, -0.105, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_gray,
        name="seat_right_barrel",
    )
    seat.visual(
        Box((0.060, 0.040, 0.018)),
        origin=Origin(xyz=(0.030, 0.105, -0.008)),
        material=hinge_gray,
        name="seat_left_hinge_bridge",
    )
    seat.visual(
        Box((0.060, 0.040, 0.018)),
        origin=Origin(xyz=(0.030, -0.105, -0.008)),
        material=hinge_gray,
        name="seat_right_hinge_bridge",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.45, 0.37, 0.05)),
        mass=2.2,
        origin=Origin(xyz=(0.245, 0.0, 0.010)),
    )

    lid_panel_geom = ExtrudeGeometry(
        _offset_profile(rounded_rect_profile(0.45, 0.37, 0.12, corner_segments=10), dx=0.245),
        height=0.019,
        center=True,
    )
    lid.visual(
        _save_mesh("toilet_lid_panel", lid_panel_geom),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=seat_plastic,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.034),
        origin=Origin(xyz=(0.0, 0.160, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_gray,
        name="lid_left_barrel",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.034),
        origin=Origin(xyz=(0.0, -0.160, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_gray,
        name="lid_right_barrel",
    )
    lid.visual(
        Box((0.080, 0.030, 0.020)),
        origin=Origin(xyz=(0.040, 0.160, 0.010)),
        material=hinge_gray,
        name="lid_left_hinge_arm",
    )
    lid.visual(
        Box((0.080, 0.030, 0.020)),
        origin=Origin(xyz=(0.040, -0.160, 0.010)),
        material=hinge_gray,
        name="lid_right_hinge_arm",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.28, 0.10, 0.003)),
        material=hinge_gray,
        name="lid_left_bumper",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.28, -0.10, 0.003)),
        material=hinge_gray,
        name="lid_right_bumper",
    )

    flush_lever = model.part("flush_lever")
    flush_lever.inertial = Inertial.from_geometry(
        Box((0.08, 0.05, 0.04)),
        mass=0.12,
        origin=Origin(xyz=(0.04, 0.025, -0.010)),
    )
    flush_lever.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lever_rosette",
    )
    flush_lever.visual(
        Box((0.052, 0.012, 0.010)),
        origin=Origin(xyz=(0.026, 0.026, -0.010)),
        material=chrome,
        name="lever_handle",
    )
    flush_lever.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.056, 0.032, -0.010)),
        material=chrome,
        name="lever_knob",
    )

    model.articulation(
        "bowl_to_tank",
        ArticulationType.FIXED,
        parent=bowl,
        child=tank,
        origin=Origin(xyz=(-0.235, 0.0, 0.408)),
    )
    model.articulation(
        "bowl_to_seat",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=seat,
        origin=Origin(xyz=(-0.10, 0.0, 0.432)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(-0.10, 0.0, 0.432)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "tank_to_flush_lever",
        ArticulationType.REVOLUTE,
        parent=tank,
        child=flush_lever,
        origin=place_on_face(
            tank,
            "+y",
            face_pos=(0.065, 0.060),
            proud=0.0,
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bowl = object_model.get_part("bowl_base")
    tank = object_model.get_part("tank")
    seat = object_model.get_part("seat")
    lid = object_model.get_part("lid")
    flush_lever = object_model.get_part("flush_lever")
    seat_hinge = object_model.get_articulation("bowl_to_seat")
    lid_hinge = object_model.get_articulation("bowl_to_lid")
    lever_joint = object_model.get_articulation("tank_to_flush_lever")

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
        tank,
        bowl,
        elem_a="tank_foot",
        elem_b="tank_deck",
        name="tank foot contacts rear deck",
    )

    with ctx.pose({seat_hinge: 0.0, lid_hinge: 0.0}):
        ctx.expect_gap(
            seat,
            bowl,
            axis="z",
            max_gap=0.002,
            max_penetration=1e-5,
            positive_elem="seat_panel",
            negative_elem="bowl_rim",
            name="seat rests on bowl rim",
        )
        ctx.expect_overlap(
            seat,
            bowl,
            axes="xy",
            elem_a="seat_panel",
            elem_b="bowl_rim",
            min_overlap=0.18,
            name="seat covers the bowl opening footprint",
        )
        ctx.expect_gap(
            lid,
            seat,
            axis="z",
            min_gap=0.0,
            max_gap=0.010,
            positive_elem="lid_panel",
            negative_elem="seat_panel",
            name="lid nests just above the seat",
        )
        ctx.expect_overlap(
            lid,
            seat,
            axes="xy",
            elem_a="lid_panel",
            elem_b="seat_panel",
            min_overlap=0.18,
            name="lid covers the seat footprint",
        )
        ctx.expect_contact(
            lid,
            seat,
            elem_a="lid_left_bumper",
            elem_b="seat_panel",
            name="lid bumper lands on the seat ring",
        )

    seat_closed_aabb = ctx.part_element_world_aabb(seat, elem="seat_panel")
    with ctx.pose({seat_hinge: 1.55}):
        seat_open_aabb = ctx.part_element_world_aabb(seat, elem="seat_panel")
    ctx.check(
        "seat lifts upward on the rear hinge",
        seat_closed_aabb is not None
        and seat_open_aabb is not None
        and seat_open_aabb[1][2] > seat_closed_aabb[1][2] + 0.16,
        details=f"closed={seat_closed_aabb}, open={seat_open_aabb}",
    )

    lid_closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.70}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    ctx.check(
        "lid swings farther upward than the seat",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.20,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    lever_rest_aabb = ctx.part_element_world_aabb(flush_lever, elem="lever_handle")
    with ctx.pose({lever_joint: 0.45}):
        lever_flush_aabb = ctx.part_element_world_aabb(flush_lever, elem="lever_handle")
    ctx.check(
        "flush lever depresses around the tank-side pivot",
        lever_rest_aabb is not None
        and lever_flush_aabb is not None
        and lever_flush_aabb[0][2] < lever_rest_aabb[0][2] - 0.010,
        details=f"rest={lever_rest_aabb}, flushed={lever_flush_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
