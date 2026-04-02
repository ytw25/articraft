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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _loop(profile, z: float, *, dx: float = 0.0, dy: float = 0.0):
    return [(x + dx, y + dy, z) for x, y in profile]


def _circle_points(
    *,
    center: tuple[float, float, float],
    radius: float,
    segments: int = 16,
    plane: str = "xz",
) -> list[tuple[float, float, float]]:
    cx, cy, cz = center
    points: list[tuple[float, float, float]] = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        c = math.cos(angle) * radius
        s = math.sin(angle) * radius
        if plane == "xz":
            points.append((cx + c, cy, cz + s))
        elif plane == "yz":
            points.append((cx, cy + c, cz + s))
        else:
            points.append((cx + c, cy + s, cz))
    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_high_tank_toilet")

    porcelain = model.material("porcelain", rgba=(0.95, 0.95, 0.93, 1.0))
    porcelain_shadow = model.material("porcelain_shadow", rgba=(0.86, 0.87, 0.86, 1.0))
    wood = model.material("wood", rgba=(0.50, 0.33, 0.17, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.39, 0.24, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.72, 0.60, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_void = model.material("dark_void", rgba=(0.08, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.70, 0.52, 1.86)),
        mass=48.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.93)),
    )

    bowl_outer_profile = rounded_rect_profile(0.42, 0.36, 0.11, corner_segments=8)
    bowl_mid_profile = rounded_rect_profile(0.32, 0.29, 0.08, corner_segments=8)
    bowl_low_profile = rounded_rect_profile(0.19, 0.18, 0.05, corner_segments=8)
    bowl_inner_top = rounded_rect_profile(0.26, 0.20, 0.08, corner_segments=8)
    bowl_inner_mid = rounded_rect_profile(0.19, 0.15, 0.06, corner_segments=8)
    bowl_inner_low = rounded_rect_profile(0.11, 0.10, 0.04, corner_segments=8)

    bowl_outer_shell = section_loft(
        [
            _loop(bowl_low_profile, 0.040),
            _loop(bowl_mid_profile, 0.220),
            _loop(bowl_outer_profile, 0.402),
        ],
        cap=False,
        solid=False,
    )
    bowl_inner_shell = section_loft(
        [
            _loop(bowl_inner_top, 0.402),
            _loop(bowl_inner_mid, 0.300),
            _loop(bowl_inner_low, 0.180),
        ],
        cap=False,
        solid=False,
    )
    bowl_rim = ExtrudeWithHolesGeometry(
        bowl_outer_profile,
        [bowl_inner_top],
        0.024,
        center=True,
    )
    cistern_shell = ExtrudeGeometry(
        rounded_rect_profile(0.20, 0.44, 0.020, corner_segments=8),
        0.28,
        center=True,
    )

    flush_pipe = tube_from_spline_points(
        [
            (-0.085, 0.0, 1.440),
            (-0.085, 0.0, 1.120),
            (-0.100, 0.0, 0.760),
            (-0.118, 0.0, 0.560),
        ],
        radius=0.024,
        samples_per_segment=16,
        radial_segments=20,
        cap_ends=True,
    )

    chain_guide = wire_from_points(
        [
            (0.0, -0.018, 0.0),
            (0.0, -0.018, -0.045),
            (0.0, 0.018, -0.045),
            (0.0, 0.018, 0.0),
            (0.0, 0.004, -0.040),
            (0.002, 0.0, -0.160),
            (0.008, 0.0, -0.310),
            (0.018, 0.0, -0.470),
            (0.030, 0.0, -0.530),
            (0.052, 0.0, -0.545),
            (0.052, 0.0, -0.568),
            (0.032, 0.0, -0.582),
            (0.010, 0.0, -0.572),
            (0.004, 0.0, -0.548),
            (0.020, 0.0, -0.530),
            (0.045, 0.0, -0.530),
            (0.056, 0.0, -0.548),
        ],
        radius=0.0032,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.007,
        corner_segments=10,
    )

    body.visual(
        Box((0.30, 0.22, 0.040)),
        origin=Origin(xyz=(-0.035, 0.0, 0.020)),
        material=porcelain,
        name="base_foot",
    )
    body.visual(
        _mesh("toilet_bowl_outer_shell", bowl_outer_shell),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=porcelain,
        name="bowl_outer_shell",
    )
    body.visual(
        _mesh("toilet_bowl_rim", bowl_rim),
        origin=Origin(xyz=(0.075, 0.0, 0.414)),
        material=porcelain,
        name="bowl_rim",
    )
    body.visual(
        _mesh("toilet_bowl_inner_shell", bowl_inner_shell),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=porcelain_shadow,
        name="bowl_inner_shell",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.180),
        origin=Origin(xyz=(0.075, 0.0, 0.090)),
        material=dark_void,
        name="trap_throat",
    )
    body.visual(
        Box((0.18, 0.24, 0.090)),
        origin=Origin(xyz=(-0.105, 0.0, 0.389)),
        material=porcelain,
        name="rear_deck",
    )
    body.visual(
        Box((0.054, 0.340, 0.160)),
        origin=Origin(xyz=(-0.182, 0.0, 0.470)),
        material=porcelain,
        name="rear_bowl_neck",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.085),
        origin=Origin(xyz=(-0.126, 0.0, 0.545), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=porcelain,
        name="flush_spud",
    )
    body.visual(
        Cylinder(radius=0.017, length=1.440),
        origin=Origin(xyz=(-0.145, -0.160, 0.720)),
        material=brass,
        name="left_support_leg",
    )
    body.visual(
        Cylinder(radius=0.017, length=1.440),
        origin=Origin(xyz=(-0.145, 0.160, 0.720)),
        material=brass,
        name="right_support_leg",
    )
    body.visual(
        Box((0.080, 0.38, 0.020)),
        origin=Origin(xyz=(-0.145, 0.0, 1.446)),
        material=brass,
        name="support_crossbar",
    )
    body.visual(
        _mesh("toilet_cistern_shell", cistern_shell),
        origin=Origin(xyz=(-0.095, 0.0, 1.580)),
        material=porcelain,
        name="cistern_body",
    )
    body.visual(
        Box((0.235, 0.475, 0.020)),
        origin=Origin(xyz=(-0.095, 0.0, 1.730)),
        material=porcelain,
        name="cistern_lid",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(-0.085, 0.0, 1.435)),
        material=brass,
        name="cistern_outlet",
    )
    body.visual(
        _mesh("toilet_flush_pipe", flush_pipe),
        material=brass,
        name="flush_pipe",
    )
    body.visual(
        _mesh("toilet_chain_guide", chain_guide),
        origin=Origin(xyz=(-0.045, 0.0, 1.440)),
        material=brass,
        name="chain_guide",
    )

    seat = model.part("seat")
    seat.inertial = Inertial.from_geometry(
        Box((0.44, 0.38, 0.070)),
        mass=2.0,
        origin=Origin(xyz=(0.215, 0.0, -0.018)),
    )
    seat_ring_mesh = _mesh(
        "toilet_seat_ring",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.42, 0.36, 0.11, corner_segments=8),
            [rounded_rect_profile(0.27, 0.21, 0.08, corner_segments=8)],
            0.022,
            center=True,
        ),
    )
    seat.visual(
        seat_ring_mesh,
        origin=Origin(xyz=(0.215, 0.0, -0.017)),
        material=wood,
        name="seat_ring",
    )
    seat.visual(
        Box((0.060, 0.040, 0.018)),
        origin=Origin(xyz=(0.025, -0.098, -0.009)),
        material=brass,
        name="seat_left_arm",
    )
    seat.visual(
        Box((0.060, 0.040, 0.018)),
        origin=Origin(xyz=(0.025, 0.098, -0.009)),
        material=brass,
        name="seat_right_arm",
    )
    seat.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(xyz=(0.0, -0.098, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="seat_left_barrel",
    )
    seat.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(xyz=(0.0, 0.098, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="seat_right_barrel",
    )
    seat.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.145, -0.095, -0.030)),
        material=rubber,
        name="seat_bumper_left",
    )
    seat.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.145, 0.095, -0.030)),
        material=rubber,
        name="seat_bumper_right",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.44, 0.38, 0.060)),
        mass=2.2,
        origin=Origin(xyz=(0.215, 0.0, -0.011)),
    )
    lid_lower = rounded_rect_profile(0.43, 0.37, 0.11, corner_segments=8)
    lid_upper = rounded_rect_profile(0.40, 0.34, 0.095, corner_segments=8)
    lid_panel_mesh = _mesh(
        "toilet_lid_panel",
        section_loft(
            [
                _loop(lid_lower, -0.009),
                _loop(lid_upper, 0.009),
            ],
        ),
    )
    lid.visual(
        lid_panel_mesh,
        origin=Origin(xyz=(0.227, 0.0, -0.011)),
        material=wood_dark,
        name="lid_panel",
    )
    lid.visual(
        Box((0.056, 0.038, 0.016)),
        origin=Origin(xyz=(0.022, -0.095, -0.006)),
        material=brass,
        name="lid_left_arm",
    )
    lid.visual(
        Box((0.056, 0.038, 0.016)),
        origin=Origin(xyz=(0.022, 0.095, -0.006)),
        material=brass,
        name="lid_right_arm",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, -0.095, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="lid_left_barrel",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.095, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="lid_right_barrel",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.230, -0.132, -0.022)),
        material=rubber,
        name="lid_bumper_left",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.230, 0.132, -0.022)),
        material=rubber,
        name="lid_bumper_right",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.060, 0.110, 0.070)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.038, -0.022)),
    )
    handle.visual(
        Cylinder(radius=0.009, length=0.038),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="handle_pivot",
    )
    handle.visual(
        Box((0.022, 0.064, 0.014)),
        origin=Origin(xyz=(0.0, 0.022, -0.010)),
        material=brass,
        name="handle_arm",
    )
    handle.visual(
        Cylinder(radius=0.0105, length=0.026),
        origin=Origin(xyz=(0.0, 0.056, -0.024), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wood_dark,
        name="handle_grip",
    )

    model.articulation(
        "body_to_seat",
        ArticulationType.REVOLUTE,
        parent=body,
        child=seat,
        origin=Origin(xyz=(-0.145, 0.0, 0.458)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.145, 0.0, 0.476)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.055, 0.22894, 1.575)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    seat = object_model.get_part("seat")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    seat_hinge = object_model.get_articulation("body_to_seat")
    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_joint = object_model.get_articulation("body_to_handle")

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

    ctx.check(
        "seat hinge opens upward on rear horizontal axis",
        seat_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={seat_hinge.axis}",
    )
    ctx.check(
        "lid hinge opens upward on parallel rear horizontal axis",
        lid_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "handle pivot is horizontal on the cistern side",
        handle_joint.axis == (-1.0, 0.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )

    with ctx.pose({seat_hinge: 0.0, lid_hinge: 0.0, handle_joint: 0.0}):
        ctx.expect_contact(
            seat,
            body,
            elem_a="seat_bumper_left",
            elem_b="bowl_rim",
            contact_tol=0.001,
            name="left seat bumper rests on bowl rim",
        )
        ctx.expect_contact(
            seat,
            body,
            elem_a="seat_bumper_right",
            elem_b="bowl_rim",
            contact_tol=0.001,
            name="right seat bumper rests on bowl rim",
        )
        ctx.expect_overlap(
            seat,
            body,
            axes="xy",
            elem_a="seat_ring",
            elem_b="bowl_rim",
            min_overlap=0.20,
            name="seat footprint covers bowl rim",
        )
        ctx.expect_contact(
            lid,
            seat,
            elem_a="lid_bumper_left",
            elem_b="seat_ring",
            contact_tol=0.001,
            name="left lid bumper lands on seat",
        )
        ctx.expect_contact(
            lid,
            seat,
            elem_a="lid_bumper_right",
            elem_b="seat_ring",
            contact_tol=0.001,
            name="right lid bumper lands on seat",
        )
        ctx.expect_overlap(
            lid,
            seat,
            axes="xy",
            elem_a="lid_panel",
            elem_b="seat_ring",
            min_overlap=0.20,
            name="lid covers the wooden seat",
        )
        ctx.expect_contact(
            handle,
            body,
            elem_a="handle_pivot",
            elem_b="cistern_body",
            contact_tol=0.001,
            name="handle pivot seats against cistern wall",
        )

        seat_closed = ctx.part_element_world_aabb(seat, elem="seat_ring")
        lid_closed = ctx.part_element_world_aabb(lid, elem="lid_panel")
        handle_closed = ctx.part_element_world_aabb(handle, elem="handle_grip")

    with ctx.pose({seat_hinge: math.radians(72.0)}):
        seat_open = ctx.part_element_world_aabb(seat, elem="seat_ring")

    with ctx.pose({lid_hinge: math.radians(95.0)}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({handle_joint: 0.72}):
        handle_flushed = ctx.part_element_world_aabb(handle, elem="handle_grip")

    ctx.check(
        "seat rotates up away from the bowl",
        seat_closed is not None
        and seat_open is not None
        and seat_open[1][2] > seat_closed[1][2] + 0.20,
        details=f"closed={seat_closed}, open={seat_open}",
    )
    ctx.check(
        "lid swings up above the seat on its own hinge",
        lid_closed is not None
        and lid_open is not None
        and lid_open[1][2] > lid_closed[1][2] + 0.28,
        details=f"closed={lid_closed}, open={lid_open}",
    )
    ctx.check(
        "flush handle rotates downward when actuated",
        handle_closed is not None
        and handle_flushed is not None
        and handle_flushed[0][2] < handle_closed[0][2] - 0.015,
        details=f"closed={handle_closed}, flushed={handle_flushed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
