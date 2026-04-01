from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gooseneck_electric_kettle")

    steel = model.material("steel", rgba=(0.86, 0.86, 0.84, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.13, 0.13, 0.14, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.20, 0.20, 0.22, 1.0))

    power_base = model.part("power_base")
    power_base.visual(
        Cylinder(radius=0.085, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black_plastic,
        name="base_plate",
    )
    power_base.visual(
        Cylinder(radius=0.056, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=dark_trim,
        name="base_pedestal",
    )
    power_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.085, length=0.028),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    body = model.part("body")
    outer_profile = [
        (0.052, 0.000),
        (0.060, 0.015),
        (0.079, 0.080),
        (0.082, 0.155),
        (0.076, 0.205),
        (0.062, 0.238),
        (0.048, 0.255),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.052, 0.011),
        (0.071, 0.080),
        (0.074, 0.155),
        (0.068, 0.205),
        (0.055, 0.238),
        (0.043, 0.249),
    ]
    body_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=72),
        "kettle_body_shell",
    )
    body.visual(body_shell, material=steel, name="body_shell")
    body.visual(
        Cylinder(radius=0.048, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=steel,
        name="body_bottom_plate",
    )

    spout_geom = tube_from_spline_points(
        [
            (0.074, 0.000, 0.145),
            (0.102, 0.000, 0.158),
            (0.150, 0.000, 0.202),
            (0.205, 0.000, 0.228),
            (0.248, 0.000, 0.220),
            (0.280, 0.000, 0.198),
        ],
        radius=0.007,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    spout_geom.merge(
        CylinderGeometry(radius=0.0052, height=0.030, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.276, 0.000, 0.199)
    )
    body.visual(
        mesh_from_geometry(spout_geom, "kettle_gooseneck_spout"),
        material=steel,
        name="gooseneck_spout",
    )

    handle_profile = rounded_rect_profile(0.018, 0.030, 0.005, corner_segments=6)
    handle_geom = sweep_profile_along_spline(
        [
            (-0.034, 0.094, 0.206),
            (-0.046, 0.112, 0.186),
            (-0.058, 0.120, 0.148),
            (-0.058, 0.118, 0.108),
            (-0.048, 0.108, 0.082),
            (-0.034, 0.096, 0.072),
        ],
        profile=handle_profile,
        samples_per_segment=18,
        cap_profile=True,
    )
    handle_geom.merge(BoxGeometry((0.022, 0.022, 0.018)).translate(-0.026, 0.064, 0.205))
    handle_geom.merge(BoxGeometry((0.022, 0.022, 0.018)).translate(-0.022, 0.067, 0.064))
    handle_geom.merge(BoxGeometry((0.028, 0.034, 0.028)).translate(-0.034, 0.088, 0.207))
    handle_geom.merge(BoxGeometry((0.020, 0.024, 0.010)).translate(-0.034, 0.089, 0.219))
    handle_geom.merge(BoxGeometry((0.028, 0.036, 0.032)).translate(-0.032, 0.090, 0.067))
    body.visual(
        mesh_from_geometry(handle_geom, "kettle_handle"),
        material=black_plastic,
        name="handle_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.165, 0.125, 0.255)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
    )

    model.articulation(
        "base_to_body",
        ArticulationType.FIXED,
        parent=power_base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(xyz=(0.044, 0.0, 0.003)),
        material=steel,
        name="lid_plate",
    )
    lid.visual(
        mesh_from_geometry(
            DomeGeometry(radius=0.037, radial_segments=30, height_segments=12).translate(0.044, 0.0, 0.006),
            "kettle_lid_dome",
        ),
        material=steel,
        name="lid_dome",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.044, 0.0, 0.013)),
        material=dark_trim,
        name="lid_knob_stem",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.044, 0.0, 0.023)),
        material=dark_trim,
        name="lid_knob",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.094, 0.094, 0.034)),
        mass=0.12,
        origin=Origin(xyz=(0.044, 0.0, 0.017)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.041, 0.0, 0.255)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    power_lever = model.part("power_lever")
    power_lever.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="lever_pivot_barrel",
    )
    power_lever.visual(
        Box((0.020, 0.010, 0.034)),
        origin=Origin(xyz=(0.010, 0.001, -0.020)),
        material=dark_trim,
        name="lever_paddle",
    )
    power_lever.inertial = Inertial.from_geometry(
        Box((0.024, 0.014, 0.038)),
        mass=0.03,
        origin=Origin(xyz=(0.010, 0.0, -0.018)),
    )

    model.articulation(
        "body_to_power_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_lever,
        origin=Origin(xyz=(-0.024, 0.114, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=math.radians(-28.0),
            upper=math.radians(20.0),
        ),
    )

    lid_release_plunger = model.part("lid_release_plunger")
    lid_release_plunger.visual(
        Box((0.018, 0.024, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_trim,
        name="plunger_cap",
    )
    lid_release_plunger.visual(
        Box((0.010, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_trim,
        name="plunger_ridge",
    )
    lid_release_plunger.inertial = Inertial.from_geometry(
        Box((0.018, 0.024, 0.014)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    model.articulation(
        "body_to_lid_release_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid_release_plunger,
        origin=Origin(xyz=(-0.034, 0.089, 0.224)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    power_base = object_model.get_part("power_base")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    power_lever = object_model.get_part("power_lever")
    plunger = object_model.get_part("lid_release_plunger")

    lid_hinge = object_model.get_articulation("body_to_lid")
    lever_joint = object_model.get_articulation("body_to_power_lever")
    plunger_joint = object_model.get_articulation("body_to_lid_release_plunger")

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

    ctx.expect_contact(body, power_base, name="kettle body sits on the power base")
    ctx.expect_contact(power_lever, body, name="power lever is seated on the handle housing")
    ctx.expect_contact(plunger, body, name="lid release plunger sits on its guide housing")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.003,
            max_penetration=0.0,
            name="closed lid seats on the fill opening",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.04,
            name="closed lid covers the opening footprint",
        )

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.035,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    with ctx.pose({lever_joint: lever_joint.motion_limits.lower}):
        lever_low_center = aabb_center(ctx.part_world_aabb(power_lever))
    with ctx.pose({lever_joint: lever_joint.motion_limits.upper}):
        lever_high_center = aabb_center(ctx.part_world_aabb(power_lever))
    ctx.check(
        "power lever toggles around its short pivot",
        lever_low_center is not None
        and lever_high_center is not None
        and abs(lever_high_center[0] - lever_low_center[0]) > 0.008
        and abs(lever_high_center[2] - lever_low_center[2]) > 0.004,
        details=f"low={lever_low_center}, high={lever_high_center}",
    )

    plunger_rest = ctx.part_world_position(plunger)
    with ctx.pose({plunger_joint: plunger_joint.motion_limits.upper}):
        plunger_extended = ctx.part_world_position(plunger)
    ctx.check(
        "lid release plunger translates along the upper handle guide",
        plunger_rest is not None
        and plunger_extended is not None
        and plunger_extended[2] > plunger_rest[2] + 0.004
        and abs(plunger_extended[0] - plunger_rest[0]) < 0.001
        and abs(plunger_extended[1] - plunger_rest[1]) < 0.001,
        details=f"rest={plunger_rest}, extended={plunger_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
