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
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_loop(
    x: float,
    width: float,
    height: float,
    radius: float,
    *,
    y_center: float = 0.0,
    z_center: float = 0.0,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    return [
        (x, y_center + y, z_center + z)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def _build_spout_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                _yz_loop(0.086, 0.048, 0.040, 0.010, z_center=0.154),
                _yz_loop(0.108, 0.044, 0.036, 0.010, z_center=0.159),
                _yz_loop(0.132, 0.030, 0.024, 0.008, z_center=0.164),
                _yz_loop(0.153, 0.018, 0.013, 0.005, z_center=0.169),
            ]
        ),
        "kettle_spout",
    )


def _build_lid_cap_mesh():
    lid_geom = CylinderGeometry(radius=0.046, height=0.006, radial_segments=48)
    lid_geom.translate(0.052, 0.0, -0.002)

    lid_dome = DomeGeometry(radius=0.046, radial_segments=48, height_segments=12, closed=True)
    lid_dome.scale(1.0, 1.0, 0.22)
    lid_dome.translate(0.052, 0.0, 0.001)
    lid_geom.merge(lid_dome)

    return mesh_from_geometry(lid_geom, "kettle_lid_cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_kettle")

    body_white = model.material("body_white", rgba=(0.94, 0.94, 0.92, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.72, 0.74, 0.77, 1.0))
    handle_black = model.material("handle_black", rgba=(0.12, 0.12, 0.13, 1.0))
    switch_black = model.material("switch_black", rgba=(0.07, 0.07, 0.08, 1.0))
    button_gray = model.material("button_gray", rgba=(0.48, 0.50, 0.53, 1.0))

    body = model.part("body")
    body_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0, 0.000),
                (0.046, 0.000),
                (0.074, 0.020),
                (0.090, 0.086),
                (0.092, 0.146),
                (0.080, 0.188),
                (0.062, 0.210),
                (0.053, 0.218),
            ],
            [
                (0.0, 0.006),
                (0.036, 0.010),
                (0.066, 0.024),
                (0.080, 0.086),
                (0.081, 0.146),
                (0.069, 0.187),
                (0.050, 0.208),
                (0.042, 0.214),
            ],
            segments=64,
        ),
        "kettle_body_shell",
    )
    body.visual(body_shell, material=body_white, name="body_shell")
    body.visual(
        Cylinder(radius=0.077, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim_gray,
        name="base_ring",
    )
    body.visual(
        Cylinder(radius=0.084, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=handle_black,
        name="foot_ring",
    )
    body.visual(_build_spout_mesh(), material=body_white, name="spout")
    body.visual(
        Box((0.018, 0.032, 0.028)),
        origin=Origin(xyz=(-0.101, 0.0, 0.058)),
        material=body_white,
        name="lower_handle_boss",
    )
    body.visual(
        Box((0.018, 0.030, 0.020)),
        origin=Origin(xyz=(-0.065, 0.0, 0.220)),
        material=body_white,
        name="upper_handle_boss",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(-0.118, 0.0, 0.082), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="handle_lower_curve",
    )
    body.visual(
        Box((0.016, 0.030, 0.104)),
        origin=Origin(xyz=(-0.128, 0.0, 0.140)),
        material=handle_black,
        name="handle_grip_spine",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(-0.116, 0.0, 0.198), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="handle_upper_curve",
    )
    body.visual(
        Box((0.040, 0.030, 0.020)),
        origin=Origin(xyz=(-0.094, 0.0, 0.214)),
        material=handle_black,
        name="handle_top_bridge",
    )
    body.visual(
        Box((0.028, 0.028, 0.020)),
        origin=Origin(xyz=(-0.116, 0.0, 0.070)),
        material=handle_black,
        name="switch_support_block",
    )
    body.visual(
        Box((0.008, 0.034, 0.026)),
        origin=Origin(xyz=(-0.147, 0.0, 0.076)),
        material=handle_black,
        name="switch_backplate",
    )
    body.visual(
        Box((0.013, 0.004, 0.022)),
        origin=Origin(xyz=(-0.1365, -0.016, 0.076)),
        material=handle_black,
        name="left_switch_web",
    )
    body.visual(
        Box((0.013, 0.004, 0.022)),
        origin=Origin(xyz=(-0.1365, 0.016, 0.076)),
        material=handle_black,
        name="right_switch_web",
    )
    body.visual(
        Box((0.014, 0.008, 0.026)),
        origin=Origin(xyz=(-0.136, -0.020, 0.076)),
        material=handle_black,
        name="left_switch_cheek",
    )
    body.visual(
        Box((0.014, 0.008, 0.026)),
        origin=Origin(xyz=(-0.136, 0.020, 0.076)),
        material=handle_black,
        name="right_switch_cheek",
    )
    body.visual(
        Box((0.008, 0.006, 0.020)),
        origin=Origin(xyz=(-0.139, -0.019, 0.076)),
        material=handle_black,
        name="left_switch_lug",
    )
    body.visual(
        Box((0.008, 0.006, 0.020)),
        origin=Origin(xyz=(-0.139, 0.019, 0.076)),
        material=handle_black,
        name="right_switch_lug",
    )
    body.visual(
        Box((0.026, 0.026, 0.008)),
        origin=Origin(xyz=(-0.090, 0.0, 0.227)),
        material=handle_black,
        name="button_housing",
    )
    body.visual(
        Box((0.018, 0.006, 0.014)),
        origin=Origin(xyz=(-0.090, -0.012, 0.238)),
        material=handle_black,
        name="left_button_rail",
    )
    body.visual(
        Box((0.018, 0.006, 0.014)),
        origin=Origin(xyz=(-0.090, 0.012, 0.238)),
        material=handle_black,
        name="right_button_rail",
    )
    body.visual(
        Box((0.020, 0.066, 0.010)),
        origin=Origin(xyz=(-0.041, 0.0, 0.218)),
        material=body_white,
        name="rear_hinge_bridge",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(-0.033, -0.019, 0.224), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(-0.033, 0.019, 0.224), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="right_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.196, 0.196, 0.226)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
    )

    lid = model.part("lid")
    lid.visual(_build_lid_cap_mesh(), material=body_white, name="lid_shell")
    lid.visual(
        Box((0.046, 0.028, 0.010)),
        origin=Origin(xyz=(0.023, 0.0, -0.003)),
        material=body_white,
        name="lid_hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=0.0042, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="lid_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.018),
        mass=0.10,
        origin=Origin(xyz=(0.030, 0.0, -0.006)),
    )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Cylinder(radius=0.003, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=switch_black,
        name="switch_axle",
    )
    power_switch.visual(
        Box((0.010, 0.020, 0.032)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=switch_black,
        name="switch_paddle",
    )
    power_switch.inertial = Inertial.from_geometry(
        Box((0.014, 0.028, 0.032)),
        mass=0.02,
        origin=Origin(xyz=(-0.008, 0.0, 0.0)),
    )

    lid_release_button = model.part("lid_release_button")
    lid_release_button.visual(
        Box((0.018, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=button_gray,
        name="button_cap",
    )
    lid_release_button.visual(
        Box((0.008, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=button_gray,
        name="button_stem",
    )
    lid_release_button.inertial = Inertial.from_geometry(
        Box((0.018, 0.020, 0.016)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.033, 0.0, 0.224)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "handle_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=power_switch,
        origin=Origin(xyz=(-0.139, 0.0, 0.076)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=-0.32,
            upper=0.32,
        ),
    )
    model.articulation(
        "handle_to_lid_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid_release_button,
        origin=Origin(xyz=(-0.090, 0.0, 0.246)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    power_switch = object_model.get_part("power_switch")
    lid_release_button = object_model.get_part("lid_release_button")

    lid_hinge = object_model.get_articulation("body_to_lid")
    switch_joint = object_model.get_articulation("handle_to_power_switch")
    button_joint = object_model.get_articulation("handle_to_lid_release_button")

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
        "all prompt-critical parts exist",
        all(part is not None for part in (body, lid, power_switch, lid_release_button)),
        details=f"parts={[part.name for part in (body, lid, power_switch, lid_release_button)]}",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="lid_knuckle",
        elem_b="left_hinge_barrel",
        contact_tol=0.0015,
        name="lid is physically captured at the rear hinge",
    )
    ctx.expect_contact(
        power_switch,
        body,
        elem_a="switch_axle",
        elem_b="left_switch_lug",
        name="power switch axle is supported by handle lugs",
    )
    ctx.expect_contact(
        lid_release_button,
        body,
        elem_a="button_stem",
        elem_b="left_button_rail",
        name="lid release button stem is guided by handle rails",
    )

    ctx.check(
        "articulation axes match kettle mechanisms",
        lid_hinge.axis == (0.0, -1.0, 0.0)
        and switch_joint.axis == (0.0, -1.0, 0.0)
        and button_joint.axis == (0.0, 0.0, -1.0),
        details=(
            f"lid_axis={lid_hinge.axis}, "
            f"switch_axis={switch_joint.axis}, "
            f"button_axis={button_joint.axis}"
        ),
    )

    lid_open_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    button_press_upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
    switch_lower = switch_joint.motion_limits.lower if switch_joint.motion_limits is not None else None
    switch_upper = switch_joint.motion_limits.upper if switch_joint.motion_limits is not None else None

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid seats on the rim without sinking into the body",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.070,
            name="closed lid covers the top opening",
        )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    with ctx.pose({lid_hinge: lid_open_upper or 0.0}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    ctx.check(
        "lid opens upward from rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.035,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    button_rest_aabb = ctx.part_element_world_aabb(lid_release_button, elem="button_cap")
    with ctx.pose({button_joint: button_press_upper or 0.0}):
        button_pressed_aabb = ctx.part_element_world_aabb(lid_release_button, elem="button_cap")
    ctx.check(
        "lid release button travels downward on its vertical guide",
        button_rest_aabb is not None
        and button_pressed_aabb is not None
        and button_pressed_aabb[1][2] < button_rest_aabb[1][2] - 0.003,
        details=f"rest={button_rest_aabb}, pressed={button_pressed_aabb}",
    )

    with ctx.pose({switch_joint: switch_lower or 0.0}):
        switch_low_aabb = ctx.part_element_world_aabb(power_switch, elem="switch_paddle")
    with ctx.pose({switch_joint: switch_upper or 0.0}):
        switch_high_aabb = ctx.part_element_world_aabb(power_switch, elem="switch_paddle")
    ctx.check(
        "rocker switch tips through its transverse pivot",
        switch_low_aabb is not None
        and switch_high_aabb is not None
        and abs(switch_high_aabb[1][2] - switch_low_aabb[1][2]) > 0.002,
        details=f"low={switch_low_aabb}, high={switch_high_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
