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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _spout_loop(
    x: float,
    width: float,
    z_bottom: float,
    z_mid: float,
    z_top: float,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    shoulder = width * 0.42
    return [
        (x, -half_w, z_bottom),
        (x, half_w, z_bottom),
        (x, shoulder, z_mid),
        (x, 0.0, z_top),
        (x, -shoulder, z_mid),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_body_electric_kettle")

    charcoal = model.material("charcoal", rgba=(0.12, 0.12, 0.13, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    switch_gray = model.material("switch_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.84, 0.94, 0.98, 0.35))

    body = model.part("body")

    glass_shell = _save_mesh(
        "kettle_glass_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.071, 0.000),
                (0.073, 0.008),
                (0.075, 0.028),
                (0.076, 0.075),
                (0.075, 0.118),
                (0.073, 0.138),
                (0.072, 0.145),
            ],
            inner_profile=[
                (0.067, 0.004),
                (0.069, 0.018),
                (0.0715, 0.075),
                (0.070, 0.128),
                (0.068, 0.141),
            ],
            segments=72,
        ),
    )
    top_collar = _save_mesh(
        "kettle_top_collar",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.075, 0.000),
                (0.081, 0.004),
                (0.082, 0.010),
                (0.082, 0.018),
                (0.078, 0.020),
            ],
            inner_profile=[
                (0.072, 0.002),
                (0.0735, 0.014),
                (0.071, 0.018),
            ],
            segments=72,
        ),
    )
    spout_mesh = _save_mesh(
        "kettle_spout",
        section_loft(
            [
                _spout_loop(0.072, 0.044, 0.160, 0.182, 0.194),
                _spout_loop(0.092, 0.036, 0.163, 0.184, 0.194),
                _spout_loop(0.112, 0.024, 0.166, 0.186, 0.193),
            ]
        ),
    )

    body.visual(
        Cylinder(radius=0.086, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=charcoal,
        name="base_skirt",
    )
    body.visual(
        Cylinder(radius=0.078, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=trim_silver,
        name="heater_band",
    )
    body.visual(
        Cylinder(radius=0.074, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=charcoal,
        name="lower_support_ring",
    )
    body.visual(
        Cylinder(radius=0.069, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=trim_silver,
        name="heater_plate",
    )
    body.visual(
        glass_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=glass_tint,
        name="glass_shell",
    )
    body.visual(
        top_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.183)),
        material=black_plastic,
        name="top_collar",
    )
    body.visual(
        Box((0.014, 0.054, 0.032)),
        origin=Origin(xyz=(-0.082, 0.0, 0.036)),
        material=black_plastic,
        name="lower_handle_boss",
    )
    body.visual(
        Box((0.018, 0.034, 0.030)),
        origin=Origin(xyz=(-0.089, 0.0, 0.198)),
        material=black_plastic,
        name="rear_hinge_housing",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(-0.078, -0.016, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="left_hinge_ear",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(-0.078, 0.016, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="right_hinge_ear",
    )
    body.visual(spout_mesh, material=black_plastic, name="spout")
    body.inertial = Inertial.from_geometry(
        Box((0.190, 0.190, 0.230)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    handle = model.part("handle")
    handle_arch = _save_mesh(
        "kettle_handle_arch",
        sweep_profile_along_spline(
            [
                (-0.116, 0.0, 0.036),
                (-0.134, 0.0, 0.070),
                (-0.144, 0.0, 0.114),
                (-0.146, 0.0, 0.160),
                (-0.142, 0.0, 0.198),
                (-0.138, 0.0, 0.214),
            ],
            profile=rounded_rect_profile(0.046, 0.030, radius=0.011, corner_segments=8),
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
    )
    handle.visual(handle_arch, material=black_plastic, name="handle_arch")
    handle.visual(
        Cylinder(radius=0.011, length=0.052),
        origin=Origin(xyz=(-0.100, 0.0, 0.036), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="lower_mount",
    )
    handle.visual(
        Cylinder(radius=0.011, length=0.052),
        origin=Origin(xyz=(-0.105, 0.0, 0.204), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="upper_mount",
    )
    handle.visual(
        Box((0.022, 0.036, 0.028)),
        origin=Origin(xyz=(-0.112, 0.0, 0.080)),
        material=black_plastic,
        name="switch_cradle",
    )
    handle.visual(
        Box((0.032, 0.056, 0.022)),
        origin=Origin(xyz=(-0.120, 0.0, 0.225)),
        material=black_plastic,
        name="thumb_pad_housing",
    )
    handle.visual(
        Box((0.040, 0.030, 0.028)),
        origin=Origin(xyz=(-0.118, 0.0, 0.204)),
        material=black_plastic,
        name="upper_socket",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.060, 0.080, 0.210)),
        mass=0.38,
        origin=Origin(xyz=(-0.114, 0.0, 0.130)),
    )

    lid = model.part("lid")
    lid_shell = _save_mesh(
        "kettle_lid_shell",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.000, 0.024),
                (0.018, 0.027),
                (0.044, 0.024),
                (0.064, 0.016),
                (0.078, 0.006),
                (0.079, 0.000),
            ],
            inner_profile=[
                (0.000, 0.020),
                (0.016, 0.023),
                (0.040, 0.020),
                (0.060, 0.013),
                (0.074, 0.005),
                (0.075, 0.002),
            ],
            segments=72,
        ),
    )
    lid.visual(
        lid_shell,
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material=black_plastic,
        name="lid_shell",
    )
    lid.visual(
        Box((0.022, 0.018, 0.012)),
        origin=Origin(xyz=(0.011, 0.0, 0.008)),
        material=black_plastic,
        name="hinge_saddle",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.078, 0.0, 0.028)),
        material=switch_gray,
        name="lid_knob",
    )
    lid.visual(
        Cylinder(radius=0.005, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="hinge_knuckle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.160, 0.160, 0.040)),
        mass=0.24,
        origin=Origin(xyz=(0.086, 0.0, 0.015)),
    )

    power_toggle = model.part("power_toggle")
    power_toggle.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=switch_gray,
        name="toggle_barrel",
    )
    power_toggle.visual(
        Box((0.012, 0.022, 0.022)),
        origin=Origin(xyz=(-0.010, 0.0, 0.014)),
        material=switch_gray,
        name="toggle_body",
    )
    power_toggle.visual(
        Box((0.012, 0.020, 0.012)),
        origin=Origin(xyz=(-0.014, 0.0, 0.029)),
        material=charcoal,
        name="toggle_tip",
    )
    power_toggle.inertial = Inertial.from_geometry(
        Box((0.032, 0.028, 0.042)),
        mass=0.04,
        origin=Origin(xyz=(-0.010, 0.0, 0.017)),
    )

    lid_button = model.part("lid_button")
    lid_button.visual(
        Box((0.018, 0.032, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=switch_gray,
        name="button_cap",
    )
    lid_button.visual(
        Box((0.012, 0.024, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=trim_silver,
        name="button_crown",
    )
    lid_button.inertial = Inertial.from_geometry(
        Box((0.018, 0.032, 0.008)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.FIXED,
        parent=body,
        child=handle,
        origin=Origin(),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.078, 0.0, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "handle_to_power_toggle",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=power_toggle,
        origin=Origin(xyz=(-0.129, 0.0, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "handle_to_lid_button",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=lid_button,
        origin=Origin(xyz=(-0.120, 0.0, 0.236)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0035,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    lid = object_model.get_part("lid")
    power_toggle = object_model.get_part("power_toggle")
    lid_button = object_model.get_part("lid_button")

    lid_hinge = object_model.get_articulation("body_to_lid")
    toggle_joint = object_model.get_articulation("handle_to_power_toggle")
    button_joint = object_model.get_articulation("handle_to_lid_button")

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
        handle,
        body,
        elem_a="lower_mount",
        elem_b="lower_handle_boss",
        name="handle lower mount is seated on the rear body boss",
    )
    ctx.expect_contact(
        handle,
        body,
        elem_a="upper_socket",
        elem_b="rear_hinge_housing",
        name="handle upper mount is seated below the hinge housing",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="hinge_knuckle",
        elem_b="left_hinge_ear",
        name="lid hinge knuckle is captured by the rear hinge ear",
    )
    ctx.expect_contact(
        power_toggle,
        handle,
        elem_a="toggle_barrel",
        elem_b="switch_cradle",
        name="power toggle mounts against the lower handle cradle",
    )
    ctx.expect_contact(
        lid_button,
        handle,
        elem_a="button_cap",
        elem_b="thumb_pad_housing",
        name="lid release button sits proud on the handle crown",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="top_collar",
            max_gap=0.010,
            max_penetration=0.0,
            name="closed lid rests just above the top collar",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="top_collar",
            min_overlap=0.120,
            name="closed lid covers the opening footprint",
        )

    closed_knob = ctx.part_element_world_aabb(lid, elem="lid_knob")
    with ctx.pose({lid_hinge: math.radians(72.0)}):
        open_knob = ctx.part_element_world_aabb(lid, elem="lid_knob")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_knob is not None
        and open_knob is not None
        and open_knob[0][2] > closed_knob[1][2] + 0.030,
        details=f"closed_knob={closed_knob}, open_knob={open_knob}",
    )

    toggle_rest = ctx.part_element_world_aabb(power_toggle, elem="toggle_tip")
    with ctx.pose({toggle_joint: 0.28}):
        toggle_on = ctx.part_element_world_aabb(power_toggle, elem="toggle_tip")
    ctx.check(
        "power toggle rotates on its handle hinge",
        toggle_rest is not None
        and toggle_on is not None
        and toggle_on[1][0] > toggle_rest[1][0] + 0.004
        and toggle_on[1][2] > toggle_rest[1][2] + 0.001,
        details=f"toggle_rest={toggle_rest}, toggle_on={toggle_on}",
    )

    button_rest = ctx.part_world_position(lid_button)
    with ctx.pose({button_joint: 0.0035}):
        button_pressed = ctx.part_world_position(lid_button)
    ctx.check(
        "lid release button depresses into the top of the handle",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[2] < button_rest[2] - 0.003,
        details=f"button_rest={button_rest}, button_pressed={button_pressed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
