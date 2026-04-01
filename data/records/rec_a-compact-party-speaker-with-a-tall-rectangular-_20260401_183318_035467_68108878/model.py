from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

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
    wire_from_points,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def _axis_matches(axis, target, tol: float = 1e-6) -> bool:
    return all(abs(axis[i] - target[i]) <= tol for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_party_speaker")

    cabinet_black = model.material("cabinet_black", rgba=(0.12, 0.13, 0.14, 1.0))
    grille_black = model.material("grille_black", rgba=(0.06, 0.06, 0.07, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.68, 0.70, 0.72, 1.0))
    amber = model.material("amber", rgba=(0.86, 0.52, 0.14, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.74, 0.74, 0.72, 1.0))

    cabinet_w = 0.32
    cabinet_d = 0.29
    body_base_h = 0.61
    top_rim_h = 0.05
    body_h = body_base_h + top_rim_h
    front_y = cabinet_d * 0.5
    top_z = body_h

    body = model.part("speaker_body")
    body.inertial = Inertial.from_geometry(
        Box((cabinet_w, cabinet_d, body_h)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )
    body.visual(
        Box((cabinet_w, cabinet_d, body_base_h)),
        origin=Origin(xyz=(0.0, 0.0, body_base_h * 0.5)),
        material=cabinet_black,
        name="cabinet_shell",
    )
    body.visual(
        Box((0.085, 0.18, top_rim_h)),
        origin=Origin(xyz=(-0.1175, 0.0, body_base_h + top_rim_h * 0.5)),
        material=cabinet_black,
        name="left_top_rail",
    )
    body.visual(
        Box((0.085, 0.18, top_rim_h)),
        origin=Origin(xyz=(0.1175, 0.0, body_base_h + top_rim_h * 0.5)),
        material=cabinet_black,
        name="right_top_rail",
    )
    body.visual(
        Box((0.15, 0.055, top_rim_h)),
        origin=Origin(xyz=(0.0, 0.1175, body_base_h + top_rim_h * 0.5)),
        material=cabinet_black,
        name="front_top_bridge",
    )
    body.visual(
        Box((0.15, 0.050, top_rim_h)),
        origin=Origin(xyz=(0.0, -0.120, body_base_h + top_rim_h * 0.5)),
        material=cabinet_black,
        name="rear_top_bridge",
    )
    body.visual(
        Box((0.276, 0.012, 0.475)),
        origin=Origin(xyz=(0.0, front_y + 0.002, 0.275)),
        material=grille_black,
        name="front_grille",
    )
    body.visual(
        Cylinder(radius=0.092, length=0.010),
        origin=Origin(xyz=(0.0, front_y + 0.006, 0.392), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="upper_driver_ring",
    )
    body.visual(
        Cylinder(radius=0.074, length=0.008),
        origin=Origin(xyz=(0.0, front_y + 0.008, 0.392), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grille_black,
        name="upper_driver_cone",
    )
    body.visual(
        Cylinder(radius=0.092, length=0.010),
        origin=Origin(xyz=(0.0, front_y + 0.006, 0.212), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="lower_driver_ring",
    )
    body.visual(
        Cylinder(radius=0.074, length=0.008),
        origin=Origin(xyz=(0.0, front_y + 0.008, 0.212), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grille_black,
        name="lower_driver_cone",
    )
    body.visual(
        Box((0.094, 0.008, 0.026)),
        origin=Origin(xyz=(0.0, front_y + 0.006, 0.520)),
        material=trim_dark,
        name="tweeter_slot",
    )
    body.visual(
        Box((0.180, 0.022, 0.098)),
        origin=Origin(xyz=(0.0, front_y + 0.005, 0.585)),
        material=trim_dark,
        name="control_bezel",
    )
    body.visual(
        Box((0.046, 0.004, 0.012)),
        origin=Origin(xyz=(-0.046, front_y + 0.016, 0.615)),
        material=satin_silver,
        name="knob_tick_window",
    )

    handle = model.part("carry_handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.17, 0.13, 0.05)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.055, 0.015)),
    )
    handle_frame = wire_from_points(
        [
            (-0.057, 0.0, 0.0),
            (-0.050, 0.040, 0.007),
            (-0.040, 0.082, 0.014),
            (-0.026, 0.112, 0.018),
            (0.026, 0.112, 0.018),
            (0.040, 0.082, 0.014),
            (0.050, 0.040, 0.007),
            (0.057, 0.0, 0.0),
        ],
        radius=0.0085,
        radial_segments=16,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.018,
        corner_segments=8,
    )
    handle.visual(
        mesh_from_geometry(handle_frame, "speaker_handle_frame"),
        material=trim_dark,
        name="handle_frame",
    )
    handle.visual(
        Box((0.120, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.110, 0.017)),
        material=warm_gray,
        name="grip_bar",
    )
    handle.visual(
        Cylinder(radius=0.0075, length=0.012),
        origin=Origin(xyz=(-0.069, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_dark,
        name="left_trunnion",
    )
    handle.visual(
        Cylinder(radius=0.0075, length=0.012),
        origin=Origin(xyz=(0.069, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_dark,
        name="right_trunnion",
    )

    knob = model.part("control_knob")
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.032),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    knob.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="knob_collar",
    )
    knob.visual(
        Cylinder(radius=0.021, length=0.026),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.029, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_silver,
        name="knob_face",
    )
    knob.visual(
        Box((0.004, 0.006, 0.011)),
        origin=Origin(xyz=(0.0, 0.029, 0.015)),
        material=amber,
        name="indicator_ridge",
    )

    mode_button = model.part("mode_button")
    mode_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.013),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0065, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    mode_button.visual(
        Cylinder(radius=0.013, length=0.003),
        origin=Origin(xyz=(0.0, 0.0015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="button_flange",
    )
    mode_button.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_gray,
        name="button_cap",
    )

    power_button = model.part("power_button")
    power_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.013),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0065, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
    )
    power_button.visual(
        Cylinder(radius=0.013, length=0.003),
        origin=Origin(xyz=(0.0, 0.0015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="button_flange",
    )
    power_button.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=amber,
        name="button_cap",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, -0.050, 0.621)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=radians(100.0),
        ),
    )
    model.articulation(
        "body_to_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(-0.045, 0.161, 0.585)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=4.0,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "body_to_mode_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=mode_button,
        origin=Origin(xyz=(0.058, 0.161, 0.607)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=0.004,
        ),
    )
    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(0.058, 0.161, 0.565)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("speaker_body")
    handle = object_model.get_part("carry_handle")
    knob = object_model.get_part("control_knob")
    mode_button = object_model.get_part("mode_button")
    power_button = object_model.get_part("power_button")

    handle_joint = object_model.get_articulation("body_to_handle")
    knob_joint = object_model.get_articulation("body_to_knob")
    mode_joint = object_model.get_articulation("body_to_mode_button")
    power_joint = object_model.get_articulation("body_to_power_button")

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

    ctx.expect_contact(handle, body, contact_tol=0.001, name="handle is supported by the top recess")
    ctx.expect_contact(
        knob,
        body,
        elem_a="knob_collar",
        elem_b="control_bezel",
        contact_tol=0.001,
        name="knob mounts flush to the control bezel",
    )
    ctx.expect_contact(
        mode_button,
        body,
        elem_a="button_flange",
        elem_b="control_bezel",
        contact_tol=0.001,
        name="mode button seats on the control bezel",
    )
    ctx.expect_contact(
        power_button,
        body,
        elem_a="button_flange",
        elem_b="control_bezel",
        contact_tol=0.001,
        name="power button seats on the control bezel",
    )

    ctx.check(
        "handle hinge axis spans the cabinet width",
        _axis_matches(handle_joint.axis, (1.0, 0.0, 0.0)),
        details=f"axis={handle_joint.axis}",
    )
    ctx.check(
        "knob rotates on a front-facing axis",
        _axis_matches(knob_joint.axis, (0.0, 1.0, 0.0)),
        details=f"axis={knob_joint.axis}",
    )
    ctx.check(
        "mode button plunges inward along the front normal",
        _axis_matches(mode_joint.axis, (0.0, -1.0, 0.0)),
        details=f"axis={mode_joint.axis}",
    )
    ctx.check(
        "power button plunges inward along the front normal",
        _axis_matches(power_joint.axis, (0.0, -1.0, 0.0)),
        details=f"axis={power_joint.axis}",
    )

    rest_grip = _aabb_center(ctx.part_element_world_aabb(handle, elem="grip_bar"))
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        open_grip = _aabb_center(ctx.part_element_world_aabb(handle, elem="grip_bar"))
    ctx.check(
        "handle lifts upward out of the recess",
        rest_grip is not None
        and open_grip is not None
        and open_grip[2] > rest_grip[2] + 0.06
        and open_grip[1] < rest_grip[1] - 0.10,
        details=f"rest={rest_grip}, open={open_grip}",
    )

    rest_indicator = _aabb_center(ctx.part_element_world_aabb(knob, elem="indicator_ridge"))
    with ctx.pose({knob_joint: 1.5}):
        turned_indicator = _aabb_center(ctx.part_element_world_aabb(knob, elem="indicator_ridge"))
    ctx.check(
        "knob indicator sweeps when the knob turns",
        rest_indicator is not None
        and turned_indicator is not None
        and abs(turned_indicator[0] - rest_indicator[0]) > 0.008,
        details=f"rest={rest_indicator}, turned={turned_indicator}",
    )

    rest_mode = ctx.part_world_position(mode_button)
    with ctx.pose({mode_joint: mode_joint.motion_limits.upper}):
        pressed_mode = ctx.part_world_position(mode_button)
    ctx.check(
        "mode button travels inward when pressed",
        rest_mode is not None
        and pressed_mode is not None
        and pressed_mode[1] < rest_mode[1] - 0.003
        and abs(pressed_mode[0] - rest_mode[0]) < 1e-6
        and abs(pressed_mode[2] - rest_mode[2]) < 1e-6,
        details=f"rest={rest_mode}, pressed={pressed_mode}",
    )

    rest_power = ctx.part_world_position(power_button)
    with ctx.pose({power_joint: power_joint.motion_limits.upper}):
        pressed_power = ctx.part_world_position(power_button)
    ctx.check(
        "power button travels inward when pressed",
        rest_power is not None
        and pressed_power is not None
        and pressed_power[1] < rest_power[1] - 0.003
        and abs(pressed_power[0] - rest_power[0]) < 1e-6
        and abs(pressed_power[2] - rest_power[2]) < 1e-6,
        details=f"rest={rest_power}, pressed={pressed_power}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
