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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def x_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def y_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_trigger_spray_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.84, 0.87, 0.84, 0.78))
    head_polymer = model.material("head_polymer", rgba=(0.20, 0.22, 0.24, 1.0))
    trigger_polymer = model.material("trigger_polymer", rgba=(0.14, 0.15, 0.16, 1.0))
    safety_red = model.material("safety_red", rgba=(0.79, 0.16, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.65, 0.69, 1.0))
    housing_lift = 0.03

    bottle = model.part("bottle")
    bottle_shell = LatheGeometry.from_shell_profiles(
        [
            (0.028, 0.000),
            (0.040, 0.006),
            (0.042, 0.026),
            (0.042, 0.168),
            (0.039, 0.205),
            (0.031, 0.230),
            (0.022, 0.246),
            (0.016, 0.254),
            (0.016, 0.276),
        ],
        [
            (0.024, 0.003),
            (0.037, 0.010),
            (0.039, 0.028),
            (0.039, 0.165),
            (0.036, 0.201),
            (0.028, 0.225),
            (0.019, 0.241),
            (0.0125, 0.250),
            (0.0125, 0.272),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )
    bottle.visual(
        mesh_from_geometry(bottle_shell, "bottle_shell"),
        material=bottle_plastic,
        name="bottle_shell",
    )

    head = model.part("head_body")
    head.visual(
        Box((0.060, 0.004, 0.094)),
        origin=Origin(xyz=(0.008, 0.022, 0.014 + housing_lift)),
        material=head_polymer,
        name="left_side_plate",
    )
    head.visual(
        Box((0.060, 0.004, 0.094)),
        origin=Origin(xyz=(0.008, -0.022, 0.014 + housing_lift)),
        material=head_polymer,
        name="right_side_plate",
    )
    head.visual(
        Box((0.060, 0.040, 0.016)),
        origin=Origin(xyz=(0.008, 0.0, 0.058 + housing_lift)),
        material=head_polymer,
        name="top_bridge",
    )
    head.visual(
        Box((0.030, 0.040, 0.016)),
        origin=Origin(xyz=(0.046, 0.0, 0.056 + housing_lift)),
        material=head_polymer,
        name="nozzle_block",
    )
    head.visual(
        Cylinder(radius=0.0045, length=0.020),
        origin=x_cylinder_origin((0.068, 0.0, 0.056 + housing_lift)),
        material=trigger_polymer,
        name="nozzle_tube",
    )
    head.visual(
        Box((0.018, 0.040, 0.088)),
        origin=Origin(xyz=(-0.018, 0.0, 0.002 + housing_lift)),
        material=head_polymer,
        name="rear_spine",
    )
    head.visual(
        Box((0.010, 0.018, 0.002)),
        origin=Origin(xyz=(-0.004, 0.0, -0.009)),
        material=head_polymer,
        name="mount_pad",
    )
    head.visual(
        Box((0.004, 0.044, 0.024)),
        origin=Origin(xyz=(-0.002, 0.0, -0.026)),
        material=head_polymer,
        name="collar_front",
    )
    head.visual(
        Box((0.004, 0.044, 0.024)),
        origin=Origin(xyz=(-0.038, 0.0, -0.026)),
        material=head_polymer,
        name="collar_back",
    )
    head.visual(
        Box((0.036, 0.004, 0.024)),
        origin=Origin(xyz=(-0.020, 0.020, -0.026)),
        material=head_polymer,
        name="collar_left",
    )
    head.visual(
        Box((0.036, 0.004, 0.024)),
        origin=Origin(xyz=(-0.020, -0.020, -0.026)),
        material=head_polymer,
        name="collar_right",
    )
    head.visual(
        Box((0.018, 0.008, 0.022)),
        origin=Origin(xyz=(-0.010, 0.018, 0.012 + housing_lift)),
        material=head_polymer,
        name="brace_left",
    )
    head.visual(
        Box((0.018, 0.008, 0.022)),
        origin=Origin(xyz=(-0.010, -0.018, 0.012 + housing_lift)),
        material=head_polymer,
        name="brace_right",
    )
    head.visual(
        Box((0.016, 0.004, 0.028)),
        origin=Origin(xyz=(0.008, 0.011, 0.048 + housing_lift)),
        material=head_polymer,
        name="guide_left",
    )
    head.visual(
        Box((0.016, 0.004, 0.028)),
        origin=Origin(xyz=(0.008, -0.011, 0.048 + housing_lift)),
        material=head_polymer,
        name="guide_right",
    )
    head.visual(
        Box((0.060, 0.004, 0.010)),
        origin=Origin(xyz=(0.010, 0.022, -0.044 + housing_lift)),
        material=head_polymer,
        name="guard_left",
    )
    head.visual(
        Box((0.060, 0.004, 0.010)),
        origin=Origin(xyz=(0.010, -0.022, -0.044 + housing_lift)),
        material=head_polymer,
        name="guard_right",
    )
    head.visual(
        Box((0.010, 0.044, 0.010)),
        origin=Origin(xyz=(0.035, 0.0, -0.043 + housing_lift)),
        material=head_polymer,
        name="guard_front_bridge",
    )
    head.visual(
        Box((0.012, 0.044, 0.010)),
        origin=Origin(xyz=(-0.014, 0.0, -0.038 + housing_lift)),
        material=head_polymer,
        name="guard_rear_bridge",
    )
    head.visual(
        Box((0.008, 0.006, 0.062)),
        origin=Origin(xyz=(0.030, 0.019, -0.007 + housing_lift)),
        material=head_polymer,
        name="nose_strut_left",
    )
    head.visual(
        Box((0.008, 0.006, 0.062)),
        origin=Origin(xyz=(0.030, -0.019, -0.007 + housing_lift)),
        material=head_polymer,
        name="nose_strut_right",
    )
    head.visual(
        Box((0.006, 0.008, 0.012)),
        origin=Origin(xyz=(-0.008, 0.016, -0.015 + housing_lift)),
        material=head_polymer,
        name="trigger_stop_left",
    )
    head.visual(
        Box((0.006, 0.008, 0.012)),
        origin=Origin(xyz=(-0.008, -0.016, -0.015 + housing_lift)),
        material=head_polymer,
        name="trigger_stop_right",
    )
    head.visual(
        Box((0.012, 0.032, 0.008)),
        origin=Origin(xyz=(-0.017, 0.0, -0.009 + housing_lift)),
        material=head_polymer,
        name="trigger_stop_bridge",
    )
    head.visual(
        Cylinder(radius=0.004, length=0.002),
        origin=y_cylinder_origin((0.000, 0.025, 0.000 + housing_lift)),
        material=steel,
        name="pivot_bolt_left",
    )
    head.visual(
        Cylinder(radius=0.004, length=0.002),
        origin=y_cylinder_origin((0.000, -0.025, 0.000 + housing_lift)),
        material=steel,
        name="pivot_bolt_right",
    )
    head.visual(
        Cylinder(radius=0.003, length=0.002),
        origin=y_cylinder_origin((0.030, 0.025, 0.046 + housing_lift)),
        material=steel,
        name="housing_bolt_left",
    )
    head.visual(
        Cylinder(radius=0.003, length=0.002),
        origin=y_cylinder_origin((0.030, -0.025, 0.046 + housing_lift)),
        material=steel,
        name="housing_bolt_right",
    )
    head.visual(
        Cylinder(radius=0.003, length=0.002),
        origin=y_cylinder_origin((-0.010, 0.025, 0.012 + housing_lift)),
        material=steel,
        name="brace_bolt_left",
    )
    head.visual(
        Cylinder(radius=0.003, length=0.002),
        origin=y_cylinder_origin((-0.010, -0.025, 0.012 + housing_lift)),
        material=steel,
        name="brace_bolt_right",
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.004, length=0.036),
        origin=y_cylinder_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="pivot_tube",
    )
    trigger.visual(
        Box((0.012, 0.008, 0.028)),
        origin=Origin(xyz=(0.002, 0.016, -0.004)),
        material=trigger_polymer,
        name="left_yoke",
    )
    trigger.visual(
        Box((0.012, 0.008, 0.028)),
        origin=Origin(xyz=(0.002, -0.016, -0.004)),
        material=trigger_polymer,
        name="right_yoke",
    )
    trigger.visual(
        Box((0.018, 0.034, 0.060)),
        origin=Origin(xyz=(0.022, 0.0, -0.036)),
        material=trigger_polymer,
        name="trigger_blade",
    )
    trigger.visual(
        Box((0.014, 0.028, 0.044)),
        origin=Origin(xyz=(0.015, 0.0, -0.010)),
        material=trigger_polymer,
        name="blade_web",
    )
    trigger.visual(
        Box((0.010, 0.038, 0.018)),
        origin=Origin(xyz=(0.032, 0.0, -0.062)),
        material=trigger_polymer,
        name="finger_pad",
    )
    trigger.visual(
        Box((0.010, 0.032, 0.018)),
        origin=Origin(xyz=(0.026, 0.0, -0.047)),
        material=trigger_polymer,
        name="finger_neck",
    )
    trigger.visual(
        Box((0.024, 0.014, 0.028)),
        origin=Origin(xyz=(0.004, 0.0, 0.014)),
        material=trigger_polymer,
        name="upper_link",
    )
    trigger.visual(
        Box((0.012, 0.018, 0.010)),
        origin=Origin(xyz=(-0.004, 0.0, 0.022)),
        material=steel,
        name="drive_pad",
    )
    trigger.visual(
        Box((0.014, 0.020, 0.010)),
        origin=Origin(xyz=(-0.004, 0.0, -0.020)),
        material=trigger_polymer,
        name="stop_bar",
    )
    trigger.visual(
        Box((0.006, 0.018, 0.018)),
        origin=Origin(xyz=(0.006, 0.0, -0.017)),
        material=trigger_polymer,
        name="stop_rib",
    )

    plunger = model.part("pump_plunger")
    plunger.visual(
        Box((0.016, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=steel,
        name="cam_pad",
    )
    plunger.visual(
        Box((0.006, 0.010, 0.014)),
        origin=Origin(xyz=(-0.006, 0.013, -0.013)),
        material=steel,
        name="pump_tongue",
    )
    plunger.visual(
        Box((0.005, 0.008, 0.016)),
        origin=Origin(xyz=(-0.006, 0.013, -0.028)),
        material=steel,
        name="pump_rod",
    )

    lockout = model.part("safety_lockout")
    lockout.visual(
        Cylinder(radius=0.0025, length=0.004),
        origin=y_cylinder_origin((0.0, 0.0, 0.0)),
        material=steel,
        name="lock_pivot",
    )
    lockout.visual(
        Box((0.010, 0.006, 0.012)),
        origin=Origin(xyz=(0.000, 0.005, 0.000)),
        material=safety_red,
        name="left_ear",
    )
    lockout.visual(
        Box((0.010, 0.006, 0.012)),
        origin=Origin(xyz=(0.000, 0.005, 0.012)),
        material=safety_red,
        name="right_ear",
    )
    lockout.visual(
        Box((0.012, 0.006, 0.010)),
        origin=Origin(xyz=(-0.022, 0.005, 0.029)),
        material=safety_red,
        name="blocker",
    )
    lockout.visual(
        Box((0.014, 0.012, 0.018)),
        origin=Origin(xyz=(0.014, 0.010, 0.003)),
        material=safety_red,
        name="thumb_tab",
    )
    lockout.visual(
        Box((0.030, 0.006, 0.012)),
        origin=Origin(xyz=(-0.008, 0.005, 0.018)),
        material=safety_red,
        name="blocker_arm",
    )

    model.articulation(
        "bottle_to_head",
        ArticulationType.FIXED,
        parent=bottle,
        child=head,
        origin=Origin(xyz=(0.020, 0.0, 0.286)),
    )
    model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head,
        child=trigger,
        origin=Origin(xyz=(0.0, 0.0, housing_lift)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=0.72,
        ),
    )
    model.articulation(
        "head_to_plunger",
        ArticulationType.PRISMATIC,
        parent=head,
        child=plunger,
        origin=Origin(xyz=(0.008, 0.0, 0.072)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.20,
            lower=0.0,
            upper=0.010,
        ),
    )
    model.articulation(
        "head_to_lockout",
        ArticulationType.REVOLUTE,
        parent=head,
        child=lockout,
        origin=Origin(xyz=(-0.006, 0.026, 0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    head = object_model.get_part("head_body")
    trigger = object_model.get_part("trigger")
    plunger = object_model.get_part("pump_plunger")
    lockout = object_model.get_part("safety_lockout")

    trigger_joint = object_model.get_articulation("head_to_trigger")
    plunger_joint = object_model.get_articulation("head_to_plunger")
    lockout_joint = object_model.get_articulation("head_to_lockout")

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
        head,
        bottle,
        elem_a="mount_pad",
        elem_b="bottle_shell",
        name="head_mount_pad_is_supported_by_bottle",
    )
    ctx.expect_contact(
        trigger,
        head,
        elem_a="left_yoke",
        elem_b="left_side_plate",
        name="trigger_left_yoke_is_supported",
    )
    ctx.expect_contact(
        trigger,
        head,
        elem_a="right_yoke",
        elem_b="right_side_plate",
        name="trigger_right_yoke_is_supported",
    )
    ctx.expect_contact(
        plunger,
        head,
        elem_a="cam_pad",
        elem_b="guide_left",
        name="plunger_is_guided_on_left",
    )
    ctx.expect_contact(
        plunger,
        head,
        elem_a="cam_pad",
        elem_b="guide_right",
        name="plunger_is_guided_on_right",
    )
    ctx.expect_contact(
        lockout,
        head,
        elem_a="lock_pivot",
        elem_b="left_side_plate",
        name="lockout_pivot_is_supported",
    )

    with ctx.pose({trigger_joint: 0.0, plunger_joint: 0.0, lockout_joint: 0.0}):
        ctx.expect_gap(
            plunger,
            trigger,
            axis="z",
            positive_elem="cam_pad",
            negative_elem="drive_pad",
            min_gap=0.001,
            max_gap=0.008,
            name="rest_pose_has_small_trigger_to_pump_clearance",
        )
        ctx.expect_gap(
            lockout,
            trigger,
            axis="z",
            positive_elem="blocker",
            negative_elem="upper_link",
            min_gap=0.004,
            max_gap=0.036,
            name="open_lockout_clears_trigger_link",
        )

    with ctx.pose({trigger_joint: 0.62, plunger_joint: 0.009, lockout_joint: 0.0}):
        ctx.expect_contact(
            trigger,
            plunger,
            elem_a="drive_pad",
            elem_b="cam_pad",
            name="trigger_stroke_reaches_visible_pump_path",
        )
        ctx.expect_contact(
            trigger,
            head,
            elem_a="stop_bar",
            elem_b="trigger_stop_bridge",
            name="overtravel_stop_engages_under_full_pull",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
