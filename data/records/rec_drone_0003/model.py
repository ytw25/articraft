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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hybrid_vtol_drone")

    composite = model.part("airframe")
    camera = model.part("nose_camera")

    body = model.material("body_gray", rgba=(0.38, 0.41, 0.45, 1.0))
    wing = model.material("wing_gray", rgba=(0.46, 0.49, 0.53, 1.0))
    carbon = model.material("carbon_black", rgba=(0.10, 0.11, 0.12, 1.0))
    prop = model.material("prop_gray", rgba=(0.18, 0.19, 0.20, 1.0))
    sensor = model.material("sensor_glass", rgba=(0.10, 0.14, 0.18, 0.80))
    accent = model.material("accent_orange", rgba=(0.86, 0.43, 0.14, 1.0))

    composite.visual(
        Box((0.42, 0.09, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=body,
        name="fuselage_shell",
    )
    composite.visual(
        Box((0.14, 0.06, 0.03)),
        origin=Origin(xyz=(0.01, 0.0, 0.165)),
        material=body,
        name="avionics_hump",
    )
    composite.visual(
        Box((0.028, 0.040, 0.020)),
        origin=Origin(xyz=(0.1965, 0.0, 0.051)),
        material=carbon,
        name="nose_mount",
    )

    composite.visual(
        Box((0.18, 0.26, 0.014)),
        origin=Origin(xyz=(0.02, 0.0, 0.112)),
        material=wing,
        name="center_wing",
    )
    composite.visual(
        Box((0.15, 0.18, 0.010)),
        origin=Origin(xyz=(0.03, 0.22, 0.110)),
        material=wing,
        name="left_wing_tip",
    )
    composite.visual(
        Box((0.15, 0.18, 0.010)),
        origin=Origin(xyz=(0.03, -0.22, 0.110)),
        material=wing,
        name="right_wing_tip",
    )

    composite.visual(
        Box((0.04, 0.64, 0.024)),
        origin=Origin(xyz=(0.08, 0.0, 0.145)),
        material=carbon,
        name="front_crossbar",
    )
    composite.visual(
        Box((0.04, 0.64, 0.024)),
        origin=Origin(xyz=(-0.05, 0.0, 0.145)),
        material=carbon,
        name="rear_crossbar",
    )

    motor_specs = (
        ("front_left", 0.08, 0.32),
        ("front_right", 0.08, -0.32),
        ("rear_left", -0.05, 0.32),
        ("rear_right", -0.05, -0.32),
    )
    for name, x_pos, y_pos in motor_specs:
        composite.visual(
            Cylinder(radius=0.016, length=0.026),
            origin=Origin(xyz=(x_pos, y_pos, 0.170)),
            material=accent,
            name=f"{name}_hub",
        )
        composite.visual(
            Cylinder(radius=0.018, length=0.034),
            origin=Origin(xyz=(x_pos, y_pos, 0.200)),
            material=carbon,
            name=f"{name}_motor",
        )
        composite.visual(
            Box((0.15, 0.012, 0.0035)),
            origin=Origin(xyz=(x_pos, y_pos, 0.21875)),
            material=prop,
            name=f"{name}_blade_long",
        )
        composite.visual(
            Box((0.012, 0.15, 0.0035)),
            origin=Origin(xyz=(x_pos, y_pos, 0.21875)),
            material=prop,
            name=f"{name}_blade_cross",
        )

    composite.visual(
        Box((0.12, 0.05, 0.06)),
        origin=Origin(xyz=(-0.225, 0.0, 0.165)),
        material=body,
        name="tail_boom",
    )
    composite.visual(
        Box((0.10, 0.012, 0.12)),
        origin=Origin(xyz=(-0.235, 0.0, 0.235)),
        material=wing,
        name="vertical_fin",
    )
    composite.visual(
        Box((0.09, 0.12, 0.010)),
        origin=Origin(xyz=(-0.255, 0.05, 0.195)),
        material=wing,
        name="left_tailplane",
    )
    composite.visual(
        Box((0.09, 0.12, 0.010)),
        origin=Origin(xyz=(-0.255, -0.05, 0.195)),
        material=wing,
        name="right_tailplane",
    )
    composite.visual(
        Box((0.055, 0.038, 0.038)),
        origin=Origin(xyz=(-0.308, 0.0, 0.18)),
        material=carbon,
        name="pusher_pylon",
    )
    composite.visual(
        Cylinder(radius=0.015, length=0.03),
        origin=Origin(xyz=(-0.336, 0.0, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent,
        name="pusher_hub",
    )
    composite.visual(
        Box((0.006, 0.18, 0.018)),
        origin=Origin(xyz=(-0.352, 0.0, 0.18)),
        material=prop,
        name="pusher_blade_horizontal",
    )
    composite.visual(
        Box((0.006, 0.018, 0.18)),
        origin=Origin(xyz=(-0.352, 0.0, 0.18)),
        material=prop,
        name="pusher_blade_vertical",
    )

    composite.visual(
        Cylinder(radius=0.006, length=0.28),
        origin=Origin(xyz=(0.0, 0.07, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=carbon,
        name="left_skid",
    )
    composite.visual(
        Cylinder(radius=0.006, length=0.28),
        origin=Origin(xyz=(0.0, -0.07, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=carbon,
        name="right_skid",
    )
    composite.visual(
        Cylinder(radius=0.004, length=0.14),
        origin=Origin(xyz=(0.075, 0.0, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=carbon,
        name="front_skid_crossbar",
    )
    composite.visual(
        Cylinder(radius=0.004, length=0.14),
        origin=Origin(xyz=(-0.075, 0.0, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=carbon,
        name="rear_skid_crossbar",
    )
    for name, x_pos, y_pos in (
        ("front_left_strut", 0.075, 0.04),
        ("front_right_strut", 0.075, -0.04),
        ("rear_left_strut", -0.075, 0.04),
        ("rear_right_strut", -0.075, -0.04),
    ):
        composite.visual(
            Box((0.012, 0.02, 0.058)),
            origin=Origin(xyz=(x_pos, y_pos, 0.047)),
            material=carbon,
            name=name,
        )

    composite.inertial = Inertial.from_geometry(
        Box((0.42, 0.16, 0.18)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    camera.visual(
        Cylinder(radius=0.0048, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=carbon,
        name="pivot_barrel",
    )
    camera.visual(
        Box((0.022, 0.014, 0.012)),
        origin=Origin(xyz=(0.014, 0.0, -0.002)),
        material=carbon,
        name="gimbal_fork",
    )
    camera.visual(
        Box((0.036, 0.012, 0.024)),
        origin=Origin(xyz=(0.031, 0.0, -0.017)),
        material=carbon,
        name="tilt_arm",
    )
    camera.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.056, 0.0, -0.032)),
        material=sensor,
        name="camera_body",
    )
    camera.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.074, 0.0, -0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=sensor,
        name="camera_lens",
    )
    camera.inertial = Inertial.from_geometry(
        Box((0.088, 0.05, 0.062)),
        mass=0.28,
        origin=Origin(xyz=(0.040, 0.0, -0.024)),
    )

    model.articulation(
        "camera_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=composite,
        child=camera,
        origin=Origin(xyz=(0.2150, 0.0, 0.051)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-0.45,
            upper=0.70,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    airframe = object_model.get_part("airframe")
    camera = object_model.get_part("nose_camera")
    camera_tilt = object_model.get_articulation("camera_tilt_joint")

    fuselage_shell = airframe.get_visual("fuselage_shell")
    nose_mount = airframe.get_visual("nose_mount")
    left_wing_tip = airframe.get_visual("left_wing_tip")
    right_wing_tip = airframe.get_visual("right_wing_tip")
    center_wing = airframe.get_visual("center_wing")
    front_crossbar = airframe.get_visual("front_crossbar")
    rear_crossbar = airframe.get_visual("rear_crossbar")
    front_left_hub = airframe.get_visual("front_left_hub")
    front_right_hub = airframe.get_visual("front_right_hub")
    rear_left_hub = airframe.get_visual("rear_left_hub")
    rear_right_hub = airframe.get_visual("rear_right_hub")
    front_left_motor = airframe.get_visual("front_left_motor")
    front_right_motor = airframe.get_visual("front_right_motor")
    rear_left_motor = airframe.get_visual("rear_left_motor")
    rear_right_motor = airframe.get_visual("rear_right_motor")
    left_tailplane = airframe.get_visual("left_tailplane")
    pusher_hub = airframe.get_visual("pusher_hub")
    left_skid = airframe.get_visual("left_skid")
    right_skid = airframe.get_visual("right_skid")
    front_left_strut = airframe.get_visual("front_left_strut")
    pivot_barrel = camera.get_visual("pivot_barrel")
    camera_body = camera.get_visual("camera_body")
    camera_lens = camera.get_visual("camera_lens")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.

    ctx.expect_gap(
        airframe,
        airframe,
        axis="y",
        positive_elem=left_wing_tip,
        negative_elem=fuselage_shell,
        min_gap=0.08,
        name="left_wing_projects_outboard_of_fuselage",
    )
    ctx.expect_gap(
        airframe,
        airframe,
        axis="y",
        positive_elem=fuselage_shell,
        negative_elem=right_wing_tip,
        min_gap=0.08,
        name="right_wing_projects_outboard_of_fuselage",
    )
    ctx.expect_overlap(
        airframe,
        airframe,
        axes="yz",
        elem_a=front_crossbar,
        elem_b=fuselage_shell,
        min_overlap=0.015,
        name="front_boom_crosses_fuselage",
    )
    ctx.expect_overlap(
        airframe,
        airframe,
        axes="yz",
        elem_a=rear_crossbar,
        elem_b=fuselage_shell,
        min_overlap=0.015,
        name="rear_boom_crosses_fuselage",
    )
    ctx.expect_gap(
        airframe,
        airframe,
        axis="x",
        positive_elem=front_crossbar,
        negative_elem=rear_crossbar,
        min_gap=0.08,
        name="front_and_rear_booms_are_separated_along_fuselage",
    )

    for hub_name, hub, bar in (
        ("front_left", front_left_hub, front_crossbar),
        ("front_right", front_right_hub, front_crossbar),
        ("rear_left", rear_left_hub, rear_crossbar),
        ("rear_right", rear_right_hub, rear_crossbar),
    ):
        ctx.expect_gap(
            airframe,
            airframe,
            axis="z",
            positive_elem=hub,
            negative_elem=bar,
            max_gap=0.001,
            max_penetration=0.001,
            name=f"{hub_name}_hub_sits_on_boom_tip",
        )

    for motor_name, motor in (
        ("front_left", front_left_motor),
        ("rear_left", rear_left_motor),
    ):
        ctx.expect_gap(
            airframe,
            airframe,
            axis="y",
            positive_elem=motor,
            negative_elem=fuselage_shell,
            min_gap=0.23,
            name=f"{motor_name}_motor_is_far_outboard",
        )
    for motor_name, motor in (
        ("front_right", front_right_motor),
        ("rear_right", rear_right_motor),
    ):
        ctx.expect_gap(
            airframe,
            airframe,
            axis="y",
            positive_elem=fuselage_shell,
            negative_elem=motor,
            min_gap=0.23,
            name=f"{motor_name}_motor_is_far_outboard",
        )

    ctx.expect_gap(
        airframe,
        airframe,
        axis="x",
        positive_elem=center_wing,
        negative_elem=left_tailplane,
        min_gap=0.11,
        name="tailplane_sits_behind_main_wing",
    )
    ctx.expect_gap(
        airframe,
        airframe,
        axis="x",
        positive_elem=fuselage_shell,
        negative_elem=pusher_hub,
        min_gap=0.09,
        name="pusher_prop_mount_sits_behind_rear_fuselage",
    )

    ctx.expect_gap(
        airframe,
        airframe,
        axis="z",
        positive_elem=fuselage_shell,
        negative_elem=left_skid,
        min_gap=0.03,
        name="left_skid_sits_below_fuselage",
    )
    ctx.expect_gap(
        airframe,
        airframe,
        axis="z",
        positive_elem=fuselage_shell,
        negative_elem=right_skid,
        min_gap=0.03,
        name="right_skid_sits_below_fuselage",
    )
    ctx.expect_overlap(
        airframe,
        airframe,
        axes="xy",
        elem_a=front_left_strut,
        elem_b=fuselage_shell,
        min_overlap=0.008,
        name="landing_gear_strut_attaches_into_fuselage",
    )

    ctx.expect_overlap(
        camera,
        airframe,
        axes="yz",
        elem_a=pivot_barrel,
        elem_b=nose_mount,
        min_overlap=0.008,
        name="camera_pivot_barrel_aligns_with_nose_mount",
    )
    ctx.expect_gap(
        camera,
        airframe,
        axis="x",
        positive_elem=pivot_barrel,
        negative_elem=nose_mount,
        max_gap=0.001,
        max_penetration=0.001,
        name="camera_pivot_barrel_seats_on_nose_mount",
    )
    ctx.expect_gap(
        camera,
        airframe,
        axis="x",
        positive_elem=camera_lens,
        negative_elem=fuselage_shell,
        min_gap=0.05,
        name="camera_lens_projects_ahead_of_nose",
    )
    ctx.expect_gap(
        airframe,
        camera,
        axis="z",
        positive_elem=fuselage_shell,
        negative_elem=camera_body,
        min_gap=0.02,
        name="camera_body_hangs_below_fuselage",
    )

    with ctx.pose({camera_tilt: -0.45}):
        ctx.expect_gap(
            camera,
            airframe,
            axis="x",
            positive_elem=pivot_barrel,
            negative_elem=nose_mount,
            max_gap=0.001,
            max_penetration=0.001,
            name="camera_mount_remains_seated_when_tilted_up",
        )
        ctx.expect_gap(
            camera,
            airframe,
            axis="x",
            positive_elem=camera_lens,
            negative_elem=fuselage_shell,
            min_gap=0.07,
            name="tilted_up_camera_still_looks_forward",
        )

    with ctx.pose({camera_tilt: 0.70}):
        ctx.expect_gap(
            camera,
            airframe,
            axis="x",
            positive_elem=pivot_barrel,
            negative_elem=nose_mount,
            max_gap=0.001,
            max_penetration=0.001,
            name="camera_mount_remains_seated_when_tilted_down",
        )
        ctx.expect_gap(
            camera,
            airframe,
            axis="x",
            positive_elem=camera_lens,
            negative_elem=fuselage_shell,
            min_gap=0.01,
            name="tilted_down_camera_stays_ahead_of_nose",
        )
        ctx.expect_gap(
            airframe,
            camera,
            axis="z",
            positive_elem=fuselage_shell,
            negative_elem=camera_body,
            min_gap=0.005,
            name="tilted_down_camera_remains_below_fuselage",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
