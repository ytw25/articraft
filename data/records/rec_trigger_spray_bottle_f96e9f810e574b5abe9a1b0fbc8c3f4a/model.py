from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _x_cylinder_mesh(radius: float, length: float, xyz: tuple[float, float, float]):
    return CylinderGeometry(radius, length).rotate_y(pi / 2.0).translate(*xyz)


def _x_cone_mesh(radius: float, length: float, xyz: tuple[float, float, float]):
    return ConeGeometry(radius, length).rotate_y(pi / 2.0).translate(*xyz)


def _build_bottle_mesh():
    outer_profile = [
        (0.040, 0.000),
        (0.045, 0.008),
        (0.047, 0.032),
        (0.047, 0.130),
        (0.044, 0.166),
        (0.038, 0.183),
        (0.031, 0.194),
        (0.024, 0.202),
        (0.0175, 0.209),
        (0.015, 0.215),
    ]
    inner_profile = [
        (0.000, 0.000),
        (0.036, 0.003),
        (0.041, 0.032),
        (0.041, 0.130),
        (0.0385, 0.166),
        (0.0335, 0.183),
        (0.0270, 0.194),
        (0.0200, 0.202),
        (0.0135, 0.209),
        (0.0110, 0.215),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_head_seal_flange_mesh():
    collar = LatheGeometry.from_shell_profiles(
        [
            (0.021, 0.000),
            (0.021, 0.004),
        ],
        [
            (0.0112, 0.000),
            (0.0112, 0.004),
        ],
        segments=60,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    return collar


def _build_head_collar_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0245, 0.004),
            (0.0245, 0.015),
            (0.0225, 0.023),
        ],
        [
            (0.0178, 0.004),
            (0.0172, 0.015),
            (0.0165, 0.023),
        ],
        segments=60,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )


def _build_trigger_body_mesh():
    profile = [
        (-0.010, 0.020),
        (0.004, 0.020),
        (0.006, 0.012),
        (0.004, 0.006),
        (0.000, -0.006),
        (-0.004, -0.024),
        (-0.008, -0.040),
        (-0.012, -0.046),
        (-0.016, -0.046),
        (-0.014, -0.030),
        (-0.012, -0.012),
        (-0.012, 0.004),
        (-0.011, 0.012),
    ]
    return ExtrudeGeometry.centered(profile, 0.018).rotate_x(pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_trigger_spray_bottle")

    bottle_mat = model.material("bottle_hdpe", rgba=(0.90, 0.92, 0.88, 0.94))
    head_mat = model.material("head_polymer", rgba=(0.18, 0.21, 0.18, 1.0))
    trigger_mat = model.material("trigger_polymer", rgba=(0.10, 0.10, 0.10, 1.0))
    seal_mat = model.material("seal_elastomer", rgba=(0.08, 0.08, 0.08, 1.0))
    metal_mat = model.material("stainless_hardware", rgba=(0.75, 0.78, 0.80, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_build_bottle_mesh(), "spray_bottle_shell"),
        material=bottle_mat,
        name="bottle_shell",
    )

    head = model.part("trigger_head")
    head.visual(
        mesh_from_geometry(_build_head_seal_flange_mesh(), "trigger_head_seal_flange"),
        material=seal_mat,
        name="seal_flange",
    )
    head.visual(
        mesh_from_geometry(_build_head_collar_mesh(), "trigger_head_collar"),
        material=head_mat,
        name="head_collar",
    )
    head.visual(
        Box((0.034, 0.032, 0.020)),
        origin=Origin(xyz=(0.018, 0.000, 0.018)),
        material=head_mat,
        name="rear_bridge",
    )
    head.visual(
        Box((0.104, 0.006, 0.030)),
        origin=Origin(xyz=(0.080, 0.016, 0.020)),
        material=head_mat,
        name="left_cheek",
    )
    head.visual(
        Box((0.104, 0.006, 0.030)),
        origin=Origin(xyz=(0.080, -0.016, 0.020)),
        material=head_mat,
        name="right_cheek",
    )
    head.visual(
        Box((0.116, 0.032, 0.012)),
        origin=Origin(xyz=(0.096, 0.000, 0.039)),
        material=head_mat,
        name="roof",
    )
    head.visual(
        Box((0.024, 0.024, 0.015)),
        origin=Origin(xyz=(0.131, 0.000, 0.028)),
        material=head_mat,
        name="front_bridge",
    )
    head.visual(
        mesh_from_geometry(_x_cylinder_mesh(0.010, 0.040, (0.163, 0.000, 0.032)), "nozzle_shroud"),
        material=head_mat,
        name="nozzle_shroud",
    )
    head.visual(
        mesh_from_geometry(_x_cone_mesh(0.006, 0.014, (0.190, 0.000, 0.032)), "nozzle_tip"),
        material=head_mat,
        name="nozzle_tip",
    )
    head.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.090, 0.015, 0.011), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="left_pivot_bearing",
    )
    head.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.090, -0.015, 0.011), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="right_pivot_bearing",
    )

    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_geometry(_build_trigger_body_mesh(), "trigger_body_shell"),
        material=trigger_mat,
        name="trigger_body",
    )
    trigger.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.000, 0.011, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="left_pivot_lug",
    )
    trigger.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.000, -0.011, 0.000), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="right_pivot_lug",
    )
    trigger.visual(
        Box((0.014, 0.018, 0.014)),
        origin=Origin(xyz=(-0.013, 0.000, -0.052)),
        material=trigger_mat,
        name="finger_pad",
    )
    trigger.visual(
        Box((0.012, 0.012, 0.008)),
        origin=Origin(xyz=(0.004, 0.000, 0.010)),
        material=trigger_mat,
        name="upper_shoe",
    )
    trigger.visual(
        Box((0.004, 0.003, 0.018)),
        origin=Origin(xyz=(0.001, 0.009, 0.010)),
        material=metal_mat,
        name="left_linkage_strip",
    )
    trigger.visual(
        Box((0.004, 0.003, 0.018)),
        origin=Origin(xyz=(0.001, -0.009, 0.010)),
        material=metal_mat,
        name="right_linkage_strip",
    )

    plunger = model.part("pump_plunger")
    plunger.visual(
        Box((0.006, 0.014, 0.012)),
        origin=Origin(xyz=(0.003, 0.000, 0.000)),
        material=metal_mat,
        name="push_pad",
    )
    plunger.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.012, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_mat,
        name="rod",
    )
    plunger.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(0.017, 0.000, 0.000), rpy=(0.0, pi / 2.0, 0.0)),
        material=seal_mat,
        name="seal_boot",
    )

    model.articulation(
        "bottle_to_head",
        ArticulationType.FIXED,
        parent=bottle,
        child=head,
        origin=Origin(xyz=(0.000, 0.000, 0.215)),
    )
    model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head,
        child=trigger,
        origin=Origin(xyz=(0.090, 0.000, 0.011)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=0.45,
        ),
    )
    model.articulation(
        "head_to_pump_plunger",
        ArticulationType.PRISMATIC,
        parent=head,
        child=plunger,
        origin=Origin(xyz=(0.100, 0.000, 0.019)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.08,
            lower=0.0,
            upper=0.008,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    head = object_model.get_part("trigger_head")
    trigger = object_model.get_part("trigger")
    plunger = object_model.get_part("pump_plunger")

    trigger_hinge = object_model.get_articulation("head_to_trigger")
    plunger_slide = object_model.get_articulation("head_to_pump_plunger")

    seal_flange = head.get_visual("seal_flange")
    left_pivot_lug = trigger.get_visual("left_pivot_lug")
    right_pivot_lug = trigger.get_visual("right_pivot_lug")
    upper_shoe = trigger.get_visual("upper_shoe")
    finger_pad = trigger.get_visual("finger_pad")
    left_bearing = head.get_visual("left_pivot_bearing")
    right_bearing = head.get_visual("right_pivot_bearing")
    push_pad = plunger.get_visual("push_pad")

    def aabb_center_x(aabb) -> float:
        return 0.5 * (aabb[0][0] + aabb[1][0])

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
        bottle,
        head,
        elem_b=seal_flange,
        name="sealed head flange lands on bottle finish",
    )
    ctx.expect_contact(
        trigger,
        head,
        elem_a=left_pivot_lug,
        elem_b=left_bearing,
        name="left pivot bearing supports trigger",
    )
    ctx.expect_contact(
        trigger,
        head,
        elem_a=right_pivot_lug,
        elem_b=right_bearing,
        name="right pivot bearing supports trigger",
    )
    ctx.expect_contact(
        trigger,
        plunger,
        elem_a=upper_shoe,
        elem_b=push_pad,
        name="trigger shoe bears on pump plunger at rest",
    )
    ctx.expect_within(
        plunger,
        head,
        axes="yz",
        margin=0.0,
        name="pump plunger stays protected inside head envelope",
    )

    closed_finger = ctx.part_element_world_aabb(trigger, elem=finger_pad)
    closed_push_pad = ctx.part_element_world_aabb(plunger, elem=push_pad)

    with ctx.pose({trigger_hinge: 0.35, plunger_slide: 0.006}):
        ctx.expect_contact(
            trigger,
            plunger,
            elem_a=upper_shoe,
            elem_b=push_pad,
            contact_tol=0.002,
            name="pressed trigger still drives plunger through visible linkage",
        )
        ctx.expect_gap(
            trigger,
            bottle,
            axis="x",
            min_gap=0.001,
            positive_elem=finger_pad,
            name="finger pad clears bottle when trigger is squeezed",
        )
        pressed_finger = ctx.part_element_world_aabb(trigger, elem=finger_pad)
        pressed_push_pad = ctx.part_element_world_aabb(plunger, elem=push_pad)

    ctx.check(
        "trigger pulls rearward under positive stroke",
        aabb_center_x(pressed_finger) < aabb_center_x(closed_finger) - 0.010,
        details=(
            f"closed_x={aabb_center_x(closed_finger):.4f}, "
            f"pressed_x={aabb_center_x(pressed_finger):.4f}"
        ),
    )
    ctx.check(
        "plunger advances forward under squeeze",
        aabb_center_x(pressed_push_pad) > aabb_center_x(closed_push_pad) + 0.004,
        details=(
            f"closed_x={aabb_center_x(closed_push_pad):.4f}, "
            f"pressed_x={aabb_center_x(pressed_push_pad):.4f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
