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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_blimp")

    envelope_fabric = model.material("envelope_fabric", rgba=(0.88, 0.90, 0.92, 1.0))
    fin_red = model.material("fin_red", rgba=(0.73, 0.13, 0.14, 1.0))
    gondola_gray = model.material("gondola_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    mast_gray = model.material("mast_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    window_tint = model.material("window_tint", rgba=(0.20, 0.33, 0.40, 0.55))

    hull = model.part("hull")
    hull_profile = [
        (0.0, -4.35),
        (0.14, -4.18),
        (0.52, -3.82),
        (0.92, -3.18),
        (1.16, -2.10),
        (1.28, -0.60),
        (1.30, 0.55),
        (1.20, 1.85),
        (0.92, 3.00),
        (0.52, 3.92),
        (0.18, 4.28),
        (0.0, 4.46),
    ]
    hull_geom = LatheGeometry(hull_profile, segments=88).rotate_y(math.pi / 2.0)
    hull.visual(
        mesh_from_geometry(hull_geom, "blimp_envelope"),
        material=envelope_fabric,
        name="envelope",
    )
    hull.visual(
        Cylinder(radius=0.08, length=0.26),
        origin=Origin(xyz=(4.33, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fin_red,
        name="nose_cone",
    )
    hull.visual(
        Box((0.66, 0.08, 1.24)),
        origin=Origin(xyz=(-3.75, 0.0, 0.86)),
        material=fin_red,
        name="tail_fin",
    )
    hull.visual(
        Box((0.46, 0.08, 0.56)),
        origin=Origin(xyz=(-3.76, 0.0, -0.86)),
        material=fin_red,
        name="keel_fin",
    )
    hull.visual(
        Box((0.40, 1.04, 0.05)),
        origin=Origin(xyz=(-3.83, 0.62, 0.20)),
        material=fin_red,
        name="left_tailplane",
    )
    hull.visual(
        Box((0.40, 1.04, 0.05)),
        origin=Origin(xyz=(-3.83, -0.62, 0.20)),
        material=fin_red,
        name="right_tailplane",
    )
    hull.visual(
        Cylinder(radius=0.045, length=0.34),
        origin=Origin(xyz=(0.32, 0.0, -1.32)),
        material=mast_gray,
        name="mast",
    )
    hull.visual(
        Box((1.72, 0.76, 0.58)),
        origin=Origin(xyz=(0.50, 0.0, -1.78)),
        material=gondola_gray,
        name="cabin",
    )
    hull.visual(
        Box((0.48, 0.72, 0.22)),
        origin=Origin(xyz=(1.08, 0.0, -1.62)),
        material=window_tint,
        name="cockpit_glass",
    )
    hull.visual(
        Box((0.26, 0.20, 0.10)),
        origin=Origin(xyz=(-0.26, 0.0, -1.76)),
        material=mast_gray,
        name="engine_housing",
    )
    hull.inertial = Inertial.from_geometry(
        Box((8.92, 2.62, 2.62)),
        mass=420.0,
        origin=Origin(),
    )

    rudder = model.part("rudder")
    rudder.visual(
        Box((0.32, 0.05, 0.82)),
        origin=Origin(xyz=(-0.16, 0.0, 0.0)),
        material=fin_red,
        name="rudder_panel",
    )
    rudder.inertial = Inertial.from_geometry(
        Box((0.32, 0.05, 0.82)),
        mass=7.5,
        origin=Origin(xyz=(-0.16, 0.0, 0.0)),
    )
    model.articulation(
        "hull_to_rudder",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=rudder,
        origin=Origin(xyz=(-4.08, 0.0, 0.86)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.45,
        ),
    )

    left_elevator = model.part("left_elevator")
    left_elevator.visual(
        Box((0.28, 0.54, 0.035)),
        origin=Origin(xyz=(-0.14, 0.0, 0.0)),
        material=fin_red,
        name="elevator_panel",
    )
    left_elevator.inertial = Inertial.from_geometry(
        Box((0.28, 0.54, 0.035)),
        mass=5.0,
        origin=Origin(xyz=(-0.14, 0.0, 0.0)),
    )
    model.articulation(
        "hull_to_left_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=left_elevator,
        origin=Origin(xyz=(-4.03, 0.87, 0.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.2,
            lower=-0.30,
            upper=0.55,
        ),
    )

    right_elevator = model.part("right_elevator")
    right_elevator.visual(
        Box((0.28, 0.54, 0.035)),
        origin=Origin(xyz=(-0.14, 0.0, 0.0)),
        material=fin_red,
        name="elevator_panel",
    )
    right_elevator.inertial = Inertial.from_geometry(
        Box((0.28, 0.54, 0.035)),
        mass=5.0,
        origin=Origin(xyz=(-0.14, 0.0, 0.0)),
    )
    model.articulation(
        "hull_to_right_elevator",
        ArticulationType.REVOLUTE,
        parent=hull,
        child=right_elevator,
        origin=Origin(xyz=(-4.03, -0.87, 0.20)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.2,
            lower=-0.30,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hull = object_model.get_part("hull")
    rudder = object_model.get_part("rudder")
    left_elevator = object_model.get_part("left_elevator")
    right_elevator = object_model.get_part("right_elevator")

    rudder_joint = object_model.get_articulation("hull_to_rudder")
    left_elevator_joint = object_model.get_articulation("hull_to_left_elevator")
    right_elevator_joint = object_model.get_articulation("hull_to_right_elevator")

    ctx.expect_contact(
        hull,
        rudder,
        elem_a="tail_fin",
        elem_b="rudder_panel",
        name="rudder seats on the tail fin hinge line",
    )
    ctx.expect_contact(
        hull,
        left_elevator,
        elem_a="left_tailplane",
        elem_b="elevator_panel",
        name="left elevator seats on the tailplane hinge line",
    )
    ctx.expect_contact(
        hull,
        right_elevator,
        elem_a="right_tailplane",
        elem_b="elevator_panel",
        name="right elevator seats on the tailplane hinge line",
    )

    envelope_aabb = ctx.part_element_world_aabb(hull, elem="envelope")
    cabin_aabb = ctx.part_element_world_aabb(hull, elem="cabin")
    ctx.check(
        "gondola hangs below the envelope",
        envelope_aabb is not None
        and cabin_aabb is not None
        and cabin_aabb[1][2] < envelope_aabb[0][2] - 0.05,
        details=f"envelope={envelope_aabb}, cabin={cabin_aabb}",
    )

    rudder_rest_aabb = ctx.part_world_aabb(rudder)
    with ctx.pose({rudder_joint: rudder_joint.motion_limits.upper}):
        rudder_turned_aabb = ctx.part_world_aabb(rudder)
    ctx.check(
        "rudder deflects outward at positive command",
        rudder_rest_aabb is not None
        and rudder_turned_aabb is not None
        and rudder_turned_aabb[1][1] > rudder_rest_aabb[1][1] + 0.08,
        details=f"rest={rudder_rest_aabb}, turned={rudder_turned_aabb}",
    )

    left_rest_aabb = ctx.part_world_aabb(left_elevator)
    right_rest_aabb = ctx.part_world_aabb(right_elevator)
    with ctx.pose(
        {
            left_elevator_joint: left_elevator_joint.motion_limits.upper,
            right_elevator_joint: right_elevator_joint.motion_limits.upper,
        }
    ):
        left_up_aabb = ctx.part_world_aabb(left_elevator)
        right_up_aabb = ctx.part_world_aabb(right_elevator)
    ctx.check(
        "left elevator rises at positive command",
        left_rest_aabb is not None
        and left_up_aabb is not None
        and left_up_aabb[1][2] > left_rest_aabb[1][2] + 0.03,
        details=f"rest={left_rest_aabb}, up={left_up_aabb}",
    )
    ctx.check(
        "right elevator rises at positive command",
        right_rest_aabb is not None
        and right_up_aabb is not None
        and right_up_aabb[1][2] > right_rest_aabb[1][2] + 0.03,
        details=f"rest={right_rest_aabb}, up={right_up_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
