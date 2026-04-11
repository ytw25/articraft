from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HANDLE_THICKNESS = 0.0128
SCALE_THICKNESS = 0.0026
BOLSTER_THICKNESS = 0.0032
BLADE_THICKNESS = 0.0022
LOCKBACK_THICKNESS = 0.0028

PIVOT_X = 0.041
PIVOT_Y = -0.0015
LOCKBACK_PIVOT_X = -0.024
LOCKBACK_PIVOT_Y = 0.0022

PIVOT_PIN_RADIUS = 0.00185
LOCKBACK_PIN_RADIUS = 0.00155


def _handle_outline() -> list[tuple[float, float]]:
    return [
        (-0.051, -0.0034),
        (-0.047, 0.0058),
        (-0.020, 0.0076),
        (0.028, 0.0076),
        (0.048, 0.0058),
        (0.054, 0.0012),
        (0.051, -0.0048),
        (0.028, -0.0081),
        (-0.022, -0.0082),
        (-0.049, -0.0052),
    ]


def _scale_shape(z_lower: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .polyline(_handle_outline())
        .close()
        .extrude(SCALE_THICKNESS)
        .translate((0.0, 0.0, z_lower))
    )


def _blade_shape() -> cq.Workplane:
    blade_body = (
        cq.Workplane("XY")
        .polyline(
            [
                (0.001, 0.0048),
                (0.020, 0.0072),
                (0.049, 0.0074),
                (0.062, 0.0050),
                (0.071, 0.0021),
                (0.074, 0.0000),
                (0.060, -0.0023),
                (0.031, -0.0046),
                (0.010, -0.0052),
                (-0.002, -0.0040),
                (-0.006, -0.0010),
                (-0.004, 0.0020),
            ]
        )
        .close()
        .extrude(BLADE_THICKNESS)
        .translate((0.0, 0.0, -BLADE_THICKNESS / 2.0))
    )
    tang = (
        cq.Workplane("XY")
        .circle(0.0068)
        .extrude(BLADE_THICKNESS)
        .translate((0.0, 0.0, -BLADE_THICKNESS / 2.0))
    )
    pivot_hole = (
        cq.Workplane("XY")
        .circle(PIVOT_PIN_RADIUS + 0.00035)
        .extrude(BLADE_THICKNESS + 0.002)
        .translate((0.0, 0.0, -BLADE_THICKNESS / 2.0 - 0.001))
    )
    lock_notch = (
        cq.Workplane("XY")
        .box(0.006, 0.0032, BLADE_THICKNESS + 0.002, centered=(True, True, True))
        .translate((0.001, 0.0052, 0.0))
    )
    return tang.union(blade_body).cut(pivot_hole).cut(lock_notch).rotate((0, 0, 0), (0, 0, 1), 180)


def _lockback_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(0.066, 0.0034, LOCKBACK_THICKNESS, centered=(True, True, True))
        .translate((0.016, 0.0042, 0.0))
    )
    rear_pad = (
        cq.Workplane("XY")
        .box(0.020, 0.0054, LOCKBACK_THICKNESS, centered=(True, True, True))
        .translate((-0.010, 0.0010, 0.0))
    )
    front_tooth = (
        cq.Workplane("XY")
        .box(0.010, 0.0060, LOCKBACK_THICKNESS, centered=(True, True, True))
        .translate((0.051, -0.0006, 0.0))
    )
    front_bridge = (
        cq.Workplane("XY")
        .box(0.014, 0.0032, LOCKBACK_THICKNESS, centered=(True, True, True))
        .translate((0.045, 0.0020, 0.0))
    )
    front_nose = (
        cq.Workplane("XY")
        .box(0.009, 0.0032, LOCKBACK_THICKNESS, centered=(True, True, True))
        .translate((0.0545, 0.0020, 0.0))
    )
    pivot_hole = (
        cq.Workplane("XY")
        .circle(LOCKBACK_PIN_RADIUS + 0.00035)
        .extrude(LOCKBACK_THICKNESS + 0.002)
        .translate((0.0, 0.0, -LOCKBACK_THICKNESS / 2.0 - 0.001))
    )
    return body.union(rear_pad).union(front_tooth).union(front_bridge).union(front_nose).cut(pivot_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lockback_pocket_knife")

    wood = model.material("wood", rgba=(0.47, 0.30, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    bolster = model.material("bolster", rgba=(0.82, 0.83, 0.85, 1.0))
    pin = model.material("pin", rgba=(0.60, 0.61, 0.64, 1.0))

    handle = model.part("handle")
    left_scale = _scale_shape(HANDLE_THICKNESS / 2.0 - SCALE_THICKNESS)
    right_scale = _scale_shape(-HANDLE_THICKNESS / 2.0)
    handle.visual(mesh_from_cadquery(left_scale, "left_scale"), material=wood, name="left_scale")
    handle.visual(mesh_from_cadquery(right_scale, "right_scale"), material=wood, name="right_scale")

    bolster_z = HANDLE_THICKNESS / 2.0 - BOLSTER_THICKNESS / 2.0
    for z_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        z_center = z_sign * bolster_z
        handle.visual(
            Box((0.022, 0.0158, BOLSTER_THICKNESS)),
            origin=Origin(xyz=(0.043, -0.0005, z_center)),
            material=bolster,
            name=f"front_bolster_{side_name}",
        )
        handle.visual(
            Box((0.018, 0.0148, BOLSTER_THICKNESS)),
            origin=Origin(xyz=(-0.042, -0.0005, z_center)),
            material=bolster,
            name=f"rear_bolster_{side_name}",
        )

    handle.visual(
        Cylinder(radius=PIVOT_PIN_RADIUS, length=HANDLE_THICKNESS),
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0)),
        material=pin,
        name="pivot_pin",
    )
    handle.visual(
        Cylinder(radius=LOCKBACK_PIN_RADIUS, length=HANDLE_THICKNESS),
        origin=Origin(xyz=(LOCKBACK_PIVOT_X, LOCKBACK_PIVOT_Y, 0.0)),
        material=pin,
        name="lockback_pin",
    )
    handle.visual(
        Cylinder(radius=0.0013, length=HANDLE_THICKNESS),
        origin=Origin(xyz=(-0.040, -0.0023, 0.0)),
        material=pin,
        name="tail_pin",
    )
    handle.visual(
        Cylinder(radius=0.0013, length=HANDLE_THICKNESS),
        origin=Origin(xyz=(0.006, -0.0048, 0.0)),
        material=pin,
        name="center_pin",
    )

    blade = model.part("blade")
    blade.visual(mesh_from_cadquery(_blade_shape(), "blade"), material=steel, name="blade_body")

    lockback = model.part("lockback")
    lockback.visual(mesh_from_cadquery(_lockback_shape(), "lockback"), material=steel, name="lockback_body")

    model.articulation(
        "handle_to_blade",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=blade,
        origin=Origin(xyz=(PIVOT_X, PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=3.02, effort=2.0, velocity=4.0),
    )

    model.articulation(
        "handle_to_lockback",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=lockback,
        origin=Origin(xyz=(LOCKBACK_PIVOT_X, LOCKBACK_PIVOT_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.22, effort=0.5, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    blade = object_model.get_part("blade")
    lockback = object_model.get_part("lockback")
    blade_joint = object_model.get_articulation("handle_to_blade")
    lockback_joint = object_model.get_articulation("handle_to_lockback")

    blade_limits = blade_joint.motion_limits
    lockback_limits = lockback_joint.motion_limits

    if blade_limits is not None and blade_limits.lower is not None and blade_limits.upper is not None:
        with ctx.pose({blade_joint: blade_limits.lower}):
            handle_aabb = ctx.part_world_aabb(handle)
            blade_aabb = ctx.part_world_aabb(blade)
            closed_ok = (
                handle_aabb is not None
                and blade_aabb is not None
                and blade_aabb[0][0] >= handle_aabb[0][0] - 0.004
                and blade_aabb[1][0] <= handle_aabb[1][0] + 0.001
                and blade_aabb[1][1] <= handle_aabb[1][1] + 0.001
                and blade_aabb[0][1] >= handle_aabb[0][1] - 0.001
            )
            ctx.check(
                "closed blade nests inside handle silhouette",
                closed_ok,
                details=f"handle_aabb={handle_aabb}, blade_aabb={blade_aabb}",
            )
            ctx.expect_overlap(
                blade,
                handle,
                axes="xy",
                min_overlap=0.012,
                name="closed blade remains nested in handle footprint",
            )

        with ctx.pose({blade_joint: blade_limits.upper}):
            handle_aabb = ctx.part_world_aabb(handle)
            blade_aabb = ctx.part_world_aabb(blade)
            open_ok = (
                handle_aabb is not None
                and blade_aabb is not None
                and blade_aabb[1][0] > handle_aabb[1][0] + 0.040
                and blade_aabb[0][1] < handle_aabb[0][1] + 0.002
            )
            ctx.check(
                "open blade swings forward beyond handle",
                open_ok,
                details=f"handle_aabb={handle_aabb}, blade_aabb={blade_aabb}",
            )

    if lockback_limits is not None and lockback_limits.lower is not None and lockback_limits.upper is not None:
        with ctx.pose({lockback_joint: lockback_limits.lower}):
            rest_aabb = ctx.part_world_aabb(lockback)
        with ctx.pose({lockback_joint: lockback_limits.upper}):
            unlock_aabb = ctx.part_world_aabb(lockback)
        rocker_ok = (
            rest_aabb is not None
            and unlock_aabb is not None
            and unlock_aabb[0][1] < rest_aabb[0][1] - 0.001
            and unlock_aabb[1][1] > rest_aabb[1][1] + 0.001
        )
        ctx.check(
            "lockback pivots with rear press and front lift",
            rocker_ok,
            details=f"rest_aabb={rest_aabb}, unlock_aabb={unlock_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
