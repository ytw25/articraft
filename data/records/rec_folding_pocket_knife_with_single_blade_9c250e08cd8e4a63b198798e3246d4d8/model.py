from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


HANDLE_LENGTH = 0.106
HANDLE_THICKNESS = 0.0145
HANDLE_HEIGHT = 0.029
INNER_CAVITY_WIDTH = 0.0082

BLADE_LENGTH = 0.078
BLADE_THICKNESS = 0.0032

LOCK_RADIUS = 0.0021
LOCK_LENGTH = 0.0122
LOCK_X = 0.0175
LOCK_Z = 0.0075
LOCK_TRAVEL = 0.0050


def _make_handle_shape() -> cq.Workplane:
    handle_outline = [
        (-0.0080, 0.0090),
        (0.0020, 0.0118),
        (0.0200, 0.0130),
        (0.0520, 0.0136),
        (0.0820, 0.0121),
        (0.0980, 0.0085),
        (0.1060, 0.0025),
        (0.1040, -0.0065),
        (0.0920, -0.0125),
        (0.0660, -0.0150),
        (0.0300, -0.0144),
        (0.0060, -0.0118),
        (-0.0070, -0.0065),
    ]

    outer = cq.Workplane("XZ").polyline(handle_outline).close().extrude(HANDLE_THICKNESS / 2.0, both=True)

    blade_cavity = (
        cq.Workplane("XY")
        .box(0.078, INNER_CAVITY_WIDTH, 0.0118, centered=(True, True, True))
        .translate((0.056, 0.0, -0.0010))
    )
    pivot_relief = (
        cq.Workplane("XY")
        .box(0.024, INNER_CAVITY_WIDTH, 0.0200, centered=(True, True, True))
        .translate((0.006, 0.0, 0.0000))
    )
    lock_race = (
        cq.Workplane("XY")
        .box(0.015, INNER_CAVITY_WIDTH + 0.0004, 0.0054, centered=(True, True, True))
        .translate((0.0210, 0.0, LOCK_Z))
    )

    slot_depth = 0.0043
    slot_box = cq.Workplane("XY").box(0.0122, slot_depth, 0.0053, centered=(True, True, True))
    slot_y = (HANDLE_THICKNESS * 0.5) - (slot_depth * 0.5)
    side_slot_pos = slot_box.translate((0.0210, slot_y, LOCK_Z))
    side_slot_neg = slot_box.translate((0.0210, -slot_y, LOCK_Z))

    body = outer.cut(blade_cavity.union(pivot_relief).union(lock_race))
    body = body.cut(side_slot_pos).cut(side_slot_neg)
    return body


def _make_blade_shape() -> cq.Workplane:
    blade_outline = [
        (-0.0045, 0.0048),
        (0.0010, 0.0080),
        (0.0160, 0.0100),
        (0.0420, 0.0089),
        (0.0640, 0.0054),
        (0.0780, 0.0010),
        (0.0700, -0.0009),
        (0.0480, -0.0034),
        (0.0200, -0.0050),
        (0.0060, -0.0059),
        (-0.0030, -0.0063),
        (-0.0050, -0.0024),
    ]

    blade = cq.Workplane("XZ").polyline(blade_outline).close().extrude(BLADE_THICKNESS / 2.0, both=True)
    pivot_hole = cq.Workplane("XZ").circle(0.0023).extrude(BLADE_THICKNESS, both=True)
    return blade.cut(pivot_hole)


def _make_lock_shape() -> cq.Workplane:
    return cq.Workplane("XZ").circle(LOCK_RADIUS).extrude(LOCK_LENGTH / 2.0, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_pocket_knife")

    handle_finish = model.material("handle_finish", rgba=(0.14, 0.15, 0.17, 1.0))
    blade_finish = model.material("blade_finish", rgba=(0.73, 0.75, 0.77, 1.0))
    lock_finish = model.material("lock_finish", rgba=(0.56, 0.58, 0.62, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_make_handle_shape(), "handle_frame"),
        material=handle_finish,
        name="handle_body",
    )
    handle.inertial = Inertial.from_geometry(
        Box((HANDLE_LENGTH, HANDLE_THICKNESS, HANDLE_HEIGHT)),
        mass=0.11,
        origin=Origin(xyz=(0.049, 0.0, -0.001)),
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_make_blade_shape(), "drop_point_blade"),
        material=blade_finish,
        name="blade_body",
    )
    blade.inertial = Inertial.from_geometry(
        Box((BLADE_LENGTH, BLADE_THICKNESS, 0.018)),
        mass=0.05,
        origin=Origin(xyz=(0.034, 0.0, 0.001)),
    )

    crossbar = model.part("crossbar")
    crossbar.visual(
        mesh_from_cadquery(_make_lock_shape(), "crossbar_lock"),
        material=lock_finish,
        name="lock_bar",
    )
    crossbar.inertial = Inertial.from_geometry(
        Cylinder(radius=LOCK_RADIUS, length=LOCK_LENGTH),
        mass=0.01,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "handle_to_blade",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=blade,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=4.0,
            lower=0.0,
            upper=3.05,
        ),
    )
    model.articulation(
        "handle_to_crossbar",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=crossbar,
        origin=Origin(xyz=(LOCK_X, 0.0, LOCK_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.08,
            lower=0.0,
            upper=LOCK_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    blade = object_model.get_part("blade")
    crossbar = object_model.get_part("crossbar")
    blade_joint = object_model.get_articulation("handle_to_blade")
    lock_joint = object_model.get_articulation("handle_to_crossbar")

    ctx.expect_within(
        crossbar,
        handle,
        axes="yz",
        margin=0.0012,
        name="crossbar stays inside handle thickness and height",
    )
    ctx.expect_overlap(
        blade,
        handle,
        axes="xy",
        min_overlap=0.0025,
        name="closed blade remains nested within handle footprint",
    )

    handle_box = ctx.part_world_aabb(handle)
    blade_box = ctx.part_element_world_aabb(blade, elem="blade_body")
    ctx.check(
        "closed blade sits below handle spine",
        handle_box is not None
        and blade_box is not None
        and blade_box[1][2] <= handle_box[1][2] - 0.001,
        details=f"handle_box={handle_box}, blade_box={blade_box}",
    )

    rest_lock_pos = ctx.part_world_position(crossbar)
    upper_blade = blade_joint.motion_limits.upper if blade_joint.motion_limits is not None else None
    upper_lock = lock_joint.motion_limits.upper if lock_joint.motion_limits is not None else None

    if upper_blade is not None:
        with ctx.pose({blade_joint: upper_blade}):
            open_blade_box = ctx.part_element_world_aabb(blade, elem="blade_body")
            ctx.check(
                "blade opens into a working position ahead of the pivot",
                open_blade_box is not None
                and open_blade_box[0][0] < -0.060
                and open_blade_box[1][0] < 0.010,
                details=f"open_blade_box={open_blade_box}",
            )
            ctx.expect_overlap(
                blade,
                handle,
                axes="y",
                min_overlap=0.0025,
                name="open blade stays aligned with the handle thickness",
            )

    if upper_lock is not None:
        with ctx.pose({lock_joint: upper_lock}):
            ctx.expect_within(
                crossbar,
                handle,
                axes="yz",
                margin=0.0012,
                name="retracted crossbar stays inside the handle guides",
            )
            pulled_lock_pos = ctx.part_world_position(crossbar)
        ctx.check(
            "crossbar retracts rearward along the side slots",
            rest_lock_pos is not None
            and pulled_lock_pos is not None
            and pulled_lock_pos[0] > rest_lock_pos[0] + 0.004,
            details=f"rest_lock_pos={rest_lock_pos}, pulled_lock_pos={pulled_lock_pos}",
        )

    return ctx.report()


object_model = build_object_model()
