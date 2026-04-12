from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HANDLE_LENGTH = 0.060
HANDLE_FRONT_OVERHANG = 0.004
HANDLE_THICKNESS = 0.010
HANDLE_HEIGHT = 0.014
HANDLE_CAVITY_LENGTH = 0.052
HANDLE_CAVITY_THICKNESS = 0.0048
HANDLE_CAVITY_HEIGHT = 0.0104
PIVOT_CLEARANCE_RADIUS = 0.0053

BLADE_THICKNESS = 0.0022
BLADE_OPEN_ANGLE = 3.05

BAIL_RING_WIDTH = 0.017
BAIL_RING_HEIGHT = 0.018
BAIL_RING_THICKNESS = 0.0028
BAIL_RING_STRAP = 0.0024

TAIL_HINGE_X = -0.0535
TAIL_HINGE_Z = -0.0094


def _handle_shape() -> cq.Workplane:
    handle_center_x = HANDLE_FRONT_OVERHANG - HANDLE_LENGTH / 2.0
    cavity_min_x = HANDLE_FRONT_OVERHANG - HANDLE_CAVITY_LENGTH
    cavity_center_x = (HANDLE_FRONT_OVERHANG + cavity_min_x) / 2.0

    outer = (
        cq.Workplane("XY")
        .box(HANDLE_LENGTH, HANDLE_THICKNESS, HANDLE_HEIGHT)
        .translate((handle_center_x, 0.0, 0.0))
        .edges("|Y")
        .fillet(0.0032)
        .edges("|X")
        .fillet(0.0010)
    )

    cavity = (
        cq.Workplane("XY")
        .box(HANDLE_CAVITY_LENGTH, HANDLE_CAVITY_THICKNESS, HANDLE_CAVITY_HEIGHT)
        .translate((cavity_center_x, 0.0, 0.0))
    )

    pivot_clearance = (
        cq.Workplane("XZ")
        .circle(PIVOT_CLEARANCE_RADIUS)
        .extrude(HANDLE_CAVITY_THICKNESS + 0.002, both=True)
    )

    tail_lug = (
        cq.Workplane("XY")
        .box(0.008, 0.0046, 0.0046)
        .translate((TAIL_HINGE_X + 0.0005, 0.0, TAIL_HINGE_Z + 0.0018))
        .edges("|Y")
        .fillet(0.0012)
    )

    left_pivot_boss = (
        cq.Workplane("XZ")
        .circle(0.0062)
        .extrude(0.0012, both=True)
        .translate((0.0, HANDLE_THICKNESS / 2.0 - 0.0003, 0.0))
    )
    right_pivot_boss = (
        cq.Workplane("XZ")
        .circle(0.0062)
        .extrude(0.0012, both=True)
        .translate((0.0, -HANDLE_THICKNESS / 2.0 + 0.0003, 0.0))
    )

    return outer.cut(cavity).cut(pivot_clearance).union(tail_lug).union(left_pivot_boss).union(right_pivot_boss)


def _blade_shape() -> cq.Workplane:
    blade_profile = [
        (0.0034, 0.0012),
        (0.0018, 0.0039),
        (-0.0080, 0.0056),
        (-0.0280, 0.0049),
        (-0.0435, 0.0004),
        (-0.0428, -0.0005),
        (-0.0270, -0.0011),
        (-0.0120, -0.0023),
        (-0.0038, -0.0030),
        (0.0026, -0.0016),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(blade_profile)
        .close()
        .extrude(BLADE_THICKNESS, both=True)
        .edges("|Y")
        .fillet(0.00045)
    )


def _bail_ring_shape() -> cq.Workplane:
    outer_center_z = -BAIL_RING_HEIGHT / 2.0
    inner_width = BAIL_RING_WIDTH - 2.0 * BAIL_RING_STRAP
    inner_height = BAIL_RING_HEIGHT - 2.0 * BAIL_RING_STRAP

    outer = (
        cq.Workplane("XY")
        .box(BAIL_RING_WIDTH, BAIL_RING_THICKNESS, BAIL_RING_HEIGHT)
        .translate((0.0, 0.0, outer_center_z))
        .edges("|Y")
        .fillet(0.0030)
        .edges("|X")
        .fillet(0.0008)
    )
    inner = (
        cq.Workplane("XY")
        .box(inner_width, BAIL_RING_THICKNESS + 0.002, inner_height)
        .translate((0.0, 0.0, outer_center_z))
    )
    return outer.cut(inner)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_pocket_knife")

    model.material("handle_finish", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shape(), "handle_shell"),
        material="handle_finish",
        name="handle_shell",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_blade_shape(), "blade_body"),
        material="steel",
        name="blade_body",
    )

    bail_ring = model.part("bail_ring")
    bail_ring.visual(
        mesh_from_cadquery(_bail_ring_shape(), "bail_loop"),
        material="steel",
        name="bail_loop",
    )

    model.articulation(
        "blade_pivot",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=blade,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=BLADE_OPEN_ANGLE, effort=4.0, velocity=5.0),
    )
    model.articulation(
        "bail_hinge",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=bail_ring,
        origin=Origin(xyz=(TAIL_HINGE_X, 0.0, TAIL_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    blade = object_model.get_part("blade")
    bail_ring = object_model.get_part("bail_ring")
    blade_pivot = object_model.get_articulation("blade_pivot")
    bail_hinge = object_model.get_articulation("bail_hinge")

    ctx.expect_within(
        blade,
        handle,
        axes="xyz",
        inner_elem="blade_body",
        outer_elem="handle_shell",
        margin=0.0008,
        name="closed blade nests inside handle envelope",
    )

    handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_shell")
    ring_aabb = ctx.part_element_world_aabb(bail_ring, elem="bail_loop")
    handle_center = _aabb_center(handle_aabb)
    ring_center = _aabb_center(ring_aabb)
    ctx.check(
        "bail ring hangs below handle at rest",
        handle_center is not None and ring_center is not None and ring_center[2] < handle_center[2] - 0.010,
        details=f"handle_center={handle_center}, ring_center={ring_center}",
    )

    with ctx.pose({blade_pivot: pi / 2.0}):
        open_mid_blade = ctx.part_element_world_aabb(blade, elem="blade_body")
        open_mid_handle = ctx.part_element_world_aabb(handle, elem="handle_shell")
        ctx.check(
            "blade swings upward through the front opening",
            open_mid_blade is not None
            and open_mid_handle is not None
            and open_mid_blade[1][2] > open_mid_handle[1][2] + 0.004,
            details=f"blade_aabb={open_mid_blade}, handle_aabb={open_mid_handle}",
        )

    with ctx.pose({blade_pivot: BLADE_OPEN_ANGLE}):
        open_blade = ctx.part_element_world_aabb(blade, elem="blade_body")
        open_handle = ctx.part_element_world_aabb(handle, elem="handle_shell")
        ctx.check(
            "opened blade projects forward of the handle",
            open_blade is not None
            and open_handle is not None
            and open_blade[1][0] > open_handle[1][0] + 0.028,
            details=f"blade_aabb={open_blade}, handle_aabb={open_handle}",
        )

    with ctx.pose({bail_hinge: 1.0}):
        rotated_ring = ctx.part_element_world_aabb(bail_ring, elem="bail_loop")
        rotated_center = _aabb_center(rotated_ring)
        ctx.check(
            "bail ring rotates off its hanging position",
            ring_center is not None
            and rotated_center is not None
            and (
                abs(rotated_center[0] - ring_center[0]) > 0.004
                or abs(rotated_center[2] - ring_center[2]) > 0.004
            ),
            details=f"rest_center={ring_center}, rotated_center={rotated_center}",
        )

    return ctx.report()


object_model = build_object_model()
