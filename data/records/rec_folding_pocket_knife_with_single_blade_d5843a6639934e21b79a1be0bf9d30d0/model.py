from __future__ import annotations

import math

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


HANDLE_REAR_X = -0.103
WOOD_FRONT_X = -0.007
FERRULE_START_X = -0.014
FERRULE_LENGTH = 0.020
COLLAR_LENGTH = 0.009
COLLAR_CENTER_X = 0.002
BLADE_THICKNESS = 0.0023


def _centered_extrude_on_xz(shape: cq.Workplane, depth: float) -> cq.Workplane:
    return shape.extrude(depth).translate((0.0, -0.5 * depth, 0.0))


def _handle_wood_shape() -> cq.Workplane:
    body = (
        cq.Workplane("YZ")
        .workplane(offset=HANDLE_REAR_X)
        .ellipse(0.0060, 0.0065)
        .workplane(offset=0.022)
        .ellipse(0.0078, 0.0086)
        .workplane(offset=0.030)
        .ellipse(0.0091, 0.0098)
        .workplane(offset=0.028)
        .ellipse(0.0088, 0.0092)
        .workplane(offset=0.016)
        .ellipse(0.0080, 0.0082)
        .loft(combine=True)
    )

    slot = cq.Workplane("XY").box(0.088, 0.0036, 0.0136).translate((-0.045, 0.0, 0.0))
    pivot_clearance = _centered_extrude_on_xz(cq.Workplane("XZ").circle(0.0063), 0.0048)
    front_throat = cq.Workplane("XY").box(0.020, 0.0038, 0.0138).translate((-0.002, 0.0, 0.0))
    butt_round = cq.Workplane("XY").sphere(0.0068).translate((HANDLE_REAR_X + 0.001, 0.0, 0.0))

    return body.union(butt_round).cut(slot.union(pivot_clearance).union(front_throat))


def _ferrule_shape() -> cq.Workplane:
    main_outer = cq.Workplane("YZ").workplane(offset=FERRULE_START_X).circle(0.0101).extrude(FERRULE_LENGTH)
    retaining_ridge = cq.Workplane("YZ").workplane(offset=-0.0031).circle(0.0103).extrude(0.0006)
    outer = main_outer.union(retaining_ridge)
    inner = cq.Workplane("YZ").workplane(offset=FERRULE_START_X).circle(0.0079).extrude(FERRULE_LENGTH)
    slot = cq.Workplane("XY").box(FERRULE_LENGTH + 0.001, 0.0039, 0.0144).translate(
        (FERRULE_START_X + 0.5 * FERRULE_LENGTH, 0.0, 0.0)
    )
    return outer.cut(inner).cut(slot)


def _locking_collar_shape() -> cq.Workplane:
    outer = cq.Workplane("YZ").workplane(offset=-0.5 * COLLAR_LENGTH).circle(0.0115).extrude(COLLAR_LENGTH)
    inner = cq.Workplane("YZ").workplane(offset=-0.5 * COLLAR_LENGTH).circle(0.0103).extrude(COLLAR_LENGTH)
    slot = cq.Workplane("XY").box(COLLAR_LENGTH + 0.001, 0.0038, 0.0148)
    return outer.cut(inner).cut(slot)


def _blade_shape() -> cq.Workplane:
    blade_blank = _centered_extrude_on_xz(
        cq.Workplane("XZ")
        .moveTo(0.0022, 0.0032)
        .lineTo(-0.010, 0.0074)
        .threePointArc((-0.046, 0.0092), (-0.081, 0.0028))
        .lineTo(-0.086, 0.0002)
        .lineTo(-0.079, -0.0012)
        .lineTo(-0.046, -0.0041)
        .lineTo(-0.012, -0.0054)
        .lineTo(0.0016, -0.0026)
        .close(),
        BLADE_THICKNESS,
    )
    tang = _centered_extrude_on_xz(cq.Workplane("XZ").circle(0.0056), BLADE_THICKNESS)
    pivot_hole = _centered_extrude_on_xz(cq.Workplane("XZ").circle(0.0016), BLADE_THICKNESS + 0.0012)
    swedge = _centered_extrude_on_xz(
        cq.Workplane("XZ")
        .moveTo(-0.050, 0.0061)
        .lineTo(-0.073, 0.0032)
        .lineTo(-0.064, 0.0056)
        .close(),
        BLADE_THICKNESS + 0.0012,
    )
    return tang.union(blade_blank).cut(pivot_hole).cut(swedge)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_pocket_knife")

    wood = model.material("wood", rgba=(0.62, 0.45, 0.26, 1.0))
    ferrule_steel = model.material("ferrule_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.78, 0.80, 0.84, 1.0))
    collar_steel = model.material("collar_steel", rgba=(0.56, 0.58, 0.61, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_wood_shape(), "knife_wood_body"),
        material=wood,
        name="wood_body",
    )
    handle.visual(
        mesh_from_cadquery(_ferrule_shape(), "knife_ferrule"),
        material=ferrule_steel,
        name="ferrule",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_blade_shape(), "knife_blade"),
        material=blade_steel,
        name="blade_leaf",
    )

    collar = model.part("collar")
    collar.visual(
        mesh_from_cadquery(_locking_collar_shape(), "knife_collar"),
        material=collar_steel,
        name="collar_ring",
    )

    model.articulation(
        "blade_pivot",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=blade,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=0.0,
            upper=math.pi,
        ),
    )
    model.articulation(
        "collar_roll",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=collar,
        origin=Origin(xyz=(COLLAR_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    blade = object_model.get_part("blade")
    collar = object_model.get_part("collar")
    blade_pivot = object_model.get_articulation("blade_pivot")

    ctx.expect_within(
        blade,
        handle,
        axes="yz",
        elem_a="blade_leaf",
        margin=0.0015,
        name="closed blade stays within handle thickness",
    )
    ctx.expect_overlap(
        collar,
        handle,
        axes="yz",
        elem_a="collar_ring",
        elem_b="ferrule",
        min_overlap=0.018,
        name="locking collar stays concentric on bolster",
    )

    handle_closed_aabb = ctx.part_world_aabb(handle)
    blade_closed_aabb = ctx.part_world_aabb(blade)
    ctx.check(
        "closed blade stays tucked behind the ferrule front",
        handle_closed_aabb is not None
        and blade_closed_aabb is not None
        and blade_closed_aabb[1][0] <= handle_closed_aabb[1][0] + 0.001,
        details=f"handle={handle_closed_aabb}, blade={blade_closed_aabb}",
    )

    with ctx.pose({blade_pivot: math.pi}):
        ctx.expect_overlap(
            blade,
            handle,
            axes="yz",
            elem_a="blade_leaf",
            elem_b="ferrule",
            min_overlap=0.002,
            name="open blade stays aligned on the pivot axis",
        )
        handle_open_aabb = ctx.part_world_aabb(handle)
        blade_open_aabb = ctx.part_world_aabb(blade)
        ctx.check(
            "open blade extends clearly beyond the handle",
            handle_open_aabb is not None
            and blade_open_aabb is not None
            and blade_open_aabb[1][0] >= handle_open_aabb[1][0] + 0.065,
            details=f"handle={handle_open_aabb}, blade={blade_open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
