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


def _rounded_body(length: float, width: float, height: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).edges("|X").fillet(radius)


def _front_wheel_gear(x_pos: float, y_pos: float) -> cq.Workplane:
    strut = cq.Workplane("XY").box(0.18, 0.12, 0.58).translate((x_pos, y_pos, 0.88))
    axle = cq.Workplane("XZ").circle(0.05).extrude(0.34, both=True).translate((x_pos, y_pos, 0.62))
    wheel = cq.Workplane("XZ").circle(0.30).extrude(0.16, both=True).translate((x_pos, y_pos, 0.38))
    return strut.union(axle).union(wheel)


def _aft_wheel_gear(x_pos: float, y_pos: float) -> cq.Workplane:
    strut = cq.Workplane("XY").box(0.18, 0.12, 0.68).translate((x_pos, y_pos, 0.95))
    axle = cq.Workplane("XZ").circle(0.05).extrude(0.34, both=True).translate((x_pos, y_pos, 0.68))
    wheel = cq.Workplane("XZ").circle(0.33).extrude(0.18, both=True).translate((x_pos, y_pos, 0.42))
    return strut.union(axle).union(wheel)


def _build_fuselage_shell() -> cq.Workplane:
    cabin = _rounded_body(8.0, 2.70, 2.50, 0.38).translate((0.20, 0.0, 2.00))

    nose = _rounded_body(4.40, 2.35, 2.24, 0.30).translate((5.20, 0.0, 2.02))
    nose = nose.cut(
        cq.Workplane("XY")
        .box(3.8, 4.2, 2.3)
        .translate((7.35, 0.0, 3.30))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 24.0)
    )
    nose = nose.cut(
        cq.Workplane("XY")
        .box(3.6, 4.2, 1.5)
        .translate((7.30, 0.0, 0.70))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
    )

    tail_lower = _rounded_body(4.20, 2.48, 2.18, 0.28).translate((-5.55, 0.0, 1.90))
    tail_lower = tail_lower.cut(
        cq.Workplane("XY")
        .box(3.8, 4.0, 1.6)
        .translate((-7.10, 0.0, 3.20))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
    )

    front_pylon = _rounded_body(1.60, 1.78, 1.08, 0.18).translate((5.00, 0.0, 3.74))
    rear_pylon = _rounded_body(3.50, 1.46, 2.82, 0.22).translate((-5.85, 0.0, 4.10))

    left_engine = _rounded_body(4.40, 0.96, 0.96, 0.18).translate((-0.30, 1.66, 3.72))
    right_engine = _rounded_body(4.40, 0.96, 0.96, 0.18).translate((-0.30, -1.66, 3.72))

    left_sponson = _rounded_body(6.60, 0.66, 0.84, 0.12).translate((0.10, 1.56, 1.18))
    right_sponson = _rounded_body(6.60, 0.66, 0.84, 0.12).translate((0.10, -1.56, 1.18))
    left_engine_bridge = _rounded_body(3.00, 0.54, 0.52, 0.08).translate((-0.35, 1.34, 3.16))
    right_engine_bridge = _rounded_body(3.00, 0.54, 0.52, 0.08).translate((-0.35, -1.34, 3.16))

    body = cabin
    for shape in (
        nose,
        tail_lower,
        front_pylon,
        rear_pylon,
        left_engine,
        right_engine,
        left_engine_bridge,
        right_engine_bridge,
        left_sponson,
        right_sponson,
    ):
        body = body.union(shape)

    inner_cabin = _rounded_body(13.00, 2.08, 1.86, 0.22).translate((-0.45, 0.0, 2.10))
    body = body.cut(inner_cabin)
    body = body.cut(cq.Workplane("XY").box(1.24, 2.12, 1.62).translate((-7.72, 0.0, 1.66)))

    for gear in (
        _front_wheel_gear(3.20, 1.62),
        _front_wheel_gear(3.20, -1.62),
        _aft_wheel_gear(-2.45, 1.66),
        _aft_wheel_gear(-2.45, -1.66),
    ):
        body = body.union(gear)

    return body


def _build_cockpit_glazing() -> cq.Workplane:
    glass = _rounded_body(2.45, 2.06, 0.98, 0.12).translate((6.05, 0.0, 2.56))
    glass = glass.cut(
        cq.Workplane("XY")
        .box(2.8, 3.0, 1.5)
        .translate((6.96, 0.0, 3.18))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 24.0)
    )
    glass = glass.cut(cq.Workplane("XY").box(2.8, 3.0, 1.1).translate((6.15, 0.0, 1.84)))
    glass = glass.cut(cq.Workplane("XY").box(1.0, 0.22, 1.2).translate((6.65, 0.0, 2.54)))
    return glass


def _build_cabin_window_strip(y_pos: float) -> cq.Workplane:
    return cq.Workplane("XY").box(6.60, 0.10, 0.52).translate((0.65, y_pos, 2.52))


def _build_rotor_mast(height: float) -> cq.Workplane:
    column = cq.Workplane("XY").circle(0.14).extrude(height)
    flange = cq.Workplane("XY").circle(0.24).extrude(0.08)
    return flange.union(column)


def _build_rotor_hub() -> cq.Workplane:
    spindle = cq.Workplane("XY").circle(0.14).extrude(0.18)
    hub = cq.Workplane("XY").circle(0.42).extrude(0.16).translate((0.0, 0.0, 0.04))
    cap = cq.Workplane("XY").circle(0.28).extrude(0.10).translate((0.0, 0.0, 0.18))
    return spindle.union(hub).union(cap)


def _build_rotor_blade(angle_deg: float) -> cq.Workplane:
    planform = [
        (0.30, -0.24),
        (1.00, -0.31),
        (6.70, -0.18),
        (7.72, -0.08),
        (8.02, 0.00),
        (7.72, 0.08),
        (6.70, 0.18),
        (1.00, 0.31),
        (0.30, 0.24),
    ]
    blade = cq.Workplane("XY").polyline(planform).close().extrude(0.07, both=True)
    root_cuff = cq.Workplane("XY").box(0.62, 0.50, 0.10).translate((0.36, 0.0, 0.0))
    return (
        blade.union(root_cuff)
        .translate((0.0, 0.0, 0.18))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
    )


def _build_ramp() -> cq.Workplane:
    skin = cq.Workplane("XY").box(0.10, 2.10, 1.64).translate((-0.05, 0.0, 0.82))
    left_rib = cq.Workplane("XY").box(0.16, 0.18, 1.38).translate((-0.02, 0.80, 0.72))
    right_rib = cq.Workplane("XY").box(0.16, 0.18, 1.38).translate((-0.02, -0.80, 0.72))
    upper_beam = cq.Workplane("XY").box(0.12, 1.98, 0.14).translate((-0.03, 0.0, 1.54))
    left_hinge = cq.Workplane("XZ").circle(0.05).extrude(0.16, both=True).translate((0.01, 0.86, 0.08))
    right_hinge = cq.Workplane("XZ").circle(0.05).extrude(0.16, both=True).translate((0.01, -0.86, 0.08))
    return skin.union(left_rib).union(right_rib).union(upper_beam).union(left_hinge).union(right_hinge)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chinook_helicopter")

    drab = model.material("drab", rgba=(0.39, 0.43, 0.29, 1.0))
    drab_dark = model.material("drab_dark", rgba=(0.28, 0.31, 0.21, 1.0))
    rotor_gray = model.material("rotor_gray", rgba=(0.25, 0.27, 0.29, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.14, 0.20, 0.24, 0.95))

    fuselage = model.part("fuselage")
    fuselage.visual(
        mesh_from_cadquery(_build_fuselage_shell(), "fuselage_shell"),
        material=drab,
        name="shell",
    )
    fuselage.visual(
        mesh_from_cadquery(_build_cockpit_glazing(), "cockpit_glazing"),
        material=glass,
        name="cockpit_glazing",
    )
    fuselage.visual(
        mesh_from_cadquery(_build_cabin_window_strip(1.32), "cabin_window_strip_0"),
        material=glass,
        name="cabin_window_strip_0",
    )
    fuselage.visual(
        mesh_from_cadquery(_build_cabin_window_strip(-1.32), "cabin_window_strip_1"),
        material=glass,
        name="cabin_window_strip_1",
    )

    front_mast = model.part("front_mast")
    front_mast.visual(
        mesh_from_cadquery(_build_rotor_mast(0.34), "front_mast"),
        material=drab_dark,
        name="column",
    )

    rear_mast = model.part("rear_mast")
    rear_mast.visual(
        mesh_from_cadquery(_build_rotor_mast(0.26), "rear_mast"),
        material=drab_dark,
        name="column",
    )

    front_rotor = model.part("front_rotor")
    front_rotor.visual(
        mesh_from_cadquery(_build_rotor_hub(), "front_rotor_hub"),
        material=drab_dark,
        name="hub",
    )
    for index, angle in enumerate((0.0, 120.0, 240.0)):
        front_rotor.visual(
            mesh_from_cadquery(_build_rotor_blade(angle), f"front_rotor_blade_{index}"),
            material=rotor_gray,
            name=f"blade_{index}",
        )

    rear_rotor = model.part("rear_rotor")
    rear_rotor.visual(
        mesh_from_cadquery(_build_rotor_hub(), "rear_rotor_hub"),
        material=drab_dark,
        name="hub",
    )
    for index, angle in enumerate((60.0, 180.0, 300.0)):
        rear_rotor.visual(
            mesh_from_cadquery(_build_rotor_blade(angle), f"rear_rotor_blade_{index}"),
            material=rotor_gray,
            name=f"blade_{index}",
        )

    ramp = model.part("ramp")
    ramp.visual(
        mesh_from_cadquery(_build_ramp(), "rear_ramp"),
        material=drab,
        name="panel",
    )

    rotor_limits = MotionLimits(effort=1200.0, velocity=40.0)
    model.articulation(
        "fuselage_to_front_mast",
        ArticulationType.FIXED,
        parent=fuselage,
        child=front_mast,
        origin=Origin(xyz=(5.00, 0.0, 4.28)),
    )
    model.articulation(
        "front_mast_to_front_rotor",
        ArticulationType.CONTINUOUS,
        parent=front_mast,
        child=front_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=rotor_limits,
    )
    model.articulation(
        "fuselage_to_rear_mast",
        ArticulationType.FIXED,
        parent=fuselage,
        child=rear_mast,
        origin=Origin(xyz=(-5.85, 0.0, 5.51)),
    )
    model.articulation(
        "rear_mast_to_rear_rotor",
        ArticulationType.CONTINUOUS,
        parent=rear_mast,
        child=rear_rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=rotor_limits,
    )
    model.articulation(
        "fuselage_to_ramp",
        ArticulationType.REVOLUTE,
        parent=fuselage,
        child=ramp,
        origin=Origin(xyz=(-7.16, 0.0, 0.86)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=800.0, velocity=1.5, lower=0.0, upper=1.30),
    )

    return model


def _axis_span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: int) -> float:
    return aabb[1][axis] - aabb[0][axis]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fuselage = object_model.get_part("fuselage")
    front_rotor = object_model.get_part("front_rotor")
    rear_rotor = object_model.get_part("rear_rotor")
    ramp = object_model.get_part("ramp")
    front_mast = object_model.get_part("front_mast")

    front_spin = object_model.get_articulation("front_mast_to_front_rotor")
    ramp_hinge = object_model.get_articulation("fuselage_to_ramp")

    ctx.allow_overlap(
        front_mast,
        fuselage,
        elem_a="column",
        elem_b="shell",
        reason="The front rotor mast is intentionally represented as passing into the forward pylon housing.",
    )
    ctx.allow_overlap(
        ramp,
        fuselage,
        elem_a="panel",
        elem_b="shell",
        reason="The closed ramp seats into a simplified aft fuselage shell proxy rather than a fully modeled recessed frame.",
    )

    ctx.expect_origin_distance(
        front_rotor,
        rear_rotor,
        axes="x",
        min_dist=10.0,
        max_dist=11.4,
        name="tandem rotor masts span the long fuselage",
    )

    with ctx.pose({ramp_hinge: 0.0}):
        ctx.expect_overlap(
            ramp,
            fuselage,
            axes="yz",
            min_overlap=1.45,
            name="closed ramp covers the cargo opening",
        )
        ctx.expect_contact(
            ramp,
            fuselage,
            name="closed ramp stays attached to the rear frame",
        )
        blade_aabb_0 = ctx.part_element_world_aabb(front_rotor, elem="blade_0")
        closed_ramp_aabb = ctx.part_world_aabb(ramp)

    with ctx.pose({front_spin: math.pi / 2.0}):
        blade_aabb_90 = ctx.part_element_world_aabb(front_rotor, elem="blade_0")

    with ctx.pose({ramp_hinge: 1.20}):
        open_ramp_aabb = ctx.part_world_aabb(ramp)

    ctx.check(
        "front rotor blade sweeps the rotor plane",
        blade_aabb_0 is not None
        and blade_aabb_90 is not None
        and _axis_span(blade_aabb_0, 0) > 7.0
        and _axis_span(blade_aabb_0, 1) < 1.2
        and _axis_span(blade_aabb_90, 1) > 7.0
        and _axis_span(blade_aabb_90, 0) < 1.2,
        details=f"q0={blade_aabb_0}, q90={blade_aabb_90}",
    )
    ctx.check(
        "rear ramp lowers for loading",
        closed_ramp_aabb is not None
        and open_ramp_aabb is not None
        and open_ramp_aabb[1][2] < closed_ramp_aabb[1][2] - 0.85
        and open_ramp_aabb[0][0] < closed_ramp_aabb[0][0] - 1.0,
        details=f"closed={closed_ramp_aabb}, open={open_ramp_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
