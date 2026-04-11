from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


Vec3 = tuple[float, float, float]


def _cylinder_between(start: Vec3, end: Vec3, radius: float):
    direction = cq.Vector(end[0] - start[0], end[1] - start[1], end[2] - start[2])
    length = math.dist(start, end)
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), direction)


def _loft_circles_on_x(profiles: list[tuple[float, float]]):
    workplane = cq.Workplane("YZ", origin=(profiles[0][0], 0.0, 0.0)).circle(profiles[0][1])
    last_x = profiles[0][0]
    for x_pos, radius in profiles[1:]:
        workplane = workplane.workplane(offset=x_pos - last_x).circle(radius)
        last_x = x_pos
    return workplane.loft(combine=True).val()


def _loft_rects_on_x(profiles: list[tuple[float, float, float, float]]):
    x_pos, width, height, center_z = profiles[0]
    workplane = cq.Workplane("YZ", origin=(x_pos, 0.0, center_z)).rect(width, height)
    last_x = x_pos
    for x_pos, width, height, center_z in profiles[1:]:
        workplane = (
            workplane.workplane(offset=x_pos - last_x, origin=(0.0, 0.0, center_z))
            .rect(width, height)
        )
        last_x = x_pos
    return workplane.loft(combine=True).val()


def _build_envelope():
    return _loft_circles_on_x(
        [
            (-18.2, 0.22),
            (-15.8, 1.65),
            (-11.5, 3.50),
            (-6.0, 4.35),
            (-0.5, 4.70),
            (5.5, 4.45),
            (11.2, 3.45),
            (15.5, 2.15),
            (18.2, 0.92),
            (20.0, 0.24),
        ]
    )


def _build_tail_assembly():
    fin_profile = [
        (12.0, 1.20),
        (13.7, 1.70),
        (16.8, 3.40),
        (18.9, 4.45),
        (19.6, 2.05),
        (16.5, 1.86),
        (13.0, 1.36),
    ]
    fin = cq.Workplane("XZ").polyline(fin_profile).close().extrude(0.18, both=True).val()

    assembly = cq.Assembly(name="tail")
    for index, angle in enumerate((45.0, 135.0, 225.0, 315.0)):
        assembly.add(fin.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), angle), name=f"fin_{index}")
    assembly.add(
        cq.Workplane("YZ", origin=(14.9, 0.0, 0.0)).circle(1.02).extrude(4.6).val(),
        name="tail_hub",
    )
    return assembly


def _build_keel():
    return _loft_rects_on_x(
        [
            (-4.8, 0.52, 0.70, -4.48),
            (-2.6, 0.98, 0.96, -4.58),
            (0.2, 1.24, 1.10, -4.66),
            (2.8, 1.10, 0.98, -4.58),
            (4.6, 0.58, 0.72, -4.42),
        ]
    )


def _build_gondola_body():
    cabin = (
        cq.Workplane("XY")
        .box(4.4, 1.65, 1.55)
        .edges("|Z")
        .fillet(0.24)
        .translate((0.30, 0.0, -1.95))
    ).val()
    nose = (
        cq.Workplane("YZ", origin=(2.15, 0.0, -1.88))
        .circle(0.42)
        .workplane(offset=0.72)
        .circle(0.05)
        .loft(combine=True)
        .val()
    )
    assembly = cq.Assembly(name="gondola_body")
    assembly.add(cabin, name="cabin")
    assembly.add(nose, name="nose")
    return assembly


def _build_gondola_support():
    roof_saddle = cq.Workplane("XY").box(1.65, 0.42, 0.18).translate((0.0, 0.0, -0.09)).val()
    front_strut = _cylinder_between((0.58, 0.0, -0.18), (1.10, 0.0, -1.18), 0.08)
    rear_strut = _cylinder_between((-0.62, 0.0, -0.18), (-1.00, 0.0, -1.14), 0.08)
    center_strut = _cylinder_between((0.0, 0.0, -0.18), (0.0, 0.0, -1.08), 0.10)

    assembly = cq.Assembly(name="gondola_support")
    assembly.add(roof_saddle, name="roof_saddle")
    assembly.add(front_strut, name="front_strut")
    assembly.add(rear_strut, name="rear_strut")
    assembly.add(center_strut, name="center_strut")
    return assembly


def _build_engine_pod(side_sign: float):
    pod_center_y = side_sign * 1.55
    pod_center_z = -1.32
    pod = cq.Workplane("YZ", origin=(-0.85, pod_center_y, pod_center_z)).circle(0.56).extrude(2.90).val()
    spinner = (
        cq.Workplane("YZ", origin=(2.05, pod_center_y, pod_center_z))
        .circle(0.24)
        .workplane(offset=0.45)
        .circle(0.03)
        .loft(combine=True)
        .val()
    )
    hub = cq.Workplane("YZ", origin=(2.00, pod_center_y, pod_center_z)).circle(0.10).extrude(0.18).val()
    pod_ring = cq.Workplane("YZ", origin=(0.18, pod_center_y, pod_center_z)).circle(0.63).circle(0.56).extrude(0.18).val()

    assembly = cq.Assembly(name=f"engine_pod_{0 if side_sign > 0 else 1}")
    assembly.add(pod, name="pod")
    assembly.add(spinner, name="spinner")
    assembly.add(hub, name="hub")
    assembly.add(pod_ring, name="pod_ring")
    return assembly


def _build_engine_mount(side_sign: float):
    leading_pylon = _cylinder_between((0.0, 0.0, 0.0), (1.10, side_sign * 1.42, -0.92), 0.085)
    trailing_pylon = _cylinder_between((0.0, 0.0, 0.0), (-0.20, side_sign * 1.40, -0.92), 0.085)
    root_fairing = cq.Workplane("XY").box(0.42, 0.26, 0.18).translate((0.02, side_sign * 0.13, -0.09)).val()

    assembly = cq.Assembly(name=f"engine_mount_{0 if side_sign > 0 else 1}")
    assembly.add(leading_pylon, name="leading_pylon")
    assembly.add(trailing_pylon, name="trailing_pylon")
    assembly.add(root_fairing, name="root_fairing")
    return assembly


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="blimp")

    envelope_white = model.material("envelope_white", rgba=(0.90, 0.92, 0.95, 1.0))
    navy = model.material("navy", rgba=(0.11, 0.18, 0.34, 1.0))
    gondola_blue = model.material("gondola_blue", rgba=(0.22, 0.34, 0.54, 1.0))
    glass = model.material("glass", rgba=(0.48, 0.68, 0.82, 0.42))
    engine_grey = model.material("engine_grey", rgba=(0.56, 0.59, 0.63, 1.0))
    prop_black = model.material("prop_black", rgba=(0.09, 0.10, 0.11, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_envelope(), "blimp_envelope"),
        material=envelope_white,
        name="envelope",
    )
    body.visual(
        mesh_from_cadquery(_build_tail_assembly(), "blimp_tail"),
        material=navy,
        name="tail",
    )
    body.visual(
        mesh_from_cadquery(_build_keel(), "blimp_keel"),
        material=navy,
        name="keel",
    )

    gondola = model.part("gondola")
    gondola.visual(
        mesh_from_cadquery(_build_gondola_body(), "blimp_gondola_body"),
        material=gondola_blue,
        name="cabin_body",
    )
    gondola.visual(
        mesh_from_cadquery(_build_gondola_support(), "blimp_gondola_support"),
        material=engine_grey,
        name="support_frame",
    )
    gondola.visual(
        mesh_from_cadquery(
            cq.Workplane("XY").box(0.10, 1.05, 0.48).translate((2.34, 0.0, -1.46)).val(),
            "blimp_gondola_glass",
        ),
        material=glass,
        name="windshield",
    )

    engine_0 = model.part("engine_0")
    engine_0.visual(
        mesh_from_cadquery(_build_engine_pod(1.0), "blimp_engine_0_shell"),
        material=engine_grey,
        name="pod_shell",
    )
    engine_0.visual(
        mesh_from_cadquery(_build_engine_mount(1.0), "blimp_engine_0_mount"),
        material=engine_grey,
        name="mount_frame",
    )
    engine_0.visual(
        mesh_from_cadquery(
            cq.Assembly()
            .add(cq.Workplane("XY").box(0.08, 2.45, 0.16).translate((2.12, 1.55, -1.32)).val())
            .add(cq.Workplane("XY").box(0.08, 0.16, 2.45).translate((2.12, 1.55, -1.32)).val()),
            "blimp_engine_0_prop",
        ),
        material=prop_black,
        name="propeller",
    )

    engine_1 = model.part("engine_1")
    engine_1.visual(
        mesh_from_cadquery(_build_engine_pod(-1.0), "blimp_engine_1_shell"),
        material=engine_grey,
        name="pod_shell",
    )
    engine_1.visual(
        mesh_from_cadquery(_build_engine_mount(-1.0), "blimp_engine_1_mount"),
        material=engine_grey,
        name="mount_frame",
    )
    engine_1.visual(
        mesh_from_cadquery(
            cq.Assembly()
            .add(cq.Workplane("XY").box(0.08, 2.45, 0.16).translate((2.12, -1.55, -1.32)).val())
            .add(cq.Workplane("XY").box(0.08, 0.16, 2.45).translate((2.12, -1.55, -1.32)).val()),
            "blimp_engine_1_prop",
        ),
        material=prop_black,
        name="propeller",
    )

    model.articulation(
        "body_to_gondola",
        ArticulationType.FIXED,
        parent=body,
        child=gondola,
        origin=Origin(xyz=(-0.9, 0.0, -4.70)),
    )
    model.articulation(
        "body_to_engine_0",
        ArticulationType.FIXED,
        parent=body,
        child=engine_0,
        origin=Origin(xyz=(1.4, 4.50, -1.42)),
    )
    model.articulation(
        "body_to_engine_1",
        ArticulationType.FIXED,
        parent=body,
        child=engine_1,
        origin=Origin(xyz=(1.4, -4.50, -1.42)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    gondola = object_model.get_part("gondola")
    engine_0 = object_model.get_part("engine_0")
    engine_1 = object_model.get_part("engine_1")

    ctx.allow_overlap(
        body,
        gondola,
        elem_a="envelope",
        elem_b="support_frame",
        reason="The gondola roof saddle is intentionally faired into the simplified envelope skin to represent its structural mount.",
    )
    ctx.allow_overlap(
        body,
        gondola,
        elem_a="keel",
        elem_b="support_frame",
        reason="The gondola support frame intentionally blends into the keel fairing rather than stopping at a literal seam.",
    )
    ctx.allow_overlap(
        body,
        engine_0,
        elem_a="envelope",
        elem_b="mount_frame",
        reason="The starboard engine pylon root is intentionally blended into the simplified envelope shell at its mount.",
    )
    ctx.allow_overlap(
        body,
        engine_1,
        elem_a="envelope",
        elem_b="mount_frame",
        reason="The port engine pylon root is intentionally blended into the simplified envelope shell at its mount.",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_aabb_available", "The body AABB could not be measured.")
    else:
        length = body_aabb[1][0] - body_aabb[0][0]
        height = body_aabb[1][2] - body_aabb[0][2]
        width = body_aabb[1][1] - body_aabb[0][1]
        ctx.check(
            "blimp_has_airship_scale",
            length > 37.0 and width > 9.0 and height > 9.0,
            details=f"length={length:.3f}, width={width:.3f}, height={height:.3f}",
        )

    ctx.expect_gap(
        body,
        gondola,
        axis="z",
        min_gap=0.65,
        positive_elem="envelope",
        negative_elem="cabin_body",
        name="gondola_hangs_below_envelope",
    )
    ctx.expect_origin_distance(
        gondola,
        body,
        axes="y",
        max_dist=0.05,
        name="gondola_stays_on_centerline",
    )
    ctx.expect_origin_gap(
        engine_0,
        body,
        axis="y",
        min_gap=3.8,
        name="engine_0_mounts_on_positive_y_side",
    )
    ctx.expect_origin_gap(
        body,
        engine_1,
        axis="y",
        min_gap=3.8,
        name="engine_1_mounts_on_negative_y_side",
    )
    ctx.expect_origin_distance(
        engine_0,
        engine_1,
        axes="y",
        min_dist=7.5,
        name="twin_engines_span_both_sides",
    )

    return ctx.report()


object_model = build_object_model()
