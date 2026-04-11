from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


STEEL_THICKNESS = 0.008
GRIP_THICKNESS = 0.010
LOWER_STEEL_Z = 0.004
UPPER_STEEL_Z = -0.004
LOWER_GRIP_Z = 0.0085
UPPER_GRIP_Z = -0.0085
BOSS_RADIUS = 0.018
SHAFT_RADIUS = 0.0054
HOLE_RADIUS = 0.0068
OPEN_LIMIT = 0.90


def _poly_extrude(points: list[tuple[float, float]], thickness: float):
    return cq.Workplane("XY").polyline(points).close().extrude(thickness / 2.0, both=True)


def _member_blank(thickness: float):
    jaw = _poly_extrude(
        [
            (0.000, 0.0105),
            (0.050, 0.0068),
            (0.082, 0.0040),
            (0.105, 0.0021),
            (0.105, -0.0021),
            (0.082, -0.0040),
            (0.050, -0.0068),
            (0.000, -0.0105),
        ],
        thickness,
    )
    handle = _poly_extrude(
        [
            (0.018, 0.0120),
            (-0.028, 0.0155),
            (-0.078, 0.0150),
            (-0.114, 0.0134),
            (-0.137, 0.0100),
            (-0.137, -0.0100),
            (-0.114, -0.0134),
            (-0.078, -0.0150),
            (-0.028, -0.0155),
            (0.018, -0.0120),
        ],
        thickness,
    )
    boss = cq.Workplane("XY").circle(BOSS_RADIUS).extrude(thickness / 2.0, both=True)
    nose = (
        cq.Workplane("XY")
        .circle(0.0021)
        .extrude(thickness / 2.0, both=True)
        .translate((0.105, 0.0, 0.0))
    )
    return jaw.union(handle).union(boss).union(nose)


def _upper_member_steel():
    blank = _member_blank(STEEL_THICKNESS)
    hole = cq.Workplane("XY").circle(HOLE_RADIUS).extrude(STEEL_THICKNESS, both=True)
    collar = (
        cq.Workplane("XY")
        .circle(0.0115)
        .circle(HOLE_RADIUS)
        .extrude(0.0008 / 2.0, both=True)
        .translate((0.0, 0.0, -0.0036))
    )
    return blank.cut(hole).union(collar)


def _lower_member_steel():
    return _member_blank(STEEL_THICKNESS)


def _rivet_shape():
    shaft = (
        cq.Workplane("XY")
        .circle(SHAFT_RADIUS)
        .extrude(0.012 / 2.0, both=True)
        .translate((0.0, 0.0, -0.0020))
    )
    head = (
        cq.Workplane("XY")
        .circle(0.0105)
        .extrude(0.0028 / 2.0, both=True)
        .translate((0.0, 0.0, 0.0054))
    )
    return shaft.union(head)


def _grip_shape():
    grip = _poly_extrude(
        [
            (-0.018, 0.0140),
            (-0.050, 0.0188),
            (-0.094, 0.0180),
            (-0.124, 0.0164),
            (-0.133, 0.0142),
            (-0.133, -0.0142),
            (-0.124, -0.0164),
            (-0.094, -0.0180),
            (-0.050, -0.0188),
            (-0.018, -0.0140),
        ],
        GRIP_THICKNESS,
    )
    butt = (
        cq.Workplane("XY")
        .circle(0.0138)
        .extrude(GRIP_THICKNESS / 2.0, both=True)
        .translate((-0.133, 0.0, 0.0))
    )
    grip = grip.union(butt)

    for groove_x in (-0.118, -0.101, -0.084, -0.067, -0.050):
        top_cut = cq.Workplane("XY").box(0.003, 0.045, 0.0026).translate((groove_x, 0.0, 0.0042))
        bottom_cut = cq.Workplane("XY").box(0.003, 0.045, 0.0026).translate((groove_x, 0.0, -0.0042))
        grip = grip.cut(top_cut).cut(bottom_cut)

    return grip


def _add_member_visuals(
    part,
    *,
    steel_shape,
    steel_z: float,
    grip_z: float,
    mesh_prefix: str,
    steel_material,
    grip_material,
    add_rivet: bool = False,
) -> None:
    part.visual(
        mesh_from_cadquery(steel_shape, f"{mesh_prefix}_steel"),
        origin=Origin(xyz=(0.0, 0.0, steel_z)),
        material=steel_material,
        name="steel",
    )
    part.visual(
        mesh_from_cadquery(_grip_shape(), f"{mesh_prefix}_grip"),
        origin=Origin(xyz=(0.0, 0.0, grip_z)),
        material=grip_material,
        name="grip",
    )
    part.visual(
        Box((0.040, 0.0080, 0.0060)),
        origin=Origin(xyz=(0.080, 0.0, steel_z)),
        material=steel_material,
        name="tip",
    )
    if add_rivet:
        part.visual(
            mesh_from_cadquery(_rivet_shape(), f"{mesh_prefix}_rivet"),
            origin=Origin(xyz=(0.0, 0.0, steel_z)),
            material=steel_material,
            name="rivet",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="needle_nose_pliers")

    satin_steel = model.material("satin_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.34, 0.35, 0.37, 1.0))
    grip_red = model.material("grip_red", rgba=(0.70, 0.14, 0.11, 1.0))

    lower_member = model.part("lower_member")
    _add_member_visuals(
        lower_member,
        steel_shape=_lower_member_steel(),
        steel_z=LOWER_STEEL_Z,
        grip_z=LOWER_GRIP_Z,
        mesh_prefix="lower_member",
        steel_material=satin_steel,
        grip_material=grip_red,
        add_rivet=True,
    )
    lower_member.visual(
        Box((0.024, 0.010, 0.0030)),
        origin=Origin(xyz=(-0.010, 0.0, LOWER_STEEL_Z + 0.0032)),
        material=dark_steel,
        name="pivot_cap",
    )

    upper_member = model.part("upper_member")
    _add_member_visuals(
        upper_member,
        steel_shape=_upper_member_steel(),
        steel_z=UPPER_STEEL_Z,
        grip_z=UPPER_GRIP_Z,
        mesh_prefix="upper_member",
        steel_material=satin_steel,
        grip_material=grip_red,
    )

    model.articulation(
        "plier_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_member,
        child=upper_member,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=4.0,
            lower=0.0,
            upper=OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_member = object_model.get_part("lower_member")
    upper_member = object_model.get_part("upper_member")
    pivot = object_model.get_articulation("plier_pivot")
    limits = pivot.motion_limits

    ctx.expect_gap(
        lower_member,
        upper_member,
        axis="z",
        positive_elem="steel",
        negative_elem="steel",
        max_gap=0.0005,
        max_penetration=0.0,
        name="steel members close onto a common working plane",
    )
    ctx.expect_overlap(
        lower_member,
        upper_member,
        axes="x",
        elem_a="steel",
        elem_b="steel",
        min_overlap=0.22,
        name="closed members share the full plier length",
    )
    ctx.expect_overlap(
        lower_member,
        upper_member,
        axes="y",
        elem_a="tip",
        elem_b="tip",
        min_overlap=0.003,
        name="closed jaws align at the nose",
    )

    if limits is not None and limits.upper is not None:
        rest_tip = ctx.part_element_world_aabb(upper_member, elem="tip")
        with ctx.pose({pivot: limits.upper}):
            open_tip = ctx.part_element_world_aabb(upper_member, elem="tip")
            ctx.expect_gap(
                upper_member,
                lower_member,
                axis="y",
                positive_elem="tip",
                negative_elem="tip",
                min_gap=0.040,
                name="opened jaws separate laterally at the tip",
            )

        ctx.check(
            "upper jaw swings toward positive y",
            rest_tip is not None and open_tip is not None and open_tip[0][1] > rest_tip[0][1] + 0.040,
            details=f"rest_tip={rest_tip}, open_tip={open_tip}",
        )

    return ctx.report()


object_model = build_object_model()
