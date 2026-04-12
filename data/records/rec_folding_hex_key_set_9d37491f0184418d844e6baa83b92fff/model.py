from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LEN = 0.074
BODY_WIDTH = 0.030
BODY_THICK = 0.018
BODY_EDGE_R = 0.0035

POCKET_LEN = 0.030
POCKET_WIDTH = 0.019
POCKET_DEPTH = 0.0045
POCKET_CENTER_X = 0.016
PIVOT_X = 0.031

KEY_DIAMETERS = (0.0015, 0.0017, 0.0019, 0.0021)
KEY_LONGS = (0.020, 0.023, 0.026, 0.029)
KEY_SHORTS = (0.0055, 0.0065, 0.0075, 0.0085)
KEY_GAP = 0.00035
PIN_RADIUS = 0.0025
PIN_CAP_RADIUS = 0.0040
PIN_CAP_THICK = 0.0012
PIN_EMBED = 0.0014
KEY_ELBOW_SHIFT = 0.0012

STACK_HEIGHT = sum(KEY_DIAMETERS) + KEY_GAP * (len(KEY_DIAMETERS) - 1)
AXIS_Z = BODY_THICK / 2 - POCKET_DEPTH + STACK_HEIGHT / 2
WING_UPPER = math.radians(135.0)


def _layer_centers(diameters: tuple[float, ...], gap: float) -> list[float]:
    total = sum(diameters) + gap * (len(diameters) - 1)
    cursor = -total / 2
    centers: list[float] = []
    for diameter in diameters:
        centers.append(cursor + diameter / 2)
        cursor += diameter + gap
    return centers


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_LEN, BODY_WIDTH, BODY_THICK)
    body = body.edges("|Z").fillet(BODY_EDGE_R)

    left_pocket = (
        cq.Workplane("XY")
        .box(POCKET_LEN, POCKET_WIDTH, POCKET_DEPTH)
        .translate((-POCKET_CENTER_X, 0.0, BODY_THICK / 2 - POCKET_DEPTH / 2))
    )
    right_pocket = (
        cq.Workplane("XY")
        .box(POCKET_LEN, POCKET_WIDTH, POCKET_DEPTH)
        .translate((POCKET_CENTER_X, 0.0, BODY_THICK / 2 - POCKET_DEPTH / 2))
    )

    grip_relief = (
        cq.Workplane("YZ")
        .circle(0.012)
        .extrude(0.020)
        .translate((-0.010, 0.020, 0.0))
    ).union(
        cq.Workplane("YZ")
        .circle(0.012)
        .extrude(0.020)
        .translate((-0.010, -0.040, 0.0))
    )

    return body.cut(left_pocket).cut(right_pocket).cut(grip_relief)


def _key_shape(
    *,
    rod_diameter: float,
    long_length: float,
    short_length: float,
    layer_z: float,
) -> cq.Workplane:
    rod_radius = rod_diameter / 2
    collar_thick = rod_diameter * 0.65
    collar_radius = max(rod_radius * 1.9, PIN_RADIUS + 0.00045)
    elbow_x = long_length - KEY_ELBOW_SHIFT

    long_bar = (
        cq.Workplane("YZ")
        .circle(rod_radius)
        .extrude(long_length + PIN_EMBED)
        .translate((-PIN_EMBED, 0.0, layer_z))
    )
    short_bar = (
        cq.Workplane("XZ")
        .circle(rod_radius)
        .extrude(short_length)
        .translate((elbow_x, 0.0, layer_z))
    )
    elbow = cq.Workplane("XY").sphere(rod_radius * 1.05).translate(
        (elbow_x, rod_radius * 1.1, layer_z)
    )
    collar = (
        cq.Workplane("XY")
        .circle(collar_radius)
        .extrude(collar_thick)
        .translate((0.0, 0.0, layer_z - collar_thick / 2))
    )

    return long_bar.union(short_bar).union(elbow).union(collar)


def _wing_key_shapes(*, mirrored: bool) -> list[tuple[str, cq.Workplane]]:
    shapes: list[tuple[str, cq.Workplane]] = []
    for index, (diameter, long_length, short_length, layer_z) in enumerate(
        zip(KEY_DIAMETERS, KEY_LONGS, KEY_SHORTS, _layer_centers(KEY_DIAMETERS, KEY_GAP))
    ):
        shape = _key_shape(
            rod_diameter=diameter,
            long_length=long_length,
            short_length=short_length,
            layer_z=layer_z,
        )
        if mirrored:
            shape = shape.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 180.0)
        shapes.append((f"key_{index}", shape))
    return shapes


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_wing_hex_key_set")

    body_mat = model.material("body_polymer", rgba=(0.17, 0.17, 0.18, 1.0))
    key_mat = model.material("key_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    pin_mat = model.material("pin_oxide", rgba=(0.22, 0.22, 0.23, 1.0))
    cap_mat = model.material("cap_oxide", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=body_mat,
        name="shell",
    )

    left_stack = model.part("left_stack")
    left_stack.visual(
        Cylinder(radius=PIN_RADIUS, length=STACK_HEIGHT + 2 * PIN_CAP_THICK),
        material=pin_mat,
        name="pivot_pin",
    )
    left_stack.visual(
        Cylinder(radius=PIN_CAP_RADIUS, length=PIN_CAP_THICK),
        origin=Origin(xyz=(0.0, 0.0, STACK_HEIGHT / 2 + PIN_CAP_THICK / 2)),
        material=cap_mat,
        name="top_cap",
    )
    left_stack.visual(
        Cylinder(radius=PIN_CAP_RADIUS, length=PIN_CAP_THICK),
        origin=Origin(xyz=(0.0, 0.0, -STACK_HEIGHT / 2 - PIN_CAP_THICK / 2)),
        material=cap_mat,
        name="bottom_cap",
    )
    for key_name, key_shape in _wing_key_shapes(mirrored=False):
        left_stack.visual(
            mesh_from_cadquery(key_shape, f"left_{key_name}"),
            material=key_mat,
            name=key_name,
        )

    right_stack = model.part("right_stack")
    right_stack.visual(
        Cylinder(radius=PIN_RADIUS, length=STACK_HEIGHT + 2 * PIN_CAP_THICK),
        material=pin_mat,
        name="pivot_pin",
    )
    right_stack.visual(
        Cylinder(radius=PIN_CAP_RADIUS, length=PIN_CAP_THICK),
        origin=Origin(xyz=(0.0, 0.0, STACK_HEIGHT / 2 + PIN_CAP_THICK / 2)),
        material=cap_mat,
        name="top_cap",
    )
    right_stack.visual(
        Cylinder(radius=PIN_CAP_RADIUS, length=PIN_CAP_THICK),
        origin=Origin(xyz=(0.0, 0.0, -STACK_HEIGHT / 2 - PIN_CAP_THICK / 2)),
        material=cap_mat,
        name="bottom_cap",
    )
    for key_name, key_shape in _wing_key_shapes(mirrored=True):
        right_stack.visual(
            mesh_from_cadquery(key_shape, f"right_{key_name}"),
            material=key_mat,
            name=key_name,
        )

    model.articulation(
        "body_to_left_stack",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_stack,
        origin=Origin(xyz=(-PIVOT_X, 0.0, AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=WING_UPPER,
        ),
    )
    model.articulation(
        "body_to_right_stack",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_stack,
        origin=Origin(xyz=(PIVOT_X, 0.0, AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=WING_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_stack = object_model.get_part("left_stack")
    right_stack = object_model.get_part("right_stack")
    left_joint = object_model.get_articulation("body_to_left_stack")
    right_joint = object_model.get_articulation("body_to_right_stack")

    ctx.expect_within(
        left_stack,
        body,
        axes="xy",
        margin=0.0020,
        name="left stack folds within the body footprint",
    )
    ctx.expect_within(
        right_stack,
        body,
        axes="xy",
        margin=0.0020,
        name="right stack folds within the body footprint",
    )

    body_aabb = ctx.part_world_aabb(body)

    left_limits = left_joint.motion_limits
    if left_limits is not None and left_limits.upper is not None:
        with ctx.pose({left_joint: left_limits.upper, right_joint: 0.0}):
            ctx.expect_within(
                right_stack,
                body,
                axes="xy",
                margin=0.0020,
                name="right stack can stay tucked while left stack opens",
            )
            left_open = ctx.part_world_aabb(left_stack)
            ctx.check(
                "left stack swings outward from the grip",
                body_aabb is not None
                and left_open is not None
                and left_open[1][1] > body_aabb[1][1] + 0.006,
                details=f"body_aabb={body_aabb}, left_open={left_open}",
            )

    right_limits = right_joint.motion_limits
    if right_limits is not None and right_limits.upper is not None:
        with ctx.pose({left_joint: 0.0, right_joint: right_limits.upper}):
            ctx.expect_within(
                left_stack,
                body,
                axes="xy",
                margin=0.0020,
                name="left stack can stay tucked while right stack opens",
            )
            right_open = ctx.part_world_aabb(right_stack)
            ctx.check(
                "right stack swings outward from the grip",
                body_aabb is not None
                and right_open is not None
                and right_open[0][1] < body_aabb[0][1] - 0.006,
                details=f"body_aabb={body_aabb}, right_open={right_open}",
            )

    return ctx.report()


object_model = build_object_model()
