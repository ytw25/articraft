from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_LENGTH = 0.090
PIVOT_OFFSET_X = 0.039
PLATE_THICKNESS = 0.0025
PLATE_HEIGHT = 0.034
PLATE_CENTER_Y = 0.015
END_BLOCK_LENGTH = 0.012
END_BLOCK_WIDTH = 0.028
END_BLOCK_HEIGHT = 0.006
END_BLOCK_Z = 0.011
GRIP_LENGTH = 0.046
GRIP_THICKNESS = 0.004
GRIP_HEIGHT = 0.024
GRIP_CENTER_Y = 0.01725
SCREW_HEAD_RADIUS = 0.0038
SCREW_HEAD_THICKNESS = 0.002


def _hex_profile(across_flats: float) -> list[tuple[float, float]]:
    radius = across_flats / math.sqrt(3.0)
    return [
        (
            radius * math.cos(math.pi / 6.0 + index * math.pi / 3.0),
            radius * math.sin(math.pi / 6.0 + index * math.pi / 3.0),
        )
        for index in range(6)
    ]


def _l_key_mesh(
    *,
    name: str,
    long_arm: float,
    short_arm: float,
    across_flats: float,
    long_dir: float,
    short_dir: float,
):
    profile = _hex_profile(across_flats)

    long_geom = ExtrudeGeometry.from_z0(profile, long_arm)
    long_geom.rotate_y(math.pi / 2.0 if long_dir > 0.0 else -math.pi / 2.0)

    short_geom = ExtrudeGeometry.from_z0(profile, short_arm)
    if short_dir < 0.0:
        short_geom.rotate_x(math.pi)
    short_geom.translate(long_dir * long_arm, 0.0, 0.0)

    elbow_geom = SphereGeometry(across_flats * 0.70)
    elbow_geom.translate(long_dir * long_arm, 0.0, 0.0)

    long_geom.merge(short_geom)
    long_geom.merge(elbow_geom)
    return mesh_from_geometry(long_geom, name)


def _add_handle_plate(part, *, y_center: float, material, prefix: str) -> None:
    part.visual(
        Box((BODY_LENGTH - 0.012, PLATE_THICKNESS, PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, y_center, 0.0)),
        material=material,
        name=f"{prefix}_plate_mid",
    )
    for suffix, x_center in (("left", -PIVOT_OFFSET_X), ("right", PIVOT_OFFSET_X)):
        part.visual(
            Cylinder(radius=PLATE_HEIGHT * 0.5, length=PLATE_THICKNESS),
            origin=Origin(
                xyz=(x_center, y_center, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_plate_{suffix}_cap",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_hex_key_set")

    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.10, 0.10, 0.11, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.18, 0.19, 0.20, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.48, 0.50, 0.53, 1.0))

    body = model.part("body")
    _add_handle_plate(body, y_center=-PLATE_CENTER_Y, material=steel, prefix="neg")
    _add_handle_plate(body, y_center=PLATE_CENTER_Y, material=steel, prefix="pos")

    for x_center, side_name in ((-PIVOT_OFFSET_X, "left"), (PIVOT_OFFSET_X, "right")):
        for z_center, z_name in ((-END_BLOCK_Z, "lower"), (END_BLOCK_Z, "upper")):
            body.visual(
                Box((END_BLOCK_LENGTH, END_BLOCK_WIDTH, END_BLOCK_HEIGHT)),
                origin=Origin(xyz=(x_center, 0.0, z_center)),
                material=steel,
                name=f"{side_name}_stack_{z_name}_bridge",
            )
        body.visual(
            Cylinder(radius=0.0021, length=END_BLOCK_WIDTH + 0.004),
            origin=Origin(
                xyz=(x_center, 0.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=screw_steel,
            name=f"{side_name}_pivot_pin",
        )

    for y_center, prefix in ((-GRIP_CENTER_Y, "neg"), (GRIP_CENTER_Y, "pos")):
        body.visual(
            Box((GRIP_LENGTH, GRIP_THICKNESS, GRIP_HEIGHT)),
            origin=Origin(xyz=(0.0, y_center, 0.0)),
            material=grip_rubber,
            name=f"{prefix}_grip_pad",
        )
        body.visual(
            Cylinder(radius=GRIP_HEIGHT * 0.5, length=GRIP_THICKNESS),
            origin=Origin(
                xyz=(-GRIP_LENGTH * 0.5, y_center, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=grip_rubber,
            name=f"{prefix}_grip_left_cap",
        )
        body.visual(
            Cylinder(radius=GRIP_HEIGHT * 0.5, length=GRIP_THICKNESS),
            origin=Origin(
                xyz=(GRIP_LENGTH * 0.5, y_center, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=grip_rubber,
            name=f"{prefix}_grip_right_cap",
        )

    screw_head_y = PLATE_CENTER_Y + (PLATE_THICKNESS + SCREW_HEAD_THICKNESS) * 0.5 - 0.0002
    for x_center, side_name in ((-PIVOT_OFFSET_X, "left"), (PIVOT_OFFSET_X, "right")):
        for y_center, y_name in ((-screw_head_y, "neg"), (screw_head_y, "pos")):
            body.visual(
                Cylinder(radius=SCREW_HEAD_RADIUS, length=SCREW_HEAD_THICKNESS),
                origin=Origin(
                    xyz=(x_center, y_center, 0.0),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=screw_steel,
                name=f"{side_name}_stack_{y_name}_screw",
            )

    left_keys = [
        ("left_key_0", -0.0090, 0.0025, 0.048, 0.012),
        ("left_key_1", -0.0055, 0.0030, 0.054, 0.0135),
        ("left_key_2", -0.0015, 0.0035, 0.060, 0.0150),
    ]
    right_keys = [
        ("right_key_0", 0.0042, 0.0040, 0.062, 0.0150),
        ("right_key_1", 0.0098, 0.0050, 0.068, 0.0160),
    ]

    for part_name, y_center, flats, long_arm, short_arm in left_keys:
        key = model.part(part_name)
        key.visual(
            _l_key_mesh(
                name=f"{part_name}_mesh",
                long_arm=long_arm,
                short_arm=short_arm,
                across_flats=flats,
                long_dir=1.0,
                short_dir=-1.0,
            ),
            material=black_oxide,
            name="key",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=key,
            origin=Origin(xyz=(-PIVOT_OFFSET_X, y_center, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=3.0,
                lower=0.0,
                upper=3.20,
            ),
        )

    for part_name, y_center, flats, long_arm, short_arm in right_keys:
        key = model.part(part_name)
        key.visual(
            _l_key_mesh(
                name=f"{part_name}_mesh",
                long_arm=long_arm,
                short_arm=short_arm,
                across_flats=flats,
                long_dir=-1.0,
                short_dir=1.0,
            ),
            material=black_oxide,
            name="key",
        )
        model.articulation(
            f"body_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=key,
            origin=Origin(xyz=(PIVOT_OFFSET_X, y_center, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=3.0,
                lower=0.0,
                upper=3.20,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    for key_name in ("left_key_0", "left_key_1", "left_key_2"):
        ctx.allow_overlap(
            body,
            key_name,
            elem_a="left_pivot_pin",
            elem_b="key",
            reason="The left stack keys are intentionally represented as solid hex bars passing through the shared left pivot pin instead of modeling drilled pin holes.",
        )
    for key_name in ("right_key_0", "right_key_1"):
        ctx.allow_overlap(
            body,
            key_name,
            elem_a="right_pivot_pin",
            elem_b="key",
            reason="The right stack keys are intentionally represented as solid hex bars passing through the shared right pivot pin instead of modeling drilled pin holes.",
        )

    folded_keys = [
        object_model.get_part("left_key_0"),
        object_model.get_part("left_key_1"),
        object_model.get_part("left_key_2"),
        object_model.get_part("right_key_0"),
        object_model.get_part("right_key_1"),
    ]

    for key in folded_keys:
        ctx.expect_within(
            key,
            body,
            axes="yz",
            margin=0.001,
            name=f"{key.name} stays within folded handle envelope",
        )
        ctx.expect_overlap(
            key,
            body,
            axes="x",
            min_overlap=0.040,
            name=f"{key.name} remains substantially nested in the handle",
        )

    left_joint = object_model.get_articulation("body_to_left_key_2")
    right_joint = object_model.get_articulation("body_to_right_key_1")
    left_key = object_model.get_part("left_key_2")
    right_key = object_model.get_part("right_key_1")

    left_upper = left_joint.motion_limits.upper if left_joint.motion_limits is not None else None
    right_upper = right_joint.motion_limits.upper if right_joint.motion_limits is not None else None

    if left_upper is not None:
        with ctx.pose({left_joint: left_upper}):
            body_aabb = ctx.part_world_aabb(body)
            left_aabb = ctx.part_world_aabb(left_key)
            left_outward = (
                body_aabb is not None
                and left_aabb is not None
                and left_aabb[0][0] < body_aabb[0][0] - 0.018
            )
            ctx.check(
                "left stack can swing a long key out of the left end",
                left_outward,
                details=f"body_aabb={body_aabb}, left_aabb={left_aabb}",
            )

    if right_upper is not None:
        with ctx.pose({right_joint: right_upper}):
            body_aabb = ctx.part_world_aabb(body)
            right_aabb = ctx.part_world_aabb(right_key)
            right_outward = (
                body_aabb is not None
                and right_aabb is not None
                and right_aabb[1][0] > body_aabb[1][0] + 0.018
            )
            ctx.check(
                "right stack can swing a long key out of the right end",
                right_outward,
                details=f"body_aabb={body_aabb}, right_aabb={right_aabb}",
            )

    if left_upper is not None and right_upper is not None:
        with ctx.pose({left_joint: left_upper, right_joint: right_upper}):
            ctx.expect_gap(
                right_key,
                left_key,
                axis="x",
                min_gap=0.060,
                name="opposed long keys can deploy simultaneously from opposite pivots",
            )

    return ctx.report()


object_model = build_object_model()
