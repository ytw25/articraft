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


KEY_SPECS = (
    {"size_mm": 2.0, "long_mm": 34.0, "short_mm": 6.5, "upper": 1.15},
    {"size_mm": 2.5, "long_mm": 38.0, "short_mm": 7.5, "upper": 1.35},
    {"size_mm": 3.0, "long_mm": 43.0, "short_mm": 8.5, "upper": 1.55},
    {"size_mm": 4.0, "long_mm": 50.0, "short_mm": 9.5, "upper": 1.80},
    {"size_mm": 5.0, "long_mm": 57.0, "short_mm": 10.5, "upper": 2.05},
    {"size_mm": 6.0, "long_mm": 64.0, "short_mm": 11.5, "upper": 2.30},
)

PLATE_LENGTH_MM = 68.0
PLATE_WIDTH_MM = 26.0
PLATE_THICKNESS_MM = 2.4
STACK_CLEARANCE_MM = 0.35
PIVOT_RADIUS_MM = 2.25
REAR_POST_RADIUS_MM = 2.8
REAR_POST_X_MM = 55.0
REAR_POST_Z_MM = 7.5


def _hex_points(across_flats_mm: float) -> list[tuple[float, float]]:
    radius = across_flats_mm / math.sqrt(3.0)
    half_flats = across_flats_mm * 0.5
    return [
        (-radius, 0.0),
        (-radius * 0.5, half_flats),
        (radius * 0.5, half_flats),
        (radius, 0.0),
        (radius * 0.5, -half_flats),
        (-radius * 0.5, -half_flats),
    ]


def _centered_extrusion(shape: cq.Workplane, span_mm: float, axis: str) -> cq.Workplane:
    if axis == "x":
        return shape.translate((-span_mm * 0.5, 0.0, 0.0))
    if axis == "y":
        return shape.translate((0.0, span_mm * 0.5, 0.0))
    return shape.translate((0.0, 0.0, -span_mm * 0.5))


def _hex_prism_x(across_flats_mm: float, length_mm: float, *, start_x_mm: float = 0.0) -> cq.Workplane:
    prism = cq.Workplane("YZ").polyline(_hex_points(across_flats_mm)).close().extrude(length_mm)
    return prism.translate((start_x_mm, 0.0, 0.0))


def _hex_prism_z(
    across_flats_mm: float,
    length_mm: float,
    *,
    x_mm: float = 0.0,
    top_z_mm: float = 0.0,
) -> cq.Workplane:
    prism = cq.Workplane("XY").polyline(_hex_points(across_flats_mm)).close().extrude(length_mm)
    return prism.translate((x_mm, 0.0, top_z_mm - length_mm))


def _make_plate_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .center(PLATE_LENGTH_MM * 0.5, 0.0)
        .slot2D(PLATE_LENGTH_MM, PLATE_WIDTH_MM)
        .extrude(PLATE_THICKNESS_MM)
    )
    plate = _centered_extrusion(outer, PLATE_THICKNESS_MM, "y")

    window = (
        cq.Workplane("XZ")
        .center(39.0, 0.0)
        .slot2D(30.0, 10.0)
        .extrude(PLATE_THICKNESS_MM + 2.0)
    )
    plate = plate.cut(_centered_extrusion(window, PLATE_THICKNESS_MM + 2.0, "y"))

    for hole_x_mm, hole_z_mm, hole_radius_mm in (
        (0.0, 0.0, PIVOT_RADIUS_MM - 0.08),
        (REAR_POST_X_MM, REAR_POST_Z_MM, REAR_POST_RADIUS_MM - 0.08),
        (REAR_POST_X_MM, -REAR_POST_Z_MM, REAR_POST_RADIUS_MM - 0.08),
    ):
        hole = (
            cq.Workplane("XZ")
            .center(hole_x_mm, hole_z_mm)
            .circle(hole_radius_mm)
            .extrude(PLATE_THICKNESS_MM + 2.0)
        )
        plate = plate.cut(_centered_extrusion(hole, PLATE_THICKNESS_MM + 2.0, "y"))

    return plate


def _make_y_cylinder(radius_mm: float, length_mm: float) -> cq.Workplane:
    cylinder = cq.Workplane("XZ").circle(radius_mm).extrude(length_mm)
    return _centered_extrusion(cylinder, length_mm, "y")


def _make_key_boss(size_mm: float) -> cq.Workplane:
    outer_radius_mm = max(4.2, PIVOT_RADIUS_MM + 1.4, size_mm * 0.78 + 2.0)
    boss = cq.Workplane("XZ").circle(outer_radius_mm).extrude(size_mm)
    boss = _centered_extrusion(boss, size_mm, "y")
    hole = cq.Workplane("XZ").circle(PIVOT_RADIUS_MM + 0.25).extrude(size_mm + 2.0)
    hole = _centered_extrusion(hole, size_mm + 2.0, "y")
    return boss.cut(hole)


def _make_key_body(size_mm: float, long_mm: float, short_mm: float) -> cq.Workplane:
    boss_outer_radius_mm = max(4.2, PIVOT_RADIUS_MM + 1.4, size_mm * 0.78 + 2.0)
    bridge_start_x_mm = PIVOT_RADIUS_MM + 0.65
    elbow_x_mm = boss_outer_radius_mm + max(1.6, size_mm * 0.28)

    long_arm = _hex_prism_x(size_mm, long_mm - bridge_start_x_mm, start_x_mm=bridge_start_x_mm)
    short_arm = _hex_prism_z(size_mm, short_mm, x_mm=elbow_x_mm, top_z_mm=0.0)
    return long_arm.union(short_arm)


def _key_slot_centers_mm() -> list[float]:
    total_stack_mm = sum(spec["size_mm"] for spec in KEY_SPECS) + STACK_CLEARANCE_MM * (len(KEY_SPECS) - 1)
    centers: list[float] = []
    cursor_mm = -total_stack_mm * 0.5
    for index, spec in enumerate(KEY_SPECS):
        size_mm = spec["size_mm"]
        centers.append(cursor_mm + size_mm * 0.5)
        cursor_mm += size_mm
        if index < len(KEY_SPECS) - 1:
            cursor_mm += STACK_CLEARANCE_MM
    return centers


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_hex_key_set")

    holder_dark = model.material("holder_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    barrel_black = model.material("barrel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    key_steel = model.material("key_steel", rgba=(0.73, 0.75, 0.78, 1.0))

    total_stack_mm = sum(spec["size_mm"] for spec in KEY_SPECS) + STACK_CLEARANCE_MM * (len(KEY_SPECS) - 1)
    total_holder_thickness_mm = total_stack_mm + 2.0 * PLATE_THICKNESS_MM
    plate_center_y_mm = total_stack_mm * 0.5 + PLATE_THICKNESS_MM * 0.5

    plate_mesh = mesh_from_cadquery(_make_plate_shape(), "side_plate", unit_scale=0.001)
    pivot_barrel_mesh = mesh_from_cadquery(
        _make_y_cylinder(PIVOT_RADIUS_MM, total_holder_thickness_mm + 0.20),
        "pivot_barrel",
        unit_scale=0.001,
    )
    rear_post_mesh = mesh_from_cadquery(
        _make_y_cylinder(REAR_POST_RADIUS_MM, total_holder_thickness_mm + 0.20),
        "rear_post",
        unit_scale=0.001,
    )

    holder = model.part("holder")
    holder.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, -plate_center_y_mm * 0.001, 0.0)),
        material=holder_dark,
        name="plate_0",
    )
    holder.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, plate_center_y_mm * 0.001, 0.0)),
        material=holder_dark,
        name="plate_1",
    )
    holder.visual(pivot_barrel_mesh, material=barrel_black, name="pivot_barrel")
    holder.visual(
        rear_post_mesh,
        origin=Origin(xyz=(REAR_POST_X_MM * 0.001, 0.0, REAR_POST_Z_MM * 0.001)),
        material=barrel_black,
        name="rear_post_0",
    )
    holder.visual(
        rear_post_mesh,
        origin=Origin(xyz=(REAR_POST_X_MM * 0.001, 0.0, -REAR_POST_Z_MM * 0.001)),
        material=barrel_black,
        name="rear_post_1",
    )

    for index, (spec, center_y_mm) in enumerate(zip(KEY_SPECS, _key_slot_centers_mm(), strict=True)):
        boss_mesh = mesh_from_cadquery(_make_key_boss(spec["size_mm"]), f"key_{index}_boss", unit_scale=0.001)
        body_mesh = mesh_from_cadquery(
            _make_key_body(spec["size_mm"], spec["long_mm"], spec["short_mm"]),
            f"key_{index}_body",
            unit_scale=0.001,
        )

        key = model.part(f"key_{index}")
        key.visual(boss_mesh, material=key_steel, name="pivot_boss")
        key.visual(body_mesh, material=key_steel, name="key_body")

        model.articulation(
            f"key_{index}_pivot",
            ArticulationType.REVOLUTE,
            parent=holder,
            child=key,
            origin=Origin(xyz=(0.0, center_y_mm * 0.001, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=4.0,
                lower=0.0,
                upper=spec["upper"],
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    holder = object_model.get_part("holder")
    small_key = object_model.get_part("key_0")
    large_key = object_model.get_part("key_5")
    small_joint = object_model.get_articulation("key_0_pivot")
    large_joint = object_model.get_articulation("key_5_pivot")

    ctx.expect_contact(
        small_key,
        holder,
        elem_a="pivot_boss",
        elem_b="plate_0",
        name="small key seats against plate_0",
    )
    ctx.expect_contact(
        large_key,
        holder,
        elem_a="pivot_boss",
        elem_b="plate_1",
        name="large key seats against plate_1",
    )
    ctx.expect_within(
        small_key,
        holder,
        axes="y",
        margin=0.0,
        name="small key stays within holder thickness",
    )
    ctx.expect_within(
        large_key,
        holder,
        axes="y",
        margin=0.0,
        name="large key stays within holder thickness",
    )
    ctx.expect_overlap(
        large_key,
        holder,
        axes="x",
        elem_a="key_body",
        elem_b="plate_0",
        min_overlap=0.040,
        name="largest key nests along holder length",
    )

    holder_aabb = ctx.part_world_aabb(holder)
    small_rest = ctx.part_element_world_aabb(small_key, elem="key_body")
    large_rest = ctx.part_element_world_aabb(large_key, elem="key_body")

    small_upper = small_joint.motion_limits.upper if small_joint.motion_limits is not None else None
    large_upper = large_joint.motion_limits.upper if large_joint.motion_limits is not None else None

    if holder_aabb is not None and small_rest is not None and large_rest is not None and small_upper is not None:
        with ctx.pose({small_joint: small_upper}):
            small_open = ctx.part_element_world_aabb(small_key, elem="key_body")
            large_closed = ctx.part_element_world_aabb(large_key, elem="key_body")
        ctx.check(
            "small key opens independently",
            small_open is not None
            and large_closed is not None
            and small_open[1][2] > holder_aabb[1][2] + 0.006
            and large_closed[1][2] <= holder_aabb[1][2] + 0.002,
            details=f"holder={holder_aabb}, small_open={small_open}, large_closed={large_closed}",
        )

    if holder_aabb is not None and large_rest is not None and large_upper is not None:
        with ctx.pose({large_joint: large_upper}):
            large_open = ctx.part_element_world_aabb(large_key, elem="key_body")
        ctx.check(
            "large key fans above holder",
            large_open is not None and large_open[1][2] > holder_aabb[1][2] + 0.030,
            details=f"holder={holder_aabb}, large_rest={large_rest}, large_open={large_open}",
        )

    return ctx.report()


object_model = build_object_model()
