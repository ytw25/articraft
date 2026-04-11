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


HANDLE_LENGTH = 0.092
HANDLE_WIDTH = 0.0245
HANDLE_HEIGHT = 0.022
HANDLE_CENTER_X = 0.039
HANDLE_CENTER_Z = 0.0045
NOSE_RADIUS = 0.0105
AXLE_RADIUS = 0.0008
AXLE_CLEARANCE = 0.0
CAP_RADIUS = 0.0034
CAP_LENGTH = 0.0014
CAVITY_LENGTH = 0.080
INNER_WIDTH = 0.0211
CAVITY_HEIGHT = 0.023
CAVITY_CENTER_X = 0.039
CAVITY_CENTER_Z = 0.0085
KEY_GAP = 0.00015
PIVOT_RELIEF_RADIUS = 0.0088

KEY_SPECS = (
    {"size": 0.0015, "long": 0.042, "short": 0.0075, "upper": 2.00},
    {"size": 0.0020, "long": 0.049, "short": 0.0085, "upper": 2.02},
    {"size": 0.0025, "long": 0.055, "short": 0.0095, "upper": 2.05},
    {"size": 0.0030, "long": 0.062, "short": 0.0105, "upper": 2.08},
    {"size": 0.0040, "long": 0.069, "short": 0.0112, "upper": 2.12},
    {"size": 0.0050, "long": 0.076, "short": 0.0118, "upper": 2.16},
)


def hex_points(across_flats: float) -> list[tuple[float, float]]:
    radius = across_flats / math.sqrt(3.0)
    return [
        (
            radius * math.cos(math.radians(30.0 + 60.0 * i)),
            radius * math.sin(math.radians(30.0 + 60.0 * i)),
        )
        for i in range(6)
    ]


def key_stack_offsets() -> list[float]:
    total = sum(spec["size"] for spec in KEY_SPECS) + KEY_GAP * (len(KEY_SPECS) - 1)
    cursor = -0.5 * total
    offsets: list[float] = []
    for spec in KEY_SPECS:
        offsets.append(cursor + 0.5 * spec["size"])
        cursor += spec["size"] + KEY_GAP
    return offsets


def make_key_shape(size: float, long_len: float, short_len: float) -> cq.Workplane:
    profile = hex_points(size)
    collar_radius = max(size * 1.55, AXLE_RADIUS + 0.0018)
    heel_radius = max(size * 0.72, collar_radius * 0.62)

    short_arm = cq.Workplane("XY").polyline(profile).close().extrude(short_len)
    long_arm = cq.Workplane("YZ").polyline(profile).close().extrude(long_len).translate((0.0, 0.0, short_len))
    collar = cq.Workplane("XZ").circle(collar_radius).extrude(0.45 * size, both=True)
    heel = cq.Workplane("XZ").circle(heel_radius).extrude(0.42 * size, both=True).translate((0.0, 0.0, short_len))
    hole = cq.Workplane("XZ").circle(AXLE_RADIUS + AXLE_CLEARANCE).extrude(size, both=True)

    return short_arm.union(long_arm).union(collar).union(heel).cut(hole)


def make_holder_shell_shape() -> cq.Workplane:
    plate_thickness = 0.5 * (HANDLE_WIDTH - INNER_WIDTH)
    plate_center_y = 0.5 * INNER_WIDTH + 0.5 * plate_thickness

    plate = cq.Workplane("XY").box(HANDLE_LENGTH, plate_thickness, HANDLE_HEIGHT)
    left_plate = plate.translate((HANDLE_CENTER_X, plate_center_y, HANDLE_CENTER_Z))
    right_plate = plate.translate((HANDLE_CENTER_X, -plate_center_y, HANDLE_CENTER_Z))

    rear_block = (
        cq.Workplane("XY")
        .box(0.008, INNER_WIDTH, 0.016)
        .translate((0.082, 0.0, 0.0045))
    )
    spine = (
        cq.Workplane("XY")
        .box(0.066, INNER_WIDTH, 0.0032)
        .translate((0.047, 0.0, -0.0042))
    )

    nose_ring = cq.Workplane("XZ").circle(NOSE_RADIUS).extrude(0.5 * INNER_WIDTH, both=True)
    nose_relief = cq.Workplane("XZ").circle(PIVOT_RELIEF_RADIUS).extrude(0.5 * INNER_WIDTH, both=True)
    nose_slot = (
        cq.Workplane("XY")
        .box(0.018, INNER_WIDTH, 0.017)
        .translate((0.003, 0.0, 0.009))
    )

    return left_plate.union(right_plate).union(rear_block).union(spine).union(
        nose_ring.cut(nose_relief).cut(nose_slot)
    )


def make_pivot_hardware_shape() -> cq.Workplane:
    axle = cq.Workplane("XZ").circle(AXLE_RADIUS).extrude(0.5 * HANDLE_WIDTH, both=True)
    cap = cq.Workplane("XZ").circle(CAP_RADIUS).extrude(0.5 * CAP_LENGTH, both=True)
    cap_pos = 0.5 * HANDLE_WIDTH + 0.5 * CAP_LENGTH
    cap_a = cap.translate((0.0, cap_pos, 0.0))
    cap_b = cap.translate((0.0, -cap_pos, 0.0))
    return axle.union(cap_a).union(cap_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_allen_key_holder")

    shell_mat = model.material("shell_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel_mat = model.material("steel", rgba=(0.69, 0.72, 0.75, 1.0))
    hardware_mat = model.material("hardware", rgba=(0.26, 0.27, 0.29, 1.0))

    holder = model.part("holder")
    holder.visual(
        mesh_from_cadquery(make_holder_shell_shape(), "holder_shell"),
        material=shell_mat,
        name="shell",
    )
    holder.visual(
        mesh_from_cadquery(make_pivot_hardware_shape(), "pivot_hardware"),
        material=hardware_mat,
        name="pivot_hardware",
    )

    for index, (spec, y_offset) in enumerate(zip(KEY_SPECS, key_stack_offsets(), strict=True)):
        key = model.part(f"key_{index}")
        key.visual(
            mesh_from_cadquery(
                make_key_shape(spec["size"], spec["long"], spec["short"]),
                f"key_{index}_body",
            ),
            material=steel_mat,
            name="body",
        )

        model.articulation(
            f"holder_to_key_{index}",
            ArticulationType.REVOLUTE,
            parent=holder,
            child=key,
            origin=Origin(xyz=(0.0, y_offset, 0.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=4.0,
                lower=0.0,
                upper=spec["upper"],
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    holder = object_model.get_part("holder")
    largest_key = object_model.get_part("key_5")
    mid_key = object_model.get_part("key_2")
    largest_joint = object_model.get_articulation("holder_to_key_5")

    for index in range(len(KEY_SPECS)):
        ctx.allow_overlap(
            holder,
            object_model.get_part(f"key_{index}"),
            elem_a="pivot_hardware",
            elem_b="body",
            reason="Each folded key is intentionally represented as a tight captured fit around the common pivot barrel.",
        )

    ctx.check(
        "six folding keys are authored",
        len([part for part in object_model.parts if part.name.startswith("key_")]) == 6,
        details=f"parts={[part.name for part in object_model.parts]}",
    )

    ctx.expect_overlap(
        largest_key,
        holder,
        axes="x",
        min_overlap=0.060,
        elem_a="body",
        elem_b="shell",
        name="largest key stays tucked inside the holder footprint",
    )
    ctx.expect_overlap(
        mid_key,
        holder,
        axes="x",
        min_overlap=0.045,
        elem_a="body",
        elem_b="shell",
        name="mid key also nests inside the holder footprint",
    )

    closed_aabb = ctx.part_element_world_aabb(largest_key, elem="body")
    with ctx.pose({largest_joint: largest_joint.motion_limits.upper}):
        opened_aabb = ctx.part_element_world_aabb(largest_key, elem="body")
        ctx.expect_overlap(
            largest_key,
            holder,
            axes="yz",
            min_overlap=0.001,
            elem_a="body",
            elem_b="pivot_hardware",
            name="opened key still shares the common axle line",
        )

    ctx.check(
        "largest key swings upward from the shell",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.035
        and opened_aabb[1][0] < closed_aabb[1][0] - 0.020,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
