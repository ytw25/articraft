from __future__ import annotations

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


HOLDER_LENGTH = 0.168
HOLDER_WIDTH = 0.048
HOLDER_THICKNESS = 0.022
STACK_OFFSET_Y = 0.011
PIVOT_OFFSET_X = 0.069
KEY_THICKNESS = 0.0024
PIN_RADIUS = 0.0022
PIN_CLEARANCE_RADIUS = 0.0027
WASHER_RADIUS = 0.0078
WASHER_THICKNESS = 0.0008
HEAD_RADIUS = 0.0046
HEAD_THICKNESS = 0.0014
STACK_Z = (-0.0042, -0.0014, 0.0014, 0.0042)
OPEN_LIMIT = 2.55


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cq_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate(
        (center[0], center[1], center[2] - length / 2.0)
    )


def _build_holder_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(HOLDER_LENGTH, HOLDER_WIDTH, HOLDER_THICKNESS)
    shell = shell.edges("|Z").fillet(0.010)

    pocket_depth = 0.018
    pocket_z = HOLDER_THICKNESS * 0.5 - pocket_depth * 0.5
    pocket_radius = 0.0098
    channel_length = 0.078
    channel_width = 0.016
    inner_cap_radius = channel_width * 0.5

    for pin_x, stack_y, inward_sign in (
        (-PIVOT_OFFSET_X, STACK_OFFSET_Y, 1.0),
        (PIVOT_OFFSET_X, -STACK_OFFSET_Y, -1.0),
    ):
        pivot_pocket = _cq_cylinder(pocket_radius, pocket_depth, (pin_x, stack_y, pocket_z))
        trough = _cq_box(
            (channel_length, channel_width, pocket_depth),
            (pin_x + inward_sign * channel_length * 0.5, stack_y, pocket_z),
        )
        inner_cap = _cq_cylinder(
            inner_cap_radius,
            pocket_depth,
            (pin_x + inward_sign * channel_length, stack_y, pocket_z),
        )
        shell = shell.cut(pivot_pocket.union(trough).union(inner_cap))

    side_relief_radius = 0.034
    side_relief_length = HOLDER_LENGTH * 0.62
    side_relief_z = -0.001
    left_relief = (
        cq.Workplane("YZ")
        .circle(side_relief_radius)
        .extrude(side_relief_length)
        .translate((-side_relief_length * 0.5, HOLDER_WIDTH * 0.5 + 0.024, side_relief_z))
    )
    right_relief = (
        cq.Workplane("YZ")
        .circle(side_relief_radius)
        .extrude(side_relief_length)
        .translate((-side_relief_length * 0.5, -HOLDER_WIDTH * 0.5 - 0.024, side_relief_z))
    )
    shell = shell.cut(left_relief).cut(right_relief)
    return shell


def _build_key_mesh(
    *,
    name: str,
    long_arm: float,
    short_arm: float,
    shaft_width: float,
) -> object:
    tab_radius = 0.0066

    pivot_tab = _cq_cylinder(tab_radius, KEY_THICKNESS, (0.0, 0.0, 0.0))
    short_leg = _cq_box(
        (shaft_width, short_arm + shaft_width * 0.75, KEY_THICKNESS),
        (0.0, -(short_arm + shaft_width * 0.75) * 0.5, 0.0),
    )
    elbow = _cq_cylinder(shaft_width * 0.55, KEY_THICKNESS, (0.0, -short_arm, 0.0))
    long_leg = _cq_box(
        (long_arm, shaft_width, KEY_THICKNESS),
        (long_arm * 0.5, -short_arm, 0.0),
    )
    tip = _cq_cylinder(shaft_width * 0.5, KEY_THICKNESS, (long_arm, -short_arm, 0.0))
    key = pivot_tab.union(short_leg).union(elbow).union(long_leg).union(tip)
    pivot_hole = _cq_cylinder(PIN_CLEARANCE_RADIUS, KEY_THICKNESS + 0.004, (0.0, 0.0, 0.0))
    key = key.cut(pivot_hole)
    return mesh_from_cadquery(key, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_hex_key_set")

    holder_body = model.material("holder_body", rgba=(0.16, 0.17, 0.18, 1.0))
    hardware = model.material("hardware", rgba=(0.73, 0.74, 0.76, 1.0))
    key_steel = model.material("key_steel", rgba=(0.63, 0.65, 0.68, 1.0))

    holder = model.part("holder")
    holder.visual(
        mesh_from_cadquery(_build_holder_shell(), "holder_shell"),
        material=holder_body,
        name="holder_shell",
    )

    for pivot_index, (pin_x, stack_y) in enumerate(
        ((-PIVOT_OFFSET_X, STACK_OFFSET_Y), (PIVOT_OFFSET_X, -STACK_OFFSET_Y))
    ):
        holder.visual(
            Cylinder(radius=PIN_RADIUS, length=HOLDER_THICKNESS + 2.0 * (WASHER_THICKNESS + HEAD_THICKNESS)),
            origin=Origin(xyz=(pin_x, stack_y, 0.0)),
            material=hardware,
            name=f"pin_{pivot_index}",
        )
        for side_name, z_sign in (("top", 1.0), ("bottom", -1.0)):
            holder.visual(
                Cylinder(radius=WASHER_RADIUS, length=WASHER_THICKNESS),
                origin=Origin(
                    xyz=(
                        pin_x,
                        stack_y,
                        z_sign * (HOLDER_THICKNESS * 0.5 + WASHER_THICKNESS * 0.5),
                    )
                ),
                material=hardware,
                name=f"washer_{pivot_index}_{side_name}",
            )
            holder.visual(
                Cylinder(radius=HEAD_RADIUS, length=HEAD_THICKNESS),
                origin=Origin(
                    xyz=(
                        pin_x,
                        stack_y,
                        z_sign
                        * (
                            HOLDER_THICKNESS * 0.5
                            + WASHER_THICKNESS
                            + HEAD_THICKNESS * 0.5
                        ),
                    )
                ),
                material=hardware,
                name=f"head_{pivot_index}_{side_name}",
            )

    key_specs = (
        {"index": 0, "stack": 0, "layer": 0, "long_arm": 0.064, "short_arm": 0.018, "shaft_width": 0.0054},
        {"index": 1, "stack": 0, "layer": 1, "long_arm": 0.058, "short_arm": 0.016, "shaft_width": 0.0048},
        {"index": 2, "stack": 0, "layer": 2, "long_arm": 0.052, "short_arm": 0.014, "shaft_width": 0.0041},
        {"index": 3, "stack": 0, "layer": 3, "long_arm": 0.046, "short_arm": 0.012, "shaft_width": 0.0035},
        {"index": 4, "stack": 1, "layer": 0, "long_arm": 0.062, "short_arm": 0.018, "shaft_width": 0.0052},
        {"index": 5, "stack": 1, "layer": 1, "long_arm": 0.056, "short_arm": 0.016, "shaft_width": 0.0046},
        {"index": 6, "stack": 1, "layer": 2, "long_arm": 0.050, "short_arm": 0.014, "shaft_width": 0.0040},
        {"index": 7, "stack": 1, "layer": 3, "long_arm": 0.044, "short_arm": 0.012, "shaft_width": 0.0034},
    )

    for spec in key_specs:
        key = model.part(f"key_{spec['index']}")
        key.visual(
            _build_key_mesh(
                name=f"key_mesh_{spec['index']}",
                long_arm=spec["long_arm"],
                short_arm=spec["short_arm"],
                shaft_width=spec["shaft_width"],
            ),
            origin=Origin(rpy=(0.0, 0.0, 0.0 if spec["stack"] == 0 else 3.141592653589793)),
            material=key_steel,
            name="key_body",
        )

        pin_x = -PIVOT_OFFSET_X if spec["stack"] == 0 else PIVOT_OFFSET_X
        stack_y = STACK_OFFSET_Y if spec["stack"] == 0 else -STACK_OFFSET_Y
        model.articulation(
            f"holder_to_key_{spec['index']}",
            ArticulationType.REVOLUTE,
            parent=holder,
            child=key,
            origin=Origin(xyz=(pin_x, stack_y, STACK_Z[spec["layer"]])),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=4.0,
                lower=0.0,
                upper=OPEN_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    holder = object_model.get_part("holder")
    key_0 = object_model.get_part("key_0")
    key_1 = object_model.get_part("key_1")
    key_4 = object_model.get_part("key_4")
    joint_0 = object_model.get_articulation("holder_to_key_0")
    joint_1 = object_model.get_articulation("holder_to_key_1")
    joint_4 = object_model.get_articulation("holder_to_key_4")

    ctx.expect_within(
        key_0,
        holder,
        axes="xy",
        elem_a="key_body",
        elem_b="holder_shell",
        margin=0.002,
        name="stack_0 outer key folds inside holder footprint",
    )
    ctx.expect_within(
        key_4,
        holder,
        axes="xy",
        elem_a="key_body",
        elem_b="holder_shell",
        margin=0.002,
        name="stack_1 outer key folds inside holder footprint",
    )
    ctx.expect_overlap(
        key_0,
        holder,
        axes="x",
        elem_a="key_body",
        elem_b="holder_shell",
        min_overlap=0.050,
        name="stack_0 folded key nests along holder length",
    )
    ctx.expect_overlap(
        key_4,
        holder,
        axes="x",
        elem_a="key_body",
        elem_b="holder_shell",
        min_overlap=0.048,
        name="stack_1 folded key nests along holder length",
    )

    closed_left = ctx.part_element_world_aabb(key_0, elem="key_body")
    closed_right = ctx.part_element_world_aabb(key_4, elem="key_body")

    with ctx.pose({joint_0: 1.35, joint_4: 1.35}):
        open_left = ctx.part_element_world_aabb(key_0, elem="key_body")
        open_right = ctx.part_element_world_aabb(key_4, elem="key_body")
        ctx.check(
            "stack_0 key opens outward",
            closed_left is not None
            and open_left is not None
            and open_left[1][1] > closed_left[1][1] + 0.030,
            details=f"closed_left={closed_left}, open_left={open_left}",
        )
        ctx.check(
            "stack_1 key opens outward",
            closed_right is not None
            and open_right is not None
            and open_right[0][1] < closed_right[0][1] - 0.030,
            details=f"closed_right={closed_right}, open_right={open_right}",
        )

    with ctx.pose({joint_0: 1.45, joint_1: 0.35}):
        key_0_aabb = ctx.part_element_world_aabb(key_0, elem="key_body")
        key_1_aabb = ctx.part_element_world_aabb(key_1, elem="key_body")
        ctx.check(
            "stack_0 keys can hold distinct angles",
            key_0_aabb is not None
            and key_1_aabb is not None
            and key_1_aabb[1][0] > key_0_aabb[1][0] + 0.020,
            details=f"key_0_aabb={key_0_aabb}, key_1_aabb={key_1_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
