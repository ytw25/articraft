from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PRIMARY_WAND_LENGTH = 0.66
SECONDARY_WAND_LENGTH = 0.25
PRIMARY_REST_PITCH = 0.78
SECONDARY_REST_PITCH = 0.57
NOZZLE_REST_PITCH = -1.33


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_wand_vacuum")

    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.20, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.28, 0.29, 0.32, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    clear_bin = model.material("clear_bin", rgba=(0.72, 0.82, 0.90, 0.42))
    red = model.material("red", rgba=(0.82, 0.15, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.072, length=0.18),
        origin=Origin(xyz=(-0.11, 0.0, 0.07), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="motor_shell",
    )
    body.visual(
        Cylinder(radius=0.050, length=0.22),
        origin=Origin(xyz=(0.03, 0.0, 0.14), rpy=(0.0, 0.0, 0.0)),
        material=clear_bin,
        name="dust_bin",
    )
    body.visual(
        Box((0.09, 0.04, 0.23)),
        origin=Origin(xyz=(-0.16, 0.0, 0.19)),
        material=dark_grey,
        name="handle_spine",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.13),
        origin=Origin(xyz=(-0.16, 0.0, 0.305), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="grip_bar",
    )
    body.visual(
        Box((0.14, 0.07, 0.06)),
        origin=Origin(xyz=(-0.20, 0.0, 0.04)),
        material=charcoal,
        name="battery_pack",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.08),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="shoulder_collar",
    )

    primary_wand = model.part("primary_wand")
    primary_wand.visual(
        Cylinder(radius=0.017, length=PRIMARY_WAND_LENGTH),
        origin=Origin(xyz=(PRIMARY_WAND_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="tube_shell",
    )
    primary_wand.visual(
        Cylinder(radius=0.022, length=0.08),
        origin=Origin(xyz=(0.09, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_grey,
        name="upper_cuff",
    )
    primary_wand.visual(
        Cylinder(radius=0.030, length=0.05),
        origin=Origin(xyz=(PRIMARY_WAND_LENGTH, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="distal_hub",
    )

    secondary_wand = model.part("secondary_wand")
    secondary_wand.visual(
        Cylinder(radius=0.016, length=0.22),
        origin=Origin(xyz=(0.14, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="tube_shell",
    )
    secondary_wand.visual(
        Cylinder(radius=0.021, length=0.08),
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_grey,
        name="elbow_cuff",
    )
    secondary_wand.visual(
        Cylinder(radius=0.025, length=0.05),
        origin=Origin(xyz=(SECONDARY_WAND_LENGTH, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="nozzle_socket",
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        Box((0.27, 0.085, 0.028)),
        origin=Origin(xyz=(0.155, 0.0, -0.046)),
        material=charcoal,
        name="head_shell",
    )
    nozzle.visual(
        Box((0.054, 0.04, 0.045)),
        origin=Origin(xyz=(0.048, 0.0, -0.027)),
        material=dark_grey,
        name="neck_shell",
    )
    nozzle.visual(
        Box((0.06, 0.090, 0.020)),
        origin=Origin(xyz=(0.275, 0.0, -0.039)),
        material=red,
        name="front_bumper",
    )

    model.articulation(
        "body_to_primary",
        ArticulationType.REVOLUTE,
        parent=body,
        child=primary_wand,
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, PRIMARY_REST_PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.35, upper=0.55),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_wand,
        child=secondary_wand,
        origin=Origin(xyz=(PRIMARY_WAND_LENGTH, 0.0, 0.0), rpy=(0.0, SECONDARY_REST_PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-0.30, upper=0.75),
    )
    model.articulation(
        "secondary_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=secondary_wand,
        child=nozzle,
        origin=Origin(xyz=(SECONDARY_WAND_LENGTH, 0.0, 0.0), rpy=(0.0, NOZZLE_REST_PITCH, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.35, upper=0.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    primary_wand = object_model.get_part("primary_wand")
    secondary_wand = object_model.get_part("secondary_wand")
    nozzle = object_model.get_part("nozzle")

    shoulder = object_model.get_articulation("body_to_primary")
    elbow = object_model.get_articulation("primary_to_secondary")
    nozzle_hinge = object_model.get_articulation("secondary_to_nozzle")

    ctx.allow_overlap(
        body,
        primary_wand,
        elem_a="shoulder_collar",
        elem_b="tube_shell",
        reason="The body shoulder is represented as a solid collar proxy around the wand root rather than as a hollow socket.",
    )
    ctx.allow_overlap(
        secondary_wand,
        nozzle,
        elem_a="nozzle_socket",
        elem_b="neck_shell",
        reason="The lower hinge is simplified as overlapping socket and yoke solids instead of a fully hollow knuckle-and-pin assembly.",
    )

    ctx.expect_origin_gap(
        nozzle,
        body,
        axis="x",
        min_gap=0.48,
        max_gap=0.58,
        name="floor nozzle projects ahead of the body",
    )
    ctx.expect_origin_gap(
        body,
        nozzle,
        axis="z",
        min_gap=0.66,
        max_gap=0.75,
        name="body rides well above the floor nozzle",
    )
    ctx.expect_overlap(
        primary_wand,
        secondary_wand,
        axes="y",
        elem_a="tube_shell",
        elem_b="tube_shell",
        min_overlap=0.03,
        name="wand links stay laterally aligned",
    )
    ctx.expect_overlap(
        secondary_wand,
        nozzle,
        axes="y",
        elem_a="tube_shell",
        elem_b="neck_shell",
        min_overlap=0.03,
        name="nozzle hinge stays centered on the lower wand",
    )

    rest_nozzle_pos = ctx.part_world_position(nozzle)
    with ctx.pose({shoulder: 0.45}):
        raised_by_shoulder = ctx.part_world_position(nozzle)
    ctx.check(
        "shoulder joint lifts the wand chain",
        rest_nozzle_pos is not None
        and raised_by_shoulder is not None
        and raised_by_shoulder[2] > rest_nozzle_pos[2] + 0.18,
        details=f"rest={rest_nozzle_pos}, shoulder_raised={raised_by_shoulder}",
    )

    with ctx.pose({elbow: 0.70}):
        raised_by_elbow = ctx.part_world_position(nozzle)
    ctx.check(
        "secondary elbow folds upward",
        rest_nozzle_pos is not None
        and raised_by_elbow is not None
        and raised_by_elbow[2] > rest_nozzle_pos[2] + 0.06,
        details=f"rest={rest_nozzle_pos}, elbow_raised={raised_by_elbow}",
    )

    rest_head = ctx.part_element_world_aabb(nozzle, elem="head_shell")
    with ctx.pose({nozzle_hinge: 0.40}):
        pitched_head = ctx.part_element_world_aabb(nozzle, elem="head_shell")
    ctx.check(
        "nozzle hinge pitches the front edge upward",
        rest_head is not None
        and pitched_head is not None
        and pitched_head[1][2] > rest_head[1][2] + 0.04,
        details=f"rest={rest_head}, pitched={pitched_head}",
    )

    return ctx.report()


object_model = build_object_model()
