from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

CASE_W = 0.232
CASE_D = 0.470
CASE_H = 0.505
BODY_Z0 = 0.018
CASE_TOP_Z = BODY_Z0 + CASE_H

FEET_H = 0.018
PANEL_T = 0.007
PANEL_D = CASE_D - 0.020
PANEL_H = CASE_H - 0.016
DOOR_T = 0.014
DOOR_W = CASE_W - 0.008
DOOR_H = CASE_H - 0.024
FRONT_PANEL_T = 0.008
FRONT_PANEL_W = CASE_W - 0.022
FRONT_PANEL_H = CASE_H - 0.064
TOP_VENT_T = 0.004
TOP_VENT_W = 0.148
TOP_VENT_D = 0.234


def _front_intake_panel() -> cq.Workplane:
    panel = cq.Workplane("XY").box(
        FRONT_PANEL_W,
        FRONT_PANEL_T,
        FRONT_PANEL_H,
        centered=(True, True, False),
    )
    slot_width = FRONT_PANEL_W - 0.030
    slot_height = 0.004
    slot_margin = 0.040
    slot_count = 20
    slot_pitch = (FRONT_PANEL_H - 2.0 * slot_margin) / (slot_count - 1)
    for index in range(slot_count):
        z_pos = slot_margin + index * slot_pitch
        cutter = (
            cq.Workplane("XY")
            .box(slot_width, FRONT_PANEL_T * 3.0, slot_height, centered=(True, True, False))
            .translate((0.0, 0.0, z_pos))
            .val()
        )
        panel = panel.cut(cutter)
    return panel


def _top_vent_panel() -> cq.Workplane:
    vent = cq.Workplane("XY").box(
        TOP_VENT_W,
        TOP_VENT_D,
        TOP_VENT_T,
        centered=(True, True, False),
    )
    slot_depth = TOP_VENT_D - 0.028
    slot_width = 0.010
    slot_count = 8
    slot_pitch = (TOP_VENT_W - 0.038) / (slot_count - 1)
    for index in range(slot_count):
        x_pos = -0.5 * (TOP_VENT_W - 0.038) + index * slot_pitch
        cutter = (
            cq.Workplane("XY")
            .box(slot_width, slot_depth, TOP_VENT_T * 3.0, centered=(True, True, False))
            .translate((x_pos, 0.0, 0.0))
            .val()
        )
        vent = vent.cut(cutter)
    return vent


def _side_panel_frame() -> cq.Workplane:
    frame = (
        cq.Workplane("XY")
        .box(PANEL_T, PANEL_D, PANEL_H, centered=(True, True, False))
        .translate((0.0, -0.5 * PANEL_D, 0.0))
    )
    window = (
        cq.Workplane("XY")
        .box(PANEL_T * 3.0, PANEL_D - 0.036, PANEL_H - 0.050, centered=(True, True, False))
        .translate((0.0, -0.5 * PANEL_D, 0.025))
        .val()
    )
    return frame.cut(window)


def _front_door() -> cq.Workplane:
    door = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H, centered=(False, True, False))
        .translate((-DOOR_W, 0.0, 0.0))
    )
    slot_width = 0.007
    slot_height = DOOR_H - 0.084
    slot_count = 12
    slot_span = DOOR_W - 0.050
    slot_pitch = slot_span / (slot_count - 1)
    for index in range(slot_count):
        x_pos = -DOOR_W + 0.025 + index * slot_pitch
        cutter = (
            cq.Workplane("XY")
            .box(slot_width, DOOR_T * 3.0, slot_height, centered=(True, True, False))
            .translate((x_pos, 0.0, 0.042))
            .val()
        )
        door = door.cut(cutter)
    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_pc_tower", assets=ASSETS)

    model.material("body_black", rgba=(0.11, 0.12, 0.13, 1.0))
    model.material("powder_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("mesh_black", rgba=(0.07, 0.08, 0.09, 1.0))
    model.material("satin_aluminum", rgba=(0.36, 0.37, 0.39, 1.0))
    model.material("fan_gray", rgba=(0.23, 0.24, 0.25, 1.0))
    model.material("smoked_glass", rgba=(0.38, 0.44, 0.48, 0.28))
    model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    model.material("io_dark", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("status_led", rgba=(0.14, 0.39, 0.76, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((CASE_W, CASE_D, CASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0 + 0.5 * CASE_H)),
        material="body_black",
    )
    chassis.visual(
        mesh_from_cadquery(_front_intake_panel(), "front_intake_panel.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, -0.5 * CASE_D + 0.5 * FRONT_PANEL_T, BODY_Z0 + 0.032)),
        material="mesh_black",
    )
    chassis.visual(
        mesh_from_cadquery(_top_vent_panel(), "top_vent_panel.obj", assets=ASSETS),
        origin=Origin(xyz=(0.010, -0.012, CASE_TOP_Z - 0.001)),
        material="mesh_black",
    )
    for fan_z in (BODY_Z0 + 0.174, BODY_Z0 + 0.318):
        chassis.visual(
            Cylinder(radius=0.064, length=0.014),
            origin=Origin(
                xyz=(0.0, -0.5 * CASE_D + 0.030, fan_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="fan_gray",
        )
        chassis.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(
                xyz=(0.0, -0.5 * CASE_D + 0.030, fan_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="io_dark",
        )
    chassis.visual(
        Box((0.184, 0.006, 0.344)),
        origin=Origin(xyz=(0.0, 0.5 * CASE_D + 0.003, BODY_Z0 + 0.264)),
        material="powder_steel",
    )
    chassis.visual(
        Box((0.152, 0.006, 0.096)),
        origin=Origin(xyz=(0.0, 0.5 * CASE_D + 0.003, BODY_Z0 + 0.082)),
        material="io_dark",
    )
    for index in range(7):
        chassis.visual(
            Box((0.110, 0.007, 0.008)),
            origin=Origin(
                xyz=(0.032, 0.5 * CASE_D + 0.0035, BODY_Z0 + 0.157 + 0.012 * index),
            ),
            material="satin_aluminum",
        )
    for x_pos, z_pos in (
        (-0.070, BODY_Z0 + 0.374),
        (-0.062, BODY_Z0 + 0.336),
        (-0.080, BODY_Z0 + 0.300),
        (-0.048, BODY_Z0 + 0.300),
    ):
        chassis.visual(
            Box((0.018, 0.008, 0.014)),
            origin=Origin(xyz=(x_pos, 0.5 * CASE_D + 0.004, z_pos)),
            material="io_dark",
        )
    chassis.visual(
        Cylinder(radius=0.060, length=0.010),
        origin=Origin(
            xyz=(0.056, 0.5 * CASE_D + 0.002, BODY_Z0 + 0.404),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="mesh_black",
    )
    chassis.visual(
        Cylinder(radius=0.015, length=0.014),
        origin=Origin(
            xyz=(0.056, 0.5 * CASE_D + 0.002, BODY_Z0 + 0.404),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="io_dark",
    )
    chassis.visual(
        Box((0.080, 0.050, 0.010)),
        origin=Origin(xyz=(0.024, -0.124, CASE_TOP_Z + 0.004)),
        material="powder_steel",
    )
    chassis.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.057, -0.120, CASE_TOP_Z + 0.008)),
        material="satin_aluminum",
    )
    chassis.visual(
        Box((0.020, 0.010, 0.004)),
        origin=Origin(xyz=(0.016, -0.125, CASE_TOP_Z + 0.007)),
        material="io_dark",
    )
    chassis.visual(
        Box((0.006, 0.002, 0.002)),
        origin=Origin(xyz=(0.057, -0.120, CASE_TOP_Z + 0.0115)),
        material="status_led",
    )
    chassis.visual(
        Box((0.010, 0.008, 0.180)),
        origin=Origin(xyz=(0.5 * CASE_W - 0.010, -0.5 * CASE_D + 0.004, BODY_Z0 + 0.275)),
        material="satin_aluminum",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            chassis.visual(
                Cylinder(radius=0.018, length=FEET_H),
                origin=Origin(
                    xyz=(
                        x_sign * (0.5 * CASE_W - 0.038),
                        y_sign * (0.5 * CASE_D - 0.055),
                        0.5 * FEET_H,
                    )
                ),
                material="rubber",
            )
    chassis.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, CASE_H)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0 + 0.5 * CASE_H)),
    )

    side_panel = model.part("side_panel")
    side_panel.visual(
        mesh_from_cadquery(_side_panel_frame(), "side_panel_frame.obj", assets=ASSETS),
        origin=Origin(),
        material="powder_steel",
    )
    side_panel.visual(
        Box((0.003, PANEL_D - 0.030, PANEL_H - 0.044)),
        origin=Origin(xyz=(0.0, -0.5 * PANEL_D, 0.5 * PANEL_H)),
        material="smoked_glass",
    )
    for latch_z in (0.118, 0.374):
        side_panel.visual(
            Cylinder(radius=0.005, length=0.010),
            origin=Origin(
                xyz=(0.0, -PANEL_D + 0.020, latch_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="satin_aluminum",
        )
    side_panel.inertial = Inertial.from_geometry(
        Box((PANEL_T, PANEL_D, PANEL_H)),
        mass=2.8,
        origin=Origin(xyz=(0.0, -0.5 * PANEL_D, 0.5 * PANEL_H)),
    )

    front_door = model.part("front_door")
    front_door.visual(
        mesh_from_cadquery(_front_door(), "front_door.obj", assets=ASSETS),
        origin=Origin(),
        material="powder_steel",
    )
    front_door.visual(
        Box((0.010, 0.018, 0.208)),
        origin=Origin(xyz=(-DOOR_W + 0.014, -0.012, 0.245)),
        material="satin_aluminum",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        mass=1.3,
        origin=Origin(xyz=(-0.5 * DOOR_W, 0.0, 0.5 * DOOR_H)),
    )

    model.articulation(
        "chassis_to_side_panel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(
            xyz=(-0.5 * CASE_W - 0.5 * PANEL_T - 0.001, 0.5 * CASE_D - 0.010, BODY_Z0 + 0.008),
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.22, effort=6.0, velocity=1.4),
    )
    model.articulation(
        "chassis_to_front_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(
            xyz=(0.5 * DOOR_W, -0.5 * CASE_D - 0.5 * DOOR_T, BODY_Z0 + 0.012),
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.65, effort=4.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("side_panel", "chassis", axes="yz", min_overlap=0.40)
    ctx.expect_aabb_gap("chassis", "side_panel", axis="x", max_gap=0.002, max_penetration=0.002)
    ctx.expect_aabb_overlap("front_door", "chassis", axes="xz", min_overlap=0.20)
    ctx.expect_aabb_gap("chassis", "front_door", axis="y", max_gap=0.0001, max_penetration=0.001)
    ctx.expect_aabb_contact("side_panel", "chassis")
    ctx.expect_joint_motion_axis(
        "chassis_to_side_panel",
        "side_panel",
        world_axis="x",
        direction="negative",
        min_delta=0.08,
    )
    ctx.expect_joint_motion_axis(
        "chassis_to_front_door",
        "front_door",
        world_axis="y",
        direction="positive",
        min_delta=0.08,
    )

    with ctx.pose(chassis_to_side_panel=1.10):
        ctx.expect_aabb_overlap("side_panel", "chassis", axes="z", min_overlap=0.45)

    with ctx.pose(chassis_to_front_door=1.45):
        ctx.expect_aabb_overlap("front_door", "chassis", axes="z", min_overlap=0.45)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
