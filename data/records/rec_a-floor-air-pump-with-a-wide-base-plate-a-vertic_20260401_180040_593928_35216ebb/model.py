from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


BASE_LEN = 0.28
BASE_DEPTH = 0.12
BASE_THICK = 0.012

CYL_OUTER_R = 0.034
CYL_WALL = 0.0035
CYL_INNER_R = CYL_OUTER_R - CYL_WALL
CYL_HEIGHT = 0.44
LOWER_COLLAR_R = 0.042
LOWER_COLLAR_H = 0.06

GUIDE_OUTER_R = 0.028
GUIDE_HOLE_R = 0.0072
GUIDE_H = 0.015
GUIDE_BASE_Z = BASE_THICK + CYL_HEIGHT
SLIDE_JOINT_Z = GUIDE_BASE_Z + GUIDE_H

ROD_R = 0.0062
ROD_DOWN = 0.255
ROD_UP = 0.155
ROD_TOTAL = ROD_DOWN + ROD_UP
STOP_COLLAR_R = 0.012
STOP_COLLAR_H = 0.008
PISTON_R = 0.027
PISTON_H = 0.012
HANDLE_BOSS_R = 0.010
HANDLE_BOSS_H = 0.022
HANDLE_Z = 0.165
HANDLE_BAR_R = 0.012
HANDLE_BAR_LEN = 0.22
GRIP_R = 0.017
GRIP_LEN = 0.075
SLIDE_TRAVEL = 0.18

GAUGE_Z = BASE_THICK + 0.17
GAUGE_BOSS_R = 0.023
GAUGE_BOSS_LEN = 0.014
GAUGE_BOSS_START_Y = CYL_OUTER_R - 0.002
GAUGE_JOINT_Y = GAUGE_BOSS_START_Y + GAUGE_BOSS_LEN

KNOB_HUB_R = 0.016
KNOB_HUB_LEN = 0.010
KNOB_R = 0.024
KNOB_LEN = 0.020
POINTER_W = 0.008
POINTER_H = 0.016
POINTER_T = 0.004


def _base_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_LEN, BASE_DEPTH, BASE_THICK, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
    )


def _cylinder_stack_shape() -> cq.Workplane:
    lower_collar = (
        cq.Workplane("XY")
        .circle(LOWER_COLLAR_R)
        .circle(CYL_INNER_R)
        .extrude(LOWER_COLLAR_H)
        .translate((0.0, 0.0, BASE_THICK))
    )
    tube = (
        cq.Workplane("XY")
        .circle(CYL_OUTER_R)
        .circle(CYL_INNER_R)
        .extrude(CYL_HEIGHT - LOWER_COLLAR_H)
        .translate((0.0, 0.0, BASE_THICK + LOWER_COLLAR_H))
    )
    return lower_collar.union(tube)


def _guide_ring_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(CYL_OUTER_R)
        .circle(GUIDE_HOLE_R)
        .extrude(GUIDE_H)
        .translate((0.0, 0.0, GUIDE_BASE_Z))
    )


def _gauge_boss_shape() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(GAUGE_BOSS_R)
        .extrude(GAUGE_BOSS_LEN)
        .translate((0.0, GAUGE_BOSS_START_Y + GAUGE_BOSS_LEN, GAUGE_Z))
    )


def _rod_core_shape() -> cq.Workplane:
    rod = (
        cq.Workplane("XY")
        .circle(ROD_R)
        .extrude(ROD_TOTAL)
        .translate((0.0, 0.0, -ROD_DOWN))
    )
    piston = (
        cq.Workplane("XY")
        .circle(PISTON_R)
        .extrude(PISTON_H)
        .translate((0.0, 0.0, -ROD_DOWN))
    )
    handle_boss = (
        cq.Workplane("XY")
        .circle(HANDLE_BOSS_R)
        .extrude(HANDLE_BOSS_H)
        .translate((0.0, 0.0, HANDLE_Z - HANDLE_BOSS_H / 2.0))
    )
    return rod.union(piston).union(handle_boss)


def _stop_collar_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(STOP_COLLAR_R).extrude(STOP_COLLAR_H)


def _handle_bar_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(HANDLE_BAR_R)
        .extrude(HANDLE_BAR_LEN)
        .translate((-HANDLE_BAR_LEN / 2.0, 0.0, HANDLE_Z))
    )


def _grip_shape(x_start: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(GRIP_R)
        .extrude(GRIP_LEN)
        .translate((x_start, 0.0, HANDLE_Z))
    )


def _gauge_knob_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XZ")
        .circle(KNOB_HUB_R)
        .extrude(KNOB_HUB_LEN)
        .translate((0.0, KNOB_HUB_LEN, 0.0))
    )
    knob = (
        cq.Workplane("XZ")
        .circle(KNOB_R)
        .extrude(KNOB_LEN)
        .translate((0.0, KNOB_LEN, 0.0))
    )
    pointer = (
        cq.Workplane("XZ")
        .rect(POINTER_W, POINTER_H)
        .extrude(POINTER_T)
        .translate((KNOB_R * 0.62, KNOB_LEN + 0.002, 0.0))
    )
    return knob.union(hub).union(pointer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_air_pump")

    body_mat = model.material("body_gray", rgba=(0.22, 0.22, 0.24, 1.0))
    steel_mat = model.material("steel", rgba=(0.76, 0.77, 0.80, 1.0))
    rubber_mat = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    knob_mat = model.material("knob_orange", rgba=(0.88, 0.47, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_base_plate_shape(), "pump_base_plate"),
        material=body_mat,
        name="base_plate",
    )
    body.visual(
        mesh_from_cadquery(_cylinder_stack_shape(), "pump_cylinder_stack"),
        material=body_mat,
        name="cylinder_stack",
    )
    body.visual(
        mesh_from_cadquery(_guide_ring_shape(), "pump_guide_ring"),
        material=steel_mat,
        name="guide_ring",
    )
    body.visual(
        mesh_from_cadquery(_gauge_boss_shape(), "pump_gauge_boss"),
        material=steel_mat,
        name="gauge_boss",
    )

    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(_rod_core_shape(), "pump_rod_core"),
        material=steel_mat,
        name="rod_core",
    )
    slider.visual(
        mesh_from_cadquery(_stop_collar_shape(), "pump_stop_collar"),
        material=steel_mat,
        name="stop_collar",
    )
    slider.visual(
        mesh_from_cadquery(_handle_bar_shape(), "pump_handle_bar"),
        material=steel_mat,
        name="handle_bar",
    )
    slider.visual(
        mesh_from_cadquery(_grip_shape(-HANDLE_BAR_LEN / 2.0), "pump_left_grip"),
        material=rubber_mat,
        name="left_grip",
    )
    slider.visual(
        mesh_from_cadquery(_grip_shape(HANDLE_BAR_LEN / 2.0 - GRIP_LEN), "pump_right_grip"),
        material=rubber_mat,
        name="right_grip",
    )

    gauge_knob = model.part("gauge_knob")
    gauge_knob.visual(
        mesh_from_cadquery(_gauge_knob_shape(), "pump_gauge_knob"),
        material=knob_mat,
        name="knob",
    )

    model.articulation(
        "body_to_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.6,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_gauge_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=gauge_knob,
        origin=Origin(xyz=(0.0, GAUGE_JOINT_Y, GAUGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-1.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    slider = object_model.get_part("slider")
    gauge_knob = object_model.get_part("gauge_knob")
    slide_joint = object_model.get_articulation("body_to_slider")
    knob_joint = object_model.get_articulation("body_to_gauge_knob")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "slider articulation is vertical prismatic",
        slide_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in slide_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={slide_joint.articulation_type}, axis={slide_joint.axis}",
    )
    ctx.check(
        "gauge knob articulation spins about front axis",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )

    ctx.expect_origin_gap(
        slider,
        body,
        axis="z",
        min_gap=SLIDE_JOINT_Z - 0.005,
        max_gap=SLIDE_JOINT_Z + 0.005,
        name="slider starts at the top of the pump cylinder",
    )
    ctx.expect_origin_gap(
        gauge_knob,
        body,
        axis="y",
        min_gap=GAUGE_JOINT_Y - 0.004,
        max_gap=GAUGE_JOINT_Y + 0.004,
        name="gauge knob sits on the cylinder front",
    )
    ctx.expect_contact(
        slider,
        body,
        elem_a="stop_collar",
        elem_b="guide_ring",
        name="slider stop collar seats on the guide ring at rest",
    )
    ctx.expect_contact(
        gauge_knob,
        body,
        elem_a="knob",
        elem_b="gauge_boss",
        name="gauge knob is mounted against its front boss",
    )
    ctx.expect_within(
        slider,
        body,
        axes="xy",
        inner_elem="rod_core",
        outer_elem="guide_ring",
        margin=0.0015,
        name="piston rod stays centered through the top guide",
    )

    rest_pos = ctx.part_world_position(slider)
    slide_upper = slide_joint.motion_limits.upper

    with ctx.pose({slide_joint: slide_upper}):
        ctx.expect_within(
            slider,
            body,
            axes="xy",
            inner_elem="rod_core",
            outer_elem="guide_ring",
            margin=0.0015,
            name="raised piston rod stays centered through the top guide",
        )
        ctx.expect_overlap(
            slider,
            body,
            axes="z",
            elem_a="rod_core",
            elem_b="cylinder_stack",
            min_overlap=0.06,
            name="piston rod retains insertion at full stroke",
        )
        raised_pos = ctx.part_world_position(slider)

    ctx.check(
        "handle rises through a useful pump stroke",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.15,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    with ctx.pose({knob_joint: 0.7}):
        ctx.expect_contact(
            gauge_knob,
            body,
            elem_a="knob",
            elem_b="gauge_boss",
            name="gauge knob stays seated while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
