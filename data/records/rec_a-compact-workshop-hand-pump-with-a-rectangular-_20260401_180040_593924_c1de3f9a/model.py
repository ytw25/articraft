from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATFORM_L = 0.30
PLATFORM_W = 0.095
PLATFORM_T = 0.014

PUMP_HEAD_R = 0.038
PUMP_HEAD_H = 0.018

CYLINDER_OUTER_R = 0.029
CYLINDER_INNER_R = 0.0255
CYLINDER_H = 0.150
CYLINDER_Z0 = PLATFORM_T + PUMP_HEAD_H

GUIDE_OUTER_R = 0.031
GUIDE_INNER_R = 0.0076
GUIDE_H = 0.014
GUIDE_Z0 = CYLINDER_Z0 + CYLINDER_H

PLUNGER_TRAVEL = 0.110
PLUNGER_GUIDE_Z = GUIDE_Z0 + (GUIDE_H / 2.0)
ROD_R = 0.0065
ROD_LOWER = 0.156
ROD_UPPER = 0.255
ROD_LEN = ROD_LOWER + ROD_UPPER

HANDLE_BAR_R = 0.008
HANDLE_BAR_LEN = 0.190
HANDLE_Z = 0.255
GRIP_R = 0.015
GRIP_LEN = 0.078
GRIP_CENTER_Y = 0.070

PISTON_R = 0.0245
PISTON_LEN = 0.018
PISTON_CENTER_Z = -0.147

HINGE_X = (PLATFORM_L / 2.0) + 0.011
HINGE_R = 0.009
HINGE_Z = PLATFORM_T + HINGE_R + 0.001
BASE_KNUCKLE_LEN = 0.016
PEG_BARREL_LEN = 0.024
BASE_KNUCKLE_Y = (PEG_BARREL_LEN + BASE_KNUCKLE_LEN) / 2.0
PEG_FOLD_UPPER = 1.30

PEG_LEN = 0.090
PEG_W = 0.050
PEG_T = 0.010


def _build_base_body() -> cq.Workplane:
    platform = (
        cq.Workplane("XY")
        .box(PLATFORM_L, PLATFORM_W, PLATFORM_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )

    pump_head = (
        cq.Workplane("XY")
        .circle(PUMP_HEAD_R)
        .extrude(PUMP_HEAD_H)
        .translate((0.0, 0.0, PLATFORM_T))
    )

    shoulder = (
        cq.Workplane("XY")
        .circle(PUMP_HEAD_R + 0.010)
        .extrude(0.006)
        .translate((0.0, 0.0, PLATFORM_T))
    )

    left_hinge_support = (
        cq.Workplane("XY")
        .box(0.022, 0.016, HINGE_Z, centered=(False, True, False))
        .translate((PLATFORM_L / 2.0 - 0.020, -BASE_KNUCKLE_Y, 0.0))
    )
    right_hinge_support = (
        cq.Workplane("XY")
        .box(0.022, 0.016, HINGE_Z, centered=(False, True, False))
        .translate((PLATFORM_L / 2.0 - 0.020, BASE_KNUCKLE_Y, 0.0))
    )

    rear_rib = (
        cq.Workplane("XY")
        .box(0.090, 0.030, 0.010, centered=(True, True, False))
        .translate((-0.040, 0.0, PLATFORM_T))
    )

    return (
        platform.union(shoulder)
        .union(pump_head)
        .union(left_hinge_support)
        .union(right_hinge_support)
        .union(rear_rib)
    )


def _build_cylinder_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(CYLINDER_OUTER_R).extrude(CYLINDER_H)
    inner = cq.Workplane("XY").circle(CYLINDER_INNER_R).extrude(CYLINDER_H)
    return outer.cut(inner).translate((0.0, 0.0, CYLINDER_Z0))


def _build_guide_cap() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(GUIDE_OUTER_R).extrude(GUIDE_H)
    inner = cq.Workplane("XY").circle(GUIDE_INNER_R).extrude(GUIDE_H)
    return outer.cut(inner).translate((0.0, 0.0, GUIDE_Z0))


def _build_foot_peg_body() -> cq.Workplane:
    barrel = (
        cq.Workplane("XY")
        .cylinder(PEG_BARREL_LEN, HINGE_R)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )

    web = (
        cq.Workplane("XY")
        .box(0.032, 0.032, 0.020, centered=(False, True, False))
        .translate((0.002, 0.0, -0.022))
    )

    plate = (
        cq.Workplane("XY")
        .box(PEG_LEN, PEG_W, PEG_T, centered=(False, True, False))
        .translate((0.012, 0.0, -0.026))
        .edges("|Z")
        .fillet(0.003)
    )

    nose = (
        cq.Workplane("XY")
        .cylinder(PEG_W * 0.48, PEG_T / 2.0)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((PEG_LEN + 0.016, 0.0, -0.021))
    )

    tread = (
        cq.Workplane("XY")
        .pushPoints([(0.032, 0.0), (0.052, 0.0), (0.072, 0.0)])
        .box(0.004, PEG_W * 0.78, 0.003, centered=(True, True, False))
        .translate((0.0, 0.0, -0.016))
    )

    return barrel.union(web).union(plate).union(nose).union(tread)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_hand_pump")

    model.material("body_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("rubber_gray", rgba=(0.18, 0.19, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_body(), "pump_base_body"),
        name="base_body",
        material="body_dark",
    )
    base.visual(
        mesh_from_cadquery(_build_cylinder_shell(), "pump_cylinder_shell"),
        name="cylinder_shell",
        material="steel",
    )
    base.visual(
        mesh_from_cadquery(_build_guide_cap(), "pump_guide_cap"),
        name="guide_cap",
        material="body_dark",
    )
    base.visual(
        Cylinder(radius=HINGE_R, length=BASE_KNUCKLE_LEN),
        origin=Origin(
            xyz=(HINGE_X, -BASE_KNUCKLE_Y, HINGE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        name="left_knuckle",
        material="body_dark",
    )
    base.visual(
        Cylinder(radius=HINGE_R, length=BASE_KNUCKLE_LEN),
        origin=Origin(
            xyz=(HINGE_X, BASE_KNUCKLE_Y, HINGE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        name="right_knuckle",
        material="body_dark",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=ROD_R, length=ROD_LEN),
        origin=Origin(xyz=(0.0, 0.0, (ROD_UPPER - ROD_LOWER) / 2.0)),
        name="rod",
        material="steel",
    )
    plunger.visual(
        Cylinder(radius=PISTON_R, length=PISTON_LEN),
        origin=Origin(xyz=(0.0, 0.0, PISTON_CENTER_Z)),
        name="piston_head",
        material="rubber_gray",
    )
    plunger.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, HANDLE_Z - 0.010)),
        name="handle_hub",
        material="steel",
    )
    plunger.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        name="stop_collar",
        material="steel",
    )
    plunger.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, HANDLE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        name="handle_bar",
        material="steel",
    )
    plunger.visual(
        Cylinder(radius=GRIP_R, length=GRIP_LEN),
        origin=Origin(
            xyz=(0.0, -GRIP_CENTER_Y, HANDLE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        name="left_grip",
        material="rubber_black",
    )
    plunger.visual(
        Cylinder(radius=GRIP_R, length=GRIP_LEN),
        origin=Origin(
            xyz=(0.0, GRIP_CENTER_Y, HANDLE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        name="right_grip",
        material="rubber_black",
    )

    foot_peg = model.part("foot_peg")
    foot_peg.visual(
        mesh_from_cadquery(_build_foot_peg_body(), "pump_foot_peg_body"),
        name="peg_body",
        material="body_dark",
    )

    model.articulation(
        "base_to_plunger",
        ArticulationType.PRISMATIC,
        parent=base,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, PLUNGER_GUIDE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PLUNGER_TRAVEL,
            effort=180.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "base_to_foot_peg",
        ArticulationType.REVOLUTE,
        parent=base,
        child=foot_peg,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=PEG_FOLD_UPPER,
            effort=20.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    plunger = object_model.get_part("plunger")
    foot_peg = object_model.get_part("foot_peg")
    pump_slide = object_model.get_articulation("base_to_plunger")
    peg_hinge = object_model.get_articulation("base_to_foot_peg")

    rod = plunger.get_visual("rod")
    piston_head = plunger.get_visual("piston_head")
    stop_collar = plunger.get_visual("stop_collar")
    peg_body = foot_peg.get_visual("peg_body")
    cylinder_shell = base.get_visual("cylinder_shell")
    left_knuckle = base.get_visual("left_knuckle")
    right_knuckle = base.get_visual("right_knuckle")
    guide_cap = base.get_visual("guide_cap")

    ctx.allow_overlap(
        plunger,
        base,
        elem_a=piston_head,
        elem_b=cylinder_shell,
        reason="The hidden piston seal intentionally bears against the cylinder wall to represent the pump's sliding seal.",
    )
    ctx.allow_overlap(
        foot_peg,
        base,
        elem_a=peg_body,
        elem_b=left_knuckle,
        reason="The hinge eye is represented as a solid barrel sitting inside a simplified left knuckle sleeve proxy.",
    )
    ctx.allow_overlap(
        foot_peg,
        base,
        elem_a=peg_body,
        elem_b=right_knuckle,
        reason="The hinge eye is represented as a solid barrel sitting inside a simplified right knuckle sleeve proxy.",
    )

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

    ctx.expect_within(
        plunger,
        base,
        axes="xy",
        inner_elem=rod,
        outer_elem=cylinder_shell,
        margin=0.0,
        name="plunger rod stays centered in the pressure cylinder",
    )
    ctx.expect_overlap(
        plunger,
        base,
        axes="z",
        elem_a=rod,
        elem_b=cylinder_shell,
        min_overlap=0.145,
        name="closed plunger keeps substantial rod insertion",
    )
    ctx.expect_contact(
        plunger,
        base,
        elem_a=stop_collar,
        elem_b=guide_cap,
        contact_tol=1e-5,
        name="depressed plunger rests on the guide cap stop",
    )
    ctx.expect_contact(
        foot_peg,
        base,
        elem_a=peg_body,
        elem_b=left_knuckle,
        contact_tol=1e-5,
        name="foot peg barrel is captured by the left hinge knuckle",
    )
    ctx.expect_contact(
        foot_peg,
        base,
        elem_a=peg_body,
        elem_b=right_knuckle,
        contact_tol=1e-5,
        name="foot peg barrel is captured by the right hinge knuckle",
    )

    plunger_rest = ctx.part_world_position(plunger)
    with ctx.pose({pump_slide: PLUNGER_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no unintended overlap with the plunger fully extended")
        ctx.expect_within(
            plunger,
            base,
            axes="xy",
            inner_elem=rod,
            outer_elem=cylinder_shell,
            margin=0.0,
            name="extended plunger rod remains centered in the pressure cylinder",
        )
        ctx.expect_overlap(
            plunger,
            base,
            axes="z",
            elem_a=rod,
            elem_b=cylinder_shell,
            min_overlap=0.035,
            name="extended plunger retains insertion in the pressure cylinder",
        )
        plunger_extended = ctx.part_world_position(plunger)

    ctx.check(
        "plunger rises upward along the cylinder axis",
        plunger_rest is not None
        and plunger_extended is not None
        and plunger_extended[2] > plunger_rest[2] + 0.08,
        details=f"rest={plunger_rest}, extended={plunger_extended}",
    )

    peg_rest_aabb = ctx.part_world_aabb(foot_peg)
    with ctx.pose({peg_hinge: PEG_FOLD_UPPER}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no unintended overlap with the foot peg folded")
        peg_folded_aabb = ctx.part_world_aabb(foot_peg)

    peg_rest = (
        None
        if peg_rest_aabb is None
        else tuple((peg_rest_aabb[0][i] + peg_rest_aabb[1][i]) / 2.0 for i in range(3))
    )
    peg_folded = (
        None
        if peg_folded_aabb is None
        else tuple((peg_folded_aabb[0][i] + peg_folded_aabb[1][i]) / 2.0 for i in range(3))
    )

    ctx.check(
        "foot peg folds upward on its transverse hinge",
        peg_rest is not None and peg_folded is not None and peg_folded[2] > peg_rest[2] + 0.03,
        details=f"rest={peg_rest}, folded={peg_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
