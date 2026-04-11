from __future__ import annotations

import math

import cadquery as cq

from sdk import (
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


HANDLE_LENGTH = 0.118
HANDLE_SCALE_THICKNESS = 0.0053
HANDLE_GAP = 0.0064
HANDLE_HALF_THICKNESS = HANDLE_SCALE_THICKNESS + HANDLE_GAP / 2.0

PIVOT_RADIUS = 0.0061
BLADE_THICKNESS = 0.0038
PIVOT_HOLE_RADIUS = 0.00675

BUTTON_X = -0.014
BUTTON_Z = 0.002
BUTTON_CAP_RADIUS = 0.0058
BUTTON_CAP_THICKNESS = 0.0028
BUTTON_TRAVEL = 0.0022


def _handle_scale_shape(button_side: bool = False) -> cq.Workplane:
    scale = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.004, -0.0045),
                (-0.010, -0.0085),
                (-0.043, -0.0122),
                (-0.079, -0.0126),
                (-0.107, -0.0102),
                (-0.116, -0.0068),
                (-0.114, 0.0048),
                (-0.102, 0.0102),
                (-0.071, 0.0136),
                (-0.038, 0.0135),
                (-0.010, 0.0108),
                (0.004, 0.0062),
            ]
        )
        .close()
        .extrude(HANDLE_SCALE_THICKNESS)
    )

    if button_side:
        scale = (
            scale.faces(">Y")
            .workplane()
            .pushPoints([(BUTTON_X, BUTTON_Z)])
            .circle(BUTTON_CAP_RADIUS + 0.0004)
            .cutBlind(-0.0025)
        )

    return scale


def _blade_shape() -> cq.Workplane:
    blade = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0088, 0.0064),
                (-0.010, 0.0114),
                (-0.035, 0.0144),
                (-0.067, 0.0135),
                (-0.090, 0.0082),
                (-0.094, 0.0043),
                (-0.082, -0.0008),
                (-0.046, -0.0056),
                (-0.012, -0.0062),
                (0.0082, -0.0034),
            ]
        )
        .close()
        .extrude(BLADE_THICKNESS)
    )
    blade = (
        blade.faces(">Y")
        .workplane()
        .pushPoints([(0.0, 0.0)])
        .circle(PIVOT_HOLE_RADIUS)
        .cutThruAll()
    )
    return blade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_pocket_knife")

    handle_material = model.material("handle_material", rgba=(0.20, 0.21, 0.22, 1.0))
    blade_material = model.material("blade_material", rgba=(0.76, 0.78, 0.80, 1.0))
    hardware_material = model.material("hardware_material", rgba=(0.36, 0.37, 0.39, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_scale_shape(button_side=False), "left_scale"),
        origin=Origin(xyz=(0.0, -HANDLE_GAP / 2.0, 0.0)),
        material=handle_material,
        name="left_scale",
    )
    handle.visual(
        mesh_from_cadquery(_handle_scale_shape(button_side=True), "right_scale"),
        origin=Origin(xyz=(0.0, HANDLE_HALF_THICKNESS, 0.0)),
        material=handle_material,
        name="right_scale",
    )
    handle.visual(
        Box((0.047, HANDLE_GAP + 2.0 * HANDLE_SCALE_THICKNESS, 0.0052)),
        origin=Origin(xyz=(-0.081, 0.0, 0.0108)),
        material=handle_material,
        name="backspacer",
    )
    handle.visual(
        Cylinder(radius=PIVOT_RADIUS, length=HANDLE_GAP + 2.0 * HANDLE_SCALE_THICKNESS + 0.0010),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name="pivot_barrel",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_blade_shape(), "blade_body"),
        origin=Origin(xyz=(0.0, BLADE_THICKNESS / 2.0, 0.0)),
        material=blade_material,
        name="blade_body",
    )
    blade.visual(
        Cylinder(radius=0.0022, length=0.0048),
        origin=Origin(xyz=(-0.024, 0.0, 0.0076), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name="thumb_stud",
    )

    button = model.part("button")
    button.visual(
        Cylinder(radius=BUTTON_CAP_RADIUS, length=BUTTON_CAP_THICKNESS),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_material,
        name="button_cap",
    )

    model.articulation(
        "blade_hinge",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=blade,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=4.0,
            lower=0.0,
            upper=2.65,
        ),
    )
    model.articulation(
        "button_slide",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=button,
        origin=Origin(
            xyz=(
                BUTTON_X,
                HANDLE_HALF_THICKNESS + BUTTON_CAP_THICKNESS / 2.0,
                BUTTON_Z,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    blade = object_model.get_part("blade")
    blade_hinge = object_model.get_articulation("blade_hinge")

    ctx.expect_overlap(
        blade,
        handle,
        axes="xz",
        min_overlap=0.019,
        name="closed blade nests inside the handle silhouette",
    )

    closed_blade_aabb = ctx.part_world_aabb(blade)
    handle_aabb = ctx.part_world_aabb(handle)
    ctx.check(
        "closed blade stays within the front handle envelope",
        closed_blade_aabb is not None
        and handle_aabb is not None
        and closed_blade_aabb[1][0] <= handle_aabb[1][0] + 0.003,
        details=f"blade_aabb={closed_blade_aabb}, handle_aabb={handle_aabb}",
    )

    limits = blade_hinge.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({blade_hinge: limits.upper}):
            open_blade_aabb = ctx.part_world_aabb(blade)
            open_handle_aabb = ctx.part_world_aabb(handle)
            ctx.check(
                "blade opens forward and above the handle",
                open_blade_aabb is not None
                and open_handle_aabb is not None
                and open_blade_aabb[1][0] > open_handle_aabb[1][0] + 0.065
                and open_blade_aabb[1][2] > open_handle_aabb[1][2] + 0.030,
                details=f"blade_aabb={open_blade_aabb}, handle_aabb={open_handle_aabb}",
            )

    button = object_model.get_part("button")
    button_slide = object_model.get_articulation("button_slide")

    ctx.expect_gap(
        button,
        handle,
        axis="y",
        positive_elem="button_cap",
        negative_elem="right_scale",
        max_gap=0.0003,
        max_penetration=0.0,
        name="button cap starts flush with the lock side scale",
    )

    rest_button_pos = ctx.part_world_position(button)
    slide_limits = button_slide.motion_limits
    if slide_limits is not None and slide_limits.upper is not None:
        with ctx.pose({button_slide: slide_limits.upper}):
            pressed_button_pos = ctx.part_world_position(button)
            ctx.check(
                "button presses inward on its local axis",
                rest_button_pos is not None
                and pressed_button_pos is not None
                and pressed_button_pos[1] < rest_button_pos[1] - 0.0015,
                details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
            )

    return ctx.report()


object_model = build_object_model()
