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


def _z_clip_box(z_min: float, height: float, span: float = 5.0) -> cq.Workplane:
    return cq.Workplane("XY").box(span, span, height).translate((0.0, 0.0, z_min + height / 2.0))


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float, z_min: float) -> cq.Workplane:
    outer = cq.Workplane("XY").cylinder(height, outer_radius).translate((0.0, 0.0, z_min + height / 2.0))
    inner = cq.Workplane("XY").cylinder(height + 0.01, inner_radius).translate((0.0, 0.0, z_min + height / 2.0))
    return outer.cut(inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.73, 0.74, 0.76, 1.0))
    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.18, 0.20, 0.24, 1.0))

    base_outer_radius = 2.20
    ring_inner_radius = 1.98
    slab_height = 0.18
    ring_height = 0.48
    bearing_height = 0.04
    shell_base_z = slab_height + ring_height + bearing_height

    pedestal_radius = 0.30
    pedestal_height = 0.74
    cap_radius = 0.38
    cap_height = 0.04

    dome_radius = 2.00
    shell_thickness = 0.06
    inner_radius = dome_radius - shell_thickness
    skirt_height = 0.45
    stage_bottom_z = 0.27
    stage_thickness = 0.08

    slit_width = 0.74
    slit_height = 1.06
    slit_bottom = 1.24
    slit_depth = 2.70
    slit_center_y = 1.22
    hinge_z = slit_bottom
    hinge_y = math.sqrt(max(dome_radius**2 - (hinge_z - skirt_height) ** 2, 0.0))

    foundation = cq.Workplane("XY").cylinder(slab_height, base_outer_radius).translate((0.0, 0.0, slab_height / 2.0))
    ring_wall = _annular_cylinder(base_outer_radius, ring_inner_radius, ring_height, slab_height)
    bearing_ring = _annular_cylinder(2.08, 1.86, bearing_height, slab_height + ring_height)
    base_ring_shape = foundation.union(ring_wall).union(bearing_ring)
    pedestal = cq.Workplane("XY").cylinder(pedestal_height, pedestal_radius).translate(
        (0.0, 0.0, slab_height + pedestal_height / 2.0)
    )
    pedestal_cap = cq.Workplane("XY").cylinder(cap_height, cap_radius).translate(
        (0.0, 0.0, slab_height + pedestal_height + cap_height / 2.0)
    )
    support_shape = pedestal.union(pedestal_cap)

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_ring_shape, "base_ring"),
        material=concrete,
        name="base_ring",
    )
    base.visual(
        mesh_from_cadquery(support_shape, "support_column"),
        material=concrete,
        name="support_column",
    )

    upper_clip = _z_clip_box(skirt_height, dome_radius + 0.15, span=4.6)
    outer_cap = cq.Workplane("XY").sphere(dome_radius).translate((0.0, 0.0, skirt_height)).intersect(upper_clip)
    inner_cap = cq.Workplane("XY").sphere(inner_radius).translate((0.0, 0.0, skirt_height)).intersect(upper_clip)
    outer_skirt = cq.Workplane("XY").cylinder(skirt_height, dome_radius).translate((0.0, 0.0, skirt_height / 2.0))
    inner_skirt = cq.Workplane("XY").cylinder(skirt_height + 0.01, inner_radius).translate(
        (0.0, 0.0, skirt_height / 2.0)
    )
    stage_disk = _annular_cylinder(inner_radius, 0.46, stage_thickness, stage_bottom_z)
    shell_shape = outer_cap.union(outer_skirt).union(stage_disk).cut(inner_cap.union(inner_skirt))

    slit_cutter = cq.Workplane("XY").box(slit_width, slit_depth, slit_height).translate(
        (0.0, slit_center_y, slit_bottom + slit_height / 2.0)
    )
    shell_shape = shell_shape.cut(slit_cutter)

    hinge_mount_shape = cq.Workplane("XY").box(0.08, 0.10, 0.12).translate((0.31, hinge_y + 0.005, hinge_z + 0.005))
    hinge_mount_shape = hinge_mount_shape.union(
        cq.Workplane("XY").box(0.08, 0.10, 0.12).translate((-0.31, hinge_y + 0.005, hinge_z + 0.005))
    )

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(shell_shape, "shell"),
        material=shell_white,
        name="shell_body",
    )

    shell.visual(
        mesh_from_cadquery(
            _annular_cylinder(2.02, 1.90, 0.025, 0.015),
            "rotation_skirt",
        ),
        material=trim_dark,
        name="rotation_skirt",
    )
    shell.visual(
        mesh_from_cadquery(hinge_mount_shape, "hinge_mount"),
        material=trim_dark,
        name="hinge_mount",
    )

    leaf_outer_radius = dome_radius + 0.035
    leaf_thickness = 0.04
    leaf_top = 2.38
    leaf_width = slit_width + 0.10
    leaf_front_depth = 0.70

    leaf_outer = cq.Workplane("XY").sphere(leaf_outer_radius).translate((0.0, 0.0, skirt_height))
    leaf_inner = cq.Workplane("XY").sphere(leaf_outer_radius - leaf_thickness).translate((0.0, 0.0, skirt_height))
    leaf_box = cq.Workplane("XY").box(leaf_width, leaf_front_depth, leaf_top - hinge_z + 0.04).translate(
        (0.0, hinge_y + leaf_front_depth / 2.0 - 0.03, (hinge_z + leaf_top + 0.04) / 2.0)
    )
    leaf_cover_shape = leaf_outer.cut(leaf_inner).intersect(leaf_box).translate((0.0, -hinge_y + 0.165, -hinge_z))

    shutter = model.part("shutter")
    shutter.visual(
        mesh_from_cadquery(leaf_cover_shape, "leaf_cover"),
        material=shell_white,
        name="leaf_cover",
    )
    hinge_arm_shape = cq.Workplane("YZ").cylinder(leaf_width * 0.76, 0.025)
    hinge_arm_shape = hinge_arm_shape.union(
        cq.Workplane("XY").box(0.10, 0.14, 0.72).translate((0.24, 0.07, 0.36))
    ).union(
        cq.Workplane("XY").box(0.10, 0.14, 0.72).translate((-0.24, 0.07, 0.36))
    )
    shutter.visual(
        mesh_from_cadquery(hinge_arm_shape, "hinge_arm"),
        material=trim_dark,
        name="hinge_arm",
    )

    model.articulation(
        "base_to_shell",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shell,
        origin=Origin(xyz=(0.0, 0.0, shell_base_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2),
    )

    model.articulation(
        "shell_to_shutter",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=shutter,
        origin=Origin(xyz=(0.0, hinge_y + 0.03, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=0.0, upper=1.30),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    shutter = object_model.get_part("shutter")
    dome_turn = object_model.get_articulation("base_to_shell")
    shutter_hinge = object_model.get_articulation("shell_to_shutter")

    ctx.allow_overlap(
        shell,
        shutter,
        elem_a="hinge_mount",
        elem_b="hinge_arm",
        reason="The shutter hinge arm is intentionally simplified as nesting into the slit-edge hinge mount.",
    )

    ctx.expect_overlap(
        shutter,
        shell,
        axes="x",
        elem_a="leaf_cover",
        elem_b="shell_body",
        min_overlap=0.62,
        name="closed shutter spans the slit width",
    )

    ctx.expect_overlap(
        shutter,
        shell,
        axes="z",
        elem_a="leaf_cover",
        elem_b="shell_body",
        min_overlap=0.12,
        name="closed shutter covers the crown opening height",
    )

    ctx.expect_gap(
        shutter,
        shell,
        axis="y",
        positive_elem="leaf_cover",
        negative_elem="shell_body",
        min_gap=-0.005,
        max_gap=0.09,
        name="closed shutter sits just proud of the dome shell",
    )

    rest_leaf_origin = ctx.part_world_position(shutter)
    with ctx.pose({dome_turn: 0.80}):
        turned_leaf_origin = ctx.part_world_position(shutter)
    ctx.check(
        "dome rotation carries the shutter around the vertical axis",
        rest_leaf_origin is not None
        and turned_leaf_origin is not None
        and abs(turned_leaf_origin[0]) > 1.10
        and turned_leaf_origin[1] < rest_leaf_origin[1] - 0.40,
        details=f"rest={rest_leaf_origin}, turned={turned_leaf_origin}",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(shutter, elem="leaf_cover")
    with ctx.pose({shutter_hinge: 1.10}):
        open_cover_aabb = ctx.part_element_world_aabb(shutter, elem="leaf_cover")
    ctx.check(
        "shutter leaf swings outward from the crown opening",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][1] > closed_cover_aabb[1][1] + 0.015
        and open_cover_aabb[1][2] < closed_cover_aabb[1][2] - 0.12,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
