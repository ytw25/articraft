from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="molded_toolbox")

    plastic_body = model.material("plastic_body", rgba=(0.18, 0.19, 0.20, 1.0))
    plastic_dark = model.material("plastic_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    plastic_accent = model.material("plastic_accent", rgba=(0.78, 0.17, 0.10, 1.0))
    tray_color = model.material("tray_color", rgba=(0.72, 0.72, 0.74, 1.0))

    width = 0.56
    depth = 0.30
    body_height = 0.235
    wall = 0.006
    lid_height = 0.045
    lid_skin = 0.005
    hinge_z = body_height + 0.002

    tray_width = width - 2.0 * (wall + 0.012)
    tray_depth = depth - 2.0 * (wall + 0.018)
    tray_height = 0.042
    tray_floor = 0.004
    tray_rest_z = 0.156

    body = model.part("body")
    body.visual(
        Box((width, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall / 2.0)),
        material=plastic_body,
        name="tub_floor",
    )
    wall_height = body_height - wall
    wall_z = wall + wall_height / 2.0
    body.visual(
        Box((width, wall, wall_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, wall_z)),
        material=plastic_body,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, wall_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, wall_z)),
        material=plastic_body,
        name="rear_wall",
    )
    inner_depth = depth - 2.0 * wall + 0.002
    body.visual(
        Box((wall, inner_depth, wall_height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, wall_z)),
        material=plastic_body,
        name="right_wall",
    )
    body.visual(
        Box((wall, inner_depth, wall_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, wall_z)),
        material=plastic_body,
        name="left_wall",
    )

    rim_thickness = 0.008
    rim_width = 0.018
    rim_z = body_height - rim_thickness / 2.0
    body.visual(
        Box((width - 2.0 * wall, rim_width, rim_thickness)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall - rim_width / 2.0, rim_z)),
        material=plastic_body,
        name="front_rim",
    )
    body.visual(
        Box((width - 2.0 * wall, rim_width, rim_thickness)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall + rim_width / 2.0, rim_z)),
        material=plastic_body,
        name="rear_rim",
    )
    side_rim_depth = depth - 2.0 * wall - 2.0 * rim_width + 0.002
    body.visual(
        Box((rim_width, side_rim_depth, rim_thickness)),
        origin=Origin(xyz=(width / 2.0 - wall - rim_width / 2.0, 0.0, rim_z)),
        material=plastic_body,
        name="right_rim",
    )
    body.visual(
        Box((rim_width, side_rim_depth, rim_thickness)),
        origin=Origin(xyz=(-width / 2.0 + wall + rim_width / 2.0, 0.0, rim_z)),
        material=plastic_body,
        name="left_rim",
    )

    rail_z = tray_rest_z - 0.002
    rail_len = tray_depth - 0.016
    rail_offset_x = tray_width / 2.0 + 0.008
    for side, x in (("left", -rail_offset_x), ("right", rail_offset_x)):
        body.visual(
            Box((0.020, rail_len, 0.006)),
            origin=Origin(xyz=(x, 0.0, rail_z)),
            material=plastic_dark,
            name=f"{side}_tray_rail",
        )

    body.visual(
        Box((0.064, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.004, body_height - 0.022)),
        material=plastic_accent,
        name="latch_keeper",
    )

    foot_z = 0.006 / 2.0
    for x in (-0.20, 0.20):
        for y in (-0.10, 0.10):
            body.visual(
                Box((0.055, 0.035, 0.006)),
                origin=Origin(xyz=(x, y, foot_z)),
                material=plastic_dark,
                name=f"foot_{'r' if x > 0 else 'l'}_{'f' if y > 0 else 'r'}",
            )

    body.inertial = Inertial.from_geometry(
        Box((width, depth, body_height)),
        mass=3.3,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    lid = model.part("lid")
    recess_width = 0.270
    recess_depth = 0.120
    recess_floor_z = 0.016
    recess_wall_height = lid_height - lid_skin - recess_floor_z + 0.001
    top_ring_y = depth / 2.0
    top_ring_z = lid_height - lid_skin / 2.0
    side_ring_width = (width - recess_width) / 2.0 + 0.002
    front_ring_depth = (depth - recess_depth) / 2.0 + 0.002

    lid.visual(
        Box((side_ring_width, depth, lid_skin)),
        origin=Origin(
            xyz=((width + recess_width) / 4.0, top_ring_y, top_ring_z)
        ),
        material=plastic_body,
        name="right_top_rail",
    )
    lid.visual(
        Box((side_ring_width, depth, lid_skin)),
        origin=Origin(
            xyz=(-(width + recess_width) / 4.0, top_ring_y, top_ring_z)
        ),
        material=plastic_body,
        name="left_top_rail",
    )
    lid.visual(
        Box((recess_width + 0.002, front_ring_depth, lid_skin)),
        origin=Origin(
            xyz=(0.0, front_ring_depth / 2.0, top_ring_z)
        ),
        material=plastic_body,
        name="rear_top_rail",
    )
    lid.visual(
        Box((recess_width + 0.002, front_ring_depth, lid_skin)),
        origin=Origin(
            xyz=(0.0, depth - front_ring_depth / 2.0, top_ring_z)
        ),
        material=plastic_body,
        name="front_top_rail",
    )

    skirt_height = lid_height - lid_skin + 0.001
    skirt_z = skirt_height / 2.0
    lid.visual(
        Box((0.150, wall, skirt_height)),
        origin=Origin(xyz=(-0.195, wall / 2.0, skirt_z)),
        material=plastic_body,
        name="left_rear_skirt",
    )
    lid.visual(
        Box((0.150, wall, skirt_height)),
        origin=Origin(xyz=(0.195, wall / 2.0, skirt_z)),
        material=plastic_body,
        name="right_rear_skirt",
    )
    lid.visual(
        Box((width, wall, skirt_height)),
        origin=Origin(xyz=(0.0, depth - wall / 2.0, skirt_z)),
        material=plastic_body,
        name="front_skirt",
    )
    lid.visual(
        Box((wall, depth - 2.0 * wall + 0.002, skirt_height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, depth / 2.0, skirt_z)),
        material=plastic_body,
        name="right_skirt",
    )
    lid.visual(
        Box((wall, depth - 2.0 * wall + 0.002, skirt_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, depth / 2.0, skirt_z)),
        material=plastic_body,
        name="left_skirt",
    )

    lid.visual(
        Box((recess_width, recess_depth, 0.004)),
        origin=Origin(xyz=(0.0, depth / 2.0, recess_floor_z)),
        material=plastic_dark,
        name="handle_recess_floor",
    )
    lid.visual(
        Box((wall, recess_depth + 0.004, recess_wall_height)),
        origin=Origin(
            xyz=(
                recess_width / 2.0 - wall / 2.0,
                depth / 2.0,
                recess_floor_z + recess_wall_height / 2.0,
            )
        ),
        material=plastic_body,
        name="recess_right_wall",
    )
    lid.visual(
        Box((wall, recess_depth + 0.004, recess_wall_height)),
        origin=Origin(
            xyz=(
                -recess_width / 2.0 + wall / 2.0,
                depth / 2.0,
                recess_floor_z + recess_wall_height / 2.0,
            )
        ),
        material=plastic_body,
        name="recess_left_wall",
    )
    lid.visual(
        Box((recess_width, wall, recess_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                depth / 2.0 - recess_depth / 2.0 + wall / 2.0,
                recess_floor_z + recess_wall_height / 2.0,
            )
        ),
        material=plastic_body,
        name="recess_rear_wall",
    )
    lid.visual(
        Box((recess_width, wall, recess_wall_height)),
        origin=Origin(
            xyz=(
                0.0,
                depth / 2.0 + recess_depth / 2.0 - wall / 2.0,
                recess_floor_z + recess_wall_height / 2.0,
            )
        ),
        material=plastic_body,
        name="recess_front_wall",
    )
    lid.visual(
        Box((0.090, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, depth / 2.0 - 0.030, recess_floor_z + 0.002)),
        material=plastic_accent,
        name="brand_pad",
    )

    body.visual(
        Cylinder(radius=0.007, length=0.170),
        origin=Origin(
            xyz=(-0.105, -depth / 2.0 - 0.006, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=plastic_dark,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.170),
        origin=Origin(
            xyz=(0.105, -depth / 2.0 - 0.006, hinge_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=plastic_dark,
        name="right_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.040),
        origin=Origin(
            xyz=(0.0, -0.010, hinge_z - body_height - 0.002),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=plastic_dark,
        name="center_hinge_barrel",
    )
    lid.visual(
        Box((0.030, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, 0.000, 0.020)),
        material=plastic_body,
        name="center_hinge_web",
    )

    lid.inertial = Inertial.from_geometry(
        Box((width, depth, lid_height)),
        mass=1.2,
        origin=Origin(xyz=(0.0, depth / 2.0, lid_height / 2.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -depth / 2.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    for side, x in (("left", -0.029), ("right", 0.029)):
        body.visual(
            Box((0.018, 0.020, 0.042)),
            origin=Origin(
                xyz=(x, depth / 2.0 + 0.001, body_height - 0.053),
            ),
            material=plastic_body,
            name=f"{side}_latch_pivot_boss",
        )
        body.visual(
            Cylinder(radius=0.007, length=0.024),
            origin=Origin(
                xyz=(x, depth / 2.0 + 0.011, body_height - 0.053),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=plastic_dark,
            name=f"{side}_latch_pivot_mount",
        )

    latch = model.part("latch")
    latch.visual(
        Box((0.018, 0.010, 0.024)),
        origin=Origin(xyz=(-0.029, 0.001, -0.029)),
        material=plastic_dark,
        name="left_latch_ear",
    )
    latch.visual(
        Box((0.018, 0.010, 0.024)),
        origin=Origin(xyz=(0.029, 0.001, -0.029)),
        material=plastic_dark,
        name="right_latch_ear",
    )
    latch.visual(
        Box((0.076, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.001, -0.018)),
        material=plastic_dark,
        name="latch_crossbar",
    )
    latch.visual(
        Box((0.054, 0.010, 0.042)),
        origin=Origin(xyz=(0.0, 0.001, -0.040)),
        material=plastic_accent,
        name="main_latch_plate",
    )
    latch.visual(
        Box((0.032, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -0.001, -0.019)),
        material=plastic_dark,
        name="latch_hook",
    )
    latch.visual(
        Box((0.036, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.006, -0.056)),
        material=plastic_dark,
        name="finger_tab",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.076, 0.024, 0.070)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.002, -0.035)),
    )

    model.articulation(
        "body_to_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(0.0, depth / 2.0 + 0.011, body_height - 0.053)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )

    lid.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(
            xyz=(-0.119, depth / 2.0, 0.030),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=plastic_dark,
        name="left_handle_mount",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(
            xyz=(0.119, depth / 2.0, 0.030),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=plastic_dark,
        name="right_handle_mount",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0065, length=0.206),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_dark,
        name="handle_axle",
    )
    handle.visual(
        Box((0.016, 0.036, 0.016)),
        origin=Origin(xyz=(-0.082, 0.012, -0.002)),
        material=plastic_dark,
        name="left_handle_leg",
    )
    handle.visual(
        Box((0.016, 0.036, 0.016)),
        origin=Origin(xyz=(0.082, 0.012, -0.002)),
        material=plastic_dark,
        name="right_handle_leg",
    )
    handle.visual(
        Box((0.186, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, 0.028, -0.003)),
        material=plastic_dark,
        name="handle_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.206, 0.050, 0.024)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.018, -0.002)),
    )

    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, depth / 2.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )

    tray = model.part("tray")
    runner_center_x = tray_width / 2.0 - 0.004
    tray.visual(
        Box((0.016, tray_depth - 0.024, 0.006)),
        origin=Origin(xyz=(-runner_center_x, 0.0, 0.003)),
        material=tray_color,
        name="left_runner",
    )
    tray.visual(
        Box((0.016, tray_depth - 0.024, 0.006)),
        origin=Origin(xyz=(runner_center_x, 0.0, 0.003)),
        material=tray_color,
        name="right_runner",
    )
    tray.visual(
        Box((tray_width, tray_depth, tray_floor)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=tray_color,
        name="tray_floor",
    )
    tray.visual(
        Box((tray_width, wall, tray_height)),
        origin=Origin(xyz=(0.0, tray_depth / 2.0 - wall / 2.0, 0.006 + tray_height / 2.0)),
        material=tray_color,
        name="tray_front_wall",
    )
    tray.visual(
        Box((tray_width, wall, tray_height)),
        origin=Origin(xyz=(0.0, -tray_depth / 2.0 + wall / 2.0, 0.006 + tray_height / 2.0)),
        material=tray_color,
        name="tray_rear_wall",
    )
    tray.visual(
        Box((wall, tray_depth - 2.0 * wall + 0.002, tray_height)),
        origin=Origin(xyz=(tray_width / 2.0 - wall / 2.0, 0.0, 0.006 + tray_height / 2.0)),
        material=tray_color,
        name="tray_right_wall",
    )
    tray.visual(
        Box((wall, tray_depth - 2.0 * wall + 0.002, tray_height)),
        origin=Origin(xyz=(-tray_width / 2.0 + wall / 2.0, 0.0, 0.006 + tray_height / 2.0)),
        material=tray_color,
        name="tray_left_wall",
    )
    tray.visual(
        Box((wall, tray_depth - 0.040, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=tray_color,
        name="long_divider",
    )
    tray.visual(
        Box((tray_width * 0.34, wall, 0.020)),
        origin=Origin(xyz=(-tray_width * 0.16, 0.036, 0.016)),
        material=tray_color,
        name="short_divider",
    )
    tray.visual(
        Box((0.130, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, tray_depth / 2.0 + 0.002, 0.034)),
        material=plastic_accent,
        name="tray_pull_lip",
    )
    tray.inertial = Inertial.from_geometry(
        Box((tray_width, tray_depth, tray_height + 0.010)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, tray_rest_z + 0.001)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.15,
            lower=0.0,
            upper=0.115,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch = object_model.get_part("latch")
    handle = object_model.get_part("handle")
    tray = object_model.get_part("tray")
    lid_hinge = object_model.get_articulation("body_to_lid")
    latch_pivot = object_model.get_articulation("body_to_latch")
    handle_pivot = object_model.get_articulation("lid_to_handle")
    tray_slide = object_model.get_articulation("body_to_tray")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            negative_elem="front_wall",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid front skirt sits tightly on the body",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="left_skirt",
            negative_elem="left_wall",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid left skirt sits tightly on the body",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="right_skirt",
            negative_elem="right_wall",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid right skirt sits tightly on the body",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="left_rear_skirt",
            negative_elem="rear_wall",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid left rear skirt sits tightly on the body",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="right_rear_skirt",
            negative_elem="rear_wall",
            max_gap=0.004,
            max_penetration=0.0,
            name="closed lid right rear skirt sits tightly on the body",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            name="closed lid covers the body opening",
        )
        ctx.expect_gap(
            lid,
            handle,
            axis="z",
            positive_elem="front_top_rail",
            negative_elem="handle_grip",
            min_gap=0.004,
            name="folded handle stays recessed below the lid crown",
        )
        ctx.expect_overlap(
            latch,
            body,
            axes="xy",
            elem_a="main_latch_plate",
            elem_b="latch_keeper",
            min_overlap=0.004,
            name="closed latch stays centered over the keeper",
        )
        ctx.expect_within(
            tray,
            body,
            axes="xy",
            inner_elem="tray_floor",
            margin=0.0,
            name="tray remains nested inside the tub at rest",
        )

    with ctx.pose({lid_hinge: math.radians(95.0)}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.08,
            positive_elem="front_skirt",
            negative_elem="front_wall",
            name="opened lid lifts its front edge well above the box front",
        )

    with ctx.pose({latch_pivot: 0.0}):
        latch_tab_rest = aabb_center(ctx.part_element_world_aabb(latch, elem="finger_tab"))
    with ctx.pose({latch_pivot: math.radians(70.0)}):
        latch_tab_open = aabb_center(ctx.part_element_world_aabb(latch, elem="finger_tab"))
    ctx.check(
        "latch finger tab lifts as the latch opens",
        latch_tab_rest is not None
        and latch_tab_open is not None
        and latch_tab_open[2] > latch_tab_rest[2] + 0.015,
        details=f"rest={latch_tab_rest}, open={latch_tab_open}",
    )

    with ctx.pose({handle_pivot: 0.0}):
        handle_grip_rest = aabb_center(
            ctx.part_element_world_aabb(handle, elem="handle_grip")
        )
    with ctx.pose({handle_pivot: math.radians(80.0)}):
        handle_grip_open = aabb_center(
            ctx.part_element_world_aabb(handle, elem="handle_grip")
        )
    ctx.check(
        "handle grip lifts clearly above its recessed rest pose",
        handle_grip_rest is not None
        and handle_grip_open is not None
        and handle_grip_open[2] > handle_grip_rest[2] + 0.020,
        details=f"rest={handle_grip_rest}, open={handle_grip_open}",
    )

    tray_rest_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.115}):
        tray_open_pos = ctx.part_world_position(tray)
        ctx.expect_within(
            tray,
            body,
            axes="xy",
            inner_elem="tray_floor",
            margin=0.0,
            name="tray stays centered in the tub when raised",
        )
        ctx.expect_gap(
            tray,
            body,
            axis="z",
            positive_elem="tray_pull_lip",
            negative_elem="front_wall",
            min_gap=0.05,
            name="raised tray pull lip stands above the box opening",
        )
    ctx.check(
        "tray slides upward out of the box",
        tray_rest_pos is not None
        and tray_open_pos is not None
        and tray_open_pos[2] > tray_rest_pos[2] + 0.10,
        details=f"rest={tray_rest_pos}, open={tray_open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
