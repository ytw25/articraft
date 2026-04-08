from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    model = ArticulatedObject(name="contractor_toolbox")

    body_plastic = model.material("body_plastic", rgba=(0.15, 0.15, 0.16, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.11, 0.11, 0.12, 1.0))
    latch_yellow = model.material("latch_yellow", rgba=(0.88, 0.72, 0.10, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hardware = model.material("hardware", rgba=(0.55, 0.57, 0.60, 1.0))
    drawer_trim = model.material("drawer_trim", rgba=(0.22, 0.22, 0.24, 1.0))

    body_w = 0.68
    body_d = 0.36
    lower_h = 0.11
    upper_h = 0.24
    body_h = lower_h + upper_h
    wall = 0.014
    base_t = 0.012
    deck_t = 0.012

    lid_w = 0.70
    lid_d = 0.372
    lid_t = 0.012
    lid_skirt_t = 0.016
    lid_skirt_h = 0.026

    drawer_panel_w = 0.64
    drawer_panel_t = 0.018
    drawer_panel_h = 0.100
    drawer_body_w = 0.618
    drawer_depth = 0.270
    drawer_h = 0.060
    drawer_wall = 0.010
    drawer_bottom = 0.008
    drawer_travel = 0.18

    latch_x = 0.22
    latch_pivot_y = body_d * 0.5 + 0.008
    latch_pivot_z = body_h - 0.108

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )
    body.visual(
        Box((body_w - 2.0 * wall, body_d, base_t)),
        origin=Origin(xyz=(0.0, 0.0, base_t * 0.5)),
        material=body_plastic,
        name="base_plate",
    )
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(body_w * 0.5 - wall * 0.5, 0.0, body_h * 0.5)),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((wall, body_d, body_h)),
        origin=Origin(xyz=(-body_w * 0.5 + wall * 0.5, 0.0, body_h * 0.5)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall, wall, body_h)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + wall * 0.5, body_h * 0.5)),
        material=body_plastic,
        name="rear_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall, body_d - 2.0 * wall, deck_t)),
        origin=Origin(xyz=(0.0, 0.0, lower_h - deck_t * 0.5)),
        material=body_plastic,
        name="drawer_ceiling",
    )
    body.visual(
        Box((body_w - 2.0 * wall, wall, upper_h)),
        origin=Origin(
            xyz=(0.0, body_d * 0.5 - wall * 0.5, lower_h + upper_h * 0.5),
        ),
        material=body_plastic,
        name="front_upper_wall",
    )
    body.visual(
        Box((body_w - 2.0 * wall, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -body_d * 0.5 + 0.020, body_h - 0.010)),
        material=body_plastic,
        name="rear_hinge_pad",
    )
    body.visual(
        Box((0.090, 0.018, 0.022)),
        origin=Origin(xyz=(latch_x, body_d * 0.5 - 0.020, latch_pivot_z - 0.012)),
        material=body_plastic,
        name="right_latch_mount",
    )
    body.visual(
        Box((0.090, 0.018, 0.022)),
        origin=Origin(xyz=(-latch_x, body_d * 0.5 - 0.020, latch_pivot_z - 0.012)),
        material=body_plastic,
        name="left_latch_mount",
    )
    body.visual(
        Box((0.014, 0.240, 0.010)),
        origin=Origin(
            xyz=(
                body_w * 0.5 - wall - 0.007,
                body_d * 0.5 - wall - 0.120,
                0.093,
            )
        ),
        material=drawer_trim,
        name="right_runner",
    )
    body.visual(
        Box((0.014, 0.240, 0.010)),
        origin=Origin(
            xyz=(
                -body_w * 0.5 + wall + 0.007,
                body_d * 0.5 - wall - 0.120,
                0.093,
            )
        ),
        material=drawer_trim,
        name="left_runner",
    )
    body.visual(
        Box((0.080, 0.050, 0.012)),
        origin=Origin(xyz=(0.24, 0.0, 0.006)),
        material=body_plastic,
        name="right_foot",
    )
    body.visual(
        Box((0.080, 0.050, 0.012)),
        origin=Origin(xyz=(-0.24, 0.0, 0.006)),
        material=body_plastic,
        name="left_foot",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(
            xyz=(-0.24, -body_d * 0.5 + 0.010, body_h - 0.004),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=hardware,
        name="left_body_hinge",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(
            xyz=(0.24, -body_d * 0.5 + 0.010, body_h - 0.004),
            rpy=(0.0, pi * 0.5, 0.0),
        ),
        material=hardware,
        name="right_body_hinge",
    )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, 0.040)),
        mass=1.5,
        origin=Origin(xyz=(0.0, lid_d * 0.5, 0.0)),
    )
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, lid_d * 0.5 - 0.004, lid_t * 0.5)),
        material=lid_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.010, lid_d - 0.018, lid_skirt_h)),
        origin=Origin(
            xyz=(
                0.345,
                lid_d * 0.5 - 0.004,
                -lid_skirt_h * 0.5,
            )
        ),
        material=lid_plastic,
        name="right_lid_skirt",
    )
    lid.visual(
        Box((0.010, lid_d - 0.018, lid_skirt_h)),
        origin=Origin(
            xyz=(
                -0.345,
                lid_d * 0.5 - 0.004,
                -lid_skirt_h * 0.5,
            )
        ),
        material=lid_plastic,
        name="left_lid_skirt",
    )
    lid.visual(
        Box((lid_w - 2.0 * lid_skirt_t, 0.018, lid_skirt_h)),
        origin=Origin(
            xyz=(0.0, lid_d - 0.013, -lid_skirt_h * 0.5),
        ),
        material=lid_plastic,
        name="front_lid_skirt",
    )
    lid.visual(
        Box((0.066, 0.024, 0.018)),
        origin=Origin(xyz=(latch_x, lid_d - 0.004, -0.016)),
        material=hardware,
        name="right_keeper",
    )
    lid.visual(
        Box((0.066, 0.024, 0.018)),
        origin=Origin(xyz=(-latch_x, lid_d - 0.004, -0.016)),
        material=hardware,
        name="left_keeper",
    )
    lid.visual(
        Box((0.040, 0.020, 0.006)),
        origin=Origin(xyz=(0.270, 0.140, 0.015)),
        material=lid_plastic,
        name="right_handle_boss",
    )
    lid.visual(
        Box((0.040, 0.020, 0.006)),
        origin=Origin(xyz=(-0.270, 0.140, 0.015)),
        material=lid_plastic,
        name="left_handle_boss",
    )
    lid.visual(
        Box((0.46, 0.06, 0.010)),
        origin=Origin(xyz=(0.0, 0.185, 0.017)),
        material=lid_plastic,
        name="center_rib",
    )
    drawer = model.part("drawer")
    drawer.inertial = Inertial.from_geometry(
        Box((drawer_panel_w, drawer_depth + drawer_panel_t, drawer_panel_h)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -0.14, 0.0)),
    )
    drawer.visual(
        Box((drawer_panel_w, drawer_panel_t, drawer_panel_h)),
        origin=Origin(xyz=(0.0, -drawer_panel_t * 0.5, 0.0)),
        material=lid_plastic,
        name="drawer_front",
    )
    drawer.visual(
        Box((drawer_body_w - 2.0 * drawer_wall, drawer_depth, drawer_bottom)),
        origin=Origin(
            xyz=(
                0.0,
                -drawer_panel_t - drawer_depth * 0.5,
                -drawer_h * 0.5 + drawer_bottom * 0.5,
            )
        ),
        material=body_plastic,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((drawer_wall, drawer_depth, drawer_h)),
        origin=Origin(
            xyz=(
                drawer_body_w * 0.5 - drawer_wall * 0.5,
                -drawer_panel_t - drawer_depth * 0.5,
                -0.003,
            )
        ),
        material=body_plastic,
        name="drawer_right_wall",
    )
    drawer.visual(
        Box((drawer_wall, drawer_depth, drawer_h)),
        origin=Origin(
            xyz=(
                -drawer_body_w * 0.5 + drawer_wall * 0.5,
                -drawer_panel_t - drawer_depth * 0.5,
                -0.003,
            )
        ),
        material=body_plastic,
        name="drawer_left_wall",
    )
    drawer.visual(
        Box((drawer_body_w - 2.0 * drawer_wall, drawer_wall, drawer_h)),
        origin=Origin(
            xyz=(
                0.0,
                -drawer_panel_t - drawer_depth + drawer_wall * 0.5,
                -0.003,
            )
        ),
        material=body_plastic,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.20, 0.024, 0.026)),
        origin=Origin(xyz=(0.0, 0.012, -0.008)),
        material=drawer_trim,
        name="drawer_pull",
    )
    right_latch = model.part("right_latch")
    right_latch.inertial = Inertial.from_geometry(
        Box((0.070, 0.036, 0.092)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.008, 0.040)),
    )
    right_latch.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=latch_yellow,
        name="latch_barrel",
    )
    right_latch.visual(
        Box((0.065, 0.012, 0.074)),
        origin=Origin(xyz=(0.0, 0.004, 0.037)),
        material=latch_yellow,
        name="latch_body",
    )
    right_latch.visual(
        Box((0.050, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, 0.014, 0.012)),
        material=latch_yellow,
        name="release_tab",
    )
    right_latch.visual(
        Box((0.040, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, 0.010, 0.072)),
        material=latch_yellow,
        name="latch_hook",
    )

    left_latch = model.part("left_latch")
    left_latch.inertial = Inertial.from_geometry(
        Box((0.070, 0.036, 0.092)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.008, 0.040)),
    )
    left_latch.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=latch_yellow,
        name="latch_barrel",
    )
    left_latch.visual(
        Box((0.065, 0.012, 0.074)),
        origin=Origin(xyz=(0.0, 0.004, 0.037)),
        material=latch_yellow,
        name="latch_body",
    )
    left_latch.visual(
        Box((0.050, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, 0.014, 0.012)),
        material=latch_yellow,
        name="release_tab",
    )
    left_latch.visual(
        Box((0.040, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, 0.010, 0.072)),
        material=latch_yellow,
        name="latch_hook",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.58, 0.14, 0.06)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.060, 0.020)),
    )
    handle.visual(
        Box((0.034, 0.016, 0.008)),
        origin=Origin(xyz=(-0.270, 0.0, 0.004)),
        material=handle_black,
        name="left_pivot_pad",
    )
    handle.visual(
        Box((0.034, 0.016, 0.008)),
        origin=Origin(xyz=(0.270, 0.0, 0.004)),
        material=handle_black,
        name="right_pivot_pad",
    )
    handle.visual(
        Box((0.026, 0.090, 0.014)),
        origin=Origin(xyz=(-0.270, 0.045, 0.014)),
        material=handle_black,
        name="left_handle_arm",
    )
    handle.visual(
        Box((0.026, 0.090, 0.014)),
        origin=Origin(xyz=(0.270, 0.045, 0.014)),
        material=handle_black,
        name="right_handle_arm",
    )
    handle.visual(
        Box((0.520, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.090, 0.028)),
        material=handle_black,
        name="handle_bridge",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.220),
        origin=Origin(xyz=(0.0, 0.090, 0.028), rpy=(0.0, pi * 0.5, 0.0)),
        material=handle_black,
        name="grip_bar",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -body_d * 0.5 + 0.010, body_h)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, body_d * 0.5, 0.058)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.35,
            lower=0.0,
            upper=drawer_travel,
        ),
    )
    model.articulation(
        "body_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_latch,
        origin=Origin(xyz=(latch_x, latch_pivot_y, latch_pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_latch,
        origin=Origin(xyz=(-latch_x, latch_pivot_y, latch_pivot_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, 0.140, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")
    handle = object_model.get_part("handle")

    lid_joint = object_model.get_articulation("body_to_lid")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    left_latch_joint = object_model.get_articulation("body_to_left_latch")
    right_latch_joint = object_model.get_articulation("body_to_right_latch")
    handle_joint = object_model.get_articulation("lid_to_handle")

    def elem_center_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    def elem_center_y(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    def elem_max_y(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return aabb[1][1]

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="front_upper_wall",
        max_gap=0.004,
        max_penetration=0.001,
        name="lid sits down on the upper tub rim when closed",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.30,
        name="lid covers the main toolbox opening footprint",
    )
    ctx.expect_gap(
        lid,
        left_latch,
        axis="z",
        positive_elem="left_keeper",
        negative_elem="latch_hook",
        max_gap=0.010,
        max_penetration=0.003,
        name="left latch hook reaches the lid keeper",
    )
    ctx.expect_gap(
        lid,
        right_latch,
        axis="z",
        positive_elem="right_keeper",
        negative_elem="latch_hook",
        max_gap=0.010,
        max_penetration=0.003,
        name="right latch hook reaches the lid keeper",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        margin=0.020,
        name="drawer stays centered within the lower drawer bay",
    )

    closed_drawer_front = elem_max_y(drawer, "drawer_front")
    closed_lid_front_z = elem_center_z(lid, "front_lid_skirt")
    closed_handle_z = elem_center_z(handle, "grip_bar")
    closed_left_latch_y = elem_center_y(left_latch, "latch_hook")
    closed_right_latch_y = elem_center_y(right_latch, "latch_hook")

    with ctx.pose({drawer_joint: drawer_joint.motion_limits.upper}):
        ctx.expect_within(
            drawer,
            body,
            axes="xz",
            margin=0.020,
            name="extended drawer remains on the runner line",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            min_overlap=0.08,
            name="extended drawer retains meaningful insertion depth",
        )
        extended_drawer_front = elem_max_y(drawer, "drawer_front")
        ctx.check(
            "drawer pulls out forward",
            closed_drawer_front is not None
            and extended_drawer_front is not None
            and extended_drawer_front > closed_drawer_front + 0.12,
            details=f"closed={closed_drawer_front}, extended={extended_drawer_front}",
        )

    with ctx.pose({lid_joint: 1.35}):
        open_lid_front_z = elem_center_z(lid, "front_lid_skirt")
        ctx.check(
            "lid opens upward on the rear hinge",
            closed_lid_front_z is not None
            and open_lid_front_z is not None
            and open_lid_front_z > closed_lid_front_z + 0.10,
            details=f"closed={closed_lid_front_z}, open={open_lid_front_z}",
        )

    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        raised_handle_z = elem_center_z(handle, "grip_bar")
        ctx.check(
            "carry handle flips up over the lid",
            closed_handle_z is not None
            and raised_handle_z is not None
            and raised_handle_z > closed_handle_z + 0.05,
            details=f"closed={closed_handle_z}, raised={raised_handle_z}",
        )

    with ctx.pose({left_latch_joint: left_latch_joint.motion_limits.upper}):
        open_left_latch_y = elem_center_y(left_latch, "latch_hook")
        ctx.check(
            "left latch swings forward to release",
            closed_left_latch_y is not None
            and open_left_latch_y is not None
            and open_left_latch_y > closed_left_latch_y + 0.04,
            details=f"closed={closed_left_latch_y}, open={open_left_latch_y}",
        )

    with ctx.pose({right_latch_joint: right_latch_joint.motion_limits.upper}):
        open_right_latch_y = elem_center_y(right_latch, "latch_hook")
        ctx.check(
            "right latch swings forward to release",
            open_right_latch_y is not None
            and closed_right_latch_y is not None
            and open_right_latch_y > closed_right_latch_y + 0.04,
            details=f"closed={closed_right_latch_y}, open={open_right_latch_y}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
