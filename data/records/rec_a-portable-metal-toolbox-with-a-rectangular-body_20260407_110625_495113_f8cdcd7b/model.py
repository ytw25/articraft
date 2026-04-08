from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_metal_toolbox")

    steel = model.material("steel", rgba=(0.62, 0.66, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.28, 0.31, 1.0))
    grip_metal = model.material("grip_metal", rgba=(0.18, 0.19, 0.20, 1.0))

    body_w = 0.56
    body_d = 0.24
    body_h = 0.22
    sheet_t = 0.0025
    eps = 0.0008

    lid_sheet_t = 0.0022
    lid_skirt_t = 0.0022
    lid_gap = 0.0
    lid_side_clear = 0.0012
    hinge_backset = 0.004
    lid_front_clear = 0.0012
    lid_skirt_drop = 0.035
    lid_outer_w = body_w + 2.0 * (lid_side_clear + lid_skirt_t)
    lid_depth = body_d + hinge_backset + lid_front_clear + lid_skirt_t

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, sheet_t)),
        origin=Origin(xyz=(0.0, 0.0, sheet_t / 2.0)),
        material=steel,
        name="body_bottom",
    )
    body.visual(
        Box((sheet_t, body_d, body_h + eps)),
        origin=Origin(
            xyz=(-body_w / 2.0 + sheet_t / 2.0, 0.0, body_h / 2.0 - eps / 2.0)
        ),
        material=steel,
        name="body_left_wall",
    )
    body.visual(
        Box((sheet_t, body_d, body_h + eps)),
        origin=Origin(
            xyz=(body_w / 2.0 - sheet_t / 2.0, 0.0, body_h / 2.0 - eps / 2.0)
        ),
        material=steel,
        name="body_right_wall",
    )
    body.visual(
        Box((body_w - 2.0 * sheet_t + 2.0 * eps, sheet_t, body_h + eps)),
        origin=Origin(
            xyz=(0.0, -body_d / 2.0 + sheet_t / 2.0, body_h / 2.0 - eps / 2.0)
        ),
        material=steel,
        name="body_front_wall",
    )
    body.visual(
        Box((body_w - 2.0 * sheet_t + 2.0 * eps, sheet_t, body_h + eps)),
        origin=Origin(
            xyz=(0.0, body_d / 2.0 - sheet_t / 2.0, body_h / 2.0 - eps / 2.0)
        ),
        material=steel,
        name="body_rear_wall",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_outer_w, lid_depth, lid_sheet_t)),
        origin=Origin(xyz=(0.0, -lid_depth / 2.0, 0.0)),
        material=steel,
        name="lid_panel",
    )
    lid.visual(
        Box((lid_skirt_t, lid_depth + eps, lid_skirt_drop + lid_sheet_t)),
        origin=Origin(
            xyz=(
                -lid_outer_w / 2.0 + lid_skirt_t / 2.0,
                -lid_depth / 2.0,
                -lid_skirt_drop / 2.0,
            )
        ),
        material=steel,
        name="lid_left_skirt",
    )
    lid.visual(
        Box((lid_skirt_t, lid_depth + eps, lid_skirt_drop + lid_sheet_t)),
        origin=Origin(
            xyz=(
                lid_outer_w / 2.0 - lid_skirt_t / 2.0,
                -lid_depth / 2.0,
                -lid_skirt_drop / 2.0,
            )
        ),
        material=steel,
        name="lid_right_skirt",
    )
    lid.visual(
        Box((lid_outer_w - 2.0 * lid_skirt_t + 2.0 * eps, lid_skirt_t, lid_skirt_drop + lid_sheet_t)),
        origin=Origin(
            xyz=(
                0.0,
                -lid_depth + lid_skirt_t / 2.0,
                -lid_skirt_drop / 2.0,
            )
        ),
        material=steel,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((0.11, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.095, 0.006)),
        material=dark_steel,
        name="lid_handle_pad",
    )
    lid.visual(
        Box((0.020, 0.018, 0.012)),
        origin=Origin(xyz=(-0.115, -0.155, 0.005)),
        material=dark_steel,
        name="lid_left_handle_bracket",
    )
    lid.visual(
        Box((0.020, 0.018, 0.012)),
        origin=Origin(xyz=(0.115, -0.155, 0.005)),
        material=dark_steel,
        name="lid_right_handle_bracket",
    )
    lid.visual(
        Box((0.040, 0.004, 0.018)),
        origin=Origin(xyz=(-0.205, -lid_depth - 0.002, -0.014)),
        material=dark_steel,
        name="lid_left_catch",
    )
    lid.visual(
        Box((0.040, 0.004, 0.018)),
        origin=Origin(xyz=(0.205, -lid_depth - 0.002, -0.014)),
        material=dark_steel,
        name="lid_right_catch",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(
            xyz=(
                0.0,
                body_d / 2.0 + hinge_backset,
                body_h + lid_gap + lid_sheet_t / 2.0,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=0.0,
            upper=2.15,
        ),
    )

    latch_width = 0.030
    latch_thickness = 0.003
    latch_height = 0.082
    latch_pivot_z = 0.120
    latch_x = 0.205

    left_latch = model.part("left_latch")
    left_latch.visual(
        Box((0.036, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=dark_steel,
        name="left_latch_pivot",
    )
    left_latch.visual(
        Box((latch_width, latch_thickness, latch_height)),
        origin=Origin(xyz=(0.0, -latch_thickness / 2.0, 0.034)),
        material=dark_steel,
        name="left_latch_body",
    )
    left_latch.visual(
        Box((0.020, latch_thickness, 0.018)),
        origin=Origin(xyz=(0.0, -latch_thickness / 2.0, 0.072)),
        material=dark_steel,
        name="left_latch_head",
    )
    left_latch.visual(
        Box((0.024, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.004, 0.010)),
        material=grip_metal,
        name="left_latch_pull",
    )

    right_latch = model.part("right_latch")
    right_latch.visual(
        Box((0.036, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=dark_steel,
        name="right_latch_pivot",
    )
    right_latch.visual(
        Box((latch_width, latch_thickness, latch_height)),
        origin=Origin(xyz=(0.0, -latch_thickness / 2.0, 0.034)),
        material=dark_steel,
        name="right_latch_body",
    )
    right_latch.visual(
        Box((0.020, latch_thickness, 0.018)),
        origin=Origin(xyz=(0.0, -latch_thickness / 2.0, 0.072)),
        material=dark_steel,
        name="right_latch_head",
    )
    right_latch.visual(
        Box((0.024, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.004, 0.010)),
        material=grip_metal,
        name="right_latch_pull",
    )

    model.articulation(
        "body_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_latch,
        origin=Origin(xyz=(-latch_x, -body_d / 2.0, latch_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_latch,
        origin=Origin(xyz=(latch_x, -body_d / 2.0, latch_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(-0.105, 0.009, 0.005)),
        material=dark_steel,
        name="handle_left_foot",
    )
    handle.visual(
        Box((0.022, 0.018, 0.010)),
        origin=Origin(xyz=(0.105, 0.009, 0.005)),
        material=dark_steel,
        name="handle_right_foot",
    )
    handle.visual(
        Box((0.012, 0.060, 0.010)),
        origin=Origin(xyz=(-0.105, 0.030, 0.005)),
        material=dark_steel,
        name="handle_left_leg",
    )
    handle.visual(
        Box((0.012, 0.060, 0.010)),
        origin=Origin(xyz=(0.105, 0.030, 0.005)),
        material=dark_steel,
        name="handle_right_leg",
    )
    handle.visual(
        Box((0.198, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, 0.060, 0.005)),
        material=grip_metal,
        name="handle_grip",
    )

    model.articulation(
        "lid_to_handle",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, -0.155, 0.011)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=3.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")
    left_latch_hinge = object_model.get_articulation("body_to_left_latch")
    right_latch_hinge = object_model.get_articulation("body_to_right_latch")
    handle = object_model.get_part("handle")
    handle_hinge = object_model.get_articulation("lid_to_handle")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        max_gap=0.003,
        max_penetration=0.0,
        name="closed lid panel sits just above the box rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        min_overlap=0.22,
        name="closed lid covers the toolbox opening footprint",
    )

    closed_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    with ctx.pose({lid_hinge: 1.2}):
        opened_panel = ctx.part_element_world_aabb(lid, elem="lid_panel")
    lid_opens = (
        closed_panel is not None
        and opened_panel is not None
        and opened_panel[1][2] > closed_panel[1][2] + 0.08
        and opened_panel[0][1] > closed_panel[0][1] + 0.03
    )
    ctx.check(
        "lid rotates upward about the rear hinge",
        lid_opens,
        details=f"closed={closed_panel}, opened={opened_panel}",
    )

    left_closed = ctx.part_element_world_aabb(left_latch, elem="left_latch_body")
    right_closed = ctx.part_element_world_aabb(right_latch, elem="right_latch_body")
    with ctx.pose({left_latch_hinge: 1.0, right_latch_hinge: 1.0}):
        left_open = ctx.part_element_world_aabb(left_latch, elem="left_latch_body")
        right_open = ctx.part_element_world_aabb(right_latch, elem="right_latch_body")
    latches_open = (
        left_closed is not None
        and right_closed is not None
        and left_open is not None
        and right_open is not None
        and left_open[0][1] < left_closed[0][1] - 0.025
        and right_open[0][1] < right_closed[0][1] - 0.025
    )
    ctx.check(
        "front latches swing outward from the box front",
        latches_open,
        details=(
            f"left_closed={left_closed}, left_open={left_open}, "
            f"right_closed={right_closed}, right_open={right_open}"
        ),
    )

    handle_closed = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({handle_hinge: 1.15}):
        handle_raised = ctx.part_element_world_aabb(handle, elem="handle_grip")
    handle_lifts = (
        handle_closed is not None
        and handle_raised is not None
        and handle_raised[1][2] > handle_closed[1][2] + 0.045
    )
    ctx.check(
        "carry handle folds up from the lid crown",
        handle_lifts,
        details=f"closed={handle_closed}, raised={handle_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
