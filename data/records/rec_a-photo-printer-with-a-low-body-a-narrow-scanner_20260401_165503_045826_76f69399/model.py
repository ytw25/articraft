from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="photo_printer")

    body_white = model.material("body_white", rgba=(0.90, 0.91, 0.92, 1.0))
    lid_white = model.material("lid_white", rgba=(0.95, 0.95, 0.96, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.10, 0.16, 0.18, 0.75))
    mid_grey = model.material("mid_grey", rgba=(0.55, 0.57, 0.60, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.46, 0.34, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=body_white,
        name="lower_shell",
    )
    body.visual(
        Box((0.35, 0.23, 0.04)),
        origin=Origin(xyz=(0.0, -0.045, 0.09)),
        material=body_white,
        name="scanner_plinth",
    )
    body.visual(
        Box((0.40, 0.098, 0.034)),
        origin=Origin(xyz=(0.0, 0.129, 0.087)),
        material=body_white,
        name="front_brow",
    )
    body.visual(
        Box((0.31, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.164, 0.09)),
        material=charcoal,
        name="paper_slot_trim",
    )
    body.visual(
        Box((0.32, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, 0.1735, 0.101)),
        material=mid_grey,
        name="shelf_seat",
    )
    body.visual(
        Box((0.09, 0.045, 0.003)),
        origin=Origin(xyz=(0.125, 0.105, 0.0995)),
        material=dark_glass,
        name="status_panel",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.46, 0.34, 0.12)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    scanner_lid = model.part("scanner_lid")
    scanner_lid.visual(
        Box((0.31, 0.22, 0.018)),
        origin=Origin(xyz=(0.0, 0.11, 0.009)),
        material=lid_white,
        name="lid_panel",
    )
    scanner_lid.visual(
        Box((0.29, 0.20, 0.004)),
        origin=Origin(xyz=(0.0, 0.11, 0.018)),
        material=mid_grey,
        name="lid_bezel",
    )
    scanner_lid.visual(
        Box((0.22, 0.05, 0.002)),
        origin=Origin(xyz=(0.0, 0.105, 0.021)),
        material=dark_glass,
        name="scanner_glass",
    )
    scanner_lid.visual(
        Box((0.12, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.214, 0.008)),
        material=lid_white,
        name="finger_lip",
    )
    scanner_lid.inertial = Inertial.from_geometry(
        Box((0.31, 0.22, 0.022)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.11, 0.011)),
    )

    output_shelf = model.part("output_shelf")
    output_shelf.visual(
        Box((0.34, 0.012, 0.095)),
        origin=Origin(xyz=(0.0, 0.006, 0.0475)),
        material=lid_white,
        name="tray_panel",
    )
    output_shelf.visual(
        Box((0.32, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, 0.09)),
        material=mid_grey,
        name="paper_stop",
    )
    output_shelf.visual(
        Box((0.10, 0.004, 0.006)),
        origin=Origin(xyz=(-0.113, -0.002, 0.046)),
        material=charcoal,
        name="left_rail",
    )
    output_shelf.visual(
        Box((0.10, 0.004, 0.006)),
        origin=Origin(xyz=(0.113, -0.002, 0.046)),
        material=charcoal,
        name="right_rail",
    )
    output_shelf.inertial = Inertial.from_geometry(
        Box((0.34, 0.02, 0.095)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.002, 0.0475)),
    )

    def add_side_guide(
        *,
        part_name: str,
        fence_name: str,
        skid_name: str,
        shoe_name: str,
        outer_x: float,
    ):
        guide = model.part(part_name)
        guide.visual(
            Box((0.024, 0.003, 0.012)),
            origin=Origin(xyz=(0.0, -0.0015, 0.052)),
            material=mid_grey,
            name=skid_name,
        )
        guide.visual(
            Box((0.004, 0.012, 0.05)),
            origin=Origin(xyz=(outer_x, -0.006, 0.056)),
            material=mid_grey,
            name=fence_name,
        )
        guide.visual(
            Box((0.018, 0.004, 0.006)),
            origin=Origin(xyz=(0.0, -0.006, 0.046)),
            material=charcoal,
            name=shoe_name,
        )
        guide.inertial = Inertial.from_geometry(
            Box((0.028, 0.012, 0.05)),
            mass=0.05,
            origin=Origin(xyz=(0.0, -0.006, 0.055)),
        )
        return guide

    left_guide = add_side_guide(
        part_name="left_guide",
        fence_name="left_fence",
        skid_name="left_skid",
        shoe_name="left_slider_shoe",
        outer_x=-0.01,
    )
    right_guide = add_side_guide(
        part_name="right_guide",
        fence_name="right_fence",
        skid_name="right_skid",
        shoe_name="right_slider_shoe",
        outer_x=0.01,
    )

    model.articulation(
        "body_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=scanner_lid,
        origin=Origin(xyz=(0.0, -0.155, 0.11)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    model.articulation(
        "body_to_output_shelf",
        ArticulationType.REVOLUTE,
        parent=body,
        child=output_shelf,
        origin=Origin(xyz=(0.0, 0.185, 0.012)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )

    model.articulation(
        "shelf_to_left_guide",
        ArticulationType.PRISMATIC,
        parent=output_shelf,
        child=left_guide,
        origin=Origin(xyz=(-0.141, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.08,
            lower=0.0,
            upper=0.05,
        ),
    )

    model.articulation(
        "shelf_to_right_guide",
        ArticulationType.PRISMATIC,
        parent=output_shelf,
        child=right_guide,
        origin=Origin(xyz=(0.141, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.08,
            lower=0.0,
            upper=0.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    scanner_lid = object_model.get_part("scanner_lid")
    output_shelf = object_model.get_part("output_shelf")
    left_guide = object_model.get_part("left_guide")
    right_guide = object_model.get_part("right_guide")

    lid_hinge = object_model.get_articulation("body_to_scanner_lid")
    shelf_hinge = object_model.get_articulation("body_to_output_shelf")
    left_slide = object_model.get_articulation("shelf_to_left_guide")
    right_slide = object_model.get_articulation("shelf_to_right_guide")

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

    ctx.expect_gap(
        scanner_lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="scanner_plinth",
        max_gap=0.001,
        max_penetration=0.0,
        name="scanner lid seats on rear plinth",
    )
    ctx.expect_overlap(
        scanner_lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="scanner_plinth",
        min_overlap=0.20,
        name="scanner lid covers the scanner plinth footprint",
    )
    ctx.expect_gap(
        output_shelf,
        body,
        axis="y",
        positive_elem="tray_panel",
        negative_elem="paper_slot_trim",
        min_gap=0.01,
        max_gap=0.02,
        name="output shelf nests just ahead of the paper slot",
    )
    ctx.expect_contact(
        output_shelf,
        body,
        elem_a="paper_stop",
        elem_b="shelf_seat",
        name="output shelf is seated on the front shelf seat when closed",
    )
    ctx.expect_contact(
        left_guide,
        output_shelf,
        elem_a="left_skid",
        elem_b="tray_panel",
        name="left guide is supported by the tray panel",
    )
    ctx.expect_contact(
        right_guide,
        output_shelf,
        elem_a="right_skid",
        elem_b="tray_panel",
        name="right guide is supported by the tray panel",
    )
    ctx.expect_contact(
        left_guide,
        output_shelf,
        elem_a="left_slider_shoe",
        elem_b="left_rail",
        name="left guide rides on the left rail",
    )
    ctx.expect_contact(
        right_guide,
        output_shelf,
        elem_a="right_slider_shoe",
        elem_b="right_rail",
        name="right guide rides on the right rail",
    )

    lid_upper = lid_hinge.motion_limits.upper or 0.0
    shelf_upper = shelf_hinge.motion_limits.upper or 0.0
    left_upper = left_slide.motion_limits.upper or 0.0
    right_upper = right_slide.motion_limits.upper or 0.0

    closed_lid_aabb = ctx.part_world_aabb(scanner_lid)
    with ctx.pose({lid_hinge: lid_upper}):
        open_lid_aabb = ctx.part_world_aabb(scanner_lid)
    ctx.check(
        "scanner lid lifts upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_shelf_aabb = ctx.part_element_world_aabb(output_shelf, elem="tray_panel")
    with ctx.pose({shelf_hinge: shelf_upper}):
        open_shelf_aabb = ctx.part_element_world_aabb(output_shelf, elem="tray_panel")
    ctx.check(
        "output shelf folds downward and forward",
        closed_shelf_aabb is not None
        and open_shelf_aabb is not None
        and open_shelf_aabb[1][1] > closed_shelf_aabb[1][1] + 0.05
        and open_shelf_aabb[1][2] < closed_shelf_aabb[1][2] - 0.05,
        details=f"closed={closed_shelf_aabb}, open={open_shelf_aabb}",
    )

    left_rest_pos = ctx.part_world_position(left_guide)
    right_rest_pos = ctx.part_world_position(right_guide)
    with ctx.pose({left_slide: left_upper, right_slide: right_upper}):
        left_extended_pos = ctx.part_world_position(left_guide)
        right_extended_pos = ctx.part_world_position(right_guide)
    ctx.check(
        "paper guides slide inward from the shelf edges",
        left_rest_pos is not None
        and right_rest_pos is not None
        and left_extended_pos is not None
        and right_extended_pos is not None
        and left_extended_pos[0] > left_rest_pos[0] + 0.035
        and right_extended_pos[0] < right_rest_pos[0] - 0.035,
        details=(
            f"left_rest={left_rest_pos}, left_extended={left_extended_pos}, "
            f"right_rest={right_rest_pos}, right_extended={right_extended_pos}"
        ),
    )

    with ctx.pose({shelf_hinge: shelf_upper, left_slide: left_upper, right_slide: right_upper}):
        ctx.expect_contact(
            left_guide,
            output_shelf,
            elem_a="left_skid",
            elem_b="tray_panel",
            name="left guide stays supported when the tray is open",
        )
        ctx.expect_contact(
            right_guide,
            output_shelf,
            elem_a="right_skid",
            elem_b="tray_panel",
            name="right guide stays supported when the tray is open",
        )
        ctx.expect_contact(
            left_guide,
            output_shelf,
            elem_a="left_slider_shoe",
            elem_b="left_rail",
            name="left guide stays on the rail when extended",
        )
        ctx.expect_contact(
            right_guide,
            output_shelf,
            elem_a="right_slider_shoe",
            elem_b="right_rail",
            name="right guide stays on the rail when extended",
        )
        ctx.expect_overlap(
            left_guide,
            output_shelf,
            axes="x",
            elem_a="left_slider_shoe",
            elem_b="left_rail",
            min_overlap=0.015,
            name="left guide retains overlap with its rail at full travel",
        )
        ctx.expect_overlap(
            right_guide,
            output_shelf,
            axes="x",
            elem_a="right_slider_shoe",
            elem_b="right_rail",
            min_overlap=0.015,
            name="right guide retains overlap with its rail at full travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
