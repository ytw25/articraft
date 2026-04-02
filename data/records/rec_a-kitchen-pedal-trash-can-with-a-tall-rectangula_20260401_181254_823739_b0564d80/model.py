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
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_pedal_trash_can")

    body_depth = 0.33
    body_width = 0.26
    body_height = 0.68
    body_corner_radius = 0.032
    wall_thickness = 0.012
    floor_thickness = 0.014

    lid_depth = 0.335
    lid_width = 0.246
    lid_thickness = 0.022
    lid_front_skirt_height = 0.050
    lid_side_skirt_height = 0.028
    lid_hinge_radius = 0.006

    pedal_pivot_height = 0.060
    pedal_mount_x = body_depth / 2.0 + 0.013
    pedal_rest_tilt = -0.22

    brushed_steel = model.material("brushed_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    graphite = model.material("graphite", rgba=(0.14, 0.15, 0.16, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.33, 0.34, 0.36, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    outer_profile = rounded_rect_profile(
        body_depth,
        body_width,
        body_corner_radius,
        corner_segments=10,
    )
    inner_profile = rounded_rect_profile(
        body_depth - 2.0 * wall_thickness,
        body_width - 2.0 * wall_thickness,
        body_corner_radius - wall_thickness * 0.65,
        corner_segments=10,
    )
    body_shell_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            [inner_profile],
            body_height,
            cap=False,
            center=False,
        ),
        "trash_can_body_shell",
    )

    body = model.part("body")
    body.visual(body_shell_mesh, material=brushed_steel, name="body_shell")
    body.visual(
        Box((body_depth - 0.010, body_width - 0.010, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness / 2.0)),
        material=brushed_steel,
        name="body_floor",
    )
    body.visual(
        Box((0.026, 0.032, 0.060)),
        origin=Origin(
            xyz=(
                body_depth / 2.0 - 0.008,
                body_width / 2.0 - 0.025,
                0.055,
            )
        ),
        material=dark_metal,
        name="pedal_bracket_left",
    )
    body.visual(
        Box((0.026, 0.032, 0.060)),
        origin=Origin(
            xyz=(
                body_depth / 2.0 - 0.008,
                -(body_width / 2.0 - 0.025),
                0.055,
            )
        ),
        material=dark_metal,
        name="pedal_bracket_right",
    )
    body.visual(
        Box((0.020, 0.040, 0.020)),
        origin=Origin(
            xyz=(
                -(body_depth / 2.0) - 0.002,
                body_width / 2.0 - 0.045,
                body_height - 0.011,
            )
        ),
        material=dark_metal,
        name="hinge_block_left",
    )
    body.visual(
        Box((0.020, 0.040, 0.020)),
        origin=Origin(
            xyz=(
                -(body_depth / 2.0) - 0.002,
                -(body_width / 2.0 - 0.045),
                body_height - 0.011,
            )
        ),
        material=dark_metal,
        name="hinge_block_right",
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_depth, lid_width, lid_thickness)),
        origin=Origin(
            xyz=(
                0.008 + lid_depth / 2.0,
                0.0,
                lid_thickness / 2.0,
            )
        ),
        material=graphite,
        name="lid_top",
    )
    lid.visual(
        Box((0.032, lid_width - 0.018, lid_front_skirt_height)),
        origin=Origin(
            xyz=(
                0.008 + lid_depth + 0.010,
                0.0,
                -0.013,
            )
        ),
        material=graphite,
        name="front_skirt",
    )
    lid.visual(
        Box((lid_depth - 0.020, 0.018, lid_side_skirt_height)),
        origin=Origin(
            xyz=(
                0.008 + (lid_depth - 0.020) / 2.0,
                lid_width / 2.0 - 0.009,
                0.008,
            )
        ),
        material=graphite,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_depth - 0.020, 0.018, lid_side_skirt_height)),
        origin=Origin(
            xyz=(
                0.008 + (lid_depth - 0.020) / 2.0,
                -(lid_width / 2.0 - 0.009),
                0.008,
            )
        ),
        material=graphite,
        name="right_skirt",
    )
    lid.visual(
        Cylinder(radius=lid_hinge_radius, length=0.140),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.022, 0.028, 0.006)),
        origin=Origin(
            xyz=(0.011, lid_width / 2.0 - 0.040, 0.003),
        ),
        material=dark_metal,
        name="hinge_leaf_left",
    )
    lid.visual(
        Box((0.022, 0.028, 0.006)),
        origin=Origin(
            xyz=(0.011, -(lid_width / 2.0 - 0.040), 0.003),
        ),
        material=dark_metal,
        name="hinge_leaf_right",
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.008, length=0.180),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_barrel",
    )
    pedal.visual(
        Box((0.024, 0.180, 0.018)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_hub",
    )
    pedal.visual(
        Box((0.064, 0.168, 0.012)),
        origin=Origin(
            xyz=(0.030, 0.0, -0.010),
            rpy=(0.0, pedal_rest_tilt, 0.0),
        ),
        material=graphite,
        name="lever_plate",
    )
    pedal.visual(
        Box((0.118, 0.220, 0.018)),
        origin=Origin(
            xyz=(0.082, 0.0, -0.015),
            rpy=(0.0, pedal_rest_tilt, 0.0),
        ),
        material=graphite,
        name="tread_plate",
    )
    pedal.visual(
        Box((0.078, 0.184, 0.007)),
        origin=Origin(
            xyz=(0.093, 0.0, -0.004),
            rpy=(0.0, pedal_rest_tilt, 0.0),
        ),
        material=rubber,
        name="tread_pad",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(
            xyz=(
                -(body_depth / 2.0) - 0.007,
                0.0,
                body_height + 0.008,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(pedal_mount_x, 0.0, pedal_pivot_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.5,
            lower=0.0,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")

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

    closed_lid_front = None
    open_lid_front = None
    closed_pedal_tread = None
    pressed_pedal_tread = None

    with ctx.pose({lid_hinge: 0.0, pedal_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            max_gap=0.012,
            positive_elem="lid_top",
            negative_elem="body_shell",
            name="closed lid sits just above the rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.20,
            elem_a="lid_top",
            elem_b="body_shell",
            name="lid covers the body opening",
        )
        ctx.expect_gap(
            pedal,
            body,
            axis="x",
            min_gap=0.020,
            max_gap=0.090,
            positive_elem="tread_plate",
            negative_elem="body_shell",
            name="pedal projects from the lower front face",
        )
        closed_lid_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
        closed_pedal_tread = ctx.part_element_world_aabb(pedal, elem="tread_plate")

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper, pedal_hinge: pedal_hinge.motion_limits.upper}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.090,
            positive_elem="front_skirt",
            negative_elem="body_shell",
            name="opened lid lifts clear of the body",
        )
        open_lid_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
        pressed_pedal_tread = ctx.part_element_world_aabb(pedal, elem="tread_plate")

    ctx.check(
        "lid front edge rises when opened",
        closed_lid_front is not None
        and open_lid_front is not None
        and open_lid_front[1][2] > closed_lid_front[1][2] + 0.12,
        details=f"closed={closed_lid_front}, open={open_lid_front}",
    )
    ctx.check(
        "pedal swings downward when pressed",
        closed_pedal_tread is not None
        and pressed_pedal_tread is not None
        and pressed_pedal_tread[0][2] < closed_pedal_tread[0][2] - 0.020,
        details=f"closed={closed_pedal_tread}, pressed={pressed_pedal_tread}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
