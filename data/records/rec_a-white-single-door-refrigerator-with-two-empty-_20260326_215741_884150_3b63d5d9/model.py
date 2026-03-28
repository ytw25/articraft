from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

def _add_drawer_shell(
    part,
    *,
    material,
    width: float,
    depth: float,
    height: float,
    wall: float,
    bottom: float,
    skid_width: float,
    skid_depth: float,
    skid_height: float,
    skid_x: float,
) -> None:
    half_height = height * 0.5
    half_depth = depth * 0.5
    half_width = width * 0.5

    part.visual(
        Box((width, depth, bottom)),
        origin=Origin(xyz=(0.0, 0.0, -half_height + bottom * 0.5)),
        material=material,
        name="bin_bottom",
    )
    part.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, half_depth - wall * 0.5, 0.0)),
        material=material,
        name="front_wall",
    )
    part.visual(
        Box((width, wall, height)),
        origin=Origin(xyz=(0.0, -half_depth + wall * 0.5, 0.0)),
        material=material,
        name="rear_wall",
    )
    part.visual(
        Box((wall, depth - 2.0 * wall, height)),
        origin=Origin(xyz=(-half_width + wall * 0.5, 0.0, 0.0)),
        material=material,
        name="left_wall",
    )
    part.visual(
        Box((wall, depth - 2.0 * wall, height)),
        origin=Origin(xyz=(half_width - wall * 0.5, 0.0, 0.0)),
        material=material,
        name="right_wall",
    )
    part.visual(
        Box((skid_width, skid_depth, skid_height)),
        origin=Origin(xyz=(-skid_x, 0.0, -half_height - skid_height * 0.5)),
        material=material,
        name="left_skid",
    )
    part.visual(
        Box((skid_width, skid_depth, skid_height)),
        origin=Origin(xyz=(skid_x, 0.0, -half_height - skid_height * 0.5)),
        material=material,
        name="right_skid",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_door_refrigerator", assets=ASSETS)

    white_enamel = model.material("white_enamel", rgba=(0.97, 0.97, 0.98, 1.0))
    liner_white = model.material("liner_white", rgba=(0.92, 0.93, 0.95, 1.0))
    drawer_clear = model.material("drawer_clear", rgba=(0.88, 0.94, 0.98, 0.48))
    rubber_gray = model.material("rubber_gray", rgba=(0.42, 0.43, 0.45, 1.0))

    body_width = 0.50
    body_depth = 0.53
    body_height = 0.84
    side_wall = 0.04
    top_wall = 0.05
    bottom_wall = 0.06
    back_wall = 0.03

    door_width = 0.486
    door_height = 0.816
    door_thickness = 0.048
    hinge_x = body_width * 0.5
    hinge_y = body_depth * 0.5 + door_thickness * 0.5

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((side_wall, body_depth, body_height)),
        origin=Origin(xyz=(-body_width * 0.5 + side_wall * 0.5, 0.0, body_height * 0.5)),
        material=white_enamel,
        name="left_wall",
    )
    cabinet.visual(
        Box((side_wall, body_depth, body_height)),
        origin=Origin(xyz=(body_width * 0.5 - side_wall * 0.5, 0.0, body_height * 0.5)),
        material=white_enamel,
        name="right_wall",
    )
    cabinet.visual(
        Box((body_width - 2.0 * side_wall, body_depth, bottom_wall)),
        origin=Origin(xyz=(0.0, 0.0, bottom_wall * 0.5)),
        material=white_enamel,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((body_width - 2.0 * side_wall, body_depth, top_wall)),
        origin=Origin(xyz=(0.0, 0.0, body_height - top_wall * 0.5)),
        material=white_enamel,
        name="top_panel",
    )
    cabinet.visual(
        Box((body_width - 2.0 * side_wall, back_wall, body_height - top_wall - bottom_wall)),
        origin=Origin(
            xyz=(
                0.0,
                -body_depth * 0.5 + back_wall * 0.5,
                bottom_wall + (body_height - top_wall - bottom_wall) * 0.5,
            )
        ),
        material=liner_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((body_width - 2.0 * side_wall, 0.44, 0.008)),
        origin=Origin(xyz=(0.0, -0.02, 0.45)),
        material=liner_white,
        name="mid_shelf",
    )

    rail_depth = 0.32
    rail_height = 0.008
    rail_width = 0.024
    rail_y = 0.02
    rail_x = 0.198
    lower_rail_z = 0.083
    upper_rail_z = 0.253
    for rail_name, rail_center_x, rail_center_z in (
        ("lower_left_rail", -rail_x, lower_rail_z),
        ("lower_right_rail", rail_x, lower_rail_z),
        ("upper_left_rail", -rail_x, upper_rail_z),
        ("upper_right_rail", rail_x, upper_rail_z),
    ):
        cabinet.visual(
            Box((rail_width, rail_depth, rail_height)),
            origin=Origin(xyz=(rail_center_x, rail_y, rail_center_z)),
            material=liner_white,
            name=rail_name,
        )

    door = model.part("door")
    handle_offset_x = -0.12
    handle_offset_z = 0.34
    recess_depth = 0.012
    pocket_width = 0.18
    pocket_height = 0.038
    door_back_thickness = door_thickness - recess_depth
    front_skin_y = door_thickness * 0.5 - recess_depth * 0.5
    door_min_x = -door_width
    door_max_x = 0.0
    door_min_z = -door_height * 0.5
    door_max_z = door_height * 0.5
    pocket_center_x = -door_width * 0.5 + handle_offset_x
    pocket_left = pocket_center_x - pocket_width * 0.5
    pocket_right = pocket_center_x + pocket_width * 0.5
    pocket_bottom = handle_offset_z - pocket_height * 0.5
    pocket_top = handle_offset_z + pocket_height * 0.5

    door.visual(
        Box((door_width, door_back_thickness, door_height)),
        origin=Origin(xyz=(-door_width * 0.5, -recess_depth * 0.5, 0.0)),
        material=white_enamel,
        name="door_core",
    )
    door.visual(
        Box((door_width, recess_depth, door_max_z - pocket_top)),
        origin=Origin(xyz=(-door_width * 0.5, front_skin_y, (door_max_z + pocket_top) * 0.5)),
        material=white_enamel,
        name="upper_face_strip",
    )
    door.visual(
        Box((door_width, recess_depth, pocket_bottom - door_min_z)),
        origin=Origin(xyz=(-door_width * 0.5, front_skin_y, (pocket_bottom + door_min_z) * 0.5)),
        material=white_enamel,
        name="lower_face_strip",
    )
    door.visual(
        Box((pocket_left - door_min_x, recess_depth, pocket_height)),
        origin=Origin(xyz=((pocket_left + door_min_x) * 0.5, front_skin_y, handle_offset_z)),
        material=white_enamel,
        name="left_face_strip",
    )
    door.visual(
        Box((door_max_x - pocket_right, recess_depth, pocket_height)),
        origin=Origin(xyz=((door_max_x + pocket_right) * 0.5, front_skin_y, handle_offset_z)),
        material=white_enamel,
        name="right_face_strip",
    )
    door.visual(
        Box((0.14, 0.006, 0.022)),
        origin=Origin(
            xyz=(
                pocket_center_x,
                door_thickness * 0.5 - recess_depth + 0.003,
                handle_offset_z,
            )
        ),
        material=rubber_gray,
        name="handle_insert",
    )
    drawer_width = 0.352
    drawer_depth = 0.28
    drawer_height = 0.13
    drawer_wall = 0.004
    drawer_bottom = 0.004
    skid_width = 0.024
    skid_depth = 0.24
    skid_height = 0.008
    skid_x = 0.174

    upper_drawer = model.part("upper_drawer")
    _add_drawer_shell(
        upper_drawer,
        material=drawer_clear,
        width=drawer_width,
        depth=drawer_depth,
        height=drawer_height,
        wall=drawer_wall,
        bottom=drawer_bottom,
        skid_width=skid_width,
        skid_depth=skid_depth,
        skid_height=skid_height,
        skid_x=skid_x,
    )

    lower_drawer = model.part("lower_drawer")
    _add_drawer_shell(
        lower_drawer,
        material=drawer_clear,
        width=drawer_width,
        depth=drawer_depth,
        height=drawer_height,
        wall=drawer_wall,
        bottom=drawer_bottom,
        skid_width=skid_width,
        skid_depth=skid_depth,
        skid_height=skid_height,
        skid_x=skid_x,
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, body_height * 0.5)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "upper_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_drawer,
        origin=Origin(xyz=(0.0, 0.10, 0.33)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.14,
        ),
    )
    model.articulation(
        "lower_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_drawer,
        origin=Origin(xyz=(0.0, 0.10, 0.16)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.14,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    upper_drawer = object_model.get_part("upper_drawer")
    lower_drawer = object_model.get_part("lower_drawer")
    door_hinge = object_model.get_articulation("door_hinge")
    upper_slide = object_model.get_articulation("upper_drawer_slide")
    lower_slide = object_model.get_articulation("lower_drawer_slide")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
        "door_hinge_axis_is_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"door_hinge axis was {door_hinge.axis}",
    )
    ctx.check(
        "drawer_axes_slide_forward",
        tuple(upper_slide.axis) == (0.0, 1.0, 0.0) and tuple(lower_slide.axis) == (0.0, 1.0, 0.0),
        details=f"upper={upper_slide.axis}, lower={lower_slide.axis}",
    )
    ctx.check(
        "door_hinge_origin_on_front_right_edge",
        abs(door_hinge.origin.xyz[0] - 0.25) < 0.002
        and abs(door_hinge.origin.xyz[1] - 0.289) < 0.002
        and abs(door_hinge.origin.xyz[2] - 0.42) < 0.002,
        details=f"door_hinge origin was {door_hinge.origin.xyz}",
    )

    ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.45, name="door_covers_front_opening")
    ctx.expect_gap(door, cabinet, axis="y", min_gap=-0.03, max_gap=0.03, name="door_seats_close_to_front_frame")

    ctx.expect_contact(
        upper_drawer,
        cabinet,
        elem_a="left_skid",
        elem_b="upper_left_rail",
        name="upper_drawer_left_support_contact",
    )
    ctx.expect_contact(
        upper_drawer,
        cabinet,
        elem_a="right_skid",
        elem_b="upper_right_rail",
        name="upper_drawer_right_support_contact",
    )
    ctx.expect_contact(
        lower_drawer,
        cabinet,
        elem_a="left_skid",
        elem_b="lower_left_rail",
        name="lower_drawer_left_support_contact",
    )
    ctx.expect_contact(
        lower_drawer,
        cabinet,
        elem_a="right_skid",
        elem_b="lower_right_rail",
        name="lower_drawer_right_support_contact",
    )
    ctx.expect_within(upper_drawer, cabinet, axes="xz", name="upper_drawer_within_cabinet")
    ctx.expect_within(lower_drawer, cabinet, axes="xz", name="lower_drawer_within_cabinet")
    ctx.expect_gap(upper_drawer, lower_drawer, axis="z", min_gap=0.02, name="drawer_vertical_separation")

    door_aabb = ctx.part_world_aabb(door)
    handle_aabb = ctx.part_element_world_aabb(door, elem="handle_insert")
    if door_aabb is None or handle_aabb is None:
        ctx.fail("door_handle_measurements_available", "Door or handle AABB was unavailable.")
    else:
        door_center = tuple((door_aabb[0][i] + door_aabb[1][i]) * 0.5 for i in range(3))
        handle_center = tuple((handle_aabb[0][i] + handle_aabb[1][i]) * 0.5 for i in range(3))
        ctx.check(
            "handle_is_in_upper_left_corner",
            handle_center[0] < door_center[0] - 0.08 and handle_center[2] > door_center[2] + 0.20,
            details=f"handle_center={handle_center}, door_center={door_center}",
        )
        ctx.check(
            "handle_is_recessed_into_door",
            handle_aabb[1][1] < door_aabb[1][1] - 0.004,
            details=f"handle_max_y={handle_aabb[1][1]}, door_max_y={door_aabb[1][1]}",
        )

    door_limits = door_hinge.motion_limits
    if door_limits is not None and door_limits.upper is not None and door_aabb is not None:
        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
            ctx.fail_if_isolated_parts(name="door_open_no_floating")
            ctx.expect_overlap(door, cabinet, axes="z", min_overlap=0.75, name="door_open_stays_hinge_aligned")
            opened_door_aabb = ctx.part_world_aabb(door)
            if opened_door_aabb is None:
                ctx.fail("door_open_measurement_available", "Opened door AABB was unavailable.")
            else:
                ctx.check(
                    "door_swings_outward",
                    opened_door_aabb[1][1] > door_aabb[1][1] + 0.18,
                    details=f"closed_max_y={door_aabb[1][1]}, open_max_y={opened_door_aabb[1][1]}",
                )

    upper_rest = ctx.part_world_position(upper_drawer)
    lower_rest = ctx.part_world_position(lower_drawer)
    upper_limits = upper_slide.motion_limits
    lower_limits = lower_slide.motion_limits

    if upper_limits is not None and upper_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper if door_limits is not None and door_limits.upper is not None else 0.0, upper_slide: upper_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="upper_drawer_open_no_overlap")
            ctx.fail_if_isolated_parts(name="upper_drawer_open_no_floating")
            ctx.expect_contact(
                upper_drawer,
                cabinet,
                elem_a="left_skid",
                elem_b="upper_left_rail",
                name="upper_drawer_left_support_contact_open",
            )
            ctx.expect_contact(
                upper_drawer,
                cabinet,
                elem_a="right_skid",
                elem_b="upper_right_rail",
                name="upper_drawer_right_support_contact_open",
            )
            upper_open = ctx.part_world_position(upper_drawer)
            if upper_rest is not None and upper_open is not None:
                ctx.check(
                    "upper_drawer_slides_forward",
                    upper_open[1] > upper_rest[1] + 0.12,
                    details=f"upper_rest={upper_rest}, upper_open={upper_open}",
                )

    if lower_limits is not None and lower_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.upper if door_limits is not None and door_limits.upper is not None else 0.0, lower_slide: lower_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lower_drawer_open_no_overlap")
            ctx.fail_if_isolated_parts(name="lower_drawer_open_no_floating")
            ctx.expect_contact(
                lower_drawer,
                cabinet,
                elem_a="left_skid",
                elem_b="lower_left_rail",
                name="lower_drawer_left_support_contact_open",
            )
            ctx.expect_contact(
                lower_drawer,
                cabinet,
                elem_a="right_skid",
                elem_b="lower_right_rail",
                name="lower_drawer_right_support_contact_open",
            )
            lower_open = ctx.part_world_position(lower_drawer)
            if lower_rest is not None and lower_open is not None:
                ctx.check(
                    "lower_drawer_slides_forward",
                    lower_open[1] > lower_rest[1] + 0.12,
                    details=f"lower_rest={lower_rest}, lower_open={lower_open}",
                )

    if (
        door_limits is not None
        and door_limits.upper is not None
        and upper_limits is not None
        and upper_limits.upper is not None
        and lower_limits is not None
        and lower_limits.upper is not None
    ):
        with ctx.pose(
            {
                door_hinge: door_limits.upper,
                upper_slide: upper_limits.upper,
                lower_slide: lower_limits.upper,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="fridge_access_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="fridge_access_pose_no_floating")

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
