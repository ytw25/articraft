from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from pathlib import Path

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
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_stove", assets=ASSETS)

    model.material("appliance_white", rgba=(0.93, 0.93, 0.94, 1.0))
    model.material("trim_dark", rgba=(0.14, 0.14, 0.16, 1.0))
    model.material("glass_top_black", rgba=(0.08, 0.08, 0.10, 0.92))
    model.material("zone_mark", rgba=(0.20, 0.20, 0.22, 1.0))
    model.material("burner_dark", rgba=(0.28, 0.14, 0.14, 1.0))
    model.material("window_glass", rgba=(0.16, 0.19, 0.23, 0.55))
    model.material("knob_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("knob_marker", rgba=(0.82, 0.82, 0.84, 1.0))

    width = 0.60
    depth = 0.63
    body_height = 0.888
    top_thickness = 0.012
    side_thickness = 0.018
    back_thickness = 0.014
    front_frame_depth = 0.030
    inner_width = width - (2.0 * side_thickness)
    cavity_depth = depth - front_frame_depth - back_thickness

    plinth_height = 0.10
    floor_thickness = 0.020
    ceiling_thickness = 0.020
    floor_center_z = 0.090
    ceiling_center_z = 0.730
    vent_strip_height = 0.040
    vent_strip_center_z = 0.740

    control_rail_height = 0.10
    control_rail_depth = 0.055
    control_rail_center_y = -(depth / 2.0) + (control_rail_depth / 2.0)
    control_rail_center_z = 0.810

    glass_center_z = body_height + (top_thickness / 2.0)

    door_width = 0.556
    door_height = 0.620
    door_thickness = 0.045
    hinge_radius = 0.008
    hinge_axis_y = -(depth / 2.0) - hinge_radius
    door_panel_center_y = -(door_thickness / 2.0) + hinge_radius

    chassis = model.part("chassis")
    chassis.visual(
        Box((side_thickness, depth, body_height)),
        origin=Origin(xyz=(-(width / 2.0) + (side_thickness / 2.0), 0.0, body_height / 2.0)),
        material="appliance_white",
        name="left_side",
    )
    chassis.visual(
        Box((side_thickness, depth, body_height)),
        origin=Origin(xyz=((width / 2.0) - (side_thickness / 2.0), 0.0, body_height / 2.0)),
        material="appliance_white",
        name="right_side",
    )
    chassis.visual(
        Box((inner_width, back_thickness, body_height - plinth_height)),
        origin=Origin(
            xyz=(
                0.0,
                (depth / 2.0) - (back_thickness / 2.0),
                plinth_height + ((body_height - plinth_height) / 2.0),
            )
        ),
        material="appliance_white",
        name="back_panel",
    )
    chassis.visual(
        Box((inner_width, cavity_depth, floor_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                (back_thickness - front_frame_depth) / 2.0,
                floor_center_z,
            )
        ),
        material="appliance_white",
        name="oven_floor",
    )
    chassis.visual(
        Box((inner_width, cavity_depth, ceiling_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                (back_thickness - front_frame_depth) / 2.0,
                ceiling_center_z,
            )
        ),
        material="appliance_white",
        name="oven_ceiling",
    )
    chassis.visual(
        Box((inner_width, front_frame_depth, plinth_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(depth / 2.0) + (front_frame_depth / 2.0),
                plinth_height / 2.0,
            )
        ),
        material="trim_dark",
        name="lower_plinth",
    )
    chassis.visual(
        Box((inner_width, front_frame_depth, vent_strip_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(depth / 2.0) + (front_frame_depth / 2.0),
                vent_strip_center_z,
            )
        ),
        material="trim_dark",
        name="vent_strip",
    )

    control_rail = model.part("control_rail")
    control_rail.visual(
        Box((inner_width, control_rail_depth, control_rail_height)),
        material="trim_dark",
        name="rail_shell",
    )
    model.articulation(
        "chassis_to_control_rail",
        ArticulationType.FIXED,
        parent=chassis,
        child=control_rail,
        origin=Origin(xyz=(0.0, control_rail_center_y, control_rail_center_z)),
    )

    glass_top = model.part("glass_top")
    glass_top.visual(
        Box((width, depth, top_thickness)),
        material="glass_top_black",
        name="glass_plate",
    )
    glass_top.visual(
        Cylinder(radius=0.078, length=0.0008),
        origin=Origin(xyz=(-0.16, -0.12, 0.0056)),
        material="zone_mark",
        name="mark_front_left",
    )
    glass_top.visual(
        Cylinder(radius=0.088, length=0.0008),
        origin=Origin(xyz=(0.16, -0.12, 0.0056)),
        material="zone_mark",
        name="mark_front_right",
    )
    glass_top.visual(
        Cylinder(radius=0.088, length=0.0008),
        origin=Origin(xyz=(-0.16, 0.12, 0.0056)),
        material="zone_mark",
        name="mark_rear_left",
    )
    glass_top.visual(
        Cylinder(radius=0.078, length=0.0008),
        origin=Origin(xyz=(0.16, 0.12, 0.0056)),
        material="zone_mark",
        name="mark_rear_right",
    )
    model.articulation(
        "chassis_to_glass_top",
        ArticulationType.FIXED,
        parent=chassis,
        child=glass_top,
        origin=Origin(xyz=(0.0, 0.0, glass_center_z)),
    )

    def add_radiant_zone(name: str, x_pos: float, y_pos: float, radius: float) -> None:
        zone = model.part(name)
        zone.visual(
            Cylinder(radius=radius, length=0.006),
            material="burner_dark",
            name="zone_disc",
        )
        model.articulation(
            f"glass_to_{name}",
            ArticulationType.FIXED,
            parent=glass_top,
            child=zone,
            origin=Origin(xyz=(x_pos, y_pos, -0.009)),
        )

    add_radiant_zone("zone_front_left", -0.16, -0.12, 0.075)
    add_radiant_zone("zone_front_right", 0.16, -0.12, 0.085)
    add_radiant_zone("zone_rear_left", -0.16, 0.12, 0.085)
    add_radiant_zone("zone_rear_right", 0.16, 0.12, 0.075)

    door = model.part("oven_door")
    handle_geometry = wire_from_points(
        [
            (-0.18, -0.030, 0.545),
            (-0.18, -0.060, 0.545),
            (0.18, -0.060, 0.545),
            (0.18, -0.030, 0.545),
        ],
        radius=0.008,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.018,
        corner_segments=10,
    )
    handle_mesh = mesh_from_geometry(
        handle_geometry,
        Path(ASSETS.asset_root) / "oven_door_handle.obj",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=inner_width),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="trim_dark",
        name="hinge_rod",
    )
    door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(0.0, door_panel_center_y, door_height / 2.0)),
        material="appliance_white",
        name="door_panel",
    )
    door.visual(
        Box((0.34, 0.006, 0.24)),
        origin=Origin(xyz=(0.0, -0.040, 0.390)),
        material="window_glass",
        name="door_window",
    )
    door.visual(handle_mesh, material="trim_dark", name="door_handle")
    model.articulation(
        "oven_door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=door,
        origin=Origin(xyz=(0.0, hinge_axis_y, plinth_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2, lower=0.0, upper=1.45),
    )

    def add_knob(name: str, x_pos: float) -> None:
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.011, length=0.012),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="trim_dark",
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.022, length=0.028),
            origin=Origin(xyz=(0.0, -0.024, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="knob_black",
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.010, 0.016)),
            origin=Origin(xyz=(0.0, -0.034, 0.014)),
            material="knob_marker",
            name="marker",
        )
        model.articulation(
            f"control_rail_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=control_rail,
            child=knob,
            origin=Origin(xyz=(x_pos, -(control_rail_depth / 2.0), 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=8.0),
        )

    add_knob("knob_left_front", -0.19)
    add_knob("knob_left_rear", -0.10)
    add_knob("knob_right_front", 0.10)
    add_knob("knob_right_rear", 0.19)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    control_rail = object_model.get_part("control_rail")
    glass_top = object_model.get_part("glass_top")
    oven_door = object_model.get_part("oven_door")
    knobs = [
        object_model.get_part("knob_left_front"),
        object_model.get_part("knob_left_rear"),
        object_model.get_part("knob_right_front"),
        object_model.get_part("knob_right_rear"),
    ]
    zones = [
        object_model.get_part("zone_front_left"),
        object_model.get_part("zone_front_right"),
        object_model.get_part("zone_rear_left"),
        object_model.get_part("zone_rear_right"),
    ]

    door_hinge = object_model.get_articulation("oven_door_hinge")
    knob_joints = [
        object_model.get_articulation("control_rail_to_knob_left_front"),
        object_model.get_articulation("control_rail_to_knob_left_rear"),
        object_model.get_articulation("control_rail_to_knob_right_front"),
        object_model.get_articulation("control_rail_to_knob_right_rear"),
    ]

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

    for part_name in (
        "chassis",
        "control_rail",
        "glass_top",
        "oven_door",
        "knob_left_front",
        "knob_left_rear",
        "knob_right_front",
        "knob_right_rear",
        "zone_front_left",
        "zone_front_right",
        "zone_rear_left",
        "zone_rear_right",
    ):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    ctx.check(
        "oven_door_hinge_axis",
        tuple(door_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected oven door hinge axis (1, 0, 0), got {door_hinge.axis}",
    )
    door_limits = door_hinge.motion_limits
    ctx.check(
        "oven_door_hinge_limits",
        door_limits is not None
        and door_limits.lower == 0.0
        and door_limits.upper == 1.45,
        details=f"unexpected oven door limits: {door_limits}",
    )

    for index, joint in enumerate(knob_joints, start=1):
        ctx.check(
            f"knob_joint_{index}_type",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint.name} should be continuous, got {joint.articulation_type}",
        )
        ctx.check(
            f"knob_joint_{index}_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"{joint.name} should rotate about +Y, got {joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"knob_joint_{index}_unbounded",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"{joint.name} should have continuous limits only, got {limits}",
        )

    ctx.expect_contact(glass_top, chassis, contact_tol=5e-4, name="glass_top_supported")
    ctx.expect_contact(control_rail, chassis, contact_tol=5e-4, name="control_rail_supported")
    ctx.expect_contact(oven_door, chassis, contact_tol=5e-4, name="door_closed_supported")
    ctx.expect_gap(
        chassis,
        oven_door,
        axis="y",
        min_gap=0.0,
        max_gap=5e-4,
        name="door_closed_flush_to_front_frame",
    )
    ctx.expect_overlap(
        oven_door,
        chassis,
        axes="xz",
        min_overlap=0.52,
        name="door_covers_oven_opening",
    )

    for knob in knobs:
        ctx.expect_contact(knob, control_rail, contact_tol=5e-4, name=f"{knob.name}_mounted")

    for zone in zones:
        ctx.expect_contact(zone, glass_top, contact_tol=5e-4, name=f"{zone.name}_under_glass")
        ctx.expect_within(zone, glass_top, axes="xy", margin=0.0, name=f"{zone.name}_within_glass")

    knob_positions = [ctx.part_world_position(knob) for knob in knobs]
    knob_xs = [pos[0] for pos in knob_positions if pos is not None]
    ctx.check(
        "knob_groups_ordered_left_and_right",
        len(knob_xs) == 4 and knob_xs[0] < knob_xs[1] < 0.0 < knob_xs[2] < knob_xs[3],
        details=f"unexpected knob x ordering: {knob_xs}",
    )
    ctx.check(
        "center_above_door_left_open",
        len(knob_xs) == 4 and (knob_xs[2] - knob_xs[1]) > 0.16,
        details=f"center control gap too small: {knob_xs}",
    )

    zone_positions = [ctx.part_world_position(zone) for zone in zones]
    zone_ys = [pos[1] for pos in zone_positions if pos is not None]
    ctx.check(
        "radiant_zones_in_two_rows",
        len(zone_ys) == 4 and zone_ys[0] < 0.0 and zone_ys[1] < 0.0 and zone_ys[2] > 0.0 and zone_ys[3] > 0.0,
        details=f"unexpected radiant zone y positions: {zone_ys}",
    )

    with ctx.pose(
        {
            knob_joints[0]: pi / 2.0,
            knob_joints[1]: pi,
            knob_joints[2]: (3.0 * pi) / 2.0,
            knob_joints[3]: pi / 4.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")
        ctx.fail_if_isolated_parts(contact_tol=5e-4, name="rotated_knobs_no_floating")

    closed_aabb = ctx.part_world_aabb(oven_door)
    if door_limits is not None and door_limits.lower is not None and door_limits.upper is not None:
        with ctx.pose({door_hinge: door_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_lower_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=5e-4, name="oven_door_lower_no_floating")
            ctx.expect_contact(oven_door, chassis, contact_tol=5e-4, name="oven_door_lower_contact")

        with ctx.pose({door_hinge: door_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(contact_tol=5e-4, name="oven_door_upper_no_floating")
            ctx.expect_contact(oven_door, chassis, contact_tol=5e-4, name="oven_door_upper_contact")
            open_aabb = ctx.part_world_aabb(oven_door)

        ctx.check(
            "oven_door_opens_downward",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] < (closed_aabb[1][2] - 0.10)
            and open_aabb[0][1] < (closed_aabb[0][1] - 0.10),
            details=f"closed door aabb={closed_aabb}, open door aabb={open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
