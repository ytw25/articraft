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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.50
BODY_DEPTH = 0.58
BODY_HEIGHT = 0.90
TOP_THICKNESS = 0.025
WALL_THICKNESS = 0.018
FRONT_FRAME_THICKNESS = 0.022
CONTROL_PANEL_BOTTOM = 0.690
CONTROL_PANEL_HEIGHT = BODY_HEIGHT - TOP_THICKNESS - CONTROL_PANEL_BOTTOM
BACKGUARD_HEIGHT = 0.18

OPENING_WIDTH = 0.456
OPENING_TOP = 0.680
KICK_HEIGHT = 0.10

DOOR_WIDTH = 0.444
DOOR_HEIGHT = 0.568
DOOR_THICKNESS = 0.026
DOOR_HINGE_Y = -(BODY_DEPTH * 0.5) - 0.011
DOOR_HINGE_Z = 0.094
DOOR_BOTTOM_CLEARANCE = 0.012
DOOR_BACK_OFFSET = 0.010

KNOB_Y = -(BODY_DEPTH * 0.5)
KNOB_Z = 0.775
KNOB_XS = (-0.165, -0.055, 0.055, 0.165)

BURNER_LAYOUT = {
    "front_left_burner": (-0.115, -0.085, 0.068),
    "front_right_burner": (0.115, -0.085, 0.068),
    "rear_left_burner": (-0.115, 0.115, 0.078),
    "rear_right_burner": (0.115, 0.115, 0.078),
}


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.asset_root / name)


def _coil_mesh(name: str, *, outer_radius: float, turns: float = 2.35, tube_radius: float = 0.004):
    inner_radius = 0.016
    samples = 28
    points: list[tuple[float, float, float]] = []
    for index in range(samples):
        fraction = index / (samples - 1)
        angle = fraction * turns * math.tau
        radius = inner_radius + (outer_radius - inner_radius) * fraction
        points.append((radius * math.cos(angle), radius * math.sin(angle), 0.015))
    return _save_mesh(
        name,
        wire_from_points(
            points,
            radius=tube_radius,
            radial_segments=16,
            cap_ends=True,
            corner_mode="miter",
        ),
    )


def _hinge_sleeve_mesh(name: str, *, length: float, inner_radius: float, outer_radius: float):
    return _save_mesh(
        name,
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -length * 0.5), (outer_radius, length * 0.5)],
            [(inner_radius, -length * 0.5), (inner_radius, length * 0.5)],
            segments=36,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apartment_stove", assets=ASSETS)

    enamel_white = model.material("enamel_white", rgba=(0.93, 0.93, 0.91, 1.0))
    cooktop_black = model.material("cooktop_black", rgba=(0.17, 0.18, 0.19, 1.0))
    burner_iron = model.material("burner_iron", rgba=(0.13, 0.13, 0.14, 1.0))
    dark_knob = model.material("dark_knob", rgba=(0.11, 0.11, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))

    small_coil = _coil_mesh("stove_small_coil.obj", outer_radius=0.068)
    large_coil = _coil_mesh("stove_large_coil.obj", outer_radius=0.078)
    hinge_sleeve = _hinge_sleeve_mesh(
        "stove_hinge_sleeve.obj",
        length=0.10,
        inner_radius=0.0065,
        outer_radius=0.011,
    )

    body = model.part("range_body")
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT + BACKGUARD_HEIGHT)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT + BACKGUARD_HEIGHT) * 0.5)),
    )

    side_height = BODY_HEIGHT - TOP_THICKNESS
    side_center_z = side_height * 0.5
    body.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, side_height)),
        origin=Origin(xyz=(-(BODY_WIDTH * 0.5) + (WALL_THICKNESS * 0.5), 0.0, side_center_z)),
        material=enamel_white,
        name="left_side_panel",
    )
    body.visual(
        Box((WALL_THICKNESS, BODY_DEPTH, side_height)),
        origin=Origin(xyz=((BODY_WIDTH * 0.5) - (WALL_THICKNESS * 0.5), 0.0, side_center_z)),
        material=enamel_white,
        name="right_side_panel",
    )
    body.visual(
        Box((BODY_WIDTH - (2.0 * WALL_THICKNESS), WALL_THICKNESS, side_height)),
        origin=Origin(
            xyz=(0.0, (BODY_DEPTH * 0.5) - (WALL_THICKNESS * 0.5), side_center_z)
        ),
        material=enamel_white,
        name="rear_panel",
    )
    body.visual(
        Box((BODY_WIDTH - (2.0 * WALL_THICKNESS), BODY_DEPTH - WALL_THICKNESS, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, WALL_THICKNESS * 0.5, WALL_THICKNESS * 0.5)),
        material=enamel_white,
        name="base_panel",
    )
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - (TOP_THICKNESS * 0.5))),
        material=cooktop_black,
        name="cooktop_panel",
    )
    body.visual(
        Box((BODY_WIDTH, FRONT_FRAME_THICKNESS, CONTROL_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_DEPTH * 0.5) + (FRONT_FRAME_THICKNESS * 0.5),
                CONTROL_PANEL_BOTTOM + (CONTROL_PANEL_HEIGHT * 0.5),
            )
        ),
        material=enamel_white,
        name="front_control_fascia",
    )

    rail_width = (BODY_WIDTH - OPENING_WIDTH) * 0.5
    opening_height = OPENING_TOP - KICK_HEIGHT
    rail_center_z = KICK_HEIGHT + (opening_height * 0.5)
    front_frame_y = -(BODY_DEPTH * 0.5) + (FRONT_FRAME_THICKNESS * 0.5)
    body.visual(
        Box((rail_width, FRONT_FRAME_THICKNESS, opening_height)),
        origin=Origin(
            xyz=(
                -(BODY_WIDTH * 0.5) + (rail_width * 0.5),
                front_frame_y,
                rail_center_z,
            )
        ),
        material=enamel_white,
        name="front_left_rail",
    )
    body.visual(
        Box((rail_width, FRONT_FRAME_THICKNESS, opening_height)),
        origin=Origin(
            xyz=(
                (BODY_WIDTH * 0.5) - (rail_width * 0.5),
                front_frame_y,
                rail_center_z,
            )
        ),
        material=enamel_white,
        name="front_right_rail",
    )
    body.visual(
        Box((BODY_WIDTH, FRONT_FRAME_THICKNESS, KICK_HEIGHT)),
        origin=Origin(xyz=(0.0, front_frame_y, KICK_HEIGHT * 0.5)),
        material=enamel_white,
        name="kick_panel",
    )
    body.visual(
        Box((OPENING_WIDTH, FRONT_FRAME_THICKNESS, CONTROL_PANEL_BOTTOM - OPENING_TOP)),
        origin=Origin(
            xyz=(
                0.0,
                front_frame_y,
                OPENING_TOP + ((CONTROL_PANEL_BOTTOM - OPENING_TOP) * 0.5),
            )
        ),
        material=enamel_white,
        name="opening_top_rail",
    )

    cavity_width = 0.414
    cavity_depth = 0.474
    cavity_height = 0.556
    cavity_center_y = -0.013
    cavity_center_z = 0.396
    body.visual(
        Box((WALL_THICKNESS, cavity_depth, cavity_height)),
        origin=Origin(
            xyz=(
                -(cavity_width * 0.5) - (WALL_THICKNESS * 0.5),
                cavity_center_y,
                cavity_center_z,
            )
        ),
        material=cooktop_black,
        name="oven_liner_left",
    )
    body.visual(
        Box((WALL_THICKNESS, cavity_depth, cavity_height)),
        origin=Origin(
            xyz=(
                (cavity_width * 0.5) + (WALL_THICKNESS * 0.5),
                cavity_center_y,
                cavity_center_z,
            )
        ),
        material=cooktop_black,
        name="oven_liner_right",
    )
    body.visual(
        Box((cavity_width, cavity_depth, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, cavity_center_y, 0.118 - (WALL_THICKNESS * 0.5))),
        material=cooktop_black,
        name="oven_floor",
    )
    body.visual(
        Box((cavity_width, cavity_depth, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, cavity_center_y, 0.674 + (WALL_THICKNESS * 0.5))),
        material=cooktop_black,
        name="oven_ceiling",
    )
    body.visual(
        Box((cavity_width, WALL_THICKNESS, cavity_height)),
        origin=Origin(
            xyz=(0.0, cavity_center_y + (cavity_depth * 0.5) - (WALL_THICKNESS * 0.5), cavity_center_z)
        ),
        material=cooktop_black,
        name="oven_back_liner",
    )
    body.visual(
        Box((0.021, WALL_THICKNESS, cavity_height)),
        origin=Origin(
            xyz=(
                -((cavity_width + 0.021) * 0.5),
                front_frame_y + (FRONT_FRAME_THICKNESS * 0.5) + (WALL_THICKNESS * 0.5),
                cavity_center_z,
            )
        ),
        material=cooktop_black,
        name="oven_front_flange_left",
    )
    body.visual(
        Box((0.021, WALL_THICKNESS, cavity_height)),
        origin=Origin(
            xyz=(
                (cavity_width + 0.021) * 0.5,
                front_frame_y + (FRONT_FRAME_THICKNESS * 0.5) + (WALL_THICKNESS * 0.5),
                cavity_center_z,
            )
        ),
        material=cooktop_black,
        name="oven_front_flange_right",
    )
    body.visual(
        Box((cavity_width, WALL_THICKNESS, 0.012)),
        origin=Origin(
            xyz=(
                0.0,
                front_frame_y + (FRONT_FRAME_THICKNESS * 0.5) + (WALL_THICKNESS * 0.5),
                0.680,
            )
        ),
        material=cooktop_black,
        name="oven_front_flange_top",
    )

    body.visual(
        Box((BODY_WIDTH, 0.045, BACKGUARD_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (BODY_DEPTH * 0.5) - 0.0225,
                BODY_HEIGHT + (BACKGUARD_HEIGHT * 0.5),
            )
        ),
        material=enamel_white,
        name="backguard_panel",
    )
    body.visual(
        Box((BODY_WIDTH * 0.90, 0.020, 0.018)),
        origin=Origin(
            xyz=(0.0, (BODY_DEPTH * 0.5) - 0.012, BODY_HEIGHT + BACKGUARD_HEIGHT - 0.009)
        ),
        material=steel,
        name="backguard_top_cap",
    )

    body.visual(
        Cylinder(radius=0.0065, length=0.48),
        origin=Origin(
            xyz=(0.0, DOOR_HINGE_Y, DOOR_HINGE_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="door_hinge_rod",
    )
    body.visual(
        Box((0.016, 0.012, 0.026)),
        origin=Origin(xyz=(-0.242, -0.296, DOOR_HINGE_Z)),
        material=enamel_white,
        name="hinge_support_left",
    )
    body.visual(
        Box((0.016, 0.012, 0.026)),
        origin=Origin(xyz=(0.242, -0.296, DOOR_HINGE_Z)),
        material=enamel_white,
        name="hinge_support_right",
    )

    for burner_name, (x_pos, y_pos, outer_radius) in BURNER_LAYOUT.items():
        burner = model.part(burner_name)
        plate_radius = outer_radius + 0.017
        burner.inertial = Inertial.from_geometry(
            Cylinder(radius=plate_radius, length=0.026),
            mass=0.55,
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
        )
        burner.visual(
            Cylinder(radius=plate_radius, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0015)),
            material=burner_iron,
            name="drip_pan",
        )
        burner.visual(
            Cylinder(radius=0.017, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=burner_iron,
            name="center_post",
        )
        burner.visual(
            large_coil if outer_radius > 0.07 else small_coil,
            material=burner_iron,
            name="coil",
        )
        model.articulation(
            f"body_to_{burner_name}",
            ArticulationType.FIXED,
            parent=body,
            child=burner,
            origin=Origin(xyz=(x_pos, y_pos, BODY_HEIGHT)),
        )

    for index, knob_x in enumerate(KNOB_XS, start=1):
        knob = model.part(f"knob_{index}")
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.022, length=0.032),
            mass=0.06,
            origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        )
        knob.visual(
            Cylinder(radius=0.022, length=0.032),
            origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_knob,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.013, length=0.032),
            origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_knob,
            name="knob_core",
        )
        knob.visual(
            Box((0.018, 0.004, 0.003)),
            origin=Origin(xyz=(0.0, -0.0315, 0.012)),
            material=steel,
            name="pointer",
        )
        model.articulation(
            f"body_to_knob_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(knob_x, KNOB_Y, KNOB_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.3, velocity=6.0),
        )

    oven_door = model.part("oven_door")
    oven_door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, 0.070, DOOR_HEIGHT + 0.040)),
        mass=7.5,
        origin=Origin(
            xyz=(
                0.0,
                DOOR_BACK_OFFSET - (DOOR_THICKNESS * 0.5),
                DOOR_BOTTOM_CLEARANCE + (DOOR_HEIGHT * 0.5),
            )
        ),
    )
    oven_door.visual(
        Box((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                DOOR_BACK_OFFSET - (DOOR_THICKNESS * 0.5),
                DOOR_BOTTOM_CLEARANCE + (DOOR_HEIGHT * 0.5),
            )
        ),
        material=enamel_white,
        name="door_panel",
    )
    oven_door.visual(
        Box((DOOR_WIDTH - 0.070, 0.010, DOOR_HEIGHT - 0.120)),
        origin=Origin(
            xyz=(
                0.0,
                DOOR_BACK_OFFSET - 0.008,
                DOOR_BOTTOM_CLEARANCE + (DOOR_HEIGHT * 0.50),
            )
        ),
        material=cooktop_black,
        name="inner_panel",
    )
    oven_door.visual(
        Cylinder(radius=0.008, length=0.30),
        origin=Origin(
            xyz=(
                0.0,
                DOOR_BACK_OFFSET - 0.048,
                DOOR_BOTTOM_CLEARANCE + DOOR_HEIGHT - 0.095,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="handle_bar",
    )
    oven_door.visual(
        Cylinder(radius=0.0055, length=0.022),
        origin=Origin(
            xyz=(
                -0.128,
                DOOR_BACK_OFFSET - 0.037,
                DOOR_BOTTOM_CLEARANCE + DOOR_HEIGHT - 0.095,
            ),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="handle_standoff_left",
    )
    oven_door.visual(
        Cylinder(radius=0.0055, length=0.022),
        origin=Origin(
            xyz=(
                0.128,
                DOOR_BACK_OFFSET - 0.037,
                DOOR_BOTTOM_CLEARANCE + DOOR_HEIGHT - 0.095,
            ),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="handle_standoff_right",
    )
    oven_door.visual(
        hinge_sleeve,
        origin=Origin(xyz=(-0.120, 0.0, 0.001)),
        material=steel,
        name="hinge_sleeve_left",
    )
    oven_door.visual(
        hinge_sleeve,
        origin=Origin(xyz=(0.120, 0.0, 0.001)),
        material=steel,
        name="hinge_sleeve_right",
    )

    model.articulation(
        "body_to_oven_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=oven_door,
        origin=Origin(xyz=(0.0, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    body = object_model.get_part("range_body")
    oven_door = object_model.get_part("oven_door")
    burners = [object_model.get_part(name) for name in BURNER_LAYOUT]
    knobs = [object_model.get_part(f"knob_{index}") for index in range(1, 5)]
    door_hinge = object_model.get_articulation("body_to_oven_door")
    knob_joints = [object_model.get_articulation(f"body_to_knob_{index}") for index in range(1, 5)]

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
    ctx.allow_overlap(
        body,
        oven_door,
        elem_a="door_hinge_rod",
        elem_b="hinge_sleeve_left",
        reason="The oven door rotates on a hinge pin captured inside the left hinge sleeve.",
    )
    ctx.allow_overlap(
        body,
        oven_door,
        elem_a="door_hinge_rod",
        elem_b="hinge_sleeve_right",
        reason="The oven door rotates on a hinge pin captured inside the right hinge sleeve.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="articulation_pose_clearance")

    for burner in burners:
        ctx.expect_contact(burner, body, name=f"{burner.name}_mounted")
        ctx.expect_within(burner, body, axes="xy", margin=0.02, name=f"{burner.name}_within_plan")

    cooktop_aabb = ctx.part_element_world_aabb(body, elem="cooktop_panel")
    backguard_aabb = ctx.part_element_world_aabb(body, elem="backguard_panel")
    if cooktop_aabb is not None and backguard_aabb is not None:
        ctx.check(
            "backguard_rises_above_cooktop",
            backguard_aabb[1][2] > cooktop_aabb[1][2] + 0.15,
            details=f"backguard top {backguard_aabb[1][2]:.3f} vs cooktop top {cooktop_aabb[1][2]:.3f}",
        )

    knob_positions = [ctx.part_world_position(knob) for knob in knobs]
    if all(position is not None for position in knob_positions):
        knob_positions = [position for position in knob_positions if position is not None]
        z_values = [position[2] for position in knob_positions]
        y_values = [position[1] for position in knob_positions]
        x_values = [position[0] for position in knob_positions]
        spacings = [x_values[index + 1] - x_values[index] for index in range(len(x_values) - 1)]
        mean_spacing = sum(spacings) / len(spacings)
        ctx.check(
            "knobs_share_front_face",
            max(abs(value - y_values[0]) for value in y_values) < 1e-6,
            details=f"knob y positions: {y_values}",
        )
        ctx.check(
            "knobs_share_height",
            max(z_values) - min(z_values) < 1e-6,
            details=f"knob z positions: {z_values}",
        )
        ctx.check(
            "knobs_evenly_spaced",
            max(abs(spacing - mean_spacing) for spacing in spacings) < 0.003,
            details=f"knob x positions: {x_values}",
        )

    for index, (knob, joint) in enumerate(zip(knobs, knob_joints, strict=True), start=1):
        ctx.expect_contact(knob, body, name=f"knob_{index}_contact_at_rest")
        pointer_rest = ctx.part_element_world_aabb(knob, elem="pointer")
        with ctx.pose({joint: math.pi / 2.0}):
            ctx.expect_contact(knob, body, name=f"knob_{index}_contact_rotated")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"knob_{index}_rotated_no_overlap")
            ctx.fail_if_isolated_parts(name=f"knob_{index}_rotated_no_floating")
            pointer_rotated = ctx.part_element_world_aabb(knob, elem="pointer")
            if pointer_rest is not None and pointer_rotated is not None:
                rest_x_span = pointer_rest[1][0] - pointer_rest[0][0]
                rest_z_span = pointer_rest[1][2] - pointer_rest[0][2]
                rotated_x_span = pointer_rotated[1][0] - pointer_rotated[0][0]
                rotated_z_span = pointer_rotated[1][2] - pointer_rotated[0][2]
                ctx.check(
                    f"knob_{index}_rotates_about_front_to_back_axis",
                    (rest_x_span > rest_z_span) and (rotated_z_span > rotated_x_span),
                    details=(
                        f"rest spans x/z=({rest_x_span:.4f}, {rest_z_span:.4f}), "
                        f"rotated spans x/z=({rotated_x_span:.4f}, {rotated_z_span:.4f})"
                    ),
                )

    front_left_rail_aabb = ctx.part_element_world_aabb(body, elem="front_left_rail")
    closed_door_panel_aabb = None
    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_contact(oven_door, body, name="oven_door_hinge_contact_closed")
        closed_door_panel_aabb = ctx.part_element_world_aabb(oven_door, elem="door_panel")
        if front_left_rail_aabb is not None and closed_door_panel_aabb is not None:
            front_gap = front_left_rail_aabb[0][1] - closed_door_panel_aabb[1][1]
            ctx.check(
                "oven_door_front_gap_closed",
                0.0 <= front_gap <= 0.004,
                details=f"closed door gap to front frame: {front_gap:.4f}",
            )

    limits = door_hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({door_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_lower_no_floating")
            ctx.expect_contact(oven_door, body, name="oven_door_hinge_contact_lower")
        with ctx.pose({door_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="oven_door_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="oven_door_upper_no_floating")
            ctx.expect_contact(oven_door, body, name="oven_door_hinge_contact_open")
            open_door_panel_aabb = ctx.part_element_world_aabb(oven_door, elem="door_panel")
            if closed_door_panel_aabb is not None and open_door_panel_aabb is not None:
                ctx.check(
                    "oven_door_swings_downward",
                    (open_door_panel_aabb[0][1] < closed_door_panel_aabb[0][1] - 0.40)
                    and (open_door_panel_aabb[1][2] < closed_door_panel_aabb[1][2] - 0.40),
                    details=(
                        f"closed min y/max z=({closed_door_panel_aabb[0][1]:.3f}, {closed_door_panel_aabb[1][2]:.3f}), "
                        f"open min y/max z=({open_door_panel_aabb[0][1]:.3f}, {open_door_panel_aabb[1][2]:.3f})"
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
