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
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

COOKTOP_WIDTH = 0.76
COOKTOP_DEPTH = 0.56
TOP_PLATE_THICKNESS = 0.008
BODY_TOP_Z = 0.14

BURNER_LAYOUT: dict[str, tuple[float, float]] = {
    "rear_left": (-0.19, 0.12),
    "front_left": (-0.19, -0.10),
    "rear_right": (0.19, 0.12),
    "front_right": (0.19, -0.10),
}

CONTROL_LAYOUT: dict[str, tuple[float, float]] = {
    "rear_left": (-0.19, 0.024),
    "front_left": (-0.19, -0.024),
    "rear_right": (0.19, 0.024),
    "front_right": (0.19, -0.024),
}


def _add_vertical_slotted_fastener(
    part,
    *,
    name_prefix: str,
    xyz: tuple[float, float, float],
    head_radius: float,
    head_length: float,
    material,
    slot_material,
) -> None:
    x, y, z = xyz
    part.visual(
        Cylinder(radius=head_radius, length=head_length),
        origin=Origin(xyz=(x, y, z + head_length * 0.5)),
        material=material,
        name=f"{name_prefix}_head",
    )
    part.visual(
        Box((head_radius * 1.45, head_radius * 0.22, head_length * 0.45)),
        origin=Origin(
            xyz=(
                x,
                y,
                z + head_length - head_length * 0.18,
            )
        ),
        material=slot_material,
        name=f"{name_prefix}_slot",
    )


def _add_front_slotted_fastener(
    part,
    *,
    name_prefix: str,
    xyz: tuple[float, float, float],
    head_radius: float,
    head_length: float,
    material,
    slot_material,
) -> None:
    x, y, z = xyz
    part.visual(
        Cylinder(radius=head_radius, length=head_length),
        origin=Origin(
            xyz=(x, y - head_length * 0.5, z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=material,
        name=f"{name_prefix}_head",
    )
    part.visual(
        Box((head_radius * 1.30, head_length * 0.40, head_radius * 0.20)),
        origin=Origin(xyz=(x, y - head_length * 0.82, z)),
        material=slot_material,
        name=f"{name_prefix}_slot",
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_front_fascia_mesh():
    half_width = COOKTOP_WIDTH * 0.5

    def section(x: float, front_y: float) -> list[tuple[float, float, float]]:
        return [
            (x, front_y, -0.0525),
            (x, 0.022, -0.0525),
            (x, 0.025, 0.040),
            (x, 0.018, 0.0525),
            (x, -0.014, 0.0525),
            (x, front_y, 0.030),
        ]

    geom = section_loft(
        [
            section(-half_width, -0.025),
            section(0.0, -0.028),
            section(half_width, -0.025),
        ]
    )
    return _save_mesh("utility_stovetop_front_fascia.obj", geom)


def _build_knob_shell_mesh():
    profile = [
        (0.0, 0.020),
        (0.023, 0.020),
        (0.023, 0.023),
        (0.020, 0.026),
        (0.020, 0.039),
        (0.018, 0.044),
        (0.015, 0.049),
        (0.0, 0.049),
    ]
    geom = LatheGeometry(profile, segments=48)
    geom.rotate_x(math.pi * 0.5)
    return _save_mesh("utility_stovetop_knob_shell.obj", geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_stovetop", assets=ASSETS)

    body_paint = model.material("body_paint", rgba=(0.25, 0.26, 0.28, 1.0))
    top_enamel = model.material("top_enamel", rgba=(0.17, 0.18, 0.19, 1.0))
    reinforcement = model.material("reinforcement", rgba=(0.22, 0.23, 0.24, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.11, 0.11, 0.12, 1.0))
    burner_steel = model.material("burner_steel", rgba=(0.47, 0.49, 0.52, 1.0))
    cap_black = model.material("cap_black", rgba=(0.15, 0.15, 0.16, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.68, 0.69, 0.71, 1.0))
    fastener_zinc = model.material("fastener_zinc", rgba=(0.66, 0.68, 0.71, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    indicator = model.material("indicator", rgba=(0.87, 0.84, 0.75, 1.0))
    ceramic = model.material("ceramic", rgba=(0.86, 0.87, 0.83, 1.0))
    front_fascia_shell_mesh = _build_front_fascia_mesh()
    knob_shell_mesh = _build_knob_shell_mesh()

    body = model.part("body")
    body.visual(
        Box((0.68, 0.46, 0.122)),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=body_paint,
        name="undercase",
    )
    body.visual(
        Box((0.74, 0.03, 0.014)),
        origin=Origin(xyz=(0.0, -0.215, 0.119)),
        material=reinforcement,
        name="front_cross_rail",
    )
    body.visual(
        Box((0.74, 0.03, 0.014)),
        origin=Origin(xyz=(0.0, 0.215, 0.119)),
        material=reinforcement,
        name="rear_cross_rail",
    )
    body.visual(
        Box((0.03, 0.44, 0.014)),
        origin=Origin(xyz=(-0.355, 0.0, 0.119)),
        material=reinforcement,
        name="left_mount_rail",
    )
    body.visual(
        Box((0.03, 0.44, 0.014)),
        origin=Origin(xyz=(0.355, 0.0, 0.119)),
        material=reinforcement,
        name="right_mount_rail",
    )
    body.visual(
        Box((0.05, 0.05, 0.022)),
        origin=Origin(xyz=(-0.315, -0.195, 0.115)),
        material=reinforcement,
        name="front_left_gusset",
    )
    body.visual(
        Box((0.05, 0.05, 0.022)),
        origin=Origin(xyz=(0.315, -0.195, 0.115)),
        material=reinforcement,
        name="front_right_gusset",
    )
    body.visual(
        Box((0.05, 0.05, 0.022)),
        origin=Origin(xyz=(-0.315, 0.195, 0.115)),
        material=reinforcement,
        name="rear_left_gusset",
    )
    body.visual(
        Box((0.05, 0.05, 0.022)),
        origin=Origin(xyz=(0.315, 0.195, 0.115)),
        material=reinforcement,
        name="rear_right_gusset",
    )
    body.visual(
        Box((0.72, 0.03, 0.014)),
        origin=Origin(xyz=(0.0, 0.245, 0.133)),
        material=body_paint,
        name="rear_service_lip",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.76, 0.56, 0.16)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    top_plate = model.part("top_plate")
    top_plate.visual(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, TOP_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_THICKNESS * 0.5)),
        material=top_enamel,
        name="deck",
    )
    top_plate.visual(
        Box((0.70, 0.04, 0.014)),
        origin=Origin(xyz=(0.0, -0.19, -0.007)),
        material=reinforcement,
        name="front_stiffener",
    )
    top_plate.visual(
        Box((0.70, 0.04, 0.014)),
        origin=Origin(xyz=(0.0, 0.19, -0.007)),
        material=reinforcement,
        name="rear_stiffener",
    )
    top_plate.visual(
        Box((0.03, 0.46, 0.014)),
        origin=Origin(xyz=(-0.30, 0.0, -0.007)),
        material=reinforcement,
        name="left_stiffener",
    )
    top_plate.visual(
        Box((0.03, 0.46, 0.014)),
        origin=Origin(xyz=(0.30, 0.0, -0.007)),
        material=reinforcement,
        name="right_stiffener",
    )
    for fastener_name, x_pos, y_pos in (
        ("front_left", -0.325, -0.235),
        ("front_right", 0.325, -0.235),
        ("rear_left", -0.325, 0.235),
        ("rear_right", 0.325, 0.235),
    ):
        _add_vertical_slotted_fastener(
            top_plate,
            name_prefix=f"deck_{fastener_name}",
            xyz=(x_pos, y_pos, TOP_PLATE_THICKNESS),
            head_radius=0.010,
            head_length=0.004,
            material=fastener_zinc,
            slot_material=reinforcement,
        )
    top_plate.inertial = Inertial.from_geometry(
        Box((COOKTOP_WIDTH, COOKTOP_DEPTH, 0.022)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_top_plate",
        ArticulationType.FIXED,
        parent=body,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z)),
    )

    front_fascia = model.part("front_fascia")
    front_fascia.visual(
        front_fascia_shell_mesh,
        material=body_paint,
        name="panel_shell",
    )
    front_fascia.visual(
        Box((0.12, 0.012, 0.065)),
        origin=Origin(xyz=(-0.19, 0.019, 0.0)),
        material=reinforcement,
        name="left_control_backer",
    )
    front_fascia.visual(
        Box((0.12, 0.012, 0.065)),
        origin=Origin(xyz=(0.19, 0.019, 0.0)),
        material=reinforcement,
        name="right_control_backer",
    )
    front_fascia.visual(
        Box((0.58, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.019, -0.036)),
        material=reinforcement,
        name="lower_service_rib",
    )
    for fastener_name, x_pos, z_pos in (
        ("panel_left_upper", -0.34, 0.028),
        ("panel_left_lower", -0.34, -0.028),
        ("panel_right_upper", 0.34, 0.028),
        ("panel_right_lower", 0.34, -0.028),
    ):
        _add_front_slotted_fastener(
            front_fascia,
            name_prefix=fastener_name,
            xyz=(x_pos, -0.025, z_pos),
            head_radius=0.007,
            head_length=0.004,
            material=fastener_zinc,
            slot_material=reinforcement,
        )
    front_fascia.inertial = Inertial.from_geometry(
        Box((COOKTOP_WIDTH, 0.05, 0.105)),
        mass=3.4,
        origin=Origin(),
    )
    model.articulation(
        "body_to_front_fascia",
        ArticulationType.FIXED,
        parent=body,
        child=front_fascia,
        origin=Origin(xyz=(0.0, -0.255, 0.0875)),
    )

    for burner_name, (burner_x, burner_y) in BURNER_LAYOUT.items():
        burner = model.part(f"burner_{burner_name}")
        burner.visual(
            Cylinder(radius=0.067, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.005)),
            material=cast_iron,
            name="spill_ring",
        )
        burner.visual(
            Cylinder(radius=0.052, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=burner_steel,
            name="flame_ring",
        )
        burner.visual(
            Cylinder(radius=0.042, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.018)),
            material=burner_steel,
            name="burner_head",
        )
        burner.visual(
            Cylinder(radius=0.028, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, 0.029)),
            material=cap_black,
            name="burner_cap",
        )
        burner.visual(
            Box((0.008, 0.012, 0.018)),
            origin=Origin(xyz=(0.041, 0.0, 0.009)),
            material=ceramic,
            name="igniter_block",
        )
        burner.visual(
            Cylinder(radius=0.003, length=0.016),
            origin=Origin(xyz=(-0.037, 0.0, 0.008)),
            material=burner_steel,
            name="thermocouple",
        )
        _add_vertical_slotted_fastener(
            burner,
            name_prefix=f"{burner_name}_service_fastener_left",
            xyz=(-0.018, -0.040, 0.010),
            head_radius=0.005,
            head_length=0.003,
            material=fastener_zinc,
            slot_material=reinforcement,
        )
        _add_vertical_slotted_fastener(
            burner,
            name_prefix=f"{burner_name}_service_fastener_right",
            xyz=(0.018, -0.040, 0.010),
            head_radius=0.005,
            head_length=0.003,
            material=fastener_zinc,
            slot_material=reinforcement,
        )
        burner.inertial = Inertial.from_geometry(
            Box((0.14, 0.14, 0.042)),
            mass=0.65,
            origin=Origin(xyz=(0.0, 0.0, 0.021)),
        )
        model.articulation(
            f"top_plate_to_burner_{burner_name}",
            ArticulationType.FIXED,
            parent=top_plate,
            child=burner,
            origin=Origin(xyz=(burner_x, burner_y, TOP_PLATE_THICKNESS)),
        )

        grate = model.part(f"grate_{burner_name}")
        for foot_name, foot_x, foot_y in (
            ("front_left", -0.071, -0.071),
            ("front_right", 0.071, -0.071),
            ("rear_left", -0.071, 0.071),
            ("rear_right", 0.071, 0.071),
        ):
            grate.visual(
                Box((0.022, 0.022, 0.028)),
                origin=Origin(xyz=(foot_x, foot_y, 0.014)),
                material=cast_iron,
                name=f"{foot_name}_foot",
            )
        grate.visual(
            Box((0.018, 0.190, 0.014)),
            origin=Origin(xyz=(-0.060, 0.0, 0.034)),
            material=cast_iron,
            name="left_rail",
        )
        grate.visual(
            Box((0.018, 0.190, 0.014)),
            origin=Origin(xyz=(0.060, 0.0, 0.034)),
            material=cast_iron,
            name="right_rail",
        )
        grate.visual(
            Box((0.190, 0.018, 0.014)),
            origin=Origin(xyz=(0.0, -0.060, 0.034)),
            material=cast_iron,
            name="front_rail",
        )
        grate.visual(
            Box((0.190, 0.018, 0.014)),
            origin=Origin(xyz=(0.0, 0.060, 0.034)),
            material=cast_iron,
            name="rear_rail",
        )
        grate.visual(
            Box((0.150, 0.014, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.040)),
            material=cast_iron,
            name="crossbar_x",
        )
        grate.visual(
            Box((0.014, 0.150, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.040)),
            material=cast_iron,
            name="crossbar_y",
        )
        grate.inertial = Inertial.from_geometry(
            Box((0.20, 0.20, 0.05)),
            mass=1.25,
            origin=Origin(xyz=(0.0, 0.0, 0.025)),
        )
        model.articulation(
            f"top_plate_to_grate_{burner_name}",
            ArticulationType.FIXED,
            parent=top_plate,
            child=grate,
            origin=Origin(xyz=(burner_x, burner_y, TOP_PLATE_THICKNESS)),
        )

    for control_name, (control_x, control_z) in CONTROL_LAYOUT.items():
        shaft = model.part(f"shaft_{control_name}")
        shaft.visual(
            Box((0.070, 0.004, 0.050)),
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=reinforcement,
            name="mount_pad",
        )
        shaft.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(
                xyz=(0.0, -0.003, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=shaft_steel,
            name="shaft_collar",
        )
        shaft.visual(
            Cylinder(radius=0.005, length=0.014),
            origin=Origin(
                xyz=(0.0, -0.007, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=shaft_steel,
            name="shaft",
        )
        _add_front_slotted_fastener(
            shaft,
            name_prefix=f"{control_name}_mount_fastener_left",
            xyz=(-0.024, 0.0, 0.0),
            head_radius=0.0055,
            head_length=0.003,
            material=fastener_zinc,
            slot_material=reinforcement,
        )
        _add_front_slotted_fastener(
            shaft,
            name_prefix=f"{control_name}_mount_fastener_right",
            xyz=(0.024, 0.0, 0.0),
            head_radius=0.0055,
            head_length=0.003,
            material=fastener_zinc,
            slot_material=reinforcement,
        )
        shaft.inertial = Inertial.from_geometry(
            Box((0.07, 0.02, 0.05)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.005, 0.0)),
        )
        model.articulation(
            f"front_fascia_to_shaft_{control_name}",
            ArticulationType.FIXED,
            parent=front_fascia,
            child=shaft,
            origin=Origin(xyz=(control_x, -0.025, control_z)),
        )

        knob = model.part(f"knob_{control_name}")
        knob.visual(
            Cylinder(radius=0.009, length=0.012),
            origin=Origin(
                xyz=(0.0, -0.020, 0.0),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=knob_black,
            name="hub",
        )
        knob.visual(
            knob_shell_mesh,
            material=knob_black,
            name="knob_shell",
        )
        knob.visual(
            Box((0.006, 0.010, 0.014)),
            origin=Origin(xyz=(-0.020, -0.039, 0.0)),
            material=knob_black,
            name="left_grip",
        )
        knob.visual(
            Box((0.006, 0.010, 0.014)),
            origin=Origin(xyz=(0.020, -0.039, 0.0)),
            material=knob_black,
            name="right_grip",
        )
        knob.visual(
            Box((0.014, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, -0.039, 0.020)),
            material=knob_black,
            name="top_grip",
        )
        knob.visual(
            Box((0.014, 0.010, 0.006)),
            origin=Origin(xyz=(0.0, -0.039, -0.020)),
            material=knob_black,
            name="bottom_grip",
        )
        knob.visual(
            Box((0.004, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, -0.051, 0.012)),
            material=indicator,
            name="indicator",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.05, 0.06, 0.05)),
            mass=0.16,
            origin=Origin(xyz=(0.0, -0.036, 0.0)),
        )
        model.articulation(
            f"shaft_to_knob_{control_name}",
            ArticulationType.REVOLUTE,
            parent=shaft,
            child=knob,
            origin=Origin(),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.4,
                velocity=2.5,
                lower=0.0,
                upper=4.2,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    top_plate = object_model.get_part("top_plate")
    front_fascia = object_model.get_part("front_fascia")
    burners = {
        name: object_model.get_part(f"burner_{name}") for name in BURNER_LAYOUT
    }
    grates = {
        name: object_model.get_part(f"grate_{name}") for name in BURNER_LAYOUT
    }
    shafts = {
        name: object_model.get_part(f"shaft_{name}") for name in CONTROL_LAYOUT
    }
    knobs = {
        name: object_model.get_part(f"knob_{name}") for name in CONTROL_LAYOUT
    }
    knob_joints = {
        name: object_model.get_articulation(f"shaft_to_knob_{name}")
        for name in CONTROL_LAYOUT
    }

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(top_plate, body, name="top_plate_supported_by_body")
    ctx.expect_gap(
        top_plate,
        body,
        axis="z",
        positive_elem="front_stiffener",
        negative_elem="front_cross_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="top_plate_front_stiffener_seated_on_body",
    )
    ctx.expect_gap(
        top_plate,
        body,
        axis="z",
        positive_elem="rear_stiffener",
        negative_elem="rear_cross_rail",
        max_gap=0.001,
        max_penetration=0.0,
        name="top_plate_rear_stiffener_seated_on_body",
    )
    ctx.expect_contact(front_fascia, body, name="front_fascia_contacts_body")
    ctx.expect_gap(
        top_plate,
        front_fascia,
        axis="z",
        positive_elem="deck",
        negative_elem="panel_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="front_fascia_captured_under_top_plate",
    )

    for burner_name in BURNER_LAYOUT:
        burner = burners[burner_name]
        grate = grates[burner_name]
        shaft = shafts[burner_name]
        knob = knobs[burner_name]

        ctx.expect_contact(
            burner,
            top_plate,
            name=f"{burner_name}_burner_seated_on_deck",
        )
        ctx.expect_contact(
            grate,
            top_plate,
            name=f"{burner_name}_grate_seated_on_deck",
        )
        ctx.expect_overlap(
            grate,
            burner,
            axes="xy",
            min_overlap=0.10,
            name=f"{burner_name}_grate_covers_burner",
        )
        ctx.expect_within(
            burner,
            grate,
            axes="xy",
            margin=0.0,
            name=f"{burner_name}_burner_within_grate_span",
        )
        ctx.expect_contact(
            shaft,
            front_fascia,
            name=f"{burner_name}_shaft_mounts_to_panel",
        )
        ctx.expect_contact(
            knob,
            shaft,
            name=f"{burner_name}_knob_contacts_shaft",
        )
        ctx.expect_origin_distance(
            knob,
            burner,
            axes="x",
            max_dist=0.005,
            name=f"{burner_name}_control_column_aligned",
        )

    left_rear_knob_pos = ctx.part_world_position(knobs["rear_left"])
    left_front_knob_pos = ctx.part_world_position(knobs["front_left"])
    right_rear_knob_pos = ctx.part_world_position(knobs["rear_right"])
    right_front_knob_pos = ctx.part_world_position(knobs["front_right"])
    left_rear_burner_pos = ctx.part_world_position(burners["rear_left"])
    left_front_burner_pos = ctx.part_world_position(burners["front_left"])
    right_rear_burner_pos = ctx.part_world_position(burners["rear_right"])
    right_front_burner_pos = ctx.part_world_position(burners["front_right"])

    if (
        left_rear_knob_pos is not None
        and left_front_knob_pos is not None
        and left_rear_burner_pos is not None
        and left_front_burner_pos is not None
    ):
        ctx.check(
            "left_control_stack_encodes_front_rear_burners",
            left_rear_knob_pos[2] > left_front_knob_pos[2] + 0.03
            and left_rear_burner_pos[1] > left_front_burner_pos[1] + 0.15,
            details="Left control column should stack rear burner above front burner.",
        )
    if (
        right_rear_knob_pos is not None
        and right_front_knob_pos is not None
        and right_rear_burner_pos is not None
        and right_front_burner_pos is not None
    ):
        ctx.check(
            "right_control_stack_encodes_front_rear_burners",
            right_rear_knob_pos[2] > right_front_knob_pos[2] + 0.03
            and right_rear_burner_pos[1] > right_front_burner_pos[1] + 0.15,
            details="Right control column should stack rear burner above front burner.",
        )

    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_pose_no_floating")
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    for control_name, joint in knob_joints.items():
        limits = joint.motion_limits
        knob = knobs[control_name]
        shaft = shafts[control_name]
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.expect_contact(
                knob,
                shaft,
                name=f"{control_name}_knob_lower_pose_on_shaft",
            )
        with ctx.pose({joint: limits.upper}):
            ctx.expect_contact(
                knob,
                shaft,
                name=f"{control_name}_knob_upper_pose_on_shaft",
            )
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"{control_name}_knob_upper_pose_no_overlap"
            )
            ctx.fail_if_isolated_parts(
                name=f"{control_name}_knob_upper_pose_no_floating"
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
