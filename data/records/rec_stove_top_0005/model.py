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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

TOP_WIDTH = 0.90
TOP_DEPTH = 0.52
TOP_RADIUS = 0.035
DECK_WIDTH = 0.838
DECK_DEPTH = 0.458
DECK_RADIUS = 0.024
FRAME_THICKNESS = 0.0045
DECK_THICKNESS = 0.0055

BURNER_SPECS = (
    {
        "key": "rear_left",
        "xy": (-0.23, 0.115),
        "base_radius": 0.044,
        "head_radius": 0.032,
        "cap_radius": 0.023,
        "cap_height": 0.006,
        "control_x": -0.26,
    },
    {
        "key": "front_left",
        "xy": (-0.23, -0.105),
        "base_radius": 0.048,
        "head_radius": 0.035,
        "cap_radius": 0.026,
        "cap_height": 0.006,
        "control_x": -0.15,
    },
    {
        "key": "center",
        "xy": (0.0, 0.020),
        "base_radius": 0.066,
        "head_radius": 0.050,
        "cap_radius": 0.036,
        "cap_height": 0.007,
        "control_x": 0.0,
    },
    {
        "key": "front_right",
        "xy": (0.23, -0.105),
        "base_radius": 0.048,
        "head_radius": 0.035,
        "cap_radius": 0.026,
        "cap_height": 0.006,
        "control_x": 0.15,
    },
    {
        "key": "rear_right",
        "xy": (0.23, 0.115),
        "base_radius": 0.044,
        "head_radius": 0.032,
        "cap_radius": 0.023,
        "cap_height": 0.006,
        "control_x": 0.26,
    },
)

GRATE_SPECS = (
    {
        "key": "left",
        "xy": (-0.23, 0.005),
        "size": (0.196, 0.372),
        "supports": ("rear_left", "front_left"),
    },
    {
        "key": "center",
        "xy": (0.0, 0.020),
        "size": (0.148, 0.148),
        "supports": ("center",),
    },
    {
        "key": "right",
        "xy": (0.23, 0.005),
        "size": (0.196, 0.372),
        "supports": ("rear_right", "front_right"),
    },
)

CONTROL_ORDER = tuple(spec["key"] for spec in BURNER_SPECS)


def _burner_part_name(key: str) -> str:
    return f"burner_{key}"


def _grate_part_name(key: str) -> str:
    return f"grate_{key}"


def _shaft_part_name(key: str) -> str:
    return f"shaft_{key}"


def _knob_part_name(key: str) -> str:
    return f"knob_{key}"


def _shaft_joint_name(key: str) -> str:
    return f"fascia_to_shaft_{key}"


def _knob_joint_name(key: str) -> str:
    return f"shaft_to_knob_{key}"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stove_top", assets=ASSETS)

    satin_steel = model.material("satin_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.79, 0.80, 0.81, 1.0))
    matte_graphite = model.material("matte_graphite", rgba=(0.14, 0.15, 0.16, 1.0))
    matte_black = model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    satin_black = model.material("satin_black", rgba=(0.19, 0.20, 0.21, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    burner_metal = model.material("burner_metal", rgba=(0.48, 0.49, 0.50, 1.0))
    ceramic = model.material("ceramic", rgba=(0.90, 0.90, 0.88, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.84, 0.44, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=matte_graphite,
        name="undertray",
    )
    chassis.visual(
        Box((0.82, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, -0.220, -0.020)),
        material=matte_graphite,
        name="front_brace",
    )
    chassis.visual(
        Box((0.044, 0.270, 0.018)),
        origin=Origin(xyz=(-0.398, 0.0, -0.009)),
        material=matte_graphite,
        name="left_mount_rail",
    )
    chassis.visual(
        Box((0.044, 0.270, 0.018)),
        origin=Origin(xyz=(0.398, 0.0, -0.009)),
        material=matte_graphite,
        name="right_mount_rail",
    )
    chassis.visual(
        Box((0.180, 0.044, 0.024)),
        origin=Origin(xyz=(0.0, 0.202, -0.012)),
        material=matte_graphite,
        name="rear_service_box",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((0.84, 0.46, 0.12)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
    )

    top_plate = model.part("top_plate")
    deck_profile = rounded_rect_profile(DECK_WIDTH, DECK_DEPTH, DECK_RADIUS)
    top_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(TOP_WIDTH, TOP_DEPTH, TOP_RADIUS),
            [deck_profile],
            FRAME_THICKNESS,
            center=True,
        ),
        ASSETS.mesh_path("premium_stove_top_frame.obj"),
    )
    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry(deck_profile, DECK_THICKNESS, center=True),
        ASSETS.mesh_path("premium_stove_top_deck.obj"),
    )
    top_plate.visual(
        top_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, FRAME_THICKNESS * 0.5)),
        material=satin_steel,
        name="frame_ring",
    )
    top_plate.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, DECK_THICKNESS * 0.5)),
        material=matte_black,
        name="cook_surface",
    )
    top_plate.inertial = Inertial.from_geometry(
        Box((TOP_WIDTH, TOP_DEPTH, DECK_THICKNESS)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, DECK_THICKNESS * 0.5)),
    )
    model.articulation(
        "chassis_to_top_plate",
        ArticulationType.FIXED,
        parent=chassis,
        child=top_plate,
        origin=Origin(),
    )

    fascia = model.part("fascia")
    fascia.visual(
        Box((0.80, 0.022, 0.036)),
        material=satin_black,
        name="fascia_panel",
    )
    shaft_origins: dict[str, Origin] = {}
    for spec in BURNER_SPECS:
        shaft_origins[spec["key"]] = Origin(xyz=(float(spec["control_x"]), -0.011, 0.0))
    for spec in BURNER_SPECS:
        key = spec["key"]
        fascia.visual(
            Cylinder(radius=0.022, length=0.004),
            origin=Origin(
                xyz=(spec["control_x"], -0.011, -0.002),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=brushed_aluminum,
            name=f"bezel_{key}",
        )
        fascia.visual(
            Box((0.014, 0.0015, 0.004)),
            origin=Origin(xyz=(spec["control_x"], -0.0118, 0.010)),
            material=brushed_aluminum,
            name=f"tick_{key}",
        )
    fascia.inertial = Inertial.from_geometry(
        Box((0.80, 0.024, 0.038)),
        mass=1.0,
    )
    model.articulation(
        "chassis_to_fascia",
        ArticulationType.FIXED,
        parent=chassis,
        child=fascia,
        origin=Origin(xyz=(0.0, -0.261, -0.018)),
    )

    def add_burner(spec: dict[str, object]) -> None:
        key = str(spec["key"])
        base_radius = float(spec["base_radius"])
        head_radius = float(spec["head_radius"])
        cap_radius = float(spec["cap_radius"])
        cap_height = float(spec["cap_height"])
        burner = model.part(_burner_part_name(key))
        burner.visual(
            Cylinder(radius=base_radius + 0.006, length=0.002),
            origin=Origin(xyz=(0.0, 0.0, 0.0065)),
            material=satin_steel,
            name="trim_ring",
        )
        burner.visual(
            Cylinder(radius=base_radius, length=0.009),
            origin=Origin(xyz=(0.0, 0.0, 0.0100)),
            material=burner_metal,
            name="skirt",
        )
        burner.visual(
            Cylinder(radius=head_radius, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, 0.0150)),
            material=satin_black,
            name="head",
        )
        burner.visual(
            Cylinder(radius=cap_radius, length=cap_height),
            origin=Origin(xyz=(0.0, 0.0, 0.0205 + 0.5 * (cap_height - 0.006))),
            material=matte_black,
            name="cap",
        )
        burner.visual(
            Box((0.005, 0.010, 0.010)),
            origin=Origin(xyz=(base_radius * 0.62, -0.008, 0.0105)),
            material=ceramic,
            name="igniter",
        )
        burner.visual(
            Cylinder(radius=0.0025, length=0.010),
            origin=Origin(xyz=(-base_radius * 0.58, 0.010, 0.0105)),
            material=burner_metal,
            name="thermocouple",
        )
        burner.inertial = Inertial.from_geometry(
            Cylinder(radius=base_radius + 0.008, length=0.026),
            mass=0.22 if key != "center" else 0.34,
            origin=Origin(xyz=(0.0, 0.0, 0.014)),
        )
        burner_x, burner_y = spec["xy"]
        model.articulation(
            f"top_plate_to_{_burner_part_name(key)}",
            ArticulationType.FIXED,
            parent=top_plate,
            child=burner,
            origin=Origin(xyz=(float(burner_x), float(burner_y), 0.0)),
        )

    def add_grate(spec: dict[str, object]) -> None:
        key = str(spec["key"])
        width, depth = spec["size"]
        grate = model.part(_grate_part_name(key))
        frame_thickness = 0.012
        frame_width = 0.010
        support_height = 0.028
        support_center_z = 0.0195
        frame_center_z = 0.039
        foot_x = width * 0.41
        foot_y = depth * 0.41

        if key in {"left", "right"}:
            grate.visual(
                Box((frame_width, depth, frame_thickness)),
                origin=Origin(xyz=(-width * 0.40, 0.0, frame_center_z)),
                material=cast_iron,
                name="frame_left",
            )
            grate.visual(
                Box((frame_width, depth, frame_thickness)),
                origin=Origin(xyz=(width * 0.40, 0.0, frame_center_z)),
                material=cast_iron,
                name="frame_right",
            )
            grate.visual(
                Box((width, frame_width, frame_thickness)),
                origin=Origin(xyz=(0.0, -depth * 0.35, frame_center_z)),
                material=cast_iron,
                name="bridge_front",
            )
            grate.visual(
                Box((width, frame_width, frame_thickness)),
                origin=Origin(xyz=(0.0, depth * 0.35, frame_center_z)),
                material=cast_iron,
                name="bridge_rear",
            )
            grate.visual(
                Box((width * 0.86, frame_width, frame_thickness)),
                origin=Origin(xyz=(0.0, 0.0, frame_center_z)),
                material=cast_iron,
                name="center_bridge",
            )
        else:
            grate.visual(
                Box((width, frame_width, frame_thickness)),
                origin=Origin(xyz=(0.0, 0.0, frame_center_z)),
                material=cast_iron,
                name="bridge_x",
            )
            grate.visual(
                Box((frame_width, depth, frame_thickness)),
                origin=Origin(xyz=(0.0, 0.0, frame_center_z)),
                material=cast_iron,
                name="bridge_y",
            )
            grate.visual(
                Box((width, frame_width, frame_thickness)),
                origin=Origin(xyz=(0.0, -depth * 0.40, frame_center_z)),
                material=cast_iron,
                name="frame_front",
            )
            grate.visual(
                Box((width, frame_width, frame_thickness)),
                origin=Origin(xyz=(0.0, depth * 0.40, frame_center_z)),
                material=cast_iron,
                name="frame_rear",
            )
            grate.visual(
                Box((frame_width, depth, frame_thickness)),
                origin=Origin(xyz=(-width * 0.40, 0.0, frame_center_z)),
                material=cast_iron,
                name="frame_left",
            )
            grate.visual(
                Box((frame_width, depth, frame_thickness)),
                origin=Origin(xyz=(width * 0.40, 0.0, frame_center_z)),
                material=cast_iron,
                name="frame_right",
            )

        for suffix, sx, sy in (
            ("front_left", -foot_x, -foot_y),
            ("front_right", foot_x, -foot_y),
            ("rear_left", -foot_x, foot_y),
            ("rear_right", foot_x, foot_y),
        ):
            grate.visual(
                Box((0.018, 0.018, support_height)),
                origin=Origin(xyz=(sx, sy, support_center_z)),
                material=cast_iron,
                name=f"foot_{suffix}",
            )

        grate.inertial = Inertial.from_geometry(
            Box((float(width), float(depth), 0.046)),
            mass=0.9 if key != "center" else 0.55,
            origin=Origin(xyz=(0.0, 0.0, 0.022)),
        )
        grate_x, grate_y = spec["xy"]
        model.articulation(
            f"top_plate_to_{_grate_part_name(key)}",
            ArticulationType.FIXED,
            parent=top_plate,
            child=grate,
            origin=Origin(xyz=(float(grate_x), float(grate_y), 0.0)),
        )

    def add_control(spec: dict[str, object]) -> None:
        key = str(spec["key"])
        shaft = model.part(_shaft_part_name(key))
        shaft.visual(
            Cylinder(radius=0.004, length=0.021),
            origin=Origin(xyz=(0.0, -0.0105, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_aluminum,
            name="shaft_core",
        )
        shaft.visual(
            Cylinder(radius=0.0065, length=0.004),
            origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_aluminum,
            name="shaft_shoulder",
        )
        shaft.inertial = Inertial.from_geometry(
            Box((0.013, 0.021, 0.013)),
            mass=0.024,
            origin=Origin(xyz=(0.0, -0.0105, 0.0)),
        )
        model.articulation(
            _shaft_joint_name(key),
            ArticulationType.REVOLUTE,
            parent=fascia,
            child=shaft,
            origin=shaft_origins[key],
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.35,
                velocity=2.2,
                lower=0.0,
                upper=math.radians(270.0),
            ),
        )

        knob = model.part(_knob_part_name(key))
        knob.visual(
            Cylinder(radius=0.0185, length=0.006),
            origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name="rear_hub",
        )
        knob.visual(
            Cylinder(radius=0.0225, length=0.012),
            origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=matte_graphite,
            name="body_barrel",
        )
        knob.visual(
            Cylinder(radius=0.0195, length=0.010),
            origin=Origin(xyz=(0.0, -0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_black,
            name="front_crown",
        )
        knob.visual(
            Box((0.009, 0.017, 0.003)),
            origin=Origin(xyz=(0.0, -0.023, 0.0185)),
            material=brushed_aluminum,
            name="indicator_ridge",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.046, 0.028, 0.046)),
            mass=0.085,
            origin=Origin(xyz=(0.0, -0.015, 0.0)),
        )
        model.articulation(
            _knob_joint_name(key),
            ArticulationType.FIXED,
            parent=shaft,
            child=knob,
            origin=Origin(xyz=(0.0, -0.021, 0.0)),
        )

    for burner_spec in BURNER_SPECS:
        add_burner(burner_spec)

    for grate_spec in GRATE_SPECS:
        add_grate(grate_spec)

    for burner_spec in BURNER_SPECS:
        add_control(burner_spec)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(max_pose_samples=12)
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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_fixed=True,
    )

    chassis = object_model.get_part("chassis")
    top_plate = object_model.get_part("top_plate")
    fascia = object_model.get_part("fascia")

    burners = {spec["key"]: object_model.get_part(_burner_part_name(spec["key"])) for spec in BURNER_SPECS}
    grates = {spec["key"]: object_model.get_part(_grate_part_name(spec["key"])) for spec in GRATE_SPECS}
    shafts = {spec["key"]: object_model.get_part(_shaft_part_name(spec["key"])) for spec in BURNER_SPECS}
    knobs = {spec["key"]: object_model.get_part(_knob_part_name(spec["key"])) for spec in BURNER_SPECS}
    shaft_joints = {spec["key"]: object_model.get_articulation(_shaft_joint_name(spec["key"])) for spec in BURNER_SPECS}

    ctx.expect_contact(top_plate, chassis, name="top_plate_seated_on_chassis")
    ctx.expect_contact(fascia, chassis, name="fascia_braced_to_chassis")
    ctx.expect_within(top_plate, chassis, axes="xy", margin=0.05, name="top_plate_over_chassis")

    for spec in BURNER_SPECS:
        key = spec["key"]
        burner = burners[key]
        shaft = shafts[key]
        knob = knobs[key]

        ctx.expect_contact(burner, top_plate, name=f"{key}_burner_mount_contact")
        ctx.expect_within(burner, top_plate, axes="xy", margin=0.0, name=f"{key}_burner_inside_plate")
        ctx.expect_contact(shaft, fascia, name=f"{key}_shaft_contacts_bezel")
        ctx.expect_contact(knob, shaft, name=f"{key}_knob_on_shaft")
        ctx.expect_gap(
            fascia,
            knob,
            axis="y",
            min_gap=0.017,
            max_gap=0.021,
            name=f"{key}_knob_front_clearance",
        )

        burner_pos = ctx.part_world_position(burner)
        knob_pos = ctx.part_world_position(knob)
        assert burner_pos is not None
        assert knob_pos is not None
        ctx.check(
            f"{key}_control_tracks_burner_x",
            abs(knob_pos[0] - burner_pos[0]) <= (0.001 if key == "center" else 0.085),
            details=f"burner_x={burner_pos[0]:.4f}, knob_x={knob_pos[0]:.4f}",
        )

        joint = shaft_joints[key]
        limits = joint.motion_limits
        assert limits is not None
        ctx.check(
            f"{key}_shaft_axis_is_front_back",
            abs(joint.axis[1]) > 0.99 and abs(joint.axis[0]) < 1e-6 and abs(joint.axis[2]) < 1e-6,
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"{key}_shaft_limits_realistic",
            limits.lower == 0.0 and limits.upper is not None and 4.5 <= limits.upper <= 4.8,
            details=f"limits=({limits.lower}, {limits.upper})",
        )
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{key}_lower_pose_clear")
            ctx.fail_if_isolated_parts(name=f"{key}_lower_pose_grounded")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{key}_upper_pose_clear")
            ctx.fail_if_isolated_parts(name=f"{key}_upper_pose_grounded")

    ctx.expect_origin_gap(
        burners["rear_left"],
        burners["front_left"],
        axis="y",
        min_gap=0.20,
        max_gap=0.24,
        name="left_burners_front_rear_spacing",
    )
    ctx.expect_origin_gap(
        burners["rear_right"],
        burners["front_right"],
        axis="y",
        min_gap=0.20,
        max_gap=0.24,
        name="right_burners_front_rear_spacing",
    )

    left_front_pos = ctx.part_world_position(burners["front_left"])
    right_front_pos = ctx.part_world_position(burners["front_right"])
    left_rear_pos = ctx.part_world_position(burners["rear_left"])
    right_rear_pos = ctx.part_world_position(burners["rear_right"])
    center_pos = ctx.part_world_position(burners["center"])
    assert left_front_pos is not None
    assert right_front_pos is not None
    assert left_rear_pos is not None
    assert right_rear_pos is not None
    assert center_pos is not None
    ctx.check(
        "front_burners_symmetric",
        abs(left_front_pos[0] + right_front_pos[0]) <= 0.002 and abs(left_front_pos[1] - right_front_pos[1]) <= 0.002,
        details=f"front_left={left_front_pos}, front_right={right_front_pos}",
    )
    ctx.check(
        "rear_burners_symmetric",
        abs(left_rear_pos[0] + right_rear_pos[0]) <= 0.002 and abs(left_rear_pos[1] - right_rear_pos[1]) <= 0.002,
        details=f"rear_left={left_rear_pos}, rear_right={right_rear_pos}",
    )
    ctx.check(
        "center_burner_centered",
        abs(center_pos[0]) <= 0.002 and -0.005 <= center_pos[1] <= 0.045,
        details=f"center={center_pos}",
    )

    for grate_spec in GRATE_SPECS:
        grate = grates[grate_spec["key"]]
        ctx.expect_contact(grate, top_plate, name=f"{grate_spec['key']}_grate_lands_on_plate")
        ctx.expect_within(grate, top_plate, axes="xy", margin=0.0, name=f"{grate_spec['key']}_grate_within_plate")
        for supported in grate_spec["supports"]:
            ctx.expect_overlap(
                grate,
                burners[supported],
                axes="xy",
                min_overlap=0.07 if supported != "center" else 0.10,
                name=f"{grate_spec['key']}_covers_{supported}",
            )

    ctx.expect_gap(
        grates["left"],
        burners["front_left"],
        axis="z",
        positive_elem="bridge_front",
        negative_elem="cap",
        min_gap=0.008,
        max_gap=0.016,
        name="left_front_pot_support_clearance",
    )
    ctx.expect_gap(
        grates["left"],
        burners["rear_left"],
        axis="z",
        positive_elem="bridge_rear",
        negative_elem="cap",
        min_gap=0.008,
        max_gap=0.016,
        name="left_rear_pot_support_clearance",
    )
    ctx.expect_gap(
        grates["right"],
        burners["front_right"],
        axis="z",
        positive_elem="bridge_front",
        negative_elem="cap",
        min_gap=0.008,
        max_gap=0.016,
        name="right_front_pot_support_clearance",
    )
    ctx.expect_gap(
        grates["right"],
        burners["rear_right"],
        axis="z",
        positive_elem="bridge_rear",
        negative_elem="cap",
        min_gap=0.008,
        max_gap=0.016,
        name="right_rear_pot_support_clearance",
    )
    ctx.expect_gap(
        grates["center"],
        burners["center"],
        axis="z",
        positive_elem="bridge_x",
        negative_elem="cap",
        min_gap=0.007,
        max_gap=0.016,
        name="center_support_clearance",
    )

    control_positions = []
    for key in CONTROL_ORDER:
        knob_pos = ctx.part_world_position(knobs[key])
        assert knob_pos is not None
        control_positions.append(knob_pos[0])
    ctx.check(
        "controls_ordered_left_to_right",
        all(next_x > current_x for current_x, next_x in zip(control_positions, control_positions[1:])),
        details=f"control_x={control_positions}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
