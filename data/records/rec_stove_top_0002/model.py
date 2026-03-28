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

BURNER_LAYOUT = {
    "front_left": {"xy": (-0.145, -0.090), "tray_r": 0.060, "crown_r": 0.045, "cap_r": 0.023},
    "rear_left": {"xy": (-0.145, 0.090), "tray_r": 0.052, "crown_r": 0.038, "cap_r": 0.020},
    "front_right": {"xy": (0.145, -0.090), "tray_r": 0.054, "crown_r": 0.040, "cap_r": 0.021},
    "rear_right": {"xy": (0.145, 0.090), "tray_r": 0.046, "crown_r": 0.033, "cap_r": 0.017},
}

CONTROL_LAYOUT = (
    ("rear_left", -0.170, 0.095),
    ("front_left", -0.090, 0.095),
    ("front_right", 0.090, 0.095),
    ("rear_right", 0.170, 0.095),
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _knob_mesh():
    profile = [
        (0.0, 0.000),
        (0.010, 0.000),
        (0.017, 0.002),
        (0.0225, 0.008),
        (0.0240, 0.018),
        (0.0220, 0.026),
        (0.0160, 0.032),
        (0.0, 0.032),
    ]
    return _save_mesh("utility_knob.obj", LatheGeometry(profile, segments=56))


def _rect_loop_xz(width: float, y: float, z0: float, z1: float) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    return [
        (-half_width, y, z0),
        (half_width, y, z0),
        (half_width, y, z1),
        (-half_width, y, z1),
    ]


def _control_housing_mesh():
    return _save_mesh(
        "utility_control_housing.obj",
        section_loft(
            [
                _rect_loop_xz(0.600, -0.220, 0.044, 0.082),
                _rect_loop_xz(0.612, -0.186, 0.048, 0.092),
                _rect_loop_xz(0.620, -0.150, 0.054, 0.100),
            ]
        ),
    )


def _add_panel_fastener(part, *, x: float, z: float, material, name_prefix: str) -> None:
    part.visual(
        Cylinder(radius=0.0027, length=0.010),
        origin=Origin(xyz=(x, -0.215, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{name_prefix}_stem",
    )
    part.visual(
        Cylinder(radius=0.0062, length=0.003),
        origin=Origin(xyz=(x, -0.2215, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{name_prefix}_head",
    )


def _add_plate_fastener(part, *, x: float, y: float, material, name_prefix: str) -> None:
    part.visual(
        Cylinder(radius=0.0032, length=0.010),
        origin=Origin(xyz=(x, y, 0.005)),
        material=material,
        name=f"{name_prefix}_stem",
    )
    part.visual(
        Cylinder(radius=0.0066, length=0.003),
        origin=Origin(xyz=(x, y, 0.0105)),
        material=material,
        name=f"{name_prefix}_head",
    )


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_stovetop", assets=ASSETS)

    body_paint = model.material("body_paint", rgba=(0.30, 0.32, 0.34, 1.0))
    cooktop_finish = model.material("cooktop_finish", rgba=(0.42, 0.45, 0.48, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.10, 0.10, 0.11, 1.0))
    burner_steel = model.material("burner_steel", rgba=(0.28, 0.30, 0.32, 1.0))
    shaft_metal = model.material("shaft_metal", rgba=(0.56, 0.58, 0.61, 1.0))
    fastener_zinc = model.material("fastener_zinc", rgba=(0.66, 0.68, 0.71, 1.0))
    knob_polymer = model.material("knob_polymer", rgba=(0.13, 0.14, 0.15, 1.0))
    knob_marker = model.material("knob_marker", rgba=(0.78, 0.79, 0.80, 1.0))

    knob_mesh = _knob_mesh()
    control_housing_mesh = _control_housing_mesh()

    body = model.part("body")
    body.visual(
        Box((0.620, 0.440, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=body_paint,
        name="lower_shell",
    )
    body.visual(control_housing_mesh, material=body_paint, name="control_housing")
    body.visual(
        Box((0.050, 0.080, 0.048)),
        origin=Origin(xyz=(-0.285, -0.145, 0.076)),
        material=body_paint,
        name="left_front_gusset",
    )
    body.visual(
        Box((0.050, 0.080, 0.048)),
        origin=Origin(xyz=(0.285, -0.145, 0.076)),
        material=body_paint,
        name="right_front_gusset",
    )
    body.visual(
        Box((0.580, 0.035, 0.040)),
        origin=Origin(xyz=(0.0, 0.2025, 0.100)),
        material=body_paint,
        name="rear_riser",
    )
    body.visual(
        Box((0.560, 0.025, 0.022)),
        origin=Origin(xyz=(0.0, -0.104, 0.109)),
        material=body_paint,
        name="front_guard_lip",
    )
    body.visual(
        Box((0.020, 0.380, 0.030)),
        origin=Origin(xyz=(-0.300, 0.010, 0.105)),
        material=body_paint,
        name="left_side_rail",
    )
    body.visual(
        Box((0.020, 0.380, 0.030)),
        origin=Origin(xyz=(0.300, 0.010, 0.105)),
        material=body_paint,
        name="right_side_rail",
    )
    body.visual(
        Box((0.560, 0.030, 0.025)),
        origin=Origin(xyz=(0.0, 0.170, 0.1075)),
        material=body_paint,
        name="rear_support_rail",
    )
    body.visual(
        Box((0.560, 0.020, 0.025)),
        origin=Origin(xyz=(0.0, -0.120, 0.1075)),
        material=body_paint,
        name="front_support_rail",
    )
    for index, x_pos in enumerate((-0.245, -0.085, 0.085, 0.245)):
        _add_panel_fastener(
            body,
            x=x_pos,
            z=0.084,
            material=fastener_zinc,
            name_prefix=f"panel_fastener_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.620, 0.440, 0.160)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((0.580, 0.380, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cooktop_finish,
        name="cooktop_plate",
    )
    cooktop.visual(
        Box((0.560, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, -0.175, 0.008)),
        material=cooktop_finish,
        name="front_stiffener",
    )
    cooktop.visual(
        Box((0.560, 0.030, 0.008)),
        origin=Origin(xyz=(0.0, 0.175, 0.008)),
        material=cooktop_finish,
        name="rear_stiffener",
    )
    cooktop.visual(
        Box((0.030, 0.340, 0.008)),
        origin=Origin(xyz=(-0.275, 0.0, 0.008)),
        material=cooktop_finish,
        name="left_stiffener",
    )
    cooktop.visual(
        Box((0.030, 0.340, 0.008)),
        origin=Origin(xyz=(0.275, 0.0, 0.008)),
        material=cooktop_finish,
        name="right_stiffener",
    )
    for burner_name, burner_spec in BURNER_LAYOUT.items():
        bx, by = burner_spec["xy"]
        cooktop.visual(
            Cylinder(radius=burner_spec["tray_r"] + 0.010, length=0.003),
            origin=Origin(xyz=(bx, by, 0.0105)),
            material=cooktop_finish,
            name=f"{burner_name}_seat_ring",
        )
    for index, (x_pos, y_pos) in enumerate(
        ((-0.245, -0.155), (0.245, -0.155), (-0.245, 0.155), (0.245, 0.155))
    ):
        _add_plate_fastener(
            cooktop,
            x=x_pos,
            y=y_pos,
            material=fastener_zinc,
            name_prefix=f"plate_fastener_{index}",
        )
    cooktop.inertial = Inertial.from_geometry(
        Box((0.580, 0.380, 0.016)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    model.articulation(
        "body_to_cooktop",
        ArticulationType.FIXED,
        parent=body,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )

    for burner_name, burner_spec in BURNER_LAYOUT.items():
        burner = model.part(f"burner_{burner_name}")
        tray_r = burner_spec["tray_r"]
        crown_r = burner_spec["crown_r"]
        cap_r = burner_spec["cap_r"]
        burner.visual(
            Cylinder(radius=tray_r, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=burner_steel,
            name="burner_tray",
        )
        burner.visual(
            Cylinder(radius=crown_r, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=burner_steel,
            name="burner_crown",
        )
        burner.visual(
            Cylinder(radius=cap_r, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, 0.0175)),
            material=cast_iron,
            name="burner_cap",
        )
        lug_offset = crown_r - 0.006
        burner.visual(
            Box((0.012, 0.005, 0.004)),
            origin=Origin(xyz=(lug_offset, 0.0, 0.014)),
            material=burner_steel,
            name="lug_pos_x",
        )
        burner.visual(
            Box((0.012, 0.005, 0.004)),
            origin=Origin(xyz=(-lug_offset, 0.0, 0.014)),
            material=burner_steel,
            name="lug_neg_x",
        )
        burner.visual(
            Box((0.005, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, lug_offset, 0.014)),
            material=burner_steel,
            name="lug_pos_y",
        )
        burner.visual(
            Box((0.005, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, -lug_offset, 0.014)),
            material=burner_steel,
            name="lug_neg_y",
        )
        burner.inertial = Inertial.from_geometry(
            Cylinder(radius=tray_r, length=0.022),
            mass=0.45,
            origin=Origin(xyz=(0.0, 0.0, 0.011)),
        )
        bx, by = burner_spec["xy"]
        model.articulation(
            f"cooktop_to_burner_{burner_name}",
            ArticulationType.FIXED,
            parent=cooktop,
            child=burner,
            origin=Origin(xyz=(bx, by, 0.012)),
        )

    grate_specs = {
        "left": {"x": -0.145, "y": 0.0},
        "right": {"x": 0.145, "y": 0.0},
    }
    for grate_name, grate_spec in grate_specs.items():
        grate = model.part(f"grate_{grate_name}")
        for ix in (-0.097, 0.097):
            for iy in (-0.128, 0.128):
                grate.visual(
                    Box((0.016, 0.016, 0.030)),
                    origin=Origin(xyz=(ix, iy, 0.015)),
                    material=cast_iron,
                    name=f"foot_{'p' if ix > 0 else 'n'}x_{'p' if iy > 0 else 'n'}y",
                )
        grate.visual(
            Box((0.018, 0.270, 0.012)),
            origin=Origin(xyz=(-0.097, 0.0, 0.031)),
            material=cast_iron,
            name="left_side_bar",
        )
        grate.visual(
            Box((0.018, 0.270, 0.012)),
            origin=Origin(xyz=(0.097, 0.0, 0.031)),
            material=cast_iron,
            name="right_side_bar",
        )
        grate.visual(
            Box((0.210, 0.018, 0.012)),
            origin=Origin(xyz=(0.0, -0.128, 0.031)),
            material=cast_iron,
            name="front_bar",
        )
        grate.visual(
            Box((0.210, 0.018, 0.012)),
            origin=Origin(xyz=(0.0, 0.128, 0.031)),
            material=cast_iron,
            name="rear_bar",
        )
        grate.visual(
            Box((0.018, 0.220, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.031)),
            material=cast_iron,
            name="center_spine",
        )
        grate.visual(
            Box((0.180, 0.014, 0.010)),
            origin=Origin(xyz=(0.0, -0.092, 0.031)),
            material=cast_iron,
            name="front_bridge",
        )
        grate.visual(
            Box((0.180, 0.014, 0.010)),
            origin=Origin(xyz=(0.0, 0.092, 0.031)),
            material=cast_iron,
            name="rear_bridge",
        )
        grate.visual(
            Box((0.120, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.031)),
            material=cast_iron,
            name="service_crossbar",
        )
        grate.inertial = Inertial.from_geometry(
            Box((0.230, 0.300, 0.040)),
            mass=1.7,
            origin=Origin(xyz=(0.0, 0.0, 0.020)),
        )
        model.articulation(
            f"cooktop_to_grate_{grate_name}",
            ArticulationType.FIXED,
            parent=cooktop,
            child=grate,
            origin=Origin(xyz=(grate_spec["x"], grate_spec["y"], 0.012)),
        )

    for control_name, x_pos, z_pos in CONTROL_LAYOUT:
        shaft = model.part(f"shaft_{control_name}")
        shaft.visual(
            Cylinder(radius=0.0045, length=0.022),
            origin=Origin(xyz=(0.0, 0.011, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=shaft_metal,
            name="shaft_stem",
        )
        shaft.visual(
            Cylinder(radius=0.011, length=0.008),
            origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=shaft_metal,
            name="bushing_collar",
        )
        shaft.visual(
            Box((0.024, 0.002, 0.024)),
            origin=Origin(xyz=(0.0, 0.029, 0.0)),
            material=fastener_zinc,
            name="mount_plate",
        )
        shaft.inertial = Inertial.from_geometry(
            Box((0.024, 0.030, 0.024)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.015, 0.0)),
        )
        model.articulation(
            f"body_to_shaft_{control_name}",
            ArticulationType.FIXED,
            parent=body,
            child=shaft,
            origin=Origin(xyz=(x_pos, -0.250, z_pos)),
        )

        knob = model.part(f"knob_{control_name}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=knob_polymer,
            name="knob_shell",
        )
        knob.visual(
            Box((0.004, 0.010, 0.003)),
            origin=Origin(xyz=(0.0, -0.025, 0.0215)),
            material=knob_marker,
            name="pointer_ridge",
        )
        knob.inertial = Inertial.from_geometry(
            Box((0.048, 0.034, 0.035)),
            mass=0.09,
            origin=Origin(xyz=(0.0, -0.017, 0.0175)),
        )
        model.articulation(
            f"shaft_{control_name}_to_knob_{control_name}",
            ArticulationType.REVOLUTE,
            parent=shaft,
            child=knob,
            origin=Origin(),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.8,
                velocity=4.5,
                lower=-2.35,
                upper=2.35,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    cooktop = object_model.get_part("cooktop")
    grate_left = object_model.get_part("grate_left")
    grate_right = object_model.get_part("grate_right")
    burners = {name: object_model.get_part(f"burner_{name}") for name in BURNER_LAYOUT}
    shafts = {name: object_model.get_part(f"shaft_{name}") for name, _, _ in CONTROL_LAYOUT}
    knobs = {name: object_model.get_part(f"knob_{name}") for name, _, _ in CONTROL_LAYOUT}
    knob_joints = {
        name: object_model.get_articulation(f"shaft_{name}_to_knob_{name}")
        for name, _, _ in CONTROL_LAYOUT
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

    ctx.expect_contact(cooktop, body, name="cooktop_seats_on_body")
    ctx.expect_overlap(cooktop, body, axes="xy", min_overlap=0.34, name="cooktop_footprint_covers_body")
    ctx.expect_gap(
        cooktop,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        name="cooktop_rest_gap",
    )

    for burner_name, burner in burners.items():
        ctx.expect_contact(burner, cooktop, name=f"{burner_name}_contacts_cooktop")
        ctx.expect_within(
            burner,
            cooktop,
            axes="xy",
            margin=0.015,
            name=f"{burner_name}_within_cooktop",
        )
        ctx.expect_overlap(
            burner,
            cooktop,
            axes="xy",
            min_overlap=0.070,
            name=f"{burner_name}_seat_overlap",
        )

    for grate_name, grate, covered in (
        ("left", grate_left, ("front_left", "rear_left")),
        ("right", grate_right, ("front_right", "rear_right")),
    ):
        ctx.expect_contact(grate, cooktop, name=f"grate_{grate_name}_rests_on_cooktop")
        for burner_name in covered:
            ctx.expect_overlap(
                grate,
                burners[burner_name],
                axes="xy",
                min_overlap=0.070,
                name=f"grate_{grate_name}_covers_{burner_name}",
            )

    ctx.expect_gap(
        grate_right,
        grate_left,
        axis="x",
        min_gap=0.012,
        name="service_gap_between_grates",
    )

    for control_name, _, _ in CONTROL_LAYOUT:
        shaft = shafts[control_name]
        knob = knobs[control_name]
        joint = knob_joints[control_name]
        pointer = knob.get_visual("pointer_ridge")

        ctx.expect_contact(shaft, body, name=f"{control_name}_shaft_mounts_to_body")
        ctx.expect_contact(knob, shaft, name=f"{control_name}_knob_contacts_shaft")
        ctx.expect_origin_distance(
            knob,
            shaft,
            axes="xz",
            max_dist=1e-6,
            name=f"{control_name}_knob_axis_alignment",
        )

        rest_aabb = ctx.part_element_world_aabb(knob, elem=pointer)
        turned_aabb = None
        with ctx.pose({joint: 1.2}):
            ctx.expect_contact(knob, shaft, name=f"{control_name}_contact_when_turned")
            turned_aabb = ctx.part_element_world_aabb(knob, elem=pointer)

        sweep_amount = 0.0
        if rest_aabb is not None and turned_aabb is not None:
            rest_center = _aabb_center(rest_aabb)
            turned_center = _aabb_center(turned_aabb)
            sweep_amount = abs(rest_center[0] - turned_center[0]) + abs(rest_center[2] - turned_center[2])
        ctx.check(
            f"{control_name}_pointer_sweeps_with_rotation",
            sweep_amount > 0.012,
            details=f"pointer movement was {sweep_amount:.5f} m",
        )

    burner_positions = {name: ctx.part_world_position(part) for name, part in burners.items()}
    knob_positions = {name: ctx.part_world_position(part) for name, part in knobs.items()}

    left_burner_x = sum(burner_positions[name][0] for name in ("front_left", "rear_left")) / 2.0
    right_burner_x = sum(burner_positions[name][0] for name in ("front_right", "rear_right")) / 2.0
    left_knob_x = sum(knob_positions[name][0] for name in ("front_left", "rear_left")) / 2.0
    right_knob_x = sum(knob_positions[name][0] for name in ("front_right", "rear_right")) / 2.0

    ctx.check(
        "left_controls_group_under_left_burners",
        abs(left_knob_x - left_burner_x) < 0.09,
        details=f"left burner mean x={left_burner_x:.3f}, left knob mean x={left_knob_x:.3f}",
    )
    ctx.check(
        "right_controls_group_under_right_burners",
        abs(right_knob_x - right_burner_x) < 0.09,
        details=f"right burner mean x={right_burner_x:.3f}, right knob mean x={right_knob_x:.3f}",
    )
    ctx.check(
        "front_controls_are_below_cooktop",
        all(knob_positions[name][2] < burner_positions[name][2] - 0.02 for name in BURNER_LAYOUT),
        details=(
            "expected every knob shaft line to sit well below its burner module; "
            f"burner z values={burner_positions}, knob z values={knob_positions}"
        ),
    )
    ctx.check(
        "burner_rows_read_front_to_back",
        burner_positions["front_left"][1] < burner_positions["rear_left"][1] - 0.12
        and burner_positions["front_right"][1] < burner_positions["rear_right"][1] - 0.12,
        details=f"burner positions={burner_positions}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
