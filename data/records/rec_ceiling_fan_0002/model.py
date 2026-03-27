from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _radial_pattern(base: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base.copy().rotate_z(angle_offset + (index * math.tau / count)))
    return patterned


def _smooth_closed_outline(points: list[tuple[float, float]], samples_per_segment: int = 10):
    return sample_catmull_rom_spline_2d(
        points,
        samples_per_segment=samples_per_segment,
        closed=True,
    )


def _build_canopy_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.392),
            (0.022, 0.392),
            (0.055, 0.384),
            (0.071, 0.366),
            (0.068, 0.344),
            (0.050, 0.326),
            (0.022, 0.320),
            (0.0, 0.320),
        ],
        segments=64,
    )


def _build_motor_shell_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.0),
            (0.034, 0.0),
            (0.046, -0.012),
            (0.073, -0.032),
            (0.102, -0.060),
            (0.118, -0.096),
            (0.116, -0.138),
            (0.098, -0.174),
            (0.076, -0.196),
            (0.0, -0.196),
        ],
        segments=72,
    )


def _build_blade_bracket_mesh() -> MeshGeometry:
    bracket_outline = _smooth_closed_outline(
        [
            (-0.086, -0.012),
            (-0.066, -0.015),
            (-0.034, -0.019),
            (0.002, -0.023),
            (0.032, -0.026),
            (0.056, -0.028),
            (0.056, 0.028),
            (0.032, 0.026),
            (0.002, 0.023),
            (-0.034, 0.019),
            (-0.066, 0.015),
            (-0.086, 0.012),
        ],
        samples_per_segment=8,
    )
    bracket_plate = ExtrudeGeometry.centered(bracket_outline, 0.008)
    bracket_plate.translate(0.0, 0.0, -0.004)
    return bracket_plate


def _build_blade_mesh() -> MeshGeometry:
    blade_outline = [
        (-0.010, -0.022),
        (0.018, -0.026),
        (0.074, -0.033),
        (0.150, -0.043),
        (0.242, -0.051),
        (0.332, -0.053),
        (0.396, -0.047),
        (0.438, -0.032),
        (0.454, 0.0),
        (0.438, 0.032),
        (0.396, 0.047),
        (0.332, 0.053),
        (0.242, 0.051),
        (0.150, 0.043),
        (0.074, 0.033),
        (0.018, 0.026),
        (-0.010, 0.022),
    ]
    return ExtrudeGeometry.centered(blade_outline, 0.010)


def _build_light_fitter_mesh() -> MeshGeometry:
    return LatheGeometry(
        [
            (0.0, 0.0),
            (0.034, 0.0),
            (0.047, -0.008),
            (0.055, -0.018),
            (0.052, -0.028),
            (0.0, -0.028),
        ],
        segments=56,
    )


def _build_rotor_ring_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.062, -0.020),
            (0.090, -0.020),
            (0.108, -0.014),
            (0.114, -0.004),
            (0.110, 0.0),
            (0.066, 0.0),
        ],
        [
            (0.054, -0.020),
            (0.078, -0.020),
            (0.094, -0.014),
            (0.100, -0.004),
            (0.098, 0.0),
            (0.058, 0.0),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_globe_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.046, -0.028),
            (0.068, -0.044),
            (0.094, -0.082),
            (0.102, -0.112),
            (0.084, -0.136),
            (0.046, -0.152),
            (0.014, -0.160),
        ],
        [
            (0.041, -0.029),
            (0.060, -0.044),
            (0.082, -0.080),
            (0.090, -0.110),
            (0.074, -0.132),
            (0.040, -0.146),
            (0.0, -0.153),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tropical_ceiling_fan", assets=ASSETS)

    oil_rubbed_bronze = model.material("oil_rubbed_bronze", rgba=(0.23, 0.16, 0.09, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.55, 0.35, 0.18, 1.0))
    opal_glass = model.material("opal_glass", rgba=(0.93, 0.92, 0.88, 0.42))

    canopy_mesh = _save_mesh(_build_canopy_mesh(), "tropical_fan_canopy.obj")
    motor_shell_mesh = _save_mesh(_build_motor_shell_mesh(), "tropical_fan_motor_shell.obj")
    bracket_mesh = _save_mesh(_build_blade_bracket_mesh(), "tropical_fan_blade_bracket.obj")
    blade_mesh = _save_mesh(_build_blade_mesh(), "tropical_fan_blade.obj")
    light_fitter_mesh = _save_mesh(_build_light_fitter_mesh(), "tropical_fan_light_fitter.obj")
    rotor_ring_mesh = _save_mesh(_build_rotor_ring_mesh(), "tropical_fan_rotor_ring.obj")
    globe_shell_mesh = _save_mesh(_build_globe_shell_mesh(), "tropical_fan_globe_shell.obj")

    mount = model.part("mount")
    mount.visual(canopy_mesh, material=oil_rubbed_bronze, name="canopy_shell")
    mount.visual(
        Cylinder(radius=0.0125, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=oil_rubbed_bronze,
        name="downrod_tube",
    )
    mount.visual(
        Cylinder(radius=0.026, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1275)),
        material=oil_rubbed_bronze,
        name="lower_collar",
    )
    mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.280),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
    )

    motor = model.part("motor_housing")
    motor.visual(motor_shell_mesh, material=oil_rubbed_bronze, name="housing_shell")
    motor.visual(
        Cylinder(radius=0.033, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=oil_rubbed_bronze,
        name="top_collar",
    )
    motor.visual(
        Cylinder(radius=0.122, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.094)),
        material=oil_rubbed_bronze,
        name="vent_band",
    )
    motor.visual(
        Cylinder(radius=0.112, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.190)),
        material=oil_rubbed_bronze,
        name="lower_seat",
    )
    motor.visual(
        Cylinder(radius=0.052, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, -0.246)),
        material=oil_rubbed_bronze,
        name="light_collar",
    )
    motor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.120, length=0.208),
        mass=7.6,
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        rotor_ring_mesh,
        material=oil_rubbed_bronze,
        name="hub_shell",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.060),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
    )

    light_kit = model.part("light_kit")
    light_kit.visual(light_fitter_mesh, material=oil_rubbed_bronze, name="light_fitter")
    light_kit.visual(globe_shell_mesh, material=opal_glass, name="globe_shell")
    light_kit.inertial = Inertial.from_geometry(
        Cylinder(radius=0.102, length=0.160),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    blades = []
    for index in range(5):
        blade = model.part(f"blade_{index + 1}")
        blade.visual(bracket_mesh, material=oil_rubbed_bronze, name="blade_bracket")
        blade.visual(blade_mesh, material=warm_wood, name="blade_panel")
        blade.inertial = Inertial.from_geometry(
            Cylinder(radius=0.058, length=0.440),
            mass=0.8,
            origin=Origin(xyz=(0.224, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        blades.append(blade)

    model.articulation(
        "mount_to_motor",
        ArticulationType.FIXED,
        parent=mount,
        child=motor,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
    )
    model.articulation(
        "motor_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=motor,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.196)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=10.0),
    )
    model.articulation(
        "motor_to_light_kit",
        ArticulationType.FIXED,
        parent=motor,
        child=light_kit,
        origin=Origin(xyz=(0.0, 0.0, -0.298)),
    )

    for index, blade in enumerate(blades):
        angle = index * math.tau / 5.0
        model.articulation(
            f"rotor_to_blade_{index + 1}",
            ArticulationType.FIXED,
            parent=rotor,
            child=blade,
            origin=Origin(
                xyz=(0.170 * math.cos(angle), 0.170 * math.sin(angle), -0.020),
                rpy=(0.0, 0.0, angle),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    mount = object_model.get_part("mount")
    motor = object_model.get_part("motor_housing")
    rotor = object_model.get_part("rotor")
    light_kit = object_model.get_part("light_kit")
    blades = [object_model.get_part(f"blade_{index}") for index in range(1, 6)]
    rotor_spin = object_model.get_articulation("motor_to_rotor")

    mount_lower_collar = mount.get_visual("lower_collar")
    motor_top_collar = motor.get_visual("top_collar")
    motor_lower_seat = motor.get_visual("lower_seat")
    motor_light_collar = motor.get_visual("light_collar")
    rotor_hub = rotor.get_visual("hub_shell")
    light_fitter = light_kit.get_visual("light_fitter")
    globe_shell = light_kit.get_visual("globe_shell")
    blade_brackets = [blade.get_visual("blade_bracket") for blade in blades]
    blade_panels = [blade.get_visual("blade_panel") for blade in blades]

    def dims_from_aabb(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple(upper[i] - lower[i] for i in range(3))

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

    expected_parts = {
        "mount",
        "motor_housing",
        "rotor",
        "light_kit",
        "blade_1",
        "blade_2",
        "blade_3",
        "blade_4",
        "blade_5",
    }
    ctx.check(
        "expected_part_tree_present",
        {part.name for part in object_model.parts} == expected_parts,
        f"present parts: {sorted(part.name for part in object_model.parts)}",
    )
    ctx.check(
        "expected_joint_count_present",
        len(object_model.articulations) == 8,
        f"found joints: {[articulation.name for articulation in object_model.articulations]}",
    )

    ctx.expect_contact(
        motor,
        mount,
        elem_a=motor_top_collar,
        elem_b=mount_lower_collar,
        name="motor_housing_seated_on_downrod",
    )
    ctx.expect_contact(
        rotor,
        motor,
        elem_a=rotor_hub,
        elem_b=motor_lower_seat,
        name="rotor_hub_meets_motor_lower_seat",
    )
    ctx.expect_contact(
        light_kit,
        motor,
        elem_a=light_fitter,
        elem_b=motor_light_collar,
        name="light_kit_hangs_from_motor",
    )
    ctx.expect_origin_distance(
        rotor,
        motor,
        axes="xy",
        max_dist=0.001,
        name="rotor_is_centered_under_motor",
    )
    ctx.expect_origin_distance(
        light_kit,
        motor,
        axes="xy",
        max_dist=0.001,
        name="light_is_centered_under_motor",
    )
    ctx.expect_origin_gap(
        rotor,
        light_kit,
        axis="z",
        min_gap=0.09,
        max_gap=0.13,
        name="light_hangs_well_below_rotor",
    )

    for index, blade in enumerate(blades, start=1):
        bracket = blade_brackets[index - 1]
        panel = blade_panels[index - 1]
        ctx.expect_contact(
            blade,
            rotor,
            elem_a=bracket,
            elem_b=rotor_hub,
            name=f"blade_{index}_bracket_bolts_to_hub",
        )
        ctx.expect_overlap(
            blade,
            rotor,
            axes="xy",
            min_overlap=0.03,
            elem_a=bracket,
            elem_b=rotor_hub,
            name=f"blade_{index}_bracket_overlaps_hub_footprint",
        )
        ctx.expect_overlap(
            blade,
            blade,
            axes="xy",
            min_overlap=0.03,
            elem_a=panel,
            elem_b=bracket,
            name=f"blade_{index}_panel_overlaps_its_bracket",
        )
        ctx.expect_origin_distance(
            blade,
            rotor,
            axes="xy",
            min_dist=0.165,
            max_dist=0.175,
            name=f"blade_{index}_mount_radius",
        )
        ctx.expect_origin_gap(
            rotor,
            blade,
            axis="z",
            min_gap=0.018,
            max_gap=0.022,
            name=f"blade_{index}_sits_below_hub_center",
        )

    blade_aabb = ctx.part_element_world_aabb(blades[0], elem=blade_panels[0])
    blade_dims = dims_from_aabb(blade_aabb)
    if blade_dims is None:
        ctx.fail("blade_1_has_measurable_geometry", "blade_1 AABB was unavailable")
    else:
        ctx.check(
            "blade_1_proportions_are_wide_and_flat",
            0.44 <= blade_dims[0] <= 0.47
            and 0.10 <= blade_dims[1] <= 0.12
            and 0.009 <= blade_dims[2] <= 0.011,
            f"blade_1 dims={blade_dims}",
        )

    globe_aabb = ctx.part_element_world_aabb(light_kit, elem=globe_shell)
    globe_dims = dims_from_aabb(globe_aabb)
    if globe_dims is None:
        ctx.fail("globe_has_measurable_geometry", "globe shell AABB was unavailable")
    else:
        ctx.check(
            "globe_is_round_and_bowl_sized",
            0.19 <= globe_dims[0] <= 0.21
            and 0.19 <= globe_dims[1] <= 0.21
            and 0.12 <= globe_dims[2] <= 0.14,
            f"globe dims={globe_dims}",
        )

    rotor_axis = tuple(round(value, 4) for value in rotor_spin.axis)
    ctx.check(
        "rotor_joint_is_vertical_continuous",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS
        and rotor_axis == (0.0, 0.0, 1.0),
        f"type={rotor_spin.articulation_type}, axis={rotor_spin.axis}",
    )

    rest_pos = ctx.part_world_position(blades[0])
    with ctx.pose({rotor_spin: math.pi / 2.0}):
        quarter_pos = ctx.part_world_position(blades[0])
        ctx.expect_contact(
            blades[0],
            rotor,
            elem_a=blade_brackets[0],
            elem_b=rotor_hub,
            name="blade_1_bracket_stays_attached_while_rotor_turns",
        )
        ctx.expect_origin_distance(
            blades[0],
            rotor,
            axes="xy",
            min_dist=0.165,
            max_dist=0.175,
            name="blade_1_keeps_same_spin_radius",
        )

    if rest_pos is None or quarter_pos is None:
        ctx.fail("blade_1_spin_pose_positions_available", "blade_1 world positions were unavailable")
    else:
        rest_radius = math.hypot(rest_pos[0], rest_pos[1])
        quarter_radius = math.hypot(quarter_pos[0], quarter_pos[1])
        ctx.check(
            "rotor_spin_moves_blade_1_around_z",
            abs(rest_radius - quarter_radius) <= 0.001
            and abs(quarter_pos[0] + rest_pos[1]) <= 0.01
            and abs(quarter_pos[1] - rest_pos[0]) <= 0.01
            and abs(quarter_pos[2] - rest_pos[2]) <= 1e-6,
            f"rest={rest_pos}, quarter_turn={quarter_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
