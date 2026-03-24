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
    ConeGeometry,
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
IDENTITY = Origin(xyz=(0.0, 0.0, 0.0))
ROTOR_FRAME_Z = 0.77


def _merge_geometries(geometries):
    merged = None
    for geometry in geometries:
        if merged is None:
            merged = geometry
        else:
            merged.merge(geometry)
    return merged


def _airfoil_section(
    radius: float,
    z_center: float,
    chord: float,
    thickness: float,
    lean: float,
) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (radius, lean - half_thickness, z_center - 0.50 * chord),
        (radius, lean + 0.35 * half_thickness, z_center - 0.08 * chord),
        (radius, lean + half_thickness, z_center + 0.46 * chord),
        (radius, lean - 0.22 * half_thickness, z_center + 0.12 * chord),
    ]


def _build_blade_ring(
    *,
    root_radius: float,
    mid_radius: float,
    tip_radius: float,
    root_z: float,
    mid_z: float,
    tip_z: float,
    root_chord: float,
    tip_chord: float,
    root_thickness: float,
    tip_thickness: float,
    root_lean: float,
    tip_lean: float,
    count: int,
    phase: float = 0.0,
):
    blade = section_loft(
        [
            _airfoil_section(root_radius, root_z, root_chord, root_thickness, root_lean),
            _airfoil_section(
                mid_radius,
                mid_z,
                0.5 * (root_chord + tip_chord),
                0.5 * (root_thickness + tip_thickness),
                0.5 * (root_lean + tip_lean),
            ),
            _airfoil_section(tip_radius, tip_z, tip_chord, tip_thickness, tip_lean),
        ]
    )
    return _merge_geometries(
        [
            blade.copy().rotate_z(phase + (2.0 * math.pi * index / count))
            for index in range(count)
        ]
    )


def _petal_section(
    z_pos: float,
    inner_radius: float,
    outer_radius: float,
    width: float,
) -> list[tuple[float, float, float]]:
    half_width = width * 0.5
    return [
        (inner_radius, -half_width, z_pos),
        (outer_radius, -0.42 * half_width, z_pos),
        (outer_radius, 0.42 * half_width, z_pos),
        (inner_radius, half_width, z_pos),
    ]


def _build_nozzle_petals(count: int):
    petal = section_loft(
        [
            _petal_section(2.78, 0.458, 0.515, 0.162),
            _petal_section(2.99, 0.404, 0.470, 0.132),
            _petal_section(3.18, 0.338, 0.408, 0.102),
        ]
    )
    return _merge_geometries(
        [petal.copy().rotate_z(2.0 * math.pi * index / count) for index in range(count)]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fighter_jet_engine", assets=ASSETS)

    titanium = model.material("titanium", rgba=(0.55, 0.57, 0.60, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.31, 0.33, 0.36, 1.0))
    heat_stained = model.material("heat_stained", rgba=(0.52, 0.47, 0.40, 1.0))
    blackened = model.material("blackened", rgba=(0.18, 0.19, 0.20, 1.0))

    casing = model.part("casing")
    rotor = model.part("rotor")

    shell_profile = [
        (0.58, 0.00),
        (0.60, 0.10),
        (0.58, 0.45),
        (0.56, 1.05),
        (0.59, 1.55),
        (0.62, 1.85),
        (0.58, 2.20),
        (0.50, 2.65),
        (0.45, 2.90),
        (0.39, 3.12),
        (0.26, 3.12),
        (0.30, 2.90),
        (0.35, 2.60),
        (0.38, 2.20),
        (0.43, 1.70),
        (0.44, 1.05),
        (0.46, 0.45),
        (0.47, 0.02),
    ]
    shell_mesh = mesh_from_geometry(
        LatheGeometry(shell_profile, segments=72),
        ASSETS.mesh_path("fighter_engine_shell.obj"),
    )
    casing.visual(shell_mesh, origin=IDENTITY, material=titanium, name="engine_shell")

    bearing_ring_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.034, 0.67),
                (0.055, 0.67),
                (0.055, 0.77),
                (0.034, 0.77),
            ],
            segments=48,
        ),
        ASSETS.mesh_path("fighter_engine_bearing_ring.obj"),
    )
    casing.visual(
        bearing_ring_mesh,
        origin=IDENTITY,
        material=dark_steel,
        name="front_bearing",
    )

    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        casing.visual(
            Box((0.43, 0.034, 0.06)),
            origin=Origin(
                xyz=(0.248 * math.cos(angle), 0.248 * math.sin(angle), 0.72),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark_steel,
            name=f"support_{index}",
        )

    nozzle_petals_mesh = mesh_from_geometry(
        _build_nozzle_petals(12),
        ASSETS.mesh_path("fighter_engine_nozzle_petals.obj"),
    )
    casing.visual(
        nozzle_petals_mesh,
        origin=IDENTITY,
        material=heat_stained,
        name="nozzle_petals",
    )

    casing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.62, length=3.12),
        mass=950.0,
        origin=Origin(xyz=(0.0, 0.0, 1.56)),
    )

    spinner_mesh = mesh_from_geometry(
        ConeGeometry(radius=0.11, height=0.36, radial_segments=44, closed=True).translate(
            0.0, 0.0, 0.20 - ROTOR_FRAME_Z
        ),
        ASSETS.mesh_path("fighter_engine_spinner.obj"),
    )
    rotor.visual(spinner_mesh, origin=IDENTITY, material=blackened, name="spinner")

    rotor.visual(
        Cylinder(radius=0.145, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.45 - ROTOR_FRAME_Z)),
        material=dark_steel,
        name="fan_hub",
    )
    rotor.visual(
        Cylinder(radius=0.03, length=2.25),
        origin=Origin(xyz=(0.0, 0.0, 1.65 - ROTOR_FRAME_Z)),
        material=dark_steel,
        name="hub_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.055, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.79 - ROTOR_FRAME_Z)),
        material=dark_steel,
        name="bearing_collar",
    )

    fan_blades_mesh = mesh_from_geometry(
        _build_blade_ring(
            root_radius=0.13,
            mid_radius=0.28,
            tip_radius=0.42,
            root_z=0.36 - ROTOR_FRAME_Z,
            mid_z=0.46 - ROTOR_FRAME_Z,
            tip_z=0.56 - ROTOR_FRAME_Z,
            root_chord=0.18,
            tip_chord=0.09,
            root_thickness=0.020,
            tip_thickness=0.010,
            root_lean=-0.020,
            tip_lean=0.016,
            count=11,
            phase=math.radians(8.0),
        ),
        ASSETS.mesh_path("fighter_engine_fan_blades.obj"),
    )
    rotor.visual(
        fan_blades_mesh,
        origin=IDENTITY,
        material=titanium,
        name="fan_blades",
    )

    compressor_blades_geom = _merge_geometries(
        [
            _build_blade_ring(
                root_radius=0.09,
                mid_radius=0.17,
                tip_radius=0.27,
                root_z=0.90 - ROTOR_FRAME_Z,
                mid_z=0.97 - ROTOR_FRAME_Z,
                tip_z=1.05 - ROTOR_FRAME_Z,
                root_chord=0.11,
                tip_chord=0.065,
                root_thickness=0.014,
                tip_thickness=0.007,
                root_lean=-0.014,
                tip_lean=0.010,
                count=15,
            ),
            _build_blade_ring(
                root_radius=0.09,
                mid_radius=0.17,
                tip_radius=0.27,
                root_z=1.15 - ROTOR_FRAME_Z,
                mid_z=1.22 - ROTOR_FRAME_Z,
                tip_z=1.30 - ROTOR_FRAME_Z,
                root_chord=0.10,
                tip_chord=0.060,
                root_thickness=0.013,
                tip_thickness=0.0065,
                root_lean=-0.012,
                tip_lean=0.009,
                count=15,
                phase=math.radians(6.0),
            ),
            _build_blade_ring(
                root_radius=0.09,
                mid_radius=0.16,
                tip_radius=0.25,
                root_z=1.39 - ROTOR_FRAME_Z,
                mid_z=1.47 - ROTOR_FRAME_Z,
                tip_z=1.54 - ROTOR_FRAME_Z,
                root_chord=0.095,
                tip_chord=0.055,
                root_thickness=0.012,
                tip_thickness=0.006,
                root_lean=-0.010,
                tip_lean=0.008,
                count=14,
                phase=math.radians(3.0),
            ),
        ]
    )
    compressor_blades_mesh = mesh_from_geometry(
        compressor_blades_geom,
        ASSETS.mesh_path("fighter_engine_compressor_blades.obj"),
    )
    rotor.visual(
        compressor_blades_mesh,
        origin=IDENTITY,
        material=titanium,
        name="compressor_blades",
    )

    turbine_blades_geom = _merge_geometries(
        [
            _build_blade_ring(
                root_radius=0.08,
                mid_radius=0.14,
                tip_radius=0.21,
                root_z=2.10 - ROTOR_FRAME_Z,
                mid_z=2.04 - ROTOR_FRAME_Z,
                tip_z=1.98 - ROTOR_FRAME_Z,
                root_chord=0.080,
                tip_chord=0.050,
                root_thickness=0.010,
                tip_thickness=0.005,
                root_lean=0.012,
                tip_lean=-0.010,
                count=13,
                phase=math.radians(4.0),
            ),
            _build_blade_ring(
                root_radius=0.08,
                mid_radius=0.14,
                tip_radius=0.20,
                root_z=2.32 - ROTOR_FRAME_Z,
                mid_z=2.26 - ROTOR_FRAME_Z,
                tip_z=2.20 - ROTOR_FRAME_Z,
                root_chord=0.075,
                tip_chord=0.048,
                root_thickness=0.009,
                tip_thickness=0.0045,
                root_lean=0.010,
                tip_lean=-0.008,
                count=13,
            ),
        ]
    )
    turbine_blades_mesh = mesh_from_geometry(
        turbine_blades_geom,
        ASSETS.mesh_path("fighter_engine_turbine_blades.obj"),
    )
    rotor.visual(
        turbine_blades_mesh,
        origin=IDENTITY,
        material=heat_stained,
        name="turbine_blades",
    )

    for index, (radius, length, z_pos) in enumerate(
        (
            (0.11, 0.035, 0.97 - ROTOR_FRAME_Z),
            (0.11, 0.035, 1.22 - ROTOR_FRAME_Z),
            (0.10, 0.035, 1.47 - ROTOR_FRAME_Z),
            (0.09, 0.030, 2.04 - ROTOR_FRAME_Z),
            (0.085, 0.030, 2.26 - ROTOR_FRAME_Z),
        )
    ):
        rotor.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=dark_steel,
            name=f"stage_disc_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.42, length=2.78),
        mass=280.0,
        origin=Origin(xyz=(0.0, 0.0, 1.55 - ROTOR_FRAME_Z)),
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=casing,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.77)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=300.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    casing = object_model.get_part("casing")
    rotor = object_model.get_part("rotor")
    rotor_spin = object_model.get_articulation("rotor_spin")

    front_bearing = casing.get_visual("front_bearing")
    nozzle_petals = casing.get_visual("nozzle_petals")
    bearing_collar = rotor.get_visual("bearing_collar")
    fan_blades = rotor.get_visual("fan_blades")
    turbine_blades = rotor.get_visual("turbine_blades")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.04)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance(rotor, casing, axes="xy", max_dist=0.005)
    ctx.expect_within(rotor, casing, axes="xy")
    ctx.expect_overlap(rotor, casing, axes="xy", min_overlap=0.20)
    ctx.expect_gap(
        rotor,
        casing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=bearing_collar,
        negative_elem=front_bearing,
        name="rotor collar seats against front bearing",
    )
    ctx.expect_gap(
        casing,
        rotor,
        axis="z",
        min_gap=0.03,
        max_gap=0.18,
        positive_elem=front_bearing,
        negative_elem=fan_blades,
        name="front fan stays just ahead of the support bearing",
    )
    ctx.expect_gap(
        casing,
        rotor,
        axis="z",
        min_gap=0.28,
        positive_elem=nozzle_petals,
        negative_elem=turbine_blades,
        name="variable nozzle petals sit aft of the turbine section",
    )

    with ctx.pose({rotor_spin: math.radians(75.0)}):
        ctx.expect_origin_distance(rotor, casing, axes="xy", max_dist=0.005)
        ctx.expect_within(rotor, casing, axes="xy")
        ctx.expect_gap(
            rotor,
            casing,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=bearing_collar,
            negative_elem=front_bearing,
            name="rotor collar stays seated while spinning",
        )
        ctx.expect_gap(
            casing,
            rotor,
            axis="z",
            min_gap=0.28,
            positive_elem=nozzle_petals,
            negative_elem=turbine_blades,
            name="rear nozzle stays aft of turbine blades while spinning",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
