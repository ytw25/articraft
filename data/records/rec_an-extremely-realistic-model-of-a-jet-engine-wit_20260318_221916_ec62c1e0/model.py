from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _airfoil_loop(
    y: float,
    chord: float,
    thickness: float,
    camber: float,
    *,
    x_shift: float = 0.0,
    z_shift: float = 0.0,
    twist: float = 0.0,
) -> list[tuple[float, float, float]]:
    profile = [
        (-0.48 * chord, 0.00),
        (-0.18 * chord, thickness * 0.68 + camber * 0.18),
        (0.10 * chord, thickness * 0.58 + camber * 0.46),
        (0.50 * chord, 0.05 * thickness),
        (0.16 * chord, -0.92 * thickness + camber * 0.16),
        (-0.30 * chord, -0.58 * thickness),
    ]
    c = cos(twist)
    s = sin(twist)
    loop: list[tuple[float, float, float]] = []
    for x, z in profile:
        xr = c * x - s * z + x_shift
        yr = s * x + c * z + z_shift
        loop.append((xr, yr, y))
    return loop


def _blade_geometry(
    *,
    root_radius: float,
    tip_radius: float,
    chord_root: float,
    chord_tip: float,
    thickness_root: float,
    thickness_tip: float,
    camber: float,
    sweep: float,
    lean: float,
    twist_root: float,
    twist_tip: float,
) -> MeshGeometry:
    mid_radius = 0.5 * (root_radius + tip_radius)
    return LoftGeometry(
        [
            _airfoil_loop(
                root_radius,
                chord_root,
                thickness_root,
                camber,
                twist=twist_root,
            ),
            _airfoil_loop(
                mid_radius,
                0.58 * (chord_root + chord_tip),
                0.58 * (thickness_root + thickness_tip),
                0.92 * camber,
                x_shift=0.5 * sweep,
                z_shift=0.5 * lean,
                twist=0.5 * (twist_root + twist_tip),
            ),
            _airfoil_loop(
                tip_radius,
                chord_tip,
                thickness_tip,
                0.84 * camber,
                x_shift=sweep,
                z_shift=lean,
                twist=twist_tip,
            ),
        ],
        cap=False,
        closed=True,
    )


def _blade_ring(
    *,
    count: int,
    stage_x: float,
    root_radius: float,
    tip_radius: float,
    chord_root: float,
    chord_tip: float,
    thickness_root: float,
    thickness_tip: float,
    camber: float,
    sweep: float,
    lean: float,
    twist_root: float,
    twist_tip: float,
    azimuth_offset: float = 0.0,
) -> MeshGeometry:
    base_blade = _blade_geometry(
        root_radius=root_radius,
        tip_radius=tip_radius,
        chord_root=chord_root,
        chord_tip=chord_tip,
        thickness_root=thickness_root,
        thickness_tip=thickness_tip,
        camber=camber,
        sweep=sweep,
        lean=lean,
        twist_root=twist_root,
        twist_tip=twist_tip,
    )
    ring = MeshGeometry()
    for idx in range(count):
        blade = base_blade.clone()
        blade.translate(stage_x, 0.0, 0.0)
        blade.rotate_x(azimuth_offset + (2.0 * pi * idx / count))
        ring.merge(blade)
    return ring


def _cylinder_x(radius: float, length: float, x_center: float, *, radial_segments: int = 40) -> MeshGeometry:
    return CylinderGeometry(radius, length, radial_segments=radial_segments, closed=True).rotate_y(pi / 2).translate(
        x_center,
        0.0,
        0.0,
    )


def _cone_x(radius: float, length: float, x_center: float, *, radial_segments: int = 40) -> MeshGeometry:
    return ConeGeometry(radius, length, radial_segments=radial_segments, closed=True).rotate_y(pi / 2).translate(
        x_center,
        0.0,
        0.0,
    )


def _lathe_x(profile: list[tuple[float, float]], *, segments: int = 72) -> MeshGeometry:
    return LatheGeometry(profile, segments=segments).rotate_y(pi / 2)


def _write_mesh(name: str, geom: MeshGeometry):
    return mesh_from_geometry(geom, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="detailed_turbofan_engine", assets=ASSETS)

    titanium = model.material("titanium", rgba=(0.64, 0.66, 0.70, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.51, 0.53, 0.56, 1.0))
    smoked_shell = model.material("smoked_shell", rgba=(0.30, 0.34, 0.39, 0.26))
    dark_composite = model.material("dark_composite", rgba=(0.14, 0.15, 0.17, 1.0))
    hot_alloy = model.material("hot_alloy", rgba=(0.60, 0.43, 0.31, 1.0))
    bronze = model.material("bronze", rgba=(0.66, 0.53, 0.30, 1.0))

    engine_body = model.part("engine_body")

    outer_shell = _lathe_x(
        [
            (0.43, 0.00),
            (0.50, 0.07),
            (0.52, 0.28),
            (0.50, 0.66),
            (0.44, 1.06),
            (0.36, 1.42),
            (0.28, 1.72),
        ],
        segments=96,
    )
    engine_body.visual(
        _write_mesh("engine_outer_shell.obj", outer_shell),
        material=smoked_shell,
        name="outer_shell",
    )

    core_casing = _lathe_x(
        [
            (0.10, 0.52),
            (0.18, 0.62),
            (0.20, 0.86),
            (0.22, 1.02),
            (0.19, 1.22),
            (0.15, 1.42),
            (0.10, 1.58),
        ],
        segments=80,
    )
    engine_body.visual(
        _write_mesh("engine_core_casing.obj", core_casing),
        material=titanium,
        name="core_casing",
    )

    fan_guide_and_stators = MeshGeometry()
    fan_guide_and_stators.merge(
        _blade_ring(
            count=11,
            stage_x=0.46,
            root_radius=0.035,
            tip_radius=0.505,
            chord_root=0.13,
            chord_tip=0.10,
            thickness_root=0.016,
            thickness_tip=0.010,
            camber=0.012,
            sweep=-0.03,
            lean=0.01,
            twist_root=0.40,
            twist_tip=0.14,
            azimuth_offset=0.08,
        )
    )
    for stage_x, tip_radius, chord in (
        (0.67, 0.195, 0.085),
        (0.79, 0.185, 0.078),
        (0.91, 0.172, 0.072),
    ):
        fan_guide_and_stators.merge(
            _blade_ring(
                count=18,
                stage_x=stage_x,
                root_radius=0.033,
                tip_radius=tip_radius,
                chord_root=chord,
                chord_tip=0.75 * chord,
                thickness_root=0.010,
                thickness_tip=0.006,
                camber=0.008,
                sweep=-0.016,
                lean=0.0,
                twist_root=0.30,
                twist_tip=0.10,
                azimuth_offset=0.05,
            )
        )
    engine_body.visual(
        _write_mesh("engine_static_vanes.obj", fan_guide_and_stators),
        material=brushed_steel,
        name="static_vanes",
    )

    hot_section_vanes = _blade_ring(
        count=20,
        stage_x=1.12,
        root_radius=0.032,
        tip_radius=0.155,
        chord_root=0.070,
        chord_tip=0.050,
        thickness_root=0.008,
        thickness_tip=0.005,
        camber=0.008,
        sweep=0.010,
        lean=0.0,
        twist_root=-0.24,
        twist_tip=-0.08,
        azimuth_offset=0.06,
    )
    engine_body.visual(
        _write_mesh("engine_hot_section_vanes.obj", hot_section_vanes),
        material=hot_alloy,
        name="hot_section_vanes",
    )

    engine_body.visual(
        Cylinder(radius=0.050, length=1.28),
        origin=Origin(xyz=(0.86, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=dark_composite,
        name="center_support_shaft",
    )
    engine_body.visual(
        Cylinder(radius=0.15, length=0.28),
        origin=Origin(xyz=(1.08, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=bronze,
        name="combustor_drum",
    )
    engine_body.visual(
        Cylinder(radius=0.46, length=0.34),
        origin=Origin(xyz=(0.27, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=dark_composite,
        name="fan_case_liner",
    )
    engine_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.52, length=1.72),
        mass=720.0,
        origin=Origin(xyz=(0.86, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
    )

    fan_rotor = model.part("fan_rotor")

    front_fan_blades = _blade_ring(
        count=24,
        stage_x=0.02,
        root_radius=0.105,
        tip_radius=0.425,
        chord_root=0.23,
        chord_tip=0.11,
        thickness_root=0.024,
        thickness_tip=0.011,
        camber=0.022,
        sweep=-0.070,
        lean=0.016,
        twist_root=0.98,
        twist_tip=0.24,
        azimuth_offset=0.05,
    )
    booster_blades = _blade_ring(
        count=18,
        stage_x=0.16,
        root_radius=0.080,
        tip_radius=0.205,
        chord_root=0.10,
        chord_tip=0.060,
        thickness_root=0.012,
        thickness_tip=0.006,
        camber=0.010,
        sweep=-0.025,
        lean=0.0,
        twist_root=0.65,
        twist_tip=0.16,
        azimuth_offset=0.12,
    )
    fan_rotor.visual(
        _write_mesh("fan_rotor_main_blades.obj", front_fan_blades),
        material=titanium,
        name="front_fan_blades",
    )
    fan_rotor.visual(
        _write_mesh("fan_rotor_booster_blades.obj", booster_blades),
        material=brushed_steel,
        name="booster_blades",
    )
    fan_rotor.visual(
        Cylinder(radius=0.132, length=0.19),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=brushed_steel,
        name="fan_hub",
    )
    fan_rotor.visual(
        Cylinder(radius=0.09, length=0.03),
        origin=Origin(xyz=(0.16, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=brushed_steel,
        name="booster_disk",
    )
    fan_rotor.visual(
        _write_mesh("fan_spinner.obj", _cone_x(0.16, 0.26, -0.06, radial_segments=48)),
        material=dark_composite,
        name="fan_spinner",
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.43, length=0.30),
        mass=115.0,
        origin=Origin(xyz=(0.05, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
    )

    core_rotor = model.part("core_rotor")

    compressor_geom = MeshGeometry()
    compressor_stages = [
        (-0.38, 0.078, 0.18, 18, 0.10, 0.060, 0.78, 0.24, -0.018),
        (-0.26, 0.072, 0.16, 20, 0.092, 0.056, 0.70, 0.20, -0.016),
        (-0.14, 0.066, 0.145, 22, 0.086, 0.050, 0.62, 0.18, -0.014),
        (-0.02, 0.060, 0.130, 24, 0.078, 0.046, 0.54, 0.16, -0.012),
    ]
    for stage_x, disk_radius, tip_radius, blade_count, chord_root, chord_tip, twist_root, twist_tip, sweep in compressor_stages:
        compressor_geom.merge(_cylinder_x(disk_radius, 0.030, stage_x, radial_segments=28))
        compressor_geom.merge(
            _blade_ring(
                count=blade_count,
                stage_x=stage_x,
                root_radius=max(0.040, disk_radius - 0.006),
                tip_radius=tip_radius,
                chord_root=chord_root,
                chord_tip=chord_tip,
                thickness_root=0.010,
                thickness_tip=0.005,
                camber=0.008,
                sweep=sweep,
                lean=0.0,
                twist_root=twist_root,
                twist_tip=twist_tip,
                azimuth_offset=0.04,
            )
        )

    turbine_geom = MeshGeometry()
    turbine_stages = [
        (0.22, 0.070, 0.152, 22, 0.076, 0.058, -0.34, -0.10, 0.010),
        (0.36, 0.064, 0.134, 20, 0.070, 0.052, -0.28, -0.08, 0.008),
    ]
    for stage_x, disk_radius, tip_radius, blade_count, chord_root, chord_tip, twist_root, twist_tip, sweep in turbine_stages:
        turbine_geom.merge(_cylinder_x(disk_radius, 0.034, stage_x, radial_segments=28))
        turbine_geom.merge(
            _blade_ring(
                count=blade_count,
                stage_x=stage_x,
                root_radius=max(0.038, disk_radius - 0.006),
                tip_radius=tip_radius,
                chord_root=chord_root,
                chord_tip=chord_tip,
                thickness_root=0.009,
                thickness_tip=0.005,
                camber=0.006,
                sweep=sweep,
                lean=0.0,
                twist_root=twist_root,
                twist_tip=twist_tip,
                azimuth_offset=0.08,
            )
        )

    core_rotor.visual(
        Cylinder(radius=0.048, length=1.06),
        origin=Origin(xyz=(0.00, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=dark_composite,
        name="core_shaft",
    )
    core_rotor.visual(
        _write_mesh("core_rotor_compressor.obj", compressor_geom),
        material=titanium,
        name="compressor_stages",
    )
    core_rotor.visual(
        _write_mesh("core_rotor_turbines.obj", turbine_geom),
        material=hot_alloy,
        name="turbine_stages",
    )
    core_rotor.visual(
        _write_mesh("core_rotor_tail_cone.obj", _cone_x(0.085, 0.22, 0.48, radial_segments=36)),
        material=brushed_steel,
        name="tail_cone",
    )
    core_rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.19, length=1.10),
        mass=135.0,
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent="engine_body",
        child="fan_rotor",
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=160.0),
    )
    model.articulation(
        "core_spin",
        ArticulationType.CONTINUOUS,
        parent="engine_body",
        child="core_rotor",
        origin=Origin(xyz=(0.98, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=220.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("fan_rotor", "engine_body", axes="yz", max_dist=0.005)
    ctx.expect_origin_distance("core_rotor", "engine_body", axes="yz", max_dist=0.005)
    ctx.expect_aabb_overlap("fan_rotor", "engine_body", axes="yz", min_overlap=0.78)
    ctx.expect_aabb_overlap("fan_rotor", "engine_body", axes="x", min_overlap=0.16)
    ctx.expect_aabb_overlap("core_rotor", "engine_body", axes="yz", min_overlap=0.24)
    ctx.expect_aabb_overlap("core_rotor", "engine_body", axes="x", min_overlap=0.85)
    ctx.expect_aabb_gap("core_rotor", "fan_rotor", axis="x", max_gap=0.012, max_penetration=0.0)
    ctx.expect_aabb_overlap("fan_rotor", "core_rotor", axes="yz", min_overlap=0.26)

    for angle in (0.0, pi / 2, pi):
        with ctx.pose(fan_spin=angle):
            ctx.expect_origin_distance("fan_rotor", "engine_body", axes="yz", max_dist=0.005)
            ctx.expect_aabb_overlap("fan_rotor", "engine_body", axes="yz", min_overlap=0.78)
        with ctx.pose(core_spin=angle):
            ctx.expect_origin_distance("core_rotor", "engine_body", axes="yz", max_dist=0.005)
            ctx.expect_aabb_overlap("core_rotor", "engine_body", axes="yz", min_overlap=0.24)

    with ctx.pose(fan_spin=pi / 2, core_spin=pi):
        ctx.expect_aabb_gap("core_rotor", "fan_rotor", axis="x", max_gap=0.012, max_penetration=0.0)
        ctx.expect_aabb_overlap("fan_rotor", "core_rotor", axes="yz", min_overlap=0.26)
        ctx.expect_origin_distance("fan_rotor", "engine_body", axes="yz", max_dist=0.005)
        ctx.expect_origin_distance("core_rotor", "engine_body", axes="yz", max_dist=0.005)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
