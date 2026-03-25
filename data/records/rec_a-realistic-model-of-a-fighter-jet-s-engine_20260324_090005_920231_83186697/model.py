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
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
ENGINE_AXIS_RPY = (0.0, math.pi / 2.0, 0.0)


def _x_cylinder_origin(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=ENGINE_AXIS_RPY)


def _engine_shell_mesh():
    shell_profile = [
        (0.425, -0.820),
        (0.455, -0.760),
        (0.490, -0.560),
        (0.505, -0.180),
        (0.455, 0.180),
        (0.395, 0.550),
        (0.340, 0.800),
        (0.300, 0.920),
        (0.255, 0.920),
        (0.285, 0.780),
        (0.325, 0.520),
        (0.360, 0.180),
        (0.395, -0.180),
        (0.380, -0.520),
        (0.350, -0.820),
    ]
    shell_geom = LatheGeometry(shell_profile, segments=64).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell_geom, ASSETS.mesh_dir / "fighter_engine_shell.obj")


def _torus_mesh(filename: str, radius: float, tube: float):
    geom = TorusGeometry(radius=radius, tube=tube, radial_segments=20, tubular_segments=52)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_dir / filename)


def _bearing_ring_mesh():
    ring_geom = LatheGeometry(
        [
            (0.072, -0.018),
            (0.140, -0.018),
            (0.140, 0.018),
            (0.072, 0.018),
        ],
        segments=56,
    ).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(ring_geom, ASSETS.mesh_dir / "fighter_engine_bearing_ring.obj")


def _spinner_mesh():
    spinner_geom = LatheGeometry(
        [
            (0.0, -0.120),
            (0.014, -0.110),
            (0.038, -0.080),
            (0.064, -0.010),
            (0.058, 0.046),
            (0.040, 0.092),
            (0.016, 0.114),
            (0.0, 0.120),
        ],
        segments=48,
    ).rotate_y(math.pi / 2.0)
    return mesh_from_geometry(spinner_geom, ASSETS.mesh_dir / "fighter_engine_spinner.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fighter_jet_engine", assets=ASSETS)

    titanium = model.material("titanium", rgba=(0.62, 0.64, 0.67, 1.0))
    steel = model.material("steel", rgba=(0.46, 0.48, 0.51, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    heat_stained = model.material("heat_stained", rgba=(0.38, 0.34, 0.31, 1.0))

    shell_mesh = _engine_shell_mesh()
    intake_lip_mesh = _torus_mesh("fighter_engine_intake_lip.obj", radius=0.386, tube=0.020)
    bearing_ring_mesh = _bearing_ring_mesh()
    nozzle_rim_mesh = _torus_mesh("fighter_engine_nozzle_rim.obj", radius=0.273, tube=0.013)
    spinner_mesh = _spinner_mesh()

    engine_core = model.part("engine_core")
    engine_core.visual(shell_mesh, material=titanium, name="casing_shell")
    engine_core.visual(
        intake_lip_mesh,
        origin=Origin(xyz=(-0.805, 0.0, 0.0)),
        material=steel,
        name="intake_lip_trim",
    )
    engine_core.visual(
        Cylinder(radius=0.388, length=0.052),
        origin=_x_cylinder_origin(-0.735),
        material=steel,
        name="intake_collar",
    )
    engine_core.visual(
        nozzle_rim_mesh,
        origin=Origin(xyz=(0.920, 0.0, 0.0)),
        material=heat_stained,
        name="nozzle_rim_trim",
    )
    engine_core.visual(
        bearing_ring_mesh,
        origin=Origin(xyz=(-0.330, 0.0, 0.0)),
        material=dark_metal,
        name="bearing_plate",
    )
    for i in range(4):
        angle = i * (math.tau / 4.0)
        engine_core.visual(
            Box((0.410, 0.248, 0.020)),
            origin=Origin(
                xyz=(-0.532, 0.264 * math.cos(angle), 0.264 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=dark_metal,
            name=f"support_vane_{i}",
        )
    for i in range(12):
        angle = i * (math.tau / 12.0)
        engine_core.visual(
            Box((0.160, 0.010, 0.018)),
            origin=Origin(
                xyz=(0.760, 0.327 * math.cos(angle), 0.327 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=heat_stained,
            name=f"nozzle_ridge_{i}",
        )
    engine_core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.505, length=1.740),
        mass=1180.0,
        origin=_x_cylinder_origin(0.050),
    )

    fan_assembly = model.part("fan_assembly")
    fan_assembly.visual(
        Cylinder(radius=0.072, length=0.042),
        origin=_x_cylinder_origin(0.039),
        material=dark_metal,
        name="hub_sleeve",
    )
    fan_assembly.visual(
        Cylinder(radius=0.148, length=0.060),
        origin=_x_cylinder_origin(0.055),
        material=steel,
        name="hub_disk",
    )
    fan_assembly.visual(
        spinner_mesh,
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
        material=dark_metal,
        name="spinner",
    )
    for i in range(12):
        angle = i * (math.tau / 12.0)
        fan_assembly.visual(
            Box((0.042, 0.360, 0.016)),
            origin=Origin(
                xyz=(0.060, 0.180 * math.cos(angle), 0.180 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=steel,
            name=f"blade_{i}",
        )
    fan_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.320, length=0.260),
        mass=145.0,
        origin=_x_cylinder_origin(-0.075),
    )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=engine_core,
        child=fan_assembly,
        origin=Origin(xyz=(-0.330, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=650.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    engine_core = object_model.get_part("engine_core")
    fan_assembly = object_model.get_part("fan_assembly")
    fan_spin = object_model.get_articulation("fan_spin")

    casing_shell = engine_core.get_visual("casing_shell")
    intake_collar = engine_core.get_visual("intake_collar")
    intake_lip_trim = engine_core.get_visual("intake_lip_trim")
    nozzle_rim_trim = engine_core.get_visual("nozzle_rim_trim")
    bearing_plate = engine_core.get_visual("bearing_plate")

    hub_sleeve = fan_assembly.get_visual("hub_sleeve")
    spinner = fan_assembly.get_visual("spinner")
    hub_disk = fan_assembly.get_visual("hub_disk")
    blade_0 = fan_assembly.get_visual("blade_0")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.080)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )
    ctx.expect_origin_distance(fan_assembly, engine_core, axes="yz", max_dist=0.001)
    ctx.expect_overlap(fan_assembly, engine_core, axes="yz", min_overlap=0.700)
    ctx.expect_contact(fan_assembly, engine_core, elem_a=hub_sleeve, elem_b=bearing_plate)
    ctx.expect_gap(
        fan_assembly,
        engine_core,
        axis="x",
        min_gap=0.120,
        positive_elem=spinner,
        negative_elem=intake_collar,
        name="spinner_set_back_inside_intake",
    )
    ctx.expect_gap(
        engine_core,
        fan_assembly,
        axis="x",
        min_gap=1.050,
        positive_elem=nozzle_rim_trim,
        negative_elem=spinner,
        name="long_core_to_exhaust_nozzle",
    )
    ctx.expect_gap(
        engine_core,
        fan_assembly,
        axis="x",
        min_gap=1.150,
        positive_elem=nozzle_rim_trim,
        negative_elem=blade_0,
        name="blade_stage_well_forward_of_nozzle",
    )
    ctx.expect_overlap(fan_assembly, engine_core, axes="yz", min_overlap=0.700)
    ctx.expect_contact(fan_assembly, fan_assembly, elem_a=spinner, elem_b=hub_disk)
    with ctx.pose({fan_spin: 1.300}):
        ctx.expect_origin_distance(fan_assembly, engine_core, axes="yz", max_dist=0.001)
        ctx.expect_overlap(fan_assembly, engine_core, axes="yz", min_overlap=0.700)
        ctx.expect_contact(fan_assembly, engine_core, elem_a=hub_sleeve, elem_b=bearing_plate)
        ctx.expect_gap(
            engine_core,
            fan_assembly,
            axis="x",
            min_gap=1.050,
            positive_elem=nozzle_rim_trim,
            negative_elem=spinner,
            name="posed_spinner_stays_forward_of_exhaust_nozzle",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
