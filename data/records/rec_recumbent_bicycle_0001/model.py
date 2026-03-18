from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged: MeshGeometry | None = None
    for geometry in geometries:
        copied = geometry.copy()
        if merged is None:
            merged = copied
        else:
            merged.merge(copied)
    return merged if merged is not None else MeshGeometry()


def _tube(
    points: list[tuple[float, float, float]],
    radius: float,
    *,
    samples: int = 12,
    radial: int = 16,
) -> MeshGeometry:
    return tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=samples,
        radial_segments=radial,
        cap_ends=True,
    )


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_wheel_meshes(
    prefix: str,
    *,
    outer_radius: float,
    tire_radius: float,
    rim_radius: float,
    hub_radius: float,
    hub_length: float,
    spoke_count: int,
    add_cassette: bool = False,
) -> dict[str, object]:
    tire_major = outer_radius - tire_radius
    rim_major = tire_major - 0.006

    tire = TorusGeometry(
        tire_major,
        tire_radius,
        radial_segments=22,
        tubular_segments=64,
    ).rotate_x(math.pi / 2.0)

    rim = TorusGeometry(
        rim_major,
        rim_radius,
        radial_segments=18,
        tubular_segments=48,
    ).rotate_x(math.pi / 2.0)

    hub = CylinderGeometry(
        hub_radius,
        hub_length,
        radial_segments=24,
    ).rotate_x(math.pi / 2.0)

    spokes: list[MeshGeometry] = []
    flange_offset = hub_length * 0.32
    spoke_radius = 0.0028 if outer_radius > 0.28 else 0.0025
    for index in range(spoke_count):
        angle = (2.0 * math.pi * index) / spoke_count
        end = (rim_major * math.cos(angle), 0.0, rim_major * math.sin(angle))
        start = (0.0, flange_offset if index % 2 == 0 else -flange_offset, 0.0)
        spokes.append(_tube([start, end], spoke_radius, samples=1, radial=10))

    rotor = (
        CylinderGeometry(
            0.070 if outer_radius > 0.28 else 0.052,
            0.0025,
            radial_segments=32,
        )
        .rotate_x(math.pi / 2.0)
        .translate(0.0, hub_length * 0.44, 0.0)
    )

    metal = _merge_geometries(rim, hub, rotor, *spokes)

    if add_cassette:
        for offset_index, radius in enumerate((0.045, 0.050, 0.055, 0.060, 0.066)):
            sprocket = (
                CylinderGeometry(
                    radius,
                    0.003,
                    radial_segments=28,
                )
                .rotate_x(math.pi / 2.0)
                .translate(
                    0.0,
                    -hub_length * 0.46 - 0.0034 * offset_index,
                    0.0,
                )
            )
            metal.merge(sprocket)

    return {
        "tire": _save_mesh(f"{prefix}_tire.obj", tire),
        "metal": _save_mesh(f"{prefix}_metal.obj", metal),
        "valve_mount_z": tire_major + tire_radius * 0.72,
    }


def _build_frame_meshes() -> dict[str, object]:
    frame = _merge_geometries(
        _tube(
            [
                (-0.03, 0.0, 0.36),
                (0.22, 0.0, 0.31),
                (0.58, 0.0, 0.28),
                (0.92, 0.0, 0.31),
                (1.00, 0.0, 0.35),
            ],
            0.028,
            samples=18,
            radial=22,
        ),
        _tube(
            [
                (0.88, 0.0, 0.30),
                (1.02, 0.0, 0.34),
                (1.08, 0.0, 0.38),
            ],
            0.016,
            samples=10,
            radial=16,
        ),
        _tube(
            [
                (0.40, 0.0, 0.74),
                (0.72, 0.0, 0.56),
                (1.04, 0.0, 0.44),
            ],
            0.015,
            samples=16,
            radial=16,
        ),
        _tube(
            [
                (0.04, -0.055, 0.33),
                (0.22, -0.060, 0.31),
                (0.52, -0.070, 0.29),
                (0.90, -0.060, 0.34),
            ],
            0.013,
            samples=12,
            radial=16,
        ),
        _tube(
            [
                (0.04, 0.055, 0.33),
                (0.22, 0.060, 0.31),
                (0.52, 0.070, 0.29),
                (0.90, 0.060, 0.34),
            ],
            0.013,
            samples=12,
            radial=16,
        ),
        _tube(
            [
                (0.00, -0.055, 0.33),
                (0.20, -0.080, 0.45),
                (0.40, -0.120, 0.70),
            ],
            0.011,
            samples=12,
            radial=14,
        ),
        _tube(
            [
                (0.00, 0.055, 0.33),
                (0.20, 0.080, 0.45),
                (0.40, 0.120, 0.70),
            ],
            0.011,
            samples=12,
            radial=14,
        ),
        _tube(
            [
                (0.36, -0.120, 0.41),
                (0.52, -0.120, 0.46),
                (0.62, -0.120, 0.49),
                (0.54, -0.120, 0.63),
                (0.42, -0.120, 0.80),
            ],
            0.012,
            samples=14,
            radial=14,
        ),
        _tube(
            [
                (0.36, 0.120, 0.41),
                (0.52, 0.120, 0.46),
                (0.62, 0.120, 0.49),
                (0.54, 0.120, 0.63),
                (0.42, 0.120, 0.80),
            ],
            0.012,
            samples=14,
            radial=14,
        ),
        _tube([(0.44, -0.10, 0.30), (0.43, -0.11, 0.44)], 0.010, samples=4, radial=12),
        _tube([(0.44, 0.10, 0.30), (0.43, 0.11, 0.44)], 0.010, samples=4, radial=12),
        _tube([(0.58, -0.08, 0.29), (0.56, -0.11, 0.56)], 0.010, samples=6, radial=12),
        _tube([(0.58, 0.08, 0.29), (0.56, 0.11, 0.56)], 0.010, samples=6, radial=12),
        _tube([(0.58, -0.02, 0.29), (0.64, -0.05, 0.30)], 0.007, samples=4, radial=12),
        CylinderGeometry(0.014, 0.14, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, 0.33),
        CylinderGeometry(0.012, 0.24, radial_segments=18)
        .rotate_x(math.pi / 2.0)
        .translate(0.46, 0.0, 0.43),
        CylinderGeometry(0.012, 0.24, radial_segments=18)
        .rotate_x(math.pi / 2.0)
        .translate(0.59, 0.0, 0.49),
        CylinderGeometry(0.012, 0.26, radial_segments=18)
        .rotate_x(math.pi / 2.0)
        .translate(0.45, 0.0, 0.72),
        CylinderGeometry(0.037, 0.20, radial_segments=24).translate(1.08, 0.0, 0.42),
        CylinderGeometry(0.033, 0.12, radial_segments=24)
        .rotate_x(math.pi / 2.0)
        .translate(0.98, 0.0, 0.35),
    )

    chain_loop = wire_from_points(
        [
            (0.98, -0.055, 0.41),
            (0.80, -0.055, 0.40),
            (0.64, -0.055, 0.34),
            (0.18, -0.055, 0.37),
            (0.02, -0.055, 0.34),
            (0.04, -0.055, 0.28),
            (0.52, -0.055, 0.26),
            (0.76, -0.055, 0.27),
            (0.98, -0.055, 0.29),
        ],
        radius=0.004,
        radial_segments=12,
        closed_path=True,
        cap_ends=False,
        corner_mode="fillet",
        corner_radius=0.030,
        corner_segments=10,
    )
    chain_loop.merge(
        _tube([(0.64, -0.055, 0.30), (0.60, -0.020, 0.29)], 0.003, samples=1, radial=10)
    )

    return {
        "frame": _save_mesh("recumbent_frame.obj", frame),
        "chain": _save_mesh("recumbent_chain.obj", chain_loop),
    }


def _build_fork_mesh() -> object:
    fork = _merge_geometries(
        CylinderGeometry(0.018, 0.24, radial_segments=20).translate(0.0, 0.0, 0.04),
        _tube(
            [
                (0.02, -0.022, -0.03),
                (0.12, -0.032, -0.12),
                (0.28, -0.040, -0.21),
            ],
            0.010,
            samples=12,
            radial=14,
        ),
        _tube(
            [
                (0.02, 0.022, -0.03),
                (0.12, 0.032, -0.12),
                (0.28, 0.040, -0.21),
            ],
            0.010,
            samples=12,
            radial=14,
        ),
        _tube([(0.0, 0.0, -0.04), (0.15, 0.0, -0.13)], 0.012, samples=4, radial=14),
        CylinderGeometry(0.015, 0.085, radial_segments=18)
        .rotate_x(math.pi / 2.0)
        .translate(0.17, 0.0, -0.15),
        CylinderGeometry(0.008, 0.082, radial_segments=16)
        .rotate_x(math.pi / 2.0)
        .translate(0.28, 0.0, -0.21),
        _tube(
            [
                (-0.02, 0.0, 0.08),
                (-0.05, 0.0, 0.18),
                (-0.12, 0.0, 0.26),
            ],
            0.014,
            samples=12,
            radial=16,
        ),
        _tube(
            [
                (-0.14, -0.20, 0.23),
                (-0.13, -0.10, 0.25),
                (-0.12, 0.0, 0.26),
                (-0.13, 0.10, 0.25),
                (-0.14, 0.20, 0.23),
            ],
            0.011,
            samples=14,
            radial=14,
        ),
    )
    return _save_mesh("recumbent_fork.obj", fork)


def _build_crank_meshes() -> dict[str, object]:
    crank_arms = _merge_geometries(
        CylinderGeometry(0.018, 0.14, radial_segments=24).rotate_x(math.pi / 2.0),
        _tube([(0.0, -0.03, 0.0), (0.17, -0.03, 0.0)], 0.012, samples=1, radial=14),
        _tube([(0.0, 0.03, 0.0), (-0.17, 0.03, 0.0)], 0.012, samples=1, radial=14),
        CylinderGeometry(0.009, 0.07, radial_segments=18)
        .rotate_x(math.pi / 2.0)
        .translate(0.17, -0.065, 0.0),
        CylinderGeometry(0.009, 0.07, radial_segments=18)
        .rotate_x(math.pi / 2.0)
        .translate(-0.17, 0.065, 0.0),
    )

    ring_geometries: list[MeshGeometry] = [
        TorusGeometry(0.090, 0.005, radial_segments=16, tubular_segments=44)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.03, 0.0),
        TorusGeometry(0.075, 0.004, radial_segments=16, tubular_segments=40)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.02, 0.0),
    ]
    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0
        ring_geometries.append(
            _tube(
                [
                    (0.0, -0.03, 0.0),
                    (0.070 * math.cos(angle), -0.03, 0.070 * math.sin(angle)),
                ],
                0.006,
                samples=1,
                radial=10,
            )
        )
    chainring = _merge_geometries(*ring_geometries)

    return {
        "arms": _save_mesh("recumbent_crank_arms.obj", crank_arms),
        "chainring": _save_mesh("recumbent_chainring.obj", chainring),
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recumbent_bicycle", assets=ASSETS)

    frame_paint = model.material("frame_paint", rgba=(0.16, 0.18, 0.22, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    chain_steel = model.material("chain_steel", rgba=(0.36, 0.37, 0.39, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    fabric = model.material("fabric", rgba=(0.13, 0.14, 0.15, 1.0))
    cleat_black = model.material("cleat_black", rgba=(0.10, 0.11, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.65, 0.54, 0.25, 1.0))

    frame_meshes = _build_frame_meshes()
    fork_mesh = _build_fork_mesh()
    crank_meshes = _build_crank_meshes()
    front_wheel_meshes = _build_wheel_meshes(
        "front_wheel",
        outer_radius=0.23,
        tire_radius=0.021,
        rim_radius=0.011,
        hub_radius=0.026,
        hub_length=0.080,
        spoke_count=18,
    )
    rear_wheel_meshes = _build_wheel_meshes(
        "rear_wheel",
        outer_radius=0.33,
        tire_radius=0.024,
        rim_radius=0.012,
        hub_radius=0.030,
        hub_length=0.135,
        spoke_count=24,
        add_cassette=True,
    )
    rear_valve_phase = math.radians(128.0)
    rear_valve_radius = float(rear_wheel_meshes["valve_mount_z"])

    main_frame = model.part("main_frame")
    main_frame.visual(frame_meshes["frame"], origin=Origin(), material=frame_paint)
    main_frame.visual(frame_meshes["chain"], origin=Origin(), material=chain_steel)
    main_frame.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.64, -0.05, 0.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    main_frame.visual(
        Box((0.30, 0.24, 0.03)),
        origin=Origin(xyz=(0.49, 0.0, 0.44), rpy=(0.0, 0.12, 0.0)),
        material=fabric,
    )
    main_frame.visual(
        Box((0.025, 0.24, 0.50)),
        origin=Origin(xyz=(0.46, 0.0, 0.60), rpy=(0.0, -0.60, 0.0)),
        material=fabric,
    )
    main_frame.inertial = Inertial.from_geometry(
        Box((1.25, 0.34, 0.78)),
        mass=13.0,
        origin=Origin(xyz=(0.54, 0.0, 0.43)),
    )

    front_fork = model.part("front_fork")
    front_fork.visual(fork_mesh, origin=Origin(), material=frame_paint)
    front_fork.visual(
        Cylinder(radius=0.016, length=0.10),
        origin=Origin(xyz=(-0.14, -0.19, 0.23), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
    )
    front_fork.visual(
        Cylinder(radius=0.016, length=0.10),
        origin=Origin(xyz=(-0.14, 0.19, 0.23), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_rubber,
    )
    front_fork.inertial = Inertial.from_geometry(
        Box((0.40, 0.45, 0.55)),
        mass=2.6,
        origin=Origin(xyz=(-0.02, 0.0, 0.03)),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(front_wheel_meshes["tire"], origin=Origin(), material=rubber)
    front_wheel.visual(front_wheel_meshes["metal"], origin=Origin(), material=aluminum)
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.23, length=0.08),
        mass=1.9,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(rear_wheel_meshes["tire"], origin=Origin(), material=rubber)
    rear_wheel.visual(rear_wheel_meshes["metal"], origin=Origin(), material=steel)
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.33, length=0.135),
        mass=2.7,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    crankset = model.part("crankset")
    crankset.visual(crank_meshes["arms"], origin=Origin(), material=frame_paint)
    crankset.visual(crank_meshes["chainring"], origin=Origin(), material=steel)
    crankset.inertial = Inertial.from_geometry(
        Cylinder(radius=0.12, length=0.14),
        mass=1.4,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    left_pedal = model.part("left_pedal")
    left_pedal.visual(
        Cylinder(radius=0.005, length=0.05),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    left_pedal.visual(
        Box((0.10, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
        material=grip_rubber,
    )
    left_pedal.visual(
        Box((0.085, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.060, 0.008)),
        material=aluminum,
    )

    right_pedal = model.part("right_pedal")
    right_pedal.visual(
        Cylinder(radius=0.005, length=0.05),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    right_pedal.visual(
        Box((0.10, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.060, 0.0)),
        material=grip_rubber,
    )
    right_pedal.visual(
        Box((0.085, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, -0.060, 0.008)),
        material=aluminum,
    )

    front_valve = model.part("front_valve")
    front_valve.visual(
        Cylinder(radius=0.003, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=brass,
    )

    rear_valve = model.part("rear_valve")
    rear_valve.visual(
        Cylinder(radius=0.003, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=brass,
    )

    left_cleat = model.part("left_cleat")
    left_cleat.visual(Box((0.035, 0.016, 0.010)), origin=Origin(), material=cleat_black)

    right_cleat = model.part("right_cleat")
    right_cleat.visual(Box((0.035, 0.016, 0.010)), origin=Origin(), material=cleat_black)

    model.articulation(
        "steering_head",
        ArticulationType.REVOLUTE,
        parent="main_frame",
        child="front_fork",
        origin=Origin(xyz=(1.08, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "front_hub",
        ArticulationType.CONTINUOUS,
        parent="front_fork",
        child="front_wheel",
        origin=Origin(xyz=(0.28, 0.0, -0.21)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=35.0,
        ),
    )
    model.articulation(
        "rear_hub",
        ArticulationType.CONTINUOUS,
        parent="main_frame",
        child="rear_wheel",
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=35.0,
        ),
    )
    model.articulation(
        "crank_rotation",
        ArticulationType.CONTINUOUS,
        parent="main_frame",
        child="crankset",
        origin=Origin(xyz=(0.98, 0.0, 0.35)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=18.0,
        ),
    )
    model.articulation(
        "left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="crankset",
        child="left_pedal",
        origin=Origin(xyz=(-0.17, 0.10, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=24.0,
        ),
    )
    model.articulation(
        "right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="crankset",
        child="right_pedal",
        origin=Origin(xyz=(0.17, -0.10, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=24.0,
        ),
    )
    model.articulation(
        "front_valve_mount",
        ArticulationType.FIXED,
        parent="front_wheel",
        child="front_valve",
        origin=Origin(xyz=(0.0, 0.0, float(front_wheel_meshes["valve_mount_z"]))),
    )
    model.articulation(
        "rear_valve_mount",
        ArticulationType.FIXED,
        parent="rear_wheel",
        child="rear_valve",
        origin=Origin(
            xyz=(
                rear_valve_radius * math.cos(rear_valve_phase),
                0.0,
                rear_valve_radius * math.sin(rear_valve_phase),
            )
        ),
    )
    model.articulation(
        "left_cleat_mount",
        ArticulationType.FIXED,
        parent="left_pedal",
        child="left_cleat",
        origin=Origin(xyz=(0.034, 0.060, 0.016)),
    )
    model.articulation(
        "right_cleat_mount",
        ArticulationType.FIXED,
        parent="right_pedal",
        child="right_cleat",
        origin=Origin(xyz=(0.034, -0.060, 0.016)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.allow_overlap(
        "front_wheel",
        "main_frame",
        reason="full-lock steering on a compact recumbent can trip conservative convex-hull QC around the front boom",
    )
    ctx.allow_overlap(
        "crankset",
        "front_fork",
        reason="cockpit and crank envelopes are close on a short-wheelbase recumbent, and convex-hull decomposition is conservative",
    )
    ctx.allow_overlap(
        "crankset",
        "front_wheel",
        reason="short-wheelbase recumbents can have a very tight pedal-wheel envelope, and convex-hull QC around the chainring and tire is conservative",
    )
    ctx.allow_overlap(
        "main_frame",
        "rear_valve",
        reason="the rear valve stem is a tiny rim-mounted detail that can nick conservative frame hulls in sampled wheel poses",
    )
    ctx.check_no_overlaps(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("front_fork", "main_frame")
    ctx.expect_aabb_contact("crankset", "main_frame")
    ctx.expect_aabb_contact("left_pedal", "crankset")
    ctx.expect_aabb_contact("right_pedal", "crankset")
    ctx.expect_aabb_contact("front_valve", "front_wheel")
    ctx.expect_aabb_contact("rear_valve", "rear_wheel")
    ctx.expect_aabb_overlap("left_cleat", "left_pedal", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("right_cleat", "right_pedal", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_gap("left_cleat", "left_pedal", axis="z", max_gap=0.001, max_penetration=0.001)
    ctx.expect_aabb_gap(
        "right_cleat", "right_pedal", axis="z", max_gap=0.001, max_penetration=0.001
    )
    ctx.expect_aabb_overlap("front_fork", "front_wheel", axes="xz", min_overlap=0.10)
    ctx.expect_aabb_overlap("rear_wheel", "main_frame", axes="xz", min_overlap=0.05)
    ctx.expect_origin_distance("front_wheel", "rear_wheel", axes="y", max_dist=0.001)
    ctx.expect_origin_distance("front_wheel", "crankset", axes="y", max_dist=0.001)

    def require(condition: bool, message: str) -> None:
        if not condition:
            raise AssertionError(message)

    def axis_is(name: str, expected: tuple[float, float, float]) -> None:
        articulation = object_model.get_articulation(name)
        require(
            articulation.articulation_type is not ArticulationType.FIXED,
            f"{name} must be an articulated joint",
        )
        require(
            articulation.axis is not None
            and all(
                abs(actual - target) < 1e-6 for actual, target in zip(articulation.axis, expected)
            ),
            f"{name} axis should be {expected}, got {articulation.axis}",
        )

    axis_is("steering_head", (0.0, 0.0, 1.0))
    axis_is("front_hub", (0.0, 1.0, 0.0))
    axis_is("rear_hub", (0.0, 1.0, 0.0))
    axis_is("crank_rotation", (0.0, 1.0, 0.0))
    axis_is("left_pedal_spin", (0.0, 1.0, 0.0))
    axis_is("right_pedal_spin", (0.0, 1.0, 0.0))

    steering = object_model.get_articulation("steering_head")
    require(
        steering.motion_limits is not None
        and steering.motion_limits.lower < 0.0
        and steering.motion_limits.upper > 0.0
        and (steering.motion_limits.upper - steering.motion_limits.lower) > 1.0,
        "Steering head needs realistic left/right travel",
    )

    rear_wheel_pos = ctx.part_world_position("rear_wheel")
    front_wheel_pos = ctx.part_world_position("front_wheel")
    crank_pos = ctx.part_world_position("crankset")
    require(
        front_wheel_pos[0] - rear_wheel_pos[0] > 1.10, "Wheelbase should read as a long recumbent"
    )
    require(
        crank_pos[0] > 0.90,
        "Bottom bracket should sit well forward of the seat for a recumbent layout",
    )
    require(
        front_wheel_pos[2] < rear_wheel_pos[2],
        "Front wheel should be smaller and lower than the rear wheel",
    )

    right_pedal_rest = ctx.part_world_position("right_pedal")
    left_pedal_rest = ctx.part_world_position("left_pedal")
    require(
        right_pedal_rest[0] > crank_pos[0], "Right pedal should start forward of the crank axis"
    )
    require(left_pedal_rest[0] < crank_pos[0], "Left pedal should start aft of the crank axis")

    with ctx.pose(steering_head=0.45):
        turned_left = ctx.part_world_position("front_wheel")
        ctx.expect_aabb_contact("front_fork", "main_frame")
    with ctx.pose(steering_head=-0.45):
        turned_right = ctx.part_world_position("front_wheel")
        ctx.expect_aabb_contact("front_fork", "main_frame")
    require(turned_left[1] > 0.04, "Positive steering should sweep the front wheel to rider left")
    require(
        turned_right[1] < -0.04, "Negative steering should sweep the front wheel to rider right"
    )

    with ctx.pose(crank_rotation=math.pi):
        right_pedal_inverted = ctx.part_world_position("right_pedal")
        left_pedal_inverted = ctx.part_world_position("left_pedal")
        ctx.expect_aabb_contact("left_pedal", "crankset")
        ctx.expect_aabb_contact("right_pedal", "crankset")
    require(
        right_pedal_inverted[0] < crank_pos[0] - 0.05,
        "Right pedal should swing behind the crank after half a turn",
    )
    require(
        left_pedal_inverted[0] > crank_pos[0] + 0.05,
        "Left pedal should swing ahead after half a turn",
    )

    with ctx.pose(crank_rotation=math.pi / 2.0):
        right_pedal_quarter = ctx.part_world_position("right_pedal")
    require(
        abs(right_pedal_quarter[2] - right_pedal_rest[2]) > 0.12,
        "Crank rotation should move the pedal through a large circular path",
    )

    front_valve_rest = ctx.part_world_position("front_valve")
    rear_valve_rest = ctx.part_world_position("rear_valve")
    front_wheel_center = ctx.part_world_position("front_wheel")
    rear_wheel_center = ctx.part_world_position("rear_wheel")
    with ctx.pose(front_hub=math.pi / 2.0, rear_hub=math.pi / 2.0):
        front_valve_quarter = ctx.part_world_position("front_valve")
        rear_valve_quarter = ctx.part_world_position("rear_valve")
        ctx.expect_aabb_contact("front_valve", "front_wheel")
        ctx.expect_aabb_contact("rear_valve", "rear_wheel")
    require(
        math.dist(front_valve_quarter, front_valve_rest) > 0.18,
        "Front wheel rotation should carry the valve around the rim",
    )
    require(
        math.dist(rear_valve_quarter, rear_valve_rest) > 0.25,
        "Rear wheel rotation should carry the valve around the rim",
    )
    require(
        abs(
            math.dist(front_valve_rest, front_wheel_center)
            - math.dist(front_valve_quarter, front_wheel_center)
        )
        < 1e-6,
        "Front valve should stay at a constant wheel radius while the wheel turns",
    )
    require(
        abs(
            math.dist(rear_valve_rest, rear_wheel_center)
            - math.dist(rear_valve_quarter, rear_wheel_center)
        )
        < 1e-6,
        "Rear valve should stay at a constant wheel radius while the wheel turns",
    )

    left_cleat_rest = ctx.part_world_position("left_cleat")
    right_cleat_rest = ctx.part_world_position("right_cleat")
    left_pedal_origin_rest = ctx.part_world_position("left_pedal")
    right_pedal_origin_rest = ctx.part_world_position("right_pedal")
    left_cleat_offset_rest = math.dist(left_cleat_rest, left_pedal_origin_rest)
    right_cleat_offset_rest = math.dist(right_cleat_rest, right_pedal_origin_rest)
    with ctx.pose(left_pedal_spin=math.pi / 2.0, right_pedal_spin=-math.pi / 2.0):
        left_cleat_spun = ctx.part_world_position("left_cleat")
        right_cleat_spun = ctx.part_world_position("right_cleat")
        left_pedal_origin_spun = ctx.part_world_position("left_pedal")
        right_pedal_origin_spun = ctx.part_world_position("right_pedal")
    require(
        abs(left_cleat_spun[0] - left_cleat_rest[0]) > 0.01
        and abs(left_cleat_spun[2] - left_cleat_rest[2]) > 0.01,
        "Left pedal spin should rotate its cleat mount around the spindle",
    )
    require(
        abs(right_cleat_spun[0] - right_cleat_rest[0]) > 0.01
        and abs(right_cleat_spun[2] - right_cleat_rest[2]) > 0.01,
        "Right pedal spin should rotate its cleat mount around the spindle",
    )
    require(
        abs(math.dist(left_cleat_spun, left_pedal_origin_spun) - left_cleat_offset_rest) < 1e-6,
        "Left cleat should remain rigidly mounted to the pedal through pedal spin",
    )
    require(
        abs(math.dist(right_cleat_spun, right_pedal_origin_spun) - right_cleat_offset_rest) < 1e-6,
        "Right cleat should remain rigidly mounted to the pedal through pedal spin",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
