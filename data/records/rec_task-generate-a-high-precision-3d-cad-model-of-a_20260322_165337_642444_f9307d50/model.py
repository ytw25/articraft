from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

MODEL_NAME = "deep_groove_ball_bearing"

# ISO 6204-class metric bearing proportions.
INNER_DIAMETER = 0.020
OUTER_DIAMETER = 0.047
BEARING_WIDTH = 0.014
INNER_RADIUS = INNER_DIAMETER / 2.0
OUTER_RADIUS = OUTER_DIAMETER / 2.0
HALF_WIDTH = BEARING_WIDTH / 2.0

BALL_COUNT = 8
BALL_RADIUS = 0.00375
BALL_DIAMETER = BALL_RADIUS * 2.0
PITCH_RADIUS = 0.0160

# Deep-groove raceway curvature slightly larger than the ball radius so the
# toroidal grooves read as precision-machined bearing tracks rather than a
# simple cylindrical gap.
RACEWAY_RADIUS = 0.00400
RACEWAY_TRACK_OFFSET = RACEWAY_RADIUS - BALL_RADIUS
INNER_RACEWAY_TRACK_RADIUS = PITCH_RADIUS + RACEWAY_TRACK_OFFSET
OUTER_RACEWAY_TRACK_RADIUS = PITCH_RADIUS - RACEWAY_TRACK_OFFSET

INNER_RING_SHOULDER_RADIUS = 0.01255
OUTER_RING_BORE_RADIUS = 0.01945
EDGE_BEVEL = 0.00055

OUTER_RACEWAY_LAND_Z = math.sqrt(
    max(
        RACEWAY_RADIUS**2 - (OUTER_RING_BORE_RADIUS - OUTER_RACEWAY_TRACK_RADIUS) ** 2,
        0.0,
    )
)
INNER_RACEWAY_LAND_Z = math.sqrt(
    max(
        RACEWAY_RADIUS**2 - (INNER_RACEWAY_TRACK_RADIUS - INNER_RING_SHOULDER_RADIUS) ** 2,
        0.0,
    )
)

CAGE_INNER_RADIUS = 0.01355
CAGE_OUTER_RADIUS = 0.01845
CAGE_HALF_THICKNESS = 0.00110
CAGE_HALF_Z = 0.00255
POCKET_HOLE_RADIUS = 0.00230
RIVET_RADIUS = 0.00045
RIVET_LENGTH = 0.00545

BALL_PART_NAMES = [f"ball_{index:02d}" for index in range(1, BALL_COUNT + 1)]


def _ball_centers() -> list[tuple[float, float, float]]:
    centers: list[tuple[float, float, float]] = []
    for index in range(BALL_COUNT):
        angle = 2.0 * math.pi * index / BALL_COUNT
        centers.append(
            (
                PITCH_RADIUS * math.cos(angle),
                PITCH_RADIUS * math.sin(angle),
                0.0,
            )
        )
    return centers


def _circle_profile(
    radius: float,
    *,
    segments: int,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    center_x, center_y = center
    points: list[tuple[float, float]] = []
    for step in range(segments):
        angle = 2.0 * math.pi * step / segments
        points.append(
            (
                center_x + radius * math.cos(angle),
                center_y + radius * math.sin(angle),
            )
        )
    return points


def _raceway_profile_points(
    track_radius: float,
    shoulder_radius: float,
    land_z: float,
    *,
    inward: bool,
    steps: int = 24,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for step in range(steps + 1):
        blend = step / steps
        z = land_z - 2.0 * land_z * blend
        radial_term = math.sqrt(max(RACEWAY_RADIUS**2 - z**2, 0.0))
        radius = track_radius - radial_term if inward else track_radius + radial_term
        if step == 0:
            radius = shoulder_radius
        elif step == steps:
            radius = shoulder_radius
        points.append((radius, z))
    return points


def _revolve_profile(
    profile: list[tuple[float, float]],
    *,
    radial_segments: int,
) -> MeshGeometry:
    geom = MeshGeometry()
    point_count = len(profile)
    for segment in range(radial_segments):
        angle = 2.0 * math.pi * segment / radial_segments
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        for radius, z in profile:
            geom.add_vertex(radius * cos_angle, radius * sin_angle, z)

    for segment in range(radial_segments):
        next_segment = (segment + 1) % radial_segments
        base = segment * point_count
        next_base = next_segment * point_count
        for index in range(point_count):
            next_index = (index + 1) % point_count
            a = base + index
            b = base + next_index
            c = next_base + next_index
            d = next_base + index
            geom.add_face(a, d, c)
            geom.add_face(a, c, b)
    return geom


def _outer_ring_profile() -> list[tuple[float, float]]:
    profile = [
        (OUTER_RING_BORE_RADIUS, -HALF_WIDTH),
        (OUTER_RADIUS - EDGE_BEVEL, -HALF_WIDTH),
        (OUTER_RADIUS, -HALF_WIDTH + EDGE_BEVEL),
        (OUTER_RADIUS, HALF_WIDTH - EDGE_BEVEL),
        (OUTER_RADIUS - EDGE_BEVEL, HALF_WIDTH),
        (OUTER_RING_BORE_RADIUS, HALF_WIDTH),
        (OUTER_RING_BORE_RADIUS, OUTER_RACEWAY_LAND_Z),
    ]
    profile.extend(
        _raceway_profile_points(
            OUTER_RACEWAY_TRACK_RADIUS,
            OUTER_RING_BORE_RADIUS,
            OUTER_RACEWAY_LAND_Z,
            inward=False,
        )[1:]
    )
    profile.append((OUTER_RING_BORE_RADIUS, -HALF_WIDTH))
    return profile


def _inner_ring_profile() -> list[tuple[float, float]]:
    profile = [
        (INNER_RADIUS, -HALF_WIDTH + EDGE_BEVEL),
        (INNER_RADIUS + EDGE_BEVEL, -HALF_WIDTH),
        (INNER_RING_SHOULDER_RADIUS, -HALF_WIDTH),
        (INNER_RING_SHOULDER_RADIUS, -INNER_RACEWAY_LAND_Z),
    ]
    profile.extend(
        _raceway_profile_points(
            INNER_RACEWAY_TRACK_RADIUS,
            INNER_RING_SHOULDER_RADIUS,
            INNER_RACEWAY_LAND_Z,
            inward=True,
        )[1:]
    )
    profile.extend(
        [
            (INNER_RING_SHOULDER_RADIUS, HALF_WIDTH),
            (INNER_RADIUS + EDGE_BEVEL, HALF_WIDTH),
            (INNER_RADIUS, HALF_WIDTH - EDGE_BEVEL),
        ]
    )
    return profile


def _outer_ring_mesh() -> MeshGeometry:
    return _revolve_profile(_outer_ring_profile(), radial_segments=112)


def _inner_ring_mesh() -> MeshGeometry:
    return _revolve_profile(_inner_ring_profile(), radial_segments=112)


def _cage_half_mesh(z_offset: float) -> MeshGeometry:
    outer_profile = _circle_profile(CAGE_OUTER_RADIUS, segments=72)
    hole_profiles = [_circle_profile(CAGE_INNER_RADIUS, segments=72)]
    for center_x, center_y, _ in _ball_centers():
        hole_profiles.append(
            _circle_profile(
                POCKET_HOLE_RADIUS,
                segments=32,
                center=(center_x, center_y),
            )
        )
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        height=CAGE_HALF_THICKNESS,
        center=True,
        cap=True,
        closed=True,
    )
    geom.translate(0.0, 0.0, z_offset)
    return geom


def _assert_close(value: float, target: float, tol: float, message: str) -> None:
    if abs(value - target) > tol:
        raise AssertionError(f"{message}: got {value:.6f}, expected {target:.6f} ± {tol:.6f}")


def _load_obj_vertices(path: Path) -> list[tuple[float, float, float]]:
    vertices: list[tuple[float, float, float]] = []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            _, x_str, y_str, z_str = line.split()[:4]
            vertices.append((float(x_str), float(y_str), float(z_str)))
    if not vertices:
        raise AssertionError(f"mesh file {path} has no vertices")
    return vertices


def _mesh_radial_bounds(path: Path) -> tuple[float, float, float]:
    vertices = _load_obj_vertices(path)
    radii = [math.hypot(x_coord, y_coord) for x_coord, y_coord, _ in vertices]
    z_values = [z_coord for _, _, z_coord in vertices]
    return min(radii), max(radii), max(z_values) - min(z_values)


def _mesh_z_bounds(path: Path) -> tuple[float, float]:
    vertices = _load_obj_vertices(path)
    z_values = [z_coord for _, _, z_coord in vertices]
    return min(z_values), max(z_values)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name=MODEL_NAME, assets=ASSETS)

    polished_steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    chrome_steel = model.material("chrome_steel", rgba=(0.84, 0.86, 0.89, 1.0))
    brass_cage = model.material("brass_cage", rgba=(0.74, 0.61, 0.30, 1.0))

    outer_ring = model.part("outer_ring")
    outer_ring.visual(
        mesh_from_geometry(_outer_ring_mesh(), ASSETS.mesh_path("outer_ring.obj")),
        material=polished_steel,
        name="outer_ring_shell",
    )
    outer_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=OUTER_RADIUS, length=BEARING_WIDTH),
        mass=0.12,
        origin=Origin(),
    )

    inner_ring = model.part("inner_ring")
    inner_ring.visual(
        mesh_from_geometry(_inner_ring_mesh(), ASSETS.mesh_path("inner_ring.obj")),
        material=polished_steel,
        name="inner_ring_shell",
    )
    inner_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=INNER_RING_SHOULDER_RADIUS, length=BEARING_WIDTH),
        mass=0.07,
        origin=Origin(),
    )

    cage = model.part("cage")
    cage.visual(
        mesh_from_geometry(_cage_half_mesh(CAGE_HALF_Z), ASSETS.mesh_path("cage_front_half.obj")),
        material=brass_cage,
        name="front_ribbon",
    )
    cage.visual(
        mesh_from_geometry(_cage_half_mesh(-CAGE_HALF_Z), ASSETS.mesh_path("cage_rear_half.obj")),
        material=brass_cage,
        name="rear_ribbon",
    )
    for index in range(BALL_COUNT):
        angle = 2.0 * math.pi * (index + 0.5) / BALL_COUNT
        cage.visual(
            Cylinder(radius=RIVET_RADIUS, length=RIVET_LENGTH),
            origin=Origin(
                xyz=(
                    PITCH_RADIUS * math.cos(angle),
                    PITCH_RADIUS * math.sin(angle),
                    0.0,
                )
            ),
            material=brass_cage,
            name=f"rivet_{index + 1:02d}",
        )
    cage.inertial = Inertial.from_geometry(
        Cylinder(radius=CAGE_OUTER_RADIUS, length=RIVET_LENGTH),
        mass=0.018,
        origin=Origin(),
    )

    for index, (center_x, center_y, center_z) in enumerate(_ball_centers(), start=1):
        ball_name = f"ball_{index:02d}"
        ball = model.part(ball_name)
        ball.visual(
            Sphere(radius=BALL_RADIUS),
            material=chrome_steel,
            name="ball_body",
        )
        ball.inertial = Inertial.from_geometry(
            Sphere(radius=BALL_RADIUS),
            mass=0.0021,
            origin=Origin(),
        )
        model.articulation(
            f"outer_ring_to_{ball_name}",
            ArticulationType.FIXED,
            parent="outer_ring",
            child=ball_name,
            origin=Origin(xyz=(center_x, center_y, center_z)),
        )

    model.articulation(
        "outer_ring_to_cage",
        ArticulationType.FIXED,
        parent="outer_ring",
        child="cage",
        origin=Origin(),
    )
    model.articulation(
        "outer_ring_to_inner_ring",
        ArticulationType.CONTINUOUS,
        parent="outer_ring",
        child="inner_ring",
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance("inner_ring", "outer_ring", axes="xy", max_dist=0.0001)
    ctx.expect_origin_distance("cage", "outer_ring", axes="xy", max_dist=0.0001)
    ctx.expect_aabb_overlap("inner_ring", "outer_ring", axes="z", min_overlap=0.0135)
    ctx.expect_aabb_overlap("cage", "outer_ring", axes="z", min_overlap=0.0040)
    ctx.expect_aabb_overlap("cage", "outer_ring", axes="xy", min_overlap=0.025)

    # Rolling elements must be seated in both raceways and retained by the cage.
    for ball_name in ("ball_01", "ball_03", "ball_05", "ball_07"):
        ctx.expect_aabb_contact(ball_name, "inner_ring")
        ctx.expect_aabb_contact(ball_name, "outer_ring")
        ctx.expect_aabb_contact(ball_name, "cage")
        ctx.expect_aabb_overlap(ball_name, "outer_ring", axes="z", min_overlap=BALL_DIAMETER * 0.95)

    outer_pos = ctx.part_world_position("outer_ring")
    inner_pos = ctx.part_world_position("inner_ring")
    cage_pos = ctx.part_world_position("cage")
    _assert_close(outer_pos[0], 0.0, 1e-6, "outer ring x origin")
    _assert_close(outer_pos[1], 0.0, 1e-6, "outer ring y origin")
    _assert_close(inner_pos[0], 0.0, 1e-6, "inner ring x origin")
    _assert_close(inner_pos[1], 0.0, 1e-6, "inner ring y origin")
    _assert_close(cage_pos[0], 0.0, 1e-6, "cage x origin")
    _assert_close(cage_pos[1], 0.0, 1e-6, "cage y origin")

    ball_positions = [ctx.part_world_position(ball_name) for ball_name in BALL_PART_NAMES]
    ball_angles = sorted(math.atan2(position[1], position[0]) % (2.0 * math.pi) for position in ball_positions)
    for index, position in enumerate(ball_positions):
        radius = math.hypot(position[0], position[1])
        _assert_close(radius, PITCH_RADIUS, 5e-5, f"{BALL_PART_NAMES[index]} pitch radius")
        _assert_close(position[2], 0.0, 1e-6, f"{BALL_PART_NAMES[index]} z position")
    expected_spacing = 2.0 * math.pi / BALL_COUNT
    for index, angle in enumerate(ball_angles):
        next_angle = ball_angles[(index + 1) % BALL_COUNT]
        if index == BALL_COUNT - 1:
            next_angle += 2.0 * math.pi
        _assert_close(next_angle - angle, expected_spacing, 2e-3, "ball angular spacing")

    with ctx.pose(outer_ring_to_inner_ring=math.pi / 2.0):
        ctx.expect_origin_distance("inner_ring", "outer_ring", axes="xy", max_dist=0.0001)
        ctx.expect_aabb_contact("ball_01", "inner_ring")
        posed_ball = ctx.part_world_position("ball_01")
        _assert_close(math.hypot(posed_ball[0], posed_ball[1]), PITCH_RADIUS, 5e-5, "ball_01 pitch radius at 90 deg")

    assert len(object_model.parts) == BALL_COUNT + 3, "bearing should contain two rings, one cage, and eight balls"
    root_part_names = [part.name if hasattr(part, "name") else str(part) for part in object_model.root_parts()]
    assert root_part_names == ["outer_ring"], f"unexpected root parts: {root_part_names}"

    spin_joint = object_model.get_articulation("outer_ring_to_inner_ring")
    assert spin_joint.articulation_type == ArticulationType.CONTINUOUS, "inner ring should rotate continuously"
    assert tuple(spin_joint.axis) == (0.0, 0.0, 1.0), "bearing spin axis should align with +Z"

    outer_min_r, outer_max_r, outer_width = _mesh_radial_bounds(Path(ASSETS.mesh_path("outer_ring.obj")))
    inner_min_r, inner_max_r, inner_width = _mesh_radial_bounds(Path(ASSETS.mesh_path("inner_ring.obj")))
    cage_front_z_min, cage_front_z_max = _mesh_z_bounds(Path(ASSETS.mesh_path("cage_front_half.obj")))
    cage_rear_z_min, cage_rear_z_max = _mesh_z_bounds(Path(ASSETS.mesh_path("cage_rear_half.obj")))

    _assert_close(outer_max_r * 2.0, OUTER_DIAMETER, 2e-4, "outer ring outside diameter")
    _assert_close(outer_min_r * 2.0, OUTER_RING_BORE_RADIUS * 2.0, 2e-4, "outer ring bore diameter")
    _assert_close(outer_width, BEARING_WIDTH, 2e-4, "outer ring width")
    _assert_close(inner_min_r * 2.0, INNER_DIAMETER, 2e-4, "inner ring bore diameter")
    _assert_close(inner_max_r * 2.0, INNER_RING_SHOULDER_RADIUS * 2.0, 2e-4, "inner ring shoulder diameter")
    _assert_close(inner_width, BEARING_WIDTH, 2e-4, "inner ring width")
    _assert_close(cage_front_z_max - cage_front_z_min, CAGE_HALF_THICKNESS, 2e-4, "front cage half thickness")
    _assert_close(cage_rear_z_max - cage_rear_z_min, CAGE_HALF_THICKNESS, 2e-4, "rear cage half thickness")
    _assert_close((cage_front_z_max + cage_front_z_min) / 2.0, CAGE_HALF_Z, 2e-4, "front cage half offset")
    _assert_close((cage_rear_z_max + cage_rear_z_min) / 2.0, -CAGE_HALF_Z, 2e-4, "rear cage half offset")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
