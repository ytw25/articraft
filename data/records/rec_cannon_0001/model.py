from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _cheek_outline() -> list[tuple[float, float]]:
    return [
        (-0.46, -0.24),
        (-0.48, -0.05),
        (-0.36, -0.01),
        (-0.20, 0.01),
        (0.02, 0.03),
        (0.26, 0.02),
        (0.54, 0.00),
        (0.66, -0.06),
        (0.64, -0.22),
        (0.18, -0.28),
        (-0.14, -0.27),
    ]


def _build_barrel_body_mesh():
    profile = [
        (0.0, -0.49),
        (0.028, -0.49),
        (0.040, -0.46),
        (0.062, -0.44),
        (0.154, -0.42),
        (0.170, -0.33),
        (0.172, -0.18),
        (0.166, -0.04),
        (0.160, 0.10),
        (0.148, 0.34),
        (0.136, 0.62),
        (0.122, 0.88),
        (0.130, 0.98),
        (0.146, 1.06),
        (0.152, 1.12),
        (0.0, 1.12),
    ]
    return _save_mesh(LatheGeometry(profile, segments=64), "field_cannon_barrel.obj")


def _build_cheek_mesh():
    return _save_mesh(
        ExtrudeGeometry.centered(_cheek_outline(), height=0.07, cap=True, closed=True),
        "field_cannon_cheek.obj",
    )


def _build_dolphin_mesh(sign: float, filename: str):
    return _save_mesh(
        tube_from_spline_points(
            [
                (-0.20, 0.080 * sign, 0.082),
                (-0.16, 0.068 * sign, 0.132),
                (-0.11, 0.054 * sign, 0.170),
                (-0.05, 0.060 * sign, 0.120),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
        filename,
    )


def _populate_wheel(part, tire_mesh, rim_mesh, wood_material, iron_material) -> None:
    part.visual(tire_mesh, origin=Origin(rpy=(pi / 2, 0.0, 0.0)), material=iron_material)
    part.visual(rim_mesh, origin=Origin(rpy=(pi / 2, 0.0, 0.0)), material=wood_material)
    part.visual(
        Cylinder(radius=0.122, length=0.20),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=wood_material,
    )
    part.visual(
        Cylinder(radius=0.084, length=0.24),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=iron_material,
    )
    for side in (-1.0, 1.0):
        part.visual(
            Cylinder(radius=0.103, length=0.028),
            origin=Origin(xyz=(0.0, 0.050 * side, 0.0), rpy=(pi / 2, 0.0, 0.0)),
            material=iron_material,
        )
    for spoke_idx in range(12):
        angle = spoke_idx * (pi / 6.0)
        part.visual(
            Box((0.54, 0.034, 0.030)),
            origin=Origin(
                xyz=(0.30 * cos(angle), 0.0, 0.30 * sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=wood_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_cannon", assets=ASSETS)

    bronze = model.material("aged_bronze", rgba=(0.67, 0.50, 0.28, 1.0))
    carriage_green = model.material("carriage_green", rgba=(0.19, 0.25, 0.19, 1.0))
    wood = model.material("seasoned_oak", rgba=(0.43, 0.29, 0.18, 1.0))
    iron = model.material("forged_iron", rgba=(0.14, 0.15, 0.16, 1.0))

    barrel_body_mesh = _build_barrel_body_mesh()
    cheek_mesh = _build_cheek_mesh()
    wheel_tire_mesh = _save_mesh(
        TorusGeometry(radius=0.625, tube=0.028, radial_segments=20, tubular_segments=48),
        "field_cannon_wheel_tire.obj",
    )
    wheel_rim_mesh = _save_mesh(
        TorusGeometry(radius=0.575, tube=0.044, radial_segments=20, tubular_segments=48),
        "field_cannon_wheel_rim.obj",
    )
    trail_ring_mesh = _save_mesh(
        TorusGeometry(radius=0.055, tube=0.010, radial_segments=16, tubular_segments=32),
        "field_cannon_trail_ring.obj",
    )
    dolphin_left_mesh = _build_dolphin_mesh(1.0, "field_cannon_dolphin_left.obj")
    dolphin_right_mesh = _build_dolphin_mesh(-1.0, "field_cannon_dolphin_right.obj")

    frame = model.part("carriage_frame")
    frame.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, 0.190, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=carriage_green,
    )
    frame.visual(
        cheek_mesh,
        origin=Origin(xyz=(0.0, -0.190, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=carriage_green,
    )
    frame.visual(
        Box((0.20, 0.40, 0.16)),
        origin=Origin(xyz=(-0.18, 0.0, -0.13)),
        material=carriage_green,
    )
    frame.visual(
        Box((0.18, 0.38, 0.14)),
        origin=Origin(xyz=(0.46, 0.0, -0.10)),
        material=carriage_green,
    )
    frame.visual(
        Box((0.70, 0.34, 0.07)),
        origin=Origin(xyz=(0.10, 0.0, -0.16)),
        material=carriage_green,
    )
    frame.visual(
        Box((0.18, 0.35, 0.06)),
        origin=Origin(xyz=(0.00, 0.0, 0.01)),
        material=iron,
    )
    frame.visual(
        Box((0.22, 0.22, 0.10)),
        origin=Origin(xyz=(-0.55, 0.0, -0.23)),
        material=carriage_green,
    )
    frame.visual(
        Cylinder(radius=0.086, length=1.24),
        origin=Origin(xyz=(-0.12, 0.0, -0.20), rpy=(pi / 2, 0.0, 0.0)),
        material=iron,
    )
    for side in (-1.0, 1.0):
        frame.visual(
            Cylinder(radius=0.105, length=0.08),
            origin=Origin(
                xyz=(-0.12, 0.56 * side, -0.20),
                rpy=(pi / 2, 0.0, 0.0),
            ),
            material=iron,
        )
        frame.visual(
            Box((0.18, 0.08, 0.04)),
            origin=Origin(xyz=(0.01, 0.190 * side, 0.030)),
            material=iron,
        )
        frame.visual(
            Box((1.06, 0.09, 0.15)),
            origin=Origin(
                xyz=(-0.76, 0.115 * side, -0.34),
                rpy=(0.0, 0.22, -0.06 * side),
            ),
            material=carriage_green,
        )
    frame.visual(
        Box((0.14, 0.18, 0.18)),
        origin=Origin(xyz=(-1.30, 0.0, -0.49)),
        material=carriage_green,
    )
    frame.visual(
        Box((0.08, 0.20, 0.22)),
        origin=Origin(xyz=(-1.38, 0.0, -0.64), rpy=(0.0, -0.40, 0.0)),
        material=iron,
    )
    frame.visual(
        trail_ring_mesh,
        origin=Origin(xyz=(-1.36, 0.0, -0.46), rpy=(0.0, pi / 2, 0.0)),
        material=iron,
    )
    frame.inertial = Inertial.from_geometry(
        Box((2.15, 1.46, 0.90)),
        mass=920.0,
        origin=Origin(xyz=(-0.36, 0.0, -0.32)),
    )

    left_wheel = model.part("left_wheel")
    _populate_wheel(left_wheel, wheel_tire_mesh, wheel_rim_mesh, wood, iron)
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.66, length=0.12),
        mass=74.0,
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    _populate_wheel(right_wheel, wheel_tire_mesh, wheel_rim_mesh, wood, iron)
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.66, length=0.12),
        mass=74.0,
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
    )

    barrel = model.part("barrel")
    barrel.visual(barrel_body_mesh, origin=Origin(rpy=(0.0, pi / 2, 0.0)), material=bronze)
    barrel.visual(
        Cylinder(radius=0.062, length=0.48),
        origin=Origin(rpy=(pi / 2, 0.0, 0.0)),
        material=bronze,
    )
    barrel.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(xyz=(-0.48, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=bronze,
    )
    barrel.visual(Sphere(radius=0.047), origin=Origin(xyz=(-0.54, 0.0, 0.0)), material=bronze)
    barrel.visual(
        Cylinder(radius=0.056, length=0.025),
        origin=Origin(xyz=(1.108, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=iron,
    )
    barrel.visual(
        Box((0.050, 0.030, 0.026)),
        origin=Origin(xyz=(-0.29, 0.0, 0.17)),
        material=iron,
    )
    barrel.visual(
        Box((0.040, 0.022, 0.020)),
        origin=Origin(xyz=(0.97, 0.0, 0.14)),
        material=iron,
    )
    barrel.visual(dolphin_left_mesh, material=bronze)
    barrel.visual(dolphin_right_mesh, material=bronze)
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.15, length=1.56),
        mass=620.0,
        origin=Origin(xyz=(0.18, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
    )

    model.articulation(
        "frame_to_left_wheel",
        ArticulationType.FIXED,
        parent="carriage_frame",
        child="left_wheel",
        origin=Origin(xyz=(-0.12, 0.62, -0.20)),
    )
    model.articulation(
        "frame_to_right_wheel",
        ArticulationType.FIXED,
        parent="carriage_frame",
        child="right_wheel",
        origin=Origin(xyz=(-0.12, -0.62, -0.20)),
    )
    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent="carriage_frame",
        child="barrel",
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4000.0,
            velocity=0.80,
            lower=-0.18,
            upper=0.58,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(
        max_pose_samples=160,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("left_wheel", "carriage_frame")
    ctx.expect_aabb_contact("right_wheel", "carriage_frame")
    ctx.expect_aabb_overlap("left_wheel", "carriage_frame", axes="xz", min_overlap=0.30)
    ctx.expect_aabb_overlap("right_wheel", "carriage_frame", axes="xz", min_overlap=0.30)
    ctx.expect_origin_distance("left_wheel", "right_wheel", axes="xz", max_dist=0.001)
    ctx.expect_origin_distance("barrel", "carriage_frame", axes="xy", max_dist=0.001)
    ctx.expect_aabb_contact("barrel", "carriage_frame")
    ctx.expect_aabb_overlap("barrel", "carriage_frame", axes="xy", min_overlap=0.40)
    ctx.expect_joint_motion_axis(
        "barrel_elevation",
        "barrel",
        world_axis="z",
        direction="positive",
        min_delta=0.06,
    )

    with ctx.pose(barrel_elevation=-0.18):
        ctx.expect_origin_distance("barrel", "carriage_frame", axes="y", max_dist=0.001)
        ctx.expect_aabb_contact("barrel", "carriage_frame")
        ctx.expect_aabb_overlap("barrel", "carriage_frame", axes="xy", min_overlap=0.40)

    with ctx.pose(barrel_elevation=0.58):
        ctx.expect_origin_distance("barrel", "carriage_frame", axes="y", max_dist=0.001)
        ctx.expect_aabb_contact("barrel", "carriage_frame")
        ctx.expect_aabb_overlap("barrel", "carriage_frame", axes="xy", min_overlap=0.28)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
