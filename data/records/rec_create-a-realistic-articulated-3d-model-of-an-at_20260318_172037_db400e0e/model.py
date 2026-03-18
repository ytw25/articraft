from __future__ import annotations

import math

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(ASSETS.asset_root)
MESH_DIR = HERE / "meshes"
MESH_DIR.mkdir(parents=True, exist_ok=True)

WHEEL_RADIUS = 0.30
WHEEL_WIDTH = 0.19
FRONT_TRACK = 0.92
REAR_TRACK = 0.90
STEERING_ORIGIN = (0.40, 0.0, 0.42)
FRONT_LEFT_WHEEL_LOCAL = (0.22, FRONT_TRACK / 2.0, -0.12)
FRONT_RIGHT_WHEEL_LOCAL = (0.22, -FRONT_TRACK / 2.0, -0.12)
REAR_LEFT_WHEEL_POS = (-0.56, REAR_TRACK / 2.0, WHEEL_RADIUS)
REAR_RIGHT_WHEEL_POS = (-0.56, -REAR_TRACK / 2.0, WHEEL_RADIUS)
STEERING_LIMIT = 0.60


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _rotated_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (x * c - y * s, x * s + y * c)


def _expected_front_wheel_world(
    local_pos: tuple[float, float, float], angle: float
) -> tuple[float, float, float]:
    rx, ry = _rotated_xy(local_pos[0], local_pos[1], angle)
    return (
        STEERING_ORIGIN[0] + rx,
        STEERING_ORIGIN[1] + ry,
        STEERING_ORIGIN[2] + local_pos[2],
    )


def _add_tube(
    part, p1: tuple[float, float, float], p2: tuple[float, float, float], radius: float, material
) -> None:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    dz = p2[2] - p1[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return
    span_xy = math.sqrt(dx * dx + dy * dy)
    pitch = math.atan2(span_xy, dz)
    yaw = math.atan2(dy, dx)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=_midpoint(p1, p2), rpy=(0.0, pitch, yaw)),
        material=material,
    )


def _chassis_body_mesh():
    sections = [
        (-0.76, 0.18, 0.40, 0.72),
        (-0.60, 0.20, 0.50, 0.94),
        (-0.34, 0.18, 0.44, 0.74),
        (-0.08, 0.20, 0.46, 0.54),
        (0.10, 0.28, 0.46, 0.38),
        (0.28, 0.31, 0.44, 0.28),
        (0.42, 0.30, 0.40, 0.22),
    ]
    geom = superellipse_side_loft(sections, exponents=3.2, segments=64, cap=True, closed=True)
    geom.rotate_z(-math.pi / 2.0)
    return mesh_from_geometry(geom, MESH_DIR / "atv_chassis_body.obj")


def _handlebar_mesh():
    points = [
        (-0.10, -0.36, 0.27),
        (-0.04, -0.20, 0.31),
        (0.00, 0.00, 0.24),
        (-0.04, 0.20, 0.31),
        (-0.10, 0.36, 0.27),
    ]
    geom = tube_from_spline_points(
        points,
        radius=0.012,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, MESH_DIR / "atv_handlebar.obj")


def _build_wheel(part, tire_material, rim_material, hub_material) -> None:
    wheel_origin = Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=wheel_origin,
        material=tire_material,
    )
    part.visual(
        Cylinder(radius=0.19, length=0.155),
        origin=wheel_origin,
        material=rim_material,
    )
    part.visual(
        Cylinder(radius=0.07, length=WHEEL_WIDTH + 0.05),
        origin=wheel_origin,
        material=hub_material,
    )
    part.visual(
        Cylinder(radius=0.12, length=0.035),
        origin=Origin(xyz=(0.0, 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_material,
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=18.0,
        origin=wheel_origin,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="atv_quad_bike", assets=ASSETS)

    body_red = model.material("body_red", rgba=(0.60, 0.09, 0.08, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.08, 1.0))
    frame_black = model.material("frame_black", rgba=(0.14, 0.14, 0.15, 1.0))
    rubber = model.material("rubber", rgba=(0.03, 0.03, 0.03, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.59, 0.62, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    seat_vinyl = model.material("seat_vinyl", rgba=(0.07, 0.07, 0.08, 1.0))
    spring_yellow = model.material("spring_yellow", rgba=(0.82, 0.73, 0.10, 1.0))

    chassis = model.part("chassis")
    chassis.visual(_chassis_body_mesh(), material=body_red)
    chassis.visual(
        Box((0.34, 0.24, 0.11)),
        origin=Origin(xyz=(0.02, 0.0, 0.47)),
        material=body_red,
    )
    chassis.visual(
        Box((0.48, 0.31, 0.10)),
        origin=Origin(xyz=(-0.18, 0.0, 0.54)),
        material=seat_vinyl,
    )
    chassis.visual(
        Box((0.22, 0.31, 0.08)),
        origin=Origin(xyz=(-0.42, 0.0, 0.58)),
        material=seat_vinyl,
    )
    chassis.visual(
        Box((0.28, 0.28, 0.18)),
        origin=Origin(xyz=(-0.02, 0.0, 0.27)),
        material=frame_black,
    )
    chassis.visual(
        Cylinder(radius=0.045, length=0.18),
        origin=Origin(xyz=STEERING_ORIGIN),
        material=frame_black,
    )
    chassis.visual(
        Box((0.12, 0.18, 0.12)),
        origin=Origin(xyz=REAR_LEFT_WHEEL_POS),
        material=frame_black,
    )
    chassis.visual(
        Box((0.12, 0.18, 0.12)),
        origin=Origin(xyz=REAR_RIGHT_WHEEL_POS),
        material=frame_black,
    )
    chassis.visual(
        Box((0.30, 0.24, 0.05)),
        origin=Origin(xyz=(-0.12, 0.28, 0.23)),
        material=matte_black,
    )
    chassis.visual(
        Box((0.30, 0.24, 0.05)),
        origin=Origin(xyz=(-0.12, -0.28, 0.23)),
        material=matte_black,
    )
    _add_tube(chassis, (-0.54, -0.36, 0.30), (-0.54, 0.36, 0.30), 0.030, frame_black)
    _add_tube(chassis, (-0.38, -0.17, 0.28), (0.15, -0.15, 0.28), 0.025, frame_black)
    _add_tube(chassis, (-0.38, 0.17, 0.28), (0.15, 0.15, 0.28), 0.025, frame_black)
    _add_tube(chassis, (0.10, -0.11, 0.38), (0.38, -0.06, 0.41), 0.022, frame_black)
    _add_tube(chassis, (0.10, 0.11, 0.38), (0.38, 0.06, 0.41), 0.022, frame_black)
    _add_tube(chassis, (-0.30, 0.18, 0.32), (-0.48, 0.34, 0.48), 0.018, spring_yellow)
    _add_tube(chassis, (-0.30, -0.18, 0.32), (-0.48, -0.34, 0.48), 0.018, spring_yellow)
    chassis.inertial = Inertial.from_geometry(
        Box((1.40, 0.90, 0.55)),
        mass=185.0,
        origin=Origin(xyz=(-0.03, 0.0, 0.38)),
    )

    front_steering = model.part("front_steering")
    front_steering.visual(
        Cylinder(radius=0.035, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=frame_black,
    )
    front_steering.visual(
        Box((0.09, 0.20, 0.05)),
        origin=Origin(xyz=(-0.01, 0.0, 0.22)),
        material=frame_black,
    )
    front_steering.visual(_handlebar_mesh(), material=steel)
    front_steering.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(-0.10, 0.36, 0.27), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
    )
    front_steering.visual(
        Cylinder(radius=0.016, length=0.12),
        origin=Origin(xyz=(-0.10, -0.36, 0.27), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
    )
    front_steering.visual(
        Box((0.22, 0.52, 0.05)),
        origin=Origin(xyz=(0.10, 0.0, 0.14)),
        material=body_red,
    )
    front_steering.visual(
        Box((0.18, 0.24, 0.08)),
        origin=Origin(xyz=(0.16, 0.0, 0.07)),
        material=body_red,
    )
    front_steering.visual(
        Box((0.34, 0.22, 0.05)),
        origin=Origin(xyz=(0.19, 0.30, 0.17)),
        material=body_red,
    )
    front_steering.visual(
        Box((0.34, 0.22, 0.05)),
        origin=Origin(xyz=(0.19, -0.30, 0.17)),
        material=body_red,
    )
    front_steering.visual(
        Box((0.08, 0.12, 0.24)),
        origin=Origin(xyz=FRONT_LEFT_WHEEL_LOCAL),
        material=frame_black,
    )
    front_steering.visual(
        Box((0.08, 0.12, 0.24)),
        origin=Origin(xyz=FRONT_RIGHT_WHEEL_LOCAL),
        material=frame_black,
    )
    _add_tube(front_steering, (0.02, 0.12, -0.02), (0.19, 0.39, -0.10), 0.018, frame_black)
    _add_tube(front_steering, (0.02, 0.24, -0.02), (0.19, 0.39, -0.10), 0.018, frame_black)
    _add_tube(front_steering, (0.02, -0.12, -0.02), (0.19, -0.39, -0.10), 0.018, frame_black)
    _add_tube(front_steering, (0.02, -0.24, -0.02), (0.19, -0.39, -0.10), 0.018, frame_black)
    _add_tube(front_steering, (0.00, 0.11, 0.08), (0.18, 0.37, -0.01), 0.015, frame_black)
    _add_tube(front_steering, (0.00, -0.11, 0.08), (0.18, -0.37, -0.01), 0.015, frame_black)
    _add_tube(front_steering, (0.05, 0.18, 0.02), (0.18, 0.36, 0.13), 0.018, spring_yellow)
    _add_tube(front_steering, (0.05, -0.18, 0.02), (0.18, -0.36, 0.13), 0.018, spring_yellow)
    _add_tube(front_steering, (0.16, -0.32, -0.03), (0.16, 0.32, -0.03), 0.014, steel)
    front_steering.inertial = Inertial.from_geometry(
        Box((0.72, 0.98, 0.55)),
        mass=28.0,
        origin=Origin(xyz=(0.08, 0.0, 0.08)),
    )

    front_left_wheel = model.part("front_left_wheel")
    _build_wheel(front_left_wheel, rubber, aluminum, steel)

    front_right_wheel = model.part("front_right_wheel")
    _build_wheel(front_right_wheel, rubber, aluminum, steel)

    rear_left_wheel = model.part("rear_left_wheel")
    _build_wheel(rear_left_wheel, rubber, aluminum, steel)

    rear_right_wheel = model.part("rear_right_wheel")
    _build_wheel(rear_right_wheel, rubber, aluminum, steel)

    model.articulation(
        "front_steering_joint",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="front_steering",
        origin=Origin(xyz=STEERING_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.6,
            lower=-STEERING_LIMIT,
            upper=STEERING_LIMIT,
        ),
    )
    model.articulation(
        "front_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="front_steering",
        child="front_left_wheel",
        origin=Origin(xyz=FRONT_LEFT_WHEEL_LOCAL),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=22.0),
    )
    model.articulation(
        "front_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="front_steering",
        child="front_right_wheel",
        origin=Origin(xyz=FRONT_RIGHT_WHEEL_LOCAL),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=22.0),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="chassis",
        child="rear_left_wheel",
        origin=Origin(xyz=REAR_LEFT_WHEEL_POS),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=22.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="chassis",
        child="rear_right_wheel",
        origin=Origin(xyz=REAR_RIGHT_WHEEL_POS),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=110.0, velocity=22.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.allow_overlap(
        "front_steering",
        "chassis",
        reason="steering stem is intentionally seated through the head tube",
    )
    ctx.allow_overlap(
        "front_left_wheel",
        "front_steering",
        reason="front hub carrier intentionally nests inside the wheel center",
    )
    ctx.allow_overlap(
        "front_right_wheel",
        "front_steering",
        reason="front hub carrier intentionally nests inside the wheel center",
    )
    ctx.allow_overlap(
        "rear_left_wheel",
        "chassis",
        reason="rear axle support intentionally sits inside the wheel hub envelope",
    )
    ctx.allow_overlap(
        "rear_right_wheel",
        "chassis",
        reason="rear axle support intentionally sits inside the wheel hub envelope",
    )
    ctx.check_no_overlaps(
        max_pose_samples=128,
        overlap_tol=0.006,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("front_steering", "chassis")
    ctx.expect_aabb_contact("front_left_wheel", "front_steering")
    ctx.expect_aabb_contact("front_right_wheel", "front_steering")
    ctx.expect_aabb_contact("rear_left_wheel", "chassis")
    ctx.expect_aabb_contact("rear_right_wheel", "chassis")

    fl = tuple(ctx.part_world_position("front_left_wheel"))
    fr = tuple(ctx.part_world_position("front_right_wheel"))
    rl = tuple(ctx.part_world_position("rear_left_wheel"))
    rr = tuple(ctx.part_world_position("rear_right_wheel"))

    assert fl[0] > rl[0] + 1.0, "front axle should sit well ahead of rear axle"
    assert fr[0] > rr[0] + 1.0, "front axle should sit well ahead of rear axle"
    assert abs(fl[1] + fr[1]) < 0.02, "front wheels should be laterally symmetric"
    assert abs(rl[1] + rr[1]) < 0.02, "rear wheels should be laterally symmetric"
    assert 0.88 < abs(fl[1] - fr[1]) < 0.98, "front track width should be ATV-like"
    assert 0.86 < abs(rl[1] - rr[1]) < 0.96, "rear track width should be ATV-like"
    assert abs(fl[2] - WHEEL_RADIUS) < 0.02, "front wheel hubs should sit at tire radius height"
    assert abs(rl[2] - WHEEL_RADIUS) < 0.02, "rear wheel hubs should sit at tire radius height"

    with ctx.pose(front_steering_joint=0.45):
        ctx.expect_aabb_contact("front_left_wheel", "front_steering")
        ctx.expect_aabb_contact("front_right_wheel", "front_steering")
        fl_left = tuple(ctx.part_world_position("front_left_wheel"))
        fr_left = tuple(ctx.part_world_position("front_right_wheel"))
        exp_fl_left = _expected_front_wheel_world(FRONT_LEFT_WHEEL_LOCAL, 0.45)
        exp_fr_left = _expected_front_wheel_world(FRONT_RIGHT_WHEEL_LOCAL, 0.45)
        assert _distance(fl_left, exp_fl_left) < 0.03, (
            "left steering pose should place the left front wheel correctly"
        )
        assert _distance(fr_left, exp_fr_left) < 0.03, (
            "left steering pose should place the right front wheel correctly"
        )
        assert fl_left[1] > fl[1], "positive steering should move the left wheel toward +Y"
        assert fr_left[1] > fr[1], "positive steering should move the right wheel toward +Y"
        assert abs(tuple(ctx.part_world_position("rear_left_wheel"))[1] - rl[1]) < 1e-6, (
            "rear axle should stay fixed while steering"
        )

    with ctx.pose(front_steering_joint=-0.45):
        fl_right = tuple(ctx.part_world_position("front_left_wheel"))
        fr_right = tuple(ctx.part_world_position("front_right_wheel"))
        exp_fl_right = _expected_front_wheel_world(FRONT_LEFT_WHEEL_LOCAL, -0.45)
        exp_fr_right = _expected_front_wheel_world(FRONT_RIGHT_WHEEL_LOCAL, -0.45)
        assert _distance(fl_right, exp_fl_right) < 0.03, (
            "right steering pose should place the left front wheel correctly"
        )
        assert _distance(fr_right, exp_fr_right) < 0.03, (
            "right steering pose should place the right front wheel correctly"
        )
        assert fl_right[1] < fl[1], "negative steering should move the left wheel toward -Y"
        assert fr_right[1] < fr[1], "negative steering should move the right wheel toward -Y"

    with ctx.pose(front_left_wheel_spin=1.4, rear_right_wheel_spin=2.2):
        fl_spin = tuple(ctx.part_world_position("front_left_wheel"))
        rr_spin = tuple(ctx.part_world_position("rear_right_wheel"))
        assert _distance(fl_spin, fl) < 1e-6, "wheel spin should not translate the front left wheel"
        assert _distance(rr_spin, rr) < 1e-6, "wheel spin should not translate the rear right wheel"

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
