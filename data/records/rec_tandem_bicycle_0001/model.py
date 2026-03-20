from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir

REAR_AXLE = (-0.71, 0.0, 0.34)
FRONT_AXLE = (0.88, 0.0, 0.34)
HEAD_BOTTOM = (0.65, 0.0, 0.70)
HEAD_TOP = (0.59, 0.0, 0.88)
CAPTAIN_BB = (0.08, 0.0, 0.29)
STOKER_BB = (-0.38, 0.0, 0.29)
CAPTAIN_SEAT_CLUSTER = (0.02, 0.0, 0.80)
STOKER_SEAT_CLUSTER = (-0.42, 0.0, 0.78)
CAPTAIN_SADDLE = (0.00, 0.0, 0.92)
STOKER_SADDLE = (-0.44, 0.0, 0.89)

PEDAL_OUTBOARD = 0.085
CRANK_ARM = 0.16


def _sub(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(v: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def _length(v: tuple[float, float, float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _unit(v: tuple[float, float, float]) -> tuple[float, float, float]:
    mag = _length(v)
    return (v[0] / mag, v[1] / mag, v[2] / mag)


def _mid(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _segment_origin(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx, dy, dz = _sub(b, a)
    length = max(_length((dx, dy, dz)), 1e-6)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    return Origin(xyz=_mid(a, b), rpy=(0.0, pitch, yaw)), length


def _add_tube(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
) -> None:
    origin, length = _segment_origin(a, b)
    part.visual(Cylinder(radius=radius, length=length), origin=origin)


def _save_mesh(geometry, name: str):
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, MESH_DIR / name)


def _make_wheel_mesh(name: str):
    geom = TorusGeometry(radius=0.322, tube=0.0185, radial_segments=18, tubular_segments=56)
    geom.rotate_x(math.pi / 2.0)
    geom.merge(
        TorusGeometry(radius=0.304, tube=0.008, radial_segments=16, tubular_segments=56).rotate_x(
            math.pi / 2.0
        )
    )
    geom.merge(
        CylinderGeometry(radius=0.016, height=0.022, radial_segments=24)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.026, 0.0)
    )
    geom.merge(
        CylinderGeometry(radius=0.016, height=0.022, radial_segments=24)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, -0.026, 0.0)
    )
    geom.merge(
        CylinderGeometry(radius=0.006, height=0.012, radial_segments=18).rotate_x(math.pi / 2.0)
    )

    radial_span = 0.292
    spoke_length = math.sqrt(radial_span * radial_span + 0.026 * 0.026)
    spoke_tilt = math.atan2(0.026, radial_span)
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        for side, phase in ((0.026, 0.0), (-0.026, math.pi / 12.0)):
            spoke = CylinderGeometry(radius=0.0019, height=spoke_length, radial_segments=10)
            spoke.rotate_x(spoke_tilt if side > 0.0 else -spoke_tilt)
            spoke.translate(0.0, side * 0.5, 0.171)
            spoke.rotate_y(angle + phase)
            geom.merge(spoke)
    return _save_mesh(geom, name)


def _make_handlebar_mesh(name: str, width: float, drop_back: float, rise: float, radius: float):
    half = width * 0.5
    points = [
        (0.02, -half, 0.0),
        (-0.01, -half * 0.78, rise * 0.45),
        (-drop_back * 0.7, -half * 0.38, rise),
        (-drop_back, 0.0, rise * 1.05),
        (-drop_back * 0.7, half * 0.38, rise),
        (-0.01, half * 0.78, rise * 0.45),
        (0.02, half, 0.0),
    ]
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return _save_mesh(geom, name)


def _make_saddle_mesh(name: str):
    profile = [
        (-0.115, -0.050),
        (-0.075, -0.062),
        (-0.010, -0.060),
        (0.055, -0.040),
        (0.115, -0.018),
        (0.130, 0.000),
        (0.115, 0.018),
        (0.055, 0.040),
        (-0.010, 0.060),
        (-0.075, 0.062),
        (-0.115, 0.050),
    ]
    geom = ExtrudeGeometry.centered(profile, 0.028)
    return _save_mesh(geom, name)


STEER_AXIS = _unit(_sub(HEAD_TOP, HEAD_BOTTOM))
HEADSET_TO_FORK_AXLE = _sub(FRONT_AXLE, HEAD_TOP)

CAPTAIN_LEFT_PEDAL_ORIGIN = (CRANK_ARM, PEDAL_OUTBOARD, 0.0)
CAPTAIN_RIGHT_PEDAL_ORIGIN = (-CRANK_ARM, -PEDAL_OUTBOARD, 0.0)
STOKER_LEFT_PEDAL_ORIGIN = (CRANK_ARM, PEDAL_OUTBOARD, 0.0)
STOKER_RIGHT_PEDAL_ORIGIN = (-CRANK_ARM, -PEDAL_OUTBOARD, 0.0)


def _build_pedal(part, side_sign: float) -> None:
    spindle_center_y = 0.020 * side_sign
    body_center_y = 0.044 * side_sign
    part.visual(
        Cylinder(radius=0.005, length=0.040),
        origin=Origin(xyz=(0.0, spindle_center_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    part.visual(Box((0.095, 0.034, 0.014)), origin=Origin(xyz=(0.015, body_center_y, 0.0)))
    part.visual(Box((0.045, 0.010, 0.026)), origin=Origin(xyz=(-0.022, body_center_y, 0.0)))


def _build_crankset(part, front_chainring: bool, rear_chainring: bool) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    part.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    part.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    _add_tube(part, (0.0, 0.035, 0.0), CAPTAIN_LEFT_PEDAL_ORIGIN, 0.010)
    _add_tube(part, (0.0, -0.035, 0.0), CAPTAIN_RIGHT_PEDAL_ORIGIN, 0.010)
    if front_chainring:
        part.visual(
            Cylinder(radius=0.098, length=0.004),
            origin=Origin(xyz=(0.0, 0.043, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        )
    if rear_chainring:
        part.visual(
            Cylinder(radius=0.108, length=0.004),
            origin=Origin(xyz=(0.0, -0.043, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        )


def build_object_model() -> ArticulatedObject:
    MESH_DIR.mkdir(parents=True, exist_ok=True)

    front_wheel_mesh = _make_wheel_mesh("front_wheel.obj")
    rear_wheel_mesh = _make_wheel_mesh("rear_wheel.obj")
    captain_bar_mesh = _make_handlebar_mesh(
        "captain_bar.obj", width=0.47, drop_back=0.065, rise=0.052, radius=0.009
    )
    stoker_bar_mesh = _make_handlebar_mesh(
        "stoker_bar.obj", width=0.39, drop_back=0.040, rise=0.036, radius=0.008
    )
    saddle_mesh = _make_saddle_mesh("saddle.obj")

    model = ArticulatedObject(name="tandem_bicycle", assets=ASSETS)

    main_frame = model.part("main_frame")
    _add_tube(main_frame, HEAD_BOTTOM, HEAD_TOP, 0.026)
    _add_tube(main_frame, HEAD_TOP, CAPTAIN_SEAT_CLUSTER, 0.022)
    _add_tube(main_frame, CAPTAIN_SEAT_CLUSTER, STOKER_SEAT_CLUSTER, 0.022)
    _add_tube(main_frame, HEAD_BOTTOM, CAPTAIN_BB, 0.024)
    _add_tube(main_frame, CAPTAIN_BB, STOKER_BB, 0.028)
    _add_tube(main_frame, CAPTAIN_BB, CAPTAIN_SEAT_CLUSTER, 0.026)
    _add_tube(main_frame, STOKER_BB, STOKER_SEAT_CLUSTER, 0.028)
    _add_tube(main_frame, CAPTAIN_SEAT_CLUSTER, STOKER_BB, 0.017)
    _add_tube(
        main_frame, (STOKER_BB[0], 0.018, STOKER_BB[2]), (REAR_AXLE[0], 0.010, REAR_AXLE[2]), 0.012
    )
    _add_tube(
        main_frame,
        (STOKER_BB[0], -0.018, STOKER_BB[2]),
        (REAR_AXLE[0], -0.010, REAR_AXLE[2]),
        0.012,
    )
    _add_tube(main_frame, STOKER_SEAT_CLUSTER, (REAR_AXLE[0], 0.035, REAR_AXLE[2] + 0.015), 0.010)
    _add_tube(main_frame, STOKER_SEAT_CLUSTER, (REAR_AXLE[0], -0.035, REAR_AXLE[2] + 0.015), 0.010)
    _add_tube(main_frame, (0.02, 0.0, 0.79), (-0.17, 0.0, 0.74), 0.015)
    _add_tube(main_frame, (CAPTAIN_SADDLE[0], 0.0, CAPTAIN_SADDLE[2] - 0.10), CAPTAIN_SADDLE, 0.012)
    _add_tube(main_frame, (STOKER_SADDLE[0], 0.0, STOKER_SADDLE[2] - 0.10), STOKER_SADDLE, 0.013)
    main_frame.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(CAPTAIN_BB[0], 0.018, CAPTAIN_BB[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    main_frame.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(CAPTAIN_BB[0], -0.018, CAPTAIN_BB[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    main_frame.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(STOKER_BB[0], 0.018, STOKER_BB[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    main_frame.visual(
        Cylinder(radius=0.026, length=0.022),
        origin=Origin(xyz=(STOKER_BB[0], -0.018, STOKER_BB[2]), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    main_frame.visual(
        Box((0.020, 0.008, 0.055)), origin=Origin(xyz=(REAR_AXLE[0], 0.010, REAR_AXLE[2]))
    )
    main_frame.visual(
        Box((0.020, 0.008, 0.055)), origin=Origin(xyz=(REAR_AXLE[0], -0.010, REAR_AXLE[2]))
    )
    main_frame.visual(saddle_mesh, origin=Origin(xyz=CAPTAIN_SADDLE, rpy=(-0.08, 0.0, 0.0)))
    main_frame.visual(saddle_mesh, origin=Origin(xyz=STOKER_SADDLE, rpy=(-0.06, 0.0, 0.0)))
    main_frame.visual(stoker_bar_mesh, origin=Origin(xyz=(-0.28, 0.0, 0.73), rpy=(0.0, 0.0, 0.08)))
    main_frame.inertial = Inertial.from_geometry(
        Box((1.72, 0.22, 0.88)),
        mass=18.0,
        origin=Origin(xyz=(0.08, 0.0, 0.54)),
    )

    front_fork = model.part("front_fork")
    stem_center = _scale(STEER_AXIS, 0.018)
    front_fork.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=stem_center, rpy=(0.0, math.atan2(STEER_AXIS[0], STEER_AXIS[2]), 0.0)),
    )
    front_fork.visual(
        captain_bar_mesh,
        origin=Origin(xyz=(-0.065, 0.0, 0.115), rpy=(0.0, 0.0, 0.0)),
    )
    _add_tube(front_fork, (0.000, 0.0, 0.032), (-0.118, 0.0, 0.162), 0.012)
    front_fork.visual(Box((0.030, 0.044, 0.028)), origin=Origin(xyz=(-0.118, 0.0, 0.162)))
    fork_spine = tube_from_spline_points(
        [
            (0.015, 0.0, 0.012),
            (0.055, 0.0, -0.070),
            (0.130, 0.0, -0.190),
            (0.185, 0.0, -0.285),
        ],
        radius=0.015,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )
    front_fork.visual(_save_mesh(fork_spine, "fork_spine.obj"), origin=Origin(xyz=(0.0, 0.0, 0.0)))
    front_fork.visual(Box((0.090, 0.052, 0.030)), origin=Origin(xyz=(0.185, 0.0, -0.285)))
    _add_tube(
        front_fork,
        (0.188, 0.035, -0.278),
        (HEADSET_TO_FORK_AXLE[0], 0.010, HEADSET_TO_FORK_AXLE[2]),
        0.012,
    )
    _add_tube(
        front_fork,
        (0.188, -0.035, -0.278),
        (HEADSET_TO_FORK_AXLE[0], -0.010, HEADSET_TO_FORK_AXLE[2]),
        0.012,
    )
    front_fork.visual(
        Cylinder(radius=0.006, length=0.078),
        origin=Origin(xyz=(0.245, 0.0, -0.405), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    front_fork.visual(
        Box((0.020, 0.008, 0.052)),
        origin=Origin(xyz=(HEADSET_TO_FORK_AXLE[0], 0.010, HEADSET_TO_FORK_AXLE[2])),
    )
    front_fork.visual(
        Box((0.020, 0.008, 0.052)),
        origin=Origin(xyz=(HEADSET_TO_FORK_AXLE[0], -0.010, HEADSET_TO_FORK_AXLE[2])),
    )
    front_fork.inertial = Inertial.from_geometry(
        Box((0.42, 0.16, 0.78)),
        mass=3.4,
        origin=Origin(xyz=(0.14, 0.0, -0.24)),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(front_wheel_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0)))
    front_wheel.inertial = Inertial.from_geometry(
        Box((0.69, 0.07, 0.69)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(rear_wheel_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0)))
    rear_wheel.visual(
        Cylinder(radius=0.040, length=0.006),
        origin=Origin(xyz=(0.0, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    rear_wheel.visual(
        Cylinder(radius=0.034, length=0.005),
        origin=Origin(xyz=(0.0, -0.051, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    rear_wheel.visual(
        Cylinder(radius=0.028, length=0.005),
        origin=Origin(xyz=(0.0, -0.057, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Box((0.69, 0.09, 0.69)),
        mass=2.5,
        origin=Origin(xyz=(0.0, -0.01, 0.0)),
    )

    captain_crankset = model.part("captain_crankset")
    _build_crankset(captain_crankset, front_chainring=True, rear_chainring=False)
    captain_crankset.inertial = Inertial.from_geometry(
        Box((0.38, 0.20, 0.22)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    stoker_crankset = model.part("stoker_crankset")
    _build_crankset(stoker_crankset, front_chainring=True, rear_chainring=True)
    stoker_crankset.inertial = Inertial.from_geometry(
        Box((0.38, 0.24, 0.22)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    captain_left_pedal = model.part("captain_left_pedal")
    _build_pedal(captain_left_pedal, 1.0)
    captain_left_pedal.inertial = Inertial.from_geometry(
        Box((0.11, 0.07, 0.03)),
        mass=0.28,
        origin=Origin(xyz=(0.02, 0.05, 0.0)),
    )

    captain_right_pedal = model.part("captain_right_pedal")
    _build_pedal(captain_right_pedal, -1.0)
    captain_right_pedal.inertial = Inertial.from_geometry(
        Box((0.11, 0.07, 0.03)),
        mass=0.28,
        origin=Origin(xyz=(0.02, -0.05, 0.0)),
    )

    stoker_left_pedal = model.part("stoker_left_pedal")
    _build_pedal(stoker_left_pedal, 1.0)
    stoker_left_pedal.inertial = Inertial.from_geometry(
        Box((0.11, 0.07, 0.03)),
        mass=0.28,
        origin=Origin(xyz=(0.02, 0.05, 0.0)),
    )

    stoker_right_pedal = model.part("stoker_right_pedal")
    _build_pedal(stoker_right_pedal, -1.0)
    stoker_right_pedal.inertial = Inertial.from_geometry(
        Box((0.11, 0.07, 0.03)),
        mass=0.28,
        origin=Origin(xyz=(0.02, -0.05, 0.0)),
    )

    model.articulation(
        "steer_headset",
        ArticulationType.REVOLUTE,
        parent="main_frame",
        child="front_fork",
        origin=Origin(xyz=HEAD_TOP),
        axis=STEER_AXIS,
        motion_limits=MotionLimits(effort=20.0, velocity=2.5, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="front_fork",
        child="front_wheel",
        origin=Origin(xyz=HEADSET_TO_FORK_AXLE),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="main_frame",
        child="rear_wheel",
        origin=Origin(xyz=REAR_AXLE),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )
    model.articulation(
        "captain_crank_spin",
        ArticulationType.CONTINUOUS,
        parent="main_frame",
        child="captain_crankset",
        origin=Origin(xyz=CAPTAIN_BB),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=14.0),
    )
    model.articulation(
        "stoker_crank_spin",
        ArticulationType.CONTINUOUS,
        parent="main_frame",
        child="stoker_crankset",
        origin=Origin(xyz=STOKER_BB),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=14.0),
    )
    model.articulation(
        "captain_left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="captain_crankset",
        child="captain_left_pedal",
        origin=Origin(xyz=CAPTAIN_LEFT_PEDAL_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "captain_right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="captain_crankset",
        child="captain_right_pedal",
        origin=Origin(xyz=CAPTAIN_RIGHT_PEDAL_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "stoker_left_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="stoker_crankset",
        child="stoker_left_pedal",
        origin=Origin(xyz=STOKER_LEFT_PEDAL_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "stoker_right_pedal_spin",
        ArticulationType.CONTINUOUS,
        parent="stoker_crankset",
        child="stoker_right_pedal",
        origin=Origin(xyz=STOKER_RIGHT_PEDAL_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")

    ctx.allow_overlap(
        "main_frame",
        "front_fork",
        reason="headset bearing stack is modeled with a near-flush steering interface",
    )
    ctx.allow_overlap(
        "front_fork",
        "front_wheel",
        reason="fork blades live inside the wheel opening and wheel hull generation is conservative",
    )
    ctx.allow_overlap(
        "main_frame",
        "rear_wheel",
        reason="rear stays occupy the wheel opening around the hub and spoke hulls can overfill",
    )
    ctx.allow_overlap(
        "main_frame",
        "captain_crankset",
        reason="split bottom bracket shell runs with millimeter spindle clearance",
    )
    ctx.allow_overlap(
        "main_frame",
        "stoker_crankset",
        reason="split bottom bracket shell runs with millimeter spindle clearance",
    )
    ctx.allow_overlap(
        "rear_wheel",
        "stoker_crankset",
        reason="rear wheel collision hull fills the spoke envelope; the stoker crank orbit runs in front of the actual rim but can clip the conservative hull",
    )
    ctx.allow_overlap(
        "captain_crankset",
        "captain_left_pedal",
        reason="pedal spindle seats directly against the crank arm",
    )
    ctx.allow_overlap(
        "captain_crankset",
        "captain_right_pedal",
        reason="pedal spindle seats directly against the crank arm",
    )
    ctx.allow_overlap(
        "stoker_crankset",
        "stoker_left_pedal",
        reason="pedal spindle seats directly against the crank arm",
    )
    ctx.allow_overlap(
        "stoker_crankset",
        "stoker_right_pedal",
        reason="pedal spindle seats directly against the crank arm",
    )

    ctx.check_no_overlaps(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("front_wheel", "front_fork", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("rear_wheel", "main_frame", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("captain_crankset", "main_frame", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("stoker_crankset", "main_frame", axes="xy", min_overlap=0.02)

    rear_pos = ctx.part_world_position("rear_wheel")
    front_pos = ctx.part_world_position("front_wheel")
    captain_pos = ctx.part_world_position("captain_crankset")
    stoker_pos = ctx.part_world_position("stoker_crankset")
    assert abs(rear_pos[1]) < 1e-4 and abs(front_pos[1]) < 1e-4, (
        "wheels should sit on the bicycle center plane"
    )
    assert abs(rear_pos[2] - front_pos[2]) < 1e-4, "wheel centers should share the same ride height"
    assert front_pos[0] - rear_pos[0] > 1.28, "tandem wheelbase should be clearly elongated"
    assert rear_pos[0] < stoker_pos[0] < captain_pos[0] < front_pos[0], (
        "both rider stations should sit between the axles"
    )

    with ctx.pose(front_wheel_spin=1.25, rear_wheel_spin=2.50):
        spun_front = ctx.part_world_position("front_wheel")
        spun_rear = ctx.part_world_position("rear_wheel")
    assert abs(spun_front[0] - front_pos[0]) < 1e-5 and abs(spun_front[2] - front_pos[2]) < 1e-5, (
        "front wheel should spin in place"
    )
    assert abs(spun_rear[0] - rear_pos[0]) < 1e-5 and abs(spun_rear[2] - rear_pos[2]) < 1e-5, (
        "rear wheel should spin in place"
    )

    with ctx.pose(steer_headset=0.45):
        steer_left = ctx.part_world_position("front_wheel")
        ctx.expect_aabb_overlap("front_wheel", "front_fork", axes="xy", min_overlap=0.015)
    with ctx.pose(steer_headset=-0.45):
        steer_right = ctx.part_world_position("front_wheel")
        ctx.expect_aabb_overlap("front_wheel", "front_fork", axes="xy", min_overlap=0.015)
    assert steer_left[1] * steer_right[1] < 0.0, (
        "steering should sweep the front wheel to opposite sides"
    )
    assert abs(steer_left[1] - steer_right[1]) > 0.05, "steering sweep should be visibly meaningful"

    captain_left_rest = ctx.part_world_position("captain_left_pedal")
    with ctx.pose(captain_crank_spin=math.pi / 2.0):
        captain_left_quarter = ctx.part_world_position("captain_left_pedal")
        captain_right_quarter = ctx.part_world_position("captain_right_pedal")
    assert abs(captain_left_quarter[2] - captain_left_rest[2]) > 0.12, (
        "captain crank rotation should move the pedal through a large arc"
    )
    assert abs(captain_left_quarter[2] - captain_right_quarter[2]) > 0.28, (
        "opposed captain pedals should separate vertically at quarter turn"
    )

    with ctx.pose(captain_crank_spin=math.pi / 2.0, stoker_crank_spin=math.pi / 2.0):
        synced_captain = ctx.part_world_position("captain_left_pedal")
        synced_stoker = ctx.part_world_position("stoker_left_pedal")
    assert abs(synced_captain[2] - synced_stoker[2]) < 1e-5, (
        "both tandem cranksets should present the same pedaling phase when posed together"
    )

    captain_left_default = ctx.part_world_position("captain_left_pedal")
    with ctx.pose(captain_left_pedal_spin=1.1):
        captain_left_spun = ctx.part_world_position("captain_left_pedal")
    assert abs(captain_left_default[0] - captain_left_spun[0]) < 1e-6
    assert abs(captain_left_default[1] - captain_left_spun[1]) < 1e-6
    assert abs(captain_left_default[2] - captain_left_spun[2]) < 1e-6, (
        "pedal spin should occur about the spindle without translating the pedal"
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
