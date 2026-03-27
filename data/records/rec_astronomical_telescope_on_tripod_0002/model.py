from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

LEG_SPLAY = math.radians(31.0)
LEG_SOCKET_RADIUS = 0.052
LEG_SOCKET_Z = -0.082
UPPER_LEG_LENGTH = 0.38
LOWER_LEG_LENGTH = 0.30
LEG_INSERT_DEPTH = 0.12
LEG_EXTENSION_MAX = 0.08
LOWER_STOP_LENGTH = 0.015
LOWER_STOP_CENTER = 0.020
UPPER_LIMIT_LENGTH = 0.015
UPPER_LOWER_JOINT_Z = UPPER_LEG_LENGTH - LEG_INSERT_DEPTH
UPPER_LIMIT_CENTER = (
    UPPER_LOWER_JOINT_Z
    + LEG_EXTENSION_MAX
    + (LOWER_STOP_CENTER - LOWER_STOP_LENGTH / 2.0)
    - UPPER_LIMIT_LENGTH / 2.0
)
LEG_AZIMUTHS = (
    0.0,
    2.0 * math.pi / 3.0,
    4.0 * math.pi / 3.0,
)

TUBE_LENGTH = 0.60
TUBE_OUTER_RADIUS = 0.095
FORK_ARM_Y = 0.120
FORK_PIVOT_X = -0.020
FORK_PIVOT_Z = 0.380


def _leg_direction(phi: float) -> tuple[float, float, float]:
    return (
        math.sin(LEG_SPLAY) * math.cos(phi),
        math.sin(LEG_SPLAY) * math.sin(phi),
        -math.cos(LEG_SPLAY),
    )


def _offset(point: tuple[float, float, float], direction: tuple[float, float, float], dist: float) -> tuple[float, float, float]:
    return (
        point[0] + direction[0] * dist,
        point[1] + direction[1] * dist,
        point[2] + direction[2] * dist,
    )


def _leg_socket_point(phi: float) -> tuple[float, float, float]:
    return (
        LEG_SOCKET_RADIUS * math.cos(phi),
        LEG_SOCKET_RADIUS * math.sin(phi),
        LEG_SOCKET_Z,
    )


def _leg_mount_origin(phi: float) -> Origin:
    direction = _leg_direction(phi)
    mount_point = _offset(_leg_socket_point(phi), direction, 0.020)
    return Origin(xyz=mount_point, rpy=(0.0, math.pi - LEG_SPLAY, phi))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="newtonian_reflector_mount")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    tube_white = model.material("tube_white", rgba=(0.88, 0.89, 0.90, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.59, 0.61, 0.64, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    head = model.part("head")
    head.visual(Cylinder(radius=0.074, length=0.024), origin=Origin(xyz=(0.0, 0.0, -0.012)), material=graphite, name="top_plate")
    head.visual(Cylinder(radius=0.055, length=0.080), origin=Origin(xyz=(0.0, 0.0, -0.056)), material=graphite, name="center_column")
    head.visual(Sphere(radius=0.078), origin=Origin(xyz=(0.0, 0.0, -0.100)), material=graphite, name="casting_bulb")
    for index, phi in enumerate(LEG_AZIMUTHS, start=1):
        direction = _leg_direction(phi)
        socket_center = _offset(_leg_socket_point(phi), direction, 0.010)
        head.visual(
            Cylinder(radius=0.022, length=0.020),
            origin=Origin(xyz=socket_center, rpy=(0.0, math.pi - LEG_SPLAY, phi)),
            material=graphite,
            name=f"socket_{index}",
        )

    fork = model.part("fork")
    fork.visual(Cylinder(radius=0.100, length=0.024), origin=Origin(xyz=(0.0, 0.0, 0.012)), material=graphite, name="azimuth_base")
    fork.visual(Box((0.040, 0.024, 0.430)), origin=Origin(xyz=(-0.104, 0.124, 0.239)), material=graphite, name="left_arm")
    fork.visual(Box((0.040, 0.024, 0.430)), origin=Origin(xyz=(-0.104, -0.124, 0.239)), material=graphite, name="right_arm")
    fork.visual(Box((0.032, 0.052, 0.210)), origin=Origin(xyz=(-0.104, 0.0, 0.129)), material=graphite, name="center_spine")
    fork.visual(Box((0.072, 0.020, 0.070)), origin=Origin(xyz=(-0.076, 0.122, FORK_PIVOT_Z)), material=graphite, name="left_cheek")
    fork.visual(Box((0.072, 0.020, 0.070)), origin=Origin(xyz=(-0.076, -0.122, FORK_PIVOT_Z)), material=graphite, name="right_cheek")
    fork.visual(Box((0.028, 0.020, 0.070)), origin=Origin(xyz=(-0.034, 0.120, FORK_PIVOT_Z)), material=brushed_metal, name="left_bearing")
    fork.visual(Box((0.028, 0.020, 0.070)), origin=Origin(xyz=(-0.034, -0.120, FORK_PIVOT_Z)), material=brushed_metal, name="right_bearing")
    fork.visual(Box((0.030, 0.250, 0.030)), origin=Origin(xyz=(-0.104, 0.0, 0.085)), material=graphite, name="lower_brace")

    tube = model.part("tube")
    tube.visual(Cylinder(radius=0.093, length=0.540), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=tube_white, name="tube_shell")
    tube.visual(Cylinder(radius=0.098, length=0.018), origin=Origin(xyz=(0.279, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=tube_white, name="front_rim")
    tube.visual(Cylinder(radius=0.091, length=0.040), origin=Origin(xyz=(-0.270, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=satin_black, name="mirror_cell")
    tube.visual(Cylinder(radius=0.024, length=0.028), origin=Origin(xyz=(FORK_PIVOT_X, 0.096, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=graphite, name="left_trunnion")
    tube.visual(Cylinder(radius=0.024, length=0.028), origin=Origin(xyz=(FORK_PIVOT_X, -0.096, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=graphite, name="right_trunnion")

    focuser = model.part("focuser")
    focuser.visual(Box((0.080, 0.050, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.005)), material=graphite, name="base_plate")
    focuser.visual(Box((0.060, 0.045, 0.024)), origin=Origin(xyz=(0.0, 0.0, 0.022)), material=graphite, name="housing")
    focuser.visual(Cylinder(radius=0.010, length=0.016), origin=Origin(xyz=(0.0, 0.030, 0.017), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=graphite, name="knob_socket")

    knob = model.part("focuser_knob")
    knob.visual(Cylinder(radius=0.012, length=0.024), origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=brushed_metal, name="knob_body")

    for index, phi in enumerate(LEG_AZIMUTHS, start=1):
        upper = model.part(f"leg_{index}_upper")
        upper.visual(Cylinder(radius=0.021, length=0.030), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=graphite, name="upper_collar")
        upper.visual(Cylinder(radius=0.016, length=0.350), origin=Origin(xyz=(0.0, 0.0, 0.205)), material=graphite, name="upper_tube")
        upper.visual(Cylinder(radius=0.018, length=UPPER_LIMIT_LENGTH), origin=Origin(xyz=(0.0, 0.0, UPPER_LIMIT_CENTER)), material=brushed_metal, name="extension_limit")

        lower = model.part(f"leg_{index}_lower")
        lower.visual(Cylinder(radius=0.011, length=LOWER_LEG_LENGTH), origin=Origin(xyz=(0.0, 0.0, LOWER_LEG_LENGTH / 2.0)), material=brushed_metal, name="lower_tube")
        lower.visual(Cylinder(radius=0.015, length=LOWER_STOP_LENGTH), origin=Origin(xyz=(0.0, 0.0, LOWER_STOP_CENTER)), material=graphite, name="extension_stop")
        lower.visual(Cylinder(radius=0.016, length=0.030), origin=Origin(xyz=(0.0, 0.0, LOWER_LEG_LENGTH - 0.015)), material=rubber, name="foot")

        model.articulation(
            f"head_to_leg_{index}_upper",
            ArticulationType.FIXED,
            parent=head,
            child=upper,
            origin=_leg_mount_origin(phi),
        )
        model.articulation(
            f"leg_{index}_upper_to_lower",
            ArticulationType.PRISMATIC,
            parent=upper,
            child=lower,
            origin=Origin(xyz=(0.0, 0.0, UPPER_LOWER_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=25.0,
                velocity=0.12,
                lower=0.0,
                upper=LEG_EXTENSION_MAX,
            ),
        )

    model.articulation(
        "head_to_fork",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=fork,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8),
    )
    model.articulation(
        "fork_to_tube",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=tube,
        origin=Origin(xyz=(FORK_PIVOT_X, 0.0, FORK_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.8,
            lower=-0.25,
            upper=1.00,
        ),
    )
    model.articulation(
        "tube_to_focuser",
        ArticulationType.FIXED,
        parent=tube,
        child=focuser,
        origin=Origin(xyz=(0.140, 0.0, TUBE_OUTER_RADIUS)),
    )
    model.articulation(
        "focuser_to_knob",
        ArticulationType.FIXED,
        parent=focuser,
        child=knob,
        origin=Origin(xyz=(0.0, 0.038, 0.017)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head = object_model.get_part("head")
    fork = object_model.get_part("fork")
    tube = object_model.get_part("tube")
    focuser = object_model.get_part("focuser")
    knob = object_model.get_part("focuser_knob")
    head_to_fork = object_model.get_articulation("head_to_fork")
    fork_to_tube = object_model.get_articulation("fork_to_tube")

    top_plate = head.get_visual("top_plate")
    fork_base = fork.get_visual("azimuth_base")
    left_bearing = fork.get_visual("left_bearing")
    right_bearing = fork.get_visual("right_bearing")
    lower_brace = fork.get_visual("lower_brace")
    tube_shell = tube.get_visual("tube_shell")
    left_trunnion = tube.get_visual("left_trunnion")
    right_trunnion = tube.get_visual("right_trunnion")
    focuser_base = focuser.get_visual("base_plate")
    focuser_housing = focuser.get_visual("housing")
    knob_socket = focuser.get_visual("knob_socket")
    knob_body = knob.get_visual("knob_body")

    upper_legs = []
    lower_legs = []
    upper_collars = []
    head_sockets = []
    extension_limits = []
    extension_stops = []
    lower_slides = []
    feet = []
    for index in range(1, 4):
        upper = object_model.get_part(f"leg_{index}_upper")
        lower = object_model.get_part(f"leg_{index}_lower")
        upper_legs.append(upper)
        lower_legs.append(lower)
        upper_collars.append(upper.get_visual("upper_collar"))
        extension_limits.append(upper.get_visual("extension_limit"))
        extension_stops.append(lower.get_visual("extension_stop"))
        head_sockets.append(head.get_visual(f"socket_{index}"))
        feet.append(lower.get_visual("foot"))
        lower_slides.append(object_model.get_articulation(f"leg_{index}_upper_to_lower"))
        ctx.allow_overlap(lower, upper, reason="telescoping lower leg section nests inside the upper tripod tube")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.12)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(fork, head, elem_a=fork_base, elem_b=top_plate)
    ctx.expect_overlap(fork, head, axes="xy", elem_a=fork_base, elem_b=top_plate, min_overlap=0.010)

    for upper, collar, socket in zip(upper_legs, upper_collars, head_sockets):
        ctx.expect_contact(upper, head, elem_a=collar, elem_b=socket)
    for lower, foot in zip(lower_legs, feet):
        ctx.expect_gap(
            head,
            lower,
            axis="z",
            positive_elem=top_plate,
            negative_elem=foot,
            min_gap=0.50,
        )

    ctx.expect_contact(fork, tube, elem_a=left_bearing, elem_b=left_trunnion)
    ctx.expect_contact(fork, tube, elem_a=right_bearing, elem_b=right_trunnion)
    ctx.expect_overlap(fork, tube, axes="xz", elem_a=left_bearing, elem_b=left_trunnion, min_overlap=0.010)
    ctx.expect_overlap(fork, tube, axes="xz", elem_a=right_bearing, elem_b=right_trunnion, min_overlap=0.010)
    ctx.expect_gap(
        tube,
        fork,
        axis="z",
        positive_elem=tube_shell,
        negative_elem=lower_brace,
        min_gap=0.070,
    )

    ctx.expect_overlap(focuser, tube, axes="xy", elem_a=focuser_base, elem_b=tube_shell, min_overlap=0.010)
    ctx.expect_gap(
        focuser,
        tube,
        axis="z",
        positive_elem=focuser_base,
        negative_elem=tube_shell,
        max_gap=0.003,
        max_penetration=0.004,
    )
    ctx.expect_overlap(knob, focuser, axes="xz", elem_a=knob_body, elem_b=focuser_housing, min_overlap=0.010)
    ctx.expect_contact(knob, focuser, elem_a=knob_body, elem_b=knob_socket)
    ctx.expect_gap(
        knob,
        tube,
        axis="z",
        positive_elem=knob_body,
        negative_elem=tube_shell,
        min_gap=0.001,
    )

    with ctx.pose({fork_to_tube: 0.85, head_to_fork: 0.65}):
        ctx.expect_contact(fork, tube, elem_a=left_bearing, elem_b=left_trunnion)
        ctx.expect_contact(fork, tube, elem_a=right_bearing, elem_b=right_trunnion)
        ctx.expect_gap(tube, head, axis="z", positive_elem=tube_shell, negative_elem=top_plate, min_gap=0.020)
        ctx.expect_gap(tube, fork, axis="z", positive_elem=tube_shell, negative_elem=fork_base, min_gap=0.020)
        ctx.expect_gap(tube, fork, axis="z", positive_elem=tube_shell, negative_elem=lower_brace, min_gap=0.012)

    with ctx.pose({slide: LEG_EXTENSION_MAX for slide in lower_slides}):
        for lower, upper, stop, limit in zip(lower_legs, upper_legs, extension_stops, extension_limits):
            ctx.expect_contact(lower, upper, elem_a=stop, elem_b=limit)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
