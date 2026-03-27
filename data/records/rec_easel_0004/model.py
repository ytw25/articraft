from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd

try:
    _REAL_GETCWD()
except FileNotFoundError:
    os.chdir("/")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

LEG_RADIUS = 0.008
LEG_COLLAR_RADIUS = 0.012
LEG_COLLAR_LENGTH = 0.026
LEG_LENGTH = 1.34
FRONT_LEG_TILT = math.radians(17.0)
REAR_LEG_TILT = math.radians(24.0)
LEFT_FRONT_YAW = math.radians(38.0)
RIGHT_FRONT_YAW = math.radians(-38.0)
REAR_YAW = math.pi
RING_RADIUS = 0.028
LEG_ATTACH_RADIUS = 0.022
LEDGE_DISTANCE = 0.74
LEDGE_WIDTH = 0.34


def _tilted_down_point(distance: float, tilt: float) -> tuple[float, float, float]:
    return (0.0, distance * math.sin(tilt), -distance * math.cos(tilt))


def _yawed_point(distance: float, tilt: float, yaw: float) -> tuple[float, float, float]:
    _, y_local, z_local = _tilted_down_point(distance, tilt)
    return (-math.sin(yaw) * y_local, math.cos(yaw) * y_local, z_local)


def _rotate_local(point: tuple[float, float, float], tilt: float, yaw: float) -> tuple[float, float, float]:
    x, y, z = point
    y_roll = (y * math.cos(tilt)) - (z * math.sin(tilt))
    z_roll = (y * math.sin(tilt)) + (z * math.cos(tilt))
    return (
        (x * math.cos(yaw)) - (y_roll * math.sin(yaw)),
        (x * math.sin(yaw)) + (y_roll * math.cos(yaw)),
        z_roll,
    )


def _offset_point(
    base: tuple[float, float, float],
    local_offset: tuple[float, float, float],
    tilt: float,
    yaw: float,
) -> tuple[float, float, float]:
    dx, dy, dz = _rotate_local(local_offset, tilt, yaw)
    return (base[0] + dx, base[1] + dy, base[2] + dz)


def _segment_pose(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[tuple[float, float, float], float, tuple[float, float, float]]:
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    vz = end[2] - start[2]
    length = math.sqrt((vx * vx) + (vy * vy) + (vz * vz))
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.sqrt((vx * vx) + (vy * vy)), vz)
    return (
        ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (start[2] + end[2]) / 2.0),
        length,
        (0.0, pitch, yaw),
    )


def _angled_origin(distance: float, tilt: float, yaw: float) -> Origin:
    return Origin(xyz=_yawed_point(distance, tilt, yaw), rpy=(tilt, 0.0, yaw))


def _build_leg(part, *, tilt: float, aluminum, rubber, foot_name: str):
    tube_center = LEG_COLLAR_LENGTH + (LEG_LENGTH / 2.0)
    part.visual(
        Cylinder(radius=LEG_RADIUS, length=LEG_LENGTH),
        origin=Origin(
            xyz=_tilted_down_point(tube_center, tilt),
            rpy=(tilt, 0.0, 0.0),
        ),
        material=aluminum,
        name="leg_tube",
    )
    part.visual(
        Cylinder(radius=LEG_COLLAR_RADIUS, length=LEG_COLLAR_LENGTH),
        origin=Origin(
            xyz=_tilted_down_point(LEG_COLLAR_LENGTH / 2.0, tilt),
            rpy=(tilt, 0.0, 0.0),
        ),
        material=aluminum,
        name="top_collar",
    )
    part.visual(
        Cylinder(radius=LEG_RADIUS * 1.15, length=0.028),
        origin=Origin(
            xyz=_tilted_down_point(LEG_COLLAR_LENGTH + LEG_LENGTH - 0.014, tilt),
            rpy=(tilt, 0.0, 0.0),
        ),
        material=rubber,
        name=foot_name,
    )
    part.inertial = Inertial.from_geometry(
        Cylinder(radius=LEG_RADIUS, length=LEG_LENGTH),
        mass=0.55,
        origin=Origin(
            xyz=_tilted_down_point(tube_center, tilt),
            rpy=(tilt, 0.0, 0.0),
        ),
    )


def build_object_model() -> ArticulatedObject:
    os.chdir("/")
    model = ArticulatedObject(name="portable_sketch_easel")

    aluminum = model.material("aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.12, 0.12, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    top_ring = model.part("top_ring")
    for index, angle in enumerate([i * (math.pi / 6.0) for i in range(12)]):
        top_ring.visual(
            Box((0.024, 0.010, 0.010)),
            origin=Origin(
                xyz=(RING_RADIUS * math.cos(angle), RING_RADIUS * math.sin(angle), 0.0),
                rpy=(0.0, 0.0, angle + (math.pi / 2.0)),
            ),
            material=aluminum,
            name=f"pivot_ring_segment_{index}",
        )
    for index, angle in enumerate((LEFT_FRONT_YAW, RIGHT_FRONT_YAW, REAR_YAW)):
        top_ring.visual(
            Box((0.018, 0.010, 0.010)),
            origin=Origin(
                xyz=(0.019 * math.cos(angle), 0.019 * math.sin(angle), 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=aluminum,
            name=f"pivot_bridge_{index}",
        )
    top_ring.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(),
        material=aluminum,
        name="pivot_hub",
    )
    top_ring.visual(
        Box((0.034, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.016, -0.005)),
        material=dark_plastic,
        name="top_cap",
    )

    left_front_leg = model.part("left_front_leg")
    _build_leg(left_front_leg, tilt=FRONT_LEG_TILT, aluminum=aluminum, rubber=rubber, foot_name="left_foot")

    right_front_leg = model.part("right_front_leg")
    _build_leg(right_front_leg, tilt=FRONT_LEG_TILT, aluminum=aluminum, rubber=rubber, foot_name="right_foot")

    rear_brace_leg = model.part("rear_brace_leg")
    _build_leg(rear_brace_leg, tilt=REAR_LEG_TILT, aluminum=aluminum, rubber=rubber, foot_name="rear_foot")

    ledge = model.part("canvas_ledge")

    left_slider_world = _yawed_point(LEDGE_DISTANCE, FRONT_LEG_TILT, LEFT_FRONT_YAW)
    right_slider_world = _yawed_point(LEDGE_DISTANCE, FRONT_LEG_TILT, RIGHT_FRONT_YAW)
    bar_z = left_slider_world[2] - 0.095
    bar_y = left_slider_world[1] + 0.085
    bar_center_world = (0.0, bar_y, bar_z)
    left_slider_center = (
        left_slider_world[0] - bar_center_world[0],
        left_slider_world[1] - bar_center_world[1],
        left_slider_world[2] - bar_center_world[2],
    )
    right_slider_center = (
        right_slider_world[0] - bar_center_world[0],
        right_slider_world[1] - bar_center_world[1],
        right_slider_world[2] - bar_center_world[2],
    )
    left_bar_anchor = (-(LEDGE_WIDTH / 2.0) + 0.042, 0.001, 0.004)
    right_bar_anchor = ((LEDGE_WIDTH / 2.0) - 0.042, 0.001, 0.004)
    left_arm_center, left_arm_length, left_arm_rpy = _segment_pose(left_bar_anchor, left_slider_center)
    right_arm_center, right_arm_length, right_arm_rpy = _segment_pose(right_bar_anchor, right_slider_center)

    ledge.visual(
        Cylinder(radius=0.0135, length=0.060),
        origin=Origin(xyz=left_slider_center, rpy=(FRONT_LEG_TILT, 0.0, LEFT_FRONT_YAW)),
        material=aluminum,
        name="left_slider_sleeve",
    )
    ledge.visual(
        Cylinder(radius=0.0135, length=0.060),
        origin=Origin(xyz=right_slider_center, rpy=(FRONT_LEG_TILT, 0.0, RIGHT_FRONT_YAW)),
        material=aluminum,
        name="right_slider_sleeve",
    )
    ledge.visual(
        Box((0.024, 0.016, 0.016)),
        origin=Origin(xyz=(left_slider_center[0], left_slider_center[1], left_slider_center[2] - 0.024)),
        material=aluminum,
        name="left_clamp_pad",
    )
    ledge.visual(
        Box((0.024, 0.016, 0.016)),
        origin=Origin(xyz=(right_slider_center[0], right_slider_center[1], right_slider_center[2] - 0.024)),
        material=aluminum,
        name="right_clamp_pad",
    )

    ledge.visual(
        Box((0.012, 0.012, left_arm_length)),
        origin=Origin(xyz=left_arm_center, rpy=left_arm_rpy),
        material=aluminum,
        name="left_bracket_arm",
    )
    ledge.visual(
        Box((0.012, 0.012, right_arm_length)),
        origin=Origin(xyz=right_arm_center, rpy=right_arm_rpy),
        material=aluminum,
        name="right_bracket_arm",
    )
    ledge.visual(
        Box((LEDGE_WIDTH, 0.016, 0.012)),
        origin=Origin(),
        material=aluminum,
        name="canvas_ledge_bar",
    )
    ledge.visual(
        Box((LEDGE_WIDTH - 0.02, 0.009, 0.024)),
        origin=Origin(xyz=(0.0, 0.010, 0.006)),
        material=aluminum,
        name="canvas_lip",
    )
    ledge.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(
            xyz=left_slider_center,
            rpy=(0.0, math.pi / 2.0, LEFT_FRONT_YAW),
        ),
        material=dark_plastic,
        name="left_thumbscrew",
    )
    ledge.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(
            xyz=right_slider_center,
            rpy=(0.0, math.pi / 2.0, RIGHT_FRONT_YAW),
        ),
        material=dark_plastic,
        name="right_thumbscrew",
    )
    ledge.visual(
        Box((0.070, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.003, 0.010)),
        material=rubber,
        name="canvas_pad",
    )
    ledge.inertial = Inertial.from_geometry(
        Box((LEDGE_WIDTH, 0.016, 0.012)),
        mass=0.32,
        origin=Origin(),
    )

    model.articulation(
        "left_front_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=top_ring,
        child=left_front_leg,
        origin=Origin(
            xyz=(-math.sin(LEFT_FRONT_YAW) * LEG_ATTACH_RADIUS, math.cos(LEFT_FRONT_YAW) * LEG_ATTACH_RADIUS, 0.0),
            rpy=(0.0, 0.0, LEFT_FRONT_YAW),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-FRONT_LEG_TILT + math.radians(1.2),
            upper=math.radians(6.0),
        ),
    )
    model.articulation(
        "right_front_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=top_ring,
        child=right_front_leg,
        origin=Origin(
            xyz=(-math.sin(RIGHT_FRONT_YAW) * LEG_ATTACH_RADIUS, math.cos(RIGHT_FRONT_YAW) * LEG_ATTACH_RADIUS, 0.0),
            rpy=(0.0, 0.0, RIGHT_FRONT_YAW),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-FRONT_LEG_TILT + math.radians(1.2),
            upper=math.radians(6.0),
        ),
    )
    model.articulation(
        "rear_brace_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=top_ring,
        child=rear_brace_leg,
        origin=Origin(
            xyz=(-math.sin(REAR_YAW) * LEG_ATTACH_RADIUS, math.cos(REAR_YAW) * LEG_ATTACH_RADIUS, 0.0),
            rpy=(0.0, 0.0, REAR_YAW),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-REAR_LEG_TILT + math.radians(1.0),
            upper=math.radians(5.0),
        ),
    )
    model.articulation(
        "ledge_mount",
        ArticulationType.FIXED,
        parent=top_ring,
        child=ledge,
        origin=Origin(xyz=bar_center_world),
    )

    return model


def run_tests() -> TestReport:
    os.chdir("/")
    ctx = TestContext(object_model)
    top_ring = object_model.get_part("top_ring")
    left_front_leg = object_model.get_part("left_front_leg")
    right_front_leg = object_model.get_part("right_front_leg")
    rear_brace_leg = object_model.get_part("rear_brace_leg")
    ledge = object_model.get_part("canvas_ledge")

    left_front_leg_hinge = object_model.get_articulation("left_front_leg_hinge")
    right_front_leg_hinge = object_model.get_articulation("right_front_leg_hinge")
    rear_brace_leg_hinge = object_model.get_articulation("rear_brace_leg_hinge")

    left_slider_sleeve = ledge.get_visual("left_slider_sleeve")
    right_slider_sleeve = ledge.get_visual("right_slider_sleeve")
    left_leg_tube = left_front_leg.get_visual("leg_tube")
    right_leg_tube = right_front_leg.get_visual("leg_tube")
    pivot_hub = top_ring.get_visual("pivot_hub")
    left_foot = left_front_leg.get_visual("left_foot")
    right_foot = right_front_leg.get_visual("right_foot")
    rear_foot = rear_brace_leg.get_visual("rear_foot")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        top_ring,
        left_front_leg,
        reason="left leg collar nests around the compact pivot hub inside the center ring",
    )
    ctx.allow_overlap(
        top_ring,
        right_front_leg,
        reason="right leg collar nests around the compact pivot hub inside the center ring",
    )
    ctx.allow_overlap(
        top_ring,
        rear_brace_leg,
        reason="rear brace collar nests around the compact pivot hub inside the center ring",
    )
    ctx.allow_overlap(
        ledge,
        left_front_leg,
        reason="left slider sleeve intentionally wraps around the left front leg tube and locks with a thumbscrew",
    )
    ctx.allow_overlap(
        ledge,
        right_front_leg,
        reason="right slider sleeve intentionally wraps around the right front leg tube and locks with a thumbscrew",
    )

    # Skip the global articulation-origin proximity sensor here because the fixed ledge placement
    # joint is a long translational mount used to locate the locked shelf assembly below the ring,
    # and that intentional offset would dominate the warning signal.
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(left_front_leg, top_ring, axes="xy", min_overlap=0.015)
    ctx.expect_overlap(right_front_leg, top_ring, axes="xy", min_overlap=0.015)
    ctx.expect_overlap(rear_brace_leg, top_ring, axes="xy", min_overlap=0.015)
    ctx.expect_overlap(ledge, left_front_leg, axes="xy", min_overlap=0.015)
    ctx.expect_overlap(ledge, right_front_leg, axes="xy", min_overlap=0.015)
    ctx.expect_overlap(ledge, left_front_leg, axes="xy", min_overlap=0.02, elem_a=left_slider_sleeve, elem_b=left_leg_tube)
    ctx.expect_overlap(ledge, right_front_leg, axes="xy", min_overlap=0.02, elem_a=right_slider_sleeve, elem_b=right_leg_tube)
    ctx.expect_gap(top_ring, ledge, axis="z", max_gap=0.85, min_gap=0.55)
    ctx.expect_origin_distance(ledge, top_ring, axes="x", max_dist=0.01)
    ctx.expect_overlap(ledge, top_ring, axes="x", min_overlap=0.015)
    ctx.expect_gap(
        right_front_leg,
        left_front_leg,
        axis="x",
        min_gap=0.45,
        positive_elem=right_foot,
        negative_elem=left_foot,
    )
    ctx.expect_gap(
        left_front_leg,
        rear_brace_leg,
        axis="y",
        min_gap=0.5,
        positive_elem=left_foot,
        negative_elem=rear_foot,
    )
    ctx.expect_gap(
        right_front_leg,
        rear_brace_leg,
        axis="y",
        min_gap=0.5,
        positive_elem=right_foot,
        negative_elem=rear_foot,
    )

    with ctx.pose(
        {
            left_front_leg_hinge: -FRONT_LEG_TILT + math.radians(1.2),
            right_front_leg_hinge: -FRONT_LEG_TILT + math.radians(1.2),
            rear_brace_leg_hinge: -REAR_LEG_TILT + math.radians(1.0),
        }
    ):
        ctx.expect_gap(
            right_front_leg,
            left_front_leg,
            axis="x",
            max_gap=0.05,
            max_penetration=0.0,
            positive_elem=right_foot,
            negative_elem=left_foot,
        )
        ctx.expect_gap(
            left_front_leg,
            rear_brace_leg,
            axis="y",
            max_gap=0.09,
            max_penetration=0.0,
            positive_elem=left_foot,
            negative_elem=rear_foot,
        )
        ctx.expect_gap(
            right_front_leg,
            rear_brace_leg,
            axis="y",
            max_gap=0.09,
            max_penetration=0.0,
            positive_elem=right_foot,
            negative_elem=rear_foot,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
