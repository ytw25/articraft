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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

EYE_OFFSET = 0.040
EYE_OUTER_RADIUS = 0.0205
EYE_BORE_RADIUS = 0.0169
EYE_LENGTH = 0.024
BRIDGE_Z = 0.056
BRIDGE_XY_SPAN = 0.066
BRIDGE_THICKNESS = 0.026
BRIDGE_HEIGHT = 0.018
ARM_BRIDGE_OFFSET = 0.021
ARM_BRIDGE_Z = 0.049
ARM_EYE_OFFSET = 0.036
ARM_EYE_Z = 0.025
ARM_RADIUS = 0.0085
NECK_RADIUS = 0.020
NECK_LENGTH = 0.032
NECK_CENTER_Z = 0.084
SHAFT_RADIUS = 0.012
SHAFT_LENGTH = 0.072
SHAFT_CENTER_Z = 0.124
SPLINE_LENGTH = 0.038
SPLINE_CENTER_Z = 0.160
TRUNNION_RADIUS = 0.0085
TRUNNION_LENGTH = 0.024
TRUNNION_CENTER = 0.020
SPIDER_HUB_RADIUS = 0.0135
CUP_OUTER_RADIUS = 0.0165
ROLLER_RADIUS = 0.00175
ROLLER_LENGTH = 0.010
CUP_INNER_RADIUS = TRUNNION_RADIUS + 2.0 * ROLLER_RADIUS
CUP_LENGTH = 0.018
CUP_OPEN_POS = 0.029
CUP_CENTER = CUP_OPEN_POS + CUP_LENGTH * 0.5
CUP_CAP_THICKNESS = 0.002
CUP_CAP_CENTER = CUP_OPEN_POS + CUP_LENGTH - CUP_CAP_THICKNESS * 0.5
SNAP_RING_OUTER = 0.0183
SNAP_RING_INNER = 0.0169
SNAP_RING_THICKNESS = 0.0012
SNAP_RING_CENTER = CUP_OPEN_POS + SNAP_RING_THICKNESS * 0.5
ROLLER_CENTER_RADIUS = TRUNNION_RADIUS + ROLLER_RADIUS
ROLLER_AXIS_CENTER = CUP_OPEN_POS + 0.0095
ROLLER_COUNT = 8
FORTY_FIVE_DEG = math.radians(45.0)


def _circle_profile(radius: float, segments: int = 32) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(math.tau * i / segments),
            radius * math.sin(math.tau * i / segments),
        )
        for i in range(segments)
    ]


def _spline_profile(root_radius: float, tooth_radius: float, teeth: int = 12) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    for i in range(teeth * 2):
        angle = math.pi * i / teeth
        radius = tooth_radius if i % 2 == 0 else root_radius
        pts.append((radius * math.cos(angle), radius * math.sin(angle)))
    return pts


def _axis_rpy(axis: str, side: int) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi * 0.5 if side > 0 else -math.pi * 0.5, 0.0)
    return (-math.pi * 0.5 if side > 0 else math.pi * 0.5, 0.0, 0.0)


def _axis_center(
    axis: str,
    side: int,
    offset: float,
    radial_a: float = 0.0,
    radial_b: float = 0.0,
) -> tuple[float, float, float]:
    if axis == "x":
        return (side * offset, radial_a, radial_b)
    return (radial_a, side * offset, radial_b)


def _segment_pose(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    vz = end[2] - start[2]
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.hypot(vx, vy), vz)
    return (
        Origin(
            xyz=(
                (start[0] + end[0]) * 0.5,
                (start[1] + end[1]) * 0.5,
                (start[2] + end[2]) * 0.5,
            ),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_arm_strut(
    part,
    prefix: str,
    side_name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    material,
) -> None:
    origin, length = _segment_pose(start, end)
    part.visual(
        Cylinder(radius=ARM_RADIUS, length=length),
        origin=origin,
        material=material,
        name=f"{prefix}_arm_{side_name}",
    )
    part.visual(
        Sphere(radius=ARM_RADIUS * 1.08),
        origin=Origin(xyz=start),
        material=material,
        name=f"{prefix}_arm_{side_name}_bridge_fillet",
    )
    part.visual(
        Sphere(radius=ARM_RADIUS * 0.92),
        origin=Origin(xyz=end),
        material=material,
        name=f"{prefix}_arm_{side_name}_eye_fillet",
    )


def _add_cup_cluster(
    part,
    prefix: str,
    axis: str,
    side: int,
    cup_shell,
    snap_ring,
    cup_material,
    roller_material,
    ring_material,
) -> None:
    side_name = "pos" if side > 0 else "neg"
    rpy = _axis_rpy(axis, side)

    part.visual(
        cup_shell,
        origin=Origin(xyz=_axis_center(axis, side, CUP_CENTER), rpy=rpy),
        material=cup_material,
        name=f"{prefix}_cup_{side_name}",
    )
    part.visual(
        Cylinder(radius=CUP_OUTER_RADIUS - 0.0004, length=CUP_CAP_THICKNESS),
        origin=Origin(xyz=_axis_center(axis, side, CUP_CAP_CENTER), rpy=rpy),
        material=cup_material,
        name=f"{prefix}_cup_{side_name}_cap",
    )
    part.visual(
        snap_ring,
        origin=Origin(xyz=_axis_center(axis, side, SNAP_RING_CENTER), rpy=rpy),
        material=ring_material,
        name=f"{prefix}_snap_ring_{side_name}",
    )

    for i in range(ROLLER_COUNT):
        angle = math.tau * i / ROLLER_COUNT
        radial_a = ROLLER_CENTER_RADIUS * math.cos(angle)
        radial_b = ROLLER_CENTER_RADIUS * math.sin(angle)
        part.visual(
            Cylinder(radius=ROLLER_RADIUS, length=ROLLER_LENGTH),
            origin=Origin(
                xyz=_axis_center(axis, side, ROLLER_AXIS_CENTER, radial_a, radial_b),
                rpy=rpy,
            ),
            material=roller_material,
            name=f"{prefix}_roller_{side_name}_{i}",
        )


def _add_yoke(
    part,
    prefix: str,
    shaft_sign: int,
    fork_axis: str,
    spline_mesh,
    eyelet_shell,
    cup_shell,
    snap_ring,
    forged_material,
    machined_material,
    cup_material,
    roller_material,
    ring_material,
) -> None:
    z_sign = float(shaft_sign)

    part.visual(
        Cylinder(radius=NECK_RADIUS, length=NECK_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, z_sign * NECK_CENTER_Z)),
        material=forged_material,
        name=f"{prefix}_hub",
    )
    part.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, z_sign * SHAFT_CENTER_Z)),
        material=machined_material,
        name=f"{prefix}_shaft",
    )
    part.visual(
        spline_mesh,
        origin=Origin(xyz=(0.0, 0.0, z_sign * SPLINE_CENTER_Z)),
        material=machined_material,
        name=f"{prefix}_spline",
    )

    bridge_size = (
        (BRIDGE_XY_SPAN, BRIDGE_THICKNESS, BRIDGE_HEIGHT)
        if fork_axis == "x"
        else (BRIDGE_THICKNESS, BRIDGE_XY_SPAN, BRIDGE_HEIGHT)
    )
    part.visual(
        Box(bridge_size),
        origin=Origin(xyz=(0.0, 0.0, z_sign * BRIDGE_Z)),
        material=forged_material,
        name=f"{prefix}_bridge",
    )

    if fork_axis == "x":
        eye_centers = [(-EYE_OFFSET, 0.0, 0.0), (EYE_OFFSET, 0.0, 0.0)]
        arm_starts = [
            (-ARM_BRIDGE_OFFSET, 0.0, z_sign * ARM_BRIDGE_Z),
            (ARM_BRIDGE_OFFSET, 0.0, z_sign * ARM_BRIDGE_Z),
        ]
        arm_ends = [
            (-ARM_EYE_OFFSET, 0.0, z_sign * ARM_EYE_Z),
            (ARM_EYE_OFFSET, 0.0, z_sign * ARM_EYE_Z),
        ]
        eye_rpy = (0.0, math.pi * 0.5, 0.0)
    else:
        eye_centers = [(0.0, -EYE_OFFSET, 0.0), (0.0, EYE_OFFSET, 0.0)]
        arm_starts = [
            (0.0, -ARM_BRIDGE_OFFSET, z_sign * ARM_BRIDGE_Z),
            (0.0, ARM_BRIDGE_OFFSET, z_sign * ARM_BRIDGE_Z),
        ]
        arm_ends = [
            (0.0, -ARM_EYE_OFFSET, z_sign * ARM_EYE_Z),
            (0.0, ARM_EYE_OFFSET, z_sign * ARM_EYE_Z),
        ]
        eye_rpy = (-math.pi * 0.5, 0.0, 0.0)

    for idx, start in enumerate(arm_starts):
        side_name = "neg" if idx == 0 else "pos"
        _add_arm_strut(
            part,
            prefix=prefix,
            side_name=side_name,
            start=start,
            end=arm_ends[idx],
            material=forged_material,
        )

    for idx, center in enumerate(eye_centers):
        side = -1 if idx == 0 else 1
        side_name = "neg" if side < 0 else "pos"
        part.visual(
            eyelet_shell,
            origin=Origin(xyz=center, rpy=eye_rpy),
            material=forged_material,
            name=f"{prefix}_eye_{side_name}",
        )
        _add_cup_cluster(
            part,
            prefix=prefix,
            axis=fork_axis,
            side=side,
            cup_shell=cup_shell,
            snap_ring=snap_ring,
            cup_material=cup_material,
            roller_material=roller_material,
            ring_material=ring_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="universal_joint_cardan", assets=ASSETS)

    forged_steel = model.material("forged_steel", rgba=(0.29, 0.30, 0.32, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    oxide_black = model.material("oxide_black", rgba=(0.10, 0.10, 0.11, 1.0))

    eyelet_shell = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(EYE_OUTER_RADIUS, segments=48),
            [_circle_profile(EYE_BORE_RADIUS, segments=48)],
            EYE_LENGTH,
            cap=True,
            center=True,
            closed=True,
        ),
        ASSETS.mesh_path("yoke_eyelet_shell.obj"),
    )
    cup_shell = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(CUP_OUTER_RADIUS, segments=40),
            [_circle_profile(CUP_INNER_RADIUS, segments=40)],
            CUP_LENGTH,
            cap=True,
            center=True,
            closed=True,
        ),
        ASSETS.mesh_path("bearing_cup_shell.obj"),
    )
    snap_ring = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(SNAP_RING_OUTER, segments=36),
            [_circle_profile(SNAP_RING_INNER, segments=36)],
            SNAP_RING_THICKNESS,
            cap=True,
            center=True,
            closed=True,
        ),
        ASSETS.mesh_path("snap_ring.obj"),
    )
    spline_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            _spline_profile(SHAFT_RADIUS * 0.92, SHAFT_RADIUS * 1.18, teeth=12),
            SPLINE_LENGTH,
            cap=True,
            closed=True,
        ),
        ASSETS.mesh_path("spline_stub.obj"),
    )

    input_yoke = model.part("input_yoke")
    _add_yoke(
        input_yoke,
        prefix="input",
        shaft_sign=-1,
        fork_axis="x",
        spline_mesh=spline_mesh,
        eyelet_shell=eyelet_shell,
        cup_shell=cup_shell,
        snap_ring=snap_ring,
        forged_material=forged_steel,
        machined_material=machined_steel,
        cup_material=machined_steel,
        roller_material=bearing_steel,
        ring_material=oxide_black,
    )
    input_yoke.inertial = Inertial.from_geometry(
        Box((0.096, 0.070, 0.190)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    spider = model.part("spider")
    spider.visual(
        Sphere(radius=SPIDER_HUB_RADIUS),
        material=oxide_black,
        name="spider_hub",
    )
    spider.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(xyz=(TRUNNION_CENTER, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=oxide_black,
        name="trunnion_pos_x",
    )
    spider.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(xyz=(-TRUNNION_CENTER, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=oxide_black,
        name="trunnion_neg_x",
    )
    spider.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(xyz=(0.0, TRUNNION_CENTER, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=oxide_black,
        name="trunnion_pos_y",
    )
    spider.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(xyz=(0.0, -TRUNNION_CENTER, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=oxide_black,
        name="trunnion_neg_y",
    )
    spider.inertial = Inertial.from_geometry(
        Box((0.082, 0.082, 0.030)),
        mass=0.65,
        origin=Origin(),
    )

    output_yoke = model.part("output_yoke")
    _add_yoke(
        output_yoke,
        prefix="output",
        shaft_sign=1,
        fork_axis="y",
        spline_mesh=spline_mesh,
        eyelet_shell=eyelet_shell,
        cup_shell=cup_shell,
        snap_ring=snap_ring,
        forged_material=forged_steel,
        machined_material=machined_steel,
        cup_material=machined_steel,
        roller_material=bearing_steel,
        ring_material=oxide_black,
    )
    output_yoke.inertial = Inertial.from_geometry(
        Box((0.070, 0.096, 0.190)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    model.articulation(
        "input_trunnion_pair",
        ArticulationType.REVOLUTE,
        parent="input_yoke",
        child="spider",
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=4.0,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "output_trunnion_pair",
        ArticulationType.REVOLUTE,
        parent="spider",
        child="output_yoke",
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=4.0,
            lower=-FORTY_FIVE_DEG,
            upper=FORTY_FIVE_DEG,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.03)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("spider", "input_yoke", axes="xy", max_dist=0.001)
    ctx.expect_origin_distance("spider", "input_yoke", axes="z", max_dist=0.001)
    ctx.expect_origin_distance("spider", "output_yoke", axes="xy", max_dist=0.001)
    ctx.expect_origin_distance("spider", "output_yoke", axes="z", max_dist=0.001)
    ctx.expect_aabb_overlap("input_yoke", "spider", axes="yz", min_overlap=0.024)
    ctx.expect_aabb_overlap("output_yoke", "spider", axes="xz", min_overlap=0.024)
    ctx.expect_aabb_gap(
        "input_yoke",
        "spider",
        axis="x",
        min_gap=0.010,
        max_gap=0.016,
        positive_elem="input_cup_pos_cap",
        negative_elem="trunnion_pos_x",
    )
    ctx.expect_aabb_gap(
        "spider",
        "input_yoke",
        axis="x",
        min_gap=0.010,
        max_gap=0.016,
        positive_elem="trunnion_neg_x",
        negative_elem="input_cup_neg_cap",
    )
    ctx.expect_aabb_gap(
        "output_yoke",
        "spider",
        axis="y",
        min_gap=0.010,
        max_gap=0.016,
        positive_elem="output_cup_pos_cap",
        negative_elem="trunnion_pos_y",
    )
    ctx.expect_aabb_gap(
        "spider",
        "output_yoke",
        axis="y",
        min_gap=0.010,
        max_gap=0.016,
        positive_elem="trunnion_neg_y",
        negative_elem="output_cup_neg_cap",
    )
    ctx.expect_aabb_gap(
        "output_yoke",
        "input_yoke",
        axis="z",
        min_gap=0.240,
        positive_elem="output_spline",
        negative_elem="input_spline",
    )
    ctx.expect_joint_motion_axis(
        "output_trunnion_pair",
        "output_yoke",
        world_axis="x",
        direction="positive",
        min_delta=0.035,
    )

    with ctx.pose(output_trunnion_pair=FORTY_FIVE_DEG):
        ctx.expect_aabb_gap(
            "output_yoke",
            "input_yoke",
            axis="x",
            min_gap=0.055,
            positive_elem="output_spline",
            negative_elem="input_spline",
        )
        ctx.expect_aabb_overlap("output_yoke", "spider", axes="xy", min_overlap=0.018)
        ctx.expect_aabb_gap(
            "output_yoke",
            "spider",
            axis="y",
            min_gap=0.010,
            max_gap=0.016,
            positive_elem="output_cup_pos_cap",
            negative_elem="trunnion_pos_y",
        )
        ctx.expect_aabb_gap(
            "spider",
            "output_yoke",
            axis="y",
            min_gap=0.010,
            max_gap=0.016,
            positive_elem="trunnion_neg_y",
            negative_elem="output_cup_neg_cap",
        )

    with ctx.pose(input_trunnion_pair=0.30):
        ctx.expect_aabb_gap(
            "input_yoke",
            "spider",
            axis="x",
            min_gap=0.010,
            max_gap=0.016,
            positive_elem="input_cup_pos_cap",
            negative_elem="trunnion_pos_x",
        )
        ctx.expect_aabb_gap(
            "spider",
            "input_yoke",
            axis="x",
            min_gap=0.010,
            max_gap=0.016,
            positive_elem="trunnion_neg_x",
            negative_elem="input_cup_neg_cap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
