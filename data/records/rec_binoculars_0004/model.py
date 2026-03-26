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
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_tripod_binocular", assets=ASSETS)

    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    cast_gray = model.material("cast_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    scale_white = model.material("scale_white", rgba=(0.84, 0.84, 0.80, 1.0))
    glass = model.material("glass", rgba=(0.24, 0.34, 0.39, 0.32))

    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.085, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.995)),
        material=dark_metal,
        name="crown",
    )
    tripod.visual(
        Cylinder(radius=0.027, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 1.078)),
        material=aluminum,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.038, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 1.150)),
        material=dark_metal,
        name="head_receiver",
    )
    tripod.visual(
        Box((0.124, 0.124, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 1.172)),
        material=dark_metal,
        name="top_platform",
    )
    tripod.visual(
        Cylinder(radius=0.048, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        material=dark_metal,
        name="spreader_hub",
    )
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0 + math.pi / 6.0
        ca = math.cos(angle)
        sa = math.sin(angle)
        top = (0.082 * ca, 0.082 * sa, 0.985)
        mid = (0.215 * ca, 0.215 * sa, 0.555)
        foot = (0.448 * ca, 0.448 * sa, 0.040)
        tip = (0.472 * ca, 0.472 * sa, 0.006)
        _add_member(tripod, top, mid, 0.0125, aluminum, name=f"upper_leg_{index}")
        _add_member(tripod, mid, foot, 0.0105, aluminum, name=f"lower_leg_{index}")
        _add_member(tripod, foot, tip, 0.0145, rubber_black, name=f"foot_cap_{index}")
        _add_member(tripod, (0.0, 0.0, 0.690), mid, 0.0060, aluminum, name=f"spreader_{index}")
    tripod.inertial = Inertial.from_geometry(
        Box((1.04, 1.04, 1.22)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
    )

    collar = model.part("swivel_collar")
    collar.visual(
        Cylinder(radius=0.108, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_gray,
        name="azimuth_ring",
    )
    collar.visual(
        Cylinder(radius=0.052, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_metal,
        name="vertical_collar",
    )
    collar.visual(
        Cylinder(radius=0.078, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=cast_gray,
        name="top_flange",
    )
    for tick_index in range(12):
        angle = (2.0 * math.pi * tick_index) / 12.0
        radius = 0.100
        tick_length = 0.018 if tick_index % 3 == 0 else 0.011
        collar.visual(
            Box((tick_length, 0.0035, 0.005)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.0205),
                rpy=(0.0, 0.0, angle),
            ),
            material=scale_white,
            name=f"azimuth_tick_{tick_index}",
        )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.108, length=0.118),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
    )

    yoke = model.part("cradle_yoke")
    yoke.visual(
        Box((0.120, 0.050, 0.018)),
        origin=Origin(xyz=(-0.050, 0.0, 0.009)),
        material=cast_gray,
        name="pivot_base",
    )
    yoke.visual(
        Box((0.090, 0.060, 0.010)),
        origin=Origin(xyz=(-0.055, 0.0, 0.029)),
        material=cast_gray,
        name="saddle",
    )
    yoke.visual(
        Box((0.040, 0.032, 0.016)),
        origin=Origin(xyz=(-0.055, 0.0, 0.026)),
        material=cast_gray,
        name="saddle_post",
    )
    yoke.visual(
        Box((0.114, 0.012, 0.108)),
        origin=Origin(xyz=(-0.002, 0.184, 0.066)),
        material=cast_gray,
        name="left_arm",
    )
    yoke.visual(
        Box((0.114, 0.012, 0.108)),
        origin=Origin(xyz=(-0.002, -0.184, 0.066)),
        material=cast_gray,
        name="right_arm",
    )
    _add_member(
        yoke,
        (-0.030, 0.022, 0.010),
        (-0.028, 0.180, 0.026),
        0.008,
        cast_gray,
        name="left_brace",
    )
    _add_member(
        yoke,
        (-0.030, -0.022, 0.010),
        (-0.028, -0.180, 0.026),
        0.008,
        cast_gray,
        name="right_brace",
    )
    yoke.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.175, 0.069), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_pivot_collar",
    )
    yoke.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, -0.175, 0.069), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_pivot_collar",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.180, 0.370, 0.140)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    body = model.part("binocular_body")
    for side_name, side_y in (("left", 0.086), ("right", -0.086)):
        body.visual(
            Cylinder(radius=0.068, length=0.340),
            origin=Origin(xyz=(0.002, side_y * 1.06, 0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=cast_gray,
            name=f"{side_name}_objective_tube",
        )
        body.visual(
            Cylinder(radius=0.074, length=0.250),
            origin=Origin(xyz=(-0.012, side_y * 1.06, 0.036), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_black,
            name=f"{side_name}_armor",
        )
        body.visual(
            Cylinder(radius=0.079, length=0.016),
            origin=Origin(xyz=(0.176, side_y * 1.06, 0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_metal,
            name=f"{side_name}_objective_rim",
        )
        body.visual(
            Cylinder(radius=0.031, length=0.072),
            origin=Origin(xyz=(-0.184, side_y * 0.83, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_black,
            name=f"{side_name}_eyepiece",
        )
        body.visual(
            Cylinder(radius=0.026, length=0.028),
            origin=Origin(xyz=(-0.220, side_y * 0.83, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber_black,
            name=f"{side_name}_eyecup",
        )
        body.visual(
            Cylinder(radius=0.019, length=0.012),
            origin=Origin(xyz=(-0.239, side_y * 0.83, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass,
            name=f"{side_name}_eyelens",
        )
        body.visual(
            Cylinder(radius=0.059, length=0.008),
            origin=Origin(xyz=(0.184, side_y * 1.06, 0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass,
            name=f"{side_name}_objective_glass",
        )
    body.visual(
        Box((0.170, 0.184, 0.086)),
        origin=Origin(xyz=(-0.022, 0.0, 0.050)),
        material=cast_gray,
        name="center_housing",
    )
    body.visual(
        Box((0.112, 0.140, 0.054)),
        origin=Origin(xyz=(-0.052, 0.0, 0.094)),
        material=cast_gray,
        name="prism_housing",
    )
    body.visual(
        Box((0.100, 0.110, 0.044)),
        origin=Origin(xyz=(-0.126, 0.0, 0.058)),
        material=cast_gray,
        name="eyepiece_bridge",
    )
    body.visual(
        Box((0.124, 0.090, 0.020)),
        origin=Origin(xyz=(-0.006, 0.0, -0.025)),
        material=dark_metal,
        name="mount_foot",
    )
    body.visual(
        Box((0.086, 0.078, 0.014)),
        origin=Origin(xyz=(-0.012, 0.0, -0.008)),
        material=rubber_black,
        name="underside_armor",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(0.0, 0.159, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_trunnion",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(0.0, -0.159, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_trunnion",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.042),
        origin=Origin(xyz=(-0.036, 0.0, 0.142)),
        material=dark_metal,
        name="focus_bridge",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.169, 0.09116, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_cap_pin",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.024),
        origin=Origin(xyz=(0.169, -0.09116, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_cap_pin",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.470, 0.360, 0.220)),
        mass=5.6,
        origin=Origin(xyz=(-0.010, 0.0, 0.050)),
    )

    left_cap = model.part("left_lens_cap")
    left_cap.visual(
        Cylinder(radius=0.072, length=0.010),
        origin=Origin(xyz=(0.020, 0.0, -0.076), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="lid",
    )
    left_cap.visual(
        Box((0.016, 0.032, 0.064)),
        origin=Origin(xyz=(0.010, 0.0, -0.032)),
        material=rubber_black,
        name="hinge_strap",
    )
    left_cap.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="hinge_sleeve",
    )
    left_cap.inertial = Inertial.from_geometry(
        Box((0.102, 0.086, 0.106)),
        mass=0.08,
        origin=Origin(xyz=(0.012, 0.0, -0.040)),
    )

    right_cap = model.part("right_lens_cap")
    right_cap.visual(
        Cylinder(radius=0.072, length=0.010),
        origin=Origin(xyz=(0.020, 0.0, -0.076), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="lid",
    )
    right_cap.visual(
        Box((0.016, 0.032, 0.064)),
        origin=Origin(xyz=(0.010, 0.0, -0.032)),
        material=rubber_black,
        name="hinge_strap",
    )
    right_cap.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="hinge_sleeve",
    )
    right_cap.inertial = Inertial.from_geometry(
        Box((0.102, 0.086, 0.106)),
        mass=0.08,
        origin=Origin(xyz=(0.012, 0.0, -0.040)),
    )

    model.articulation(
        "tripod_swivel",
        ArticulationType.CONTINUOUS,
        parent="tripod",
        child="swivel_collar",
        origin=Origin(xyz=(0.0, 0.0, 1.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.6),
    )
    model.articulation(
        "collar_to_yoke",
        ArticulationType.FIXED,
        parent="swivel_collar",
        child="cradle_yoke",
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
    )
    model.articulation(
        "yoke_tilt",
        ArticulationType.REVOLUTE,
        parent="cradle_yoke",
        child="binocular_body",
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=0.45),
    )
    model.articulation(
        "left_cap_hinge",
        ArticulationType.REVOLUTE,
        parent="binocular_body",
        child="left_lens_cap",
        origin=Origin(xyz=(0.169, 0.09116, 0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.40, upper=0.0),
    )
    model.articulation(
        "right_cap_hinge",
        ArticulationType.REVOLUTE,
        parent="binocular_body",
        child="right_lens_cap",
        origin=Origin(xyz=(0.169, -0.09116, 0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.40, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    collar = object_model.get_part("swivel_collar")
    yoke = object_model.get_part("cradle_yoke")
    body = object_model.get_part("binocular_body")
    left_cap = object_model.get_part("left_lens_cap")
    right_cap = object_model.get_part("right_lens_cap")
    swivel = object_model.get_articulation("tripod_swivel")
    collar_mount = object_model.get_articulation("collar_to_yoke")
    tilt = object_model.get_articulation("yoke_tilt")
    left_hinge = object_model.get_articulation("left_cap_hinge")
    right_hinge = object_model.get_articulation("right_cap_hinge")

    tripod_platform = tripod.get_visual("top_platform")
    azimuth_ring = collar.get_visual("azimuth_ring")
    collar_top = collar.get_visual("top_flange")
    yoke_base = yoke.get_visual("pivot_base")
    yoke_saddle = yoke.get_visual("saddle")
    left_yoke_collar = yoke.get_visual("left_pivot_collar")
    right_yoke_collar = yoke.get_visual("right_pivot_collar")
    body_mount = body.get_visual("mount_foot")
    left_trunnion = body.get_visual("left_trunnion")
    right_trunnion = body.get_visual("right_trunnion")
    left_tube = body.get_visual("left_objective_tube")
    right_tube = body.get_visual("right_objective_tube")
    left_rim = body.get_visual("left_objective_rim")
    right_rim = body.get_visual("right_objective_rim")
    left_objective_glass = body.get_visual("left_objective_glass")
    right_objective_glass = body.get_visual("right_objective_glass")
    left_pin = body.get_visual("left_cap_pin")
    right_pin = body.get_visual("right_cap_pin")
    left_body_tube = body.get_visual("left_objective_tube")
    left_cap_lid = left_cap.get_visual("lid")
    right_cap_lid = right_cap.get_visual("lid")
    left_hinge_strap = left_cap.get_visual("hinge_strap")
    right_hinge_strap = right_cap.get_visual("hinge_strap")
    left_hinge_sleeve = left_cap.get_visual("hinge_sleeve")
    right_hinge_sleeve = right_cap.get_visual("hinge_sleeve")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.allow_overlap(
        left_trunnion,
        left_yoke_collar,
        reason="left tilt trunnion is captured by the cradle-side bearing collar",
    )
    ctx.allow_overlap(
        right_trunnion,
        right_yoke_collar,
        reason="right tilt trunnion is captured by the cradle-side bearing collar",
    )
    ctx.allow_overlap(
        left_cap,
        body,
        reason="left objective cap closes over the glass and its hinge strap wraps the rim-side hinge hardware",
    )
    ctx.allow_overlap(
        right_cap,
        body,
        reason="right objective cap closes over the glass and its hinge strap wraps the rim-side hinge hardware",
    )
    ctx.allow_overlap(
        left_pin,
        left_hinge_sleeve,
        reason="left cap hinge sleeve rotates around the cap pin",
    )
    ctx.allow_overlap(
        right_pin,
        right_hinge_sleeve,
        reason="right cap hinge sleeve rotates around the cap pin",
    )
    ctx.allow_overlap(
        left_pin,
        left_hinge_strap,
        reason="left cap hinge strap wraps around the hinge pin root",
    )
    ctx.allow_overlap(
        right_pin,
        right_hinge_strap,
        reason="right cap hinge strap wraps around the hinge pin root",
    )
    ctx.allow_overlap(
        left_rim,
        left_hinge_sleeve,
        reason="left cap hinge sleeve nests against the objective rim",
    )
    ctx.allow_overlap(
        right_rim,
        right_hinge_sleeve,
        reason="right cap hinge sleeve nests against the objective rim",
    )
    ctx.allow_overlap(
        left_rim,
        left_hinge_strap,
        reason="left cap hinge strap seats against the objective rim",
    )
    ctx.allow_overlap(
        right_rim,
        right_hinge_strap,
        reason="right cap hinge strap seats against the objective rim",
    )
    ctx.allow_overlap(
        left_objective_glass,
        left_cap_lid,
        reason="left cap lid closes directly over the objective glass",
    )
    ctx.allow_overlap(
        right_objective_glass,
        right_cap_lid,
        reason="right cap lid closes directly over the objective glass",
    )
    ctx.allow_overlap(
        left_objective_glass,
        left_hinge_strap,
        reason="left cap hinge strap tucks into the objective surround when closed",
    )
    ctx.allow_overlap(
        right_objective_glass,
        right_hinge_strap,
        reason="right cap hinge strap tucks into the objective surround when closed",
    )
    ctx.allow_overlap(
        left_tube,
        left_hinge_strap,
        reason="left cap hinge strap hugs the outside of the objective tube",
    )
    ctx.allow_overlap(
        right_tube,
        right_hinge_strap,
        reason="right cap hinge strap hugs the outside of the objective tube",
    )
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        collar,
        tripod,
        axes="xy",
        min_overlap=0.015,
        elem_a=azimuth_ring,
        elem_b=tripod_platform,
    )
    ctx.expect_gap(
        collar,
        tripod,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=azimuth_ring,
        negative_elem=tripod_platform,
    )
    ctx.expect_overlap(
        yoke,
        collar,
        axes="xy",
        min_overlap=0.010,
        elem_a=yoke_base,
        elem_b=collar_top,
    )
    ctx.expect_gap(
        yoke,
        collar,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=yoke_base,
        negative_elem=collar_top,
    )
    ctx.expect_overlap(
        body,
        yoke,
        axes="yz",
        min_overlap=0.001,
        elem_a=left_trunnion,
        elem_b=left_yoke_collar,
    )
    ctx.expect_overlap(
        body,
        yoke,
        axes="yz",
        min_overlap=0.001,
        elem_a=right_trunnion,
        elem_b=right_yoke_collar,
    )
    ctx.expect_gap(
        body,
        yoke,
        axis="z",
        max_gap=0.003,
        max_penetration=0.001,
        positive_elem=body_mount,
        negative_elem=yoke_saddle,
    )
    ctx.expect_gap(
        left_cap,
        body,
        axis="x",
        max_gap=0.003,
        max_penetration=0.001,
        positive_elem=left_cap_lid,
        negative_elem=left_rim,
    )
    ctx.expect_gap(
        right_cap,
        body,
        axis="x",
        max_gap=0.003,
        max_penetration=0.001,
        positive_elem=right_cap_lid,
        negative_elem=right_rim,
    )
    ctx.expect_overlap(
        left_cap,
        body,
        axes="yz",
        min_overlap=0.010,
        elem_a=left_cap_lid,
        elem_b=left_rim,
    )
    ctx.expect_overlap(
        right_cap,
        body,
        axes="yz",
        min_overlap=0.010,
        elem_a=right_cap_lid,
        elem_b=right_rim,
    )
    ctx.expect_overlap(
        left_cap,
        body,
        axes="yz",
        min_overlap=0.006,
        elem_a=left_hinge_sleeve,
        elem_b=left_pin,
    )
    ctx.expect_overlap(
        right_cap,
        body,
        axes="yz",
        min_overlap=0.006,
        elem_a=right_hinge_sleeve,
        elem_b=right_pin,
    )

    with ctx.pose({swivel: 0.85}):
        ctx.expect_overlap(
            collar,
            tripod,
            axes="xy",
            min_overlap=0.010,
            elem_a=azimuth_ring,
            elem_b=tripod_platform,
        )
        ctx.expect_gap(
            collar,
            tripod,
            axis="z",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem=azimuth_ring,
            negative_elem=tripod_platform,
        )
        ctx.expect_gap(
            yoke,
            collar,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=yoke_base,
            negative_elem=collar_top,
        )

    with ctx.pose({tilt: 0.45}):
        ctx.expect_gap(
            body,
            tripod,
            axis="z",
            min_gap=0.030,
            positive_elem=left_body_tube,
            negative_elem=tripod_platform,
        )
        ctx.expect_overlap(
            body,
            yoke,
            axes="yz",
            min_overlap=0.001,
            elem_a=left_trunnion,
            elem_b=left_yoke_collar,
        )
        ctx.expect_overlap(
            body,
            yoke,
            axes="yz",
            min_overlap=0.001,
            elem_a=right_trunnion,
            elem_b=right_yoke_collar,
        )

    with ctx.pose({left_hinge: -1.30, right_hinge: -1.30}):
        ctx.expect_overlap(
            left_cap,
            body,
            axes="yz",
            min_overlap=0.006,
            elem_a=left_hinge_sleeve,
            elem_b=left_pin,
        )
        ctx.expect_overlap(
            right_cap,
            body,
            axes="yz",
            min_overlap=0.006,
            elem_a=right_hinge_sleeve,
            elem_b=right_pin,
        )
        ctx.expect_gap(
            left_cap,
            tripod,
            axis="z",
            min_gap=0.090,
            positive_elem=left_cap_lid,
            negative_elem=tripod_platform,
        )
        ctx.expect_gap(
            right_cap,
            tripod,
            axis="z",
            min_gap=0.090,
            positive_elem=right_cap_lid,
            negative_elem=tripod_platform,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
