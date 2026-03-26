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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)

ASSETS = AssetContext.from_script(__file__)

BASE_LENGTH = 0.063
BASE_REAR_WIDTH = 0.0145
BASE_MID_WIDTH = 0.0138
BASE_FRONT_WIDTH = 0.0128
BASE_THICKNESS = 0.0018

TOP_REAR_WIDTH = 0.0133
TOP_MID_WIDTH = 0.0127
TOP_FRONT_WIDTH = 0.0115
TOP_THICKNESS = 0.0016

PIN_X = 0.048
PIN_HOLE_RADIUS = 0.00175
REAR_FLEX_X = 0.004

PIN_FLANGE_THICKNESS = 0.0009
PIN_FLANGE_RADIUS = 0.0027
PIN_STEM_RADIUS = 0.00135
PIN_STEM_HEIGHT = 0.0043
PIN_NECK_RADIUS = 0.00105
PIN_NECK_HEIGHT = 0.0010
PIN_HEAD_RADIUS = 0.00175
PIN_HEAD_THICKNESS = 0.0008
PIN_NECK_Z = PIN_STEM_HEIGHT + 0.5 * PIN_NECK_HEIGHT

LEVER_LENGTH = 0.056
LEVER_BODY_THICKNESS = 0.0020
LEVER_BODY_WIDTH = 0.0108
LEVER_FRONT_WIDTH = 0.0090
LEVER_TAIL_WIDTH = 0.0124
LEVER_YOKE_LENGTH = 0.0082
LEVER_YOKE_THICKNESS = 0.0022
LEVER_SLOT_HALF = 0.00185
LEVER_CHEEK_WIDTH = 0.00155
LEVER_REST_TILT = 0.58
TOP_STACK_GAP = 0.00004


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _smoothstep(edge0: float, edge1: float, x: float) -> float:
    if edge0 == edge1:
        return 1.0 if x >= edge1 else 0.0
    t = _clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _strip_closed(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if points and points[0] == points[-1]:
        return points[:-1]
    return points


def _circle_profile(
    center: tuple[float, float],
    radius: float,
    segments: int = 28,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * i) / segments),
            cy + radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _jaw_outline(
    *,
    length: float,
    rear_width: float,
    mid_width: float,
    front_width: float,
    bite_depth: float,
) -> list[tuple[float, float]]:
    pts = [
        (0.0000, -rear_width * 0.48),
        (0.0065, -rear_width * 0.50),
        (0.0200, -mid_width * 0.52),
        (0.0400, -mid_width * 0.49),
        (length - 0.0095, -front_width * 0.51),
        (length - 0.0020, -front_width * 0.27),
        (length - bite_depth, 0.0000),
        (length - 0.0020, front_width * 0.27),
        (length - 0.0095, front_width * 0.51),
        (0.0400, mid_width * 0.49),
        (0.0200, mid_width * 0.52),
        (0.0065, rear_width * 0.50),
        (0.0000, rear_width * 0.48),
    ]
    return _strip_closed(
        sample_catmull_rom_spline_2d(pts, samples_per_segment=12, closed=True)
    )


def _lever_outline() -> list[tuple[float, float]]:
    pts = [
        (-LEVER_LENGTH, -LEVER_TAIL_WIDTH * 0.22),
        (-LEVER_LENGTH * 0.94, -LEVER_TAIL_WIDTH * 0.45),
        (-LEVER_LENGTH * 0.78, -LEVER_TAIL_WIDTH * 0.53),
        (-LEVER_LENGTH * 0.55, -LEVER_BODY_WIDTH * 0.50),
        (-LEVER_LENGTH * 0.28, -LEVER_FRONT_WIDTH * 0.47),
        (-0.0070, -LEVER_FRONT_WIDTH * 0.41),
        (-0.0025, -LEVER_FRONT_WIDTH * 0.30),
        (-0.0020, 0.0000),
        (-0.0025, LEVER_FRONT_WIDTH * 0.30),
        (-0.0070, LEVER_FRONT_WIDTH * 0.41),
        (-LEVER_LENGTH * 0.28, LEVER_FRONT_WIDTH * 0.47),
        (-LEVER_LENGTH * 0.55, LEVER_BODY_WIDTH * 0.50),
        (-LEVER_LENGTH * 0.78, LEVER_TAIL_WIDTH * 0.53),
        (-LEVER_LENGTH * 0.94, LEVER_TAIL_WIDTH * 0.45),
        (-LEVER_LENGTH, LEVER_TAIL_WIDTH * 0.22),
    ]
    return _strip_closed(
        sample_catmull_rom_spline_2d(pts, samples_per_segment=12, closed=True)
    )


def _deform_leaf(
    geom,
    *,
    length: float,
    width: float,
    longitudinal_rise: float,
    cross_crown: float,
    tip_bevel: float,
    bevel_direction: str,
) -> None:
    max_half_width = width * 0.5
    for i, (x, y, z) in enumerate(list(geom.vertices)):
        s = _clamp(x / length, 0.0, 1.0)
        rear_blend = _smoothstep(0.08, 0.38, s)
        y_norm = _clamp(abs(y) / max_half_width, 0.0, 1.0)
        cross = cross_crown * rear_blend * (1.0 - y_norm**2)
        sweep = longitudinal_rise * (_smoothstep(0.04, 1.0, s) ** 1.35)
        front_ramp = _smoothstep(length - 0.011, length - 0.001, x)
        bevel = tip_bevel * front_ramp * (0.55 + 0.45 * (1.0 - y_norm))

        z_new = z + cross + sweep
        if bevel_direction == "up" and z > 0.0:
            z_new += bevel
        elif bevel_direction == "down" and z < 0.0:
            z_new -= bevel

        geom.vertices[i] = (x, y, z_new)


def _deform_lever_body(geom) -> None:
    for i, (x, y, z) in enumerate(list(geom.vertices)):
        s = _clamp((-x) / LEVER_LENGTH, 0.0, 1.0)
        y_norm = _clamp(abs(y) / (LEVER_TAIL_WIDTH * 0.5), 0.0, 1.0)
        top_round = 0.00020 * (1.0 - y_norm**2)
        pad = math.exp(-((s - 0.56) / 0.22) ** 2)
        front_stiffen = 1.0 - _smoothstep(0.22, 0.45, s)
        z_new = z
        if z > 0.0:
            depression = -0.00042 * pad * (1.0 - y_norm**1.7)
            z_new += top_round + depression
        else:
            z_new += 0.00012 * front_stiffen * (1.0 - y_norm**2)
        z_new += 0.00010 * math.exp(-((s - 0.90) / 0.10) ** 2) * (1.0 - y_norm)
        geom.vertices[i] = (x, y, z_new)


def _leaf_mesh(
    name: str,
    *,
    length: float,
    rear_width: float,
    mid_width: float,
    front_width: float,
    thickness: float,
    bite_depth: float,
    longitudinal_rise: float,
    cross_crown: float,
    tip_bevel: float,
    bevel_direction: str,
):
    geom = ExtrudeWithHolesGeometry(
        outer_profile=_jaw_outline(
            length=length,
            rear_width=rear_width,
            mid_width=mid_width,
            front_width=front_width,
            bite_depth=bite_depth,
        ),
        hole_profiles=[_circle_profile((PIN_X, 0.0), PIN_HOLE_RADIUS)],
        height=thickness,
        center=True,
        closed=True,
    )
    _deform_leaf(
        geom,
        length=length,
        width=max(rear_width, mid_width, front_width),
        longitudinal_rise=longitudinal_rise,
        cross_crown=cross_crown,
        tip_bevel=tip_bevel,
        bevel_direction=bevel_direction,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(name))


def _lever_body_mesh():
    geom = ExtrudeGeometry(
        _lever_outline(),
        height=LEVER_BODY_THICKNESS,
        center=True,
        closed=True,
    )
    _deform_lever_body(geom)
    return mesh_from_geometry(geom, ASSETS.mesh_path("lever_body.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compound_lever_nail_clipper", assets=ASSETS)

    polished_steel = model.material("polished_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.84, 0.86, 0.88, 1.0))
    shadowed_steel = model.material("shadowed_steel", rgba=(0.44, 0.46, 0.49, 1.0))

    base_mesh = _leaf_mesh(
        "base_jaw.obj",
        length=BASE_LENGTH,
        rear_width=BASE_REAR_WIDTH,
        mid_width=BASE_MID_WIDTH,
        front_width=BASE_FRONT_WIDTH,
        thickness=BASE_THICKNESS,
        bite_depth=0.0060,
        longitudinal_rise=0.00022,
        cross_crown=0.00032,
        tip_bevel=0.00038,
        bevel_direction="up",
    )
    top_mesh = _leaf_mesh(
        "top_jaw.obj",
        length=BASE_LENGTH,
        rear_width=TOP_REAR_WIDTH,
        mid_width=TOP_MID_WIDTH,
        front_width=TOP_FRONT_WIDTH,
        thickness=TOP_THICKNESS,
        bite_depth=0.0064,
        longitudinal_rise=0.0019,
        cross_crown=0.00025,
        tip_bevel=0.00034,
        bevel_direction="down",
    )
    lever_mesh = _lever_body_mesh()

    base = model.part("base_jaw")
    base.visual(
        base_mesh,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material=polished_steel,
    )

    top = model.part("top_jaw")
    top.visual(
        top_mesh,
        origin=Origin(
            xyz=(
                -REAR_FLEX_X,
                0.0,
                BASE_THICKNESS + TOP_THICKNESS * 0.5 + TOP_STACK_GAP,
            )
        ),
        material=polished_steel,
    )

    pin = model.part("pivot_pin")
    pin.visual(
        Cylinder(radius=PIN_FLANGE_RADIUS, length=PIN_FLANGE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -PIN_FLANGE_THICKNESS * 0.5)),
        material=bright_steel,
    )
    pin.visual(
        Cylinder(radius=PIN_STEM_RADIUS, length=PIN_STEM_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PIN_STEM_HEIGHT * 0.5)),
        material=bright_steel,
    )
    pin.visual(
        Cylinder(radius=PIN_NECK_RADIUS, length=PIN_NECK_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, PIN_NECK_Z)),
        material=shadowed_steel,
    )
    pin.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                PIN_STEM_HEIGHT + PIN_NECK_HEIGHT + PIN_HEAD_THICKNESS * 0.5,
            )
        ),
        material=bright_steel,
    )

    lever = model.part("lever")
    lever.visual(
        lever_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.00010)),
        material=brushed_steel,
    )
    cheek_length = LEVER_YOKE_LENGTH
    lever.visual(
        Box((cheek_length, LEVER_CHEEK_WIDTH, LEVER_YOKE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0002,
                LEVER_SLOT_HALF + LEVER_CHEEK_WIDTH * 0.5,
                -0.00010,
            )
        ),
        material=brushed_steel,
    )
    lever.visual(
        Box((cheek_length, LEVER_CHEEK_WIDTH, LEVER_YOKE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0002,
                -(LEVER_SLOT_HALF + LEVER_CHEEK_WIDTH * 0.5),
                -0.00010,
            )
        ),
        material=brushed_steel,
    )
    lever.visual(
        Cylinder(radius=0.0028, length=0.0064),
        origin=Origin(
            xyz=(-0.0048, 0.0, -0.00195),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=shadowed_steel,
    )

    model.articulation(
        "base_to_top_jaw",
        ArticulationType.REVOLUTE,
        parent="base_jaw",
        child="top_jaw",
        origin=Origin(xyz=(REAR_FLEX_X, 0.0, BASE_THICKNESS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=4.0,
            lower=-0.060,
            upper=0.0,
        ),
    )
    model.articulation(
        "base_to_pivot_pin",
        ArticulationType.FIXED,
        parent="base_jaw",
        child="pivot_pin",
        origin=Origin(xyz=(PIN_X, 0.0, 0.0)),
    )
    model.articulation(
        "lever_press",
        ArticulationType.REVOLUTE,
        parent="pivot_pin",
        child="lever",
        origin=Origin(xyz=(0.0, 0.0, PIN_NECK_Z), rpy=(0.0, LEVER_REST_TILT, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=5.0,
            lower=0.0,
            upper=0.86,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, geometry_source="collision")
    ctx.allow_overlap(
        "pivot_pin",
        "lever",
        reason="The hooked lever yoke intentionally nests around the pin neck and cap.",
    )
    ctx.allow_overlap(
        "pivot_pin",
        "top_jaw",
        reason="Generated convex hulls can conservatively overfill the drilled upper jaw hole.",
    )
    ctx.allow_overlap(
        "pivot_pin",
        "base_jaw",
        reason="Generated convex hulls can conservatively overfill the stamped lower jaw bore.",
    )
    ctx.allow_overlap(
        "lever",
        "top_jaw",
        reason="The cam-action lever intentionally bears on the sprung upper jaw, and convex hulls overfill the shallow stamped lever dish.",
    )
    ctx.allow_overlap(
        "lever",
        "base_jaw",
        reason="The folded handle passes tightly over the stacked steel leaves, so generated hulls can conservatively report overlap through the thin jaw stack.",
    )
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected(use="visual")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.0008,
        overlap_volume_tol=0.0,
    )
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.0008,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("base_jaw", "top_jaw", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_overlap("pivot_pin", "base_jaw", axes="xy", min_overlap=0.002)
    ctx.expect_aabb_overlap("pivot_pin", "top_jaw", axes="xy", min_overlap=0.002)
    ctx.expect_aabb_contact("top_jaw", "base_jaw")
    ctx.expect_aabb_contact("pivot_pin", "base_jaw")
    ctx.expect_aabb_contact("lever", "pivot_pin")
    ctx.expect_joint_motion_axis(
        "base_to_top_jaw",
        "top_jaw",
        world_axis="z",
        direction="negative",
        min_delta=0.0015,
    )
    ctx.expect_joint_motion_axis(
        "lever_press",
        "lever",
        world_axis="z",
        direction="negative",
        min_delta=0.006,
    )

    with ctx.pose(lever_press=0.0, base_to_top_jaw=0.0):
        ctx.expect_aabb_overlap("lever", "top_jaw", axes="xy", min_overlap=0.008)
        ctx.expect_aabb_contact("lever", "top_jaw")

    with ctx.pose(lever_press=0.68, base_to_top_jaw=-0.045):
        ctx.expect_aabb_overlap("top_jaw", "base_jaw", axes="xy", min_overlap=0.010)
        ctx.expect_aabb_contact("lever", "top_jaw")
        ctx.expect_aabb_contact("top_jaw", "base_jaw")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
