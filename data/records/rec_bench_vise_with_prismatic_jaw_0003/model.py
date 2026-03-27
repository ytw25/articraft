from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib
import tempfile

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

_SAFE_CWD = tempfile.gettempdir()
_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir(_SAFE_CWD)
        except FileNotFoundError:
            pass
        return _SAFE_CWD


os.getcwd = _safe_getcwd
pathlib.Path.cwd = classmethod(lambda cls: cls(_safe_getcwd()))
os.chdir(_SAFE_CWD)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pipe_fitting_mechanics_vise")

    cast_iron = model.material("cast_iron", rgba=(0.24, 0.25, 0.27, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    insert_iron = model.material("insert_iron", rgba=(0.14, 0.14, 0.15, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.11, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.190, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="floor_flange",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Cylinder(radius=0.016, length=0.028),
                origin=Origin(xyz=(x_sign * 0.122, y_sign * 0.122, 0.031)),
                material=machined_steel,
            )
    body.visual(
        Box((0.160, 0.180, 0.260)),
        origin=Origin(xyz=(0.0, 0.0, 0.1475)),
        material=cast_iron,
        name="base_stem",
    )
    body.visual(
        Box((0.048, 0.080, 0.300)),
        origin=Origin(xyz=(0.0, -0.045, 0.427)),
        material=cast_iron,
        name="rear_spine",
    )
    body.visual(
        Box((0.140, 0.110, 0.046)),
        origin=Origin(xyz=(0.0, 0.070, 0.298)),
        material=cast_iron,
        name="lower_jaw_casting",
    )
    body.visual(
        Box((0.016, 0.055, 0.030)),
        origin=Origin(xyz=(-0.042, 0.100, 0.336)),
        material=cast_iron,
        name="lower_jaw_nose_left",
    )
    body.visual(
        Box((0.016, 0.055, 0.030)),
        origin=Origin(xyz=(0.042, 0.100, 0.336)),
        material=cast_iron,
        name="lower_jaw_nose_right",
    )
    body.visual(
        Box((0.078, 0.044, 0.006)),
        origin=Origin(xyz=(0.0, 0.070, 0.324)),
        material=insert_iron,
        name="lower_insert",
    )
    for index in range(7):
        tooth_name = "lower_insert_tooth_mid" if index == 3 else None
        body.visual(
            Box((0.0075, 0.044, 0.006)),
            origin=Origin(xyz=(-0.030 + 0.010 * index, 0.070, 0.330)),
            material=insert_iron,
            name=tooth_name,
        )
    body.visual(
        Box((0.052, 0.040, 0.250)),
        origin=Origin(xyz=(0.0, -0.035, 0.437)),
        material=cast_iron,
        name="guide_rail",
    )
    body.visual(
        Box((0.060, 0.120, 0.270)),
        origin=Origin(xyz=(-0.095, 0.005, 0.485)),
        material=cast_iron,
        name="left_yoke_cheek",
    )
    body.visual(
        Box((0.060, 0.120, 0.270)),
        origin=Origin(xyz=(0.095, 0.005, 0.485)),
        material=cast_iron,
        name="right_yoke_cheek",
    )
    body.visual(
        Box((0.094, 0.130, 0.060)),
        origin=Origin(xyz=(-0.083, 0.010, 0.650)),
        material=cast_iron,
        name="left_bridge_mass",
    )
    body.visual(
        Box((0.094, 0.130, 0.060)),
        origin=Origin(xyz=(0.083, 0.010, 0.650)),
        material=cast_iron,
        name="right_bridge_mass",
    )
    body.visual(
        Box((0.082, 0.032, 0.050)),
        origin=Origin(xyz=(0.0, -0.018, 0.650)),
        material=cast_iron,
        name="rear_bridge_web",
    )
    body.visual(
        Box((0.056, 0.040, 0.074)),
        origin=Origin(xyz=(0.0, -0.020, 0.614)),
        material=cast_iron,
        name="rear_upright_web",
    )
    body.visual(
        Box((0.082, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.060, 0.650)),
        material=cast_iron,
        name="front_bridge_web",
    )
    body.visual(
        Box((0.080, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, 0.054, 0.687)),
        material=cast_iron,
        name="screw_boss",
    )
    body.visual(
        Box((0.080, 0.020, 0.080)),
        origin=Origin(xyz=(0.0, -0.006, 0.687)),
        material=cast_iron,
        name="boss_rear_web",
    )
    body.visual(
        Box((0.006, 0.060, 0.080)),
        origin=Origin(xyz=(-0.033, 0.024, 0.687)),
        material=cast_iron,
        name="boss_liner_left",
    )
    body.visual(
        Box((0.006, 0.060, 0.080)),
        origin=Origin(xyz=(0.033, 0.024, 0.687)),
        material=cast_iron,
        name="boss_liner_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.380, 0.300, 0.720)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
    )

    upper_jaw = model.part("upper_jaw")
    upper_jaw.visual(
        Box((0.018, 0.050, 0.110)),
        origin=Origin(xyz=(-0.040, -0.023, -0.015)),
        material=cast_iron,
        name="guide_cheek_left",
    )
    upper_jaw.visual(
        Box((0.018, 0.050, 0.110)),
        origin=Origin(xyz=(0.040, -0.023, -0.015)),
        material=cast_iron,
        name="guide_cheek_right",
    )
    upper_jaw.visual(
        Box((0.064, 0.044, 0.085)),
        origin=Origin(xyz=(0.0, 0.024, -0.005)),
        material=cast_iron,
        name="screw_carrier",
    )
    upper_jaw.visual(
        Cylinder(radius=0.028, length=0.046),
        origin=Origin(xyz=(0.0, 0.024, 0.012)),
        material=cast_iron,
        name="jaw_collar",
    )
    upper_jaw.visual(
        Box((0.110, 0.084, 0.050)),
        origin=Origin(xyz=(0.0, 0.072, -0.010)),
        material=cast_iron,
        name="upper_jaw_block",
    )
    upper_jaw.visual(
        Box((0.088, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, 0.095, -0.026)),
        material=cast_iron,
        name="upper_jaw_nose",
    )
    upper_jaw.visual(
        Box((0.078, 0.044, 0.006)),
        origin=Origin(xyz=(0.0, 0.070, -0.063)),
        material=insert_iron,
        name="upper_insert",
    )
    upper_jaw.visual(
        Box((0.028, 0.044, 0.028)),
        origin=Origin(xyz=(0.0, 0.070, -0.049)),
        material=cast_iron,
        name="upper_insert_backing",
    )
    for index in range(7):
        tooth_name = "upper_insert_tooth_mid" if index == 3 else None
        upper_jaw.visual(
            Box((0.0075, 0.044, 0.006)),
            origin=Origin(xyz=(-0.030 + 0.010 * index, 0.070, -0.069)),
            material=insert_iron,
            name=tooth_name,
        )
    upper_jaw.visual(
        Cylinder(radius=0.018, length=0.446),
        origin=Origin(xyz=(0.0, 0.024, 0.199)),
        material=screw_steel,
        name="screw_core",
    )
    for index in range(24):
        t = index / 23.0
        angle = math.tau * 5.2 * t
        radius = 0.014
        upper_jaw.visual(
            Box((0.022, 0.012, 0.010)),
            origin=Origin(
                xyz=(
                    radius * math.cos(angle),
                    0.024 + radius * math.sin(angle),
                    0.050 + 0.300 * t,
                ),
                rpy=(0.0, 0.0, angle + math.pi * 0.5),
            ),
            material=machined_steel,
        )
    upper_jaw.visual(
        Cylinder(radius=0.008, length=0.180),
        origin=Origin(xyz=(0.0, 0.024, 0.420), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=handle_dark,
        name="cross_handle",
    )
    upper_jaw.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.090, 0.024, 0.420)),
        material=handle_dark,
        name="handle_end_left",
    )
    upper_jaw.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.090, 0.024, 0.420)),
        material=handle_dark,
        name="handle_end_right",
    )
    upper_jaw.inertial = Inertial.from_geometry(
        Box((0.180, 0.160, 0.550)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.025, 0.170)),
    )

    model.articulation(
        "upper_jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=upper_jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=0.155,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    upper_jaw = object_model.get_part("upper_jaw")
    jaw_slide = object_model.get_articulation("upper_jaw_slide")

    lower_jaw_casting = body.get_visual("lower_jaw_casting")
    lower_insert = body.get_visual("lower_insert")
    lower_insert_tooth_mid = body.get_visual("lower_insert_tooth_mid")
    guide_rail = body.get_visual("guide_rail")
    screw_boss = body.get_visual("screw_boss")
    boss_rear_web = body.get_visual("boss_rear_web")
    boss_liner_left = body.get_visual("boss_liner_left")
    boss_liner_right = body.get_visual("boss_liner_right")

    upper_jaw_block = upper_jaw.get_visual("upper_jaw_block")
    upper_insert = upper_jaw.get_visual("upper_insert")
    upper_insert_tooth_mid = upper_jaw.get_visual("upper_insert_tooth_mid")
    guide_cheek_left = upper_jaw.get_visual("guide_cheek_left")
    guide_cheek_right = upper_jaw.get_visual("guide_cheek_right")
    screw_core = upper_jaw.get_visual("screw_core")
    cross_handle = upper_jaw.get_visual("cross_handle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(body, body, axes="xy", inner_elem=lower_insert, outer_elem=lower_jaw_casting)
    ctx.expect_within(
        upper_jaw,
        upper_jaw,
        axes="xy",
        inner_elem=upper_insert,
        outer_elem=upper_jaw_block,
    )
    ctx.expect_overlap(
        body,
        body,
        axes="xy",
        elem_a=lower_insert_tooth_mid,
        elem_b=lower_insert,
        min_overlap=0.002,
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        positive_elem=lower_insert_tooth_mid,
        negative_elem=lower_insert,
        max_gap=0.0,
        max_penetration=0.0,
    )
    ctx.expect_overlap(
        upper_jaw,
        upper_jaw,
        axes="xy",
        elem_a=upper_insert,
        elem_b=upper_insert_tooth_mid,
        min_overlap=0.002,
    )
    ctx.expect_gap(
        upper_jaw,
        upper_jaw,
        axis="z",
        positive_elem=upper_insert,
        negative_elem=upper_insert_tooth_mid,
        max_gap=0.0,
        max_penetration=0.0,
    )
    ctx.expect_overlap(upper_jaw, body, axes="xy", elem_a=upper_insert, elem_b=lower_insert, min_overlap=0.002)
    ctx.expect_gap(
        upper_jaw,
        body,
        axis="z",
        positive_elem=upper_insert,
        negative_elem=lower_insert,
        min_gap=0.150,
        max_gap=0.190,
    )
    ctx.expect_gap(
        upper_jaw,
        body,
        axis="z",
        positive_elem=upper_insert_tooth_mid,
        negative_elem=lower_insert_tooth_mid,
        min_gap=0.145,
        max_gap=0.165,
    )
    ctx.expect_gap(
        body,
        upper_jaw,
        axis="x",
        positive_elem=guide_rail,
        negative_elem=guide_cheek_left,
        min_gap=0.003,
        max_gap=0.006,
    )
    ctx.expect_gap(
        upper_jaw,
        body,
        axis="x",
        positive_elem=guide_cheek_right,
        negative_elem=guide_rail,
        min_gap=0.003,
        max_gap=0.006,
    )
    ctx.expect_gap(
        upper_jaw,
        body,
        axis="x",
        positive_elem=screw_core,
        negative_elem=boss_liner_left,
        min_gap=0.009,
        max_gap=0.015,
    )
    ctx.expect_gap(
        body,
        upper_jaw,
        axis="x",
        positive_elem=boss_liner_right,
        negative_elem=screw_core,
        min_gap=0.009,
        max_gap=0.015,
    )
    ctx.expect_gap(
        body,
        upper_jaw,
        axis="y",
        positive_elem=screw_boss,
        negative_elem=screw_core,
        min_gap=0.001,
        max_gap=0.004,
    )
    ctx.expect_gap(
        upper_jaw,
        body,
        axis="y",
        positive_elem=screw_core,
        negative_elem=boss_rear_web,
        min_gap=0.001,
        max_gap=0.004,
    )
    ctx.expect_gap(
        upper_jaw,
        body,
        axis="z",
        positive_elem=cross_handle,
        negative_elem=screw_boss,
        min_gap=0.220,
    )

    with ctx.pose({jaw_slide: 0.155}):
        ctx.expect_overlap(
            upper_jaw,
            body,
            axes="xy",
            elem_a=upper_insert,
            elem_b=lower_insert,
            min_overlap=0.002,
        )
        ctx.expect_gap(
            upper_jaw,
            body,
            axis="z",
            positive_elem=upper_insert,
            negative_elem=lower_insert,
            max_gap=0.022,
            max_penetration=0.0,
        )
        ctx.expect_gap(
            upper_jaw,
            body,
            axis="z",
            positive_elem=upper_insert_tooth_mid,
            negative_elem=lower_insert_tooth_mid,
            max_gap=0.001,
            max_penetration=0.0,
        )
        ctx.expect_gap(
            upper_jaw,
            body,
            axis="z",
            positive_elem=cross_handle,
            negative_elem=screw_boss,
            min_gap=0.070,
        )
        ctx.expect_gap(
            body,
            upper_jaw,
            axis="x",
            positive_elem=guide_rail,
            negative_elem=guide_cheek_left,
            min_gap=0.003,
            max_gap=0.006,
        )
        ctx.expect_gap(
            upper_jaw,
            body,
            axis="x",
            positive_elem=guide_cheek_right,
            negative_elem=guide_rail,
            min_gap=0.003,
            max_gap=0.006,
        )
        ctx.expect_gap(
            body,
            upper_jaw,
            axis="y",
            positive_elem=screw_boss,
            negative_elem=screw_core,
            min_gap=0.001,
            max_gap=0.004,
        )
        ctx.expect_gap(
            upper_jaw,
            body,
            axis="y",
            positive_elem=screw_core,
            negative_elem=boss_rear_web,
            min_gap=0.001,
            max_gap=0.004,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
