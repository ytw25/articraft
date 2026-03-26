from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

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

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except OSError:
            pass
        return "/"


os.getcwd = _safe_getcwd
try:
    os.chdir("/")
except OSError:
    pass

SAFE_ASSET_ROOT = Path("/")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flatbed_scanner_with_adf")

    body = model.material("body", rgba=(0.84, 0.86, 0.88, 1.0))
    lid_body = model.material("lid_body", rgba=(0.92, 0.93, 0.94, 1.0))
    trim = model.material("trim", rgba=(0.21, 0.23, 0.25, 1.0))
    glass = model.material("glass", rgba=(0.68, 0.82, 0.90, 0.45))
    hinge_metal = model.material("hinge_metal", rgba=(0.42, 0.45, 0.48, 1.0))
    tray = model.material("tray", rgba=(0.79, 0.81, 0.83, 1.0))

    base_w = 0.44
    base_d = 0.30
    base_h = 0.075

    lid_w = 0.438
    lid_d = 0.298
    lid_t = 0.026

    hinge_axis_y = 0.152
    hinge_axis_z = base_h + 0.001

    base = model.part("base")
    base.visual(
        Box((base_w, base_d, base_h)),
        origin=Origin(xyz=(0.0, 0.0, base_h / 2.0)),
        material=body,
        name="base_shell",
    )
    base.visual(
        Box((0.338, 0.244, 0.003)),
        origin=Origin(xyz=(-0.008, -0.008, base_h - 0.0025)),
        material=trim,
        name="platen_recess",
    )
    base.visual(
        Box((0.318, 0.224, 0.004)),
        origin=Origin(xyz=(-0.008, -0.008, base_h - 0.004)),
        material=glass,
        name="glass_platen",
    )
    base.visual(
        Box((0.060, 0.220, 0.003)),
        origin=Origin(xyz=(0.165, -0.008, base_h - 0.0025)),
        material=trim,
        name="control_strip",
    )
    for side, x_pos in (("left", -0.145), ("right", 0.145)):
        base.visual(
            Box((0.074, 0.020, 0.012)),
            origin=Origin(xyz=(x_pos, hinge_axis_y - 0.014, hinge_axis_z)),
            material=hinge_metal,
            name=f"base_hinge_{side}",
        )
        base.visual(
            Cylinder(radius=0.004, length=0.014),
            origin=Origin(
                xyz=(x_pos, hinge_axis_y - 0.014, hinge_axis_z),
            ),
            material=hinge_metal,
            name=f"base_spring_{side}",
        )
    base.inertial = Inertial.from_geometry(
        Box((base_w, base_d, base_h)),
        mass=5.6,
        origin=Origin(xyz=(0.0, 0.0, base_h / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, -0.171, lid_t / 2.0)),
        material=lid_body,
        name="lid_shell",
    )
    lid.visual(
        Box((0.372, 0.112, 0.034)),
        origin=Origin(xyz=(0.0, -0.054, 0.043)),
        material=trim,
        name="adf_housing",
    )
    lid.visual(
        Box((0.344, 0.064, 0.012)),
        origin=Origin(
            xyz=(0.0, -0.126, 0.037),
            rpy=(math.radians(16.0), 0.0, 0.0),
        ),
        material=trim,
        name="adf_ramp",
    )
    lid.visual(
        Box((0.254, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, -0.110, 0.029)),
        material=lid_body,
        name="adf_output_slot",
    )
    lid.visual(
        Box((0.272, 0.094, 0.005)),
        origin=Origin(
            xyz=(0.0, 0.030, 0.056),
            rpy=(math.radians(38.0), 0.0, 0.0),
        ),
        material=tray,
        name="input_tray",
    )
    lid.visual(
        Box((0.272, 0.020, 0.010)),
        origin=Origin(
            xyz=(0.0, 0.076, 0.085),
            rpy=(math.radians(38.0), 0.0, 0.0),
        ),
        material=tray,
        name="input_tip",
    )
    lid.visual(
        Box((0.248, 0.100, 0.005)),
        origin=Origin(
            xyz=(0.0, -0.343, 0.001),
            rpy=(math.radians(28.0), 0.0, 0.0),
        ),
        material=tray,
        name="output_tray",
    )
    lid.visual(
        Box((0.248, 0.014, 0.008)),
        origin=Origin(
            xyz=(0.0, -0.392, -0.020),
            rpy=(math.radians(28.0), 0.0, 0.0),
        ),
        material=tray,
        name="output_tip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, 0.060)),
        mass=2.3,
        origin=Origin(xyz=(0.0, -0.171, 0.030)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.22,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SAFE_ASSET_ROOT)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    base_shell = base.get_visual("base_shell")
    glass_platen = base.get_visual("glass_platen")
    base_hinge_left = base.get_visual("base_hinge_left")
    base_hinge_right = base.get_visual("base_hinge_right")
    base_spring_left = base.get_visual("base_spring_left")
    base_spring_right = base.get_visual("base_spring_right")

    lid_shell = lid.get_visual("lid_shell")
    adf_housing = lid.get_visual("adf_housing")
    input_tip = lid.get_visual("input_tip")
    output_tip = lid.get_visual("output_tip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.025)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        min_overlap=0.10,
        elem_a=lid_shell,
        elem_b=base_shell,
        name="lid_covers_flatbed_base",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem=lid_shell,
        negative_elem=base_shell,
        name="closed_lid_sits_on_scanner_base",
    )
    ctx.expect_within(
        base,
        base,
        axes="xy",
        inner_elem=glass_platen,
        outer_elem=base_shell,
        name="glass_platen_is_nested_inside_base",
    )
    ctx.expect_overlap(
        lid,
        lid,
        axes="x",
        min_overlap=0.25,
        elem_a=adf_housing,
        elem_b=lid_shell,
        name="adf_spans_most_of_lid_width",
    )
    ctx.expect_gap(
        lid,
        lid,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=adf_housing,
        negative_elem=lid_shell,
        name="adf_module_is_seated_on_lid_shell",
    )
    ctx.expect_overlap(
        lid,
        lid,
        axes="y",
        min_overlap=0.08,
        elem_a=adf_housing,
        elem_b=lid_shell,
        name="adf_module_integrates_into_rear_of_lid",
    )
    ctx.expect_contact(
        base,
        base,
        elem_a=base_spring_left,
        elem_b=base_hinge_left,
        name="left_piano_hinge_segment_has_visible_spring_barrel",
    )
    ctx.expect_contact(
        base,
        base,
        elem_a=base_spring_right,
        elem_b=base_hinge_right,
        name="right_piano_hinge_segment_has_visible_spring_barrel",
    )
    ctx.expect_gap(
        base,
        base,
        axis="x",
        min_gap=0.18,
        positive_elem=base_hinge_right,
        negative_elem=base_hinge_left,
        name="dual_hinge_segments_are_spaced_apart_across_lid_width",
    )
    ctx.expect_gap(
        lid,
        lid,
        axis="y",
        min_gap=0.02,
        positive_elem=input_tip,
        negative_elem=lid_shell,
        name="input_tray_extends_behind_lid",
    )
    ctx.expect_gap(
        lid,
        lid,
        axis="z",
        min_gap=0.012,
        positive_elem=input_tip,
        negative_elem=adf_housing,
        name="input_tray_rises_above_adf_body",
    )
    ctx.expect_gap(
        lid,
        lid,
        axis="y",
        min_gap=0.04,
        positive_elem=lid_shell,
        negative_elem=output_tip,
        name="output_tray_projects_forward_of_lid",
    )
    ctx.expect_gap(
        lid,
        lid,
        axis="z",
        min_gap=0.008,
        positive_elem=lid_shell,
        negative_elem=output_tip,
        name="output_tray_drops_below_lid_plane",
    )
    with ctx.pose({lid_hinge: 1.05}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.12,
            positive_elem=output_tip,
            negative_elem=base_shell,
            name="opened_lid_lifts_front_clear_of_base",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
