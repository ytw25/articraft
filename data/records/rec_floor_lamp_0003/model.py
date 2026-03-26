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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arc_floor_lamp", assets=ASSETS)

    stone = model.material("stone", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.16, 0.16, 0.17, 1.0))
    aged_brass = model.material("aged_brass", rgba=(0.67, 0.56, 0.32, 1.0))
    satin_brass = model.material("satin_brass", rgba=(0.77, 0.67, 0.41, 1.0))
    linen = model.material("linen", rgba=(0.88, 0.85, 0.77, 1.0))
    shadow = model.material("shadow", rgba=(0.25, 0.23, 0.20, 1.0))

    arc_pole_mesh = _save_mesh(
        "arc_pole.obj",
        tube_from_spline_points(
            [
                (0.00, 0.00, 0.58),
                (0.05, 0.00, 0.84),
                (0.18, 0.00, 1.18),
                (0.40, 0.00, 1.50),
                (0.69, 0.00, 1.77),
                (1.00, 0.00, 1.93),
                (1.24, 0.00, 1.92),
            ],
            radius=0.021,
            samples_per_segment=20,
            radial_segments=20,
            cap_ends=True,
        ),
    )
    hook_fitting_mesh = _save_mesh(
        "hook_fitting.obj",
        tube_from_spline_points(
            [
                (1.325, 0.0, 1.918),
                (1.333, 0.0, 1.875),
                (1.333, 0.0, 1.828),
                (1.320, 0.0, 1.794),
                (1.292, 0.0, 1.778),
            ],
            radius=0.007,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    shade_outer = CylinderGeometry(radius=0.165, height=0.40, radial_segments=56)
    shade_inner = CylinderGeometry(radius=0.145, height=0.40, radial_segments=56).translate(
        0.0,
        0.0,
        -0.018,
    )
    top_hole = CylinderGeometry(radius=0.020, height=0.080, radial_segments=32).translate(
        0.0,
        0.0,
        0.180,
    )
    shade_shell_mesh = _save_mesh(
        "pendant_shade.obj",
        boolean_difference(boolean_difference(shade_outer, shade_inner), top_hole),
    )

    base = model.part("base")
    base.visual(
        Box((0.78, 0.42, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.0505)),
        material=stone,
        name="base_slab",
    )
    for name, x, y in (
        ("front_left_foot", 0.305, 0.155),
        ("front_right_foot", 0.305, -0.155),
        ("rear_left_foot", -0.305, 0.155),
        ("rear_right_foot", -0.305, -0.155),
    ):
        base.visual(
            Box((0.064, 0.064, 0.018)),
            origin=Origin(xyz=(x, y, 0.009)),
            material=rubber,
            name=name,
        )
    base.visual(
        Cylinder(radius=0.076, length=0.018),
        origin=Origin(xyz=(-0.23, 0.0, 0.092)),
        material=stone,
        name="collar_plate",
    )
    base.visual(
        Box((0.088, 0.018, 0.162)),
        origin=Origin(xyz=(-0.23, 0.039, 0.164)),
        material=aged_brass,
        name="clamp_right",
    )
    base.visual(
        Box((0.088, 0.018, 0.162)),
        origin=Origin(xyz=(-0.23, -0.039, 0.164)),
        material=aged_brass,
        name="clamp_left",
    )
    base.visual(
        Box((0.022, 0.060, 0.162)),
        origin=Origin(xyz=(-0.263, 0.0, 0.164)),
        material=aged_brass,
        name="clamp_back",
    )
    base.visual(
        Box((0.032, 0.030, 0.026)),
        origin=Origin(xyz=(-0.176, 0.028, 0.233)),
        material=satin_brass,
        name="clamp_cap_right",
    )
    base.visual(
        Box((0.032, 0.030, 0.026)),
        origin=Origin(xyz=(-0.176, -0.028, 0.233)),
        material=satin_brass,
        name="clamp_cap_left",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.052),
        origin=Origin(xyz=(-0.188, 0.067, 0.187), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_brass,
        name="collar_knob",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.78, 0.42, 0.25)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
    )

    arc_arm = model.part("arc_arm")
    arc_arm.visual(
        Cylinder(radius=0.024, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        material=aged_brass,
        name="lower_stem",
    )
    arc_arm.visual(
        arc_pole_mesh,
        material=aged_brass,
        name="arc_pole",
    )
    arc_arm.visual(
        Cylinder(radius=0.022, length=0.058),
        origin=Origin(xyz=(1.228, 0.0, 1.920), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_brass,
        name="tip_neck",
    )
    arc_arm.visual(
        Cylinder(radius=0.028, length=0.082),
        origin=Origin(xyz=(1.284, 0.0, 1.918), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_brass,
        name="tip_socket",
    )
    arc_arm.visual(
        Cylinder(radius=0.0075, length=0.060),
        origin=Origin(xyz=(1.329, 0.0, 1.889)),
        material=satin_brass,
        name="hook_post",
    )
    arc_arm.visual(
        hook_fitting_mesh,
        material=satin_brass,
        name="hook_fitting",
    )
    arc_arm.inertial = Inertial.from_geometry(
        Box((1.40, 0.08, 2.04)),
        mass=4.8,
        origin=Origin(xyz=(0.70, 0.0, 1.02)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.0045, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, -0.065)),
        material=satin_brass,
        name="hanger_cable",
    )
    shade.visual(
        shade_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.325)),
        material=linen,
        name="pendant_shade",
    )
    shade.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.128)),
        material=satin_brass,
        name="shade_top_cap",
    )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.54),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.27)),
    )

    model.articulation(
        "height_adjust",
        ArticulationType.PRISMATIC,
        parent=base,
        child=arc_arm,
        origin=Origin(xyz=(-0.23, 0.0, 0.101)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.25, lower=0.0, upper=0.14),
    )
    model.articulation(
        "shade_mount",
        ArticulationType.FIXED,
        parent=arc_arm,
        child=shade,
        origin=Origin(xyz=(1.292, 0.0, 1.778)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arc_arm = object_model.get_part("arc_arm")
    shade = object_model.get_part("shade")
    height_adjust = object_model.get_articulation("height_adjust")

    base_slab = base.get_visual("base_slab")
    front_left_foot = base.get_visual("front_left_foot")
    front_right_foot = base.get_visual("front_right_foot")
    rear_left_foot = base.get_visual("rear_left_foot")
    rear_right_foot = base.get_visual("rear_right_foot")
    collar_plate = base.get_visual("collar_plate")
    clamp_left = base.get_visual("clamp_left")
    clamp_right = base.get_visual("clamp_right")
    lower_stem = arc_arm.get_visual("lower_stem")
    tip_socket = arc_arm.get_visual("tip_socket")
    hook_fitting = arc_arm.get_visual("hook_fitting")
    hanger_cable = shade.get_visual("hanger_cable")
    pendant_shade = shade.get_visual("pendant_shade")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    for foot in (front_left_foot, front_right_foot, rear_left_foot, rear_right_foot):
        ctx.expect_within(base, base, axes="xy", inner_elem=foot, outer_elem=base_slab)
        ctx.expect_gap(
            base,
            base,
            axis="z",
            positive_elem=base_slab,
            negative_elem=foot,
            max_gap=0.001,
            max_penetration=0.0,
        )

    ctx.expect_within(arc_arm, base, axes="xy", inner_elem=lower_stem, outer_elem=collar_plate)
    ctx.expect_gap(
        arc_arm,
        base,
        axis="z",
        positive_elem=lower_stem,
        negative_elem=collar_plate,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        base,
        arc_arm,
        axis="y",
        positive_elem=clamp_right,
        negative_elem=lower_stem,
        max_gap=0.012,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        arc_arm,
        base,
        axis="y",
        positive_elem=lower_stem,
        negative_elem=clamp_left,
        max_gap=0.012,
        max_penetration=0.0,
    )
    ctx.expect_contact(arc_arm, shade, elem_a=hook_fitting, elem_b=hanger_cable)
    ctx.expect_overlap(arc_arm, shade, axes="xy", elem_a=hook_fitting, elem_b=hanger_cable, min_overlap=0.004)
    ctx.expect_gap(
        arc_arm,
        shade,
        axis="z",
        positive_elem=hook_fitting,
        negative_elem=pendant_shade,
        min_gap=0.11,
    )
    ctx.expect_gap(
        arc_arm,
        base,
        axis="x",
        positive_elem=tip_socket,
        negative_elem=base_slab,
        min_gap=0.55,
    )
    ctx.expect_gap(
        shade,
        base,
        axis="x",
        positive_elem=pendant_shade,
        negative_elem=base_slab,
        min_gap=0.45,
    )
    ctx.expect_gap(
        shade,
        base,
        axis="z",
        positive_elem=pendant_shade,
        negative_elem=base_slab,
        min_gap=1.12,
    )

    with ctx.pose({height_adjust: 0.14}):
        ctx.expect_within(arc_arm, base, axes="xy", inner_elem=lower_stem, outer_elem=collar_plate)
        ctx.expect_gap(
            arc_arm,
            base,
            axis="z",
            positive_elem=lower_stem,
            negative_elem=collar_plate,
            min_gap=0.13,
        )
        ctx.expect_gap(
            base,
            arc_arm,
            axis="y",
            positive_elem=clamp_right,
            negative_elem=lower_stem,
            max_gap=0.012,
            max_penetration=0.0,
        )
        ctx.expect_gap(
            arc_arm,
            base,
            axis="y",
            positive_elem=lower_stem,
            negative_elem=clamp_left,
            max_gap=0.012,
            max_penetration=0.0,
        )
        ctx.expect_contact(arc_arm, shade, elem_a=hook_fitting, elem_b=hanger_cable)
        ctx.expect_gap(
            shade,
            base,
            axis="x",
            positive_elem=pendant_shade,
            negative_elem=base_slab,
            min_gap=0.45,
        )
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            positive_elem=pendant_shade,
            negative_elem=base_slab,
            min_gap=1.26,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
