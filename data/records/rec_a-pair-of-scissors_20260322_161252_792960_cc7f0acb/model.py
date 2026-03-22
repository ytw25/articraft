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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scissors", assets=ASSETS)

    steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    handle_plastic = model.material("handle_plastic", rgba=(0.13, 0.15, 0.18, 1.0))
    screw_finish = model.material("pivot_screw", rgba=(0.84, 0.86, 0.89, 1.0))
    screw_slot = model.material("screw_slot", rgba=(0.20, 0.22, 0.24, 1.0))

    mesh_dir = ASSETS.mesh_dir

    blade_profile = [
        (-0.060, 0.0045),
        (-0.026, 0.0082),
        (0.010, 0.0090),
        (0.064, 0.0072),
        (0.116, 0.0048),
        (0.158, 0.0008),
        (0.126, -0.0028),
        (0.074, -0.0042),
        (0.016, -0.0067),
        (-0.024, -0.0090),
        (-0.052, -0.0082),
        (-0.062, -0.0036),
    ]
    blade_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(blade_profile, 0.0020),
        mesh_dir / "scissor_blade.obj",
    )

    finger_loop_points = [
        (0.000, 0.017, 0.0000),
        (-0.018, 0.030, -0.0004),
        (-0.052, 0.034, -0.0008),
        (-0.085, 0.020, -0.0011),
        (-0.096, 0.000, -0.0012),
        (-0.085, -0.020, -0.0011),
        (-0.052, -0.034, -0.0008),
        (-0.018, -0.030, -0.0004),
        (0.000, -0.017, 0.0000),
    ]
    finger_loop_mesh = mesh_from_geometry(
        tube_from_spline_points(
            finger_loop_points,
            radius=0.0046,
            samples_per_segment=18,
            closed_spline=True,
            radial_segments=18,
        ),
        mesh_dir / "scissor_finger_loop.obj",
    )

    thumb_loop_points = [
        (0.000, 0.012, 0.0000),
        (-0.015, 0.022, 0.0003),
        (-0.038, 0.026, 0.0007),
        (-0.061, 0.015, 0.0010),
        (-0.070, 0.000, 0.0011),
        (-0.061, -0.015, 0.0010),
        (-0.038, -0.026, 0.0007),
        (-0.015, -0.022, 0.0003),
        (0.000, -0.012, 0.0000),
    ]
    thumb_loop_mesh = mesh_from_geometry(
        tube_from_spline_points(
            thumb_loop_points,
            radius=0.0041,
            samples_per_segment=18,
            closed_spline=True,
            radial_segments=18,
        ),
        mesh_dir / "scissor_thumb_loop.obj",
    )

    lower_half = model.part("lower_half")
    lower_half.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, -0.0012), rpy=(0.0, 0.0, -0.14)),
        material=steel,
        name="blade",
    )
    lower_half.visual(
        finger_loop_mesh,
        origin=Origin(xyz=(-0.014, 0.054, -0.0060), rpy=(0.0, 0.0, -0.14)),
        material=handle_plastic,
        name="finger_loop",
    )
    lower_half.visual(
        Box((0.062, 0.014, 0.006)),
        origin=Origin(xyz=(-0.032, 0.030, -0.0048), rpy=(0.0, 0.0, 0.34)),
        material=handle_plastic,
        name="finger_shank",
    )
    lower_half.visual(
        Cylinder(radius=0.0090, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, -0.0010)),
        material=steel,
        name="pivot_boss",
    )
    lower_half.inertial = Inertial.from_geometry(
        Box((0.23, 0.11, 0.014)),
        mass=0.085,
        origin=Origin(xyz=(-0.010, 0.010, -0.0030)),
    )

    upper_half = model.part("upper_half")
    upper_half.visual(
        blade_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0012), rpy=(0.0, 0.0, 0.14)),
        material=steel,
        name="blade",
    )
    upper_half.visual(
        thumb_loop_mesh,
        origin=Origin(xyz=(-0.012, -0.044, 0.0060), rpy=(0.0, 0.0, 0.12)),
        material=handle_plastic,
        name="thumb_loop",
    )
    upper_half.visual(
        Box((0.050, 0.012, 0.006)),
        origin=Origin(xyz=(-0.026, -0.026, 0.0048), rpy=(0.0, 0.0, -0.38)),
        material=handle_plastic,
        name="thumb_shank",
    )
    upper_half.visual(
        Cylinder(radius=0.0090, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=steel,
        name="pivot_boss",
    )
    upper_half.visual(
        Cylinder(radius=0.0060, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.0028)),
        material=screw_finish,
        name="pivot_screw_head",
    )
    upper_half.visual(
        Box((0.0060, 0.0010, 0.0005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0037), rpy=(0.0, 0.0, math.radians(18.0))),
        material=screw_slot,
        name="pivot_screw_slot",
    )
    upper_half.inertial = Inertial.from_geometry(
        Box((0.21, 0.09, 0.014)),
        mass=0.075,
        origin=Origin(xyz=(-0.006, -0.010, 0.0030)),
    )

    model.articulation(
        "blade_pivot",
        ArticulationType.REVOLUTE,
        parent="lower_half",
        child="upper_half",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=-0.35,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
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

    # Add narrow allowances here when conservative QC reports acceptable cases.
    # Add prompt-specific expect_* semantic checks below; they are the main regressions.
    ctx.expect_aabb_gap(
        "upper_half",
        "lower_half",
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="pivot_boss",
        negative_elem="pivot_boss",
        name="pivot_faces_meet_cleanly",
    )
    ctx.expect_aabb_gap(
        "upper_half",
        "lower_half",
        axis="z",
        max_gap=0.0030,
        max_penetration=0.0,
        positive_elem="blade",
        negative_elem="blade",
        name="blades_stay_stacked_not_intersecting",
    )
    ctx.expect_aabb_overlap(
        "upper_half",
        "lower_half",
        axes="xy",
        min_overlap=0.018,
    )

    with ctx.pose(blade_pivot=0.75):
        ctx.expect_aabb_gap(
            "lower_half",
            "upper_half",
            axis="y",
            min_gap=0.020,
            positive_elem="finger_shank",
            negative_elem="thumb_shank",
            name="open_handles_have_clear_grip_span",
        )
        ctx.expect_aabb_gap(
            "upper_half",
            "lower_half",
            axis="z",
            max_gap=0.0030,
            max_penetration=0.0,
            positive_elem="blade",
            negative_elem="blade",
            name="open_blades_remain_layered",
        )

    with ctx.pose(blade_pivot=-0.25):
        ctx.expect_aabb_gap(
            "upper_half",
            "lower_half",
            axis="z",
            max_gap=0.0030,
            max_penetration=0.0,
            positive_elem="blade",
            negative_elem="blade",
            name="closed_blades_can_nearly_meet_without_clipping",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
