from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _add_xz_cylinder(part, start, end, radius, material) -> None:
    dx = end[0] - start[0]
    dz = end[2] - start[2]
    length = math.hypot(dx, dz)
    if length <= 1e-6:
        return
    mid = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    angle = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=(0.0, angle, 0.0)),
        material=material,
    )


def _add_y_cylinder(part, center, radius, length, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
    )


def _build_hook_mesh():
    hook_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, -0.03),
            (0.014, 0.0, -0.07),
            (0.040, 0.0, -0.108),
            (0.064, 0.0, -0.080),
        ],
        radius=0.006,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(hook_geom, ASSETS.mesh_path("crane_hook.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_crane", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.63, 0.64, 0.66, 1.0))
    crane_yellow = model.material("crane_yellow", rgba=(0.92, 0.72, 0.16, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    cable_black = model.material("cable_black", rgba=(0.09, 0.09, 0.10, 1.0))
    glass = model.material("glass", rgba=(0.73, 0.84, 0.90, 0.42))
    warning_red = model.material("warning_red", rgba=(0.63, 0.12, 0.12, 1.0))

    base = model.part("base_frame")
    base.visual(Box((1.40, 1.20, 0.18)), origin=Origin(xyz=(0.0, 0.0, 0.09)), material=concrete)
    base.visual(Box((1.70, 0.18, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.24)), material=dark_steel)
    base.visual(Box((0.18, 1.50, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.24)), material=dark_steel)
    for x in (-0.64, 0.64):
        for y in (-0.48, 0.48):
            base.visual(
                Box((0.24, 0.24, 0.12)), origin=Origin(xyz=(x, y, 0.24)), material=dark_steel
            )
    base.visual(Box((0.42, 0.42, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.35)), material=steel)
    base.visual(Box((0.30, 0.08, 0.06)), origin=Origin(xyz=(0.38, 0.0, 0.27)), material=warning_red)
    base.inertial = Inertial.from_geometry(
        Box((1.40, 1.20, 0.18)),
        mass=6200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.19, length=0.14), origin=Origin(xyz=(0.0, 0.0, 0.07)), material=steel
    )
    mast.visual(
        Cylinder(radius=0.13, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        material=crane_yellow,
    )
    mast.visual(
        Cylinder(radius=0.18, length=0.12), origin=Origin(xyz=(0.0, 0.0, 1.08)), material=steel
    )
    mast.visual(Box((0.10, 0.04, 0.32)), origin=Origin(xyz=(-0.17, 0.0, 0.34)), material=dark_steel)
    mast.inertial = Inertial.from_geometry(
        Cylinder(radius=0.13, length=0.88),
        mass=1400.0,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
    )

    upper = model.part("upper")
    upper.visual(
        Cylinder(radius=0.30, length=0.10), origin=Origin(xyz=(0.0, 0.0, 0.05)), material=steel
    )
    upper.visual(
        Box((0.84, 0.54, 0.12)), origin=Origin(xyz=(0.02, 0.0, 0.16)), material=crane_yellow
    )
    upper.visual(
        Box((0.56, 0.34, 0.34)), origin=Origin(xyz=(-0.04, 0.08, 0.39)), material=dark_steel
    )
    upper.visual(Box((0.28, 0.46, 0.22)), origin=Origin(xyz=(-0.42, 0.02, 0.25)), material=steel)
    upper.visual(
        Box((0.26, 0.24, 0.18)), origin=Origin(xyz=(0.24, -0.22, 0.31)), material=dark_steel
    )
    upper.visual(
        Box((0.02, 0.22, 0.16)),
        origin=Origin(xyz=(0.37, -0.22, 0.31), rpy=(0.0, -0.35, 0.0)),
        material=glass,
    )
    upper.visual(
        Box((0.18, 0.02, 0.14)),
        origin=Origin(xyz=(0.25, -0.33, 0.31), rpy=(0.0, -0.12, 0.0)),
        material=glass,
    )
    upper.visual(Box((0.08, 0.05, 0.18)), origin=Origin(xyz=(0.32, 0.10, 0.31)), material=steel)
    upper.visual(Box((0.08, 0.05, 0.18)), origin=Origin(xyz=(0.32, -0.10, 0.31)), material=steel)
    _add_y_cylinder(upper, center=(0.32, 0.0, 0.31), radius=0.032, length=0.24, material=dark_steel)
    _add_y_cylinder(upper, center=(0.08, 0.0, 0.28), radius=0.07, length=0.18, material=dark_steel)
    _add_xz_cylinder(upper, (-0.08, 0.12, 0.22), (0.08, 0.12, 0.56), 0.022, steel)
    _add_xz_cylinder(upper, (-0.08, -0.12, 0.22), (0.08, -0.12, 0.56), 0.022, steel)
    upper.visual(Box((0.12, 0.34, 0.06)), origin=Origin(xyz=(0.08, 0.0, 0.56)), material=steel)
    upper.visual(
        Cylinder(radius=0.025, length=0.16), origin=Origin(xyz=(-0.18, 0.21, 0.64)), material=steel
    )
    upper.inertial = Inertial.from_geometry(
        Box((0.84, 0.54, 0.12)),
        mass=2100.0,
        origin=Origin(xyz=(0.02, 0.0, 0.16)),
    )

    boom = model.part("boom")
    boom.visual(Box((0.18, 0.10, 0.14)), origin=Origin(xyz=(0.09, 0.0, 0.0)), material=steel)
    boom.visual(Box((0.16, 0.04, 0.18)), origin=Origin(xyz=(0.08, 0.085, 0.0)), material=steel)
    boom.visual(Box((0.16, 0.04, 0.18)), origin=Origin(xyz=(0.08, -0.085, 0.0)), material=steel)
    boom.visual(Box((1.18, 0.14, 0.16)), origin=Origin(xyz=(0.70, 0.0, 0.0)), material=crane_yellow)
    boom.visual(Box((0.98, 0.10, 0.12)), origin=Origin(xyz=(1.62, 0.0, 0.0)), material=crane_yellow)
    boom.visual(Box((0.16, 0.14, 0.18)), origin=Origin(xyz=(2.12, 0.0, 0.0)), material=steel)
    boom.visual(Box((1.10, 0.08, 0.05)), origin=Origin(xyz=(1.26, 0.0, 0.11)), material=steel)
    _add_xz_cylinder(boom, (0.20, 0.055, 0.08), (1.96, 0.055, 0.12), 0.018, steel)
    _add_xz_cylinder(boom, (0.20, -0.055, 0.08), (1.96, -0.055, 0.12), 0.018, steel)
    _add_xz_cylinder(boom, (0.34, 0.055, -0.06), (1.72, 0.055, 0.05), 0.012, steel)
    _add_xz_cylinder(boom, (0.34, -0.055, -0.06), (1.72, -0.055, 0.05), 0.012, steel)
    _add_y_cylinder(boom, center=(2.10, 0.0, 0.05), radius=0.035, length=0.12, material=dark_steel)
    _add_y_cylinder(boom, center=(2.05, 0.0, -0.05), radius=0.028, length=0.12, material=dark_steel)
    boom.inertial = Inertial.from_geometry(
        Box((1.80, 0.14, 0.16)),
        mass=950.0,
        origin=Origin(xyz=(0.95, 0.0, 0.0)),
    )

    hoist = model.part("hoist")
    hook_mesh = _build_hook_mesh()
    hoist.visual(Box((0.12, 0.10, 0.06)), origin=Origin(xyz=(0.0, 0.0, -0.03)), material=steel)
    hoist.visual(
        Cylinder(radius=0.004, length=0.72),
        origin=Origin(xyz=(0.018, 0.0, -0.42)),
        material=cable_black,
    )
    hoist.visual(
        Cylinder(radius=0.004, length=0.72),
        origin=Origin(xyz=(-0.018, 0.0, -0.42)),
        material=cable_black,
    )
    _add_y_cylinder(hoist, center=(0.0, 0.0, -0.03), radius=0.025, length=0.08, material=dark_steel)
    hoist.visual(
        Box((0.10, 0.09, 0.18)), origin=Origin(xyz=(0.0, 0.0, -0.87)), material=warning_red
    )
    _add_y_cylinder(hoist, center=(0.0, 0.0, -0.81), radius=0.030, length=0.08, material=dark_steel)
    hoist.visual(hook_mesh, origin=Origin(xyz=(0.0, 0.0, -0.96)), material=steel)
    hoist.inertial = Inertial.from_geometry(
        Box((0.10, 0.09, 0.18)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, -0.87)),
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.FIXED,
        parent="base_frame",
        child="mast",
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
    )
    model.articulation(
        "mast_slew",
        ArticulationType.CONTINUOUS,
        parent="mast",
        child="upper",
        origin=Origin(xyz=(0.0, 0.0, 1.14)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14000.0, velocity=0.65),
    )
    model.articulation(
        "boom_luff",
        ArticulationType.REVOLUTE,
        parent="upper",
        child="boom",
        origin=Origin(xyz=(0.32, 0.0, 0.31)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=9000.0, velocity=0.8, lower=0.0, upper=1.05),
    )
    model.articulation(
        "boom_to_hoist",
        ArticulationType.FIXED,
        parent="boom",
        child="hoist",
        origin=Origin(xyz=(2.10, 0.0, 0.02)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.allow_overlap(
        "upper", "boom", reason="boom heel sits nested between upper hinge cheek plates"
    )
    ctx.check_no_overlaps(
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("mast", "base_frame", axes="xy", max_dist=0.01)
    ctx.expect_origin_distance("upper", "mast", axes="xy", max_dist=0.01)
    ctx.expect_aabb_gap("mast", "base_frame", axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_aabb_overlap("mast", "base_frame", axes="xy", min_overlap=0.26)
    ctx.expect_aabb_contact("mast", "upper")
    ctx.expect_aabb_overlap("upper", "mast", axes="xy", min_overlap=0.24)
    ctx.expect_aabb_overlap("upper", "base_frame", axes="xy", min_overlap=0.40)
    ctx.expect_aabb_contact("upper", "boom")
    ctx.expect_aabb_contact("boom", "hoist")
    ctx.expect_joint_motion_axis(
        "boom_luff", "boom", world_axis="z", direction="positive", min_delta=0.30
    )

    with ctx.pose(boom_luff=1.05):
        ctx.expect_aabb_contact("upper", "boom")
        ctx.expect_aabb_contact("boom", "hoist")
        ctx.expect_aabb_gap("hoist", "base_frame", axis="z", max_gap=3.5, max_penetration=0.0)

    with ctx.pose(mast_slew=math.pi / 2.0):
        ctx.expect_origin_distance("upper", "mast", axes="xy", max_dist=0.01)
        ctx.expect_aabb_contact("mast", "upper")
        ctx.expect_aabb_overlap("upper", "base_frame", axes="xy", min_overlap=0.40)

    with ctx.pose(mast_slew=math.pi / 2.0, boom_luff=0.75):
        ctx.expect_aabb_contact("boom", "hoist")
        ctx.expect_aabb_gap("hoist", "base_frame", axis="z", max_gap=3.5, max_penetration=0.0)
        ctx.expect_aabb_gap("hoist", "mast", axis="y", max_gap=2.5, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
