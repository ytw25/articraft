from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_crane", assets=ASSETS)

    # Materials
    yellow = model.material("construction_yellow", rgba=(0.9, 0.7, 0.1, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.2, 0.2, 0.2, 1.0))
    black = model.material("black", rgba=(0.05, 0.05, 0.05, 1.0))
    steel = model.material("steel", rgba=(0.6, 0.6, 0.6, 1.0))
    glass = model.material("glass", rgba=(0.7, 0.8, 0.9, 0.5))

    # 1. Chassis (Base)
    chassis = model.part("chassis")
    ch_w, ch_l, ch_h = 0.5, 1.2, 0.15
    chassis.visual(Box((ch_l, ch_w, ch_h)), material=yellow, origin=Origin(xyz=(0, 0, ch_h / 2)))

    # Wheels
    wheel_r, wheel_w = 0.12, 0.1
    for i in [-1, 0, 1]:
        for side in [-1, 1]:
            wheel_x = i * 0.4
            wheel_y = side * (ch_w / 2 + wheel_w / 2)
            chassis.visual(
                Cylinder(radius=wheel_r, length=wheel_w),
                material=black,
                origin=Origin(xyz=(wheel_x, wheel_y, wheel_r), rpy=(1.57, 0, 0)),
            )

    # Slewing ring base
    chassis.visual(
        Cylinder(radius=0.2, length=0.05),
        material=dark_grey,
        origin=Origin(xyz=(0, 0, ch_h + 0.025)),
    )

    chassis.inertial = Inertial.from_geometry(Box((ch_l, ch_w + 0.2, ch_h + 0.1)), mass=1000.0)

    # 2. Slewing Platform
    slewing_platform = model.part("slewing_platform")
    sp_w, sp_l, sp_h = 0.55, 0.7, 0.1
    slewing_platform.visual(
        Box((sp_l, sp_w, sp_h)), material=yellow, origin=Origin(xyz=(0.1, 0, sp_h / 2))
    )

    # Slewing ring top
    slewing_platform.visual(
        Cylinder(radius=0.19, length=0.05), material=dark_grey, origin=Origin(xyz=(0, 0, -0.025))
    )

    # Counterweight
    slewing_platform.visual(
        Box((0.2, sp_w, 0.2)), material=dark_grey, origin=Origin(xyz=(-0.15, 0, sp_h + 0.1))
    )

    # Operator Cab
    cab_size = (0.25, 0.15, 0.25)
    slewing_platform.visual(
        Box(cab_size), material=yellow, origin=Origin(xyz=(0.2, 0.18, sp_h + cab_size[2] / 2))
    )
    slewing_platform.visual(
        Box((0.1, 0.14, 0.15)), material=glass, origin=Origin(xyz=(0.28, 0.18, sp_h + 0.15))
    )

    # Boom support brackets
    bracket_h = 0.3
    for side in [-1, 1]:
        slewing_platform.visual(
            Box((0.15, 0.04, bracket_h)),
            material=yellow,
            origin=Origin(xyz=(0.25, side * 0.12, sp_h + bracket_h / 2)),
        )

    # Pivot pin (cross-bar to satisfy articulation origin proximity)
    slewing_platform.visual(
        Cylinder(radius=0.02, length=0.24),
        material=steel,
        origin=Origin(xyz=(0.25, 0, sp_h + bracket_h), rpy=(1.57, 0, 0)),
    )

    slewing_platform.inertial = Inertial.from_geometry(Box((sp_l, sp_w, sp_h + 0.3)), mass=500.0)

    model.articulation(
        "slewing_joint",
        ArticulationType.CONTINUOUS,
        parent="chassis",
        child="slewing_platform",
        origin=Origin(xyz=(0, 0, ch_h + 0.05)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1000.0, velocity=1.0),
    )

    # 3. Boom Base
    boom_base = model.part("boom_base")
    bb_l, bb_w, bb_h = 1.0, 0.15, 0.15
    boom_base.visual(Box((bb_l, bb_w, bb_h)), material=yellow, origin=Origin(xyz=(bb_l / 2, 0, 0)))

    # Piston barrel
    boom_base.visual(
        Cylinder(radius=0.04, length=0.3),
        material=steel,
        origin=Origin(xyz=(0.3, 0, -0.08), rpy=(0, 1.57, 0)),
    )

    boom_base.inertial = Inertial.from_geometry(Box((bb_l, bb_w, bb_h)), mass=200.0)

    model.articulation(
        "boom_pitch",
        ArticulationType.REVOLUTE,
        parent="slewing_platform",
        child="boom_base",
        origin=Origin(xyz=(0.25, 0, sp_h + bracket_h)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(lower=0.0, upper=1.4, effort=5000.0, velocity=0.5),
    )

    # 4. Boom Extension
    boom_ext = model.part("boom_ext")
    be_l, be_w, be_h = 0.9, 0.12, 0.12
    boom_ext.visual(Box((be_l, be_w, be_h)), material=yellow, origin=Origin(xyz=(be_l / 2, 0, 0)))

    # Pulley
    boom_ext.visual(
        Cylinder(radius=0.06, length=0.05),
        material=dark_grey,
        origin=Origin(xyz=(be_l, 0, 0), rpy=(1.57, 0, 0)),
    )

    boom_ext.inertial = Inertial.from_geometry(Box((be_l, be_w, be_h)), mass=100.0)

    model.articulation(
        "boom_extension",
        ArticulationType.PRISMATIC,
        parent="boom_base",
        child="boom_ext",
        origin=Origin(xyz=(0.1, 0, 0)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(lower=0.0, upper=0.7, effort=2000.0, velocity=0.5),
    )

    # 5. Hook Block
    hook_block = model.part("hook_block")
    hook_block.visual(Box((0.1, 0.08, 0.15)), material=dark_grey, origin=Origin(xyz=(0, 0, -0.075)))

    # Cable visual
    hook_block.visual(
        Cylinder(radius=0.005, length=1.5), material=black, origin=Origin(xyz=(0, 0, 0.75))
    )

    # Hook mesh
    hook_geom = tube_from_spline_points(
        [(0, 0, -0.15), (0, 0, -0.22), (0.05, 0, -0.25), (0.08, 0, -0.20)],
        radius=0.015,
        cap_ends=True,
    )
    hook_mesh = mesh_from_geometry(hook_geom, ASSETS.mesh_path("hook.obj"))
    hook_block.visual(hook_mesh, material=steel)

    hook_block.inertial = Inertial.from_geometry(Box((0.1, 0.1, 0.2)), mass=20.0)

    model.articulation(
        "hoist",
        ArticulationType.PRISMATIC,
        parent="boom_ext",
        child="hook_block",
        origin=Origin(
            xyz=(be_l + 0.06, 0, -0.05)
        ),  # Moved slightly further to avoid rest-pose overlap
        axis=(0, 0, -1),
        motion_limits=MotionLimits(lower=0.0, upper=2.0, effort=1000.0, velocity=1.0),
    )

    return model


def run_tests() -> TestReport:
    model = build_object_model()
    ctx = TestContext(model, asset_root=HERE, geometry_source="collision")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.02)
    ctx.check_part_geometry_connected(use="visual")

    # Allowances
    ctx.allow_overlap("boom_base", "slewing_platform", reason="pivot mount and bracket proximity")
    ctx.allow_overlap("boom_base", "boom_ext", reason="telescoping fit")
    ctx.allow_overlap("chassis", "slewing_platform", reason="slewing ring assembly")
    ctx.allow_overlap("hook_block", "boom_ext", reason="hoist cable and pulley")
    ctx.allow_overlap("hook_block", "boom_base", reason="hoist cable overlaps boom when retracted")
    ctx.allow_overlap("boom_base", "chassis", reason="boom tail/piston clearance at high pitch")

    ctx.check_no_overlaps(
        max_pose_samples=64,
        overlap_tol=0.01,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    # Intent checks
    ctx.expect_joint_motion_axis(
        "boom_pitch", "boom_base", world_axis="z", direction="positive", min_delta=0.1
    )
    ctx.expect_joint_motion_axis(
        "boom_extension", "boom_ext", world_axis="x", direction="positive", min_delta=0.1
    )
    ctx.expect_joint_motion_axis(
        "hoist", "hook_block", world_axis="z", direction="negative", min_delta=0.1
    )

    return ctx.report()


if __name__ == "__main__":
    object_model = build_object_model()
    report = run_tests()
    print(report.summary())
# >>> USER_CODE_END

object_model = build_object_model()
