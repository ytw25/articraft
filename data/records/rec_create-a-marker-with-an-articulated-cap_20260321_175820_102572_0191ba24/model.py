from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BARREL_RADIUS = 0.0077
BARREL_FRONT_Z = 0.043
BARREL_REAR_Z = -0.058
SEAL_COLLAR_RADIUS = 0.00835
SEAL_COLLAR_LENGTH = 0.006
TIP_JOINT_Z = 0.043
CAP_JOINT_Z = 0.037
CAP_TRAVEL = 0.085


def _mesh_file(name: str) -> str:
    return str(ASSETS.mesh_dir / name)


def _rect_loop(cx: float, cy: float, sx: float, sy: float, z: float) -> list[tuple[float, float, float]]:
    hx = sx * 0.5
    hy = sy * 0.5
    return [
        (cx - hx, cy - hy, z),
        (cx + hx, cy - hy, z),
        (cx + hx, cy + hy, z),
        (cx - hx, cy + hy, z),
    ]


def _build_tip_holder_mesh():
    profile = [
        (0.0, 0.0),
        (0.0069, 0.0),
        (0.0068, 0.003),
        (0.0059, 0.008),
        (0.0047, 0.013),
        (0.0035, 0.0175),
        (0.0027, 0.0205),
        (0.0, 0.0215),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=36), _mesh_file("marker_tip_holder.obj"))


def _build_nib_mesh():
    profile = [
        (0.0, 0.0),
        (0.0022, 0.0),
        (0.0020, 0.007),
        (0.0017, 0.0125),
        (0.0013, 0.0155),
        (0.0, 0.017),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=28), _mesh_file("marker_nib.obj"))


def _build_cap_shell_mesh():
    profile = [
        (0.00825, 0.0),
        (0.0096, 0.0),
        (0.0095, 0.009),
        (0.0092, 0.047),
        (0.0087, 0.061),
        (0.0072, 0.0705),
        (0.0, 0.074),
        (0.0031, 0.0695),
        (0.0062, 0.066),
        (0.00805, 0.051),
        (0.00825, 0.012),
        (0.00825, 0.0),
    ]
    return mesh_from_geometry(LatheGeometry(profile, segments=48), _mesh_file("marker_cap_shell.obj"))


def _build_cap_clip_mesh():
    sections = [
        _rect_loop(0.0, 0.0096, 0.0034, 0.0014, 0.009),
        _rect_loop(0.0, 0.0102, 0.0045, 0.0018, 0.031),
        _rect_loop(0.0, 0.0100, 0.0040, 0.0017, 0.054),
        _rect_loop(0.0, 0.0092, 0.0022, 0.0014, 0.068),
    ]
    clip = section_loft(SectionLoftSpec(sections=tuple(sections), cap=True, solid=True, repair="auto"))
    return mesh_from_geometry(clip, _mesh_file("marker_cap_clip.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="whiteboard_marker", assets=ASSETS)

    white_plastic = model.material("white_plastic", rgba=(0.97, 0.97, 0.95, 1.0))
    green_plastic = model.material("green_plastic", rgba=(0.06, 0.78, 0.58, 1.0))
    dark_green = model.material("dark_green", rgba=(0.05, 0.50, 0.37, 1.0))
    blue_ink = model.material("blue_ink", rgba=(0.20, 0.36, 0.85, 1.0))
    felt_gray = model.material("felt_gray", rgba=(0.44, 0.45, 0.48, 1.0))

    tip_holder_mesh = _build_tip_holder_mesh()
    nib_mesh = _build_nib_mesh()
    cap_shell_mesh = _build_cap_shell_mesh()
    cap_clip_mesh = _build_cap_clip_mesh()

    barrel = model.part("barrel")
    barrel.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_FRONT_Z - BARREL_REAR_Z),
        origin=Origin(xyz=(0.0, 0.0, (BARREL_FRONT_Z + BARREL_REAR_Z) * 0.5)),
        material=white_plastic,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=SEAL_COLLAR_RADIUS, length=SEAL_COLLAR_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, CAP_JOINT_Z + 0.003)),
        material=white_plastic,
        name="front_seal_collar",
    )
    barrel.visual(
        Cylinder(radius=0.00755, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.053)),
        material=green_plastic,
        name="rear_plug",
    )
    barrel.visual(
        Cylinder(radius=0.00762, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=blue_ink,
        name="printed_band",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0080, length=0.118),
        mass=0.018,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
    )

    tip = model.part("tip")
    tip.visual(
        tip_holder_mesh,
        origin=Origin(),
        material=white_plastic,
        name="tip_holder",
    )
    tip.visual(
        nib_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=felt_gray,
        name="felt_nib",
    )
    tip.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0045, length=0.032),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    cap = model.part("cap")
    cap.visual(
        cap_shell_mesh,
        origin=Origin(),
        material=green_plastic,
        name="cap_shell",
    )
    cap.visual(
        cap_clip_mesh,
        origin=Origin(),
        material=dark_green,
        name="cap_clip",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0096, length=0.074),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
    )

    model.articulation(
        "barrel_to_tip",
        ArticulationType.FIXED,
        parent="barrel",
        child="tip",
        origin=Origin(xyz=(0.0, 0.0, TIP_JOINT_Z)),
    )
    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent="barrel",
        child="cap",
        origin=Origin(xyz=(0.0, 0.0, CAP_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.25,
            lower=0.0,
            upper=CAP_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    ctx.allow_overlap("barrel", "cap", reason="The hollow cap seats as a light friction fit over the barrel collar.")
    ctx.allow_overlap("tip", "cap", reason="The cap intentionally encloses the writing tip in the parked pose.")
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_gap("tip", "barrel", axis="z", max_gap=0.001, max_penetration=0.0001)
    ctx.expect_aabb_overlap("tip", "barrel", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_contact("cap", "barrel")
    ctx.expect_aabb_contact("cap", "tip")
    ctx.expect_aabb_overlap("cap", "barrel", axes="xy", min_overlap=0.014)
    ctx.expect_joint_motion_axis("cap_slide", "cap", world_axis="z", direction="positive", min_delta=0.03)

    with ctx.pose(cap_slide=CAP_TRAVEL):
        ctx.expect_aabb_overlap("cap", "barrel", axes="xy", min_overlap=0.014)
        ctx.expect_aabb_gap("cap", "tip", axis="z", max_gap=0.055, max_penetration=0.0)
        ctx.expect_aabb_gap("cap", "barrel", axis="z", max_gap=0.09, max_penetration=0.0)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
