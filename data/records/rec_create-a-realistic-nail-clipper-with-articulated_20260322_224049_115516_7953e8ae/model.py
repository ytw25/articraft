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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nail_clipper", assets=ASSETS)

    brushed_steel = model.material("brushed_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    polished_edge = model.material("polished_edge", rgba=(0.86, 0.88, 0.90, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.39, 0.41, 0.44, 1.0))

    def lofted_strip(
        filename: str,
        sections: list[tuple[float, float, float, float]],
    ):
        loops = []
        for x_pos, width, thickness, z_center in sections:
            profile = rounded_rect_profile(
                width,
                thickness,
                radius=min(width, thickness) * 0.45,
                corner_segments=10,
            )
            loops.append([(x_pos, y_pos, z_center + z_pos) for y_pos, z_pos in profile])
        geom = repair_loft(section_loft(loops))
        return mesh_from_geometry(geom, ASSETS.mesh_dir / filename)

    lower_mesh = lofted_strip(
        "nail_clipper_lower_body.obj",
        [
            (-0.0260, 0.0136, 0.0017, 0.00085),
            (-0.0140, 0.0133, 0.0018, 0.00100),
            (0.0030, 0.0129, 0.00195, 0.00122),
            (0.0210, 0.0121, 0.00205, 0.00115),
            (0.0365, 0.0112, 0.00235, 0.00108),
        ],
    )
    upper_mesh = lofted_strip(
        "nail_clipper_upper_blade.obj",
        [
            (0.0000, 0.0074, 0.0022, 0.00028),
            (0.0050, 0.0098, 0.00165, 0.00045),
            (0.0140, 0.0118, 0.00132, 0.00060),
            (0.0320, 0.0117, 0.00128, 0.00068),
            (0.0490, 0.0110, 0.00148, 0.00074),
            (0.0600, 0.0102, 0.00182, 0.00066),
        ],
    )
    lever_mesh = lofted_strip(
        "nail_clipper_lever.obj",
        [
            (-0.0440, 0.0090, 0.00135, 0.00060),
            (-0.0260, 0.0088, 0.00155, 0.00068),
            (-0.0080, 0.0080, 0.00150, 0.00070),
            (0.0030, 0.0068, 0.00170, 0.00076),
            (0.0070, 0.0056, 0.00190, 0.00078),
        ],
    )

    lower_body = model.part("lower_body")
    lower_body.visual(lower_mesh, material=brushed_steel)
    lower_body.visual(
        Cylinder(radius=0.00145, length=0.0032),
        origin=Origin(xyz=(-0.0250, -0.0036, 0.00305), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_pivot_lug_left",
    )
    lower_body.visual(
        Cylinder(radius=0.00145, length=0.0032),
        origin=Origin(xyz=(-0.0250, 0.0036, 0.00305), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_pivot_lug_right",
    )
    lower_body.visual(
        Cylinder(radius=0.00070, length=0.0110),
        origin=Origin(xyz=(0.0340, 0.0, 0.00238), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_edge,
        name="lower_cutting_edge",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((0.063, 0.014, 0.006)),
        mass=0.018,
        origin=Origin(xyz=(0.004, 0.0, 0.0020)),
    )

    upper_blade = model.part("upper_blade")
    upper_blade.visual(upper_mesh, material=brushed_steel)
    upper_blade.visual(
        Cylinder(radius=0.00125, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.00015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_pivot_barrel",
    )
    upper_blade.visual(
        Cylinder(radius=0.00062, length=0.0106),
        origin=Origin(xyz=(0.0580, 0.0, 0.00018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_edge,
        name="upper_cutting_edge",
    )
    upper_blade.visual(
        Cylinder(radius=0.00090, length=0.0048),
        origin=Origin(xyz=(0.0228, 0.0, 0.00134), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lever_pivot_boss",
    )
    upper_blade.inertial = Inertial.from_geometry(
        Box((0.061, 0.012, 0.005)),
        mass=0.010,
        origin=Origin(xyz=(0.030, 0.0, 0.0016)),
    )

    lever = model.part("lever")
    lever.visual(lever_mesh, material=brushed_steel)
    lever.visual(
        Cylinder(radius=0.00125, length=0.0064),
        origin=Origin(xyz=(0.0, 0.0, 0.00086), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_sleeve",
    )
    lever.visual(
        Box((0.0150, 0.0074, 0.00034)),
        origin=Origin(xyz=(-0.0270, 0.0, 0.00142)),
        material=polished_edge,
        name="thumb_pad",
    )
    lever.visual(
        Box((0.0052, 0.0054, 0.00110)),
        origin=Origin(xyz=(0.0032, 0.0, -0.00002)),
        material=dark_steel,
        name="cam_nose",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.052, 0.010, 0.005)),
        mass=0.007,
        origin=Origin(xyz=(-0.020, 0.0, 0.0012)),
    )

    model.articulation(
        "blade_hinge",
        ArticulationType.REVOLUTE,
        parent="lower_body",
        child="upper_blade",
        origin=Origin(xyz=(-0.0250, 0.0, 0.00330)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.007,
            upper=0.030,
        ),
    )
    model.articulation(
        "lever_hinge",
        ArticulationType.REVOLUTE,
        parent="upper_blade",
        child="lever",
        origin=Origin(xyz=(0.0228, 0.0, 0.00134)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=4.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, geometry_source="collision")
    ctx.allow_overlap(
        "lower_body",
        "upper_blade",
        reason="rear hinge sleeves and tightly seated cutting lips intentionally nest like a stamped nail clipper",
    )
    ctx.allow_overlap(
        "upper_blade",
        "lever",
        reason="the operating lever sleeve and resting cam nest against the blade and pivot boss",
    )
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected(use="visual")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_overlap("upper_blade", "lower_body", axes="x", min_overlap=0.052)
    ctx.expect_aabb_overlap("upper_blade", "lower_body", axes="y", min_overlap=0.009)
    ctx.expect_aabb_overlap("lever", "upper_blade", axes="x", min_overlap=0.030)
    ctx.expect_aabb_overlap("lever", "upper_blade", axes="y", min_overlap=0.005)
    ctx.expect_joint_motion_axis(
        "blade_hinge",
        "upper_blade",
        world_axis="z",
        direction="positive",
        min_delta=0.0005,
    )
    ctx.expect_joint_motion_axis(
        "lever_hinge",
        "lever",
        world_axis="z",
        direction="positive",
        min_delta=0.012,
    )

    with ctx.pose(blade_hinge=-0.0065):
        ctx.expect_aabb_contact("upper_blade", "lower_body")
        ctx.expect_aabb_overlap("upper_blade", "lower_body", axes="x", min_overlap=0.052)

    with ctx.pose(blade_hinge=0.028):
        ctx.expect_aabb_overlap("upper_blade", "lower_body", axes="x", min_overlap=0.050)
        ctx.expect_aabb_overlap("upper_blade", "lower_body", axes="y", min_overlap=0.0085)

    with ctx.pose(lever_hinge=0.0):
        ctx.expect_aabb_contact("lever", "upper_blade")

    with ctx.pose(lever_hinge=1.10):
        ctx.expect_aabb_overlap("lever", "upper_blade", axes="x", min_overlap=0.010)
        ctx.expect_aabb_overlap("lever", "upper_blade", axes="y", min_overlap=0.004)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
