from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    DomeGeometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _superellipse_section(
    width: float,
    depth: float,
    z: float,
    *,
    exponent: float = 2.9,
    segments: int = 56,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in superellipse_profile(width, depth, exponent=exponent, segments=segments)]


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(f"{name}.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_kettle", assets=ASSETS)

    gloss_black = model.material("gloss_black", rgba=(0.06, 0.06, 0.07, 1.0))
    matte_black = model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.16, 0.16, 0.17, 1.0))
    smoked_lid = model.material("smoked_lid", rgba=(0.68, 0.76, 0.84, 0.42))
    gauge_tint = model.material("gauge_tint", rgba=(0.78, 0.80, 0.82, 0.40))
    clear_indicator = model.material("clear_indicator", rgba=(0.92, 0.94, 1.0, 0.35))

    body = model.part("body")

    shell_sections = [
        _superellipse_section(0.094, 0.080, 0.012, exponent=2.7),
        _superellipse_section(0.102, 0.086, 0.040, exponent=2.8),
        _superellipse_section(0.108, 0.090, 0.110, exponent=2.95),
        _superellipse_section(0.110, 0.091, 0.185, exponent=3.0),
        _superellipse_section(0.102, 0.086, 0.214, exponent=2.7),
    ]
    body_shell = _save_mesh(
        "body_shell",
        section_loft(SectionLoftSpec(sections=tuple(shell_sections), cap=True)),
    )
    body.visual(body_shell, material=gloss_black)
    body.visual(
        Cylinder(radius=0.051, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_black,
    )

    rim_ring_geom = ExtrudeWithHolesGeometry(
        outer_profile=superellipse_profile(0.098, 0.082, exponent=3.0, segments=52),
        hole_profiles=[superellipse_profile(0.081, 0.066, exponent=3.0, segments=52)],
        height=0.004,
        center=False,
        cap=True,
        closed=True,
    ).translate(0.0, 0.0, 0.212)
    body.visual(_save_mesh("top_rim", rim_ring_geom), material=matte_black)

    gauge_geom = ExtrudeGeometry(
        rounded_rect_profile(0.028, 0.145, radius=0.010, corner_segments=10),
        height=0.0024,
        center=True,
        cap=True,
        closed=True,
    ).rotate_x(-math.pi / 2.0).translate(-0.012, -0.0455, 0.104)
    body.visual(_save_mesh("water_gauge", gauge_geom), material=gauge_tint)

    body.visual(
        Box((0.028, 0.028, 0.006)),
        origin=Origin(xyz=(0.064, 0.0, 0.192), rpy=(0.0, -0.32, 0.0)),
        material=matte_black,
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(0.061, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clear_indicator,
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.224),
        mass=1.05,
        origin=Origin(xyz=(0.0, 0.0, 0.112)),
    )

    spout = model.part("spout")
    spout_geom = ExtrudeGeometry.centered(
        [
            (0.000, 0.000),
            (0.014, 0.002),
            (0.028, 0.010),
            (0.022, 0.024),
            (0.000, 0.022),
        ],
        height=0.030,
        cap=True,
        closed=True,
    ).rotate_x(-math.pi / 2.0).translate(-0.007, 0.0, 0.003)
    spout.visual(_save_mesh("spout", spout_geom), material=gloss_black)
    spout.visual(
        Box((0.012, 0.028, 0.018)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=gloss_black,
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.032, 0.030, 0.026)),
        mass=0.06,
        origin=Origin(xyz=(0.004, 0.0, 0.002)),
    )

    handle = model.part("handle")
    handle_geom = sweep_profile_along_spline(
        [
            (0.000, 0.0, 0.000),
            (0.046, 0.0, -0.038),
            (0.052, 0.0, -0.092),
            (0.032, 0.0, -0.144),
            (-0.006, 0.0, -0.178),
        ],
        profile=rounded_rect_profile(0.024, 0.018, radius=0.006, corner_segments=6),
        samples_per_segment=20,
        cap_profile=True,
    )
    handle.visual(_save_mesh("handle", handle_geom), material=gloss_black)
    handle.visual(
        Box((0.014, 0.018, 0.024)),
        origin=Origin(xyz=(0.002, 0.0, -0.010)),
        material=gloss_black,
    )
    handle.visual(
        Box((0.012, 0.018, 0.022)),
        origin=Origin(xyz=(-0.004, 0.0, -0.168)),
        material=gloss_black,
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.070, 0.030, 0.188)),
        mass=0.12,
        origin=Origin(xyz=(0.026, 0.0, -0.089)),
    )

    lid = model.part("lid")
    lid_dome = _save_mesh(
        "lid_dome",
        DomeGeometry(radius=(0.052, 0.042, 0.013), radial_segments=30, height_segments=12, closed=True),
    )
    lid.visual(
        lid_dome,
        origin=Origin(xyz=(-0.046, 0.0, -0.002)),
        material=smoked_lid,
    )
    lid.visual(
        Box((0.053, 0.014, 0.008)),
        origin=Origin(xyz=(-0.022, 0.0, 0.006), rpy=(0.0, 0.0, math.radians(-18.0))),
        material=gloss_black,
    )
    lid.visual(
        Box((0.018, 0.028, 0.013)),
        origin=Origin(xyz=(0.002, 0.0, 0.005)),
        material=gloss_black,
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.112, 0.086, 0.020)),
        mass=0.14,
        origin=Origin(xyz=(-0.046, 0.0, 0.006)),
    )

    switch = model.part("switch")
    switch.visual(
        Box((0.020, 0.024, 0.009)),
        origin=Origin(xyz=(0.015, 0.0, 0.0045), rpy=(0.0, -0.26, 0.0)),
        material=rubber_black,
    )
    switch.visual(
        Cylinder(radius=0.0048, length=0.004),
        origin=Origin(xyz=(0.022, 0.0, 0.0085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
    )
    switch.inertial = Inertial.from_geometry(
        Box((0.020, 0.024, 0.009)),
        mass=0.03,
        origin=Origin(xyz=(0.015, 0.0, 0.0045)),
    )

    model.articulation(
        "body_to_spout",
        ArticulationType.FIXED,
        parent="body",
        child="spout",
        origin=Origin(xyz=(-0.049, 0.0, 0.202)),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.FIXED,
        parent="body",
        child="handle",
        origin=Origin(xyz=(0.050, 0.0, 0.206)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="lid",
        origin=Origin(xyz=(0.043, 0.0, 0.214)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.18),
    )
    model.articulation(
        "switch_hinge",
        ArticulationType.REVOLUTE,
        parent="handle",
        child="switch",
        origin=Origin(xyz=(0.020, 0.0, -0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual", ignore_adjacent=True, ignore_fixed=True)
    ctx.allow_overlap("handle", "lid", reason="the lid swings back toward the handle stop and collision hulls are conservative")
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("lid", "body")
    ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.060)
    ctx.expect_aabb_gap("lid", "body", axis="z", max_gap=0.008, max_penetration=0.012)
    ctx.expect_joint_motion_axis("lid_hinge", "lid", world_axis="z", direction="positive", min_delta=0.020)

    ctx.expect_aabb_contact("spout", "body")
    ctx.expect_aabb_overlap("spout", "body", axes="yz", min_overlap=0.020)

    ctx.expect_aabb_contact("handle", "body")
    ctx.expect_aabb_overlap("handle", "body", axes="yz", min_overlap=0.020)

    ctx.expect_aabb_contact("switch", "handle")
    ctx.expect_aabb_overlap("switch", "handle", axes="xy", min_overlap=0.012)
    ctx.expect_joint_motion_axis("switch_hinge", "switch", world_axis="z", direction="negative", min_delta=0.002)

    with ctx.pose(lid_hinge=1.05):
        ctx.expect_aabb_overlap("lid", "body", axes="y", min_overlap=0.050)

    with ctx.pose(switch_hinge=0.40):
        ctx.expect_aabb_contact("switch", "handle")
        ctx.expect_aabb_overlap("switch", "handle", axes="xy", min_overlap=0.010)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
