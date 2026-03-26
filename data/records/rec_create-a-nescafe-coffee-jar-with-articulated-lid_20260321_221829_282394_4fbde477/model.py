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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _rounded_rect_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    corner_segments: int = 10,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    return [(x, y, z) for x, y in profile]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nescafe_coffee_jar", assets=ASSETS)

    glass = model.material("glass", rgba=(0.84, 0.91, 0.95, 0.28))
    champagne_metal = model.material("champagne_metal", rgba=(0.72, 0.66, 0.47, 1.0))
    lid_top = model.material("lid_top", rgba=(0.88, 0.82, 0.58, 1.0))
    label_gold = model.material("label_gold", rgba=(0.63, 0.53, 0.18, 0.97))
    print_white = model.material("print_white", rgba=(0.96, 0.95, 0.92, 1.0))
    dark_print = model.material("dark_print", rgba=(0.17, 0.18, 0.20, 1.0))
    coffee = model.material("coffee", rgba=(0.19, 0.11, 0.06, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.25, 0.24, 0.22, 1.0))

    body_outer = repair_loft(
        section_loft(
            SectionLoftSpec(
                sections=(
                    _rounded_rect_section(0.086, 0.055, 0.014, 0.000),
                    _rounded_rect_section(0.091, 0.060, 0.016, 0.008),
                    _rounded_rect_section(0.091, 0.060, 0.016, 0.108),
                    _rounded_rect_section(0.090, 0.059, 0.015, 0.130),
                    _rounded_rect_section(0.086, 0.055, 0.014, 0.146),
                    _rounded_rect_section(0.082, 0.052, 0.013, 0.156),
                ),
                cap=True,
                solid=True,
            )
        )
    )
    body_outer_mesh = _save_mesh("nescafe_body_outer.obj", body_outer)

    coffee_surface_geom = ExtrudeGeometry(
        rounded_rect_profile(0.071, 0.044, 0.010, corner_segments=10),
        0.006,
        center=True,
        cap=True,
        closed=True,
    )
    coffee_surface_mesh = _save_mesh("nescafe_coffee_surface.obj", coffee_surface_geom)

    lid_shell_geom = ExtrudeGeometry(
        rounded_rect_profile(0.099, 0.067, 0.017, corner_segments=12),
        0.037,
        center=True,
        cap=True,
        closed=True,
    )
    lid_shell_mesh = _save_mesh("nescafe_lid_shell.obj", lid_shell_geom)

    lid_insert_geom = ExtrudeGeometry(
        rounded_rect_profile(0.091, 0.059, 0.015, corner_segments=12),
        0.003,
        center=True,
        cap=True,
        closed=True,
    )
    lid_insert_mesh = _save_mesh("nescafe_lid_insert.obj", lid_insert_geom)

    body = model.part("body")
    body.visual(body_outer_mesh, material=glass)
    body.visual(
        coffee_surface_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=coffee,
    )
    body.visual(
        Box((0.078, 0.0016, 0.106)),
        origin=Origin(xyz=(0.0, 0.0298, 0.074)),
        material=label_gold,
    )
    body.visual(
        Box((0.078, 0.0016, 0.106)),
        origin=Origin(xyz=(0.0, -0.0298, 0.074)),
        material=label_gold,
    )
    body.visual(
        Box((0.0016, 0.043, 0.106)),
        origin=Origin(xyz=(0.0442, 0.0, 0.074)),
        material=label_gold,
    )
    body.visual(
        Box((0.0016, 0.043, 0.106)),
        origin=Origin(xyz=(-0.0442, 0.0, 0.074)),
        material=label_gold,
    )
    body.visual(
        Box((0.061, 0.0022, 0.034)),
        origin=Origin(xyz=(0.008, 0.0304, 0.086)),
        material=print_white,
    )
    body.visual(
        Box((0.046, 0.0022, 0.017)),
        origin=Origin(xyz=(0.003, 0.0306, 0.048)),
        material=dark_print,
    )
    body.visual(
        Box((0.028, 0.0020, 0.050)),
        origin=Origin(xyz=(-0.0448, -0.010, 0.083)),
        material=dark_print,
    )
    body.visual(
        Box((0.008, 0.0022, 0.008)),
        origin=Origin(xyz=(-0.0448, 0.010, 0.053)),
        material=print_white,
    )
    body.visual(
        Box((0.080, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.0310, 0.163)),
        material=champagne_metal,
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.084),
        origin=Origin(xyz=(0.0, -0.0335, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
    )
    body.inertial = Inertial.from_geometry(
        Box((0.094, 0.062, 0.156)),
        mass=0.78,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0335, -0.012)),
        material=champagne_metal,
    )
    lid.visual(
        lid_insert_mesh,
        origin=Origin(xyz=(0.0, 0.0335, 0.008)),
        material=lid_top,
    )
    lid.visual(
        Box((0.018, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.064, -0.005)),
        material=lid_top,
    )
    lid.visual(
        Cylinder(radius=0.0040, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.100, 0.067, 0.038)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0335, -0.012)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent="body",
        child="lid",
        origin=Origin(xyz=(0.0, -0.0335, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    ctx.allow_overlap(
        "body",
        "lid",
        reason="Nested flip-top lid seats over the neck and shares an overlapping hinge barrel.",
    )
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )
    ctx.expect_aabb_contact("body", "lid")
    ctx.expect_aabb_overlap("body", "lid", axes="xy", min_overlap=0.040)
    ctx.expect_aabb_overlap("body", "lid", axes="y", min_overlap=0.050)
    ctx.expect_origin_distance("lid", "body", axes="x", max_dist=0.002)
    ctx.expect_joint_motion_axis(
        "body_to_lid",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.020,
    )
    with ctx.pose(body_to_lid=1.85):
        ctx.expect_aabb_contact("body", "lid")
        ctx.expect_aabb_overlap("body", "lid", axes="x", min_overlap=0.070)
        ctx.expect_origin_distance("lid", "body", axes="x", max_dist=0.002)
    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
