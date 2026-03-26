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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _loop_at_x(
    profile: list[tuple[float, float]],
    x: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for (y, z) in profile]


def _circle_loop(
    radius: float,
    x: float,
    segments: int = 24,
) -> list[tuple[float, float, float]]:
    return [
        (
            x,
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="whiteboard_marker", assets=ASSETS)

    body_white = model.material("body_white", rgba=(0.98, 0.98, 0.96, 1.0))
    cap_green = model.material("cap_green", rgba=(0.08, 0.78, 0.60, 1.0))
    ink_blue = model.material("ink_blue", rgba=(0.20, 0.36, 0.88, 1.0))
    tip_gray = model.material("tip_gray", rgba=(0.22, 0.22, 0.24, 1.0))

    cap_shell_profile = rounded_rect_profile(
        0.0182,
        0.0182,
        radius=0.0032,
        corner_segments=8,
    )
    cap_inner_profile = rounded_rect_profile(
        0.0166,
        0.0166,
        radius=0.0027,
        corner_segments=8,
    )
    cap_shell_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            cap_shell_profile,
            [cap_inner_profile],
            height=0.028,
            cap=True,
            center=True,
            closed=True,
        ).rotate_y(math.pi / 2.0),
        ASSETS.mesh_path("marker_cap_shell.obj"),
    )
    cap_nose_mesh = mesh_from_geometry(
        section_loft(
            [
                _loop_at_x(
                    rounded_rect_profile(
                        0.0144,
                        0.0144,
                        radius=0.0025,
                        corner_segments=8,
                    ),
                    -0.0035,
                ),
                _loop_at_x(
                    rounded_rect_profile(
                        0.0168,
                        0.0168,
                        radius=0.0029,
                        corner_segments=8,
                    ),
                    0.0002,
                ),
                _loop_at_x(
                    rounded_rect_profile(
                        0.0182,
                        0.0182,
                        radius=0.0032,
                        corner_segments=8,
                    ),
                    0.0035,
                ),
            ]
        ),
        ASSETS.mesh_path("marker_cap_nose.obj"),
    )
    nib_mesh = mesh_from_geometry(
        section_loft(
            [
                _circle_loop(0.0007, -0.0040, segments=20),
                _circle_loop(0.0022, -0.0010, segments=20),
                _circle_loop(0.0050, 0.0040, segments=20),
            ]
        ),
        ASSETS.mesh_path("marker_nib.obj"),
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.0087, length=0.103),
        origin=Origin(xyz=(0.0085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_white,
        name="barrel",
    )
    base.visual(
        Box((0.060, 0.0045, 0.0004)),
        origin=Origin(xyz=(0.0120, 0.0, 0.0085)),
        material=ink_blue,
        name="barrel_graphic",
    )
    base.visual(
        Cylinder(radius=0.0088, length=0.010),
        origin=Origin(xyz=(0.0650, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cap_green,
        name="rear_plug",
    )
    base.visual(
        Cylinder(radius=0.0072, length=0.008),
        origin=Origin(xyz=(-0.0470, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cap_green,
        name="front_collar",
    )
    base.visual(
        nib_mesh,
        origin=Origin(xyz=(-0.0510, 0.0, 0.0)),
        material=tip_gray,
        name="marker_tip",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0088, length=0.125),
        mass=0.018,
        origin=Origin(xyz=(0.0075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    cap = model.part("cap")
    cap.visual(
        cap_shell_mesh,
        origin=Origin(xyz=(0.0035, 0.0, 0.0)),
        material=cap_green,
        name="cap_shell",
    )
    cap.visual(
        cap_nose_mesh,
        origin=Origin(xyz=(-0.0140, 0.0, 0.0)),
        material=cap_green,
        name="cap_nose",
    )
    cap.visual(
        Box((0.021, 0.0048, 0.0008)),
        origin=Origin(xyz=(0.0040, 0.0, 0.0092)),
        material=cap_green,
        name="clip",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0095, length=0.035),
        mass=0.006,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "cap_slide",
        ArticulationType.PRISMATIC,
        parent="base",
        child="cap",
        origin=Origin(xyz=(-0.0450, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.18,
            lower=0.0,
            upper=0.055,
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
        "base",
        "cap",
        reason="the closed cap intentionally sleeves over the marker front and hides the tip",
    )
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("base", "cap", axes="yz", max_dist=0.001)
    ctx.expect_aabb_overlap("base", "cap", axes="yz", min_overlap=0.016)
    ctx.expect_aabb_overlap("base", "cap", axes="x", min_overlap=0.020)
    ctx.expect_aabb_contact("base", "cap")
    ctx.expect_joint_motion_axis(
        "cap_slide",
        "cap",
        world_axis="x",
        direction="negative",
        min_delta=0.030,
    )

    with ctx.pose(cap_slide=0.020):
        ctx.expect_aabb_contact("base", "cap")
        ctx.expect_aabb_overlap("base", "cap", axes="x", min_overlap=0.006)

    with ctx.pose(cap_slide=0.055):
        ctx.expect_origin_distance("base", "cap", axes="yz", max_dist=0.001)
        ctx.expect_aabb_overlap("base", "cap", axes="yz", min_overlap=0.016)
        ctx.expect_aabb_gap("base", "cap", axis="x", max_gap=0.030, max_penetration=0.0)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
