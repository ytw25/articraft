from __future__ import annotations

from pathlib import Path

from sdk_hybrid import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    cadquery_available,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)
# >>> USER_CODE_START
try:
    import cadquery as cq
except Exception:  # pragma: no cover - fallback if cadquery is unavailable
    cq = None


BASE_OUTER_X = 0.040
BASE_OUTER_Y = 0.030
BASE_WALL = 0.0025
BASE_HEIGHT = 0.340

MIDDLE_OUTER_X = 0.032
MIDDLE_OUTER_Y = 0.022
MIDDLE_WALL = 0.0020
MIDDLE_HEIGHT = 0.300
MIDDLE_EMBED = 0.180
MIDDLE_MIN_INSERT = 0.060

UPPER_OUTER_X = 0.025
UPPER_OUTER_Y = 0.016
UPPER_WALL = 0.0018
UPPER_HEIGHT = 0.260
UPPER_EMBED = 0.160
UPPER_MIN_INSERT = 0.055

TOP_CAP_HEIGHT = 0.004
ANTENNA_TIP_RADIUS = 0.003
ANTENNA_TIP_LENGTH = 0.180


def _box_origin(center_xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=center_xyz)


def _tube_center_z(bottom_z: float, height: float) -> float:
    return bottom_z + (0.5 * height)


def _tube_shape(
    *,
    outer_x: float,
    outer_y: float,
    height: float,
    wall: float,
    bottom_z: float,
    collar_height: float = 0.0,
    collar_growth: float = 0.0,
):
    if cq is None:
        raise RuntimeError("CadQuery is required for mesh generation")

    outer = (
        cq.Workplane("XY")
        .box(outer_x, outer_y, height, centered=(True, True, False))
        .translate((0.0, 0.0, bottom_z))
    )
    inner = (
        cq.Workplane("XY")
        .box(
            outer_x - (2.0 * wall),
            outer_y - (2.0 * wall),
            height + 0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, bottom_z - 0.002))
    )
    tube = outer.cut(inner)

    if collar_height > 0.0 and collar_growth > 0.0:
        collar_outer = (
            cq.Workplane("XY")
            .box(
                outer_x + (2.0 * collar_growth),
                outer_y + (2.0 * collar_growth),
                collar_height,
                centered=(True, True, False),
            )
            .translate((0.0, 0.0, bottom_z + height - collar_height))
        )
        collar_inner = (
            cq.Workplane("XY")
            .box(
                outer_x - (2.0 * wall),
                outer_y - (2.0 * wall),
                collar_height + 0.004,
                centered=(True, True, False),
            )
            .translate((0.0, 0.0, bottom_z + height - collar_height - 0.002))
        )
        tube = tube.union(collar_outer.cut(collar_inner))

    return tube


def _top_stage_shape():
    if cq is None:
        raise RuntimeError("CadQuery is required for mesh generation")

    bottom_z = -UPPER_EMBED
    sleeve = _tube_shape(
        outer_x=UPPER_OUTER_X,
        outer_y=UPPER_OUTER_Y,
        height=UPPER_HEIGHT,
        wall=UPPER_WALL,
        bottom_z=bottom_z,
        collar_height=0.020,
        collar_growth=0.002,
    )
    cap = (
        cq.Workplane("XY")
        .box(
            UPPER_OUTER_X - (2.0 * UPPER_WALL),
            UPPER_OUTER_Y - (2.0 * UPPER_WALL),
            TOP_CAP_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, bottom_z + UPPER_HEIGHT))
    )
    tip = (
        cq.Workplane("XY")
        .circle(ANTENNA_TIP_RADIUS)
        .extrude(ANTENNA_TIP_LENGTH)
        .translate((0.0, 0.0, bottom_z + UPPER_HEIGHT + TOP_CAP_HEIGHT))
    )
    return sleeve.union(cap).union(tip)


def _add_rectangular_tube_collisions(
    part,
    *,
    outer_x: float,
    outer_y: float,
    wall: float,
    height: float,
    bottom_z: float,
):
    center_z = _tube_center_z(bottom_z, height)






def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_antenna_mast")

    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.82, 1.0))
    dark_mount = model.material("dark_mount", rgba=(0.18, 0.18, 0.20, 1.0))

    base = model.part("base_section")
    middle = model.part("middle_section")
    upper = model.part("upper_section")

    if cadquery_available() and cq is not None:
        base_mesh = mesh_from_cadquery(
            _tube_shape(
                outer_x=BASE_OUTER_X,
                outer_y=BASE_OUTER_Y,
                height=BASE_HEIGHT,
                wall=BASE_WALL,
                bottom_z=-BASE_HEIGHT,
                collar_height=0.030,
                collar_growth=0.003,
            )
            .union(
                cq.Workplane("XY")
                .box(0.062, 0.048, 0.055, centered=(True, True, False))
                .translate((0.0, 0.0, -BASE_HEIGHT - 0.055))
            )
            .union(
                cq.Workplane("XY")
                .box(0.078, 0.060, 0.012, centered=(True, True, False))
                .translate((0.0, 0.0, -BASE_HEIGHT - 0.067))
            ),
            MESH_DIR / "base_section.obj",
        )
        middle_mesh = mesh_from_cadquery(
            _tube_shape(
                outer_x=MIDDLE_OUTER_X,
                outer_y=MIDDLE_OUTER_Y,
                height=MIDDLE_HEIGHT,
                wall=MIDDLE_WALL,
                bottom_z=-MIDDLE_EMBED,
                collar_height=0.024,
                collar_growth=0.002,
            ),
            MESH_DIR / "middle_section.obj",
        )
        upper_mesh = mesh_from_cadquery(_top_stage_shape(), MESH_DIR / "upper_section.obj")

        base.visual(base_mesh, material=dark_mount)
        middle.visual(middle_mesh, material=aluminum)
        upper.visual(upper_mesh, material=aluminum)
    else:  # pragma: no cover - lightweight fallback
        base.visual(
            Box((BASE_OUTER_X, BASE_OUTER_Y, BASE_HEIGHT)),
            origin=_box_origin((0.0, 0.0, -0.5 * BASE_HEIGHT)),
            material=dark_mount,
        )
        base.visual(
            Box((0.062, 0.048, 0.067)),
            origin=_box_origin((0.0, 0.0, -BASE_HEIGHT - 0.0335)),
            material=dark_mount,
        )
        middle.visual(
            Box((MIDDLE_OUTER_X, MIDDLE_OUTER_Y, MIDDLE_HEIGHT)),
            origin=_box_origin((0.0, 0.0, -MIDDLE_EMBED + (0.5 * MIDDLE_HEIGHT))),
            material=aluminum,
        )
        upper.visual(
            Box((UPPER_OUTER_X, UPPER_OUTER_Y, UPPER_HEIGHT)),
            origin=_box_origin((0.0, 0.0, -UPPER_EMBED + (0.5 * UPPER_HEIGHT))),
            material=aluminum,
        )
        upper.visual(
            Cylinder(radius=ANTENNA_TIP_RADIUS, length=ANTENNA_TIP_LENGTH),
            origin=_box_origin(
                (
                    0.0,
                    0.0,
                    -UPPER_EMBED + UPPER_HEIGHT + TOP_CAP_HEIGHT + (0.5 * ANTENNA_TIP_LENGTH),
                )
            ),
            material=aluminum,
        )

    _add_rectangular_tube_collisions(
        base,
        outer_x=BASE_OUTER_X,
        outer_y=BASE_OUTER_Y,
        wall=BASE_WALL,
        height=BASE_HEIGHT,
        bottom_z=-BASE_HEIGHT,
    )



    _add_rectangular_tube_collisions(
        middle,
        outer_x=MIDDLE_OUTER_X,
        outer_y=MIDDLE_OUTER_Y,
        wall=MIDDLE_WALL,
        height=MIDDLE_HEIGHT,
        bottom_z=-MIDDLE_EMBED,
    )





    base.inertial = Inertial.from_geometry(
        Box((0.078, 0.060, 0.407)),
        mass=1.9,
        origin=_box_origin((0.0, 0.0, -0.2035)),
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_OUTER_X, MIDDLE_OUTER_Y, MIDDLE_HEIGHT)),
        mass=0.72,
        origin=_box_origin((0.0, 0.0, -MIDDLE_EMBED + (0.5 * MIDDLE_HEIGHT))),
    )
    upper.inertial = Inertial.from_geometry(
        Box((0.026, 0.018, 0.444)),
        mass=0.48,
        origin=_box_origin((0.0, 0.0, 0.062)),
    )

    model.articulation(
        "base_to_middle",
        ArticulationType.PRISMATIC,
        parent="base_section",
        child="middle_section",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_EMBED - MIDDLE_MIN_INSERT,
            effort=70.0,
            velocity=0.40,
        ),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.PRISMATIC,
        parent="middle_section",
        child="upper_section",
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_HEIGHT - MIDDLE_EMBED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=UPPER_EMBED - UPPER_MIN_INSERT,
            effort=45.0,
            velocity=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_joint_origin_near_geometry(tol=0.02)
    ctx.check_articulation_origin_near_geometry(tol=0.02)
    ctx.check_no_overlaps(
        max_pose_samples=192,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("middle_section", "base_section", axes="xy", max_dist=0.002)
    ctx.expect_origin_distance("upper_section", "middle_section", axes="xy", max_dist=0.002)
    ctx.expect_aabb_overlap("middle_section", "base_section", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("upper_section", "middle_section", axes="xy", min_overlap=0.012)
    ctx.expect_origin_distance("upper_section", "base_section", axes="xy", max_dist=0.002)
    ctx.expect_aabb_overlap("upper_section", "base_section", axes="xy", min_overlap=0.012)
    ctx.expect_joint_motion_axis(
        "base_to_middle",
        "middle_section",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "middle_to_upper",
        "upper_section",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
