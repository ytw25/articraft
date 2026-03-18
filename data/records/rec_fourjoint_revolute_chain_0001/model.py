from __future__ import annotations

from pathlib import Path

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    cadquery_available,
    mesh_from_cadquery,
)

HERE = Path(__file__).resolve().parent
MESH_DIR = HERE / "meshes"
MESH_DIR.mkdir(exist_ok=True)


# >>> USER_CODE_START
import math

from sdk_hybrid import Cylinder

if cadquery_available():
    import cadquery as cq
else:
    cq = None


BASE_SIZE = (0.16, 0.14, 0.09)
BASE_TOP_Z = BASE_SIZE[2] / 2.0
LINK_LENGTHS = (0.22, 0.18, 0.14, 0.10)
LINK_WIDTH_Y = 0.05
LINK_HEIGHT_Z = 0.02
LINK1_Z_OFFSET = 0.014


def _maybe_mesh(name: str, solid):
    if cq is None:
        return None
    return mesh_from_cadquery(solid, MESH_DIR / f"{name}.obj")


def _base_visual_shape():
    pedestal = cq.Workplane("XY").box(*BASE_SIZE)
    collar = (
        cq.Workplane("XY").circle(0.055).extrude(0.014).translate((0.0, 0.0, BASE_TOP_Z - 0.014))
    )
    turret = cq.Workplane("XY").circle(0.038).extrude(0.03).translate((0.0, 0.0, BASE_TOP_Z - 0.03))
    return pedestal.union(collar).union(turret)


def _arm_visual_shape(length: float, *, z_offset: float = 0.0, root_turntable: bool = False):
    beam_length = max(length - 0.036, 0.02)
    beam = (
        cq.Workplane("XY")
        .box(beam_length, 0.034, 0.02)
        .translate((0.018 + (beam_length / 2.0), 0.0, z_offset))
    )
    proximal_knuckle = (
        cq.Workplane("XZ").circle(0.022).extrude(0.05).translate((0.024, 0.025, z_offset))
    )
    distal_knuckle = (
        cq.Workplane("XZ").circle(0.022).extrude(0.05).translate((length - 0.022, 0.025, z_offset))
    )
    shape = beam.union(proximal_knuckle).union(distal_knuckle)
    if root_turntable:
        root = cq.Workplane("XY").circle(0.03).extrude(0.026).translate((0.0, 0.0, 0.002))
        shape = shape.union(root)
    return shape


def _sensor_visual_shape():
    mast = cq.Workplane("XY").box(0.022, 0.026, 0.018).translate((0.014, 0.0, 0.028))
    body = cq.Workplane("XY").box(0.052, 0.04, 0.03).translate((0.036, 0.0, 0.052))
    visor = cq.Workplane("XY").box(0.03, 0.048, 0.008).translate((0.03, 0.0, 0.069))
    lens = cq.Workplane("YZ").circle(0.011).extrude(0.014).translate((0.062, 0.0, 0.052))
    return mast.union(body).union(visor).union(lens)


def _add_base_visuals(part, *, base_material: str, accent_material: str) -> None:
    mesh = _maybe_mesh("deployable_arm_base", _base_visual_shape()) if cq is not None else None
    if mesh is not None:
        part.visual(mesh, material=base_material)
        return

    part.visual(Box(BASE_SIZE), material=base_material)
    part.visual(
        Cylinder(radius=0.055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z - 0.007)),
        material=accent_material,
    )
    part.visual(
        Cylinder(radius=0.038, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z - 0.015)),
        material=accent_material,
    )


def _add_arm_visuals(
    part,
    *,
    name: str,
    length: float,
    material: str,
    z_offset: float = 0.0,
    root_turntable: bool = False,
) -> None:
    mesh = (
        _maybe_mesh(
            name, _arm_visual_shape(length, z_offset=z_offset, root_turntable=root_turntable)
        )
        if cq is not None
        else None
    )
    if mesh is not None:
        part.visual(mesh, material=material)
        return

    beam_length = max(length - 0.036, 0.02)
    part.visual(
        Box((beam_length, 0.034, 0.02)),
        origin=Origin(xyz=(0.018 + (beam_length / 2.0), 0.0, z_offset)),
        material=material,
    )
    part.visual(
        Cylinder(radius=0.022, length=0.05),
        origin=Origin(xyz=(0.024, 0.0, z_offset), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
    )
    part.visual(
        Cylinder(radius=0.022, length=0.05),
        origin=Origin(
            xyz=(length - 0.022, 0.0, z_offset),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
    )
    if root_turntable:
        part.visual(
            Cylinder(radius=0.03, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
            material=material,
        )


def _add_sensor_visuals(part, *, body_material: str, lens_material: str) -> None:
    mesh = (
        _maybe_mesh("deployable_arm_sensor_pod", _sensor_visual_shape()) if cq is not None else None
    )
    if mesh is not None:
        part.visual(mesh, material=body_material)
        return

    part.visual(
        Box((0.022, 0.026, 0.018)),
        origin=Origin(xyz=(0.014, 0.0, 0.028)),
        material=body_material,
    )
    part.visual(
        Box((0.052, 0.04, 0.03)),
        origin=Origin(xyz=(0.036, 0.0, 0.052)),
        material=body_material,
    )
    part.visual(
        Box((0.03, 0.048, 0.008)),
        origin=Origin(xyz=(0.03, 0.0, 0.069)),
        material=body_material,
    )
    part.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.069, 0.0, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_material,
    )


def _add_link_collisions(part, length: float, *, z_offset: float = 0.0) -> None:
    beam_length = max(length - 0.042, 0.02)
    part.collision(
        Box((0.022, LINK_WIDTH_Y, LINK_HEIGHT_Z)),
        origin=Origin(xyz=(0.016, 0.0, z_offset)),
    )
    part.collision(
        Box((beam_length, 0.034, LINK_HEIGHT_Z)),
        origin=Origin(xyz=(0.025 + (beam_length / 2.0), 0.0, z_offset)),
    )
    part.collision(
        Box((0.018, LINK_WIDTH_Y, LINK_HEIGHT_Z)),
        origin=Origin(xyz=(length - 0.009, 0.0, z_offset)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_link_deployable_arm")

    model.material("base_metal", rgba=(0.34, 0.37, 0.41, 1.0))
    model.material("accent_dark", rgba=(0.18, 0.2, 0.23, 1.0))
    model.material("arm_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("arm_joint_dark", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("sensor_body", rgba=(0.16, 0.18, 0.2, 1.0))
    model.material("sensor_lens", rgba=(0.32, 0.52, 0.68, 1.0))

    base = model.part("base")
    _add_base_visuals(base, base_material="base_metal", accent_material="accent_dark")
    base.collision(Box(BASE_SIZE))
    base.inertial = Inertial.from_geometry(Box(BASE_SIZE), mass=4.5)

    link1 = model.part("link1")
    _add_arm_visuals(
        link1,
        name="deployable_arm_link1",
        length=LINK_LENGTHS[0],
        material="arm_metal",
        z_offset=LINK1_Z_OFFSET,
        root_turntable=True,
    )
    _add_link_collisions(link1, LINK_LENGTHS[0], z_offset=LINK1_Z_OFFSET)
    link1.inertial = Inertial.from_geometry(
        Box((LINK_LENGTHS[0], 0.05, 0.03)),
        mass=1.2,
        origin=Origin(xyz=(LINK_LENGTHS[0] / 2.0, 0.0, LINK1_Z_OFFSET)),
    )

    link2 = model.part("link2")
    _add_arm_visuals(
        link2,
        name="deployable_arm_link2",
        length=LINK_LENGTHS[1],
        material="arm_metal",
    )
    _add_link_collisions(link2, LINK_LENGTHS[1])
    link2.inertial = Inertial.from_geometry(
        Box((LINK_LENGTHS[1], 0.05, 0.03)),
        mass=0.95,
        origin=Origin(xyz=(LINK_LENGTHS[1] / 2.0, 0.0, 0.0)),
    )

    link3 = model.part("link3")
    _add_arm_visuals(
        link3,
        name="deployable_arm_link3",
        length=LINK_LENGTHS[2],
        material="arm_metal",
    )
    _add_link_collisions(link3, LINK_LENGTHS[2])
    link3.inertial = Inertial.from_geometry(
        Box((LINK_LENGTHS[2], 0.045, 0.028)),
        mass=0.7,
        origin=Origin(xyz=(LINK_LENGTHS[2] / 2.0, 0.0, 0.0)),
    )

    link4 = model.part("link4")
    _add_arm_visuals(
        link4,
        name="deployable_arm_link4",
        length=LINK_LENGTHS[3],
        material="arm_metal",
    )
    _add_link_collisions(link4, LINK_LENGTHS[3])
    link4.inertial = Inertial.from_geometry(
        Box((LINK_LENGTHS[3], 0.042, 0.026)),
        mass=0.45,
        origin=Origin(xyz=(LINK_LENGTHS[3] / 2.0, 0.0, 0.0)),
    )

    sensor_pod = model.part("sensor_pod")
    _add_sensor_visuals(sensor_pod, body_material="sensor_body", lens_material="sensor_lens")
    sensor_pod.collision(
        Box((0.02, 0.024, 0.018)),
        origin=Origin(xyz=(0.014, 0.0, 0.028)),
    )
    sensor_pod.collision(
        Box((0.052, 0.04, 0.03)),
        origin=Origin(xyz=(0.036, 0.0, 0.052)),
    )
    sensor_pod.inertial = Inertial.from_geometry(
        Box((0.07, 0.05, 0.06)),
        mass=0.18,
        origin=Origin(xyz=(0.036, 0.0, 0.045)),
    )

    model.articulation(
        "base_to_link1",
        ArticulationType.REVOLUTE,
        parent="base",
        child="link1",
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.6,
            upper=1.6,
            effort=12.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent="link1",
        child="link2",
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, LINK1_Z_OFFSET)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.2,
            effort=9.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent="link2",
        child="link3",
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.35,
            effort=7.5,
            velocity=2.0,
        ),
    )
    model.articulation(
        "link3_to_link4",
        ArticulationType.REVOLUTE,
        parent="link3",
        child="link4",
        origin=Origin(xyz=(LINK_LENGTHS[2], 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=1.2,
            effort=5.5,
            velocity=2.2,
        ),
    )
    model.articulation(
        "link4_to_sensor_pod",
        ArticulationType.FIXED,
        parent="link4",
        child="sensor_pod",
        origin=Origin(xyz=(LINK_LENGTHS[3], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_joint_origin_near_geometry(tol=0.02)
    ctx.check_joint_origin_near_physical_geometry(tol=0.02)
    ctx.check_no_overlaps(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap_xy("link1", "base", min_overlap=0.03)
    ctx.expect_aabb_gap_z("link1", "base", max_gap=0.02, max_penetration=0.0)
    ctx.expect_above("link2", "base", min_clearance=0.0)
    ctx.expect_above("sensor_pod", "base", min_clearance=0.01)
    ctx.expect_xy_distance("sensor_pod", "base", max_dist=0.72)
    ctx.expect_aabb_gap_z("sensor_pod", "link4", max_gap=0.08, max_penetration=0.0)
    ctx.expect_xy_distance("sensor_pod", "link4", max_dist=0.11)

    ctx.expect_joint_motion_axis(
        "base_to_link1",
        "link1",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "link1_to_link2",
        "link2",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "link2_to_link3",
        "link3",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "link3_to_link4",
        "link4",
        world_axis="z",
        direction="positive",
        min_delta=0.015,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
