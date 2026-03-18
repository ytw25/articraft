from __future__ import annotations

from math import pi

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)


def _mount_base_shape():
    flange = cq.Workplane("XY").circle(0.21).extrude(0.022)
    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=0.022)
        .circle(0.155)
        .workplane(offset=0.15)
        .circle(0.105)
        .loft(combine=True)
    )
    collar = cq.Workplane("XY").workplane(offset=0.115).circle(0.145).extrude(0.034)
    top_plate = cq.Workplane("XY").workplane(offset=0.186).circle(0.128).extrude(0.034)
    bolt_caps = (
        cq.Workplane("XY")
        .workplane(offset=0.004)
        .rarray(0.30, 0.30, 2, 2)
        .circle(0.024)
        .extrude(0.014)
    )

    rib = cq.Workplane("XY").box(0.05, 0.11, 0.11).translate((0.12, 0.0, 0.077))
    ribs = (
        rib.union(rib.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 90.0))
        .union(rib.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 180.0))
        .union(rib.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 270.0))
    )
    service_pack = cq.Workplane("XY").box(0.08, 0.06, 0.06).translate((-0.11, 0.0, 0.074))

    return (
        flange.union(pedestal)
        .union(collar)
        .union(top_plate)
        .union(bolt_caps)
        .union(ribs)
        .union(service_pack)
    )


def _pan_housing_shape():
    ring = cq.Workplane("XY").circle(0.145).extrude(0.03)
    armored_body = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.15, 0.03),
                (-0.15, 0.20),
                (-0.07, 0.28),
                (0.08, 0.30),
                (0.17, 0.23),
                (0.17, 0.10),
                (0.12, 0.05),
                (-0.02, 0.03),
            ]
        )
        .close()
        .extrude(0.24)
        .translate((0.0, -0.12, 0.0))
    )
    front_brow = cq.Workplane("XY").box(0.08, 0.28, 0.05).translate((0.12, 0.0, 0.27))
    roof_hatch = cq.Workplane("XY").box(0.12, 0.14, 0.03).translate((-0.01, 0.0, 0.305))
    rear_pack = cq.Workplane("XY").box(0.10, 0.18, 0.12).translate((-0.16, 0.0, 0.14))
    left_countermass = cq.Workplane("XY").box(0.08, 0.08, 0.10).translate((-0.02, -0.16, 0.18))
    optic_pod = cq.Workplane("XY").box(0.11, 0.09, 0.13).translate((0.03, 0.165, 0.20))
    left_cheek = cq.Workplane("XY").box(0.17, 0.035, 0.19).translate((0.09, -0.1375, 0.185))
    right_cheek = cq.Workplane("XY").box(0.17, 0.035, 0.19).translate((0.09, 0.1375, 0.185))
    left_boss = cq.Workplane("XZ").circle(0.034).extrude(0.03).translate((0.12, -0.15, 0.18))
    right_boss = cq.Workplane("XZ").circle(0.034).extrude(0.03).translate((0.12, 0.12, 0.18))

    return (
        ring.union(armored_body)
        .union(front_brow)
        .union(roof_hatch)
        .union(rear_pack)
        .union(left_countermass)
        .union(optic_pod)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_boss)
        .union(right_boss)
    )


def _weapon_cradle_shape():
    cross_shaft = cq.Workplane("XZ").circle(0.028).extrude(0.26).translate((0.0, -0.13, 0.0))
    receiver = cq.Workplane("XY").box(0.30, 0.16, 0.12).translate((0.17, 0.0, -0.015))
    breech = cq.Workplane("XY").box(0.09, 0.14, 0.09).translate((0.02, 0.0, -0.005))
    top_cover = cq.Workplane("XY").box(0.18, 0.12, 0.04).translate((0.17, 0.0, 0.07))
    undercarriage = cq.Workplane("XY").box(0.26, 0.10, 0.07).translate((0.21, 0.0, -0.085))
    feed_stub = cq.Workplane("XY").box(0.05, 0.08, 0.06).translate((0.06, -0.09, 0.02))
    left_yoke = cq.Workplane("XY").box(0.12, 0.035, 0.14).translate((0.03, -0.1025, -0.02))
    right_yoke = cq.Workplane("XY").box(0.12, 0.035, 0.14).translate((0.03, 0.1025, -0.02))
    collar = cq.Workplane("YZ").circle(0.045).extrude(0.04).translate((0.22, 0.0, 0.01))

    return (
        cross_shaft.union(receiver)
        .union(breech)
        .union(top_cover)
        .union(undercarriage)
        .union(feed_stub)
        .union(left_yoke)
        .union(right_yoke)
        .union(collar)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="remote_weapon_station", assets=ASSETS)

    model.material("dark_steel", rgba=(0.21, 0.22, 0.25, 1.0))
    model.material("olive_drab", rgba=(0.34, 0.38, 0.27, 1.0))
    model.material("gunmetal", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("sensor_glass", rgba=(0.26, 0.39, 0.44, 0.78))
    model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    mount_base = model.part("mount_base")
    mount_base.visual(
        mesh_from_cadquery(_mount_base_shape(), "mount_base.obj", assets=ASSETS),
        material="dark_steel",
    )
    mount_base.inertial = Inertial.from_geometry(
        Box((0.42, 0.42, 0.22)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    pan_housing = model.part("pan_housing")
    pan_housing.visual(
        mesh_from_cadquery(_pan_housing_shape(), "pan_housing.obj", assets=ASSETS),
        material="olive_drab",
    )
    pan_housing.visual(
        Box((0.008, 0.068, 0.052)),
        origin=Origin(xyz=(0.082, 0.164, 0.205)),
        material="sensor_glass",
    )
    pan_housing.visual(
        Sphere(0.028),
        origin=Origin(xyz=(-0.005, 0.164, 0.286)),
        material="sensor_glass",
    )
    pan_housing.visual(
        Cylinder(radius=0.026, length=0.22),
        origin=Origin(xyz=(0.12, 0.0, 0.18), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
    )
    pan_housing.visual(
        Box((0.08, 0.16, 0.05)),
        origin=Origin(xyz=(0.085, 0.0, 0.175)),
        material="dark_steel",
    )
    pan_housing.inertial = Inertial.from_geometry(
        Box((0.38, 0.32, 0.34)),
        mass=55.0,
        origin=Origin(xyz=(0.02, 0.0, 0.17)),
    )

    weapon_cradle = model.part("weapon_cradle")
    weapon_cradle.visual(
        mesh_from_cadquery(_weapon_cradle_shape(), "weapon_cradle.obj", assets=ASSETS),
        material="olive_drab",
    )
    weapon_cradle.visual(
        Box((0.18, 0.08, 0.16)),
        origin=Origin(xyz=(0.09, -0.11, -0.02)),
        material="dark_steel",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.03, length=0.48),
        origin=Origin(xyz=(0.49, 0.0, 0.01), rpy=(0.0, pi / 2.0, 0.0)),
        material="gunmetal",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.78, 0.0, 0.01), rpy=(0.0, pi / 2.0, 0.0)),
        material="gunmetal",
    )
    weapon_cradle.visual(
        Cylinder(radius=0.023, length=0.05),
        origin=Origin(xyz=(0.855, 0.0, 0.01), rpy=(0.0, pi / 2.0, 0.0)),
        material="black_rubber",
    )
    weapon_cradle.inertial = Inertial.from_geometry(
        Box((0.84, 0.30, 0.22)),
        mass=28.0,
        origin=Origin(xyz=(0.30, 0.0, -0.01)),
    )

    model.articulation(
        "turret_pan",
        ArticulationType.REVOLUTE,
        parent=mount_base,
        child=pan_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=1200.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "weapon_elevation",
        ArticulationType.REVOLUTE,
        parent=pan_housing,
        child=weapon_cradle,
        origin=Origin(xyz=(0.12, 0.0, 0.18)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.15,
            upper=1.0,
            effort=450.0,
            velocity=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(
        max_pose_samples=96,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("pan_housing", "mount_base", axes="xy", min_overlap=0.18)
    ctx.expect_aabb_gap("pan_housing", "mount_base", axis="z", max_gap=0.004, max_penetration=0.001)
    ctx.expect_origin_distance("pan_housing", "mount_base", axes="xy", max_dist=0.02)

    ctx.expect_aabb_overlap("weapon_cradle", "pan_housing", axes="yz", min_overlap=0.10)
    ctx.expect_origin_distance("weapon_cradle", "pan_housing", axes="y", max_dist=0.01)
    ctx.expect_aabb_gap("weapon_cradle", "mount_base", axis="z", max_gap=0.12, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "weapon_elevation",
        "weapon_cradle",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )

    with ctx.pose(turret_pan=pi / 2.0):
        ctx.expect_aabb_overlap("pan_housing", "mount_base", axes="xy", min_overlap=0.18)
        ctx.expect_origin_distance("pan_housing", "mount_base", axes="xy", max_dist=0.02)
        ctx.expect_aabb_overlap("weapon_cradle", "pan_housing", axes="yz", min_overlap=0.08)

    with ctx.pose(weapon_elevation=1.0):
        ctx.expect_aabb_overlap("weapon_cradle", "pan_housing", axes="yz", min_overlap=0.08)
        ctx.expect_aabb_gap(
            "weapon_cradle", "mount_base", axis="z", max_gap=0.75, max_penetration=0.0
        )

    with ctx.pose(weapon_elevation=-0.15):
        ctx.expect_aabb_overlap("weapon_cradle", "pan_housing", axes="yz", min_overlap=0.08)
        ctx.expect_aabb_gap(
            "weapon_cradle", "mount_base", axis="z", max_gap=0.18, max_penetration=0.0
        )

    with ctx.pose({"turret_pan": pi / 2.0, "weapon_elevation": 0.9}):
        ctx.expect_aabb_overlap("pan_housing", "mount_base", axes="xy", min_overlap=0.18)
        ctx.expect_aabb_gap(
            "weapon_cradle", "mount_base", axis="z", max_gap=0.75, max_penetration=0.0
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
