from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from math import pi

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir

BOTTLE_HEIGHT = 0.280
LIQUID_FRAME_X = 0.030
LIQUID_FRAME_Z = 0.150
LEFT_PIVOT_X = -0.026
PIVOT_Z = 0.206
WIRE_RADIUS = 0.0019
WIRE_MIRROR_X = 0.026


def _write_mesh(geometry, filename: str):
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, MESH_DIR / filename)


def _mirror_about_wire_center(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(2.0 * WIRE_MIRROR_X - x, y, z) for x, y, z in points]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swing_top_bottle", assets=ASSETS)

    glass = model.material("glass", rgba=(0.55, 0.71, 0.62, 0.34))
    liquid_material = model.material("amber_liquid", rgba=(0.63, 0.34, 0.09, 0.58))
    steel = model.material("steel", rgba=(0.67, 0.68, 0.71, 1.0))
    ceramic = model.material("ceramic", rgba=(0.95, 0.94, 0.90, 1.0))
    rubber = model.material("seal_rubber", rgba=(0.72, 0.25, 0.10, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        _write_mesh(
            LatheGeometry(
                [
                    (0.0, 0.000),
                    (0.010, 0.005),
                    (0.020, 0.010),
                    (0.034, 0.016),
                    (0.037, 0.060),
                    (0.038, 0.150),
                    (0.036, 0.188),
                    (0.031, 0.214),
                    (0.024, 0.235),
                    (0.018, 0.250),
                    (0.017, 0.266),
                    (0.019, 0.271),
                    (0.018, BOTTLE_HEIGHT),
                    (0.0, BOTTLE_HEIGHT),
                ],
                segments=64,
            ),
            "bottle_body.obj",
        ),
        material=glass,
    )
    bottle_body.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=glass,
        name="neck_collar",
    )
    bottle_body.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.272)),
        material=glass,
        name="lip_ring",
    )
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        x_pos = sign * 0.026
        bottle_body.visual(
            Cylinder(radius=0.004, length=0.010),
            origin=Origin(xyz=(x_pos, 0.0, PIVOT_Z), rpy=(0.0, pi / 2.0, 0.0)),
            material=glass,
            name=f"{side_name}_lug",
        )
        bottle_body.visual(
            Sphere(radius=0.0044),
            origin=Origin(xyz=(x_pos + sign * 0.005, 0.0, PIVOT_Z)),
            material=glass,
            name=f"{side_name}_lug_tip",
        )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.038, length=BOTTLE_HEIGHT),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, BOTTLE_HEIGHT / 2.0)),
    )

    liquid = model.part("liquid")
    liquid.visual(
        _write_mesh(
            LatheGeometry(
                [
                    (0.0, 0.003),
                    (0.020, 0.007),
                    (0.031, 0.014),
                    (0.035, 0.022),
                    (0.0372, 0.152),
                    (0.0350, 0.182),
                    (0.0285, 0.198),
                    (0.0, 0.198),
                ],
                segments=56,
            ),
            "liquid.obj",
        ),
        origin=Origin(xyz=(-LIQUID_FRAME_X, 0.0, -LIQUID_FRAME_Z)),
        material=liquid_material,
    )
    liquid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.196),
        mass=0.45,
        origin=Origin(xyz=(-LIQUID_FRAME_X, 0.0, -0.050)),
    )

    wire_frame = model.part("wire_frame")
    left_outer_arm = [
        (0.000, 0.000, 0.000),
        (0.003, -0.006, 0.020),
        (0.010, -0.010, 0.042),
        (0.017, -0.006, 0.062),
        (0.012, 0.004, 0.070),
        (0.006, 0.012, 0.070),
    ]
    left_inner_arm = [
        (0.000, 0.000, 0.000),
        (0.004, 0.004, 0.018),
        (0.006, 0.009, 0.040),
        (0.006, 0.012, 0.070),
    ]
    right_outer_arm = _mirror_about_wire_center(left_outer_arm)
    right_inner_arm = _mirror_about_wire_center(left_inner_arm)
    rear_crown = [
        left_outer_arm[3],
        (WIRE_MIRROR_X, -0.014, 0.068),
        right_outer_arm[3],
    ]
    front_crossbar = [
        left_outer_arm[-1],
        (WIRE_MIRROR_X, 0.016, 0.074),
        right_outer_arm[-1],
    ]
    wire_paths = {
        "left_outer_arm.obj": left_outer_arm,
        "left_inner_arm.obj": left_inner_arm,
        "right_outer_arm.obj": right_outer_arm,
        "right_inner_arm.obj": right_inner_arm,
        "rear_crown.obj": rear_crown,
        "front_crossbar.obj": front_crossbar,
    }
    for filename, path_points in wire_paths.items():
        wire_frame.visual(
            _write_mesh(
                tube_from_spline_points(
                    path_points,
                    radius=WIRE_RADIUS,
                    samples_per_segment=20,
                    radial_segments=18,
                    cap_ends=True,
                ),
                filename,
            ),
            material=steel,
        )
    wire_frame.inertial = Inertial.from_geometry(
        Box((0.052, 0.032, 0.074)),
        mass=0.06,
        origin=Origin(xyz=(0.026, 0.001, 0.037)),
    )

    stopper = model.part("stopper")
    stopper.visual(
        _write_mesh(
            LatheGeometry(
                [
                    (0.0, -0.004),
                    (0.012, -0.004),
                    (0.018, 0.000),
                    (0.018, 0.006),
                    (0.015, 0.013),
                    (0.010, 0.020),
                    (0.0, 0.022),
                ],
                segments=48,
            ),
            "stopper_top.obj",
        ),
        origin=Origin(xyz=(0.020, -0.009, 0.008)),
        material=ceramic,
    )
    stopper.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.020, -0.011, 0.001)),
        material=rubber,
        name="gasket_ring",
    )
    stopper.visual(
        Cylinder(radius=0.0115, length=0.020),
        origin=Origin(xyz=(0.020, -0.011, -0.008)),
        material=rubber,
        name="plug",
    )
    stopper.visual(
        Cylinder(radius=0.0025, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )
    stopper.visual(
        Sphere(radius=0.0042),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=ceramic,
        name="left_ear",
    )
    stopper.visual(
        Sphere(radius=0.0042),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=ceramic,
        name="right_ear",
    )
    stopper.inertial = Inertial.from_geometry(
        Box((0.040, 0.030, 0.030)),
        mass=0.10,
        origin=Origin(xyz=(0.020, -0.009, 0.006)),
    )

    model.articulation(
        "bottle_body_to_liquid",
        ArticulationType.FIXED,
        parent="bottle_body",
        child="liquid",
        origin=Origin(xyz=(LIQUID_FRAME_X, 0.0, LIQUID_FRAME_Z)),
    )
    model.articulation(
        "bottle_to_wire_frame",
        ArticulationType.REVOLUTE,
        parent="bottle_body",
        child="wire_frame",
        origin=Origin(xyz=(LEFT_PIVOT_X, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "wire_frame_to_stopper",
        ArticulationType.REVOLUTE,
        parent="wire_frame",
        child="stopper",
        origin=Origin(xyz=(0.006, 0.012, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-1.10, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "bottle_body",
        "stopper",
        reason="the rubber plug seats into the bottle neck and the two-joint wire approximation samples some coupled closure poses independently",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("liquid", "bottle_body", axes="xy", min_overlap=0.060)
    ctx.expect_aabb_contact("liquid", "bottle_body")

    ctx.expect_aabb_contact("wire_frame", "bottle_body")
    ctx.expect_aabb_contact("stopper", "wire_frame")
    ctx.expect_aabb_overlap("wire_frame", "bottle_body", axes="x", min_overlap=0.048)
    ctx.expect_aabb_overlap("wire_frame", "stopper", axes="x", min_overlap=0.034)
    ctx.expect_aabb_overlap("stopper", "bottle_body", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("stopper", "bottle_body", axis="z", max_gap=0.006, max_penetration=0.030)
    ctx.expect_joint_motion_axis(
        "bottle_to_wire_frame",
        "wire_frame",
        world_axis="y",
        direction="negative",
        min_delta=0.015,
    )
    ctx.expect_joint_motion_axis(
        "bottle_to_wire_frame",
        "stopper",
        world_axis="y",
        direction="negative",
        min_delta=0.035,
    )

    with ctx.pose(bottle_to_wire_frame=0.55, wire_frame_to_stopper=-0.35):
        ctx.expect_aabb_contact("wire_frame", "bottle_body")
        ctx.expect_aabb_contact("stopper", "wire_frame")

    with ctx.pose(bottle_to_wire_frame=1.05, wire_frame_to_stopper=-0.85):
        ctx.expect_aabb_contact("wire_frame", "bottle_body")
        ctx.expect_aabb_contact("stopper", "wire_frame")
        ctx.expect_aabb_overlap("wire_frame", "stopper", axes="x", min_overlap=0.034)
        ctx.expect_aabb_overlap("stopper", "bottle_body", axes="x", min_overlap=0.020)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
