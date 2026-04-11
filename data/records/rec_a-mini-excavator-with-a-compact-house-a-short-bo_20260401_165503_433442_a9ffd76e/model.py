from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float], fillet: float | None = None):
    solid = cq.Workplane("XY").box(*size)
    if fillet is not None and fillet > 0.0:
        solid = solid.edges("|Z").fillet(fillet)
    return solid.translate(center)


def _extruded_side_profile(points: list[tuple[float, float]], width: float):
    return cq.Workplane("XZ").polyline(points).close().extrude(width, both=True)


def make_lower_frame():
    track_left = _box((1.78, 0.24, 0.36), (0.0, 0.39, 0.18), fillet=0.04)
    track_right = _box((1.78, 0.24, 0.36), (0.0, -0.39, 0.18), fillet=0.04)
    cross_member = _box((0.92, 0.66, 0.18), (0.0, 0.0, 0.34), fillet=0.03)
    center_table = cq.Workplane("XY").circle(0.28).extrude(0.14).translate((0.0, 0.0, 0.36))
    top_pad = _box((0.56, 0.38, 0.08), (0.0, 0.0, 0.46), fillet=0.02)
    return (
        track_left
        .union(track_right)
        .union(cross_member)
        .union(center_table)
        .union(top_pad)
    )


def make_house():
    slew_ring = cq.Workplane("XY").circle(0.32).extrude(0.06)
    deck = _box((1.28, 0.92, 0.22), (0.05, 0.0, 0.17), fillet=0.04)

    engine_cover = _box((0.82, 0.86, 0.80), (-0.20, -0.08, 0.58), fillet=0.05)
    engine_slope_cutter = (
        cq.Workplane("XY")
        .box(0.64, 1.00, 0.42)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -17.0)
        .translate((0.18, 0.0, 0.93))
    )
    engine_cover = engine_cover.cut(engine_slope_cutter)

    cab_body = _box((0.56, 0.50, 0.95), (0.36, 0.20, 0.59), fillet=0.03)
    windshield_cutter = (
        cq.Workplane("XY")
        .box(0.34, 0.62, 0.86)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 12.0)
        .translate((0.58, 0.20, 0.60))
    )
    cab_body = cab_body.cut(windshield_cutter)
    cab_roof = _box((0.64, 0.58, 0.06), (0.34, 0.20, 1.06), fillet=0.02)

    boom_tower = _box((0.30, 0.38, 0.42), (0.57, 0.0, 0.44), fillet=0.03)
    boom_mount = _box((0.16, 0.28, 0.18), (0.64, 0.0, 0.64), fillet=0.02)

    return (
        slew_ring
        .union(deck)
        .union(engine_cover)
        .union(cab_body)
        .union(cab_roof)
        .union(boom_tower)
        .union(boom_mount)
    )


def make_boom():
    boom_body = _extruded_side_profile(
        [
            (0.00, -0.10),
            (0.18, -0.12),
            (0.42, -0.03),
            (0.78, 0.18),
            (1.04, 0.31),
            (1.08, 0.38),
            (0.92, 0.42),
            (0.60, 0.27),
            (0.22, 0.10),
            (0.00, 0.08),
        ],
        0.18,
    )
    root_pad = _box((0.22, 0.26, 0.24), (0.11, 0.0, -0.01), fillet=0.02)
    tip_pad = _box((0.14, 0.22, 0.18), (1.01, 0.0, 0.31), fillet=0.02)
    return boom_body.union(root_pad).union(tip_pad)


def make_arm():
    arm_body = _extruded_side_profile(
        [
            (0.00, -0.08),
            (0.15, -0.10),
            (0.34, -0.08),
            (0.58, -0.01),
            (0.78, 0.04),
            (0.84, 0.10),
            (0.70, 0.15),
            (0.42, 0.10),
            (0.14, 0.05),
            (0.00, 0.04),
        ],
        0.16,
    )
    root_pad = _box((0.18, 0.22, 0.22), (0.09, 0.0, -0.01), fillet=0.02)
    tip_pad = _box((0.10, 0.20, 0.18), (0.81, 0.0, 0.03), fillet=0.02)
    return arm_body.union(root_pad).union(tip_pad)


def make_bucket():
    outer = _extruded_side_profile(
        [
            (0.00, 0.12),
            (0.10, 0.16),
            (0.30, 0.15),
            (0.52, 0.08),
            (0.62, -0.03),
            (0.60, -0.18),
            (0.46, -0.27),
            (0.18, -0.25),
            (0.02, -0.16),
        ],
        0.36,
    )
    cavity = _extruded_side_profile(
        [
            (0.05, 0.08),
            (0.18, 0.10),
            (0.35, 0.10),
            (0.58, 0.04),
            (0.70, 0.02),
            (0.56, -0.14),
            (0.42, -0.21),
            (0.20, -0.20),
            (0.07, -0.12),
        ],
        0.30,
    )
    shell = outer.cut(cavity)
    rear_bridge = _box((0.10, 0.24, 0.12), (0.05, 0.0, 0.03), fillet=0.012)
    upper_gusset = _box((0.18, 0.22, 0.10), (0.12, 0.0, 0.09), fillet=0.010)
    left_ear = _box((0.16, 0.07, 0.20), (0.10, 0.13, 0.12), fillet=0.010)
    right_ear = _box((0.16, 0.07, 0.20), (0.10, -0.13, 0.12), fillet=0.010)
    return shell.union(rear_bridge).union(upper_gusset).union(left_ear).union(right_ear)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_excavator")

    body_paint = model.material("excavator_yellow", rgba=(0.93, 0.76, 0.18, 1.0))
    undercarriage_paint = model.material("undercarriage_charcoal", rgba=(0.22, 0.23, 0.24, 1.0))
    bucket_paint = model.material("bucket_gray", rgba=(0.40, 0.42, 0.45, 1.0))

    lower_frame = model.part("lower_frame")
    lower_frame.visual(
        mesh_from_cadquery(make_lower_frame(), "lower_frame"),
        origin=Origin(),
        material=undercarriage_paint,
        name="lower_frame_shell",
    )

    house = model.part("house")
    house.visual(
        mesh_from_cadquery(make_house(), "house"),
        origin=Origin(),
        material=body_paint,
        name="house_shell",
    )

    boom = model.part("boom")
    boom.visual(
        mesh_from_cadquery(make_boom(), "boom"),
        origin=Origin(),
        material=body_paint,
        name="boom_shell",
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(make_arm(), "arm"),
        origin=Origin(),
        material=body_paint,
        name="arm_shell",
    )

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(make_bucket(), "bucket"),
        origin=Origin(),
        material=bucket_paint,
        name="bucket_shell",
    )

    model.articulation(
        "lower_frame_to_house",
        ArticulationType.CONTINUOUS,
        parent=lower_frame,
        child=house,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.8),
    )

    model.articulation(
        "house_to_boom",
        ArticulationType.REVOLUTE,
        parent=house,
        child=boom,
        origin=Origin(xyz=(0.72, 0.0, 0.64)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=-0.70, upper=1.10),
    )

    model.articulation(
        "boom_to_arm",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=arm,
        origin=Origin(xyz=(1.08, 0.0, 0.31)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-1.40, upper=1.20),
    )

    model.articulation(
        "arm_to_bucket",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=bucket,
        origin=Origin(xyz=(0.86, 0.0, 0.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=-1.70, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_frame = object_model.get_part("lower_frame")
    house = object_model.get_part("house")
    boom = object_model.get_part("boom")
    arm = object_model.get_part("arm")
    bucket = object_model.get_part("bucket")

    slew = object_model.get_articulation("lower_frame_to_house")
    boom_joint = object_model.get_articulation("house_to_boom")
    arm_joint = object_model.get_articulation("boom_to_arm")
    bucket_joint = object_model.get_articulation("arm_to_bucket")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(house, lower_frame, contact_tol=0.0005, name="house sits on the slew table")
    ctx.expect_contact(boom, house, contact_tol=0.0005, name="boom root meets the house nose mount")
    ctx.expect_contact(arm, boom, contact_tol=0.0005, name="arm root meets the boom elbow")
    ctx.expect_contact(bucket, arm, contact_tol=0.0005, name="bucket brackets meet the arm tip")

    ctx.expect_overlap(house, lower_frame, axes="xy", min_overlap=0.40, name="house stays centered over the undercarriage")
    ctx.expect_overlap(bucket, arm, axes="yz", min_overlap=0.10, name="bucket pivot region is aligned to the arm tip")

    ctx.check(
        "slew joint is continuous vertical rotation",
        slew.articulation_type == ArticulationType.CONTINUOUS
        and slew.axis == (0.0, 0.0, 1.0)
        and slew.motion_limits is not None
        and slew.motion_limits.lower is None
        and slew.motion_limits.upper is None,
        details=f"type={slew.articulation_type}, axis={slew.axis}, limits={slew.motion_limits}",
    )
    ctx.check(
        "boom arm and bucket joints pitch about transverse axes",
        boom_joint.axis == (0.0, -1.0, 0.0)
        and arm_joint.axis == (0.0, -1.0, 0.0)
        and bucket_joint.axis == (0.0, -1.0, 0.0),
        details=f"boom={boom_joint.axis}, arm={arm_joint.axis}, bucket={bucket_joint.axis}",
    )

    boom_rest = ctx.part_world_position(boom)
    with ctx.pose({slew: math.pi / 2.0}):
        boom_swung = ctx.part_world_position(boom)
    ctx.check(
        "house slew swings the boom around the vertical axis",
        boom_rest is not None
        and boom_swung is not None
        and abs(boom_swung[1]) > 0.60
        and abs(boom_swung[0]) < 0.20,
        details=f"rest={boom_rest}, swung={boom_swung}",
    )

    arm_rest = ctx.part_world_position(arm)
    boom_upper = boom_joint.motion_limits.upper if boom_joint.motion_limits is not None else None
    with ctx.pose({boom_joint: boom_upper if boom_upper is not None else 0.9}):
        arm_raised = ctx.part_world_position(arm)
    ctx.check(
        "boom positive rotation lifts the arm",
        arm_rest is not None and arm_raised is not None and arm_raised[2] > arm_rest[2] + 0.18,
        details=f"rest={arm_rest}, raised={arm_raised}",
    )

    with ctx.pose({boom_joint: 0.45, arm_joint: -0.90}):
        bucket_low = ctx.part_world_aabb(bucket)
    with ctx.pose({boom_joint: 0.45, arm_joint: 0.60}):
        bucket_high = ctx.part_world_aabb(bucket)
    ctx.check(
        "arm positive rotation folds the bucket upward relative to the boom",
        bucket_low is not None and bucket_high is not None and bucket_high[1][2] > bucket_low[1][2] + 0.12,
        details=f"low={bucket_low}, high={bucket_high}",
    )

    with ctx.pose({boom_joint: 0.25, arm_joint: -0.35, bucket_joint: -1.20}):
        bucket_dump = ctx.part_world_aabb(bucket)
    with ctx.pose({boom_joint: 0.25, arm_joint: -0.35, bucket_joint: 0.55}):
        bucket_curl = ctx.part_world_aabb(bucket)
    ctx.check(
        "bucket positive rotation curls the mouth upward",
        bucket_dump is not None and bucket_curl is not None and bucket_curl[1][2] > bucket_dump[1][2] + 0.10,
        details=f"dump={bucket_dump}, curl={bucket_curl}",
    )

    bucket_aabb = ctx.part_world_aabb(bucket)
    ctx.check(
        "bucket reads as a wide grading bucket",
        bucket_aabb is not None
        and (bucket_aabb[1][1] - bucket_aabb[0][1]) > 0.68
        and (bucket_aabb[1][0] - bucket_aabb[0][0]) > 0.58,
        details=f"bucket_aabb={bucket_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
