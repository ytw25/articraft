from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WHEEL_RADIUS = 0.43
WHEEL_WIDTH = 0.28
AXLE_RADIUS = 0.065
WHEELBASE_HALF = 1.08
TRACK_HALF = 0.95
WHEEL_CENTER_Z = WHEEL_RADIUS
SLEW_Z = 0.98
BOOM_PIVOT_X = 0.76
BOOM_PIVOT_Y = 0.18
BOOM_PIVOT_Z = 0.42
BOOM_KNUCKLE_X = 1.50
BOOM_KNUCKLE_Z = 0.02
DIPPER_TIP_X = 1.00
DIPPER_TIP_Z = 0.00


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return (
        cq.Workplane("XY", origin=(0.0, 0.0, -(length / 2.0)))
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x, y, z))
    )


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _make_wheel_shape() -> cq.Workplane:
    tire = _y_cylinder(WHEEL_RADIUS, WHEEL_WIDTH, (0.0, 0.0, 0.0))
    tire = tire.cut(_y_cylinder(0.31, WHEEL_WIDTH - 0.02, (0.0, 0.0, 0.0)))
    rim = _y_cylinder(0.33, WHEEL_WIDTH * 0.82, (0.0, 0.0, 0.0))
    rim = rim.cut(_y_cylinder(0.10, WHEEL_WIDTH + 0.02, (0.0, 0.0, 0.0)))
    return tire.union(rim)


def _make_chassis_shape() -> cq.Workplane:
    spine = _box((2.28, 0.52, 0.20), (0.0, 0.0, 0.64))
    front_axle = _box((0.46, 1.62, 0.16), (WHEELBASE_HALF, 0.0, 0.58))
    rear_axle = _box((0.46, 1.62, 0.16), (-WHEELBASE_HALF, 0.0, 0.58))
    deck = _box((1.52, 0.90, 0.10), (0.0, 0.0, 0.82))
    turntable_plate = cq.Workplane("XY", origin=(0.0, 0.0, 0.91)).circle(0.41).extrude(0.05)
    front_taper = _box((0.54, 0.64, 0.10), (0.86, 0.0, 0.76)).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -8.0)
    rear_tail = _box((0.62, 0.64, 0.08), (-0.88, 0.0, 0.76))
    return spine.union(front_axle).union(rear_axle).union(deck).union(turntable_plate).union(front_taper).union(rear_tail)


def _make_upper_platform_shape() -> cq.Workplane:
    lower_ring = cq.Workplane("XY", origin=(0.0, 0.0, 0.0)).circle(0.42).extrude(0.04)
    deck = _box((1.56, 1.12, 0.08), (-0.06, 0.0, 0.08))
    engine_house = _box((1.08, 0.84, 0.46), (-0.24, -0.02, 0.31))
    counterweight = _box((0.46, 0.92, 0.34), (-0.82, 0.0, 0.24))
    cab_shell = _box((0.48, 0.50, 0.62), (0.24, 0.28, 0.35))
    cab_shell = cab_shell.cut(_box((0.30, 0.34, 0.40), (0.28, 0.28, 0.30)))
    cab_shell = cab_shell.cut(_box((0.50, 0.56, 0.22), (0.42, 0.28, 0.58)).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 24.0))
    cab_base = _box((0.20, 0.24, 0.08), (0.08, 0.20, 0.13))
    boom_support = _box((0.40, 0.24, 0.26), (0.56, BOOM_PIVOT_Y, 0.29))
    nose_top = _box((0.24, 0.22, 0.12), (0.70, BOOM_PIVOT_Y, 0.42))
    return lower_ring.union(deck).union(engine_house).union(counterweight).union(cab_shell).union(cab_base).union(boom_support).union(nose_top)


def _make_boom_shape() -> cq.Workplane:
    root = _box((0.14, 0.12, 0.14), (0.07, 0.0, 0.03))
    lower = _box((0.74, 0.12, 0.12), (0.42, 0.0, 0.10)).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 14.0)
    upper = _box((0.80, 0.10, 0.10), (0.98, 0.0, 0.22)).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 10.0)
    knuckle = _box((0.12, 0.12, 0.12), (BOOM_KNUCKLE_X - 0.06, 0.0, BOOM_KNUCKLE_Z))
    return root.union(lower).union(upper).union(knuckle)


def _make_dipper_shape() -> cq.Workplane:
    root = _box((0.12, 0.10, 0.12), (0.06, 0.0, 0.04))
    body = _box((0.74, 0.10, 0.10), (0.43, 0.0, 0.05)).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -5.0)
    tip = _box((0.12, 0.10, 0.10), (DIPPER_TIP_X - 0.06, 0.0, DIPPER_TIP_Z + 0.04))
    return root.union(body).union(tip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheeled_excavator")
    model.material("construction_yellow", rgba=(0.88, 0.70, 0.14, 1.0))
    model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    model.material("steel", rgba=(0.46, 0.48, 0.50, 1.0))

    chassis = model.part("chassis")
    chassis.visual(Box((2.28, 0.52, 0.20)), origin=Origin(xyz=(0.0, 0.0, 0.64)), material="charcoal", name="chassis_spine")
    chassis.visual(Box((0.46, 1.62, 0.16)), origin=Origin(xyz=(WHEELBASE_HALF, 0.0, 0.58)), material="charcoal", name="front_axle")
    chassis.visual(Box((0.46, 1.62, 0.16)), origin=Origin(xyz=(-WHEELBASE_HALF, 0.0, 0.58)), material="charcoal", name="rear_axle")
    chassis.visual(Box((1.52, 0.90, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.79)), material="charcoal", name="upper_deck")
    chassis.visual(Cylinder(radius=0.24, length=0.14), origin=Origin(xyz=(0.0, 0.0, 0.91)), material="charcoal", name="slew_pedestal")
    chassis.visual(Box((0.54, 0.64, 0.10)), origin=Origin(xyz=(0.86, 0.0, 0.76), rpy=(0.0, -0.14, 0.0)), material="charcoal", name="front_taper")
    chassis.visual(Box((0.62, 0.64, 0.08)), origin=Origin(xyz=(-0.88, 0.0, 0.76)), material="charcoal", name="rear_tail")

    front_left_wheel = model.part("front_left_wheel")
    front_left_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="wheel_body",
    )
    front_left_wheel.visual(
        Cylinder(radius=0.30, length=WHEEL_WIDTH * 0.82),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="wheel_rim",
    )
    front_right_wheel = model.part("front_right_wheel")
    front_right_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="wheel_body",
    )
    front_right_wheel.visual(
        Cylinder(radius=0.30, length=WHEEL_WIDTH * 0.82),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="wheel_rim",
    )
    rear_left_wheel = model.part("rear_left_wheel")
    rear_left_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="wheel_body",
    )
    rear_left_wheel.visual(
        Cylinder(radius=0.30, length=WHEEL_WIDTH * 0.82),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="wheel_rim",
    )
    rear_right_wheel = model.part("rear_right_wheel")
    rear_right_wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="rubber_black",
        name="wheel_body",
    )
    rear_right_wheel.visual(
        Cylinder(radius=0.30, length=WHEEL_WIDTH * 0.82),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="wheel_rim",
    )

    upper_platform = model.part("upper_platform")
    upper_platform.visual(Cylinder(radius=0.42, length=0.04), origin=Origin(xyz=(0.0, 0.0, 0.02)), material="construction_yellow", name="platform_body")
    upper_platform.visual(Box((1.56, 1.12, 0.08)), origin=Origin(xyz=(-0.06, 0.0, 0.075)), material="construction_yellow", name="platform_deck")
    upper_platform.visual(Box((1.08, 0.84, 0.46)), origin=Origin(xyz=(-0.24, -0.02, 0.34)), material="construction_yellow", name="engine_house")
    upper_platform.visual(Box((0.46, 0.92, 0.34)), origin=Origin(xyz=(-0.82, 0.0, 0.24)), material="construction_yellow", name="counterweight")
    upper_platform.visual(Box((0.20, 0.24, 0.08)), origin=Origin(xyz=(0.08, 0.20, 0.13)), material="charcoal", name="cab_base")
    upper_platform.visual(Box((0.48, 0.50, 0.62)), origin=Origin(xyz=(0.24, 0.28, 0.35)), material="charcoal", name="cab_shell")
    upper_platform.visual(Box((0.50, 0.52, 0.03)), origin=Origin(xyz=(0.24, 0.28, 0.665)), material="charcoal", name="cab_roof")
    upper_platform.visual(Box((0.40, 0.24, 0.24)), origin=Origin(xyz=(0.56, BOOM_PIVOT_Y, 0.30)), material="construction_yellow", name="boom_support")

    boom = model.part("boom")
    boom.visual(Box((1.50, 0.12, 0.16)), origin=Origin(xyz=(0.75, 0.0, -0.04)), material="construction_yellow", name="boom_body")

    dipper = model.part("dipper")
    dipper.visual(Box((1.00, 0.10, 0.14)), origin=Origin(xyz=(0.50, 0.0, -0.02)), material="construction_yellow", name="dipper_body")

    bucket = model.part("bucket")
    bucket.visual(Box((0.10, 0.10, 0.12)), origin=Origin(xyz=(0.05, 0.0, 0.02)), material="steel", name="bucket_hinge_block")
    bucket.visual(Box((0.24, 0.08, 0.03)), origin=Origin(xyz=(0.18, 0.0, -0.06), rpy=(0.0, -0.45, 0.0)), material="steel", name="bucket_shell")
    bucket.visual(Box((0.18, 0.08, 0.03)), origin=Origin(xyz=(0.10, 0.0, 0.02), rpy=(0.0, -1.05, 0.0)), material="steel", name="bucket_back")
    bucket.visual(Box((0.24, 0.012, 0.16)), origin=Origin(xyz=(0.18, 0.044, -0.03), rpy=(0.0, -0.55, 0.0)), material="steel", name="bucket_side_left")
    bucket.visual(Box((0.24, 0.012, 0.16)), origin=Origin(xyz=(0.18, -0.044, -0.03), rpy=(0.0, -0.55, 0.0)), material="steel", name="bucket_side_right")

    model.articulation(
        "chassis_to_front_left_wheel",
        ArticulationType.FIXED,
        parent=chassis,
        child=front_left_wheel,
        origin=Origin(xyz=(WHEELBASE_HALF, TRACK_HALF, WHEEL_CENTER_Z)),
    )
    model.articulation(
        "chassis_to_front_right_wheel",
        ArticulationType.FIXED,
        parent=chassis,
        child=front_right_wheel,
        origin=Origin(xyz=(WHEELBASE_HALF, -TRACK_HALF, WHEEL_CENTER_Z)),
    )
    model.articulation(
        "chassis_to_rear_left_wheel",
        ArticulationType.FIXED,
        parent=chassis,
        child=rear_left_wheel,
        origin=Origin(xyz=(-WHEELBASE_HALF, TRACK_HALF, WHEEL_CENTER_Z)),
    )
    model.articulation(
        "chassis_to_rear_right_wheel",
        ArticulationType.FIXED,
        parent=chassis,
        child=rear_right_wheel,
        origin=Origin(xyz=(-WHEELBASE_HALF, -TRACK_HALF, WHEEL_CENTER_Z)),
    )
    model.articulation(
        "slew",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=upper_platform,
        origin=Origin(xyz=(0.0, 0.0, SLEW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.2),
    )
    model.articulation(
        "boom_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_platform,
        child=boom,
        origin=Origin(xyz=(BOOM_PIVOT_X, BOOM_PIVOT_Y, BOOM_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.9, lower=-0.35, upper=1.10),
    )
    model.articulation(
        "dipper_pitch",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=dipper,
        origin=Origin(xyz=(BOOM_KNUCKLE_X, 0.0, BOOM_KNUCKLE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.1, lower=-1.35, upper=0.85),
    )
    model.articulation(
        "bucket_pitch",
        ArticulationType.REVOLUTE,
        parent=dipper,
        child=bucket,
        origin=Origin(xyz=(DIPPER_TIP_X, 0.0, DIPPER_TIP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.3, lower=-1.60, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    upper_platform = object_model.get_part("upper_platform")
    boom = object_model.get_part("boom")
    dipper = object_model.get_part("dipper")
    bucket = object_model.get_part("bucket")

    slew = object_model.get_articulation("slew")
    boom_pitch = object_model.get_articulation("boom_pitch")
    dipper_pitch = object_model.get_articulation("dipper_pitch")
    bucket_pitch = object_model.get_articulation("bucket_pitch")

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

    ctx.check(
        "all key parts present",
        all(
            part is not None
            for part in (
                chassis,
                front_left_wheel,
                front_right_wheel,
                rear_left_wheel,
                rear_right_wheel,
                upper_platform,
                boom,
                dipper,
                bucket,
            )
        ),
    )

    for wheel_name, wheel in (
        ("front left wheel mounted", front_left_wheel),
        ("front right wheel mounted", front_right_wheel),
        ("rear left wheel mounted", rear_left_wheel),
        ("rear right wheel mounted", rear_right_wheel),
    ):
        ctx.expect_contact(wheel, chassis, name=wheel_name)

    ctx.expect_contact(upper_platform, chassis, name="slew platform seats on chassis pedestal")
    ctx.expect_contact(boom, upper_platform, name="boom pin contacts platform pedestal")
    ctx.expect_contact(dipper, boom, name="dipper pin contacts boom knuckle")
    ctx.expect_contact(bucket, dipper, name="bucket pin contacts dipper fork")

    ctx.check(
        "slew articulation is continuous about vertical centerline",
        slew.joint_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in slew.axis) == (0.0, 0.0, 1.0),
        details=f"type={slew.joint_type}, axis={slew.axis}",
    )

    boom_lower = boom_pitch.motion_limits.lower
    boom_upper = boom_pitch.motion_limits.upper
    dipper_upper = dipper_pitch.motion_limits.upper
    bucket_upper = bucket_pitch.motion_limits.upper

    with ctx.pose({boom_pitch: 0.0, dipper_pitch: 0.0, bucket_pitch: 0.0, slew: 0.0}):
        rest_bucket_pos = ctx.part_world_position(bucket)
        rest_bucket_aabb = ctx.part_element_world_aabb(bucket, elem="bucket_shell")

    with ctx.pose({boom_pitch: boom_upper, dipper_pitch: 0.0, bucket_pitch: 0.0, slew: 0.0}):
        raised_bucket_pos = ctx.part_world_position(bucket)

    with ctx.pose({boom_pitch: 0.40, dipper_pitch: dipper_upper, bucket_pitch: 0.0, slew: 0.0}):
        folded_bucket_pos = ctx.part_world_position(bucket)

    with ctx.pose({boom_pitch: 0.35, dipper_pitch: 0.20, bucket_pitch: 0.0, slew: 0.0}):
        uncurl_bucket_aabb = ctx.part_element_world_aabb(bucket, elem="bucket_shell")

    with ctx.pose({boom_pitch: 0.35, dipper_pitch: 0.20, bucket_pitch: bucket_upper, slew: 0.0}):
        curl_bucket_aabb = ctx.part_element_world_aabb(bucket, elem="bucket_shell")

    with ctx.pose({boom_pitch: 0.35, dipper_pitch: 0.15, bucket_pitch: 0.0, slew: 0.0}):
        slew_zero_bucket = ctx.part_world_position(bucket)
    with ctx.pose({boom_pitch: 0.35, dipper_pitch: 0.15, bucket_pitch: 0.0, slew: pi / 2.0}):
        slew_quarter_bucket = ctx.part_world_position(bucket)

    ctx.check(
        "boom positive pitch raises the digging chain",
        rest_bucket_pos is not None
        and raised_bucket_pos is not None
        and boom_lower is not None
        and raised_bucket_pos[2] > rest_bucket_pos[2] + 0.40,
        details=f"rest={rest_bucket_pos}, raised={raised_bucket_pos}, lower={boom_lower}, upper={boom_upper}",
    )
    ctx.check(
        "dipper positive pitch folds the short stick upward",
        rest_bucket_pos is not None
        and folded_bucket_pos is not None
        and folded_bucket_pos[2] > rest_bucket_pos[2] + 0.12,
        details=f"rest={rest_bucket_pos}, folded={folded_bucket_pos}, upper={dipper_upper}",
    )
    ctx.check(
        "bucket positive pitch curls the trenching bucket upward",
        uncurl_bucket_aabb is not None
        and curl_bucket_aabb is not None
        and curl_bucket_aabb[1][2] > uncurl_bucket_aabb[1][2] + 0.10,
        details=f"uncurled={uncurl_bucket_aabb}, curled={curl_bucket_aabb}, upper={bucket_upper}",
    )
    ctx.check(
        "slew rotates the upper structure around the vertical centerline",
        slew_zero_bucket is not None
        and slew_quarter_bucket is not None
        and abs(slew_zero_bucket[2] - slew_quarter_bucket[2]) < 0.01
        and abs(abs(slew_zero_bucket[0]) - abs(slew_quarter_bucket[1])) < 0.08
        and abs(abs(slew_zero_bucket[1]) - abs(slew_quarter_bucket[0])) < 0.08,
        details=f"q0={slew_zero_bucket}, q90={slew_quarter_bucket}",
    )
    ctx.check(
        "bucket remains visually narrow for trenching",
        rest_bucket_aabb is not None and (rest_bucket_aabb[1][1] - rest_bucket_aabb[0][1]) < 0.36,
        details=f"bucket_aabb={rest_bucket_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
