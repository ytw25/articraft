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


def _centered_xz_extrusion(points: list[tuple[float, float]], width: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .moveTo(*points[0])
        .polyline(points[1:])
        .close()
        .extrude(width / 2.0, both=True)
    )


def _axis_cylinder(radius: float, length: float, axis: str = "z") -> cq.Workplane:
    solid = cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -length / 2.0))
    if axis == "y":
        return solid.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    if axis == "x":
        return solid.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    return solid


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def _checked_shape(label: str, builder) -> cq.Workplane:
    try:
        return builder()
    except Exception as exc:
        raise RuntimeError(f"{label} shape build failed: {exc}") from exc


def _make_track_module() -> cq.Workplane:
    outer_box = cq.Workplane("XY").box(3.10, 0.46, 0.60).translate((0.0, 0.0, 0.30))
    front_drum = _axis_cylinder(0.30, 0.46, axis="y").translate((1.23, 0.0, 0.30))
    rear_drum = _axis_cylinder(0.30, 0.46, axis="y").translate((-1.23, 0.0, 0.30))
    track = outer_box.union(front_drum).union(rear_drum)

    inner_void = cq.Workplane("XY").box(2.46, 0.30, 0.32).translate((0.0, 0.0, 0.30))
    side_relief = cq.Workplane("XY").box(2.20, 0.52, 0.18).translate((0.0, 0.0, 0.53))
    top_slot = cq.Workplane("XY").box(1.70, 0.24, 0.10).translate((0.0, 0.0, 0.49))

    return track.cut(inner_void).cut(side_relief).cut(top_slot).clean()


def _make_undercarriage_shape() -> cq.Workplane:
    left_track = _make_track_module().translate((0.0, 1.05, 0.0))
    right_track = _make_track_module().translate((0.0, -1.05, 0.0))

    center_frame = cq.Workplane("XY").box(1.45, 1.08, 0.34).translate((0.0, 0.0, 0.56))
    front_crossmember = cq.Workplane("XY").box(0.58, 1.76, 0.22).translate((0.82, 0.0, 0.48))
    rear_crossmember = cq.Workplane("XY").box(0.78, 1.76, 0.24).translate((-0.76, 0.0, 0.48))
    center_spine = cq.Workplane("XY").box(1.55, 0.44, 0.14).translate((0.0, 0.0, 0.65))
    slew_pedestal = _axis_cylinder(0.46, 0.12, axis="z").translate((0.0, 0.0, 0.70))

    return (
        left_track.union(right_track)
        .union(center_frame)
        .union(front_crossmember)
        .union(rear_crossmember)
        .union(center_spine)
        .union(slew_pedestal)
        .clean()
    )


def _make_upper_house_body() -> cq.Workplane:
    slew_ring = _axis_cylinder(0.58, 0.08, axis="z").translate((0.0, 0.0, 0.04))
    turntable = cq.Workplane("XY").box(2.20, 1.86, 0.20).translate((-0.14, 0.0, 0.18))
    front_deck = cq.Workplane("XY").box(0.62, 1.42, 0.16).translate((0.88, 0.0, 0.26))
    upper_housing = cq.Workplane("XY").box(1.36, 1.56, 1.12).translate((-0.48, -0.02, 0.76))
    rear_weight = _centered_xz_extrusion(
        [(-0.92, 0.26), (-1.12, 0.36), (-1.24, 0.72), (-1.20, 1.02), (-0.88, 1.14), (-0.58, 1.02), (-0.50, 0.48)],
        1.56,
    )
    pedestal_base = cq.Workplane("XY").box(0.42, 0.82, 0.28).translate((0.78, 0.0, 0.40))
    left_cheek = cq.Workplane("XY").box(0.28, 0.18, 0.62).translate((1.05, 0.24, 0.59))
    right_cheek = cq.Workplane("XY").box(0.28, 0.18, 0.62).translate((1.05, -0.24, 0.59))
    boom_slot = cq.Workplane("XY").box(0.42, 0.30, 0.74).translate((1.00, 0.0, 0.44))

    body = (
        slew_ring.union(turntable)
        .union(front_deck)
        .union(upper_housing)
        .union(rear_weight)
        .union(pedestal_base)
        .union(left_cheek)
        .union(right_cheek)
    )
    return body.cut(boom_slot).clean()


def _make_cab_shape() -> cq.Workplane:
    cab = cq.Workplane("XY").box(0.82, 0.74, 1.14).translate((0.42, 0.50, 0.71))
    windshield_cut = (
        cq.Workplane("XY")
        .box(0.72, 0.90, 1.00)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
        .translate((0.86, 0.50, 1.02))
    )
    roof = cq.Workplane("XY").box(0.90, 0.82, 0.06).translate((0.38, 0.50, 1.30))
    rear_cut = cq.Workplane("XY").box(0.40, 0.90, 0.80).translate((0.06, 0.50, 1.02))

    return cab.cut(windshield_cut).cut(rear_cut).union(roof).clean()


def _make_boom_shape() -> cq.Workplane:
    boom_profile = [
        (0.00, -0.10),
        (0.24, -0.14),
        (0.96, -0.08),
        (1.82, 0.08),
        (2.06, 0.11),
        (2.12, 0.19),
        (1.74, 0.35),
        (0.78, 0.32),
        (0.18, 0.18),
    ]
    main_body = _centered_xz_extrusion(boom_profile, 0.30)
    root_boss = _axis_cylinder(0.09, 0.46, axis="y").translate((0.09, 0.0, 0.00))
    root_cheek = cq.Workplane("XY").box(0.20, 0.38, 0.18).translate((0.10, 0.0, 0.00))
    mid_bulge = cq.Workplane("XY").box(0.24, 0.22, 0.10).translate((1.02, 0.0, 0.05))
    tip_block = cq.Workplane("XY").box(0.18, 0.30, 0.16).translate((2.11, 0.0, 0.14))

    return main_body.union(root_boss).union(root_cheek).union(mid_bulge).union(tip_block).clean()


def _make_stick_shape() -> cq.Workplane:
    stick_profile = [
        (0.00, -0.10),
        (0.22, -0.12),
        (0.76, -0.08),
        (1.02, -0.03),
        (1.12, 0.00),
        (1.10, 0.11),
        (0.60, 0.18),
        (0.10, 0.15),
    ]
    main_arm = _centered_xz_extrusion(stick_profile, 0.20)
    base_boss = _axis_cylinder(0.08, 0.20, axis="y").translate((0.08, 0.0, 0.00))
    base_cheek = cq.Workplane("XY").box(0.18, 0.20, 0.18).translate((0.09, 0.0, 0.00))
    mid_web = cq.Workplane("XY").box(0.18, 0.14, 0.08).translate((0.74, 0.0, 0.07))
    tip_block = cq.Workplane("XY").box(0.16, 0.20, 0.18).translate((1.10, 0.0, 0.02))

    return main_arm.union(base_boss).union(base_cheek).union(mid_web).union(tip_block).clean()


def _make_bucket_shape() -> cq.Workplane:
    side_profile = [
        (0.02, 0.13),
        (0.00, -0.02),
        (0.08, -0.14),
        (0.22, -0.28),
        (0.41, -0.42),
        (0.61, -0.35),
        (0.56, -0.10),
        (0.28, 0.10),
        (0.10, 0.16),
    ]
    left_side = _centered_xz_extrusion(side_profile, 0.03).translate((0.0, 0.155, 0.0))
    right_side = _centered_xz_extrusion(side_profile, 0.03).translate((0.0, -0.155, 0.0))

    rear_spine = cq.Workplane("XY").box(0.18, 0.34, 0.18).translate((0.08, 0.0, 0.03))
    hinge_collar = _axis_cylinder(0.07, 0.20, axis="y").translate((0.07, 0.0, 0.04))
    hinge_bridge = cq.Workplane("XY").box(0.16, 0.18, 0.12).translate((0.14, 0.0, 0.01))
    left_ear = cq.Workplane("XY").box(0.16, 0.08, 0.18).translate((0.12, 0.13, 0.05))
    right_ear = cq.Workplane("XY").box(0.16, 0.08, 0.18).translate((0.12, -0.13, 0.05))

    rear_floor = (
        cq.Workplane("XY")
        .box(0.22, 0.30, 0.08)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -8.0)
        .translate((0.14, 0.0, -0.03))
    )
    mid_floor = (
        cq.Workplane("XY")
        .box(0.28, 0.30, 0.08)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -24.0)
        .translate((0.31, 0.0, -0.16))
    )
    front_floor = (
        cq.Workplane("XY")
        .box(0.20, 0.30, 0.08)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -42.0)
        .translate((0.50, 0.0, -0.28))
    )
    cutting_edge = cq.Workplane("XY").box(0.10, 0.30, 0.06).translate((0.60, 0.0, -0.31))

    return (
        left_side.union(right_side)
        .union(rear_spine)
        .union(hinge_collar)
        .union(hinge_bridge)
        .union(left_ear)
        .union(right_ear)
        .union(rear_floor)
        .union(mid_floor)
        .union(front_floor)
        .union(cutting_edge)
        .clean()
        .translate((0.009, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tracked_excavator_upper_works")

    track_material = model.material("track_material", rgba=(0.18, 0.18, 0.18, 1.0))
    body_material = model.material("body_material", rgba=(0.93, 0.72, 0.16, 1.0))
    arm_material = model.material("arm_material", rgba=(0.96, 0.74, 0.18, 1.0))
    bucket_material = model.material("bucket_material", rgba=(0.33, 0.30, 0.28, 1.0))
    cab_material = model.material("cab_material", rgba=(0.52, 0.62, 0.72, 0.92))

    undercarriage = model.part("undercarriage")
    undercarriage.visual(
        mesh_from_cadquery(_checked_shape("undercarriage", _make_undercarriage_shape), "undercarriage"),
        material=track_material,
        name="undercarriage_body",
    )

    upper_house = model.part("upper_house")
    upper_house.visual(
        mesh_from_cadquery(_checked_shape("upper_house_body", _make_upper_house_body), "upper_house_body"),
        material=body_material,
        name="house_body",
    )
    upper_house.visual(
        mesh_from_cadquery(_checked_shape("upper_house_cab", _make_cab_shape), "upper_house_cab"),
        material=cab_material,
        name="cab",
    )

    boom = model.part("boom")
    boom.visual(
        mesh_from_cadquery(_checked_shape("boom", _make_boom_shape), "boom"),
        material=arm_material,
        name="boom_body",
    )

    stick = model.part("stick")
    stick.visual(
        mesh_from_cadquery(_checked_shape("stick", _make_stick_shape), "stick"),
        material=arm_material,
        name="stick_body",
    )

    bucket = model.part("bucket")
    bucket.visual(
        mesh_from_cadquery(_checked_shape("bucket", _make_bucket_shape), "bucket"),
        material=bucket_material,
        name="bucket_body",
    )

    model.articulation(
        "slew",
        ArticulationType.CONTINUOUS,
        parent=undercarriage,
        child=upper_house,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=22000.0, velocity=1.3),
    )

    model.articulation(
        "boom_shoulder",
        ArticulationType.REVOLUTE,
        parent=upper_house,
        child=boom,
        origin=Origin(xyz=(1.19, 0.0, 0.50)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.9, lower=-0.55, upper=1.10),
    )

    model.articulation(
        "stick_joint",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=stick,
        origin=Origin(xyz=(2.20, 0.0, 0.16)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14000.0, velocity=1.0, lower=-1.35, upper=0.45),
    )

    model.articulation(
        "bucket_joint",
        ArticulationType.REVOLUTE,
        parent=stick,
        child=bucket,
        origin=Origin(xyz=(1.18, 0.0, 0.01)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12000.0, velocity=1.2, lower=-1.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    undercarriage = object_model.get_part("undercarriage")
    upper_house = object_model.get_part("upper_house")
    boom = object_model.get_part("boom")
    stick = object_model.get_part("stick")
    bucket = object_model.get_part("bucket")

    slew = object_model.get_articulation("slew")
    boom_shoulder = object_model.get_articulation("boom_shoulder")
    stick_joint = object_model.get_articulation("stick_joint")
    bucket_joint = object_model.get_articulation("bucket_joint")

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
        "all excavator parts exist",
        all(part is not None for part in (undercarriage, upper_house, boom, stick, bucket)),
        details=f"parts={[p.name for p in (undercarriage, upper_house, boom, stick, bucket)]}",
    )
    ctx.check(
        "joint axes match excavator kinematics",
        slew.axis == (0.0, 0.0, 1.0)
        and boom_shoulder.axis == (0.0, -1.0, 0.0)
        and stick_joint.axis == (0.0, -1.0, 0.0)
        and bucket_joint.axis == (0.0, -1.0, 0.0),
        details=(
            f"slew={slew.axis}, boom={boom_shoulder.axis}, "
            f"stick={stick_joint.axis}, bucket={bucket_joint.axis}"
        ),
    )

    ctx.expect_contact(upper_house, undercarriage, name="upper house sits on slew pedestal")
    ctx.expect_contact(boom, upper_house, name="boom is supported by the front pedestal")
    ctx.expect_contact(stick, boom, name="stick is supported at the boom tip")
    ctx.expect_contact(bucket, stick, name="bucket is supported at the stick end")

    cab_at_rest = _aabb_center(ctx.part_element_world_aabb(upper_house, elem="cab"))
    with ctx.pose({slew: math.pi / 2.0}):
        cab_at_quarter_turn = _aabb_center(ctx.part_element_world_aabb(upper_house, elem="cab"))
    ctx.check(
        "slew rotates the offset cab around the vertical axis",
        cab_at_rest is not None
        and cab_at_quarter_turn is not None
        and cab_at_rest[0] > 0.20
        and cab_at_rest[1] > 0.20
        and cab_at_quarter_turn[0] < -0.20
        and cab_at_quarter_turn[1] > 0.20,
        details=f"rest={cab_at_rest}, quarter_turn={cab_at_quarter_turn}",
    )

    stick_rest = ctx.part_world_position(stick)
    with ctx.pose({boom_shoulder: 0.78}):
        stick_raised = ctx.part_world_position(stick)
    ctx.check(
        "boom shoulder raises the stick joint upward",
        stick_rest is not None and stick_raised is not None and stick_raised[2] > stick_rest[2] + 0.40,
        details=f"rest={stick_rest}, raised={stick_raised}",
    )

    with ctx.pose({boom_shoulder: 0.55, stick_joint: -1.10}):
        bucket_extended = _aabb_center(ctx.part_world_aabb(bucket))
    with ctx.pose({boom_shoulder: 0.55, stick_joint: 0.25}):
        bucket_tucked = _aabb_center(ctx.part_world_aabb(bucket))
    ctx.check(
        "stick joint folds the bucket upward toward the boom",
        bucket_extended is not None
        and bucket_tucked is not None
        and bucket_tucked[2] > bucket_extended[2] + 0.30
        and bucket_tucked[0] <= bucket_extended[0] + 0.05,
        details=f"extended={bucket_extended}, tucked={bucket_tucked}",
    )

    with ctx.pose({boom_shoulder: 0.42, stick_joint: -0.75, bucket_joint: -1.10}):
        bucket_dumped_aabb = ctx.part_world_aabb(bucket)
    with ctx.pose({boom_shoulder: 0.42, stick_joint: -0.75, bucket_joint: 0.35}):
        bucket_curled_aabb = ctx.part_world_aabb(bucket)
    ctx.check(
        "bucket joint curls the bucket back from a dumped attitude",
        bucket_dumped_aabb is not None
        and bucket_curled_aabb is not None
        and bucket_curled_aabb[0][2] > bucket_dumped_aabb[0][2] + 0.10,
        details=f"dumped={bucket_dumped_aabb}, curled={bucket_curled_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
