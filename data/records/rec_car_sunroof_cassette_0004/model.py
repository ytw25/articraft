from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

OUTER_X = 0.98
OUTER_Y = 0.82
FLANGE_T = 0.012
OPENING_X = 0.70
OPENING_Y = 0.56

SIDE_FLANGE_Y = (OUTER_Y - OPENING_Y) / 2.0
END_FLANGE_X = (OUTER_X - OPENING_X) / 2.0
TRACK_Y = OPENING_Y / 2.0 - 0.025

TRACK_X = 0.74
TRACK_W = 0.050
TRACK_H = 0.012

GLASS_X = 0.76
GLASS_Y = 0.62
GLASS_T = 0.016
GLASS_SLIDE_TRAVEL = 0.21
GLASS_TILT_ANGLE = 0.06

HOUSING_X = 0.14
HOUSING_Y = 0.72
HOUSING_H = 0.056
BLIND_WIDTH = 0.53
BLIND_LENGTH = 0.63
BLIND_T = 0.0016
BLIND_TRAVEL = 0.18


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_slide_glass_blind_cassette")

    frame_trim = model.material("frame_trim", rgba=(0.30, 0.31, 0.34, 1.0))
    frame_dark = model.material("frame_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    guide_dark = model.material("guide_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    hardware = model.material("hardware", rgba=(0.45, 0.47, 0.49, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.47, 0.61, 0.69, 0.35))
    seal_dark = model.material("seal_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    blind_fabric = model.material("blind_fabric", rgba=(0.82, 0.82, 0.79, 1.0))
    blind_bar = model.material("blind_bar", rgba=(0.61, 0.61, 0.59, 1.0))
    roller_dark = model.material("roller_dark", rgba=(0.08, 0.08, 0.09, 1.0))

    roof = model.part("roof_frame")
    roof.inertial = Inertial.from_geometry(
        Box((OUTER_X, OUTER_Y, 0.12)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
    )
    roof.visual(
        Box((END_FLANGE_X, OUTER_Y, FLANGE_T)),
        origin=Origin(xyz=(-(OPENING_X + END_FLANGE_X) / 2.0, 0.0, FLANGE_T / 2.0)),
        material=frame_trim,
        name="front_flange",
    )
    roof.visual(
        Box((END_FLANGE_X, OUTER_Y, FLANGE_T)),
        origin=Origin(xyz=((OPENING_X + END_FLANGE_X) / 2.0, 0.0, FLANGE_T / 2.0)),
        material=frame_trim,
        name="rear_flange",
    )
    roof.visual(
        Box((OPENING_X, SIDE_FLANGE_Y, FLANGE_T)),
        origin=Origin(xyz=(0.0, (OPENING_Y + SIDE_FLANGE_Y) / 2.0, FLANGE_T / 2.0)),
        material=frame_trim,
        name="left_flange",
    )
    roof.visual(
        Box((OPENING_X, SIDE_FLANGE_Y, FLANGE_T)),
        origin=Origin(xyz=(0.0, -(OPENING_Y + SIDE_FLANGE_Y) / 2.0, FLANGE_T / 2.0)),
        material=frame_trim,
        name="right_flange",
    )
    roof.visual(
        Box((0.040, OPENING_Y, 0.020)),
        origin=Origin(xyz=(-0.390, 0.0, -0.010)),
        material=frame_dark,
        name="front_bridge",
    )
    roof.visual(
        Box((0.040, OPENING_Y, 0.020)),
        origin=Origin(xyz=(0.450, 0.0, -0.010)),
        material=frame_dark,
        name="rear_bridge",
    )
    roof.visual(
        Box((TRACK_X, TRACK_W, TRACK_H)),
        origin=Origin(xyz=(0.060, TRACK_Y, -0.010)),
        material=guide_dark,
        name="left_track",
    )
    roof.visual(
        Box((TRACK_X, TRACK_W, TRACK_H)),
        origin=Origin(xyz=(0.060, -TRACK_Y, -0.010)),
        material=guide_dark,
        name="right_track",
    )
    roof.visual(
        Box((0.030, 0.080, 0.026)),
        origin=Origin(xyz=(0.410, TRACK_Y, -0.019)),
        material=frame_dark,
        name="left_rear_stop",
    )
    roof.visual(
        Box((0.030, 0.080, 0.026)),
        origin=Origin(xyz=(0.410, -TRACK_Y, -0.019)),
        material=frame_dark,
        name="right_rear_stop",
    )

    carriage = model.part("glass_carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((0.58, 0.56, 0.05)),
        mass=2.0,
        origin=Origin(xyz=(0.28, 0.0, -0.008)),
    )
    carriage.visual(
        Box((0.10, 0.038, 0.016)),
        origin=Origin(xyz=(0.150, TRACK_Y, -0.024)),
        material=hardware,
        name="left_shoe",
    )
    carriage.visual(
        Box((0.10, 0.038, 0.016)),
        origin=Origin(xyz=(0.150, -TRACK_Y, -0.024)),
        material=hardware,
        name="right_shoe",
    )
    carriage.visual(
        Box((0.020, OPENING_Y - 0.200, 0.012)),
        origin=Origin(xyz=(0.040, 0.0, -0.025)),
        material=guide_dark,
        name="front_crossmember",
    )
    carriage.visual(
        Box((0.110, 0.036, 0.010)),
        origin=Origin(xyz=(0.095, 0.218, -0.026)),
        material=hardware,
        name="left_side_beam",
    )
    carriage.visual(
        Box((0.110, 0.036, 0.010)),
        origin=Origin(xyz=(0.095, -0.218, -0.026)),
        material=hardware,
        name="right_side_beam",
    )
    carriage.visual(
        Box((0.020, 0.040, 0.040)),
        origin=Origin(xyz=(0.040, 0.180, -0.005)),
        material=hardware,
        name="left_riser",
    )
    carriage.visual(
        Box((0.020, 0.040, 0.040)),
        origin=Origin(xyz=(0.040, -0.180, -0.005)),
        material=hardware,
        name="right_riser",
    )
    carriage.visual(
        Box((0.040, 0.060, 0.006)),
        origin=Origin(xyz=(0.020, 0.180, 0.015)),
        material=hardware,
        name="left_hinge_pad",
    )
    carriage.visual(
        Box((0.040, 0.060, 0.006)),
        origin=Origin(xyz=(0.020, -0.180, 0.015)),
        material=hardware,
        name="right_hinge_pad",
    )
    glass = model.part("glass_panel")
    glass.inertial = Inertial.from_geometry(
        Box((GLASS_X, GLASS_Y, 0.04)),
        mass=7.0,
        origin=Origin(xyz=(GLASS_X / 2.0, 0.0, 0.020)),
    )
    glass.visual(
        Box((GLASS_X, GLASS_Y, GLASS_T)),
        origin=Origin(xyz=(GLASS_X / 2.0, 0.0, GLASS_T / 2.0)),
        material=glass_tint,
        name="glass_lite",
    )
    glass.visual(
        Box((0.050, GLASS_Y - 0.040, 0.004)),
        origin=Origin(xyz=(0.025, 0.0, 0.002)),
        material=seal_dark,
        name="front_trim",
    )
    glass.visual(
        Box((0.040, GLASS_Y - 0.030, 0.004)),
        origin=Origin(xyz=(GLASS_X - 0.020, 0.0, 0.002)),
        material=seal_dark,
        name="rear_trim",
    )
    glass.visual(
        Box((GLASS_X - 0.090, 0.018, 0.004)),
        origin=Origin(xyz=(GLASS_X / 2.0, GLASS_Y / 2.0 - 0.009, 0.002)),
        material=seal_dark,
        name="left_trim",
    )
    glass.visual(
        Box((GLASS_X - 0.090, 0.018, 0.004)),
        origin=Origin(xyz=(GLASS_X / 2.0, -(GLASS_Y / 2.0 - 0.009), 0.002)),
        material=seal_dark,
        name="right_trim",
    )
    glass.visual(
        Box((0.040, 0.060, 0.004)),
        origin=Origin(xyz=(0.020, 0.180, 0.002)),
        material=hardware,
        name="left_hinge_plate",
    )
    glass.visual(
        Box((0.040, 0.060, 0.004)),
        origin=Origin(xyz=(0.020, -0.180, 0.002)),
        material=hardware,
        name="right_hinge_plate",
    )

    blind_cassette = model.part("blind_cassette")
    blind_cassette.inertial = Inertial.from_geometry(
        Box((HOUSING_X, HOUSING_Y, HOUSING_H)),
        mass=3.0,
        origin=Origin(xyz=(-HOUSING_X / 2.0, 0.0, -HOUSING_H / 2.0)),
    )
    blind_cassette.visual(
        Box((HOUSING_X, HOUSING_Y, 0.004)),
        origin=Origin(xyz=(-HOUSING_X / 2.0, 0.0, -0.002)),
        material=frame_dark,
        name="housing_top",
    )
    blind_cassette.visual(
        Box((0.010, HOUSING_Y, HOUSING_H)),
        origin=Origin(xyz=(-HOUSING_X + 0.005, 0.0, -HOUSING_H / 2.0)),
        material=frame_dark,
        name="housing_front_wall",
    )
    blind_cassette.visual(
        Box((HOUSING_X, 0.010, HOUSING_H)),
        origin=Origin(xyz=(-HOUSING_X / 2.0, HOUSING_Y / 2.0 - 0.005, -HOUSING_H / 2.0)),
        material=frame_dark,
        name="housing_left_wall",
    )
    blind_cassette.visual(
        Box((HOUSING_X, 0.010, HOUSING_H)),
        origin=Origin(xyz=(-HOUSING_X / 2.0, -(HOUSING_Y / 2.0 - 0.005), -HOUSING_H / 2.0)),
        material=frame_dark,
        name="housing_right_wall",
    )
    blind_cassette.visual(
        Box((0.010, HOUSING_Y, HOUSING_H)),
        origin=Origin(xyz=(-0.005, 0.0, -HOUSING_H / 2.0)),
        material=frame_dark,
        name="housing_back_wall",
    )
    blind_cassette.visual(
        Box((0.024, 0.620, 0.008)),
        origin=Origin(xyz=(-0.017, 0.0, -0.0498)),
        material=frame_dark,
        name="slot_lip",
    )
    blind_cassette.visual(
        Cylinder(radius=0.024, length=0.700),
        origin=Origin(xyz=(-0.085, 0.0, -0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_dark,
        name="spring_dowel",
    )

    blind = model.part("roller_blind")
    blind.inertial = Inertial.from_geometry(
        Box((BLIND_LENGTH + 0.024, BLIND_WIDTH + 0.050, 0.020)),
        mass=1.6,
        origin=Origin(xyz=((BLIND_LENGTH + 0.024) / 2.0, 0.0, -0.056)),
    )
    blind.visual(
        Box((BLIND_LENGTH, BLIND_WIDTH, BLIND_T)),
        origin=Origin(xyz=(BLIND_LENGTH / 2.0, 0.0, -0.0546)),
        material=blind_fabric,
        name="blind_fabric",
    )
    blind.visual(
        Box((0.024, BLIND_WIDTH + 0.050, 0.018)),
        origin=Origin(xyz=(BLIND_LENGTH + 0.012, 0.0, -0.060)),
        material=blind_bar,
        name="lead_bar",
    )
    blind.visual(
        Box((0.012, 0.140, 0.012)),
        origin=Origin(xyz=(BLIND_LENGTH + 0.014, 0.0, -0.072)),
        material=blind_bar,
        name="pull_tab",
    )

    model.articulation(
        "glass_slide",
        ArticulationType.PRISMATIC,
        parent=roof,
        child=carriage,
        origin=Origin(xyz=(-0.340, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=GLASS_SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "glass_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=glass,
        origin=Origin(xyz=(0.020, 0.0, 0.016)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.0,
            lower=0.0,
            upper=GLASS_TILT_ANGLE,
        ),
    )
    model.articulation(
        "blind_cassette_mount",
        ArticulationType.FIXED,
        parent=roof,
        child=blind_cassette,
        origin=Origin(xyz=(-0.350, 0.0, 0.0)),
    )
    model.articulation(
        "blind_pull",
        ArticulationType.PRISMATIC,
        parent=blind_cassette,
        child=blind,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.45,
            lower=0.0,
            upper=BLIND_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    roof = object_model.get_part("roof_frame")
    carriage = object_model.get_part("glass_carriage")
    glass = object_model.get_part("glass_panel")
    blind_cassette = object_model.get_part("blind_cassette")
    blind = object_model.get_part("roller_blind")
    glass_slide = object_model.get_articulation("glass_slide")
    glass_tilt = object_model.get_articulation("glass_tilt")
    blind_pull = object_model.get_articulation("blind_pull")

    front_flange = roof.get_visual("front_flange")
    rear_flange = roof.get_visual("rear_flange")
    left_track = roof.get_visual("left_track")
    right_track = roof.get_visual("right_track")
    front_bridge = roof.get_visual("front_bridge")
    rear_bridge = roof.get_visual("rear_bridge")

    left_shoe = carriage.get_visual("left_shoe")
    right_shoe = carriage.get_visual("right_shoe")
    left_hinge_pad = carriage.get_visual("left_hinge_pad")
    right_hinge_pad = carriage.get_visual("right_hinge_pad")

    glass_lite = glass.get_visual("glass_lite")
    rear_trim = glass.get_visual("rear_trim")
    left_hinge_plate = glass.get_visual("left_hinge_plate")
    right_hinge_plate = glass.get_visual("right_hinge_plate")

    housing_top = blind_cassette.get_visual("housing_top")
    slot_lip = blind_cassette.get_visual("slot_lip")
    spring_dowel = blind_cassette.get_visual("spring_dowel")

    blind_fabric = blind.get_visual("blind_fabric")
    lead_bar = blind.get_visual("lead_bar")
    pull_tab = blind.get_visual("pull_tab")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        roof,
        blind_cassette,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=front_flange,
        negative_elem=housing_top,
    )
    ctx.expect_overlap(
        blind_cassette,
        roof,
        axes="xy",
        min_overlap=0.08,
        elem_a=housing_top,
        elem_b=front_flange,
    )
    ctx.expect_within(blind_cassette, roof, axes="y", inner_elem=spring_dowel, outer_elem=front_flange)

    ctx.expect_contact(carriage, roof, elem_a=left_shoe, elem_b=left_track)
    ctx.expect_contact(carriage, roof, elem_a=right_shoe, elem_b=right_track)
    ctx.expect_within(carriage, roof, axes="xy", inner_elem=left_shoe, outer_elem=left_track)
    ctx.expect_within(carriage, roof, axes="xy", inner_elem=right_shoe, outer_elem=right_track)

    ctx.expect_contact(glass, carriage, elem_a=left_hinge_plate, elem_b=left_hinge_pad)
    ctx.expect_contact(glass, carriage, elem_a=right_hinge_plate, elem_b=right_hinge_pad)
    ctx.expect_overlap(glass, roof, axes="xy", min_overlap=0.26, elem_a=glass_lite)
    ctx.expect_gap(
        glass,
        roof,
        axis="z",
        min_gap=0.003,
        max_gap=0.028,
        positive_elem=glass_lite,
        negative_elem=front_flange,
    )

    ctx.expect_gap(
        glass,
        blind,
        axis="z",
        min_gap=0.060,
        max_gap=0.100,
        positive_elem=glass_lite,
        negative_elem=blind_fabric,
    )
    ctx.expect_gap(
        blind_cassette,
        blind,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=slot_lip,
        negative_elem=blind_fabric,
    )
    ctx.expect_within(blind, blind_cassette, axes="y", inner_elem=blind_fabric, outer_elem=slot_lip)
    ctx.expect_overlap(blind, roof, axes="xy", min_overlap=0.20, elem_a=blind_fabric)
    ctx.expect_gap(
        roof,
        blind,
        axis="x",
        max_gap=0.060,
        max_penetration=0.0,
        positive_elem=rear_flange,
        negative_elem=lead_bar,
    )

    with ctx.pose({glass_tilt: GLASS_TILT_ANGLE}):
        ctx.expect_contact(glass, carriage, elem_a=left_hinge_plate, elem_b=left_hinge_pad)
        ctx.expect_contact(glass, carriage, elem_a=right_hinge_plate, elem_b=right_hinge_pad)
        ctx.expect_gap(
            glass,
            roof,
            axis="z",
            min_gap=0.030,
            max_gap=0.070,
            positive_elem=rear_trim,
            negative_elem=rear_bridge,
        )

    with ctx.pose({glass_slide: GLASS_SLIDE_TRAVEL}):
        ctx.expect_contact(carriage, roof, elem_a=left_shoe, elem_b=left_track)
        ctx.expect_contact(carriage, roof, elem_a=right_shoe, elem_b=right_track)
        ctx.expect_within(carriage, roof, axes="xy", inner_elem=left_shoe, outer_elem=left_track)
        ctx.expect_within(carriage, roof, axes="xy", inner_elem=right_shoe, outer_elem=right_track)
        ctx.expect_gap(
            glass,
            roof,
            axis="x",
            min_gap=0.18,
            max_gap=0.26,
            positive_elem=glass_lite,
            negative_elem=front_flange,
        )
        ctx.expect_overlap(glass, roof, axes="xy", min_overlap=0.10, elem_a=glass_lite)

    with ctx.pose({blind_pull: BLIND_TRAVEL}):
        ctx.expect_gap(
            blind,
            roof,
            axis="x",
            min_gap=-0.02,
            max_gap=0.08,
            max_penetration=0.02,
            positive_elem=lead_bar,
            negative_elem=rear_bridge,
        )
        ctx.expect_overlap(blind, roof, axes="xy", min_overlap=0.24, elem_a=blind_fabric)

    with ctx.pose({glass_slide: GLASS_SLIDE_TRAVEL, blind_pull: BLIND_TRAVEL}):
        ctx.expect_gap(
            glass,
            blind,
            axis="z",
            min_gap=0.060,
            max_gap=0.100,
            positive_elem=glass_lite,
            negative_elem=blind_fabric,
        )
        ctx.expect_overlap(glass, blind, axes="xy", min_overlap=0.08, elem_a=glass_lite, elem_b=blind_fabric)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
