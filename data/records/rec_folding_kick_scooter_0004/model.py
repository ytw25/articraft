from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

WHEEL_RADIUS = 0.105
WHEEL_WIDTH = 0.038
DECK_LENGTH = 0.58
DECK_WIDTH = 0.148
DECK_THICKNESS = 0.028
DECK_TOP_Z = 0.068
DECK_CENTER_Z = DECK_TOP_Z - (DECK_THICKNESS / 2.0)
HINGE_X = 0.270
HINGE_Z = 0.215
FRONT_AXLE_LOCAL_X = 0.130
FRONT_AXLE_LOCAL_Z = -0.110
REAR_AXLE_X = -0.255
AXLE_Z = WHEEL_RADIUS
FENDER_PIVOT_X = -0.248
FENDER_PIVOT_Z = 0.154
FOLDED_STEM_ANGLE = -1.25
FOLDED_FENDER_ANGLE = 0.25


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_electric_scooter", assets=ASSETS)

    frame_dark = model.material("frame_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    silver = model.material("silver", rgba=(0.70, 0.72, 0.75, 1.0))
    battery_gray = model.material("battery_gray", rgba=(0.32, 0.34, 0.36, 1.0))
    accent = model.material("accent", rgba=(0.86, 0.50, 0.14, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=frame_dark,
        name="deck_body",
    )
    deck.visual(
        Box((0.52, 0.020, 0.004)),
        origin=Origin(xyz=(-0.01, 0.054, DECK_TOP_Z - 0.002)),
        material=charcoal,
        name="deck_left_rail",
    )
    deck.visual(
        Box((0.52, 0.020, 0.004)),
        origin=Origin(xyz=(-0.01, -0.054, DECK_TOP_Z - 0.002)),
        material=charcoal,
        name="deck_right_rail",
    )
    deck.visual(
        Box((0.055, 0.108, 0.004)),
        origin=Origin(xyz=(0.252, 0.0, DECK_TOP_Z - 0.002)),
        material=charcoal,
        name="deck_front_rail",
    )
    deck.visual(
        Box((0.055, 0.108, 0.004)),
        origin=Origin(xyz=(-0.252, 0.0, DECK_TOP_Z - 0.002)),
        material=charcoal,
        name="deck_rear_rail",
    )
    deck.visual(
        Box((0.39, 0.104, 0.004)),
        origin=Origin(xyz=(-0.015, 0.0, DECK_TOP_Z - 0.007)),
        material=battery_gray,
        name="battery_panel",
    )
    deck.visual(
        Box((0.095, 0.106, 0.060)),
        origin=Origin(xyz=(0.200, 0.0, 0.096)),
        material=frame_dark,
        name="neck_base",
    )
    deck.visual(
        Box((0.060, 0.074, 0.112)),
        origin=Origin(xyz=(0.214, 0.0, 0.134)),
        material=frame_dark,
        name="neck_housing",
    )
    deck.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.260, 0.037, 0.205), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hinge_mount_left",
    )
    deck.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.260, -0.037, 0.205), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hinge_mount_right",
    )
    deck.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.260, 0.048, 0.205), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hinge_pin_cap_left",
    )
    deck.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.260, -0.048, 0.205), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hinge_pin_cap_right",
    )
    deck.visual(
        Box((0.062, 0.016, 0.100)),
        origin=Origin(xyz=(REAR_AXLE_X + 0.014, 0.046, 0.086)),
        material=frame_dark,
        name="rear_standoff_left",
    )
    deck.visual(
        Box((0.062, 0.016, 0.100)),
        origin=Origin(xyz=(REAR_AXLE_X + 0.014, -0.046, 0.086)),
        material=frame_dark,
        name="rear_standoff_right",
    )
    deck.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(REAR_AXLE_X, 0.056, AXLE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_axle_stub_left",
    )
    deck.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(REAR_AXLE_X, -0.056, AXLE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_axle_stub_right",
    )
    deck.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, AXLE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_wheel",
    )
    deck.visual(
        Cylinder(radius=0.052, length=0.022),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, AXLE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_hub",
    )
    deck.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(
            xyz=(FENDER_PIVOT_X, 0.040, FENDER_PIVOT_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=silver,
        name="fender_pivot_mount_left",
    )
    deck.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(
            xyz=(FENDER_PIVOT_X, -0.040, FENDER_PIVOT_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=silver,
        name="fender_pivot_mount_right",
    )
    deck.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(
            xyz=(FENDER_PIVOT_X, 0.052, FENDER_PIVOT_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=silver,
        name="fender_pivot_cap_left",
    )
    deck.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(
            xyz=(FENDER_PIVOT_X, -0.052, FENDER_PIVOT_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=silver,
        name="fender_pivot_cap_right",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.72, 0.16, 0.24)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.0, 0.019, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hinge_knuckle_left",
    )
    stem.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hinge_knuckle_right",
    )
    stem.visual(
        Box((0.050, 0.040, 0.046)),
        origin=Origin(xyz=(0.012, 0.0, 0.020)),
        material=silver,
        name="hinge_block",
    )
    stem.visual(
        Cylinder(radius=0.020, length=0.68),
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        material=frame_dark,
        name="stem_tube",
    )
    stem.visual(
        Box((0.060, 0.060, 0.070)),
        origin=Origin(xyz=(0.040, 0.0, -0.020)),
        material=frame_dark,
        name="fork_crown",
    )
    stem.visual(
        Cylinder(radius=0.009, length=0.128),
        origin=Origin(xyz=(0.088, 0.028, -0.066), rpy=(0.0, -0.72, 0.0)),
        material=frame_dark,
        name="fork_left",
    )
    stem.visual(
        Cylinder(radius=0.009, length=0.128),
        origin=Origin(xyz=(0.088, -0.028, -0.066), rpy=(0.0, -0.72, 0.0)),
        material=frame_dark,
        name="fork_right",
    )
    stem.visual(
        Cylinder(radius=0.014, length=0.52),
        origin=Origin(xyz=(0.010, 0.0, 0.690), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=frame_dark,
        name="handlebar",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.100),
        origin=Origin(xyz=(0.010, 0.205, 0.690), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="left_grip",
    )
    stem.visual(
        Cylinder(radius=0.017, length=0.100),
        origin=Origin(xyz=(0.010, -0.205, 0.690), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="right_grip",
    )
    stem.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(
            xyz=(FRONT_AXLE_LOCAL_X, 0.0, FRONT_AXLE_LOCAL_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="front_wheel",
    )
    stem.visual(
        Cylinder(radius=0.058, length=0.022),
        origin=Origin(
            xyz=(FRONT_AXLE_LOCAL_X, 0.0, FRONT_AXLE_LOCAL_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=silver,
        name="hub_motor",
    )
    stem.visual(
        Cylinder(radius=0.007, length=0.082),
        origin=Origin(
            xyz=(FRONT_AXLE_LOCAL_X, 0.0, FRONT_AXLE_LOCAL_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=silver,
        name="front_axle",
    )
    stem.visual(
        Box((0.102, 0.018, 0.040)),
        origin=Origin(xyz=(-0.050, 0.0, 0.440)),
        material=silver,
        name="latch_bracket",
    )
    stem.visual(
        Cylinder(radius=0.0045, length=0.056),
        origin=Origin(xyz=(-0.118, 0.0, 0.444), rpy=(0.0, 0.92, 0.0)),
        material=accent,
        name="latch_hook",
    )
    stem.visual(
        Box((0.012, 0.010, 0.032)),
        origin=Origin(xyz=(-0.104, 0.0, 0.428)),
        material=accent,
        name="latch_drop",
    )
    stem.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.84),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
    )

    fender = model.part("rear_fender")
    fender.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.0, 0.031, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="pivot_ear_left",
    )
    fender.visual(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(0.0, -0.031, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="pivot_ear_right",
    )
    fender.visual(
        Box((0.020, 0.016, 0.074)),
        origin=Origin(xyz=(-0.008, 0.031, 0.040)),
        material=frame_dark,
        name="support_left",
    )
    fender.visual(
        Box((0.020, 0.016, 0.074)),
        origin=Origin(xyz=(-0.008, -0.031, 0.040)),
        material=frame_dark,
        name="support_right",
    )
    fender.visual(
        Box((0.056, 0.082, 0.006)),
        origin=Origin(xyz=(0.014, 0.0, 0.072), rpy=(0.0, -0.40, 0.0)),
        material=charcoal,
        name="fender_front",
    )
    fender.visual(
        Box((0.090, 0.086, 0.006)),
        origin=Origin(xyz=(-0.042, 0.0, 0.082), rpy=(0.0, 0.10, 0.0)),
        material=charcoal,
        name="fender_crown",
    )
    fender.visual(
        Box((0.078, 0.086, 0.006)),
        origin=Origin(xyz=(-0.096, 0.0, 0.060), rpy=(0.0, 0.44, 0.0)),
        material=charcoal,
        name="fender_rear",
    )
    fender.visual(
        Box((0.050, 0.020, 0.020)),
        origin=Origin(xyz=(-0.072, 0.0, 0.070)),
        material=charcoal,
        name="rear_join",
    )
    fender.visual(
        Box((0.036, 0.082, 0.006)),
        origin=Origin(xyz=(-0.132, 0.0, 0.010), rpy=(0.0, 0.88, 0.0)),
        material=charcoal,
        name="tail_flap",
    )
    fender.visual(
        Box((0.072, 0.024, 0.040)),
        origin=Origin(xyz=(-0.112, 0.0, 0.040)),
        material=charcoal,
        name="tail_join",
    )
    fender.visual(
        Box((0.032, 0.050, 0.012)),
        origin=Origin(xyz=(0.048, 0.0, 0.060)),
        material=accent,
        name="fender_catch",
    )
    fender.visual(
        Box((0.038, 0.020, 0.020)),
        origin=Origin(xyz=(0.028, 0.0, 0.066)),
        material=accent,
        name="catch_arm",
    )
    fender.inertial = Inertial.from_geometry(
        Box((0.20, 0.10, 0.08)),
        mass=0.9,
        origin=Origin(xyz=(-0.050, 0.0, 0.030)),
    )

    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.0,
            lower=FOLDED_STEM_ANGLE,
            upper=0.0,
        ),
    )
    model.articulation(
        "rear_fender_pivot",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=fender,
        origin=Origin(xyz=(FENDER_PIVOT_X, 0.0, FENDER_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.04,
            upper=FOLDED_FENDER_ANGLE,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    fender = object_model.get_part("rear_fender")
    stem_fold = object_model.get_articulation("stem_fold")
    fender_pivot = object_model.get_articulation("rear_fender_pivot")

    deck_body = deck.get_visual("deck_body")
    battery_panel = deck.get_visual("battery_panel")
    deck_left_rail = deck.get_visual("deck_left_rail")
    hinge_mount_left = deck.get_visual("hinge_mount_left")
    hinge_mount_right = deck.get_visual("hinge_mount_right")
    pivot_mount_left = deck.get_visual("fender_pivot_mount_left")
    pivot_mount_right = deck.get_visual("fender_pivot_mount_right")
    rear_wheel = deck.get_visual("rear_wheel")
    front_wheel = stem.get_visual("front_wheel")
    hub_motor = stem.get_visual("hub_motor")
    stem_tube = stem.get_visual("stem_tube")
    handlebar = stem.get_visual("handlebar")
    hinge_knuckle_left = stem.get_visual("hinge_knuckle_left")
    hinge_knuckle_right = stem.get_visual("hinge_knuckle_right")
    latch_hook = stem.get_visual("latch_hook")
    pivot_ear_left = fender.get_visual("pivot_ear_left")
    pivot_ear_right = fender.get_visual("pivot_ear_right")
    fender_crown = fender.get_visual("fender_crown")
    fender_catch = fender.get_visual("fender_catch")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        deck,
        fender,
        reason="rear mudguard hinge sleeves are simplified as solid pivot ears nesting around pivot mounts",
    )
    ctx.allow_overlap(
        stem,
        fender,
        reason="folding latch hook slightly nests into the rear catch arm at the transport-lock pose",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(
        deck,
        deck,
        axes="xy",
        inner_elem=battery_panel,
        outer_elem=deck_body,
    )
    ctx.expect_gap(
        deck,
        deck,
        axis="z",
        positive_elem=deck_left_rail,
        negative_elem=battery_panel,
        min_gap=0.0005,
        max_gap=0.003,
    )
    ctx.expect_within(
        stem,
        stem,
        axes="xz",
        inner_elem=hub_motor,
        outer_elem=front_wheel,
    )
    ctx.expect_contact(stem, deck, elem_a=hinge_knuckle_left, elem_b=hinge_mount_left)
    ctx.expect_contact(stem, deck, elem_a=hinge_knuckle_right, elem_b=hinge_mount_right)
    ctx.expect_contact(fender, deck, elem_a=pivot_ear_left, elem_b=pivot_mount_left)
    ctx.expect_contact(fender, deck, elem_a=pivot_ear_right, elem_b=pivot_mount_right)
    ctx.expect_gap(
        stem,
        deck,
        axis="x",
        positive_elem=front_wheel,
        negative_elem=deck_body,
        min_gap=0.004,
        max_gap=0.030,
    )
    ctx.expect_overlap(
        fender,
        deck,
        axes="xy",
        elem_a=fender_crown,
        elem_b=rear_wheel,
        min_overlap=0.020,
    )
    ctx.expect_gap(
        fender,
        deck,
        axis="z",
        positive_elem=fender_crown,
        negative_elem=rear_wheel,
        min_gap=0.004,
        max_gap=0.040,
    )

    with ctx.pose({stem_fold: FOLDED_STEM_ANGLE}):
        ctx.expect_overlap(
            stem,
            deck,
            axes="xy",
            elem_a=stem_tube,
            elem_b=deck_body,
            min_overlap=0.040,
        )
        ctx.expect_gap(
            stem,
            deck,
            axis="z",
            positive_elem=stem_tube,
            negative_elem=deck_body,
            min_gap=0.015,
            max_gap=0.160,
        )

    with ctx.pose({stem_fold: FOLDED_STEM_ANGLE, fender_pivot: FOLDED_FENDER_ANGLE}):
        ctx.expect_overlap(
            stem,
            fender,
            axes="xy",
            elem_a=latch_hook,
            elem_b=fender_catch,
            min_overlap=0.003,
        )
        ctx.expect_gap(
            stem,
            fender,
            axis="z",
            positive_elem=latch_hook,
            negative_elem=fender_catch,
            max_gap=0.015,
            max_penetration=0.003,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
