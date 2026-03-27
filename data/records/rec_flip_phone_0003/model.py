from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

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

WIDTH = 0.052
LOWER_LEN = 0.078
UPPER_LEN = 0.072
LOWER_THICKNESS = 0.016
UPPER_THICKNESS = 0.012
HINGE_RADIUS = 0.0044
HINGE_PIN_RADIUS = 0.0022
HINGE_SIDE_LENGTH = 0.009
HINGE_CENTER_LENGTH = 0.022
HINGE_GAP = 0.001
HERE = Path(__file__).parent
ASSETS = AssetContext(HERE)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamshell_flip_phone", assets=ASSETS)

    body_silver = model.material("body_silver", rgba=(0.73, 0.75, 0.78, 1.0))
    bezel_graphite = model.material("bezel_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    key_gray = model.material("key_gray", rgba=(0.34, 0.35, 0.38, 1.0))
    grille_black = model.material("grille_black", rgba=(0.08, 0.09, 0.10, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.28, 0.46, 0.58, 0.92))
    dark_glass = model.material("dark_glass", rgba=(0.13, 0.17, 0.20, 0.94))
    hinge_metal = model.material("hinge_metal", rgba=(0.56, 0.58, 0.61, 1.0))

    lower = model.part("lower_half")
    lower.visual(
        Box((WIDTH, LOWER_LEN, LOWER_THICKNESS * 0.72)),
        origin=Origin(xyz=(0.0, -LOWER_LEN * 0.5, -LOWER_THICKNESS * 0.64)),
        material=body_silver,
        name="lower_core",
    )
    lower.visual(
        Box((WIDTH * 0.94, LOWER_LEN * 0.94, LOWER_THICKNESS * 0.38)),
        origin=Origin(xyz=(0.0, -LOWER_LEN * 0.5, -LOWER_THICKNESS * 0.19)),
        material=body_silver,
        name="lower_shell",
    )
    lower.visual(
        Box((WIDTH * 0.84, 0.012, 0.0024)),
        origin=Origin(xyz=(0.0, -0.006, -LOWER_THICKNESS * 0.05)),
        material=body_silver,
        name="lower_hinge_bridge",
    )
    lower.visual(
        Box((0.041, 0.050, 0.0008)),
        origin=Origin(xyz=(0.0, -0.042, 0.0004)),
        material=bezel_graphite,
        name="keypad_bed",
    )

    key_centers_x = (-0.013, 0.0, 0.013)
    key_centers_y = (-0.022, -0.035, -0.048, -0.061)
    key_names = (
        "1",
        "2",
        "3",
        "4",
        "5",
        "6",
        "7",
        "8",
        "9",
        "star",
        "0",
        "hash",
    )
    key_index = 0
    for row_y in key_centers_y:
        for column_x in key_centers_x:
            lower.visual(
                Box((0.0098, 0.0078, 0.0010)),
                origin=Origin(xyz=(column_x, row_y, 0.0013)),
                material=key_gray,
                name=f"key_{key_names[key_index]}",
            )
            lower.visual(
                Box((0.0062, 0.0009, 0.00025)),
                origin=Origin(xyz=(column_x, row_y + 0.0015, 0.0017)),
                material=body_silver,
                name=f"key_texture_{key_names[key_index]}",
            )
            key_index += 1

    for index, x_center in enumerate((-0.009, -0.003, 0.003, 0.009), start=1):
        lower.visual(
            Box((0.0032, 0.0011, 0.00045)),
            origin=Origin(xyz=(x_center, -0.069, 0.00018)),
            material=grille_black,
            name=f"mic_slat_{index}",
        )

    hinge_side_offset = HINGE_CENTER_LENGTH * 0.5 + HINGE_GAP + HINGE_SIDE_LENGTH * 0.5
    lower.visual(
        Cylinder(radius=HINGE_PIN_RADIUS, length=HINGE_CENTER_LENGTH + 2.0 * HINGE_SIDE_LENGTH + 2.0 * HINGE_GAP),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="hinge_pin",
    )
    lower.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_SIDE_LENGTH),
        origin=Origin(xyz=(-hinge_side_offset, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="lower_hinge_left",
    )
    lower.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_SIDE_LENGTH),
        origin=Origin(xyz=(hinge_side_offset, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="lower_hinge_right",
    )
    lower.inertial = Inertial.from_geometry(
        Box((WIDTH, LOWER_LEN, LOWER_THICKNESS + HINGE_RADIUS * 0.9)),
        mass=0.11,
        origin=Origin(xyz=(0.0, -LOWER_LEN * 0.5, -LOWER_THICKNESS * 0.5)),
    )

    upper = model.part("upper_half")
    upper.visual(
        Box((WIDTH * 0.96, UPPER_LEN, UPPER_THICKNESS * 0.70)),
        origin=Origin(xyz=(0.0, UPPER_LEN * 0.5, -UPPER_THICKNESS * 0.63)),
        material=body_silver,
        name="upper_core",
    )
    upper.visual(
        Box((WIDTH * 0.90, UPPER_LEN * 0.94, UPPER_THICKNESS * 0.36)),
        origin=Origin(xyz=(0.0, UPPER_LEN * 0.5, -UPPER_THICKNESS * 0.18)),
        material=body_silver,
        name="upper_shell",
    )
    upper.visual(
        Box((WIDTH * 0.80, 0.010, 0.0022)),
        origin=Origin(xyz=(0.0, 0.005, -UPPER_THICKNESS * 0.04)),
        material=body_silver,
        name="upper_hinge_bridge",
    )
    upper.visual(
        Box((0.036, 0.029, 0.0009)),
        origin=Origin(xyz=(0.0, 0.031, 0.00045)),
        material=bezel_graphite,
        name="screen_bezel",
    )
    upper.visual(
        Box((0.031, 0.024, 0.0005)),
        origin=Origin(xyz=(0.0, 0.031, 0.00105)),
        material=glass_blue,
        name="screen",
    )
    for index, x_center in enumerate((-0.008, -0.003, 0.003, 0.008), start=1):
        upper.visual(
            Box((0.0030, 0.0010, 0.0004)),
            origin=Origin(xyz=(x_center, 0.063, 0.00018)),
            material=grille_black,
            name=f"earpiece_slat_{index}",
        )
    upper.visual(
        Box((0.022, 0.014, 0.0009)),
        origin=Origin(xyz=(0.0, 0.040, -UPPER_THICKNESS + 0.00065)),
        material=bezel_graphite,
        name="outer_display_trim",
    )
    upper.visual(
        Box((0.018, 0.010, 0.0005)),
        origin=Origin(xyz=(0.0, 0.040, -UPPER_THICKNESS + 0.0010)),
        material=dark_glass,
        name="outer_display",
    )
    upper.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_CENTER_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=hinge_metal,
        name="upper_barrel",
    )
    upper.inertial = Inertial.from_geometry(
        Box((WIDTH * 0.96, UPPER_LEN, UPPER_THICKNESS + HINGE_RADIUS * 0.8)),
        mass=0.08,
        origin=Origin(xyz=(0.0, UPPER_LEN * 0.5, -UPPER_THICKNESS * 0.5)),
    )

    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=4.0,
            lower=0.0,
            upper=math.pi,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    hinge = object_model.get_articulation("flip_hinge")

    lower_shell = lower.get_visual("lower_shell")
    keypad_bed = lower.get_visual("keypad_bed")
    key_5 = lower.get_visual("key_5")
    mic_slat = lower.get_visual("mic_slat_2")
    hinge_pin = lower.get_visual("hinge_pin")
    lower_hinge_bridge = lower.get_visual("lower_hinge_bridge")

    upper_shell = upper.get_visual("upper_shell")
    screen_bezel = upper.get_visual("screen_bezel")
    screen = upper.get_visual("screen")
    earpiece = upper.get_visual("earpiece_slat_2")
    outer_display_trim = upper.get_visual("outer_display_trim")
    outer_display = upper.get_visual("outer_display")
    upper_barrel = upper.get_visual("upper_barrel")
    upper_hinge_bridge = upper.get_visual("upper_hinge_bridge")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.allow_overlap(upper, lower, reason="upper hinge barrel rotates concentrically around the lower hinge pin")

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_within(lower, lower, axes="xy", inner_elem=keypad_bed, outer_elem=lower_shell)
    ctx.expect_within(lower, lower, axes="xy", inner_elem=key_5, outer_elem=keypad_bed)
    ctx.expect_gap(
        lower,
        lower,
        axis="z",
        positive_elem=key_5,
        negative_elem=keypad_bed,
        max_gap=0.0005,
        max_penetration=1e-6,
        name="key_5_is_seated_on_keypad_bed",
    )
    ctx.expect_within(lower, lower, axes="xy", inner_elem=mic_slat, outer_elem=lower_shell)
    ctx.expect_contact(lower, lower, elem_a=mic_slat, elem_b=lower_shell)
    ctx.expect_within(upper, upper, axes="xy", inner_elem=screen, outer_elem=screen_bezel)
    ctx.expect_contact(upper, upper, elem_a=screen, elem_b=screen_bezel)
    ctx.expect_within(upper, upper, axes="xy", inner_elem=earpiece, outer_elem=upper_shell)
    ctx.expect_contact(upper, upper, elem_a=earpiece, elem_b=upper_shell)
    ctx.expect_within(upper, upper, axes="xy", inner_elem=outer_display, outer_elem=outer_display_trim)
    ctx.expect_contact(upper, upper, elem_a=outer_display, elem_b=outer_display_trim)
    ctx.expect_contact(upper, lower, elem_a=upper_barrel, elem_b=hinge_pin)

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(upper, lower, elem_a=upper_hinge_bridge, elem_b=lower_hinge_bridge)
        ctx.expect_origin_distance(upper, lower, axes="x", max_dist=0.002)

    with ctx.pose({hinge: math.pi * 0.5}):
        ctx.expect_contact(upper, lower, elem_a=upper_barrel, elem_b=hinge_pin)

    with ctx.pose({hinge: math.pi}):
        ctx.expect_origin_distance(upper, lower, axes="xy", max_dist=0.005)
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            positive_elem=upper_shell,
            negative_elem=lower_shell,
            max_gap=0.001,
            max_penetration=0.001,
            name="closed_phone_shells_stack_compactly",
        )
        ctx.expect_overlap(upper, lower, axes="xy", min_overlap=0.04)
        ctx.expect_contact(upper, lower, elem_a=upper_barrel, elem_b=hinge_pin)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
