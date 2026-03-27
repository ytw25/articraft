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

ASSETS = AssetContext.from_script("/tmp/model.py")

BODY_SIZE = (0.320, 0.220, 0.048)
BODY_WIDTH = BODY_SIZE[0]
BODY_DEPTH = BODY_SIZE[1]
BODY_HALF_WIDTH = BODY_WIDTH * 0.5
BODY_HALF_DEPTH = BODY_DEPTH * 0.5
BODY_FRONT_Y = -BODY_HALF_DEPTH

FLOOR_Z = 0.008
FLOOR_TOP_Z = 0.016
MID_WALL_TOP_Z = 0.044
BODY_TOP_Z = 0.056
TOP_LAYER_CENTER_Z = 0.050

JOG_CENTER_X = -0.072
JOG_CENTER_Y = 0.020
JOG_RECESS_RADIUS = 0.068
JOG_WELL_RING_RADIUS = 0.066
JOG_RECESS_FLOOR_Z = 0.046
JOG_PLATTER_RADIUS = 0.060

LCD_CENTER_X = 0.082
LCD_CENTER_Y = 0.040
LCD_FLOOR_SIZE = (0.102, 0.072, 0.004)
LCD_FRAME_SIZE = (0.096, 0.066, 0.004)
LCD_RECESS_FLOOR_Z = 0.046

BUTTON_BANK_Y = -0.077
SLOT_OPENING_Z = 0.031


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cdj_media_player", assets=ASSETS)

    body_black = model.material("body_black", rgba=(0.12, 0.13, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.17, 0.18, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.21, 0.23, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.46, 0.48, 0.50, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.19, 0.34, 0.38, 0.38))
    marker_white = model.material("marker_white", rgba=(0.88, 0.89, 0.90, 1.0))
    slot_dark = model.material("slot_dark", rgba=(0.03, 0.03, 0.04, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=body_black,
        name="floor_plate",
    )
    body.visual(
        Box((0.012, BODY_DEPTH, 0.028)),
        origin=Origin(xyz=(-0.154, 0.0, 0.030)),
        material=body_black,
        name="left_wall",
    )
    body.visual(
        Box((0.012, BODY_DEPTH, 0.028)),
        origin=Origin(xyz=(0.154, 0.0, 0.030)),
        material=body_black,
        name="right_wall",
    )
    body.visual(
        Box((0.296, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, 0.104, 0.030)),
        material=body_black,
        name="rear_wall",
    )
    body.visual(
        Box((0.080, 0.012, 0.028)),
        origin=Origin(xyz=(-0.108, -0.104, 0.030)),
        material=body_black,
        name="front_left_face",
    )
    body.visual(
        Box((0.080, 0.012, 0.028)),
        origin=Origin(xyz=(0.108, -0.104, 0.030)),
        material=body_black,
        name="front_right_face",
    )
    body.visual(
        Box((0.136, 0.012, 0.013)),
        origin=Origin(xyz=(0.0, -0.104, 0.0225)),
        material=charcoal,
        name="slot_lower_lip",
    )
    body.visual(
        Box((0.136, 0.012, 0.011)),
        origin=Origin(xyz=(0.0, -0.104, 0.0385)),
        material=charcoal,
        name="slot_upper_lip",
    )
    body.visual(
        Box((0.296, 0.044, 0.012)),
        origin=Origin(xyz=(0.0, -0.088, TOP_LAYER_CENTER_Z)),
        material=body_black,
        name="front_deck",
    )
    body.visual(
        Box((0.296, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.096, TOP_LAYER_CENTER_Z)),
        material=body_black,
        name="rear_deck",
    )
    body.visual(
        Box((0.020, 0.148, 0.012)),
        origin=Origin(xyz=(-0.150, 0.014, TOP_LAYER_CENTER_Z)),
        material=body_black,
        name="left_top_strip",
    )
    body.visual(
        Box((0.036, 0.148, 0.012)),
        origin=Origin(xyz=(0.014, 0.014, TOP_LAYER_CENTER_Z)),
        material=body_black,
        name="center_top_strip",
    )
    body.visual(
        Box((0.028, 0.148, 0.012)),
        origin=Origin(xyz=(0.146, 0.014, TOP_LAYER_CENTER_Z)),
        material=body_black,
        name="right_top_strip",
    )
    body.visual(
        Box((0.100, 0.016, 0.012)),
        origin=Origin(xyz=(LCD_CENTER_X, -0.003, TOP_LAYER_CENTER_Z)),
        material=body_black,
        name="display_front_bezel",
    )
    body.visual(
        Box((0.100, 0.035, 0.012)),
        origin=Origin(xyz=(LCD_CENTER_X, 0.0925, TOP_LAYER_CENTER_Z)),
        material=body_black,
        name="display_rear_bezel",
    )
    body.visual(
        Box(LCD_FLOOR_SIZE),
        origin=Origin(xyz=(LCD_CENTER_X, LCD_CENTER_Y, LCD_RECESS_FLOOR_Z)),
        material=satin_black,
        name="display_floor",
    )
    body.visual(
        Cylinder(radius=JOG_RECESS_RADIUS, length=0.002),
        origin=Origin(xyz=(JOG_CENTER_X, JOG_CENTER_Y, 0.043)),
        material=satin_black,
        name="jog_well_floor",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(JOG_CENTER_X, JOG_CENTER_Y, JOG_RECESS_FLOOR_Z)),
        material=charcoal,
        name="bearing_pedestal",
    )
    body.visual(
        Box((0.096, 0.003, 0.0008)),
        origin=Origin(xyz=(LCD_CENTER_X, LCD_CENTER_Y + 0.039, BODY_TOP_Z + 0.0004)),
        material=charcoal,
        name="lcd_top_lip",
    )
    body.visual(
        Box((0.020, 0.012, 0.0008)),
        origin=Origin(xyz=(JOG_CENTER_X, JOG_CENTER_Y + 0.061, BODY_TOP_Z + 0.0004)),
        material=body_black,
        name="jog_north_pad",
    )
    body.visual(
        Box((0.014, 0.020, 0.0008)),
        origin=Origin(xyz=(JOG_CENTER_X + 0.061, JOG_CENTER_Y, BODY_TOP_Z + 0.0004)),
        material=body_black,
        name="jog_east_pad",
    )
    body.visual(
        Box((0.012, 0.004, 0.0008)),
        origin=Origin(xyz=(JOG_CENTER_X, JOG_CENTER_Y + 0.055, BODY_TOP_Z + 0.0004)),
        material=marker_white,
        name="jog_north_tick",
    )
    body.visual(
        Box((0.004, 0.012, 0.0008)),
        origin=Origin(xyz=(JOG_CENTER_X + 0.055, JOG_CENTER_Y, BODY_TOP_Z + 0.0004)),
        material=marker_white,
        name="jog_east_tick",
    )
    body.visual(
        Box((0.138, 0.0012, 0.014)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.0006, SLOT_OPENING_Z)),
        material=charcoal,
        name="slot_front_lip",
    )
    for foot_index, (foot_x, foot_y) in enumerate(
        (
            (-0.128, -0.085),
            (0.128, -0.085),
            (-0.128, 0.085),
            (0.128, 0.085),
        )
    ):
        body.visual(
            Cylinder(radius=0.0155, length=0.008),
            origin=Origin(xyz=(foot_x, foot_y, 0.004)),
            material=rubber,
            name=f"foot_{foot_index + 1}",
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_TOP_Z - FLOOR_Z)),
        mass=2.9,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
    )

    display = model.part("display_panel")
    display.visual(
        Box(LCD_FRAME_SIZE),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=satin_black,
        name="lcd_frame",
    )
    display.visual(
        Box((0.084, 0.054, 0.0012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0034)),
        material=screen_glass,
        name="lcd_glass",
    )
    display.inertial = Inertial.from_geometry(
        Box(LCD_FRAME_SIZE),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    buttons = model.part("button_bank")
    buttons.visual(
        Box((0.218, 0.048, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=satin_black,
        name="button_panel",
    )
    for button_index, button_x in enumerate((-0.080, -0.048, -0.016, 0.016, 0.048, 0.080)):
        buttons.visual(
            Box((0.024, 0.014, 0.0032)),
            origin=Origin(xyz=(button_x, 0.0, 0.0036)),
            material=charcoal,
            name=f"button_{button_index + 1}",
        )
    buttons.inertial = Inertial.from_geometry(
        Box((0.218, 0.048, 0.0052)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.0026)),
    )

    media_slot = model.part("media_slot")
    media_slot.visual(
        Box((0.116, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.005, 0.002)),
        material=slot_dark,
        name="slot_shadow",
    )
    media_slot.inertial = Inertial.from_geometry(
        Box((0.116, 0.010, 0.004)),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    jog_wheel = model.part("jog_wheel")
    jog_wheel.visual(
        Cylinder(radius=JOG_PLATTER_RADIUS, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=gunmetal,
        name="platter",
    )
    jog_wheel.visual(
        Cylinder(radius=0.064, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=rubber,
        name="grip_ring",
    )
    jog_wheel.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=satin_black,
        name="center_hub",
    )
    jog_wheel.visual(
        Box((0.008, 0.004, 0.0012)),
        origin=Origin(xyz=(0.0, 0.053, 0.0084)),
        material=marker_white,
        name="marker",
    )
    jog_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.063, length=0.013),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.0055)),
    )

    model.articulation(
        "body_to_display_panel",
        ArticulationType.FIXED,
        parent=body,
        child=display,
        origin=Origin(xyz=(LCD_CENTER_X, LCD_CENTER_Y, 0.048)),
    )
    model.articulation(
        "body_to_button_bank",
        ArticulationType.FIXED,
        parent=body,
        child=buttons,
        origin=Origin(xyz=(0.0, BUTTON_BANK_Y, BODY_TOP_Z)),
    )
    model.articulation(
        "body_to_media_slot",
        ArticulationType.FIXED,
        parent=body,
        child=media_slot,
        origin=Origin(xyz=(0.0, -0.110, SLOT_OPENING_Z - 0.002)),
    )
    model.articulation(
        "body_to_jog_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=jog_wheel,
        origin=Origin(xyz=(JOG_CENTER_X, JOG_CENTER_Y, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    display = object_model.get_part("display_panel")
    buttons = object_model.get_part("button_bank")
    media_slot = object_model.get_part("media_slot")
    jog_wheel = object_model.get_part("jog_wheel")
    jog_joint = object_model.get_articulation("body_to_jog_wheel")

    floor_plate = body.get_visual("floor_plate")
    front_deck = body.get_visual("front_deck")
    display_floor = body.get_visual("display_floor")
    display_front_bezel = body.get_visual("display_front_bezel")
    lcd_top_lip = body.get_visual("lcd_top_lip")
    jog_north_tick = body.get_visual("jog_north_tick")
    jog_east_tick = body.get_visual("jog_east_tick")
    slot_front_lip = body.get_visual("slot_front_lip")
    slot_lower_lip = body.get_visual("slot_lower_lip")
    slot_upper_lip = body.get_visual("slot_upper_lip")
    bearing_pedestal = body.get_visual("bearing_pedestal")
    foot_1 = body.get_visual("foot_1")
    foot_2 = body.get_visual("foot_2")
    foot_3 = body.get_visual("foot_3")
    foot_4 = body.get_visual("foot_4")

    lcd_frame = display.get_visual("lcd_frame")
    button_panel = buttons.get_visual("button_panel")
    slot_shadow = media_slot.get_visual("slot_shadow")
    wheel_platter = jog_wheel.get_visual("platter")
    wheel_marker = jog_wheel.get_visual("marker")
    center_hub = jog_wheel.get_visual("center_hub")

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
    ctx.expect_within(display, body, axes="xy", inner_elem=lcd_frame, outer_elem=display_floor)
    ctx.expect_gap(
        display,
        jog_wheel,
        axis="x",
        min_gap=0.035,
        positive_elem=lcd_frame,
        negative_elem=wheel_platter,
    )
    ctx.expect_gap(
        body,
        display,
        axis="z",
        min_gap=0.002,
        max_gap=0.006,
        positive_elem=lcd_top_lip,
        negative_elem=lcd_frame,
    )
    ctx.expect_gap(
        buttons,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=button_panel,
        negative_elem=front_deck,
    )
    ctx.expect_gap(
        jog_wheel,
        buttons,
        axis="y",
        min_gap=0.010,
        positive_elem=wheel_platter,
        negative_elem=button_panel,
    )
    ctx.expect_gap(
        media_slot,
        body,
        axis="y",
        max_gap=0.001,
        positive_elem=slot_shadow,
        negative_elem=slot_front_lip,
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        min_gap=0.003,
        max_gap=0.006,
        positive_elem=slot_upper_lip,
        negative_elem=slot_lower_lip,
    )
    ctx.expect_gap(
        body,
        jog_wheel,
        axis="z",
        min_gap=0.002,
        max_gap=0.007,
        positive_elem=jog_north_tick,
        negative_elem=wheel_platter,
    )
    ctx.expect_overlap(
        jog_wheel,
        body,
        axes="xy",
        min_overlap=0.030,
        elem_a=center_hub,
        elem_b=bearing_pedestal,
    )
    ctx.expect_gap(
        jog_wheel,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=center_hub,
        negative_elem=bearing_pedestal,
    )
    ctx.expect_overlap(
        jog_wheel,
        body,
        axes="xy",
        min_overlap=0.00001,
        elem_a=wheel_marker,
        elem_b=jog_north_tick,
    )
    ctx.expect_within(body, body, axes="xy", inner_elem=foot_1, outer_elem=floor_plate)
    ctx.expect_within(body, body, axes="xy", inner_elem=foot_2, outer_elem=floor_plate)
    ctx.expect_within(body, body, axes="xy", inner_elem=foot_3, outer_elem=floor_plate)
    ctx.expect_within(body, body, axes="xy", inner_elem=foot_4, outer_elem=floor_plate)
    ctx.expect_gap(
        body,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=floor_plate,
        negative_elem=foot_1,
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=floor_plate,
        negative_elem=foot_2,
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=floor_plate,
        negative_elem=foot_3,
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=floor_plate,
        negative_elem=foot_4,
    )
    with ctx.pose({jog_joint: -math.pi / 2.0}):
        ctx.expect_overlap(
            jog_wheel,
            body,
            axes="xy",
            min_overlap=0.00001,
            elem_a=wheel_marker,
            elem_b=jog_east_tick,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
