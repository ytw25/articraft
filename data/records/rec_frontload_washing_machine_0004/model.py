from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
from math import pi

try:
    os.chdir("/tmp")
except FileNotFoundError:
    pass

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


WIDTH = 0.60
DEPTH = 0.67
HEIGHT = 0.85
PANEL = 0.018
FRONT_THICK = 0.028
TOP_THICK = 0.018
FRONT_Y = DEPTH * 0.5
TOP_Z = HEIGHT - TOP_THICK * 0.5

DOOR_CENTER_X = 0.03
DOOR_CENTER_Z = 0.38
DOOR_OUTER_R = 0.205
DOOR_WINDOW_R = 0.145

DRAWER_X = -0.16
DRAWER_Z = 0.69
DRAWER_TRAY_SIZE = (0.158, 0.175, 0.065)
DRAWER_FACE_SIZE = (0.186, 0.022, 0.085)
DRAWER_FACE_Y = DRAWER_TRAY_SIZE[1] * 0.5 + DRAWER_FACE_SIZE[1] * 0.5

LINT_OPENING_CENTER_Y = -0.02
LINT_OPENING_SIZE = (0.308, 0.192)
LINT_LID_SIZE = (0.288, 0.172, 0.014)
LINT_HINGE_Y = LINT_OPENING_CENTER_Y - LINT_OPENING_SIZE[1] * 0.5
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="washer_dryer_combo")

    appliance_white = model.material("appliance_white", rgba=(0.95, 0.96, 0.97, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.93, 0.94, 1.0))
    chrome = model.material("chrome", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.60, 0.72, 0.80, 0.35))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.09, 0.10, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.48, 0.50, 0.52, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((WIDTH, DEPTH, HEIGHT)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, HEIGHT * 0.5)),
    )

    body.visual(
        Box((PANEL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-WIDTH * 0.5 + PANEL * 0.5, 0.0, HEIGHT * 0.5)),
        material=appliance_white,
        name="left_side",
    )
    body.visual(
        Box((PANEL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(WIDTH * 0.5 - PANEL * 0.5, 0.0, HEIGHT * 0.5)),
        material=appliance_white,
        name="right_side",
    )
    body.visual(
        Box((WIDTH - 2.0 * PANEL, PANEL, HEIGHT)),
        origin=Origin(xyz=(0.0, -DEPTH * 0.5 + PANEL * 0.5, HEIGHT * 0.5)),
        material=appliance_white,
        name="back_panel",
    )
    body.visual(
        Box((WIDTH - 2.0 * PANEL, DEPTH - 2.0 * PANEL, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=appliance_white,
        name="base_pan",
    )
    body.visual(
        Box((WIDTH - 2.0 * PANEL, FRONT_THICK, 0.112)),
        origin=Origin(xyz=(0.0, FRONT_Y - FRONT_THICK * 0.5, 0.792)),
        material=appliance_white,
        name="top_fascia",
    )
    body.visual(
        Box((0.096, FRONT_THICK, 0.510)),
        origin=Origin(xyz=(-0.234, FRONT_Y - FRONT_THICK * 0.5, 0.345)),
        material=appliance_white,
        name="left_front_jamb",
    )
    body.visual(
        Box((0.196, FRONT_THICK, 0.560)),
        origin=Origin(xyz=(0.176, FRONT_Y - FRONT_THICK * 0.5, 0.370)),
        material=appliance_white,
        name="right_front_jamb",
    )
    body.visual(
        Box((0.102, FRONT_THICK, 0.145)),
        origin=Origin(xyz=(-0.234, FRONT_Y - FRONT_THICK * 0.5, 0.562)),
        material=appliance_white,
        name="left_mid_panel",
    )
    body.visual(
        Box((0.192, 0.002, 0.089)),
        origin=Origin(xyz=(DRAWER_X, FRONT_Y - 0.003, DRAWER_Z)),
        material=warm_white,
        name="drawer_face_seat",
    )
    body.visual(
        Box((0.180, FRONT_THICK, 0.118)),
        origin=Origin(xyz=(0.006, FRONT_Y - FRONT_THICK * 0.5, 0.584)),
        material=appliance_white,
        name="door_upper_bridge",
    )
    body.visual(
        Cylinder(radius=DOOR_OUTER_R - 0.018, length=0.018),
        origin=Origin(xyz=(DOOR_CENTER_X, FRONT_Y - 0.009, DOOR_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=warm_white,
        name="front_bezel",
    )
    body.visual(
        Box((WIDTH - 2.0 * PANEL, 0.060, 0.090)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.030, 0.045)),
        material=warm_white,
        name="toe_kick",
    )
    body.visual(
        Box((0.236, 0.018, 0.110)),
        origin=Origin(xyz=(0.145, FRONT_Y - 0.009, 0.695)),
        material=warm_white,
        name="control_panel",
    )
    body.visual(
        Cylinder(radius=0.032, length=0.014),
        origin=Origin(xyz=(0.105, FRONT_Y - 0.006, 0.695), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="selector_dial",
    )
    body.visual(
        Box((0.074, 0.010, 0.030)),
        origin=Origin(xyz=(0.205, FRONT_Y - 0.004, 0.712)),
        material=control_black,
        name="display_window",
    )
    body.visual(
        Box((0.096, 0.008, 0.014)),
        origin=Origin(xyz=(0.205, FRONT_Y - 0.005, 0.675)),
        material=control_black,
        name="button_strip",
    )
    body.visual(
        Cylinder(radius=0.163, length=0.268),
        origin=Origin(xyz=(DOOR_CENTER_X, 0.173, DOOR_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="drum_liner",
    )
    body.visual(
        Cylinder(radius=0.126, length=0.008),
        origin=Origin(xyz=(DOOR_CENTER_X, 0.039, DOOR_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=control_black,
        name="drum_backplate",
    )
    body.visual(
        Cylinder(radius=0.154, length=0.018),
        origin=Origin(xyz=(DOOR_CENTER_X, 0.304, DOOR_CENTER_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="drum_gasket",
    )
    body.visual(
        Box((0.028, 0.030, 0.055)),
        origin=Origin(xyz=(DOOR_CENTER_X - DOOR_OUTER_R - 0.008, FRONT_Y - 0.015, DOOR_CENTER_Z + 0.105)),
        material=hinge_gray,
        name="upper_hinge_block",
    )
    body.visual(
        Box((0.028, 0.030, 0.055)),
        origin=Origin(xyz=(DOOR_CENTER_X - DOOR_OUTER_R - 0.008, FRONT_Y - 0.015, DOOR_CENTER_Z - 0.105)),
        material=hinge_gray,
        name="lower_hinge_block",
    )
    body.visual(
        Box((0.172, 0.190, 0.010)),
        origin=Origin(xyz=(DRAWER_X, 0.240, 0.645)),
        material=charcoal,
        name="drawer_bay_floor",
    )
    body.visual(
        Box((0.172, 0.190, 0.010)),
        origin=Origin(xyz=(DRAWER_X, 0.240, 0.735)),
        material=charcoal,
        name="drawer_bay_roof",
    )
    body.visual(
        Box((0.010, 0.190, 0.080)),
        origin=Origin(xyz=(DRAWER_X - 0.089, 0.240, DRAWER_Z)),
        material=charcoal,
        name="drawer_bay_left_wall",
    )
    body.visual(
        Box((0.010, 0.190, 0.080)),
        origin=Origin(xyz=(DRAWER_X + 0.089, 0.240, DRAWER_Z)),
        material=charcoal,
        name="drawer_bay_right_wall",
    )
    body.visual(
        Box((0.172, 0.010, 0.080)),
        origin=Origin(xyz=(DRAWER_X, 0.145, DRAWER_Z)),
        material=charcoal,
        name="drawer_bay_back_wall",
    )
    body.visual(
        Box((WIDTH - 2.0 * PANEL, 0.259, TOP_THICK)),
        origin=Origin(xyz=(0.0, 0.2055, TOP_Z)),
        material=appliance_white,
        name="top_front_strip",
    )
    body.visual(
        Box((WIDTH - 2.0 * PANEL, 0.190, TOP_THICK)),
        origin=Origin(xyz=(0.0, -0.240, TOP_Z)),
        material=appliance_white,
        name="top_rear_strip",
    )
    body.visual(
        Box((0.128, LINT_OPENING_SIZE[1], TOP_THICK)),
        origin=Origin(xyz=(-0.218, LINT_OPENING_CENTER_Y, TOP_Z)),
        material=appliance_white,
        name="top_left_strip",
    )
    body.visual(
        Box((0.128, LINT_OPENING_SIZE[1], TOP_THICK)),
        origin=Origin(xyz=(0.218, LINT_OPENING_CENTER_Y, TOP_Z)),
        material=appliance_white,
        name="top_right_strip",
    )
    body.visual(
        Box((0.300, 0.184, 0.004)),
        origin=Origin(xyz=(0.0, LINT_OPENING_CENTER_Y, TOP_Z - 0.020)),
        material=charcoal,
        name="top_frame",
    )
    body.visual(
        Box((0.294, 0.178, 0.008)),
        origin=Origin(xyz=(0.0, LINT_OPENING_CENTER_Y, 0.742)),
        material=charcoal,
        name="lint_well_floor",
    )
    body.visual(
        Box((0.294, 0.008, 0.092)),
        origin=Origin(xyz=(0.0, 0.072, 0.792)),
        material=charcoal,
        name="lint_well_front_wall",
    )
    body.visual(
        Box((0.294, 0.008, 0.074)),
        origin=Origin(xyz=(0.0, -0.112, 0.773)),
        material=charcoal,
        name="lint_well_rear_wall",
    )
    body.visual(
        Box((0.008, 0.176, 0.092)),
        origin=Origin(xyz=(-0.147, LINT_OPENING_CENTER_Y, 0.792)),
        material=charcoal,
        name="lint_well_left_wall",
    )
    body.visual(
        Box((0.008, 0.176, 0.092)),
        origin=Origin(xyz=(0.147, LINT_OPENING_CENTER_Y, 0.792)),
        material=charcoal,
        name="lint_well_right_wall",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(-0.108, -0.126, TOP_Z - 0.027), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_gray,
        name="hinge_pin_left",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.108, -0.126, TOP_Z - 0.027), rpy=(0.0, pi / 2.0, 0.0)),
        material=hinge_gray,
        name="hinge_pin_right",
    )
    body.visual(
        Box((0.010, 0.014, 0.012)),
        origin=Origin(xyz=(-0.108, -0.119, TOP_Z - 0.033)),
        material=hinge_gray,
        name="hinge_pedestal_left",
    )
    body.visual(
        Box((0.010, 0.014, 0.012)),
        origin=Origin(xyz=(0.108, -0.119, TOP_Z - 0.033)),
        material=hinge_gray,
        name="hinge_pedestal_right",
    )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((0.42, 0.06, 0.42)),
        mass=8.0,
        origin=Origin(xyz=(0.21, 0.0, 0.0)),
    )
    door.visual(
        Cylinder(radius=DOOR_OUTER_R, length=0.050),
        origin=Origin(xyz=(0.213, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=DOOR_WINDOW_R, length=0.014),
        origin=Origin(xyz=(0.213, -0.006, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=tinted_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.080, 0.024, 0.034)),
        origin=Origin(xyz=(0.040, -0.013, 0.105)),
        material=chrome,
        name="upper_hinge_pad",
    )
    door.visual(
        Box((0.080, 0.024, 0.034)),
        origin=Origin(xyz=(0.040, -0.013, -0.105)),
        material=chrome,
        name="lower_hinge_pad",
    )
    door.visual(
        Box((0.026, 0.050, 0.020)),
        origin=Origin(xyz=(0.360, 0.020, 0.015)),
        material=chrome,
        name="door_handle",
    )

    drawer = model.part("drawer")
    drawer.inertial = Inertial.from_geometry(
        Box((0.19, 0.19, 0.09)),
        mass=2.2,
        origin=Origin(),
    )
    drawer.visual(
        Box(DRAWER_TRAY_SIZE),
        material=charcoal,
        name="drawer_tray",
    )
    drawer.visual(
        Box(DRAWER_FACE_SIZE),
        origin=Origin(xyz=(0.0, DRAWER_FACE_Y, 0.0)),
        material=appliance_white,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.096, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, DRAWER_FACE_Y + 0.006, 0.0)),
        material=control_black,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.008, 0.130, 0.048)),
        origin=Origin(xyz=(-0.026, -0.020, 0.0)),
        material=dark_trim,
        name="left_detergent_divider",
    )
    drawer.visual(
        Box((0.008, 0.130, 0.048)),
        origin=Origin(xyz=(0.026, -0.020, 0.0)),
        material=dark_trim,
        name="right_detergent_divider",
    )

    lint_lid = model.part("lint_lid")
    lint_lid.inertial = Inertial.from_geometry(
        Box((0.31, 0.20, 0.03)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.09, 0.0)),
    )
    lint_lid.visual(
        Box((0.288, 0.172, 0.010)),
        origin=Origin(xyz=(0.0, 0.102, 0.014)),
        material=warm_white,
        name="lid_panel",
    )
    lint_lid.visual(
        Box((0.280, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.182, 0.016)),
        material=warm_white,
        name="lid_front_edge",
    )
    lint_lid.visual(
        Box((0.090, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.162, 0.023)),
        material=control_black,
        name="lid_pull",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_CENTER_X - DOOR_OUTER_R - 0.008, FRONT_Y + 0.025, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(DRAWER_X, 0.2455, DRAWER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.25, lower=0.0, upper=0.12),
    )
    model.articulation(
        "lint_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lint_lid,
        origin=Origin(xyz=(0.0, -0.126, TOP_Z - 0.027)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    drawer = object_model.get_part("drawer")
    lint_lid = object_model.get_part("lint_lid")
    door_hinge = object_model.get_articulation("door_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    lint_lid_hinge = object_model.get_articulation("lint_lid_hinge")

    front_bezel = body.get_visual("front_bezel")
    top_frame = body.get_visual("top_frame")
    upper_hinge_block = body.get_visual("upper_hinge_block")
    lower_hinge_block = body.get_visual("lower_hinge_block")
    hinge_pin_left = body.get_visual("hinge_pin_left")
    hinge_pin_right = body.get_visual("hinge_pin_right")
    control_panel = body.get_visual("control_panel")

    door_ring = door.get_visual("door_ring")
    upper_hinge_pad = door.get_visual("upper_hinge_pad")
    lower_hinge_pad = door.get_visual("lower_hinge_pad")

    drawer_face = drawer.get_visual("drawer_face")
    drawer_face_seat = body.get_visual("drawer_face_seat")
    lid_panel = lint_lid.get_visual("lid_panel")
    lid_front_edge = lint_lid.get_visual("lid_front_edge")

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

    ctx.expect_overlap(door, body, axes="xz", elem_a=door_ring, elem_b=front_bezel, min_overlap=0.28)
    ctx.expect_gap(
        door,
        body,
        axis="y",
        positive_elem=door_ring,
        negative_elem=front_bezel,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_contact(door, body, elem_a=upper_hinge_pad, elem_b=upper_hinge_block)
    ctx.expect_contact(door, body, elem_a=lower_hinge_pad, elem_b=lower_hinge_block)

    ctx.expect_gap(
        drawer,
        body,
        axis="y",
        positive_elem=drawer_face,
        negative_elem=drawer_face_seat,
        min_gap=-0.001,
        max_gap=0.001,
    )
    ctx.expect_overlap(drawer, body, axes="xz", elem_a=drawer_face, elem_b=drawer_face_seat, min_overlap=0.01)
    ctx.expect_within(drawer, body, axes="xz", inner_elem=drawer_face, outer_elem=drawer_face_seat)
    ctx.expect_gap(
        body,
        drawer,
        axis="x",
        positive_elem=control_panel,
        negative_elem=drawer_face,
        min_gap=0.08,
    )
    ctx.expect_gap(
        drawer,
        door,
        axis="z",
        positive_elem=drawer_face,
        negative_elem=door_ring,
        min_gap=0.05,
    )

    ctx.expect_overlap(lint_lid, body, axes="xy", elem_a=lid_panel, elem_b=top_frame, min_overlap=0.05)
    ctx.expect_within(lint_lid, body, axes="xy", inner_elem=lid_panel, outer_elem=top_frame)
    ctx.expect_gap(
        lint_lid,
        body,
        axis="z",
        positive_elem=lid_panel,
        negative_elem=top_frame,
        max_gap=0.004,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        lint_lid,
        body,
        axis="y",
        positive_elem=lid_panel,
        negative_elem=hinge_pin_left,
        min_gap=0.008,
        max_gap=0.03,
    )
    ctx.expect_gap(
        lint_lid,
        body,
        axis="y",
        positive_elem=lid_panel,
        negative_elem=hinge_pin_right,
        min_gap=0.008,
        max_gap=0.03,
    )
    ctx.expect_overlap(lint_lid, body, axes="x", elem_a=lid_panel, elem_b=hinge_pin_left, min_overlap=0.010)
    ctx.expect_overlap(lint_lid, body, axes="x", elem_a=lid_panel, elem_b=hinge_pin_right, min_overlap=0.010)
    ctx.expect_gap(
        lint_lid,
        body,
        axis="z",
        positive_elem=lid_panel,
        negative_elem=hinge_pin_left,
        min_gap=0.004,
        max_gap=0.02,
    )
    ctx.expect_gap(
        lint_lid,
        body,
        axis="z",
        positive_elem=lid_panel,
        negative_elem=hinge_pin_right,
        min_gap=0.004,
        max_gap=0.02,
    )

    with ctx.pose({door_hinge: 1.35}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem=door.get_visual("door_handle"),
            negative_elem=front_bezel,
            min_gap=0.20,
        )

    with ctx.pose({drawer_slide: 0.12}):
        ctx.expect_gap(
            drawer,
            body,
            axis="y",
            positive_elem=drawer_face,
            negative_elem=drawer_face_seat,
            min_gap=0.11,
        )

    with ctx.pose({lint_lid_hinge: 0.85}):
        ctx.expect_gap(
            lint_lid,
            body,
            axis="z",
            positive_elem=lid_front_edge,
            negative_elem=top_frame,
            min_gap=0.07,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
