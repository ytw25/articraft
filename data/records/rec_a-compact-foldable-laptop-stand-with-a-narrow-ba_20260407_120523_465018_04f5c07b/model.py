from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_DEPTH = 0.185
BASE_WIDTH = 0.115
BASE_THICKNESS = 0.010
BASE_CORNER = 0.012

BASE_PIVOT_X = -0.048
BASE_LOWER_Z = 0.024
PIVOT_SEPARATION = 0.042
SIDE_OFFSET_Y = 0.064

LINK_LENGTH = 0.160
LINK_WIDTH = 0.022
LINK_THICKNESS = 0.004
LINK_HOLE_RADIUS = 0.0043
PIN_RADIUS = LINK_HOLE_RADIUS
PIN_LENGTH = 0.012

REST_ANGLE = 0.64
RAISED_ANGLE = 1.04
LINK_SWEEP = RAISED_ANGLE - REST_ANGLE

REST_DX = LINK_LENGTH * math.cos(REST_ANGLE)
REST_DZ = LINK_LENGTH * math.sin(REST_ANGLE)
RAISED_DX = LINK_LENGTH * math.cos(RAISED_ANGLE)
RAISED_DZ = LINK_LENGTH * math.sin(RAISED_ANGLE)

DECK_TRAVEL = (RAISED_DX - REST_DX, 0.0, RAISED_DZ - REST_DZ)
DECK_TRAVEL_LEN = math.sqrt(sum(v * v for v in DECK_TRAVEL))
DECK_AXIS = tuple(v / DECK_TRAVEL_LEN for v in DECK_TRAVEL)

DECK_PLATE_LENGTH = 0.255
DECK_PLATE_WIDTH = 0.270
DECK_PLATE_THICKNESS = 0.005
DECK_PLATE_CORNER = 0.011
DECK_PLATE_CENTER_X = 0.082
DECK_PLATE_BOTTOM_Z = 0.046
DECK_BRACKET_WIDTH_X = 0.026
DECK_BRACKET_HEIGHT = 0.054
DECK_BRACKET_THICKNESS = 0.008
DECK_BRACKET_CENTER_Y = 0.058
DECK_BRACKET_BOTTOM_Z = -0.004

LEDGE_THICKNESS_X = 0.006
LEDGE_HEIGHT = 0.012
LEDGE_WIDTH = 0.210
LEDGE_X = DECK_PLATE_CENTER_X + DECK_PLATE_LENGTH / 2.0 - LEDGE_THICKNESS_X / 2.0

PAD_LENGTH = 0.072
PAD_WIDTH = 0.018
PAD_THICKNESS = 0.002
PAD_X_POSITIONS = (0.050, 0.120)
PAD_Y_POSITIONS = (-0.060, 0.060)


def _rounded_plate(length: float, width: float, thickness: float, radius: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, thickness, centered=(True, True, False))
        .edges("|Z")
        .fillet(radius)
    )


def _pin_at(x: float, y: float, z: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(PIN_RADIUS)
        .extrude(PIN_LENGTH / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )


def _side_bracket(center_x: float, center_y: float, bottom_z: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, bottom_z + height / 2.0)
        .rect(DECK_BRACKET_WIDTH_X, height)
        .extrude(DECK_BRACKET_THICKNESS / 2.0, both=True)
        .translate((0.0, center_y, 0.0))
        .edges("|Y")
        .fillet(0.004)
    )


def _build_base_shape() -> cq.Workplane:
    shape = _rounded_plate(BASE_DEPTH, BASE_WIDTH, BASE_THICKNESS, BASE_CORNER)

    bracket_height = 0.074
    bracket_width_x = 0.030
    bracket_thickness = 0.008
    bracket_center_y = BASE_WIDTH / 2.0 - bracket_thickness / 2.0
    bracket_center_z = (BASE_LOWER_Z + (BASE_LOWER_Z + PIVOT_SEPARATION)) / 2.0

    for sign in (-1.0, 1.0):
        bracket = (
            cq.Workplane("XZ")
            .center(BASE_PIVOT_X, bracket_center_z)
            .rect(bracket_width_x, bracket_height)
            .extrude(bracket_thickness / 2.0, both=True)
            .translate((0.0, sign * bracket_center_y, 0.0))
            .edges("|Y")
            .fillet(0.004)
        )
        shape = shape.union(bracket)

        pin_y = sign * (bracket_center_y + bracket_thickness / 2.0)
        for pivot_z in (BASE_LOWER_Z, BASE_LOWER_Z + PIVOT_SEPARATION):
            shape = shape.union(_pin_at(BASE_PIVOT_X, pin_y, pivot_z))

    return shape


def _build_deck_shape() -> cq.Workplane:
    plate = _rounded_plate(
        DECK_PLATE_LENGTH,
        DECK_PLATE_WIDTH,
        DECK_PLATE_THICKNESS,
        DECK_PLATE_CORNER,
    ).translate((DECK_PLATE_CENTER_X, 0.0, DECK_PLATE_BOTTOM_Z))

    ledge = (
        cq.Workplane("XY")
        .box(
            LEDGE_THICKNESS_X,
            LEDGE_WIDTH,
            LEDGE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((LEDGE_X, 0.0, DECK_PLATE_BOTTOM_Z + DECK_PLATE_THICKNESS))
        .edges("|Z")
        .fillet(0.002)
    )

    shape = plate.union(ledge)

    for sign in (-1.0, 1.0):
        bracket = _side_bracket(
            center_x=0.0,
            center_y=sign * DECK_BRACKET_CENTER_Y,
            bottom_z=DECK_BRACKET_BOTTOM_Z,
            height=DECK_BRACKET_HEIGHT,
        )
        shape = shape.union(bracket)

        pin_y = sign * (DECK_BRACKET_CENTER_Y + DECK_BRACKET_THICKNESS / 2.0)
        for pivot_z in (0.0, PIVOT_SEPARATION):
            shape = shape.union(_pin_at(0.0, pin_y, pivot_z))

    return shape


def _build_link_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .center(LINK_LENGTH / 2.0, 0.0)
        .slot2D(LINK_LENGTH, LINK_WIDTH, 0.0)
        .extrude(LINK_THICKNESS / 2.0, both=True)
    )

    inner = (
        cq.Workplane("XZ")
        .center(LINK_LENGTH / 2.0, 0.0)
        .slot2D(LINK_LENGTH - 0.060, LINK_WIDTH - 0.010, 0.0)
        .extrude((LINK_THICKNESS + 0.002) / 2.0, both=True)
    )

    link = outer.cut(inner)
    for hole_x in (0.0, LINK_LENGTH):
        hole = (
            cq.Workplane("XZ")
            .center(hole_x, 0.0)
            .circle(LINK_HOLE_RADIUS)
            .extrude((LINK_THICKNESS + 0.002) / 2.0, both=True)
        )
        link = link.cut(hole)

    return link.edges("|Y").fillet(0.0012)


def _build_pad_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PAD_LENGTH, PAD_WIDTH, PAD_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0035)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foldable_laptop_stand")

    model.material("powder_aluminum", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("graphite_metal", rgba=(0.47, 0.49, 0.53, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base_shell"),
        material="graphite_metal",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_DEPTH, BASE_WIDTH, 0.082)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(_build_deck_shape(), "deck_shell"),
        material="powder_aluminum",
        name="deck_shell",
    )
    pad_shape = _build_pad_shape()
    pad_z = DECK_PLATE_BOTTOM_Z + DECK_PLATE_THICKNESS
    for xi in PAD_X_POSITIONS:
        for yi in PAD_Y_POSITIONS:
            deck.visual(
                mesh_from_cadquery(pad_shape, f"pad_{xi:.3f}_{yi:.3f}".replace(".", "_")),
                origin=Origin(xyz=(xi, yi, pad_z)),
                material="rubber_black",
                name=f"pad_{xi:.3f}_{yi:.3f}".replace(".", "_"),
            )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_PLATE_LENGTH, DECK_PLATE_WIDTH, 0.060)),
        mass=0.58,
        origin=Origin(xyz=(DECK_PLATE_CENTER_X, 0.0, 0.030)),
    )

    link_shape = _build_link_shape()
    link_origin = Origin(xyz=(0.0, 0.0, 0.0))

    left_lower = model.part("left_lower_link")
    left_lower.visual(
        mesh_from_cadquery(link_shape, "left_lower_link"),
        origin=link_origin,
        material="powder_aluminum",
        name="left_lower_link",
    )
    left_lower.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH, LINK_THICKNESS, LINK_WIDTH)),
        mass=0.09,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    right_lower = model.part("right_lower_link")
    right_lower.visual(
        mesh_from_cadquery(link_shape, "right_lower_link"),
        origin=link_origin,
        material="powder_aluminum",
        name="right_lower_link",
    )
    right_lower.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH, LINK_THICKNESS, LINK_WIDTH)),
        mass=0.09,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    left_upper = model.part("left_upper_link")
    left_upper.visual(
        mesh_from_cadquery(link_shape, "left_upper_link"),
        origin=link_origin,
        material="powder_aluminum",
        name="left_upper_link",
    )
    left_upper.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH, LINK_THICKNESS, LINK_WIDTH)),
        mass=0.09,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    right_upper = model.part("right_upper_link")
    right_upper.visual(
        mesh_from_cadquery(link_shape, "right_upper_link"),
        origin=link_origin,
        material="powder_aluminum",
        name="right_upper_link",
    )
    right_upper.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH, LINK_THICKNESS, LINK_WIDTH)),
        mass=0.09,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_deck",
        ArticulationType.PRISMATIC,
        parent=base,
        child=deck,
        origin=Origin(xyz=(BASE_PIVOT_X + REST_DX, 0.0, BASE_LOWER_Z + REST_DZ)),
        axis=DECK_AXIS,
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=DECK_TRAVEL_LEN,
        ),
    )

    for side_name, child, y_sign, z_origin in (
        ("left_lower", left_lower, 1.0, BASE_LOWER_Z),
        ("right_lower", right_lower, -1.0, BASE_LOWER_Z),
        ("left_upper", left_upper, 1.0, BASE_LOWER_Z + PIVOT_SEPARATION),
        ("right_upper", right_upper, -1.0, BASE_LOWER_Z + PIVOT_SEPARATION),
    ):
        model.articulation(
            f"base_to_{side_name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=child,
            origin=Origin(
                xyz=(BASE_PIVOT_X, y_sign * SIDE_OFFSET_Y, z_origin),
                rpy=(0.0, -REST_ANGLE, 0.0),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=18.0,
                velocity=1.2,
                lower=0.0,
                upper=LINK_SWEEP,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    base = object_model.get_part("base")
    deck = object_model.get_part("deck")
    slide = object_model.get_articulation("base_to_deck")
    left_lower = object_model.get_articulation("base_to_left_lower")
    right_lower = object_model.get_articulation("base_to_right_lower")
    left_upper = object_model.get_articulation("base_to_left_upper")
    right_upper = object_model.get_articulation("base_to_right_upper")

    ctx.expect_gap(
        deck,
        base,
        axis="z",
        min_gap=0.030,
        name="resting deck clears the narrow base",
    )
    ctx.expect_overlap(
        deck,
        base,
        axes="y",
        min_overlap=0.100,
        name="deck stays centered over the narrow base footprint",
    )

    rest_pos = ctx.part_world_position(deck)
    with ctx.pose(
        {
            slide: DECK_TRAVEL_LEN,
            left_lower: LINK_SWEEP,
            right_lower: LINK_SWEEP,
            left_upper: LINK_SWEEP,
            right_upper: LINK_SWEEP,
        }
    ):
        ctx.expect_gap(
            deck,
            base,
            axis="z",
            min_gap=0.070,
            name="raised deck still clears the base",
        )
        ctx.expect_overlap(
            deck,
            base,
            axes="y",
            min_overlap=0.100,
            name="raised deck remains centered over the base",
        )
        raised_pos = ctx.part_world_position(deck)

    ctx.check(
        "parallel links lift the deck upward and slightly rearward",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.040
        and raised_pos[0] < rest_pos[0] - 0.035,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
