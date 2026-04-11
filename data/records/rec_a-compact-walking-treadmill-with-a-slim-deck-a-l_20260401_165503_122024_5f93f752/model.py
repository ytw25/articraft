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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_WIDTH = 0.62
DECK_LENGTH = 1.10
DECK_WIDTH = 0.58
RAIL_WIDTH = 0.04
RAIL_HEIGHT = 0.044
HINGE_AXIS_Z = 0.038
HINGE_TRUNNION_RADIUS = 0.012
HINGE_TRUNNION_LENGTH = 0.024
HINGE_Y_CENTERS = (-0.245, 0.245)
FRONT_ROLLER_X = -0.055
REAR_ROLLER_X = -1.045
ROLLER_CENTER_Z = -0.018
ROLLER_AXLE_RADIUS = 0.008
ROLLER_DRUM_RADIUS = 0.022
ROLLER_TOTAL_LENGTH = DECK_WIDTH - 2.0 * RAIL_WIDTH
ROLLER_DRUM_LENGTH = 0.44
UPRIGHT_X = 0.160
UPRIGHT_BASE_Z = 0.060
UPRIGHT_HEIGHT = 0.710
CONSOLE_MOUNT_Z = 0.855
DECK_OPEN_ANGLE = 1.28
FLAP_OPEN_ANGLE = 0.90


def _fuse_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    fused = shapes[0]
    for shape in shapes[1:]:
        fused = fused.union(shape)
    return fused


def _box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(xyz)


def _cylinder_y(radius: float, length: float, xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(xyz)


def _build_front_frame_shape() -> cq.Workplane:
    hood = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.018, 0.000),
                (0.245, 0.000),
                (0.245, 0.016),
                (0.188, 0.074),
                (0.114, 0.102),
                (0.040, 0.082),
                (0.018, 0.054),
            ]
        )
        .close()
        .extrude(FRAME_WIDTH / 2.0, both=True)
    )

    upright = (
        cq.Workplane("XY")
        .rect(0.055, 0.090)
        .workplane(offset=UPRIGHT_HEIGHT)
        .rect(0.036, 0.062)
        .loft(combine=True)
        .translate((UPRIGHT_X, 0.0, UPRIGHT_BASE_Z))
    )

    console_stem = _box((0.034, 0.056, 0.118), (UPRIGHT_X - 0.010, 0.0, 0.824))

    hinge_blocks: list[cq.Workplane] = []
    side_cheeks: list[cq.Workplane] = []
    for y_center in HINGE_Y_CENTERS:
        inner_block_y = y_center - (HINGE_TRUNNION_LENGTH / 2.0 + 0.007)
        outer_block_y = y_center + (HINGE_TRUNNION_LENGTH / 2.0 + 0.007)
        hinge_blocks.extend(
            [
                _box((0.018, 0.014, 0.044), (0.022, inner_block_y, HINGE_AXIS_Z)),
                _box((0.018, 0.014, 0.044), (0.022, outer_block_y, HINGE_AXIS_Z)),
            ]
        )
        side_cheeks.append(_box((0.082, 0.055, 0.054), (0.056, y_center, 0.037)))

    return _fuse_all([hood, upright, console_stem, *hinge_blocks, *side_cheeks])


def _build_deck_shape() -> cq.Workplane:
    left_rail = _box((1.040, RAIL_WIDTH, RAIL_HEIGHT), (-0.555, 0.270, -0.022))
    right_rail = _box((1.040, RAIL_WIDTH, RAIL_HEIGHT), (-0.555, -0.270, -0.022))
    running_board = _box((0.840, 0.504, 0.012), (-0.560, 0.000, -0.026))
    front_bridge = _box((0.060, 0.504, 0.014), (-0.120, 0.000, -0.028))
    rear_bridge = _box((0.060, 0.504, 0.014), (-0.980, 0.000, -0.028))
    rear_glides = [
        _box((0.070, 0.040, 0.012), (-1.075, -0.240, -0.040)),
        _box((0.070, 0.040, 0.012), (-1.075, 0.240, -0.040)),
    ]

    hinge_arms = [
        _box((0.060, 0.020, 0.028), (-0.015, -0.245, -0.010)),
        _box((0.060, 0.020, 0.028), (-0.015, 0.245, -0.010)),
    ]
    trunnions = [
        _cylinder_y(HINGE_TRUNNION_RADIUS, HINGE_TRUNNION_LENGTH, (0.0, -0.245, 0.0)),
        _cylinder_y(HINGE_TRUNNION_RADIUS, HINGE_TRUNNION_LENGTH, (0.0, 0.245, 0.0)),
    ]

    return _fuse_all(
        [
            left_rail,
            right_rail,
            running_board,
            front_bridge,
            rear_bridge,
            *rear_glides,
            *hinge_arms,
            *trunnions,
        ]
    )


def _build_roller_shape() -> cq.Workplane:
    axle = _cylinder_y(ROLLER_AXLE_RADIUS, ROLLER_TOTAL_LENGTH, (0.0, 0.0, 0.0))
    drum = _cylinder_y(ROLLER_DRUM_RADIUS, ROLLER_DRUM_LENGTH, (0.0, 0.0, 0.0))
    return axle.union(drum)


def _build_console_shape() -> cq.Workplane:
    housing = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.000, -0.026),
                (0.000, 0.026),
                (-0.018, 0.040),
                (-0.084, 0.044),
                (-0.118, 0.020),
                (-0.110, -0.024),
                (-0.026, -0.040),
            ]
        )
        .close()
        .extrude(0.130, both=True)
    )

    lower_pod = _box((0.024, 0.090, 0.018), (-0.098, 0.0, -0.016))
    latch_tongue = _box((0.014, 0.024, 0.010), (-0.1135, 0.0, -0.013))

    return _fuse_all([housing, lower_pod, latch_tongue])


def _build_safety_flap_shape() -> cq.Workplane:
    pivot_rod = _cylinder_y(0.0035, 0.024, (0.0, 0.0, 0.0))
    hinge_pad = _box((0.008, 0.024, 0.006), (-0.003, 0.0, -0.003))
    neck = _box((0.010, 0.020, 0.010), (-0.011, 0.0, -0.007))
    tab = _box((0.022, 0.032, 0.004), (-0.022, 0.0, -0.011))
    return _fuse_all([pivot_rod, hinge_pad, neck, tab])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_walking_treadmill")

    model.material("frame_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("deck_graphite", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("console_black", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("roller_dark", rgba=(0.20, 0.20, 0.20, 1.0))
    model.material("safety_red", rgba=(0.78, 0.14, 0.14, 1.0))

    front_frame = model.part("front_frame")
    front_frame.visual(
        mesh_from_cadquery(_build_front_frame_shape(), "front_frame"),
        material="frame_silver",
        name="front_frame_shell",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((0.28, FRAME_WIDTH, 0.92)),
        mass=18.0,
        origin=Origin(xyz=(0.13, 0.0, 0.46)),
    )

    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(_build_deck_shape(), "deck"),
        material="deck_graphite",
        name="deck_shell",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.06)),
        mass=14.0,
        origin=Origin(xyz=(-DECK_LENGTH / 2.0, 0.0, -0.022)),
    )

    front_frame.visual(
        mesh_from_cadquery(
            _build_console_shape().translate((0.165, 0.0, CONSOLE_MOUNT_Z)),
            "console_housing",
        ),
        material="console_black",
        name="console_shell",
    )

    front_roller = model.part("front_roller")
    front_roller.visual(
        mesh_from_cadquery(_build_roller_shape(), "front_roller"),
        material="roller_dark",
        name="front_roller_shell",
    )
    front_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=ROLLER_DRUM_RADIUS, length=ROLLER_TOTAL_LENGTH),
        mass=1.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rear_roller = model.part("rear_roller")
    rear_roller.visual(
        mesh_from_cadquery(_build_roller_shape(), "rear_roller"),
        material="roller_dark",
        name="rear_roller_shell",
    )
    rear_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=ROLLER_DRUM_RADIUS, length=ROLLER_TOTAL_LENGTH),
        mass=1.0,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    safety_flap = model.part("safety_flap")
    safety_flap.visual(
        mesh_from_cadquery(_build_safety_flap_shape(), "safety_flap"),
        material="safety_red",
        name="safety_flap_shell",
    )
    safety_flap.inertial = Inertial.from_geometry(
        Box((0.026, 0.040, 0.010)),
        mass=0.06,
        origin=Origin(xyz=(-0.013, 0.0, -0.005)),
    )

    model.articulation(
        "front_frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=320.0, velocity=1.0, lower=0.0, upper=DECK_OPEN_ANGLE),
    )
    model.articulation(
        "deck_to_front_roller",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_roller,
        origin=Origin(xyz=(FRONT_ROLLER_X, 0.0, ROLLER_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "deck_to_rear_roller",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_roller,
        origin=Origin(xyz=(REAR_ROLLER_X, 0.0, ROLLER_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "front_frame_to_safety_flap",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=safety_flap,
        origin=Origin(xyz=(0.0410, 0.0, 0.8425)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=FLAP_OPEN_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    deck = object_model.get_part("deck")
    front_roller = object_model.get_part("front_roller")
    rear_roller = object_model.get_part("rear_roller")
    safety_flap = object_model.get_part("safety_flap")

    deck_hinge = object_model.get_articulation("front_frame_to_deck")
    front_roller_joint = object_model.get_articulation("deck_to_front_roller")
    rear_roller_joint = object_model.get_articulation("deck_to_rear_roller")
    flap_joint = object_model.get_articulation("front_frame_to_safety_flap")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        deck,
        front_roller,
        reason="The slim deck is represented as a simplified fused frame/deck shell around the internal front roller bearings, so QC reads the internal roller seating as penetration.",
    )
    ctx.allow_overlap(
        deck,
        rear_roller,
        reason="The slim deck is represented as a simplified fused frame/deck shell around the internal rear roller bearings, so QC reads the internal roller seating as penetration.",
    )

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

    ctx.expect_contact(deck, front_frame, name="deck trunnions seat in the front hinge supports")
    ctx.expect_contact(front_roller, deck, name="front roller is journaled by the deck rails")
    ctx.expect_contact(rear_roller, deck, name="rear roller is journaled by the deck rails")
    ctx.expect_contact(safety_flap, front_frame, name="safety flap hangs from the console nose without floating")

    ctx.check(
        "deck hinge uses a transverse folding axis",
        deck_hinge.axis == (0.0, 1.0, 0.0),
        details=f"axis={deck_hinge.axis}",
    )
    ctx.check(
        "rollers use continuous transverse joints",
        front_roller_joint.articulation_type == ArticulationType.CONTINUOUS
        and rear_roller_joint.articulation_type == ArticulationType.CONTINUOUS
        and front_roller_joint.axis == (0.0, 1.0, 0.0)
        and rear_roller_joint.axis == (0.0, 1.0, 0.0),
        details=(
            f"front={front_roller_joint.articulation_type, front_roller_joint.axis}, "
            f"rear={rear_roller_joint.articulation_type, rear_roller_joint.axis}"
        ),
    )
    ctx.check(
        "safety flap hinge uses a small console-side pivot",
        flap_joint.axis == (0.0, 1.0, 0.0)
        and flap_joint.motion_limits is not None
        and flap_joint.motion_limits.upper is not None
        and flap_joint.motion_limits.upper >= 0.75,
        details=f"axis={flap_joint.axis}, limits={flap_joint.motion_limits}",
    )

    closed_deck_aabb = ctx.part_world_aabb(deck)
    with ctx.pose({deck_hinge: DECK_OPEN_ANGLE}):
        open_deck_aabb = ctx.part_world_aabb(deck)
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps when the deck is folded upward")
    ctx.check(
        "deck folds upward above the hood",
        closed_deck_aabb is not None
        and open_deck_aabb is not None
        and open_deck_aabb[1][2] > closed_deck_aabb[1][2] + 0.45,
        details=f"closed={closed_deck_aabb}, open={open_deck_aabb}",
    )

    closed_flap_aabb = ctx.part_world_aabb(safety_flap)
    with ctx.pose({flap_joint: FLAP_OPEN_ANGLE}):
        open_flap_aabb = ctx.part_world_aabb(safety_flap)
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps when the safety flap is opened")
    ctx.check(
        "safety flap visibly opens",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.010,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
