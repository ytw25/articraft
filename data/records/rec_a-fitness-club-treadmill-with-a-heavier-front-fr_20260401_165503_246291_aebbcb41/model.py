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
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_X = 0.42
HINGE_Z = 0.12

DECK_LENGTH = 1.56
DECK_OUTER_WIDTH = 0.78
DECK_INNER_WIDTH = 0.65
ROLLER_RADIUS = 0.035
FRONT_ROLLER_X = 0.13
REAR_ROLLER_X = 1.45
BELT_LENGTH = REAR_ROLLER_X - FRONT_ROLLER_X
BELT_WIDTH = 0.56


def _box_wp(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    fillet: float | None = None,
) -> cq.Workplane:
    wp = cq.Workplane("XY").box(*size)
    if fillet is not None and fillet > 0.0:
        wp = wp.edges("|Z").fillet(fillet)
    return wp.translate(center)


def _cylinder_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
) -> cq.Workplane:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    direction = cq.Vector(dx / length, dy / length, dz / length)
    solid = cq.Solid.makeCylinder(radius, length, cq.Vector(*start), direction)
    return cq.Workplane(obj=solid)


def _union_workplanes(items: list[cq.Workplane]) -> cq.Shape:
    result = items[0]
    for item in items[1:]:
        result = result.union(item, clean=False)
    return result.val()


def _build_front_frame_shape() -> cq.Shape:
    solids = [
        _box_wp((0.38, 0.52, 0.20), (0.19, 0.0, 0.10), fillet=0.02),
        _box_wp((0.34, 0.26, 0.14), (0.17, 0.33, 0.07), fillet=0.02),
        _box_wp((0.34, 0.26, 0.14), (0.17, -0.33, 0.07), fillet=0.02),
        _box_wp((0.10, 0.08, 1.12), (0.32, 0.34, 0.68), fillet=0.012),
        _box_wp((0.10, 0.08, 1.12), (0.32, -0.34, 0.68), fillet=0.012),
        _box_wp((0.05, 0.50, 0.08), (0.395, 0.0, 0.12), fillet=0.01),
        _box_wp((0.14, 0.30, 0.18), (0.41, 0.0, 1.13), fillet=0.015),
        _box_wp((0.16, 0.76, 0.24), (0.47, 0.0, 1.33), fillet=0.02),
        _cylinder_between((0.36, 0.33, 0.88), (1.14, 0.33, 0.72), 0.025),
        _cylinder_between((0.36, -0.33, 0.88), (1.14, -0.33, 0.72), 0.025),
    ]
    return _union_workplanes(solids)


def _build_deck_shape() -> cq.Shape:
    rail_y = 0.36
    solids = [
        _box_wp((1.52, 0.06, 0.10), (0.80, rail_y, -0.005), fillet=0.008),
        _box_wp((1.52, 0.06, 0.10), (0.80, -rail_y, -0.005), fillet=0.008),
        _box_wp((1.24, 0.62, 0.024), (0.82, 0.0, 0.038)),
        _box_wp((0.05, 0.08, 0.08), (0.025, 0.25, 0.04), fillet=0.006),
        _box_wp((0.05, 0.08, 0.08), (0.025, -0.25, 0.04), fillet=0.006),
        _box_wp((0.06, 0.05, 0.07), (FRONT_ROLLER_X, 0.35, 0.02), fillet=0.004),
        _box_wp((0.06, 0.05, 0.07), (FRONT_ROLLER_X, -0.35, 0.02), fillet=0.004),
        _box_wp((0.06, 0.05, 0.07), (REAR_ROLLER_X, 0.35, 0.02), fillet=0.004),
        _box_wp((0.06, 0.05, 0.07), (REAR_ROLLER_X, -0.35, 0.02), fillet=0.004),
        _box_wp((0.10, 0.72, 0.02), (FRONT_ROLLER_X, 0.0, -0.03), fillet=0.004),
        _box_wp((0.10, 0.72, 0.02), (REAR_ROLLER_X, 0.0, -0.03), fillet=0.004),
    ]
    return _union_workplanes(solids)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fitness_club_treadmill")

    model.material("frame_gray", rgba=(0.27, 0.28, 0.30, 1.0))
    model.material("deck_gray", rgba=(0.20, 0.20, 0.21, 1.0))
    model.material("belt_black", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("roller_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("screen_glass", rgba=(0.10, 0.15, 0.18, 1.0))
    model.material("shelf_black", rgba=(0.10, 0.10, 0.11, 1.0))

    handrail_pitch = math.atan2(0.72 - 0.87, 1.12 - 0.35)

    front_frame = model.part("front_frame")
    front_frame.visual(
        Box((0.38, 0.52, 0.20)),
        origin=Origin(xyz=(0.19, 0.0, 0.10)),
        material="frame_gray",
        name="motor_cover",
    )
    front_frame.visual(
        Box((0.28, 0.18, 0.14)),
        origin=Origin(xyz=(0.14, 0.27, 0.07)),
        material="frame_gray",
        name="left_base_pod",
    )
    front_frame.visual(
        Box((0.28, 0.18, 0.14)),
        origin=Origin(xyz=(0.14, -0.27, 0.07)),
        material="frame_gray",
        name="right_base_pod",
    )
    front_frame.visual(
        Box((0.04, 0.48, 0.08)),
        origin=Origin(xyz=(0.40, 0.0, 0.08)),
        material="frame_gray",
        name="hinge_beam",
    )
    front_frame.visual(
        Box((0.08, 0.08, 1.06)),
        origin=Origin(xyz=(0.32, 0.34, 0.65)),
        material="frame_gray",
        name="left_upright",
    )
    front_frame.visual(
        Box((0.08, 0.08, 1.06)),
        origin=Origin(xyz=(0.32, -0.34, 0.65)),
        material="frame_gray",
        name="right_upright",
    )
    front_frame.visual(
        Box((0.08, 0.24, 0.18)),
        origin=Origin(xyz=(0.40, 0.0, 1.02)),
        material="frame_gray",
        name="console_neck",
    )
    front_frame.visual(
        Box((0.10, 0.70, 0.08)),
        origin=Origin(xyz=(0.41, 0.0, 1.14)),
        material="frame_gray",
        name="console_bridge",
    )
    front_frame.visual(
        Box((0.18, 0.76, 0.24)),
        origin=Origin(xyz=(0.46, 0.0, 1.30)),
        material="frame_gray",
        name="console_head",
    )
    front_frame.visual(
        Box((0.03, 0.28, 0.02)),
        origin=Origin(xyz=(0.565, 0.0, 1.19)),
        material="frame_gray",
        name="shelf_bracket",
    )
    front_frame.visual(
        Box((math.sqrt((1.12 - 0.35) ** 2 + (0.72 - 0.87) ** 2), 0.045, 0.045)),
        origin=Origin(
            xyz=((0.35 + 1.12) / 2.0, 0.33, (0.87 + 0.72) / 2.0),
            rpy=(0.0, handrail_pitch, 0.0),
        ),
        material="frame_gray",
        name="left_handrail",
    )
    front_frame.visual(
        Box((math.sqrt((1.12 - 0.35) ** 2 + (0.72 - 0.87) ** 2), 0.045, 0.045)),
        origin=Origin(
            xyz=((0.35 + 1.12) / 2.0, -0.33, (0.87 + 0.72) / 2.0),
            rpy=(0.0, handrail_pitch, 0.0),
        ),
        material="frame_gray",
        name="right_handrail",
    )
    front_frame.visual(
        Box((0.008, 0.42, 0.14)),
        origin=Origin(xyz=(0.554, 0.0, 1.34)),
        material="screen_glass",
        name="screen_glass",
    )

    deck = model.part("deck")
    deck.visual(
        Box((1.54, 0.06, 0.09)),
        origin=Origin(xyz=(0.79, 0.36, 0.0)),
        material="deck_gray",
        name="left_rail",
    )
    deck.visual(
        Box((1.54, 0.06, 0.09)),
        origin=Origin(xyz=(0.79, -0.36, 0.0)),
        material="deck_gray",
        name="right_rail",
    )
    deck.visual(
        Box((0.02, 0.72, 0.08)),
        origin=Origin(xyz=(0.01, 0.0, 0.04)),
        material="deck_gray",
        name="front_hinge_nose",
    )
    deck.visual(
        Box((0.10, 0.72, 0.04)),
        origin=Origin(xyz=(FRONT_ROLLER_X, 0.0, -0.02)),
        material="deck_gray",
        name="front_saddle",
    )
    deck.visual(
        Box((0.10, 0.72, 0.04)),
        origin=Origin(xyz=(REAR_ROLLER_X, 0.0, -0.02)),
        material="deck_gray",
        name="rear_saddle",
    )
    deck.visual(
        Box((1.18, 0.66, 0.024)),
        origin=Origin(xyz=(0.77, 0.0, 0.033)),
        material="deck_gray",
        name="deck_board",
    )
    deck.visual(
        Box((1.18, BELT_WIDTH, 0.006)),
        origin=Origin(xyz=(0.77, 0.0, 0.048)),
        material="belt_black",
        name="running_belt",
    )

    front_roller = model.part("front_roller")
    front_roller.visual(
        Cylinder(radius=ROLLER_RADIUS, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="roller_black",
        name="drum",
    )

    rear_roller = model.part("rear_roller")
    rear_roller.visual(
        Cylinder(radius=ROLLER_RADIUS, length=0.66),
        origin=Origin(xyz=(0.0, 0.0, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="roller_black",
        name="drum",
    )

    tablet_shelf = model.part("tablet_shelf")
    tablet_shelf.visual(
        Box((0.018, 0.28, 0.02)),
        origin=Origin(xyz=(0.009, 0.0, -0.01)),
        material="shelf_black",
        name="shelf_hinge_leaf",
    )
    tablet_shelf.visual(
        Box((0.016, 0.34, 0.20)),
        origin=Origin(xyz=(0.008, 0.0, -0.10)),
        material="shelf_black",
        name="shelf_panel",
    )

    model.articulation(
        "frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=deck,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=1.2,
            lower=0.0,
            upper=1.10,
        ),
    )

    model.articulation(
        "deck_to_front_roller",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_roller,
        origin=Origin(xyz=(FRONT_ROLLER_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=30.0),
    )

    model.articulation(
        "deck_to_rear_roller",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_roller,
        origin=Origin(xyz=(REAR_ROLLER_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=30.0),
    )

    model.articulation(
        "frame_to_tablet_shelf",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=tablet_shelf,
        origin=Origin(xyz=(0.55, 0.0, 1.18)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    deck = object_model.get_part("deck")
    front_roller = object_model.get_part("front_roller")
    rear_roller = object_model.get_part("rear_roller")
    tablet_shelf = object_model.get_part("tablet_shelf")

    deck_hinge = object_model.get_articulation("frame_to_deck")
    front_spin = object_model.get_articulation("deck_to_front_roller")
    rear_spin = object_model.get_articulation("deck_to_rear_roller")
    shelf_hinge = object_model.get_articulation("frame_to_tablet_shelf")

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

    ctx.expect_contact(
        deck,
        front_frame,
        name="deck rests against the front frame stop beneath the motor cover",
    )
    ctx.expect_contact(
        front_roller,
        deck,
        name="front roller stays physically supported inside the deck",
    )
    ctx.expect_contact(
        rear_roller,
        deck,
        name="rear roller stays physically supported inside the deck",
    )
    ctx.expect_contact(
        tablet_shelf,
        front_frame,
        name="tablet shelf is stowed against the console at rest",
    )

    transverse_axes_ok = (
        deck_hinge.articulation_type == ArticulationType.REVOLUTE
        and front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and shelf_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(deck_hinge.axis[0]) < 1e-9
        and abs(abs(deck_hinge.axis[1]) - 1.0) < 1e-9
        and abs(deck_hinge.axis[2]) < 1e-9
        and abs(front_spin.axis[0]) < 1e-9
        and abs(abs(front_spin.axis[1]) - 1.0) < 1e-9
        and abs(front_spin.axis[2]) < 1e-9
        and abs(rear_spin.axis[0]) < 1e-9
        and abs(abs(rear_spin.axis[1]) - 1.0) < 1e-9
        and abs(rear_spin.axis[2]) < 1e-9
        and abs(shelf_hinge.axis[0]) < 1e-9
        and abs(abs(shelf_hinge.axis[1]) - 1.0) < 1e-9
        and abs(shelf_hinge.axis[2]) < 1e-9
    )
    ctx.check(
        "deck, rollers, and shelf all rotate about horizontal transverse axes",
        transverse_axes_ok,
        details=(
            f"deck_axis={deck_hinge.axis}, front_axis={front_spin.axis}, "
            f"rear_axis={rear_spin.axis}, shelf_axis={shelf_hinge.axis}"
        ),
    )

    rear_rest = ctx.part_world_position(rear_roller)
    with ctx.pose({deck_hinge: deck_hinge.motion_limits.upper}):
        rear_folded = ctx.part_world_position(rear_roller)
    ctx.check(
        "folding the deck lifts the rear roller upward and forward",
        rear_rest is not None
        and rear_folded is not None
        and rear_folded[2] > rear_rest[2] + 0.55
        and rear_folded[0] < rear_rest[0] - 0.35,
        details=f"rest={rear_rest}, folded={rear_folded}",
    )

    shelf_rest = ctx.part_element_world_aabb(tablet_shelf, elem="shelf_panel")
    with ctx.pose({shelf_hinge: shelf_hinge.motion_limits.upper}):
        shelf_deployed = ctx.part_element_world_aabb(tablet_shelf, elem="shelf_panel")
    ctx.check(
        "tablet shelf folds out toward the runner",
        shelf_rest is not None
        and shelf_deployed is not None
        and shelf_deployed[1][0] > shelf_rest[1][0] + 0.12,
        details=f"rest={shelf_rest}, deployed={shelf_deployed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
