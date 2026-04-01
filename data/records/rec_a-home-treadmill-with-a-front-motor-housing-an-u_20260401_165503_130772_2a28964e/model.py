from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


DECK_HINGE_ORIGIN = (-0.06, 0.0, 0.11)
DECK_FOLD_UPPER = 1.25


def box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length / 2.0, both=True).translate(center)


def make_base_structure() -> cq.Workplane:
    hood = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.02, 0.00),
                (0.30, 0.00),
                (0.30, 0.06),
                (0.22, 0.14),
                (0.08, 0.18),
                (0.00, 0.16),
                (-0.02, 0.11),
            ]
        )
        .close()
        .extrude(0.62, both=True)
        .edges("|Y")
        .fillet(0.02)
    )

    runner_left = box_at((0.24, 0.09, 0.04), (0.09, 0.29, 0.02))
    runner_right = box_at((0.24, 0.09, 0.04), (0.09, -0.29, 0.02))

    upright_left = (
        cq.Workplane("XY")
        .box(0.065, 0.055, 1.12)
        .edges("|Z")
        .fillet(0.009)
        .translate((0.07, 0.28, 0.68))
        .rotate((0.07, 0.28, 0.12), (0.07, 1.28, 0.12), -9.0)
    )
    upright_right = (
        cq.Workplane("XY")
        .box(0.065, 0.055, 1.12)
        .edges("|Z")
        .fillet(0.009)
        .translate((0.07, -0.28, 0.68))
        .rotate((0.07, -0.28, 0.12), (0.07, -1.28, 0.12), -9.0)
    )

    handlebar = box_at((0.05, 0.66, 0.05), (-0.03, 0.0, 0.93)).edges("|Z").fillet(0.008)
    console_bar = box_at((0.04, 0.54, 0.04), (-0.09, 0.0, 1.20)).edges("|Z").fillet(0.006)
    console_neck = box_at((0.08, 0.46, 0.09), (-0.12, 0.0, 1.27)).edges("|Z").fillet(0.01)
    console_pod = (
        cq.Workplane("XY")
        .box(0.12, 0.56, 0.18)
        .edges("|Z")
        .fillet(0.018)
        .translate((-0.13, 0.0, 1.315))
        .rotate((-0.13, 0.0, 1.315), (-0.13, 1.0, 1.315), 8.0)
    )

    left_fork_inner = box_at((0.05, 0.012, 0.07), (0.23, 0.335, 0.035))
    left_fork_outer = box_at((0.05, 0.012, 0.07), (0.23, 0.369, 0.035))
    right_fork_inner = box_at((0.05, 0.012, 0.07), (0.23, -0.335, 0.035))
    right_fork_outer = box_at((0.05, 0.012, 0.07), (0.23, -0.369, 0.035))

    return (
        hood.union(runner_left)
        .union(runner_right)
        .union(upright_left)
        .union(upright_right)
        .union(handlebar)
        .union(console_bar)
        .union(console_neck)
        .union(console_pod)
        .union(left_fork_inner)
        .union(left_fork_outer)
        .union(right_fork_inner)
        .union(right_fork_outer)
    )


def make_deck_frame() -> cq.Workplane:
    left_rail = box_at((1.52, 0.11, 0.08), (-0.80, 0.315, -0.02))
    right_rail = box_at((1.52, 0.11, 0.08), (-0.80, -0.315, -0.02))
    front_member = box_at((0.12, 0.56, 0.06), (-0.20, 0.0, -0.03))
    rear_member = box_at((0.06, 0.56, 0.06), (-1.55, 0.0, -0.03))
    lower_pan = box_at((1.18, 0.62, 0.02), (-0.82, 0.0, -0.05))
    rear_foot_left = box_at((0.05, 0.08, 0.05), (-1.55, 0.25, -0.085))
    rear_foot_right = box_at((0.05, 0.08, 0.05), (-1.55, -0.25, -0.085))

    left_hinge_block = box_at((0.04, 0.05, 0.05), (-0.03, 0.315, -0.01))
    right_hinge_block = box_at((0.04, 0.05, 0.05), (-0.03, -0.315, -0.01))
    left_knuckle = y_cylinder(0.022, 0.038, (-0.01, 0.315, 0.0))
    right_knuckle = y_cylinder(0.022, 0.038, (-0.01, -0.315, 0.0))

    return (
        left_rail.union(right_rail)
        .union(front_member)
        .union(rear_member)
        .union(lower_pan)
        .union(rear_foot_left)
        .union(rear_foot_right)
        .union(left_hinge_block)
        .union(right_hinge_block)
        .union(left_knuckle)
        .union(right_knuckle)
    )


def make_roller(length: float = 0.52, radius: float = 0.03) -> cq.Workplane:
    return y_cylinder(radius, length, (0.0, 0.0, 0.0))


def make_transport_wheel() -> cq.Workplane:
    tire = y_cylinder(0.035, 0.022, (0.0, 0.0, 0.0))
    hub = y_cylinder(0.026, 0.018, (0.0, 0.0, 0.0))
    return tire.union(hub)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_treadmill")

    base_color = model.material("base_graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    deck_color = model.material("deck_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    tread_color = model.material("tread_black", rgba=(0.07, 0.07, 0.08, 1.0))
    roller_color = model.material("roller_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    wheel_color = model.material("wheel_rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_structure(), "base_structure"),
        material=base_color,
        name="base_structure",
    )
    base.visual(
        Box((0.09, 0.05, 0.03)),
        origin=Origin(xyz=(-0.065, 0.315, 0.055)),
        material=base_color,
        name="left_hinge_bridge",
    )
    base.visual(
        Box((0.09, 0.05, 0.03)),
        origin=Origin(xyz=(-0.065, -0.315, 0.055)),
        material=base_color,
        name="right_hinge_bridge",
    )
    deck = model.part("deck")
    deck.visual(
        Box((1.50, 0.10, 0.06)),
        origin=Origin(xyz=(-0.80, 0.30, -0.05)),
        material=deck_color,
        name="left_rail",
    )
    deck.visual(
        Box((1.50, 0.10, 0.06)),
        origin=Origin(xyz=(-0.80, -0.30, -0.05)),
        material=deck_color,
        name="right_rail",
    )
    deck.visual(
        Box((0.20, 0.52, 0.03)),
        origin=Origin(xyz=(-0.24, 0.0, -0.02)),
        material=deck_color,
        name="front_platform",
    )
    deck.visual(
        Box((0.10, 0.08, 0.03)),
        origin=Origin(xyz=(-1.53, 0.295, -0.02)),
        material=deck_color,
        name="left_tail_cap",
    )
    deck.visual(
        Box((0.10, 0.08, 0.03)),
        origin=Origin(xyz=(-1.53, -0.295, -0.02)),
        material=deck_color,
        name="right_tail_cap",
    )
    deck.visual(
        Box((0.04, 0.05, 0.06)),
        origin=Origin(xyz=(-0.03, 0.315, -0.01)),
        material=deck_color,
        name="left_hinge_block",
    )
    deck.visual(
        Box((0.04, 0.05, 0.06)),
        origin=Origin(xyz=(-0.03, -0.315, -0.01)),
        material=deck_color,
        name="right_hinge_block",
    )
    deck.visual(
        Box((1.18, 0.56, 0.02)),
        origin=Origin(xyz=(-0.82, 0.0, -0.07)),
        material=deck_color,
        name="support_pan",
    )
    deck.visual(
        Box((0.06, 0.09, 0.04)),
        origin=Origin(xyz=(-1.55, 0.255, -0.08)),
        material=deck_color,
        name="left_rear_foot",
    )
    deck.visual(
        Box((0.06, 0.09, 0.04)),
        origin=Origin(xyz=(-1.55, -0.255, -0.08)),
        material=deck_color,
        name="right_rear_foot",
    )
    deck.visual(
        Cylinder(radius=0.022, length=0.05),
        origin=Origin(xyz=(-0.01, 0.315, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=deck_color,
        name="left_hinge_knuckle",
    )
    deck.visual(
        Cylinder(radius=0.022, length=0.05),
        origin=Origin(xyz=(-0.01, -0.315, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=deck_color,
        name="right_hinge_knuckle",
    )
    deck.visual(
        Box((0.06, 0.05, 0.03)),
        origin=Origin(xyz=(-0.08, 0.255, -0.01)),
        material=deck_color,
        name="front_left_bearing_block",
    )
    deck.visual(
        Box((0.06, 0.03, 0.03)),
        origin=Origin(xyz=(-0.08, -0.255, -0.01)),
        material=deck_color,
        name="front_right_bearing_block",
    )
    deck.visual(
        Box((0.06, 0.03, 0.03)),
        origin=Origin(xyz=(-1.46, 0.255, -0.01)),
        material=deck_color,
        name="rear_left_bearing_block",
    )
    deck.visual(
        Box((0.06, 0.03, 0.03)),
        origin=Origin(xyz=(-1.46, -0.255, -0.01)),
        material=deck_color,
        name="rear_right_bearing_block",
    )
    deck.visual(
        Box((1.56, 0.56, 0.012)),
        origin=Origin(xyz=(-0.80, 0.0, 0.004)),
        material=deck_color,
        name="deck_frame",
    )
    deck.visual(
        Box((1.22, 0.52, 0.014)),
        origin=Origin(xyz=(-0.82, 0.0, 0.017)),
        material=tread_color,
        name="tread_surface",
    )

    front_roller = model.part("front_roller")
    front_roller.visual(
        Cylinder(radius=0.03, length=0.40),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=roller_color,
        name="front_roller_drum",
    )
    front_roller.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(0.0, 0.22, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=roller_color,
        name="front_left_endcap",
    )
    front_roller.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(0.0, -0.22, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=roller_color,
        name="front_right_endcap",
    )

    rear_roller = model.part("rear_roller")
    rear_roller.visual(
        Cylinder(radius=0.03, length=0.40),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=roller_color,
        name="rear_roller_drum",
    )
    rear_roller.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(0.0, 0.22, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=roller_color,
        name="rear_left_endcap",
    )
    rear_roller.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(0.0, -0.22, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=roller_color,
        name="rear_right_endcap",
    )

    left_wheel = model.part("left_transport_wheel")
    left_wheel.visual(
        Cylinder(radius=0.035, length=0.022),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=wheel_color,
        name="left_transport_wheel_shell",
    )
    left_wheel.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=roller_color,
        name="left_transport_hub",
    )

    right_wheel = model.part("right_transport_wheel")
    right_wheel.visual(
        Cylinder(radius=0.035, length=0.022),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=wheel_color,
        name="right_transport_wheel_shell",
    )
    right_wheel.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
        material=roller_color,
        name="right_transport_hub",
    )

    model.articulation(
        "base_to_deck",
        ArticulationType.REVOLUTE,
        parent=base,
        child=deck,
        origin=Origin(xyz=DECK_HINGE_ORIGIN),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.2,
            lower=0.0,
            upper=DECK_FOLD_UPPER,
        ),
    )
    model.articulation(
        "deck_to_front_roller",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_roller,
        origin=Origin(xyz=(-0.08, 0.0, -0.043)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "deck_to_rear_roller",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_roller,
        origin=Origin(xyz=(-1.46, 0.0, -0.043)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )
    model.articulation(
        "base_to_left_transport_wheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=left_wheel,
        origin=Origin(xyz=(0.23, 0.352, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    model.articulation(
        "base_to_right_transport_wheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=right_wheel,
        origin=Origin(xyz=(0.23, -0.352, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    deck = object_model.get_part("deck")
    front_roller = object_model.get_part("front_roller")
    rear_roller = object_model.get_part("rear_roller")
    left_wheel = object_model.get_part("left_transport_wheel")
    right_wheel = object_model.get_part("right_transport_wheel")

    deck_hinge = object_model.get_articulation("base_to_deck")
    front_roller_joint = object_model.get_articulation("deck_to_front_roller")
    rear_roller_joint = object_model.get_articulation("deck_to_rear_roller")
    left_wheel_joint = object_model.get_articulation("base_to_left_transport_wheel")
    right_wheel_joint = object_model.get_articulation("base_to_right_transport_wheel")

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
        "deck hinge is a transverse folding revolute",
        deck_hinge.articulation_type == ArticulationType.REVOLUTE
        and deck_hinge.axis == (0.0, 1.0, 0.0)
        and deck_hinge.motion_limits is not None
        and deck_hinge.motion_limits.lower == 0.0
        and deck_hinge.motion_limits.upper is not None
        and deck_hinge.motion_limits.upper >= 1.1,
        details=(
            f"type={deck_hinge.articulation_type}, axis={deck_hinge.axis}, "
            f"limits={deck_hinge.motion_limits}"
        ),
    )
    ctx.check(
        "rollers and wheels use continuous transverse axles",
        all(
            joint.articulation_type == ArticulationType.CONTINUOUS and joint.axis == (0.0, 1.0, 0.0)
            for joint in (
                front_roller_joint,
                rear_roller_joint,
                left_wheel_joint,
                right_wheel_joint,
            )
        ),
        details=(
            f"front={front_roller_joint.axis}, rear={rear_roller_joint.axis}, "
            f"left={left_wheel_joint.axis}, right={right_wheel_joint.axis}"
        ),
    )
    ctx.expect_contact(
        deck,
        base,
        elem_b="left_hinge_bridge",
        name="left hinge bridge supports the deck at the front pivot",
    )
    ctx.expect_contact(
        deck,
        base,
        elem_b="right_hinge_bridge",
        name="right hinge bridge supports the deck at the front pivot",
    )

    ctx.expect_contact(
        front_roller,
        deck,
        name="front roller is mounted between the deck rails",
    )
    ctx.expect_contact(
        rear_roller,
        deck,
        name="rear roller is mounted between the deck rails",
    )
    ctx.expect_contact(
        left_wheel,
        base,
        name="left transport wheel is carried by the front fork",
    )
    ctx.expect_contact(
        right_wheel,
        base,
        name="right transport wheel is carried by the front fork",
    )
    ctx.expect_within(
        front_roller,
        deck,
        axes="y",
        margin=0.0,
        name="front roller stays within the deck width",
    )
    ctx.expect_within(
        rear_roller,
        deck,
        axes="y",
        margin=0.0,
        name="rear roller stays within the deck width",
    )

    base_aabb = ctx.part_world_aabb(base)
    deck_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "console frame reaches standing height",
        base_aabb is not None and base_aabb[1][2] > 1.30,
        details=f"base_aabb={base_aabb}",
    )
    ctx.check(
        "running deck has realistic treadmill length",
        deck_aabb is not None and (deck_aabb[1][0] - deck_aabb[0][0]) > 1.45,
        details=f"deck_aabb={deck_aabb}",
    )

    rest_rear_pos = ctx.part_world_position(rear_roller)
    with ctx.pose({deck_hinge: DECK_FOLD_UPPER}):
        folded_rear_pos = ctx.part_world_position(rear_roller)

    ctx.check(
        "deck folds upward from the front hinge",
        rest_rear_pos is not None
        and folded_rear_pos is not None
        and folded_rear_pos[2] > rest_rear_pos[2] + 1.0
        and folded_rear_pos[0] > rest_rear_pos[0] + 0.8,
        details=f"rest={rest_rear_pos}, folded={folded_rear_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
