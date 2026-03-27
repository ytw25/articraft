from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent

COUNTER_SIZE = (0.92, 0.66, 0.04)
COUNTER_CUTOUT = (0.68, 0.40)
DECK_SIZE = (0.76, 0.50, 0.004)
KNOB_RADIUS = 0.021
KNOB_DEPTH = 0.036

BURNER_LAYOUT = {
    "rear_left": {"xy": (-0.18, 0.115), "hole_r": 0.044, "body_r": 0.040, "rim_r": 0.050, "crown_r": 0.033, "cap_r": 0.026, "grate": 0.16},
    "rear_right": {"xy": (0.18, 0.115), "hole_r": 0.052, "body_r": 0.047, "rim_r": 0.058, "crown_r": 0.040, "cap_r": 0.032, "grate": 0.19},
    "front_left": {"xy": (-0.18, -0.045), "hole_r": 0.040, "body_r": 0.036, "rim_r": 0.046, "crown_r": 0.029, "cap_r": 0.023, "grate": 0.15},
    "front_right": {"xy": (0.18, -0.045), "hole_r": 0.042, "body_r": 0.038, "rim_r": 0.049, "crown_r": 0.031, "cap_r": 0.025, "grate": 0.16},
}

KNOB_LAYOUT = {
    "upper_left": (-0.285, -0.154),
    "upper_right": (0.285, -0.154),
    "lower_left": (-0.285, -0.214),
    "lower_right": (0.285, -0.214),
}


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, HERE / name)


def _translate_profile(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return superellipse_profile(radius * 2.0, radius * 2.0, exponent=2.0, segments=segments)


def _add_burner_assembly(model: ArticulatedObject, deck, name: str, spec: dict[str, float], burner_steel, burner_black, grate_iron) -> None:
    x, y = spec["xy"]

    burner = model.part(f"{name}_burner")
    burner.inertial = Inertial.from_geometry(
        Cylinder(radius=spec["rim_r"], length=0.050),
        mass=1.2 if spec["grate"] < 0.20 else 1.6,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )
    burner.visual(Cylinder(radius=spec["body_r"], length=0.042), origin=Origin(xyz=(0.0, 0.0, -0.021)), material=burner_steel, name="bowl")
    burner.visual(Cylinder(radius=spec["rim_r"], length=0.004), origin=Origin(xyz=(0.0, 0.0, 0.002)), material=burner_steel, name="rim")
    burner.visual(Cylinder(radius=spec["crown_r"], length=0.010), origin=Origin(xyz=(0.0, 0.0, 0.009)), material=burner_black, name="crown")
    burner.visual(Cylinder(radius=spec["cap_r"], length=0.012), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=burner_black, name="cap")

    grate = model.part(f"{name}_grate")
    grate.inertial = Inertial.from_geometry(
        Box((spec["grate"], spec["grate"], 0.040)),
        mass=1.0 if spec["grate"] < 0.20 else 1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )
    size = spec["grate"]
    support_span = size * 0.74
    loop_radius = 0.006
    loop_half = support_span * 0.5 - loop_radius
    frame_z = 0.036
    frame_h = loop_radius * 2.0
    foot = 0.016
    foot_h = 0.030
    cross_w = 0.012
    grate_loop = wire_from_points(
        [
            (-loop_half, -loop_half, frame_z),
            (-loop_half, loop_half, frame_z),
            (loop_half, loop_half, frame_z),
            (loop_half, -loop_half, frame_z),
        ],
        radius=loop_radius,
        closed_path=True,
        cap_ends=False,
        corner_mode="fillet",
        corner_radius=0.016,
        corner_segments=10,
    )
    grate.visual(_save_mesh(f"{name}_grate_loop.obj", grate_loop), material=grate_iron, name="outer_loop")
    grate.visual(
        Box((2.0 * (loop_half - loop_radius), cross_w, frame_h)),
        origin=Origin(xyz=(0.0, 0.0, frame_z)),
        material=grate_iron,
        name="cross_x",
    )
    grate.visual(
        Box((cross_w, 2.0 * (loop_half - loop_radius), frame_h)),
        origin=Origin(xyz=(0.0, 0.0, frame_z)),
        material=grate_iron,
        name="cross_y",
    )
    for suffix, dx, dy in (
        ("rear_left", -loop_half, loop_half),
        ("rear_right", loop_half, loop_half),
        ("front_left", -loop_half, -loop_half),
        ("front_right", loop_half, -loop_half),
    ):
        grate.visual(
            Box((foot, foot, foot_h)),
            origin=Origin(xyz=(dx, dy, foot_h * 0.5)),
            material=grate_iron,
            name=f"foot_{suffix}",
        )

    model.articulation(
        f"{name}_burner_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=burner,
        origin=Origin(xyz=(x, y, DECK_SIZE[2])),
    )
    model.articulation(
        f"{name}_grate_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=grate,
        origin=Origin(xyz=(x, y, DECK_SIZE[2])),
    )


def _add_knob(model: ArticulatedObject, deck, name: str, x: float, y: float, knob_metal, knob_dark) -> None:
    knob = model.part(f"{name}_knob")
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
        mass=0.18,
        origin=Origin(xyz=(0.0, -KNOB_DEPTH * 0.5, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
    )
    knob.visual(
        Cylinder(radius=KNOB_RADIUS, length=KNOB_DEPTH),
        origin=Origin(xyz=(0.0, -KNOB_DEPTH * 0.5, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=knob_metal,
        name="dial",
    )
    knob.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, -KNOB_DEPTH + 0.003, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=knob_dark,
        name="hub",
    )
    knob.visual(
        Box((0.008, 0.010, KNOB_RADIUS * 0.90)),
        origin=Origin(xyz=(KNOB_RADIUS * 0.55, -KNOB_DEPTH * 0.72, 0.0)),
        material=knob_dark,
        name="pointer",
    )

    model.articulation(
        f"{name}_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=knob,
        origin=Origin(xyz=(x, y, 0.033)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_gas_cooktop", assets=ASSETS)

    stone = model.material("stone", rgba=(0.84, 0.84, 0.82, 1.0))
    black_glass = model.material("black_glass", rgba=(0.10, 0.10, 0.11, 1.0))
    burner_steel = model.material("burner_steel", rgba=(0.63, 0.64, 0.66, 1.0))
    burner_black = model.material("burner_black", rgba=(0.17, 0.17, 0.18, 1.0))
    grate_iron = model.material("grate_iron", rgba=(0.16, 0.16, 0.16, 1.0))
    knob_metal = model.material("knob_metal", rgba=(0.75, 0.77, 0.79, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    counter = model.part("counter")
    counter.inertial = Inertial.from_geometry(
        Box(COUNTER_SIZE),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, COUNTER_SIZE[2] * 0.5)),
    )
    counter_shell = ExtrudeWithHolesGeometry(
        rounded_rect_profile(COUNTER_SIZE[0], COUNTER_SIZE[1], 0.030, corner_segments=10),
        [rounded_rect_profile(COUNTER_CUTOUT[0], COUNTER_CUTOUT[1], 0.018, corner_segments=8)],
        COUNTER_SIZE[2],
        center=False,
    )
    counter.visual(_save_mesh("cooktop_counter_shell.obj", counter_shell), material=stone, name="counter_shell")

    deck = model.part("cooktop_deck")
    deck.inertial = Inertial.from_geometry(
        Box((DECK_SIZE[0], DECK_SIZE[1], 0.090)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )
    deck_holes = [
        _translate_profile(_circle_profile(spec["hole_r"]), spec["xy"][0], spec["xy"][1])
        for spec in BURNER_LAYOUT.values()
    ]
    deck_plate = ExtrudeWithHolesGeometry(
        rounded_rect_profile(DECK_SIZE[0], DECK_SIZE[1], 0.016, corner_segments=10),
        deck_holes,
        DECK_SIZE[2],
        center=False,
    )
    deck.visual(_save_mesh("cooktop_deck_plate.obj", deck_plate), material=black_glass, name="deck_plate")
    deck.visual(
        Box((0.72, 0.12, 0.008)),
        origin=Origin(xyz=(0.0, -0.19, 0.008)),
        material=black_glass,
        name="control_fascia",
    )

    model.articulation(
        "counter_to_deck",
        ArticulationType.FIXED,
        parent=counter,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, COUNTER_SIZE[2])),
    )

    for burner_name, spec in BURNER_LAYOUT.items():
        _add_burner_assembly(model, deck, burner_name, spec, burner_steel, burner_black, grate_iron)

    for knob_name, (x, y) in KNOB_LAYOUT.items():
        _add_knob(model, deck, knob_name, x, y, knob_metal, knob_dark)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    counter = object_model.get_part("counter")
    deck = object_model.get_part("cooktop_deck")
    burner_parts = {name: object_model.get_part(f"{name}_burner") for name in BURNER_LAYOUT}
    grate_parts = {name: object_model.get_part(f"{name}_grate") for name in BURNER_LAYOUT}
    knob_parts = {name: object_model.get_part(f"{name}_knob") for name in KNOB_LAYOUT}
    knob_joints = {name: object_model.get_articulation(f"{name}_knob_spin") for name in KNOB_LAYOUT}

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=20, name="knob_rotation_clearance")
    ctx.fail_if_isolated_parts(max_pose_samples=10, name="sampled_pose_no_floating")

    ctx.expect_contact(deck, counter, elem_a="deck_plate", elem_b="counter_shell", name="deck_supported_by_counter")
    ctx.expect_within(deck, counter, axes="xy", inner_elem="deck_plate", outer_elem="counter_shell", margin=0.0, name="deck_within_counter_bounds")
    ctx.expect_overlap(deck, counter, axes="xy", elem_a="deck_plate", elem_b="counter_shell", min_overlap=0.45, name="deck_overlaps_counter_plan")

    for name in BURNER_LAYOUT:
        burner = burner_parts[name]
        grate = grate_parts[name]
        spec = BURNER_LAYOUT[name]
        ctx.expect_contact(burner, deck, elem_a="rim", elem_b="deck_plate", name=f"{name}_burner_supported")
        ctx.expect_contact(grate, deck, name=f"{name}_grate_supported")
        ctx.expect_overlap(grate, burner, axes="xy", min_overlap=spec["cap_r"] * 2.0, name=f"{name}_grate_covers_burner")

    for name in KNOB_LAYOUT:
        knob = knob_parts[name]
        joint = knob_joints[name]
        limits = joint.motion_limits
        ctx.expect_contact(knob, deck, elem_a="dial", elem_b="control_fascia", name=f"{name}_knob_mounted")
        ctx.check(
            f"{name}_knob_axis_is_front_to_back",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"{name}_knob_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )
        ctx.check(
            f"{name}_knob_has_unbounded_rotation",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"limits={limits}",
        )
        for angle, label in ((0.0, "rest"), (pi * 0.5, "quarter_turn"), (pi, "half_turn")):
            with ctx.pose({joint: angle}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{name}_{label}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{name}_{label}_no_floating")

    burner_positions = {name: ctx.part_world_position(part) for name, part in burner_parts.items()}
    knob_positions = {name: ctx.part_world_position(part) for name, part in knob_parts.items()}

    ctx.check(
        "burners_form_two_rows",
        burner_positions["rear_left"][1] > 0.05
        and burner_positions["rear_right"][1] > 0.05
        and burner_positions["front_left"][1] < -0.04
        and burner_positions["front_right"][1] < -0.04,
        details=str(burner_positions),
    )
    ctx.check(
        "burners_span_left_and_right",
        burner_positions["rear_left"][0] < -0.10
        and burner_positions["front_left"][0] < -0.10
        and burner_positions["rear_right"][0] > 0.10
        and burner_positions["front_right"][0] > 0.10,
        details=str(burner_positions),
    )
    ctx.check(
        "knobs_stay_at_control_area_corners",
        knob_positions["upper_left"][0] < -0.20
        and knob_positions["lower_left"][0] < -0.20
        and knob_positions["upper_right"][0] > 0.20
        and knob_positions["lower_right"][0] > 0.20
        and knob_positions["upper_left"][1] > knob_positions["lower_left"][1]
        and knob_positions["upper_right"][1] > knob_positions["lower_right"][1]
        and min(position[1] for position in knob_positions.values()) < -0.16
        and max(position[1] for position in knob_positions.values()) < -0.12,
        details=str(knob_positions),
    )
    ctx.check(
        "center_of_control_area_open",
        min(abs(position[0]) for position in knob_positions.values()) > 0.20,
        details=str(knob_positions),
    )
    ctx.check(
        "only_four_nonfixed_articulations",
        len([joint for joint in object_model.articulations if joint.articulation_type != ArticulationType.FIXED]) == 4,
        details=str([(joint.name, joint.articulation_type) for joint in object_model.articulations]),
    )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
