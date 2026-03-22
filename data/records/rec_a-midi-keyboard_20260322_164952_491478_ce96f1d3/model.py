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
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _named_visual(part: object, name: str) -> object:
    for visual in getattr(part, "visuals", []) or []:
        if getattr(visual, "name", None) == name:
            return visual
    raise ValueError(f"Unknown visual {name!r} on part {getattr(part, 'name', part)!r}")


def _wedge_section(
    x: float,
    y_front: float,
    y_rear: float,
    z_bottom: float,
    z_front_top: float,
    z_rear_top: float,
) -> list[tuple[float, float, float]]:
    depth = y_rear - y_front
    front_r = min(0.006, 0.18 * depth, 0.45 * max(z_front_top - z_bottom, 0.001))
    rear_r = min(0.008, 0.18 * depth, 0.45 * max(z_rear_top - z_bottom, 0.001))
    slope = z_rear_top - z_front_top
    return [
        (x, y_front + front_r, z_bottom),
        (x, y_front, z_bottom + 0.35 * front_r),
        (x, y_front, z_front_top - 0.65 * front_r),
        (x, y_front + front_r, z_front_top),
        (x, y_front + 0.28 * depth, z_front_top + 0.30 * slope),
        (x, y_front + 0.68 * depth, z_front_top + 0.78 * slope),
        (x, y_rear - rear_r, z_rear_top),
        (x, y_rear, z_rear_top - 0.65 * rear_r),
        (x, y_rear, z_bottom + 0.35 * rear_r),
        (x, y_rear - rear_r, z_bottom),
        (x, y_front + 0.72 * depth, z_bottom),
        (x, y_front + 0.30 * depth, z_bottom),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="midi_keyboard", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_panel = model.material("dark_panel", rgba=(0.19, 0.20, 0.22, 1.0))
    warm_white = model.material("warm_white", rgba=(0.93, 0.93, 0.91, 1.0))
    satin_black = model.material("satin_black", rgba=(0.06, 0.06, 0.07, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.24, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.61, 0.63, 1.0))
    glass = model.material("display_glass", rgba=(0.10, 0.13, 0.16, 0.82))

    chassis = model.part("chassis")

    lower_shell = mesh_from_geometry(
        repair_loft(
            section_loft(
                [
                    _wedge_section(-0.240, -0.095, 0.095, 0.000, 0.0100, 0.0140),
                    _wedge_section(-0.080, -0.095, 0.095, 0.000, 0.0106, 0.0146),
                    _wedge_section(0.240, -0.094, 0.094, 0.000, 0.0090, 0.0130),
                ]
            )
        ),
        ASSETS.mesh_path("midi_lower_shell.obj"),
    )
    rear_deck = mesh_from_geometry(
        repair_loft(
            section_loft(
                [
                    _wedge_section(-0.135, 0.040, 0.095, 0.0135, 0.0260, 0.0400),
                    _wedge_section(0.050, 0.042, 0.095, 0.0135, 0.0240, 0.0380),
                    _wedge_section(0.235, 0.045, 0.094, 0.0132, 0.0220, 0.0350),
                ]
            )
        ),
        ASSETS.mesh_path("midi_rear_deck.obj"),
    )

    chassis.visual(lower_shell, material=matte_black, name="lower_shell")
    chassis.visual(rear_deck, material=matte_black, name="rear_deck")
    chassis.visual(
        Box((0.372, 0.108, 0.003)),
        origin=Origin(xyz=(0.040, -0.036, 0.0130)),
        material=dark_panel,
        name="keybed",
    )
    chassis.visual(
        Box((0.074, 0.094, 0.004)),
        origin=Origin(xyz=(-0.193, 0.003, 0.0130)),
        material=dark_panel,
        name="wheel_panel",
    )

    for wheel_name, wheel_y in (("pitch", -0.020), ("mod", 0.026)):
        chassis.visual(
            Box((0.044, 0.024, 0.004)),
            origin=Origin(xyz=(-0.193, wheel_y, 0.0120)),
            material=dark_panel,
            name=f"{wheel_name}_dock",
        )
        for side, support_x in (("left", -0.214), ("right", -0.172)):
            chassis.visual(
                Box((0.006, 0.016, 0.018)),
                origin=Origin(xyz=(support_x, wheel_y, 0.0232)),
                material=dark_panel,
                name=f"{wheel_name}_support_{side}",
            )

    chassis.visual(
        Box((0.034, 0.020, 0.003)),
        origin=Origin(xyz=(-0.110, 0.069, 0.0360)),
        material=glass,
        name="display",
    )

    for index, button_x in enumerate((-0.072, -0.054, -0.036, -0.018), start=1):
        chassis.visual(
            Box((0.012, 0.010, 0.004)),
            origin=Origin(xyz=(button_x, 0.047, 0.0370)),
            material=graphite,
            name=f"button_{index:02d}",
        )

    for index in range(8):
        chassis.visual(
            Cylinder(radius=0.0075, length=0.012),
            origin=Origin(xyz=(-0.010 + 0.031 * index, 0.068, 0.0430)),
            material=graphite,
            name=f"knob_{index + 1:02d}",
        )

    keyboard_x0 = -0.143
    keyboard_len = 0.366
    white_key_pitch = keyboard_len / 15.0
    white_key_width = white_key_pitch - 0.0012
    white_key_y = -0.040
    white_key_length = 0.110
    white_key_height = 0.011
    white_key_center_z = 0.0145 + 0.5 * white_key_height

    for index in range(15):
        chassis.visual(
            Box((white_key_width, white_key_length, white_key_height)),
            origin=Origin(
                xyz=(
                    keyboard_x0 + (index + 0.5) * white_key_pitch,
                    white_key_y,
                    white_key_center_z,
                )
            ),
            material=warm_white,
            name=f"white_key_{index + 1:02d}",
        )

    black_after_white = (0, 1, 3, 4, 5, 7, 8, 10, 11, 12)
    for black_index, white_index in enumerate(black_after_white, start=1):
        chassis.visual(
            Box((0.014, 0.068, 0.019)),
            origin=Origin(
                xyz=(
                    keyboard_x0 + (white_index + 1.0) * white_key_pitch,
                    -0.015,
                    0.0245,
                )
            ),
            material=satin_black,
            name=f"black_key_{black_index:02d}",
        )

    chassis.inertial = Inertial.from_geometry(
        Box((0.480, 0.190, 0.050)),
        mass=1.8,
        origin=Origin(xyz=(0.000, 0.000, 0.025)),
    )

    pitch_wheel = model.part("pitch_wheel")
    pitch_wheel.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
        material=graphite,
        name="pitch_tire",
    )
    pitch_wheel.visual(
        Cylinder(radius=0.0025, length=0.038),
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
        material=steel,
        name="pitch_axle",
    )
    pitch_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.024),
        mass=0.045,
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
    )

    mod_wheel = model.part("mod_wheel")
    mod_wheel.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
        material=graphite,
        name="mod_tire",
    )
    mod_wheel.visual(
        Cylinder(radius=0.0025, length=0.038),
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
        material=steel,
        name="mod_axle",
    )
    mod_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.024),
        mass=0.045,
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
    )

    model.articulation(
        "pitch_bend",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=pitch_wheel,
        origin=Origin(xyz=(-0.193, -0.020, 0.0272)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "modulation_wheel",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=mod_wheel,
        origin=Origin(xyz=(-0.193, 0.026, 0.0272)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=0.0, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    pitch_wheel = object_model.get_part("pitch_wheel")
    mod_wheel = object_model.get_part("mod_wheel")
    pitch_bend = object_model.get_articulation("pitch_bend")
    modulation_wheel = object_model.get_articulation("modulation_wheel")
    rear_deck = _named_visual(chassis, "rear_deck")
    keybed = _named_visual(chassis, "keybed")
    wheel_panel = _named_visual(chassis, "wheel_panel")
    pitch_dock = _named_visual(chassis, "pitch_dock")
    mod_dock = _named_visual(chassis, "mod_dock")
    pitch_tire = _named_visual(pitch_wheel, "pitch_tire")
    mod_tire = _named_visual(mod_wheel, "mod_tire")
    display = _named_visual(chassis, "display")
    first_button = _named_visual(chassis, "button_01")
    last_button = _named_visual(chassis, "button_04")
    first_white = _named_visual(chassis, "white_key_01")
    last_white = _named_visual(chassis, "white_key_15")
    mid_white = _named_visual(chassis, "white_key_08")
    first_black = _named_visual(chassis, "black_key_01")
    last_black = _named_visual(chassis, "black_key_10")
    first_knob = _named_visual(chassis, "knob_01")
    last_knob = _named_visual(chassis, "knob_08")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(pitch_wheel, chassis, reason="wheel axle nests in the molded wheel supports")
    ctx.allow_overlap(mod_wheel, chassis, reason="wheel axle nests in the molded wheel supports")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_gap(
        chassis,
        chassis,
        axis="x",
        positive_elem=last_white,
        negative_elem=first_white,
        min_gap=0.300,
        name="white_key_bank_spans_keyboard_width",
    )
    ctx.expect_aabb_gap(
        chassis,
        chassis,
        axis="x",
        positive_elem=last_black,
        negative_elem=first_black,
        min_gap=0.220,
        name="black_key_bank_spans_middle_of_keyboard",
    )
    ctx.expect_aabb_gap(
        chassis,
        chassis,
        axis="y",
        positive_elem=rear_deck,
        negative_elem=mid_white,
        min_gap=0.020,
        max_gap=0.030,
        name="rear_control_deck_sits_behind_keys",
    )
    ctx.expect_aabb_gap(
        chassis,
        chassis,
        axis="z",
        positive_elem=first_knob,
        negative_elem=keybed,
        min_gap=0.020,
        name="knobs_stand_proud_of_the_control_surface",
    )
    ctx.expect_aabb_gap(
        chassis,
        chassis,
        axis="x",
        positive_elem=first_white,
        negative_elem=wheel_panel,
        min_gap=0.010,
        name="wheel_section_sits_left_of_the_keybed",
    )
    ctx.expect_aabb_gap(
        chassis,
        chassis,
        axis="x",
        positive_elem=last_knob,
        negative_elem=first_knob,
        min_gap=0.190,
        name="knob_row_spans_the_right_control_bank",
    )
    ctx.expect_aabb_gap(
        chassis,
        chassis,
        axis="y",
        positive_elem=display,
        negative_elem=mid_white,
        min_gap=0.040,
        name="display_sits_on_the_rear_panel_above_keys",
    )
    ctx.expect_aabb_gap(
        chassis,
        chassis,
        axis="x",
        positive_elem=last_button,
        negative_elem=first_button,
        min_gap=0.040,
        name="transport_buttons_form_a_small_button_row",
    )
    ctx.expect_aabb_gap(
        pitch_wheel,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=0.006,
        positive_elem=pitch_tire,
        negative_elem=pitch_dock,
        name="pitch_wheel_sits_on_its_dock",
    )
    ctx.expect_aabb_gap(
        mod_wheel,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=0.006,
        positive_elem=mod_tire,
        negative_elem=mod_dock,
        name="mod_wheel_sits_on_its_dock",
    )
    ctx.expect_aabb_overlap(pitch_wheel, chassis, axes="yz", min_overlap=0.024)
    ctx.expect_aabb_overlap(mod_wheel, chassis, axes="yz", min_overlap=0.024)
    ctx.expect_origin_distance(pitch_wheel, mod_wheel, axes="x", max_dist=0.002)
    ctx.expect_aabb_gap(
        mod_wheel,
        pitch_wheel,
        axis="y",
        min_gap=0.010,
        max_gap=0.020,
        name="mod_wheel_sits_behind_pitch_wheel",
    )
    with ctx.pose({pitch_bend: -0.65, modulation_wheel: 0.80}):
        ctx.expect_aabb_gap(
            pitch_wheel,
            chassis,
            axis="z",
            max_gap=0.001,
            max_penetration=0.006,
            positive_elem=pitch_tire,
            negative_elem=pitch_dock,
            name="pitch_wheel_stays_seated_at_full_throw",
        )
        ctx.expect_aabb_gap(
            mod_wheel,
            chassis,
            axis="z",
            max_gap=0.001,
            max_penetration=0.006,
            positive_elem=mod_tire,
            negative_elem=mod_dock,
            name="mod_wheel_stays_seated_at_upper_throw",
        )
        ctx.expect_aabb_overlap(pitch_wheel, chassis, axes="yz", min_overlap=0.024)
        ctx.expect_aabb_overlap(mod_wheel, chassis, axes="yz", min_overlap=0.024)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
