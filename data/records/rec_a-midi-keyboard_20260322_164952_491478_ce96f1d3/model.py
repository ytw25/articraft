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
    MeshGeometry,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
)

ASSETS = AssetContext.from_script(__file__)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _build_rear_housing_mesh() -> MeshGeometry:
    geom = MeshGeometry()

    vertices = [
        (-0.250, 0.000, 0.000),
        (0.250, 0.000, 0.000),
        (0.250, 0.095, 0.000),
        (-0.250, 0.095, 0.000),
        (-0.246, 0.000, 0.028),
        (0.246, 0.000, 0.028),
        (0.248, 0.095, 0.044),
        (-0.248, 0.095, 0.044),
    ]
    ids = [geom.add_vertex(*vertex) for vertex in vertices]

    _add_quad(geom, ids[0], ids[1], ids[2], ids[3])  # bottom
    _add_quad(geom, ids[4], ids[7], ids[6], ids[5])  # top
    _add_quad(geom, ids[0], ids[4], ids[5], ids[1])  # front
    _add_quad(geom, ids[1], ids[5], ids[6], ids[2])  # right
    _add_quad(geom, ids[2], ids[6], ids[7], ids[3])  # back
    _add_quad(geom, ids[3], ids[7], ids[4], ids[0])  # left
    return geom

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="midi_keyboard", assets=ASSETS)

    body = model.material("body_charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    deck = model.material("deck_black", rgba=(0.10, 0.11, 0.12, 1.0))
    key_white = model.material("key_white", rgba=(0.94, 0.95, 0.96, 1.0))
    key_black = model.material("key_black", rgba=(0.08, 0.08, 0.09, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.22, 0.23, 0.25, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.12, 0.12, 0.13, 1.0))
    display_glass = model.material("display_glass", rgba=(0.18, 0.29, 0.34, 0.45))
    accent = model.material("accent_grey", rgba=(0.44, 0.46, 0.49, 1.0))

    chassis = model.part("chassis")
    rear_housing_mesh = mesh_from_geometry(
        _build_rear_housing_mesh(),
        ASSETS.mesh_path("midi_keyboard_rear_housing.obj"),
    )
    rear_housing = chassis.visual(rear_housing_mesh, material=body, name="rear_housing")
    chassis.visual(
        Box((0.500, 0.100, 0.018)),
        origin=Origin(xyz=(0.000, -0.050, 0.009)),
        material=body,
        name="front_base",
    )
    chassis.visual(
        Box((0.382, 0.010, 0.020)),
        origin=Origin(xyz=(0.000, 0.030, 0.010)),
        material=deck,
        name="rear_key_wall",
    )
    chassis.visual(
        Box((0.010, 0.115, 0.024)),
        origin=Origin(xyz=(-0.186, -0.026, 0.012)),
        material=deck,
        name="left_key_cheek",
    )
    chassis.visual(
        Box((0.010, 0.115, 0.024)),
        origin=Origin(xyz=(0.186, -0.026, 0.012)),
        material=deck,
        name="right_key_cheek",
    )
    chassis.visual(
        Box((0.358, 0.114, 0.002)),
        origin=Origin(xyz=(0.000, -0.026, 0.017)),
        material=deck,
        name="keybed_seat",
    )
    chassis.visual(
        Box((0.322, 0.034, 0.002)),
        origin=Origin(xyz=(0.000, -0.004, 0.023)),
        material=deck,
        name="black_key_seat",
    )
    chassis.visual(
        Box((0.496, 0.008, 0.006)),
        origin=Origin(xyz=(0.000, -0.094, 0.015)),
        material=accent,
        name="front_trim",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((0.500, 0.195, 0.055)),
        mass=2.8,
        origin=Origin(xyz=(0.000, 0.000, 0.0275)),
    )

    white_keys = model.part("white_keys")
    white_keys.visual(
        Box((0.358, 0.114, 0.002)),
        material=deck,
        name="white_key_support",
    )
    white_key_width = 0.0222
    white_key_pitch = 0.0235
    white_key_centers = [(-7 + index) * white_key_pitch for index in range(15)]
    for index, center_x in enumerate(white_key_centers):
        white_keys.visual(
            Box((white_key_width, 0.112, 0.008)),
            origin=Origin(xyz=(center_x, -0.001, 0.004)),
            material=key_white,
            name=f"white_key_body_{index}",
        )
        white_keys.visual(
            Box((white_key_width * 0.9, 0.080, 0.014)),
            origin=Origin(xyz=(center_x, -0.018, 0.008)),
            material=key_white,
            name=f"white_key_{index}",
        )
    white_keys.inertial = Inertial.from_geometry(
        Box((0.358, 0.114, 0.016)),
        mass=0.65,
        origin=Origin(xyz=(0.000, 0.000, 0.008)),
    )

    black_keys = model.part("black_keys")
    black_keys.visual(
        Box((0.322, 0.034, 0.004)),
        material=deck,
        name="black_key_rail",
    )
    black_key_positions = [0, 1, 3, 4, 5, 7, 8, 10, 11, 12]
    for visual_index, white_index in enumerate(black_key_positions):
        key_center_x = (white_key_centers[white_index] + white_key_centers[white_index + 1]) / 2.0
        black_keys.visual(
            Box((0.010, 0.022, 0.012)),
            origin=Origin(
                xyz=(
                    key_center_x,
                    0.010,
                    0.008,
                )
            ),
            material=key_black,
            name=f"black_key_stem_{visual_index}",
        )
        black_keys.visual(
            Box((0.013, 0.042, 0.014)),
            origin=Origin(
                xyz=(
                    key_center_x,
                    -0.014,
                    0.015,
                )
            ),
            material=key_black,
            name=f"black_key_{visual_index}",
        )
        black_keys.visual(
            Box((0.010, 0.024, 0.004)),
            origin=Origin(
                xyz=(
                    key_center_x,
                    -0.012,
                    0.024,
                )
            ),
            material=key_black,
            name=f"black_key_crown_{visual_index}",
        )
    black_keys.inertial = Inertial.from_geometry(
        Box((0.322, 0.060, 0.028)),
        mass=0.28,
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
    )

    def _mount_to_chassis_surface(
        child_part,
        *,
        articulation_name: str,
        point_hint: tuple[float, float, float],
        clearance: float = 0.0,
    ) -> None:
        model.articulation(
            articulation_name,
            ArticulationType.FIXED,
            parent=chassis,
            child=child_part,
            origin=place_on_surface(
                child_part,
                rear_housing,
                point_hint=point_hint,
                clearance=clearance,
                asset_root=ASSETS.asset_root,
                prefer_collisions=False,
                child_prefer_collisions=False,
            ),
        )

    display_part = model.part("display_module")
    display_part.visual(
        Box((0.050, 0.024, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=display_glass,
        name="display",
    )
    display_part.inertial = Inertial.from_geometry(
        Box((0.050, 0.024, 0.006)),
        mass=0.04,
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
    )
    _mount_to_chassis_surface(
        display_part,
        articulation_name="chassis_to_display_module",
        point_hint=(-0.012, 0.060, 0.080),
    )

    first_knob_part = None
    for row_index, local_y in enumerate((-0.014, 0.014)):
        for column_index, local_x in enumerate((-0.040, -0.010, 0.020, 0.050)):
            knob_part = model.part(f"knob_{row_index}_{column_index}_part")
            knob_part.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(0.000, 0.000, 0.005)),
                material=knob_dark,
                name="knob",
            )
            knob_part.inertial = Inertial.from_geometry(
                Cylinder(radius=0.008, length=0.010),
                mass=0.012,
                origin=Origin(xyz=(0.000, 0.000, 0.005)),
            )
            _mount_to_chassis_surface(
                knob_part,
                articulation_name=f"chassis_to_knob_{row_index}_{column_index}",
                point_hint=(0.055 + local_x, 0.056 + local_y, 0.080),
            )
            if first_knob_part is None:
                first_knob_part = knob_part

    first_pad_part = None
    for row_index, local_y in enumerate((-0.014, 0.014)):
        for column_index, local_x in enumerate((0.098, 0.128)):
            pad_part = model.part(f"pad_{row_index}_{column_index}_part")
            pad_part.visual(
                Box((0.024, 0.024, 0.005)),
                origin=Origin(xyz=(0.000, 0.000, 0.0025)),
                material=pad_rubber,
                name="pad",
            )
            pad_part.inertial = Inertial.from_geometry(
                Box((0.024, 0.024, 0.005)),
                mass=0.02,
                origin=Origin(xyz=(0.000, 0.000, 0.0025)),
            )
            _mount_to_chassis_surface(
                pad_part,
                articulation_name=f"chassis_to_pad_{row_index}_{column_index}",
                point_hint=(0.055 + local_x, 0.056 + local_y, 0.080),
            )
            if first_pad_part is None:
                first_pad_part = pad_part

    for index, local_x in enumerate((0.066, 0.088, 0.110)):
        button_part = model.part(f"transport_button_{index}_part")
        button_part.visual(
            Box((0.010, 0.026, 0.004)),
            origin=Origin(xyz=(0.000, 0.000, 0.002)),
            material=accent,
            name="transport_button",
        )
        button_part.inertial = Inertial.from_geometry(
            Box((0.010, 0.026, 0.004)),
            mass=0.006,
            origin=Origin(xyz=(0.000, 0.000, 0.002)),
        )
        _mount_to_chassis_surface(
            button_part,
            articulation_name=f"chassis_to_transport_button_{index}",
            point_hint=(0.055 + local_x, 0.024, 0.070),
        )

    pitch_wheel_part = model.part("pitch_wheel_part")
    pitch_wheel_part.visual(
        Box((0.012, 0.032, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        material=deck,
        name="pitch_wheel_floor",
    )
    pitch_wheel_part.visual(
        Box((0.003, 0.032, 0.010)),
        origin=Origin(xyz=(-0.0065, 0.000, 0.005)),
        material=deck,
        name="pitch_wheel_left_cheek",
    )
    pitch_wheel_part.visual(
        Box((0.003, 0.032, 0.010)),
        origin=Origin(xyz=(0.0065, 0.000, 0.005)),
        material=deck,
        name="pitch_wheel_right_cheek",
    )
    pitch_wheel_part.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.007), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="pitch_wheel",
    )
    pitch_wheel_part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.010),
        mass=0.02,
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
    )
    _mount_to_chassis_surface(
        pitch_wheel_part,
        articulation_name="chassis_to_pitch_wheel",
        point_hint=(-0.224, 0.026, 0.070),
    )

    mod_wheel_part = model.part("mod_wheel_part")
    mod_wheel_part.visual(
        Box((0.012, 0.032, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        material=deck,
        name="mod_wheel_floor",
    )
    mod_wheel_part.visual(
        Box((0.003, 0.032, 0.010)),
        origin=Origin(xyz=(-0.0065, 0.000, 0.005)),
        material=deck,
        name="mod_wheel_left_cheek",
    )
    mod_wheel_part.visual(
        Box((0.003, 0.032, 0.010)),
        origin=Origin(xyz=(0.0065, 0.000, 0.005)),
        material=deck,
        name="mod_wheel_right_cheek",
    )
    mod_wheel_part.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.007), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="mod_wheel",
    )
    mod_wheel_part.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.010),
        mass=0.02,
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
    )
    _mount_to_chassis_surface(
        mod_wheel_part,
        articulation_name="chassis_to_mod_wheel",
        point_hint=(-0.198, 0.026, 0.070),
    )

    model.articulation(
        "chassis_to_white_keys",
        ArticulationType.FIXED,
        parent=chassis,
        child=white_keys,
        origin=Origin(xyz=(0.000, -0.026, 0.019)),
    )
    model.articulation(
        "chassis_to_black_keys",
        ArticulationType.FIXED,
        parent=chassis,
        child=black_keys,
        origin=Origin(xyz=(0.000, -0.004, 0.026)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    white_keys = object_model.get_part("white_keys")
    black_keys = object_model.get_part("black_keys")
    display_part = object_model.get_part("display_module")
    first_knob_part = object_model.get_part("knob_0_0_part")
    first_pad_part = object_model.get_part("pad_0_0_part")
    pitch_wheel_part = object_model.get_part("pitch_wheel_part")
    mod_wheel_part = object_model.get_part("mod_wheel_part")

    rear_housing = chassis.get_visual("rear_housing")
    keybed_seat = chassis.get_visual("keybed_seat")
    black_key_seat = chassis.get_visual("black_key_seat")
    white_key_support = white_keys.get_visual("white_key_support")
    first_white = white_keys.get_visual("white_key_0")
    black_key_rail = black_keys.get_visual("black_key_rail")
    first_black = black_keys.get_visual("black_key_0")
    display = display_part.get_visual("display")
    first_knob = first_knob_part.get_visual("knob")
    first_pad = first_pad_part.get_visual("pad")
    pitch_wheel = pitch_wheel_part.get_visual("pitch_wheel")
    mod_wheel = mod_wheel_part.get_visual("mod_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    if any(
        articulation.articulation_type != ArticulationType.FIXED
        for articulation in object_model.articulations
    ):
        ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        white_keys,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem=white_key_support,
        negative_elem=keybed_seat,
        name="white_keys_seated_on_keybed",
    )
    ctx.expect_gap(
        black_keys,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=black_key_rail,
        negative_elem=black_key_seat,
        name="black_keys_seated_on_rail",
    )
    ctx.expect_overlap(white_keys, chassis, axes="xy", min_overlap=0.10)
    ctx.expect_overlap(black_keys, white_keys, axes="xy", min_overlap=0.05)
    ctx.expect_within(white_keys, chassis, axes="xy")
    ctx.expect_within(black_keys, chassis, axes="xy")
    ctx.expect_within(display_part, chassis, axes="xy")
    ctx.expect_within(first_knob_part, chassis, axes="xy")
    ctx.expect_within(first_pad_part, chassis, axes="xy")
    ctx.expect_within(pitch_wheel_part, chassis, axes="xy")
    ctx.expect_within(mod_wheel_part, chassis, axes="xy")
    ctx.expect_gap(
        white_keys,
        chassis,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem=first_white,
        negative_elem=keybed_seat,
        name="white_keys_proud_of_body",
    )
    ctx.expect_gap(
        black_keys,
        white_keys,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem=first_black,
        negative_elem=first_white,
        name="black_keys_hover_above_white_keys",
    )
    ctx.expect_contact(
        first_knob_part,
        chassis,
        contact_tol=1e-4,
        elem_a=first_knob,
        elem_b=rear_housing,
        name="knob_directly_mounted_to_housing",
    )
    ctx.expect_contact(
        first_pad_part,
        chassis,
        contact_tol=1e-4,
        elem_a=first_pad,
        elem_b=rear_housing,
        name="pad_directly_mounted_to_housing",
    )
    ctx.expect_contact(
        display_part,
        chassis,
        contact_tol=1e-4,
        elem_a=display,
        elem_b=rear_housing,
        name="display_directly_mounted_to_housing",
    )
    ctx.expect_contact(
        pitch_wheel_part,
        chassis,
        contact_tol=1e-4,
        elem_a=pitch_wheel,
        elem_b=rear_housing,
        name="pitch_wheel_directly_mounted_to_housing",
    )
    ctx.expect_gap(
        white_keys,
        white_keys,
        axis="x",
        min_gap=0.30,
        max_gap=0.32,
        positive_elem=white_keys.get_visual("white_key_14"),
        negative_elem=first_white,
        name="white_key_span_reads_as_compact_two_octave_layout",
    )
    ctx.expect_gap(
        display_part,
        mod_wheel_part,
        axis="x",
        min_gap=0.15,
        max_gap=0.18,
        positive_elem=display,
        negative_elem=mod_wheel,
        name="wheels_are_clearly_left_of_control_section",
    )
    ctx.expect_gap(
        mod_wheel_part,
        pitch_wheel_part,
        axis="x",
        min_gap=0.012,
        max_gap=0.016,
        positive_elem=mod_wheel,
        negative_elem=pitch_wheel,
        name="mod_wheel_sits_just_right_of_pitch_wheel",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
