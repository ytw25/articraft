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

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_field_flip_phone", assets=ASSETS)

    body_plastic = model.material("body_plastic", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.64, 1.0))
    display_glass = model.material("display_glass", rgba=(0.08, 0.11, 0.13, 1.0))
    bezel = model.material("bezel", rgba=(0.14, 0.15, 0.16, 1.0))
    keycap = model.material("keycap", rgba=(0.15, 0.16, 0.17, 1.0))

    lower = model.part("lower")
    lower_shell_size = (0.078, 0.114, 0.030)
    lower_shell_origin = Origin(xyz=(0.0, 0.0, 0.015))
    lower.visual(
        Box(lower_shell_size),
        origin=lower_shell_origin,
        material=body_plastic,
        name="lower_shell",
    )
    lower.inertial = Inertial.from_geometry(
        Box(lower_shell_size),
        mass=0.18,
        origin=lower_shell_origin,
    )

    lower.visual(
        Box((0.078, 0.015, 0.026)),
        origin=Origin(xyz=(0.0, 0.0495, 0.013)),
        material=rubber,
        name="lower_front_cap",
    )
    lower.visual(
        Box((0.068, 0.082, 0.0034)),
        origin=Origin(xyz=(0.0, 0.010, 0.0250)),
        material=bezel,
        name="keyboard_deck",
    )
    for name, x_pos in (
        ("lower_hinge_leaf_left", -0.028),
        ("lower_hinge_leaf_center", 0.0),
        ("lower_hinge_leaf_right", 0.028),
    ):
        lower.visual(
            Box((0.010, 0.008, 0.005)),
            origin=Origin(xyz=(x_pos, -0.0585, 0.0293)),
            material=steel,
            name=name,
        )

    knuckle_radius = 0.0035
    knuckle_length = 0.012
    knuckle_rpy = (0.0, math.pi / 2.0, 0.0)
    for name, x_pos in (
        ("lower_knuckle_left", -0.028),
        ("lower_knuckle_center", 0.0),
        ("lower_knuckle_right", 0.028),
    ):
        lower.visual(
            Cylinder(radius=knuckle_radius, length=knuckle_length),
            origin=Origin(xyz=(x_pos, -0.0605, 0.032), rpy=knuckle_rpy),
            material=steel,
            name=name,
        )

    for name, x_pos in (("lower_lock_tab_left", -0.022), ("lower_lock_tab_right", 0.022)):
        lower.visual(
            Box((0.006, 0.012, 0.0028)),
            origin=Origin(xyz=(x_pos, -0.0515, 0.0312)),
            material=steel,
            name=name,
        )

    def add_key_row(row_y: float, row_height: float, specs: list[tuple[str, float, float]]) -> None:
        key_bottom = 0.0267
        key_center_z = key_bottom + (row_height / 2.0)
        for key_name, x_pos, key_width in specs:
            lower.visual(
                Box((key_width, 0.0075, row_height)),
                origin=Origin(xyz=(x_pos, row_y, key_center_z)),
                material=keycap,
                name=key_name,
            )

    add_key_row(
        row_y=-0.012,
        row_height=0.0030,
        specs=[
            ("key_q", -0.0306, 0.0058),
            ("key_w", -0.0238, 0.0058),
            ("key_e", -0.0170, 0.0058),
            ("key_r", -0.0102, 0.0058),
            ("key_t", -0.0034, 0.0058),
            ("key_y", 0.0034, 0.0058),
            ("key_u", 0.0102, 0.0058),
            ("key_i", 0.0170, 0.0058),
            ("key_o", 0.0238, 0.0058),
            ("key_p", 0.0306, 0.0058),
        ],
    )
    add_key_row(
        row_y=0.002,
        row_height=0.0034,
        specs=[
            ("key_a", -0.0276, 0.0063),
            ("key_s", -0.0207, 0.0063),
            ("key_d", -0.0138, 0.0063),
            ("key_f", -0.0069, 0.0063),
            ("key_g", 0.0000, 0.0063),
            ("key_h", 0.0069, 0.0063),
            ("key_j", 0.0138, 0.0063),
            ("key_k", 0.0207, 0.0063),
            ("key_l", 0.0276, 0.0063),
        ],
    )
    add_key_row(
        row_y=0.017,
        row_height=0.0038,
        specs=[
            ("key_z", -0.0225, 0.0070),
            ("key_x", -0.0150, 0.0070),
            ("key_c", -0.0075, 0.0070),
            ("key_v", 0.0000, 0.0070),
            ("key_b", 0.0075, 0.0070),
            ("key_n", 0.0150, 0.0070),
            ("key_m", 0.0225, 0.0070),
        ],
    )
    add_key_row(
        row_y=0.034,
        row_height=0.0036,
        specs=[
            ("key_shift", -0.0240, 0.0070),
            ("key_fn", -0.0120, 0.0070),
            ("key_space", 0.0000, 0.0180),
            ("key_alt", 0.0120, 0.0070),
            ("key_back", 0.0240, 0.0070),
        ],
    )

    upper = model.part("upper")
    upper_shell_size = (0.078, 0.112, 0.024)
    upper_shell_origin = Origin(xyz=(0.0, -0.060, -0.018))
    upper.visual(
        Box(upper_shell_size),
        origin=upper_shell_origin,
        material=body_plastic,
        name="upper_shell",
    )
    upper.inertial = Inertial.from_geometry(
        Box(upper_shell_size),
        mass=0.16,
        origin=upper_shell_origin,
    )

    upper.visual(
        Box((0.078, 0.015, 0.020)),
        origin=Origin(xyz=(0.0, -0.1085, -0.018)),
        material=rubber,
        name="upper_front_cap",
    )
    upper.visual(
        Box((0.066, 0.090, 0.0012)),
        origin=Origin(xyz=(0.0, -0.064, -0.0066)),
        material=bezel,
        name="display_recess",
    )
    upper.visual(
        Box((0.058, 0.082, 0.0014)),
        origin=Origin(xyz=(0.0, -0.064, -0.0079)),
        material=display_glass,
        name="display_screen",
    )
    upper.visual(
        Box((0.0045, 0.090, 0.0030)),
        origin=Origin(xyz=(-0.0325, -0.064, -0.0050)),
        material=steel,
        name="frame_left",
    )
    upper.visual(
        Box((0.0045, 0.090, 0.0030)),
        origin=Origin(xyz=(0.0325, -0.064, -0.0050)),
        material=steel,
        name="frame_right",
    )
    upper.visual(
        Box((0.064, 0.0045, 0.0030)),
        origin=Origin(xyz=(0.0, -0.1075, -0.0050)),
        material=steel,
        name="frame_top",
    )
    upper.visual(
        Box((0.064, 0.0045, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0205, -0.0050)),
        material=steel,
        name="frame_bottom",
    )
    for name, x_pos in (("upper_hinge_leaf_left", -0.014), ("upper_hinge_leaf_right", 0.014)):
        upper.visual(
            Box((0.010, 0.009, 0.006)),
            origin=Origin(xyz=(x_pos, -0.0032, -0.0045)),
            material=steel,
            name=name,
        )

    for name, x_pos in (("upper_knuckle_left", -0.014), ("upper_knuckle_right", 0.014)):
        upper.visual(
            Cylinder(radius=knuckle_radius, length=knuckle_length),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=knuckle_rpy),
            material=steel,
            name=name,
        )

    for name, x_pos in (("upper_lock_tab_left", -0.022), ("upper_lock_tab_right", 0.022)):
        upper.visual(
            Box((0.006, 0.012, 0.0022)),
            origin=Origin(xyz=(x_pos, -0.0085, -0.0014)),
            material=steel,
            name=name,
        )

    model.articulation(
        "clamshell_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, -0.0605, 0.032), rpy=(math.pi, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=2.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    lower = object_model.get_part("lower")
    upper = object_model.get_part("upper")
    hinge = object_model.get_articulation("clamshell_hinge")

    lower_shell = lower.get_visual("lower_shell")
    lower_front_cap = lower.get_visual("lower_front_cap")
    keyboard_deck = lower.get_visual("keyboard_deck")
    lower_hinge_leaf_left = lower.get_visual("lower_hinge_leaf_left")
    lower_hinge_leaf_center = lower.get_visual("lower_hinge_leaf_center")
    lower_hinge_leaf_right = lower.get_visual("lower_hinge_leaf_right")
    lower_knuckle_left = lower.get_visual("lower_knuckle_left")
    lower_knuckle_center = lower.get_visual("lower_knuckle_center")
    lower_knuckle_right = lower.get_visual("lower_knuckle_right")
    lower_lock_tab_left = lower.get_visual("lower_lock_tab_left")
    key_q = lower.get_visual("key_q")
    key_p = lower.get_visual("key_p")
    key_a = lower.get_visual("key_a")
    key_l = lower.get_visual("key_l")
    key_z = lower.get_visual("key_z")
    key_m = lower.get_visual("key_m")
    key_space = lower.get_visual("key_space")

    upper_shell = upper.get_visual("upper_shell")
    upper_front_cap = upper.get_visual("upper_front_cap")
    display_recess = upper.get_visual("display_recess")
    display_screen = upper.get_visual("display_screen")
    frame_left = upper.get_visual("frame_left")
    frame_right = upper.get_visual("frame_right")
    frame_top = upper.get_visual("frame_top")
    frame_bottom = upper.get_visual("frame_bottom")
    upper_hinge_leaf_left = upper.get_visual("upper_hinge_leaf_left")
    upper_hinge_leaf_right = upper.get_visual("upper_hinge_leaf_right")
    upper_knuckle_left = upper.get_visual("upper_knuckle_left")
    upper_knuckle_right = upper.get_visual("upper_knuckle_right")
    upper_lock_tab_left = upper.get_visual("upper_lock_tab_left")

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

    ctx.expect_overlap(upper, lower, axes="xy", min_overlap=0.006, elem_a=upper_shell, elem_b=lower_shell)
    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        max_gap=0.009,
        max_penetration=0.0,
        positive_elem=upper_shell,
        negative_elem=lower_shell,
    )

    ctx.expect_within(lower, lower, axes="xz", inner_elem=lower_front_cap, outer_elem=lower_shell)
    ctx.expect_within(upper, upper, axes="xz", inner_elem=upper_front_cap, outer_elem=upper_shell)

    ctx.expect_within(upper, upper, axes="xy", inner_elem=display_screen, outer_elem=upper_shell)
    ctx.expect_within(upper, upper, axes="xy", inner_elem=display_recess, outer_elem=upper_shell)
    ctx.expect_gap(
        upper,
        upper,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=display_screen,
        negative_elem=frame_left,
    )
    ctx.expect_gap(
        upper,
        upper,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=frame_right,
        negative_elem=display_screen,
    )
    ctx.expect_gap(
        upper,
        upper,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=frame_top,
        negative_elem=display_screen,
    )
    ctx.expect_gap(
        upper,
        upper,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=display_screen,
        negative_elem=frame_bottom,
    )
    ctx.expect_gap(
        upper,
        upper,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=display_screen,
        negative_elem=frame_left,
    )

    for key in (key_q, key_a, key_z, key_space):
        ctx.expect_within(lower, lower, axes="xy", inner_elem=key, outer_elem=keyboard_deck)
        ctx.expect_gap(
            lower,
            lower,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=key,
            negative_elem=keyboard_deck,
        )
    for key in (key_p, key_l, key_m):
        ctx.expect_within(lower, lower, axes="xy", inner_elem=key, outer_elem=keyboard_deck)

    ctx.expect_gap(
        lower,
        lower,
        axis="y",
        min_gap=0.004,
        positive_elem=key_a,
        negative_elem=key_q,
    )
    ctx.expect_gap(
        lower,
        lower,
        axis="y",
        min_gap=0.004,
        positive_elem=key_z,
        negative_elem=key_a,
    )
    ctx.expect_gap(
        lower,
        lower,
        axis="y",
        min_gap=0.006,
        positive_elem=key_space,
        negative_elem=key_z,
    )
    ctx.expect_gap(
        lower,
        lower,
        axis="x",
        min_gap=0.050,
        positive_elem=key_p,
        negative_elem=key_q,
    )
    ctx.expect_gap(
        lower,
        lower,
        axis="x",
        min_gap=0.042,
        positive_elem=key_l,
        negative_elem=key_a,
    )
    ctx.expect_gap(
        lower,
        lower,
        axis="x",
        min_gap=0.030,
        positive_elem=key_m,
        negative_elem=key_z,
    )

    ctx.expect_overlap(lower, upper, axes="yz", min_overlap=0.00001, elem_a=lower_knuckle_left, elem_b=upper_knuckle_left)
    ctx.expect_overlap(lower, upper, axes="yz", min_overlap=0.00001, elem_a=lower_knuckle_center, elem_b=upper_knuckle_left)
    ctx.expect_overlap(lower, upper, axes="yz", min_overlap=0.00001, elem_a=lower_knuckle_center, elem_b=upper_knuckle_right)
    ctx.expect_overlap(lower, upper, axes="yz", min_overlap=0.00001, elem_a=lower_knuckle_right, elem_b=upper_knuckle_right)
    ctx.expect_contact(lower, lower, elem_a=lower_hinge_leaf_left, elem_b=lower_knuckle_left)
    ctx.expect_contact(lower, lower, elem_a=lower_hinge_leaf_center, elem_b=lower_knuckle_center)
    ctx.expect_contact(lower, lower, elem_a=lower_hinge_leaf_right, elem_b=lower_knuckle_right)
    ctx.expect_contact(upper, upper, elem_a=upper_hinge_leaf_left, elem_b=upper_knuckle_left)
    ctx.expect_contact(upper, upper, elem_a=upper_hinge_leaf_right, elem_b=upper_knuckle_right)
    ctx.expect_gap(
        upper,
        lower,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem=upper_knuckle_left,
        negative_elem=lower_knuckle_left,
    )
    ctx.expect_gap(
        lower,
        upper,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem=lower_knuckle_center,
        negative_elem=upper_knuckle_left,
    )
    ctx.expect_gap(
        upper,
        lower,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem=upper_knuckle_right,
        negative_elem=lower_knuckle_center,
    )
    ctx.expect_gap(
        lower,
        upper,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem=lower_knuckle_right,
        negative_elem=upper_knuckle_right,
    )

    ctx.expect_gap(
        upper,
        lower,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem=upper_lock_tab_left,
        negative_elem=lower_lock_tab_left,
    )
    with ctx.pose({hinge: 1.95}):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.018,
            positive_elem=display_screen,
            negative_elem=key_a,
        )
        ctx.expect_gap(
            lower,
            upper,
            axis="y",
            min_gap=0.070,
            positive_elem=key_a,
            negative_elem=display_screen,
        )
        ctx.expect_gap(
            lower,
            upper,
            axis="y",
            min_gap=0.004,
            positive_elem=lower_lock_tab_left,
            negative_elem=upper_lock_tab_left,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
