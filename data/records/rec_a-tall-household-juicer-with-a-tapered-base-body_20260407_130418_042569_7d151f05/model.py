from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _translate_profile(
    profile: list[tuple[float, float]],
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_household_juicer")

    body_charcoal = model.material("body_charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    front_panel_black = model.material("front_panel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.77, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.78, 0.88, 0.93, 0.35))
    clear_gloss = model.material("clear_gloss", rgba=(0.92, 0.96, 0.98, 0.24))
    pusher_black = model.material("pusher_black", rgba=(0.12, 0.12, 0.13, 1.0))
    basket_metal = model.material("basket_metal", rgba=(0.84, 0.85, 0.86, 1.0))
    button_dark = model.material("button_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    button_power = model.material("button_power", rgba=(0.17, 0.52, 0.27, 1.0))

    body = model.part("body")

    body_shell = section_loft(
        [
            _xy_section(0.208, 0.186, 0.032, 0.000),
            _xy_section(0.196, 0.176, 0.034, 0.060),
            _xy_section(0.178, 0.158, 0.032, 0.135),
            _xy_section(0.160, 0.146, 0.028, 0.205),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell, "juicer_body_shell"),
        material=body_charcoal,
        name="body_shell",
    )

    chamber_rim = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.089, 0.000), (0.089, 0.018)],
        inner_profile=[(0.070, 0.000), (0.070, 0.018)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(chamber_rim, "juicer_chamber_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=stainless,
        name="chamber_rim",
    )

    body.visual(
        Box((0.068, 0.014, 0.060)),
        origin=Origin(xyz=(0.0, 0.086, 0.114)),
        material=front_panel_black,
        name="front_panel",
    )
    body.visual(
        Box((0.096, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, -0.087, 0.212)),
        material=body_charcoal,
        name="rear_hinge_bridge",
    )
    body.visual(
        Box((0.012, 0.024, 0.064)),
        origin=Origin(xyz=(0.101, -0.038, 0.241)),
        material=body_charcoal,
        name="latch_tower",
    )
    body.visual(
        Box((0.028, 0.018, 0.022)),
        origin=Origin(xyz=(0.088, -0.034, 0.214)),
        material=body_charcoal,
        name="latch_brace",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.101, -0.038, 0.277)),
        material=body_charcoal,
        name="latch_pivot_boss",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.213)),
        material=stainless,
        name="drive_spindle",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.208, 0.186, 0.302)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.151)),
    )

    lid = model.part("lid")

    lid_skirt = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.090, 0.000), (0.090, 0.068)],
        inner_profile=[(0.082, 0.000), (0.082, 0.068)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )
    lid.visual(
        mesh_from_geometry(lid_skirt, "juicer_lid_skirt"),
        origin=Origin(xyz=(0.0, 0.082, 0.0)),
        material=smoked_clear,
        name="lid_skirt",
    )

    chute_hole_profile = _translate_profile(rounded_rect_profile(0.042, 0.032, 0.006), 0.070, 0.044)
    lid_top = ExtrudeWithHolesGeometry(
        outer_profile=superellipse_profile(0.194, 0.194, exponent=2.4, segments=48),
        hole_profiles=[chute_hole_profile],
        height=0.004,
        center=False,
    )
    lid.visual(
        mesh_from_geometry(lid_top, "juicer_lid_top"),
        origin=Origin(xyz=(0.0, 0.082, 0.064)),
        material=clear_gloss,
        name="lid_top",
    )

    chute_collar = ExtrudeWithHolesGeometry(
        outer_profile=rounded_rect_profile(0.056, 0.044, 0.010),
        hole_profiles=[rounded_rect_profile(0.042, 0.032, 0.006)],
        height=0.014,
        center=False,
    )
    lid.visual(
        mesh_from_geometry(chute_collar, "juicer_chute_collar"),
        origin=Origin(xyz=(0.070, 0.126, 0.060)),
        material=smoked_clear,
        name="chute_collar",
    )

    chute_height = 0.158
    lid.visual(
        Box((0.056, 0.005, chute_height)),
        origin=Origin(xyz=(0.070, 0.1455, 0.147)),
        material=smoked_clear,
        name="chute_shell_front",
    )
    lid.visual(
        Box((0.056, 0.005, chute_height)),
        origin=Origin(xyz=(0.070, 0.1065, 0.147)),
        material=smoked_clear,
        name="chute_shell_rear",
    )
    lid.visual(
        Box((0.006, 0.034, chute_height)),
        origin=Origin(xyz=(0.095, 0.126, 0.147)),
        material=smoked_clear,
        name="chute_shell_right",
    )
    lid.visual(
        Box((0.006, 0.034, chute_height)),
        origin=Origin(xyz=(0.045, 0.126, 0.147)),
        material=smoked_clear,
        name="chute_shell_left",
    )

    lid.visual(
        Box((0.108, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, -0.006, 0.009)),
        material=body_charcoal,
        name="hinge_strap",
    )
    for index, x_pos in enumerate((-0.032, 0.0, 0.032)):
        lid.visual(
            Cylinder(radius=0.006, length=0.028),
            origin=Origin(
                xyz=(x_pos, -0.010, 0.009),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=body_charcoal,
            name=f"hinge_knuckle_{index}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((0.194, 0.246, 0.238)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.094, 0.090)),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.040, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=pusher_black,
        name="pusher_nose",
    )
    pusher.visual(
        Box((0.032, 0.022, 0.112)),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=pusher_black,
        name="pusher_shaft",
    )
    pusher.visual(
        Box((0.078, 0.064, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.146)),
        material=pusher_black,
        name="pusher_handle",
    )
    pusher.inertial = Inertial.from_geometry(
        Box((0.078, 0.064, 0.156)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
    )

    basket = model.part("filter_basket")
    basket.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=basket_metal,
        name="basket_hub",
    )
    basket.visual(
        Cylinder(radius=0.008, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=basket_metal,
        name="basket_shaft",
    )
    basket.visual(
        Cylinder(radius=0.040, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=basket_metal,
        name="basket_lower_ring",
    )
    basket.visual(
        Cylinder(radius=0.043, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=basket_metal,
        name="basket_upper_ring",
    )
    for index in range(12):
        angle = index * math.tau / 12.0
        basket.visual(
            Box((0.006, 0.003, 0.040)),
            origin=Origin(
                xyz=(0.037 * math.cos(angle), 0.037 * math.sin(angle), 0.029),
                rpy=(0.0, 0.0, angle),
            ),
            material=basket_metal,
            name=f"basket_slat_{index}",
        )
    basket.visual(
        Cylinder(radius=0.030, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=stainless,
        name="cutting_disc",
    )
    basket.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=stainless,
        name="disc_nut",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.090),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    button_specs = [
        ("left_button", -0.020, button_dark),
        ("center_button", 0.000, button_power),
        ("right_button", 0.020, button_dark),
    ]
    for button_name, x_pos, material in button_specs:
        button = model.part(button_name)
        button.visual(
            Box((0.010, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, 0.002, 0.006)),
            material=front_panel_black,
            name="button_stem",
        )
        button.visual(
            Box((0.014, 0.006, 0.014)),
            origin=Origin(xyz=(0.0, 0.007, 0.007)),
            material=material,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.014, 0.010, 0.014)),
            mass=0.01,
            origin=Origin(xyz=(0.0, 0.005, 0.007)),
        )
        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.093, 0.114)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.03,
                lower=0.0,
                upper=0.0035,
            ),
        )

    latch = model.part("locking_latch")
    latch.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_charcoal,
        name="latch_pivot",
    )
    latch.visual(
        Box((0.060, 0.010, 0.008)),
        origin=Origin(xyz=(-0.030, 0.0, 0.010)),
        material=body_charcoal,
        name="latch_arm",
    )
    latch.visual(
        Box((0.010, 0.010, 0.012)),
        origin=Origin(xyz=(-0.060, 0.0, 0.010)),
        material=body_charcoal,
        name="latch_hook",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.060, 0.016, 0.020)),
        mass=0.045,
        origin=Origin(xyz=(-0.026, 0.0, 0.010)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.082, 0.223)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.070, 0.126, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=0.0,
            upper=0.095,
        ),
    )
    model.articulation(
        "body_to_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(0.101, -0.038, 0.285)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "body_to_filter_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=35.0),
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

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    latch = object_model.get_part("locking_latch")

    lid_joint = object_model.get_articulation("body_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    latch_joint = object_model.get_articulation("body_to_latch")
    basket_joint = object_model.get_articulation("body_to_filter_basket")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_skirt",
        negative_elem="chamber_rim",
        max_gap=0.004,
        max_penetration=0.001,
        name="lid seats on chamber rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_skirt",
        elem_b="chamber_rim",
        min_overlap=0.120,
        name="lid covers the chamber opening",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="chute_collar",
        margin=0.010,
        name="pusher shaft stays within chute footprint",
    )

    closed_lid_top_aabb = ctx.part_element_world_aabb(lid, elem="lid_top")
    rest_pusher_pos = ctx.part_world_position(pusher)

    with ctx.pose({lid_joint: math.radians(62.0)}):
        open_lid_top_aabb = ctx.part_element_world_aabb(lid, elem="lid_top")

    with ctx.pose({pusher_joint: 0.085}):
        raised_pusher_pos = ctx.part_world_position(pusher)
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="chute_collar",
            margin=0.010,
            name="raised pusher remains guided by chute",
        )

    ctx.check(
        "lid opens upward",
        closed_lid_top_aabb is not None
        and open_lid_top_aabb is not None
        and _aabb_center(open_lid_top_aabb)[2] > _aabb_center(closed_lid_top_aabb)[2] + 0.025,
        details=f"closed={closed_lid_top_aabb}, open={open_lid_top_aabb}",
    )
    ctx.check(
        "pusher rises upward",
        rest_pusher_pos is not None
        and raised_pusher_pos is not None
        and raised_pusher_pos[2] > rest_pusher_pos[2] + 0.070,
        details=f"rest={rest_pusher_pos}, raised={raised_pusher_pos}",
    )

    for button_name in ("left_button", "center_button", "right_button"):
        button = object_model.get_part(button_name)
        joint = object_model.get_articulation(f"body_to_{button_name}")
        rest_button_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.0035}):
            pressed_button_pos = ctx.part_world_position(button)
        ctx.check(
            f"{button_name} presses into front panel",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[1] < rest_button_pos[1] - 0.0025,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    closed_hook_aabb = ctx.part_element_world_aabb(latch, elem="latch_hook")
    with ctx.pose({latch_joint: 1.05}):
        open_hook_aabb = ctx.part_element_world_aabb(latch, elem="latch_hook")
    closed_hook_center = _aabb_center(closed_hook_aabb)
    open_hook_center = _aabb_center(open_hook_aabb)
    ctx.check(
        "latch swings away from lid edge",
        closed_hook_center is not None
        and open_hook_center is not None
        and (
            abs(open_hook_center[0] - closed_hook_center[0]) > 0.020
            or abs(open_hook_center[1] - closed_hook_center[1]) > 0.020
        ),
        details=f"closed={closed_hook_center}, open={open_hook_center}",
    )

    limits = basket_joint.motion_limits
    ctx.check(
        "filter basket uses continuous vertical spin",
        basket_joint.articulation_type == ArticulationType.CONTINUOUS
        and basket_joint.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=(
            f"type={basket_joint.articulation_type}, axis={basket_joint.axis}, "
            f"limits={limits}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
