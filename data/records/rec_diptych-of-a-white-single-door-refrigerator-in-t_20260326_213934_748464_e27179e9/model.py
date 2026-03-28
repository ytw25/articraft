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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BODY_W = 0.55
BODY_D = 0.58
BODY_H = 0.84
SHELL_T = 0.022
DOOR_T = 0.048
DOOR_W = BODY_W - 0.008
DOOR_H = BODY_H - 0.006
DOOR_OPEN_ANGLE = math.radians(88.0)
DRAWER_REST_EXTENSION = 0.06
DRAWER_EXTRA_EXTENSION = 0.10


def _rotate_xy(x: float, y: float, angle: float) -> tuple[float, float]:
    c = math.cos(angle)
    s = math.sin(angle)
    return (x * c - y * s, x * s + y * c)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="undercounter_refrigerator", assets=ASSETS)

    appliance_white = model.material("appliance_white", rgba=(0.95, 0.96, 0.97, 1.0))
    inner_white = model.material("inner_white", rgba=(0.88, 0.90, 0.93, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.78, 0.91, 0.98, 0.35))
    drawer_clear = model.material("drawer_clear", rgba=(0.76, 0.92, 1.0, 0.28))
    warm_light = model.material("warm_light", rgba=(1.0, 0.97, 0.74, 0.72))
    wood = model.material("countertop_wood", rgba=(0.73, 0.60, 0.42, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    sticker_blue = model.material("sticker_blue", rgba=(0.12, 0.38, 0.82, 1.0))
    bottle_green = model.material("bottle_green", rgba=(0.46, 0.73, 0.50, 0.65))
    milk_white = model.material("milk_white", rgba=(0.97, 0.98, 0.99, 1.0))
    can_red = model.material("can_red", rgba=(0.78, 0.16, 0.18, 1.0))
    produce_orange = model.material("produce_orange", rgba=(0.92, 0.56, 0.18, 1.0))
    produce_green = model.material("produce_green", rgba=(0.50, 0.72, 0.28, 1.0))

    cabinet = model.part("cabinet")

    cabinet.visual(
        Box((BODY_W, BODY_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_T / 2.0)),
        material=appliance_white,
        name="cabinet_floor",
    )
    cabinet.visual(
        Box((BODY_W, BODY_D, SHELL_T)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - SHELL_T / 2.0)),
        material=appliance_white,
        name="cabinet_ceiling",
    )
    cabinet.visual(
        Box((SHELL_T, BODY_D, BODY_H - 2.0 * SHELL_T)),
        origin=Origin(xyz=(-BODY_W / 2.0 + SHELL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=appliance_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((SHELL_T, BODY_D, BODY_H - 2.0 * SHELL_T)),
        origin=Origin(xyz=(BODY_W / 2.0 - SHELL_T / 2.0, 0.0, BODY_H / 2.0)),
        material=appliance_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((BODY_W - 2.0 * SHELL_T, SHELL_T, BODY_H - 2.0 * SHELL_T)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + SHELL_T / 2.0, BODY_H / 2.0)),
        material=appliance_white,
        name="back_wall",
    )

    inner_depth = BODY_D - SHELL_T
    inner_depth_center_y = SHELL_T / 2.0
    inner_width = BODY_W - 2.0 * SHELL_T
    support_t = 0.008
    shelf_t = 0.006
    upper_support_z = 0.204
    shelf_mid_z = 0.402
    shelf_top_z = 0.575

    cabinet.visual(
        Box((inner_width, inner_depth, support_t)),
        origin=Origin(xyz=(0.0, inner_depth_center_y, upper_support_z)),
        material=inner_white,
        name="upper_drawer_support",
    )
    cabinet.visual(
        Box((inner_width, inner_depth, shelf_t)),
        origin=Origin(xyz=(0.0, inner_depth_center_y, shelf_mid_z)),
        material=glass_clear,
        name="shelf_mid_glass",
    )
    cabinet.visual(
        Box((inner_width, inner_depth, shelf_t)),
        origin=Origin(xyz=(0.0, inner_depth_center_y, shelf_top_z)),
        material=glass_clear,
        name="shelf_top_glass",
    )
    cabinet.visual(
        Box((0.18, 0.022, 0.010)),
        origin=Origin(
            xyz=(0.0, BODY_D / 2.0 - 0.045, BODY_H - SHELL_T - 0.005),
        ),
        material=warm_light,
        name="interior_light",
    )

    shelf_top_surface = shelf_top_z + shelf_t / 2.0
    shelf_mid_surface = shelf_mid_z + shelf_t / 2.0
    cabinet.visual(
        Cylinder(radius=0.028, length=0.235),
        origin=Origin(xyz=(-0.14, 0.10, shelf_top_surface + 0.1175)),
        material=bottle_green,
        name="top_bottle",
    )
    cabinet.visual(
        Box((0.075, 0.055, 0.19)),
        origin=Origin(xyz=(0.05, -0.04, shelf_top_surface + 0.095)),
        material=milk_white,
        name="milk_carton",
    )
    cabinet.visual(
        Cylinder(radius=0.032, length=0.105),
        origin=Origin(xyz=(-0.12, -0.08, shelf_mid_surface + 0.0525)),
        material=can_red,
        name="red_can",
    )
    cabinet.visual(
        Cylinder(radius=0.027, length=0.11),
        origin=Origin(xyz=(0.04, 0.08, shelf_mid_surface + 0.055)),
        material=milk_white,
        name="white_jar",
    )
    cabinet.visual(
        Box((0.09, 0.06, 0.05)),
        origin=Origin(xyz=(0.15, -0.02, shelf_mid_surface + 0.025)),
        material=glass_clear,
        name="food_box",
    )

    hinge_x = -BODY_W / 2.0 - 0.002
    hinge_y = BODY_D / 2.0 + 0.003
    hinge_pin_r = 0.004
    hinge_pin_h = 0.05
    bottom_hinge_z = 0.025
    top_hinge_z = BODY_H - 0.025

    cabinet.visual(
        Cylinder(radius=hinge_pin_r, length=hinge_pin_h),
        origin=Origin(xyz=(hinge_x, hinge_y, bottom_hinge_z)),
        material=gasket_gray,
        name="bottom_hinge_pin",
    )
    cabinet.visual(
        Cylinder(radius=hinge_pin_r, length=hinge_pin_h),
        origin=Origin(xyz=(hinge_x, hinge_y, top_hinge_z)),
        material=gasket_gray,
        name="top_hinge_pin",
    )
    door = model.part("door")

    def door_origin_from_closed(
        closed_xyz: tuple[float, float, float],
        *,
        yaw_closed: float = 0.0,
    ) -> Origin:
        x_rel = closed_xyz[0] - hinge_x
        y_rel = closed_xyz[1] - hinge_y
        x_open, y_open = _rotate_xy(x_rel, y_rel, DOOR_OPEN_ANGLE)
        return Origin(
            xyz=(x_open, y_open, closed_xyz[2]),
            rpy=(0.0, 0.0, DOOR_OPEN_ANGLE + yaw_closed),
        )

    def add_door_box(
        name: str,
        size: tuple[float, float, float],
        closed_xyz: tuple[float, float, float],
        material,
        *,
        yaw_closed: float = 0.0,
    ) -> None:
        door.visual(
            Box(size),
            origin=door_origin_from_closed(closed_xyz, yaw_closed=yaw_closed),
            material=material,
            name=name,
        )

    outer_skin_t = 0.016
    liner_t = 0.006
    door_inner_face_y = BODY_D / 2.0 + 0.001
    door_outer_skin_y = door_inner_face_y + DOOR_T - outer_skin_t / 2.0
    door_liner_y = door_inner_face_y + liner_t / 2.0
    rim_y = (door_inner_face_y + liner_t + door_outer_skin_y - outer_skin_t / 2.0) / 2.0

    add_door_box(
        "outer_skin",
        (DOOR_W, outer_skin_t, DOOR_H),
        (0.0, door_outer_skin_y, BODY_H / 2.0),
        appliance_white,
    )
    add_door_box(
        "outer_left_round",
        (0.016, outer_skin_t, DOOR_H - 0.020),
        (-DOOR_W / 2.0 + 0.008, door_outer_skin_y, BODY_H / 2.0),
        appliance_white,
    )
    add_door_box(
        "outer_right_round",
        (0.016, outer_skin_t, DOOR_H - 0.020),
        (DOOR_W / 2.0 - 0.008, door_outer_skin_y, BODY_H / 2.0),
        appliance_white,
    )
    add_door_box(
        "inner_liner",
        (DOOR_W - 0.056, liner_t, DOOR_H - 0.074),
        (0.0, door_liner_y, BODY_H / 2.0),
        inner_white,
    )
    add_door_box(
        "hinge_rim",
        (0.028, 0.026, DOOR_H),
        (-DOOR_W / 2.0 + 0.014, rim_y, BODY_H / 2.0),
        appliance_white,
    )
    add_door_box(
        "latch_rim",
        (0.028, 0.026, DOOR_H),
        (DOOR_W / 2.0 - 0.014, rim_y, BODY_H / 2.0),
        appliance_white,
    )
    add_door_box(
        "top_rim",
        (DOOR_W - 0.056, 0.026, 0.036),
        (0.0, rim_y, BODY_H - 0.021),
        appliance_white,
    )
    add_door_box(
        "bottom_rim",
        (DOOR_W - 0.056, 0.026, 0.036),
        (0.0, rim_y, 0.021),
        appliance_white,
    )
    add_door_box(
        "handle_grip",
        (0.25, 0.008, 0.020),
        (-0.01, door_outer_skin_y + outer_skin_t / 2.0 + 0.004, BODY_H - 0.064),
        gasket_gray,
    )
    door.visual(
        Sphere(radius=0.010),
        origin=door_origin_from_closed(
            (-0.135, door_outer_skin_y + outer_skin_t / 2.0 + 0.004, BODY_H - 0.064),
        ),
        material=gasket_gray,
        name="handle_left_cap",
    )
    door.visual(
        Sphere(radius=0.010),
        origin=door_origin_from_closed(
            (0.115, door_outer_skin_y + outer_skin_t / 2.0 + 0.004, BODY_H - 0.064),
        ),
        material=gasket_gray,
        name="handle_right_cap",
    )
    add_door_box(
        "handle_mount",
        (0.29, 0.004, 0.010),
        (-0.01, door_outer_skin_y + outer_skin_t / 2.0 + 0.002, BODY_H - 0.064),
        gasket_gray,
    )
    add_door_box(
        "energy_badge",
        (0.060, 0.003, 0.022),
        (0.0, door_outer_skin_y + outer_skin_t / 2.0 + 0.0015, BODY_H - 0.090),
        gasket_gray,
    )
    add_door_box(
        "warranty_sticker",
        (0.065, 0.003, 0.065),
        (0.18, door_outer_skin_y + outer_skin_t / 2.0 + 0.0015, BODY_H - 0.105),
        charcoal,
    )
    add_door_box(
        "fire_sticker",
        (0.058, 0.003, 0.075),
        (0.17, door_outer_skin_y + outer_skin_t / 2.0 + 0.0015, BODY_H - 0.195),
        sticker_blue,
    )

    knuckle_t = 0.006
    knuckle_y = rim_y - 0.010
    add_door_box(
        "bottom_hinge_knuckle",
        (knuckle_t, 0.016, hinge_pin_h),
        (hinge_x + hinge_pin_r + knuckle_t / 2.0, knuckle_y, bottom_hinge_z),
        gasket_gray,
    )
    add_door_box(
        "top_hinge_knuckle",
        (knuckle_t, 0.016, hinge_pin_h),
        (hinge_x + hinge_pin_r + knuckle_t / 2.0, knuckle_y, top_hinge_z),
        gasket_gray,
    )

    def add_door_bin(prefix: str, z_center: float, width: float, height: float) -> None:
        bin_back_t = 0.004
        bin_side_t = 0.004
        bin_bottom_t = 0.004
        depth = 0.060
        back_y = door_inner_face_y - bin_back_t / 2.0
        center_y = back_y - depth / 2.0 + bin_back_t / 2.0
        front_y = back_y - depth + bin_side_t / 2.0
        side_h = height - bin_bottom_t
        bottom_z = z_center - height / 2.0 + bin_bottom_t / 2.0
        side_z = z_center - height / 2.0 + bin_bottom_t + side_h / 2.0

        add_door_box(
            f"{prefix}_back",
            (width, bin_back_t, height),
            (0.0, back_y, z_center),
            drawer_clear,
        )
        add_door_box(
            f"{prefix}_bottom",
            (width, depth - bin_back_t, bin_bottom_t),
            (0.0, center_y, bottom_z),
            drawer_clear,
        )
        add_door_box(
            f"{prefix}_front",
            (width, bin_side_t, height * 0.55),
            (0.0, front_y, z_center - height * 0.08),
            drawer_clear,
        )
        add_door_box(
            f"{prefix}_left",
            (bin_side_t, depth - bin_back_t, side_h),
            (-width / 2.0 + bin_side_t / 2.0, center_y, side_z),
            drawer_clear,
        )
        add_door_box(
            f"{prefix}_right",
            (bin_side_t, depth - bin_back_t, side_h),
            (width / 2.0 - bin_side_t / 2.0, center_y, side_z),
            drawer_clear,
        )

    add_door_bin("door_bin_lower", z_center=0.310, width=0.42, height=0.10)
    add_door_bin("door_bin_upper", z_center=0.565, width=0.40, height=0.10)

    upper_drawer = model.part("upper_drawer")
    lower_drawer = model.part("lower_drawer")

    def build_drawer(
        part,
        *,
        width: float,
        depth: float,
        height: float,
        support_top_z: float,
        closed_center_y: float,
        produce_specs: tuple[tuple[str, str, tuple[float, float, float]], ...],
    ) -> None:
        wall_t = 0.004
        bottom_t = 0.004
        center_y = closed_center_y + DRAWER_REST_EXTENSION
        side_h = height - bottom_t
        side_z = support_top_z + bottom_t + side_h / 2.0

        part.visual(
            Box((width, depth, bottom_t)),
            origin=Origin(xyz=(0.0, center_y, support_top_z + bottom_t / 2.0)),
            material=drawer_clear,
            name="drawer_bottom",
        )
        part.visual(
            Box((wall_t, depth, side_h)),
            origin=Origin(
                xyz=(-width / 2.0 + wall_t / 2.0, center_y, side_z),
            ),
            material=drawer_clear,
            name="drawer_left",
        )
        part.visual(
            Box((wall_t, depth, side_h)),
            origin=Origin(
                xyz=(width / 2.0 - wall_t / 2.0, center_y, side_z),
            ),
            material=drawer_clear,
            name="drawer_right",
        )
        part.visual(
            Box((width - 2.0 * wall_t, wall_t, side_h)),
            origin=Origin(
                xyz=(0.0, center_y - depth / 2.0 + wall_t / 2.0, side_z),
            ),
            material=drawer_clear,
            name="drawer_back",
        )
        part.visual(
            Box((width - 2.0 * wall_t, wall_t, side_h)),
            origin=Origin(
                xyz=(0.0, center_y + depth / 2.0 - wall_t / 2.0, side_z),
            ),
            material=drawer_clear,
            name="drawer_front",
        )
        part.visual(
            Box((0.18, 0.010, 0.020)),
            origin=Origin(
                xyz=(0.0, center_y + depth / 2.0 + 0.003, support_top_z + height * 0.55),
            ),
            material=gasket_gray,
            name="drawer_handle",
        )

        for visual_name, color_name, (px, py, radius) in produce_specs:
            material = produce_orange if color_name == "orange" else produce_green
            part.visual(
                Sphere(radius=radius),
                origin=Origin(
                    xyz=(px, center_y + py, support_top_z + bottom_t + radius),
                ),
                material=material,
                name=visual_name,
            )

    drawer_width = inner_width - 0.020
    drawer_depth = 0.330

    build_drawer(
        lower_drawer,
        width=drawer_width,
        depth=drawer_depth,
        height=0.160,
        support_top_z=SHELL_T,
        closed_center_y=BODY_D / 2.0 - drawer_depth / 2.0,
        produce_specs=(
            ("produce_left", "orange", (-0.11, -0.05, 0.040)),
            ("produce_right", "green", (0.10, 0.03, 0.036)),
        ),
    )
    build_drawer(
        upper_drawer,
        width=drawer_width,
        depth=drawer_depth,
        height=0.145,
        support_top_z=upper_support_z + support_t / 2.0,
        closed_center_y=BODY_D / 2.0 - drawer_depth / 2.0,
        produce_specs=(
            ("produce_left", "green", (-0.09, -0.04, 0.034)),
            ("produce_right", "orange", (0.08, 0.04, 0.032)),
        ),
    )

    countertop = model.part("countertop")
    countertop.visual(
        Box((0.72, 0.64, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H + 0.0175)),
        material=wood,
        name="countertop_panel",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.4,
            lower=-DOOR_OPEN_ANGLE,
            upper=0.0,
        ),
    )
    model.articulation(
        "upper_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_drawer,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=-DRAWER_REST_EXTENSION,
            upper=DRAWER_EXTRA_EXTENSION,
        ),
    )
    model.articulation(
        "lower_drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_drawer,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=-DRAWER_REST_EXTENSION,
            upper=DRAWER_EXTRA_EXTENSION,
        ),
    )
    model.articulation(
        "cabinet_to_countertop",
        ArticulationType.FIXED,
        parent=cabinet,
        child=countertop,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    upper_drawer = object_model.get_part("upper_drawer")
    lower_drawer = object_model.get_part("lower_drawer")
    countertop = object_model.get_part("countertop")

    door_hinge = object_model.get_articulation("door_hinge")
    upper_slide = object_model.get_articulation("upper_drawer_slide")
    lower_slide = object_model.get_articulation("lower_drawer_slide")

    cabinet_floor = cabinet.get_visual("cabinet_floor")
    cabinet_ceiling = cabinet.get_visual("cabinet_ceiling")
    upper_support = cabinet.get_visual("upper_drawer_support")
    shelf_mid = cabinet.get_visual("shelf_mid_glass")
    shelf_top = cabinet.get_visual("shelf_top_glass")
    top_pin = cabinet.get_visual("top_hinge_pin")
    bottom_pin = cabinet.get_visual("bottom_hinge_pin")
    top_bottle = cabinet.get_visual("top_bottle")
    milk_carton = cabinet.get_visual("milk_carton")
    red_can = cabinet.get_visual("red_can")

    inner_liner = door.get_visual("inner_liner")
    top_knuckle = door.get_visual("top_hinge_knuckle")
    bottom_knuckle = door.get_visual("bottom_hinge_knuckle")

    upper_drawer_bottom = upper_drawer.get_visual("drawer_bottom")
    lower_drawer_bottom = lower_drawer.get_visual("drawer_bottom")
    countertop_panel = countertop.get_visual("countertop_panel")

    def axis_close(actual, expected, tol: float = 1e-6) -> bool:
        return all(abs(a - b) <= tol for a, b in zip(actual, expected))

    def size_from_aabb(aabb):
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    def check_same_part_supported(part, elem_upper, elem_lower, *, axis: str, name: str) -> None:
        upper_aabb = ctx.part_element_world_aabb(part, elem=elem_upper)
        lower_aabb = ctx.part_element_world_aabb(part, elem=elem_lower)
        if upper_aabb is None or lower_aabb is None:
            ctx.fail(name, "Missing visual AABB for same-part support check.")
            return
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        gap = upper_aabb[0][axis_index] - lower_aabb[1][axis_index]
        ctx.check(name, abs(gap) <= 0.0025, f"expected contact-like support gap, got {gap:.4f} m")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    body_aabb = ctx.part_world_aabb(cabinet)
    if body_aabb is None:
        ctx.fail("cabinet_aabb_exists", "Cabinet world AABB could not be resolved.")
    else:
        body_size = size_from_aabb(body_aabb)
        ctx.check(
            "cabinet_realistic_size",
            0.52 <= body_size[0] <= 0.58 and 0.56 <= body_size[1] <= 0.60 and 0.82 <= body_size[2] <= 0.86,
            f"cabinet size was {body_size}",
        )

    ctx.check(
        "door_hinge_axis_vertical",
        axis_close(door_hinge.axis, (0.0, 0.0, 1.0)),
        f"door hinge axis was {door_hinge.axis}",
    )
    ctx.check(
        "drawer_axes_forward",
        axis_close(upper_slide.axis, (0.0, 1.0, 0.0)) and axis_close(lower_slide.axis, (0.0, 1.0, 0.0)),
        f"drawer axes were {upper_slide.axis} and {lower_slide.axis}",
    )

    ctx.expect_contact(
        countertop,
        cabinet,
        elem_a=countertop_panel,
        elem_b=cabinet_ceiling,
        contact_tol=0.001,
        name="countertop_resting_on_cabinet",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a=top_knuckle,
        elem_b=top_pin,
        contact_tol=0.0015,
        name="top_hinge_connected",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a=bottom_knuckle,
        elem_b=bottom_pin,
        contact_tol=0.0015,
        name="bottom_hinge_connected",
    )
    ctx.expect_contact(
        upper_drawer,
        cabinet,
        elem_a=upper_drawer_bottom,
        elem_b=upper_support,
        contact_tol=0.0015,
        name="upper_drawer_supported",
    )
    ctx.expect_contact(
        lower_drawer,
        cabinet,
        elem_a=lower_drawer_bottom,
        elem_b=cabinet_floor,
        contact_tol=0.0015,
        name="lower_drawer_supported",
    )
    ctx.expect_within(upper_drawer, cabinet, axes="x", margin=0.012, name="upper_drawer_within_width")
    ctx.expect_within(lower_drawer, cabinet, axes="x", margin=0.012, name="lower_drawer_within_width")
    ctx.expect_gap(upper_drawer, lower_drawer, axis="z", min_gap=0.015, name="drawers_are_distinct")

    check_same_part_supported(cabinet, top_bottle, shelf_top, axis="z", name="top_bottle_on_shelf")
    check_same_part_supported(cabinet, milk_carton, shelf_top, axis="z", name="milk_carton_on_shelf")
    check_same_part_supported(cabinet, red_can, shelf_mid, axis="z", name="mid_can_on_shelf")

    door_open_aabb = ctx.part_world_aabb(door)
    if door_open_aabb is None or body_aabb is None:
        ctx.fail("open_door_pose_resolves", "Could not measure open-pose AABBs.")
    else:
        open_forward = door_open_aabb[1][1] - body_aabb[1][1]
        open_left = body_aabb[0][0] - door_open_aabb[0][0]
        ctx.check(
            "door_reads_open",
            open_forward >= 0.22 and open_left >= 0.01,
            f"door open offsets were forward={open_forward:.3f}, left={open_left:.3f}",
        )

    with ctx.pose({door_hinge: -DOOR_OPEN_ANGLE}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_closed_no_overlap")
        ctx.fail_if_isolated_parts(name="door_closed_no_floating")
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem=inner_liner,
            negative_elem=cabinet_floor,
            max_gap=0.012,
            max_penetration=0.0,
            name="door_closes_against_front_plane",
        )
        ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.45, name="door_covers_opening")
        ctx.expect_contact(
            door,
            cabinet,
            elem_a=top_knuckle,
            elem_b=top_pin,
            contact_tol=0.0015,
            name="top_hinge_connected_closed",
        )

    for joint, label in ((upper_slide, "upper_drawer"), (lower_slide, "lower_drawer")):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"{label}_limits_present", "Drawer limits were missing.")
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_closed_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_closed_no_floating")
            drawer_aabb = ctx.part_world_aabb(upper_drawer if label == "upper_drawer" else lower_drawer)
            cabinet_pose_aabb = ctx.part_world_aabb(cabinet)
            if drawer_aabb is None or cabinet_pose_aabb is None:
                ctx.fail(f"{label}_closed_aabb", "Missing AABB while checking closed drawer pose.")
            else:
                protrusion = drawer_aabb[1][1] - cabinet_pose_aabb[1][1]
                ctx.check(
                    f"{label}_closes_inside_case",
                    protrusion <= 0.004,
                    f"{label} protruded {protrusion:.4f} m in closed pose",
                )
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_extended_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{label}_extended_no_floating")
            drawer_aabb = ctx.part_world_aabb(upper_drawer if label == "upper_drawer" else lower_drawer)
            cabinet_pose_aabb = ctx.part_world_aabb(cabinet)
            if drawer_aabb is None or cabinet_pose_aabb is None:
                ctx.fail(f"{label}_extended_aabb", "Missing AABB while checking extended drawer pose.")
            else:
                protrusion = drawer_aabb[1][1] - cabinet_pose_aabb[1][1]
                ctx.check(
                    f"{label}_extends_forward",
                    protrusion >= 0.10,
                    f"{label} protrusion was {protrusion:.4f} m",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
