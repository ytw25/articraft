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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translated_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _add_slot_liner(
    body,
    *,
    slot_index: int,
    slot_center_y: float,
    slot_len_x: float,
    slot_width_y: float,
    top_z: float,
    liner_depth: float,
    liner_thickness: float,
    material,
) -> None:
    z_center = top_z - liner_depth / 2.0 + 0.0005
    x_side = slot_len_x / 2.0 + liner_thickness / 2.0
    y_side = slot_width_y / 2.0 + liner_thickness / 2.0

    body.visual(
        Box((liner_thickness, slot_width_y + 2.0 * liner_thickness, liner_depth)),
        origin=Origin(xyz=(-x_side, slot_center_y, z_center)),
        material=material,
        name=f"slot_{slot_index}_front_liner",
    )
    body.visual(
        Box((liner_thickness, slot_width_y + 2.0 * liner_thickness, liner_depth)),
        origin=Origin(xyz=(x_side, slot_center_y, z_center)),
        material=material,
        name=f"slot_{slot_index}_rear_liner",
    )
    body.visual(
        Box((slot_len_x, liner_thickness, liner_depth)),
        origin=Origin(xyz=(0.0, slot_center_y - y_side, z_center)),
        material=material,
        name=f"slot_{slot_index}_left_liner",
    )
    body.visual(
        Box((slot_len_x, liner_thickness, liner_depth)),
        origin=Origin(xyz=(0.0, slot_center_y + y_side, z_center)),
        material=material,
        name=f"slot_{slot_index}_right_liner",
    )


def _build_carriage_part(model: ArticulatedObject, name: str, material) -> object:
    carriage = model.part(name)
    carriage.visual(
        Box((0.128, 0.124, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.072)),
        material=material,
        name="lift_plate",
    )
    carriage.visual(
        Box((0.006, 0.088, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=material,
        name="center_spine",
    )

    for suffix, local_y in (("a", -0.045), ("b", 0.045)):
        carriage.visual(
            Box((0.104, 0.004, 0.008)),
            origin=Origin(xyz=(0.0, local_y, -0.006)),
            material=material,
            name=f"slot_tab_{suffix}",
        )
        carriage.visual(
            Box((0.004, 0.004, 0.066)),
            origin=Origin(xyz=(-0.044, local_y, -0.039)),
            material=material,
            name=f"slot_post_{suffix}_left",
        )
        carriage.visual(
            Box((0.004, 0.004, 0.066)),
            origin=Origin(xyz=(0.044, local_y, -0.039)),
            material=material,
            name=f"slot_post_{suffix}_right",
        )
        carriage.visual(
            Box((0.112, 0.006, 0.054)),
            origin=Origin(xyz=(0.0, local_y, -0.036)),
            material=material,
            name=f"bread_guide_{suffix}",
        )

    carriage.inertial = Inertial.from_geometry(
        Box((0.128, 0.124, 0.080)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )
    return carriage


def _build_lever_part(model: ArticulatedObject, name: str, direction_sign: float, material, trim_material) -> object:
    lever = model.part(name)
    lever.visual(
        Box((0.010, 0.132, 0.006)),
        origin=Origin(xyz=(0.0, -0.066 * direction_sign, 0.0)),
        material=trim_material,
        name="lever_link",
    )
    lever.visual(
        Box((0.014, 0.026, 0.008)),
        origin=Origin(),
        material=material,
        name="lever_stem",
    )
    lever.visual(
        Box((0.026, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, 0.019 * direction_sign, 0.0)),
        material=material,
        name="lever_handle",
    )
    lever.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(
            xyz=(0.0, 0.026 * direction_sign, -0.017),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=trim_material,
        name="lever_grip",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.026, 0.044, 0.050)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.014 * direction_sign, 0.0)),
    )
    return lever


def _build_knob_part(model: ArticulatedObject, name: str, material, accent_material) -> object:
    knob = model.part(name)
    knob.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_material,
        name="shaft",
    )
    knob.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(-0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_material,
        name="collar",
    )
    knob.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.003, 0.010)),
        origin=Origin(xyz=(-0.026, 0.0, 0.012)),
        material=accent_material,
        name="pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.032, 0.036, 0.036)),
        mass=0.04,
        origin=Origin(xyz=(-0.016, 0.0, 0.0)),
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_slot_family_toaster")

    steel = model.material("steel", rgba=(0.78, 0.79, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.17, 0.18, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_marker = model.material("knob_marker", rgba=(0.86, 0.22, 0.14, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.62, 0.63, 0.66, 1.0))

    depth_x = 0.180
    length_y = 0.440
    height_z = 0.220
    wall_t = 0.003
    top_t = 0.003
    slot_len_x = 0.124
    slot_width_y = 0.028
    slot_centers_y = (-0.135, -0.045, 0.045, 0.135)
    top_z = height_z - top_t / 2.0

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((depth_x, length_y, height_z)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, height_z / 2.0)),
    )

    top_profile = rounded_rect_profile(depth_x - 0.008, length_y - 0.010, 0.020)
    hole_profile = rounded_rect_profile(slot_len_x, slot_width_y, 0.010)
    top_panel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            top_profile,
            [_translated_profile(hole_profile, dy=slot_center) for slot_center in slot_centers_y],
            top_t,
            center=True,
        ),
        "toaster_top_panel",
    )
    body.visual(
        top_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        material=steel,
        name="top_panel",
    )

    body.visual(
        Box((wall_t, length_y, height_z - 0.002)),
        origin=Origin(xyz=(-depth_x / 2.0 + wall_t / 2.0, 0.0, (height_z - 0.002) / 2.0)),
        material=steel,
        name="front_wall",
    )

    body.visual(
        Box((wall_t, length_y, 0.020)),
        origin=Origin(xyz=(depth_x / 2.0 - wall_t / 2.0, 0.0, 0.010)),
        material=steel,
        name="rear_lower_wall",
    )
    body.visual(
        Box((wall_t, length_y, 0.182)),
        origin=Origin(xyz=(depth_x / 2.0 - wall_t / 2.0, 0.0, 0.129)),
        material=steel,
        name="rear_upper_wall",
    )
    body.visual(
        Box((wall_t, 0.120, 0.018)),
        origin=Origin(xyz=(depth_x / 2.0 - wall_t / 2.0, -0.160, 0.029)),
        material=steel,
        name="tray_opening_left_jamb",
    )
    body.visual(
        Box((wall_t, 0.120, 0.018)),
        origin=Origin(xyz=(depth_x / 2.0 - wall_t / 2.0, 0.160, 0.029)),
        material=steel,
        name="tray_opening_right_jamb",
    )

    for end_name, end_sign in (("left", -1.0), ("right", 1.0)):
        end_y = end_sign * (length_y / 2.0 - wall_t / 2.0)
        body.visual(
            Box((depth_x - 0.010, wall_t, 0.055)),
            origin=Origin(xyz=(0.0, end_y, 0.0275)),
            material=steel,
            name=f"{end_name}_end_lower",
        )
        body.visual(
            Box((depth_x - 0.006, wall_t, 0.080)),
            origin=Origin(xyz=(0.0, end_y, 0.180)),
            material=steel,
            name=f"{end_name}_end_upper",
        )
        body.visual(
            Box((0.036, wall_t, 0.085)),
            origin=Origin(xyz=(-0.072, end_y, 0.0975)),
            material=steel,
            name=f"{end_name}_lever_slot_front_strip",
        )
        body.visual(
            Box((0.130, wall_t, 0.085)),
            origin=Origin(xyz=(0.025, end_y, 0.0975)),
            material=steel,
            name=f"{end_name}_lever_slot_rear_strip",
        )

    body.visual(
        Cylinder(radius=0.010, length=length_y - 0.020),
        origin=Origin(
            xyz=(-depth_x / 2.0 + 0.010, 0.0, height_z - 0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="front_shoulder",
    )
    body.visual(
        Cylinder(radius=0.010, length=length_y - 0.020),
        origin=Origin(
            xyz=(depth_x / 2.0 - 0.010, 0.0, height_z - 0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="rear_shoulder",
    )
    body.visual(
        Box((0.120, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, -0.203, 0.006)),
        material=dark_trim,
        name="left_base_runner",
    )
    body.visual(
        Box((0.120, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, 0.203, 0.006)),
        material=dark_trim,
        name="right_base_runner",
    )

    for slot_index, slot_center in enumerate(slot_centers_y):
        _add_slot_liner(
            body,
            slot_index=slot_index,
            slot_center_y=slot_center,
            slot_len_x=slot_len_x,
            slot_width_y=slot_width_y,
            top_z=height_z - top_t,
            liner_depth=0.092,
            liner_thickness=0.003,
            material=dark_trim,
        )

    left_carriage = _build_carriage_part(model, "left_carriage", dark_trim)
    right_carriage = _build_carriage_part(model, "right_carriage", dark_trim)

    model.articulation(
        "body_to_left_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_carriage,
        origin=Origin(xyz=(0.0, -0.090, 0.202)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.15, lower=0.0, upper=0.070),
    )
    model.articulation(
        "body_to_right_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_carriage,
        origin=Origin(xyz=(0.0, 0.090, 0.202)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.15, lower=0.0, upper=0.070),
    )

    left_lever = _build_lever_part(model, "left_lever", -1.0, black_plastic, dark_trim)
    right_lever = _build_lever_part(model, "right_lever", 1.0, black_plastic, dark_trim)

    model.articulation(
        "left_carriage_to_lever",
        ArticulationType.FIXED,
        parent=left_carriage,
        child=left_lever,
        origin=Origin(xyz=(-0.047, -0.1285, -0.102)),
    )
    model.articulation(
        "right_carriage_to_lever",
        ArticulationType.FIXED,
        parent=right_carriage,
        child=right_lever,
        origin=Origin(xyz=(-0.047, 0.1285, -0.102)),
    )

    timer_knob = _build_knob_part(model, "timer_knob", black_plastic, knob_marker)
    browning_knob = _build_knob_part(model, "browning_knob", black_plastic, knob_marker)

    model.articulation(
        "body_to_timer_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=timer_knob,
        origin=Origin(xyz=(-depth_x / 2.0, -0.070, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=5.0,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "body_to_browning_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=browning_knob,
        origin=Origin(xyz=(-depth_x / 2.0, 0.070, 0.086)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=5.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        Box((0.180, 0.320, 0.0025)),
        origin=Origin(xyz=(-0.090, 0.0, 0.00125)),
        material=tray_metal,
        name="tray_floor",
    )
    crumb_tray.visual(
        Box((0.180, 0.003, 0.010)),
        origin=Origin(xyz=(-0.090, -0.1585, 0.005)),
        material=tray_metal,
        name="tray_left_lip",
    )
    crumb_tray.visual(
        Box((0.180, 0.003, 0.010)),
        origin=Origin(xyz=(-0.090, 0.1585, 0.005)),
        material=tray_metal,
        name="tray_right_lip",
    )
    crumb_tray.visual(
        Box((0.003, 0.320, 0.010)),
        origin=Origin(xyz=(-0.1785, 0.0, 0.005)),
        material=tray_metal,
        name="tray_inner_stop",
    )
    crumb_tray.visual(
        Box((0.018, 0.120, 0.018)),
        origin=Origin(xyz=(0.006, 0.0, 0.009)),
        material=black_plastic,
        name="tray_pull",
    )
    crumb_tray.inertial = Inertial.from_geometry(
        Box((0.192, 0.320, 0.018)),
        mass=0.18,
        origin=Origin(xyz=(-0.086, 0.0, 0.009)),
    )

    model.articulation(
        "body_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=crumb_tray,
        origin=Origin(xyz=(depth_x / 2.0 - wall_t / 2.0, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.20, lower=0.0, upper=0.095),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_carriage = object_model.get_part("left_carriage")
    right_carriage = object_model.get_part("right_carriage")
    left_lever = object_model.get_part("left_lever")
    right_lever = object_model.get_part("right_lever")
    crumb_tray = object_model.get_part("crumb_tray")

    left_carriage_joint = object_model.get_articulation("body_to_left_carriage")
    right_carriage_joint = object_model.get_articulation("body_to_right_carriage")
    crumb_tray_joint = object_model.get_articulation("body_to_crumb_tray")

    ctx.expect_gap(
        body,
        left_carriage,
        axis="z",
        positive_elem="top_panel",
        max_gap=0.028,
        max_penetration=0.0,
        name="left carriage parks just below the slot panel",
    )
    ctx.expect_gap(
        body,
        right_carriage,
        axis="z",
        positive_elem="top_panel",
        max_gap=0.028,
        max_penetration=0.0,
        name="right carriage parks just below the slot panel",
    )

    left_carriage_rest = ctx.part_world_position(left_carriage)
    left_lever_rest = ctx.part_world_position(left_lever)
    with ctx.pose({left_carriage_joint: 0.060}):
        left_carriage_lowered = ctx.part_world_position(left_carriage)
        left_lever_lowered = ctx.part_world_position(left_lever)
        ctx.check(
            "left carriage lowers when pressed",
            left_carriage_rest is not None
            and left_carriage_lowered is not None
            and left_carriage_lowered[2] < left_carriage_rest[2] - 0.045,
            details=f"rest={left_carriage_rest}, lowered={left_carriage_lowered}",
        )
        ctx.check(
            "left lever follows left carriage",
            left_carriage_rest is not None
            and left_carriage_lowered is not None
            and left_lever_rest is not None
            and left_lever_lowered is not None
            and abs(
                (left_carriage_rest[2] - left_carriage_lowered[2])
                - (left_lever_rest[2] - left_lever_lowered[2])
            )
            < 0.003,
            details=(
                f"carriage_rest={left_carriage_rest}, carriage_lowered={left_carriage_lowered}, "
                f"lever_rest={left_lever_rest}, lever_lowered={left_lever_lowered}"
            ),
        )

    right_carriage_rest = ctx.part_world_position(right_carriage)
    right_lever_rest = ctx.part_world_position(right_lever)
    with ctx.pose({right_carriage_joint: 0.052}):
        right_carriage_lowered = ctx.part_world_position(right_carriage)
        right_lever_lowered = ctx.part_world_position(right_lever)
        ctx.check(
            "right carriage lowers when pressed",
            right_carriage_rest is not None
            and right_carriage_lowered is not None
            and right_carriage_lowered[2] < right_carriage_rest[2] - 0.038,
            details=f"rest={right_carriage_rest}, lowered={right_carriage_lowered}",
        )
        ctx.check(
            "right lever follows right carriage",
            right_carriage_rest is not None
            and right_carriage_lowered is not None
            and right_lever_rest is not None
            and right_lever_lowered is not None
            and abs(
                (right_carriage_rest[2] - right_carriage_lowered[2])
                - (right_lever_rest[2] - right_lever_lowered[2])
            )
            < 0.003,
            details=(
                f"carriage_rest={right_carriage_rest}, carriage_lowered={right_carriage_lowered}, "
                f"lever_rest={right_lever_rest}, lever_lowered={right_lever_lowered}"
            ),
        )

    crumb_tray_rest = ctx.part_world_position(crumb_tray)
    with ctx.pose({crumb_tray_joint: 0.095}):
        crumb_tray_extended = ctx.part_world_position(crumb_tray)
        ctx.check(
            "crumb tray slides out from the side",
            crumb_tray_rest is not None
            and crumb_tray_extended is not None
            and crumb_tray_extended[0] > crumb_tray_rest[0] + 0.090,
            details=f"rest={crumb_tray_rest}, extended={crumb_tray_extended}",
        )
        ctx.expect_within(
            crumb_tray,
            body,
            axes="yz",
            margin=0.025,
            name="crumb tray stays aligned with the body slot",
        )
        ctx.expect_overlap(
            crumb_tray,
            body,
            axes="x",
            min_overlap=0.050,
            name="crumb tray retains insertion when extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
