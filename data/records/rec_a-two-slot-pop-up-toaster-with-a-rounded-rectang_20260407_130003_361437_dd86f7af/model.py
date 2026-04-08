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


BODY_W = 0.310
BODY_D = 0.185
BODY_H = 0.202
CORNER_R = 0.034
WALL_T = 0.003
BASE_H = 0.012
SHELL_H = 0.182
TOP_T = 0.004
TOP_Z = BASE_H + SHELL_H + TOP_T * 0.5
SLOT_W = 0.032
SLOT_D = 0.118
SLOT_X = 0.055


def _offset_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_slot_popup_toaster")

    stainless = model.material("stainless", rgba=(0.80, 0.81, 0.82, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.14, 0.15, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.65, 0.66, 0.68, 1.0))
    interior_gray = model.material("interior_gray", rgba=(0.45, 0.46, 0.48, 1.0))

    housing = model.part("housing")

    wall_center_z = BASE_H + SHELL_H * 0.5
    flat_span_x = BODY_W - 2.0 * CORNER_R
    flat_span_y = BODY_D - 2.0 * CORNER_R

    housing.visual(
        Box((BODY_W - 0.040, BODY_D - 0.030, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H * 0.5)),
        material=dark_plastic,
        name="base_plinth",
    )

    housing.visual(
        Box((flat_span_x, WALL_T, SHELL_H)),
        origin=Origin(
            xyz=(0.0, -(BODY_D * 0.5 - WALL_T * 0.5), wall_center_z),
        ),
        material=stainless,
        name="front_wall",
    )
    housing.visual(
        Box((flat_span_x, WALL_T, SHELL_H)),
        origin=Origin(
            xyz=(0.0, BODY_D * 0.5 - WALL_T * 0.5, wall_center_z),
        ),
        material=stainless,
        name="rear_wall",
    )
    housing.visual(
        Box((WALL_T, flat_span_y, SHELL_H)),
        origin=Origin(
            xyz=(-(BODY_W * 0.5 - WALL_T * 0.5), 0.0, wall_center_z),
        ),
        material=stainless,
        name="left_wall",
    )

    right_wall_x = BODY_W * 0.5 - WALL_T * 0.5
    housing.visual(
        Box((WALL_T, 0.028, SHELL_H)),
        origin=Origin(xyz=(right_wall_x, -0.0445, wall_center_z)),
        material=stainless,
        name="right_wall_front_strip",
    )
    housing.visual(
        Box((WALL_T, 0.028, SHELL_H)),
        origin=Origin(xyz=(right_wall_x, 0.0445, wall_center_z)),
        material=stainless,
        name="right_wall_rear_strip",
    )
    housing.visual(
        Box((WALL_T, 0.061, 0.052)),
        origin=Origin(xyz=(right_wall_x, 0.0, 0.170)),
        material=stainless,
        name="right_wall_upper_bridge",
    )
    housing.visual(
        Box((WALL_T, 0.061, 0.014)),
        origin=Origin(xyz=(right_wall_x, 0.0, 0.019)),
        material=stainless,
        name="right_wall_lower_bridge",
    )
    housing.visual(
        Box((WALL_T, 0.012, 0.118)),
        origin=Origin(xyz=(right_wall_x, -0.002, 0.085)),
        material=stainless,
        name="right_wall_center_web",
    )
    housing.visual(
        Box((0.003, 0.014, 0.090)),
        origin=Origin(xyz=(BODY_W * 0.5 + 0.0045, 0.022, 0.100)),
        material=dark_plastic,
        name="lever_guide_pad",
    )
    housing.visual(
        Box((0.003, 0.010, 0.006)),
        origin=Origin(xyz=(BODY_W * 0.5 + 0.0015, 0.022, 0.147)),
        material=dark_plastic,
        name="lever_guide_strut",
    )
    housing.visual(
        Box((0.003, 0.024, 0.034)),
        origin=Origin(xyz=(BODY_W * 0.5 + 0.0045, -0.022, 0.082)),
        material=dark_plastic,
        name="knob_bezel_mount",
    )
    housing.visual(
        Box((0.003, 0.017, 0.008)),
        origin=Origin(xyz=(BODY_W * 0.5 + 0.0015, -0.027, 0.102)),
        material=dark_plastic,
        name="knob_bezel_strut",
    )

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            housing.visual(
                Cylinder(radius=CORNER_R, length=SHELL_H),
                origin=Origin(
                    xyz=(
                        sx * (BODY_W * 0.5 - CORNER_R),
                        sy * (BODY_D * 0.5 - CORNER_R),
                        wall_center_z,
                    )
                ),
                material=stainless,
                name=f"corner_{'r' if sx > 0 else 'l'}_{'rear' if sy > 0 else 'front'}",
            )

    top_panel = ExtrudeWithHolesGeometry(
        rounded_rect_profile(BODY_W, BODY_D, CORNER_R),
        [
            _offset_profile(rounded_rect_profile(SLOT_W, SLOT_D, 0.010), -SLOT_X, 0.0),
            _offset_profile(rounded_rect_profile(SLOT_W, SLOT_D, 0.010), SLOT_X, 0.0),
        ],
        TOP_T,
        cap=True,
        center=True,
        closed=True,
    )
    housing.visual(
        mesh_from_geometry(top_panel, "toaster_top_panel"),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        material=stainless,
        name="top_panel",
    )

    liner_h = 0.060
    liner_t = 0.0025
    liner_z = TOP_Z - TOP_T * 0.5 - liner_h * 0.5
    for slot_name, slot_x in (("left", -SLOT_X), ("right", SLOT_X)):
        housing.visual(
            Box((liner_t, SLOT_D, liner_h)),
            origin=Origin(xyz=(slot_x - SLOT_W * 0.5 - liner_t * 0.5, 0.0, liner_z)),
            material=interior_gray,
            name=f"{slot_name}_liner_left",
        )
        housing.visual(
            Box((liner_t, SLOT_D, liner_h)),
            origin=Origin(xyz=(slot_x + SLOT_W * 0.5 + liner_t * 0.5, 0.0, liner_z)),
            material=interior_gray,
            name=f"{slot_name}_liner_right",
        )
        housing.visual(
            Box((SLOT_W + 2.0 * liner_t, liner_t, liner_h)),
            origin=Origin(xyz=(slot_x, -(SLOT_D * 0.5 + liner_t * 0.5), liner_z)),
            material=interior_gray,
            name=f"{slot_name}_liner_front",
        )
        housing.visual(
            Box((SLOT_W + 2.0 * liner_t, liner_t, liner_h)),
            origin=Origin(xyz=(slot_x, SLOT_D * 0.5 + liner_t * 0.5, liner_z)),
            material=interior_gray,
            name=f"{slot_name}_liner_rear",
        )

    housing.visual(
        Box((0.108, 0.118, 0.002)),
        origin=Origin(xyz=(0.100, 0.0, 0.025)),
        material=interior_gray,
        name="tray_guide_floor",
    )

    for fx, fy, name in (
        (-0.105, -0.060, "front_left_foot"),
        (0.105, -0.060, "front_right_foot"),
        (-0.105, 0.060, "rear_left_foot"),
        (0.105, 0.060, "rear_right_foot"),
    ):
        housing.visual(
            Box((0.022, 0.014, 0.005)),
            origin=Origin(xyz=(fx, fy, 0.0025)),
            material=dark_plastic,
            name=name,
        )

    housing.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
    )

    carriage = model.part("bread_carriage")
    carriage.visual(
        Box((0.145, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=interior_gray,
        name="crossbar",
    )
    for slot_name, slot_x in (("left", -SLOT_X), ("right", SLOT_X)):
        carriage.visual(
            Box((0.022, 0.100, 0.002)),
            origin=Origin(xyz=(slot_x, 0.0, 0.020)),
            material=interior_gray,
            name=f"{slot_name}_lift_plate",
        )
        carriage.visual(
            Box((0.004, 0.010, 0.040)),
            origin=Origin(xyz=(slot_x - 0.010, 0.0, 0.0)),
            material=interior_gray,
            name=f"{slot_name}_post_inner",
        )
        carriage.visual(
            Box((0.004, 0.010, 0.040)),
            origin=Origin(xyz=(slot_x + 0.010, 0.0, 0.0)),
            material=interior_gray,
            name=f"{slot_name}_post_outer",
        )
        carriage.visual(
            Box((0.028, 0.050, 0.003)),
            origin=Origin(xyz=(slot_x, 0.0, -0.019)),
            material=interior_gray,
            name=f"{slot_name}_bread_support",
        )
    carriage.inertial = Inertial.from_geometry(
        Box((0.150, 0.120, 0.045)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )

    carriage_joint = model.articulation(
        "housing_to_bread_carriage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.075),
    )

    lever = model.part("carriage_lever")
    lever.visual(
        Box((0.040, 0.012, 0.006)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=interior_gray,
        name="lever_link",
    )
    lever.visual(
        Box((0.037, 0.006, 0.006)),
        origin=Origin(xyz=(0.0185, 0.0, 0.0)),
        material=interior_gray,
        name="lever_shaft",
    )
    lever.visual(
        Box((0.018, 0.030, 0.012)),
        origin=Origin(xyz=(0.049, -0.020, 0.0)),
        material=dark_plastic,
        name="lever_handle",
    )
    lever.visual(
        Box((0.003, 0.018, 0.010)),
        origin=Origin(xyz=(0.0385, -0.012, 0.0)),
        material=dark_plastic,
        name="lever_handle_neck",
    )
    lever.visual(
        Box((0.003, 0.014, 0.016)),
        origin=Origin(xyz=(0.0385, 0.0, 0.0)),
        material=dark_plastic,
        name="lever_slide_shoe",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.075, 0.030, 0.012)),
        mass=0.08,
        origin=Origin(xyz=(0.015, 0.0, 0.0)),
    )
    model.articulation(
        "bread_carriage_to_lever",
        ArticulationType.FIXED,
        parent=carriage,
        child=lever,
        origin=Origin(xyz=(0.118, 0.018, -0.018)),
    )

    knob = model.part("browning_knob")
    knob.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=interior_gray,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Box((0.010, 0.002, 0.010)),
        origin=Origin(xyz=(0.024, 0.014, 0.0)),
        material=interior_gray,
        name="knob_indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.022),
        mass=0.05,
        origin=Origin(xyz=(0.0125, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
    )
    knob_joint = model.articulation(
        "housing_to_browning_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(BODY_W * 0.5 + 0.006, -0.022, 0.082)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=-math.radians(150.0),
            upper=math.radians(150.0),
        ),
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        Box((0.110, 0.114, 0.002)),
        origin=Origin(xyz=(-0.055, 0.0, 0.001)),
        material=tray_metal,
        name="tray_pan",
    )
    crumb_tray.visual(
        Box((0.110, 0.004, 0.008)),
        origin=Origin(xyz=(-0.055, -0.055, 0.004)),
        material=tray_metal,
        name="tray_front_flange",
    )
    crumb_tray.visual(
        Box((0.110, 0.004, 0.008)),
        origin=Origin(xyz=(-0.055, 0.055, 0.004)),
        material=tray_metal,
        name="tray_rear_flange",
    )
    crumb_tray.visual(
        Box((0.004, 0.114, 0.010)),
        origin=Origin(xyz=(-0.108, 0.0, 0.005)),
        material=tray_metal,
        name="tray_inner_wall",
    )
    crumb_tray.visual(
        Box((0.012, 0.052, 0.018)),
        origin=Origin(xyz=(0.006, 0.0, 0.009)),
        material=dark_plastic,
        name="tray_handle",
    )
    crumb_tray.inertial = Inertial.from_geometry(
        Box((0.122, 0.114, 0.018)),
        mass=0.16,
        origin=Origin(xyz=(-0.049, 0.0, 0.009)),
    )
    crumb_tray_joint = model.articulation(
        "housing_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crumb_tray,
        origin=Origin(xyz=(BODY_W * 0.5, 0.0, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.15, lower=0.0, upper=0.065),
    )

    model.meta["carriage_joint_name"] = carriage_joint.name
    model.meta["knob_joint_name"] = knob_joint.name
    model.meta["crumb_tray_joint_name"] = crumb_tray_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    carriage = object_model.get_part("bread_carriage")
    lever = object_model.get_part("carriage_lever")
    knob = object_model.get_part("browning_knob")
    crumb_tray = object_model.get_part("crumb_tray")

    carriage_joint = object_model.get_articulation("housing_to_bread_carriage")
    crumb_tray_joint = object_model.get_articulation("housing_to_crumb_tray")
    knob_joint = object_model.get_articulation("housing_to_browning_knob")

    ctx.expect_origin_gap(
        lever,
        knob,
        axis="z",
        min_gap=0.035,
        max_gap=0.080,
        name="lever sits above browning knob",
    )

    ctx.expect_gap(
        crumb_tray,
        housing,
        axis="z",
        positive_elem="tray_pan",
        negative_elem="tray_guide_floor",
        min_gap=0.001,
        max_gap=0.004,
        name="crumb tray rides just above guide floor when closed",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_lever_pos = ctx.part_world_position(lever)
    rest_tray_pos = ctx.part_world_position(crumb_tray)

    with ctx.pose({carriage_joint: carriage_joint.motion_limits.upper}):
        lowered_carriage_pos = ctx.part_world_position(carriage)
        lowered_lever_pos = ctx.part_world_position(lever)
        ctx.expect_overlap(
            carriage,
            housing,
            axes="xy",
            min_overlap=0.080,
            name="bread carriage remains centered inside housing while lowered",
        )

    with ctx.pose({crumb_tray_joint: crumb_tray_joint.motion_limits.upper}):
        extended_tray_pos = ctx.part_world_position(crumb_tray)
        ctx.expect_gap(
            crumb_tray,
            housing,
            axis="z",
            positive_elem="tray_pan",
            negative_elem="tray_guide_floor",
            min_gap=0.001,
            max_gap=0.004,
            name="crumb tray stays on the same guide plane when extended",
        )
        ctx.expect_overlap(
            crumb_tray,
            housing,
            axes="y",
            min_overlap=0.050,
            name="crumb tray stays laterally aligned with the toaster opening when extended",
        )

    ctx.check(
        "carriage lowers through a usable pop-up stroke",
        rest_carriage_pos is not None
        and lowered_carriage_pos is not None
        and lowered_carriage_pos[2] < rest_carriage_pos[2] - 0.060,
        details=f"rest={rest_carriage_pos}, lowered={lowered_carriage_pos}",
    )
    ctx.check(
        "lever follows the bread carriage on the same vertical path",
        rest_carriage_pos is not None
        and lowered_carriage_pos is not None
        and rest_lever_pos is not None
        and lowered_lever_pos is not None
        and abs((rest_lever_pos[2] - rest_carriage_pos[2]) - (lowered_lever_pos[2] - lowered_carriage_pos[2])) < 1e-6
        and lowered_lever_pos[2] < rest_lever_pos[2] - 0.060,
        details=(
            f"rest_carriage={rest_carriage_pos}, lowered_carriage={lowered_carriage_pos}, "
            f"rest_lever={rest_lever_pos}, lowered_lever={lowered_lever_pos}"
        ),
    )
    ctx.check(
        "crumb tray slides outward from the lower body",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > rest_tray_pos[0] + 0.050,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )
    ctx.check(
        "browning knob rotates on a short side shaft",
        knob_joint.motion_limits is not None
        and knob_joint.axis == (1.0, 0.0, 0.0)
        and knob_joint.motion_limits.lower is not None
        and knob_joint.motion_limits.upper is not None
        and knob_joint.motion_limits.upper - knob_joint.motion_limits.lower > 4.5,
        details=f"axis={knob_joint.axis}, limits={knob_joint.motion_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
