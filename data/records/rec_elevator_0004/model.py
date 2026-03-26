from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dumbwaiter", assets=ASSETS)

    steel = model.material("steel", rgba=(0.60, 0.62, 0.64, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.42, 0.44, 0.46, 1.0))
    car_shell = model.material("car_shell", rgba=(0.78, 0.80, 0.74, 1.0))
    door_finish = model.material("door_finish", rgba=(0.72, 0.67, 0.56, 1.0))
    rope_fiber = model.material("rope_fiber", rgba=(0.66, 0.55, 0.35, 1.0))

    shaft_w = 0.34
    shaft_d = 0.28
    shaft_h = 0.86
    wall_t = 0.012
    left_inner_x = -shaft_w / 2.0 + wall_t
    right_inner_x = shaft_w / 2.0 - wall_t

    rail_t = 0.006
    rail_leg_y = 0.028
    rail_leg_x = 0.022
    rail_corner_y = 0.052
    rail_z0 = 0.14
    rail_h = 0.66

    car_w = 0.22
    car_d = 0.18
    shell_t = 0.012
    car_z0 = 0.02
    car_h = 0.24
    roof_t = 0.012
    front_plane_y = -car_d / 2.0
    back_plane_y = car_d / 2.0

    shaft = model.part("shaft")
    shaft.visual(
        Box((wall_t, shaft_d, shaft_h)),
        origin=Origin(xyz=(-shaft_w / 2.0 + wall_t / 2.0, 0.0, shaft_h / 2.0)),
        material=steel,
        name="left_wall",
    )
    shaft.visual(
        Box((wall_t, shaft_d, shaft_h)),
        origin=Origin(xyz=(shaft_w / 2.0 - wall_t / 2.0, 0.0, shaft_h / 2.0)),
        material=steel,
        name="right_wall",
    )
    shaft.visual(
        Box((shaft_w - 2.0 * wall_t, wall_t, shaft_h)),
        origin=Origin(xyz=(0.0, shaft_d / 2.0 - wall_t / 2.0, shaft_h / 2.0)),
        material=steel,
        name="back_wall",
    )
    shaft.visual(
        Box((shaft_w, shaft_d, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=steel,
        name="base_plate",
    )
    shaft.visual(
        Box((shaft_w, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, -shaft_d / 2.0 + 0.014, shaft_h - 0.010)),
        material=steel,
        name="front_lintel",
    )
    shaft.visual(
        Box((shaft_w, 0.100, 0.020)),
        origin=Origin(xyz=(0.0, 0.075, shaft_h - 0.010)),
        material=steel,
        name="top_crosshead",
    )

    shaft.visual(
        Box((rail_t, rail_leg_y, rail_h)),
        origin=Origin(
            xyz=(
                left_inner_x + rail_t / 2.0,
                rail_corner_y + rail_leg_y / 2.0,
                rail_z0 + rail_h / 2.0,
            )
        ),
        material=rail_steel,
        name="left_rail_wall",
    )
    shaft.visual(
        Box((rail_leg_x, rail_t, rail_h)),
        origin=Origin(
            xyz=(
                left_inner_x + rail_t + rail_leg_x / 2.0,
                rail_corner_y + rail_t / 2.0,
                rail_z0 + rail_h / 2.0,
            )
        ),
        material=rail_steel,
        name="left_rail_flange",
    )
    shaft.visual(
        Box((rail_t, rail_leg_y, rail_h)),
        origin=Origin(
            xyz=(
                right_inner_x - rail_t / 2.0,
                rail_corner_y + rail_leg_y / 2.0,
                rail_z0 + rail_h / 2.0,
            )
        ),
        material=rail_steel,
        name="right_rail_wall",
    )
    shaft.visual(
        Box((rail_leg_x, rail_t, rail_h)),
        origin=Origin(
            xyz=(
                right_inner_x - rail_t - rail_leg_x / 2.0,
                rail_corner_y + rail_t / 2.0,
                rail_z0 + rail_h / 2.0,
            )
        ),
        material=rail_steel,
        name="right_rail_flange",
    )

    pulley_x = -0.112
    pulley_y = -0.106
    pulley_z = 0.815
    pulley_radius = 0.028
    pulley_width = 0.018

    shaft.visual(
        Box((0.006, 0.022, 0.050)),
        origin=Origin(xyz=(pulley_x - 0.012, -0.111, shaft_h - 0.035)),
        material=steel,
        name="pulley_left_bracket",
    )
    shaft.visual(
        Box((0.006, 0.022, 0.050)),
        origin=Origin(xyz=(pulley_x + 0.012, -0.111, shaft_h - 0.035)),
        material=steel,
        name="pulley_right_bracket",
    )
    shaft.visual(
        Cylinder(radius=0.003, length=0.030),
        origin=Origin(xyz=(pulley_x, pulley_y, pulley_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pulley_axle",
    )
    shaft.visual(
        Cylinder(radius=pulley_radius, length=pulley_width),
        origin=Origin(xyz=(pulley_x, pulley_y, pulley_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_steel,
        name="pulley_wheel",
    )
    shaft.visual(
        Cylinder(radius=0.003, length=0.052),
        origin=Origin(xyz=(pulley_x, pulley_y, pulley_z + pulley_radius - 0.001), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rope_fiber,
        name="top_rope_bridge",
    )
    shaft.visual(
        Cylinder(radius=0.003, length=0.560),
        origin=Origin(xyz=(pulley_x, pulley_y - pulley_radius, 0.547), rpy=(0.0, 0.0, 0.0)),
        material=rope_fiber,
        name="hand_pull_rope",
    )
    shaft.inertial = Inertial.from_geometry(
        Box((shaft_w, shaft_d, shaft_h)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, shaft_h / 2.0)),
    )

    car = model.part("car")
    car.visual(
        Box((car_w, car_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, car_z0 + shell_t / 2.0)),
        material=car_shell,
        name="floor",
    )
    car.visual(
        Box((car_w, car_d, roof_t)),
        origin=Origin(xyz=(0.0, 0.0, car_z0 + car_h - roof_t / 2.0)),
        material=car_shell,
        name="roof",
    )
    car.visual(
        Box((shell_t, car_d, car_h - roof_t)),
        origin=Origin(xyz=(-car_w / 2.0 + shell_t / 2.0, 0.0, car_z0 + (car_h - roof_t) / 2.0)),
        material=car_shell,
        name="left_side",
    )
    car.visual(
        Box((shell_t, car_d, car_h - roof_t)),
        origin=Origin(xyz=(car_w / 2.0 - shell_t / 2.0, 0.0, car_z0 + (car_h - roof_t) / 2.0)),
        material=car_shell,
        name="right_side",
    )
    car.visual(
        Box((car_w, shell_t, car_h - roof_t)),
        origin=Origin(xyz=(0.0, back_plane_y - shell_t / 2.0, car_z0 + (car_h - roof_t) / 2.0)),
        material=car_shell,
        name="back_panel",
    )
    car.visual(
        Box((0.148, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.084, 0.026)),
        material=car_shell,
        name="front_sill",
    )
    car.visual(
        Box((0.024, 0.012, 0.162)),
        origin=Origin(xyz=(-0.086, -0.084, 0.113)),
        material=car_shell,
        name="left_post",
    )
    car.visual(
        Box((0.024, 0.012, 0.162)),
        origin=Origin(xyz=(0.086, -0.084, 0.113)),
        material=car_shell,
        name="right_post",
    )

    track_y = -0.096
    car.visual(
        Box((0.300, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, track_y, 0.197)),
        material=rail_steel,
        name="track_web",
    )
    car.visual(
        Box((0.300, 0.002, 0.018)),
        origin=Origin(xyz=(0.0, track_y - 0.007, 0.188)),
        material=rail_steel,
        name="track_front_lip",
    )
    car.visual(
        Box((0.300, 0.002, 0.018)),
        origin=Origin(xyz=(0.0, track_y + 0.007, 0.188)),
        material=rail_steel,
        name="track_rear_lip",
    )

    tab_height = 0.045
    side_tab_x = 0.046
    side_tab_y = 0.020
    side_tab_center_y = 0.042
    rear_tab_x = 0.042
    rear_tab_y = 0.032
    rear_tab_center_x = 0.131
    upper_tab_z = 0.175
    lower_tab_z = 0.085

    car.visual(
        Box((side_tab_x, side_tab_y, tab_height)),
        origin=Origin(xyz=(-0.129, side_tab_center_y, upper_tab_z)),
        material=rail_steel,
        name="left_upper_side_tab",
    )
    car.visual(
        Box((rear_tab_x, rear_tab_y, tab_height)),
        origin=Origin(xyz=(-rear_tab_center_x, 0.074, upper_tab_z)),
        material=rail_steel,
        name="left_upper_rear_tab",
    )
    car.visual(
        Box((side_tab_x, side_tab_y, tab_height)),
        origin=Origin(xyz=(-0.129, side_tab_center_y, lower_tab_z)),
        material=rail_steel,
        name="left_lower_side_tab",
    )
    car.visual(
        Box((rear_tab_x, rear_tab_y, tab_height)),
        origin=Origin(xyz=(-rear_tab_center_x, 0.074, lower_tab_z)),
        material=rail_steel,
        name="left_lower_rear_tab",
    )
    car.visual(
        Box((side_tab_x, side_tab_y, tab_height)),
        origin=Origin(xyz=(0.129, side_tab_center_y, upper_tab_z)),
        material=rail_steel,
        name="right_upper_side_tab",
    )
    car.visual(
        Box((rear_tab_x, rear_tab_y, tab_height)),
        origin=Origin(xyz=(rear_tab_center_x, 0.074, upper_tab_z)),
        material=rail_steel,
        name="right_upper_rear_tab",
    )
    car.visual(
        Box((side_tab_x, side_tab_y, tab_height)),
        origin=Origin(xyz=(0.129, side_tab_center_y, lower_tab_z)),
        material=rail_steel,
        name="right_lower_side_tab",
    )
    car.visual(
        Box((rear_tab_x, rear_tab_y, tab_height)),
        origin=Origin(xyz=(rear_tab_center_x, 0.074, lower_tab_z)),
        material=rail_steel,
        name="right_lower_rear_tab",
    )
    car.inertial = Inertial.from_geometry(
        Box((car_w, car_d, car_h)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, car_h / 2.0)),
    )

    door = model.part("door")
    door.visual(
        Box((0.150, 0.008, 0.154)),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=door_finish,
        name="door_panel",
    )
    door.visual(
        Box((0.018, 0.006, 0.008)),
        origin=Origin(xyz=(-0.045, 0.0, -0.004)),
        material=rail_steel,
        name="left_hanger",
    )
    door.visual(
        Box((0.018, 0.006, 0.008)),
        origin=Origin(xyz=(0.045, 0.0, -0.004)),
        material=rail_steel,
        name="right_hanger",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.150, 0.008, 0.154)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
    )

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.25, lower=0.0, upper=0.34),
    )
    model.articulation(
        "car_to_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door,
        origin=Origin(xyz=(0.0, -0.096, 0.194)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.50, lower=0.0, upper=0.075),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    door = object_model.get_part("door")
    shaft_to_car = object_model.get_articulation("shaft_to_car")
    car_to_door = object_model.get_articulation("car_to_door")

    back_wall = shaft.get_visual("back_wall")
    left_rail_wall = shaft.get_visual("left_rail_wall")
    left_rail_flange = shaft.get_visual("left_rail_flange")
    right_rail_wall = shaft.get_visual("right_rail_wall")
    right_rail_flange = shaft.get_visual("right_rail_flange")
    pulley_wheel = shaft.get_visual("pulley_wheel")
    hand_pull_rope = shaft.get_visual("hand_pull_rope")

    back_panel = car.get_visual("back_panel")
    roof = car.get_visual("roof")
    left_post = car.get_visual("left_post")
    left_upper_side_tab = car.get_visual("left_upper_side_tab")
    left_upper_rear_tab = car.get_visual("left_upper_rear_tab")
    left_lower_side_tab = car.get_visual("left_lower_side_tab")
    left_lower_rear_tab = car.get_visual("left_lower_rear_tab")
    right_upper_side_tab = car.get_visual("right_upper_side_tab")
    right_upper_rear_tab = car.get_visual("right_upper_rear_tab")
    right_lower_side_tab = car.get_visual("right_lower_side_tab")
    right_lower_rear_tab = car.get_visual("right_lower_rear_tab")
    track_web = car.get_visual("track_web")
    track_rear_lip = car.get_visual("track_rear_lip")

    door_panel = door.get_visual("door_panel")
    left_hanger = door.get_visual("left_hanger")
    right_hanger = door.get_visual("right_hanger")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # The car rides on guide tabs but translates along a centered lift axis inside an open shaft cavity,
    # so the articulation origin is intentionally away from the shaft shell geometry.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.130)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(car, shaft, axes="xy", max_dist=0.015)
    ctx.expect_within(car, shaft, axes="xy")
    ctx.expect_gap(
        shaft,
        car,
        axis="y",
        min_gap=0.020,
        max_gap=0.040,
        positive_elem=back_wall,
        negative_elem=back_panel,
        name="car stays forward of the shaft back wall",
    )

    ctx.expect_contact(car, shaft, elem_a=left_upper_side_tab, elem_b=left_rail_wall)
    ctx.expect_contact(car, shaft, elem_a=left_upper_rear_tab, elem_b=left_rail_flange)
    ctx.expect_contact(car, shaft, elem_a=right_upper_side_tab, elem_b=right_rail_wall)
    ctx.expect_contact(car, shaft, elem_a=right_upper_rear_tab, elem_b=right_rail_flange)

    ctx.expect_contact(door, car, elem_a=left_hanger, elem_b=track_web)
    ctx.expect_contact(door, car, elem_a=right_hanger, elem_b=track_web)
    ctx.expect_overlap(door, car, axes="xz", min_overlap=0.14)
    ctx.expect_gap(
        car,
        door,
        axis="y",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem=track_rear_lip,
        negative_elem=door_panel,
        name="front hatch sits just proud of the car face",
    )
    ctx.expect_gap(
        door,
        shaft,
        axis="y",
        min_gap=0.020,
        max_gap=0.060,
        positive_elem=door_panel,
        negative_elem=hand_pull_rope,
        name="hand pull rope hangs ahead of the car face",
    )

    with ctx.pose({car_to_door: 0.075}):
        ctx.expect_contact(door, car, elem_a=left_hanger, elem_b=track_web)
        ctx.expect_contact(door, car, elem_a=right_hanger, elem_b=track_web)
        ctx.expect_gap(
            door,
            car,
            axis="x",
            min_gap=0.070,
            positive_elem=door_panel,
            negative_elem=left_post,
            name="door slides to the right to uncover the opening",
        )

    with ctx.pose({shaft_to_car: 0.34}):
        ctx.expect_contact(car, shaft, elem_a=left_lower_side_tab, elem_b=left_rail_wall)
        ctx.expect_contact(car, shaft, elem_a=left_lower_rear_tab, elem_b=left_rail_flange)
        ctx.expect_contact(car, shaft, elem_a=right_lower_side_tab, elem_b=right_rail_wall)
        ctx.expect_contact(car, shaft, elem_a=right_lower_rear_tab, elem_b=right_rail_flange)
        ctx.expect_gap(
            shaft,
            car,
            axis="z",
            min_gap=0.006,
            max_gap=0.025,
            positive_elem=pulley_wheel,
            negative_elem=roof,
            name="pulley stays just above the car roof at the top of travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
