from __future__ import annotations

import os

os.chdir("/")

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return _ORIG_GETCWD()


os.getcwd = _safe_getcwd

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


SHAFT_W = 2.20
SHAFT_D = 1.85
SHAFT_H = 4.80
WALL_T = 0.08

CAR_W = 1.42
CAR_D = 1.22
CAR_H = 2.02
PLATFORM_T = 0.10
CAR_ROOF_T = 0.04

CAR_RAIL_W = 0.10
CAR_RAIL_D = 0.05
CAR_RAIL_WEB = 0.028
CAR_RAIL_FLANGE = 0.012
CAR_RAIL_Z0 = 0.16
CAR_RAIL_H = 4.34

CW_RAIL_W = 0.08
CW_RAIL_D = 0.05
CW_RAIL_WEB = 0.022
CW_RAIL_FLANGE = 0.012
CW_RAIL_Z0 = 0.20
CW_RAIL_H = 4.20

CAR_RAIL_X = SHAFT_W / 2.0 - WALL_T - CAR_RAIL_W / 2.0
REAR_RAIL_Y = -SHAFT_D / 2.0 + WALL_T + CW_RAIL_D / 2.0

CAR_Y_IN_SHAFT = 0.18
CAR_Z0 = 0.55
CAR_TRAVEL = 1.39

COUNTERWEIGHT_Y = -0.655
COUNTERWEIGHT_Z0 = 2.65
COUNTERWEIGHT_TRAVEL = 1.39

DOOR_Y = CAR_D / 2.0 + 0.02
DOOR_W = 0.84
DOOR_H = 1.78
DOOR_TRAVEL = 0.28


def _add_i_rail(
    part,
    prefix: str,
    center_x: float,
    center_y: float,
    bottom_z: float,
    height: float,
    width_x: float,
    depth_y: float,
    web_x: float,
    flange_y: float,
    material,
) -> None:
    center_z = bottom_z + height / 2.0
    flange_offset = (depth_y - flange_y) / 2.0
    part.visual(
        Box((web_x, depth_y, height)),
        origin=Origin(xyz=(center_x, center_y, center_z)),
        material=material,
        name=f"{prefix}_web",
    )
    part.visual(
        Box((width_x, flange_y, height)),
        origin=Origin(xyz=(center_x, center_y + flange_offset, center_z)),
        material=material,
        name=f"{prefix}_front_flange",
    )
    part.visual(
        Box((width_x, flange_y, height)),
        origin=Origin(xyz=(center_x, center_y - flange_offset, center_z)),
        material=material,
        name=f"{prefix}_rear_flange",
    )


def _add_car_guide_shoe(
    part,
    prefix: str,
    side: float,
    z_center: float,
    material,
) -> None:
    arm_center_x = side * (CAR_W / 2.0 + 0.10)
    pad_center_x = side * (CAR_RAIL_X - 0.07)
    part.visual(
        Box((0.20, 0.08, 0.12)),
        origin=Origin(xyz=(arm_center_x, 0.0, z_center)),
        material=material,
        name=f"{prefix}_arm",
    )
    for suffix, y_center in (("front", 0.019), ("rear", -0.019)):
        part.visual(
            Box((0.04, 0.012, 0.12)),
            origin=Origin(xyz=(pad_center_x, y_center, z_center)),
            material=material,
            name=f"{prefix}_shoe_{suffix}",
        )


def _add_counterweight_shoe(
    part,
    prefix: str,
    x_center: float,
    z_center: float,
    material,
) -> None:
    part.visual(
        Box((0.06, 0.08, 0.10)),
        origin=Origin(xyz=(x_center, -0.09, z_center)),
        material=material,
        name=f"{prefix}_arm",
    )
    part.visual(
        Box((0.035, 0.04, 0.10)),
        origin=Origin(xyz=(x_center, -0.12, z_center)),
        material=material,
        name=f"{prefix}_shoe",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traction_elevator_shaft")

    concrete = model.material("concrete", rgba=(0.78, 0.79, 0.80, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.56, 0.59, 0.63, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.36, 0.38, 0.42, 1.0))
    cab_shell = model.material("cab_shell", rgba=(0.83, 0.84, 0.82, 1.0))
    door_metal = model.material("door_metal", rgba=(0.73, 0.75, 0.77, 1.0))
    counterweight_iron = model.material("counterweight_iron", rgba=(0.28, 0.30, 0.33, 1.0))
    machine_steel = model.material("machine_steel", rgba=(0.26, 0.27, 0.29, 1.0))

    shaft = model.part("shaft")
    shaft.visual(
        Box((WALL_T, SHAFT_D, SHAFT_H)),
        origin=Origin(xyz=(-SHAFT_W / 2.0 + WALL_T / 2.0, 0.0, SHAFT_H / 2.0)),
        material=concrete,
        name="left_wall",
    )
    shaft.visual(
        Box((WALL_T, SHAFT_D, SHAFT_H)),
        origin=Origin(xyz=(SHAFT_W / 2.0 - WALL_T / 2.0, 0.0, SHAFT_H / 2.0)),
        material=concrete,
        name="right_wall",
    )
    shaft.visual(
        Box((SHAFT_W - 2.0 * WALL_T, WALL_T, SHAFT_H)),
        origin=Origin(xyz=(0.0, -SHAFT_D / 2.0 + WALL_T / 2.0, SHAFT_H / 2.0)),
        material=concrete,
        name="rear_wall",
    )
    shaft.visual(
        Box((SHAFT_W - 2.0 * WALL_T, 0.14, 0.16)),
        origin=Origin(xyz=(0.0, SHAFT_D / 2.0 - 0.07, SHAFT_H - 0.08)),
        material=painted_steel,
        name="front_lintel",
    )
    shaft.visual(
        Box((SHAFT_W - 2.0 * WALL_T, 0.20, 0.14)),
        origin=Origin(xyz=(0.0, -0.16, SHAFT_H - 0.12)),
        material=painted_steel,
        name="machine_beam",
    )
    shaft.visual(
        Box((0.10, 0.18, 0.48)),
        origin=Origin(xyz=(-0.34, -0.16, SHAFT_H - 0.38)),
        material=machine_steel,
        name="sheave_support_left",
    )
    shaft.visual(
        Box((0.10, 0.18, 0.48)),
        origin=Origin(xyz=(0.34, -0.16, SHAFT_H - 0.38)),
        material=machine_steel,
        name="sheave_support_right",
    )
    shaft.visual(
        Cylinder(radius=0.035, length=0.74),
        origin=Origin(
            xyz=(0.0, -0.16, SHAFT_H - 0.32),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machine_steel,
        name="sheave_axle",
    )
    shaft.visual(
        Cylinder(radius=0.26, length=0.12),
        origin=Origin(
            xyz=(0.0, -0.16, SHAFT_H - 0.32),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machine_steel,
        name="sheave_pulley",
    )
    _add_i_rail(
        shaft,
        "left_car_rail",
        -CAR_RAIL_X,
        CAR_Y_IN_SHAFT,
        CAR_RAIL_Z0,
        CAR_RAIL_H,
        CAR_RAIL_W,
        CAR_RAIL_D,
        CAR_RAIL_WEB,
        CAR_RAIL_FLANGE,
        rail_steel,
    )
    _add_i_rail(
        shaft,
        "right_car_rail",
        CAR_RAIL_X,
        CAR_Y_IN_SHAFT,
        CAR_RAIL_Z0,
        CAR_RAIL_H,
        CAR_RAIL_W,
        CAR_RAIL_D,
        CAR_RAIL_WEB,
        CAR_RAIL_FLANGE,
        rail_steel,
    )
    _add_i_rail(
        shaft,
        "rear_left_counterweight_rail",
        -0.24,
        REAR_RAIL_Y,
        CW_RAIL_Z0,
        CW_RAIL_H,
        CW_RAIL_W,
        CW_RAIL_D,
        CW_RAIL_WEB,
        CW_RAIL_FLANGE,
        rail_steel,
    )
    _add_i_rail(
        shaft,
        "rear_right_counterweight_rail",
        0.24,
        REAR_RAIL_Y,
        CW_RAIL_Z0,
        CW_RAIL_H,
        CW_RAIL_W,
        CW_RAIL_D,
        CW_RAIL_WEB,
        CW_RAIL_FLANGE,
        rail_steel,
    )

    car = model.part("car")
    car.visual(
        Box((CAR_W, CAR_D, PLATFORM_T)),
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_T / 2.0)),
        material=painted_steel,
        name="platform",
    )
    car.visual(
        Box((0.035, CAR_D, CAR_H)),
        origin=Origin(xyz=(-CAR_W / 2.0 + 0.0175, 0.0, PLATFORM_T + CAR_H / 2.0)),
        material=cab_shell,
        name="left_wall_panel",
    )
    car.visual(
        Box((0.035, CAR_D, CAR_H)),
        origin=Origin(xyz=(CAR_W / 2.0 - 0.0175, 0.0, PLATFORM_T + CAR_H / 2.0)),
        material=cab_shell,
        name="right_wall_panel",
    )
    car.visual(
        Box((CAR_W - 0.07, 0.035, CAR_H)),
        origin=Origin(xyz=(0.0, -CAR_D / 2.0 + 0.0175, PLATFORM_T + CAR_H / 2.0)),
        material=cab_shell,
        name="rear_wall_panel",
    )
    car.visual(
        Box((CAR_W, CAR_D, CAR_ROOF_T)),
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_T + CAR_H + CAR_ROOF_T / 2.0)),
        material=cab_shell,
        name="roof",
    )
    car.visual(
        Box((CAR_W - 0.18, 0.10, 0.18)),
        origin=Origin(xyz=(0.0, CAR_D / 2.0 - 0.05, PLATFORM_T + CAR_H - 0.07)),
        material=painted_steel,
        name="door_header",
    )
    car.visual(
        Box((1.06, 0.022, 0.06)),
        origin=Origin(xyz=(0.0, CAR_D / 2.0 - 0.035, PLATFORM_T + 1.84)),
        material=painted_steel,
        name="door_track_back",
    )
    car.visual(
        Box((1.06, 0.065, 0.028)),
        origin=Origin(xyz=(0.0, CAR_D / 2.0 - 0.002, PLATFORM_T + 1.864)),
        material=painted_steel,
        name="door_track_top",
    )
    car.visual(
        Box((1.06, 0.012, 0.10)),
        origin=Origin(xyz=(0.0, CAR_D / 2.0 - 0.004, PLATFORM_T + 1.82)),
        material=painted_steel,
        name="door_track_front",
    )
    _add_car_guide_shoe(car, "left_lower", -1.0, 0.28, rail_steel)
    _add_car_guide_shoe(car, "right_lower", 1.0, 0.28, rail_steel)
    _add_car_guide_shoe(car, "left_upper", -1.0, 1.92, rail_steel)
    _add_car_guide_shoe(car, "right_upper", 1.0, 1.92, rail_steel)

    door = model.part("door")
    door.visual(
        Box((DOOR_W, 0.035, DOOR_H)),
        origin=Origin(xyz=(0.0, 0.0, DOOR_H / 2.0)),
        material=door_metal,
        name="door_panel",
    )
    door.visual(
        Box((0.09, 0.018, 0.11)),
        origin=Origin(xyz=(-0.25, -0.008, 1.795)),
        material=machine_steel,
        name="left_hanger",
    )
    door.visual(
        Box((0.09, 0.018, 0.11)),
        origin=Origin(xyz=(0.25, -0.008, 1.795)),
        material=machine_steel,
        name="right_hanger",
    )

    counterweight = model.part("counterweight")
    counterweight.visual(
        Box((0.05, 0.16, 1.50)),
        origin=Origin(xyz=(-0.205, 0.0, 0.75)),
        material=counterweight_iron,
        name="left_stile",
    )
    counterweight.visual(
        Box((0.05, 0.16, 1.50)),
        origin=Origin(xyz=(0.205, 0.0, 0.75)),
        material=counterweight_iron,
        name="right_stile",
    )
    counterweight.visual(
        Box((0.46, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=counterweight_iron,
        name="bottom_crosshead",
    )
    counterweight.visual(
        Box((0.46, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.46)),
        material=counterweight_iron,
        name="top_crosshead",
    )
    for index, z_center in enumerate((0.28, 0.54, 0.80, 1.06), start=1):
        counterweight.visual(
            Box((0.36, 0.13, 0.18)),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=machine_steel,
            name=f"weight_slab_{index}",
        )
    _add_counterweight_shoe(counterweight, "left_lower", -0.24, 0.24, rail_steel)
    _add_counterweight_shoe(counterweight, "right_lower", 0.24, 0.24, rail_steel)
    _add_counterweight_shoe(counterweight, "left_upper", -0.24, 1.28, rail_steel)
    _add_counterweight_shoe(counterweight, "right_upper", 0.24, 1.28, rail_steel)

    model.articulation(
        "shaft_to_car",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=car,
        origin=Origin(xyz=(0.0, CAR_Y_IN_SHAFT, CAR_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8000.0,
            velocity=1.5,
            lower=0.0,
            upper=CAR_TRAVEL,
        ),
    )
    model.articulation(
        "car_to_door",
        ArticulationType.PRISMATIC,
        parent=car,
        child=door,
        origin=Origin(xyz=(0.0, DOOR_Y, PLATFORM_T)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=1.0,
            lower=0.0,
            upper=DOOR_TRAVEL,
        ),
    )
    model.articulation(
        "shaft_to_counterweight",
        ArticulationType.PRISMATIC,
        parent=shaft,
        child=counterweight,
        origin=Origin(xyz=(0.0, COUNTERWEIGHT_Y, COUNTERWEIGHT_Z0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8000.0,
            velocity=1.5,
            lower=0.0,
            upper=COUNTERWEIGHT_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shaft = object_model.get_part("shaft")
    car = object_model.get_part("car")
    door = object_model.get_part("door")
    counterweight = object_model.get_part("counterweight")
    shaft_to_car = object_model.get_articulation("shaft_to_car")
    car_to_door = object_model.get_articulation("car_to_door")
    shaft_to_counterweight = object_model.get_articulation("shaft_to_counterweight")

    left_car_rail_front = shaft.get_visual("left_car_rail_front_flange")
    right_car_rail_front = shaft.get_visual("right_car_rail_front_flange")
    rear_left_cw_rail_front = shaft.get_visual("rear_left_counterweight_rail_front_flange")
    rear_right_cw_rail_front = shaft.get_visual("rear_right_counterweight_rail_front_flange")
    sheave_pulley = shaft.get_visual("sheave_pulley")

    left_lower_shoe_front = car.get_visual("left_lower_shoe_front")
    right_lower_shoe_front = car.get_visual("right_lower_shoe_front")
    left_upper_shoe_front = car.get_visual("left_upper_shoe_front")
    right_upper_shoe_front = car.get_visual("right_upper_shoe_front")
    car_roof = car.get_visual("roof")
    rear_wall_panel = car.get_visual("rear_wall_panel")
    right_wall_panel = car.get_visual("right_wall_panel")
    door_track_top = car.get_visual("door_track_top")
    door_track_front = car.get_visual("door_track_front")

    door_panel = door.get_visual("door_panel")
    left_hanger = door.get_visual("left_hanger")
    right_hanger = door.get_visual("right_hanger")

    cw_left_lower_shoe = counterweight.get_visual("left_lower_shoe")
    cw_right_lower_shoe = counterweight.get_visual("right_lower_shoe")
    cw_left_upper_shoe = counterweight.get_visual("left_upper_shoe")
    cw_right_upper_shoe = counterweight.get_visual("right_upper_shoe")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Large prismatic origins live in open shaft volume rather than on compact hinge seats,
    # so the default fixed-distance origin warning is not meaningful here.
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(car, shaft, elem_a=left_lower_shoe_front, elem_b=left_car_rail_front)
    ctx.expect_contact(car, shaft, elem_a=right_lower_shoe_front, elem_b=right_car_rail_front)
    ctx.expect_contact(counterweight, shaft, elem_a=cw_left_lower_shoe, elem_b=rear_left_cw_rail_front)
    ctx.expect_contact(counterweight, shaft, elem_a=cw_right_lower_shoe, elem_b=rear_right_cw_rail_front)
    ctx.expect_gap(car, counterweight, axis="y", min_gap=0.12)
    ctx.expect_gap(
        car,
        shaft,
        axis="y",
        min_gap=0.30,
        positive_elem=rear_wall_panel,
        negative_elem=rear_left_cw_rail_front,
    )
    ctx.expect_gap(
        car,
        shaft,
        axis="y",
        min_gap=0.30,
        positive_elem=rear_wall_panel,
        negative_elem=rear_right_cw_rail_front,
    )
    ctx.expect_contact(door, car, elem_a=left_hanger, elem_b=door_track_top)
    ctx.expect_contact(door, car, elem_a=right_hanger, elem_b=door_track_top)
    ctx.expect_gap(
        door,
        car,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=door_panel,
        negative_elem=door_track_front,
    )

    with ctx.pose({car_to_door: DOOR_TRAVEL}):
        ctx.expect_contact(door, car, elem_a=left_hanger, elem_b=door_track_top)
        ctx.expect_contact(door, car, elem_a=right_hanger, elem_b=door_track_top)
        ctx.expect_gap(
            door,
            car,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem=door_panel,
            negative_elem=door_track_front,
        )
        ctx.expect_overlap(door, car, axes="xz", min_overlap=0.02, elem_a=door_panel, elem_b=right_wall_panel)

    with ctx.pose({shaft_to_car: CAR_TRAVEL, shaft_to_counterweight: COUNTERWEIGHT_TRAVEL}):
        ctx.expect_contact(car, shaft, elem_a=left_upper_shoe_front, elem_b=left_car_rail_front)
        ctx.expect_contact(car, shaft, elem_a=right_upper_shoe_front, elem_b=right_car_rail_front)
        ctx.expect_contact(counterweight, shaft, elem_a=cw_left_upper_shoe, elem_b=rear_left_cw_rail_front)
        ctx.expect_contact(counterweight, shaft, elem_a=cw_right_upper_shoe, elem_b=rear_right_cw_rail_front)
        ctx.expect_gap(
            shaft,
            car,
            axis="z",
            min_gap=0.12,
            positive_elem=sheave_pulley,
            negative_elem=car_roof,
        )
        ctx.expect_overlap(
            shaft,
            car,
            axes="xy",
            min_overlap=0.10,
            elem_a=sheave_pulley,
            elem_b=car_roof,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
