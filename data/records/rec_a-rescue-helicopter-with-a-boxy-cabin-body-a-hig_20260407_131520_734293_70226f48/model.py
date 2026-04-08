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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_wheel_assembly(part, *, x: float, y: float, wheel_radius: float, gear_material, wheel_material) -> None:
    wheel_width = 0.12
    part.visual(
        Box((0.12, 0.10, 0.10)),
        origin=Origin(xyz=(x, y, 0.51)),
        material=gear_material,
        name=f"gear_strut_{'r' if y > 0 else 'l'}_{'f' if x > 0 else 'a'}",
    )
    part.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(xyz=(x, y, wheel_radius), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_material,
        name=f"wheel_{'r' if y > 0 else 'l'}_{'f' if x > 0 else 'a'}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rescue_helicopter")

    rescue_white = model.material("rescue_white", rgba=(0.92, 0.94, 0.95, 1.0))
    rescue_red = model.material("rescue_red", rgba=(0.80, 0.14, 0.12, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.20, 0.22, 0.24, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.72, 1.0))
    glass = model.material("glass", rgba=(0.55, 0.74, 0.86, 0.35))

    body = model.part("body")

    floor_z = 0.68
    cabin_len = 3.40
    cabin_half_w = 0.95
    cabin_roof_z = 2.43
    side_wall_center_z = 1.555

    body.visual(
        Box((cabin_len, 1.90, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, floor_z + 0.06)),
        material=dark_grey,
        name="cabin_floor",
    )
    body.visual(
        Box((cabin_len, 1.90, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, cabin_roof_z - 0.04)),
        material=rescue_white,
        name="cabin_roof",
    )
    body.visual(
        Box((cabin_len, 0.08, 1.59)),
        origin=Origin(xyz=(0.0, -0.91, side_wall_center_z)),
        material=rescue_white,
        name="left_side_wall",
    )
    body.visual(
        Box((0.76, 0.08, 1.59)),
        origin=Origin(xyz=(1.32, 0.91, side_wall_center_z)),
        material=rescue_white,
        name="right_front_wall",
    )
    body.visual(
        Box((1.36, 0.08, 1.59)),
        origin=Origin(xyz=(-1.02, 0.91, side_wall_center_z)),
        material=rescue_white,
        name="right_rear_wall",
    )
    body.visual(
        Box((1.28, 0.08, 0.28)),
        origin=Origin(xyz=(0.30, 0.91, 0.82)),
        material=rescue_white,
        name="door_sill",
    )
    body.visual(
        Box((1.28, 0.08, 0.34)),
        origin=Origin(xyz=(0.30, 0.91, 2.18)),
        material=rescue_white,
        name="door_header",
    )
    body.visual(
        Box((0.08, 1.74, 1.59)),
        origin=Origin(xyz=(1.66, 0.0, side_wall_center_z)),
        material=rescue_white,
        name="front_bulkhead",
    )
    body.visual(
        Box((0.08, 1.74, 1.59)),
        origin=Origin(xyz=(-1.66, 0.0, side_wall_center_z)),
        material=rescue_white,
        name="rear_bulkhead",
    )

    body.visual(
        Box((1.10, 1.55, 0.78)),
        origin=Origin(xyz=(2.08, 0.0, 1.08)),
        material=rescue_white,
        name="nose_lower",
    )
    body.visual(
        Box((0.92, 1.38, 0.62)),
        origin=Origin(xyz=(2.15, 0.0, 1.60)),
        material=rescue_white,
        name="nose_upper",
    )
    body.visual(
        Box((0.55, 1.12, 0.44)),
        origin=Origin(xyz=(2.70, 0.0, 1.48)),
        material=rescue_red,
        name="nose_tip",
    )
    body.visual(
        Box((0.64, 1.26, 0.16)),
        origin=Origin(xyz=(1.15, 0.0, 0.72)),
        material=dark_grey,
        name="belly_fairing",
    )

    body.visual(
        Box((0.18, 1.20, 0.55)),
        origin=Origin(xyz=(2.72, 0.0, 1.62)),
        material=glass,
        name="front_windshield",
    )
    body.visual(
        Box((0.50, 0.04, 0.48)),
        origin=Origin(xyz=(2.18, -0.69, 1.64)),
        material=glass,
        name="left_cockpit_window",
    )
    body.visual(
        Box((0.50, 0.04, 0.48)),
        origin=Origin(xyz=(2.18, 0.69, 1.64)),
        material=glass,
        name="right_cockpit_window",
    )
    body.visual(
        Box((0.72, 0.04, 0.46)),
        origin=Origin(xyz=(0.08, -0.90, 1.67)),
        material=glass,
        name="left_cabin_window",
    )
    body.visual(
        Box((0.42, 0.04, 0.42)),
        origin=Origin(xyz=(-1.06, 0.90, 1.64)),
        material=glass,
        name="right_rear_window",
    )

    body.visual(
        Box((2.10, 0.24, 0.30)),
        origin=Origin(xyz=(-0.05, -1.07, 0.69)),
        material=rescue_red,
        name="left_sponson",
    )
    body.visual(
        Box((2.10, 0.24, 0.30)),
        origin=Origin(xyz=(-0.05, 1.07, 0.69)),
        material=rescue_red,
        name="right_sponson",
    )
    for gear_x in (0.95, -1.05):
        _add_wheel_assembly(
            body,
            x=gear_x,
            y=-1.07,
            wheel_radius=0.24,
            gear_material=steel,
            wheel_material=black_rubber,
        )
        _add_wheel_assembly(
            body,
            x=gear_x,
            y=1.07,
            wheel_radius=0.24,
            gear_material=steel,
            wheel_material=black_rubber,
        )

    body.visual(
        Box((0.90, 0.60, 0.32)),
        origin=Origin(xyz=(-0.12, 0.0, 2.56)),
        material=rescue_red,
        name="mast_pylon",
    )
    body.visual(
        Cylinder(radius=0.16, length=0.36),
        origin=Origin(xyz=(-0.10, 0.0, 2.87)),
        material=steel,
        name="mast_support",
    )

    body.visual(
        Cylinder(radius=0.19, length=4.35),
        origin=Origin(xyz=(-3.82, 0.0, 1.76), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rescue_white,
        name="tail_boom",
    )
    body.visual(
        Box((0.55, 1.10, 0.06)),
        origin=Origin(xyz=(-5.05, 0.0, 1.58)),
        material=rescue_white,
        name="horizontal_stabilizer",
    )
    body.visual(
        Box((0.30, 0.20, 1.40)),
        origin=Origin(xyz=(-5.86, 0.0, 2.22)),
        material=rescue_red,
        name="vertical_fin",
    )
    body.visual(
        Box((0.18, 0.16, 0.16)),
        origin=Origin(xyz=(-6.08, 0.0, 2.55)),
        material=steel,
        name="tail_rotor_mount",
    )

    body.visual(
        Box((2.65, 0.08, 0.08)),
        origin=Origin(xyz=(-0.35, 1.11, 2.13)),
        material=steel,
        name="door_upper_rail",
    )
    for bracket_x in (-1.20, -0.30, 0.60):
        body.visual(
            Box((0.10, 0.16, 0.08)),
            origin=Origin(xyz=(bracket_x, 1.03, 2.13)),
            material=steel,
            name=f"door_rail_bracket_{str(bracket_x).replace('-', 'm').replace('.', '_')}",
        )
    body.visual(
        Box((1.60, 0.10, 0.06)),
        origin=Origin(xyz=(-0.45, 0.99, 1.00)),
        material=steel,
        name="door_lower_guide",
    )

    body.inertial = Inertial.from_geometry(
        Box((9.20, 2.30, 3.20)),
        mass=3200.0,
        origin=Origin(xyz=(-1.62, 0.0, 1.60)),
    )

    side_door = model.part("side_door")
    side_door.visual(
        Box((1.28, 0.05, 1.08)),
        material=rescue_white,
        name="door_panel",
    )
    side_door.visual(
        Box((1.04, 0.04, 0.06)),
        origin=Origin(xyz=(-0.02, 0.0, 0.57)),
        material=steel,
        name="door_slider_carriage",
    )
    side_door.visual(
        Box((0.60, 0.02, 0.42)),
        origin=Origin(xyz=(0.12, 0.02, 0.18)),
        material=glass,
        name="door_window",
    )
    side_door.visual(
        Box((0.22, 0.06, 0.08)),
        origin=Origin(xyz=(-0.25, 0.0, -0.50)),
        material=steel,
        name="door_lower_shoe",
    )
    side_door.inertial = Inertial.from_geometry(Box((1.28, 0.08, 1.12)), mass=95.0)

    main_rotor = model.part("main_rotor")
    main_rotor.visual(
        Cylinder(radius=0.18, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_grey,
        name="main_hub",
    )
    main_rotor.visual(
        Box((8.80, 0.24, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=dark_grey,
        name="main_blade_longitudinal",
    )
    main_rotor.visual(
        Box((0.24, 8.80, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=dark_grey,
        name="main_blade_lateral",
    )
    main_rotor.inertial = Inertial.from_geometry(
        Box((8.80, 8.80, 0.18)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    tail_rotor = model.part("tail_rotor")
    tail_rotor.visual(
        Cylinder(radius=0.03, length=0.44),
        origin=Origin(xyz=(0.22, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tail_drive_shaft",
    )
    tail_rotor.visual(
        Cylinder(radius=0.07, length=0.14),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="tail_hub",
    )
    tail_rotor.visual(
        Box((0.36, 0.03, 0.08)),
        origin=Origin(xyz=(-0.15, 0.0, 0.0)),
        material=dark_grey,
        name="tail_blade_left",
    )
    tail_rotor.visual(
        Box((0.36, 0.03, 0.08)),
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        material=dark_grey,
        name="tail_blade_right",
    )
    tail_rotor.visual(
        Box((0.08, 0.03, 0.36)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_grey,
        name="tail_blade_top",
    )
    tail_rotor.visual(
        Box((0.08, 0.03, 0.36)),
        origin=Origin(xyz=(0.0, 0.0, -0.15)),
        material=dark_grey,
        name="tail_blade_bottom",
    )
    tail_rotor.inertial = Inertial.from_geometry(Box((0.80, 0.16, 0.80)), mass=28.0)

    model.articulation(
        "main_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=main_rotor,
        origin=Origin(xyz=(-0.10, 0.0, 3.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=400.0, velocity=35.0),
    )
    model.articulation(
        "tail_rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=tail_rotor,
        origin=Origin(xyz=(-6.61, 0.0, 2.55)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=65.0),
    )
    model.articulation(
        "side_door_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=side_door,
        origin=Origin(xyz=(0.30, 1.07, 1.48)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.8, lower=0.0, upper=1.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    side_door = object_model.get_part("side_door")
    main_rotor = object_model.get_part("main_rotor")
    tail_rotor = object_model.get_part("tail_rotor")

    door_slide = object_model.get_articulation("side_door_slide")
    main_spin = object_model.get_articulation("main_rotor_spin")
    tail_spin = object_model.get_articulation("tail_rotor_spin")

    ctx.check(
        "main rotor spins on vertical mast axis",
        main_spin.joint_type == ArticulationType.CONTINUOUS and tuple(main_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={main_spin.joint_type}, axis={main_spin.axis}",
    )
    ctx.check(
        "tail rotor spins on lateral axis",
        tail_spin.joint_type == ArticulationType.CONTINUOUS and tuple(tail_spin.axis) == (0.0, 1.0, 0.0),
        details=f"type={tail_spin.joint_type}, axis={tail_spin.axis}",
    )
    ctx.check(
        "cabin door slides aft along rail",
        door_slide.joint_type == ArticulationType.PRISMATIC and tuple(door_slide.axis) == (-1.0, 0.0, 0.0),
        details=f"type={door_slide.joint_type}, axis={door_slide.axis}",
    )

    ctx.expect_gap(
        side_door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="door_sill",
        min_gap=0.07,
        max_gap=0.12,
        name="closed door sits slightly proud of fuselage side",
    )
    ctx.expect_gap(
        main_rotor,
        body,
        axis="z",
        positive_elem="main_blade_longitudinal",
        negative_elem="mast_support",
        min_gap=0.05,
        max_gap=0.20,
        name="main rotor blades clear mast support",
    )
    ctx.expect_gap(
        body,
        tail_rotor,
        axis="x",
        positive_elem="vertical_fin",
        negative_elem="tail_blade_right",
        min_gap=0.10,
        max_gap=0.35,
        name="tail rotor disk stays aft of the fin trailing face",
    )

    door_closed = ctx.part_world_position(side_door)
    with ctx.pose({door_slide: 1.12}):
        ctx.expect_gap(
            side_door,
            body,
            axis="y",
            positive_elem="door_panel",
            negative_elem="door_sill",
            min_gap=0.07,
            max_gap=0.12,
            name="open door remains on the outside track",
        )
        door_open = ctx.part_world_position(side_door)

    ctx.check(
        "door translates rearward when opened",
        door_closed is not None and door_open is not None and door_open[0] < door_closed[0] - 0.90,
        details=f"closed={door_closed}, open={door_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
