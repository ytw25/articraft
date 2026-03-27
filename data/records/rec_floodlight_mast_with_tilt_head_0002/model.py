from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

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

HERE = Path(__file__).resolve().parent


def _tube_mesh(name: str, *, width: float, depth: float, wall: float, length: float):
    _ = (name, wall)
    return Box((width, depth, length))


def _ring_mesh(name: str, *, outer: float, band: float, height: float):
    _ = (name, band)
    return Box((outer, outer, height))


def _add_clamp_ring(
    part,
    *,
    ring_mesh,
    z0: float,
    ring_height: float,
    ring_outer: float,
    steel,
    ring_name: str = "clamp_ring",
) -> None:
    center_z = z0 + ring_height * 0.5
    latch_y = ring_outer * 0.5 + 0.007
    part.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, center_z)),
        material=steel,
        name=ring_name,
    )
    part.visual(
        Box((0.032, 0.014, ring_height * 0.9)),
        origin=Origin(xyz=(0.0, latch_y, center_z)),
        material=steel,
        name="clamp_latch",
    )
    part.visual(
        Box((0.050, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, latch_y + 0.010, center_z)),
        material=steel,
    )
    part.visual(
        Box((0.010, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, latch_y + 0.010, center_z)),
        material=steel,
    )


def _add_light_yoke(
    part,
    *,
    side: str,
    x_center: float,
    pin_z: float,
    yoke_gap: float,
    yoke_depth: float,
    yoke_height: float,
    steel,
) -> None:
    cheek_thickness = 0.012
    cheek_offset = yoke_gap * 0.5 + cheek_thickness * 0.5
    part.visual(
        Box((yoke_gap + 0.070, 0.040, 0.036)),
        origin=Origin(xyz=(x_center, 0.0, pin_z + yoke_height * 0.44)),
        material=steel,
        name=f"{side}_yoke_bridge",
    )
    for cheek_side, sign in (("left", -1.0), ("right", 1.0)):
        cheek_x = x_center + sign * cheek_offset
        part.visual(
            Box((cheek_thickness, yoke_depth, yoke_height)),
            origin=Origin(xyz=(cheek_x, 0.0, pin_z - 0.020)),
            material=steel,
            name=f"{side}_yoke_{cheek_side}_cheek",
        )
        part.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(
                xyz=(cheek_x + sign * 0.009, 0.0, pin_z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=steel,
            name=f"{side}_pin_{cheek_side}",
        )
        wing_x = cheek_x + sign * 0.009
        part.visual(
            Box((0.010, 0.030, 0.006)),
            origin=Origin(xyz=(wing_x, 0.0, pin_z)),
            material=steel,
            name=f"{side}_wingnut_{cheek_side}",
        )
        part.visual(
            Box((0.010, 0.006, 0.030)),
            origin=Origin(xyz=(wing_x, 0.0, pin_z)),
            material=steel,
        )


def _add_outrigger_part(
    part,
    *,
    axis: str,
    sign: float,
    arm_length: float,
    arm_width: float,
    arm_height: float,
    inserted_length: float,
    steel,
) -> None:
    _ = inserted_length
    center = sign * arm_length * 0.5
    xyz = {"x": (center, 0.0, 0.0), "y": (0.0, center, 0.0)}[axis]
    size = {"x": (arm_length, arm_width, arm_height), "y": (arm_width, arm_length, arm_height)}[axis]
    part.visual(Box(size), origin=Origin(xyz=xyz), material=steel, name="arm_tube")
    foot_offset = sign * (arm_length - 0.030)
    if axis == "x":
        foot_xyz = (foot_offset, 0.0, -0.030)
        jack_xyz = (foot_offset, 0.0, -0.005)
    else:
        foot_xyz = (0.0, foot_offset, -0.030)
        jack_xyz = (0.0, foot_offset, -0.005)
    part.visual(Box((0.060, 0.060, 0.014)), origin=Origin(xyz=foot_xyz), material=steel, name="foot_pad")
    part.visual(Box((0.030, 0.030, 0.052)), origin=Origin(xyz=jack_xyz), material=steel, name="jack_post")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_field_mast")

    trailer_steel = model.material("trailer_steel", rgba=(0.28, 0.31, 0.34, 1.0))
    galvanized = model.material("galvanized", rgba=(0.68, 0.70, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    light_black = model.material("light_black", rgba=(0.18, 0.19, 0.20, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.86, 0.90, 0.94, 0.38))
    reflector = model.material("reflector", rgba=(0.88, 0.89, 0.90, 1.0))

    mast_1_mesh = _tube_mesh("mast_section_1.obj", width=0.140, depth=0.140, wall=0.008, length=1.050)
    mast_2_mesh = _tube_mesh("mast_section_2.obj", width=0.112, depth=0.112, wall=0.007, length=0.920)
    mast_3_mesh = _tube_mesh("mast_section_3.obj", width=0.088, depth=0.088, wall=0.006, length=0.800)
    mast_4_mesh = _tube_mesh("mast_section_4.obj", width=0.066, depth=0.066, wall=0.005, length=0.680)
    mast_1_ring = _ring_mesh("mast_section_1_ring.obj", outer=0.162, band=0.006, height=0.038)
    mast_2_ring = _ring_mesh("mast_section_2_ring.obj", outer=0.134, band=0.006, height=0.036)
    mast_3_ring = _ring_mesh("mast_section_3_ring.obj", outer=0.108, band=0.005, height=0.032)
    mast_4_ring = _ring_mesh("mast_section_4_ring.obj", outer=0.084, band=0.0045, height=0.028)
    top_collar_ring = _ring_mesh("top_collar_ring.obj", outer=0.110, band=0.008, height=0.090)
    mast_socket_mesh = _tube_mesh("mast_socket.obj", width=0.168, depth=0.168, wall=0.008, length=0.120)
    side_sleeve_mesh = _tube_mesh("side_sleeve.obj", width=0.070, depth=0.094, wall=0.006, length=0.180)
    front_sleeve_mesh = _tube_mesh("front_sleeve.obj", width=0.094, depth=0.070, wall=0.006, length=0.180)

    chassis = model.part("chassis")
    chassis.visual(Box((0.760, 0.420, 0.090)), origin=Origin(xyz=(0.0, 0.0, 0.085)), material=trailer_steel, name="chassis_body")
    chassis.visual(Box((0.620, 0.280, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.155)), material=dark_steel, name="deck_box")
    chassis.visual(Box((0.240, 0.220, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.220)), material=dark_steel, name="mast_plinth")
    chassis.visual(mast_socket_mesh, origin=Origin(xyz=(0.0, 0.0, 0.220)), material=galvanized, name="mast_socket")
    chassis.visual(Box((0.160, 0.260, 0.180)), origin=Origin(xyz=(-0.180, 0.0, 0.225)), material=trailer_steel, name="equipment_box")
    chassis.visual(Box((0.190, 0.030, 0.030)), origin=Origin(xyz=(0.360, -0.070, 0.090)), material=trailer_steel)
    chassis.visual(Box((0.190, 0.030, 0.030)), origin=Origin(xyz=(0.360, 0.070, 0.090)), material=trailer_steel)
    chassis.visual(Box((0.180, 0.120, 0.020)), origin=Origin(xyz=(0.470, 0.0, 0.145)), material=trailer_steel, name="tow_bar")
    chassis.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.565, 0.0, 0.170), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=galvanized,
        name="tow_eye",
    )
    for wheel_x in (-0.270, 0.270):
        for wheel_y in (-0.145, 0.145):
            hub_name = f"hub_{'r' if wheel_x > 0.0 else 'l'}_{'f' if wheel_y > 0.0 else 'r'}"
            chassis.visual(Box((0.030, 0.090, 0.050)), origin=Origin(xyz=(wheel_x * 0.92, wheel_y, 0.062)), material=dark_steel, name=hub_name)
            chassis.visual(
                Cylinder(radius=0.105, length=0.050),
                origin=Origin(xyz=(wheel_x, wheel_y, 0.080), rpy=(0.0, math.pi * 0.5, 0.0)),
                material=rubber,
                name=f"wheel_{hub_name}",
            )
    chassis.visual(
        side_sleeve_mesh,
        origin=Origin(xyz=(0.335, 0.0, 0.105), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=galvanized,
        name="right_sleeve",
    )
    chassis.visual(
        side_sleeve_mesh,
        origin=Origin(xyz=(-0.335, 0.0, 0.105), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=galvanized,
        name="left_sleeve",
    )
    chassis.visual(
        front_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.235, 0.105), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=galvanized,
        name="front_sleeve",
    )
    chassis.visual(
        front_sleeve_mesh,
        origin=Origin(xyz=(0.0, -0.235, 0.105), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=galvanized,
        name="rear_sleeve",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((1.000, 0.520, 0.420)),
        mass=155.0,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )

    mast_1 = model.part("mast_section_1")
    mast_1.visual(mast_1_mesh, origin=Origin(xyz=(0.0, 0.0, 0.525)), material=galvanized, name="mast_tube")
    _add_clamp_ring(mast_1, ring_mesh=mast_1_ring, z0=0.785, ring_height=0.038, ring_outer=0.162, steel=dark_steel)
    mast_1.inertial = Inertial.from_geometry(Box((0.160, 0.160, 1.050)), mass=22.0, origin=Origin(xyz=(0.0, 0.0, 0.525)))

    mast_2 = model.part("mast_section_2")
    mast_2.visual(mast_2_mesh, origin=Origin(xyz=(0.0, 0.0, 0.460)), material=galvanized, name="mast_tube")
    _add_clamp_ring(mast_2, ring_mesh=mast_2_ring, z0=0.665, ring_height=0.036, ring_outer=0.134, steel=dark_steel)
    mast_2.inertial = Inertial.from_geometry(Box((0.130, 0.130, 0.920)), mass=15.0, origin=Origin(xyz=(0.0, 0.0, 0.460)))

    mast_3 = model.part("mast_section_3")
    mast_3.visual(mast_3_mesh, origin=Origin(xyz=(0.0, 0.0, 0.400)), material=galvanized, name="mast_tube")
    _add_clamp_ring(mast_3, ring_mesh=mast_3_ring, z0=0.565, ring_height=0.032, ring_outer=0.108, steel=dark_steel)
    mast_3.inertial = Inertial.from_geometry(Box((0.104, 0.104, 0.800)), mass=11.5, origin=Origin(xyz=(0.0, 0.0, 0.400)))

    mast_4 = model.part("mast_section_4")
    mast_4.visual(mast_4_mesh, origin=Origin(xyz=(0.0, 0.0, 0.340)), material=galvanized, name="mast_tube")
    _add_clamp_ring(mast_4, ring_mesh=mast_4_ring, z0=0.460, ring_height=0.028, ring_outer=0.084, steel=dark_steel)
    mast_4.visual(top_collar_ring, origin=Origin(xyz=(0.0, 0.0, 0.600)), material=dark_steel, name="top_collar")
    mast_4.visual(Box((0.068, 0.048, 0.185)), origin=Origin(xyz=(0.0, 0.0, 0.675)), material=dark_steel, name="tee_plate")
    mast_4.visual(Box((0.840, 0.060, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.748)), material=dark_steel, name="crossbar_beam")
    mast_4.visual(Box((0.162, 0.058, 0.050)), origin=Origin(xyz=(0.0, 0.0, 0.706)), material=dark_steel, name="t_clamp_body")
    mast_4.visual(Box((0.126, 0.024, 0.042)), origin=Origin(xyz=(0.0, 0.034, 0.694)), material=dark_steel, name="t_clamp_jaw")
    _add_light_yoke(mast_4, side="left", x_center=-0.235, pin_z=0.642, yoke_gap=0.194, yoke_depth=0.040, yoke_height=0.160, steel=dark_steel)
    _add_light_yoke(mast_4, side="right", x_center=0.235, pin_z=0.642, yoke_gap=0.194, yoke_depth=0.040, yoke_height=0.160, steel=dark_steel)
    mast_4.inertial = Inertial.from_geometry(Box((0.900, 0.140, 0.820)), mass=18.0, origin=Origin(xyz=(0.0, 0.0, 0.410)))

    for name, axis, sign in (
        ("left_outrigger", "x", -1.0),
        ("right_outrigger", "x", 1.0),
        ("front_outrigger", "y", 1.0),
        ("rear_outrigger", "y", -1.0),
    ):
        part = model.part(name)
        _add_outrigger_part(
            part,
            axis=axis,
            sign=sign,
            arm_length=0.320,
            arm_width=0.060,
            arm_height=0.046,
            inserted_length=0.090,
            steel=galvanized,
        )
        part.inertial = Inertial.from_geometry(Box((0.380, 0.100, 0.120)), mass=8.0, origin=Origin(xyz=(0.0, 0.0, -0.005)))

    def _add_floodlight(part) -> None:
        part.visual(Box((0.188, 0.102, 0.116)), origin=Origin(xyz=(0.0, 0.035, -0.012)), material=light_black, name="housing")
        part.visual(Box((0.172, 0.012, 0.098)), origin=Origin(xyz=(0.0, 0.084, -0.012)), material=reflector, name="reflector_frame")
        part.visual(Box((0.166, 0.004, 0.090)), origin=Origin(xyz=(0.0, 0.092, -0.012)), material=lens_glass, name="lens")
        part.visual(Box((0.120, 0.036, 0.072)), origin=Origin(xyz=(0.0, -0.030, -0.012)), material=dark_steel, name="rear_heat_sink")
        part.visual(Box((0.188, 0.040, 0.018)), origin=Origin(xyz=(0.0, 0.055, 0.046)), material=light_black, name="hood")
        part.visual(
            Cylinder(radius=0.009, length=0.020),
            origin=Origin(xyz=(-0.100, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=galvanized,
            name="left_trunnion",
        )
        part.visual(
            Cylinder(radius=0.009, length=0.020),
            origin=Origin(xyz=(0.100, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
            material=galvanized,
            name="right_trunnion",
        )

    left_light = model.part("left_floodlight")
    _add_floodlight(left_light)
    left_light.inertial = Inertial.from_geometry(Box((0.210, 0.140, 0.140)), mass=7.0, origin=Origin(xyz=(0.0, 0.020, -0.005)))

    right_light = model.part("right_floodlight")
    _add_floodlight(right_light)
    right_light.inertial = Inertial.from_geometry(Box((0.210, 0.140, 0.140)), mass=7.0, origin=Origin(xyz=(0.0, 0.020, -0.005)))

    model.articulation("chassis_to_mast_section_1", ArticulationType.FIXED, parent=chassis, child=mast_1, origin=Origin(xyz=(0.0, 0.0, 0.250)))
    model.articulation(
        "mast_section_1_to_2",
        ArticulationType.PRISMATIC,
        parent=mast_1,
        child=mast_2,
        origin=Origin(xyz=(0.0, 0.0, 0.490)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.35, lower=0.0, upper=0.420),
    )
    model.articulation(
        "mast_section_2_to_3",
        ArticulationType.PRISMATIC,
        parent=mast_2,
        child=mast_3,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.360),
    )
    model.articulation(
        "mast_section_3_to_4",
        ArticulationType.PRISMATIC,
        parent=mast_3,
        child=mast_4,
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.35, lower=0.0, upper=0.300),
    )
    model.articulation(
        "chassis_to_left_outrigger",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child="left_outrigger",
        origin=Origin(xyz=(-0.425, 0.0, 0.105)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.20, lower=0.0, upper=0.110),
    )
    model.articulation(
        "chassis_to_right_outrigger",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child="right_outrigger",
        origin=Origin(xyz=(0.425, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.20, lower=0.0, upper=0.110),
    )
    model.articulation(
        "chassis_to_front_outrigger",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child="front_outrigger",
        origin=Origin(xyz=(0.0, 0.325, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.20, lower=0.0, upper=0.110),
    )
    model.articulation(
        "chassis_to_rear_outrigger",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child="rear_outrigger",
        origin=Origin(xyz=(0.0, -0.325, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.20, lower=0.0, upper=0.110),
    )
    model.articulation(
        "mast_head_to_left_floodlight",
        ArticulationType.REVOLUTE,
        parent=mast_4,
        child=left_light,
        origin=Origin(xyz=(-0.235, 0.0, 0.642)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.85, upper=0.55),
    )
    model.articulation(
        "mast_head_to_right_floodlight",
        ArticulationType.REVOLUTE,
        parent=mast_4,
        child=right_light,
        origin=Origin(xyz=(0.235, 0.0, 0.642)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=-0.85, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    chassis = object_model.get_part("chassis")
    mast_1 = object_model.get_part("mast_section_1")
    mast_2 = object_model.get_part("mast_section_2")
    mast_3 = object_model.get_part("mast_section_3")
    mast_4 = object_model.get_part("mast_section_4")
    left_outrigger = object_model.get_part("left_outrigger")
    right_outrigger = object_model.get_part("right_outrigger")
    front_outrigger = object_model.get_part("front_outrigger")
    rear_outrigger = object_model.get_part("rear_outrigger")
    left_light = object_model.get_part("left_floodlight")
    right_light = object_model.get_part("right_floodlight")
    mast_lift_1 = object_model.get_articulation("mast_section_1_to_2")
    mast_lift_2 = object_model.get_articulation("mast_section_2_to_3")
    mast_lift_3 = object_model.get_articulation("mast_section_3_to_4")
    left_out_slide = object_model.get_articulation("chassis_to_left_outrigger")
    right_out_slide = object_model.get_articulation("chassis_to_right_outrigger")
    front_out_slide = object_model.get_articulation("chassis_to_front_outrigger")
    rear_out_slide = object_model.get_articulation("chassis_to_rear_outrigger")
    left_tilt = object_model.get_articulation("mast_head_to_left_floodlight")
    right_tilt = object_model.get_articulation("mast_head_to_right_floodlight")

    mast_socket = chassis.get_visual("mast_socket")
    left_sleeve = chassis.get_visual("left_sleeve")
    right_sleeve = chassis.get_visual("right_sleeve")
    front_sleeve = chassis.get_visual("front_sleeve")
    rear_sleeve = chassis.get_visual("rear_sleeve")
    mast_1_tube = mast_1.get_visual("mast_tube")
    mast_2_tube = mast_2.get_visual("mast_tube")
    mast_3_tube = mast_3.get_visual("mast_tube")
    mast_4_tube = mast_4.get_visual("mast_tube")
    mast_1_ring = mast_1.get_visual("clamp_ring")
    mast_2_ring = mast_2.get_visual("clamp_ring")
    mast_3_ring = mast_3.get_visual("clamp_ring")
    mast_4_ring = mast_4.get_visual("clamp_ring")
    crossbar = mast_4.get_visual("crossbar_beam")
    left_yoke = mast_4.get_visual("left_yoke_bridge")
    right_yoke = mast_4.get_visual("right_yoke_bridge")
    left_yoke_right_cheek = mast_4.get_visual("left_yoke_right_cheek")
    right_yoke_left_cheek = mast_4.get_visual("right_yoke_left_cheek")
    left_arm = left_outrigger.get_visual("arm_tube")
    right_arm = right_outrigger.get_visual("arm_tube")
    front_arm = front_outrigger.get_visual("arm_tube")
    rear_arm = rear_outrigger.get_visual("arm_tube")
    left_housing = left_light.get_visual("housing")
    right_housing = right_light.get_visual("housing")
    left_trunnion = left_light.get_visual("left_trunnion")
    right_trunnion = right_light.get_visual("right_trunnion")

    ctx.allow_overlap(mast_1, mast_2, reason="nested telescoping mast sections intentionally share sleeve volume")
    ctx.allow_overlap(mast_2, mast_3, reason="nested telescoping mast sections intentionally share sleeve volume")
    ctx.allow_overlap(mast_3, mast_4, reason="nested telescoping mast sections intentionally share sleeve volume")
    ctx.allow_overlap(mast_1, mast_3, reason="retracted nested mast sections overlap through the outer sleeve stack")
    ctx.allow_overlap(mast_2, mast_4, reason="retracted nested mast sections overlap through the outer sleeve stack")
    ctx.allow_overlap(left_light, mast_4, reason="floodlight trunnion pins nest through the fork yoke cheeks")
    ctx.allow_overlap(right_light, mast_4, reason="floodlight trunnion pins nest through the fork yoke cheeks")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.060)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(mast_1, chassis, axes="xy", elem_a=mast_1_tube, elem_b=mast_socket, min_overlap=0.012)
    ctx.expect_gap(
        mast_1,
        chassis,
        axis="z",
        max_gap=0.002,
        max_penetration=0.040,
        positive_elem=mast_1_tube,
        negative_elem=mast_socket,
    )
    ctx.expect_within(mast_2, mast_1, axes="xy", inner_elem=mast_2_tube, outer_elem=mast_1_tube)
    ctx.expect_within(mast_3, mast_2, axes="xy", inner_elem=mast_3_tube, outer_elem=mast_2_tube)
    ctx.expect_within(mast_4, mast_3, axes="xy", inner_elem=mast_4_tube, outer_elem=mast_3_tube)
    ctx.expect_overlap(mast_2, mast_1, axes="xy", elem_a=mast_2_tube, elem_b=mast_1_ring, min_overlap=0.010)
    ctx.expect_overlap(mast_3, mast_2, axes="xy", elem_a=mast_3_tube, elem_b=mast_2_ring, min_overlap=0.008)
    ctx.expect_overlap(mast_4, mast_3, axes="xy", elem_a=mast_4_tube, elem_b=mast_3_ring, min_overlap=0.006)
    ctx.expect_overlap(mast_4, mast_3, axes="xy", elem_a=mast_4_ring, elem_b=mast_3_tube, min_overlap=0.006)
    ctx.expect_within(left_outrigger, chassis, axes="yz", inner_elem=left_arm, outer_elem=left_sleeve)
    ctx.expect_within(right_outrigger, chassis, axes="yz", inner_elem=right_arm, outer_elem=right_sleeve)
    ctx.expect_within(front_outrigger, chassis, axes="xz", inner_elem=front_arm, outer_elem=front_sleeve)
    ctx.expect_within(rear_outrigger, chassis, axes="xz", inner_elem=rear_arm, outer_elem=rear_sleeve)
    ctx.expect_overlap(left_light, mast_4, axes="xy", elem_a=left_housing, elem_b=left_yoke, min_overlap=0.015)
    ctx.expect_overlap(right_light, mast_4, axes="xy", elem_a=right_housing, elem_b=right_yoke, min_overlap=0.015)
    ctx.expect_overlap(left_light, mast_4, axes="x", elem_a=left_housing, elem_b=crossbar, min_overlap=0.160)
    ctx.expect_overlap(right_light, mast_4, axes="x", elem_a=right_housing, elem_b=crossbar, min_overlap=0.160)
    ctx.expect_gap(
        mast_4,
        left_light,
        axis="z",
        max_gap=0.060,
        max_penetration=0.0,
        positive_elem=crossbar,
        negative_elem=left_housing,
    )
    ctx.expect_gap(
        mast_4,
        right_light,
        axis="z",
        max_gap=0.060,
        max_penetration=0.0,
        positive_elem=crossbar,
        negative_elem=right_housing,
    )
    ctx.expect_overlap(left_light, mast_4, axes="yz", elem_a=right_trunnion, elem_b=left_yoke_right_cheek, min_overlap=0.006)
    ctx.expect_overlap(right_light, mast_4, axes="yz", elem_a=left_trunnion, elem_b=right_yoke_left_cheek, min_overlap=0.006)
    ctx.expect_gap(right_light, left_light, axis="x", min_gap=0.110, positive_elem=right_housing, negative_elem=left_housing)

    with ctx.pose(
        {
            mast_lift_1: 0.330,
            mast_lift_2: 0.280,
            mast_lift_3: 0.240,
            left_out_slide: 0.100,
            right_out_slide: 0.100,
            front_out_slide: 0.100,
            rear_out_slide: 0.100,
            left_tilt: 0.45,
            right_tilt: 0.45,
        }
    ):
        ctx.expect_within(mast_2, mast_1, axes="xy", inner_elem=mast_2_tube, outer_elem=mast_1_tube)
        ctx.expect_within(mast_3, mast_2, axes="xy", inner_elem=mast_3_tube, outer_elem=mast_2_tube)
        ctx.expect_within(mast_4, mast_3, axes="xy", inner_elem=mast_4_tube, outer_elem=mast_3_tube)
        ctx.expect_overlap(mast_4, mast_3, axes="xy", elem_a=mast_4_ring, elem_b=mast_3_tube, min_overlap=0.006)
        ctx.expect_within(left_outrigger, chassis, axes="yz", inner_elem=left_arm, outer_elem=left_sleeve)
        ctx.expect_within(right_outrigger, chassis, axes="yz", inner_elem=right_arm, outer_elem=right_sleeve)
        ctx.expect_within(front_outrigger, chassis, axes="xz", inner_elem=front_arm, outer_elem=front_sleeve)
        ctx.expect_within(rear_outrigger, chassis, axes="xz", inner_elem=rear_arm, outer_elem=rear_sleeve)
        ctx.expect_overlap(left_light, mast_4, axes="xy", elem_a=left_housing, elem_b=left_yoke, min_overlap=0.010)
        ctx.expect_overlap(right_light, mast_4, axes="xy", elem_a=right_housing, elem_b=right_yoke, min_overlap=0.010)
        ctx.expect_overlap(left_light, mast_4, axes="x", elem_a=left_housing, elem_b=crossbar, min_overlap=0.160)
        ctx.expect_overlap(right_light, mast_4, axes="x", elem_a=right_housing, elem_b=crossbar, min_overlap=0.160)
        ctx.expect_gap(
            mast_4,
            left_light,
            axis="z",
            max_gap=0.080,
            max_penetration=0.0,
            positive_elem=crossbar,
            negative_elem=left_housing,
        )
        ctx.expect_gap(
            mast_4,
            right_light,
            axis="z",
            max_gap=0.080,
            max_penetration=0.0,
            positive_elem=crossbar,
            negative_elem=right_housing,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
