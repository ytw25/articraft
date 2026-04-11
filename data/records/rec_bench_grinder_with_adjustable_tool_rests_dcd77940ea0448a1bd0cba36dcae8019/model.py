from __future__ import annotations

from math import pi

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


WHEEL_Z = 0.170
LEFT_WHEEL_X = -0.180
RIGHT_WHEEL_X = 0.180
REST_PIVOT_Y = 0.050
REST_PIVOT_Z = 0.068
SHIELD_HINGE_Y = 0.080
SHIELD_HINGE_Z = 0.236


def _add_guard_visuals(part, *, prefix: str, x_center: float, side_sign: float, material) -> None:
    part.visual(
        Box((0.070, 0.130, 0.020)),
        origin=Origin(xyz=(x_center, -0.026, 0.256)),
        material=material,
        name=f"{prefix}_guard_top",
    )
    part.visual(
        Box((0.070, 0.012, 0.158)),
        origin=Origin(xyz=(x_center, -0.086, WHEEL_Z)),
        material=material,
        name=f"{prefix}_guard_rear",
    )
    part.visual(
        Box((0.012, 0.130, 0.120)),
        origin=Origin(xyz=(x_center + side_sign * 0.035, -0.020, 0.190)),
        material=material,
        name=f"{prefix}_guard_side",
    )
    part.visual(
        Box((0.014, 0.090, 0.130)),
        origin=Origin(xyz=(x_center - side_sign * 0.025, -0.040, 0.191)),
        material=material,
        name=f"{prefix}_guard_mount",
    )
    part.visual(
        Box((0.050, 0.024, 0.018)),
        origin=Origin(xyz=(x_center - side_sign * 0.010, 0.034, 0.070)),
        material=material,
        name=f"{prefix}_rest_bracket",
    )
    part.visual(
        Box((0.014, 0.084, 0.080)),
        origin=Origin(xyz=(x_center - side_sign * 0.024, -0.008, 0.096)),
        material=material,
        name=f"{prefix}_rest_post",
    )
    part.visual(
        Box((0.014, 0.090, 0.090)),
        origin=Origin(xyz=(x_center - side_sign * 0.024, 0.045, 0.206)),
        material=material,
        name=f"{prefix}_shield_post",
    )
    part.visual(
        Box((0.075, 0.003, 0.026)),
        origin=Origin(xyz=(x_center, 0.073, 0.207)),
        material=material,
        name=f"{prefix}_spark_plate",
    )
    part.visual(
        Box((0.016, 0.022, 0.030)),
        origin=Origin(xyz=(x_center, 0.078, 0.232)),
        material=material,
        name=f"{prefix}_hinge_ear",
    )


def _add_wire_wheel(part, brush_material, hub_material) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=0.074, length=0.012),
        origin=spin_origin,
        material=brush_material,
        name="wire_fringe",
    )
    part.visual(
        Cylinder(radius=0.071, length=0.022),
        origin=spin_origin,
        material=brush_material,
        name="wire_brush",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=spin_origin,
        material=hub_material,
        name="wire_hub",
    )
    for x_offset in (-0.010, 0.010):
        part.visual(
            Cylinder(radius=0.032, length=0.003),
            origin=Origin(xyz=(x_offset, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hub_material,
            name=f"wire_flange_{0 if x_offset < 0.0 else 1}",
        )


def _add_stone_wheel(part, stone_material, hub_material) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=0.075, length=0.025),
        origin=spin_origin,
        material=stone_material,
        name="stone_disc",
    )
    part.visual(
        Cylinder(radius=0.026, length=0.031),
        origin=spin_origin,
        material=hub_material,
        name="stone_hub",
    )
    for x_offset in (-0.012, 0.012):
        part.visual(
            Cylinder(radius=0.031, length=0.003),
            origin=Origin(xyz=(x_offset, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hub_material,
            name=f"stone_flange_{0 if x_offset < 0.0 else 1}",
        )


def _add_rest(part, material) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name="pivot_barrel",
    )
    part.visual(
        Box((0.018, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.010, 0.011)),
        material=material,
        name="rest_arm",
    )
    part.visual(
        Box((0.066, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, 0.024, 0.017)),
        material=material,
        name="rest_plate",
    )
    part.visual(
        Box((0.066, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.039, 0.015)),
        material=material,
        name="front_lip",
    )


def _add_eye_shield(part, clamp_material, panel_material) -> None:
    part.visual(
        Cylinder(radius=0.003, length=0.036),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=clamp_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.034, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.018, -0.006)),
        material=clamp_material,
        name="shield_clamp",
    )
    part.visual(
        Box((0.010, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, -0.006)),
        material=clamp_material,
        name="shield_stem",
    )
    part.visual(
        Box((0.078, 0.003, 0.060)),
        origin=Origin(xyz=(0.0, 0.018, -0.040)),
        material=panel_material,
        name="shield_panel",
    )


def _add_frame_body(part, *, machine_grey, base_black) -> None:
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    part.visual(
        Box((0.320, 0.185, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=base_black,
        name="base_cover",
    )
    part.visual(
        Box((0.072, 0.090, 0.056)),
        origin=Origin(xyz=(-0.058, 0.0, 0.048)),
        material=machine_grey,
        name="left_pedestal",
    )
    part.visual(
        Box((0.072, 0.090, 0.056)),
        origin=Origin(xyz=(0.058, 0.0, 0.048)),
        material=machine_grey,
        name="right_pedestal",
    )
    part.visual(
        Box((0.046, 0.072, 0.042)),
        origin=Origin(xyz=(-0.058, 0.0, 0.097)),
        material=machine_grey,
        name="left_cap",
    )
    part.visual(
        Box((0.046, 0.072, 0.042)),
        origin=Origin(xyz=(0.058, 0.0, 0.097)),
        material=machine_grey,
        name="right_cap",
    )
    part.visual(
        Cylinder(radius=0.060, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.150), rpy=(0.0, pi / 2.0, 0.0)),
        material=machine_grey,
        name="motor_barrel",
    )
    part.visual(
        Cylinder(radius=0.068, length=0.048),
        origin=Origin(xyz=(-0.118, 0.0, 0.150), rpy=(0.0, pi / 2.0, 0.0)),
        material=machine_grey,
        name="left_bell",
    )
    part.visual(
        Cylinder(radius=0.068, length=0.048),
        origin=Origin(xyz=(0.118, 0.0, 0.150), rpy=(0.0, pi / 2.0, 0.0)),
        material=machine_grey,
        name="right_bell",
    )
    part.visual(
        Cylinder(radius=0.044, length=0.026),
        origin=Origin(xyz=(-0.150, 0.0, 0.165), rpy=(0.0, pi / 2.0, 0.0)),
        material=machine_grey,
        name="left_neck",
    )
    part.visual(
        Cylinder(radius=0.044, length=0.026),
        origin=Origin(xyz=(0.150, 0.0, 0.165), rpy=(0.0, pi / 2.0, 0.0)),
        material=machine_grey,
        name="right_neck",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(-0.150, 0.0, WHEEL_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=base_black,
        name="left_shaft",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(xyz=(0.1495, 0.0, WHEEL_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=base_black,
        name="right_shaft",
    )
    part.visual(
        Box((0.058, 0.014, 0.038)),
        origin=Origin(xyz=(0.0, 0.064, 0.112)),
        material=machine_grey,
        name="switch_bezel",
    )
    part.visual(
        Box((0.040, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.053, 0.112)),
        material=machine_grey,
        name="switch_podium",
    )
    part.visual(
        Box((0.128, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.056, 0.155)),
        material=base_black,
        name="label_panel",
    )


def _add_switch(part, pivot_material, cap_material) -> None:
    part.visual(
        Cylinder(radius=0.004, length=0.050),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=pivot_material,
        name="switch_pivot",
    )
    part.visual(
        Box((0.042, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.006, -0.010)),
        material=cap_material,
        name="switch_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_grinder")

    machine_grey = model.material("machine_grey", rgba=(0.58, 0.61, 0.64, 1.0))
    base_black = model.material("base_black", rgba=(0.16, 0.17, 0.18, 1.0))
    guard_grey = model.material("guard_grey", rgba=(0.50, 0.53, 0.56, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.31, 0.33, 0.35, 1.0))
    abrasive = model.material("abrasive", rgba=(0.69, 0.69, 0.64, 1.0))
    wire = model.material("wire", rgba=(0.66, 0.68, 0.71, 1.0))
    amber = model.material("amber", rgba=(0.85, 0.72, 0.24, 0.40))
    switch_red = model.material("switch_red", rgba=(0.77, 0.12, 0.10, 1.0))

    frame = model.part("frame")
    _add_frame_body(frame, machine_grey=machine_grey, base_black=base_black)
    _add_guard_visuals(frame, prefix="left", x_center=LEFT_WHEEL_X, side_sign=-1.0, material=guard_grey)
    _add_guard_visuals(frame, prefix="right", x_center=RIGHT_WHEEL_X, side_sign=1.0, material=guard_grey)

    wire_wheel = model.part("wire_wheel")
    _add_wire_wheel(wire_wheel, brush_material=wire, hub_material=dark_steel)

    stone_wheel = model.part("stone_wheel")
    _add_stone_wheel(stone_wheel, stone_material=abrasive, hub_material=dark_steel)

    left_rest = model.part("left_rest")
    _add_rest(left_rest, material=steel)

    right_rest = model.part("right_rest")
    _add_rest(right_rest, material=steel)

    left_shield = model.part("left_shield")
    _add_eye_shield(left_shield, clamp_material=dark_steel, panel_material=amber)

    right_shield = model.part("right_shield")
    _add_eye_shield(right_shield, clamp_material=dark_steel, panel_material=amber)

    power_switch = model.part("power_switch")
    _add_switch(power_switch, pivot_material=dark_steel, cap_material=switch_red)

    model.articulation(
        "wire_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wire_wheel,
        origin=Origin(xyz=(LEFT_WHEEL_X, 0.0, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )
    model.articulation(
        "stone_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=stone_wheel,
        origin=Origin(xyz=(RIGHT_WHEEL_X, 0.0, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=30.0),
    )

    model.articulation(
        "left_rest_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_rest,
        origin=Origin(xyz=(LEFT_WHEEL_X, REST_PIVOT_Y, REST_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "right_rest_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_rest,
        origin=Origin(xyz=(RIGHT_WHEEL_X, REST_PIVOT_Y, REST_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.35, upper=0.35),
    )

    model.articulation(
        "left_shield_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_shield,
        origin=Origin(xyz=(LEFT_WHEEL_X, SHIELD_HINGE_Y, SHIELD_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5, lower=0.0, upper=1.05),
    )
    model.articulation(
        "right_shield_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_shield,
        origin=Origin(xyz=(RIGHT_WHEEL_X, SHIELD_HINGE_Y, SHIELD_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.5, lower=0.0, upper=1.05),
    )

    model.articulation(
        "switch_toggle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=power_switch,
        origin=Origin(xyz=(0.0, 0.071, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=1.5, lower=-0.18, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    wire_wheel = object_model.get_part("wire_wheel")
    stone_wheel = object_model.get_part("stone_wheel")
    left_rest = object_model.get_part("left_rest")
    right_rest = object_model.get_part("right_rest")
    left_shield = object_model.get_part("left_shield")
    right_shield = object_model.get_part("right_shield")
    power_switch = object_model.get_part("power_switch")

    left_rest_tilt = object_model.get_articulation("left_rest_tilt")
    right_rest_tilt = object_model.get_articulation("right_rest_tilt")
    left_shield_tilt = object_model.get_articulation("left_shield_tilt")
    switch_toggle = object_model.get_articulation("switch_toggle")

    ctx.expect_gap(
        frame,
        wire_wheel,
        axis="z",
        positive_elem="left_guard_top",
        negative_elem="wire_brush",
        min_gap=0.000,
        max_gap=0.020,
        name="left guard top sits just above wire wheel",
    )
    ctx.expect_gap(
        frame,
        stone_wheel,
        axis="z",
        positive_elem="right_guard_top",
        negative_elem="stone_disc",
        min_gap=0.000,
        max_gap=0.020,
        name="right guard top sits just above stone wheel",
    )
    ctx.expect_gap(
        wire_wheel,
        frame,
        axis="y",
        positive_elem="wire_brush",
        negative_elem="left_guard_rear",
        min_gap=0.002,
        max_gap=0.020,
        name="wire wheel clears the left rear guard wall",
    )
    ctx.expect_gap(
        stone_wheel,
        frame,
        axis="y",
        positive_elem="stone_disc",
        negative_elem="right_guard_rear",
        min_gap=0.002,
        max_gap=0.020,
        name="stone wheel clears the right rear guard wall",
    )

    ctx.expect_gap(
        wire_wheel,
        left_rest,
        axis="z",
        positive_elem="wire_brush",
        negative_elem="rest_plate",
        min_gap=0.004,
        max_gap=0.020,
        name="left rest sits just below wire wheel",
    )
    ctx.expect_gap(
        stone_wheel,
        right_rest,
        axis="z",
        positive_elem="stone_disc",
        negative_elem="rest_plate",
        min_gap=0.004,
        max_gap=0.020,
        name="right rest sits just below stone wheel",
    )
    ctx.expect_overlap(
        wire_wheel,
        left_rest,
        axes="x",
        elem_a="wire_brush",
        elem_b="rest_plate",
        min_overlap=0.018,
        name="left rest spans the wire wheel width",
    )
    ctx.expect_overlap(
        stone_wheel,
        right_rest,
        axes="x",
        elem_a="stone_disc",
        elem_b="rest_plate",
        min_overlap=0.020,
        name="right rest spans the stone wheel width",
    )

    ctx.expect_gap(
        left_shield,
        wire_wheel,
        axis="y",
        positive_elem="shield_panel",
        negative_elem="wire_brush",
        min_gap=0.008,
        max_gap=0.030,
        name="left shield sits ahead of the wire wheel",
    )
    ctx.expect_gap(
        right_shield,
        stone_wheel,
        axis="y",
        positive_elem="shield_panel",
        negative_elem="stone_disc",
        min_gap=0.008,
        max_gap=0.030,
        name="right shield sits ahead of the stone wheel",
    )

    left_shield_limits = left_shield_tilt.motion_limits
    if left_shield_limits is not None and left_shield_limits.upper is not None:
        closed_aabb = ctx.part_element_world_aabb(left_shield, elem="shield_panel")
        with ctx.pose({left_shield_tilt: left_shield_limits.upper}):
            opened_aabb = ctx.part_element_world_aabb(left_shield, elem="shield_panel")
        ctx.check(
            "left shield tilts upward",
            closed_aabb is not None
            and opened_aabb is not None
            and opened_aabb[0][2] > closed_aabb[0][2] + 0.015,
            details=f"closed={closed_aabb}, opened={opened_aabb}",
        )

    right_rest_limits = right_rest_tilt.motion_limits
    if right_rest_limits is not None and right_rest_limits.lower is not None and right_rest_limits.upper is not None:
        with ctx.pose({right_rest_tilt: right_rest_limits.lower}):
            low_aabb = ctx.part_element_world_aabb(right_rest, elem="rest_plate")
        with ctx.pose({right_rest_tilt: right_rest_limits.upper}):
            high_aabb = ctx.part_element_world_aabb(right_rest, elem="rest_plate")
        ctx.check(
            "right rest pivots upward at higher angle",
            low_aabb is not None
            and high_aabb is not None
            and high_aabb[1][2] > low_aabb[1][2] + 0.010,
            details=f"low={low_aabb}, high={high_aabb}",
        )

    switch_limits = switch_toggle.motion_limits
    if switch_limits is not None and switch_limits.lower is not None and switch_limits.upper is not None:
        with ctx.pose({switch_toggle: switch_limits.lower}):
            low_aabb = ctx.part_element_world_aabb(power_switch, elem="switch_cap")
        with ctx.pose({switch_toggle: switch_limits.upper}):
            high_aabb = ctx.part_element_world_aabb(power_switch, elem="switch_cap")
        ctx.check(
            "power switch rocks outward at positive angle",
            low_aabb is not None
            and high_aabb is not None
            and high_aabb[1][1] > low_aabb[1][1] + 0.003,
            details=f"low={low_aabb}, high={high_aabb}",
        )

    ctx.allow_overlap(
        frame,
        left_shield,
        elem_a="left_hinge_ear",
        elem_b="hinge_barrel",
        reason="The left eye shield hinge barrel intentionally nests through the fixed hinge ear.",
    )
    ctx.allow_overlap(
        frame,
        left_shield,
        elem_a="left_hinge_ear",
        elem_b="shield_stem",
        reason="The left shield stem passes through the hinge ear clearance around the pivot.",
    )
    ctx.allow_overlap(
        frame,
        right_shield,
        elem_a="right_hinge_ear",
        elem_b="hinge_barrel",
        reason="The right eye shield hinge barrel intentionally nests through the fixed hinge ear.",
    )
    ctx.allow_overlap(
        frame,
        right_shield,
        elem_a="right_hinge_ear",
        elem_b="shield_stem",
        reason="The right shield stem passes through the hinge ear clearance around the pivot.",
    )

    return ctx.report()


object_model = build_object_model()
