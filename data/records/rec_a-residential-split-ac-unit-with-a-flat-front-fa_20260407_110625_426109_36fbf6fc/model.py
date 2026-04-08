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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_ac_unit")

    housing_white = model.material("housing_white", rgba=(0.93, 0.94, 0.95, 1.0))
    housing_trim = model.material("housing_trim", rgba=(0.87, 0.89, 0.91, 1.0))
    vent_dark = model.material("vent_dark", rgba=(0.19, 0.21, 0.24, 1.0))
    button_grey = model.material("button_grey", rgba=(0.76, 0.79, 0.82, 1.0))

    shell = model.part("shell")
    shell.visual(
        Box((0.92, 0.010, 0.31)),
        origin=Origin(xyz=(0.0, 0.005, 0.155)),
        material=housing_trim,
        name="back_plate",
    )
    shell.visual(
        Box((0.92, 0.21, 0.03)),
        origin=Origin(xyz=(0.0, 0.105, 0.295)),
        material=housing_white,
        name="top_cover",
    )
    shell.visual(
        Box((0.02, 0.21, 0.255)),
        origin=Origin(xyz=(-0.45, 0.105, 0.1525)),
        material=housing_white,
        name="left_endcap",
    )
    shell.visual(
        Box((0.02, 0.21, 0.255)),
        origin=Origin(xyz=(0.45, 0.105, 0.1525)),
        material=housing_white,
        name="right_endcap",
    )
    shell.visual(
        Box((0.795, 0.015, 0.155)),
        origin=Origin(xyz=(-0.0425, 0.2075, 0.2025)),
        material=housing_white,
        name="upper_fascia_main",
    )
    shell.visual(
        Box((0.018, 0.015, 0.155)),
        origin=Origin(xyz=(0.364, 0.2075, 0.2025)),
        material=housing_white,
        name="button_column_left_strip",
    )
    shell.visual(
        Box((0.033, 0.015, 0.155)),
        origin=Origin(xyz=(0.4235, 0.2075, 0.2025)),
        material=housing_white,
        name="button_column_right_strip",
    )
    shell.visual(
        Box((0.034, 0.015, 0.028)),
        origin=Origin(xyz=(0.39, 0.2075, 0.266)),
        material=housing_white,
        name="button_opening_top_strip",
    )
    shell.visual(
        Box((0.034, 0.015, 0.006)),
        origin=Origin(xyz=(0.39, 0.2075, 0.233)),
        material=housing_white,
        name="button_opening_middle_strip",
    )
    shell.visual(
        Box((0.034, 0.015, 0.089)),
        origin=Origin(xyz=(0.39, 0.2075, 0.1695)),
        material=housing_white,
        name="button_opening_bottom_strip",
    )
    shell.visual(
        Box((0.92, 0.12, 0.055)),
        origin=Origin(xyz=(0.0, 0.07, 0.0275)),
        material=housing_trim,
        name="lower_body",
    )
    shell.visual(
        Box((0.92, 0.09, 0.02)),
        origin=Origin(xyz=(0.0, 0.165, 0.065)),
        material=housing_white,
        name="front_sill",
    )
    shell.visual(
        Box((0.86, 0.08, 0.012)),
        origin=Origin(xyz=(0.0, 0.16, 0.119)),
        material=vent_dark,
        name="outlet_ceiling",
    )
    shell.visual(
        Box((0.03, 0.09, 0.05)),
        origin=Origin(xyz=(-0.43, 0.165, 0.1)),
        material=vent_dark,
        name="left_outlet_cheek",
    )
    shell.visual(
        Box((0.03, 0.09, 0.05)),
        origin=Origin(xyz=(0.43, 0.165, 0.1)),
        material=vent_dark,
        name="right_outlet_cheek",
    )
    shell.inertial = Inertial.from_geometry(
        Box((0.92, 0.225, 0.31)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.1125, 0.155)),
    )

    flap = model.part("front_flap")
    flap.visual(
        Cylinder(radius=0.004, length=0.84),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_trim,
        name="flap_hinge_barrel",
    )
    flap.visual(
        Box((0.84, 0.004, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=housing_trim,
        name="flap_panel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.84, 0.014, 0.07)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
    )

    model.articulation(
        "shell_to_front_flap",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=flap,
        origin=Origin(xyz=(0.0, 0.219, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.6,
            lower=0.0,
            upper=1.15,
        ),
    )

    button_zs = (0.244, 0.222)
    for index, button_z in enumerate(button_zs):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.034, 0.004, 0.016)),
            origin=Origin(xyz=(0.0, 0.006, 0.0)),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.014, 0.010)),
            origin=Origin(xyz=(0.0, -0.003, 0.0)),
            material=button_grey,
            name="button_plunger",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.034, 0.018, 0.016)),
            mass=0.03,
            origin=Origin(xyz=(0.0, 0.001, 0.0)),
        )
        model.articulation(
            f"shell_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=shell,
            child=button,
            origin=Origin(xyz=(0.39, 0.207, button_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.06,
                lower=0.0,
                upper=0.004,
            ),
        )

    for index, fin_x in enumerate((-0.30, -0.15, 0.0, 0.15, 0.30)):
        fin = model.part(f"fin_{index}")
        fin.visual(
            Box((0.003, 0.052, 0.03)),
            origin=Origin(),
            material=vent_dark,
            name="fin_blade",
        )
        fin.visual(
            Cylinder(radius=0.0022, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.017)),
            material=vent_dark,
            name="fin_top_pin",
        )
        fin.visual(
            Cylinder(radius=0.0022, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=vent_dark,
            name="fin_bottom_pin",
        )
        fin.inertial = Inertial.from_geometry(
            Box((0.008, 0.052, 0.038)),
            mass=0.025,
            origin=Origin(),
        )
        model.articulation(
            f"shell_to_fin_{index}",
            ArticulationType.REVOLUTE,
            parent=shell,
            child=fin,
            origin=Origin(xyz=(fin_x, 0.158, 0.094)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=1.6,
                lower=-0.65,
                upper=0.65,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    flap = object_model.get_part("front_flap")
    flap_joint = object_model.get_articulation("shell_to_front_flap")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    button_joint_0 = object_model.get_articulation("shell_to_button_0")
    button_joint_1 = object_model.get_articulation("shell_to_button_1")
    fin_0 = object_model.get_part("fin_0")
    fin_1 = object_model.get_part("fin_1")
    fin_4 = object_model.get_part("fin_4")
    fin_joint_0 = object_model.get_articulation("shell_to_fin_0")
    fin_joint_1 = object_model.get_articulation("shell_to_fin_1")
    fin_joint_4 = object_model.get_articulation("shell_to_fin_4")

    def _axis_index(axis: str) -> int:
        return {"x": 0, "y": 1, "z": 2}[axis]

    def _span(aabb, axis: str) -> float | None:
        if aabb is None:
            return None
        axis_index = _axis_index(axis)
        return aabb[1][axis_index] - aabb[0][axis_index]

    def _same_aabb(aabb_a, aabb_b, tol: float = 1e-6) -> bool:
        if aabb_a is None or aabb_b is None:
            return False
        for endpoint_a, endpoint_b in zip(aabb_a, aabb_b):
            for value_a, value_b in zip(endpoint_a, endpoint_b):
                if abs(value_a - value_b) > tol:
                    return False
        return True

    with ctx.pose({flap_joint: 0.0}):
        ctx.expect_overlap(
            flap,
            shell,
            axes="x",
            elem_a="flap_panel",
            elem_b="front_sill",
            min_overlap=0.80,
            name="front flap spans the outlet width",
        )
        flap_closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    with ctx.pose({flap_joint: 1.0}):
        flap_open_aabb = ctx.part_element_world_aabb(flap, elem="flap_panel")

    flap_opens_outward = (
        flap_closed_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[1][1] > flap_closed_aabb[1][1] + 0.04
        and flap_open_aabb[0][2] > flap_closed_aabb[0][2] + 0.02
    )
    ctx.check(
        "front flap rotates outward and upward",
        flap_opens_outward,
        details=f"closed={flap_closed_aabb}, open={flap_open_aabb}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_joint_0: 0.004, button_joint_1: 0.004}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_pressed = ctx.part_world_position(button_1)

    buttons_press_inward = (
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_pressed is not None
        and button_1_pressed is not None
        and button_0_pressed[1] < button_0_rest[1] - 0.0035
        and button_1_pressed[1] < button_1_rest[1] - 0.0035
    )
    ctx.check(
        "right-end buttons press inward",
        buttons_press_inward,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_pressed={button_1_pressed}"
        ),
    )

    fin_1_rest_aabb = ctx.part_element_world_aabb(fin_1, elem="fin_blade")
    with ctx.pose({fin_joint_0: 0.6}):
        fin_1_with_neighbor_moved = ctx.part_element_world_aabb(fin_1, elem="fin_blade")
    with ctx.pose({fin_joint_1: -0.6, fin_joint_4: 0.6}):
        fin_1_turned_aabb = ctx.part_element_world_aabb(fin_1, elem="fin_blade")
        fin_0_turned_aabb = ctx.part_element_world_aabb(fin_0, elem="fin_blade")
        fin_4_turned_aabb = ctx.part_element_world_aabb(fin_4, elem="fin_blade")

    fins_rotate_independently = (
        _same_aabb(fin_1_rest_aabb, fin_1_with_neighbor_moved)
        and (span_1 := _span(fin_1_turned_aabb, "x")) is not None
        and span_1 > 0.025
    )
    fins_remain_separate = (
        fin_0_turned_aabb is not None
        and fin_4_turned_aabb is not None
        and fin_4_turned_aabb[0][0] - fin_0_turned_aabb[1][0] > 0.50
    )
    ctx.check(
        "internal fins articulate independently",
        fins_rotate_independently and fins_remain_separate,
        details=(
            f"fin_1_rest={fin_1_rest_aabb}, fin_1_neighbor_pose={fin_1_with_neighbor_moved}, "
            f"fin_1_turned={fin_1_turned_aabb}, fin_0_turned={fin_0_turned_aabb}, "
            f"fin_4_turned={fin_4_turned_aabb}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
