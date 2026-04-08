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
    mesh_from_geometry,
    section_loft,
)


UNIT_WIDTH = 0.94
UNIT_HEIGHT = 0.29
UNIT_DEPTH = 0.235


def _housing_section(x: float, *, front_depth: float, lower_front_z: float) -> list[tuple[float, float, float]]:
    return [
        (x, 0.000, 0.094),
        (x, 0.000, 0.278),
        (x, 0.048, 0.289),
        (x, front_depth * 0.82, 0.283),
        (x, front_depth, 0.248),
        (x, front_depth - 0.004, lower_front_z + 0.032),
        (x, front_depth - 0.022, lower_front_z + 0.006),
        (x, 0.060, 0.094),
    ]


def _door_section(x: float, *, face_depth: float) -> list[tuple[float, float, float]]:
    return [
        (x, 0.000, 0.000),
        (x, 0.010, 0.001),
        (x, face_depth - 0.003, -0.030),
        (x, face_depth, -0.114),
        (x, face_depth - 0.004, -0.154),
        (x, 0.007, -0.158),
        (x, 0.001, -0.040),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_split_indoor_unit")

    shell_white = model.material("shell_white", rgba=(0.93, 0.94, 0.95, 1.0))
    gloss_white = model.material("gloss_white", rgba=(0.97, 0.98, 0.99, 1.0))
    vent_dark = model.material("vent_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    smoked = model.material("smoked", rgba=(0.28, 0.31, 0.34, 0.70))

    housing = model.part("housing")

    housing_mesh = mesh_from_geometry(
        section_loft(
            [
                _housing_section(-0.470, front_depth=0.166, lower_front_z=0.112),
                _housing_section(-0.430, front_depth=0.173, lower_front_z=0.108),
                _housing_section(0.000, front_depth=0.178, lower_front_z=0.104),
                _housing_section(0.430, front_depth=0.173, lower_front_z=0.108),
                _housing_section(0.470, front_depth=0.166, lower_front_z=0.112),
            ]
        ),
        "mini_split_upper_housing",
    )
    housing.visual(housing_mesh, material=shell_white, name="upper_shell")
    housing.visual(
        Box((UNIT_WIDTH, 0.012, UNIT_HEIGHT)),
        origin=Origin(xyz=(0.000, 0.006, UNIT_HEIGHT * 0.5)),
        material=shell_white,
        name="back_wall",
    )
    housing.visual(
        Box((UNIT_WIDTH, 0.118, 0.060)),
        origin=Origin(xyz=(0.000, 0.059, 0.030)),
        material=shell_white,
        name="lower_base",
    )
    housing.visual(
        Box((UNIT_WIDTH - 0.062, 0.058, 0.018)),
        origin=Origin(xyz=(0.000, 0.136, 0.101)),
        material=vent_dark,
        name="outlet_roof",
    )
    housing.visual(
        Box((UNIT_WIDTH - 0.080, 0.010, 0.012)),
        origin=Origin(xyz=(0.000, 0.166, 0.102)),
        material=vent_dark,
        name="inner_hinge_beam",
    )
    housing.visual(
        Box((UNIT_WIDTH - 0.040, 0.014, 0.010)),
        origin=Origin(xyz=(0.000, 0.189, 0.081)),
        material=shell_white,
        name="outlet_lip",
    )
    housing.visual(
        Box((0.028, 0.162, 0.060)),
        origin=Origin(xyz=(-0.456, 0.087, 0.050)),
        material=shell_white,
        name="left_outlet_cheek",
    )
    housing.visual(
        Box((0.028, 0.162, 0.060)),
        origin=Origin(xyz=(0.456, 0.087, 0.050)),
        material=shell_white,
        name="right_outlet_cheek",
    )
    housing.visual(
        Box((0.028, 0.036, 0.028)),
        origin=Origin(xyz=(-0.456, 0.180, 0.069)),
        material=shell_white,
        name="left_lip_support",
    )
    housing.visual(
        Box((0.028, 0.036, 0.028)),
        origin=Origin(xyz=(0.456, 0.180, 0.069)),
        material=shell_white,
        name="right_lip_support",
    )
    housing.visual(
        Box((0.022, 0.176, 0.020)),
        origin=Origin(xyz=(-0.452, 0.094, 0.281)),
        material=shell_white,
        name="left_door_hinge_post",
    )
    housing.visual(
        Box((0.022, 0.176, 0.020)),
        origin=Origin(xyz=(0.452, 0.094, 0.281)),
        material=shell_white,
        name="right_door_hinge_post",
    )
    housing.visual(
        Box((UNIT_WIDTH - 0.090, 0.028, 0.032)),
        origin=Origin(xyz=(0.000, 0.126, 0.052)),
        material=charcoal,
        name="outlet_shadow",
    )
    housing.inertial = Inertial.from_geometry(
        Box((UNIT_WIDTH, UNIT_DEPTH, UNIT_HEIGHT)),
        mass=10.5,
        origin=Origin(xyz=(0.000, UNIT_DEPTH * 0.45, UNIT_HEIGHT * 0.5)),
    )

    filter_door = model.part("filter_door")
    door_mesh = mesh_from_geometry(
        section_loft(
            [
                _door_section(-0.430, face_depth=0.017),
                _door_section(-0.410, face_depth=0.020),
                _door_section(0.000, face_depth=0.022),
                _door_section(0.410, face_depth=0.020),
                _door_section(0.430, face_depth=0.017),
            ]
        ),
        "mini_split_filter_door",
    )
    filter_door.visual(door_mesh, material=gloss_white, name="door_panel")
    filter_door.visual(
        Box((0.084, 0.0030, 0.018)),
        origin=Origin(xyz=(0.325, 0.0205, -0.128)),
        material=smoked,
        name="status_window",
    )
    filter_door.visual(
        Cylinder(radius=0.0032, length=0.904),
        origin=Origin(xyz=(0.000, 0.002, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=charcoal,
        name="door_hinge_barrel",
    )
    filter_door.inertial = Inertial.from_geometry(
        Box((0.860, 0.022, 0.160)),
        mass=1.1,
        origin=Origin(xyz=(0.000, 0.011, -0.080)),
    )

    outer_flap = model.part("outer_flap")
    outer_flap.visual(
        Box((0.882, 0.037, 0.010)),
        origin=Origin(xyz=(0.000, -0.0175, -0.010)),
        material=gloss_white,
        name="outer_blade",
    )
    outer_flap.visual(
        Cylinder(radius=0.0045, length=0.882),
        origin=Origin(xyz=(0.000, -0.034, -0.010), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=gloss_white,
        name="outer_round_nose",
    )
    outer_flap.inertial = Inertial.from_geometry(
        Box((0.882, 0.037, 0.010)),
        mass=0.34,
        origin=Origin(xyz=(0.000, -0.018, -0.010)),
    )

    inner_flap = model.part("inner_flap")
    inner_flap.visual(
        Box((0.838, 0.027, 0.008)),
        origin=Origin(xyz=(0.000, -0.0135, -0.014)),
        material=charcoal,
        name="inner_blade",
    )
    inner_flap.visual(
        Cylinder(radius=0.0032, length=0.838),
        origin=Origin(xyz=(0.000, -0.026, -0.014), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=charcoal,
        name="inner_round_nose",
    )
    inner_flap.visual(
        Cylinder(radius=0.0035, length=0.838),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=charcoal,
        name="inner_hinge_barrel",
    )
    inner_flap.visual(
        Box((0.838, 0.006, 0.010)),
        origin=Origin(xyz=(0.000, -0.003, -0.007)),
        material=charcoal,
        name="inner_support_web",
    )
    inner_flap.inertial = Inertial.from_geometry(
        Box((0.838, 0.027, 0.008)),
        mass=0.22,
        origin=Origin(xyz=(0.000, -0.013, -0.004)),
    )

    model.articulation(
        "housing_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=filter_door,
        origin=Origin(xyz=(0.000, 0.179, 0.281)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.3,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "housing_to_outer_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=outer_flap,
        origin=Origin(xyz=(0.000, 0.197, 0.081)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "housing_to_inner_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=inner_flap,
        origin=Origin(xyz=(0.000, 0.171, 0.102)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(65.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    filter_door = object_model.get_part("filter_door")
    outer_flap = object_model.get_part("outer_flap")
    inner_flap = object_model.get_part("inner_flap")

    door_joint = object_model.get_articulation("housing_to_filter_door")
    outer_joint = object_model.get_articulation("housing_to_outer_flap")
    inner_joint = object_model.get_articulation("housing_to_inner_flap")

    ctx.expect_gap(
        filter_door,
        housing,
        axis="y",
        min_gap=0.001,
        max_gap=0.030,
        positive_elem="door_panel",
        negative_elem="upper_shell",
        name="closed filter door sits just proud of the cabinet",
    )
    ctx.expect_overlap(
        filter_door,
        housing,
        axes="x",
        min_overlap=0.86,
        elem_a="door_panel",
        elem_b="upper_shell",
        name="filter door spans nearly the full cabinet width",
    )

    closed_door_aabb = ctx.part_element_world_aabb(filter_door, elem="door_panel")
    with ctx.pose({door_joint: door_joint.motion_limits.upper}):
        open_door_aabb = ctx.part_element_world_aabb(filter_door, elem="door_panel")
    ctx.check(
        "filter door lifts upward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][2] > closed_door_aabb[0][2] + 0.060
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.020,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_outer_aabb = ctx.part_element_world_aabb(outer_flap, elem="outer_blade")
    with ctx.pose({outer_joint: outer_joint.motion_limits.upper}):
        open_outer_aabb = ctx.part_element_world_aabb(outer_flap, elem="outer_blade")
    ctx.check(
        "outer vane swings forward and down",
        closed_outer_aabb is not None
        and open_outer_aabb is not None
        and open_outer_aabb[0][2] < closed_outer_aabb[0][2] - 0.020
        and open_outer_aabb[0][1] > closed_outer_aabb[0][1] + 0.020,
        details=f"closed={closed_outer_aabb}, open={open_outer_aabb}",
    )

    closed_inner_aabb = ctx.part_element_world_aabb(inner_flap, elem="inner_blade")
    with ctx.pose({inner_joint: inner_joint.motion_limits.upper}):
        open_inner_aabb = ctx.part_element_world_aabb(inner_flap, elem="inner_blade")
    ctx.check(
        "inner vane also swings forward",
        closed_inner_aabb is not None
        and open_inner_aabb is not None
        and open_inner_aabb[0][2] < closed_inner_aabb[0][2] - 0.012
        and open_inner_aabb[0][1] > closed_inner_aabb[0][1] + 0.010,
        details=f"closed={closed_inner_aabb}, open={open_inner_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
