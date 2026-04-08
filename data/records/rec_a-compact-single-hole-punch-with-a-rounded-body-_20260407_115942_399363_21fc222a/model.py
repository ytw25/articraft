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
    rounded_rect_profile,
    section_loft,
)


def _x_section(
    x_pos: float,
    width_y: float,
    height_z: float,
    radius: float,
    *,
    center_z: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, z + center_z)
        for y, z in rounded_rect_profile(width_y, height_z, radius, corner_segments=6)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_punch")

    body_paint = model.material("body_paint", rgba=(0.16, 0.18, 0.21, 1.0))
    lever_paint = model.material("lever_paint", rgba=(0.66, 0.10, 0.12, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    body_length = 0.104
    body_width = 0.034
    hinge_x = -0.043
    hinge_z = 0.031
    tab_hinge_x = -0.052
    tab_hinge_z = 0.022

    lower_body = model.part("lower_body")

    body_shell_geom = section_loft(
        [
            _x_section(-0.048, 0.034, 0.028, 0.0045, center_z=0.014),
            _x_section(-0.016, 0.034, 0.024, 0.0042, center_z=0.012),
            _x_section(0.018, 0.032, 0.020, 0.0038, center_z=0.010),
            _x_section(0.048, 0.028, 0.016, 0.0032, center_z=0.008),
        ]
    )
    lower_body.visual(
        mesh_from_geometry(body_shell_geom, "lower_body_shell"),
        material=body_paint,
        name="body_shell",
    )
    lower_body.visual(
        Box((0.020, 0.024, 0.010)),
        origin=Origin(xyz=(0.036, 0.0, 0.015)),
        material=dark_trim,
        name="die_housing",
    )
    lower_body.visual(
        Box((0.010, 0.028, 0.010)),
        origin=Origin(xyz=(-0.052, 0.0, 0.017)),
        material=body_paint,
        name="rear_bridge",
    )
    lower_body.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(hinge_x, -0.0115, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_hinge_ear",
    )
    lower_body.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(hinge_x, 0.0115, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_hinge_ear",
    )
    lower_body.visual(
        Cylinder(radius=0.003, length=0.009),
        origin=Origin(xyz=(tab_hinge_x, -0.008, tab_hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_tab_ear",
    )
    lower_body.visual(
        Cylinder(radius=0.003, length=0.009),
        origin=Origin(xyz=(tab_hinge_x, 0.008, tab_hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_tab_ear",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, 0.034)),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    upper_lever = model.part("upper_lever")

    lever_shell_geom = section_loft(
        [
            _x_section(0.001, 0.031, 0.013, 0.0040, center_z=0.0040),
            _x_section(0.028, 0.031, 0.012, 0.0038, center_z=0.0035),
            _x_section(0.056, 0.029, 0.011, 0.0035, center_z=0.0030),
            _x_section(0.082, 0.026, 0.010, 0.0030, center_z=0.0025),
        ]
    )
    upper_lever.visual(
        mesh_from_geometry(lever_shell_geom, "upper_lever_shell"),
        material=lever_paint,
        name="lever_shell",
    )
    upper_lever.visual(
        Cylinder(radius=0.0037, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lever_knuckle",
    )
    upper_lever.visual(
        Cylinder(radius=0.0055, length=0.008),
        origin=Origin(xyz=(0.066, 0.0, -0.0044)),
        material=steel,
        name="punch_stem",
    )
    upper_lever.visual(
        Cylinder(radius=0.0042, length=0.0026),
        origin=Origin(xyz=(0.066, 0.0, -0.0092)),
        material=steel,
        name="punch_tip",
    )
    upper_lever.inertial = Inertial.from_geometry(
        Box((0.086, body_width, 0.022)),
        mass=0.12,
        origin=Origin(xyz=(0.043, 0.0, 0.003)),
    )

    lock_tab = model.part("lock_tab")
    lock_tab.visual(
        Cylinder(radius=0.0026, length=0.007),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tab_knuckle",
    )
    lock_tab.visual(
        Box((0.022, 0.010, 0.0036)),
        origin=Origin(xyz=(-0.013, 0.0, -0.0018)),
        material=dark_trim,
        name="tab_plate",
    )
    lock_tab.visual(
        Box((0.006, 0.010, 0.006)),
        origin=Origin(xyz=(-0.022, 0.0, 0.001)),
        material=dark_trim,
        name="tab_hook",
    )
    lock_tab.inertial = Inertial.from_geometry(
        Box((0.024, 0.010, 0.008)),
        mass=0.02,
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_lever,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )

    model.articulation(
        "body_to_lock_tab",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=lock_tab,
        origin=Origin(xyz=(tab_hinge_x, 0.0, tab_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    upper_lever = object_model.get_part("upper_lever")
    lock_tab = object_model.get_part("lock_tab")
    lever_hinge = object_model.get_articulation("body_to_lever")
    tab_hinge = object_model.get_articulation("body_to_lock_tab")

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    with ctx.pose({lever_hinge: 0.0}):
        ctx.expect_gap(
            upper_lever,
            lower_body,
            axis="z",
            positive_elem="lever_shell",
            negative_elem="body_shell",
            max_gap=0.012,
            max_penetration=0.0,
            name="closed lever sits just above the lower body",
        )
        ctx.expect_overlap(
            upper_lever,
            lower_body,
            axes="xy",
            elem_a="lever_shell",
            elem_b="body_shell",
            min_overlap=0.020,
            name="closed lever spans the punch body footprint",
        )

    rest_lever_center_z = _aabb_center_z(
        ctx.part_element_world_aabb(upper_lever, elem="lever_shell")
    )
    with ctx.pose({lever_hinge: lever_hinge.motion_limits.upper}):
        opened_lever_center_z = _aabb_center_z(
            ctx.part_element_world_aabb(upper_lever, elem="lever_shell")
        )
    ctx.check(
        "lever opens upward from the rear hinge",
        rest_lever_center_z is not None
        and opened_lever_center_z is not None
        and opened_lever_center_z > rest_lever_center_z + 0.020,
        details=f"rest_z={rest_lever_center_z}, open_z={opened_lever_center_z}",
    )

    rest_tab_center_z = _aabb_center_z(ctx.part_element_world_aabb(lock_tab, elem="tab_plate"))
    with ctx.pose({tab_hinge: tab_hinge.motion_limits.upper}):
        opened_tab_center_z = _aabb_center_z(
            ctx.part_element_world_aabb(lock_tab, elem="tab_plate")
        )
    ctx.check(
        "rear lock tab folds upward",
        rest_tab_center_z is not None
        and opened_tab_center_z is not None
        and opened_tab_center_z > rest_tab_center_z + 0.010,
        details=f"rest_z={rest_tab_center_z}, open_z={opened_tab_center_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
