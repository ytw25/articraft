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
    superellipse_side_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_air_conditioner")

    plate_white = model.material("plate_white", rgba=(0.93, 0.94, 0.95, 1.0))
    shell_white = model.material("shell_white", rgba=(0.95, 0.96, 0.97, 1.0))
    vent_grey = model.material("vent_grey", rgba=(0.77, 0.79, 0.81, 1.0))
    vane_grey = model.material("vane_grey", rgba=(0.69, 0.72, 0.75, 1.0))
    knob_grey = model.material("knob_grey", rgba=(0.49, 0.52, 0.56, 1.0))
    shadow_grey = model.material("shadow_grey", rgba=(0.58, 0.60, 0.63, 1.0))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.66, 0.020, 0.46)),
        origin=Origin(xyz=(0.0, 0.010, 0.23)),
        material=plate_white,
        name="back_plate",
    )
    wall_plate.visual(
        Box((0.54, 0.008, 0.28)),
        origin=Origin(xyz=(0.0, 0.016, 0.23)),
        material=shadow_grey,
        name="mount_rail",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.66, 0.020, 0.46)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.010, 0.23)),
    )

    housing = model.part("housing")
    upper_shell = superellipse_side_loft(
        [
            (0.000, 0.220, 0.348, 0.560),
            (0.055, 0.214, 0.356, 0.580),
            (0.112, 0.208, 0.370, 0.602),
            (0.160, 0.208, 0.366, 0.620),
        ],
        exponents=2.7,
        segments=52,
    )
    housing.visual(
        mesh_from_geometry(upper_shell, "ac_upper_shell"),
        material=shell_white,
        name="upper_shell",
    )

    bezel_outer = rounded_rect_profile(0.620, 0.380, 0.048, corner_segments=8)
    outlet_hole = [
        (x, z - 0.055)
        for x, z in rounded_rect_profile(0.480, 0.110, 0.016, corner_segments=6)
    ]
    front_bezel = ExtrudeWithHolesGeometry(
        bezel_outer,
        [outlet_hole],
        0.050,
        center=True,
    ).rotate_x(math.pi / 2.0)
    housing.visual(
        mesh_from_geometry(front_bezel, "ac_front_bezel"),
        origin=Origin(xyz=(0.0, 0.135, 0.190)),
        material=shell_white,
        name="front_bezel",
    )

    housing.visual(
        Box((0.580, 0.090, 0.080)),
        origin=Origin(xyz=(0.0, 0.045, 0.040)),
        material=shell_white,
        name="lower_body",
    )
    housing.visual(
        Box((0.110, 0.110, 0.190)),
        origin=Origin(xyz=(-0.255, 0.055, 0.145)),
        material=shell_white,
        name="left_cheek",
    )
    housing.visual(
        Box((0.110, 0.110, 0.190)),
        origin=Origin(xyz=(0.255, 0.055, 0.145)),
        material=shell_white,
        name="right_cheek",
    )
    housing.visual(
        Box((0.500, 0.075, 0.016)),
        origin=Origin(xyz=(0.0, 0.090, 0.084)),
        material=vent_grey,
        name="outlet_floor",
    )
    housing.visual(
        Box((0.500, 0.075, 0.016)),
        origin=Origin(xyz=(0.0, 0.090, 0.216)),
        material=vent_grey,
        name="outlet_roof",
    )
    housing.visual(
        Box((0.018, 0.075, 0.116)),
        origin=Origin(xyz=(-0.239, 0.090, 0.150)),
        material=vent_grey,
        name="outlet_left_jamb",
    )
    housing.visual(
        Box((0.018, 0.075, 0.116)),
        origin=Origin(xyz=(0.239, 0.090, 0.150)),
        material=vent_grey,
        name="outlet_right_jamb",
    )
    housing.visual(
        Box((0.500, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.158, 0.208)),
        material=vent_grey,
        name="flap_hinge_seat",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.620, 0.160, 0.380)),
        mass=12.5,
        origin=Origin(xyz=(0.0, 0.080, 0.190)),
    )

    model.articulation(
        "wall_plate_to_housing",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=housing,
        origin=Origin(xyz=(0.0, 0.020, 0.020)),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.004, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow_grey,
        name="shaft",
    )
    control_knob.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_grey,
        name="knob_body",
    )
    control_knob.visual(
        Box((0.005, 0.006, 0.010)),
        origin=Origin(xyz=(0.012, 0.030, 0.0)),
        material=plate_white,
        name="pointer_ridge",
    )
    control_knob.inertial = Inertial.from_geometry(
        Box((0.040, 0.032, 0.040)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.016, 0.0)),
    )
    model.articulation(
        "housing_to_control_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=control_knob,
        origin=Origin(xyz=(0.236, 0.160, 0.318)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    outlet_flap = model.part("outlet_flap")
    outlet_flap.visual(
        Cylinder(radius=0.005, length=0.500),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=vent_grey,
        name="hinge_barrel",
    )
    outlet_flap.visual(
        Box((0.500, 0.006, 0.082)),
        origin=Origin(xyz=(0.0, 0.006, -0.041)),
        material=vane_grey,
        name="flap_panel",
    )
    outlet_flap.inertial = Inertial.from_geometry(
        Box((0.500, 0.012, 0.082)),
        mass=0.36,
        origin=Origin(xyz=(0.0, 0.006, -0.041)),
    )
    model.articulation(
        "housing_to_outlet_flap",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=outlet_flap,
        origin=Origin(xyz=(0.0, 0.160, 0.208)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    vane_x_positions = (-0.176, -0.088, 0.0, 0.088, 0.176)
    for index, vane_x in enumerate(vane_x_positions):
        vane = model.part(f"guide_vane_{index}")
        vane.visual(
            Box((0.007, 0.046, 0.094)),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=vane_grey,
            name="vane_blade",
        )
        vane.visual(
            Cylinder(radius=0.003, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.053)),
            material=shadow_grey,
            name="top_pivot",
        )
        vane.visual(
            Cylinder(radius=0.003, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.053)),
            material=shadow_grey,
            name="bottom_pivot",
        )
        vane.inertial = Inertial.from_geometry(
            Box((0.014, 0.050, 0.118)),
            mass=0.05,
            origin=Origin(),
        )
        model.articulation(
            f"housing_to_guide_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=vane,
            origin=Origin(xyz=(vane_x, 0.083, 0.150)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.15,
                velocity=2.0,
                lower=-0.65,
                upper=0.65,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_plate = object_model.get_part("wall_plate")
    housing = object_model.get_part("housing")
    control_knob = object_model.get_part("control_knob")
    outlet_flap = object_model.get_part("outlet_flap")
    first_vane = object_model.get_part("guide_vane_0")

    knob_joint = object_model.get_articulation("housing_to_control_knob")
    flap_joint = object_model.get_articulation("housing_to_outlet_flap")
    first_vane_joint = object_model.get_articulation("housing_to_guide_vane_0")

    def _span(aabb, axis: int) -> float:
        return aabb[1][axis] - aabb[0][axis]

    def _center(aabb, axis: int) -> float:
        return 0.5 * (aabb[0][axis] + aabb[1][axis])

    ctx.expect_gap(
        housing,
        wall_plate,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        name="housing mounts flush to the wall plate",
    )
    ctx.expect_gap(
        outlet_flap,
        first_vane,
        axis="y",
        positive_elem="flap_panel",
        negative_elem="vane_blade",
        min_gap=0.040,
        max_gap=0.090,
        name="broad flap sits in front of the guide vanes",
    )
    ctx.expect_overlap(
        outlet_flap,
        housing,
        axes="x",
        elem_a="flap_panel",
        elem_b="front_bezel",
        min_overlap=0.48,
        name="flap spans the outlet width",
    )

    vane_joint_names = [f"housing_to_guide_vane_{index}" for index in range(5)]
    ctx.check(
        "all guide vanes are independently articulated",
        all(
            object_model.get_articulation(name).articulation_type == ArticulationType.REVOLUTE
            for name in vane_joint_names
        ),
        details=str(vane_joint_names),
    )

    rest_flap_aabb = ctx.part_element_world_aabb(outlet_flap, elem="flap_panel")
    with ctx.pose({flap_joint: 0.75}):
        open_flap_aabb = ctx.part_element_world_aabb(outlet_flap, elem="flap_panel")
    ctx.check(
        "outlet flap opens outward",
        rest_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > rest_flap_aabb[1][1] + 0.030,
        details=f"rest={rest_flap_aabb}, open={open_flap_aabb}",
    )

    rest_vane_aabb = ctx.part_element_world_aabb(first_vane, elem="vane_blade")
    with ctx.pose({first_vane_joint: 0.55}):
        turned_vane_aabb = ctx.part_element_world_aabb(first_vane, elem="vane_blade")
    ctx.check(
        "guide vane yaws about a vertical pivot",
        rest_vane_aabb is not None
        and turned_vane_aabb is not None
        and _span(turned_vane_aabb, 0) > _span(rest_vane_aabb, 0) + 0.015,
        details=f"rest={rest_vane_aabb}, turned={turned_vane_aabb}",
    )

    rest_pointer_aabb = ctx.part_element_world_aabb(control_knob, elem="pointer_ridge")
    with ctx.pose({knob_joint: 1.20}):
        turned_pointer_aabb = ctx.part_element_world_aabb(control_knob, elem="pointer_ridge")
    ctx.check(
        "control knob visibly rotates on its shaft",
        rest_pointer_aabb is not None
        and turned_pointer_aabb is not None
        and (
            abs(_center(turned_pointer_aabb, 0) - _center(rest_pointer_aabb, 0)) > 0.004
            or abs(_center(turned_pointer_aabb, 2) - _center(rest_pointer_aabb, 2)) > 0.004
        ),
        details=f"rest={rest_pointer_aabb}, turned={turned_pointer_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
