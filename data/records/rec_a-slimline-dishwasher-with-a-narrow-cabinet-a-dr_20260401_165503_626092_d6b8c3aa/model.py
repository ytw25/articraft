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
    model = ArticulatedObject(name="slimline_dishwasher")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))
    tub_grey = model.material("tub_grey", rgba=(0.80, 0.82, 0.84, 1.0))
    rack_grey = model.material("rack_grey", rgba=(0.66, 0.68, 0.70, 1.0))
    roller_grey = model.material("roller_grey", rgba=(0.22, 0.24, 0.27, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.012, 0.595, 0.845)),
        origin=Origin(xyz=(-0.218, 0.035, 0.4225)),
        material=stainless,
        name="left_side",
    )
    cabinet.visual(
        Box((0.012, 0.595, 0.845)),
        origin=Origin(xyz=(0.218, 0.035, 0.4225)),
        material=stainless,
        name="right_side",
    )
    cabinet.visual(
        Box((0.424, 0.595, 0.012)),
        origin=Origin(xyz=(0.0, 0.035, 0.839)),
        material=stainless,
        name="top_shell",
    )
    cabinet.visual(
        Box((0.424, 0.012, 0.833)),
        origin=Origin(xyz=(0.0, 0.3265, 0.4165)),
        material=stainless,
        name="back_shell",
    )
    cabinet.visual(
        Box((0.424, 0.510, 0.012)),
        origin=Origin(xyz=(0.0, 0.030, 0.105)),
        material=tub_grey,
        name="tub_floor",
    )
    cabinet.visual(
        Box((0.448, 0.045, 0.075)),
        origin=Origin(xyz=(0.0, -0.2325, 0.0375)),
        material=dark_trim,
        name="toe_kick",
    )
    cabinet.visual(
        Box((0.018, 0.030, 0.708)),
        origin=Origin(xyz=(-0.203, -0.262, 0.472)),
        material=tub_grey,
        name="left_lip",
    )
    cabinet.visual(
        Box((0.018, 0.030, 0.708)),
        origin=Origin(xyz=(0.203, -0.262, 0.472)),
        material=tub_grey,
        name="right_lip",
    )
    cabinet.visual(
        Box((0.424, 0.030, 0.067)),
        origin=Origin(xyz=(0.0, -0.262, 0.8055)),
        material=tub_grey,
        name="top_lip",
    )
    cabinet.visual(
        Box((0.424, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, -0.259, 0.110)),
        material=tub_grey,
        name="bottom_sill",
    )
    cabinet.visual(
        Box((0.040, 0.300, 0.012)),
        origin=Origin(xyz=(-0.192, -0.035, 0.596)),
        material=tub_grey,
        name="left_upper_rail",
    )
    cabinet.visual(
        Box((0.040, 0.300, 0.012)),
        origin=Origin(xyz=(0.192, -0.035, 0.596)),
        material=tub_grey,
        name="right_upper_rail",
    )
    cabinet.visual(
        Box((0.040, 0.435, 0.020)),
        origin=Origin(xyz=(-0.192, 0.032, 0.246)),
        material=tub_grey,
        name="left_lower_rail",
    )
    cabinet.visual(
        Box((0.040, 0.435, 0.020)),
        origin=Origin(xyz=(0.192, 0.032, 0.246)),
        material=tub_grey,
        name="right_lower_rail",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.448, 0.595, 0.845)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.4225)),
    )

    door = model.part("door")
    door.visual(
        Box((0.442, 0.020, 0.730)),
        origin=Origin(xyz=(0.0, -0.010, 0.365)),
        material=stainless,
        name="outer_panel",
    )
    door.visual(
        Box((0.404, 0.010, 0.705)),
        origin=Origin(xyz=(0.0, 0.005, 0.3525)),
        material=tub_grey,
        name="inner_liner",
    )
    door.visual(
        Box((0.442, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.001, 0.020)),
        material=dark_trim,
        name="bottom_edge",
    )
    door.visual(
        Box((0.260, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, -0.024, 0.700)),
        material=dark_trim,
        name="pull_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.442, 0.032, 0.730)),
        mass=6.0,
        origin=Origin(xyz=(0.0, -0.008, 0.365)),
    )

    upper_tray = model.part("upper_tray")
    upper_tray.visual(
        Box((0.378, 0.392, 0.006)),
        origin=Origin(xyz=(0.0, 0.196, 0.003)),
        material=rack_grey,
        name="tray_floor",
    )
    upper_tray.visual(
        Box((0.010, 0.392, 0.030)),
        origin=Origin(xyz=(-0.184, 0.196, 0.018)),
        material=rack_grey,
        name="left_wall",
    )
    upper_tray.visual(
        Box((0.010, 0.392, 0.030)),
        origin=Origin(xyz=(0.184, 0.196, 0.018)),
        material=rack_grey,
        name="right_wall",
    )
    upper_tray.visual(
        Box((0.378, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.387, 0.017)),
        material=rack_grey,
        name="rear_lip",
    )
    upper_tray.visual(
        Box((0.300, 0.016, 0.038)),
        origin=Origin(xyz=(0.0, -0.004, 0.020)),
        material=rack_grey,
        name="front_lip",
    )
    upper_tray.visual(
        Box((0.020, 0.280, 0.010)),
        origin=Origin(xyz=(-0.184, 0.140, 0.004)),
        material=roller_grey,
        name="left_slider",
    )
    upper_tray.visual(
        Box((0.020, 0.280, 0.010)),
        origin=Origin(xyz=(0.184, 0.140, 0.004)),
        material=roller_grey,
        name="right_slider",
    )
    for index, x_pos in enumerate((-0.110, -0.040, 0.040, 0.110), start=1):
        upper_tray.visual(
            Box((0.004, 0.140, 0.018)),
            origin=Origin(xyz=(x_pos, 0.260, 0.015)),
            material=rack_grey,
            name=f"divider_{index}",
        )
    upper_tray.inertial = Inertial.from_geometry(
        Box((0.390, 0.400, 0.045)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.196, 0.0225)),
    )

    lower_rack = model.part("lower_rack")
    lower_rack.visual(
        Box((0.390, 0.445, 0.008)),
        origin=Origin(xyz=(0.0, 0.223, 0.050)),
        material=rack_grey,
        name="base_floor",
    )
    lower_rack.visual(
        Box((0.010, 0.445, 0.110)),
        origin=Origin(xyz=(-0.190, 0.223, 0.067)),
        material=rack_grey,
        name="left_side",
    )
    lower_rack.visual(
        Box((0.010, 0.445, 0.110)),
        origin=Origin(xyz=(0.190, 0.223, 0.067)),
        material=rack_grey,
        name="right_side",
    )
    lower_rack.visual(
        Box((0.390, 0.010, 0.130)),
        origin=Origin(xyz=(0.0, 0.440, 0.075)),
        material=rack_grey,
        name="rear_wall",
    )
    lower_rack.visual(
        Box((0.390, 0.010, 0.082)),
        origin=Origin(xyz=(0.0, 0.005, 0.051)),
        material=rack_grey,
        name="front_wall",
    )
    lower_rack.visual(
        Box((0.022, 0.395, 0.014)),
        origin=Origin(xyz=(-0.184, 0.198, 0.018)),
        material=roller_grey,
        name="left_runner",
    )
    lower_rack.visual(
        Box((0.022, 0.395, 0.014)),
        origin=Origin(xyz=(0.184, 0.198, 0.018)),
        material=roller_grey,
        name="right_runner",
    )
    for index, x_pos in enumerate((-0.110, -0.035, 0.035, 0.110), start=1):
        lower_rack.visual(
            Box((0.004, 0.180, 0.108)),
            origin=Origin(xyz=(x_pos, 0.160, 0.108)),
            material=rack_grey,
            name=f"plate_divider_{index}",
        )
    wheel_specs = (
        ("front_left_wheel", -0.184, 0.060),
        ("mid_left_wheel", -0.184, 0.220),
        ("rear_left_wheel", -0.184, 0.380),
        ("front_right_wheel", 0.184, 0.060),
        ("mid_right_wheel", 0.184, 0.220),
        ("rear_right_wheel", 0.184, 0.380),
    )
    for name, x_pos, y_pos in wheel_specs:
        lower_rack.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(xyz=(x_pos, y_pos, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=roller_grey,
            name=name,
        )
    lower_rack.inertial = Inertial.from_geometry(
        Box((0.404, 0.455, 0.140)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.223, 0.070)),
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, -0.286, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "cabinet_to_upper_tray",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_tray,
        origin=Origin(xyz=(0.0, -0.215, 0.603)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.30,
            lower=0.0,
            upper=0.170,
        ),
    )
    model.articulation(
        "cabinet_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_rack,
        origin=Origin(xyz=(0.0, -0.238, 0.256)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=0.35,
            lower=0.0,
            upper=0.275,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    upper_tray = object_model.get_part("upper_tray")
    lower_rack = object_model.get_part("lower_rack")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    upper_slide = object_model.get_articulation("cabinet_to_upper_tray")
    lower_slide = object_model.get_articulation("cabinet_to_lower_rack")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="top_lip",
        negative_elem="outer_panel",
        min_gap=0.004,
        max_gap=0.028,
        name="door sits slightly proud of the front opening",
    )
    ctx.expect_gap(
        upper_tray,
        lower_rack,
        axis="z",
        min_gap=0.180,
        name="upper tray stays clearly above the lower rack",
    )
    ctx.expect_contact(
        upper_tray,
        cabinet,
        elem_a="left_slider",
        elem_b="left_upper_rail",
        name="upper tray slider bears on the left short rail",
    )
    ctx.expect_overlap(
        upper_tray,
        cabinet,
        axes="y",
        elem_a="left_slider",
        elem_b="left_upper_rail",
        min_overlap=0.200,
        name="upper tray remains well inserted at rest",
    )
    ctx.expect_gap(
        lower_rack,
        cabinet,
        axis="z",
        positive_elem="front_left_wheel",
        negative_elem="left_lower_rail",
        min_gap=0.0,
        max_gap=0.0025,
        name="lower rack front wheel sits on the left guide rail",
    )
    ctx.expect_overlap(
        lower_rack,
        cabinet,
        axes="y",
        elem_a="left_runner",
        elem_b="left_lower_rail",
        min_overlap=0.280,
        name="lower rack remains deeply inserted at rest",
    )

    door_open = door_hinge.motion_limits.upper or 1.55
    upper_open = upper_slide.motion_limits.upper or 0.170
    lower_open = lower_slide.motion_limits.upper or 0.275

    closed_handle = ctx.part_element_world_aabb(door, elem="pull_handle")
    with ctx.pose({door_hinge: door_open}):
        open_handle = ctx.part_element_world_aabb(door, elem="pull_handle")
        ctx.fail_if_parts_overlap_in_current_pose(name="open door pose stays clear")

    ctx.check(
        "door opens downward on its lower hinge",
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][1] < closed_handle[0][1] - 0.45
        and open_handle[1][2] < closed_handle[1][2] - 0.55,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    upper_rest = ctx.part_world_position(upper_tray)
    with ctx.pose({door_hinge: door_open, upper_slide: upper_open}):
        upper_extended = ctx.part_world_position(upper_tray)
        ctx.expect_overlap(
            upper_tray,
            cabinet,
            axes="y",
            elem_a="left_slider",
            elem_b="left_upper_rail",
            min_overlap=0.075,
            name="upper tray retains short-rail engagement at full extension",
        )
        ctx.fail_if_parts_overlap_in_current_pose(
            name="upper tray can extend with the door open"
        )

    ctx.check(
        "upper tray slides outward from the tub",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[1] < upper_rest[1] - 0.140,
        details=f"upper_rest={upper_rest}, upper_extended={upper_extended}",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    with ctx.pose({door_hinge: door_open, lower_slide: lower_open}):
        lower_extended = ctx.part_world_position(lower_rack)
        ctx.expect_overlap(
            lower_rack,
            cabinet,
            axes="y",
            elem_a="left_runner",
            elem_b="left_lower_rail",
            min_overlap=0.060,
            name="lower rack retains long-rail engagement at full extension",
        )
        ctx.fail_if_parts_overlap_in_current_pose(
            name="lower rack can extend with the door open"
        )

    ctx.check(
        "lower rack slides farther outward than the upper tray",
        lower_rest is not None
        and lower_extended is not None
        and upper_rest is not None
        and upper_extended is not None
        and lower_extended[1] < lower_rest[1] - 0.220
        and (lower_rest[1] - lower_extended[1]) > (upper_rest[1] - upper_extended[1]) + 0.070,
        details=(
            f"lower_rest={lower_rest}, lower_extended={lower_extended}, "
            f"upper_rest={upper_rest}, upper_extended={upper_extended}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
