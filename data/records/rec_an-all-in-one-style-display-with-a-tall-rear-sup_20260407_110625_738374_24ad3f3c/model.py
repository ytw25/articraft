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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _xz_section(
    width: float,
    height: float,
    radius: float,
    y: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, z in rounded_rect_profile(width, height, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="all_in_one_display")

    stand_silver = model.material("stand_silver", rgba=(0.78, 0.79, 0.81, 1.0))
    screen_graphite = model.material("screen_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.11, 0.12, 0.13, 1.0))
    glass = model.material("glass", rgba=(0.07, 0.13, 0.17, 0.92))
    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))

    stand = model.part("stand")
    stand.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.285, 0.215, 0.055),
                0.018,
                cap=True,
                closed=True,
            ),
            "display_base_plate",
        ),
        material=stand_silver,
        name="base_plate",
    )
    stand.visual(
        Cylinder(radius=0.044, length=0.022),
        origin=Origin(xyz=(0.0, 0.050, 0.020)),
        material=stand_silver,
        name="base_neck",
    )
    stand.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _xy_section(0.082, 0.078, 0.028, 0.018, y_shift=0.050),
                    _xy_section(0.072, 0.062, 0.022, 0.140, y_shift=0.046),
                    _xy_section(0.056, 0.050, 0.018, 0.280, y_shift=0.040),
                    _xy_section(0.064, 0.054, 0.018, 0.390, y_shift=0.032),
                ]
            ),
            "display_support_arm",
        ),
        material=stand_silver,
        name="stand_arm",
    )
    stand.visual(
        Box((0.110, 0.022, 0.056)),
        origin=Origin(xyz=(0.0, 0.029, 0.405)),
        material=stand_silver,
        name="head_block",
    )
    stand.visual(
        Box((0.050, 0.022, 0.330)),
        origin=Origin(xyz=(0.0, 0.039, 0.255)),
        material=stand_silver,
        name="cable_channel_frame",
    )
    stand.inertial = Inertial.from_geometry(
        Box((0.285, 0.215, 0.455)),
        mass=6.4,
        origin=Origin(xyz=(0.0, 0.010, 0.228)),
    )

    tilt_carrier = model.part("tilt_carrier")
    tilt_carrier.visual(
        Box((0.190, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, -0.009, 0.006)),
        material=stand_silver,
        name="hinge_block",
    )
    tilt_carrier.visual(
        Box((0.018, 0.034, 0.020)),
        origin=Origin(xyz=(-0.088, -0.026, 0.0)),
        material=stand_silver,
        name="left_yoke_arm",
    )
    tilt_carrier.visual(
        Box((0.018, 0.012, 0.050)),
        origin=Origin(xyz=(-0.088, -0.046, 0.0)),
        material=stand_silver,
        name="left_yoke_pad",
    )
    tilt_carrier.visual(
        Box((0.018, 0.034, 0.020)),
        origin=Origin(xyz=(0.088, -0.026, 0.0)),
        material=stand_silver,
        name="right_yoke_arm",
    )
    tilt_carrier.visual(
        Box((0.018, 0.012, 0.050)),
        origin=Origin(xyz=(0.088, -0.046, 0.0)),
        material=stand_silver,
        name="right_yoke_pad",
    )
    tilt_carrier.visual(
        Box((0.016, 0.048, 0.018)),
        origin=Origin(xyz=(-0.070, -0.042, 0.0)),
        material=stand_silver,
        name="left_adapter_rib",
    )
    tilt_carrier.visual(
        Box((0.016, 0.048, 0.018)),
        origin=Origin(xyz=(0.070, -0.042, 0.0)),
        material=stand_silver,
        name="right_adapter_rib",
    )
    tilt_carrier.visual(
        Box((0.136, 0.002, 0.100)),
        origin=Origin(xyz=(0.0, -0.066, 0.0)),
        material=dark_trim,
        name="portrait_adapter_plate",
    )
    tilt_carrier.inertial = Inertial.from_geometry(
        Box((0.190, 0.086, 0.100)),
        mass=0.7,
        origin=Origin(xyz=(0.0, -0.034, 0.0)),
    )

    screen = model.part("screen")
    screen.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _xz_section(0.620, 0.368, 0.028, -0.020),
                    _xz_section(0.612, 0.360, 0.024, 0.020),
                ]
            ),
            "display_screen_shell",
        ),
        origin=Origin(xyz=(0.0, -0.025, 0.0)),
        material=screen_graphite,
        name="screen_shell",
    )
    screen.visual(
        Box((0.592, 0.004, 0.334)),
        origin=Origin(xyz=(0.0, -0.044, 0.0)),
        material=glass,
        name="display_glass",
    )
    screen.visual(
        Box((0.132, 0.014, 0.094)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=dark_trim,
        name="rear_mount_plate",
    )
    screen.visual(
        Cylinder(radius=0.048, length=0.022),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="rear_spindle_hub",
    )
    screen.inertial = Inertial.from_geometry(
        Box((0.620, 0.052, 0.368)),
        mass=5.2,
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
    )

    cable_cover = model.part("cable_cover")
    cable_cover.visual(
        Box((0.046, 0.0035, 0.198)),
        origin=Origin(xyz=(0.0, 0.00175, -0.099)),
        material=stand_silver,
        name="cover_panel",
    )
    cable_cover.visual(
        Box((0.030, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.006, -0.196)),
        material=stand_silver,
        name="cover_finger_lip",
    )
    cable_cover.inertial = Inertial.from_geometry(
        Box((0.046, 0.014, 0.206)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.006, -0.100)),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="knob_shaft",
    )
    control_knob.visual(
        Cylinder(radius=0.010, length=0.013),
        origin=Origin(xyz=(0.0165, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="knob_body",
    )
    control_knob.inertial = Inertial.from_geometry(
        Box((0.032, 0.024, 0.024)),
        mass=0.025,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
    )

    model.articulation(
        "stand_to_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=tilt_carrier,
        origin=Origin(xyz=(0.0, 0.018, 0.405)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=math.radians(-18.0),
            upper=math.radians(32.0),
        ),
    )
    model.articulation(
        "tilt_to_screen",
        ArticulationType.REVOLUTE,
        parent=tilt_carrier,
        child=screen,
        origin=Origin(xyz=(0.0, -0.068, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "stand_to_cable_cover",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=cable_cover,
        origin=Origin(xyz=(0.0, 0.050, 0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )
    model.articulation(
        "screen_to_control_knob",
        ArticulationType.CONTINUOUS,
        parent=screen,
        child=control_knob,
        origin=Origin(xyz=(0.308, -0.025, -0.145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    stand = object_model.get_part("stand")
    screen = object_model.get_part("screen")
    cable_cover = object_model.get_part("cable_cover")
    control_knob = object_model.get_part("control_knob")

    tilt = object_model.get_articulation("stand_to_tilt")
    portrait = object_model.get_articulation("tilt_to_screen")
    cover_hinge = object_model.get_articulation("stand_to_cable_cover")
    knob_joint = object_model.get_articulation("screen_to_control_knob")

    def _dims_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    with ctx.pose({tilt: 0.0, portrait: 0.0, cover_hinge: 0.0}):
        ctx.expect_gap(
            screen,
            stand,
            axis="z",
            min_gap=0.19,
            positive_elem="screen_shell",
            negative_elem="base_plate",
            name="screen clears the low base",
        )
        ctx.expect_overlap(
            screen,
            stand,
            axes="x",
            min_overlap=0.24,
            elem_a="screen_shell",
            elem_b="base_plate",
            name="screen stays centered over the base footprint",
        )
        ctx.expect_gap(
            cable_cover,
            stand,
            axis="y",
            min_gap=0.0,
            max_gap=0.004,
            positive_elem="cover_panel",
            negative_elem="cable_channel_frame",
            name="cable cover closes nearly flush to the stand arm",
        )
        ctx.expect_origin_gap(
            control_knob,
            screen,
            axis="x",
            min_gap=0.295,
            max_gap=0.325,
            name="control knob sits on the right side edge of the display",
        )

    with ctx.pose({tilt: 0.0, portrait: 0.0}):
        landscape_aabb = ctx.part_world_aabb(screen)
        rest_glass = ctx.part_element_world_aabb(screen, elem="display_glass")
    with ctx.pose({tilt: 0.0, portrait: math.pi / 2.0}):
        portrait_aabb = ctx.part_world_aabb(screen)
    with ctx.pose({tilt: tilt.motion_limits.upper, portrait: 0.0}):
        tilted_glass = ctx.part_element_world_aabb(screen, elem="display_glass")
    with ctx.pose({cover_hinge: cover_hinge.motion_limits.upper}):
        open_cover = ctx.part_world_aabb(cable_cover)
    with ctx.pose({cover_hinge: 0.0}):
        closed_cover = ctx.part_world_aabb(cable_cover)

    landscape_dims = _dims_from_aabb(landscape_aabb)
    portrait_dims = _dims_from_aabb(portrait_aabb)
    ctx.check(
        "portrait joint swaps the screen aspect",
        landscape_dims is not None
        and portrait_dims is not None
        and landscape_dims[0] > landscape_dims[2]
        and portrait_dims[2] > portrait_dims[0],
        details=f"landscape_dims={landscape_dims}, portrait_dims={portrait_dims}",
    )

    ctx.check(
        "tilt hinge tips the top of the screen backward",
        rest_glass is not None
        and tilted_glass is not None
        and tilted_glass[1][1] > rest_glass[1][1] + 0.04,
        details=f"rest_glass={rest_glass}, tilted_glass={tilted_glass}",
    )

    ctx.check(
        "cable cover flips down away from the arm",
        closed_cover is not None
        and open_cover is not None
        and open_cover[1][1] > closed_cover[1][1] + 0.10
        and open_cover[1][2] > closed_cover[1][2] + 0.03,
        details=f"closed_cover={closed_cover}, open_cover={open_cover}",
    )

    ctx.check(
        "articulation axes match the intended mechanisms",
        tuple(tilt.axis) == (1.0, 0.0, 0.0)
        and tuple(portrait.axis) == (0.0, 0.0, 1.0)
        and tuple(cover_hinge.axis) == (1.0, 0.0, 0.0)
        and tuple(knob_joint.axis) == (1.0, 0.0, 0.0)
        and knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=(
            f"tilt_axis={tilt.axis}, portrait_axis={portrait.axis}, "
            f"cover_axis={cover_hinge.axis}, knob_axis={knob_joint.axis}, "
            f"knob_type={knob_joint.articulation_type}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
