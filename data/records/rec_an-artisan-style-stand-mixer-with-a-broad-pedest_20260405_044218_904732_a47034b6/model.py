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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_tilt_head_stand_mixer")

    enamel_red = model.material("enamel_red", rgba=(0.70, 0.08, 0.08, 1.0))
    stainless = model.material("stainless", rgba=(0.90, 0.91, 0.93, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.79, 0.82, 1.0))

    def xy_section(
        width: float,
        depth: float,
        radius: float,
        z: float,
        *,
        x_shift: float = 0.0,
        y_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x + x_shift, y + y_shift, z)
            for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
        ]

    def yz_section(
        width: float,
        height: float,
        radius: float,
        x: float,
        *,
        z_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z + z_shift)
            for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)
        ]

    base = model.part("base")

    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.34, 0.23, 0.050), 0.032),
        "mixer_base_plate",
    )
    base.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=enamel_red,
        name="base_plate",
    )

    pedestal_shell = section_loft(
        [
            xy_section(0.24, 0.17, 0.055, 0.0, x_shift=-0.025),
            xy_section(0.18, 0.145, 0.045, 0.085, x_shift=-0.045),
            xy_section(0.12, 0.11, 0.036, 0.228, x_shift=-0.080),
            xy_section(0.10, 0.09, 0.026, 0.298, x_shift=-0.090),
        ]
    )
    base.visual(
        mesh_from_geometry(pedestal_shell, "mixer_pedestal_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=enamel_red,
        name="pedestal_shell",
    )

    for index, y_center in enumerate((-0.039, 0.039)):
        base.visual(
            Box((0.040, 0.024, 0.080)),
            origin=Origin(xyz=(-0.100, y_center, 0.330)),
            material=enamel_red,
            name=f"rear_hinge_cheek_{index}",
        )
        base.visual(
            Cylinder(radius=0.026, length=0.024),
            origin=Origin(
                xyz=(-0.108, y_center, 0.370),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=chrome,
            name=f"rear_hinge_knuckle_{index}",
        )

    base.visual(
        Box((0.180, 0.120, 0.028)),
        origin=Origin(xyz=(0.120, 0.0, 0.046)),
        material=enamel_red,
        name="front_carriage_deck",
    )
    base.visual(
        Box((0.156, 0.016, 0.015)),
        origin=Origin(xyz=(0.122, 0.050, 0.0675)),
        material=chrome,
        name="deck_right_rail",
    )
    base.visual(
        Box((0.156, 0.016, 0.015)),
        origin=Origin(xyz=(0.122, -0.050, 0.0675)),
        material=chrome,
        name="deck_left_rail",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(
            xyz=(-0.030, 0.047, 0.262),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="speed_lever_boss",
    )

    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.23, 0.40)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    bowl_carriage = model.part("bowl_carriage")
    bowl_carriage.visual(
        Box((0.080, 0.090, 0.016)),
        origin=Origin(xyz=(0.040, 0.0, 0.008)),
        material=chrome,
        name="slide_plate",
    )
    bowl_carriage.visual(
        Box((0.060, 0.050, 0.026)),
        origin=Origin(xyz=(0.052, 0.0, 0.029)),
        material=chrome,
        name="center_riser",
    )
    bowl_carriage.visual(
        Cylinder(radius=0.050, length=0.012),
        origin=Origin(xyz=(0.062, 0.0, 0.048)),
        material=chrome,
        name="bowl_pad",
    )
    bowl_carriage.inertial = Inertial.from_geometry(
        Box((0.090, 0.100, 0.070)),
        mass=0.9,
        origin=Origin(xyz=(0.045, 0.0, 0.035)),
    )

    model.articulation(
        "base_to_bowl_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bowl_carriage,
        origin=Origin(xyz=(0.100, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.08,
            lower=0.0,
            upper=0.035,
        ),
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.040, 0.000),
            (0.070, 0.010),
            (0.102, 0.060),
            (0.118, 0.125),
            (0.122, 0.155),
        ],
        inner_profile=[
            (0.026, 0.004),
            (0.063, 0.014),
            (0.095, 0.060),
            (0.111, 0.124),
            (0.115, 0.151),
        ],
        segments=64,
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "mixer_bowl_shell"),
        material=stainless,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.122, length=0.155),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
    )

    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=bowl_carriage,
        child=bowl,
        origin=Origin(xyz=(0.062, 0.0, 0.054)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="head_hinge_barrel",
    )
    head.visual(
        Box((0.038, 0.054, 0.058)),
        origin=Origin(xyz=(0.028, 0.0, 0.029)),
        material=enamel_red,
        name="hinge_web",
    )

    head_shell = section_loft(
        [
            yz_section(0.092, 0.100, 0.026, 0.026, z_shift=0.068),
            yz_section(0.152, 0.168, 0.050, 0.116, z_shift=0.102),
            yz_section(0.166, 0.182, 0.055, 0.214, z_shift=0.104),
            yz_section(0.124, 0.136, 0.040, 0.308, z_shift=0.096),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "mixer_head_shell"),
        material=enamel_red,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.060),
        origin=Origin(xyz=(0.258, 0.0, 0.003)),
        material=dark_trim,
        name="drive_nose",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.33, 0.17, 0.19)),
        mass=4.6,
        origin=Origin(xyz=(0.165, 0.0, 0.005)),
    )

    whisk = model.part("whisk")
    whisk.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=chrome,
        name="whisk_coupler",
    )
    whisk.visual(
        Cylinder(radius=0.0045, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=chrome,
        name="whisk_shaft",
    )
    whisk.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.126)),
        material=chrome,
        name="whisk_tip",
    )
    whisk_wire_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.008, 0.0, -0.004),
                (0.026, 0.0, -0.026),
                (0.040, 0.0, -0.068),
                (0.028, 0.0, -0.109),
                (0.0, 0.0, -0.128),
            ],
            radius=0.0022,
            samples_per_segment=16,
            radial_segments=12,
            cap_ends=True,
        ),
        "whisk_wire",
    )
    for wire_index in range(6):
        whisk.visual(
            whisk_wire_mesh,
            origin=Origin(rpy=(0.0, 0.0, wire_index * math.pi / 3.0)),
            material=chrome,
            name=f"whisk_wire_{wire_index}",
        )
    whisk.inertial = Inertial.from_geometry(
        Box((0.09, 0.09, 0.19)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lever_pivot",
    )
    speed_lever.visual(
        Box((0.046, 0.010, 0.008)),
        origin=Origin(xyz=(0.023, 0.011, 0.0)),
        material=chrome,
        name="lever_arm",
    )
    speed_lever.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.050, 0.011, 0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="lever_grip",
    )
    speed_lever.inertial = Inertial.from_geometry(
        Box((0.062, 0.024, 0.018)),
        mass=0.08,
        origin=Origin(xyz=(0.031, 0.011, 0.001)),
    )

    head_lock_button = model.part("head_lock_button")
    head_lock_button.visual(
        Cylinder(radius=0.0105, length=0.010),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="button_cap",
    )
    head_lock_button.visual(
        Cylinder(radius=0.0065, length=0.004),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="button_stem",
    )
    head_lock_button.inertial = Inertial.from_geometry(
        Box((0.024, 0.018, 0.024)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.108, 0.0, 0.370)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_whisk",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=whisk,
        origin=Origin(xyz=(0.258, 0.0, -0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=26.0,
        ),
    )
    model.articulation(
        "base_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(-0.030, 0.049, 0.262)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=math.radians(-22.0),
            upper=math.radians(28.0),
        ),
    )
    model.articulation(
        "base_to_head_lock_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock_button,
        origin=Origin(xyz=(-0.082, -0.049, 0.298)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.005,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    bowl_carriage = object_model.get_part("bowl_carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    whisk = object_model.get_part("whisk")
    head_lock_button = object_model.get_part("head_lock_button")

    carriage_slide = object_model.get_articulation("base_to_bowl_carriage")
    head_hinge = object_model.get_articulation("base_to_head")
    whisk_spin = object_model.get_articulation("head_to_whisk")
    speed_lever_joint = object_model.get_articulation("base_to_speed_lever")
    head_lock_joint = object_model.get_articulation("base_to_head_lock_button")

    rest_carriage_pos = ctx.part_world_position(bowl_carriage)
    with ctx.pose({carriage_slide: carriage_slide.motion_limits.upper}):
        extended_carriage_pos = ctx.part_world_position(bowl_carriage)
    ctx.check(
        "bowl carriage slides forward",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.02,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    rest_nose_aabb = ctx.part_element_world_aabb(head, elem="drive_nose")
    with ctx.pose({head_hinge: head_hinge.motion_limits.upper}):
        open_nose_aabb = ctx.part_element_world_aabb(head, elem="drive_nose")
    ctx.check(
        "tilt head opens upward",
        rest_nose_aabb is not None
        and open_nose_aabb is not None
        and ((open_nose_aabb[0][2] + open_nose_aabb[1][2]) * 0.5)
        > ((rest_nose_aabb[0][2] + rest_nose_aabb[1][2]) * 0.5) + 0.08,
        details=f"rest={rest_nose_aabb}, open={open_nose_aabb}",
    )

    ctx.expect_within(
        whisk,
        bowl,
        axes="xy",
        inner_elem="whisk_tip",
        outer_elem="bowl_shell",
        margin=0.012,
        name="whisk stays centered over the bowl",
    )
    ctx.check(
        "whisk uses continuous vertical drive",
        whisk_spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(whisk_spin.axis[2]) > 0.99,
        details=f"type={whisk_spin.articulation_type}, axis={whisk_spin.axis}",
    )

    with ctx.pose({speed_lever_joint: speed_lever_joint.motion_limits.lower}):
        slow_grip_aabb = ctx.part_element_world_aabb("speed_lever", elem="lever_grip")
    with ctx.pose({speed_lever_joint: speed_lever_joint.motion_limits.upper}):
        fast_grip_aabb = ctx.part_element_world_aabb("speed_lever", elem="lever_grip")
    ctx.check(
        "speed lever pivots upward through its arc",
        slow_grip_aabb is not None
        and fast_grip_aabb is not None
        and ((fast_grip_aabb[0][2] + fast_grip_aabb[1][2]) * 0.5)
        > ((slow_grip_aabb[0][2] + slow_grip_aabb[1][2]) * 0.5) + 0.01,
        details=f"slow={slow_grip_aabb}, fast={fast_grip_aabb}",
    )

    rest_button_pos = ctx.part_world_position(head_lock_button)
    with ctx.pose({head_lock_joint: head_lock_joint.motion_limits.upper}):
        extended_button_pos = ctx.part_world_position(head_lock_button)
    ctx.check(
        "head lock button slides outward",
        rest_button_pos is not None
        and extended_button_pos is not None
        and extended_button_pos[1] < rest_button_pos[1] - 0.003,
        details=f"rest={rest_button_pos}, extended={extended_button_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
