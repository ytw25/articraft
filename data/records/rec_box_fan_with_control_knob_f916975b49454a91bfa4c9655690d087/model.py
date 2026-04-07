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
    FanRotorGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 40,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _face_with_two_round_openings(
    *,
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    opening_radius: float,
    opening_center_x: float,
    opening_center_z: float,
    z_center: float,
):
    outer = rounded_rect_profile(width, height, corner_radius, corner_segments=8)
    holes = [
        _circle_profile(
            opening_radius,
            center=(-opening_center_x, opening_center_z - z_center),
            segments=40,
        ),
        _circle_profile(
            opening_radius,
            center=(opening_center_x, opening_center_z - z_center),
            segments=40,
        ),
    ]
    return ExtrudeWithHolesGeometry(outer, holes, thickness, cap=True, center=True, closed=True)


def _add_vertical_grille_bars(
    part,
    *,
    prefix: str,
    fan_center_x: float,
    fan_center_z: float,
    opening_radius: float,
    bar_depth: float,
    bar_width: float,
    count: int,
    material,
    local_y: float,
    z_shift: float = 0.0,
) -> None:
    usable_half_span = opening_radius * 0.82
    if count == 1:
        offsets = [0.0]
    else:
        offsets = [
            -usable_half_span + (2.0 * usable_half_span * index) / (count - 1)
            for index in range(count)
        ]
    for index, offset in enumerate(offsets):
        part.visual(
            Box((bar_width, bar_depth, opening_radius * 2.0 + 0.018)),
            origin=Origin(xyz=(fan_center_x + offset, local_y, fan_center_z + z_shift)),
            material=material,
            name=f"{prefix}_{index:02d}",
        )


def _add_horizontal_grille_bars(
    part,
    *,
    prefix: str,
    fan_center_x: float,
    fan_center_z: float,
    opening_radius: float,
    bar_depth: float,
    bar_height: float,
    count: int,
    material,
    local_y: float,
) -> None:
    usable_half_span = opening_radius * 0.74
    if count == 1:
        offsets = [0.0]
    else:
        offsets = [
            -usable_half_span + (2.0 * usable_half_span * index) / (count - 1)
            for index in range(count)
        ]
    for index, offset in enumerate(offsets):
        span = 2.0 * math.sqrt(max(opening_radius * opening_radius - offset * offset, 0.0))
        part.visual(
            Box((max(span - 0.014, 0.030), bar_depth, bar_height)),
            origin=Origin(xyz=(fan_center_x, local_y, fan_center_z + offset)),
            material=material,
            name=f"{prefix}_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_window_box_fan")

    width = 0.68
    depth = 0.18
    height = 0.38
    wall = 0.024
    bezel_thickness = 0.014
    door_thickness = 0.012
    top_band_height = 0.075
    opening_radius = 0.112
    center_divider = 0.026
    fan_center_x = opening_radius + center_divider * 0.5
    fan_center_z = 0.172
    front_bezel_y = -depth * 0.5 + bezel_thickness * 0.5
    rear_bezel_y = depth * 0.5 - bezel_thickness * 0.5
    door_bottom_z = 0.020
    door_top_z = height - top_band_height
    door_height = door_top_z - door_bottom_z
    door_y = -depth * 0.5 - door_thickness * 0.5
    rotor_axis_y = -0.010
    motor_center_y = 0.046
    motor_support_center_y = (rear_bezel_y + motor_center_y) * 0.5
    motor_support_depth = rear_bezel_y - motor_center_y + 0.020

    housing_white = model.material("housing_white", rgba=(0.90, 0.92, 0.94, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.74, 0.77, 0.80, 1.0))
    grille_gray = model.material("grille_gray", rgba=(0.63, 0.67, 0.71, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.33, 0.35, 0.38, 1.0))
    blade_blue = model.material("blade_blue", rgba=(0.58, 0.78, 0.92, 0.88))

    full_face_mesh = mesh_from_geometry(
        _face_with_two_round_openings(
            width=width,
            height=height,
            thickness=bezel_thickness,
            corner_radius=0.020,
            opening_radius=opening_radius,
            opening_center_x=fan_center_x,
            opening_center_z=fan_center_z,
            z_center=height * 0.5,
        ),
        "box_fan_full_face",
    )
    door_frame_mesh = mesh_from_geometry(
        _face_with_two_round_openings(
            width=width - 0.018,
            height=door_height,
            thickness=door_thickness,
            corner_radius=0.016,
            opening_radius=opening_radius,
            opening_center_x=fan_center_x,
            opening_center_z=fan_center_z,
            z_center=door_bottom_z + door_height * 0.5,
        ),
        "box_fan_door_frame",
    )
    rotor_mesh = mesh_from_geometry(
        FanRotorGeometry(
            opening_radius * 0.90,
            0.026,
            5,
            thickness=0.018,
            blade_pitch_deg=26.0,
            blade_sweep_deg=16.0,
            center=True,
        ),
        "box_fan_rotor",
    )

    housing = model.part("housing")
    housing.visual(
        full_face_mesh,
        origin=Origin(xyz=(0.0, front_bezel_y, height * 0.5), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_white,
        name="front_face",
    )
    housing.visual(
        full_face_mesh,
        origin=Origin(xyz=(0.0, rear_bezel_y, height * 0.5), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_white,
        name="rear_face",
    )
    housing.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(-width * 0.5 + wall * 0.5, 0.0, height * 0.5)),
        material=housing_white,
        name="left_side_shell",
    )
    housing.visual(
        Box((wall, depth, height)),
        origin=Origin(xyz=(width * 0.5 - wall * 0.5, 0.0, height * 0.5)),
        material=housing_white,
        name="right_side_shell",
    )
    housing.visual(
        Box((width - 2.0 * wall, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, wall * 0.5)),
        material=housing_white,
        name="bottom_shell",
    )
    housing.visual(
        Box((width - 2.0 * wall, depth, wall)),
        origin=Origin(xyz=(0.0, 0.0, height - wall * 0.5)),
        material=housing_white,
        name="top_shell",
    )
    housing.visual(
        Box((center_divider, depth, height - 2.0 * wall)),
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
        material=housing_white,
        name="center_divider_shell",
    )
    housing.visual(
        Box((0.182, 0.024, 0.060)),
        origin=Origin(xyz=(0.0, -depth * 0.5 + 0.015, height - top_band_height * 0.5)),
        material=trim_gray,
        name="control_panel",
    )
    housing.visual(
        Box((0.120, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -depth * 0.5 + 0.024, height - top_band_height + 0.020)),
        material=grille_gray,
        name="speed_scale",
    )
    housing.visual(
        Box((0.090, depth * 0.62, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, height - top_band_height - 0.004)),
        material=trim_gray,
        name="upper_cross_shelf",
    )
    housing.visual(
        Box((0.060, 0.028, 0.012)),
        origin=Origin(xyz=(-width * 0.5 + 0.070, 0.0, 0.006)),
        material=trim_gray,
        name="left_foot",
    )
    housing.visual(
        Box((0.060, 0.028, 0.012)),
        origin=Origin(xyz=(width * 0.5 - 0.070, 0.0, 0.006)),
        material=trim_gray,
        name="right_foot",
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        x_center = side_sign * fan_center_x
        housing.visual(
            Cylinder(radius=0.034, length=0.058),
            origin=Origin(
                xyz=(x_center, motor_center_y, fan_center_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=motor_gray,
            name=f"{side_name}_motor_can",
        )
        housing.visual(
            Cylinder(radius=0.006, length=0.016),
            origin=Origin(
                xyz=(x_center, 0.017, fan_center_z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_plastic,
            name=f"{side_name}_motor_shaft",
        )
        housing.visual(
            Box((opening_radius * 1.76, motor_support_depth, 0.012)),
            origin=Origin(xyz=(x_center, motor_support_center_y, fan_center_z)),
            material=motor_gray,
            name=f"{side_name}_motor_brace_horizontal",
        )
        housing.visual(
            Box((0.012, motor_support_depth, opening_radius * 1.76)),
            origin=Origin(xyz=(x_center, motor_support_center_y, fan_center_z)),
            material=motor_gray,
            name=f"{side_name}_motor_brace_vertical",
        )
        _add_vertical_grille_bars(
            housing,
            prefix=f"{side_name}_rear_bar",
            fan_center_x=x_center,
            fan_center_z=fan_center_z,
            opening_radius=opening_radius,
            bar_depth=0.006,
            bar_width=0.006,
            count=7,
            material=grille_gray,
            local_y=rear_bezel_y,
        )
        _add_horizontal_grille_bars(
            housing,
            prefix=f"{side_name}_rear_crossbar",
            fan_center_x=x_center,
            fan_center_z=fan_center_z,
            opening_radius=opening_radius,
            bar_depth=0.006,
            bar_height=0.006,
            count=5,
            material=grille_gray,
            local_y=rear_bezel_y,
        )

    housing.inertial = Inertial.from_geometry(
        Box((width, depth, height)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, height * 0.5)),
    )

    front_grille = model.part("front_grille")
    front_grille.visual(
        door_frame_mesh,
        origin=Origin(
            xyz=(0.0, 0.0, -door_height * 0.5),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=grille_gray,
        name="grille_frame",
    )
    _add_vertical_grille_bars(
        front_grille,
        prefix="left_front_bar",
        fan_center_x=-fan_center_x,
        fan_center_z=fan_center_z - door_top_z,
        opening_radius=opening_radius,
        bar_depth=door_thickness,
        bar_width=0.006,
        count=8,
        material=grille_gray,
        local_y=0.0,
    )
    _add_horizontal_grille_bars(
        front_grille,
        prefix="left_front_crossbar",
        fan_center_x=-fan_center_x,
        fan_center_z=fan_center_z - door_top_z,
        opening_radius=opening_radius,
        bar_depth=door_thickness,
        bar_height=0.006,
        count=5,
        material=grille_gray,
        local_y=0.0,
    )
    _add_vertical_grille_bars(
        front_grille,
        prefix="right_front_bar",
        fan_center_x=fan_center_x,
        fan_center_z=fan_center_z - door_top_z,
        opening_radius=opening_radius,
        bar_depth=door_thickness,
        bar_width=0.006,
        count=8,
        material=grille_gray,
        local_y=0.0,
    )
    _add_horizontal_grille_bars(
        front_grille,
        prefix="right_front_crossbar",
        fan_center_x=fan_center_x,
        fan_center_z=fan_center_z - door_top_z,
        opening_radius=opening_radius,
        bar_depth=door_thickness,
        bar_height=0.006,
        count=5,
        material=grille_gray,
        local_y=0.0,
    )
    front_grille.visual(
        Box((0.100, door_thickness, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=trim_gray,
        name="grille_pull",
    )
    front_grille.inertial = Inertial.from_geometry(
        Box((width - 0.018, door_thickness, door_height)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, -door_height * 0.5)),
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        rotor = model.part(f"{side_name}_rotor")
        rotor.visual(
            rotor_mesh,
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=blade_blue,
            name="fan_rotor",
        )
        rotor.visual(
            Cylinder(radius=0.013, length=0.038),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_plastic,
            name="hub_cap",
        )
        rotor.inertial = Inertial.from_geometry(
            Cylinder(radius=opening_radius * 0.90, length=0.024),
            mass=0.18,
        )
        model.articulation(
            f"{side_name}_motor_spin",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=rotor,
            origin=Origin(xyz=(side_sign * fan_center_x, rotor_axis_y, fan_center_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=32.0),
        )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="knob_body",
    )
    speed_knob.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="knob_cap",
    )
    speed_knob.visual(
        Box((0.004, 0.008, 0.020)),
        origin=Origin(xyz=(0.017, -0.010, 0.0)),
        material=trim_gray,
        name="knob_pointer",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.020),
        mass=0.08,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "front_grille_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_grille,
        origin=Origin(xyz=(0.0, door_y, door_top_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=1.8,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "speed_knob_turn",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=speed_knob,
        origin=Origin(xyz=(0.0, -depth * 0.5 - 0.010, height - top_band_height * 0.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    front_grille = object_model.get_part("front_grille")
    speed_knob = object_model.get_part("speed_knob")
    left_rotor = object_model.get_part("left_rotor")
    right_rotor = object_model.get_part("right_rotor")

    grille_hinge = object_model.get_articulation("front_grille_hinge")

    ctx.expect_overlap(
        front_grille,
        housing,
        axes="xz",
        min_overlap=0.28,
        name="front grille covers the twin fan opening area",
    )
    ctx.expect_gap(
        housing,
        front_grille,
        axis="y",
        max_gap=0.020,
        max_penetration=0.0001,
        name="closed grille sits just in front of the housing face",
    )
    ctx.expect_origin_distance(
        left_rotor,
        right_rotor,
        axes="x",
        min_dist=0.20,
        max_dist=0.30,
        name="two rotor stages are clearly separated across the housing width",
    )

    closed_grille_aabb = ctx.part_world_aabb(front_grille)
    knob_pos = ctx.part_world_position(speed_knob)
    with ctx.pose({grille_hinge: 1.05}):
        open_grille_aabb = ctx.part_world_aabb(front_grille)

    ctx.check(
        "grille lifts upward when opened",
        closed_grille_aabb is not None
        and open_grille_aabb is not None
        and open_grille_aabb[0][2] > closed_grille_aabb[0][2] + 0.10
        and open_grille_aabb[0][1] < closed_grille_aabb[0][1] - 0.08,
        details=f"closed_aabb={closed_grille_aabb}, open_aabb={open_grille_aabb}",
    )
    ctx.check(
        "speed knob stays on the exposed top center panel",
        knob_pos is not None
        and abs(knob_pos[0]) < 0.01
        and knob_pos[2] > 0.29,
        details=f"knob_pos={knob_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
