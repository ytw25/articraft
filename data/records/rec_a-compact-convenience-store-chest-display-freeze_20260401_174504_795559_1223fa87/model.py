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


def _add_rounded_prism(
    part,
    *,
    prefix: str,
    size_x: float,
    size_y: float,
    height: float,
    z0: float,
    radius: float,
    material,
) -> None:
    z_center = z0 + height * 0.5
    part.visual(
        Box((size_x - 2.0 * radius, size_y, height)),
        origin=Origin(xyz=(0.0, 0.0, z_center)),
        material=material,
        name=f"{prefix}_core_x",
    )
    part.visual(
        Box((size_x, size_y - 2.0 * radius, height)),
        origin=Origin(xyz=(0.0, 0.0, z_center)),
        material=material,
        name=f"{prefix}_core_y",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            part.visual(
                Cylinder(radius=radius, length=height),
                origin=Origin(
                    xyz=(
                        x_sign * (size_x * 0.5 - radius),
                        y_sign * (size_y * 0.5 - radius),
                        z_center,
                    )
                ),
                material=material,
                name=f"{prefix}_corner_{int((x_sign + 1.0) * 0.5)}_{int((y_sign + 1.0) * 0.5)}",
            )


def _add_rounded_wall_ring(
    part,
    *,
    prefix: str,
    size_x: float,
    size_y: float,
    wall_t: float,
    height: float,
    z0: float,
    radius: float,
    material,
) -> None:
    z_center = z0 + height * 0.5
    part.visual(
        Box((size_x - 2.0 * radius, wall_t, height)),
        origin=Origin(xyz=(0.0, -(size_y * 0.5 - wall_t * 0.5), z_center)),
        material=material,
        name=f"{prefix}_front",
    )
    part.visual(
        Box((size_x - 2.0 * radius, wall_t, height)),
        origin=Origin(xyz=(0.0, size_y * 0.5 - wall_t * 0.5, z_center)),
        material=material,
        name=f"{prefix}_rear",
    )
    part.visual(
        Box((wall_t, size_y - 2.0 * radius, height)),
        origin=Origin(xyz=(-(size_x * 0.5 - wall_t * 0.5), 0.0, z_center)),
        material=material,
        name=f"{prefix}_left",
    )
    part.visual(
        Box((wall_t, size_y - 2.0 * radius, height)),
        origin=Origin(xyz=(size_x * 0.5 - wall_t * 0.5, 0.0, z_center)),
        material=material,
        name=f"{prefix}_right",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            part.visual(
                Cylinder(radius=radius, length=height),
                origin=Origin(
                    xyz=(
                        x_sign * (size_x * 0.5 - radius),
                        y_sign * (size_y * 0.5 - radius),
                        z_center,
                    )
                ),
                material=material,
                name=f"{prefix}_corner_{int((x_sign + 1.0) * 0.5)}_{int((y_sign + 1.0) * 0.5)}",
            )


def _lid_section(
    x_pos: float,
    *,
    depth: float,
    rise: float,
    thickness: float,
    edge_drop: float,
) -> list[tuple[float, float, float]]:
    outer = [
        (-depth * 0.5 + 0.014, 0.0),
        (-depth * 0.34, rise * 0.54),
        (-depth * 0.12, rise * 0.88),
        (0.0, rise),
        (depth * 0.12, rise * 0.88),
        (depth * 0.34, rise * 0.54),
        (depth * 0.5 - 0.014, 0.0),
    ]
    inner = [
        (depth * 0.5 - 0.014, -edge_drop),
        (depth * 0.33, rise * 0.54 - thickness * 0.75),
        (depth * 0.10, rise * 0.88 - thickness),
        (0.0, rise - thickness),
        (-depth * 0.10, rise * 0.88 - thickness),
        (-depth * 0.33, rise * 0.54 - thickness * 0.75),
        (-depth * 0.5 + 0.014, -edge_drop),
    ]
    loop = outer + inner
    return [(x_pos, y, z) for y, z in loop]


def _build_lid_mesh(
    *,
    logical_name: str,
    length: float,
    depth: float,
    rise: float,
    thickness: float,
    edge_drop: float,
    end_inset: float,
):
    half = length * 0.5 - end_inset
    sections = [
        _lid_section(-half, depth=depth, rise=rise, thickness=thickness, edge_drop=edge_drop),
        _lid_section(0.0, depth=depth, rise=rise, thickness=thickness, edge_drop=edge_drop),
        _lid_section(half, depth=depth, rise=rise, thickness=thickness, edge_drop=edge_drop),
    ]
    return mesh_from_geometry(section_loft(sections), logical_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    liner_white = model.material("liner_white", rgba=(0.97, 0.98, 0.99, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.58, 0.61, 0.64, 1.0))
    plinth_dark = model.material("plinth_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    glass = model.material("glass", rgba=(0.72, 0.86, 0.92, 0.34))
    panel_black = model.material("panel_black", rgba=(0.16, 0.18, 0.20, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.11, 0.12, 1.0))
    dial_mark = model.material("dial_mark", rgba=(0.86, 0.87, 0.88, 1.0))

    body_l = 1.10
    body_d = 0.68
    body_h = 0.64
    corner_r = 0.09
    wall_t = 0.03
    plinth_h = 0.05
    top_frame_t = 0.045
    wall_z0 = plinth_h
    wall_h = body_h - top_frame_t - wall_z0
    wall_top = wall_z0 + wall_h

    liner_outer_l = 0.98
    liner_outer_d = 0.522
    liner_wall_t = 0.012
    liner_r = 0.055
    floor_z0 = 0.110
    floor_t = 0.012
    liner_wall_z0 = floor_z0 + floor_t
    liner_wall_h = wall_top - liner_wall_z0

    panel_x = 0.24
    panel_z = 0.19
    recess_w = 0.178
    recess_h = 0.080
    recess_depth = 0.025

    rail_y = 0.253
    rail_len = 0.99
    rail_depth = 0.016
    rail_h = 0.010
    lower_rail_top = body_h + 0.006
    upper_rail_top = body_h + 0.068

    lid_len = 0.55
    lid_depth = 0.524
    lid_rise = 0.060
    lid_thickness = 0.012
    lid_edge_drop = 0.010
    lid_end_inset = 0.020
    lid_closed_x = 0.265
    lid_travel = 0.320

    body = model.part("body")

    _add_rounded_prism(
        body,
        prefix="plinth",
        size_x=1.02,
        size_y=0.60,
        height=plinth_h,
        z0=0.0,
        radius=0.070,
        material=plinth_dark,
    )

    body.visual(
        Box((body_l - 2.0 * corner_r, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, body_d * 0.5 - wall_t * 0.5, wall_z0 + wall_h * 0.5)),
        material=cabinet_white,
        name="back_wall",
    )
    body.visual(
        Box((wall_t, body_d - 2.0 * corner_r, wall_h)),
        origin=Origin(xyz=(-(body_l * 0.5 - wall_t * 0.5), 0.0, wall_z0 + wall_h * 0.5)),
        material=cabinet_white,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, body_d - 2.0 * corner_r, wall_h)),
        origin=Origin(xyz=(body_l * 0.5 - wall_t * 0.5, 0.0, wall_z0 + wall_h * 0.5)),
        material=cabinet_white,
        name="right_wall",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            body.visual(
                Cylinder(radius=corner_r, length=wall_h),
                origin=Origin(
                    xyz=(
                        x_sign * (body_l * 0.5 - corner_r),
                        y_sign * (body_d * 0.5 - corner_r),
                        wall_z0 + wall_h * 0.5,
                    )
                ),
                material=cabinet_white,
                name=f"outer_corner_{int((x_sign + 1.0) * 0.5)}_{int((y_sign + 1.0) * 0.5)}",
            )

    recess_left = panel_x - recess_w * 0.5
    recess_right = panel_x + recess_w * 0.5
    front_flat_left = -(body_l * 0.5 - corner_r)
    front_flat_right = body_l * 0.5 - corner_r
    recess_bottom = panel_z - recess_h * 0.5
    recess_top = panel_z + recess_h * 0.5

    body.visual(
        Box((recess_left - front_flat_left, wall_t, wall_h)),
        origin=Origin(
            xyz=(
                (front_flat_left + recess_left) * 0.5,
                -(body_d * 0.5 - wall_t * 0.5),
                wall_z0 + wall_h * 0.5,
            )
        ),
        material=cabinet_white,
        name="front_wall_left",
    )
    body.visual(
        Box((front_flat_right - recess_right, wall_t, wall_h)),
        origin=Origin(
            xyz=(
                (recess_right + front_flat_right) * 0.5,
                -(body_d * 0.5 - wall_t * 0.5),
                wall_z0 + wall_h * 0.5,
            )
        ),
        material=cabinet_white,
        name="front_wall_right",
    )
    body.visual(
        Box((recess_w, wall_t, recess_bottom - wall_z0)),
        origin=Origin(
            xyz=(
                panel_x,
                -(body_d * 0.5 - wall_t * 0.5),
                wall_z0 + (recess_bottom - wall_z0) * 0.5,
            )
        ),
        material=cabinet_white,
        name="front_wall_lower_panel_band",
    )
    body.visual(
        Box((recess_w, wall_t, wall_top - recess_top)),
        origin=Origin(
            xyz=(
                panel_x,
                -(body_d * 0.5 - wall_t * 0.5),
                recess_top + (wall_top - recess_top) * 0.5,
            )
        ),
        material=cabinet_white,
        name="front_wall_upper_panel_band",
    )

    body.visual(
        Box((0.012, recess_depth, recess_h)),
        origin=Origin(
            xyz=(
                recess_left + 0.006,
                -(body_d * 0.5) + wall_t + recess_depth * 0.5,
                panel_z,
            )
        ),
        material=trim_grey,
        name="recess_left_wall",
    )
    body.visual(
        Box((0.012, recess_depth, recess_h)),
        origin=Origin(
            xyz=(
                recess_right - 0.006,
                -(body_d * 0.5) + wall_t + recess_depth * 0.5,
                panel_z,
            )
        ),
        material=trim_grey,
        name="recess_right_wall",
    )
    body.visual(
        Box((recess_w, recess_depth, 0.012)),
        origin=Origin(
            xyz=(
                panel_x,
                -(body_d * 0.5) + wall_t + recess_depth * 0.5,
                recess_bottom + 0.006,
            )
        ),
        material=trim_grey,
        name="recess_bottom_wall",
    )
    body.visual(
        Box((recess_w, recess_depth, 0.012)),
        origin=Origin(
            xyz=(
                panel_x,
                -(body_d * 0.5) + wall_t + recess_depth * 0.5,
                recess_top - 0.006,
            )
        ),
        material=trim_grey,
        name="recess_top_wall",
    )
    body.visual(
        Box((recess_w, 0.012, recess_h)),
        origin=Origin(
            xyz=(
                panel_x,
                -(body_d * 0.5) + wall_t + recess_depth + 0.006,
                panel_z,
            )
        ),
        material=trim_grey,
        name="recess_back_wall",
    )

    _add_rounded_prism(
        body,
        prefix="liner_floor",
        size_x=liner_outer_l - 2.0 * liner_wall_t,
        size_y=liner_outer_d - 2.0 * liner_wall_t,
        height=floor_t,
        z0=floor_z0,
        radius=0.043,
        material=liner_white,
    )
    _add_rounded_wall_ring(
        body,
        prefix="liner_wall",
        size_x=liner_outer_l,
        size_y=liner_outer_d,
        wall_t=liner_wall_t,
        height=liner_wall_h,
        z0=liner_wall_z0,
        radius=liner_r,
        material=liner_white,
    )

    front_frame_depth = body_d * 0.5 - liner_outer_d * 0.5
    side_frame_width = body_l * 0.5 - liner_outer_l * 0.5
    body.visual(
        Box((liner_outer_l, front_frame_depth, top_frame_t)),
        origin=Origin(
            xyz=(
                0.0,
                -(liner_outer_d * 0.5 + front_frame_depth * 0.5),
                body_h - top_frame_t * 0.5,
            )
        ),
        material=trim_grey,
        name="front_top_frame",
    )
    body.visual(
        Box((liner_outer_l, front_frame_depth, top_frame_t)),
        origin=Origin(
            xyz=(
                0.0,
                liner_outer_d * 0.5 + front_frame_depth * 0.5,
                body_h - top_frame_t * 0.5,
            )
        ),
        material=trim_grey,
        name="rear_top_frame",
    )
    body.visual(
        Box((side_frame_width, liner_outer_d, top_frame_t)),
        origin=Origin(
            xyz=(
                -(liner_outer_l * 0.5 + side_frame_width * 0.5),
                0.0,
                body_h - top_frame_t * 0.5,
            )
        ),
        material=trim_grey,
        name="left_top_frame",
    )
    body.visual(
        Box((side_frame_width, liner_outer_d, top_frame_t)),
        origin=Origin(
            xyz=(
                liner_outer_l * 0.5 + side_frame_width * 0.5,
                0.0,
                body_h - top_frame_t * 0.5,
            )
        ),
        material=trim_grey,
        name="right_top_frame",
    )

    for side_name, y_sign in (("front", -1.0), ("rear", 1.0)):
        body.visual(
            Box((rail_len, rail_depth, rail_h)),
            origin=Origin(
                xyz=(0.0, y_sign * rail_y, lower_rail_top - rail_h * 0.5),
            ),
            material=aluminum,
            name=f"{side_name}_lower_rail",
        )
        body.visual(
            Box((rail_len, rail_depth, rail_h)),
            origin=Origin(
                xyz=(0.0, y_sign * rail_y, upper_rail_top - rail_h * 0.5),
            ),
            material=aluminum,
            name=f"{side_name}_upper_rail",
        )
    upper_support_x = (-0.445, -0.355)
    support_w = 0.030
    support_d = 0.016
    support_h = upper_rail_top - rail_h - body_h
    support_y_offset = rail_y + rail_depth
    for side_name, y_sign in (("front", -1.0), ("rear", 1.0)):
        for index, x_center in enumerate(upper_support_x):
            body.visual(
                Box((support_w, support_d, support_h)),
                origin=Origin(
                    xyz=(
                        x_center,
                        y_sign * support_y_offset,
                        body_h + support_h * 0.5,
                    )
                ),
                material=aluminum,
                name=f"{side_name}_upper_support_{index}",
            )

    body.inertial = Inertial.from_geometry(
        Box((body_l, body_d, body_h + 0.08)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, (body_h + 0.08) * 0.5)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.150, 0.006, 0.050)),
        origin=Origin(xyz=(0.0, -0.003, 0.0)),
        material=panel_black,
        name="panel_face",
    )
    control_panel.visual(
        Box((0.138, 0.034, 0.040)),
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material=panel_black,
        name="panel_housing",
    )
    control_panel.visual(
        Box((0.020, 0.002, 0.004)),
        origin=Origin(xyz=(-0.040, -0.0065, 0.012)),
        material=dial_mark,
        name="temp_mark_cool",
    )
    control_panel.visual(
        Box((0.020, 0.002, 0.004)),
        origin=Origin(xyz=(-0.040, -0.0065, -0.012)),
        material=dial_mark,
        name="temp_mark_cold",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((0.150, 0.034, 0.050)),
        mass=0.45,
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(panel_x, -0.319, panel_z)),
    )

    knob = model.part("temperature_knob")
    knob.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_black,
        name="mount_stem",
    )
    knob.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.0, -0.019, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.029, 0.010)),
        material=dial_mark,
        name="knob_pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.030),
        mass=0.08,
    )
    model.articulation(
        "panel_to_knob",
        ArticulationType.REVOLUTE,
        parent=control_panel,
        child=knob,
        origin=Origin(xyz=(0.038, -0.006, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=2.5,
            lower=-2.4,
            upper=2.4,
        ),
    )

    lid_mesh = _build_lid_mesh(
        logical_name="curved_glass_lid",
        length=lid_len,
        depth=lid_depth,
        rise=lid_rise,
        thickness=lid_thickness,
        edge_drop=lid_edge_drop,
        end_inset=lid_end_inset,
    )

    left_lid = model.part("left_lid")
    left_lid.visual(
        lid_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=glass,
        name="glass_shell",
    )
    left_lid.visual(
        Box((lid_len - 0.024, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.234, 0.015)),
        material=aluminum,
        name="front_frame",
    )
    left_lid.visual(
        Box((lid_len - 0.024, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.234, 0.015)),
        material=aluminum,
        name="rear_frame",
    )
    left_lid.visual(
        Box((lid_len - 0.060, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -rail_y, 0.004)),
        material=aluminum,
        name="front_runner",
    )
    left_lid.visual(
        Box((lid_len - 0.060, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, rail_y, 0.004)),
        material=aluminum,
        name="rear_runner",
    )
    left_lid.visual(
        Box((0.020, 0.050, 0.014)),
        origin=Origin(xyz=(-(lid_len * 0.5 - 0.010), -0.205, 0.015)),
        material=aluminum,
        name="left_front_corner",
    )
    left_lid.visual(
        Box((0.020, 0.050, 0.014)),
        origin=Origin(xyz=(-(lid_len * 0.5 - 0.010), 0.205, 0.015)),
        material=aluminum,
        name="left_rear_corner",
    )
    left_lid.visual(
        Box((0.020, 0.050, 0.014)),
        origin=Origin(xyz=(lid_len * 0.5 - 0.010, -0.205, 0.015)),
        material=aluminum,
        name="right_front_corner",
    )
    left_lid.visual(
        Box((0.020, 0.050, 0.014)),
        origin=Origin(xyz=(lid_len * 0.5 - 0.010, 0.205, 0.015)),
        material=aluminum,
        name="right_rear_corner",
    )
    left_lid.visual(
        Box((0.018, 0.120, 0.014)),
        origin=Origin(xyz=(lid_len * 0.5 - 0.009, 0.0, 0.022)),
        material=aluminum,
        name="center_pull",
    )
    left_lid.visual(
        Box((0.012, 0.182, 0.010)),
        origin=Origin(xyz=(lid_len * 0.5 - 0.012, -0.146, 0.017)),
        material=aluminum,
        name="pull_front_bridge",
    )
    left_lid.visual(
        Box((0.012, 0.182, 0.010)),
        origin=Origin(xyz=(lid_len * 0.5 - 0.012, 0.146, 0.017)),
        material=aluminum,
        name="pull_rear_bridge",
    )
    left_lid.inertial = Inertial.from_geometry(
        Box((lid_len, lid_depth, lid_rise + 0.040)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, (lid_rise + 0.040) * 0.5)),
    )

    right_lid = model.part("right_lid")
    right_lid.visual(
        lid_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=glass,
        name="glass_shell",
    )
    right_lid.visual(
        Box((lid_len - 0.024, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, -0.234, 0.015)),
        material=aluminum,
        name="front_frame",
    )
    right_lid.visual(
        Box((lid_len - 0.024, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.234, 0.015)),
        material=aluminum,
        name="rear_frame",
    )
    right_lid.visual(
        Box((lid_len - 0.060, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, -rail_y, 0.004)),
        material=aluminum,
        name="front_runner",
    )
    right_lid.visual(
        Box((lid_len - 0.060, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, rail_y, 0.004)),
        material=aluminum,
        name="rear_runner",
    )
    right_lid.visual(
        Box((0.020, 0.050, 0.014)),
        origin=Origin(xyz=(-(lid_len * 0.5 - 0.010), -0.205, 0.015)),
        material=aluminum,
        name="left_front_corner",
    )
    right_lid.visual(
        Box((0.020, 0.050, 0.014)),
        origin=Origin(xyz=(-(lid_len * 0.5 - 0.010), 0.205, 0.015)),
        material=aluminum,
        name="left_rear_corner",
    )
    right_lid.visual(
        Box((0.020, 0.050, 0.014)),
        origin=Origin(xyz=(lid_len * 0.5 - 0.010, -0.205, 0.015)),
        material=aluminum,
        name="right_front_corner",
    )
    right_lid.visual(
        Box((0.020, 0.050, 0.014)),
        origin=Origin(xyz=(lid_len * 0.5 - 0.010, 0.205, 0.015)),
        material=aluminum,
        name="right_rear_corner",
    )
    right_lid.visual(
        Box((0.018, 0.120, 0.014)),
        origin=Origin(xyz=(-(lid_len * 0.5 - 0.009), 0.0, 0.022)),
        material=aluminum,
        name="center_pull",
    )
    right_lid.visual(
        Box((0.012, 0.182, 0.010)),
        origin=Origin(xyz=(-(lid_len * 0.5 - 0.012), -0.146, 0.017)),
        material=aluminum,
        name="pull_front_bridge",
    )
    right_lid.visual(
        Box((0.012, 0.182, 0.010)),
        origin=Origin(xyz=(-(lid_len * 0.5 - 0.012), 0.146, 0.017)),
        material=aluminum,
        name="pull_rear_bridge",
    )
    right_lid.inertial = Inertial.from_geometry(
        Box((lid_len, lid_depth, lid_rise + 0.040)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, (lid_rise + 0.040) * 0.5)),
    )

    model.articulation(
        "body_to_left_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_lid,
        origin=Origin(xyz=(-lid_closed_x, 0.0, lower_rail_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.30,
            lower=0.0,
            upper=lid_travel,
        ),
    )
    model.articulation(
        "body_to_right_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_lid,
        origin=Origin(xyz=(lid_closed_x, 0.0, upper_rail_top)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.30,
            lower=0.0,
            upper=lid_travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    control_panel = object_model.get_part("control_panel")
    knob = object_model.get_part("temperature_knob")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    left_slide = object_model.get_articulation("body_to_left_lid")
    right_slide = object_model.get_articulation("body_to_right_lid")
    knob_joint = object_model.get_articulation("panel_to_knob")

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

    ctx.expect_contact(
        control_panel,
        body,
        elem_a="panel_housing",
        elem_b="recess_back_wall",
        name="control panel housing seats against the recessed pocket back wall",
    )
    ctx.expect_contact(
        knob,
        control_panel,
        elem_a="mount_stem",
        elem_b="panel_face",
        name="temperature knob is mounted to the control panel face",
    )
    ctx.expect_contact(
        left_lid,
        body,
        elem_a="front_runner",
        elem_b="front_lower_rail",
        name="left lid front runner sits on the lower front guide rail",
    )
    ctx.expect_contact(
        left_lid,
        body,
        elem_a="rear_runner",
        elem_b="rear_lower_rail",
        name="left lid rear runner sits on the lower rear guide rail",
    )
    ctx.expect_contact(
        right_lid,
        body,
        elem_a="front_runner",
        elem_b="front_upper_rail",
        name="right lid front runner sits on the upper front guide rail",
    )
    ctx.expect_contact(
        right_lid,
        body,
        elem_a="rear_runner",
        elem_b="rear_upper_rail",
        name="right lid rear runner sits on the upper rear guide rail",
    )

    panel_face_aabb = ctx.part_element_world_aabb(control_panel, elem="panel_face")
    body_aabb = ctx.part_world_aabb(body)
    body_front_y = body_aabb[0][1] if body_aabb is not None else None
    panel_front_y = panel_face_aabb[0][1] if panel_face_aabb is not None else None
    ctx.check(
        "control panel is recessed behind the cabinet front",
        body_front_y is not None
        and panel_front_y is not None
        and panel_front_y > body_front_y + 0.010
        and panel_front_y < body_front_y + 0.030,
        details=f"body_front_y={body_front_y}, panel_front_y={panel_front_y}",
    )

    left_limits = left_slide.motion_limits
    right_limits = right_slide.motion_limits
    knob_limits = knob_joint.motion_limits
    ctx.check(
        "left lid joint slides along +x",
        left_slide.axis == (1.0, 0.0, 0.0)
        and left_limits is not None
        and left_limits.upper is not None
        and left_limits.upper >= 0.30,
        details=f"axis={left_slide.axis}, limits={left_limits}",
    )
    ctx.check(
        "right lid joint slides along -x",
        right_slide.axis == (-1.0, 0.0, 0.0)
        and right_limits is not None
        and right_limits.upper is not None
        and right_limits.upper >= 0.30,
        details=f"axis={right_slide.axis}, limits={right_limits}",
    )
    ctx.check(
        "temperature knob rotates about the front panel normal",
        knob_joint.axis in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0))
        and knob_limits is not None
        and knob_limits.lower is not None
        and knob_limits.upper is not None
        and knob_limits.lower < 0.0 < knob_limits.upper,
        details=f"axis={knob_joint.axis}, limits={knob_limits}",
    )

    left_rest = ctx.part_world_position(left_lid)
    right_rest = ctx.part_world_position(right_lid)
    with ctx.pose({left_slide: left_limits.upper if left_limits is not None and left_limits.upper is not None else 0.0}):
        left_open = ctx.part_world_position(left_lid)
        ctx.expect_contact(
            left_lid,
            body,
            elem_a="front_runner",
            elem_b="front_lower_rail",
            name="left lid remains captured on the front guide rail when opened",
        )
        ctx.expect_contact(
            left_lid,
            body,
            elem_a="rear_runner",
            elem_b="rear_lower_rail",
            name="left lid remains captured on the rear guide rail when opened",
        )
    with ctx.pose({right_slide: right_limits.upper if right_limits is not None and right_limits.upper is not None else 0.0}):
        right_open = ctx.part_world_position(right_lid)
        ctx.expect_contact(
            right_lid,
            body,
            elem_a="front_runner",
            elem_b="front_upper_rail",
            name="right lid remains captured on the front upper guide rail when opened",
        )
        ctx.expect_contact(
            right_lid,
            body,
            elem_a="rear_runner",
            elem_b="rear_upper_rail",
            name="right lid remains captured on the rear upper guide rail when opened",
        )

    ctx.check(
        "left lid opens toward the right",
        left_rest is not None and left_open is not None and left_open[0] > left_rest[0] + 0.20,
        details=f"rest={left_rest}, open={left_open}",
    )
    ctx.check(
        "right lid opens toward the left",
        right_rest is not None and right_open is not None and right_open[0] < right_rest[0] - 0.20,
        details=f"rest={right_rest}, open={right_open}",
    )

    both_open = {}
    if left_limits is not None and left_limits.upper is not None:
        both_open[left_slide] = left_limits.upper
    if right_limits is not None and right_limits.upper is not None:
        both_open[right_slide] = right_limits.upper
    with ctx.pose(both_open):
        ctx.fail_if_parts_overlap_in_current_pose(
            name="all parts remain clear when both glass lids are translated open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
