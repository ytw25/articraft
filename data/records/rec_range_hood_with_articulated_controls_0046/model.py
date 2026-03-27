from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_cabinet_range_hood", assets=ASSETS)

    housing_white = model.material("housing_white", rgba=(0.93, 0.93, 0.91, 1.0))
    filter_aluminum = model.material("filter_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    control_dark = model.material("control_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    control_mark = model.material("control_mark", rgba=(0.82, 0.84, 0.86, 1.0))

    width = 0.76
    depth = 0.50
    height = 0.14
    wall = 0.012
    top_thickness = 0.012
    front_corner_radius = 0.055

    strip_width = 0.60
    strip_depth = 0.024
    strip_height = 0.040
    strip_z = 0.086
    opening_width = 0.020
    opening_height = 0.012
    opening_bridge = (strip_height - opening_height) / 2.0
    strip_front_y = depth / 2.0 + strip_depth

    filter_width = 0.26
    filter_depth = 0.19
    filter_thickness = 0.004
    filter_center_y = 0.030
    filter_center_z = 0.007
    filter_center_x = 0.14
    filter_support_height = 0.004

    def front_rounded_profile(
        profile_width: float,
        profile_depth: float,
        radius: float,
        *,
        rear_inset: float = 0.0,
        segments: int = 10,
    ) -> list[tuple[float, float]]:
        half_w = profile_width / 2.0
        rear_y = -profile_depth / 2.0 + rear_inset
        front_y = profile_depth / 2.0 - rear_inset
        arc_center_y = front_y - radius
        pts: list[tuple[float, float]] = [
            (-half_w + rear_inset, rear_y),
            (half_w - rear_inset, rear_y),
            (half_w - rear_inset, arc_center_y),
        ]
        right_center_x = half_w - radius
        left_center_x = -half_w + radius
        for step in range(segments + 1):
            angle = (pi / 2.0) * (step / segments)
            pts.append(
                (
                    right_center_x + radius * cos(angle),
                    arc_center_y + radius * sin(angle),
                )
            )
        pts.append((left_center_x, front_y))
        for step in range(segments + 1):
            angle = pi / 2.0 + (pi / 2.0) * (step / segments)
            pts.append(
                (
                    left_center_x + radius * cos(angle),
                    arc_center_y + radius * sin(angle),
                )
            )
        return pts

    outer_profile = front_rounded_profile(width, depth, front_corner_radius)
    inner_profile = front_rounded_profile(
        width - 2.0 * wall,
        depth - 2.0 * wall,
        front_corner_radius - wall,
        rear_inset=0.0,
    )

    wall_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            [inner_profile],
            height - top_thickness,
            cap=False,
            center=False,
        ),
        ASSETS.asset_root / "range_hood_walls.obj",
    )
    top_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(outer_profile, top_thickness),
        ASSETS.asset_root / "range_hood_top.obj",
    )

    housing = model.part("housing")
    housing.visual(
        wall_mesh,
        material=housing_white,
        name="wall_shell",
    )
    housing.visual(
        top_mesh,
        origin=Origin(xyz=(0.0, 0.0, height - top_thickness)),
        material=housing_white,
        name="top_panel",
    )

    left_strip_width = (strip_width - opening_width) / 2.0
    side_strip_center = opening_width / 2.0 + left_strip_width / 2.0
    strip_center_y = depth / 2.0 + strip_depth / 2.0
    bridge_center_z = strip_z + opening_height / 2.0 + opening_bridge / 2.0

    housing.visual(
        Box((left_strip_width, strip_depth, strip_height)),
        origin=Origin(xyz=(-side_strip_center, strip_center_y, strip_z)),
        material=housing_white,
        name="control_strip_left",
    )
    housing.visual(
        Box((left_strip_width, strip_depth, strip_height)),
        origin=Origin(xyz=(side_strip_center, strip_center_y, strip_z)),
        material=housing_white,
        name="control_strip_right",
    )
    housing.visual(
        Box((opening_width, strip_depth, opening_bridge)),
        origin=Origin(xyz=(0.0, strip_center_y, bridge_center_z)),
        material=housing_white,
        name="control_strip_top",
    )
    housing.visual(
        Box((opening_width, strip_depth, opening_bridge)),
        origin=Origin(xyz=(0.0, strip_center_y, strip_z - opening_height / 2.0 - opening_bridge / 2.0)),
        material=housing_white,
        name="control_strip_bottom",
    )
    housing.visual(
        Box((filter_width, filter_depth, filter_thickness)),
        origin=Origin(xyz=(-filter_center_x, filter_center_y, filter_center_z)),
        material=filter_aluminum,
        name="filter_left",
    )
    housing.visual(
        Box((filter_width, filter_depth, filter_thickness)),
        origin=Origin(xyz=(filter_center_x, filter_center_y, filter_center_z)),
        material=filter_aluminum,
        name="filter_right",
    )
    inner_half_width = width / 2.0 - wall
    outer_support_width = inner_half_width - (filter_center_x + filter_width / 2.0)
    support_z = filter_center_z - filter_thickness / 2.0 - filter_support_height / 2.0
    support_x = filter_center_x + filter_width / 2.0 + outer_support_width / 2.0
    housing.visual(
        Box((outer_support_width, filter_depth, filter_support_height)),
        origin=Origin(xyz=(-support_x, filter_center_y, support_z)),
        material=housing_white,
        name="left_filter_support",
    )
    housing.visual(
        Box((outer_support_width, filter_depth, filter_support_height)),
        origin=Origin(xyz=(support_x, filter_center_y, support_z)),
        material=housing_white,
        name="right_filter_support",
    )

    knob_radius = 0.024
    knob_depth = 0.022
    knob_base_radius = 0.026
    knob_base_depth = 0.010
    knob_x = 0.225

    left_knob = model.part("left_knob")
    left_knob.visual(
        Cylinder(radius=knob_base_radius, length=knob_base_depth),
        origin=Origin(xyz=(0.0, knob_base_depth / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=control_dark,
        name="base",
    )
    left_knob.visual(
        Cylinder(radius=knob_radius, length=knob_depth),
        origin=Origin(xyz=(0.0, knob_depth / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=control_dark,
        name="body",
    )
    left_knob.visual(
        Box((0.005, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, knob_depth + 0.002, 0.014)),
        material=control_mark,
        name="pointer",
    )

    right_knob = model.part("right_knob")
    right_knob.visual(
        Cylinder(radius=knob_base_radius, length=knob_base_depth),
        origin=Origin(xyz=(0.0, knob_base_depth / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=control_dark,
        name="base",
    )
    right_knob.visual(
        Cylinder(radius=knob_radius, length=knob_depth),
        origin=Origin(xyz=(0.0, knob_depth / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=control_dark,
        name="body",
    )
    right_knob.visual(
        Box((0.005, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, knob_depth + 0.002, 0.014)),
        material=control_mark,
        name="pointer",
    )

    button_travel = 0.006
    center_button = model.part("center_button")
    center_button.visual(
        Box((0.028, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=control_dark,
        name="cap",
    )
    center_button.visual(
        Box((opening_width, button_travel, opening_height)),
        origin=Origin(xyz=(0.0, -button_travel / 2.0, 0.0)),
        material=control_dark,
        name="stem",
    )

    model.articulation(
        "housing_to_left_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_knob,
        origin=Origin(xyz=(-knob_x, strip_front_y, strip_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "housing_to_right_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_knob,
        origin=Origin(xyz=(knob_x, strip_front_y, strip_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "housing_to_center_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=center_button,
        origin=Origin(xyz=(0.0, strip_front_y + button_travel, strip_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.15, lower=0.0, upper=button_travel),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    left_knob = object_model.get_part("left_knob")
    right_knob = object_model.get_part("right_knob")
    center_button = object_model.get_part("center_button")
    left_knob_joint = object_model.get_articulation("housing_to_left_knob")
    right_knob_joint = object_model.get_articulation("housing_to_right_knob")
    button_joint = object_model.get_articulation("housing_to_center_button")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    ctx.expect_contact(left_knob, housing, elem_a="base", elem_b="control_strip_left", contact_tol=1e-6)
    ctx.expect_contact(right_knob, housing, elem_a="base", elem_b="control_strip_right", contact_tol=1e-6)
    ctx.expect_contact(center_button, housing, contact_tol=1e-6)
    ctx.expect_overlap(left_knob, housing, axes="xz", min_overlap=0.030, elem_a="body", elem_b="control_strip_left")
    ctx.expect_overlap(right_knob, housing, axes="xz", min_overlap=0.030, elem_a="body", elem_b="control_strip_right")
    ctx.expect_origin_gap(center_button, left_knob, axis="x", min_gap=0.18, max_gap=0.28)
    ctx.expect_origin_gap(right_knob, center_button, axis="x", min_gap=0.18, max_gap=0.28)
    ctx.expect_origin_distance(left_knob, right_knob, axes="x", min_dist=0.40, max_dist=0.50)

    left_pos = ctx.part_world_position(left_knob)
    right_pos = ctx.part_world_position(right_knob)
    button_pos = ctx.part_world_position(center_button)
    controls_aligned = (
        left_pos is not None
        and right_pos is not None
        and button_pos is not None
        and abs(left_pos[2] - right_pos[2]) < 0.002
        and abs(left_pos[2] - button_pos[2]) < 0.002
    )
    ctx.check(
        "control_strip_vertical_alignment",
        controls_aligned,
        details=f"left={left_pos}, right={right_pos}, button={button_pos}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    with ctx.pose({left_knob_joint: 0.0}):
        left_pointer_zero = aabb_center(ctx.part_element_world_aabb(left_knob, elem="pointer"))
    with ctx.pose({left_knob_joint: pi / 2.0}):
        left_pointer_quarter = aabb_center(ctx.part_element_world_aabb(left_knob, elem="pointer"))
    pointer_rotates = (
        left_pointer_zero is not None
        and left_pointer_quarter is not None
        and abs(left_pointer_zero[0] - left_pointer_quarter[0]) > 0.010
        and abs(left_pointer_zero[2] - left_pointer_quarter[2]) > 0.010
    )
    ctx.check(
        "left_knob_pointer_moves_with_rotation",
        pointer_rotates,
        details=f"zero={left_pointer_zero}, quarter={left_pointer_quarter}",
    )

    with ctx.pose({right_knob_joint: pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="right_knob_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="right_knob_quarter_turn_no_floating")

    button_limits = button_joint.motion_limits
    if button_limits is not None and button_limits.lower is not None and button_limits.upper is not None:
        with ctx.pose({button_joint: button_limits.lower}):
            rest_pos = ctx.part_world_position(center_button)
            ctx.expect_contact(center_button, housing, contact_tol=1e-6, name="center_button_rest_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="center_button_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="center_button_lower_no_floating")
        with ctx.pose({button_joint: button_limits.upper}):
            pressed_pos = ctx.part_world_position(center_button)
            ctx.expect_contact(center_button, housing, contact_tol=1e-6, name="center_button_upper_contact")
            ctx.fail_if_parts_overlap_in_current_pose(name="center_button_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="center_button_upper_no_floating")
        button_travels = (
            rest_pos is not None
            and pressed_pos is not None
            and (rest_pos[1] - pressed_pos[1]) > 0.0055
        )
        ctx.check(
            "center_button_moves_inward",
            button_travels,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
