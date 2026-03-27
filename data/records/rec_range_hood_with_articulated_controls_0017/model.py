from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    brushed_steel = model.material(
        "brushed_steel",
        rgba=(0.76, 0.78, 0.80, 1.0),
    )
    satin_steel = model.material(
        "satin_steel",
        rgba=(0.67, 0.69, 0.71, 1.0),
    )
    charcoal = model.material(
        "charcoal",
        rgba=(0.14, 0.15, 0.16, 1.0),
    )

    canopy_width_bottom = 0.90
    canopy_width_top = 0.42
    canopy_depth = 0.50
    canopy_height = 0.22
    shell_thickness = 0.012

    chimney_width = 0.28
    chimney_depth = 0.18
    chimney_height = 0.68
    chimney_wall = 0.010
    chimney_center_y = -0.11

    button_size = 0.034
    button_depth = 0.018
    button_proud = 0.007
    button_travel = 0.005
    button_x = 0.155
    button_z_top = 0.145
    button_z_bottom = 0.095

    front_panel_center_y = canopy_depth / 2.0 - shell_thickness / 2.0
    back_panel_center_y = -front_panel_center_y
    side_run = (canopy_width_bottom - canopy_width_top) / 2.0
    side_angle = atan2(side_run, canopy_height)
    side_span = sqrt(canopy_height**2 + side_run**2)
    front_outer_face_y = front_panel_center_y + shell_thickness / 2.0
    button_center_y = front_outer_face_y + button_proud - button_depth / 2.0

    hood_body = model.part("hood_body")
    hood_body.inertial = Inertial.from_geometry(
        Box((canopy_width_bottom, canopy_depth, canopy_height + chimney_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_height + chimney_height) / 2.0)),
    )

    front_profile = [
        (-canopy_width_bottom / 2.0, 0.0),
        (canopy_width_bottom / 2.0, 0.0),
        (canopy_width_top / 2.0, canopy_height),
        (-canopy_width_top / 2.0, canopy_height),
    ]

    def width_at_z(z_pos: float) -> float:
        clamped_z = max(0.0, min(canopy_height, z_pos))
        return canopy_width_bottom - (canopy_width_bottom - canopy_width_top) * (clamped_z / canopy_height)

    def add_front_strip(name: str, z_min: float, z_max: float, *, x_min: float | None = None, x_max: float | None = None) -> None:
        lower_half_width = width_at_z(z_min) / 2.0
        upper_half_width = width_at_z(z_max) / 2.0
        lower_x_min = -lower_half_width if x_min is None else x_min
        lower_x_max = lower_half_width if x_max is None else x_max
        upper_x_min = -upper_half_width if x_min is None else x_min
        upper_x_max = upper_half_width if x_max is None else x_max
        strip_profile = [
            (lower_x_min, z_min),
            (lower_x_max, z_min),
            (upper_x_max, z_max),
            (upper_x_min, z_max),
        ]
        strip_mesh = mesh_from_geometry(
            ExtrudeGeometry(strip_profile, shell_thickness, center=True).rotate_x(pi / 2.0).translate(
                0.0,
                front_panel_center_y,
                0.0,
            ),
            ASSETS.asset_root / f"chimney_range_hood_{name}.obj",
        )
        hood_body.visual(
            strip_mesh,
            material=brushed_steel,
            name=name,
        )

    button_half = button_size / 2.0
    left_button_min_x = -button_x - button_half
    left_button_max_x = -button_x + button_half
    right_button_min_x = button_x - button_half
    right_button_max_x = button_x + button_half

    add_front_strip("front_panel_bottom", 0.0, button_z_bottom - button_half)
    add_front_strip(
        "front_panel_lower_left",
        button_z_bottom - button_half,
        button_z_bottom + button_half,
        x_max=left_button_min_x,
    )
    add_front_strip(
        "front_panel_lower_center",
        button_z_bottom - button_half,
        button_z_bottom + button_half,
        x_min=left_button_max_x,
        x_max=right_button_min_x,
    )
    add_front_strip(
        "front_panel_lower_right",
        button_z_bottom - button_half,
        button_z_bottom + button_half,
        x_min=right_button_max_x,
    )
    add_front_strip("front_panel_middle", button_z_bottom + button_half, button_z_top - button_half)
    add_front_strip(
        "front_panel_upper_left",
        button_z_top - button_half,
        button_z_top + button_half,
        x_max=left_button_min_x,
    )
    add_front_strip(
        "front_panel_upper_center",
        button_z_top - button_half,
        button_z_top + button_half,
        x_min=left_button_max_x,
        x_max=right_button_min_x,
    )
    add_front_strip(
        "front_panel_upper_right",
        button_z_top - button_half,
        button_z_top + button_half,
        x_min=right_button_max_x,
    )
    add_front_strip("front_panel_top", button_z_top + button_half, canopy_height)

    back_panel_geom = ExtrudeGeometry(
        front_profile,
        shell_thickness,
        center=True,
    )
    back_panel_mesh = mesh_from_geometry(
        back_panel_geom.rotate_x(pi / 2.0).translate(0.0, back_panel_center_y, 0.0),
        ASSETS.asset_root / "chimney_range_hood_back_panel.obj",
    )
    hood_body.visual(
        back_panel_mesh,
        material=brushed_steel,
        name="back_panel",
    )

    hood_body.visual(
        Box((canopy_width_top, canopy_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - shell_thickness / 2.0)),
        material=brushed_steel,
        name="canopy_top",
    )
    hood_body.visual(
        Box((shell_thickness, canopy_depth, side_span)),
        origin=Origin(
            xyz=(-((canopy_width_bottom / 2.0 + canopy_width_top / 2.0) / 2.0), 0.0, canopy_height / 2.0),
            rpy=(0.0, side_angle, 0.0),
        ),
        material=brushed_steel,
        name="left_cheek",
    )
    hood_body.visual(
        Box((shell_thickness, canopy_depth, side_span)),
        origin=Origin(
            xyz=(((canopy_width_bottom / 2.0 + canopy_width_top / 2.0) / 2.0), 0.0, canopy_height / 2.0),
            rpy=(0.0, -side_angle, 0.0),
        ),
        material=brushed_steel,
        name="right_cheek",
    )

    filter_panel_thickness = 0.018
    hood_body.visual(
        Box((0.76, canopy_depth - 2.0 * shell_thickness, filter_panel_thickness)),
        origin=Origin(xyz=(0.0, 0.0, filter_panel_thickness / 2.0)),
        material=satin_steel,
        name="baffle_filter",
    )

    chimney_center_z = canopy_height + chimney_height / 2.0
    hood_body.visual(
        Box((chimney_width, chimney_wall, chimney_height)),
        origin=Origin(
            xyz=(0.0, chimney_center_y + chimney_depth / 2.0 - chimney_wall / 2.0, chimney_center_z),
        ),
        material=brushed_steel,
        name="chimney_front",
    )
    hood_body.visual(
        Box((chimney_width, chimney_wall, chimney_height)),
        origin=Origin(
            xyz=(0.0, chimney_center_y - chimney_depth / 2.0 + chimney_wall / 2.0, chimney_center_z),
        ),
        material=brushed_steel,
        name="chimney_back",
    )
    hood_body.visual(
        Box((chimney_wall, chimney_depth - 2.0 * chimney_wall, chimney_height)),
        origin=Origin(
            xyz=(-chimney_width / 2.0 + chimney_wall / 2.0, chimney_center_y, chimney_center_z),
        ),
        material=brushed_steel,
        name="chimney_left",
    )
    hood_body.visual(
        Box((chimney_wall, chimney_depth - 2.0 * chimney_wall, chimney_height)),
        origin=Origin(
            xyz=(chimney_width / 2.0 - chimney_wall / 2.0, chimney_center_y, chimney_center_z),
        ),
        material=brushed_steel,
        name="chimney_right",
    )
    hood_body.visual(
        Box((chimney_width, chimney_depth, chimney_wall)),
        origin=Origin(
            xyz=(0.0, chimney_center_y, canopy_height + chimney_height - chimney_wall / 2.0),
        ),
        material=brushed_steel,
        name="chimney_cap",
    )

    buttons = {
        "button_left_top": (-button_x, button_z_top),
        "button_left_bottom": (-button_x, button_z_bottom),
        "button_right_top": (button_x, button_z_top),
        "button_right_bottom": (button_x, button_z_bottom),
    }

    for button_name, (button_center_x, button_center_z) in buttons.items():
        button = model.part(button_name)
        button.visual(
            Box((button_size, button_depth, button_size)),
            material=charcoal,
            name="cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((button_size, button_depth, button_size)),
            mass=0.05,
        )
        model.articulation(
            f"{button_name}_press",
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(button_center_x, button_center_y, button_center_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.20,
                lower=0.0,
                upper=button_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    button_left_top = object_model.get_part("button_left_top")
    button_left_bottom = object_model.get_part("button_left_bottom")
    button_right_top = object_model.get_part("button_right_top")
    button_right_bottom = object_model.get_part("button_right_bottom")
    button_parts = (
        button_left_top,
        button_left_bottom,
        button_right_top,
        button_right_bottom,
    )
    button_joints = {
        button.name: object_model.get_articulation(f"{button.name}_press")
        for button in button_parts
    }

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

    for part in (hood_body, *button_parts):
        ctx.check(f"{part.name}_present", part is not None, "part lookup failed")

    for joint_name, joint in button_joints.items():
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_axis_is_inward_y",
            tuple(joint.axis) == (0.0, -1.0, 0.0),
            f"expected inward plunger axis, got {joint.axis}",
        )
        ctx.check(
            f"{joint_name}_short_prismatic_throw",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.003 <= limits.upper <= 0.010,
            f"unexpected limits {limits}",
        )

    for button in button_parts:
        ctx.expect_contact(
            button,
            hood_body,
            name=f"{button.name}_rest_contact_with_hood",
        )
        ctx.expect_overlap(
            button,
            hood_body,
            axes="xz",
            min_overlap=0.025,
            name=f"{button.name}_projects_into_front_panel_region",
        )
        ctx.expect_within(
            button,
            hood_body,
            axes="x",
            margin=0.0,
            name=f"{button.name}_within_body_span",
        )

    ctx.expect_origin_distance(
        button_left_top,
        button_left_bottom,
        axes="x",
        max_dist=0.001,
        name="left_pair_vertical_alignment",
    )
    ctx.expect_origin_distance(
        button_right_top,
        button_right_bottom,
        axes="x",
        max_dist=0.001,
        name="right_pair_vertical_alignment",
    )
    ctx.expect_origin_gap(
        button_left_top,
        button_left_bottom,
        axis="z",
        min_gap=0.035,
        max_gap=0.070,
        name="left_pair_vertical_spacing",
    )
    ctx.expect_origin_gap(
        button_right_top,
        button_right_bottom,
        axis="z",
        min_gap=0.035,
        max_gap=0.070,
        name="right_pair_vertical_spacing",
    )
    ctx.expect_origin_gap(
        button_right_top,
        button_left_top,
        axis="x",
        min_gap=0.24,
        max_gap=0.36,
        name="top_row_center_gap",
    )
    ctx.expect_origin_gap(
        button_right_bottom,
        button_left_bottom,
        axis="x",
        min_gap=0.24,
        max_gap=0.36,
        name="bottom_row_center_gap",
    )

    front_panel_aabb = ctx.part_element_world_aabb(hood_body, elem="front_panel_bottom")
    chimney_front_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_front")
    if front_panel_aabb is not None and chimney_front_aabb is not None:
        front_width = front_panel_aabb[1][0] - front_panel_aabb[0][0]
        chimney_width = chimney_front_aabb[1][0] - chimney_front_aabb[0][0]
        ctx.check(
            "chimney_cover_reads_narrow",
            chimney_width < front_width * 0.45,
            f"chimney width {chimney_width:.3f} should be much narrower than canopy width {front_width:.3f}",
        )

    body_aabb = ctx.part_world_aabb(hood_body)
    if body_aabb is not None:
        overall_height = body_aabb[1][2] - body_aabb[0][2]
        overall_width = body_aabb[1][0] - body_aabb[0][0]
        ctx.check(
            "range_hood_realistic_scale",
            0.75 <= overall_height <= 1.10 and 0.80 <= overall_width <= 1.00,
            f"overall size {overall_width:.3f} m x {overall_height:.3f} m is not range-hood-like",
        )

    for button in button_parts:
        joint = button_joints[button.name]
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
                ctx.expect_contact(
                    button,
                    hood_body,
                    name=f"{joint.name}_lower_contact",
                )
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
                ctx.expect_contact(
                    button,
                    hood_body,
                    name=f"{joint.name}_upper_contact",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
