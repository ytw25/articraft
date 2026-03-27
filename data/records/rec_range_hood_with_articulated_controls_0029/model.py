from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, pi, sin, sqrt
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
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

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_mesh = model.material("filter_dark", rgba=(0.18, 0.18, 0.18, 1.0))
    button_finish = model.material("button_finish", rgba=(0.55, 0.57, 0.60, 1.0))

    wall_thickness = 0.008
    canopy_width = 0.90
    canopy_height = 0.28
    canopy_top_width = 0.52
    canopy_top_depth = 0.30
    control_panel_width = 0.26
    control_panel_height = 0.23
    control_panel_y = 0.496
    canopy_top_z = canopy_height
    chimney_width = 0.30
    chimney_depth = 0.22
    chimney_height = 0.58
    chimney_bottom_z = canopy_top_z
    chimney_top_z = chimney_bottom_z + chimney_height

    hood_body = model.part("hood_body")
    hood_body.inertial = Inertial.from_geometry(
        Box((canopy_width, 0.50, chimney_top_z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.25, chimney_top_z / 2.0)),
    )

    def add_line_panel(
        name: str,
        start_xy: tuple[float, float],
        end_xy: tuple[float, float],
        *,
        height: float,
        z_center: float,
        thickness: float = wall_thickness,
        material=stainless,
    ) -> None:
        dx = end_xy[0] - start_xy[0]
        dy = end_xy[1] - start_xy[1]
        length = sqrt(dx * dx + dy * dy)
        hood_body.visual(
            Box((length, thickness, height)),
            origin=Origin(
                xyz=((start_xy[0] + end_xy[0]) / 2.0, (start_xy[1] + end_xy[1]) / 2.0, z_center),
                rpy=(0.0, 0.0, atan2(dy, dx)),
            ),
            material=material,
            name=name,
        )

    def circle_profile(
        radius: float,
        *,
        center_xy: tuple[float, float] = (0.0, 0.0),
        segments: int = 28,
    ) -> list[tuple[float, float]]:
        cx, cy = center_xy
        return [
            (
                cx + radius * cos((2.0 * pi * i) / segments),
                cy + radius * sin((2.0 * pi * i) / segments),
            )
            for i in range(segments)
        ]

    hood_body.visual(
        Box((canopy_width, wall_thickness, canopy_height)),
        origin=Origin(xyz=(0.0, wall_thickness / 2.0, canopy_height / 2.0)),
        material=stainless,
        name="canopy_back",
    )
    hood_body.visual(
        Box((canopy_top_width, canopy_top_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, canopy_top_depth / 2.0 + 0.02, canopy_top_z - wall_thickness / 2.0)),
        material=stainless,
        name="canopy_top",
    )
    add_line_panel(
        "canopy_left_side",
        (-0.45, wall_thickness / 2.0),
        (-0.24, 0.40),
        height=canopy_height,
        z_center=canopy_height / 2.0,
    )
    add_line_panel(
        "canopy_right_side",
        (0.24, 0.40),
        (0.45, wall_thickness / 2.0),
        height=canopy_height,
        z_center=canopy_height / 2.0,
    )
    add_line_panel(
        "front_left_facet",
        (-0.24, 0.40),
        (-control_panel_width / 2.0, control_panel_y),
        height=control_panel_height,
        z_center=0.145,
    )
    add_line_panel(
        "front_right_facet",
        (control_panel_width / 2.0, control_panel_y),
        (0.24, 0.40),
        height=control_panel_height,
        z_center=0.145,
    )
    hood_body.visual(
        Box((0.64, 0.30, 0.012)),
        origin=Origin(xyz=(0.0, 0.18, 0.012)),
        material=dark_mesh,
        name="grease_filter",
    )
    hood_body.visual(
        Box((chimney_width, wall_thickness, chimney_height)),
        origin=Origin(
            xyz=(0.0, wall_thickness / 2.0, chimney_bottom_z + chimney_height / 2.0),
        ),
        material=stainless,
        name="chimney_back",
    )
    hood_body.visual(
        Box((chimney_width, wall_thickness, chimney_height)),
        origin=Origin(
            xyz=(0.0, chimney_depth - wall_thickness / 2.0, chimney_bottom_z + chimney_height / 2.0),
        ),
        material=stainless,
        name="chimney_front",
    )
    hood_body.visual(
        Box((wall_thickness, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(-chimney_width / 2.0 + wall_thickness / 2.0, chimney_depth / 2.0, chimney_bottom_z + chimney_height / 2.0),
        ),
        material=stainless,
        name="chimney_left",
    )
    hood_body.visual(
        Box((wall_thickness, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(chimney_width / 2.0 - wall_thickness / 2.0, chimney_depth / 2.0, chimney_bottom_z + chimney_height / 2.0),
        ),
        material=stainless,
        name="chimney_right",
    )
    hood_body.visual(
        Box((chimney_width, chimney_depth, wall_thickness)),
        origin=Origin(xyz=(0.0, chimney_depth / 2.0, chimney_top_z - wall_thickness / 2.0)),
        material=stainless,
        name="chimney_cap",
    )

    big_hole_radius = 0.011
    small_hole_radius = 0.0078
    vertical_spacing = 0.067
    side_button_offset = 0.067
    control_panel_geom = ExtrudeWithHolesGeometry(
        [
            (-control_panel_width / 2.0, -control_panel_height / 2.0),
            (control_panel_width / 2.0, -control_panel_height / 2.0),
            (control_panel_width / 2.0, control_panel_height / 2.0),
            (-control_panel_width / 2.0, control_panel_height / 2.0),
        ],
        [
            circle_profile(big_hole_radius, center_xy=(0.0, vertical_spacing)),
            circle_profile(big_hole_radius, center_xy=(0.0, 0.0)),
            circle_profile(big_hole_radius, center_xy=(0.0, -vertical_spacing)),
            circle_profile(small_hole_radius, center_xy=(-side_button_offset, 0.0)),
            circle_profile(small_hole_radius, center_xy=(side_button_offset, 0.0)),
        ],
        wall_thickness,
        center=True,
    )
    control_panel_mesh = mesh_from_geometry(
        control_panel_geom,
        Path(ASSETS.asset_root) / "control_panel.obj",
    )
    hood_body.visual(
        control_panel_mesh,
        origin=Origin(xyz=(0.0, control_panel_y, 0.145), rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="control_panel",
    )

    def add_button(
        part_name: str,
        joint_name: str,
        *,
        x: float,
        z: float,
        cap_radius: float,
        stem_radius: float,
        cap_length: float,
        stem_length: float,
        travel: float,
    ) -> None:
        retainer_radius = cap_radius + 0.004
        retainer_length = 0.003
        stem_front_offset = 0.0005

        button = model.part(part_name)
        button.visual(
            Cylinder(radius=cap_radius, length=cap_length),
            origin=Origin(xyz=(0.0, cap_length / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=button_finish,
            name="cap",
        )
        button.visual(
            Cylinder(radius=stem_radius, length=stem_length),
            origin=Origin(
                xyz=(0.0, -stem_length / 2.0 + stem_front_offset, 0.0),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=button_finish,
            name="stem",
        )
        button.visual(
            Cylinder(radius=retainer_radius, length=retainer_length),
            origin=Origin(
                xyz=(0.0, -wall_thickness - retainer_length / 2.0, 0.0),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material=button_finish,
            name="retainer",
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=retainer_radius, length=cap_length + stem_length + retainer_length),
            mass=0.08,
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=hood_body,
            child=button,
            origin=Origin(xyz=(x, control_panel_y + wall_thickness / 2.0, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.10,
                lower=0.0,
                upper=travel,
            ),
        )

    add_button(
        "top_button",
        "top_button_press",
        x=0.0,
        z=0.145 + vertical_spacing,
        cap_radius=0.0102,
        stem_radius=0.0075,
        cap_length=0.007,
        stem_length=0.018,
        travel=0.0025,
    )
    add_button(
        "middle_button",
        "middle_button_press",
        x=0.0,
        z=0.145,
        cap_radius=0.0102,
        stem_radius=0.0075,
        cap_length=0.007,
        stem_length=0.018,
        travel=0.0025,
    )
    add_button(
        "bottom_button",
        "bottom_button_press",
        x=0.0,
        z=0.145 - vertical_spacing,
        cap_radius=0.0102,
        stem_radius=0.0075,
        cap_length=0.007,
        stem_length=0.018,
        travel=0.0025,
    )
    add_button(
        "left_button",
        "left_button_press",
        x=-side_button_offset,
        z=0.145,
        cap_radius=0.0068,
        stem_radius=0.0048,
        cap_length=0.006,
        stem_length=0.016,
        travel=0.0020,
    )
    add_button(
        "right_button",
        "right_button_press",
        x=side_button_offset,
        z=0.145,
        cap_radius=0.0068,
        stem_radius=0.0048,
        cap_length=0.006,
        stem_length=0.016,
        travel=0.0020,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    top_button = object_model.get_part("top_button")
    middle_button = object_model.get_part("middle_button")
    bottom_button = object_model.get_part("bottom_button")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")

    top_joint = object_model.get_articulation("top_button_press")
    middle_joint = object_model.get_articulation("middle_button_press")
    bottom_joint = object_model.get_articulation("bottom_button_press")
    left_joint = object_model.get_articulation("left_button_press")
    right_joint = object_model.get_articulation("right_button_press")

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

    joints = {
        "top": top_joint,
        "middle": middle_joint,
        "bottom": bottom_joint,
        "left": left_joint,
        "right": right_joint,
    }
    for label, joint in joints.items():
        limits = joint.motion_limits
        axis_ok = tuple(round(v, 4) for v in joint.axis) == (0.0, -1.0, 0.0)
        ctx.check(f"{label}_button_prismatic_axis", axis_ok, f"{joint.name} axis was {joint.axis}")
        ctx.check(
            f"{label}_button_limits",
            joint.joint_type == ArticulationType.PRISMATIC
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.0015 <= limits.upper <= 0.0030,
            f"{joint.name} limits/type were {joint.joint_type}, {limits}",
        )

    with ctx.pose(
        {
            top_joint: 0.0,
            middle_joint: 0.0,
            bottom_joint: 0.0,
            left_joint: 0.0,
            right_joint: 0.0,
        }
    ):
        ctx.expect_gap(
            top_button,
            hood_body,
            axis="y",
            min_gap=-0.0001,
            max_gap=0.0001,
            positive_elem="cap",
            negative_elem="control_panel",
            name="top_button_rest_flush",
        )
        ctx.expect_gap(
            middle_button,
            hood_body,
            axis="y",
            min_gap=-0.0001,
            max_gap=0.0001,
            positive_elem="cap",
            negative_elem="control_panel",
            name="middle_button_rest_flush",
        )
        ctx.expect_gap(
            bottom_button,
            hood_body,
            axis="y",
            min_gap=-0.0001,
            max_gap=0.0001,
            positive_elem="cap",
            negative_elem="control_panel",
            name="bottom_button_rest_flush",
        )
        ctx.expect_gap(
            left_button,
            hood_body,
            axis="y",
            min_gap=-0.0001,
            max_gap=0.0001,
            positive_elem="cap",
            negative_elem="control_panel",
            name="left_button_rest_flush",
        )
        ctx.expect_gap(
            right_button,
            hood_body,
            axis="y",
            min_gap=-0.0001,
            max_gap=0.0001,
            positive_elem="cap",
            negative_elem="control_panel",
            name="right_button_rest_flush",
        )
        ctx.expect_overlap(
            top_button,
            hood_body,
            axes="xz",
            min_overlap=0.018,
            elem_a="cap",
            elem_b="control_panel",
            name="top_button_on_control_panel",
        )
        ctx.expect_overlap(
            middle_button,
            hood_body,
            axes="xz",
            min_overlap=0.018,
            elem_a="cap",
            elem_b="control_panel",
            name="middle_button_on_control_panel",
        )
        ctx.expect_overlap(
            bottom_button,
            hood_body,
            axes="xz",
            min_overlap=0.018,
            elem_a="cap",
            elem_b="control_panel",
            name="bottom_button_on_control_panel",
        )
        ctx.expect_overlap(
            left_button,
            hood_body,
            axes="xz",
            min_overlap=0.012,
            elem_a="cap",
            elem_b="control_panel",
            name="left_button_on_control_panel",
        )
        ctx.expect_overlap(
            right_button,
            hood_body,
            axes="xz",
            min_overlap=0.012,
            elem_a="cap",
            elem_b="control_panel",
            name="right_button_on_control_panel",
        )
        ctx.expect_origin_distance(
            top_button,
            middle_button,
            axes="z",
            min_dist=0.066,
            max_dist=0.068,
            name="top_middle_vertical_spacing",
        )
        ctx.expect_origin_distance(
            middle_button,
            bottom_button,
            axes="z",
            min_dist=0.066,
            max_dist=0.068,
            name="middle_bottom_vertical_spacing",
        )
        ctx.expect_origin_distance(
            left_button,
            right_button,
            axes="x",
            min_dist=0.133,
            max_dist=0.135,
            name="side_button_spacing",
        )
        ctx.expect_origin_distance(
            left_button,
            middle_button,
            axes="z",
            min_dist=0.0,
            max_dist=0.0001,
            name="left_button_aligned_with_middle",
        )
        ctx.expect_origin_distance(
            right_button,
            middle_button,
            axes="z",
            min_dist=0.0,
            max_dist=0.0001,
            name="right_button_aligned_with_middle",
        )

    for joint, button, expected_recess, label in (
        (top_joint, top_button, 0.0025, "top"),
        (middle_joint, middle_button, 0.0025, "middle"),
        (bottom_joint, bottom_button, 0.0025, "bottom"),
        (left_joint, left_button, 0.0020, "left"),
        (right_joint, right_button, 0.0020, "right"),
    ):
        limits = joint.motion_limits
        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{label}_button_pressed_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{label}_button_pressed_no_floating")
                ctx.expect_gap(
                    button,
                    hood_body,
                    axis="y",
                    min_gap=-(expected_recess + 0.0001),
                    max_gap=-(expected_recess - 0.0001),
                    positive_elem="cap",
                    negative_elem="control_panel",
                    name=f"{label}_button_pressed_recess",
                )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
