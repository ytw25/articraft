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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_rice_cooker")

    body_white = model.material("body_white", rgba=(0.94, 0.94, 0.92, 1.0))
    lid_white = model.material("lid_white", rgba=(0.96, 0.96, 0.95, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.20, 0.22, 0.24, 1.0))
    soft_black = model.material("soft_black", rgba=(0.12, 0.13, 0.14, 1.0))
    display_black = model.material("display_black", rgba=(0.10, 0.17, 0.20, 0.78))
    button_grey = model.material("button_grey", rgba=(0.83, 0.84, 0.85, 1.0))
    accent_chrome = model.material("accent_chrome", rgba=(0.88, 0.89, 0.90, 1.0))
    seal_grey = model.material("seal_grey", rgba=(0.55, 0.57, 0.58, 1.0))

    def xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=10)]

    def xz_section(
        width: float,
        height: float,
        radius: float,
        y: float,
        *,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_center) for x, z in rounded_rect_profile(width, height, radius, corner_segments=10)]

    def offset_profile(
        profile: list[tuple[float, float]],
        dx: float,
        dy: float,
    ) -> list[tuple[float, float]]:
        return [(x + dx, y + dy) for x, y in profile]

    body_shell = mesh_from_geometry(
        section_loft(
            [
                xy_section(0.262, 0.224, 0.046, 0.000),
                xy_section(0.306, 0.264, 0.060, 0.040),
                xy_section(0.344, 0.302, 0.074, 0.118),
                xy_section(0.328, 0.286, 0.070, 0.204),
            ],
            cap=False,
            solid=False,
        ),
        "rice_cooker_body_shell",
    )

    lid_shell = mesh_from_geometry(
        section_loft(
            [
                xz_section(0.236, 0.030, 0.015, -0.252, z_center=0.020),
                xz_section(0.286, 0.056, 0.024, -0.172, z_center=0.034),
                xz_section(0.314, 0.066, 0.030, -0.092, z_center=0.038),
                xz_section(0.304, 0.050, 0.024, -0.016, z_center=0.028),
            ]
        ),
        "rice_cooker_lid_shell",
    )

    inner_panel_shell = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.248, 0.188, 0.018, corner_segments=10),
            0.008,
        ),
        "rice_cooker_inner_panel",
    )

    top_rim_ring = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.294, 0.248, 0.026, corner_segments=12),
            [
                rounded_rect_profile(0.258, 0.206, 0.018, corner_segments=12),
            ],
            0.006,
            center=True,
        ),
        "rice_cooker_top_rim_ring",
    )

    control_panel_face = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.176, 0.086, 0.014, corner_segments=10),
            [
                offset_profile(rounded_rect_profile(0.118, 0.030, 0.006, corner_segments=8), 0.000, 0.020),
                offset_profile(rounded_rect_profile(0.074, 0.020, 0.008, corner_segments=8), 0.000, -0.004),
                offset_profile(rounded_rect_profile(0.024, 0.014, 0.006, corner_segments=8), -0.040, -0.028),
                offset_profile(rounded_rect_profile(0.024, 0.014, 0.006, corner_segments=8), 0.040, -0.028),
            ],
            0.006,
            center=True,
        ),
        "rice_cooker_control_panel_face",
    )

    body = model.part("body")
    body.visual(body_shell, material=body_white, name="outer_shell")
    body.visual(
        Box((0.258, 0.212, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=body_white,
        name="bottom_base",
    )
    body.visual(
        top_rim_ring,
        origin=Origin(xyz=(0.000, 0.000, 0.200)),
        material=trim_silver,
        name="top_rim",
    )
    body.visual(
        control_panel_face,
        origin=Origin(xyz=(0.000, -0.151, 0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="control_panel_face",
    )
    body.visual(
        Box((0.176, 0.020, 0.018)),
        origin=Origin(xyz=(0.000, -0.129, 0.122)),
        material=trim_silver,
        name="control_top_frame",
    )
    body.visual(
        Box((0.176, 0.020, 0.018)),
        origin=Origin(xyz=(0.000, -0.129, 0.042)),
        material=trim_silver,
        name="control_bottom_frame",
    )
    body.visual(
        Box((0.018, 0.020, 0.066)),
        origin=Origin(xyz=(-0.079, -0.129, 0.082)),
        material=trim_silver,
        name="control_left_frame",
    )
    body.visual(
        Box((0.018, 0.020, 0.066)),
        origin=Origin(xyz=(0.079, -0.129, 0.082)),
        material=trim_silver,
        name="control_right_frame",
    )
    body.visual(
        Box((0.114, 0.004, 0.026)),
        origin=Origin(xyz=(0.000, -0.146, 0.105)),
        material=dark_grey,
        name="display_bezel",
    )
    body.visual(
        Box((0.286, 0.022, 0.018)),
        origin=Origin(xyz=(0.000, 0.132, 0.196)),
        material=soft_black,
        name="rear_hinge_cowl",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.308),
        origin=Origin(xyz=(0.000, 0.141, 0.186), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="rear_hinge_bar",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.344, 0.302, 0.226)),
        mass=4.4,
        origin=Origin(xyz=(0.000, 0.000, 0.113)),
    )

    lid = model.part("main_lid")
    lid.visual(
        lid_shell,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=lid_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.194, 0.122, 0.018)),
        origin=Origin(xyz=(0.000, -0.156, 0.060)),
        material=trim_silver,
        name="handle_bridge",
    )
    lid.visual(
        Box((0.236, 0.220, 0.010)),
        origin=Origin(xyz=(0.000, -0.128, 0.014)),
        material=soft_black,
        name="lid_inner_frame",
    )
    lid.visual(
        Box((0.208, 0.020, 0.012)),
        origin=Origin(xyz=(0.000, -0.010, 0.009)),
        material=soft_black,
        name="inner_panel_mount",
    )
    lid.visual(
        Box((0.054, 0.036, 0.014)),
        origin=Origin(xyz=(0.000, -0.095, 0.045)),
        material=accent_chrome,
        name="steam_vent_cap",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.314, 0.146, 0.066)),
        mass=1.2,
        origin=Origin(xyz=(0.000, -0.064, 0.030)),
    )

    inner_lid = model.part("inner_lid_panel")
    inner_lid.visual(
        inner_panel_shell,
        origin=Origin(xyz=(0.000, -0.101, -0.020)),
        material=seal_grey,
        name="panel_plate",
    )
    inner_lid.visual(
        Box((0.210, 0.014, 0.006)),
        origin=Origin(xyz=(0.000, -0.002, -0.003)),
        material=dark_grey,
        name="panel_hinge_leaf",
    )
    inner_lid.visual(
        Box((0.026, 0.010, 0.006)),
        origin=Origin(xyz=(-0.070, -0.006, -0.009)),
        material=dark_grey,
        name="left_hinge_tab",
    )
    inner_lid.visual(
        Box((0.026, 0.010, 0.006)),
        origin=Origin(xyz=(0.070, -0.006, -0.009)),
        material=dark_grey,
        name="right_hinge_tab",
    )
    inner_lid.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.000, -0.084, -0.015)),
        material=trim_silver,
        name="steam_plate",
    )
    inner_lid.inertial = Inertial.from_geometry(
        Box((0.248, 0.188, 0.014)),
        mass=0.32,
        origin=Origin(xyz=(0.000, -0.101, -0.016)),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.068, 0.006, 0.024)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=button_grey,
        name="release_cap",
    )
    release_button.visual(
        Box((0.040, 0.004, 0.014)),
        origin=Origin(xyz=(0.000, -0.004, 0.000)),
        material=dark_grey,
        name="release_stem",
    )
    release_button.visual(
        Box((0.082, 0.003, 0.030)),
        origin=Origin(xyz=(0.000, -0.006, 0.000)),
        material=dark_grey,
        name="release_retainer",
    )
    release_button.inertial = Inertial.from_geometry(
        Box((0.082, 0.010, 0.030)),
        mass=0.08,
        origin=Origin(xyz=(0.000, -0.002, 0.000)),
    )

    left_menu_button = model.part("left_menu_button")
    left_menu_button.visual(
        Box((0.022, 0.006, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=button_grey,
        name="menu_cap",
    )
    left_menu_button.visual(
        Box((0.014, 0.004, 0.010)),
        origin=Origin(xyz=(0.000, -0.004, 0.000)),
        material=dark_grey,
        name="menu_stem",
    )
    left_menu_button.visual(
        Box((0.030, 0.003, 0.020)),
        origin=Origin(xyz=(0.000, -0.006, 0.000)),
        material=dark_grey,
        name="menu_retainer",
    )
    left_menu_button.inertial = Inertial.from_geometry(
        Box((0.030, 0.010, 0.020)),
        mass=0.02,
        origin=Origin(xyz=(0.000, -0.002, 0.000)),
    )

    right_menu_button = model.part("right_menu_button")
    right_menu_button.visual(
        Box((0.022, 0.006, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=button_grey,
        name="menu_cap",
    )
    right_menu_button.visual(
        Box((0.014, 0.004, 0.010)),
        origin=Origin(xyz=(0.000, -0.004, 0.000)),
        material=dark_grey,
        name="menu_stem",
    )
    right_menu_button.visual(
        Box((0.030, 0.003, 0.020)),
        origin=Origin(xyz=(0.000, -0.006, 0.000)),
        material=dark_grey,
        name="menu_retainer",
    )
    right_menu_button.inertial = Inertial.from_geometry(
        Box((0.030, 0.010, 0.020)),
        mass=0.02,
        origin=Origin(xyz=(0.000, -0.002, 0.000)),
    )

    model.articulation(
        "body_to_main_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.000, 0.131, 0.202)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )

    model.articulation(
        "main_lid_to_inner_lid_panel",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=inner_lid,
        origin=Origin(xyz=(0.000, -0.015, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.4,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(0.000, -0.157, 0.081)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )

    model.articulation(
        "body_to_left_menu_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_menu_button,
        origin=Origin(xyz=(-0.040, -0.157, 0.057)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0025,
        ),
    )

    model.articulation(
        "body_to_right_menu_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_menu_button,
        origin=Origin(xyz=(0.040, -0.157, 0.057)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0025,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("main_lid")
    inner_lid = object_model.get_part("inner_lid_panel")
    release_button = object_model.get_part("release_button")
    left_menu_button = object_model.get_part("left_menu_button")
    right_menu_button = object_model.get_part("right_menu_button")
    lid_hinge = object_model.get_articulation("body_to_main_lid")
    inner_hinge = object_model.get_articulation("main_lid_to_inner_lid_panel")
    release_joint = object_model.get_articulation("body_to_release_button")
    left_menu_joint = object_model.get_articulation("body_to_left_menu_button")
    right_menu_joint = object_model.get_articulation("body_to_right_menu_button")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_inner_frame",
        negative_elem="top_rim",
        max_gap=0.014,
        max_penetration=0.0,
        name="closed main lid seats just above body rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="top_rim",
        min_overlap=0.220,
        name="main lid covers the cooker opening",
    )
    ctx.expect_contact(
        release_button,
        body,
        elem_a="release_cap",
        elem_b="control_panel_face",
        name="release button face sits against the control panel",
    )
    ctx.expect_contact(
        left_menu_button,
        body,
        elem_a="menu_cap",
        elem_b="control_panel_face",
        name="left menu button face sits against the control panel",
    )
    ctx.expect_contact(
        right_menu_button,
        body,
        elem_a="menu_cap",
        elem_b="control_panel_face",
        name="right menu button face sits against the control panel",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: math.radians(96.0)}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "main lid opens upward from rear hinge",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.100,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )

    rest_inner_aabb = ctx.part_world_aabb(inner_lid)
    with ctx.pose({inner_hinge: math.radians(72.0)}):
        open_inner_aabb = ctx.part_world_aabb(inner_lid)
    ctx.check(
        "inner lid panel swings downward beneath the main lid",
        rest_inner_aabb is not None
        and open_inner_aabb is not None
        and open_inner_aabb[0][2] < rest_inner_aabb[0][2] - 0.050,
        details=f"rest={rest_inner_aabb}, open={open_inner_aabb}",
    )

    rest_release_pos = ctx.part_world_position(release_button)
    with ctx.pose({release_joint: 0.003}):
        pressed_release_pos = ctx.part_world_position(release_button)
    ctx.check(
        "release button presses into the front housing",
        rest_release_pos is not None
        and pressed_release_pos is not None
        and pressed_release_pos[1] > rest_release_pos[1] + 0.0025,
        details=f"rest={rest_release_pos}, pressed={pressed_release_pos}",
    )

    rest_left_menu_pos = ctx.part_world_position(left_menu_button)
    with ctx.pose({left_menu_joint: 0.0025}):
        pressed_left_menu_pos = ctx.part_world_position(left_menu_button)
    ctx.check(
        "left menu button plunges inward",
        rest_left_menu_pos is not None
        and pressed_left_menu_pos is not None
        and pressed_left_menu_pos[1] > rest_left_menu_pos[1] + 0.0020,
        details=f"rest={rest_left_menu_pos}, pressed={pressed_left_menu_pos}",
    )

    rest_right_menu_pos = ctx.part_world_position(right_menu_button)
    with ctx.pose({right_menu_joint: 0.0025}):
        pressed_right_menu_pos = ctx.part_world_position(right_menu_button)
    ctx.check(
        "right menu button plunges inward",
        rest_right_menu_pos is not None
        and pressed_right_menu_pos is not None
        and pressed_right_menu_pos[1] > rest_right_menu_pos[1] + 0.0020,
        details=f"rest={rest_right_menu_pos}, pressed={pressed_right_menu_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
