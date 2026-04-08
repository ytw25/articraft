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
    place_on_surface,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_body_shell():
    return superellipse_side_loft(
        [
            (-0.148, 0.014, 0.162, 0.286),
            (-0.090, 0.006, 0.184, 0.338),
            (-0.018, 0.000, 0.188, 0.362),
            (0.060, 0.004, 0.184, 0.350),
            (0.132, 0.016, 0.164, 0.306),
        ],
        exponents=2.6,
        segments=72,
    )


def _build_lid_shell():
    return superellipse_side_loft(
        [
            (-0.124, 0.010, 0.034, 0.272),
            (-0.074, 0.006, 0.048, 0.302),
            (-0.016, 0.004, 0.056, 0.316),
            (0.018, 0.008, 0.040, 0.302),
        ],
        exponents=2.7,
        segments=64,
    )


def _build_handle_frame():
    return tube_from_spline_points(
        [
            (-0.146, 0.000, 0.000),
            (-0.132, -0.012, 0.006),
            (-0.100, -0.032, 0.012),
            (-0.056, -0.050, 0.017),
            (0.000, -0.058, 0.019),
            (0.056, -0.050, 0.017),
            (0.100, -0.032, 0.012),
            (0.132, -0.012, 0.006),
            (0.146, 0.000, 0.000),
        ],
        radius=0.0105,
        samples_per_segment=16,
        radial_segments=18,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="digital_rice_cooker")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.93, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.76, 0.76, 0.74, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.10, 0.11, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.22, 0.35, 0.40, 0.52))
    steel = model.material("steel", rgba=(0.82, 0.83, 0.84, 1.0))
    button_gray = model.material("button_gray", rgba=(0.84, 0.84, 0.82, 1.0))

    panel_angle = math.radians(18.0)

    body = model.part("body")
    body.visual(_mesh("rice_cooker_body_shell", _build_body_shell()), material=body_white, name="body_shell")
    body.visual(
        Box((0.270, 0.214, 0.024)),
        origin=Origin(xyz=(0.000, 0.000, 0.012)),
        material=warm_gray,
        name="base_skirt",
    )
    body.visual(
        _mesh(
            "rice_cooker_top_ring",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.264, 0.204, 0.040),
                [rounded_rect_profile(0.214, 0.156, 0.030)],
                0.010,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.000, -0.004, 0.182)),
        material=warm_gray,
        name="top_ring",
    )
    body.visual(
        _mesh(
            "rice_cooker_inner_liner",
            ExtrudeGeometry(rounded_rect_profile(0.214, 0.156, 0.028), 0.060, center=True),
        ),
        origin=Origin(xyz=(0.000, -0.004, 0.150)),
        material=steel,
        name="inner_liner",
    )
    front_panel = body.visual(
        Box((0.190, 0.016, 0.046)),
        origin=Origin(xyz=(0.000, -0.145, 0.132), rpy=(-panel_angle, 0.0, 0.0)),
        material=warm_gray,
        name="front_panel",
    )
    body.visual(
        Box((0.086, 0.006, 0.032)),
        origin=Origin(xyz=(0.000, -0.153, 0.146), rpy=(-panel_angle, 0.0, 0.0)),
        material=soft_black,
        name="display_bezel",
    )
    body.visual(
        Box((0.074, 0.003, 0.024)),
        origin=Origin(xyz=(0.000, -0.156, 0.146), rpy=(-panel_angle, 0.0, 0.0)),
        material=glass_blue,
        name="display_window",
    )
    button_bezel = body.visual(
        Box((0.166, 0.006, 0.012)),
        origin=Origin(xyz=(0.000, -0.151, 0.086), rpy=(-panel_angle, 0.0, 0.0)),
        material=charcoal,
        name="button_bezel",
    )
    body.visual(
        Box((0.244, 0.018, 0.026)),
        origin=Origin(xyz=(0.000, -0.092, 0.167)),
        material=warm_gray,
        name="liner_front_support",
    )
    body.visual(
        Box((0.244, 0.018, 0.026)),
        origin=Origin(xyz=(0.000, 0.084, 0.167)),
        material=warm_gray,
        name="liner_rear_support",
    )
    body.visual(
        Box((0.018, 0.162, 0.026)),
        origin=Origin(xyz=(-0.116, -0.004, 0.167)),
        material=warm_gray,
        name="liner_left_support",
    )
    body.visual(
        Box((0.018, 0.162, 0.026)),
        origin=Origin(xyz=(0.116, -0.004, 0.167)),
        material=warm_gray,
        name="liner_right_support",
    )
    body.visual(
        Box((0.290, 0.022, 0.028)),
        origin=Origin(xyz=(0.000, 0.131, 0.176)),
        material=warm_gray,
        name="rear_hinge_bridge",
    )
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        body.visual(
            Box((0.040, 0.046, 0.020)),
            origin=Origin(xyz=(side_sign * 0.141, -0.040, 0.170)),
            material=body_white,
            name=f"{side_name}_handle_mount_web",
        )
        body.visual(
            Box((0.022, 0.034, 0.034)),
            origin=Origin(xyz=(side_sign * 0.166, -0.040, 0.178)),
            material=body_white,
            name=f"{side_name}_shoulder",
        )
        body.visual(
            Box((0.010, 0.018, 0.012)),
            origin=Origin(xyz=(side_sign * 0.161, -0.035, 0.194)),
            material=warm_gray,
            name=f"{side_name}_pivot_pad",
        )
    for x_sign, y_sign in ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)):
        body.visual(
            Box((0.048, 0.036, 0.016)),
            origin=Origin(xyz=(x_sign * 0.096, y_sign * 0.076, 0.008)),
            material=charcoal,
            name=f"foot_{'l' if x_sign < 0 else 'r'}_{'f' if y_sign < 0 else 'r'}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.364, 0.304, 0.228)),
        mass=3.6,
        origin=Origin(xyz=(0.000, 0.000, 0.114)),
    )

    lid = model.part("lid")
    lid.visual(_mesh("rice_cooker_lid_shell", _build_lid_shell()), material=body_white, name="lid_shell")
    lid.visual(
        _mesh(
            "rice_cooker_lid_inner_ring",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.240, 0.182, 0.032),
                [rounded_rect_profile(0.196, 0.142, 0.024)],
                0.008,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.000, -0.028, 0.014)),
        material=warm_gray,
        name="lid_inner_ring",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.236),
        origin=Origin(xyz=(0.000, 0.004, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.058, 0.020, 0.010)),
        origin=Origin(xyz=(0.000, -0.106, 0.020)),
        material=warm_gray,
        name="lid_latch",
    )
    lid.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.000, -0.020, 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="steam_vent",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.316, 0.150, 0.060)),
        mass=0.95,
        origin=Origin(xyz=(0.000, -0.052, 0.028)),
    )

    carry_handle = model.part("carry_handle")
    carry_handle.visual(
        _mesh("rice_cooker_carry_handle", _build_handle_frame()),
        material=charcoal,
        name="handle_frame",
    )
    carry_handle.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(-0.146, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="left_handle_pivot",
    )
    carry_handle.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(0.146, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="right_handle_pivot",
    )
    carry_handle.inertial = Inertial.from_geometry(
        Box((0.314, 0.072, 0.044)),
        mass=0.28,
        origin=Origin(xyz=(0.000, 0.028, 0.010)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.000, 0.128, 0.188)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.4,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=carry_handle,
        origin=Origin(xyz=(0.000, -0.035, 0.194)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=1.20,
        ),
    )

    button_offsets = (-0.056, -0.018, 0.018, 0.056)
    for index, x_offset in enumerate(button_offsets):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.026, 0.004, 0.014)),
            origin=Origin(xyz=(0.000, -0.002, 0.000)),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.018, 0.006, 0.009)),
            origin=Origin(xyz=(0.000, -0.005, 0.000)),
            material=warm_gray,
            name="button_crown",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.026, 0.010, 0.014)),
            mass=0.012,
            origin=Origin(xyz=(0.000, -0.004, 0.000)),
        )
        button_origin = place_on_surface(
            button,
            button_bezel,
            point_hint=(x_offset, -0.154, 0.086),
            clearance=0.0,
            child_axis="-y",
            prefer_collisions=False,
            child_prefer_collisions=False,
        )
        model.articulation(
            f"front_panel_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=button_origin,
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0024,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("carry_handle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_hinge = object_model.get_articulation("body_to_handle")
    center_button = object_model.get_part("button_1")
    center_button_joint = object_model.get_articulation("front_panel_to_button_1")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="top_ring",
        min_gap=0.002,
        max_gap=0.022,
        name="closed lid sits just above the cooker rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="top_ring",
        min_overlap=0.09,
        name="closed lid covers the opening footprint",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.07
        and open_lid_aabb[0][1] > closed_lid_aabb[0][1] + 0.02,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_frame")
    with ctx.pose({handle_hinge: 1.20}):
        raised_handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_frame")
    ctx.check(
        "carry handle folds upward from the side shoulders",
        closed_handle_aabb is not None
        and raised_handle_aabb is not None
        and raised_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.03
        and raised_handle_aabb[0][1] > closed_handle_aabb[0][1] + 0.03,
        details=f"closed={closed_handle_aabb}, raised={raised_handle_aabb}",
    )

    rest_button_pos = ctx.part_world_position(center_button)
    with ctx.pose({center_button_joint: 0.0022}):
        pressed_button_pos = ctx.part_world_position(center_button)
    ctx.check(
        "front buttons plunge inward on short travel",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.001
        and pressed_button_pos[2] < rest_button_pos[2] - 0.0002,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
