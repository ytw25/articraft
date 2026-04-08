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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _chamber_cup_mesh(name: str):
    outer_profile = [
        (0.010, 0.000),
        (0.045, 0.000),
        (0.058, 0.010),
        (0.065, 0.032),
        (0.067, 0.058),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.040, 0.004),
        (0.053, 0.012),
        (0.060, 0.032),
        (0.060, 0.054),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _lid_dome_mesh(name: str):
    rim_radius = 0.072
    outer_profile = [
        (rim_radius, 0.000),
        (0.070, 0.016),
        (0.061, 0.042),
        (0.044, 0.070),
        (0.029, 0.086),
    ]
    inner_profile = [
        (0.067, 0.004),
        (0.065, 0.020),
        (0.056, 0.042),
        (0.039, 0.067),
        (0.024, 0.080),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _cylindrical_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 48,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, height)],
            [(inner_radius, 0.0), (inner_radius, height)],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_juicer")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    smoky_clear = model.material("smoky_clear", rgba=(0.76, 0.86, 0.90, 0.28))
    clear_lid = model.material("clear_lid", rgba=(0.82, 0.90, 0.94, 0.22))
    basket_metal = model.material("basket_metal", rgba=(0.76, 0.78, 0.80, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.72, 1.0))
    button_red = model.material("button_red", rgba=(0.76, 0.14, 0.12, 1.0))
    button_gray = model.material("button_gray", rgba=(0.82, 0.84, 0.86, 1.0))
    pusher_dark = model.material("pusher_dark", rgba=(0.22, 0.24, 0.27, 1.0))

    chamber_cup = _chamber_cup_mesh("juicer_chamber_cup")
    lid_dome = _lid_dome_mesh("juicer_lid_dome")
    lid_chute = _cylindrical_shell_mesh(
        "juicer_lid_chute",
        outer_radius=0.028,
        inner_radius=0.024,
        height=0.080,
    )
    basket_rim = mesh_from_geometry(TorusGeometry(radius=0.051, tube=0.0025), "basket_rim")
    basket_lower_ring = mesh_from_geometry(
        TorusGeometry(radius=0.049, tube=0.0022),
        "basket_lower_ring",
    )

    chamber_center = (0.0, 0.015, 0.151)
    chamber_rim_radius = 0.072
    chamber_rim_z = chamber_center[2] + 0.058

    body = model.part("body")
    body.visual(
        Box((0.240, 0.180, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=body_white,
        name="base_plinth",
    )
    body.visual(
        Box((0.076, 0.180, 0.104)),
        origin=Origin(xyz=(-0.082, 0.0, 0.088)),
        material=body_white,
        name="left_body_wall",
    )
    body.visual(
        Box((0.076, 0.180, 0.104)),
        origin=Origin(xyz=(0.082, 0.0, 0.088)),
        material=body_white,
        name="right_body_wall",
    )
    body.visual(
        Box((0.240, 0.090, 0.104)),
        origin=Origin(xyz=(0.0, 0.045, 0.088)),
        material=body_white,
        name="rear_body_wall",
    )
    body.visual(
        Box((0.210, 0.160, 0.018)),
        origin=Origin(xyz=(0.0, 0.004, 0.141)),
        material=body_white,
        name="top_deck",
    )
    body.visual(
        Cylinder(radius=0.077, length=0.016),
        origin=Origin(xyz=(chamber_center[0], chamber_center[1], 0.149)),
        material=body_white,
        name="chamber_skirt",
    )
    body.visual(
        chamber_cup,
        origin=Origin(xyz=chamber_center),
        material=smoky_clear,
        name="chamber_bowl",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(chamber_center[0], chamber_center[1], 0.156)),
        material=steel,
        name="drive_coupler",
    )
    body.visual(
        Box((0.112, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.087, 0.110)),
        material=trim_dark,
        name="control_bezel_top",
    )
    body.visual(
        Box((0.112, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.087, 0.058)),
        material=trim_dark,
        name="control_bezel_bottom",
    )
    body.visual(
        Box((0.018, 0.012, 0.064)),
        origin=Origin(xyz=(-0.047, -0.087, 0.084)),
        material=trim_dark,
        name="control_bezel_left",
    )
    body.visual(
        Box((0.018, 0.012, 0.064)),
        origin=Origin(xyz=(0.047, -0.087, 0.084)),
        material=trim_dark,
        name="control_bezel_right",
    )
    body.visual(
        Box((0.094, 0.004, 0.055)),
        origin=Origin(xyz=(0.0, -0.070, 0.084)),
        material=trim_dark,
        name="control_panel",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.003),
        origin=Origin(xyz=(-0.022, -0.0735, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="left_button_stop",
    )
    body.visual(
        Cylinder(radius=0.0038, length=0.003),
        origin=Origin(xyz=(0.022, -0.0735, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="right_button_stop",
    )
    body.visual(
        Box((0.146, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.090, 0.191)),
        material=trim_dark,
        name="rear_hinge_bridge",
    )
    body.visual(
        Box((0.030, 0.020, 0.026)),
        origin=Origin(xyz=(-0.054, 0.087, 0.197)),
        material=trim_dark,
        name="left_hinge_support",
    )
    body.visual(
        Box((0.030, 0.020, 0.026)),
        origin=Origin(xyz=(0.054, 0.087, 0.197)),
        material=trim_dark,
        name="right_hinge_support",
    )
    for foot_x in (-0.080, 0.080):
        for foot_y in (-0.055, 0.055):
            body.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(foot_x, foot_y, 0.003)),
                material=trim_dark,
                name=f"foot_{'l' if foot_x < 0 else 'r'}_{'f' if foot_y < 0 else 'b'}",
            )
    body.inertial = Inertial.from_geometry(
        Box((0.240, 0.180, 0.220)),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_dome,
        origin=Origin(xyz=(0.0, -chamber_rim_radius, 0.0)),
        material=clear_lid,
        name="lid_dome_shell",
    )
    lid.visual(
        lid_chute,
        origin=Origin(xyz=(0.0, -chamber_rim_radius, 0.078)),
        material=clear_lid,
        name="chute_sleeve",
    )
    lid.visual(
        Box((0.084, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.136, 0.025)),
        material=trim_dark,
        name="lid_handle",
    )
    lid.visual(
        Box((0.160, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.004, 0.008)),
        material=trim_dark,
        name="rear_hinge_strap",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.170, 0.170, 0.160)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.072, 0.080)),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.021, length=0.160),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=pusher_dark,
        name="pusher_stem",
    )
    pusher.visual(
        Cylinder(radius=0.027, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=pusher_dark,
        name="pusher_shoulder",
    )
    pusher.visual(
        Box((0.054, 0.054, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=pusher_dark,
        name="pusher_block",
    )
    pusher.visual(
        Box((0.060, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.181)),
        material=pusher_dark,
        name="pusher_grip",
    )
    pusher.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.210)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    basket = model.part("basket")
    basket.visual(
        basket_rim,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=basket_metal,
        name="basket_top_rim",
    )
    basket.visual(
        basket_lower_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=basket_metal,
        name="basket_lower_ring",
    )
    basket.visual(
        Cylinder(radius=0.011, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="drive_hub",
    )
    basket.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=steel,
        name="shredder_disc",
    )
    for index in range(12):
        angle = index * math.tau / 12.0
        basket.visual(
            Cylinder(radius=0.0016, length=0.034),
            origin=Origin(
                xyz=(0.050 * math.cos(angle), 0.050 * math.sin(angle), 0.026),
            ),
            material=basket_metal,
            name=f"basket_bar_{index:02d}",
        )
    for index in range(8):
        angle = index * math.tau / 8.0
        basket.visual(
            Box((0.050, 0.003, 0.004)),
            origin=Origin(
                xyz=(0.025 * math.cos(angle), 0.025 * math.sin(angle), 0.011),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"support_spoke_{index:02d}",
        )
        basket.visual(
            Box((0.030, 0.003, 0.006)),
            origin=Origin(
                xyz=(0.014 * math.cos(angle), 0.014 * math.sin(angle), 0.018),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"shredder_blade_{index:02d}",
        )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.050),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    left_button = model.part("left_button")
    left_button.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_red,
        name="button_cap",
    )
    left_button.visual(
        Cylinder(radius=0.0042, length=0.018),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="button_plunger",
    )
    left_button.inertial = Inertial.from_geometry(
        Box((0.024, 0.026, 0.024)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
    )

    right_button = model.part("right_button")
    right_button.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=button_gray,
        name="button_cap",
    )
    right_button.visual(
        Cylinder(radius=0.0038, length=0.018),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="button_plunger",
    )
    right_button.inertial = Inertial.from_geometry(
        Box((0.020, 0.026, 0.020)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, chamber_center[1] + chamber_rim_radius, chamber_rim_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -chamber_rim_radius, 0.078)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.080,
        ),
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(chamber_center[0], chamber_center[1], chamber_center[2] + 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=22.0),
    )
    model.articulation(
        "left_button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_button,
        origin=Origin(xyz=(-0.022, -0.092, 0.084)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "right_button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_button,
        origin=Origin(xyz=(0.022, -0.092, 0.084)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")
    lid_hinge = object_model.get_articulation("lid_hinge")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    left_button_press = object_model.get_articulation("left_button_press")
    right_button_press = object_model.get_articulation("right_button_press")
    basket_spin = object_model.get_articulation("basket_spin")

    with ctx.pose({pusher_slide: 0.0}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_stem",
            outer_elem="chute_sleeve",
            margin=0.003,
            name="pusher stem stays centered in chute at rest",
        )

    with ctx.pose({pusher_slide: pusher_slide.motion_limits.upper}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_stem",
            outer_elem="chute_sleeve",
            margin=0.003,
            name="pusher stem stays centered in chute when raised",
        )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: pusher_slide.motion_limits.upper}):
        pusher_raised = ctx.part_world_position(pusher)
    ctx.check(
        "pusher lifts upward",
        pusher_rest is not None
        and pusher_raised is not None
        and pusher_raised[2] > pusher_rest[2] + 0.05,
        details=f"rest={pusher_rest}, raised={pusher_raised}",
    )

    left_rest = ctx.part_world_position(left_button)
    right_rest = ctx.part_world_position(right_button)
    with ctx.pose(
        {
            left_button_press: left_button_press.motion_limits.upper,
            right_button_press: right_button_press.motion_limits.upper,
        }
    ):
        left_pressed = ctx.part_world_position(left_button)
        right_pressed = ctx.part_world_position(right_button)
    ctx.check(
        "left button presses inward",
        left_rest is not None
        and left_pressed is not None
        and left_pressed[1] > left_rest[1] + 0.004,
        details=f"rest={left_rest}, pressed={left_pressed}",
    )
    ctx.check(
        "right button presses inward",
        right_rest is not None
        and right_pressed is not None
        and right_pressed[1] > right_rest[1] + 0.004,
        details=f"rest={right_rest}, pressed={right_pressed}",
    )

    lid_handle_closed = ctx.part_element_world_aabb(lid, elem="lid_handle")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_handle_open = ctx.part_element_world_aabb(lid, elem="lid_handle")
    ctx.check(
        "lid opens upward on rear hinge",
        lid_handle_closed is not None
        and lid_handle_open is not None
        and ((lid_handle_open[0][2] + lid_handle_open[1][2]) * 0.5)
        > ((lid_handle_closed[0][2] + lid_handle_closed[1][2]) * 0.5) + 0.08,
        details=f"closed={lid_handle_closed}, open={lid_handle_open}",
    )

    basket_limits = basket_spin.motion_limits
    ctx.check(
        "basket uses continuous vertical spin axis",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 4) for v in basket_spin.axis) == (0.0, 0.0, 1.0)
        and basket_limits is not None
        and basket_limits.lower is None
        and basket_limits.upper is None,
        details=f"type={basket_spin.articulation_type}, axis={basket_spin.axis}, limits={basket_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
