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
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_section(
    *,
    width: float,
    height: float,
    radius: float,
    y: float,
    z_center: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    return [(x, y, z + z_center) for x, z in profile]


def _wheel_visuals(part, *, tire_radius: float, tire_width: float, hub_radius: float, hub_width: float, rubber, frame):
    spin_origin = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=tire_radius, length=tire_width),
        origin=spin_origin,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=hub_radius, length=hub_width),
        origin=spin_origin,
        material=frame,
        name="hub_core",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame,
        name="minus_x_cap",
    )
    part.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame,
        name="plus_x_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soft_travel_suitcase")

    fabric = model.material("fabric_charcoal", rgba=(0.23, 0.25, 0.28, 1.0))
    frame = model.material("frame_graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    rubber = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    trim = model.material("trim_black", rgba=(0.09, 0.09, 0.10, 1.0))

    case_body = model.part("case_body")
    shell = section_loft(
        [
            _rounded_section(width=0.392, height=0.530, radius=0.055, y=-0.132, z_center=0.390),
            _rounded_section(width=0.412, height=0.548, radius=0.058, y=-0.040, z_center=0.392),
            _rounded_section(width=0.418, height=0.552, radius=0.058, y=0.048, z_center=0.392),
            _rounded_section(width=0.398, height=0.538, radius=0.055, y=0.132, z_center=0.390),
        ]
    )
    case_body.visual(_mesh("suitcase_fabric_shell", shell), material=fabric, name="fabric_shell")
    case_body.visual(
        Box((0.180, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, -0.085, 0.035)),
        material=frame,
        name="front_beam",
    )
    case_body.visual(
        Box((0.085, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, -0.122, 0.009)),
        material=rubber,
        name="front_foot",
    )
    case_body.visual(
        Box((0.050, 0.220, 0.080)),
        origin=Origin(xyz=(-0.105, 0.0, 0.080)),
        material=frame,
        name="left_lower_rail",
    )
    case_body.visual(
        Box((0.050, 0.220, 0.080)),
        origin=Origin(xyz=(0.105, 0.0, 0.080)),
        material=frame,
        name="right_lower_rail",
    )
    case_body.visual(
        Box((0.180, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, 0.095, 0.085)),
        material=frame,
        name="rear_cross_beam",
    )
    case_body.visual(
        Box((0.300, 0.018, 0.400)),
        origin=Origin(xyz=(0.0, 0.133, 0.300)),
        material=frame,
        name="rear_plate",
    )
    case_body.visual(
        Box((0.022, 0.030, 0.500)),
        origin=Origin(xyz=(-0.186, -0.108, 0.390)),
        material=frame,
        name="left_front_frame",
    )
    case_body.visual(
        Box((0.022, 0.030, 0.500)),
        origin=Origin(xyz=(0.186, -0.108, 0.390)),
        material=frame,
        name="right_front_frame",
    )
    case_body.visual(
        Box((0.022, 0.030, 0.540)),
        origin=Origin(xyz=(-0.186, 0.108, 0.390)),
        material=frame,
        name="left_rear_frame",
    )
    case_body.visual(
        Box((0.022, 0.030, 0.540)),
        origin=Origin(xyz=(0.186, 0.108, 0.390)),
        material=frame,
        name="right_rear_frame",
    )
    case_body.visual(
        Box((0.140, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, 0.018, 0.666)),
        material=trim,
        name="top_handle_pad",
    )

    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        x_channel = side_sign * 0.110
        x_outer = side_sign * 0.123
        x_inner = side_sign * 0.097
        case_body.visual(
            Box((0.028, 0.008, 0.410)),
            origin=Origin(xyz=(x_channel, 0.144, 0.295)),
            material=frame,
            name=f"{side_name}_channel_back",
        )
        case_body.visual(
            Box((0.006, 0.020, 0.410)),
            origin=Origin(xyz=(x_outer, 0.151, 0.295)),
            material=frame,
            name=f"{side_name}_channel_outer",
        )
        case_body.visual(
            Box((0.006, 0.020, 0.410)),
            origin=Origin(xyz=(x_inner, 0.151, 0.295)),
            material=frame,
            name=f"{side_name}_channel_inner",
        )

        x_shroud = side_sign * 0.172
        x_axle = side_sign * 0.145
        x_inner_shroud = side_sign * 0.138
        case_body.visual(
            Box((0.060, 0.084, 0.060)),
            origin=Origin(xyz=(x_shroud, 0.098, 0.152)),
            material=frame,
            name=f"{side_name}_wheel_shroud",
        )
        case_body.visual(
            Box((0.020, 0.066, 0.080)),
            origin=Origin(xyz=(x_inner_shroud, 0.103, 0.150)),
            material=frame,
            name=f"{side_name}_inner_shroud",
        )
        case_body.visual(
            Box((0.012, 0.028, 0.082)),
            origin=Origin(xyz=(x_axle, 0.118, 0.101)),
            material=frame,
            name=f"{side_name}_axle_tower",
        )

    case_body.visual(
        Box((0.370, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, -0.129, 0.555)),
        material=trim,
        name="pocket_top_band",
    )
    case_body.visual(
        Box((0.020, 0.012, 0.184)),
        origin=Origin(xyz=(-0.173, -0.129, 0.456)),
        material=trim,
        name="pocket_left_band",
    )
    case_body.visual(
        Box((0.020, 0.012, 0.184)),
        origin=Origin(xyz=(0.173, -0.129, 0.456)),
        material=trim,
        name="pocket_right_band",
    )
    case_body.visual(
        Box((0.332, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.129, 0.357)),
        material=trim,
        name="pocket_bottom_band",
    )
    case_body.inertial = Inertial.from_geometry(
        Box((0.430, 0.300, 0.700)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
    )

    lower_handle = model.part("lower_handle_stage")
    lower_handle.visual(
        Cylinder(radius=0.009, length=0.212),
        origin=Origin(xyz=(0.0, 0.0, 0.140), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="lower_handle_crossbar",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        x_rail = side_sign * 0.110
        lower_handle.visual(
            Box((0.022, 0.006, 0.500)),
            origin=Origin(xyz=(x_rail, -0.0049, -0.110)),
            material=frame,
            name=f"{side_name}_sleeve_back",
        )
        lower_handle.visual(
            Box((0.006, 0.014, 0.500)),
            origin=Origin(xyz=(x_rail + (side_sign * 0.008), 0.0025, -0.110)),
            material=frame,
            name=f"{side_name}_sleeve_outer",
        )
        lower_handle.visual(
            Box((0.006, 0.014, 0.500)),
            origin=Origin(xyz=(x_rail - (side_sign * 0.008), 0.0025, -0.110)),
            material=frame,
            name=f"{side_name}_sleeve_inner",
        )
    lower_handle.inertial = Inertial.from_geometry(
        Box((0.240, 0.030, 0.520)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )

    upper_handle = model.part("upper_handle_stage")
    upper_handle.visual(
        Cylinder(radius=0.0045, length=0.360),
        origin=Origin(xyz=(-0.110, 0.0045, -0.050)),
        material=aluminum,
        name="left_upper_post",
    )
    upper_handle.visual(
        Cylinder(radius=0.0045, length=0.360),
        origin=Origin(xyz=(0.110, 0.0045, -0.050)),
        material=aluminum,
        name="right_upper_post",
    )
    upper_handle.visual(
        Cylinder(radius=0.0065, length=0.224),
        origin=Origin(xyz=(0.0, 0.0045, 0.130), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="upper_handle_crossbar",
    )
    upper_handle.visual(
        Box((0.160, 0.030, 0.032)),
        origin=Origin(xyz=(0.0, 0.008, 0.130)),
        material=trim,
        name="grip",
    )
    upper_handle.inertial = Inertial.from_geometry(
        Box((0.185, 0.035, 0.390)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    left_wheel = model.part("left_wheel")
    _wheel_visuals(
        left_wheel,
        tire_radius=0.060,
        tire_width=0.050,
        hub_radius=0.038,
        hub_width=0.036,
        rubber=rubber,
        frame=frame,
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.050),
        mass=0.55,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    _wheel_visuals(
        right_wheel,
        tire_radius=0.060,
        tire_width=0.050,
        hub_radius=0.038,
        hub_width=0.036,
        rubber=rubber,
        frame=frame,
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.050),
        mass=0.55,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    document_flap = model.part("document_flap")
    document_flap.visual(
        Box((0.350, 0.006, 0.192)),
        origin=Origin(xyz=(0.0, -0.018, -0.096)),
        material=fabric,
        name="flap_panel",
    )
    document_flap.visual(
        Box((0.350, 0.012, 0.026)),
        origin=Origin(xyz=(0.0, -0.015, -0.013)),
        material=trim,
        name="top_binding",
    )
    document_flap.visual(
        Box((0.055, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, -0.020, -0.190)),
        material=trim,
        name="pull_tab",
    )
    document_flap.inertial = Inertial.from_geometry(
        Box((0.355, 0.016, 0.210)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.012, -0.095)),
    )

    model.articulation(
        "case_to_lower_handle",
        ArticulationType.PRISMATIC,
        parent=case_body,
        child=lower_handle,
        origin=Origin(xyz=(0.0, 0.156, 0.495)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.160),
    )
    model.articulation(
        "lower_to_upper_handle",
        ArticulationType.PRISMATIC,
        parent=lower_handle,
        child=upper_handle,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.25, lower=0.0, upper=0.170),
    )
    model.articulation(
        "case_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=case_body,
        child=left_wheel,
        origin=Origin(xyz=(-0.176, 0.120, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0),
    )
    model.articulation(
        "case_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=case_body,
        child=right_wheel,
        origin=Origin(xyz=(0.176, 0.120, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=30.0),
    )
    model.articulation(
        "case_to_document_flap",
        ArticulationType.REVOLUTE,
        parent=case_body,
        child=document_flap,
        origin=Origin(xyz=(0.0, -0.126, 0.565)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case_body = object_model.get_part("case_body")
    lower_handle = object_model.get_part("lower_handle_stage")
    upper_handle = object_model.get_part("upper_handle_stage")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    document_flap = object_model.get_part("document_flap")

    lower_slide = object_model.get_articulation("case_to_lower_handle")
    upper_slide = object_model.get_articulation("lower_to_upper_handle")
    left_spin = object_model.get_articulation("case_to_left_wheel")
    right_spin = object_model.get_articulation("case_to_right_wheel")
    flap_hinge = object_model.get_articulation("case_to_document_flap")

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

    for part_name, part in (
        ("case body", case_body),
        ("lower handle stage", lower_handle),
        ("upper handle stage", upper_handle),
        ("left wheel", left_wheel),
        ("right wheel", right_wheel),
        ("document flap", document_flap),
    ):
        ctx.check(f"{part_name} exists", part is not None, details=f"missing {part_name}")

    ctx.expect_gap(
        lower_handle,
        case_body,
        axis="y",
        positive_elem="left_sleeve_back",
        negative_elem="left_channel_back",
        min_gap=0.0,
        max_gap=0.002,
        name="lower handle stage runs with a tight rear-channel clearance",
    )
    ctx.expect_gap(
        upper_handle,
        lower_handle,
        axis="y",
        positive_elem="left_upper_post",
        negative_elem="left_sleeve_back",
        min_gap=0.0,
        max_gap=0.012,
        name="upper handle stage sits just ahead of the lower sleeve back wall",
    )
    ctx.expect_within(
        upper_handle,
        lower_handle,
        axes="x",
        inner_elem="left_upper_post",
        outer_elem="left_sleeve_back",
        margin=0.012,
        name="upper handle stage stays laterally captured by the lower sleeve",
    )
    ctx.expect_contact(
        left_wheel,
        case_body,
        elem_a="plus_x_cap",
        elem_b="left_axle_tower",
        name="left wheel sits on its inset axle tower",
    )
    ctx.expect_contact(
        right_wheel,
        case_body,
        elem_a="minus_x_cap",
        elem_b="right_axle_tower",
        name="right wheel sits on its inset axle tower",
    )
    ctx.expect_contact(
        document_flap,
        case_body,
        elem_a="top_binding",
        elem_b="pocket_top_band",
        name="document flap closes against the pocket frame",
    )

    ctx.check(
        "handle stages slide vertically",
        lower_slide.axis == (0.0, 0.0, 1.0) and upper_slide.axis == (0.0, 0.0, 1.0),
        details=f"lower axis={lower_slide.axis}, upper axis={upper_slide.axis}",
    )
    ctx.check(
        "rear wheels spin on lateral axles",
        left_spin.axis == (1.0, 0.0, 0.0)
        and right_spin.axis == (1.0, 0.0, 0.0)
        and left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"left={left_spin.axis}/{left_spin.articulation_type}, right={right_spin.axis}/{right_spin.articulation_type}",
    )
    ctx.check(
        "document flap hinges about the top edge",
        flap_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"flap axis={flap_hinge.axis}",
    )

    lower_rest = ctx.part_world_position(lower_handle)
    upper_rest = ctx.part_world_position(upper_handle)
    lower_extended = None
    upper_extended = None
    with ctx.pose(
        {
            lower_slide: lower_slide.motion_limits.upper or 0.0,
            upper_slide: upper_slide.motion_limits.upper or 0.0,
        }
    ):
        lower_extended = ctx.part_world_position(lower_handle)
        upper_extended = ctx.part_world_position(upper_handle)

    ctx.check(
        "pull handle telescopes upward",
        lower_rest is not None
        and upper_rest is not None
        and lower_extended is not None
        and upper_extended is not None
        and lower_extended[2] > lower_rest[2] + 0.12
        and upper_extended[2] > upper_rest[2] + 0.28,
        details=f"lower rest={lower_rest}, lower extended={lower_extended}, upper rest={upper_rest}, upper extended={upper_extended}",
    )

    flap_closed_aabb = ctx.part_element_world_aabb(document_flap, elem="flap_panel")
    flap_open_aabb = None
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper or 0.0}):
        flap_open_aabb = ctx.part_element_world_aabb(document_flap, elem="flap_panel")

    ctx.check(
        "document flap swings outward",
        flap_closed_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[0][1] < flap_closed_aabb[0][1] - 0.10,
        details=f"closed={flap_closed_aabb}, open={flap_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
