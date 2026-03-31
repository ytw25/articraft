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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xz_section(
    width: float,
    height: float,
    radius: float,
    *,
    y: float,
    z_center: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for x, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def _seat_cushion_mesh():
    return section_loft(
        [
            _xz_section(0.44, 0.052, 0.020, y=0.195, z_center=0.046),
            _xz_section(0.48, 0.060, 0.024, y=0.030, z_center=0.050),
            _xz_section(0.47, 0.058, 0.022, y=-0.125, z_center=0.049),
            _xz_section(0.42, 0.048, 0.018, y=-0.245, z_center=0.044),
        ]
    )


def _arm_pad_mesh():
    return section_loft(
        [
            _xz_section(0.080, 0.026, 0.011, y=-0.125, z_center=0.039, corner_segments=6),
            _xz_section(0.104, 0.030, 0.013, y=0.000, z_center=0.041, corner_segments=6),
            _xz_section(0.086, 0.026, 0.011, y=0.125, z_center=0.038, corner_segments=6),
        ]
    )


def _gas_lift_shell_mesh(height: float):
    return LatheGeometry.from_shell_profiles(
        [(0.031, 0.0), (0.031, height)],
        [(0.024, 0.003), (0.024, height - 0.003)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _add_vertical_bolt(
    part,
    *,
    x: float,
    y: float,
    z_top: float,
    radius: float,
    height: float,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=height),
        origin=Origin(xyz=(x, y, z_top - height * 0.5)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ergonomic_office_chair")

    graphite = model.material("graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    aluminum = model.material("aluminum", rgba=(0.70, 0.72, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.84, 0.86, 0.89, 1.0))
    cushion_fabric = model.material("cushion_fabric", rgba=(0.20, 0.23, 0.26, 1.0))
    arm_pad_soft = model.material("arm_pad_soft", rgba=(0.16, 0.17, 0.18, 1.0))
    mesh_black = model.material("mesh_black", rgba=(0.11, 0.12, 0.13, 1.0))
    fastener = model.material("fastener", rgba=(0.62, 0.64, 0.68, 1.0))

    seat_tilt_upper = math.radians(22.5)
    back_recline_upper = math.radians(45.0)

    star_base = model.part("star_base")
    star_base.visual(
        Cylinder(radius=0.082, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=aluminum,
        name="hub_shell",
    )
    spoke_profile = rounded_rect_profile(0.070, 0.024, 0.010, corner_segments=8)
    caster_mounts: list[tuple[float, float, float, float]] = []
    for index in range(5):
        angle = index * (2.0 * math.pi / 5.0)
        c = math.cos(angle)
        s = math.sin(angle)
        path = [
            (0.040 * c, 0.040 * s, 0.091),
            (0.165 * c, 0.165 * s, 0.092),
            (0.310 * c, 0.310 * s, 0.086),
        ]
        star_base.visual(
            _mesh(
                f"office_chair_spoke_{index}",
                sweep_profile_along_spline(
                    path,
                    profile=spoke_profile,
                    samples_per_segment=10,
                    cap_profile=True,
                    up_hint=(0.0, 0.0, 1.0),
                ),
            ),
            material=aluminum,
            name=f"spoke_{index}",
        )
        tip_x = 0.314 * c
        tip_y = 0.314 * s
        star_base.visual(
            Cylinder(radius=0.018, length=0.034),
            origin=Origin(xyz=(tip_x, tip_y, 0.085)),
            material=dark_steel,
            name=f"socket_{index}",
        )
        caster_mounts.append((tip_x, tip_y, 0.068, angle))

    for index, (x, y, z, yaw) in enumerate(caster_mounts):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=0.0055, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, -0.016)),
            material=polished_steel,
            name="stem",
        )
        caster.visual(
            Box((0.030, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.041)),
            material=graphite,
            name="yoke_crown",
        )
        caster.visual(
            Box((0.004, 0.018, 0.028)),
            origin=Origin(xyz=(-0.018, 0.0, -0.055)),
            material=graphite,
            name="left_fork_plate",
        )
        caster.visual(
            Box((0.004, 0.018, 0.028)),
            origin=Origin(xyz=(0.018, 0.0, -0.055)),
            material=graphite,
            name="right_fork_plate",
        )
        caster.visual(
            Cylinder(radius=0.004, length=0.040),
            origin=Origin(xyz=(0.0, 0.0, -0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_steel,
            name="axle",
        )
        caster.visual(
            Cylinder(radius=0.030, length=0.012),
            origin=Origin(xyz=(-0.010, 0.0, -0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name="left_wheel",
        )
        caster.visual(
            Cylinder(radius=0.030, length=0.012),
            origin=Origin(xyz=(0.010, 0.0, -0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=graphite,
            name="right_wheel",
        )
        model.articulation(
            f"star_base_to_caster_{index}",
            ArticulationType.FIXED,
            parent=star_base,
            child=caster,
            origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, yaw)),
        )

    gas_lift_outer = model.part("gas_lift_outer")
    gas_lift_outer.visual(
        _mesh("office_chair_gas_lift_outer_shell", _gas_lift_shell_mesh(0.215)),
        material=dark_steel,
        name="outer_shell",
    )
    gas_lift_outer.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=polished_steel,
        name="lower_insert",
    )
    gas_lift_outer.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        material=polished_steel,
        name="upper_bearing_cap",
    )
    model.articulation(
        "star_base_to_gas_lift_outer",
        ArticulationType.FIXED,
        parent=star_base,
        child=gas_lift_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    gas_lift_inner = model.part("gas_lift_inner")
    gas_lift_inner.visual(
        Cylinder(radius=0.018, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.229)),
        material=polished_steel,
        name="piston_rod",
    )
    gas_lift_inner.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.248)),
        material=polished_steel,
        name="bearing_head",
    )
    gas_lift_inner.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.259)),
        material=dark_steel,
        name="seat_plate",
    )
    model.articulation(
        "gas_lift_travel",
        ArticulationType.PRISMATIC,
        parent=gas_lift_outer,
        child=gas_lift_inner,
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2500.0,
            velocity=0.25,
            lower=0.0,
            upper=0.100,
        ),
    )

    seat_assembly = model.part("seat_assembly")
    seat_assembly.visual(
        Box((0.280, 0.280, 0.045)),
        origin=Origin(xyz=(0.0, -0.055, 0.0225)),
        material=dark_steel,
        name="control_box",
    )
    seat_assembly.visual(
        Box((0.240, 0.050, 0.080)),
        origin=Origin(xyz=(0.0, -0.215, 0.040)),
        material=dark_steel,
        name="rear_bridge",
    )
    seat_assembly.visual(
        Box((0.035, 0.025, 0.090)),
        origin=Origin(xyz=(-0.170, -0.2475, 0.045)),
        material=dark_steel,
        name="left_back_bracket",
    )
    seat_assembly.visual(
        Box((0.035, 0.025, 0.090)),
        origin=Origin(xyz=(0.170, -0.2475, 0.045)),
        material=dark_steel,
        name="right_back_bracket",
    )
    seat_assembly.visual(
        Box((0.480, 0.400, 0.010)),
        origin=Origin(xyz=(0.0, -0.020, 0.050)),
        material=graphite,
        name="seat_pan_plate",
    )
    seat_assembly.visual(
        _mesh("office_chair_seat_cushion", _seat_cushion_mesh()),
        material=cushion_fabric,
        name="seat_cushion",
    )
    seat_assembly.visual(
        Cylinder(radius=0.032, length=0.160),
        origin=Origin(xyz=(0.0, 0.108, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="tilt_spring_housing",
    )
    seat_assembly.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.160, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="tension_knob",
    )
    seat_assembly.visual(
        Cylinder(radius=0.007, length=0.020),
        origin=Origin(xyz=(0.155, 0.010, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="lock_lever_pivot",
    )
    seat_assembly.visual(
        Cylinder(radius=0.005, length=0.090),
        origin=Origin(xyz=(0.165, 0.052, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="lock_lever_handle",
    )
    seat_assembly.visual(
        Box((0.018, 0.100, 0.060)),
        origin=Origin(xyz=(-0.246, -0.020, 0.030)),
        material=dark_steel,
        name="left_arm_boss",
    )
    seat_assembly.visual(
        Box((0.018, 0.100, 0.060)),
        origin=Origin(xyz=(0.246, -0.020, 0.030)),
        material=dark_steel,
        name="right_arm_boss",
    )
    for bolt_index, (x, y) in enumerate(((-0.095, -0.090), (-0.095, 0.045), (0.095, -0.090), (0.095, 0.045))):
        _add_vertical_bolt(
            seat_assembly,
            x=x,
            y=y,
            z_top=0.050,
            radius=0.0065,
            height=0.008,
            material=fastener,
            name=f"seat_mount_bolt_{bolt_index}",
        )
    for bolt_index, x in enumerate((-0.170, 0.170)):
        seat_assembly.visual(
            Cylinder(radius=0.0065, length=0.016),
            origin=Origin(xyz=(x, -0.2475, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=fastener,
            name=f"backrest_mount_bolt_{bolt_index}",
        )
    model.articulation(
        "seat_tilt",
        ArticulationType.REVOLUTE,
        parent=gas_lift_inner,
        child=seat_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.265)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=seat_tilt_upper,
        ),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.035, 0.026, 0.022)),
        origin=Origin(xyz=(-0.170, 0.013, 0.011)),
        material=dark_steel,
        name="left_hinge_foot",
    )
    backrest.visual(
        Box((0.035, 0.026, 0.022)),
        origin=Origin(xyz=(0.170, 0.013, 0.011)),
        material=dark_steel,
        name="right_hinge_foot",
    )
    backrest.visual(
        Box((0.040, 0.030, 0.620)),
        origin=Origin(xyz=(-0.170, 0.000, 0.321)),
        material=graphite,
        name="left_upright",
    )
    backrest.visual(
        Box((0.040, 0.030, 0.620)),
        origin=Origin(xyz=(0.170, 0.000, 0.321)),
        material=graphite,
        name="right_upright",
    )
    backrest.visual(
        Box((0.320, 0.030, 0.070)),
        origin=Origin(xyz=(0.0, 0.000, 0.612)),
        material=graphite,
        name="top_rail",
    )
    backrest.visual(
        Box((0.260, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, -0.004, 0.430)),
        material=graphite,
        name="mid_crossbar",
    )
    backrest.visual(
        Box((0.050, 0.020, 0.640)),
        origin=Origin(xyz=(0.0, -0.005, 0.322)),
        material=dark_steel,
        name="rear_spine",
    )
    backrest.visual(
        Box((0.050, 0.008, 0.240)),
        origin=Origin(xyz=(0.0, -0.019, 0.330)),
        material=polished_steel,
        name="lumbar_track",
    )
    backrest.visual(
        Box((0.120, 0.008, 0.260)),
        origin=Origin(xyz=(-0.095, -0.010, 0.320)),
        material=mesh_black,
        name="left_back_panel",
    )
    backrest.visual(
        Box((0.120, 0.008, 0.260)),
        origin=Origin(xyz=(0.095, -0.010, 0.320)),
        material=mesh_black,
        name="right_back_panel",
    )
    backrest.visual(
        Box((0.220, 0.008, 0.100)),
        origin=Origin(xyz=(0.0, -0.010, 0.545)),
        material=mesh_black,
        name="shoulder_panel",
    )
    model.articulation(
        "backrest_recline",
        ArticulationType.REVOLUTE,
        parent=seat_assembly,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.286, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.1,
            lower=0.0,
            upper=back_recline_upper,
        ),
    )

    lumbar_support = model.part("lumbar_support")
    lumbar_support.visual(
        Box((0.220, 0.018, 0.085)),
        origin=Origin(xyz=(0.0, 0.020, 0.0425)),
        material=cushion_fabric,
        name="lumbar_pad",
    )
    lumbar_support.visual(
        Box((0.060, 0.014, 0.070)),
        origin=Origin(xyz=(0.0, 0.012, 0.035)),
        material=dark_steel,
        name="carrier_block",
    )
    lumbar_support.visual(
        Box((0.050, 0.004, 0.080)),
        origin=Origin(xyz=(0.0, 0.008, 0.040)),
        material=polished_steel,
        name="track_shoe",
    )
    model.articulation(
        "lumbar_height",
        ArticulationType.PRISMATIC,
        parent=backrest,
        child=lumbar_support,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.12,
            lower=0.0,
            upper=0.120,
        ),
    )

    arm_pad_mesh = _mesh("office_chair_arm_pad", _arm_pad_mesh())
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        pillar = model.part(f"{side_name}_arm_pillar")
        pillar.visual(
            Box((0.018, 0.100, 0.060)),
            origin=Origin(xyz=(0.009 * side_sign, 0.0, 0.0)),
            material=dark_steel,
            name="mount_plate",
        )
        pillar.visual(
            Box((0.050, 0.060, 0.180)),
            origin=Origin(xyz=(0.043 * side_sign, 0.0, 0.060)),
            material=graphite,
            name="lower_sleeve",
        )
        pillar.visual(
            Box((0.060, 0.075, 0.020)),
            origin=Origin(xyz=(0.043 * side_sign, 0.0, 0.150)),
            material=graphite,
            name="upper_collar",
        )
        for bolt_index, y in enumerate((-0.030, 0.030)):
            pillar.visual(
                Cylinder(radius=0.005, length=0.006),
                origin=Origin(xyz=(0.000, y, 0.030)),
                material=fastener,
                name=f"m6_mount_{bolt_index}",
            )
        model.articulation(
            f"seat_to_{side_name}_arm_pillar",
            ArticulationType.FIXED,
            parent=seat_assembly,
            child=pillar,
            origin=Origin(xyz=(0.255 * side_sign, -0.020, 0.030)),
        )

        height_slider = model.part(f"{side_name}_arm_height_slider")
        height_slider.visual(
            Box((0.055, 0.070, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
            material=graphite,
            name="height_collar",
        )
        height_slider.visual(
            Box((0.034, 0.040, 0.120)),
            origin=Origin(xyz=(0.0, 0.0, 0.060)),
            material=dark_steel,
            name="height_post",
        )
        height_slider.visual(
            Box((0.060, 0.080, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, 0.126)),
            material=graphite,
            name="slider_saddle",
        )
        model.articulation(
            f"{side_name}_arm_height",
            ArticulationType.PRISMATIC,
            parent=pillar,
            child=height_slider,
            origin=Origin(xyz=(0.043 * side_sign, 0.0, 0.160)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=40.0,
                velocity=0.15,
                lower=0.0,
                upper=0.090,
            ),
        )

        fore_aft_slider = model.part(f"{side_name}_arm_fore_aft_slider")
        fore_aft_slider.visual(
            Box((0.070, 0.140, 0.016)),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=dark_steel,
            name="fore_aft_rail",
        )
        fore_aft_slider.visual(
            Box((0.012, 0.030, 0.024)),
            origin=Origin(xyz=(-0.020, -0.045, 0.012)),
            material=graphite,
            name="rear_guide_left",
        )
        fore_aft_slider.visual(
            Box((0.012, 0.030, 0.024)),
            origin=Origin(xyz=(0.020, -0.045, 0.012)),
            material=graphite,
            name="rear_guide_right",
        )
        fore_aft_slider.visual(
            Box((0.012, 0.030, 0.024)),
            origin=Origin(xyz=(-0.020, 0.045, 0.012)),
            material=graphite,
            name="front_guide_left",
        )
        fore_aft_slider.visual(
            Box((0.012, 0.030, 0.024)),
            origin=Origin(xyz=(0.020, 0.045, 0.012)),
            material=graphite,
            name="front_guide_right",
        )
        model.articulation(
            f"{side_name}_arm_slide",
            ArticulationType.PRISMATIC,
            parent=height_slider,
            child=fore_aft_slider,
            origin=Origin(xyz=(0.0, 0.0, 0.134)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=30.0,
                velocity=0.14,
                lower=-0.040,
                upper=0.040,
            ),
        )

        arm_pad = model.part(f"{side_name}_arm_pad")
        arm_pad.visual(
            Cylinder(radius=0.028, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=graphite,
            name="swivel_disc",
        )
        arm_pad.visual(
            Box((0.055, 0.060, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, 0.020)),
            material=dark_steel,
            name="pad_stem",
        )
        arm_pad.visual(
            Box((0.070, 0.110, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.030)),
            material=dark_steel,
            name="pad_subplate",
        )
        arm_pad.visual(
            arm_pad_mesh,
            material=arm_pad_soft,
            name="pad_shell",
        )
        model.articulation(
            f"{side_name}_arm_swivel",
            ArticulationType.REVOLUTE,
            parent=fore_aft_slider,
            child=arm_pad,
            origin=Origin(xyz=(0.0, 0.0, 0.016)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.5,
                lower=-math.radians(20.0),
                upper=math.radians(20.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    star_base = object_model.get_part("star_base")
    gas_lift_outer = object_model.get_part("gas_lift_outer")
    gas_lift_inner = object_model.get_part("gas_lift_inner")
    seat_assembly = object_model.get_part("seat_assembly")
    backrest = object_model.get_part("backrest")
    lumbar_support = object_model.get_part("lumbar_support")
    left_arm_pillar = object_model.get_part("left_arm_pillar")
    left_arm_height_slider = object_model.get_part("left_arm_height_slider")
    left_arm_fore_aft_slider = object_model.get_part("left_arm_fore_aft_slider")
    left_arm_pad = object_model.get_part("left_arm_pad")
    right_arm_pillar = object_model.get_part("right_arm_pillar")
    right_arm_height_slider = object_model.get_part("right_arm_height_slider")
    right_arm_fore_aft_slider = object_model.get_part("right_arm_fore_aft_slider")
    right_arm_pad = object_model.get_part("right_arm_pad")

    gas_lift_travel = object_model.get_articulation("gas_lift_travel")
    seat_tilt = object_model.get_articulation("seat_tilt")
    backrest_recline = object_model.get_articulation("backrest_recline")
    lumbar_height = object_model.get_articulation("lumbar_height")
    left_arm_height = object_model.get_articulation("left_arm_height")
    left_arm_slide = object_model.get_articulation("left_arm_slide")
    left_arm_swivel = object_model.get_articulation("left_arm_swivel")
    right_arm_height = object_model.get_articulation("right_arm_height")
    right_arm_slide = object_model.get_articulation("right_arm_slide")
    right_arm_swivel = object_model.get_articulation("right_arm_swivel")

    expected_parts = {
        "star_base",
        "gas_lift_outer",
        "gas_lift_inner",
        "seat_assembly",
        "backrest",
        "lumbar_support",
        "left_arm_pillar",
        "left_arm_height_slider",
        "left_arm_fore_aft_slider",
        "left_arm_pad",
        "right_arm_pillar",
        "right_arm_height_slider",
        "right_arm_fore_aft_slider",
        "right_arm_pad",
        "caster_0",
        "caster_1",
        "caster_2",
        "caster_3",
        "caster_4",
    }
    authored_part_names = {part.name for part in object_model.parts}
    for part_name in sorted(expected_parts):
        ctx.check(
            f"{part_name}_present",
            part_name in authored_part_names,
            details=f"Missing required part: {part_name}",
        )

    for caster_index in range(5):
        ctx.expect_contact(
            object_model.get_part(f"caster_{caster_index}"),
            star_base,
            name=f"caster_{caster_index}_mounted",
        )
        ctx.expect_origin_gap(
            object_model.get_part(f"caster_{caster_index}"),
            star_base,
            axis="z",
            min_gap=0.060,
            max_gap=0.080,
            name=f"caster_{caster_index}_below_star_base",
        )

    contact_pairs = (
        (gas_lift_outer, star_base, "gas_lift_outer_mounted"),
        (gas_lift_inner, gas_lift_outer, "gas_lift_inner_guided"),
        (seat_assembly, gas_lift_inner, "seat_mechanism_supported"),
        (backrest, seat_assembly, "backrest_hinged_to_seat"),
        (lumbar_support, backrest, "lumbar_block_guided"),
        (left_arm_pillar, seat_assembly, "left_arm_pillar_mounted"),
        (left_arm_height_slider, left_arm_pillar, "left_height_slider_supported"),
        (left_arm_fore_aft_slider, left_arm_height_slider, "left_fore_aft_slider_supported"),
        (left_arm_pad, left_arm_fore_aft_slider, "left_arm_pad_supported"),
        (right_arm_pillar, seat_assembly, "right_arm_pillar_mounted"),
        (right_arm_height_slider, right_arm_pillar, "right_height_slider_supported"),
        (right_arm_fore_aft_slider, right_arm_height_slider, "right_fore_aft_slider_supported"),
        (right_arm_pad, right_arm_fore_aft_slider, "right_arm_pad_supported"),
    )
    for child, parent, check_name in contact_pairs:
        ctx.expect_contact(child, parent, name=check_name)

    ctx.expect_gap(
        seat_assembly,
        star_base,
        axis="z",
        min_gap=0.210,
        name="seat_clears_base",
    )
    ctx.expect_gap(
        left_arm_pad,
        seat_assembly,
        axis="z",
        min_gap=0.120,
        name="left_arm_pad_above_seat",
    )
    ctx.expect_gap(
        right_arm_pad,
        seat_assembly,
        axis="z",
        min_gap=0.120,
        name="right_arm_pad_above_seat",
    )
    ctx.expect_within(
        lumbar_support,
        backrest,
        axes="x",
        margin=0.020,
        name="lumbar_within_backrest_width",
    )

    ctx.check(
        "seat_back_sync_ratio_limits",
        backrest_recline.motion_limits is not None
        and seat_tilt.motion_limits is not None
        and backrest_recline.motion_limits.upper is not None
        and seat_tilt.motion_limits.upper is not None
        and abs(backrest_recline.motion_limits.upper - 2.0 * seat_tilt.motion_limits.upper) < 1e-9,
        details="Backrest recline limit should be exactly double the seat tilt limit for 2:1 synchronous motion.",
    )
    ctx.check(
        "primary_recline_axes_collinear",
        tuple(seat_tilt.axis) == (1.0, 0.0, 0.0) and tuple(backrest_recline.axis) == (1.0, 0.0, 0.0),
        details="Seat tilt and backrest recline axes must be collinear x-axis pivots.",
    )
    ctx.check(
        "gas_lift_axis_vertical",
        tuple(gas_lift_travel.axis) == (0.0, 0.0, 1.0),
        details="Gas lift must translate on the world Z axis.",
    )
    ctx.check(
        "lumbar_axis_vertical",
        tuple(lumbar_height.axis) == (0.0, 0.0, 1.0),
        details="Lumbar support must translate vertically on the backrest spine.",
    )
    ctx.check(
        "arm_adjustment_axes_correct",
        tuple(left_arm_height.axis) == (0.0, 0.0, 1.0)
        and tuple(right_arm_height.axis) == (0.0, 0.0, 1.0)
        and tuple(left_arm_slide.axis) == (0.0, 1.0, 0.0)
        and tuple(right_arm_slide.axis) == (0.0, 1.0, 0.0)
        and tuple(left_arm_swivel.axis) == (0.0, 0.0, 1.0)
        and tuple(right_arm_swivel.axis) == (0.0, 0.0, 1.0),
        details="Armrests must provide Z lift, Y slide, and Z-axis swivel.",
    )

    seat_rest = ctx.part_world_position(seat_assembly)
    assert seat_rest is not None
    with ctx.pose({gas_lift_travel: 0.100}):
        seat_high = ctx.part_world_position(seat_assembly)
        assert seat_high is not None
        ctx.check(
            "gas_lift_has_100mm_travel",
            seat_high[2] > seat_rest[2] + 0.095,
            details=f"Expected at least 95 mm rise, got {seat_high[2] - seat_rest[2]:.4f} m.",
        )

    left_pad_rest = ctx.part_world_position(left_arm_pad)
    assert left_pad_rest is not None
    with ctx.pose({left_arm_height: 0.090}):
        left_pad_high = ctx.part_world_position(left_arm_pad)
        assert left_pad_high is not None
        ctx.check(
            "left_arm_height_adjusts_upward",
            left_pad_high[2] > left_pad_rest[2] + 0.085,
            details=f"Left arm height gain too small: {left_pad_high[2] - left_pad_rest[2]:.4f} m.",
        )

    with ctx.pose({left_arm_slide: 0.040}):
        left_pad_forward = ctx.part_world_position(left_arm_pad)
        assert left_pad_forward is not None
        ctx.check(
            "left_arm_slides_forward",
            left_pad_forward[1] > left_pad_rest[1] + 0.035,
            details=f"Left arm forward travel too small: {left_pad_forward[1] - left_pad_rest[1]:.4f} m.",
        )

    backrest_rest_aabb = ctx.part_world_aabb(backrest)
    assert backrest_rest_aabb is not None
    with ctx.pose({seat_tilt: math.radians(15.0), backrest_recline: math.radians(30.0)}):
        backrest_sync_aabb = ctx.part_world_aabb(backrest)
        assert backrest_sync_aabb is not None
        ctx.check(
            "backrest_reclines_rearward_in_sync_pose",
            backrest_sync_aabb[0][1] < backrest_rest_aabb[0][1] - 0.050,
            details="Backrest did not translate rearward enough in the representative synchronous recline pose.",
        )
        ctx.expect_gap(
            backrest,
            star_base,
            axis="z",
            min_gap=0.180,
            name="backrest_clears_base_in_recline",
        )

    lumbar_rest = ctx.part_world_position(lumbar_support)
    assert lumbar_rest is not None
    with ctx.pose({lumbar_height: 0.120}):
        lumbar_high = ctx.part_world_position(lumbar_support)
        assert lumbar_high is not None
        ctx.check(
            "lumbar_support_adjusts_upward",
            lumbar_high[2] > lumbar_rest[2] + 0.110,
            details=f"Lumbar travel too small: {lumbar_high[2] - lumbar_rest[2]:.4f} m.",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=20,
        ignore_adjacent=True,
        ignore_fixed=True,
        name="non_adjacent_clearance_in_sampled_poses",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
