from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin, sqrt
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
)

def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = atan2(dy, dx)
    pitch = atan2(hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None):
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_inner_arm(part, *, arm_material, mount_material) -> None:
    part.visual(
        Box((0.085, 0.050, 0.022)),
        origin=Origin(xyz=(0.0425, 0.0, 0.011)),
        material=mount_material,
        name="root_clamp",
    )
    part.visual(
        Box((0.120, 0.030, 0.016)),
        origin=Origin(xyz=(0.118, 0.0, 0.012)),
        material=arm_material,
        name="inner_beam",
    )
    part.visual(
        Box((0.070, 0.016, 0.006)),
        origin=Origin(xyz=(0.095, 0.0, 0.023)),
        material=mount_material,
        name="cable_guide",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.185, 0.0, 0.002)),
        material=mount_material,
        name="hinge_saddle",
    )
    part.visual(
        Box((0.034, 0.006, 0.024)),
        origin=Origin(xyz=(0.185, 0.015, 0.012)),
        material=mount_material,
        name="hinge_cheek_left",
    )
    part.visual(
        Box((0.034, 0.006, 0.024)),
        origin=Origin(xyz=(0.185, -0.015, 0.012)),
        material=mount_material,
        name="hinge_cheek_right",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.185, 0.0, 0.022)),
        material=mount_material,
        name="hinge_cap",
    )
    part.visual(
        Box((0.045, 0.018, 0.006)),
        origin=Origin(xyz=(0.160, 0.0, 0.004)),
        material=mount_material,
        name="hinge_reinforcement",
    )


def _build_outer_arm(part, *, arm_material, motor_material, accent_material) -> None:
    part.visual(
        Cylinder(radius=0.0105, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=motor_material,
        name="hinge_hub",
    )
    part.visual(
        Box((0.030, 0.020, 0.026)),
        origin=Origin(xyz=(0.020, 0.0, 0.025)),
        material=motor_material,
        name="hinge_post",
    )
    part.visual(
        Box((0.188, 0.024, 0.016)),
        origin=Origin(xyz=(0.129, 0.0, 0.034)),
        material=arm_material,
        name="outer_beam",
    )
    part.visual(
        Box((0.112, 0.016, 0.006)),
        origin=Origin(xyz=(0.096, 0.0, 0.045)),
        material=accent_material,
        name="wire_cover",
    )
    part.visual(
        Box((0.060, 0.032, 0.010)),
        origin=Origin(xyz=(0.188, 0.0, 0.046)),
        material=accent_material,
        name="esc_pod",
    )
    part.visual(
        Box((0.055, 0.055, 0.006)),
        origin=Origin(xyz=(0.240, 0.0, 0.028)),
        material=motor_material,
        name="motor_mount",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(xyz=(0.240, 0.0, 0.042)),
        material=motor_material,
        name="motor_hub",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.240, 0.0, 0.059)),
        material=accent_material,
        name="motor_cap",
    )


def _build_equipment_bay(part, *, shell_material, accent_material, sensor_material) -> None:
    part.visual(
        Box((0.170, 0.130, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=shell_material,
        name="bay_top_flange",
    )
    part.visual(
        Box((0.010, 0.114, 0.052)),
        origin=Origin(xyz=(0.075, 0.0, -0.034)),
        material=shell_material,
        name="right_wall",
    )
    part.visual(
        Box((0.010, 0.114, 0.052)),
        origin=Origin(xyz=(-0.075, 0.0, -0.034)),
        material=shell_material,
        name="left_wall",
    )
    part.visual(
        Box((0.142, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, -0.052, -0.034)),
        material=shell_material,
        name="aft_wall",
    )
    part.visual(
        Box((0.046, 0.010, 0.052)),
        origin=Origin(xyz=(-0.048, 0.052, -0.034)),
        material=shell_material,
        name="fore_wall_left",
    )
    part.visual(
        Box((0.046, 0.010, 0.052)),
        origin=Origin(xyz=(0.048, 0.052, -0.034)),
        material=shell_material,
        name="fore_wall_right",
    )
    part.visual(
        Box((0.118, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.040, -0.063)),
        material=accent_material,
        name="rim_aft",
    )
    part.visual(
        Box((0.118, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.040, -0.063)),
        material=accent_material,
        name="rim_fore",
    )
    part.visual(
        Box((0.010, 0.074, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, -0.063)),
        material=accent_material,
        name="rim_right",
    )
    part.visual(
        Box((0.010, 0.074, 0.006)),
        origin=Origin(xyz=(-0.060, 0.0, -0.063)),
        material=accent_material,
        name="rim_left",
    )
    part.visual(
        Box((0.080, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.044, -0.067)),
        material=accent_material,
        name="hatch_hinge_shelf",
    )
    part.visual(
        Box((0.014, 0.010, 0.018)),
        origin=Origin(xyz=(-0.044, 0.050, -0.060)),
        material=accent_material,
        name="hatch_ear_left",
    )
    part.visual(
        Box((0.014, 0.010, 0.018)),
        origin=Origin(xyz=(0.044, 0.050, -0.060)),
        material=accent_material,
        name="hatch_ear_right",
    )
    part.visual(
        Box((0.100, 0.072, 0.022)),
        origin=Origin(xyz=(0.0, -0.006, -0.019)),
        material=sensor_material,
        name="sensor_stack",
    )


def _build_hatch(part, *, panel_material, accent_material) -> None:
    part.visual(
        Cylinder(radius=0.005, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=accent_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.084, 0.018, 0.012)),
        origin=Origin(xyz=(0.0, -0.006, -0.006)),
        material=accent_material,
        name="hinge_strap",
    )
    part.visual(
        Box((0.126, 0.102, 0.006)),
        origin=Origin(xyz=(0.0, -0.048, -0.011)),
        material=panel_material,
        name="panel",
    )
    part.visual(
        Box((0.086, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.095, -0.010)),
        material=accent_material,
        name="latch_lip",
    )


def _build_skid(part, *, leg_material, mount_material) -> None:
    part.visual(
        Box((0.240, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=mount_material,
        name="mount_crossbar",
    )
    _add_member(
        part,
        (-0.075, 0.0, -0.010),
        (-0.120, 0.0, -0.165),
        radius=0.008,
        material=mount_material,
        name="aft_strut",
    )
    _add_member(
        part,
        (0.075, 0.0, -0.010),
        (0.120, 0.0, -0.165),
        radius=0.008,
        material=mount_material,
        name="fore_strut",
    )
    _add_member(
        part,
        (-0.098, 0.0, -0.090),
        (0.098, 0.0, -0.090),
        radius=0.0055,
        material=mount_material,
        name="mid_brace",
    )
    part.visual(
        Cylinder(radius=0.011, length=0.440),
        origin=Origin(xyz=(0.0, 0.0, -0.175), rpy=(0.0, pi / 2.0, 0.0)),
        material=leg_material,
        name="rail",
    )
    _add_member(
        part,
        (-0.205, 0.0, -0.175),
        (-0.222, 0.0, -0.148),
        radius=0.0105,
        material=leg_material,
        name="aft_tip",
    )
    _add_member(
        part,
        (0.205, 0.0, -0.175),
        (0.222, 0.0, -0.148),
        radius=0.0105,
        material=leg_material,
        name="fore_tip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_hexacopter")

    carbon_black = model.material("carbon_black", rgba=(0.10, 0.11, 0.12, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.26, 0.28, 0.31, 1.0))
    aluminum = model.material("aluminum", rgba=(0.68, 0.70, 0.73, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.84, 0.36, 0.12, 1.0))
    sensor_black = model.material("sensor_black", rgba=(0.06, 0.07, 0.08, 1.0))

    hub_frame = model.part("hub_frame")
    hub_frame.visual(
        Cylinder(radius=0.158, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=carbon_black,
        name="hub_lower_plate",
    )
    hub_frame.visual(
        Cylinder(radius=0.110, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=carbon_black,
        name="hub_top_plate",
    )
    hub_frame.visual(Cylinder(radius=0.056, length=0.018), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=gunmetal, name="center_stack")
    for index in range(6):
        angle = index * pi / 3.0
        hub_frame.visual(
            Box((0.092, 0.030, 0.006)),
            origin=Origin(
                xyz=(0.118 * cos(angle), 0.118 * sin(angle), 0.003),
                rpy=(0.0, 0.0, angle),
            ),
            material=gunmetal,
            name=f"arm_socket_{index}",
        )
    hub_frame.visual(
        Box((0.250, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, 0.138, -0.005)),
        material=gunmetal,
        name="left_skid_pad",
    )
    hub_frame.visual(
        Box((0.250, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, -0.138, -0.005)),
        material=gunmetal,
        name="right_skid_pad",
    )
    for index in range(6):
        angle = index * pi / 3.0
        hub_frame.visual(
            Cylinder(radius=0.009, length=0.020),
            origin=Origin(xyz=(0.082 * cos(angle), 0.082 * sin(angle), 0.013)),
            material=aluminum,
            name=f"standoff_{index}",
        )
    hub_frame.inertial = Inertial.from_geometry(
        Box((0.42, 0.42, 0.08)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    for index in range(6):
        inner = model.part(f"arm_inner_{index}")
        _build_inner_arm(inner, arm_material=carbon_black, mount_material=gunmetal)
        inner.inertial = Inertial.from_geometry(
            Box((0.230, 0.060, 0.035)),
            mass=0.35,
            origin=Origin(xyz=(0.115, 0.0, 0.012)),
        )
        angle = index * pi / 3.0
        model.articulation(
            f"hub_to_arm_inner_{index}",
            ArticulationType.FIXED,
            parent=hub_frame,
            child=inner,
            origin=Origin(xyz=(0.094 * cos(angle), 0.094 * sin(angle), 0.0), rpy=(0.0, 0.0, angle)),
        )

        outer = model.part(f"arm_outer_{index}")
        _build_outer_arm(outer, arm_material=carbon_black, motor_material=gunmetal, accent_material=safety_orange)
        outer.inertial = Inertial.from_geometry(
            Box((0.290, 0.070, 0.055)),
            mass=0.50,
            origin=Origin(xyz=(0.145, 0.0, 0.018)),
        )
        if index % 2 == 0:
            lower, upper = (0.0, 0.85)
        else:
            lower, upper = (-0.85, 0.0)
        model.articulation(
            f"arm_{index}_fold",
            ArticulationType.REVOLUTE,
            parent=inner,
            child=outer,
            origin=Origin(xyz=(0.185, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=lower, upper=upper),
        )

    equipment_bay = model.part("equipment_bay")
    _build_equipment_bay(
        equipment_bay,
        shell_material=gunmetal,
        accent_material=aluminum,
        sensor_material=sensor_black,
    )
    equipment_bay.inertial = Inertial.from_geometry(
        Box((0.175, 0.135, 0.075)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
    )
    model.articulation(
        "hub_to_equipment_bay",
        ArticulationType.FIXED,
        parent=hub_frame,
        child=equipment_bay,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    hatch = model.part("bay_hatch")
    _build_hatch(hatch, panel_material=carbon_black, accent_material=aluminum)
    hatch.inertial = Inertial.from_geometry(
        Box((0.130, 0.108, 0.020)),
        mass=0.24,
        origin=Origin(xyz=(0.0, -0.050, -0.012)),
    )
    model.articulation(
        "bay_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=equipment_bay,
        child=hatch,
        origin=Origin(xyz=(0.0, 0.050, -0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    left_skid = model.part("left_skid")
    _build_skid(left_skid, leg_material=aluminum, mount_material=gunmetal)
    left_skid.inertial = Inertial.from_geometry(
        Box((0.470, 0.030, 0.205)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
    )
    model.articulation(
        "hub_to_left_skid",
        ArticulationType.FIXED,
        parent=hub_frame,
        child=left_skid,
        origin=Origin(xyz=(0.0, 0.138, -0.010)),
    )

    right_skid = model.part("right_skid")
    _build_skid(right_skid, leg_material=aluminum, mount_material=gunmetal)
    right_skid.inertial = Inertial.from_geometry(
        Box((0.470, 0.030, 0.205)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
    )
    model.articulation(
        "hub_to_right_skid",
        ArticulationType.FIXED,
        parent=hub_frame,
        child=right_skid,
        origin=Origin(xyz=(0.0, -0.138, -0.010)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hub_frame = object_model.get_part("hub_frame")
    hub_lower_plate = hub_frame.get_visual("hub_lower_plate")
    left_skid_pad = hub_frame.get_visual("left_skid_pad")
    right_skid_pad = hub_frame.get_visual("right_skid_pad")

    equipment_bay = object_model.get_part("equipment_bay")
    bay_top_flange = equipment_bay.get_visual("bay_top_flange")
    bay_rim_aft = equipment_bay.get_visual("rim_aft")
    bay_hinge_shelf = equipment_bay.get_visual("hatch_hinge_shelf")

    hatch = object_model.get_part("bay_hatch")
    hatch_panel = hatch.get_visual("panel")
    hatch_hinge_barrel = hatch.get_visual("hinge_barrel")
    hatch_hinge = object_model.get_articulation("bay_hatch_hinge")

    left_skid = object_model.get_part("left_skid")
    left_skid_crossbar = left_skid.get_visual("mount_crossbar")
    left_skid_rail = left_skid.get_visual("rail")
    right_skid = object_model.get_part("right_skid")
    right_skid_crossbar = right_skid.get_visual("mount_crossbar")
    right_skid_rail = right_skid.get_visual("rail")

    arm_inners = [object_model.get_part(f"arm_inner_{index}") for index in range(6)]
    arm_outers = [object_model.get_part(f"arm_outer_{index}") for index in range(6)]
    arm_joints = [object_model.get_articulation(f"arm_{index}_fold") for index in range(6)]
    arm_root_clamps = [arm.get_visual("root_clamp") for arm in arm_inners]
    arm_hinge_saddles = [arm.get_visual("hinge_saddle") for arm in arm_inners]
    arm_hinge_hubs = [arm.get_visual("hinge_hub") for arm in arm_outers]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    for inner, outer in zip(arm_inners, arm_outers):
        ctx.allow_overlap(inner, outer, reason="fold hinge knuckle nests around the seated pivot barrel")
    ctx.allow_overlap(equipment_bay, hatch, reason="access hatch hinge barrel nests between the bay ears")
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        hub_frame,
        equipment_bay,
        axis="z",
        positive_elem=hub_lower_plate,
        negative_elem=bay_top_flange,
        max_gap=0.001,
        max_penetration=0.001,
        name="equipment_bay_hangs_from_hub",
    )
    ctx.expect_overlap(
        equipment_bay,
        hub_frame,
        axes="xy",
        elem_a=bay_top_flange,
        elem_b=hub_lower_plate,
        min_overlap=0.010,
        name="equipment_bay_centered_under_hub",
    )

    ctx.expect_gap(
        hub_frame,
        left_skid,
        axis="z",
        positive_elem=left_skid_pad,
        negative_elem=left_skid_crossbar,
        max_gap=0.001,
        max_penetration=0.001,
        name="left_skid_bracket_seated",
    )
    ctx.expect_overlap(
        left_skid,
        hub_frame,
        axes="xy",
        elem_a=left_skid_crossbar,
        elem_b=left_skid_pad,
        min_overlap=0.020,
        name="left_skid_crossbar_under_pad",
    )
    ctx.expect_gap(
        hub_frame,
        left_skid,
        axis="z",
        positive_elem=hub_lower_plate,
        negative_elem=left_skid_rail,
        min_gap=0.150,
        name="left_skid_rail_below_airframe",
    )

    ctx.expect_gap(
        hub_frame,
        right_skid,
        axis="z",
        positive_elem=right_skid_pad,
        negative_elem=right_skid_crossbar,
        max_gap=0.001,
        max_penetration=0.001,
        name="right_skid_bracket_seated",
    )
    ctx.expect_overlap(
        right_skid,
        hub_frame,
        axes="xy",
        elem_a=right_skid_crossbar,
        elem_b=right_skid_pad,
        min_overlap=0.020,
        name="right_skid_crossbar_under_pad",
    )
    ctx.expect_gap(
        hub_frame,
        right_skid,
        axis="z",
        positive_elem=hub_lower_plate,
        negative_elem=right_skid_rail,
        min_gap=0.150,
        name="right_skid_rail_below_airframe",
    )

    ctx.expect_gap(
        equipment_bay,
        hatch,
        axis="z",
        positive_elem=bay_rim_aft,
        negative_elem=hatch_panel,
        max_gap=0.004,
        max_penetration=0.0,
        name="hatch_closes_flush_to_bay",
    )
    ctx.expect_within(
        hatch,
        equipment_bay,
        axes="xy",
        inner_elem=hatch_panel,
        outer_elem=bay_top_flange,
        name="hatch_covers_access_opening",
    )
    ctx.expect_gap(
        hatch,
        equipment_bay,
        axis="z",
        positive_elem=hatch_hinge_barrel,
        negative_elem=bay_hinge_shelf,
        max_gap=0.001,
        max_penetration=0.001,
        name="hatch_hinge_seated_on_shelf",
    )

    for index, (inner, outer, joint, root_clamp, hinge_saddle, hinge_hub) in enumerate(
        zip(arm_inners, arm_outers, arm_joints, arm_root_clamps, arm_hinge_saddles, arm_hinge_hubs)
    ):
        ctx.expect_overlap(
            inner,
            hub_frame,
            axes="xy",
            elem_a=root_clamp,
            elem_b=hub_lower_plate,
            min_overlap=0.010,
            name=f"arm_{index}_root_clamp_attached",
        )
        ctx.expect_gap(
            outer,
            inner,
            axis="z",
            positive_elem=hinge_hub,
            negative_elem=hinge_saddle,
            max_gap=0.001,
            max_penetration=0.001,
            name=f"arm_{index}_hinge_hub_seated",
        )

    fold_pose = {
        arm_joints[index]: (0.80 if index % 2 == 0 else -0.80)
        for index in range(6)
    }
    with ctx.pose(fold_pose):
        for index, (inner, outer, hinge_saddle, hinge_hub) in enumerate(
            zip(arm_inners, arm_outers, arm_hinge_saddles, arm_hinge_hubs)
        ):
            ctx.expect_gap(
                outer,
                inner,
                axis="z",
                positive_elem=hinge_hub,
                negative_elem=hinge_saddle,
                max_gap=0.001,
                max_penetration=0.001,
                name=f"arm_{index}_hinge_stays_seated_folded",
            )

    with ctx.pose({hatch_hinge: 1.20}):
        ctx.expect_gap(
            hatch,
            equipment_bay,
            axis="y",
            positive_elem=hatch_panel,
            negative_elem=bay_rim_aft,
            min_gap=0.030,
            name="hatch_swings_clear_for_access",
        )
        ctx.expect_gap(
            hatch,
            equipment_bay,
            axis="z",
            positive_elem=hatch_hinge_barrel,
            negative_elem=bay_hinge_shelf,
            max_gap=0.001,
            max_penetration=0.001,
            name="hatch_hinge_stays_supported_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
