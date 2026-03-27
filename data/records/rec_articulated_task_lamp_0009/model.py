from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_shade_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.018, -0.034),
            (0.021, -0.029),
            (0.028, -0.015),
            (0.038, 0.012),
            (0.047, 0.044),
        ],
        [
            (0.010, -0.034),
            (0.012, -0.029),
            (0.019, -0.015),
            (0.031, 0.011),
            (0.041, 0.044),
        ],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_task_lamp", assets=ASSETS)

    warm_white = model.material("warm_white", rgba=(0.90, 0.89, 0.85, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    brass = model.material("brass", rgba=(0.69, 0.58, 0.34, 1.0))
    warm_glass = model.material("warm_glass", rgba=(0.95, 0.92, 0.76, 0.45))

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.010, 0.090, 0.180)),
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        material=warm_white,
        name="plate",
    )
    wall_plate.visual(
        Box((0.004, 0.054, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="pivot_seat",
    )
    wall_plate.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="upper_screw_cap",
    )
    wall_plate.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.056), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lower_screw_cap",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.010, 0.090, 0.180)),
        mass=1.2,
        origin=Origin(xyz=(-0.005, 0.0, 0.0)),
    )

    rear_arm = model.part("rear_arm")
    rear_arm.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(rpy=(0.0, 0.0, 0.0)),
        material=steel,
        name="pivot_collar",
    )
    rear_arm.visual(
        Cylinder(radius=0.018, length=0.036),
        origin=Origin(),
        material=steel,
        name="pivot_barrel",
    )
    rear_arm.visual(
        Box((0.050, 0.022, 0.020)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=steel,
        name="pivot_link",
    )
    rear_arm.visual(
        Cylinder(radius=0.0105, length=0.198),
        origin=Origin(xyz=(0.164, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_tube",
    )
    rear_arm.visual(
        Box((0.036, 0.029, 0.014)),
        origin=Origin(xyz=(0.245, 0.0, 0.0)),
        material=steel,
        name="elbow_bridge",
    )
    rear_arm.visual(
        Box((0.024, 0.008, 0.044)),
        origin=Origin(xyz=(0.275, 0.0165, 0.0)),
        material=steel,
        name="elbow_left_cheek",
    )
    rear_arm.visual(
        Box((0.024, 0.008, 0.044)),
        origin=Origin(xyz=(0.275, -0.0165, 0.0)),
        material=steel,
        name="elbow_right_cheek",
    )
    rear_arm.inertial = Inertial.from_geometry(
        Box((0.300, 0.055, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.0105, length=0.023),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.024, 0.018, 0.018)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=steel,
        name="root_block",
    )
    forearm.visual(
        Cylinder(radius=0.0095, length=0.190),
        origin=Origin(xyz=(0.127, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="forearm_tube",
    )
    forearm.visual(
        Box((0.234, 0.014, 0.014)),
        origin=Origin(xyz=(0.117, 0.0, 0.0)),
        material=charcoal,
        name="forearm_spine",
    )
    forearm.visual(
        Box((0.036, 0.029, 0.014)),
        origin=Origin(xyz=(0.212, 0.0, 0.0)),
        material=steel,
        name="yoke_bridge",
    )
    forearm.visual(
        Box((0.028, 0.008, 0.044)),
        origin=Origin(xyz=(0.244, 0.0165, 0.0)),
        material=steel,
        name="yoke_left_cheek",
    )
    forearm.visual(
        Box((0.028, 0.008, 0.044)),
        origin=Origin(xyz=(0.244, -0.0165, 0.0)),
        material=steel,
        name="yoke_right_cheek",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.290, 0.050, 0.055)),
        mass=0.75,
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.0095, length=0.025),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="shade_hub",
    )
    shade.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shade_neck",
    )
    shade.visual(
        Box((0.080, 0.012, 0.012)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=steel,
        name="shade_stem",
    )
    shade.visual(
        Box((0.024, 0.024, 0.024)),
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        material=steel,
        name="shade_adapter",
    )
    shade.visual(
        _save_mesh("task_lamp_shade_shell.obj", _build_shade_shell_mesh()),
        origin=Origin(xyz=(0.069, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.050, length=0.010),
        origin=Origin(xyz=(0.116, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="shade_rim",
    )
    shade.visual(
        Cylinder(radius=0.013, length=0.028),
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material=warm_glass,
        name="bulb",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.140, 0.110, 0.110)),
        mass=0.45,
        origin=Origin(xyz=(0.085, 0.0, 0.0)),
    )

    model.articulation(
        "wall_swing",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=rear_arm,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-1.55, upper=1.55),
    )
    model.articulation(
        "elbow_fold",
        ArticulationType.REVOLUTE,
        parent=rear_arm,
        child=forearm,
        origin=Origin(xyz=(0.275, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-1.10, upper=0.30),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=shade,
        origin=Origin(xyz=(0.244, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.4, lower=-0.90, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    rear_arm = object_model.get_part("rear_arm")
    forearm = object_model.get_part("forearm")
    shade = object_model.get_part("shade")

    wall_swing = object_model.get_articulation("wall_swing")
    elbow_fold = object_model.get_articulation("elbow_fold")
    shade_tilt = object_model.get_articulation("shade_tilt")

    plate = wall_plate.get_visual("plate")
    pivot_seat = wall_plate.get_visual("pivot_seat")
    pivot_collar = rear_arm.get_visual("pivot_collar")
    pivot_barrel = rear_arm.get_visual("pivot_barrel")
    elbow_left = rear_arm.get_visual("elbow_left_cheek")
    elbow_right = rear_arm.get_visual("elbow_right_cheek")
    elbow_hub = forearm.get_visual("elbow_hub")
    yoke_left = forearm.get_visual("yoke_left_cheek")
    yoke_right = forearm.get_visual("yoke_right_cheek")
    shade_hub = shade.get_visual("shade_hub")
    shade_shell = shade.get_visual("shade_shell")
    shade_rim = shade.get_visual("shade_rim")
    socket = shade.get_visual("socket")
    bulb = shade.get_visual("bulb")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.020)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(
        rear_arm,
        wall_plate,
        axes="yz",
        elem_a=pivot_barrel,
        elem_b=pivot_seat,
        min_overlap=0.030,
        name="pivot_collar_reads_over_wall_bearing_seat",
    )
    ctx.expect_contact(
        rear_arm,
        wall_plate,
        elem_a=pivot_barrel,
        elem_b=pivot_seat,
        name="pivot_collar_contacts_wall_bearing_seat",
    )
    ctx.expect_overlap(
        rear_arm,
        wall_plate,
        axes="yz",
        elem_a=pivot_barrel,
        elem_b=plate,
        min_overlap=0.028,
        name="pivot_collar_stays_centered_on_wall_plate",
    )

    ctx.expect_overlap(
        forearm,
        rear_arm,
        axes="xz",
        elem_a=elbow_hub,
        elem_b=elbow_left,
        min_overlap=0.008,
        name="elbow_hub_reads_inside_mid_span_fork",
    )
    ctx.expect_gap(
        rear_arm,
        forearm,
        axis="y",
        positive_elem=elbow_left,
        negative_elem=elbow_hub,
        max_gap=0.0015,
        max_penetration=0.0,
        name="elbow_left_cheek_sits_close_to_hub",
    )
    ctx.expect_gap(
        forearm,
        rear_arm,
        axis="y",
        positive_elem=elbow_hub,
        negative_elem=elbow_right,
        max_gap=0.0015,
        max_penetration=0.0,
        name="elbow_right_cheek_sits_close_to_hub",
    )

    ctx.expect_overlap(
        shade,
        forearm,
        axes="xz",
        elem_a=shade_hub,
        elem_b=yoke_left,
        min_overlap=0.008,
        name="shade_hub_reads_inside_tip_yoke",
    )
    ctx.expect_gap(
        forearm,
        shade,
        axis="y",
        positive_elem=yoke_left,
        negative_elem=shade_hub,
        max_gap=0.0015,
        max_penetration=0.0,
        name="left_yoke_cheek_sits_close_to_shade_hub",
    )
    ctx.expect_gap(
        shade,
        forearm,
        axis="y",
        positive_elem=shade_hub,
        negative_elem=yoke_right,
        max_gap=0.0015,
        max_penetration=0.0,
        name="right_yoke_cheek_sits_close_to_shade_hub",
    )
    ctx.expect_gap(
        shade,
        wall_plate,
        axis="x",
        min_gap=0.22,
        name="shade_projects_cantilevered_out_from_wall",
    )
    ctx.expect_gap(
        shade,
        forearm,
        axis="x",
        positive_elem=shade_shell,
        negative_elem=yoke_left,
        min_gap=0.018,
        name="shade_body_starts_forward_of_tip_yoke",
    )
    ctx.expect_within(
        shade,
        shade,
        axes="yz",
        inner_elem=socket,
        outer_elem=shade_shell,
        name="socket_reads_within_shade_shell",
    )
    ctx.expect_within(
        shade,
        shade,
        axes="yz",
        inner_elem=bulb,
        outer_elem=shade_rim,
        name="bulb_reads_within_shade_rim_aperture",
    )
    ctx.expect_gap(
        shade,
        shade,
        axis="x",
        positive_elem=shade_rim,
        negative_elem=socket,
        min_gap=0.025,
        name="socket_is_recessed_behind_open_shade_rim",
    )
    ctx.expect_gap(
        shade,
        shade,
        axis="x",
        positive_elem=shade_rim,
        negative_elem=bulb,
        min_gap=0.006,
        name="bulb_sits_behind_front_rim",
    )

    with ctx.pose({wall_swing: 1.20}):
        ctx.expect_contact(
            rear_arm,
            wall_plate,
            elem_a=pivot_barrel,
            elem_b=pivot_seat,
            name="pivot_collar_stays_in_contact_with_wall_bearing_seat_when_swung",
        )

    with ctx.pose({elbow_fold: -0.90}):
        ctx.expect_gap(
            rear_arm,
            forearm,
            axis="y",
            positive_elem=elbow_left,
            negative_elem=elbow_hub,
            max_gap=0.0015,
            max_penetration=0.0,
            name="elbow_left_cheek_stays_close_when_folded",
        )
        ctx.expect_gap(
            forearm,
            rear_arm,
            axis="y",
            positive_elem=elbow_hub,
            negative_elem=elbow_right,
            max_gap=0.0015,
            max_penetration=0.0,
            name="elbow_right_cheek_stays_close_when_folded",
        )
        ctx.expect_gap(
            shade,
            wall_plate,
            axis="x",
            min_gap=0.08,
            name="folded_lamp_head_stays_in_front_of_wall_plate",
        )

    with ctx.pose({shade_tilt: -0.70}):
        ctx.expect_gap(
            forearm,
            shade,
            axis="y",
            positive_elem=yoke_left,
            negative_elem=shade_hub,
            max_gap=0.0015,
            max_penetration=0.0,
            name="left_yoke_cheek_stays_close_when_tilted",
        )
        ctx.expect_gap(
            shade,
            forearm,
            axis="y",
            positive_elem=shade_hub,
            negative_elem=yoke_right,
            max_gap=0.0015,
            max_penetration=0.0,
            name="right_yoke_cheek_stays_close_when_tilted",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
