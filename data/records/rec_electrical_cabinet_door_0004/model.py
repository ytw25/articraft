from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

SAFE_ASSET_ROOT = Path("/")
ASSETS = AssetContext(SAFE_ASSET_ROOT)


def _yz_vector(length: float, angle: float) -> tuple[float, float, float]:
    return (0.0, -length * math.sin(angle), length * math.cos(angle))


def _yz_perp(offset: float, angle: float) -> tuple[float, float, float]:
    return (0.0, -offset * math.cos(angle), -offset * math.sin(angle))


def _scaled(vec: tuple[float, float, float], scale: float) -> tuple[float, float, float]:
    return (vec[0] * scale, vec[1] * scale, vec[2] * scale)


def _added(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dead_front_equipment_bay", assets=ASSETS)

    cabinet = model.material("cabinet_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.81, 0.82, 0.84, 1.0))
    hardware = model.material("hardware", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.22, 0.24, 0.27, 1.0))

    outer_w = 0.64
    outer_h = 0.50
    opening_w = 0.56
    opening_h = 0.392
    door_w = 0.548
    door_h = 0.388
    bezel_t = 0.018
    cavity_d = 0.28
    wall_t = 0.012
    door_t = 0.010
    side_strip_w = (outer_w - opening_w) / 2.0
    top_strip_h = (outer_h - opening_h) / 2.0

    hinge_r = 0.0065
    hinge_axis_y = hinge_r
    hinge_drop = hinge_r
    hinge_axis_z = opening_h / 2.0 + hinge_drop

    fastener_x = 0.18
    fastener_z = -door_h + 0.044

    prop_x = 0.214
    prop_pivot_y = -0.050
    prop_pivot_z = -0.050
    lower_len = 0.18
    upper_len = 0.21
    lower_closed_angle = 0.85
    upper_closed_angle = 2.85
    lower_open_pose = -1.55
    upper_open_pose = -2.43

    lower_vec = _yz_vector(lower_len, lower_closed_angle)
    lower_unit = _scaled(lower_vec, 1.0 / lower_len)
    lower_mid = _scaled(lower_vec, 0.5)
    elbow_clevis_center = lower_vec

    upper_vec = _yz_vector(upper_len, upper_closed_angle)
    upper_mid = _scaled(upper_vec, 0.5)
    prop_tip_center = upper_vec

    open_lower_angle = lower_closed_angle + lower_open_pose
    open_upper_angle = upper_closed_angle + lower_open_pose + upper_open_pose
    open_elbow = _yz_vector(lower_len, open_lower_angle)
    open_tip_offset = _yz_vector(upper_len, open_upper_angle)
    open_tip_center = (
        prop_x,
        prop_pivot_y + open_elbow[1] + open_tip_offset[1],
        prop_pivot_z + open_elbow[2] + open_tip_offset[2],
    )

    strike_thickness = 0.012
    strike_center_local = (
        prop_x,
        open_tip_center[2] + strike_thickness / 2.0 - hinge_axis_z,
        -(open_tip_center[1] - hinge_axis_y),
    )

    bay = model.part("bay")
    bay.visual(
        Box((outer_w, bezel_t, top_strip_h)),
        origin=Origin(xyz=(0.0, -bezel_t / 2.0, opening_h / 2.0 + top_strip_h / 2.0)),
        material=cabinet,
        name="top_bezel",
    )
    bay.visual(
        Box((outer_w, bezel_t, top_strip_h)),
        origin=Origin(xyz=(0.0, -bezel_t / 2.0, -opening_h / 2.0 - top_strip_h / 2.0)),
        material=cabinet,
        name="bottom_bezel",
    )
    bay.visual(
        Box((side_strip_w, bezel_t, opening_h)),
        origin=Origin(xyz=(-opening_w / 2.0 - side_strip_w / 2.0, -bezel_t / 2.0, 0.0)),
        material=cabinet,
        name="left_bezel",
    )
    bay.visual(
        Box((side_strip_w, bezel_t, opening_h)),
        origin=Origin(xyz=(opening_w / 2.0 + side_strip_w / 2.0, -bezel_t / 2.0, 0.0)),
        material=cabinet,
        name="right_bezel",
    )
    bay.visual(
        Box((wall_t, cavity_d, opening_h + 0.10)),
        origin=Origin(xyz=(-opening_w / 2.0 - wall_t / 2.0, -cavity_d / 2.0, 0.0)),
        material=cabinet,
        name="left_wall",
    )
    bay.visual(
        Box((wall_t, cavity_d, opening_h + 0.10)),
        origin=Origin(xyz=(opening_w / 2.0 + wall_t / 2.0, -cavity_d / 2.0, 0.0)),
        material=cabinet,
        name="right_wall",
    )
    bay.visual(
        Box((opening_w, cavity_d, wall_t)),
        origin=Origin(xyz=(0.0, -cavity_d / 2.0, opening_h / 2.0 + wall_t / 2.0)),
        material=cabinet,
        name="cavity_roof",
    )
    bay.visual(
        Box((opening_w, cavity_d, wall_t)),
        origin=Origin(xyz=(0.0, -cavity_d / 2.0, -opening_h / 2.0 - wall_t / 2.0)),
        material=cabinet,
        name="cavity_floor",
    )
    bay.visual(
        Box((opening_w + 2.0 * wall_t, wall_t, opening_h + 0.10)),
        origin=Origin(xyz=(0.0, -cavity_d + wall_t / 2.0, 0.0)),
        material=dark_hardware,
        name="back_wall",
    )
    bay.visual(
        Cylinder(radius=hinge_r, length=door_w),
        origin=Origin(
            xyz=(0.0, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware,
        name="hinge_barrel",
    )
    bracket_len = opening_w / 2.0 + wall_t / 2.0 - prop_x + 0.006
    bay.visual(
        Box((bracket_len, 0.016, 0.046)),
        origin=Origin(
            xyz=(prop_x + bracket_len / 2.0 - 0.003, prop_pivot_y, prop_pivot_z),
        ),
        material=hardware,
        name="prop_bracket",
    )
    bay.visual(
        Box((0.026, 0.008, 0.110)),
        origin=Origin(xyz=(-fastener_x, -0.0205, -0.1515)),
        material=hardware,
        name="receiver_left",
    )
    bay.visual(
        Box((0.026, 0.008, 0.110)),
        origin=Origin(xyz=(fastener_x, -0.0205, -0.1515)),
        material=hardware,
        name="receiver_right",
    )
    bay.inertial = Inertial.from_geometry(
        Box((outer_w, cavity_d, outer_h)),
        mass=11.0,
        origin=Origin(xyz=(0.0, -cavity_d / 2.0, 0.0)),
    )

    panel = model.part("panel")
    panel.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(
            xyz=(0.0, -hinge_axis_y - door_t / 2.0 + 0.0005, -(door_h / 2.0 + hinge_drop)),
        ),
        material=panel_gray,
        name="outer_face",
    )
    panel.visual(
        Box((door_w - 0.070, 0.016, 0.156)),
        origin=Origin(xyz=(0.0, -0.022, -0.113)),
        material=panel_gray,
        name="inner_pan",
    )
    panel.visual(
        Box((0.024, 0.018, 0.298)),
        origin=Origin(xyz=(-door_w / 2.0 + 0.022, -0.020, -0.224)),
        material=panel_gray,
        name="left_return",
    )
    panel.visual(
        Box((0.024, 0.018, 0.298)),
        origin=Origin(xyz=(door_w / 2.0 - 0.022, -0.020, -0.224)),
        material=panel_gray,
        name="right_return",
    )
    panel.visual(
        Box((0.024, strike_thickness, 0.018)),
        origin=Origin(xyz=strike_center_local),
        material=hardware,
        name="prop_strike",
    )
    for side_name, x_pos in (("left", -fastener_x), ("right", fastener_x)):
        panel.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(
                xyz=(x_pos, -0.0035, fastener_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"fastener_{side_name}_head",
        )
        panel.visual(
            Box((0.020, 0.0015, 0.003)),
            origin=Origin(xyz=(x_pos, -0.0007, fastener_z)),
            material=dark_hardware,
            name=f"fastener_{side_name}_slot",
        )
        panel.visual(
            Cylinder(radius=0.005, length=0.018),
            origin=Origin(
                xyz=(x_pos, -0.014, fastener_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=hardware,
            name=f"fastener_{side_name}_stem",
        )
    panel.inertial = Inertial.from_geometry(
        Box((door_w, 0.036, door_h)),
        mass=3.6,
        origin=Origin(xyz=(0.0, -0.020, -(door_h / 2.0 + hinge_drop))),
    )

    prop_lower = model.part("prop_lower")
    prop_lower.visual(
        Box((0.004, 0.010, lower_len)),
        origin=Origin(xyz=(0.002, lower_mid[1], lower_mid[2]), rpy=(lower_closed_angle, 0.0, 0.0)),
        material=hardware,
        name="lower_arm",
    )
    prop_lower.inertial = Inertial.from_geometry(
        Box((0.030, 0.020, lower_len)),
        mass=0.30,
        origin=Origin(xyz=lower_mid, rpy=(lower_closed_angle, 0.0, 0.0)),
    )

    prop_upper = model.part("prop_upper")
    prop_upper.visual(
        Box((0.004, 0.010, upper_len)),
        origin=Origin(xyz=(-0.002, upper_mid[1], upper_mid[2]), rpy=(upper_closed_angle, 0.0, 0.0)),
        material=hardware,
        name="upper_arm",
    )
    prop_upper.inertial = Inertial.from_geometry(
        Box((0.026, 0.020, upper_len)),
        mass=0.22,
        origin=Origin(xyz=upper_mid, rpy=(upper_closed_angle, 0.0, 0.0)),
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=bay,
        child=panel,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "prop_base_pivot",
        ArticulationType.REVOLUTE,
        parent=bay,
        child=prop_lower,
        origin=Origin(xyz=(prop_x, prop_pivot_y, prop_pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-1.55,
            upper=0.0,
        ),
    )
    model.articulation(
        "prop_elbow",
        ArticulationType.REVOLUTE,
        parent=prop_lower,
        child=prop_upper,
        origin=Origin(xyz=lower_vec),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=-2.43,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SAFE_ASSET_ROOT)
    bay = object_model.get_part("bay")
    panel = object_model.get_part("panel")
    prop_lower = object_model.get_part("prop_lower")
    prop_upper = object_model.get_part("prop_upper")

    panel_hinge = object_model.get_articulation("panel_hinge")
    prop_base_pivot = object_model.get_articulation("prop_base_pivot")
    prop_elbow = object_model.get_articulation("prop_elbow")

    outer_face = panel.get_visual("outer_face")
    panel_strike = panel.get_visual("prop_strike")
    fastener_left_head = panel.get_visual("fastener_left_head")
    fastener_left_stem = panel.get_visual("fastener_left_stem")
    fastener_right_stem = panel.get_visual("fastener_right_stem")

    hinge_barrel = bay.get_visual("hinge_barrel")
    bay_receiver_left = bay.get_visual("receiver_left")
    bay_receiver_right = bay.get_visual("receiver_right")
    back_wall = bay.get_visual("back_wall")
    prop_bracket = bay.get_visual("prop_bracket")
    top_bezel = bay.get_visual("top_bezel")

    lower_arm = prop_lower.get_visual("lower_arm")
    upper_arm = prop_upper.get_visual("upper_arm")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
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

    ctx.expect_overlap(panel, bay, axes="xz", min_overlap=0.18, elem_a=outer_face)
    ctx.expect_gap(
        panel,
        bay,
        axis="y",
        max_gap=0.001,
        max_penetration=0.010,
        positive_elem=outer_face,
        negative_elem=top_bezel,
        name="closed_panel_sits_within_bezel_depth",
    )
    ctx.expect_contact(prop_lower, bay, elem_a=lower_arm, elem_b=prop_bracket)
    ctx.expect_contact(prop_upper, prop_lower, elem_a=upper_arm, elem_b=lower_arm)
    ctx.expect_contact(panel, bay, elem_a=fastener_left_stem, elem_b=bay_receiver_left)
    ctx.expect_contact(panel, bay, elem_a=fastener_right_stem, elem_b=bay_receiver_right)
    ctx.expect_gap(
        panel,
        prop_upper,
        axis="y",
        min_gap=0.004,
        positive_elem=outer_face,
        negative_elem=upper_arm,
        name="closed_prop_stays_behind_deadfront",
    )

    with ctx.pose(
        {
            panel_hinge: math.pi / 2.0,
            prop_base_pivot: -1.55,
            prop_elbow: -2.43,
        }
    ):
        ctx.expect_contact(prop_upper, panel, elem_a=upper_arm, elem_b=panel_strike)
        ctx.expect_gap(
            panel,
            bay,
            axis="y",
            min_gap=0.55,
            positive_elem=fastener_left_head,
            negative_elem=back_wall,
            name="open_panel_swings_clear_of_bay_depth",
        )
        ctx.expect_overlap(panel, bay, axes="x", min_overlap=0.40, elem_a=outer_face)
        ctx.expect_contact(prop_lower, bay, elem_a=lower_arm, elem_b=prop_bracket)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
