from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib
import posixpath

if not os.path.isabs(__file__):
    __file__ = posixpath.join("/", __file__.lstrip("/"))

_ORIG_PATH_ABSOLUTE = pathlib.Path.absolute


def _safe_path_absolute(self):
    if self.is_absolute():
        return self
    try:
        return _ORIG_PATH_ABSOLUTE(self)
    except FileNotFoundError:
        return self.__class__(posixpath.join("/", str(self).lstrip("/")))


pathlib.Path.absolute = _safe_path_absolute

THIS_FILE = __file__
HERE = os.path.dirname(THIS_FILE) or "/"

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
PIVOT_Z = 1.72
PIVOT_Y = -0.02


def _add_wing_nut(part, *, x: float, y: float, z: float, prefix: str, material) -> None:
    part.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{prefix}_hub",
    )
    part.visual(
        Box((0.042, 0.006, 0.014)),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=f"{prefix}_wing_x",
    )
    part.visual(
        Box((0.016, 0.006, 0.034)),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=f"{prefix}_wing_z",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_a_frame_easel")

    wood = model.material("oak", rgba=(0.66, 0.50, 0.32, 1.0))
    wood_dark = model.material("walnut", rgba=(0.43, 0.29, 0.18, 1.0))
    metal = model.material("steel", rgba=(0.62, 0.63, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.14, 1.0))

    frame = model.part("front_frame")

    front_leg_len = 1.71
    front_leg_roll = 0.045
    front_leg_pitch = 0.198
    left_leg_center = (-0.168, 0.018, 0.885)
    right_leg_center = (0.168, 0.018, 0.885)

    frame.visual(
        Box((0.038, 0.026, front_leg_len)),
        origin=Origin(xyz=left_leg_center, rpy=(front_leg_roll, front_leg_pitch, 0.0)),
        material=wood,
        name="left_front_leg",
    )
    frame.visual(
        Box((0.038, 0.026, front_leg_len)),
        origin=Origin(xyz=right_leg_center, rpy=(front_leg_roll, -front_leg_pitch, 0.0)),
        material=wood,
        name="right_front_leg",
    )
    frame.visual(
        Box((0.66, 0.042, 0.032)),
        origin=Origin(xyz=(0.0, 0.034, 0.235)),
        material=wood_dark,
        name="lower_stretcher",
    )
    frame.visual(
        Box((0.37, 0.038, 0.028)),
        origin=Origin(xyz=(0.0, 0.014, 0.93)),
        material=wood_dark,
        name="upper_stretcher",
    )
    frame.visual(
        Box((0.058, 0.030, 1.42)),
        origin=Origin(xyz=(0.0, 0.000, 0.950)),
        material=wood_dark,
        name="center_rail",
    )
    for index, tooth_z in enumerate((0.38, 0.50, 0.62, 0.74, 0.86, 0.98, 1.10, 1.22)):
        frame.visual(
            Box((0.012, 0.010, 0.024)),
            origin=Origin(xyz=(0.037, -0.023, tooth_z)),
            material=wood,
            name=f"rail_tooth_{index}",
        )
        frame.visual(
            Box((0.004, 0.006, 0.024)),
            origin=Origin(xyz=(0.031, -0.015, tooth_z)),
            material=wood,
            name=f"rail_tooth_mount_{index}",
        )
    frame.visual(
        Box((0.16, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        material=wood_dark,
        name="top_head",
    )
    frame.visual(
        Cylinder(radius=0.023, length=0.160),
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="top_hinge_barrel",
    )
    frame.visual(
        Box((0.070, 0.018, 0.020)),
        origin=Origin(xyz=(-0.332, 0.060, 0.044)),
        material=rubber,
        name="left_foot",
    )
    frame.visual(
        Box((0.070, 0.018, 0.020)),
        origin=Origin(xyz=(0.332, 0.060, 0.044)),
        material=rubber,
        name="right_foot",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 0.24, 1.78)),
        mass=8.5,
        origin=Origin(xyz=(0.0, -0.02, 0.89)),
    )

    rear_brace = model.part("rear_brace")
    rear_brace.visual(
        Cylinder(radius=0.030, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="brace_hinge_sleeve",
    )
    rear_brace.visual(
        Box((0.034, 0.024, 1.63)),
        origin=Origin(xyz=(0.0, -0.030, -0.815)),
        material=wood,
        name="rear_leg_beam",
    )
    rear_brace.visual(
        Box((0.080, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.030, -1.626)),
        material=rubber,
        name="rear_foot",
    )
    rear_brace.inertial = Inertial.from_geometry(
        Box((0.08, 0.06, 1.66)),
        mass=2.5,
        origin=Origin(xyz=(0.0, -0.05, -0.83)),
    )

    shelf = model.part("shelf_tray")
    shelf.visual(
        Box((0.58, 0.12, 0.030)),
        origin=Origin(xyz=(0.0, 0.112, 0.0)),
        material=wood,
        name="tray_board",
    )
    shelf.visual(
        Box((0.58, 0.022, 0.045)),
        origin=Origin(xyz=(0.0, 0.172, 0.008)),
        material=wood_dark,
        name="tray_lip",
    )
    shelf.visual(
        Box((0.110, 0.016, 0.200)),
        origin=Origin(xyz=(0.0, 0.023, 0.0)),
        material=wood_dark,
        name="tray_front_guide",
    )
    shelf.visual(
        Box((0.020, 0.038, 0.200)),
        origin=Origin(xyz=(-0.040, 0.017, 0.0)),
        material=wood_dark,
        name="tray_left_cheek",
    )
    shelf.visual(
        Box((0.020, 0.038, 0.200)),
        origin=Origin(xyz=(0.040, 0.017, 0.0)),
        material=wood_dark,
        name="tray_right_cheek",
    )
    shelf.visual(
        Box((0.150, 0.050, 0.110)),
        origin=Origin(xyz=(0.0, 0.060, -0.040)),
        material=wood_dark,
        name="tray_support_block",
    )
    _add_wing_nut(shelf, x=-0.070, y=0.014, z=0.000, prefix="left_wingnut", material=metal)
    _add_wing_nut(shelf, x=0.070, y=0.014, z=0.000, prefix="right_wingnut", material=metal)
    shelf.inertial = Inertial.from_geometry(
        Box((0.60, 0.20, 0.24)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.085, 0.0)),
    )

    clamp = model.part("top_clamp")
    clamp.visual(
        Box((0.090, 0.016, 0.140)),
        origin=Origin(xyz=(0.0, 0.023, 0.0)),
        material=wood_dark,
        name="clamp_front_guide",
    )
    clamp.visual(
        Box((0.012, 0.030, 0.140)),
        origin=Origin(xyz=(-0.026, 0.030, 0.0)),
        material=wood_dark,
        name="clamp_left_cheek",
    )
    clamp.visual(
        Box((0.012, 0.030, 0.140)),
        origin=Origin(xyz=(0.026, 0.030, 0.0)),
        material=wood_dark,
        name="clamp_right_cheek",
    )
    clamp.visual(
        Box((0.230, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
        material=wood,
        name="clamp_cross_bar",
    )
    clamp.visual(
        Box((0.120, 0.028, 0.090)),
        origin=Origin(xyz=(0.0, 0.090, -0.030)),
        material=wood_dark,
        name="clamp_pressure_pad",
    )
    clamp.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(0.0, 0.102, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="clamp_thumb_screw",
    )
    clamp.inertial = Inertial.from_geometry(
        Box((0.24, 0.14, 0.20)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.060, -0.010)),
    )

    model.articulation(
        "rear_brace_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_brace,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z), rpy=(-0.30, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.14, upper=0.20),
    )
    model.articulation(
        "shelf_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=shelf,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.25, lower=0.0, upper=0.22),
    )
    model.articulation(
        "clamp_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=clamp,
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.20, lower=0.0, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    front_frame = object_model.get_part("front_frame")
    rear_brace = object_model.get_part("rear_brace")
    shelf_tray = object_model.get_part("shelf_tray")
    top_clamp = object_model.get_part("top_clamp")

    rear_brace_hinge = object_model.get_articulation("rear_brace_hinge")
    shelf_slide = object_model.get_articulation("shelf_slide")
    clamp_slide = object_model.get_articulation("clamp_slide")

    left_leg = front_frame.get_visual("left_front_leg")
    right_leg = front_frame.get_visual("right_front_leg")
    center_rail = front_frame.get_visual("center_rail")
    lower_stretcher = front_frame.get_visual("lower_stretcher")
    top_hinge_barrel = front_frame.get_visual("top_hinge_barrel")
    left_foot = front_frame.get_visual("left_foot")
    right_foot = front_frame.get_visual("right_foot")
    low_tooth = front_frame.get_visual("rail_tooth_0")
    high_tooth = front_frame.get_visual("rail_tooth_7")
    brace_hinge_sleeve = rear_brace.get_visual("brace_hinge_sleeve")
    brace_leg_beam = rear_brace.get_visual("rear_leg_beam")
    tray_board = shelf_tray.get_visual("tray_board")
    tray_front_guide = shelf_tray.get_visual("tray_front_guide")
    left_wingnut_hub = shelf_tray.get_visual("left_wingnut_hub")
    right_wingnut_hub = shelf_tray.get_visual("right_wingnut_hub")
    clamp_cross_bar = top_clamp.get_visual("clamp_cross_bar")
    clamp_front_guide = top_clamp.get_visual("clamp_front_guide")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        rear_brace,
        front_frame,
        reason="rear brace hinge sleeve intentionally nests around the metal top pivot barrel",
    )

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(rear_brace, front_frame, elem_a=brace_hinge_sleeve, elem_b=top_hinge_barrel)
    ctx.expect_gap(
        shelf_tray,
        front_frame,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tray_front_guide,
        negative_elem=center_rail,
        name="shelf_carriage_stays_seated_on_inner_rail",
    )
    ctx.expect_gap(
        top_clamp,
        front_frame,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=clamp_front_guide,
        negative_elem=center_rail,
        name="top_clamp_stays_seated_on_front_rail",
    )
    ctx.expect_origin_distance(shelf_tray, front_frame, axes="x", max_dist=0.02)
    ctx.expect_origin_distance(top_clamp, front_frame, axes="x", max_dist=0.02)
    ctx.expect_gap(
        front_frame,
        front_frame,
        axis="x",
        min_gap=0.58,
        positive_elem=right_foot,
        negative_elem=left_foot,
        name="front_feet_form_a_wide_a_frame_stance",
    )
    ctx.expect_gap(
        front_frame,
        front_frame,
        axis="y",
        min_gap=0.002,
        positive_elem=center_rail,
        negative_elem=low_tooth,
        name="notched_positions_step_behind_the_inner_rail",
    )
    ctx.expect_gap(
        front_frame,
        front_frame,
        axis="z",
        min_gap=0.78,
        positive_elem=high_tooth,
        negative_elem=low_tooth,
        name="inner_rail_carries_multiple_notch_levels",
    )
    ctx.expect_gap(
        front_frame,
        rear_brace,
        axis="y",
        min_gap=0.02,
        positive_elem=center_rail,
        negative_elem=brace_leg_beam,
        name="rear_brace_leg_sits_behind_the_front_frame",
    )
    ctx.expect_gap(
        top_clamp,
        shelf_tray,
        axis="z",
        min_gap=0.35,
        positive_elem=clamp_cross_bar,
        negative_elem=tray_board,
        name="top_clamp_starts_well_above_the_shelf",
    )
    ctx.expect_gap(
        shelf_tray,
        shelf_tray,
        axis="x",
        min_gap=0.10,
        positive_elem=right_wingnut_hub,
        negative_elem=left_wingnut_hub,
        name="dual_wingnut_fasteners_flank_the_notched_rail",
    )

    with ctx.pose({shelf_slide: 0.20}):
        ctx.expect_gap(
            shelf_tray,
            front_frame,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=tray_front_guide,
            negative_elem=center_rail,
            name="raised_shelf_remains_on_the_inner_rail",
        )
        ctx.expect_gap(
            top_clamp,
            shelf_tray,
            axis="z",
            min_gap=0.10,
            positive_elem=clamp_cross_bar,
            negative_elem=tray_board,
            name="raised_shelf_stays_below_the_top_clamp",
        )

    with ctx.pose({clamp_slide: 0.30}):
        ctx.expect_gap(
            top_clamp,
            front_frame,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=clamp_front_guide,
            negative_elem=center_rail,
            name="lowered_top_clamp_still_tracks_the_front_rail",
        )
        ctx.expect_gap(
            top_clamp,
            shelf_tray,
            axis="z",
            min_gap=0.08,
            positive_elem=clamp_cross_bar,
            negative_elem=tray_board,
            name="lowered_top_clamp_remains_above_the_shelf",
        )

    with ctx.pose({clamp_slide: 0.42}):
        ctx.expect_gap(
            top_clamp,
            front_frame,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=clamp_front_guide,
            negative_elem=center_rail,
            name="fully_raised_top_clamp_still_tracks_the_front_rail",
        )
        ctx.expect_gap(
            top_clamp,
            shelf_tray,
            axis="z",
            min_gap=0.45,
            positive_elem=clamp_cross_bar,
            negative_elem=tray_board,
            name="fully_raised_top_clamp_clears_canvas_space_above_the_shelf",
        )

    with ctx.pose({rear_brace_hinge: 0.18}):
        ctx.expect_contact(rear_brace, front_frame, elem_a=brace_hinge_sleeve, elem_b=top_hinge_barrel)
        ctx.expect_gap(
            front_frame,
            rear_brace,
            axis="y",
            min_gap=0.02,
            positive_elem=center_rail,
            negative_elem=brace_leg_beam,
            name="rear_brace_can_fold_without_crossing_the_front_frame",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
