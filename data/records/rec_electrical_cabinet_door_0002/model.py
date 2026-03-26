from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

_ORIG_GETCWD = os.getcwd
_ORIG_PATH_ABSOLUTE = pathlib.Path.absolute
_ORIG_PATH_RESOLVE = pathlib.Path.resolve


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        fallback = os.environ.get("PWD", "/")
        try:
            os.chdir(fallback)
        except Exception:
            os.chdir("/")
        return _ORIG_GETCWD()


def _safe_absolute(self):
    try:
        return _ORIG_PATH_ABSOLUTE(self)
    except FileNotFoundError:
        if self.is_absolute():
            return self
        return self.__class__(_safe_getcwd(), *self.parts)


def _safe_resolve(self, strict: bool = False):
    try:
        return _ORIG_PATH_RESOLVE(self, strict=strict)
    except FileNotFoundError:
        absolute = _safe_absolute(self)
        return absolute if not strict else _ORIG_PATH_RESOLVE(absolute, strict=False)


os.getcwd = _safe_getcwd
pathlib.os.getcwd = _safe_getcwd
pathlib.Path.absolute = _safe_absolute
pathlib.Path.resolve = _safe_resolve

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_enclosure")

    cabinet_gray = model.material("cabinet_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    door_gray = model.material("door_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    hardware = model.material("hardware", rgba=(0.26, 0.28, 0.30, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.14, 0.15, 0.16, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.07, 0.08, 0.08, 1.0))
    zinc = model.material("zinc", rgba=(0.62, 0.64, 0.67, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.62, 0.02, 0.82)),
        origin=Origin(xyz=(0.0, -0.10, 0.41)),
        material=cabinet_gray,
        name="back_panel",
    )
    cabinet.visual(
        Box((0.02, 0.22, 0.82)),
        origin=Origin(xyz=(-0.30, 0.00, 0.41)),
        material=cabinet_gray,
        name="left_wall",
    )
    cabinet.visual(
        Box((0.02, 0.22, 0.82)),
        origin=Origin(xyz=(0.30, 0.00, 0.41)),
        material=cabinet_gray,
        name="right_wall",
    )
    cabinet.visual(
        Box((0.58, 0.22, 0.02)),
        origin=Origin(xyz=(0.00, 0.00, 0.81)),
        material=cabinet_gray,
        name="top_wall",
    )
    cabinet.visual(
        Box((0.58, 0.22, 0.02)),
        origin=Origin(xyz=(0.00, 0.00, 0.01)),
        material=cabinet_gray,
        name="bottom_wall",
    )
    cabinet.visual(
        Box((0.544, 0.020, 0.050)),
        origin=Origin(xyz=(0.000, 0.0935, 0.775)),
        material=cabinet_gray,
        name="opening_flange_top",
    )
    cabinet.visual(
        Box((0.544, 0.020, 0.050)),
        origin=Origin(xyz=(0.000, 0.0935, 0.045)),
        material=cabinet_gray,
        name="opening_flange_bottom",
    )
    cabinet.visual(
        Box((0.036, 0.020, 0.720)),
        origin=Origin(xyz=(-0.272, 0.0935, 0.410)),
        material=cabinet_gray,
        name="opening_flange_left",
    )
    cabinet.visual(
        Box((0.036, 0.020, 0.720)),
        origin=Origin(xyz=(0.272, 0.0935, 0.410)),
        material=cabinet_gray,
        name="opening_flange_right",
    )
    cabinet.visual(
        Box((0.024, 0.034, 0.060)),
        origin=Origin(xyz=(0.205, 0.097, 0.775)),
        material=dark_hardware,
        name="upper_receiver",
    )
    cabinet.visual(
        Box((0.024, 0.034, 0.060)),
        origin=Origin(xyz=(0.205, 0.097, 0.045)),
        material=dark_hardware,
        name="lower_receiver",
    )
    cabinet.visual(
        Box((0.030, 0.080, 0.018)),
        origin=Origin(xyz=(-0.314, 0.070, 0.660)),
        material=hardware,
        name="upper_hinge_weldment",
    )
    cabinet.visual(
        Box((0.036, 0.034, 0.140)),
        origin=Origin(xyz=(-0.328, 0.100, 0.660)),
        material=hardware,
        name="upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.030, 0.080, 0.018)),
        origin=Origin(xyz=(-0.314, 0.070, 0.160)),
        material=hardware,
        name="lower_hinge_weldment",
    )
    cabinet.visual(
        Box((0.036, 0.034, 0.140)),
        origin=Origin(xyz=(-0.328, 0.100, 0.160)),
        material=hardware,
        name="lower_hinge_mount",
    )
    cabinet.visual(
        Box((0.022, 0.030, 0.080)),
        origin=Origin(xyz=(0.300, 0.116, 0.270)),
        material=hardware,
        name="hasp_base",
    )
    cabinet.visual(
        Cylinder(radius=0.0035, length=0.028),
        origin=Origin(xyz=(0.294, 0.129, 0.258), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="staple_leg_inner",
    )
    cabinet.visual(
        Cylinder(radius=0.0035, length=0.028),
        origin=Origin(xyz=(0.306, 0.129, 0.258), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="staple_leg_outer",
    )
    cabinet.visual(
        Cylinder(radius=0.0035, length=0.020),
        origin=Origin(xyz=(0.300, 0.143, 0.258), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="staple_bridge",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.62, 0.22, 0.82)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.41)),
    )

    door = model.part("door")
    door.visual(
        Box((0.610, 0.030, 0.800)),
        origin=Origin(xyz=(0.305, 0.015, 0.000)),
        material=door_gray,
        name="outer_skin",
    )
    door.visual(
        Box((0.540, 0.014, 0.045)),
        origin=Origin(xyz=(0.305, -0.007, 0.3775)),
        material=door_gray,
        name="inner_return_top",
    )
    door.visual(
        Box((0.540, 0.014, 0.045)),
        origin=Origin(xyz=(0.305, -0.007, -0.3775)),
        material=door_gray,
        name="inner_return_bottom",
    )
    door.visual(
        Box((0.035, 0.014, 0.700)),
        origin=Origin(xyz=(0.055, -0.007, 0.000)),
        material=door_gray,
        name="inner_return_left",
    )
    door.visual(
        Box((0.035, 0.014, 0.700)),
        origin=Origin(xyz=(0.555, -0.007, 0.000)),
        material=door_gray,
        name="inner_return_right",
    )
    door.visual(
        Box((0.470, 0.005, 0.018)),
        origin=Origin(xyz=(0.305, -0.001, 0.352)),
        material=gasket_black,
        name="gasket_top",
    )
    door.visual(
        Box((0.470, 0.005, 0.018)),
        origin=Origin(xyz=(0.305, -0.001, -0.352)),
        material=gasket_black,
        name="gasket_bottom",
    )
    door.visual(
        Box((0.018, 0.005, 0.650)),
        origin=Origin(xyz=(0.078, -0.001, 0.000)),
        material=gasket_black,
        name="gasket_left",
    )
    door.visual(
        Box((0.018, 0.005, 0.650)),
        origin=Origin(xyz=(0.532, -0.001, 0.000)),
        material=gasket_black,
        name="gasket_right",
    )
    door.visual(
        Box((0.160, 0.024, 0.220)),
        origin=Origin(xyz=(0.305, 0.018, 0.000)),
        material=hardware,
        name="handle_bezel",
    )
    door.visual(
        Box((0.118, 0.014, 0.176)),
        origin=Origin(xyz=(0.305, 0.023, 0.000)),
        material=dark_hardware,
        name="handle_well",
    )
    door.visual(
        Box((0.042, 0.004, 0.360)),
        origin=Origin(xyz=(0.520, -0.014, 0.155)),
        material=hardware,
        name="top_bar_rail_back",
    )
    door.visual(
        Box((0.006, 0.016, 0.360)),
        origin=Origin(xyz=(0.503, -0.008, 0.155)),
        material=hardware,
        name="top_bar_rail_inner",
    )
    door.visual(
        Box((0.006, 0.016, 0.360)),
        origin=Origin(xyz=(0.537, -0.008, 0.155)),
        material=hardware,
        name="top_bar_rail_outer",
    )
    door.visual(
        Box((0.042, 0.004, 0.360)),
        origin=Origin(xyz=(0.520, -0.014, -0.155)),
        material=hardware,
        name="bottom_bar_rail_back",
    )
    door.visual(
        Box((0.006, 0.016, 0.360)),
        origin=Origin(xyz=(0.503, -0.008, -0.155)),
        material=hardware,
        name="bottom_bar_rail_inner",
    )
    door.visual(
        Box((0.006, 0.016, 0.360)),
        origin=Origin(xyz=(0.537, -0.008, -0.155)),
        material=hardware,
        name="bottom_bar_rail_outer",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.160),
        origin=Origin(xyz=(0.000, 0.008, 0.250)),
        material=hardware,
        name="upper_hinge_knuckle",
    )
    door.visual(
        Box((0.055, 0.020, 0.180)),
        origin=Origin(xyz=(0.030, 0.010, 0.250)),
        material=hardware,
        name="upper_hinge_plate",
    )
    door.visual(
        Cylinder(radius=0.014, length=0.160),
        origin=Origin(xyz=(0.000, 0.008, -0.250)),
        material=hardware,
        name="lower_hinge_knuckle",
    )
    door.visual(
        Box((0.055, 0.020, 0.180)),
        origin=Origin(xyz=(0.030, 0.010, -0.250)),
        material=hardware,
        name="lower_hinge_plate",
    )
    door.visual(
        Box((0.032, 0.010, 0.090)),
        origin=Origin(xyz=(0.590, 0.030, -0.140)),
        material=hardware,
        name="door_hasp_leaf",
    )
    door.visual(
        Box((0.040, 0.008, 0.024)),
        origin=Origin(xyz=(0.600, 0.032, -0.140)),
        material=zinc,
        name="door_hasp_eye",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.610, 0.030, 0.800)),
        mass=8.5,
        origin=Origin(xyz=(0.305, 0.015, 0.000)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="spindle",
    )
    handle.visual(
        Box((0.060, 0.012, 0.030)),
        origin=Origin(xyz=(0.000, 0.006, 0.000)),
        material=hardware,
        name="grip_boss",
    )
    handle.visual(
        Box((0.022, 0.010, 0.104)),
        origin=Origin(xyz=(0.000, 0.010, 0.000)),
        material=hardware,
        name="grip",
    )
    handle.visual(
        Box((0.050, 0.012, 0.040)),
        origin=Origin(xyz=(0.000, -0.016, 0.000)),
        material=dark_hardware,
        name="inner_cam",
    )
    handle.visual(
        Box((0.018, 0.008, 0.130)),
        origin=Origin(xyz=(0.008, -0.018, 0.000)),
        material=dark_hardware,
        name="drive_link",
    )
    handle.visual(
        Box((0.215, 0.008, 0.018)),
        origin=Origin(xyz=(0.1075, -0.018, 0.065)),
        material=dark_hardware,
        name="top_coupler",
    )
    handle.visual(
        Box((0.018, 0.008, 0.055)),
        origin=Origin(xyz=(0.215, -0.018, 0.095)),
        material=dark_hardware,
        name="top_crank",
    )
    handle.visual(
        Box((0.215, 0.008, 0.018)),
        origin=Origin(xyz=(0.1075, -0.018, -0.065)),
        material=dark_hardware,
        name="bottom_coupler",
    )
    handle.visual(
        Box((0.018, 0.008, 0.055)),
        origin=Origin(xyz=(0.215, -0.018, -0.095)),
        material=dark_hardware,
        name="bottom_crank",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.278, 0.050, 0.190)),
        mass=0.9,
        origin=Origin(xyz=(0.139, -0.008, 0.000)),
    )

    top_bar = model.part("top_bar")
    top_bar.visual(
        Box((0.014, 0.012, 0.220)),
        origin=Origin(),
        material=dark_hardware,
        name="rod",
    )
    top_bar.visual(
        Box((0.014, 0.012, 0.070)),
        origin=Origin(xyz=(0.000, 0.000, 0.145)),
        material=dark_hardware,
        name="locking_tip",
    )
    top_bar.visual(
        Box((0.020, 0.014, 0.160)),
        origin=Origin(xyz=(0.000, -0.002, 0.000)),
        material=hardware,
        name="guide_shoe",
    )
    top_bar.visual(
        Box((0.020, 0.010, 0.024)),
        origin=Origin(xyz=(-0.005, 0.000, -0.070)),
        material=zinc,
        name="cam_pin",
    )
    top_bar.inertial = Inertial.from_geometry(
        Box((0.024, 0.018, 0.255)),
        mass=0.5,
        origin=Origin(xyz=(0.0, -0.001, 0.017)),
    )

    bottom_bar = model.part("bottom_bar")
    bottom_bar.visual(
        Box((0.014, 0.012, 0.220)),
        origin=Origin(),
        material=dark_hardware,
        name="rod",
    )
    bottom_bar.visual(
        Box((0.014, 0.012, 0.070)),
        origin=Origin(xyz=(0.000, 0.000, -0.145)),
        material=dark_hardware,
        name="locking_tip",
    )
    bottom_bar.visual(
        Box((0.020, 0.014, 0.160)),
        origin=Origin(xyz=(0.000, -0.002, 0.000)),
        material=hardware,
        name="guide_shoe",
    )
    bottom_bar.visual(
        Box((0.020, 0.010, 0.024)),
        origin=Origin(xyz=(-0.005, 0.000, 0.070)),
        material=zinc,
        name="cam_pin",
    )
    bottom_bar.inertial = Inertial.from_geometry(
        Box((0.024, 0.018, 0.255)),
        mass=0.5,
        origin=Origin(xyz=(0.0, -0.001, -0.017)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent="cabinet",
        child="door",
        origin=Origin(xyz=(-0.317, 0.1035, 0.410)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.75),
    )
    model.articulation(
        "handle_turn",
        ArticulationType.REVOLUTE,
        parent="door",
        child="handle",
        origin=Origin(xyz=(0.305, 0.022, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-1.10, upper=0.0),
    )
    model.articulation(
        "top_bar_slide",
        ArticulationType.PRISMATIC,
        parent="door",
        child="top_bar",
        origin=Origin(xyz=(0.520, -0.008, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=-0.040, upper=0.0),
    )
    model.articulation(
        "bottom_bar_slide",
        ArticulationType.PRISMATIC,
        parent="door",
        child="bottom_bar",
        origin=Origin(xyz=(0.520, -0.008, -0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=0.040),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    handle = object_model.get_part("handle")
    top_bar = object_model.get_part("top_bar")
    bottom_bar = object_model.get_part("bottom_bar")

    door_hinge = object_model.get_articulation("door_hinge")
    handle_turn = object_model.get_articulation("handle_turn")
    top_bar_slide = object_model.get_articulation("top_bar_slide")
    bottom_bar_slide = object_model.get_articulation("bottom_bar_slide")

    flange_top = cabinet.get_visual("opening_flange_top")
    flange_right = cabinet.get_visual("opening_flange_right")
    upper_receiver = cabinet.get_visual("upper_receiver")
    lower_receiver = cabinet.get_visual("lower_receiver")
    upper_hinge_weldment = cabinet.get_visual("upper_hinge_weldment")
    lower_hinge_weldment = cabinet.get_visual("lower_hinge_weldment")
    upper_hinge_mount = cabinet.get_visual("upper_hinge_mount")
    lower_hinge_mount = cabinet.get_visual("lower_hinge_mount")
    staple_bridge = cabinet.get_visual("staple_bridge")

    inner_return_top = door.get_visual("inner_return_top")
    inner_return_bottom = door.get_visual("inner_return_bottom")
    inner_return_left = door.get_visual("inner_return_left")
    inner_return_right = door.get_visual("inner_return_right")
    gasket_top = door.get_visual("gasket_top")
    gasket_bottom = door.get_visual("gasket_bottom")
    gasket_left = door.get_visual("gasket_left")
    gasket_right = door.get_visual("gasket_right")
    upper_hinge_knuckle = door.get_visual("upper_hinge_knuckle")
    upper_hinge_plate = door.get_visual("upper_hinge_plate")
    lower_hinge_knuckle = door.get_visual("lower_hinge_knuckle")
    lower_hinge_plate = door.get_visual("lower_hinge_plate")
    handle_well = door.get_visual("handle_well")
    top_bar_rail = door.get_visual("top_bar_rail_back")
    bottom_bar_rail = door.get_visual("bottom_bar_rail_back")
    door_hasp_eye = door.get_visual("door_hasp_eye")

    handle_grip = handle.get_visual("grip")
    top_tip = top_bar.get_visual("locking_tip")
    top_shoe = top_bar.get_visual("guide_shoe")
    bottom_tip = bottom_bar.get_visual("locking_tip")
    bottom_shoe = bottom_bar.get_visual("guide_shoe")

    ctx.allow_overlap(
        door,
        cabinet,
        reason="compressible neoprene gasket and tight hinge seating intentionally preload the weatherproof door against the front flange",
    )
    ctx.allow_overlap(
        handle,
        door,
        reason="the recessed swing handle rotates inside its pocket and spindle bosses",
    )
    ctx.allow_overlap(
        top_bar,
        door,
        reason="the upper locking rod is captured by a welded guide rail",
    )
    ctx.allow_overlap(
        bottom_bar,
        door,
        reason="the lower locking rod is captured by a welded guide rail",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(door, cabinet, axes="xz", min_overlap=0.30)
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        max_gap=0.003,
        max_penetration=0.004,
        positive_elem=gasket_top,
        negative_elem=flange_top,
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="x",
        max_gap=0.006,
        max_penetration=0.006,
        positive_elem=upper_hinge_plate,
        negative_elem=upper_hinge_mount,
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="x",
        max_gap=0.006,
        max_penetration=0.006,
        positive_elem=lower_hinge_plate,
        negative_elem=lower_hinge_mount,
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="yz",
        min_overlap=0.012,
        elem_a=upper_hinge_knuckle,
        elem_b=upper_hinge_weldment,
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="yz",
        min_overlap=0.012,
        elem_a=lower_hinge_knuckle,
        elem_b=lower_hinge_weldment,
    )
    ctx.expect_gap(
        handle,
        door,
        axis="y",
        max_gap=0.020,
        max_penetration=0.012,
        positive_elem=handle_grip,
        negative_elem=handle_well,
    )
    ctx.expect_within(handle, door, axes="xz", inner_elem=handle_grip, outer_elem=handle_well)
    ctx.expect_overlap(door, door, axes="xz", min_overlap=0.003, elem_a=gasket_top, elem_b=inner_return_top)
    ctx.expect_overlap(door, door, axes="xz", min_overlap=0.003, elem_a=gasket_bottom, elem_b=inner_return_bottom)
    ctx.expect_overlap(door, door, axes="yz", min_overlap=0.003, elem_a=gasket_left, elem_b=inner_return_left)
    ctx.expect_overlap(door, door, axes="yz", min_overlap=0.003, elem_a=gasket_right, elem_b=inner_return_right)
    ctx.expect_gap(
        top_bar,
        door,
        axis="y",
        max_gap=0.003,
        max_penetration=0.006,
        positive_elem=top_shoe,
        negative_elem=top_bar_rail,
    )
    ctx.expect_gap(
        bottom_bar,
        door,
        axis="y",
        max_gap=0.003,
        max_penetration=0.006,
        positive_elem=bottom_shoe,
        negative_elem=bottom_bar_rail,
    )
    ctx.expect_gap(top_bar, bottom_bar, axis="z", min_gap=0.080)
    ctx.expect_gap(
        cabinet,
        top_bar,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=upper_receiver,
        negative_elem=top_tip,
    )
    ctx.expect_gap(
        bottom_bar,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=bottom_tip,
        negative_elem=lower_receiver,
    )
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        max_gap=0.010,
        max_penetration=0.002,
        positive_elem=staple_bridge,
        negative_elem=door_hasp_eye,
    )

    with ctx.pose({handle_turn: -1.10, top_bar_slide: -0.040, bottom_bar_slide: 0.040}):
        ctx.expect_gap(
            cabinet,
            top_bar,
            axis="z",
            min_gap=0.030,
            positive_elem=upper_receiver,
            negative_elem=top_tip,
        )
        ctx.expect_gap(
            bottom_bar,
            cabinet,
            axis="z",
            min_gap=0.030,
            positive_elem=bottom_tip,
            negative_elem=lower_receiver,
        )
        ctx.expect_gap(
            top_bar,
            door,
            axis="y",
            max_gap=0.003,
            max_penetration=0.006,
            positive_elem=top_shoe,
            negative_elem=top_bar_rail,
        )
        ctx.expect_gap(
            bottom_bar,
            door,
            axis="y",
            max_gap=0.003,
            max_penetration=0.006,
            positive_elem=bottom_shoe,
            negative_elem=bottom_bar_rail,
        )
        ctx.expect_within(handle, door, axes="xz", inner_elem=handle_grip, outer_elem=handle_well)
        ctx.expect_gap(top_bar, bottom_bar, axis="z", min_gap=0.008)

    with ctx.pose(
        {
            door_hinge: 1.45,
            handle_turn: -1.10,
            top_bar_slide: -0.040,
            bottom_bar_slide: 0.040,
        }
    ):
        ctx.expect_gap(
            handle,
            cabinet,
            axis="y",
            min_gap=0.100,
            positive_elem=handle_grip,
            negative_elem=flange_right,
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="yz",
            min_overlap=0.010,
            elem_a=upper_hinge_plate,
            elem_b=upper_hinge_mount,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
