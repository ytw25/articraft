from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        return "/tmp"


os.getcwd = _safe_getcwd
try:
    os.chdir("/tmp")
except FileNotFoundError:
    pass

ASSETS = AssetContext(root="/tmp")

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coin_laundromat_washer", assets=ASSETS)

    cabinet_white = model.material("cabinet_white", rgba=(0.90, 0.91, 0.92, 1.0))
    appliance_gray = model.material("appliance_gray", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_charcoal = model.material("dark_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    stainless = model.material("stainless", rgba=(0.68, 0.70, 0.73, 1.0))
    drum_steel = model.material("drum_steel", rgba=(0.47, 0.49, 0.52, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.62, 0.74, 0.80, 0.35))
    coin_box_gray = model.material("coin_box_gray", rgba=(0.54, 0.56, 0.60, 1.0))

    cabinet_width = 0.72
    cabinet_depth = 0.74
    cabinet_height = 1.00
    side_thickness = 0.025
    bottom_thickness = 0.050
    top_thickness = 0.030
    front_thickness = 0.040

    front_y = cabinet_depth * 0.5 - front_thickness * 0.5
    back_y = -cabinet_depth * 0.5 + side_thickness * 0.5
    opening_center_z = 0.58
    opening_radius = 0.204
    front_inner_width = cabinet_width - 2.0 * side_thickness
    front_side_strip_width = (front_inner_width - 2.0 * opening_radius) * 0.5
    front_side_strip_x = opening_radius + front_side_strip_width * 0.5
    top_strip_height = 0.160
    bottom_strip_height = 0.276

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-cabinet_width * 0.5 + side_thickness * 0.5, 0.0, cabinet_height * 0.5)),
        material=cabinet_white,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((side_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(cabinet_width * 0.5 - side_thickness * 0.5, 0.0, cabinet_height * 0.5)),
        material=cabinet_white,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, cabinet_height - top_thickness * 0.5)),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness * 0.5)),
        material=appliance_gray,
        name="bottom_plinth",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * side_thickness, side_thickness, cabinet_height - bottom_thickness)),
        origin=Origin(xyz=(0.0, back_y, bottom_thickness + (cabinet_height - bottom_thickness) * 0.5)),
        material=cabinet_white,
        name="back_panel",
    )
    cabinet.visual(
        Box((0.050, front_thickness, 0.90)),
        origin=Origin(xyz=(-0.310, front_y, 0.55)),
        material=cabinet_white,
        name="left_front_strip",
    )
    cabinet.visual(
        Box((front_side_strip_width, front_thickness, 0.90)),
        origin=Origin(xyz=(front_side_strip_x, front_y, 0.55)),
        material=cabinet_white,
        name="front_panel",
    )
    cabinet.visual(
        Box((front_inner_width, front_thickness, top_strip_height)),
        origin=Origin(xyz=(0.0, front_y, 0.92)),
        material=cabinet_white,
        name="top_front_strip",
    )
    cabinet.visual(
        Box((front_inner_width, front_thickness, bottom_strip_height)),
        origin=Origin(xyz=(0.0, front_y, 0.238)),
        material=cabinet_white,
        name="bottom_front_strip",
    )
    cabinet.visual(
        Box((cabinet_width - 2.0 * side_thickness, front_thickness, 0.10)),
        origin=Origin(xyz=(0.0, front_y, 0.05)),
        material=appliance_gray,
        name="kick_panel",
    )
    cabinet.visual(
        Box((0.020, 0.012, 0.280)),
        origin=Origin(xyz=(0.214, 0.351, opening_center_z)),
        material=dark_rubber,
        name="door_aperture_lip",
    )
    cabinet.visual(
        Box((0.020, 0.012, 0.408)),
        origin=Origin(xyz=(-0.214, 0.351, opening_center_z)),
        material=dark_rubber,
        name="left_aperture_lip",
    )
    cabinet.visual(
        Box((0.444, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.351, opening_center_z + 0.214)),
        material=dark_rubber,
        name="top_aperture_lip",
    )
    cabinet.visual(
        Box((0.444, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.351, opening_center_z - 0.214)),
        material=dark_rubber,
        name="bottom_aperture_lip",
    )
    cabinet.visual(
        Cylinder(radius=0.215, length=0.180),
        origin=Origin(xyz=(0.0, 0.240, opening_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="drum_shroud",
    )
    cabinet.visual(
        Cylinder(radius=0.172, length=0.460),
        origin=Origin(xyz=(0.0, 0.070, opening_center_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum_steel,
        name="inner_drum",
    )
    cabinet.visual(
        Box((0.010, 0.060, 0.102)),
        origin=Origin(xyz=(-0.340, 0.340, opening_center_z + 0.126)),
        material=stainless,
        name="upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.010, 0.060, 0.102)),
        origin=Origin(xyz=(-0.340, 0.340, opening_center_z - 0.126)),
        material=stainless,
        name="lower_hinge_mount",
    )
    cabinet.visual(
        Box((0.100, 0.120, 0.228)),
        origin=Origin(xyz=(0.313, 0.430, 0.792)),
        material=coin_box_gray,
        name="coin_box_housing",
    )
    cabinet.visual(
        Box((0.090, 0.006, 0.030)),
        origin=Origin(xyz=(0.313, 0.493, 0.867)),
        material=stainless,
        name="coin_faceplate_top",
    )
    cabinet.visual(
        Box((0.090, 0.006, 0.007)),
        origin=Origin(xyz=(0.313, 0.493, 0.8495)),
        material=stainless,
        name="coin_slot_upper_lip",
    )
    cabinet.visual(
        Box((0.090, 0.006, 0.007)),
        origin=Origin(xyz=(0.313, 0.493, 0.8385)),
        material=stainless,
        name="coin_slot_lower_lip",
    )
    cabinet.visual(
        Box((0.022, 0.006, 0.011)),
        origin=Origin(xyz=(0.280, 0.493, 0.844)),
        material=stainless,
        name="coin_slot_left_shoulder",
    )
    cabinet.visual(
        Box((0.022, 0.006, 0.011)),
        origin=Origin(xyz=(0.346, 0.493, 0.844)),
        material=stainless,
        name="coin_slot_right_shoulder",
    )
    cabinet.visual(
        Box((0.090, 0.006, 0.130)),
        origin=Origin(xyz=(0.313, 0.493, 0.767)),
        material=stainless,
        name="coin_faceplate_bottom",
    )
    cabinet.visual(
        Box((0.044, 0.018, 0.011)),
        origin=Origin(xyz=(0.313, 0.479, 0.844)),
        material=dark_charcoal,
        name="coin_slot_tunnel",
    )
    cabinet.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.192, -0.380, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=appliance_gray,
        name="drain_collar",
    )
    cabinet.visual(
        Cylinder(radius=0.027, length=0.090),
        origin=Origin(xyz=(0.192, -0.415, 0.112), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="drain_stub",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    door = model.part("door")
    door.visual(
        Cylinder(radius=0.230, length=0.072),
        origin=Origin(xyz=(0.230, 0.082, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="door_frame",
    )
    door.visual(
        Cylinder(radius=0.236, length=0.018),
        origin=Origin(xyz=(0.230, 0.108, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=appliance_gray,
        name="door_bezel",
    )
    door.visual(
        Cylinder(radius=0.178, length=0.022),
        origin=Origin(xyz=(0.230, 0.060, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="door_inner_ring",
    )
    door.visual(
        Cylinder(radius=0.205, length=0.052),
        origin=Origin(xyz=(0.230, 0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="door_seal",
    )
    door.visual(
        Cylinder(radius=0.143, length=0.018),
        origin=Origin(xyz=(0.230, 0.074, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoked_glass,
        name="porthole_glass",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.300),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.096, 0.036, 0.094)),
        origin=Origin(xyz=(0.048, 0.030, 0.126)),
        material=stainless,
        name="upper_hinge_plate",
    )
    door.visual(
        Box((0.096, 0.036, 0.094)),
        origin=Origin(xyz=(0.048, 0.030, -0.126)),
        material=stainless,
        name="lower_hinge_plate",
    )
    door.visual(
        Box((0.052, 0.022, 0.190)),
        origin=Origin(xyz=(0.362, 0.088, 0.0)),
        material=appliance_gray,
        name="handle_base",
    )
    door.visual(
        Box((0.032, 0.040, 0.170)),
        origin=Origin(xyz=(0.392, 0.112, 0.0)),
        material=stainless,
        name="pull_handle",
    )
    door.visual(
        Box((0.012, 0.010, 0.300)),
        origin=Origin(xyz=(0.348, 0.086, 0.0)),
        material=stainless,
        name="latch_control_rod",
    )
    door.visual(
        Box((0.086, 0.010, 0.012)),
        origin=Origin(xyz=(0.325, 0.086, 0.0)),
        material=stainless,
        name="latch_link",
    )
    door.visual(
        Box((0.026, 0.026, 0.014)),
        origin=Origin(xyz=(0.434, 0.090, 0.126)),
        material=stainless,
        name="latch_top",
    )
    door.visual(
        Box((0.030, 0.026, 0.014)),
        origin=Origin(xyz=(0.438, 0.090, 0.0)),
        material=stainless,
        name="latch_mid",
    )
    door.visual(
        Box((0.026, 0.026, 0.014)),
        origin=Origin(xyz=(0.434, 0.090, -0.126)),
        material=stainless,
        name="latch_bottom",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.470, 0.090, 0.470)),
        mass=16.0,
        origin=Origin(xyz=(0.235, 0.035, 0.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.230, 0.363, opening_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.3, lower=0.0, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")

    front_panel = cabinet.get_visual("front_panel")
    back_panel = cabinet.get_visual("back_panel")
    door_aperture_lip = cabinet.get_visual("door_aperture_lip")
    upper_hinge_mount = cabinet.get_visual("upper_hinge_mount")
    lower_hinge_mount = cabinet.get_visual("lower_hinge_mount")
    coin_box = cabinet.get_visual("coin_box_housing")
    coin_slot_upper_lip = cabinet.get_visual("coin_slot_upper_lip")
    coin_slot_lower_lip = cabinet.get_visual("coin_slot_lower_lip")
    coin_slot_tunnel = cabinet.get_visual("coin_slot_tunnel")
    drain_stub = cabinet.get_visual("drain_stub")

    door_frame = door.get_visual("door_frame")
    door_seal = door.get_visual("door_seal")
    porthole_glass = door.get_visual("porthole_glass")
    hinge_barrel = door.get_visual("hinge_barrel")
    upper_hinge_plate = door.get_visual("upper_hinge_plate")
    lower_hinge_plate = door.get_visual("lower_hinge_plate")
    handle_base = door.get_visual("handle_base")
    pull_handle = door.get_visual("pull_handle")
    latch_control_rod = door.get_visual("latch_control_rod")
    latch_top = door.get_visual("latch_top")
    latch_mid = door.get_visual("latch_mid")
    latch_bottom = door.get_visual("latch_bottom")

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

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    ctx.expect_gap(
        door,
        door,
        axis="z",
        min_gap=0.14,
        positive_elem=upper_hinge_plate,
        negative_elem=lower_hinge_plate,
        name="two_heavy_door_hinge_plates_are_stacked_on_left_side",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        min_gap=0.14,
        positive_elem=upper_hinge_mount,
        negative_elem=lower_hinge_mount,
        name="cabinet_hinge_backers_match_the_two_plate_layout",
    )
    ctx.expect_gap(
        door,
        door,
        axis="y",
        max_gap=0.003,
        max_penetration=0.004,
        positive_elem=door_frame,
        negative_elem=upper_hinge_plate,
        name="upper_hinge_plate_is_seated_against_the_door_frame",
    )
    ctx.expect_gap(
        door,
        door,
        axis="y",
        max_gap=0.003,
        max_penetration=0.004,
        positive_elem=door_frame,
        negative_elem=lower_hinge_plate,
        name="lower_hinge_plate_is_seated_against_the_door_frame",
    )
    ctx.expect_overlap(
        door,
        door,
        axes="xz",
        min_overlap=0.010,
        elem_a=hinge_barrel,
        elem_b=upper_hinge_plate,
        name="upper_hinge_plate_wraps_the_hinge_barrel",
    )
    ctx.expect_overlap(
        door,
        door,
        axes="xz",
        min_overlap=0.010,
        elem_a=hinge_barrel,
        elem_b=lower_hinge_plate,
        name="lower_hinge_plate_wraps_the_hinge_barrel",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        max_gap=0.003,
        max_penetration=0.003,
        positive_elem=door_seal,
        negative_elem=door_aperture_lip,
        name="door_seal_compresses_lightly_against_aperture_lip",
    )
    ctx.expect_within(
        door,
        door,
        axes="xz",
        inner_elem=porthole_glass,
        outer_elem=door_frame,
        name="porthole_glass_nested_inside_heavy_door_frame",
    )
    ctx.expect_gap(
        door,
        door,
        axis="z",
        min_gap=0.10,
        positive_elem=latch_top,
        negative_elem=latch_mid,
        name="top_latch_sits_above_mid_latch",
    )
    ctx.expect_gap(
        door,
        door,
        axis="y",
        max_gap=0.002,
        max_penetration=0.008,
        positive_elem=pull_handle,
        negative_elem=handle_base,
        name="pull_handle_is_mounted_to_the_single_handle_base",
    )
    ctx.expect_gap(
        door,
        door,
        axis="y",
        max_gap=0.002,
        max_penetration=0.020,
        positive_elem=latch_control_rod,
        negative_elem=handle_base,
        name="vertical_latch_control_rod_connects_into_the_handle_base",
    )
    ctx.expect_overlap(
        door,
        door,
        axes="z",
        min_overlap=0.010,
        elem_a=latch_control_rod,
        elem_b=latch_top,
        name="control_rod_reaches_the_upper_latch_level",
    )
    ctx.expect_overlap(
        door,
        door,
        axes="z",
        min_overlap=0.010,
        elem_a=latch_control_rod,
        elem_b=latch_bottom,
        name="control_rod_reaches_the_lower_latch_level",
    )
    ctx.expect_gap(
        door,
        door,
        axis="z",
        min_gap=0.10,
        positive_elem=latch_mid,
        negative_elem=latch_bottom,
        name="mid_latch_sits_above_bottom_latch",
    )
    ctx.expect_gap(
        door,
        door,
        axis="x",
        min_gap=0.30,
        positive_elem=latch_mid,
        negative_elem=upper_hinge_plate,
        name="three_point_latch_cluster_opposes_left_hinge",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=coin_box,
        negative_elem=front_panel,
        name="coin_box_housing_seated_on_right_front_face",
    )
    ctx.expect_within(
        cabinet,
        cabinet,
        axes="xz",
        inner_elem=coin_slot_tunnel,
        outer_elem=coin_box,
        name="coin_slot_tunnel_recessed_within_coin_box",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        min_gap=0.003,
        positive_elem=coin_slot_upper_lip,
        negative_elem=coin_slot_lower_lip,
        name="coin_slot_aperture_reads_as_open_slit",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="y",
        min_gap=0.001,
        positive_elem=coin_slot_upper_lip,
        negative_elem=coin_slot_tunnel,
        name="coin_slot_opens_into_recessed_tunnel",
    )
    ctx.expect_gap(
        cabinet,
        door,
        axis="x",
        min_gap=0.020,
        positive_elem=coin_box,
        negative_elem=door_frame,
        name="coin_box_housing_sits_to_right_of_door",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=back_panel,
        negative_elem=drain_stub,
        name="drain_stub_seated_on_rear_wall",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        min_gap=0.22,
        positive_elem=door_aperture_lip,
        negative_elem=drain_stub,
        name="drain_stub_remains_low_on_rear_wall",
    )
    with ctx.pose({door_hinge: 1.35}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.20,
            positive_elem=pull_handle,
            negative_elem=front_panel,
            name="door_pull_handle_swings_forward_when_open",
        )
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.18,
            positive_elem=latch_mid,
            negative_elem=front_panel,
            name="three_point_latch_side_swings_clear_in_open_pose",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
