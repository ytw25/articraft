from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except FileNotFoundError:
            pass
        return "/"


os.getcwd = _safe_getcwd

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


BODY_D = 0.74
BODY_W = 1.36
BODY_H = 0.74
LEG_H = 0.06
BODY_WALL = 0.028
BODY_FLOOR = 0.034

LID_D = BODY_D + 0.016
LID_W = BODY_W + 0.044
LID_H = 0.056
LID_LIP_T = 0.018
LID_LIP_H = 0.028
LID_TOP = 0.022

HINGE_X = -BODY_D / 2.0 - 0.014
HINGE_Z = LEG_H + BODY_H + 0.011
BODY_KNUCKLE_X = HINGE_X - 0.020
LID_CENTER_X = LID_D / 2.0 + 0.016

HASP_PIVOT_X = BODY_D + 0.026
HASP_PIVOT_Z = -0.014
HASP_W = 0.084
HASP_H = 0.118
HASP_T = 0.006
def _add_leveling_leg(part, x: float, y: float, material) -> None:
    part.visual(
        Cylinder(radius=0.013, length=0.044),
        origin=Origin(xyz=(x, y, 0.038)),
        material=material,
    )
    part.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(x, y, 0.008)),
        material=material,
    )


def _add_corner_channel(part, *, x_sign: float, y_sign: float, material) -> None:
    z_center = LEG_H + (BODY_H * 0.5) + 0.005
    z_height = BODY_H - 0.065
    strip_w = 0.054
    strip_t = 0.007
    bead = 0.012

    front_back_x = x_sign * (BODY_D / 2.0 - strip_w / 2.0 + 0.001)
    side_y = y_sign * (BODY_W / 2.0 - strip_w / 2.0 + 0.001)
    face_y = y_sign * (BODY_W / 2.0 + strip_t / 2.0 - 0.001)
    face_x = x_sign * (BODY_D / 2.0 + strip_t / 2.0 - 0.001)
    bead_x = x_sign * (BODY_D / 2.0 + bead / 2.0 - 0.001)
    bead_y = y_sign * (BODY_W / 2.0 + bead / 2.0 - 0.001)

    part.visual(
        Box((strip_w, strip_t, z_height)),
        origin=Origin(xyz=(front_back_x, face_y, z_center)),
        material=material,
    )
    part.visual(
        Box((strip_t, strip_w, z_height)),
        origin=Origin(xyz=(face_x, side_y, z_center)),
        material=material,
    )
    part.visual(
        Box((bead, bead, z_height)),
        origin=Origin(xyz=(bead_x, bead_y, z_center)),
        material=material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garage_chest_freezer")

    freezer_white = model.material("freezer_white", rgba=(0.91, 0.92, 0.90, 1.0))
    liner_white = model.material("liner_white", rgba=(0.96, 0.97, 0.95, 1.0))
    hardware_metal = model.material("hardware_metal", rgba=(0.64, 0.67, 0.70, 1.0))
    channel_metal = model.material("channel_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.18, 0.19, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WALL, BODY_W, BODY_H)),
        origin=Origin(xyz=(BODY_D / 2.0 - BODY_WALL / 2.0, 0.0, LEG_H + (BODY_H * 0.5))),
        material=freezer_white,
        name="front_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_W, BODY_H)),
        origin=Origin(xyz=(-BODY_D / 2.0 + BODY_WALL / 2.0, 0.0, LEG_H + (BODY_H * 0.5))),
        material=freezer_white,
        name="rear_wall",
    )
    body.visual(
        Box((BODY_D - 2.0 * BODY_WALL, BODY_WALL, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_W / 2.0 - BODY_WALL / 2.0, LEG_H + (BODY_H * 0.5))),
        material=freezer_white,
        name="right_wall",
    )
    body.visual(
        Box((BODY_D - 2.0 * BODY_WALL, BODY_WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_W / 2.0 + BODY_WALL / 2.0, LEG_H + (BODY_H * 0.5))),
        material=freezer_white,
        name="left_wall",
    )
    body.visual(
        Box((BODY_D - 2.0 * BODY_WALL, BODY_W - 2.0 * BODY_WALL, BODY_FLOOR)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + BODY_FLOOR / 2.0)),
        material=liner_white,
        name="body_floor",
    )
    body.visual(
        Box((0.040, BODY_W - 0.016, 0.020)),
        origin=Origin(xyz=(BODY_D / 2.0 - 0.020, 0.0, LEG_H + BODY_H - 0.010)),
        material=freezer_white,
        name="front_rim",
    )
    body.visual(
        Box((0.030, BODY_W - 0.030, 0.018)),
        origin=Origin(xyz=(-BODY_D / 2.0 + 0.015, 0.0, LEG_H + BODY_H - 0.009)),
        material=freezer_white,
        name="rear_rim",
    )
    body.visual(
        Box((BODY_D - 0.060, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, BODY_W / 2.0 - 0.010, LEG_H + BODY_H - 0.009)),
        material=freezer_white,
        name="right_rim",
    )
    body.visual(
        Box((BODY_D - 0.060, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -BODY_W / 2.0 + 0.010, LEG_H + BODY_H - 0.009)),
        material=freezer_white,
        name="left_rim",
    )
    body.visual(
        Box((BODY_D - 0.090, BODY_W - 0.090, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, LEG_H + BODY_FLOOR + 0.005)),
        material=liner_white,
        name="interior_floor",
    )
    body.visual(
        Cylinder(radius=0.010, length=BODY_W - 0.050),
        origin=Origin(
            xyz=(BODY_KNUCKLE_X, 0.0, HINGE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_metal,
        name="body_hinge_knuckle",
    )
    body.visual(
        Box((0.034, BODY_W - 0.070, 0.010)),
        origin=Origin(xyz=(BODY_KNUCKLE_X + 0.017, 0.0, HINGE_Z - 0.015)),
        material=hardware_metal,
        name="body_hinge_leaf",
    )
    body.visual(
        Box((0.034, 0.096, 0.016)),
        origin=Origin(xyz=(BODY_D / 2.0 - 0.008, 0.0, LEG_H + BODY_H - 0.086)),
        material=hardware_metal,
        name="staple_base",
    )
    body.visual(
        Box((0.016, 0.024, 0.031)),
        origin=Origin(xyz=(BODY_D / 2.0 + 0.008, -0.019, LEG_H + BODY_H - 0.070)),
        material=hardware_metal,
        name="staple_left",
    )
    body.visual(
        Box((0.016, 0.024, 0.031)),
        origin=Origin(xyz=(BODY_D / 2.0 + 0.008, 0.019, LEG_H + BODY_H - 0.070)),
        material=hardware_metal,
        name="staple_right",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.044),
        origin=Origin(
            xyz=(BODY_D / 2.0 + 0.010, 0.0, LEG_H + BODY_H - 0.086),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_metal,
        name="staple_bridge",
    )
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            _add_leveling_leg(
                body,
                sx * (BODY_D / 2.0 - 0.060),
                sy * (BODY_W / 2.0 - 0.085),
                black_plastic,
            )
    body.inertial = Inertial.from_geometry(
        Box((BODY_D, BODY_W, BODY_H + LEG_H)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + LEG_H) * 0.5)),
    )

    corner_channels = model.part("corner_channels")
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            _add_corner_channel(corner_channels, x_sign=x_sign, y_sign=y_sign, material=channel_metal)
    corner_channels.visual(
        Box((BODY_D - 0.104, 0.007, 0.018)),
        origin=Origin(xyz=(0.0, BODY_W / 2.0 + 0.002, LEG_H + 0.039)),
        material=channel_metal,
        name="right_lower_trim",
    )
    corner_channels.visual(
        Box((BODY_D - 0.104, 0.007, 0.018)),
        origin=Origin(xyz=(0.0, -BODY_W / 2.0 - 0.002, LEG_H + 0.039)),
        material=channel_metal,
        name="left_lower_trim",
    )
    corner_channels.visual(
        Box((0.007, BODY_W - 0.104, 0.018)),
        origin=Origin(xyz=(-BODY_D / 2.0 - 0.002, 0.0, LEG_H + 0.039)),
        material=channel_metal,
        name="rear_lower_trim",
    )
    corner_channels.inertial = Inertial.from_geometry(
        Box((BODY_D + 0.020, BODY_W + 0.020, BODY_H)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, LEG_H + BODY_H * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_D - 0.032, LID_W, LID_TOP)),
        origin=Origin(xyz=(LID_CENTER_X - 0.002, 0.0, 0.003)),
        material=freezer_white,
        name="lid_shell",
    )
    lid.visual(
        Box((LID_LIP_T, LID_W - 0.040, LID_LIP_H)),
        origin=Origin(
            xyz=(LID_CENTER_X - 0.002 + (LID_D - 0.032) / 2.0 + LID_LIP_T / 2.0, 0.0, -0.022),
        ),
        material=freezer_white,
        name="lid_front_lip",
    )
    lid.visual(
        Box((0.050, LID_W - 0.082, 0.012)),
        origin=Origin(xyz=(0.025, 0.0, -0.005)),
        material=freezer_white,
        name="lid_rear_leaf",
    )
    lid.visual(
        Box((LID_D - 0.060, LID_LIP_T, LID_LIP_H)),
        origin=Origin(xyz=(LID_CENTER_X - 0.010, LID_W / 2.0 - LID_LIP_T / 2.0, -0.022)),
        material=freezer_white,
        name="lid_right_lip",
    )
    lid.visual(
        Box((LID_D - 0.060, LID_LIP_T, LID_LIP_H)),
        origin=Origin(xyz=(LID_CENTER_X - 0.010, -LID_W / 2.0 + LID_LIP_T / 2.0, -0.022)),
        material=freezer_white,
        name="lid_left_lip",
    )
    lid.visual(
        Box((BODY_D - 0.100, BODY_W - 0.100, 0.008)),
        origin=Origin(xyz=(BODY_D / 2.0 + 0.014, 0.0, -0.012)),
        material=gasket_dark,
        name="lid_gasket",
    )
    lid.visual(
        Cylinder(radius=0.010, length=BODY_W - 0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_metal,
        name="lid_hinge_knuckle",
    )
    lid.visual(
        Box((0.012, 0.064, 0.010)),
        origin=Origin(xyz=(HASP_PIVOT_X - 0.004, 0.0, -0.015)),
        material=hardware_metal,
        name="hasp_mount",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.082),
        origin=Origin(
            xyz=(HASP_PIVOT_X, 0.0, HASP_PIVOT_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware_metal,
        name="hasp_pivot_pin",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_D, LID_W, LID_H)),
        mass=16.0,
        origin=Origin(xyz=(LID_CENTER_X, 0.0, 0.0)),
    )

    hasp_plate = model.part("hasp_plate")
    hasp_plate.visual(
        Box((HASP_T, HASP_W, HASP_H)),
        origin=Origin(xyz=(HASP_T * 0.5, 0.0, -(HASP_H * 0.5))),
        material=hardware_metal,
        name="hasp_plate",
    )
    hasp_plate.visual(
        Cylinder(radius=0.0075, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_metal,
        name="plate_sleeve",
    )
    hasp_plate.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(
            xyz=(0.010, 0.0, -0.092),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_metal,
        name="padlock_boss",
    )
    hasp_plate.inertial = Inertial.from_geometry(
        Box((0.020, HASP_W, HASP_H)),
        mass=0.45,
        origin=Origin(xyz=(0.010, 0.0, -(HASP_H * 0.5))),
    )

    model.articulation(
        "body_to_corner_channels",
        ArticulationType.FIXED,
        parent=body,
        child=corner_channels,
        origin=Origin(),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.8, lower=0.0, upper=1.22),
    )
    model.articulation(
        "lid_to_hasp",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp_plate,
        origin=Origin(xyz=(HASP_PIVOT_X, 0.0, HASP_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.38),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    corner_channels = object_model.get_part("corner_channels")
    lid = object_model.get_part("lid")
    hasp_plate = object_model.get_part("hasp_plate")
    lid_hinge = object_model.get_articulation("body_to_lid")
    hasp_hinge = object_model.get_articulation("lid_to_hasp")
    body_knuckle = body.get_visual("body_hinge_knuckle")
    body_floor = body.get_visual("body_floor")
    front_wall = body.get_visual("front_wall")
    right_wall = body.get_visual("right_wall")
    staple_base = body.get_visual("staple_base")
    staple_bridge = body.get_visual("staple_bridge")
    lid_front_lip = lid.get_visual("lid_front_lip")
    lid_right_lip = lid.get_visual("lid_right_lip")
    lid_gasket = lid.get_visual("lid_gasket")
    lid_knuckle = lid.get_visual("lid_hinge_knuckle")
    lid_shell = lid.get_visual("lid_shell")
    lid_pin = lid.get_visual("hasp_pivot_pin")
    plate_face = hasp_plate.get_visual("hasp_plate")
    plate_sleeve = hasp_plate.get_visual("plate_sleeve")
    padlock_boss = hasp_plate.get_visual("padlock_boss")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.allow_overlap(hasp_plate, lid, reason="hasp sleeve nests around the lid pivot pin")
    ctx.check_articulation_overlaps(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(corner_channels, body)
    ctx.expect_overlap(corner_channels, body, axes="z", min_overlap=0.60)
    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.62)
    ctx.expect_origin_distance(lid, body, axes="y", max_dist=0.01)
    ctx.expect_within(
        lid,
        body,
        axes="xy",
        inner_elem=lid_gasket,
        outer_elem=body_floor,
        name="lid_gasket_sits_inside_the_body_opening",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=lid_knuckle,
        negative_elem=body_knuckle,
        name="rear_piano_knuckles_run_close_without_intersection",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="yz",
        min_overlap=0.018,
        elem_a=lid_knuckle,
        elem_b=body_knuckle,
        name="rear_piano_knuckles_run_the_full_body_width",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="x",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem=lid_front_lip,
        negative_elem=front_wall,
        name="front_lid_lip_covers_the_front_face_without_cutting_in",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=lid_right_lip,
        negative_elem=right_wall,
        name="side_lid_lip_covers_the_side_face_without_cutting_in",
    )
    ctx.expect_overlap(
        hasp_plate,
        lid,
        axes="yz",
        min_overlap=0.012,
        elem_a=plate_sleeve,
        elem_b=lid_pin,
        name="hasp_plate_mounts_on_front_pivot_pin",
    )
    ctx.expect_overlap(
        hasp_plate,
        body,
        axes="yz",
        min_overlap=0.008,
        elem_a=padlock_boss,
        elem_b=staple_bridge,
        name="padlock_boss_aligns_with_staple_when_latched",
    )
    ctx.expect_gap(
        hasp_plate,
        body,
        axis="x",
        max_gap=0.005,
        max_penetration=0.002,
        positive_elem=plate_face,
        negative_elem=staple_base,
        name="latched_hasp_swings_over_fixed_staple_bracket",
    )
    with ctx.pose({hasp_hinge: 1.25}):
        ctx.expect_gap(
            hasp_plate,
            body,
            axis="z",
            min_gap=0.020,
            positive_elem=padlock_boss,
            negative_elem=staple_bridge,
            name="opened_hasp_lifts_clear_of_the_staple",
        )
        ctx.expect_overlap(
            hasp_plate,
            lid,
            axes="yz",
            min_overlap=0.012,
            elem_a=plate_sleeve,
            elem_b=lid_pin,
            name="hasp_stays_carried_by_the_lid_when_opened",
        )
    with ctx.pose({lid_hinge: 1.05, hasp_hinge: 1.25}):
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem=lid_knuckle,
            negative_elem=body_knuckle,
            name="rear_knuckles_remain_seated_as_the_lid_opens",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.45,
            positive_elem=lid_front_lip,
            negative_elem=staple_base,
            name="opened_lid_lifts_well_clear_of_the_front_face",
        )
        ctx.expect_overlap(
            hasp_plate,
            lid,
            axes="yz",
            min_overlap=0.012,
            elem_a=plate_sleeve,
            elem_b=lid_pin,
            name="hasp_pivot_remains_nested_with_lid_open",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
