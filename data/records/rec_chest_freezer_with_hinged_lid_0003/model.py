from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.458
BODY_DEPTH = 0.314
BODY_HEIGHT = 0.320
OUTER_RADIUS = 0.030

SHELL_HOLE_WIDTH = 0.390
SHELL_HOLE_DEPTH = 0.246
SHELL_HOLE_RADIUS = 0.016

LINER_Z0 = 0.022
LINER_HEIGHT = 0.291
LINER_OUTER_WIDTH = 0.380
LINER_OUTER_DEPTH = 0.236
LINER_OUTER_RADIUS = 0.013
LINER_INNER_WIDTH = 0.366
LINER_INNER_DEPTH = 0.222
LINER_INNER_RADIUS = 0.009
LINER_FLOOR_THICKNESS = 0.010

TOP_BRIDGE_HEIGHT = BODY_HEIGHT - (LINER_Z0 + LINER_HEIGHT)
FRONT_BACK_GAP = 0.5 * (SHELL_HOLE_DEPTH - LINER_OUTER_DEPTH)
SIDE_GAP = 0.5 * (SHELL_HOLE_WIDTH - LINER_OUTER_WIDTH)

HINGE_Y = 0.160
HINGE_Z = 0.330
HINGE_KNUCKLE_X = 0.118


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rounded_plate_mesh(width: float, depth: float, radius: float, thickness: float, name: str):
    return _save_mesh(
        name,
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius, corner_segments=8),
            height=thickness,
            center=False,
        ),
    )


def _rounded_ring_mesh(
    outer_width: float,
    outer_depth: float,
    outer_radius: float,
    inner_width: float,
    inner_depth: float,
    inner_radius: float,
    height: float,
    name: str,
):
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_width, outer_depth, outer_radius, corner_segments=10),
            [rounded_rect_profile(inner_width, inner_depth, inner_radius, corner_segments=10)],
            height=height,
            center=False,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_chest_freezer", assets=ASSETS)

    shell_white = model.material("shell_white", rgba=(0.94, 0.95, 0.96, 1.0))
    liner_grey = model.material("liner_grey", rgba=(0.84, 0.86, 0.89, 1.0))
    gasket_dark = model.material("gasket_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.61, 0.66, 1.0))
    satin = model.material("satin", rgba=(0.70, 0.72, 0.75, 1.0))

    body = model.part("body")
    body.visual(
        _rounded_ring_mesh(
            BODY_WIDTH,
            BODY_DEPTH,
            OUTER_RADIUS,
            SHELL_HOLE_WIDTH,
            SHELL_HOLE_DEPTH,
            SHELL_HOLE_RADIUS,
            BODY_HEIGHT,
            "freezer_outer_shell.obj",
        ),
        material=shell_white,
        name="outer_shell",
    )
    body.visual(
        _rounded_ring_mesh(
            LINER_OUTER_WIDTH,
            LINER_OUTER_DEPTH,
            LINER_OUTER_RADIUS,
            LINER_INNER_WIDTH,
            LINER_INNER_DEPTH,
            LINER_INNER_RADIUS,
            LINER_HEIGHT,
            "freezer_liner_wall.obj",
        ),
        origin=Origin(xyz=(0.0, 0.0, LINER_Z0)),
        material=liner_grey,
        name="liner_wall",
    )
    body.visual(
        _rounded_plate_mesh(
            LINER_INNER_WIDTH,
            LINER_INNER_DEPTH,
            LINER_INNER_RADIUS,
            LINER_FLOOR_THICKNESS,
            "freezer_liner_floor.obj",
        ),
        origin=Origin(xyz=(0.0, 0.0, LINER_Z0)),
        material=liner_grey,
        name="liner_floor",
    )
    body.visual(
        Box((LINER_OUTER_WIDTH, FRONT_BACK_GAP, TOP_BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -0.5 * LINER_OUTER_DEPTH - (0.5 * FRONT_BACK_GAP),
                BODY_HEIGHT - (0.5 * TOP_BRIDGE_HEIGHT),
            )
        ),
        material=liner_grey,
        name="front_bridge",
    )
    body.visual(
        Box((LINER_OUTER_WIDTH, FRONT_BACK_GAP, TOP_BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.5 * LINER_OUTER_DEPTH + (0.5 * FRONT_BACK_GAP),
                BODY_HEIGHT - (0.5 * TOP_BRIDGE_HEIGHT),
            )
        ),
        material=liner_grey,
        name="rear_bridge",
    )
    body.visual(
        Box((SIDE_GAP, LINER_OUTER_DEPTH, TOP_BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(
                -0.5 * LINER_OUTER_WIDTH - (0.5 * SIDE_GAP),
                0.0,
                BODY_HEIGHT - (0.5 * TOP_BRIDGE_HEIGHT),
            )
        ),
        material=liner_grey,
        name="left_bridge",
    )
    body.visual(
        Box((SIDE_GAP, LINER_OUTER_DEPTH, TOP_BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.5 * LINER_OUTER_WIDTH + (0.5 * SIDE_GAP),
                0.0,
                BODY_HEIGHT - (0.5 * TOP_BRIDGE_HEIGHT),
            )
        ),
        material=liner_grey,
        name="right_bridge",
    )
    body.visual(
        Box((0.178, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.155, 0.314)),
        material=steel,
        name="keeper_strip",
    )
    body.visual(
        Box((0.036, 0.002, 0.030)),
        origin=Origin(xyz=(0.138, 0.156, 0.060)),
        material=shell_white,
        name="drain_seat",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=11.4,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BODY_HEIGHT)),
    )

    hinge_hardware = model.part("hinge_hardware")
    hinge_hardware.visual(
        Cylinder(radius=0.006, length=0.304),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="hinge_pin",
    )
    hinge_hardware.visual(
        Box((0.028, 0.014, 0.028)),
        origin=Origin(xyz=(-HINGE_KNUCKLE_X, -0.010, -0.015)),
        material=steel,
        name="left_mount",
    )
    hinge_hardware.visual(
        Box((0.028, 0.014, 0.028)),
        origin=Origin(xyz=(HINGE_KNUCKLE_X, -0.010, -0.015)),
        material=steel,
        name="right_mount",
    )
    hinge_hardware.visual(
        Box((0.260, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, -0.018)),
        material=steel,
        name="rear_leaf",
    )
    hinge_hardware.inertial = Inertial.from_geometry(
        Box((0.304, 0.022, 0.040)),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.006, -0.010)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.370, 0.246, 0.018)),
        origin=Origin(xyz=(0.0, -0.126, 0.002)),
        material=shell_white,
        name="main_panel",
    )
    lid.visual(
        Box((0.108, 0.054, 0.018)),
        origin=Origin(xyz=(-0.131, -0.276, 0.002)),
        material=shell_white,
        name="left_shoulder",
    )
    lid.visual(
        Box((0.108, 0.054, 0.018)),
        origin=Origin(xyz=(0.131, -0.276, 0.002)),
        material=shell_white,
        name="right_shoulder",
    )
    lid.visual(
        Box((0.020, 0.300, 0.018)),
        origin=Origin(xyz=(-0.175, -0.151, 0.002)),
        material=shell_white,
        name="left_side_rail",
    )
    lid.visual(
        Box((0.020, 0.300, 0.018)),
        origin=Origin(xyz=(0.175, -0.151, 0.002)),
        material=shell_white,
        name="right_side_rail",
    )
    lid.visual(
        Box((0.300, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.012, 0.002)),
        material=shell_white,
        name="rear_strip",
    )
    lid.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(-0.166, -0.276, 0.002)),
        material=shell_white,
        name="front_left_round",
    )
    lid.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(0.166, -0.276, 0.002)),
        material=shell_white,
        name="front_right_round",
    )
    lid.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(-0.166, -0.014, 0.002)),
        material=shell_white,
        name="rear_left_round",
    )
    lid.visual(
        Cylinder(radius=0.019, length=0.018),
        origin=Origin(xyz=(0.166, -0.014, 0.002)),
        material=shell_white,
        name="rear_right_round",
    )
    lid.visual(
        Box((0.132, 0.042, 0.006)),
        origin=Origin(xyz=(0.0, -0.276, -0.016)),
        material=liner_grey,
        name="handle_floor",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.132),
        origin=Origin(xyz=(0.0, -0.250, -0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=liner_grey,
        name="handle_back",
    )
    lid.visual(
        Box((0.010, 0.056, 0.016)),
        origin=Origin(xyz=(-0.073, -0.270, -0.024)),
        material=liner_grey,
        name="handle_left_cheek",
    )
    lid.visual(
        Box((0.010, 0.056, 0.016)),
        origin=Origin(xyz=(0.073, -0.270, -0.024)),
        material=liner_grey,
        name="handle_right_cheek",
    )
    lid.visual(
        Box((0.154, 0.022, 0.014)),
        origin=Origin(xyz=(0.0, -0.306, -0.010)),
        material=shell_white,
        name="front_lip",
    )
    lid.visual(
        Box((0.176, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.319, -0.018)),
        material=gasket_dark,
        name="latch_strip",
    )
    lid.visual(
        Box((0.330, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.016, -0.012)),
        material=gasket_dark,
        name="rear_gasket",
    )
    lid.visual(
        Box((0.008, 0.198, 0.010)),
        origin=Origin(xyz=(-0.170, -0.155, -0.012)),
        material=gasket_dark,
        name="left_gasket",
    )
    lid.visual(
        Box((0.008, 0.198, 0.010)),
        origin=Origin(xyz=(0.170, -0.155, -0.012)),
        material=gasket_dark,
        name="right_gasket",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(-HINGE_KNUCKLE_X, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="left_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(xyz=(HINGE_KNUCKLE_X, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin,
        name="right_knuckle",
    )
    lid.visual(
        Box((0.040, 0.018, 0.006)),
        origin=Origin(xyz=(-HINGE_KNUCKLE_X, -0.008, -0.004)),
        material=steel,
        name="left_knuckle_plate",
    )
    lid.visual(
        Box((0.040, 0.018, 0.006)),
        origin=Origin(xyz=(HINGE_KNUCKLE_X, -0.008, -0.004)),
        material=steel,
        name="right_knuckle_plate",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.392, 0.320, 0.032)),
        mass=1.25,
        origin=Origin(xyz=(0.0, -0.150, -0.016)),
    )

    drain_plug = model.part("drain_plug")
    drain_plug.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="threaded_neck",
    )
    drain_plug.visual(
        Cylinder(radius=0.012, length=0.0025),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="thread_ring_a",
    )
    drain_plug.visual(
        Cylinder(radius=0.012, length=0.0025),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="thread_ring_b",
    )
    drain_plug.visual(
        Cylinder(radius=0.012, length=0.0025),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="thread_ring_c",
    )
    drain_plug.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_white,
        name="drain_cap",
    )
    drain_plug.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.040),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_hinge_hardware",
        ArticulationType.FIXED,
        parent=body,
        child=hinge_hardware,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
    )
    model.articulation(
        "hinge_hardware_to_lid",
        ArticulationType.REVOLUTE,
        parent=hinge_hardware,
        child=lid,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.22),
    )
    model.articulation(
        "body_to_drain_plug",
        ArticulationType.FIXED,
        parent=body,
        child=drain_plug,
        origin=Origin(xyz=(0.138, 0.155, 0.060)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    hinge_hardware = object_model.get_part("hinge_hardware")
    lid = object_model.get_part("lid")
    drain_plug = object_model.get_part("drain_plug")
    lid_hinge = object_model.get_articulation("hinge_hardware_to_lid")

    outer_shell = body.get_visual("outer_shell")
    keeper_strip = body.get_visual("keeper_strip")
    liner_floor = body.get_visual("liner_floor")
    drain_seat = body.get_visual("drain_seat")

    hinge_pin = hinge_hardware.get_visual("hinge_pin")
    left_mount = hinge_hardware.get_visual("left_mount")
    right_mount = hinge_hardware.get_visual("right_mount")

    main_panel = lid.get_visual("main_panel")
    handle_floor = lid.get_visual("handle_floor")
    handle_back = lid.get_visual("handle_back")
    latch_strip = lid.get_visual("latch_strip")
    left_knuckle = lid.get_visual("left_knuckle")
    right_knuckle = lid.get_visual("right_knuckle")

    threaded_neck = drain_plug.get_visual("threaded_neck")
    drain_cap = drain_plug.get_visual("drain_cap")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(hinge_hardware, lid, reason="rear piano-hinge knuckles sleeve around the fixed hinge pin")
    ctx.allow_overlap(body, lid, reason="closed lid nests into the cabinet opening with gasket compression and a recessed front handle pocket")
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.0015, overlap_volume_tol=0.0)
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.0015,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_within(body, body, axes="xy", inner_elem=liner_floor, outer_elem=outer_shell)
    ctx.expect_gap(
        body,
        body,
        axis="z",
        positive_elem=keeper_strip,
        negative_elem=liner_floor,
        min_gap=0.270,
        max_gap=0.300,
    )
    ctx.expect_overlap(lid, body, axes="xy", elem_a=main_panel, elem_b=outer_shell, min_overlap=0.080)
    ctx.expect_gap(
        body,
        lid,
        axis="y",
        positive_elem=keeper_strip,
        negative_elem=latch_strip,
        max_gap=0.002,
        max_penetration=0.0005,
    )
    ctx.expect_contact(hinge_hardware, body, elem_a=left_mount, elem_b=outer_shell)
    ctx.expect_contact(hinge_hardware, body, elem_a=right_mount, elem_b=outer_shell)
    ctx.expect_overlap(lid, hinge_hardware, axes="xy", elem_a=left_knuckle, elem_b=hinge_pin, min_overlap=0.001)
    ctx.expect_overlap(lid, hinge_hardware, axes="xy", elem_a=right_knuckle, elem_b=hinge_pin, min_overlap=0.001)
    ctx.expect_gap(
        lid,
        lid,
        axis="z",
        positive_elem=main_panel,
        negative_elem=handle_floor,
        min_gap=0.005,
        max_gap=0.015,
    )
    ctx.expect_gap(
        lid,
        lid,
        axis="y",
        positive_elem=handle_back,
        negative_elem=latch_strip,
        min_gap=0.030,
        max_gap=0.070,
    )
    ctx.expect_contact(drain_plug, body, elem_a=threaded_neck, elem_b=drain_seat)
    ctx.expect_gap(
        drain_plug,
        body,
        axis="y",
        positive_elem=drain_cap,
        negative_elem=drain_seat,
        min_gap=0.010,
        max_gap=0.025,
    )
    ctx.expect_gap(
        body,
        drain_plug,
        axis="z",
        positive_elem=keeper_strip,
        negative_elem=drain_cap,
        min_gap=0.220,
        max_gap=0.290,
    )

    with ctx.pose({lid_hinge: 1.05}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=handle_floor,
            negative_elem=outer_shell,
            min_gap=0.100,
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=latch_strip,
            negative_elem=keeper_strip,
            min_gap=0.090,
        )
        ctx.expect_overlap(lid, hinge_hardware, axes="xy", elem_a=left_knuckle, elem_b=hinge_pin, min_overlap=0.001)
        ctx.expect_overlap(lid, hinge_hardware, axes="xy", elem_a=right_knuckle, elem_b=hinge_pin, min_overlap=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
