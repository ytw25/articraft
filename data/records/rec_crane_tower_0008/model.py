from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os


_ORIGINAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIGINAL_GETCWD()
    except FileNotFoundError:
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


def _beam_pose(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        raise ValueError("beam endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _add_beam(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
    *,
    name: str | None = None,
) -> None:
    origin, length = _beam_pose(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
    *,
    name: str | None = None,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_mast_lattice(
    part,
    *,
    z0: float,
    z1: float,
    half_span: float,
    bays: int,
    radius: float,
    material,
) -> None:
    corners = [
        (half_span, half_span),
        (half_span, -half_span),
        (-half_span, -half_span),
        (-half_span, half_span),
    ]
    for x, y in corners:
        _add_beam(part, (x, y, z0), (x, y, z1), radius, material)

    zs = [z0 + (z1 - z0) * i / bays for i in range(bays + 1)]
    for z in zs:
        for idx in range(4):
            ax, ay = corners[idx]
            bx, by = corners[(idx + 1) % 4]
            _add_beam(part, (ax, ay, z), (bx, by, z), radius * 0.9, material)

    for lower, upper in zip(zs[:-1], zs[1:]):
        for idx in range(4):
            ax, ay = corners[idx]
            bx, by = corners[(idx + 1) % 4]
            _add_beam(part, (ax, ay, lower), (bx, by, upper), radius * 0.8, material)
            _add_beam(part, (ax, ay, upper), (bx, by, lower), radius * 0.8, material)


def _add_truss_boom(
    part,
    *,
    x0: float,
    x1: float,
    half_width: float,
    half_depth: float,
    panels: int,
    radius: float,
    material,
    root_frame_name: str,
    tip_frame_name: str | None = None,
) -> None:
    corners = [
        (half_width, half_depth),
        (-half_width, half_depth),
        (half_width, -half_depth),
        (-half_width, -half_depth),
    ]
    for y, z in corners:
        _add_beam(part, (x0, y, z), (x1, y, z), radius, material)

    xs = [x0 + (x1 - x0) * i / panels for i in range(panels + 1)]
    for i, x in enumerate(xs):
        frame_name = None
        if i == 0:
            frame_name = root_frame_name
        elif i == len(xs) - 1 and tip_frame_name is not None:
            frame_name = tip_frame_name
        _add_box(
            part,
            (0.018, half_width * 2.0 + 0.02, half_depth * 2.0 + 0.02),
            (x + 0.009 if x1 > x0 and i == 0 else x - 0.009 if x1 < x0 and i == len(xs) - 1 else x, 0.0, 0.0),
            material,
            name=frame_name,
        )
        _add_beam(part, (x, half_width, -half_depth), (x, half_width, half_depth), radius * 0.85, material)
        _add_beam(part, (x, -half_width, -half_depth), (x, -half_width, half_depth), radius * 0.85, material)
        _add_beam(part, (x, -half_width, half_depth), (x, half_width, half_depth), radius * 0.85, material)
        _add_beam(part, (x, -half_width, -half_depth), (x, half_width, -half_depth), radius * 0.85, material)

    for xa, xb in zip(xs[:-1], xs[1:]):
        for y in (half_width, -half_width):
            _add_beam(part, (xa, y, -half_depth), (xb, y, half_depth), radius * 0.75, material)
            _add_beam(part, (xa, y, half_depth), (xb, y, -half_depth), radius * 0.75, material)
        _add_beam(part, (xa, -half_width, half_depth), (xb, half_width, half_depth), radius * 0.7, material)
        _add_beam(part, (xa, half_width, -half_depth), (xb, -half_width, -half_depth), radius * 0.7, material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hammerhead_flat_top_tower_crane")

    steel_yellow = model.material("steel_yellow", rgba=(0.92, 0.76, 0.16, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.32, 0.34, 0.36, 1.0))
    ballast_gray = model.material("ballast_gray", rgba=(0.73, 0.73, 0.72, 1.0))
    cable_dark = model.material("cable_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    anchor_frame = model.part("anchor_frame")
    mast_head = model.part("mast_head")
    forward_jib = model.part("forward_jib")
    counter_jib = model.part("counter_jib")
    ballast = model.part("ballast")
    trolley = model.part("trolley")

    anchor_frame.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 0.22)),
        mass=40.0,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )
    mast_head.inertial = Inertial.from_geometry(
        Box((0.26, 0.26, 1.02)),
        mass=60.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )
    forward_jib.inertial = Inertial.from_geometry(
        Box((1.32, 0.16, 0.14)),
        mass=26.0,
        origin=Origin(xyz=(0.66, 0.0, 0.0)),
    )
    counter_jib.inertial = Inertial.from_geometry(
        Box((0.60, 0.18, 0.16)),
        mass=18.0,
        origin=Origin(xyz=(-0.30, 0.0, 0.0)),
    )
    ballast.inertial = Inertial.from_geometry(
        Box((0.16, 0.16, 0.15)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )
    trolley.inertial = Inertial.from_geometry(
        Box((0.10, 0.14, 0.40)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, -0.20)),
    )

    # Four-point anchor frame and splayed base legs.
    for x in (-0.19, 0.19):
        for y in (-0.19, 0.19):
            _add_box(anchor_frame, (0.08, 0.08, 0.04), (x, y, 0.02), ballast_gray)
    for x in (-0.19, 0.19):
        _add_box(anchor_frame, (0.08, 0.30, 0.03), (x, 0.0, 0.055), steel_dark)
    for y in (-0.19, 0.19):
        _add_box(anchor_frame, (0.30, 0.08, 0.03), (0.0, y, 0.055), steel_dark)
    _add_beam(anchor_frame, (-0.19, -0.19, 0.07), (0.19, 0.19, 0.07), 0.011, steel_dark)
    _add_beam(anchor_frame, (-0.19, 0.19, 0.07), (0.19, -0.19, 0.07), 0.011, steel_dark)
    _add_box(anchor_frame, (0.20, 0.20, 0.02), (0.0, 0.0, 0.19), steel_dark, name="tower_mount")
    leg_top = 0.11
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            _add_beam(
                anchor_frame,
                (0.19 * x_sign, 0.19 * y_sign, 0.04),
                (leg_top * x_sign, leg_top * y_sign, 0.19),
                0.011,
                steel_yellow,
            )
    _add_beam(anchor_frame, (-leg_top, -leg_top, 0.19), (leg_top, leg_top, 0.19), 0.010, steel_yellow)
    _add_beam(anchor_frame, (-leg_top, leg_top, 0.19), (leg_top, -leg_top, 0.19), 0.010, steel_yellow)

    # Lattice mast and flat-top head assembly.
    _add_box(mast_head, (0.16, 0.16, 0.03), (0.0, 0.0, 0.015), steel_dark, name="mast_collar")
    _add_mast_lattice(
        mast_head,
        z0=0.03,
        z1=0.88,
        half_span=0.08,
        bays=6,
        radius=0.008,
        material=steel_yellow,
    )
    _add_box(mast_head, (0.18, 0.18, 0.10), (0.0, 0.0, 0.93), steel_dark)
    _add_box(mast_head, (0.28, 0.18, 0.025), (0.0, 0.0, 0.99), steel_dark)
    _add_box(mast_head, (0.02, 0.16, 0.12), (0.07, 0.0, 0.93), steel_dark, name="head_front_mount")
    _add_box(mast_head, (0.02, 0.16, 0.12), (-0.07, 0.0, 0.93), steel_dark, name="head_rear_mount")
    for y in (-0.06, 0.06):
        _add_beam(mast_head, (0.0, y, 0.88), (0.12, y, 0.99), 0.007, steel_yellow)
        _add_beam(mast_head, (0.0, y, 0.88), (-0.12, y, 0.99), 0.007, steel_yellow)

    # Forward hammerhead jib with trolley rails underneath.
    _add_truss_boom(
        forward_jib,
        x0=0.0,
        x1=1.32,
        half_width=0.07,
        half_depth=0.05,
        panels=8,
        radius=0.007,
        material=steel_yellow,
        root_frame_name="root_frame",
        tip_frame_name="tip_frame",
    )
    rail_length = 1.08
    rail_center_x = 0.64
    rail_z = -0.064
    _add_box(forward_jib, (rail_length, 0.012, 0.010), (rail_center_x, 0.036, rail_z), steel_dark, name="rail_left")
    _add_box(forward_jib, (rail_length, 0.012, 0.010), (rail_center_x, -0.036, rail_z), steel_dark, name="rail_right")
    _add_beam(forward_jib, (0.10, 0.07, -0.05), (0.10, 0.036, rail_z + 0.005), 0.0045, steel_dark)
    _add_beam(forward_jib, (0.10, -0.07, -0.05), (0.10, -0.036, rail_z + 0.005), 0.0045, steel_dark)
    _add_beam(forward_jib, (1.18, 0.07, -0.05), (1.18, 0.036, rail_z + 0.005), 0.0045, steel_dark)
    _add_beam(forward_jib, (1.18, -0.07, -0.05), (1.18, -0.036, rail_z + 0.005), 0.0045, steel_dark)

    # Rear counter-jib with ballast tray.
    _add_truss_boom(
        counter_jib,
        x0=-0.60,
        x1=0.0,
        half_width=0.08,
        half_depth=0.04,
        panels=4,
        radius=0.007,
        material=steel_yellow,
        root_frame_name="root_frame",
        tip_frame_name="tail_frame",
    )
    _add_box(counter_jib, (0.022, 0.18, 0.10), (-0.011, 0.0, 0.0), steel_dark, name="head_mount")
    _add_box(counter_jib, (0.22, 0.18, 0.02), (-0.41, 0.0, -0.04), steel_dark, name="ballast_tray")
    _add_box(counter_jib, (0.06, 0.12, 0.05), (-0.41, 0.0, -0.015), steel_dark, name="ballast_saddle")

    # Concrete counterweights stacked on the rear jib.
    _add_box(ballast, (0.16, 0.16, 0.05), (0.0, 0.0, -0.005), ballast_gray, name="ballast_block_1")
    _add_box(ballast, (0.15, 0.15, 0.05), (0.0, 0.0, 0.045), ballast_gray, name="ballast_block_2")
    _add_box(ballast, (0.14, 0.14, 0.05), (0.0, 0.0, 0.095), ballast_gray, name="ballast_block_3")

    # Traveling trolley with twin rail saddles, a suspended carriage, and hook block.
    _add_box(trolley, (0.06, 0.020, 0.016), (0.0, 0.036, -0.008), steel_dark, name="saddle_left")
    _add_box(trolley, (0.06, 0.020, 0.016), (0.0, -0.036, -0.008), steel_dark, name="saddle_right")
    _add_box(trolley, (0.085, 0.100, 0.030), (0.0, 0.0, -0.028), steel_dark, name="trolley_frame")
    _add_box(trolley, (0.050, 0.055, 0.018), (0.0, 0.0, -0.088), steel_dark, name="hanger_beam")
    _add_box(trolley, (0.012, 0.018, 0.180), (0.0, 0.022, -0.120), steel_dark, name="hanger_left")
    _add_box(trolley, (0.012, 0.018, 0.180), (0.0, -0.022, -0.120), steel_dark, name="hanger_right")
    trolley.visual(
        Cylinder(radius=0.005, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, -0.1175)),
        material=cable_dark,
        name="hoist_line",
    )
    _add_box(trolley, (0.050, 0.040, 0.060), (0.0, 0.0, -0.225), steel_dark, name="hook_block")
    trolley.visual(
        Cylinder(radius=0.005, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.300)),
        material=steel_dark,
        name="hook_tip",
    )

    model.articulation(
        "anchor_to_mast",
        ArticulationType.FIXED,
        parent=anchor_frame,
        child=mast_head,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )
    model.articulation(
        "mast_to_forward_jib",
        ArticulationType.FIXED,
        parent=mast_head,
        child=forward_jib,
        origin=Origin(xyz=(0.08, 0.0, 0.93)),
    )
    model.articulation(
        "mast_to_counter_jib",
        ArticulationType.FIXED,
        parent=mast_head,
        child=counter_jib,
        origin=Origin(xyz=(-0.08, 0.0, 0.93)),
    )
    model.articulation(
        "counter_jib_to_ballast",
        ArticulationType.FIXED,
        parent=counter_jib,
        child=ballast,
        origin=Origin(xyz=(-0.41, 0.0, 0.0)),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=forward_jib,
        child=trolley,
        origin=Origin(xyz=(0.26, 0.0, -0.069)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.8, lower=0.0, upper=0.82),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root="/")
    anchor_frame = object_model.get_part("anchor_frame")
    mast_head = object_model.get_part("mast_head")
    forward_jib = object_model.get_part("forward_jib")
    counter_jib = object_model.get_part("counter_jib")
    ballast = object_model.get_part("ballast")
    trolley = object_model.get_part("trolley")
    trolley_travel = object_model.get_articulation("trolley_travel")

    tower_mount = anchor_frame.get_visual("tower_mount")
    mast_collar = mast_head.get_visual("mast_collar")
    head_front_mount = mast_head.get_visual("head_front_mount")
    head_rear_mount = mast_head.get_visual("head_rear_mount")
    forward_root = forward_jib.get_visual("root_frame")
    counter_head_mount = counter_jib.get_visual("head_mount")
    ballast_tray = counter_jib.get_visual("ballast_tray")
    ballast_block_1 = ballast.get_visual("ballast_block_1")
    ballast_block_2 = ballast.get_visual("ballast_block_2")
    ballast_block_3 = ballast.get_visual("ballast_block_3")
    rail_left = forward_jib.get_visual("rail_left")
    rail_right = forward_jib.get_visual("rail_right")
    saddle_left = trolley.get_visual("saddle_left")
    saddle_right = trolley.get_visual("saddle_right")
    trolley_frame = trolley.get_visual("trolley_frame")
    hook_block = trolley.get_visual("hook_block")

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
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_overlap(mast_head, anchor_frame, axes="xy", min_overlap=0.02, elem_a=mast_collar, elem_b=tower_mount)
    ctx.expect_gap(
        mast_head,
        anchor_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=mast_collar,
        negative_elem=tower_mount,
    )
    ctx.expect_overlap(forward_jib, mast_head, axes="yz", min_overlap=0.02, elem_a=forward_root, elem_b=head_front_mount)
    ctx.expect_gap(
        forward_jib,
        mast_head,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=forward_root,
        negative_elem=head_front_mount,
    )
    ctx.expect_overlap(counter_jib, mast_head, axes="yz", min_overlap=0.02, elem_a=counter_head_mount, elem_b=head_rear_mount)
    ctx.expect_gap(
        mast_head,
        counter_jib,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=head_rear_mount,
        negative_elem=counter_head_mount,
    )
    ctx.expect_overlap(ballast, counter_jib, axes="xy", min_overlap=0.02, elem_a=ballast_block_1, elem_b=ballast_tray)
    ctx.expect_gap(
        ballast,
        counter_jib,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=ballast_block_1,
        negative_elem=ballast_tray,
    )
    ctx.expect_gap(
        ballast,
        ballast,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=ballast_block_2,
        negative_elem=ballast_block_1,
        name="middle_ballast_block_seats_on_lower_block",
    )
    ctx.expect_gap(
        ballast,
        ballast,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=ballast_block_3,
        negative_elem=ballast_block_2,
        name="upper_ballast_block_seats_on_middle_block",
    )
    ctx.expect_within(
        ballast,
        ballast,
        axes="xy",
        inner_elem=ballast_block_3,
        outer_elem=ballast_block_1,
        name="ballast_stack_tapers_within_bottom_block_footprint",
    )
    ctx.expect_overlap(trolley, forward_jib, axes="x", min_overlap=0.05, elem_a=saddle_left, elem_b=rail_left)
    ctx.expect_gap(
        forward_jib,
        trolley,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rail_left,
        negative_elem=saddle_left,
    )
    ctx.expect_gap(
        forward_jib,
        trolley,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rail_right,
        negative_elem=saddle_right,
    )
    ctx.expect_within(trolley, forward_jib, axes="y", inner_elem=trolley_frame)
    ctx.expect_gap(
        forward_jib,
        trolley,
        axis="z",
        min_gap=0.18,
        positive_elem=rail_left,
        negative_elem=hook_block,
        name="hook_block_hangs_below_forward_jib",
    )
    with ctx.pose({trolley_travel: 0.80}):
        ctx.expect_overlap(trolley, forward_jib, axes="x", min_overlap=0.05, elem_a=saddle_left, elem_b=rail_left)
        ctx.expect_gap(
            forward_jib,
            trolley,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=rail_left,
            negative_elem=saddle_left,
        )
        ctx.expect_gap(
            forward_jib,
            trolley,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=rail_right,
            negative_elem=saddle_right,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
