from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

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

SCRIPT_DIR = os.path.dirname(__file__) or "/tmp"
LUFF_LOWER = -0.42
LUFF_UPPER = 0.38
try:
    os.getcwd()
except FileNotFoundError:
    try:
        os.chdir(SCRIPT_DIR)
    except FileNotFoundError:
        os.chdir("/tmp")


def _box_beam(part, start, end, thickness, material, name):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (start[2] + end[2]) / 2.0)
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(xyz=mid, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _cylinder_link(part, start, end, radius, material, name):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    mid = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0, (start[2] + end[2]) / 2.0)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=mid, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.93, 0.78, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.42, 0.44, 0.47, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.26, 1.0))
    cable_black = model.material("cable_black", rgba=(0.08, 0.08, 0.09, 1.0))

    ground_frame = model.part("ground_frame")
    ground_frame.visual(
        Box((1.7, 0.16, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=steel,
        name="arm_x",
    )
    ground_frame.visual(
        Box((0.16, 1.7, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=steel,
        name="arm_y",
    )
    for name, xyz in (
        ("pad_east", (0.70, 0.0, 0.025)),
        ("pad_west", (-0.70, 0.0, 0.025)),
        ("pad_north", (0.0, 0.70, 0.025)),
        ("pad_south", (0.0, -0.70, 0.025)),
    ):
        ground_frame.visual(
            Box((0.26, 0.26, 0.05)),
            origin=Origin(xyz=xyz),
            material=dark_steel,
            name=name,
        )
    ground_frame.visual(
        Box((0.22, 0.22, 0.32)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=steel,
        name="mast_pedestal",
    )
    for name, start, end in (
        ("leg_east", (0.48, 0.0, 0.08), (0.08, 0.0, 0.32)),
        ("leg_west", (-0.48, 0.0, 0.08), (-0.08, 0.0, 0.32)),
        ("leg_north", (0.0, 0.48, 0.08), (0.0, 0.08, 0.32)),
        ("leg_south", (0.0, -0.48, 0.08), (0.0, -0.08, 0.32)),
    ):
        _box_beam(ground_frame, start, end, 0.07, steel, name)
    ground_frame.inertial = Inertial.from_geometry(
        Box((1.7, 1.7, 0.32)),
        mass=90.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.24, 0.24, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=steel,
        name="mast_base",
    )
    corner_points = {
        "fl": (0.11, 0.11),
        "fr": (0.11, -0.11),
        "rl": (-0.11, 0.11),
        "rr": (-0.11, -0.11),
    }
    for name, (x, y) in corner_points.items():
        mast.visual(
            Box((0.04, 0.04, 1.72)),
            origin=Origin(xyz=(x, y, 0.86)),
            material=crane_yellow,
            name=f"post_{name}",
        )
    mast_levels = [0.06, 0.46, 0.86, 1.26, 1.72]
    for idx, z in enumerate(mast_levels):
        mast.visual(
            Box((0.22, 0.02, 0.02)),
            origin=Origin(xyz=(0.0, 0.11, z)),
            material=crane_yellow,
            name=f"ring_front_{idx}",
        )
        mast.visual(
            Box((0.22, 0.02, 0.02)),
            origin=Origin(xyz=(0.0, -0.11, z)),
            material=crane_yellow,
            name=f"ring_rear_{idx}",
        )
        mast.visual(
            Box((0.02, 0.22, 0.02)),
            origin=Origin(xyz=(0.11, 0.0, z)),
            material=crane_yellow,
            name=f"ring_right_{idx}",
        )
        mast.visual(
            Box((0.02, 0.22, 0.02)),
            origin=Origin(xyz=(-0.11, 0.0, z)),
            material=crane_yellow,
            name=f"ring_left_{idx}",
        )
    mast_bays = list(zip(mast_levels[:-1], mast_levels[1:]))
    for bay_idx, (z0, z1) in enumerate(mast_bays):
        _box_beam(mast, (0.11, 0.11, z0), (0.11, -0.11, z1), 0.015, crane_yellow, f"brace_front_a_{bay_idx}")
        _box_beam(mast, (0.11, -0.11, z0), (0.11, 0.11, z1), 0.015, crane_yellow, f"brace_front_b_{bay_idx}")
        _box_beam(mast, (-0.11, 0.11, z0), (-0.11, -0.11, z1), 0.015, crane_yellow, f"brace_rear_a_{bay_idx}")
        _box_beam(mast, (-0.11, -0.11, z0), (-0.11, 0.11, z1), 0.015, crane_yellow, f"brace_rear_b_{bay_idx}")
        _box_beam(mast, (0.11, 0.11, z0), (-0.11, 0.11, z1), 0.015, crane_yellow, f"brace_left_a_{bay_idx}")
        _box_beam(mast, (-0.11, 0.11, z0), (0.11, 0.11, z1), 0.015, crane_yellow, f"brace_left_b_{bay_idx}")
        _box_beam(mast, (0.11, -0.11, z0), (-0.11, -0.11, z1), 0.015, crane_yellow, f"brace_right_a_{bay_idx}")
        _box_beam(mast, (-0.11, -0.11, z0), (0.11, -0.11, z1), 0.015, crane_yellow, f"brace_right_b_{bay_idx}")
    mast.visual(
        Box((0.34, 0.34, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.74)),
        material=steel,
        name="mast_cap",
    )
    mast.visual(
        Cylinder(radius=0.19, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 1.78)),
        material=dark_steel,
        name="lower_slew_ring",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 1.80)),
        mass=65.0,
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
    )

    superstructure = model.part("superstructure")
    superstructure.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_steel,
        name="upper_slew_ring",
    )
    superstructure.visual(
        Box((0.74, 0.26, 0.06)),
        origin=Origin(xyz=(0.00, 0.0, 0.07)),
        material=crane_yellow,
        name="deck",
    )
    superstructure.visual(
        Box((0.24, 0.18, 0.16)),
        origin=Origin(xyz=(-0.06, 0.0, 0.18)),
        material=crane_yellow,
        name="machinery_house",
    )
    superstructure.visual(
        Box((0.26, 0.22, 0.16)),
        origin=Origin(xyz=(-0.30, 0.0, 0.18)),
        material=dark_steel,
        name="counterweight",
    )
    superstructure.visual(
        Box((0.08, 0.03, 0.16)),
        origin=Origin(xyz=(0.30, 0.075, 0.20)),
        material=steel,
        name="pivot_cheek_left",
    )
    superstructure.visual(
        Box((0.08, 0.03, 0.16)),
        origin=Origin(xyz=(0.30, -0.075, 0.20)),
        material=steel,
        name="pivot_cheek_right",
    )
    superstructure.visual(
        Box((0.12, 0.18, 0.18)),
        origin=Origin(xyz=(0.27, 0.0, 0.19)),
        material=steel,
        name="pivot_pedestal",
    )
    _cylinder_link(superstructure, (0.30, -0.08, 0.29), (0.30, 0.08, 0.29), 0.016, steel, "pivot_pin")
    superstructure.visual(
        Box((0.04, 0.14, 0.03)),
        origin=Origin(xyz=(0.30, 0.0, 0.29)),
        material=steel,
        name="pivot_head",
    )
    _box_beam(superstructure, (-0.08, 0.09, 0.10), (-0.30, 0.04, 0.60), 0.045, crane_yellow, "a_frame_leg_left")
    _box_beam(superstructure, (-0.08, -0.09, 0.10), (-0.30, -0.04, 0.60), 0.045, crane_yellow, "a_frame_leg_right")
    _box_beam(superstructure, (-0.16, 0.0, 0.10), (-0.30, 0.0, 0.60), 0.035, crane_yellow, "a_frame_rear_stay")
    _cylinder_link(superstructure, (-0.30, -0.04, 0.60), (-0.30, 0.04, 0.60), 0.018, steel, "a_frame_tip")
    _cylinder_link(superstructure, (-0.30, 0.0, 0.60), (0.50, 0.0, 0.46), 0.006, cable_black, "pendant_line")
    superstructure.inertial = Inertial.from_geometry(
        Box((0.76, 0.30, 0.64)),
        mass=55.0,
        origin=Origin(xyz=(0.00, 0.0, 0.28)),
    )

    jib = model.part("jib")
    _cylinder_link(jib, (0.0, -0.045, 0.0), (0.0, 0.045, 0.0), 0.024, steel, "jib_root_sleeve")
    _box_beam(jib, (0.0, 0.0, 0.0), (0.16, -0.06, 0.02), 0.035, steel, "root_gusset_right")
    _box_beam(jib, (0.0, 0.0, 0.0), (0.16, 0.06, 0.02), 0.035, steel, "root_gusset_left")
    _box_beam(jib, (0.0, 0.0, 0.0), (0.18, 0.0, 0.16), 0.035, steel, "root_gusset_top")
    jib.visual(
        Box((0.06, 0.06, 0.04)),
        origin=Origin(xyz=(0.20, 0.0, 0.17)),
        material=steel,
        name="pendant_pickup",
    )
    _box_beam(jib, (0.16, -0.06, 0.02), (2.10, -0.06, 0.82), 0.024, crane_yellow, "lower_chord_right")
    _box_beam(jib, (0.16, 0.06, 0.02), (2.10, 0.06, 0.82), 0.024, crane_yellow, "lower_chord_left")
    _box_beam(jib, (0.18, 0.0, 0.16), (2.08, 0.0, 0.98), 0.024, crane_yellow, "upper_chord")
    stations = [0.26, 0.56, 0.86, 1.16, 1.46, 1.76, 2.06]
    for idx, x in enumerate(stations):
        z_low = 0.02 + ((x - 0.16) / (2.10 - 0.16)) * (0.82 - 0.02)
        z_top = 0.16 + ((x - 0.18) / (2.08 - 0.18)) * (0.98 - 0.16)
        _box_beam(jib, (x, -0.06, z_low), (x, 0.06, z_low), 0.018, crane_yellow, f"tie_{idx}")
        _box_beam(jib, (x, -0.06, z_low), (x, 0.0, z_top), 0.018, crane_yellow, f"web_right_{idx}")
        _box_beam(jib, (x, 0.06, z_low), (x, 0.0, z_top), 0.018, crane_yellow, f"web_left_{idx}")
    for idx, (x0, x1) in enumerate(zip(stations[:-1], stations[1:])):
        z0_low = 0.02 + ((x0 - 0.16) / (2.10 - 0.16)) * (0.82 - 0.02)
        z1_low = 0.02 + ((x1 - 0.16) / (2.10 - 0.16)) * (0.82 - 0.02)
        z0_top = 0.16 + ((x0 - 0.18) / (2.08 - 0.18)) * (0.98 - 0.16)
        z1_top = 0.16 + ((x1 - 0.18) / (2.08 - 0.18)) * (0.98 - 0.16)
        _box_beam(jib, (x0, -0.06, z0_low), (x1, 0.0, z1_top), 0.015, crane_yellow, f"diag_right_a_{idx}")
        _box_beam(jib, (x0, 0.06, z0_low), (x1, 0.0, z1_top), 0.015, crane_yellow, f"diag_left_a_{idx}")
        _box_beam(jib, (x0, 0.0, z0_top), (x1, -0.06, z1_low), 0.015, crane_yellow, f"diag_right_b_{idx}")
        _box_beam(jib, (x0, 0.0, z0_top), (x1, 0.06, z1_low), 0.015, crane_yellow, f"diag_left_b_{idx}")
    jib.visual(
        Box((0.12, 0.10, 0.16)),
        origin=Origin(xyz=(2.16, 0.0, 0.90)),
        material=steel,
        name="jib_tip_head",
    )
    _cylinder_link(jib, (2.16, -0.045, 0.84), (2.16, 0.045, 0.84), 0.012, steel, "hook_pin")
    jib.inertial = Inertial.from_geometry(
        Box((2.26, 0.24, 1.10)),
        mass=26.0,
        origin=Origin(xyz=(1.13, 0.0, 0.55)),
    )

    hook_block = model.part("hook_block")
    hook_block.visual(
        Box((0.05, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=steel,
        name="upper_shackle",
    )
    _cylinder_link(hook_block, (0.0, 0.0, -0.04), (0.0, 0.0, -0.48), 0.006, cable_black, "hook_hanger")
    hook_block.visual(
        Box((0.10, 0.06, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.54)),
        material=dark_steel,
        name="hook_body",
    )
    _cylinder_link(hook_block, (0.0, 0.0, -0.60), (0.04, 0.0, -0.68), 0.008, dark_steel, "hook_curve_upper")
    _cylinder_link(hook_block, (0.04, 0.0, -0.68), (0.06, 0.0, -0.77), 0.008, dark_steel, "hook_curve_mid")
    _cylinder_link(hook_block, (0.06, 0.0, -0.77), (-0.02, 0.0, -0.81), 0.008, dark_steel, "hook_curve_tip")
    hook_block.inertial = Inertial.from_geometry(
        Box((0.12, 0.08, 0.84)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, -0.40)),
    )

    model.articulation(
        "ground_frame_to_mast",
        ArticulationType.FIXED,
        parent=ground_frame,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
    )
    model.articulation(
        "mast_to_superstructure",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=superstructure,
        origin=Origin(xyz=(0.0, 0.0, 1.80)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.8),
    )
    model.articulation(
        "superstructure_to_jib",
        ArticulationType.REVOLUTE,
        parent=superstructure,
        child=jib,
        origin=Origin(xyz=(0.30, 0.0, 0.29)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.8, lower=LUFF_LOWER, upper=LUFF_UPPER),
    )
    model.articulation(
        "jib_to_hook_block",
        ArticulationType.FIXED,
        parent=jib,
        child=hook_block,
        origin=Origin(xyz=(2.16, 0.0, 0.84)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SCRIPT_DIR)
    ground_frame = object_model.get_part("ground_frame")
    mast = object_model.get_part("mast")
    superstructure = object_model.get_part("superstructure")
    jib = object_model.get_part("jib")
    hook_block = object_model.get_part("hook_block")
    slew = object_model.get_articulation("mast_to_superstructure")
    luff = object_model.get_articulation("superstructure_to_jib")
    pedestal = ground_frame.get_visual("mast_pedestal")
    mast_base = mast.get_visual("mast_base")
    lower_slew_ring = mast.get_visual("lower_slew_ring")
    upper_slew_ring = superstructure.get_visual("upper_slew_ring")
    pivot_pin = superstructure.get_visual("pivot_pin")
    a_frame_tip = superstructure.get_visual("a_frame_tip")
    pendant_line = superstructure.get_visual("pendant_line")
    jib_root_sleeve = jib.get_visual("jib_root_sleeve")
    pendant_pickup = jib.get_visual("pendant_pickup")
    jib_tip_head = jib.get_visual("jib_tip_head")
    hook_pin = jib.get_visual("hook_pin")
    upper_shackle = hook_block.get_visual("upper_shackle")
    hook_body = hook_block.get_visual("hook_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        jib,
        superstructure,
        reason="luffing jib sleeve and knuckle gussets nest around the superstructure pivot pin assembly",
    )

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
    ctx.expect_origin_distance(mast, ground_frame, axes="xy", max_dist=0.001)
    ctx.expect_contact(mast, ground_frame, elem_a=mast_base, elem_b=pedestal)
    ctx.expect_overlap(mast, ground_frame, axes="xy", elem_a=mast_base, elem_b=pedestal, min_overlap=0.04)

    ctx.expect_origin_distance(superstructure, mast, axes="xy", max_dist=0.001)
    ctx.expect_contact(superstructure, mast, elem_a=upper_slew_ring, elem_b=lower_slew_ring)
    ctx.expect_overlap(superstructure, mast, axes="xy", elem_a=upper_slew_ring, elem_b=lower_slew_ring, min_overlap=0.30)

    ctx.expect_contact(jib, superstructure, elem_a=jib_root_sleeve, elem_b=pivot_pin)
    ctx.expect_gap(superstructure, mast, axis="z", positive_elem=a_frame_tip, negative_elem=lower_slew_ring, min_gap=0.50)
    ctx.expect_contact(superstructure, superstructure, elem_a=pendant_line, elem_b=a_frame_tip)
    ctx.expect_contact(superstructure, jib, elem_a=pendant_line, elem_b=pendant_pickup)
    ctx.expect_gap(jib, jib, axis="x", positive_elem=pendant_pickup, negative_elem=jib_root_sleeve, min_gap=0.14)
    ctx.expect_gap(jib, jib, axis="z", positive_elem=pendant_pickup, negative_elem=jib_root_sleeve, min_gap=0.11)
    ctx.expect_gap(jib, superstructure, axis="z", positive_elem=jib_tip_head, negative_elem=upper_slew_ring, min_gap=1.00)
    ctx.expect_contact(hook_block, jib, elem_a=upper_shackle, elem_b=hook_pin)
    ctx.expect_gap(jib, hook_block, axis="z", positive_elem=jib_tip_head, negative_elem=hook_body, min_gap=0.18)

    with ctx.pose({slew: math.pi / 2.0}):
        ctx.expect_contact(superstructure, mast, elem_a=upper_slew_ring, elem_b=lower_slew_ring)
        ctx.expect_overlap(superstructure, mast, axes="xy", elem_a=upper_slew_ring, elem_b=lower_slew_ring, min_overlap=0.30)

    with ctx.pose({luff: -0.20}):
        ctx.expect_contact(jib, superstructure, elem_a=jib_root_sleeve, elem_b=pivot_pin)
        ctx.expect_gap(jib, superstructure, axis="z", positive_elem=jib_tip_head, negative_elem=upper_slew_ring, min_gap=1.40)
        ctx.expect_gap(jib, hook_block, axis="z", positive_elem=jib_tip_head, negative_elem=hook_body, min_gap=0.42)

    with ctx.pose({luff: 0.25}):
        ctx.expect_contact(jib, superstructure, elem_a=jib_root_sleeve, elem_b=pivot_pin)
        ctx.expect_gap(jib, superstructure, axis="z", positive_elem=jib_tip_head, negative_elem=upper_slew_ring, min_gap=0.49)
        ctx.expect_gap(jib, hook_block, axis="z", positive_elem=jib_tip_head, negative_elem=hook_body, min_gap=0.40)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
