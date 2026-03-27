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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

TRAVEL = 0.060

SLEEVE_OUTER_RADIUS = 0.0158
SLEEVE_INNER_RADIUS = 0.0143
SLEEVE_LENGTH = 0.220

INNER_TUBE_RADIUS = 0.0136
INNER_TUBE_LENGTH = 0.185
INNER_TUBE_CENTER_Z = -0.0675
INNER_TUBE_TOP_Z = INNER_TUBE_CENTER_Z + (INNER_TUBE_LENGTH / 2.0)

SEAL_HEAD_OUTER_RADIUS = 0.0168
SEAL_HEAD_INNER_RADIUS = 0.0152
SEAL_HEAD_LENGTH = 0.012

GUIDE_PAD_THICKNESS = 0.0020
GUIDE_PAD_WIDTH = 0.0060
GUIDE_PAD_HEIGHT = 0.0080
GUIDE_PAD_CENTER_RADIUS = INNER_TUBE_RADIUS + (GUIDE_PAD_THICKNESS / 2.0)
GUIDE_PAD_CENTER_Z = SLEEVE_LENGTH + (SEAL_HEAD_LENGTH / 2.0)

CLAMP_BASE_SIZE = (0.026, 0.040, 0.010)
RAIL_SADDLE_RADIUS = 0.0032
RAIL_SADDLE_LENGTH = 0.032
RAIL_SADDLE_OFFSET_Y = 0.014
UPPER_CLAMP_SIZE = (0.010, 0.044, 0.006)
UPPER_CLAMP_OFFSET_X = 0.0125
UPPER_CLAMP_CENTER_Z = 0.0162
CLAMP_BOLT_RADIUS = 0.0030
CLAMP_BOLT_LENGTH = 0.052
CLAMP_BOLT_CENTER_Z = -0.005
CLAMP_BOLT_HEAD_RADIUS = 0.0045
CLAMP_BOLT_HEAD_LENGTH = 0.004
CLAMP_BOLT_HEAD_Y = 0.026


def _tube_shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
):
    outer = CylinderGeometry(radius=outer_radius, height=length, radial_segments=56)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=length + 0.004,
        radial_segments=56,
    )
    geom = boolean_difference(outer, inner).translate(0.0, 0.0, length / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path(f"{name}.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_travel_dropper_seatpost", assets=ASSETS)

    black = model.material("black_anodized", rgba=(0.11, 0.11, 0.12, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.19, 0.20, 0.22, 1.0))
    hardware = model.material("hardware_silver", rgba=(0.74, 0.76, 0.78, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        _tube_shell_mesh(
            "outer_sleeve_body",
            outer_radius=SLEEVE_OUTER_RADIUS,
            inner_radius=SLEEVE_INNER_RADIUS,
            length=SLEEVE_LENGTH,
        ),
        material=black,
        name="sleeve_body",
    )
    outer_sleeve.visual(
        _tube_shell_mesh(
            "outer_sleeve_head",
            outer_radius=SEAL_HEAD_OUTER_RADIUS,
            inner_radius=SEAL_HEAD_INNER_RADIUS,
            length=SEAL_HEAD_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_LENGTH)),
        material=dark_metal,
        name="seal_head",
    )
    outer_sleeve.visual(
        Box((GUIDE_PAD_THICKNESS, GUIDE_PAD_WIDTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(GUIDE_PAD_CENTER_RADIUS, 0.0, GUIDE_PAD_CENTER_Z)),
        material=dark_metal,
        name="guide_pad_right",
    )
    outer_sleeve.visual(
        Box((GUIDE_PAD_THICKNESS, GUIDE_PAD_WIDTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_PAD_CENTER_RADIUS, 0.0, GUIDE_PAD_CENTER_Z)),
        material=dark_metal,
        name="guide_pad_left",
    )
    outer_sleeve.visual(
        Box((GUIDE_PAD_WIDTH, GUIDE_PAD_THICKNESS, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, GUIDE_PAD_CENTER_RADIUS, GUIDE_PAD_CENTER_Z)),
        material=dark_metal,
        name="guide_pad_front",
    )
    outer_sleeve.visual(
        Box((GUIDE_PAD_WIDTH, GUIDE_PAD_THICKNESS, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -GUIDE_PAD_CENTER_RADIUS, GUIDE_PAD_CENTER_Z)),
        material=dark_metal,
        name="guide_pad_rear",
    )
    outer_sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=SEAL_HEAD_OUTER_RADIUS, length=SLEEVE_LENGTH + SEAL_HEAD_LENGTH),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, (SLEEVE_LENGTH + SEAL_HEAD_LENGTH) / 2.0)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=INNER_TUBE_RADIUS, length=INNER_TUBE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, INNER_TUBE_CENTER_Z)),
        material=dark_metal,
        name="inner_tube",
    )
    inner_post.inertial = Inertial.from_geometry(
        Cylinder(radius=INNER_TUBE_RADIUS, length=INNER_TUBE_LENGTH),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, INNER_TUBE_CENTER_Z)),
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, SLEEVE_LENGTH + SEAL_HEAD_LENGTH)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.25,
            lower=0.0,
            upper=TRAVEL,
        ),
    )

    lower_clamp = model.part("lower_clamp")
    lower_clamp.visual(
        Box(CLAMP_BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, CLAMP_BASE_SIZE[2] / 2.0)),
        material=black,
        name="clamp_base",
    )
    lower_clamp.visual(
        Cylinder(radius=RAIL_SADDLE_RADIUS, length=RAIL_SADDLE_LENGTH),
        origin=Origin(
            xyz=(0.0, RAIL_SADDLE_OFFSET_Y, CLAMP_BASE_SIZE[2]),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black,
        name="left_rail_saddle",
    )
    lower_clamp.visual(
        Cylinder(radius=RAIL_SADDLE_RADIUS, length=RAIL_SADDLE_LENGTH),
        origin=Origin(
            xyz=(0.0, -RAIL_SADDLE_OFFSET_Y, CLAMP_BASE_SIZE[2]),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=black,
        name="right_rail_saddle",
    )
    lower_clamp.inertial = Inertial.from_geometry(
        Box((0.030, 0.044, 0.014)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    model.articulation(
        "inner_to_lower_clamp",
        ArticulationType.FIXED,
        parent=inner_post,
        child=lower_clamp,
        origin=Origin(xyz=(0.0, 0.0, INNER_TUBE_TOP_Z)),
    )

    front_clamp = model.part("front_clamp")
    front_clamp.visual(
        Box(UPPER_CLAMP_SIZE),
        material=black,
        name="front_upper_plate",
    )
    front_clamp.visual(
        Cylinder(radius=CLAMP_BOLT_RADIUS, length=CLAMP_BOLT_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, CLAMP_BOLT_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="front_bolt",
    )
    front_clamp.visual(
        Cylinder(radius=CLAMP_BOLT_HEAD_RADIUS, length=CLAMP_BOLT_HEAD_LENGTH),
        origin=Origin(
            xyz=(0.0, CLAMP_BOLT_HEAD_Y, CLAMP_BOLT_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="front_bolt_head_right",
    )
    front_clamp.visual(
        Cylinder(radius=CLAMP_BOLT_HEAD_RADIUS, length=CLAMP_BOLT_HEAD_LENGTH),
        origin=Origin(
            xyz=(0.0, -CLAMP_BOLT_HEAD_Y, CLAMP_BOLT_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="front_bolt_head_left",
    )
    front_clamp.inertial = Inertial.from_geometry(
        Box((0.012, 0.056, 0.012)),
        mass=0.05,
        origin=Origin(),
    )

    model.articulation(
        "lower_to_front_clamp",
        ArticulationType.FIXED,
        parent=lower_clamp,
        child=front_clamp,
        origin=Origin(xyz=(-UPPER_CLAMP_OFFSET_X, 0.0, UPPER_CLAMP_CENTER_Z)),
    )

    rear_clamp = model.part("rear_clamp")
    rear_clamp.visual(
        Box(UPPER_CLAMP_SIZE),
        material=black,
        name="rear_upper_plate",
    )
    rear_clamp.visual(
        Cylinder(radius=CLAMP_BOLT_RADIUS, length=CLAMP_BOLT_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, CLAMP_BOLT_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="rear_bolt",
    )
    rear_clamp.visual(
        Cylinder(radius=CLAMP_BOLT_HEAD_RADIUS, length=CLAMP_BOLT_HEAD_LENGTH),
        origin=Origin(
            xyz=(0.0, CLAMP_BOLT_HEAD_Y, CLAMP_BOLT_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="rear_bolt_head_right",
    )
    rear_clamp.visual(
        Cylinder(radius=CLAMP_BOLT_HEAD_RADIUS, length=CLAMP_BOLT_HEAD_LENGTH),
        origin=Origin(
            xyz=(0.0, -CLAMP_BOLT_HEAD_Y, CLAMP_BOLT_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=hardware,
        name="rear_bolt_head_left",
    )
    rear_clamp.inertial = Inertial.from_geometry(
        Box((0.012, 0.056, 0.012)),
        mass=0.05,
        origin=Origin(),
    )

    model.articulation(
        "lower_to_rear_clamp",
        ArticulationType.FIXED,
        parent=lower_clamp,
        child=rear_clamp,
        origin=Origin(xyz=(UPPER_CLAMP_OFFSET_X, 0.0, UPPER_CLAMP_CENTER_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    outer_sleeve = object_model.get_part("outer_sleeve")
    inner_post = object_model.get_part("inner_post")
    lower_clamp = object_model.get_part("lower_clamp")
    front_clamp = object_model.get_part("front_clamp")
    rear_clamp = object_model.get_part("rear_clamp")

    outer_to_inner = object_model.get_articulation("outer_to_inner")

    sleeve_body = outer_sleeve.get_visual("sleeve_body")
    guide_pad_right = outer_sleeve.get_visual("guide_pad_right")
    seal_head = outer_sleeve.get_visual("seal_head")
    inner_tube = inner_post.get_visual("inner_tube")
    clamp_base = lower_clamp.get_visual("clamp_base")
    left_rail_saddle = lower_clamp.get_visual("left_rail_saddle")
    right_rail_saddle = lower_clamp.get_visual("right_rail_saddle")
    front_upper_plate = front_clamp.get_visual("front_upper_plate")
    rear_upper_plate = rear_clamp.get_visual("rear_upper_plate")

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
    ctx.expect_within(
        inner_post,
        outer_sleeve,
        axes="xy",
        inner_elem=inner_tube,
        outer_elem=sleeve_body,
    )
    ctx.expect_contact(
        inner_post,
        outer_sleeve,
        elem_a=inner_tube,
        elem_b=guide_pad_right,
    )
    ctx.expect_gap(
        lower_clamp,
        inner_post,
        axis="z",
        positive_elem=clamp_base,
        negative_elem=inner_tube,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_overlap(
        lower_clamp,
        inner_post,
        axes="xy",
        min_overlap=0.0003,
        elem_a=clamp_base,
        elem_b=inner_tube,
    )
    ctx.expect_gap(
        front_clamp,
        lower_clamp,
        axis="z",
        positive_elem=front_upper_plate,
        negative_elem=left_rail_saddle,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        rear_clamp,
        lower_clamp,
        axis="z",
        positive_elem=rear_upper_plate,
        negative_elem=right_rail_saddle,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_overlap(
        front_clamp,
        lower_clamp,
        axes="xy",
        min_overlap=0.0001,
        elem_a=front_upper_plate,
        elem_b=clamp_base,
    )
    ctx.expect_overlap(
        rear_clamp,
        lower_clamp,
        axes="xy",
        min_overlap=0.0001,
        elem_a=rear_upper_plate,
        elem_b=clamp_base,
    )
    ctx.expect_overlap(
        front_clamp,
        lower_clamp,
        axes="xy",
        min_overlap=0.006,
        elem_a=front_upper_plate,
        elem_b=left_rail_saddle,
    )
    ctx.expect_overlap(
        front_clamp,
        lower_clamp,
        axes="xy",
        min_overlap=0.006,
        elem_a=front_upper_plate,
        elem_b=right_rail_saddle,
    )
    ctx.expect_overlap(
        rear_clamp,
        lower_clamp,
        axes="xy",
        min_overlap=0.006,
        elem_a=rear_upper_plate,
        elem_b=left_rail_saddle,
    )
    ctx.expect_overlap(
        rear_clamp,
        lower_clamp,
        axes="xy",
        min_overlap=0.006,
        elem_a=rear_upper_plate,
        elem_b=right_rail_saddle,
    )
    ctx.expect_origin_distance(front_clamp, rear_clamp, axes="yz", max_dist=0.002)
    ctx.expect_gap(
        rear_clamp,
        front_clamp,
        axis="x",
        min_gap=0.014,
        max_gap=0.016,
    )
    ctx.expect_gap(
        lower_clamp,
        outer_sleeve,
        axis="z",
        positive_elem=clamp_base,
        negative_elem=seal_head,
        min_gap=0.024,
        max_gap=0.026,
    )
    with ctx.pose({outer_to_inner: TRAVEL}):
        ctx.expect_within(
            inner_post,
            outer_sleeve,
            axes="xy",
            inner_elem=inner_tube,
            outer_elem=sleeve_body,
        )
        ctx.expect_contact(
            inner_post,
            outer_sleeve,
            elem_a=inner_tube,
            elem_b=guide_pad_right,
        )
        ctx.expect_gap(
            lower_clamp,
            outer_sleeve,
            axis="z",
            positive_elem=clamp_base,
            negative_elem=seal_head,
            min_gap=0.084,
            max_gap=0.086,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
