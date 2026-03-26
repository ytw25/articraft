from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="h_frame_display_easel", assets=ASSETS)

    oak = model.material("oak", rgba=(0.70, 0.56, 0.39, 1.0))
    walnut = model.material("walnut", rgba=(0.35, 0.24, 0.16, 1.0))
    iron = model.material("iron", rgba=(0.20, 0.20, 0.22, 1.0))

    front_frame = model.part("front_frame")
    front_frame.visual(
        Box((0.07, 0.032, 1.52)),
        origin=Origin(xyz=(-0.19, 0.0, 0.76)),
        material=oak,
        name="left_stile",
    )
    front_frame.visual(
        Box((0.07, 0.032, 1.52)),
        origin=Origin(xyz=(0.19, 0.0, 0.76)),
        material=oak,
        name="right_stile",
    )
    front_frame.visual(
        Box((0.45, 0.028, 0.06)),
        origin=Origin(xyz=(0.0, -0.002, 1.47)),
        material=oak,
        name="top_rail",
    )
    front_frame.visual(
        Box((0.45, 0.032, 0.08)),
        origin=Origin(xyz=(0.0, -0.002, 0.18)),
        material=oak,
        name="bottom_crossbar",
    )
    front_frame.visual(
        Box((0.05, 0.02, 1.22)),
        origin=Origin(xyz=(0.0, -0.018, 0.83)),
        material=walnut,
        name="center_mast",
    )
    front_frame.visual(
        Box((0.06, 0.02, 0.05)),
        origin=Origin(xyz=(-0.19, -0.026, 1.47)),
        material=iron,
        name="left_pivot_block",
    )
    front_frame.visual(
        Box((0.06, 0.02, 0.05)),
        origin=Origin(xyz=(0.19, -0.026, 1.47)),
        material=iron,
        name="right_pivot_block",
    )
    front_frame.visual(
        Cylinder(radius=0.008, length=0.44),
        origin=Origin(xyz=(0.0, -0.036, 1.47), rpy=(0.0, 1.5708, 0.0)),
        material=iron,
        name="pivot_pin",
    )
    front_frame.visual(
        Box((0.12, 0.034, 0.05)),
        origin=Origin(xyz=(0.0, -0.032, 0.98)),
        material=iron,
        name="upper_backbar_anchor",
    )
    front_frame.visual(
        Box((0.12, 0.034, 0.05)),
        origin=Origin(xyz=(0.0, -0.032, 0.60)),
        material=iron,
        name="lower_backbar_anchor",
    )

    rear_support = model.part("rear_support")
    rear_support.visual(
        Box((0.06, 0.02, 0.05)),
        origin=Origin(xyz=(-0.19, -0.01, 0.0)),
        material=iron,
        name="left_pivot_ear",
    )
    rear_support.visual(
        Box((0.06, 0.02, 0.05)),
        origin=Origin(xyz=(0.19, -0.01, 0.0)),
        material=iron,
        name="right_pivot_ear",
    )
    rear_support.visual(
        Box((0.38, 0.04, 0.04)),
        origin=Origin(xyz=(0.0, -0.02, 0.0)),
        material=walnut,
        name="rear_top_yoke",
    )
    rear_support.visual(
        Box((0.08, 0.032, 1.28)),
        origin=Origin(xyz=(0.0, -0.291, -0.604), rpy=(-0.42, 0.0, 0.0)),
        material=oak,
        name="rear_stile",
    )
    rear_support.visual(
        Box((0.11, 0.273, 0.026)),
        origin=Origin(xyz=(0.0, -0.1495, -0.49)),
        material=walnut,
        name="upper_backbar",
    )
    rear_support.visual(
        Box((0.11, 0.427, 0.04)),
        origin=Origin(xyz=(0.0, -0.2265, -0.85)),
        material=walnut,
        name="lower_backbar",
    )

    ledge = model.part("display_ledge")
    ledge.visual(
        Box((0.43, 0.012, 0.02)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=iron,
        name="ledge_hinge_leaf",
    )
    ledge.visual(
        Box((0.42, 0.16, 0.018)),
        origin=Origin(xyz=(0.0, 0.092, 0.0)),
        material=oak,
        name="ledge_board",
    )
    ledge.visual(
        Box((0.42, 0.018, 0.035)),
        origin=Origin(xyz=(0.0, 0.181, 0.008)),
        material=walnut,
        name="ledge_lip",
    )

    hold_down = model.part("hold_down_bar")
    hold_down.visual(
        Box((0.34, 0.024, 0.012)),
        origin=Origin(xyz=(0.0, -0.001, 0.010)),
        material=iron,
        name="clamp_saddle",
    )
    hold_down.visual(
        Box((0.34, 0.012, 0.11)),
        origin=Origin(xyz=(0.0, 0.024, -0.074)),
        material=iron,
        name="clamp_bar",
    )
    hold_down.visual(
        Box((0.03, 0.018, 0.034)),
        origin=Origin(xyz=(-0.152, 0.012, -0.010)),
        material=iron,
        name="left_hook_cheek",
    )
    hold_down.visual(
        Box((0.03, 0.018, 0.034)),
        origin=Origin(xyz=(0.152, 0.012, -0.010)),
        material=iron,
        name="right_hook_cheek",
    )

    model.articulation(
        "rear_support_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_support,
        origin=Origin(xyz=(0.0, -0.036, 1.47)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=0.10),
    )
    model.articulation(
        "ledge_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=ledge,
        origin=Origin(xyz=(0.0, 0.014, 0.17)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=0.0, upper=1.10),
    )
    model.articulation(
        "hold_down_hinge",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=hold_down,
        origin=Origin(xyz=(0.0, 0.012, 1.496)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_support = object_model.get_part("rear_support")
    ledge = object_model.get_part("display_ledge")
    hold_down = object_model.get_part("hold_down_bar")

    rear_support_hinge = object_model.get_articulation("rear_support_hinge")
    ledge_hinge = object_model.get_articulation("ledge_hinge")
    hold_down_hinge = object_model.get_articulation("hold_down_hinge")

    left_stile = front_frame.get_visual("left_stile")
    right_stile = front_frame.get_visual("right_stile")
    top_rail = front_frame.get_visual("top_rail")
    bottom_crossbar = front_frame.get_visual("bottom_crossbar")
    center_mast = front_frame.get_visual("center_mast")
    left_pivot_block = front_frame.get_visual("left_pivot_block")
    right_pivot_block = front_frame.get_visual("right_pivot_block")
    pivot_pin = front_frame.get_visual("pivot_pin")
    upper_anchor = front_frame.get_visual("upper_backbar_anchor")
    lower_anchor = front_frame.get_visual("lower_backbar_anchor")

    left_pivot_ear = rear_support.get_visual("left_pivot_ear")
    right_pivot_ear = rear_support.get_visual("right_pivot_ear")
    rear_top_yoke = rear_support.get_visual("rear_top_yoke")
    rear_stile = rear_support.get_visual("rear_stile")
    upper_backbar = rear_support.get_visual("upper_backbar")
    lower_backbar = rear_support.get_visual("lower_backbar")

    ledge_hinge_leaf = ledge.get_visual("ledge_hinge_leaf")
    ledge_board = ledge.get_visual("ledge_board")
    ledge_lip = ledge.get_visual("ledge_lip")

    clamp_saddle = hold_down.get_visual("clamp_saddle")
    clamp_bar = hold_down.get_visual("clamp_bar")
    left_hook_cheek = hold_down.get_visual("left_hook_cheek")
    right_hook_cheek = hold_down.get_visual("right_hook_cheek")

    ctx.allow_overlap(
        rear_support,
        front_frame,
        reason="rear pivot ears and back-bar ends seat into the front anchor hardware around the shared top pin",
    )
    ctx.allow_overlap(
        ledge,
        front_frame,
        reason="the fold-out ledge hinge leaf nests directly against the front crossbar at the pivot",
    )
    ctx.allow_overlap(
        hold_down,
        front_frame,
        reason="the hold-down clamp cap wraps over the top rail while pivoting on that rail",
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

    ctx.expect_gap(
        front_frame,
        front_frame,
        axis="x",
        positive_elem=right_stile,
        negative_elem=left_stile,
        min_gap=0.30,
        name="front stiles stand as a wide pair",
    )
    ctx.expect_contact(front_frame, front_frame, elem_a=top_rail, elem_b=left_stile)
    ctx.expect_contact(front_frame, front_frame, elem_a=top_rail, elem_b=right_stile)
    ctx.expect_contact(front_frame, front_frame, elem_a=bottom_crossbar, elem_b=left_stile)
    ctx.expect_contact(front_frame, front_frame, elem_a=bottom_crossbar, elem_b=right_stile)

    ctx.expect_contact(rear_support, front_frame, elem_a=left_pivot_ear, elem_b=left_pivot_block)
    ctx.expect_contact(rear_support, front_frame, elem_a=right_pivot_ear, elem_b=right_pivot_block)
    ctx.expect_contact(rear_support, front_frame, elem_a=rear_top_yoke, elem_b=pivot_pin)
    ctx.expect_contact(rear_support, front_frame, elem_a=upper_backbar, elem_b=upper_anchor)
    ctx.expect_contact(rear_support, front_frame, elem_a=lower_backbar, elem_b=lower_anchor)
    ctx.expect_gap(
        front_frame,
        rear_support,
        axis="y",
        positive_elem=center_mast,
        negative_elem=rear_stile,
        min_gap=0.01,
        name="rear stile sits behind the front frame",
    )

    ctx.expect_contact(ledge, front_frame, elem_a=ledge_hinge_leaf, elem_b=bottom_crossbar)
    ctx.expect_overlap(ledge, front_frame, axes="x", min_overlap=0.38, elem_a=ledge_board, elem_b=bottom_crossbar)
    ctx.expect_gap(
        ledge,
        front_frame,
        axis="y",
        positive_elem=ledge_lip,
        negative_elem=bottom_crossbar,
        min_gap=0.15,
        name="display ledge projects forward from the bottom rail",
    )

    ctx.expect_contact(hold_down, front_frame, elem_a=clamp_saddle, elem_b=top_rail)
    ctx.expect_within(hold_down, front_frame, axes="x", inner_elem=clamp_bar, outer_elem=top_rail)
    ctx.expect_gap(
        hold_down,
        front_frame,
        axis="y",
        positive_elem=left_hook_cheek,
        negative_elem=top_rail,
        max_gap=0.006,
        max_penetration=0.001,
        name="left clamp cheek hooks over the top rail",
    )
    ctx.expect_gap(
        hold_down,
        front_frame,
        axis="y",
        positive_elem=right_hook_cheek,
        negative_elem=top_rail,
        max_gap=0.006,
        max_penetration=0.001,
        name="right clamp cheek hooks over the top rail",
    )
    ctx.expect_overlap(hold_down, front_frame, axes="x", min_overlap=0.28, elem_a=clamp_bar, elem_b=top_rail)
    ctx.expect_gap(
        hold_down,
        front_frame,
        axis="y",
        positive_elem=clamp_bar,
        negative_elem=top_rail,
        max_gap=0.02,
        max_penetration=0.0,
        name="clamp bar seats against the front of the top rail",
    )

    with ctx.pose({ledge_hinge: 1.05}):
        ctx.expect_gap(
            ledge,
            front_frame,
            axis="z",
            positive_elem=ledge_lip,
            negative_elem=bottom_crossbar,
            min_gap=0.03,
            name="ledge lip lifts above the lower rail when folded up",
        )
        ctx.expect_overlap(ledge, front_frame, axes="x", min_overlap=0.38, elem_a=ledge_board, elem_b=bottom_crossbar)

    with ctx.pose({hold_down_hinge: 0.55}):
        ctx.expect_gap(
            hold_down,
            front_frame,
            axis="y",
            positive_elem=clamp_bar,
            negative_elem=top_rail,
            min_gap=0.020,
            name="hold-down bar swings clear to release a canvas",
        )

    with ctx.pose({rear_support_hinge: 0.08}):
        ctx.expect_contact(rear_support, front_frame, elem_a=left_pivot_ear, elem_b=left_pivot_block)
        ctx.expect_contact(rear_support, front_frame, elem_a=right_pivot_ear, elem_b=right_pivot_block)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
