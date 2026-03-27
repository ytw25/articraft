from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

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


_SCRIPT_PATH = Path(globals().get("__file__", "/tmp/model.py"))
_SCRIPT_ROOT = _SCRIPT_PATH.parent if _SCRIPT_PATH.is_absolute() else Path("/tmp")
ASSETS = AssetContext(_SCRIPT_ROOT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a3_flatbed_scanner", assets=ASSETS)

    body_color = model.material("body_shell", rgba=(0.86, 0.87, 0.88, 1.0))
    trim_color = model.material("trim_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    liner_color = model.material("lid_liner", rgba=(0.95, 0.95, 0.94, 1.0))
    glass_color = model.material("glass", rgba=(0.56, 0.70, 0.78, 0.35))
    metal_color = model.material("hinge_metal", rgba=(0.60, 0.61, 0.63, 1.0))
    rubber_color = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    lid = model.part("lid")
    left_guide = model.part("left_guide")
    right_guide = model.part("right_guide")
    usb_cover = model.part("usb_cover")

    body_w = 0.660
    body_d = 0.480
    body_h = 0.090
    foot_h = 0.012
    top_plane_z = foot_h + body_h

    base.visual(
        Box((body_w, body_d, body_h)),
        origin=Origin(xyz=(0.0, 0.0, foot_h + body_h / 2.0)),
        material=body_color,
        name="body_shell",
    )
    for x_sign, y_sign, name in (
        (-1.0, -1.0, "foot_fl"),
        (1.0, -1.0, "foot_fr"),
        (-1.0, 1.0, "foot_rl"),
        (1.0, 1.0, "foot_rr"),
    ):
        base.visual(
            Box((0.050, 0.050, foot_h)),
            origin=Origin(xyz=(x_sign * 0.275, y_sign * 0.185, foot_h / 2.0)),
            material=rubber_color,
            name=name,
        )

    base.visual(
        Box((0.516, 0.044, 0.006)),
        origin=Origin(xyz=(0.0, -0.198, top_plane_z + 0.003)),
        material=trim_color,
        name="top_frame_front",
    )
    base.visual(
        Box((0.516, 0.044, 0.006)),
        origin=Origin(xyz=(0.0, 0.198, top_plane_z + 0.003)),
        material=trim_color,
        name="top_frame_rear",
    )
    base.visual(
        Box((0.072, 0.392, 0.006)),
        origin=Origin(xyz=(-0.294, 0.0, top_plane_z + 0.003)),
        material=trim_color,
        name="top_frame_left",
    )
    base.visual(
        Box((0.072, 0.392, 0.006)),
        origin=Origin(xyz=(0.294, 0.0, top_plane_z + 0.003)),
        material=trim_color,
        name="top_frame_right",
    )
    base.visual(
        Box((0.500, 0.360, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, top_plane_z - 0.001)),
        material=glass_color,
        name="platen_glass",
    )
    base.visual(
        Box((0.628, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.235, top_plane_z + 0.003)),
        material=metal_color,
        name="base_hinge_leaf",
    )
    base.visual(
        Box((0.008, 0.026, 0.016)),
        origin=Origin(xyz=(-0.024, -0.222, top_plane_z + 0.008)),
        material=trim_color,
        name="keeper_left",
    )
    base.visual(
        Box((0.008, 0.026, 0.016)),
        origin=Origin(xyz=(0.024, -0.222, top_plane_z + 0.008)),
        material=trim_color,
        name="keeper_right",
    )
    base.visual(
        Box((0.056, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.214, top_plane_z + 0.008)),
        material=trim_color,
        name="keeper_stop",
    )
    base.visual(
        Box((0.044, 0.008, 0.028)),
        origin=Origin(xyz=(0.220, -0.236, 0.056)),
        material=trim_color,
        name="usb_bezel",
    )
    base.visual(
        Box((0.034, 0.006, 0.004)),
        origin=Origin(xyz=(0.220, -0.233, 0.067)),
        material=metal_color,
        name="usb_hinge_seat",
    )

    lid.visual(
        Box((0.646, 0.462, 0.004)),
        origin=Origin(xyz=(0.0, -0.231, 0.012)),
        material=body_color,
        name="top_skin",
    )
    lid.visual(
        Box((0.272, 0.010, 0.020)),
        origin=Origin(xyz=(-0.187, -0.463, 0.004)),
        material=body_color,
        name="front_wall_left",
    )
    lid.visual(
        Box((0.272, 0.010, 0.020)),
        origin=Origin(xyz=(0.187, -0.463, 0.004)),
        material=body_color,
        name="front_wall_right",
    )
    lid.visual(
        Box((0.060, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.463, 0.010)),
        material=body_color,
        name="front_bridge",
    )
    lid.visual(
        Box((0.646, 0.008, 0.020)),
        origin=Origin(xyz=(0.0, -0.004, 0.004)),
        material=body_color,
        name="rear_wall",
    )
    lid.visual(
        Box((0.010, 0.460, 0.020)),
        origin=Origin(xyz=(-0.318, -0.230, 0.004)),
        material=body_color,
        name="left_wall",
    )
    lid.visual(
        Box((0.010, 0.460, 0.020)),
        origin=Origin(xyz=(0.318, -0.230, 0.004)),
        material=body_color,
        name="right_wall",
    )
    lid.visual(
        Box((0.620, 0.170, 0.004)),
        origin=Origin(xyz=(0.0, -0.326, 0.008)),
        material=liner_color,
        name="front_inner_band",
    )
    lid.visual(
        Box((0.620, 0.130, 0.004)),
        origin=Origin(xyz=(0.0, -0.145, 0.008)),
        material=liner_color,
        name="rear_inner_band",
    )
    lid.visual(
        Box((0.560, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.239, 0.008)),
        material=trim_color,
        name="slot_front_rail",
    )
    lid.visual(
        Box((0.560, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -0.212, 0.008)),
        material=trim_color,
        name="slot_rear_rail",
    )
    lid.visual(
        Box((0.040, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, -0.466, 0.000)),
        material=trim_color,
        name="latch_tongue",
    )
    lid.visual(
        Box((0.634, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.001, -0.004)),
        material=metal_color,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Cylinder(radius=0.004, length=0.634),
        origin=Origin(xyz=(0.0, -0.001, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_color,
        name="hinge_barrel",
    )

    left_guide.visual(
        Box((0.034, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=trim_color,
        name="shoe",
    )
    left_guide.visual(
        Box((0.004, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=liner_color,
        name="fence",
    )

    right_guide.visual(
        Box((0.034, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=trim_color,
        name="shoe",
    )
    right_guide.visual(
        Box((0.004, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=liner_color,
        name="fence",
    )

    usb_cover.visual(
        Box((0.038, 0.003, 0.024)),
        origin=Origin(xyz=(0.0, 0.0015, -0.012)),
        material=trim_color,
        name="cover_panel",
    )
    usb_cover.visual(
        Cylinder(radius=0.002, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_color,
        name="cover_hinge_barrel",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.245, top_plane_z + 0.012)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "left_guide_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=left_guide,
        origin=Origin(xyz=(-0.180, -0.2255, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.15,
            lower=-0.080,
            upper=0.120,
        ),
    )
    model.articulation(
        "right_guide_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=right_guide,
        origin=Origin(xyz=(0.180, -0.2255, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.15,
            lower=-0.120,
            upper=0.080,
        ),
    )
    model.articulation(
        "usb_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=usb_cover,
        origin=Origin(xyz=(0.220, -0.243, 0.067)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-1.20,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    left_guide = object_model.get_part("left_guide")
    right_guide = object_model.get_part("right_guide")
    usb_cover = object_model.get_part("usb_cover")
    lid_hinge = object_model.get_articulation("lid_hinge")
    left_slide = object_model.get_articulation("left_guide_slide")
    right_slide = object_model.get_articulation("right_guide_slide")
    usb_hinge = object_model.get_articulation("usb_cover_hinge")

    top_frame_front = base.get_visual("top_frame_front")
    top_frame_rear = base.get_visual("top_frame_rear")
    base_hinge_leaf = base.get_visual("base_hinge_leaf")
    keeper_stop = base.get_visual("keeper_stop")
    usb_bezel = base.get_visual("usb_bezel")
    usb_hinge_seat = base.get_visual("usb_hinge_seat")

    front_wall_left = lid.get_visual("front_wall_left")
    front_wall_right = lid.get_visual("front_wall_right")
    latch_tongue = lid.get_visual("latch_tongue")
    lid_hinge_leaf = lid.get_visual("lid_hinge_leaf")
    hinge_barrel = lid.get_visual("hinge_barrel")
    slot_front_rail = lid.get_visual("slot_front_rail")
    slot_rear_rail = lid.get_visual("slot_rear_rail")

    left_shoe = left_guide.get_visual("shoe")
    right_shoe = right_guide.get_visual("shoe")
    usb_panel = usb_cover.get_visual("cover_panel")
    usb_barrel = usb_cover.get_visual("cover_hinge_barrel")

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

    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.20)
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=front_wall_left,
        negative_elem=top_frame_front,
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=front_wall_right,
        negative_elem=top_frame_front,
    )
    ctx.expect_gap(
        lid,
        base,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lid_hinge_leaf,
        negative_elem=base_hinge_leaf,
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem=lid_hinge_leaf,
        negative_elem=base_hinge_leaf,
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="x",
        min_overlap=0.62,
        elem_a=lid_hinge_leaf,
        elem_b=base_hinge_leaf,
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="xz",
        min_overlap=0.01,
        elem_a=latch_tongue,
        elem_b=keeper_stop,
    )
    ctx.expect_contact(lid, base, elem_a=latch_tongue, elem_b=keeper_stop)

    ctx.expect_within(left_guide, lid, axes="x", inner_elem=left_shoe)
    ctx.expect_within(right_guide, lid, axes="x", inner_elem=right_shoe)
    ctx.expect_gap(
        left_guide,
        lid,
        axis="y",
        min_gap=0.001,
        positive_elem=left_shoe,
        negative_elem=slot_front_rail,
    )
    ctx.expect_gap(
        lid,
        left_guide,
        axis="y",
        min_gap=0.001,
        positive_elem=slot_rear_rail,
        negative_elem=left_shoe,
    )
    ctx.expect_gap(
        right_guide,
        lid,
        axis="y",
        min_gap=0.001,
        positive_elem=right_shoe,
        negative_elem=slot_front_rail,
    )
    ctx.expect_gap(
        lid,
        right_guide,
        axis="y",
        min_gap=0.001,
        positive_elem=slot_rear_rail,
        negative_elem=right_shoe,
    )

    ctx.expect_contact(usb_cover, base, elem_a=usb_panel, elem_b=usb_bezel)
    ctx.expect_overlap(
        usb_cover,
        base,
        axes="x",
        min_overlap=0.030,
        elem_a=usb_barrel,
        elem_b=usb_hinge_seat,
    )

    with ctx.pose({lid_hinge: 1.15}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.18,
            positive_elem=front_wall_left,
            negative_elem=top_frame_rear,
        )
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.18,
            positive_elem=front_wall_right,
            negative_elem=top_frame_rear,
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="x",
            min_overlap=0.62,
            elem_a=hinge_barrel,
            elem_b=base_hinge_leaf,
        )

    with ctx.pose({left_slide: 0.10, right_slide: -0.10}):
        ctx.expect_within(left_guide, lid, axes="x", inner_elem=left_shoe)
        ctx.expect_within(right_guide, lid, axes="x", inner_elem=right_shoe)
        ctx.expect_gap(
            left_guide,
            lid,
            axis="y",
            min_gap=0.001,
            positive_elem=left_shoe,
            negative_elem=slot_front_rail,
        )
        ctx.expect_gap(
            lid,
            left_guide,
            axis="y",
            min_gap=0.001,
            positive_elem=slot_rear_rail,
            negative_elem=left_shoe,
        )
        ctx.expect_gap(
            right_guide,
            lid,
            axis="y",
            min_gap=0.001,
            positive_elem=right_shoe,
            negative_elem=slot_front_rail,
        )
        ctx.expect_gap(
            lid,
            right_guide,
            axis="y",
            min_gap=0.001,
            positive_elem=slot_rear_rail,
            negative_elem=right_shoe,
        )
        ctx.expect_gap(
            right_guide,
            left_guide,
            axis="x",
            min_gap=0.04,
            positive_elem=right_shoe,
            negative_elem=left_shoe,
        )

    with ctx.pose({usb_hinge: -1.05}):
        ctx.expect_gap(
            base,
            usb_cover,
            axis="y",
            min_gap=0.001,
            positive_elem=usb_bezel,
            negative_elem=usb_panel,
        )
        ctx.expect_overlap(
            usb_cover,
            base,
            axes="x",
            min_overlap=0.030,
            elem_a=usb_barrel,
            elem_b=usb_hinge_seat,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
