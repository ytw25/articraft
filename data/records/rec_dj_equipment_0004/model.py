from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import tempfile

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

ASSET_ROOT = tempfile.gettempdir()

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir(ASSET_ROOT)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_dj_controller")

    body_color = model.material("body_color", rgba=(0.14, 0.15, 0.16, 1.0))
    deck_color = model.material("deck_color", rgba=(0.20, 0.21, 0.23, 1.0))
    metal_color = model.material("metal_color", rgba=(0.68, 0.70, 0.73, 1.0))
    jog_color = model.material("jog_color", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_color = model.material("accent_color", rgba=(0.80, 0.24, 0.20, 1.0))
    blue_button = model.material("blue_button", rgba=(0.19, 0.48, 0.90, 1.0))
    amber_button = model.material("amber_button", rgba=(0.93, 0.63, 0.18, 1.0))

    body_width = 0.238
    body_depth = 0.138
    body_height = 0.012
    top_z = 0.014
    slot_y = -0.041

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, body_height)),
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
        material=body_color,
        name="body_shell",
    )
    body.visual(
        Box((0.094, 0.090, 0.002)),
        origin=Origin(xyz=(-0.072, 0.014, 0.013)),
        material=deck_color,
        name="left_deck_pad",
    )
    body.visual(
        Box((0.094, 0.090, 0.002)),
        origin=Origin(xyz=(0.072, 0.014, 0.013)),
        material=deck_color,
        name="right_deck_pad",
    )
    body.visual(
        Box((0.044, 0.104, 0.002)),
        origin=Origin(xyz=(0.0, 0.006, 0.013)),
        material=deck_color,
        name="center_strip",
    )
    body.visual(
        Box((0.090, 0.005, 0.002)),
        origin=Origin(xyz=(0.0, slot_y + 0.0065, 0.013)),
        material=metal_color,
        name="slot_upper_rail",
    )
    body.visual(
        Box((0.090, 0.005, 0.002)),
        origin=Origin(xyz=(0.0, slot_y - 0.0065, 0.013)),
        material=metal_color,
        name="slot_lower_rail",
    )
    body.visual(
        Box((0.004, 0.008, 0.002)),
        origin=Origin(xyz=(-0.043, slot_y, 0.013)),
        material=metal_color,
        name="slot_left_stop",
    )
    body.visual(
        Box((0.004, 0.008, 0.002)),
        origin=Origin(xyz=(0.043, slot_y, 0.013)),
        material=metal_color,
        name="slot_right_stop",
    )
    body.visual(
        Box((0.082, 0.008, 0.001)),
        origin=Origin(xyz=(0.0, slot_y, 0.0125)),
        material=body_color,
        name="slot_floor",
    )
    body.visual(
        Box((0.004, 0.094, 0.006)),
        origin=Origin(xyz=(-0.117, 0.0, 0.015)),
        material=body_color,
        name="left_hinge_seat",
    )
    body.visual(
        Box((0.004, 0.094, 0.006)),
        origin=Origin(xyz=(0.117, 0.0, 0.015)),
        material=body_color,
        name="right_hinge_seat",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    def add_jog(
        part_name: str,
        center_x: float,
        center_y: float,
        joint_name: str,
    ) -> None:
        jog = model.part(part_name)
        jog.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=metal_color,
            name="stub_shaft",
        )
        jog.visual(
            Box((0.052, 0.042, 0.003)),
            origin=Origin(xyz=(0.0, 0.0, 0.0055)),
            material=jog_color,
            name="jog_pad",
        )
        jog.visual(
            Box((0.032, 0.022, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, 0.0075)),
            material=deck_color,
            name="jog_inset",
        )
        jog.inertial = Inertial.from_geometry(
            Box((0.052, 0.042, 0.008)),
            mass=0.09,
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=jog,
            origin=Origin(xyz=(center_x, center_y, top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=10.0),
        )

    add_jog("left_jog", center_x=-0.072, center_y=0.020, joint_name="left_jog_spin")
    add_jog("right_jog", center_x=0.072, center_y=0.020, joint_name="right_jog_spin")

    crossfader = model.part("crossfader")
    crossfader.visual(
        Box((0.010, 0.006, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, 0.0005)),
        material=metal_color,
        name="slider_rider",
    )
    crossfader.visual(
        Box((0.015, 0.020, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material=accent_color,
        name="slider_cap",
    )
    crossfader.inertial = Inertial.from_geometry(
        Box((0.015, 0.020, 0.006)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )
    model.articulation(
        "crossfader_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=crossfader,
        origin=Origin(xyz=(0.0, slot_y, 0.013)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.4,
            lower=-0.028,
            upper=0.028,
        ),
    )

    def add_wing(
        part_name: str,
        side: str,
        joint_name: str,
        joint_origin_x: float,
        axis: tuple[float, float, float],
    ) -> None:
        sign = -1.0 if side == "left" else 1.0
        wing = model.part(part_name)
        wing.visual(
            Box((0.048, 0.094, 0.004)),
            origin=Origin(xyz=(sign * 0.024, 0.0, 0.002)),
            material=deck_color,
            name="panel",
        )
        wing.visual(
            Box((0.014, 0.016, 0.004)),
            origin=Origin(xyz=(sign * 0.028, -0.020, 0.006)),
            material=blue_button,
            name="cue_a",
        )
        wing.visual(
            Box((0.014, 0.016, 0.004)),
            origin=Origin(xyz=(sign * 0.028, 0.020, 0.006)),
            material=amber_button,
            name="cue_b",
        )
        wing.inertial = Inertial.from_geometry(
            Box((0.048, 0.094, 0.008)),
            mass=0.07,
            origin=Origin(xyz=(sign * 0.024, 0.0, 0.004)),
        )
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=wing,
            origin=Origin(xyz=(joint_origin_x, 0.0, top_z)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=2.0,
                lower=0.0,
                upper=math.radians(94.0),
            ),
        )

    add_wing(
        "left_wing",
        side="left",
        joint_name="left_wing_fold",
        joint_origin_x=-0.119,
        axis=(0.0, 1.0, 0.0),
    )
    add_wing(
        "right_wing",
        side="right",
        joint_name="right_wing_fold",
        joint_origin_x=0.119,
        axis=(0.0, -1.0, 0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSET_ROOT)
    body = object_model.get_part("body")
    left_jog = object_model.get_part("left_jog")
    right_jog = object_model.get_part("right_jog")
    crossfader = object_model.get_part("crossfader")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")

    left_jog_spin = object_model.get_articulation("left_jog_spin")
    right_jog_spin = object_model.get_articulation("right_jog_spin")
    crossfader_slide = object_model.get_articulation("crossfader_slide")
    left_wing_fold = object_model.get_articulation("left_wing_fold")
    right_wing_fold = object_model.get_articulation("right_wing_fold")

    body_shell = body.get_visual("body_shell")
    left_deck_pad = body.get_visual("left_deck_pad")
    right_deck_pad = body.get_visual("right_deck_pad")
    center_strip = body.get_visual("center_strip")
    slot_upper_rail = body.get_visual("slot_upper_rail")
    slot_lower_rail = body.get_visual("slot_lower_rail")
    slot_floor = body.get_visual("slot_floor")
    left_hinge_seat = body.get_visual("left_hinge_seat")
    right_hinge_seat = body.get_visual("right_hinge_seat")

    left_shaft = left_jog.get_visual("stub_shaft")
    right_shaft = right_jog.get_visual("stub_shaft")
    left_pad = left_jog.get_visual("jog_pad")
    right_pad = right_jog.get_visual("jog_pad")

    slider_rider = crossfader.get_visual("slider_rider")
    slider_cap = crossfader.get_visual("slider_cap")

    left_panel = left_wing.get_visual("panel")
    right_panel = right_wing.get_visual("panel")
    left_button = left_wing.get_visual("cue_a")
    left_button_b = left_wing.get_visual("cue_b")
    right_button = right_wing.get_visual("cue_a")
    right_button_b = right_wing.get_visual("cue_b")

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

    ctx.allow_overlap(left_wing, body, reason="folded left wing nests slightly over the hinge seat")
    ctx.allow_overlap(right_wing, body, reason="folded right wing nests slightly over the hinge seat")

    ctx.expect_gap(
        left_jog,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_shaft,
        negative_elem=left_deck_pad,
    )
    ctx.expect_gap(
        right_jog,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_shaft,
        negative_elem=right_deck_pad,
    )
    ctx.expect_contact(
        crossfader,
        body,
        elem_a=slider_rider,
        elem_b=slot_floor,
    )
    ctx.expect_within(
        left_jog,
        body,
        axes="xy",
        inner_elem=left_pad,
        outer_elem=left_deck_pad,
    )
    ctx.expect_within(
        right_jog,
        body,
        axes="xy",
        inner_elem=right_pad,
        outer_elem=right_deck_pad,
    )
    ctx.expect_within(
        crossfader,
        body,
        axes="xy",
        inner_elem=slider_rider,
        outer_elem=slot_floor,
    )
    ctx.expect_gap(
        body,
        body,
        axis="y",
        min_gap=0.0075,
        max_gap=0.0085,
        positive_elem=slot_upper_rail,
        negative_elem=slot_lower_rail,
    )
    ctx.expect_contact(
        body,
        left_wing,
        elem_a=left_hinge_seat,
        elem_b=left_panel,
    )
    ctx.expect_contact(
        right_wing,
        body,
        elem_a=right_panel,
        elem_b=right_hinge_seat,
    )
    ctx.expect_gap(
        body,
        left_jog,
        axis="x",
        min_gap=0.020,
        positive_elem=center_strip,
        negative_elem=left_pad,
    )
    ctx.expect_gap(
        right_jog,
        body,
        axis="x",
        min_gap=0.020,
        positive_elem=right_pad,
        negative_elem=center_strip,
    )
    ctx.expect_overlap(left_jog, body, axes="xy", min_overlap=0.001)
    ctx.expect_overlap(right_jog, body, axes="xy", min_overlap=0.001)

    with ctx.pose({crossfader_slide: -0.028}):
        ctx.expect_within(
            crossfader,
            body,
            axes="xy",
            inner_elem=slider_rider,
            outer_elem=slot_floor,
        )
    with ctx.pose({crossfader_slide: 0.028}):
        ctx.expect_within(
            crossfader,
            body,
            axes="xy",
            inner_elem=slider_rider,
            outer_elem=slot_floor,
        )

    with ctx.pose({left_jog_spin: math.radians(35.0), right_jog_spin: math.radians(-28.0)}):
        ctx.expect_gap(
            left_jog,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=left_shaft,
            negative_elem=left_deck_pad,
        )
        ctx.expect_gap(
            right_jog,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=right_shaft,
            negative_elem=right_deck_pad,
        )

    folded_angle = math.radians(88.0)
    with ctx.pose({left_wing_fold: folded_angle, right_wing_fold: folded_angle}):
        ctx.expect_gap(
            left_wing,
            body,
            axis="z",
            min_gap=0.006,
            positive_elem=left_button,
            negative_elem=body_shell,
        )
        ctx.expect_gap(
            right_wing,
            body,
            axis="z",
            min_gap=0.006,
            positive_elem=right_button,
            negative_elem=body_shell,
        )
        ctx.expect_gap(
            left_wing,
            body,
            axis="z",
            min_gap=0.006,
            positive_elem=left_button_b,
            negative_elem=body_shell,
        )
        ctx.expect_gap(
            right_wing,
            body,
            axis="z",
            min_gap=0.006,
            positive_elem=right_button_b,
            negative_elem=body_shell,
        )
        ctx.expect_overlap(
            left_wing,
            body,
            axes="yz",
            min_overlap=0.002,
            elem_a=left_panel,
            elem_b=left_hinge_seat,
        )
        ctx.expect_overlap(
            right_wing,
            body,
            axes="yz",
            min_overlap=0.002,
            elem_a=right_panel,
            elem_b=right_hinge_seat,
        )
        ctx.expect_gap(
            left_jog,
            left_wing,
            axis="x",
            min_gap=0.010,
            positive_elem=left_pad,
            negative_elem=left_panel,
        )
        ctx.expect_gap(
            right_wing,
            right_jog,
            axis="x",
            min_gap=0.010,
            positive_elem=right_panel,
            negative_elem=right_pad,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
