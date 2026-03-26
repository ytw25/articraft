from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from pathlib import Path

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

HERE = Path(__file__).resolve().parent


DESK_TOP_THICKNESS = 0.03
DESK_TOP_CENTER_Z = 0.725
DESK_UNDERSIDE_Z = DESK_TOP_CENTER_Z - (DESK_TOP_THICKNESS * 0.5)
DRAWER_FRONT_X = 0.05
DRAWER_FRONT_Y_CLOSED = -0.165
DRAWER_CENTER_Z = 0.693
def _add_leg(model: ArticulatedObject, name: str, x: float, y: float, *, leg_material, foot_material):
    leg = model.part(name)
    leg.visual(
        Box((0.045, 0.045, 0.702)),
        origin=Origin(xyz=(0.0, 0.0, -0.351)),
        material=leg_material,
        name="leg_post",
    )
    leg.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.699)),
        material=foot_material,
        name="foot_stem",
    )
    leg.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.706)),
        material=foot_material,
        name="foot_pad",
    )
    leg.inertial = Inertial.from_geometry(
        Box((0.05, 0.05, 0.71)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, -0.355)),
    )
    model.articulation(
        f"desk_to_{name}",
        ArticulationType.FIXED,
        parent="desk_frame",
        child=leg,
        origin=Origin(xyz=(x, y, DESK_UNDERSIDE_Z)),
    )
    return leg


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_desk")

    wood = model.material("warm_oak", rgba=(0.73, 0.62, 0.46, 1.0))
    frame_black = model.material("frame_black", rgba=(0.18, 0.18, 0.19, 1.0))
    drawer_black = model.material("drawer_black", rgba=(0.20, 0.20, 0.21, 1.0))
    runner_grey = model.material("runner_grey", rgba=(0.55, 0.57, 0.60, 1.0))
    grommet_grey = model.material("grommet_grey", rgba=(0.16, 0.17, 0.18, 1.0))

    grommet_center = (-0.56, 0.47)

    desk_frame = model.part("desk_frame")
    desk_frame.visual(
        Box((1.56, 0.44, DESK_TOP_THICKNESS)),
        origin=Origin(xyz=(-0.10, -0.16, DESK_TOP_CENTER_Z)),
        material=wood,
        name="main_top",
    )
    desk_frame.visual(
        Box((0.28, 0.60, DESK_TOP_THICKNESS)),
        origin=Origin(xyz=(-0.74, 0.36, DESK_TOP_CENTER_Z)),
        material=wood,
        name="return_left_panel",
    )
    desk_frame.visual(
        Box((0.40, 0.60, DESK_TOP_THICKNESS)),
        origin=Origin(xyz=(-0.32, 0.36, DESK_TOP_CENTER_Z)),
        material=wood,
        name="return_right_panel",
    )
    desk_frame.visual(
        Box((0.08, 0.37, DESK_TOP_THICKNESS)),
        origin=Origin(xyz=(-0.56, 0.245, DESK_TOP_CENTER_Z)),
        material=wood,
        name="grommet_front_bridge",
    )
    desk_frame.visual(
        Box((0.08, 0.15, DESK_TOP_THICKNESS)),
        origin=Origin(xyz=(-0.56, 0.585, DESK_TOP_CENTER_Z)),
        material=wood,
        name="grommet_rear_bridge",
    )
    desk_frame.visual(
        Box((0.60, 0.04, 0.09)),
        origin=Origin(xyz=(-0.58, -0.36, 0.665)),
        material=frame_black,
        name="front_apron_left",
    )
    desk_frame.visual(
        Box((0.30, 0.04, 0.09)),
        origin=Origin(xyz=(0.53, -0.36, 0.665)),
        material=frame_black,
        name="front_apron_right",
    )
    desk_frame.visual(
        Box((0.04, 1.00, 0.09)),
        origin=Origin(xyz=(-0.86, 0.14, 0.665)),
        material=frame_black,
        name="left_side_apron",
    )
    desk_frame.visual(
        Box((0.76, 0.04, 0.09)),
        origin=Origin(xyz=(-0.50, 0.64, 0.665)),
        material=frame_black,
        name="rear_return_apron",
    )
    desk_frame.visual(
        Box((0.04, 0.60, 0.09)),
        origin=Origin(xyz=(-0.12, 0.36, 0.665)),
        material=frame_black,
        name="inside_vertical_apron",
    )
    desk_frame.visual(
        Box((0.80, 0.04, 0.09)),
        origin=Origin(xyz=(0.28, 0.04, 0.665)),
        material=frame_black,
        name="inside_horizontal_apron",
    )
    desk_frame.visual(
        Box((0.62, 0.028, 0.010)),
        origin=Origin(xyz=(DRAWER_FRONT_X, -0.333, 0.705)),
        material=frame_black,
        name="drawer_header",
    )
    desk_frame.visual(
        Box((0.026, 0.34, 0.012)),
        origin=Origin(xyz=(DRAWER_FRONT_X, -0.165, 0.699)),
        material=runner_grey,
        name="upper_runner",
    )
    desk_frame.inertial = Inertial.from_geometry(
        Box((1.56, 1.04, 0.11)),
        mass=25.0,
        origin=Origin(xyz=(-0.10, 0.14, 0.69)),
    )

    _add_leg(model, "front_left_leg", -0.80, -0.31, leg_material=frame_black, foot_material=runner_grey)
    _add_leg(model, "front_right_leg", 0.60, -0.31, leg_material=frame_black, foot_material=runner_grey)
    _add_leg(model, "rear_left_leg", -0.80, 0.58, leg_material=frame_black, foot_material=runner_grey)
    _add_leg(model, "rear_inner_leg", -0.18, 0.58, leg_material=frame_black, foot_material=runner_grey)
    _add_leg(model, "center_support_leg", -0.18, 0.02, leg_material=frame_black, foot_material=runner_grey)

    grommet = model.part("grommet")
    grommet.visual(
        Box((0.018, 0.100, 0.004)),
        origin=Origin(xyz=(-0.041, 0.041, 0.002)),
        material=grommet_grey,
        name="grommet_left",
    )
    grommet.visual(
        Box((0.018, 0.100, 0.004)),
        origin=Origin(xyz=(0.041, 0.041, 0.002)),
        material=grommet_grey,
        name="grommet_right",
    )
    grommet.visual(
        Box((0.064, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=grommet_grey,
        name="grommet_front",
    )
    grommet.visual(
        Box((0.064, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.082, 0.002)),
        material=grommet_grey,
        name="grommet_rear",
    )
    grommet.inertial = Inertial.from_geometry(
        Box((0.08, 0.08, 0.004)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.041, 0.002)),
    )
    model.articulation(
        "desk_to_grommet",
        ArticulationType.FIXED,
        parent=desk_frame,
        child=grommet,
        origin=Origin(
            xyz=(
                grommet_center[0],
                grommet_center[1] - 0.041,
                DESK_TOP_CENTER_Z + (DESK_TOP_THICKNESS * 0.5),
            )
        ),
    )

    drawer = model.part("pencil_drawer")
    drawer.visual(
        Box((0.52, 0.28, 0.050)),
        origin=Origin(xyz=(0.0, -0.039, -0.033)),
        material=drawer_black,
        name="drawer_body",
    )
    drawer.visual(
        Box((0.56, 0.022, 0.060)),
        origin=Origin(xyz=(0.0, -0.179, -0.024)),
        material=drawer_black,
        name="drawer_front",
    )
    drawer.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, -0.196, -0.024), rpy=(pi / 2.0, 0.0, 0.0)),
        material=runner_grey,
        name="drawer_knob",
    )
    drawer.visual(
        Box((0.020, 0.24, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=runner_grey,
        name="lower_runner",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.56, 0.30, 0.07)),
        mass=3.8,
        origin=Origin(xyz=(0.0, -0.060, -0.026)),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=desk_frame,
        child=drawer,
        origin=Origin(xyz=(DRAWER_FRONT_X, DRAWER_FRONT_Y_CLOSED, DRAWER_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.35, lower=0.0, upper=0.22),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    desk_frame = object_model.get_part("desk_frame")
    drawer = object_model.get_part("pencil_drawer")
    grommet = object_model.get_part("grommet")
    drawer_slide = object_model.get_articulation("drawer_slide")

    top_surface = desk_frame.get_visual("main_top")
    return_left_panel = desk_frame.get_visual("return_left_panel")
    return_right_panel = desk_frame.get_visual("return_right_panel")
    grommet_rear_bridge = desk_frame.get_visual("grommet_rear_bridge")
    drawer_header = desk_frame.get_visual("drawer_header")
    upper_runner = desk_frame.get_visual("upper_runner")
    grommet_front_bridge = desk_frame.get_visual("grommet_front_bridge")
    grommet_left = grommet.get_visual("grommet_left")
    grommet_right = grommet.get_visual("grommet_right")
    grommet_front = grommet.get_visual("grommet_front")
    grommet_rear = grommet.get_visual("grommet_rear")
    drawer_front = drawer.get_visual("drawer_front")
    drawer_knob = drawer.get_visual("drawer_knob")
    lower_runner = drawer.get_visual("lower_runner")

    leg_names = [
        "front_left_leg",
        "front_right_leg",
        "rear_left_leg",
        "rear_inner_leg",
        "center_support_leg",
    ]
    legs = [object_model.get_part(name) for name in leg_names]
    leg_posts = [leg.get_visual("leg_post") for leg in legs]
    leg_supports = [
        top_surface,
        top_surface,
        return_left_panel,
        return_right_panel,
        top_surface,
    ]

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

    for leg, leg_post, support_surface in zip(legs, leg_posts, leg_supports):
        ctx.expect_gap(
            desk_frame,
            leg,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=support_surface,
            negative_elem=leg_post,
            name=f"{leg.name}_mount_gap",
        )
        ctx.expect_overlap(
            leg,
            desk_frame,
            axes="xy",
            min_overlap=0.001,
            elem_a=leg_post,
            elem_b=support_surface,
            name=f"{leg.name}_footprint_under_top",
        )

    ctx.expect_gap(
        grommet,
        desk_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=grommet_front,
        negative_elem=grommet_front_bridge,
        name="grommet_seated_on_desktop",
    )
    ctx.expect_overlap(
        grommet,
        desk_frame,
        axes="xy",
        min_overlap=0.001,
        elem_a=grommet_front,
        elem_b=grommet_front_bridge,
        name="grommet_overlaps_desktop_surface",
    )
    ctx.expect_gap(
        grommet,
        desk_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=grommet_left,
        negative_elem=return_left_panel,
        name="grommet_left_flange_seated",
    )
    ctx.expect_overlap(
        grommet,
        desk_frame,
        axes="xy",
        min_overlap=0.008,
        elem_a=grommet_left,
        elem_b=return_left_panel,
        name="grommet_left_flange_supported",
    )
    ctx.expect_gap(
        grommet,
        desk_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=grommet_right,
        negative_elem=return_right_panel,
        name="grommet_right_flange_seated",
    )
    ctx.expect_overlap(
        grommet,
        desk_frame,
        axes="xy",
        min_overlap=0.008,
        elem_a=grommet_right,
        elem_b=return_right_panel,
        name="grommet_right_flange_supported",
    )
    ctx.expect_gap(
        grommet,
        desk_frame,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=grommet_rear,
        negative_elem=grommet_rear_bridge,
        name="grommet_rear_flange_seated",
    )
    ctx.expect_overlap(
        grommet,
        desk_frame,
        axes="xy",
        min_overlap=0.008,
        elem_a=grommet_rear,
        elem_b=grommet_rear_bridge,
        name="grommet_rear_flange_supported",
    )

    ctx.expect_overlap(
        drawer,
        desk_frame,
        axes="xy",
        min_overlap=0.004,
        elem_a=lower_runner,
        elem_b=upper_runner,
        name="drawer_runner_pair_nested_closed",
    )
    ctx.expect_contact(
        desk_frame,
        drawer,
        elem_a=upper_runner,
        elem_b=lower_runner,
        name="drawer_runner_pair_contact_closed",
    )
    ctx.expect_gap(
        desk_frame,
        drawer,
        axis="z",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem=drawer_header,
        negative_elem=drawer_front,
        name="drawer_face_tucked_under_front_edge",
    )
    ctx.expect_within(
        drawer,
        desk_frame,
        axes="xy",
        inner_elem=drawer_front,
        outer_elem=top_surface,
        name="drawer_front_within_main_desk_width_closed",
    )
    ctx.expect_contact(
        drawer,
        drawer,
        elem_a=drawer_knob,
        elem_b=drawer_front,
        name="drawer_knob_attached_to_front",
    )

    with ctx.pose({drawer_slide: 0.20}):
        ctx.expect_gap(
            desk_frame,
            drawer,
            axis="y",
            min_gap=0.18,
            positive_elem=drawer_header,
            negative_elem=drawer_front,
            name="drawer_opens_forward",
        )
        ctx.expect_overlap(
            drawer,
            desk_frame,
            axes="xy",
            min_overlap=0.004,
            elem_a=lower_runner,
            elem_b=upper_runner,
            name="drawer_runner_pair_nested_open",
        )
        ctx.expect_contact(
            desk_frame,
            drawer,
            elem_a=upper_runner,
            elem_b=lower_runner,
            name="drawer_runner_pair_contact_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
