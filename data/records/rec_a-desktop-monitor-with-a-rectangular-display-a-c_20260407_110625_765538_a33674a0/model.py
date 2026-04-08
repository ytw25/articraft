from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
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


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def _translated_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_monitor")

    bezel_black = model.material("bezel_black", rgba=(0.09, 0.10, 0.11, 1.0))
    shell_black = model.material("shell_black", rgba=(0.14, 0.15, 0.16, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.68, 0.69, 0.71, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.07, 0.12, 0.14, 0.92))
    button_black = model.material("button_black", rgba=(0.05, 0.05, 0.06, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.260, 0.190, 0.026), 0.012),
            "monitor_base_foot",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=shell_black,
        name="foot_plate",
    )
    base.visual(
        Box((0.110, 0.080, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=dark_gray,
        name="base_pedestal",
    )
    base.visual(
        Box((0.007, 0.040, 0.230)),
        origin=Origin(xyz=(-0.0265, 0.0, 0.127)),
        material=dark_gray,
        name="outer_sleeve_left",
    )
    base.visual(
        Box((0.007, 0.040, 0.230)),
        origin=Origin(xyz=(0.0265, 0.0, 0.127)),
        material=dark_gray,
        name="outer_sleeve_right",
    )
    base.visual(
        Box((0.046, 0.007, 0.230)),
        origin=Origin(xyz=(0.0, -0.0165, 0.127)),
        material=dark_gray,
        name="outer_sleeve_front",
    )
    base.visual(
        Box((0.046, 0.007, 0.230)),
        origin=Origin(xyz=(0.0, 0.0165, 0.127)),
        material=dark_gray,
        name="outer_sleeve_back",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.260, 0.190, 0.260)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.040, 0.026, 0.370)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=satin_silver,
        name="inner_column",
    )
    mast.visual(
        Box((0.058, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=dark_gray,
        name="head_block",
    )
    mast.visual(
        Box((0.064, 0.010, 0.052)),
        origin=Origin(xyz=(0.0, -0.010, 0.280)),
        material=dark_gray,
        name="yoke_bridge",
    )
    mast.visual(
        Box((0.012, 0.032, 0.052)),
        origin=Origin(xyz=(-0.026, 0.002, 0.280)),
        material=dark_gray,
        name="left_yoke",
    )
    mast.visual(
        Box((0.012, 0.032, 0.052)),
        origin=Origin(xyz=(0.026, 0.002, 0.280)),
        material=dark_gray,
        name="right_yoke",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.080, 0.050, 0.390)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.242)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.12,
            lower=0.0,
            upper=0.080,
        ),
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Cylinder(radius=0.0095, length=0.040),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="tilt_barrel",
    )
    tilt_head.visual(
        Box((0.020, 0.017, 0.030)),
        origin=Origin(xyz=(0.0, 0.0135, 0.0)),
        material=dark_gray,
        name="tilt_stem",
    )
    tilt_head.visual(
        Box((0.022, 0.018, 0.032)),
        origin=Origin(xyz=(0.0, 0.031, 0.0)),
        material=dark_gray,
        name="tilt_connector",
    )
    tilt_head.visual(
        Box((0.068, 0.012, 0.062)),
        origin=Origin(xyz=(0.0, 0.046, 0.0)),
        material=dark_gray,
        name="tilt_plate",
    )
    tilt_head.inertial = Inertial.from_geometry(
        Box((0.090, 0.050, 0.120)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
    )

    model.articulation(
        "mast_to_tilt_head",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=tilt_head,
        origin=Origin(xyz=(0.0, 0.0, 0.280)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.20,
            upper=0.45,
        ),
    )

    portrait_mount = model.part("portrait_mount")
    portrait_mount.visual(
        Box((0.085, 0.008, 0.085)),
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
        material=dark_gray,
        name="mount_plate",
    )
    portrait_mount.visual(
        Box((0.120, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=dark_gray,
        name="vesa_crossbar",
    )
    portrait_mount.visual(
        Box((0.016, 0.010, 0.120)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=dark_gray,
        name="vesa_spine",
    )
    portrait_mount.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="portrait_collar",
    )
    portrait_mount.inertial = Inertial.from_geometry(
        Box((0.130, 0.030, 0.130)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.008, 0.0)),
    )

    model.articulation(
        "tilt_head_to_portrait_mount",
        ArticulationType.REVOLUTE,
        parent=tilt_head,
        child=portrait_mount,
        origin=Origin(xyz=(0.0, 0.052, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    display = model.part("display")
    display.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _rect_profile(0.550, 0.350),
                [_rect_profile(0.514, 0.286)],
                0.006,
            ),
            "monitor_front_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.027, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bezel_black,
        name="front_bezel",
    )
    display.visual(
        Box((0.540, 0.024, 0.320)),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=shell_black,
        name="rear_shell",
    )
    display.visual(
        Box((0.550, 0.024, 0.022)),
        origin=Origin(xyz=(0.0, -0.012, -0.152)),
        material=shell_black,
        name="lower_bezel_core",
    )
    display.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _rect_profile(0.170, 0.016),
                [
                    _translated_profile(_rect_profile(0.014, 0.010), -0.056, 0.0),
                    _translated_profile(_rect_profile(0.014, 0.010), -0.028, 0.0),
                    _translated_profile(_rect_profile(0.014, 0.010), 0.000, 0.0),
                    _translated_profile(_rect_profile(0.014, 0.010), 0.028, 0.0),
                    _translated_profile(_rect_profile(0.014, 0.010), 0.056, 0.0),
                ],
                0.012,
            ),
            "monitor_control_strip",
        ),
        origin=Origin(xyz=(0.0, -0.021, -0.169), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_black,
        name="control_strip",
    )
    display.visual(
        Box((0.507, 0.003, 0.285)),
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
        material=screen_glass,
        name="screen_glass",
    )
    display.visual(
        Box((0.195, 0.020, 0.148)),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=dark_gray,
        name="rear_housing",
    )
    display.inertial = Inertial.from_geometry(
        Box((0.550, 0.050, 0.350)),
        mass=3.4,
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
    )

    model.articulation(
        "portrait_mount_to_display",
        ArticulationType.FIXED,
        parent=portrait_mount,
        child=display,
        origin=Origin(xyz=(0.0, 0.052, 0.0)),
    )

    button_x_positions = (-0.056, -0.028, 0.0, 0.028, 0.056)
    for index, button_x in enumerate(button_x_positions, start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.012, 0.003, 0.008)),
            origin=Origin(xyz=(0.0, 0.0015, 0.0)),
            material=button_black,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.012, 0.003, 0.008)),
            mass=0.01,
            origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        )
        model.articulation(
            f"display_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(button_x, -0.033, -0.169)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.03,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    display = object_model.get_part("display")
    base = object_model.get_part("base")
    button_1 = object_model.get_part("button_1")
    button_5 = object_model.get_part("button_5")

    base_to_mast = object_model.get_articulation("base_to_mast")
    mast_to_tilt_head = object_model.get_articulation("mast_to_tilt_head")
    tilt_head_to_portrait_mount = object_model.get_articulation("tilt_head_to_portrait_mount")
    display_to_button_1 = object_model.get_articulation("display_to_button_1")

    ctx.expect_gap(
        display,
        button_1,
        axis="y",
        positive_elem="front_bezel",
        negative_elem="button_cap",
        max_gap=0.0001,
        max_penetration=0.0001,
        name="left button seats at the front bezel plane",
    )
    ctx.expect_gap(
        display,
        button_5,
        axis="y",
        positive_elem="front_bezel",
        negative_elem="button_cap",
        max_gap=0.0001,
        max_penetration=0.0001,
        name="right button seats at the front bezel plane",
    )
    ctx.expect_overlap(
        display,
        button_1,
        axes="xz",
        elem_a="control_strip",
        elem_b="button_cap",
        min_overlap=0.007,
        name="left button aligns with the control strip opening",
    )
    ctx.expect_overlap(
        display,
        button_5,
        axes="xz",
        elem_a="control_strip",
        elem_b="button_cap",
        min_overlap=0.007,
        name="right button aligns with the control strip opening",
    )
    ctx.expect_origin_gap(
        display,
        base,
        axis="z",
        min_gap=0.25,
        name="display sits well above the desk base",
    )

    rest_display_pos = ctx.part_world_position(display)
    rest_button_pos = ctx.part_world_position(button_1)

    mast_upper = (
        base_to_mast.motion_limits.upper
        if base_to_mast.motion_limits is not None and base_to_mast.motion_limits.upper is not None
        else 0.08
    )
    tilt_up = 0.35
    portrait_turn = (
        tilt_head_to_portrait_mount.motion_limits.upper
        if tilt_head_to_portrait_mount.motion_limits is not None
        and tilt_head_to_portrait_mount.motion_limits.upper is not None
        else math.pi / 2.0
    )
    button_travel = (
        display_to_button_1.motion_limits.upper
        if display_to_button_1.motion_limits is not None
        and display_to_button_1.motion_limits.upper is not None
        else 0.003
    )

    with ctx.pose({base_to_mast: mast_upper}):
        raised_display_pos = ctx.part_world_position(display)
    ctx.check(
        "stand height adjustment raises the display",
        rest_display_pos is not None
        and raised_display_pos is not None
        and raised_display_pos[2] > rest_display_pos[2] + 0.07,
        details=f"rest={rest_display_pos}, raised={raised_display_pos}",
    )

    with ctx.pose({mast_to_tilt_head: tilt_up}):
        tilted_display_pos = ctx.part_world_position(display)
    ctx.check(
        "positive tilt lifts the display",
        rest_display_pos is not None
        and tilted_display_pos is not None
        and tilted_display_pos[2] > rest_display_pos[2] + 0.02,
        details=f"rest={rest_display_pos}, tilted={tilted_display_pos}",
    )

    with ctx.pose({tilt_head_to_portrait_mount: portrait_turn}):
        portrait_aabb = ctx.part_world_aabb(display)
    portrait_dims_ok = False
    portrait_dims = None
    if portrait_aabb is not None:
        portrait_dims = (
            portrait_aabb[1][0] - portrait_aabb[0][0],
            portrait_aabb[1][1] - portrait_aabb[0][1],
            portrait_aabb[1][2] - portrait_aabb[0][2],
        )
        portrait_dims_ok = portrait_dims[2] > portrait_dims[0] + 0.12
    ctx.check(
        "portrait joint turns the panel tall",
        portrait_dims_ok,
        details=f"portrait_aabb={portrait_aabb}, portrait_dims={portrait_dims}",
    )

    with ctx.pose({display_to_button_1: button_travel}):
        pressed_button_pos = ctx.part_world_position(button_1)
        ctx.expect_contact(
            display,
            button_1,
            elem_a="control_strip",
            elem_b="button_cap",
            name="pressed button reaches the control strip opening",
        )
    ctx.check(
        "button plunger moves inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.0025,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
