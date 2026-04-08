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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_home_espresso_machine")

    stainless = model.material("stainless", rgba=(0.78, 0.79, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.14, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    soft_black = model.material("soft_black", rgba=(0.18, 0.18, 0.19, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.18, 0.22, 0.205)),
        origin=Origin(xyz=(-0.03, 0.0, 0.1575)),
        material=stainless,
        name="shell",
    )
    body.visual(
        Box((0.110, 0.220, 0.056)),
        origin=Origin(xyz=(-0.065, 0.0, 0.028)),
        material=dark_trim,
        name="rear_base",
    )
    body.visual(
        Box((0.160, 0.028, 0.056)),
        origin=Origin(xyz=(0.045, 0.096, 0.028)),
        material=dark_trim,
        name="left_rail",
    )
    body.visual(
        Box((0.160, 0.028, 0.056)),
        origin=Origin(xyz=(0.045, -0.096, 0.028)),
        material=dark_trim,
        name="right_rail",
    )
    body.visual(
        Box((0.042, 0.180, 0.025)),
        origin=Origin(xyz=(0.104, 0.0, 0.0625)),
        material=dark_trim,
        name="front_fascia",
    )
    body.visual(
        Box((0.150, 0.170, 0.008)),
        origin=Origin(xyz=(0.040, 0.0, 0.088)),
        material=soft_black,
        name="cup_deck",
    )
    body.visual(
        Box((0.060, 0.160, 0.085)),
        origin=Origin(xyz=(0.100, 0.0, 0.173)),
        material=stainless,
        name="front_nose",
    )
    body.visual(
        Box((0.030, 0.120, 0.080)),
        origin=Origin(xyz=(0.070, 0.0, 0.160)),
        material=stainless,
        name="nose_bridge",
    )
    body.visual(
        Cylinder(radius=0.033, length=0.036),
        origin=Origin(xyz=(0.098, 0.0, 0.120)),
        material=dark_trim,
        name="group_head_collar",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.080, -0.114, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="steam_support_boss",
    )
    body.visual(
        Box((0.028, 0.040, 0.040)),
        origin=Origin(xyz=(0.082, -0.094, 0.175)),
        material=dark_trim,
        name="steam_support_arm",
    )
    body.visual(
        Box((0.160, 0.010, 0.029)),
        origin=Origin(xyz=(-0.005, 0.085, 0.2735)),
        material=black_plastic,
        name="reservoir_left_wall",
    )
    body.visual(
        Box((0.160, 0.010, 0.029)),
        origin=Origin(xyz=(-0.005, -0.085, 0.2735)),
        material=black_plastic,
        name="reservoir_right_wall",
    )
    body.visual(
        Box((0.010, 0.180, 0.029)),
        origin=Origin(xyz=(-0.085, 0.0, 0.2735)),
        material=black_plastic,
        name="reservoir_back_wall",
    )
    body.visual(
        Box((0.010, 0.180, 0.029)),
        origin=Origin(xyz=(0.075, 0.0, 0.2735)),
        material=black_plastic,
        name="reservoir_front_wall",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.26, 0.24, 0.30)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )

    reservoir_lid = model.part("reservoir_lid")
    reservoir_lid.visual(
        Box((0.170, 0.186, 0.008)),
        origin=Origin(xyz=(0.085, 0.0, 0.004)),
        material=stainless,
        name="lid_panel",
    )
    reservoir_lid.visual(
        Box((0.022, 0.070, 0.014)),
        origin=Origin(xyz=(0.159, 0.0, 0.011)),
        material=black_plastic,
        name="lid_tab",
    )
    reservoir_lid.inertial = Inertial.from_geometry(
        Box((0.170, 0.186, 0.020)),
        mass=0.25,
        origin=Origin(xyz=(0.085, 0.0, 0.010)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.031, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=dark_trim,
        name="basket_cup",
    )
    portafilter.visual(
        Box((0.040, 0.018, 0.016)),
        origin=Origin(xyz=(0.036, 0.0, -0.018)),
        material=black_plastic,
        name="handle_neck",
    )
    portafilter.visual(
        Box((0.095, 0.024, 0.020)),
        origin=Origin(xyz=(0.095, 0.0, -0.032)),
        material=black_plastic,
        name="handle_grip",
    )
    portafilter.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.144, 0.0, -0.032), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="handle_end",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.160, 0.070, 0.050)),
        mass=0.45,
        origin=Origin(xyz=(0.070, 0.0, -0.020)),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="pivot_barrel",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.170),
        origin=Origin(xyz=(0.0, -0.006, -0.085)),
        material=stainless,
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.003, length=0.022),
        origin=Origin(xyz=(0.010, -0.006, -0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="nozzle_tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.180)),
        mass=0.18,
        origin=Origin(xyz=(0.002, -0.006, -0.085)),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.156, 0.164, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_trim,
        name="tray_base",
    )
    drip_tray.visual(
        Box((0.004, 0.164, 0.028)),
        origin=Origin(xyz=(0.076, 0.0, 0.014)),
        material=dark_trim,
        name="tray_front_wall",
    )
    drip_tray.visual(
        Box((0.004, 0.164, 0.018)),
        origin=Origin(xyz=(-0.076, 0.0, 0.009)),
        material=dark_trim,
        name="tray_back_wall",
    )
    drip_tray.visual(
        Box((0.148, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, 0.080, 0.014)),
        material=dark_trim,
        name="tray_left_wall",
    )
    drip_tray.visual(
        Box((0.148, 0.004, 0.028)),
        origin=Origin(xyz=(0.0, -0.080, 0.014)),
        material=dark_trim,
        name="tray_right_wall",
    )
    drip_tray.visual(
        Box((0.148, 0.160, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=soft_black,
        name="tray_grill",
    )
    drip_tray.visual(
        Box((0.020, 0.080, 0.012)),
        origin=Origin(xyz=(0.084, 0.0, 0.020)),
        material=black_plastic,
        name="tray_handle",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.160, 0.170, 0.032)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    model.articulation(
        "body_to_reservoir_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=reservoir_lid,
        origin=Origin(xyz=(-0.085, 0.0, 0.288)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.22,
        ),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.098, 0.0, 0.102)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.60,
            upper=0.60,
        ),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.080, -0.130, 0.175)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.045, 0.0, 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.12,
            lower=0.0,
            upper=0.080,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    reservoir_lid = object_model.get_part("reservoir_lid")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    drip_tray = object_model.get_part("drip_tray")

    lid_joint = object_model.get_articulation("body_to_reservoir_lid")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    steam_joint = object_model.get_articulation("body_to_steam_wand")
    tray_joint = object_model.get_articulation("body_to_drip_tray")

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            reservoir_lid,
            body,
            axis="z",
            max_gap=0.002,
            max_penetration=1e-6,
            name="reservoir lid closes onto the reservoir frame",
        )
        ctx.expect_overlap(
            reservoir_lid,
            body,
            axes="xy",
            min_overlap=0.150,
            name="reservoir lid covers the top opening footprint",
        )

    with ctx.pose({portafilter_joint: 0.0}):
        ctx.expect_gap(
            body,
            portafilter,
            axis="z",
            positive_elem="group_head_collar",
            negative_elem="basket_cup",
            max_gap=0.0015,
            max_penetration=0.0,
            name="portafilter basket seats against the group head",
        )
        ctx.expect_overlap(
            body,
            portafilter,
            axes="xy",
            elem_a="group_head_collar",
            elem_b="basket_cup",
            min_overlap=0.055,
            name="portafilter stays centered under the group head",
        )

    with ctx.pose({tray_joint: 0.0}):
        ctx.expect_within(
            drip_tray,
            body,
            axes="y",
            margin=0.0,
            name="drip tray stays centered between the body sides",
        )
        ctx.expect_gap(
            body,
            drip_tray,
            axis="z",
            positive_elem="cup_deck",
            negative_elem="tray_grill",
            min_gap=0.040,
            max_gap=0.060,
            name="cup platform sits above the tray grill",
        )

    with ctx.pose({steam_joint: 0.0}):
        ctx.expect_gap(
            body,
            steam_wand,
            axis="y",
            positive_elem="shell",
            max_gap=0.020,
            max_penetration=0.0,
            name="steam wand parks alongside the machine body",
        )

    def elem_center(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    lid_upper = lid_joint.motion_limits.upper if lid_joint.motion_limits is not None else None
    steam_upper = steam_joint.motion_limits.upper if steam_joint.motion_limits is not None else None
    tray_upper = tray_joint.motion_limits.upper if tray_joint.motion_limits is not None else None

    closed_lid_tab = None
    open_lid_tab = None
    if lid_upper is not None:
        with ctx.pose({lid_joint: 0.0}):
            closed_lid_tab = elem_center(reservoir_lid, "lid_tab")
        with ctx.pose({lid_joint: lid_upper}):
            open_lid_tab = elem_center(reservoir_lid, "lid_tab")
    ctx.check(
        "reservoir lid swings upward from the rear hinge",
        closed_lid_tab is not None
        and open_lid_tab is not None
        and open_lid_tab[2] > closed_lid_tab[2] + 0.090
        and open_lid_tab[0] < closed_lid_tab[0] - 0.040,
        details=f"closed={closed_lid_tab}, open={open_lid_tab}",
    )

    left_lock = None
    right_lock = None
    with ctx.pose({portafilter_joint: -0.55}):
        left_lock = elem_center(portafilter, "handle_end")
    with ctx.pose({portafilter_joint: 0.55}):
        right_lock = elem_center(portafilter, "handle_end")
    ctx.check(
        "portafilter handle twists through a locking arc",
        left_lock is not None
        and right_lock is not None
        and left_lock[1] < -0.040
        and right_lock[1] > 0.040,
        details=f"left={left_lock}, right={right_lock}",
    )

    stowed_nozzle = None
    opened_nozzle = None
    if steam_upper is not None:
        with ctx.pose({steam_joint: 0.0}):
            stowed_nozzle = elem_center(steam_wand, "nozzle_tip")
        with ctx.pose({steam_joint: steam_upper}):
            opened_nozzle = elem_center(steam_wand, "nozzle_tip")
    ctx.check(
        "steam wand swings outward away from the side panel",
        stowed_nozzle is not None
        and opened_nozzle is not None
        and opened_nozzle[1] < stowed_nozzle[1] - 0.080,
        details=f"stowed={stowed_nozzle}, open={opened_nozzle}",
    )

    inserted_tray = None
    extended_tray = None
    if tray_upper is not None:
        with ctx.pose({tray_joint: 0.0}):
            inserted_tray = elem_center(drip_tray, "tray_handle")
        with ctx.pose({tray_joint: tray_upper}):
            extended_tray = elem_center(drip_tray, "tray_handle")
            ctx.expect_overlap(
                body,
                drip_tray,
                axes="x",
                min_overlap=0.075,
                name="drip tray remains partially engaged when extended",
            )
    ctx.check(
        "drip tray slides forward from the machine base",
        inserted_tray is not None
        and extended_tray is not None
        and extended_tray[0] > inserted_tray[0] + 0.060,
        details=f"inserted={inserted_tray}, extended={extended_tray}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
