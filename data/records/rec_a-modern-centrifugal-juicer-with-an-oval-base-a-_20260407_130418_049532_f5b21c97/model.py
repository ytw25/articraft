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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _oval_plate_mesh(width: float, depth: float, height: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            superellipse_profile(width, depth, exponent=2.5, segments=56),
            height,
            center=True,
            cap=True,
            closed=True,
        ),
        name,
    )


def _oval_ring_mesh(
    outer_width: float,
    outer_depth: float,
    inner_width: float,
    inner_depth: float,
    height: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            superellipse_profile(outer_width, outer_depth, exponent=2.5, segments=56),
            [superellipse_profile(inner_width, inner_depth, exponent=2.25, segments=56)],
            height,
            center=True,
            cap=True,
            closed=True,
        ),
        name,
    )


def _rounded_rect_ring_mesh(
    outer_width: float,
    outer_depth: float,
    inner_width: float,
    inner_depth: float,
    height: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(
                outer_width,
                outer_depth,
                radius=min(outer_width, outer_depth) * 0.16,
                corner_segments=8,
            ),
            [
                rounded_rect_profile(
                    inner_width,
                    inner_depth,
                    radius=min(inner_width, inner_depth) * 0.14,
                    corner_segments=8,
                )
            ],
            height,
            center=True,
            cap=True,
            closed=True,
        ),
        name,
    )


def _cover_section(width: float, height: float, y: float, z_center: float):
    profile = superellipse_profile(width, height, exponent=2.3, segments=56)
    return [(x, y, z_center + z) for x, z in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="centrifugal_juicer")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.72, 0.79, 0.82, 0.38))
    basket_steel = model.material("basket_steel", rgba=(0.82, 0.84, 0.86, 1.0))
    accent_black = model.material("accent_black", rgba=(0.11, 0.11, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        _oval_plate_mesh(0.300, 0.220, 0.012, "juicer_base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_dark,
        name="base_plate",
    )
    body.visual(
        _oval_ring_mesh(0.300, 0.220, 0.186, 0.166, 0.040, "juicer_lower_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=body_white,
        name="lower_shell",
    )
    body.visual(
        Box((0.252, 0.018, 0.100)),
        origin=Origin(xyz=(0.0, 0.093, 0.102)),
        material=body_white,
        name="main_shell",
    )
    body.visual(
        Box((0.252, 0.018, 0.100)),
        origin=Origin(xyz=(0.0, -0.093, 0.102)),
        material=body_white,
        name="main_shell_rear",
    )
    body.visual(
        Box((0.018, 0.186, 0.100)),
        origin=Origin(xyz=(-0.126, 0.0, 0.102)),
        material=body_white,
        name="main_shell_left",
    )
    body.visual(
        Box((0.018, 0.186, 0.100)),
        origin=Origin(xyz=(0.126, 0.0, 0.102)),
        material=body_white,
        name="main_shell_right",
    )
    body.visual(
        Box((0.188, 0.014, 0.034)),
        origin=Origin(xyz=(0.0, 0.082, 0.169)),
        material=body_white,
        name="shoulder_shell",
    )
    body.visual(
        Box((0.188, 0.014, 0.034)),
        origin=Origin(xyz=(0.0, -0.082, 0.169)),
        material=body_white,
        name="shoulder_shell_rear",
    )
    body.visual(
        Box((0.014, 0.164, 0.034)),
        origin=Origin(xyz=(-0.101, 0.0, 0.169)),
        material=body_white,
        name="shoulder_shell_left",
    )
    body.visual(
        Box((0.014, 0.164, 0.034)),
        origin=Origin(xyz=(0.101, 0.0, 0.169)),
        material=body_white,
        name="shoulder_shell_right",
    )
    body.visual(
        _oval_ring_mesh(0.188, 0.150, 0.128, 0.100, 0.010, "juicer_top_rim"),
        origin=Origin(xyz=(0.0, 0.0, 0.191)),
        material=trim_silver,
        name="top_rim",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=trim_silver,
        name="motor_pedestal",
    )
    body.visual(
        Box((0.040, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, 0.108, 0.136)),
        material=trim_silver,
        name="spout_body",
    )
    body.visual(
        Box((0.028, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.126, 0.126), rpy=(-0.24, 0.0, 0.0)),
        material=trim_silver,
        name="spout_lip",
    )
    body.visual(
        Box((0.034, 0.024, 0.026)),
        origin=Origin(xyz=(-0.055, -0.090, 0.209)),
        material=body_white,
        name="rear_hinge_left",
    )
    body.visual(
        Box((0.034, 0.024, 0.026)),
        origin=Origin(xyz=(0.055, -0.090, 0.209)),
        material=body_white,
        name="rear_hinge_right",
    )
    body.visual(
        Box((0.146, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, -0.085, 0.184)),
        material=body_white,
        name="rear_hinge_bridge",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.146, 0.020, 0.138), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_dark,
        name="lock_pivot_boss",
    )
    body.visual(
        Box((0.040, 0.030, 0.016)),
        origin=Origin(xyz=(0.121, 0.020, 0.146)),
        material=body_dark,
        name="lock_pivot_rib",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.008),
        origin=Origin(xyz=(0.0, 0.100, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_white,
        name="control_bezel",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.300, 0.220, 0.222)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.010, 0.000),
                    (0.040, 0.000),
                    (0.047, 0.008),
                    (0.051, 0.040),
                    (0.046, 0.058),
                    (0.035, 0.072),
                ],
                [
                    (0.000, 0.004),
                    (0.034, 0.004),
                    (0.041, 0.010),
                    (0.043, 0.042),
                    (0.037, 0.058),
                    (0.028, 0.068),
                ],
                segments=60,
            ),
            "juicer_basket_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=basket_steel,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.045, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=basket_steel,
        name="cutting_disc",
    )
    basket.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=trim_silver,
        name="drive_hub",
    )
    basket.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(0.034, 0.0, 0.128)),
        material=accent_black,
        name="basket_marker",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.100),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
    )
    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=35.0),
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _cover_section(0.150, 0.018, 0.000, 0.012),
                    _cover_section(0.210, 0.042, 0.055, 0.028),
                    _cover_section(0.220, 0.040, 0.110, 0.030),
                    _cover_section(0.190, 0.022, 0.150, 0.014),
                ]
            ),
            "juicer_cover_shell",
        ),
        material=smoked_clear,
        name="cover_shell",
    )
    cover.visual(
        _rounded_rect_ring_mesh(0.094, 0.068, 0.062, 0.040, 0.020, "juicer_chute_collar"),
        origin=Origin(xyz=(0.0, 0.072, 0.055)),
        material=smoked_clear,
        name="chute_collar",
    )
    cover.visual(
        Box((0.084, 0.010, 0.140)),
        origin=Origin(xyz=(0.0, 0.097, 0.135)),
        material=smoked_clear,
        name="chute_front_wall",
    )
    cover.visual(
        Box((0.084, 0.010, 0.140)),
        origin=Origin(xyz=(0.0, 0.047, 0.135)),
        material=smoked_clear,
        name="chute_back_wall",
    )
    cover.visual(
        Box((0.010, 0.040, 0.140)),
        origin=Origin(xyz=(-0.037, 0.072, 0.135)),
        material=smoked_clear,
        name="chute_left_wall",
    )
    cover.visual(
        Box((0.010, 0.040, 0.140)),
        origin=Origin(xyz=(0.037, 0.072, 0.135)),
        material=smoked_clear,
        name="chute_right_wall",
    )
    cover.visual(
        Box((0.170, 0.022, 0.014)),
        origin=Origin(xyz=(0.0, 0.150, 0.012)),
        material=smoked_clear,
        name="front_lip",
    )
    cover.visual(
        Box((0.018, 0.024, 0.016)),
        origin=Origin(xyz=(0.094, 0.144, 0.018)),
        material=accent_black,
        name="latch_receiver",
    )
    cover.visual(
        Box((0.021, 0.028, 0.026)),
        origin=Origin(xyz=(-0.0275, -0.014, 0.016)),
        material=smoked_clear,
        name="hinge_ear_left",
    )
    cover.visual(
        Box((0.021, 0.028, 0.026)),
        origin=Origin(xyz=(0.0275, -0.014, 0.016)),
        material=smoked_clear,
        name="hinge_ear_right",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.220, 0.170, 0.220)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.080, 0.080)),
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, -0.076, 0.193)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=1.15,
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.040, 0.022, 0.140)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=body_white,
        name="pusher_stem",
    )
    pusher.visual(
        Box((0.072, 0.050, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=body_dark,
        name="pusher_handle",
    )
    pusher.visual(
        Box((0.034, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=body_white,
        name="pusher_tip",
    )
    pusher.inertial = Inertial.from_geometry(
        Box((0.072, 0.050, 0.208)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
    )
    model.articulation(
        "cover_to_pusher",
        ArticulationType.PRISMATIC,
        parent=cover,
        child=pusher,
        origin=Origin(xyz=(0.0, 0.072, 0.230)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.18,
            lower=0.0,
            upper=0.070,
        ),
    )

    locking_arm = model.part("locking_arm")
    locking_arm.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_dark,
        name="pivot_hub",
    )
    locking_arm.visual(
        Box((0.016, 0.086, 0.016)),
        origin=Origin(xyz=(0.015, 0.032, 0.038), rpy=(0.62, 0.0, 0.0)),
        material=body_dark,
        name="arm_bar",
    )
    locking_arm.visual(
        Box((0.028, 0.032, 0.018)),
        origin=Origin(xyz=(0.026, 0.050, 0.072)),
        material=body_dark,
        name="clamp_head",
    )
    locking_arm.visual(
        Box((0.018, 0.010, 0.020)),
        origin=Origin(xyz=(0.028, 0.060, 0.062)),
        material=trim_silver,
        name="clamp_hook",
    )
    locking_arm.inertial = Inertial.from_geometry(
        Box((0.052, 0.090, 0.090)),
        mass=0.14,
        origin=Origin(xyz=(0.018, 0.032, 0.038)),
    )
    model.articulation(
        "body_to_locking_arm",
        ArticulationType.REVOLUTE,
        parent=body,
        child=locking_arm,
        origin=Origin(xyz=(0.145, 0.028, 0.138)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=-1.05,
            upper=0.0,
        ),
    )

    mode_knob = model.part("mode_knob")
    mode_knob.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_silver,
        name="shaft",
    )
    mode_knob.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.0, 0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="dial",
    )
    mode_knob.visual(
        Box((0.012, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.043, 0.022)),
        material=trim_silver,
        name="indicator",
    )
    mode_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.024),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.029, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_mode_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=mode_knob,
        origin=Origin(xyz=(0.0, 0.104, 0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=4.0,
            lower=-1.5,
            upper=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    pusher = object_model.get_part("pusher")
    locking_arm = object_model.get_part("locking_arm")
    mode_knob = object_model.get_part("mode_knob")
    basket = object_model.get_part("basket")

    cover_joint = object_model.get_articulation("body_to_cover")
    pusher_joint = object_model.get_articulation("cover_to_pusher")
    arm_joint = object_model.get_articulation("body_to_locking_arm")
    knob_joint = object_model.get_articulation("body_to_mode_knob")
    basket_joint = object_model.get_articulation("body_to_basket")

    ctx.allow_overlap(
        body,
        locking_arm,
        elem_a="lock_pivot_boss",
        elem_b="pivot_hub",
        reason="The locking arm pivots on the side boss, and the hub is intentionally modeled as a coaxial sleeve around that boss.",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        )

    with ctx.pose({cover_joint: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="z",
            positive_elem="front_lip",
            negative_elem="top_rim",
            min_gap=0.0,
            max_gap=0.020,
            name="cover front lip rests just above the body rim",
        )
        ctx.expect_within(
            pusher,
            cover,
            axes="xy",
            inner_elem="pusher_stem",
            outer_elem="chute_collar",
            margin=0.006,
            name="pusher stem stays guided within the chute footprint",
        )

    with ctx.pose({cover_joint: 0.0}):
        closed_front_lip = ctx.part_element_world_aabb(cover, elem="front_lip")
    with ctx.pose({cover_joint: 0.95}):
        open_front_lip = ctx.part_element_world_aabb(cover, elem="front_lip")
    ctx.check(
        "cover opens upward on the rear hinge",
        closed_front_lip is not None
        and open_front_lip is not None
        and open_front_lip[1][2] > closed_front_lip[1][2] + 0.08,
        details=f"closed={closed_front_lip}, open={open_front_lip}",
    )

    with ctx.pose({cover_joint: 0.0, pusher_joint: 0.0}):
        pusher_rest = ctx.part_element_world_aabb(pusher, elem="pusher_tip")
    with ctx.pose({cover_joint: 0.0, pusher_joint: 0.060}):
        pusher_pressed = ctx.part_element_world_aabb(pusher, elem="pusher_tip")
    ctx.check(
        "feed pusher slides downward into the chute",
        pusher_rest is not None
        and pusher_pressed is not None
        and pusher_pressed[0][2] < pusher_rest[0][2] - 0.045,
        details=f"rest={pusher_rest}, pressed={pusher_pressed}",
    )

    with ctx.pose({arm_joint: 0.0}):
        arm_clamped = ctx.part_element_world_aabb(locking_arm, elem="clamp_hook")
    with ctx.pose({arm_joint: -0.95}):
        arm_open = ctx.part_element_world_aabb(locking_arm, elem="clamp_hook")
    ctx.check(
        "locking arm can swing down away from the lid",
        arm_clamped is not None
        and arm_open is not None
        and arm_open[1][2] < arm_clamped[1][2] - 0.06,
        details=f"clamped={arm_clamped}, open={arm_open}",
    )

    with ctx.pose({knob_joint: -0.80}):
        knob_left = _aabb_center(ctx.part_element_world_aabb(mode_knob, elem="indicator"))
    with ctx.pose({knob_joint: 0.80}):
        knob_right = _aabb_center(ctx.part_element_world_aabb(mode_knob, elem="indicator"))
    ctx.check(
        "mode knob indicator rotates around the front shaft",
        knob_left is not None
        and knob_right is not None
        and (
            abs(knob_left[0] - knob_right[0]) > 0.012
            or abs(knob_left[2] - knob_right[2]) > 0.012
        ),
        details=f"left={knob_left}, right={knob_right}",
    )

    with ctx.pose({basket_joint: 0.0}):
        basket_mark_0 = _aabb_center(ctx.part_element_world_aabb(basket, elem="basket_marker"))
    with ctx.pose({basket_joint: 1.20}):
        basket_mark_1 = _aabb_center(ctx.part_element_world_aabb(basket, elem="basket_marker"))
    ctx.check(
        "basket spins about the vertical motor axis",
        basket_mark_0 is not None
        and basket_mark_1 is not None
        and (
            abs(basket_mark_0[0] - basket_mark_1[0]) > 0.012
            or abs(basket_mark_0[1] - basket_mark_1[1]) > 0.012
        ),
        details=f"pose0={basket_mark_0}, pose1={basket_mark_1}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
