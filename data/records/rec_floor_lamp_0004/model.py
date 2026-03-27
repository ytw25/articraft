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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pharmacy_reading_lamp", assets=ASSETS)

    warm_brass = model.material("warm_brass", rgba=(0.72, 0.58, 0.31, 1.0))
    dark_brass = model.material("dark_brass", rgba=(0.43, 0.32, 0.16, 1.0))
    cream_enamel = model.material("cream_enamel", rgba=(0.92, 0.90, 0.82, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.20, 0.22, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.115, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=warm_brass,
        name="base_foot",
    )
    stand.visual(
        Cylinder(radius=0.098, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=graphite,
        name="base_pad",
    )
    stand.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=warm_brass,
        name="socket_flange",
    )
    stand.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=warm_brass,
        name="pole_socket",
    )
    stand.visual(
        Cylinder(radius=0.009, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.374)),
        material=warm_brass,
        name="pole",
    )
    stand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.74),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.010, 0.031, 0.040)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0)),
        material=warm_brass,
        name="collar_sleeve",
    )
    carriage.visual(
        Box((0.020, 0.009, 0.040)),
        origin=Origin(xyz=(-0.001, -0.0135, 0.0)),
        material=warm_brass,
        name="collar_left_half",
    )
    carriage.visual(
        Box((0.020, 0.009, 0.040)),
        origin=Origin(xyz=(-0.001, 0.0135, 0.0)),
        material=warm_brass,
        name="collar_right_half",
    )
    carriage.visual(
        Box((0.018, 0.022, 0.028)),
        origin=Origin(xyz=(0.018, 0.0, 0.003)),
        material=warm_brass,
        name="clamp_body",
    )
    carriage.visual(
        Cylinder(radius=0.0032, length=0.030),
        origin=Origin(xyz=(0.024, 0.0, 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="thumbscrew_shaft",
    )
    carriage.visual(
        Cylinder(radius=0.0062, length=0.006),
        origin=Origin(xyz=(0.042, 0.0, 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_brass,
        name="thumbscrew_head",
    )
    carriage.visual(
        Cylinder(radius=0.0065, length=0.134),
        origin=Origin(xyz=(0.094, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="arm_tube",
    )
    carriage.visual(
        Box((0.008, 0.018, 0.012)),
        origin=Origin(xyz=(0.157, 0.0, 0.010)),
        material=warm_brass,
        name="arm_tip_block",
    )
    carriage.visual(
        Box((0.008, 0.007, 0.030)),
        origin=Origin(xyz=(0.165, -0.011, 0.010)),
        material=dark_brass,
        name="left_pivot_cheek",
    )
    carriage.visual(
        Box((0.008, 0.007, 0.030)),
        origin=Origin(xyz=(0.165, 0.011, 0.010)),
        material=dark_brass,
        name="right_pivot_cheek",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.19, 0.05, 0.05)),
        mass=0.75,
        origin=Origin(xyz=(0.095, 0.0, 0.010)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.006, length=0.019),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_brass,
        name="pivot_barrel",
    )
    shade.visual(
        Cylinder(radius=0.0075, length=0.018),
        origin=Origin(xyz=(0.013, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_brass,
        name="shade_neck",
    )
    shade.visual(
        _save_mesh(
            "pharmacy_shade_shell.obj",
            LatheGeometry.from_shell_profiles(
                [
                    (0.010, -0.008),
                    (0.020, -0.004),
                    (0.034, 0.008),
                    (0.048, 0.021),
                    (0.058, 0.034),
                    (0.062, 0.043),
                ],
                [
                    (0.0, -0.007),
                    (0.009, -0.007),
                    (0.016, -0.003),
                    (0.028, 0.008),
                    (0.042, 0.022),
                    (0.054, 0.035),
                    (0.058, 0.040),
                ],
                segments=52,
                start_cap="flat",
                end_cap="flat",
            ).rotate_y(math.pi / 2.0),
        ),
        origin=Origin(xyz=(0.020, 0.0, -0.019)),
        material=cream_enamel,
        name="shade_shell",
    )
    shade.visual(
        _save_mesh(
            "pharmacy_shade_lower_rim.obj",
            tube_from_spline_points(
                [
                    (0.046, -0.038, -0.054),
                    (0.052, -0.024, -0.059),
                    (0.055, 0.0, -0.062),
                    (0.052, 0.024, -0.059),
                    (0.046, 0.038, -0.054),
                ],
                radius=0.0035,
                samples_per_segment=16,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        material=cream_enamel,
        name="shade_lower_rim",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.10, 0.11, 0.07)),
        mass=0.32,
        origin=Origin(xyz=(0.030, 0.0, -0.010)),
    )

    model.articulation(
        "collar_slide",
        ArticulationType.PRISMATIC,
        parent=stand,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.44,
        ),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shade,
        origin=Origin(xyz=(0.167, 0.0, 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.4,
            lower=-0.80,
            upper=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    stand = object_model.get_part("stand")
    carriage = object_model.get_part("carriage")
    shade = object_model.get_part("shade")
    collar_slide = object_model.get_articulation("collar_slide")
    shade_tilt = object_model.get_articulation("shade_tilt")

    base_foot = stand.get_visual("base_foot")
    pole = stand.get_visual("pole")
    collar_sleeve = carriage.get_visual("collar_sleeve")
    thumbscrew_shaft = carriage.get_visual("thumbscrew_shaft")
    arm_tube = carriage.get_visual("arm_tube")
    left_pivot_cheek = carriage.get_visual("left_pivot_cheek")
    right_pivot_cheek = carriage.get_visual("right_pivot_cheek")
    pivot_barrel = shade.get_visual("pivot_barrel")
    shade_shell = shade.get_visual("shade_shell")
    shade_lower_rim = shade.get_visual("shade_lower_rim")

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

    ctx.expect_origin_distance(
        carriage,
        stand,
        axes="xy",
        max_dist=0.0015,
        name="collar_stays_centered_on_pole",
    )
    ctx.expect_contact(
        carriage,
        stand,
        elem_a=thumbscrew_shaft,
        elem_b=pole,
        name="thumbscrew_locks_against_pole",
    )
    ctx.expect_gap(
        carriage,
        stand,
        axis="z",
        min_gap=0.07,
        positive_elem=collar_sleeve,
        negative_elem=base_foot,
        name="collar_sits_above_round_base",
    )
    ctx.expect_gap(
        shade,
        stand,
        axis="x",
        min_gap=0.14,
        positive_elem=shade_shell,
        negative_elem=pole,
        name="shade_projects_forward_from_pole",
    )
    ctx.expect_gap(
        shade,
        carriage,
        axis="x",
        min_gap=0.016,
        positive_elem=shade_shell,
        negative_elem=arm_tube,
        name="shade_shell_starts_beyond_short_horizontal_arm",
    )
    ctx.expect_contact(
        carriage,
        shade,
        elem_a=left_pivot_cheek,
        elem_b=pivot_barrel,
        name="left_cheek_seats_pivot_barrel",
    )
    ctx.expect_contact(
        carriage,
        shade,
        elem_a=right_pivot_cheek,
        elem_b=pivot_barrel,
        name="right_cheek_seats_pivot_barrel",
    )

    with ctx.pose({collar_slide: 0.40}):
        ctx.expect_origin_distance(
            carriage,
            stand,
            axes="xy",
            max_dist=0.0015,
            name="raised_collar_stays_centered_on_pole",
        )
        ctx.expect_contact(
            carriage,
            stand,
            elem_a=thumbscrew_shaft,
            elem_b=pole,
            name="raised_thumbscrew_still_contacts_pole",
        )
        ctx.expect_gap(
            carriage,
            stand,
            axis="z",
            min_gap=0.46,
            positive_elem=collar_sleeve,
            negative_elem=base_foot,
            name="collar_can_slide_up_pole",
        )
        ctx.expect_gap(
            shade,
            stand,
            axis="x",
            min_gap=0.14,
            positive_elem=shade_shell,
            negative_elem=pole,
            name="raised_shade_remains_outboard",
        )

    with ctx.pose({shade_tilt: 0.50}):
        ctx.expect_contact(
            carriage,
            shade,
            elem_a=left_pivot_cheek,
            elem_b=pivot_barrel,
            name="pivot_stays_seated_while_tilted",
        )
        ctx.expect_contact(
            carriage,
            shade,
            elem_a=right_pivot_cheek,
            elem_b=pivot_barrel,
            name="right_pivot_stays_seated_while_tilted",
        )
        ctx.expect_gap(
            carriage,
            shade,
            axis="z",
            min_gap=0.018,
            positive_elem=arm_tube,
            negative_elem=shade_lower_rim,
            name="shade_can_tilt_below_arm",
        )
        ctx.expect_gap(
            shade,
            stand,
            axis="x",
            min_gap=0.09,
            positive_elem=shade_shell,
            negative_elem=pole,
            name="tilted_shade_stays_forward_of_pole",
        )

    with ctx.pose({collar_slide: 0.30, shade_tilt: 0.50}):
        ctx.expect_gap(
            shade,
            stand,
            axis="z",
            min_gap=0.14,
            positive_elem=shade_shell,
            negative_elem=base_foot,
            name="raised_shade_clears_base",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
