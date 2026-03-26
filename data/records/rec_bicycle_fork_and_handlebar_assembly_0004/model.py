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
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    kwargs = {"material": material}
    if name is not None:
        kwargs["name"] = name
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        **kwargs,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="urban_commuter_fork", assets=ASSETS)

    fork_paint = model.material("fork_paint", rgba=(0.12, 0.12, 0.13, 1.0))
    polished_alloy = model.material("polished_alloy", rgba=(0.74, 0.76, 0.79, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.11, 0.11, 0.11, 1.0))

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.18, 0.15, 0.40)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.03, 0.20)),
    )

    fork.visual(
        Cylinder(radius=0.014, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=fork_paint,
        name="steerer",
    )
    for index, z in enumerate((0.316, 0.324, 0.332, 0.340, 0.348, 0.356)):
        fork.visual(
            Cylinder(radius=0.0155, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=steel,
            name="thread_band" if index == 2 else None,
        )
    fork.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=steel,
    )
    fork.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.303)),
        material=polished_alloy,
    )
    fork.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.290)),
        material=steel,
    )

    fork.visual(
        Box((0.108, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, 0.002, 0.168)),
        material=fork_paint,
        name="fork_crown",
    )
    fork.visual(
        Box((0.028, 0.026, 0.020)),
        origin=Origin(xyz=(-0.044, 0.010, 0.155)),
        material=fork_paint,
    )
    fork.visual(
        Box((0.028, 0.026, 0.020)),
        origin=Origin(xyz=(0.044, 0.010, 0.155)),
        material=fork_paint,
    )
    fork.visual(
        Box((0.020, 0.016, 0.010)),
        origin=Origin(xyz=(-0.040, 0.025, 0.187)),
        material=polished_alloy,
        name="left_rack_boss",
    )
    fork.visual(
        Box((0.020, 0.016, 0.010)),
        origin=Origin(xyz=(0.040, 0.025, 0.187)),
        material=polished_alloy,
        name="right_rack_boss",
    )

    _add_member(
        fork,
        (-0.046, 0.004, 0.155),
        (-0.053, 0.056, 0.028),
        radius=0.0105,
        material=fork_paint,
        name="left_blade",
    )
    _add_member(
        fork,
        (0.046, 0.004, 0.155),
        (0.053, 0.056, 0.028),
        radius=0.0105,
        material=fork_paint,
        name="right_blade",
    )

    fork.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(-0.060, 0.033, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_alloy,
        name="left_lowrider_mount",
    )
    fork.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.060, 0.033, 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_alloy,
        name="right_lowrider_mount",
    )

    fork.visual(
        Box((0.018, 0.022, 0.030)),
        origin=Origin(xyz=(-0.053, 0.056, 0.013)),
        material=fork_paint,
        name="left_dropout",
    )
    fork.visual(
        Box((0.018, 0.022, 0.030)),
        origin=Origin(xyz=(0.053, 0.056, 0.013)),
        material=fork_paint,
        name="right_dropout",
    )

    _add_member(
        fork,
        (0.0, 0.0, 0.304),
        (0.0, 0.0, 0.386),
        radius=0.011,
        material=polished_alloy,
        name="quill_shaft",
    )
    _add_member(
        fork,
        (0.0, 0.0, 0.386),
        (0.0, 0.070, 0.377),
        radius=0.012,
        material=polished_alloy,
        name="stem_extension",
    )
    fork.visual(
        Box((0.050, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.086, 0.366)),
        material=polished_alloy,
        name="stem_cradle",
    )
    fork.visual(
        Box((0.008, 0.052, 0.026)),
        origin=Origin(xyz=(-0.028, 0.086, 0.381)),
        material=polished_alloy,
        name="left_stem_cheek",
    )
    fork.visual(
        Box((0.008, 0.052, 0.026)),
        origin=Origin(xyz=(0.028, 0.086, 0.381)),
        material=polished_alloy,
        name="right_stem_cheek",
    )
    fork.visual(
        Cylinder(radius=0.0045, length=0.072),
        origin=Origin(xyz=(0.0, 0.067, 0.381), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="front_clamp_bolt",
    )
    fork.visual(
        Cylinder(radius=0.0045, length=0.072),
        origin=Origin(xyz=(0.0, 0.105, 0.381), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_clamp_bolt",
    )

    rack = model.part("rack_platform")
    rack.inertial = Inertial.from_geometry(
        Box((0.12, 0.18, 0.12)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.09, 0.055)),
    )
    rack.visual(
        Box((0.086, 0.012, 0.006)),
        material=polished_alloy,
        name="lower_tie_bar",
    )
    rack.visual(
        Box((0.018, 0.014, 0.005)),
        origin=Origin(xyz=(-0.040, 0.0, 0.0025)),
        material=polished_alloy,
        name="left_rack_foot",
    )
    rack.visual(
        Box((0.018, 0.014, 0.005)),
        origin=Origin(xyz=(0.040, 0.0, 0.0025)),
        material=polished_alloy,
        name="right_rack_foot",
    )
    _add_member(
        rack,
        (-0.040, 0.0, 0.005),
        (-0.040, 0.035, 0.1075),
        radius=0.0055,
        material=polished_alloy,
        name="left_standoff",
    )
    _add_member(
        rack,
        (0.040, 0.0, 0.005),
        (0.040, 0.035, 0.1075),
        radius=0.0055,
        material=polished_alloy,
        name="right_standoff",
    )
    rack.visual(
        Box((0.104, 0.158, 0.005)),
        origin=Origin(xyz=(0.0, 0.095, 0.110)),
        material=polished_alloy,
        name="platform_deck",
    )
    for a, b in (
        ((-0.046, 0.018, 0.1125), (-0.046, 0.172, 0.1125)),
        ((0.046, 0.018, 0.1125), (0.046, 0.172, 0.1125)),
        ((-0.046, 0.018, 0.1125), (0.046, 0.018, 0.1125)),
        ((-0.046, 0.172, 0.1125), (0.046, 0.172, 0.1125)),
        ((-0.038, 0.060, 0.1125), (0.038, 0.060, 0.1125)),
        ((-0.038, 0.102, 0.1125), (0.038, 0.102, 0.1125)),
        ((-0.038, 0.144, 0.1125), (0.038, 0.144, 0.1125)),
    ):
        _add_member(rack, a, b, radius=0.0032, material=polished_alloy)

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.66, 0.10, 0.12)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )
    handlebar.visual(
        Cylinder(radius=0.011, length=0.120),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="center_section",
    )
    handlebar.visual(
        _save_mesh(
            "riser_handlebar.obj",
            tube_from_spline_points(
                [
                    (-0.305, -0.030, 0.050),
                    (-0.235, -0.022, 0.078),
                    (-0.150, -0.012, 0.042),
                    (-0.070, -0.004, 0.008),
                    (-0.055, 0.0, 0.0),
                    (0.055, 0.0, 0.0),
                    (0.070, 0.004, 0.008),
                    (0.150, 0.012, 0.042),
                    (0.235, 0.022, 0.078),
                    (0.305, 0.030, 0.050),
                ],
                radius=0.011,
                samples_per_segment=18,
                radial_segments=18,
            ),
        ),
        material=steel,
        name="riser_bar",
    )
    handlebar.visual(
        Cylinder(radius=0.015, length=0.095),
        origin=Origin(xyz=(-0.292, -0.028, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
    )
    handlebar.visual(
        Cylinder(radius=0.015, length=0.095),
        origin=Origin(xyz=(0.292, 0.028, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
    )

    axle = model.part("front_axle")
    axle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.009, length=0.116),
        mass=0.18,
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    axle.visual(
        Cylinder(radius=0.0055, length=0.116),
        origin=Origin(xyz=(0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_shaft",
    )
    axle.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_alloy,
    )
    axle.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.116, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_alloy,
    )

    model.articulation(
        "fork_to_rack",
        ArticulationType.FIXED,
        parent=fork,
        child=rack,
        origin=Origin(xyz=(0.0, 0.025, 0.192)),
    )
    model.articulation(
        "fork_to_handlebar",
        ArticulationType.FIXED,
        parent=fork,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.086, 0.382)),
    )
    model.articulation(
        "fork_to_axle",
        ArticulationType.FIXED,
        parent=fork,
        child=axle,
        origin=Origin(xyz=(-0.058, 0.056, 0.018)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    fork = object_model.get_part("fork")
    rack = object_model.get_part("rack_platform")
    handlebar = object_model.get_part("handlebar")
    axle = object_model.get_part("front_axle")

    steerer = fork.get_visual("steerer")
    thread_band = fork.get_visual("thread_band")
    crown = fork.get_visual("fork_crown")
    left_lowrider_mount = fork.get_visual("left_lowrider_mount")
    right_lowrider_mount = fork.get_visual("right_lowrider_mount")
    left_rack_boss = fork.get_visual("left_rack_boss")
    right_rack_boss = fork.get_visual("right_rack_boss")
    left_dropout = fork.get_visual("left_dropout")
    right_dropout = fork.get_visual("right_dropout")
    stem_cradle = fork.get_visual("stem_cradle")
    left_stem_cheek = fork.get_visual("left_stem_cheek")
    front_clamp_bolt = fork.get_visual("front_clamp_bolt")
    rear_clamp_bolt = fork.get_visual("rear_clamp_bolt")

    left_rack_foot = rack.get_visual("left_rack_foot")
    right_rack_foot = rack.get_visual("right_rack_foot")
    platform_deck = rack.get_visual("platform_deck")

    center_section = handlebar.get_visual("center_section")
    axle_shaft = axle.get_visual("axle_shaft")

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

    ctx.expect_overlap(
        fork,
        fork,
        axes="xy",
        min_overlap=0.025,
        elem_a=steerer,
        elem_b=crown,
        name="threaded_steerer_passes_through_fork_crown",
    )
    ctx.expect_gap(
        fork,
        fork,
        axis="z",
        min_gap=0.14,
        positive_elem=thread_band,
        negative_elem=crown,
        name="threaded_section_sits_above_the_crown",
    )
    ctx.expect_contact(
        fork,
        fork,
        elem_a=left_lowrider_mount,
        elem_b=fork.get_visual("left_blade"),
        name="left_lowrider_mount_is_brazed_to_left_blade",
    )
    ctx.expect_contact(
        fork,
        fork,
        elem_a=right_lowrider_mount,
        elem_b=fork.get_visual("right_blade"),
        name="right_lowrider_mount_is_brazed_to_right_blade",
    )
    ctx.expect_gap(
        fork,
        fork,
        axis="z",
        min_gap=0.050,
        positive_elem=left_lowrider_mount,
        negative_elem=left_dropout,
        name="left_lowrider_mount_sits_above_the_dropout",
    )
    ctx.expect_gap(
        fork,
        fork,
        axis="z",
        min_gap=0.060,
        positive_elem=crown,
        negative_elem=left_lowrider_mount,
        name="left_lowrider_mount_sits_below_the_crown",
    )
    ctx.expect_contact(
        rack,
        fork,
        elem_a=left_rack_foot,
        elem_b=left_rack_boss,
        name="left_rack_standoff_foot_seats_on_boss",
    )
    ctx.expect_contact(
        rack,
        fork,
        elem_a=right_rack_foot,
        elem_b=right_rack_boss,
        name="right_rack_standoff_foot_seats_on_boss",
    )
    ctx.expect_gap(
        rack,
        fork,
        axis="y",
        min_gap=0.015,
        positive_elem=platform_deck,
        negative_elem=crown,
        name="rack_platform_projects_forward_of_crown",
    )
    ctx.expect_gap(
        rack,
        axle,
        axis="z",
        min_gap=0.200,
        positive_elem=platform_deck,
        negative_elem=axle_shaft,
        name="rack_platform_sits_well_above_hub_axle",
    )
    ctx.expect_gap(
        handlebar,
        fork,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=center_section,
        negative_elem=stem_cradle,
        name="riser_handlebar_sits_in_quill_stem_cradle",
    )
    ctx.expect_overlap(
        handlebar,
        fork,
        axes="xy",
        min_overlap=0.020,
        elem_a=center_section,
        elem_b=stem_cradle,
        name="riser_handlebar_center_section_is_captured_by_stem_clamp",
    )
    ctx.expect_overlap(
        fork,
        fork,
        axes="yz",
        min_overlap=0.008,
        elem_a=front_clamp_bolt,
        elem_b=left_stem_cheek,
        name="front_side_clamp_bolt_passes_through_stem_cheek",
    )
    ctx.expect_overlap(
        fork,
        fork,
        axes="yz",
        min_overlap=0.008,
        elem_a=rear_clamp_bolt,
        elem_b=left_stem_cheek,
        name="rear_side_clamp_bolt_passes_through_stem_cheek",
    )
    ctx.expect_gap(
        fork,
        fork,
        axis="y",
        min_gap=0.025,
        positive_elem=rear_clamp_bolt,
        negative_elem=front_clamp_bolt,
        name="two_side_clamp_bolts_are_visibly_separate",
    )
    ctx.expect_overlap(
        axle,
        fork,
        axes="yz",
        min_overlap=0.008,
        elem_a=axle_shaft,
        elem_b=left_dropout,
        name="axle_seated_in_left_box_dropout",
    )
    ctx.expect_overlap(
        axle,
        fork,
        axes="yz",
        min_overlap=0.008,
        elem_a=axle_shaft,
        elem_b=right_dropout,
        name="axle_seated_in_right_box_dropout",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
