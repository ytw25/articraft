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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _loop_xy(
    width: float,
    depth: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    radius: float | None = None,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    corner = radius if radius is not None else min(width, depth) * 0.24
    return [
        (x + center_x, y + center_y, z)
        for x, y in rounded_rect_profile(width, depth, corner, corner_segments=corner_segments)
    ]


def _loop_xz(
    width: float,
    height: float,
    y: float,
    *,
    center_x: float = 0.0,
    center_z: float = 0.0,
    radius: float | None = None,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    corner = radius if radius is not None else min(width, height) * 0.22
    return [
        (x + center_x, y, z + center_z)
        for x, z in rounded_rect_profile(width, height, corner, corner_segments=corner_segments)
    ]


def _build_base_shell_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _loop_xy(0.270, 0.240, 0.000, center_y=0.028, radius=0.040),
                _loop_xy(0.252, 0.226, 0.018, center_y=0.027, radius=0.038),
                _loop_xy(0.220, 0.192, 0.043, center_y=0.023, radius=0.032),
                _loop_xy(0.188, 0.158, 0.058, center_y=0.018, radius=0.026),
            ]
        )
    )


def _build_column_shell_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _loop_xy(0.120, 0.092, 0.036, center_y=-0.050, radius=0.020),
                _loop_xy(0.108, 0.080, 0.120, center_y=-0.060, radius=0.018),
                _loop_xy(0.094, 0.066, 0.212, center_y=-0.072, radius=0.016),
                _loop_xy(0.086, 0.054, 0.282, center_y=-0.082, radius=0.014),
            ]
        )
    )


def _build_head_shell_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _loop_xz(0.094, 0.080, -0.020, center_z=0.024, radius=0.016),
                _loop_xz(0.126, 0.112, 0.020, center_z=0.030, radius=0.020),
                _loop_xz(0.138, 0.116, 0.080, center_z=0.032, radius=0.022),
                _loop_xz(0.128, 0.108, 0.132, center_z=0.029, radius=0.021),
                _loop_xz(0.086, 0.082, 0.175, center_z=0.020, radius=0.016),
            ]
        )
    )


def _build_bowl_shell_mesh() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.040, 0.000),
            (0.064, 0.010),
            (0.094, 0.028),
            (0.116, 0.064),
            (0.124, 0.094),
            (0.121, 0.106),
        ],
        [
            (0.000, 0.006),
            (0.040, 0.006),
            (0.070, 0.018),
            (0.100, 0.052),
            (0.112, 0.088),
            (0.110, 0.102),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def _build_bowl_handle_mesh() -> MeshGeometry:
    return tube_from_spline_points(
        [
            (0.111, 0.000, 0.080),
            (0.145, 0.002, 0.080),
            (0.156, 0.002, 0.058),
            (0.151, 0.001, 0.034),
            (0.118, 0.000, 0.031),
        ],
        radius=0.0048,
        samples_per_segment=14,
        radial_segments=16,
    )


def _build_beater_mesh() -> MeshGeometry:
    outer_frame = wire_from_points(
        [
            (0.000, 0.000, 0.000),
            (0.026, 0.000, -0.022),
            (0.026, 0.000, -0.062),
            (0.013, 0.000, -0.108),
            (-0.013, 0.000, -0.108),
            (-0.026, 0.000, -0.062),
            (-0.026, 0.000, -0.022),
        ],
        radius=0.0038,
        radial_segments=14,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.014,
        corner_segments=8,
    )
    center_bar = tube_from_spline_points(
        [(0.000, 0.000, 0.000), (0.000, 0.000, -0.108)],
        radius=0.0032,
        samples_per_segment=2,
        radial_segments=14,
    )
    left_rib = tube_from_spline_points(
        [(0.000, 0.000, -0.010), (0.010, 0.000, -0.050), (0.012, 0.000, -0.092)],
        radius=0.0030,
        samples_per_segment=8,
        radial_segments=12,
    )
    right_rib = tube_from_spline_points(
        [(0.000, 0.000, -0.010), (-0.010, 0.000, -0.050), (-0.012, 0.000, -0.092)],
        radius=0.0030,
        samples_per_segment=8,
        radial_segments=12,
    )
    return _merge_geometries([outer_frame, center_bar, left_rib, right_rib])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer", assets=ASSETS)

    enamel = model.material("enamel", rgba=(0.66, 0.12, 0.11, 1.0))
    trim = model.material("trim", rgba=(0.80, 0.82, 0.85, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.33, 0.36, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        _save_mesh("mixer_base_shell.obj", _build_base_shell_mesh()),
        material=enamel,
        name="base_shell",
    )
    base.visual(
        _save_mesh("mixer_column_shell.obj", _build_column_shell_mesh()),
        material=enamel,
        name="column_shell",
    )
    base.visual(
        Cylinder(radius=0.076, length=0.012),
        origin=Origin(xyz=(0.0, 0.028, 0.062)),
        material=steel,
        name="bowl_plate",
    )
    base.visual(
        Box((0.100, 0.060, 0.026)),
        origin=Origin(xyz=(0.0, -0.082, 0.276)),
        material=enamel,
        name="hinge_block",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, -0.012, -0.002)),
        material=rubber,
        name="rear_foot",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.27, 0.24, 0.32)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        _save_mesh("mixer_bowl_shell.obj", _build_bowl_shell_mesh()),
        material=steel,
        name="bowl_shell",
    )
    bowl.visual(
        _save_mesh(
            "mixer_bowl_rim.obj",
            TorusGeometry(
                radius=0.1175,
                tube=0.0052,
                radial_segments=14,
                tubular_segments=56,
            ).translate(0.0, 0.0, 0.103)
        ),
        material=trim,
        name="rim_ring",
    )
    bowl.visual(
        Cylinder(radius=0.050, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="foot_ring",
    )
    bowl.visual(
        _save_mesh("mixer_bowl_handle.obj", _build_bowl_handle_mesh()),
        material=steel,
        name="handle",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.125, length=0.112),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.031, length=0.108),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel,
        name="rear_neck",
    )
    head.visual(
        _save_mesh("mixer_head_shell.obj", _build_head_shell_mesh()),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Box((0.080, 0.034, 0.016)),
        origin=Origin(xyz=(0.0, -0.006, -0.001)),
        material=enamel,
        name="rear_pad",
    )
    head.visual(
        Cylinder(radius=0.057, length=0.010),
        origin=Origin(xyz=(0.0, 0.104, 0.009), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="trim_band",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.182, 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="front_cap",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.034),
        origin=Origin(xyz=(0.0, 0.096, -0.051)),
        material=trim,
        name="planetary_hub",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.015),
        origin=Origin(xyz=(0.0, 0.096, -0.075)),
        material=trim,
        name="hub_collar",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.072),
        origin=Origin(xyz=(0.0, 0.096, -0.111)),
        material=dark_steel,
        name="shaft",
    )
    head.visual(
        _save_mesh("mixer_beater.obj", _build_beater_mesh()),
        origin=Origin(xyz=(0.0, 0.096, -0.147)),
        material=dark_steel,
        name="beater",
    )
    head.visual(
        Box((0.014, 0.030, 0.010)),
        origin=Origin(xyz=(0.061, 0.030, 0.042)),
        material=trim,
        name="speed_lever",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.15, 0.22, 0.12)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.080, 0.020)),
    )

    model.articulation(
        "bowl_mount",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.028, 0.066)),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, -0.082, 0.288)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    head_tilt = object_model.get_articulation("head_tilt")

    bowl_plate = base.get_visual("bowl_plate")
    hinge_block = base.get_visual("hinge_block")
    bowl_rim = bowl.get_visual("rim_ring")
    bowl_foot = bowl.get_visual("foot_ring")
    bowl_shell = bowl.get_visual("bowl_shell")
    rear_pad = head.get_visual("rear_pad")
    head_shell = head.get_visual("head_shell")
    planetary_hub = head.get_visual("planetary_hub")
    beater = head.get_visual("beater")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual connectivity gate for floating/disconnected subassemblies inside one part.
    ctx.fail_if_part_contains_disconnected_geometry_islands()

    # Encode the actual visual/mechanical claims with prompt-specific exact checks.
    # If you add a warning-tier heuristic and it fires, investigate it with
    # `probe_model` before editing geometry or relaxing thresholds.
    # Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is
    # genuinely uncertain or mechanically important.
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
    # If the object has a mounted subassembly, prefer exact `expect_contact(...)`,
    # `expect_gap(...)`, `expect_overlap(...)`, and `expect_within(...)` checks on
    # named local features over broad warning-tier heuristics.
    ctx.expect_overlap(
        bowl,
        base,
        axes="xy",
        min_overlap=0.090,
        elem_a=bowl_foot,
        elem_b=bowl_plate,
        name="bowl_foot_sits_over_pedestal_plate",
    )
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.003,
        max_penetration=0.004,
        positive_elem=bowl_foot,
        negative_elem=bowl_plate,
        name="bowl_foot_sits_seated_on_pedestal_plate",
    )
    ctx.expect_origin_distance(
        bowl,
        base,
        axes="x",
        max_dist=0.010,
        name="bowl_is_centered_on_mixer_base",
    )
    ctx.expect_overlap(
        head,
        bowl,
        axes="xy",
        min_overlap=0.120,
        elem_a=head_shell,
        elem_b=bowl_shell,
        name="tilt_head_covers_bowl_work_area",
    )
    ctx.expect_within(
        head,
        bowl,
        axes="xy",
        inner_elem=beater,
        outer_elem=bowl_rim,
        name="beater_hangs_inside_bowl_opening_at_rest",
    )
    ctx.expect_overlap(
        head,
        base,
        axes="x",
        min_overlap=0.070,
        elem_a=rear_pad,
        elem_b=hinge_block,
        name="head_remains_nested_over_hinge_block",
    )
    ctx.expect_origin_distance(
        head,
        bowl,
        axes="x",
        max_dist=0.012,
        name="head_and_bowl_share_centerline",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=0.010,
        positive_elem=planetary_hub,
        negative_elem=bowl_rim,
        name="planetary_hub_stays_above_bowl_rim",
    )
    with ctx.pose({head_tilt: 1.00}):
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            min_gap=0.040,
            positive_elem=beater,
            negative_elem=bowl_rim,
            name="tilted_head_lifts_beater_clear_of_bowl",
        )
        ctx.expect_overlap(
            head,
            base,
            axes="x",
            min_overlap=0.060,
            elem_a=rear_pad,
            elem_b=hinge_block,
            name="tilted_head_stays_registered_at_hinge",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
