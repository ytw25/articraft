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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    superellipse_side_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rounded_section(
    width: float,
    depth: float,
    *,
    z: float,
    y_shift: float = 0.0,
    radius_scale: float = 0.26,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    radius = min(width, depth) * radius_scale
    return [
        (x, y + y_shift, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius=radius,
            corner_segments=corner_segments,
        )
    ]


def _build_base_foot_mesh():
    return ExtrudeGeometry(
        rounded_rect_profile(0.270, 0.242, radius=0.050, corner_segments=12),
        height=0.028,
        center=False,
    )


def _build_column_mesh():
    return repair_loft(
        section_loft(
            [
                _rounded_section(0.110, 0.076, z=0.018, y_shift=-0.052, radius_scale=0.23),
                _rounded_section(0.104, 0.074, z=0.090, y_shift=-0.049, radius_scale=0.24),
                _rounded_section(0.094, 0.066, z=0.168, y_shift=-0.040, radius_scale=0.25),
                _rounded_section(0.084, 0.056, z=0.222, y_shift=-0.026, radius_scale=0.26),
            ]
        )
    )


def _build_head_shell_mesh():
    return superellipse_side_loft(
        [
            (-0.004, -0.020, 0.044, 0.094),
            (0.028, -0.022, 0.055, 0.120),
            (0.074, -0.019, 0.062, 0.142),
            (0.122, -0.015, 0.058, 0.138),
            (0.166, -0.011, 0.046, 0.110),
            (0.204, -0.008, 0.030, 0.072),
        ],
        exponents=2.5,
        segments=56,
    )


def _build_bowl_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.045, 0.010),
            (0.066, 0.022),
            (0.086, 0.050),
            (0.101, 0.086),
            (0.108, 0.112),
        ],
        [
            (0.034, 0.021),
            (0.054, 0.033),
            (0.074, 0.059),
            (0.091, 0.093),
            (0.098, 0.106),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_bowl_handle_mesh():
    return tube_from_spline_points(
        [
            (0.102, -0.006, 0.098),
            (0.124, -0.006, 0.110),
            (0.150, 0.000, 0.094),
            (0.149, 0.002, 0.060),
            (0.126, 0.006, 0.040),
            (0.103, 0.006, 0.050),
        ],
        radius=0.006,
        samples_per_segment=18,
        radial_segments=18,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer", assets=ASSETS)

    appliance_red = model.material("appliance_red", rgba=(0.71, 0.18, 0.14, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.78, 0.79, 0.82, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.29, 0.30, 0.32, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    base = model.part("base")
    foot_shell = base.visual(
        _save_mesh("mixer_base_foot.obj", _build_base_foot_mesh()),
        material=appliance_red,
        name="foot_shell",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.010),
        origin=Origin(xyz=(0.0, 0.045, 0.033)),
        material=trim_silver,
        name="bowl_seat",
    )
    base.visual(
        _save_mesh("mixer_column_shell.obj", _build_column_mesh()),
        material=appliance_red,
        name="column_shell",
    )
    base.visual(
        Box((0.076, 0.046, 0.012)),
        origin=Origin(xyz=(0.0, -0.014, 0.220)),
        material=trim_silver,
        name="column_cap",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.094),
        origin=Origin(xyz=(0.0, -0.026, 0.242), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=trim_silver,
        name="hinge_trim",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.270, 0.242, 0.275)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        Cylinder(radius=0.041, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=brushed_steel,
        name="bowl_foot",
    )
    bowl.visual(
        _save_mesh("mixer_bowl_shell.obj", _build_bowl_shell_mesh()),
        material=brushed_steel,
        name="bowl_shell",
    )
    bowl.visual(
        _save_mesh("mixer_bowl_handle.obj", _build_bowl_handle_mesh()),
        material=brushed_steel,
        name="bowl_handle",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.112, length=0.116),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.022, length=0.108),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=trim_silver,
        name="hinge_barrel",
    )
    head.visual(
        _save_mesh("mixer_head_shell.obj", _build_head_shell_mesh()),
        material=appliance_red,
        name="head_shell",
    )
    head.visual(
        Box((0.086, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.024, -0.032)),
        material=appliance_red,
        name="rear_skirt",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.0, 0.186, 0.010), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=trim_silver,
        name="hub_cap",
    )
    head.visual(
        Cylinder(radius=0.011, length=0.094),
        origin=Origin(xyz=(0.0, 0.078, -0.065)),
        material=dark_metal,
        name="beater_shaft",
    )
    head.visual(
        Box((0.046, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.078, -0.117)),
        material=dark_metal,
        name="beater_top_bar",
    )
    head.visual(
        Box((0.010, 0.012, 0.082)),
        origin=Origin(xyz=(-0.015, 0.078, -0.163)),
        material=dark_metal,
        name="beater_left_rib",
    )
    head.visual(
        Box((0.010, 0.012, 0.082)),
        origin=Origin(xyz=(0.015, 0.078, -0.163)),
        material=dark_metal,
        name="beater_right_rib",
    )
    head.visual(
        Box((0.046, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.078, -0.205)),
        material=dark_metal,
        name="beater_frame",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.148, 0.220, 0.120)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.100, 0.004)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.045, 0.038)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, -0.030, 0.270)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    tilt = object_model.get_articulation("base_to_head")

    foot_shell = base.get_visual("foot_shell")
    bowl_seat = base.get_visual("bowl_seat")
    column_cap = base.get_visual("column_cap")
    bowl_foot = bowl.get_visual("bowl_foot")
    bowl_shell = bowl.get_visual("bowl_shell")
    rear_skirt = head.get_visual("rear_skirt")
    head_shell = head.get_visual("head_shell")
    beater_shaft = head.get_visual("beater_shaft")
    beater_frame = head.get_visual("beater_frame")

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
    # For ctx.expect_* helpers, keep the first body/link arguments as Part objects.
    # Named Visuals belong only in elem_a/elem_b/positive_elem/negative_elem/inner_elem/outer_elem.
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    ctx.expect_contact(
        bowl,
        base,
        elem_a=bowl_foot,
        elem_b=bowl_seat,
        name="bowl_foot_sits_on_the_base_seat",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="xy",
        min_overlap=0.12,
        elem_a=bowl_shell,
        elem_b=foot_shell,
        name="mixing_bowl_overhangs_a_substantial_base_footprint",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a=rear_skirt,
        elem_b=column_cap,
        name="tilt_head_rests_on_the_rear_column_when_closed",
    )
    ctx.expect_overlap(
        head,
        bowl,
        axes="x",
        min_overlap=0.040,
        elem_a=beater_frame,
        elem_b=bowl_shell,
        name="beater_spans_the_bowl_centerline_in_x",
    )
    ctx.expect_overlap(
        head,
        bowl,
        axes="y",
        min_overlap=0.018,
        elem_a=beater_shaft,
        elem_b=bowl_shell,
        name="beater_hangs_over_the_bowl_centerline_in_y",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=-0.250,
        max_gap=-0.010,
        positive_elem=beater_frame,
        negative_elem=bowl_shell,
        name="beater_drops_below_the_bowl_rim_in_working_pose",
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=0.070,
        positive_elem=head_shell,
        negative_elem=bowl_shell,
        name="motor_head_shell_stays_well_above_the_bowl",
    )
    with ctx.pose({tilt: 1.0}):
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            min_gap=0.030,
            positive_elem=beater_frame,
            negative_elem=bowl_shell,
            name="tilted_head_lifts_the_beater_clear_of_the_bowl",
        )
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.020,
            positive_elem=rear_skirt,
            negative_elem=column_cap,
            name="rear_skirt_lifts_off_the_column_when_the_head_is_open",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
