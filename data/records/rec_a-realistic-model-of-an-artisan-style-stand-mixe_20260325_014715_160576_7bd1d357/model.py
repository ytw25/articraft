from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _xy_section(
    *,
    z: float,
    width: float,
    depth: float,
    y_offset: float,
    exponent: float = 2.8,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_offset, z)
        for x, y in superellipse_profile(width, depth, exponent=exponent, segments=segments)
    ]


def _xz_section(
    *,
    y: float,
    width: float,
    height: float,
    z_center: float,
    exponent: float = 2.5,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for x, z in superellipse_profile(width, height, exponent=exponent, segments=segments)
    ]


def _build_base_foot_mesh():
    return ExtrudeGeometry.from_z0(
        [
            (x, y + 0.032)
            for x, y in superellipse_profile(0.246, 0.238, exponent=3.2, segments=56)
        ],
        0.022,
    )


def _build_column_mesh():
    return repair_loft(
        section_loft(
            [
                _xy_section(z=0.018, width=0.104, depth=0.066, y_offset=-0.048),
                _xy_section(z=0.090, width=0.094, depth=0.062, y_offset=-0.056),
                _xy_section(z=0.162, width=0.086, depth=0.056, y_offset=-0.056),
                _xy_section(z=0.232, width=0.080, depth=0.052, y_offset=-0.051),
                _xy_section(z=0.286, width=0.082, depth=0.056, y_offset=-0.046),
            ]
        )
    )


def _build_head_shell_mesh():
    return repair_loft(
        section_loft(
            [
                _xz_section(y=-0.012, width=0.094, height=0.086, z_center=0.016),
                _xz_section(y=0.040, width=0.116, height=0.120, z_center=0.028),
                _xz_section(y=0.095, width=0.110, height=0.114, z_center=0.024),
                _xz_section(y=0.150, width=0.088, height=0.094, z_center=0.010),
                _xz_section(y=0.190, width=0.056, height=0.070, z_center=-0.004),
            ]
        )
    )


def _build_bowl_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.000, 0.000),
            (0.034, 0.010),
            (0.067, 0.038),
            (0.094, 0.086),
            (0.110, 0.138),
        ],
        [
            (0.000, 0.006),
            (0.024, 0.012),
            (0.056, 0.039),
            (0.082, 0.083),
            (0.098, 0.133),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_bowl_handle_mesh():
    return tube_from_spline_points(
        [
            (0.100, 0.014, 0.078),
            (0.134, 0.020, 0.074),
            (0.152, 0.024, 0.100),
            (0.128, 0.019, 0.125),
            (0.102, 0.014, 0.118),
        ],
        radius=0.007,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )


def _build_beater_frame_mesh():
    return wire_from_points(
        [
            (-0.016, 0.148, -0.102),
            (-0.022, 0.148, -0.125),
            (-0.015, 0.148, -0.152),
            (0.000, 0.148, -0.160),
            (0.015, 0.148, -0.152),
            (0.022, 0.148, -0.125),
            (0.016, 0.148, -0.102),
        ],
        radius=0.0038,
        radial_segments=14,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.012,
        corner_segments=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer", assets=ASSETS)

    enamel_red = model.material("enamel_red", rgba=(0.64, 0.08, 0.10, 1.0))
    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    black_trim = model.material("black_trim", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(_save_mesh("mixer_foot.obj", _build_base_foot_mesh()), material=enamel_red, name="foot_shell")
    base.visual(_save_mesh("mixer_column.obj", _build_column_mesh()), material=enamel_red, name="column_shell")
    base.visual(
        Cylinder(radius=0.063, length=0.010),
        origin=Origin(xyz=(0.0, 0.098, 0.027)),
        material=brushed_steel,
        name="pedestal_plate",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.094),
        origin=Origin(xyz=(0.0, -0.040, 0.296), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_pin",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, -0.040, 0.305)),
        material=dark_metal,
        name="column_cap",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.130, length=0.320),
        mass=9.2,
        origin=Origin(xyz=(0.0, -0.010, 0.160)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.020, length=0.102),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    head.visual(
        _save_mesh("mixer_head_shell.obj", _build_head_shell_mesh()),
        material=enamel_red,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.024),
        origin=Origin(xyz=(0.0, 0.148, -0.036)),
        material=brushed_steel,
        name="hub_collar",
    )
    head.visual(
        Cylinder(radius=0.0045, length=0.064),
        origin=Origin(xyz=(0.0, 0.148, -0.074)),
        material=dark_metal,
        name="hub_shaft",
    )
    head.visual(
        _save_mesh("mixer_beater_frame.obj", _build_beater_frame_mesh()),
        material=brushed_steel,
        name="beater_frame",
    )
    head.visual(
        Cylinder(radius=0.0038, length=0.040),
        origin=Origin(xyz=(0.0, 0.148, -0.129), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="beater_crossbar",
    )
    head.visual(
        Cylinder(radius=0.004, length=0.032),
        origin=Origin(xyz=(0.058, 0.014, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_trim,
        name="speed_lever",
    )
    head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.240),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.095, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        _save_mesh("mixer_bowl_shell.obj", _build_bowl_shell_mesh()),
        material=stainless,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.047, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=stainless,
        name="base_ring",
    )
    bowl.visual(
        _save_mesh("mixer_bowl_handle.obj", _build_bowl_handle_mesh()),
        material=stainless,
        name="side_handle",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.138),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, -0.040, 0.296)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.6, lower=0.0, upper=1.05),
    )
    model.articulation(
        "bowl_mount",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.098, 0.032)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    bowl = object_model.get_part("bowl")
    head_tilt = object_model.get_articulation("head_tilt")
    pedestal_plate = base.get_visual("pedestal_plate")
    hinge_pin = base.get_visual("hinge_pin")
    foot_shell = base.get_visual("foot_shell")
    column_shell = base.get_visual("column_shell")
    hinge_barrel = head.get_visual("hinge_barrel")
    beater_frame = head.get_visual("beater_frame")
    bowl_shell = bowl.get_visual("bowl_shell")
    base_ring = bowl.get_visual("base_ring")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual connectivity gate for floating/disconnected subassemblies inside one part.
    ctx.check_part_geometry_connected()
    ctx.allow_overlap(head, base, reason="rear tilt hinge barrel nests around the fixed hinge pin")
    # Default broad part-level rest-pose backstop for top-level interpenetration.
    # If a seated or nested fit is intentional, justify it with `ctx.allow_overlap(...)`.
    ctx.check_no_part_overlaps()

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
    # named local features over the broad rest-pose overlap backstop.
    # Add prompt-specific exact visual checks below; optional warning heuristics are not enough.
    ctx.expect_origin_distance(bowl, base, axes="x", max_dist=0.003)
    ctx.expect_origin_distance(bowl, base, axes="y", max_dist=0.110)
    ctx.expect_within(bowl, base, axes="xy", inner_elem=base_ring, outer_elem=pedestal_plate)
    ctx.expect_overlap(bowl, base, axes="xy", min_overlap=0.080, elem_a=base_ring, elem_b=pedestal_plate)
    ctx.expect_contact(bowl, base, elem_a=base_ring, elem_b=pedestal_plate)
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=base_ring,
        negative_elem=pedestal_plate,
    )
    ctx.expect_gap(
        bowl,
        base,
        axis="y",
        min_gap=0.001,
        positive_elem=bowl_shell,
        negative_elem=column_shell,
    )
    ctx.expect_overlap(head, bowl, axes="xz", min_overlap=0.030, elem_a=beater_frame, elem_b=bowl_shell)
    ctx.expect_contact(head, base, elem_a=hinge_barrel, elem_b=hinge_pin)
    ctx.expect_overlap(head, base, axes="x", min_overlap=0.070, elem_a=hinge_barrel, elem_b=hinge_pin)
    ctx.expect_overlap(base, bowl, axes="x", min_overlap=0.120, elem_a=foot_shell, elem_b=bowl_shell)
    with ctx.pose({head_tilt: 0.92}):
        ctx.expect_contact(head, base, elem_a=hinge_barrel, elem_b=hinge_pin)
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            min_gap=0.100,
            positive_elem=beater_frame,
            negative_elem=bowl_shell,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
