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
    repair_loft,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer", assets=ASSETS)

    enamel_red = model.material("enamel_red", rgba=(0.71, 0.12, 0.12, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.75, 0.77, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.19, 0.19, 0.21, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.64, 0.66, 0.69, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))

    def save_mesh(filename: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))

    def xy_loop(
        size_x: float,
        size_y: float,
        z: float,
        *,
        x_shift: float = 0.0,
        radius: float | None = None,
        corner_segments: int = 8,
    ) -> list[tuple[float, float, float]]:
        profile = rounded_rect_profile(
            size_x,
            size_y,
            radius if radius is not None else min(size_x, size_y) * 0.20,
            corner_segments=corner_segments,
        )
        return [(x + x_shift, y, z) for x, y in profile]

    def yz_loop(
        x_pos: float,
        size_y: float,
        size_z: float,
        z_center: float,
        *,
        exponent: float = 2.5,
        segments: int = 36,
    ) -> list[tuple[float, float, float]]:
        profile = superellipse_profile(size_y, size_z, exponent=exponent, segments=segments)
        return [(x_pos, y, z_center + z) for y, z in profile]

    pedestal_shell = repair_loft(
        section_loft(
            [
                xy_loop(0.294, 0.236, 0.014, x_shift=0.012, radius=0.050),
                xy_loop(0.270, 0.224, 0.048, x_shift=0.006, radius=0.046),
                xy_loop(0.224, 0.186, 0.082, x_shift=-0.010, radius=0.040),
            ]
        )
    )
    column_shell = repair_loft(
        section_loft(
            [
                xy_loop(0.112, 0.104, 0.082, x_shift=-0.054, radius=0.020),
                xy_loop(0.118, 0.096, 0.150, x_shift=-0.056, radius=0.020),
                xy_loop(0.102, 0.094, 0.214, x_shift=-0.060, radius=0.018),
                xy_loop(0.084, 0.102, 0.240, x_shift=-0.066, radius=0.018),
            ]
        )
    )
    head_shell = repair_loft(
        section_loft(
            [
                yz_loop(0.090, 0.068, 0.072, 0.020, exponent=2.3, segments=32),
                yz_loop(0.132, 0.094, 0.114, 0.024, exponent=2.5, segments=36),
                yz_loop(0.192, 0.108, 0.132, 0.026, exponent=2.7, segments=40),
                yz_loop(0.250, 0.096, 0.102, 0.006, exponent=2.5, segments=36),
                yz_loop(0.296, 0.056, 0.058, -0.012, exponent=2.2, segments=30),
            ]
        )
    )
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.030, 0.000),
            (0.046, 0.012),
            (0.078, 0.050),
            (0.094, 0.096),
            (0.102, 0.118),
        ],
        [
            (0.020, 0.006),
            (0.036, 0.016),
            (0.068, 0.052),
            (0.086, 0.097),
            (0.094, 0.116),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    dough_hook = tube_from_spline_points(
        [
            (0.186, 0.000, -0.128),
            (0.176, 0.010, -0.136),
            (0.160, 0.016, -0.132),
            (0.150, 0.010, -0.108),
            (0.164, 0.000, -0.094),
        ],
        radius=0.007,
        samples_per_segment=18,
        radial_segments=14,
        cap_ends=True,
    )

    base = model.part("base")
    base.visual(
        save_mesh("mixer_pedestal_shell.obj", pedestal_shell),
        material=enamel_red,
        name="pedestal_shell",
    )
    base.visual(
        save_mesh("mixer_column_shell.obj", column_shell),
        material=enamel_red,
        name="column_shell",
    )
    base.visual(
        Cylinder(radius=0.064, length=0.012),
        origin=Origin(xyz=(0.108, 0.0, 0.086)),
        material=brushed_aluminum,
        name="bowl_seat",
    )
    base.visual(
        Box((0.022, 0.116, 0.022)),
        origin=Origin(xyz=(-0.084, 0.0, 0.229)),
        material=enamel_red,
        name="hinge_bridge",
    )
    base.visual(
        Box((0.056, 0.088, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, 0.229)),
        material=dark_trim,
        name="head_rest",
    )
    base.visual(
        Box((0.022, 0.018, 0.040)),
        origin=Origin(xyz=(-0.084, 0.060, 0.249)),
        material=dark_trim,
        name="hinge_left_ear",
    )
    base.visual(
        Box((0.022, 0.018, 0.040)),
        origin=Origin(xyz=(-0.084, -0.060, 0.249)),
        material=dark_trim,
        name="hinge_right_ear",
    )
    base.visual(
        Box((0.052, 0.014, 0.022)),
        origin=Origin(xyz=(-0.058, 0.050, 0.237)),
        material=dark_trim,
        name="hinge_left_cheek",
    )
    base.visual(
        Box((0.052, 0.014, 0.022)),
        origin=Origin(xyz=(-0.058, -0.050, 0.237)),
        material=dark_trim,
        name="hinge_right_cheek",
    )
    for suffix, x_pos, y_pos in (
        ("front_left", 0.086, 0.074),
        ("front_right", 0.086, -0.074),
        ("rear_left", -0.104, 0.070),
        ("rear_right", -0.104, -0.070),
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(x_pos, y_pos, 0.007)),
            material=rubber_black,
            name=f"{suffix}_foot",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.32, 0.24, 0.27)),
        mass=11.0,
        origin=Origin(xyz=(-0.010, 0.0, 0.135)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        save_mesh("mixer_bowl_shell.obj", bowl_shell),
        material=polished_steel,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.030, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=polished_steel,
        name="foot_ring",
    )
    bowl.visual(
        Box((0.032, 0.016, 0.014)),
        origin=Origin(xyz=(0.016, 0.096, 0.090)),
        material=polished_steel,
        name="handle_upper_mount",
    )
    bowl.visual(
        Box((0.032, 0.016, 0.014)),
        origin=Origin(xyz=(0.016, 0.096, 0.052)),
        material=polished_steel,
        name="handle_lower_mount",
    )
    bowl.visual(
        Box((0.038, 0.038, 0.012)),
        origin=Origin(xyz=(0.032, 0.109, 0.090)),
        material=polished_steel,
        name="handle_upper_bridge",
    )
    bowl.visual(
        Box((0.038, 0.038, 0.012)),
        origin=Origin(xyz=(0.032, 0.109, 0.052)),
        material=polished_steel,
        name="handle_lower_bridge",
    )
    bowl.visual(
        Box((0.022, 0.012, 0.050)),
        origin=Origin(xyz=(0.048, 0.121, 0.071)),
        material=polished_steel,
        name="handle_grip",
    )
    bowl.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, 0.13)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    head = model.part("head")
    head.visual(
        save_mesh("mixer_head_shell.obj", head_shell),
        material=enamel_red,
        name="head_shell",
    )
    head.visual(
        Box((0.118, 0.094, 0.040)),
        origin=Origin(xyz=(0.058, 0.0, 0.004)),
        material=enamel_red,
        name="rear_neck",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.088),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.050, 0.078, 0.012)),
        origin=Origin(xyz=(0.070, 0.0, -0.016)),
        material=dark_trim,
        name="mount_pad",
    )
    head.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.218, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="front_cap",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.056),
        origin=Origin(xyz=(0.186, 0.0, -0.064)),
        material=brushed_aluminum,
        name="gear_hub",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.186, 0.0, -0.092)),
        material=polished_steel,
        name="attachment_collar",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.080),
        origin=Origin(xyz=(0.186, 0.0, -0.115)),
        material=polished_steel,
        name="attachment_shaft",
    )
    head.visual(
        Box((0.024, 0.030, 0.012)),
        origin=Origin(xyz=(0.090, 0.048, 0.034)),
        material=dark_trim,
        name="speed_lever_base",
    )
    head.visual(
        save_mesh("mixer_dough_hook.obj", dough_hook),
        material=polished_steel,
        name="dough_hook",
    )
    head.visual(
        Box((0.026, 0.008, 0.010)),
        origin=Origin(xyz=(0.092, 0.062, 0.036)),
        material=dark_trim,
        name="speed_lever",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.26, 0.12, 0.16)),
        mass=5.8,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
    )

    model.articulation(
        "bowl_mount",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.108, 0.0, 0.092)),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.078, 0.0, 0.258)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    head_tilt = object_model.get_articulation("head_tilt")

    bowl_seat = base.get_visual("bowl_seat")
    head_rest = base.get_visual("head_rest")
    bowl_foot = bowl.get_visual("foot_ring")
    bowl_shell = bowl.get_visual("bowl_shell")
    bowl_handle = bowl.get_visual("handle_grip")
    head_mount = head.get_visual("mount_pad")
    dough_hook = head.get_visual("dough_hook")
    head_shell = head.get_visual("head_shell")
    speed_lever = head.get_visual("speed_lever")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual connectivity gate for floating/disconnected subassemblies inside one part.
    ctx.check_part_geometry_connected()
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
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=bowl_foot,
        negative_elem=bowl_seat,
    )
    ctx.expect_within(
        bowl,
        base,
        axes="xy",
        inner_elem=bowl_foot,
        outer_elem=bowl_seat,
    )
    ctx.expect_origin_distance(bowl, base, axes="y", max_dist=0.001)
    ctx.expect_gap(
        bowl,
        bowl,
        axis="y",
        min_gap=0.010,
        positive_elem=bowl_handle,
        negative_elem=bowl_shell,
        name="bowl_handle_proud",
    )

    ctx.expect_contact(head, base, elem_a=head_mount, elem_b=head_rest)
    ctx.expect_overlap(
        head,
        base,
        axes="xy",
        min_overlap=0.025,
        elem_a=head_mount,
        elem_b=head_rest,
    )
    ctx.expect_within(
        head,
        bowl,
        axes="xy",
        inner_elem=dough_hook,
        outer_elem=bowl_shell,
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=0.005,
        positive_elem=dough_hook,
        negative_elem=bowl_foot,
    )
    ctx.expect_origin_distance(head, bowl, axes="y", max_dist=0.001)
    ctx.expect_gap(
        head,
        head,
        axis="y",
        min_gap=0.003,
        positive_elem=speed_lever,
        negative_elem=head_shell,
        name="speed_lever_proud",
    )

    with ctx.pose({head_tilt: 0.95}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.030,
            positive_elem=head_mount,
            negative_elem=head_rest,
        )
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            min_gap=0.050,
            positive_elem=dough_hook,
            negative_elem=bowl_shell,
        )
        ctx.expect_origin_distance(head, bowl, axes="y", max_dist=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
