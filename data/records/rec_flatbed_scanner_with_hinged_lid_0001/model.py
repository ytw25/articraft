from __future__ import annotations

# The harness only exposes the editable block to the model.
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

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BODY_WIDTH = 0.442
BODY_DEPTH = 0.308
LOWER_BODY_HEIGHT = 0.016
UPPER_BODY_HEIGHT = 0.036
UPPER_BODY_Z = 0.014
BODY_HEIGHT = UPPER_BODY_Z + UPPER_BODY_HEIGHT

BED_CENTER_X = -0.046
BED_CENTER_Y = 0.004
BED_BEZEL_WIDTH = 0.310
BED_BEZEL_DEPTH = 0.236
BED_WINDOW_WIDTH = 0.278
BED_WINDOW_DEPTH = 0.204
BED_GLASS_WIDTH = 0.292
BED_GLASS_DEPTH = 0.216
BED_BEZEL_HEIGHT = 0.004

CONTROL_CENTER_X = 0.147
CONTROL_CENTER_Y = 0.004
CONTROL_PANEL_WIDTH = 0.094
CONTROL_PANEL_DEPTH = 0.228
CONTROL_PANEL_HEIGHT = 0.0025

LID_WIDTH = 0.336
LID_DEPTH = 0.291
LID_CENTER_Y = LID_DEPTH / 2.0 - 0.008
LID_CLOSED_ANGLE = 0.0
LID_OPEN_ANGLE = math.radians(67.0)

HINGE_Y = -BODY_DEPTH / 2.0 + 0.012
HINGE_Z = BODY_HEIGHT + 0.0020


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _lid_mount(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flatbed_scanner", assets=ASSETS)

    silver = model.material("silver_paint", rgba=(0.82, 0.83, 0.85, 1.0))
    cool_white = model.material("cool_white", rgba=(0.92, 0.93, 0.92, 1.0))
    platen_white = model.material("platen_white", rgba=(0.96, 0.96, 0.94, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.19, 0.20, 0.22, 1.0))
    black_gloss = model.material("black_gloss", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.11, 0.11, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.84, 0.92, 0.32))
    blue_led = model.material("status_blue", rgba=(0.23, 0.78, 0.97, 0.90))

    base = model.part("base")

    lower_shell = _save_mesh(
        "scanner_body_lower.obj",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(
                BODY_WIDTH - 0.012,
                BODY_DEPTH - 0.010,
                radius=0.020,
                corner_segments=10,
            ),
            height=LOWER_BODY_HEIGHT,
        ),
    )
    base.visual(lower_shell, origin=Origin(), material=charcoal, name="lower_shell")

    upper_shell = _save_mesh(
        "scanner_body_upper.obj",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(
                BODY_WIDTH,
                BODY_DEPTH,
                radius=0.024,
                corner_segments=12,
            ),
            height=UPPER_BODY_HEIGHT,
        ),
    )
    base.visual(
        upper_shell,
        origin=Origin(xyz=(0.0, 0.0, UPPER_BODY_Z)),
        material=silver,
        name="upper_shell",
    )

    bezel = _save_mesh(
        "scanner_bed_bezel.obj",
        ExtrudeWithHolesGeometry(
            outer_profile=rounded_rect_profile(
                BED_BEZEL_WIDTH,
                BED_BEZEL_DEPTH,
                radius=0.013,
                corner_segments=10,
            ),
            hole_profiles=[
                rounded_rect_profile(
                    BED_WINDOW_WIDTH,
                    BED_WINDOW_DEPTH,
                    radius=0.008,
                    corner_segments=8,
                )
            ],
            height=BED_BEZEL_HEIGHT,
            center=False,
        ),
    )
    base.visual(
        bezel,
        origin=Origin(xyz=(BED_CENTER_X, BED_CENTER_Y, BODY_HEIGHT - BED_BEZEL_HEIGHT)),
        material=black_gloss,
        name="scan_bed_bezel",
    )

    base.visual(
        Box((BED_GLASS_WIDTH, BED_GLASS_DEPTH, 0.0032)),
        origin=Origin(xyz=(BED_CENTER_X, BED_CENTER_Y, 0.0481)),
        material=glass,
        name="scan_glass",
    )
    base.visual(
        Box((0.286, 0.206, 0.0028)),
        origin=Origin(xyz=(BED_CENTER_X, BED_CENTER_Y, 0.0458)),
        material=platen_white,
        name="scan_platen",
    )
    base.visual(
        Box((0.010, 0.190, 0.0010)),
        origin=Origin(xyz=(BED_CENTER_X - BED_WINDOW_WIDTH / 2.0 + 0.010, BED_CENTER_Y, 0.0462)),
        material=platen_white,
        name="calibration_strip",
    )

    control_panel = _save_mesh(
        "scanner_control_panel.obj",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(
                CONTROL_PANEL_WIDTH,
                CONTROL_PANEL_DEPTH,
                radius=0.010,
                corner_segments=10,
            ),
            height=CONTROL_PANEL_HEIGHT,
        ),
    )
    base.visual(
        control_panel,
        origin=Origin(
            xyz=(
                CONTROL_CENTER_X,
                CONTROL_CENTER_Y,
                BODY_HEIGHT - CONTROL_PANEL_HEIGHT,
            )
        ),
        material=black_gloss,
        name="control_panel",
    )
    base.visual(
        Box((0.050, 0.022, 0.0012)),
        origin=Origin(xyz=(CONTROL_CENTER_X, CONTROL_CENTER_Y + 0.069, 0.0492)),
        material=charcoal,
        name="status_window",
    )
    for name, x_off, y_off, radius, material in (
        ("power_button", 0.024, -0.063, 0.010, silver),
        ("start_button", -0.024, -0.063, 0.009, cool_white),
        ("stop_button", 0.000, -0.063, 0.009, cool_white),
    ):
        base.visual(
            Cylinder(radius=radius, length=0.0024),
            origin=Origin(xyz=(CONTROL_CENTER_X + x_off, CONTROL_CENTER_Y + y_off, 0.0498)),
            material=material,
            name=name,
        )
    base.visual(
        Box((0.018, 0.010, 0.0020)),
        origin=Origin(xyz=(CONTROL_CENTER_X - 0.022, CONTROL_CENTER_Y - 0.010, 0.0494)),
        material=silver,
        name="mode_button_left",
    )
    base.visual(
        Box((0.018, 0.010, 0.0020)),
        origin=Origin(xyz=(CONTROL_CENTER_X + 0.022, CONTROL_CENTER_Y - 0.010, 0.0494)),
        material=silver,
        name="mode_button_right",
    )
    base.visual(
        Box((0.006, 0.006, 0.0014)),
        origin=Origin(xyz=(CONTROL_CENTER_X - 0.034, CONTROL_CENTER_Y + 0.069, 0.0495)),
        material=blue_led,
        name="status_led",
    )
    base.visual(
        Box((0.044, 0.010, 0.0008)),
        origin=Origin(xyz=(CONTROL_CENTER_X, CONTROL_CENTER_Y + 0.103, 0.0499)),
        material=charcoal,
        name="logo_plaque",
    )

    base.visual(
        Box((LID_WIDTH + 0.024, 0.018, 0.009)),
        origin=Origin(xyz=(BED_CENTER_X, HINGE_Y, BODY_HEIGHT - 0.0035)),
        material=charcoal,
        name="hinge_cover",
    )
    base.visual(
        Box((0.340, 0.005, 0.007)),
        origin=Origin(xyz=(-0.010, BODY_DEPTH / 2.0 - 0.0025, 0.024)),
        material=black_gloss,
        name="front_fascia",
    )

    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.162, -0.112),
            (-0.162, 0.112),
            (0.162, -0.112),
            (0.162, 0.112),
        ),
        start=1,
    ):
        base.visual(
            Box((0.032, 0.018, 0.004)),
            origin=Origin(xyz=(x_pos, y_pos, 0.0015)),
            material=rubber,
            name=f"foot_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    lid = model.part("lid")

    lid_shell = _save_mesh(
        "scanner_lid_shell.obj",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(
                LID_WIDTH,
                LID_DEPTH,
                radius=0.020,
                corner_segments=12,
            ),
            height=0.015,
        ),
    )
    lid.visual(
        lid_shell,
        origin=_lid_mount((BED_CENTER_X, LID_CENTER_Y, -0.0010)),
        material=cool_white,
        name="lid_shell",
    )

    lid_backing = _save_mesh(
        "scanner_lid_backing.obj",
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(
                0.312,
                0.263,
                radius=0.016,
                corner_segments=10,
            ),
            height=0.0078,
        ),
    )
    lid.visual(
        lid_backing,
        origin=_lid_mount((BED_CENTER_X, 0.126, -0.0004)),
        material=platen_white,
        name="lid_backing",
    )
    lid.visual(
        Box((LID_WIDTH * 0.88, 0.016, 0.010)),
        origin=_lid_mount((BED_CENTER_X, 0.004, 0.0050)),
        material=charcoal,
        name="lid_hinge_trim",
    )
    lid.visual(
        Box((0.140, 0.016, 0.0036)),
        origin=_lid_mount((BED_CENTER_X, 0.272, 0.0130)),
        material=black_gloss,
        name="lid_pull_grip",
    )
    lid.visual(
        Box((0.062, 0.012, 0.0009)),
        origin=_lid_mount((BED_CENTER_X + 0.095, 0.235, 0.0153)),
        material=charcoal,
        name="lid_badge",
    )
    for index, x_pos in enumerate((-0.114, 0.114), start=1):
        lid.visual(
            Box((0.018, 0.012, 0.0030)),
            origin=_lid_mount((BED_CENTER_X + x_pos, 0.250, -0.0014)),
            material=rubber,
            name=f"lid_front_bumper_{index}",
        )
    for index, x_pos in enumerate((-0.122, 0.122), start=1):
        lid.visual(
            Box((0.018, 0.010, 0.0024)),
            origin=_lid_mount((BED_CENTER_X + x_pos, 0.018, -0.0011)),
            material=rubber,
            name=f"lid_rear_bumper_{index}",
        )

    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_DEPTH, 0.015)),
        mass=1.2,
        origin=Origin(xyz=(BED_CENTER_X, LID_CENTER_Y, 0.0065)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lid",
        origin=Origin(xyz=(BED_CENTER_X, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.5,
            lower=LID_CLOSED_ANGLE,
            upper=LID_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_joint_motion_axis(
        "lid_hinge",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_aabb_overlap("lid", "base", axes="x", min_overlap=0.30)
    ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.24)
    ctx.expect_aabb_contact("lid", "base")

    with ctx.pose(lid_hinge=LID_CLOSED_ANGLE):
        ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.24)
        ctx.expect_aabb_contact("lid", "base")

    with ctx.pose(lid_hinge=0.40):
        ctx.expect_aabb_overlap("lid", "base", axes="x", min_overlap=0.30)
        ctx.expect_aabb_overlap("lid", "base", axes="y", min_overlap=0.17)

    with ctx.pose(lid_hinge=LID_OPEN_ANGLE):
        ctx.expect_aabb_overlap("lid", "base", axes="x", min_overlap=0.30)
        ctx.expect_aabb_overlap("lid", "base", axes="y", min_overlap=0.10)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
