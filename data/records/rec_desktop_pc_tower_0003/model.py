from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
import pathlib

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except FileNotFoundError:
            pass
        return "/"


os.getcwd = _safe_getcwd
pathlib.os.getcwd = _safe_getcwd


@classmethod
def _safe_path_cwd(cls):
    return cls("/")


pathlib.Path.cwd = _safe_path_cwd
pathlib.PosixPath.cwd = _safe_path_cwd

_ORIG_PATH_RESOLVE = pathlib.Path.resolve


def _safe_path_resolve(self, strict: bool = False):
    try:
        return _ORIG_PATH_RESOLVE(self, strict=strict)
    except FileNotFoundError:
        if self.is_absolute():
            return self
        return pathlib.Path("/") / self


pathlib.Path.resolve = _safe_path_resolve
pathlib.PosixPath.resolve = _safe_path_resolve

from math import pi

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

CASE_W = 0.205
CASE_D = 0.320
CASE_H = 0.335

PANEL_TH = 0.003
PANEL_D = CASE_D - 0.018
PANEL_H = CASE_H - 0.012

FRONT_W = CASE_W - 0.018
FRONT_H = CASE_H - 0.012

GRILLE_W = 0.168
GRILLE_D = 0.252


def _add_thumb_screw(part, *, y: float, z: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.005),
        origin=Origin(xyz=(0.004, y, z), rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=f"{name}_head",
    )
    part.visual(
        Cylinder(radius=0.0022, length=0.007),
        origin=Origin(xyz=(0.0008, y, z), rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_itx_sff_case")

    powder_black = model.material("powder_black", rgba=(0.14, 0.15, 0.16, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.18, 0.19, 0.20, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    matte_mesh = model.material("matte_mesh", rgba=(0.10, 0.11, 0.12, 1.0))
    io_black = model.material("io_black", rgba=(0.05, 0.05, 0.06, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    port_metal = model.material("port_metal", rgba=(0.57, 0.59, 0.62, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.194, 0.308, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=dark_frame,
        name="bottom_floor",
    )
    chassis.visual(
        Box((0.004, 0.300, 0.304)),
        origin=Origin(xyz=(-0.099, 0.000, 0.160)),
        material=powder_black,
        name="left_wall",
    )
    chassis.visual(
        Box((0.178, 0.012, 0.044)),
        origin=Origin(xyz=(-0.004, 0.154, 0.292)),
        material=powder_black,
        name="front_top_bar",
    )
    chassis.visual(
        Box((0.178, 0.012, 0.050)),
        origin=Origin(xyz=(-0.004, 0.154, 0.060)),
        material=powder_black,
        name="front_bottom_bar",
    )
    chassis.visual(
        Box((0.010, 0.012, 0.324)),
        origin=Origin(xyz=(-0.093, 0.154, 0.169)),
        material=powder_black,
        name="front_left_post",
    )
    chassis.visual(
        Box((0.010, 0.012, 0.324)),
        origin=Origin(xyz=(0.085, 0.154, 0.169)),
        material=powder_black,
        name="front_right_post",
    )
    chassis.visual(
        Box((0.004, 0.002, 0.240)),
        origin=Origin(xyz=(0.0905, 0.151, 0.164)),
        material=dark_frame,
        name="panel_stop",
    )
    chassis.visual(
        Box((0.178, 0.012, 0.034)),
        origin=Origin(xyz=(-0.004, -0.154, 0.302)),
        material=powder_black,
        name="rear_top_bar",
    )
    chassis.visual(
        Box((0.178, 0.012, 0.070)),
        origin=Origin(xyz=(-0.004, -0.154, 0.062)),
        material=powder_black,
        name="rear_bottom_bar",
    )
    chassis.visual(
        Box((0.010, 0.012, 0.324)),
        origin=Origin(xyz=(-0.093, -0.154, 0.169)),
        material=powder_black,
        name="rear_left_post",
    )
    chassis.visual(
        Box((0.010, 0.012, 0.324)),
        origin=Origin(xyz=(0.085, -0.154, 0.169)),
        material=powder_black,
        name="rear_right_post",
    )
    chassis.visual(
        Box((0.090, 0.300, 0.008)),
        origin=Origin(xyz=(-0.051, 0.000, 0.331)),
        material=powder_black,
        name="roof_left_strip",
    )
    chassis.visual(
        Box((0.016, 0.290, 0.008)),
        origin=Origin(xyz=(0.082, 0.000, 0.331)),
        material=powder_black,
        name="roof_right_strip",
    )
    chassis.visual(
        Box((0.188, 0.012, 0.008)),
        origin=Origin(xyz=(0.000, 0.149, 0.331)),
        material=powder_black,
        name="roof_front_bar",
    )
    chassis.visual(
        Box((0.188, 0.012, 0.008)),
        origin=Origin(xyz=(0.000, -0.149, 0.331)),
        material=powder_black,
        name="roof_rear_bar",
    )
    chassis.visual(
        Box((GRILLE_W, 0.008, 0.008)),
        origin=Origin(xyz=(0.000, 0.122, 0.331)),
        material=dark_frame,
        name="vent_front_support",
    )
    chassis.visual(
        Box((GRILLE_W, 0.008, 0.008)),
        origin=Origin(xyz=(0.000, -0.122, 0.331)),
        material=dark_frame,
        name="vent_rear_support",
    )
    chassis.visual(
        Box((0.004, 0.276, 0.010)),
        origin=Origin(xyz=(0.0905, 0.000, 0.323)),
        material=dark_frame,
        name="right_guide_top",
    )
    chassis.visual(
        Box((0.004, 0.276, 0.010)),
        origin=Origin(xyz=(0.0905, 0.000, 0.012)),
        material=dark_frame,
        name="right_guide_bottom",
    )
    for x_sign, y_sign in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
        chassis.visual(
            Box((0.020, 0.020, 0.006)),
            origin=Origin(
                xyz=(
                    x_sign * 0.070,
                    y_sign * 0.105,
                    -0.003,
                )
            ),
            material=io_black,
            name=f"foot_{'l' if x_sign < 0 else 'r'}_{'r' if y_sign < 0 else 'f'}",
        )
    chassis.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, CASE_H)),
        mass=6.8,
        origin=Origin(xyz=(0.000, 0.000, CASE_H / 2.0)),
    )

    side_panel = model.part("side_panel")
    side_panel.visual(
        Box((PANEL_TH, PANEL_D, PANEL_H)),
        material=aluminum,
        name="panel_shell",
    )
    side_panel.visual(
        Box((0.010, 0.276, 0.010)),
        origin=Origin(xyz=(-0.0065, 0.000, 0.1545)),
        material=aluminum,
        name="panel_top_flange",
    )
    side_panel.visual(
        Box((0.010, 0.276, 0.010)),
        origin=Origin(xyz=(-0.0065, 0.000, -0.1545)),
        material=aluminum,
        name="panel_bottom_flange",
    )
    side_panel.visual(
        Box((0.010, 0.006, PANEL_H - 0.060)),
        origin=Origin(xyz=(-0.0065, (PANEL_D / 2.0) - 0.004, 0.000)),
        material=aluminum,
        name="panel_front_lip",
    )
    side_panel.visual(
        Box((0.010, 0.010, PANEL_H - 0.090)),
        origin=Origin(xyz=(-0.0065, -(PANEL_D / 2.0) + 0.005, 0.000)),
        material=aluminum,
        name="panel_rear_hook",
    )
    _add_thumb_screw(
        side_panel,
        y=-(PANEL_D / 2.0) + 0.020,
        z=0.108,
        material=screw_steel,
        name="thumb_screw_top",
    )
    _add_thumb_screw(
        side_panel,
        y=-(PANEL_D / 2.0) + 0.020,
        z=-0.108,
        material=screw_steel,
        name="thumb_screw_bottom",
    )
    side_panel.inertial = Inertial.from_geometry(
        Box((0.014, PANEL_D, PANEL_H)),
        mass=1.0,
        origin=Origin(),
    )

    front_panel = model.part("front_panel")
    front_panel.visual(
        Box((0.012, 0.004, FRONT_H)),
        origin=Origin(xyz=(-(FRONT_W / 2.0) + 0.006, 0.000, 0.000)),
        material=powder_black,
        name="bezel_left",
    )
    front_panel.visual(
        Box((0.012, 0.004, FRONT_H)),
        origin=Origin(xyz=((FRONT_W / 2.0) - 0.006, 0.000, 0.000)),
        material=powder_black,
        name="bezel_right",
    )
    front_panel.visual(
        Box((FRONT_W, 0.004, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, (FRONT_H / 2.0) - 0.009)),
        material=powder_black,
        name="bezel_top",
    )
    front_panel.visual(
        Box((FRONT_W, 0.004, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, -(FRONT_H / 2.0) + 0.009)),
        material=powder_black,
        name="bezel_bottom",
    )
    front_panel.visual(
        Box((FRONT_W - 0.024, 0.004, 0.070)),
        origin=Origin(xyz=(0.000, 0.000, 0.110)),
        material=powder_black,
        name="io_fascia",
    )
    front_panel.visual(
        Box((FRONT_W - 0.040, 0.002, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.071)),
        material=matte_mesh,
        name="mesh_top_bridge",
    )
    front_panel.visual(
        Box((FRONT_W - 0.040, 0.002, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, -0.1385)),
        material=matte_mesh,
        name="mesh_bottom_bridge",
    )
    for index, x in enumerate((-0.060, -0.048, -0.036, -0.024, -0.012, 0.000, 0.012, 0.024, 0.036, 0.048, 0.060)):
        front_panel.visual(
            Box((0.004, 0.002, 0.205)),
            origin=Origin(xyz=(x, 0.000, -0.0335)),
            material=matte_mesh,
            name=f"mesh_vertical_{index}",
        )
    for index, z in enumerate((-0.110, -0.086, -0.062, -0.038, -0.014, 0.010, 0.034)):
        front_panel.visual(
            Box((FRONT_W - 0.048, 0.002, 0.003)),
            origin=Origin(xyz=(0.000, 0.000, z)),
            material=matte_mesh,
            name=f"mesh_horizontal_{index}",
        )
    front_panel.visual(
        Box((0.010, 0.008, 0.016)),
        origin=Origin(xyz=(-0.083, -0.006, 0.105)),
        material=powder_black,
        name="snap_tab_tl",
    )
    front_panel.visual(
        Box((0.010, 0.008, 0.016)),
        origin=Origin(xyz=(-0.083, -0.006, -0.105)),
        material=powder_black,
        name="snap_tab_bl",
    )
    front_panel.visual(
        Box((0.010, 0.008, 0.016)),
        origin=Origin(xyz=(0.0765, -0.006, 0.105)),
        material=powder_black,
        name="snap_tab_tr",
    )
    front_panel.visual(
        Box((0.010, 0.008, 0.016)),
        origin=Origin(xyz=(0.0765, -0.006, -0.105)),
        material=powder_black,
        name="snap_tab_br",
    )
    front_panel.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(0.060, 0.001, 0.113), rpy=(pi / 2.0, 0.0, 0.0)),
        material=io_black,
        name="power_button",
    )
    front_panel.visual(
        Box((0.014, 0.006, 0.006)),
        origin=Origin(xyz=(0.010, -0.001, 0.113)),
        material=port_metal,
        name="usb_a",
    )
    front_panel.visual(
        Box((0.010, 0.005, 0.004)),
        origin=Origin(xyz=(0.028, -0.001, 0.113)),
        material=port_metal,
        name="usb_c",
    )
    front_panel.visual(
        Cylinder(radius=0.0035, length=0.005),
        origin=Origin(xyz=(0.046, 0.000, 0.091), rpy=(pi / 2.0, 0.0, 0.0)),
        material=io_black,
        name="audio_left",
    )
    front_panel.visual(
        Cylinder(radius=0.0035, length=0.005),
        origin=Origin(xyz=(0.056, 0.000, 0.091), rpy=(pi / 2.0, 0.0, 0.0)),
        material=io_black,
        name="audio_right",
    )
    front_panel.inertial = Inertial.from_geometry(
        Box((FRONT_W, 0.014, FRONT_H)),
        mass=0.55,
        origin=Origin(),
    )

    top_grille = model.part("top_grille")
    top_grille.visual(
        Box((0.008, GRILLE_D, 0.003)),
        origin=Origin(xyz=(-(GRILLE_W / 2.0) + 0.004, 0.000, 0.000)),
        material=powder_black,
        name="grille_left_rail",
    )
    top_grille.visual(
        Box((0.008, GRILLE_D, 0.003)),
        origin=Origin(xyz=((GRILLE_W / 2.0) - 0.004, 0.000, 0.000)),
        material=powder_black,
        name="grille_right_rail",
    )
    top_grille.visual(
        Box((GRILLE_W, 0.008, 0.003)),
        origin=Origin(xyz=(0.000, (GRILLE_D / 2.0) - 0.004, 0.000)),
        material=powder_black,
        name="grille_front_rail",
    )
    top_grille.visual(
        Box((GRILLE_W, 0.008, 0.003)),
        origin=Origin(xyz=(0.000, -(GRILLE_D / 2.0) + 0.004, 0.000)),
        material=powder_black,
        name="grille_rear_rail",
    )
    for index, y in enumerate((-0.094, -0.067, -0.040, -0.013, 0.014, 0.041, 0.068, 0.095)):
        top_grille.visual(
            Box((GRILLE_W - 0.016, 0.004, 0.002)),
            origin=Origin(xyz=(0.000, y, 0.000)),
            material=matte_mesh,
            name=f"grille_slat_{index}",
        )
    top_grille.inertial = Inertial.from_geometry(
        Box((GRILLE_W, GRILLE_D, 0.005)),
        mass=0.22,
        origin=Origin(),
    )

    model.articulation(
        "side_panel_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(0.104, 0.000, CASE_H / 2.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.30,
            lower=0.0,
            upper=0.050,
        ),
    )
    model.articulation(
        "front_panel_mount",
        ArticulationType.FIXED,
        parent=chassis,
        child=front_panel,
        origin=Origin(xyz=(0.000, (CASE_D / 2.0) + 0.002, CASE_H / 2.0)),
    )
    model.articulation(
        "top_grille_mount",
        ArticulationType.FIXED,
        parent=chassis,
        child=top_grille,
        origin=Origin(xyz=(0.000, 0.000, CASE_H + 0.0015)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    front_panel = object_model.get_part("front_panel")
    top_grille = object_model.get_part("top_grille")
    side_panel_slide = object_model.get_articulation("side_panel_slide")

    right_guide_top = chassis.get_visual("right_guide_top")
    right_guide_bottom = chassis.get_visual("right_guide_bottom")
    panel_stop = chassis.get_visual("panel_stop")
    front_left_post = chassis.get_visual("front_left_post")
    front_right_post = chassis.get_visual("front_right_post")
    front_top_bar = chassis.get_visual("front_top_bar")
    roof_left_strip = chassis.get_visual("roof_left_strip")

    panel_shell = side_panel.get_visual("panel_shell")
    panel_top_flange = side_panel.get_visual("panel_top_flange")
    panel_bottom_flange = side_panel.get_visual("panel_bottom_flange")
    panel_front_lip = side_panel.get_visual("panel_front_lip")
    thumb_screw_top = side_panel.get_visual("thumb_screw_top")
    thumb_screw_bottom = side_panel.get_visual("thumb_screw_bottom")

    io_fascia = front_panel.get_visual("io_fascia")
    mesh_vertical_5 = front_panel.get_visual("mesh_vertical_5")
    mesh_top_bridge = front_panel.get_visual("mesh_top_bridge")
    mesh_bottom_bridge = front_panel.get_visual("mesh_bottom_bridge")
    snap_tab_tl = front_panel.get_visual("snap_tab_tl")
    snap_tab_bl = front_panel.get_visual("snap_tab_bl")
    snap_tab_tr = front_panel.get_visual("snap_tab_tr")
    snap_tab_br = front_panel.get_visual("snap_tab_br")
    power_button = front_panel.get_visual("power_button")
    usb_a = front_panel.get_visual("usb_a")
    usb_c = front_panel.get_visual("usb_c")
    audio_left = front_panel.get_visual("audio_left")
    audio_right = front_panel.get_visual("audio_right")

    grille_left_rail = top_grille.get_visual("grille_left_rail")
    grille_right_rail = top_grille.get_visual("grille_right_rail")
    grille_slat_3 = top_grille.get_visual("grille_slat_3")

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

    # Use prompt-specific exact visual checks as the real completion criteria.
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
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_gap(
        side_panel,
        chassis,
        axis="x",
        positive_elem=panel_top_flange,
        negative_elem=right_guide_top,
        max_gap=0.001,
        max_penetration=1e-5,
    )
    ctx.expect_gap(
        side_panel,
        chassis,
        axis="x",
        positive_elem=panel_bottom_flange,
        negative_elem=right_guide_bottom,
        max_gap=0.001,
        max_penetration=1e-5,
    )
    ctx.expect_gap(
        chassis,
        side_panel,
        axis="y",
        positive_elem=panel_stop,
        negative_elem=panel_front_lip,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_overlap(side_panel, chassis, axes="yz", min_overlap=0.04)
    ctx.expect_within(
        side_panel,
        side_panel,
        axes="yz",
        inner_elem=thumb_screw_top,
        outer_elem=panel_shell,
    )
    ctx.expect_within(
        side_panel,
        side_panel,
        axes="yz",
        inner_elem=thumb_screw_bottom,
        outer_elem=panel_shell,
    )
    ctx.expect_gap(
        side_panel,
        side_panel,
        axis="y",
        positive_elem=thumb_screw_top,
        negative_elem=side_panel.get_visual("panel_rear_hook"),
        min_gap=0.004,
        max_gap=0.010,
    )
    ctx.expect_gap(
        side_panel,
        side_panel,
        axis="y",
        positive_elem=thumb_screw_bottom,
        negative_elem=side_panel.get_visual("panel_rear_hook"),
        min_gap=0.004,
        max_gap=0.010,
    )

    ctx.expect_gap(
        front_panel,
        chassis,
        axis="y",
        positive_elem=io_fascia,
        negative_elem=front_top_bar,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_overlap(front_panel, chassis, axes="xz", min_overlap=0.18)
    ctx.expect_contact(front_panel, chassis, elem_a=snap_tab_tl, elem_b=front_left_post)
    ctx.expect_contact(front_panel, chassis, elem_a=snap_tab_bl, elem_b=front_left_post)
    ctx.expect_contact(front_panel, chassis, elem_a=snap_tab_tr, elem_b=front_right_post)
    ctx.expect_contact(front_panel, chassis, elem_a=snap_tab_br, elem_b=front_right_post)
    ctx.expect_contact(front_panel, front_panel, elem_a=mesh_vertical_5, elem_b=mesh_top_bridge)
    ctx.expect_contact(front_panel, front_panel, elem_a=mesh_vertical_5, elem_b=mesh_bottom_bridge)
    ctx.expect_gap(
        front_panel,
        front_panel,
        axis="z",
        positive_elem=io_fascia,
        negative_elem=mesh_top_bridge,
        max_gap=0.001,
        max_penetration=1e-5,
    )
    ctx.expect_within(
        front_panel,
        front_panel,
        axes="xz",
        inner_elem=power_button,
        outer_elem=io_fascia,
    )
    ctx.expect_within(
        front_panel,
        front_panel,
        axes="xz",
        inner_elem=usb_a,
        outer_elem=io_fascia,
    )
    ctx.expect_within(
        front_panel,
        front_panel,
        axes="xz",
        inner_elem=usb_c,
        outer_elem=io_fascia,
    )
    ctx.expect_within(
        front_panel,
        front_panel,
        axes="xz",
        inner_elem=audio_left,
        outer_elem=io_fascia,
    )
    ctx.expect_within(
        front_panel,
        front_panel,
        axes="xz",
        inner_elem=audio_right,
        outer_elem=io_fascia,
    )

    ctx.expect_gap(
        top_grille,
        chassis,
        axis="z",
        positive_elem=grille_left_rail,
        negative_elem=roof_left_strip,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_overlap(top_grille, chassis, axes="xy", min_overlap=0.15)
    ctx.expect_contact(top_grille, top_grille, elem_a=grille_slat_3, elem_b=grille_left_rail)
    ctx.expect_contact(top_grille, top_grille, elem_a=grille_slat_3, elem_b=grille_right_rail)

    with ctx.pose({side_panel_slide: 0.040}):
        ctx.expect_gap(
            chassis,
            side_panel,
            axis="y",
            positive_elem=panel_stop,
            negative_elem=panel_front_lip,
            min_gap=0.035,
        )
        ctx.expect_gap(
            side_panel,
            chassis,
            axis="x",
            positive_elem=panel_top_flange,
            negative_elem=right_guide_top,
            max_gap=0.001,
            max_penetration=1e-5,
        )
        ctx.expect_overlap(side_panel, chassis, axes="yz", min_overlap=0.20)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
