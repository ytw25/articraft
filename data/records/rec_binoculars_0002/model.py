from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
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
    section_loft,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)

HALF_SPREAD = 0.031
OBJECTIVE_RADIUS = 0.0155
EYEPIECE_RADIUS = 0.0105
OPTICAL_Z = -0.012


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _body_section(
    x_pos: float,
    y_center: float,
    z_center: float,
    width: float,
    height: float,
    *,
    exponent: float = 2.6,
    segments: int = 22,
):
    return [
        (x_pos, y_center + y_off, z_center + z_off)
        for y_off, z_off in superellipse_profile(width, height, exponent=exponent, segments=segments)
    ]


def _build_body_shell(name: str, sign: float):
    sections = [
        _body_section(-0.078, sign * HALF_SPREAD, -0.004, 0.026, 0.030, exponent=2.4),
        _body_section(-0.046, sign * HALF_SPREAD, -0.003, 0.030, 0.034, exponent=2.8),
        _body_section(-0.014, sign * HALF_SPREAD, -0.002, 0.031, 0.033, exponent=2.8),
        _body_section(0.010, sign * HALF_SPREAD, -0.004, 0.024, 0.026, exponent=2.4),
    ]
    return _save_mesh(name, repair_loft(section_loft(sections)))


def _build_ring_band(name: str, *, outer_radius: float, inner_radius: float, length: float):
    half = 0.5 * length
    geometry = LatheGeometry.from_shell_profiles(
        [
            (outer_radius * 0.94, -half),
            (outer_radius, -half + 0.0012),
            (outer_radius, half - 0.0012),
            (outer_radius * 0.94, half),
        ],
        [
            (inner_radius, -half),
            (inner_radius, half),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)
    return _save_mesh(name, geometry)


def _build_focus_wheel_mesh():
    geometry = LatheGeometry.from_shell_profiles(
        [
            (0.0065, -0.006),
            (0.0088, -0.0035),
            (0.0092, 0.0),
            (0.0088, 0.0035),
            (0.0065, 0.006),
        ],
        [
            (0.0030, -0.006),
            (0.0030, 0.006),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    ).rotate_x(math.pi / 2.0)
    return _save_mesh("focus_wheel.obj", geometry)


def _build_eyecup_shell_mesh():
    geometry = LatheGeometry.from_shell_profiles(
        [
            (0.0132, -0.006),
            (0.0144, -0.001),
            (0.0154, 0.006),
            (0.0154, 0.012),
        ],
        [
            (0.0114, -0.0055),
            (0.0120, -0.0005),
            (0.0124, 0.006),
            (0.0124, 0.011),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi / 2.0)
    return _save_mesh("eyecup_shell.obj", geometry)


def _add_body_half(model: ArticulatedObject, name: str, sign: float, armor, metal, ring_mesh):
    body = model.part(name)
    hinge_sleeve_x = -0.010 if sign < 0.0 else -0.005
    body.visual(
        _build_body_shell(f"{name}_shell.obj", sign),
        material=armor,
        name="body_shell",
    )
    body.visual(
        Box((0.044, 0.018, 0.010)),
        origin=Origin(xyz=(-0.034, sign * HALF_SPREAD, 0.005)),
        material=armor,
        name="roof_prism_block",
    )
    body.visual(
        Box((0.022, 0.010, 0.010)),
        origin=Origin(xyz=(-0.016, sign * 0.021, 0.003)),
        material=metal,
        name="bridge_shoulder",
    )
    body.visual(
        Box((0.018, 0.018, 0.008)),
        origin=Origin(xyz=(-0.016, sign * 0.009, 0.0)),
        material=metal,
        name="bridge_yoke",
    )
    body.visual(
        Cylinder(radius=0.0048, length=0.005),
        origin=Origin(
            xyz=(hinge_sleeve_x, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="hinge_sleeve",
    )
    body.visual(
        Box((0.020, 0.018, 0.014)),
        origin=Origin(xyz=(0.010, sign * HALF_SPREAD, -0.008)),
        material=armor,
        name="objective_root",
    )
    body.visual(
        Cylinder(radius=OBJECTIVE_RADIUS, length=0.072),
        origin=Origin(
            xyz=(0.046, sign * HALF_SPREAD, OPTICAL_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=armor,
        name="objective_tube",
    )
    body.visual(
        ring_mesh,
        origin=Origin(xyz=(0.058, sign * HALF_SPREAD, OPTICAL_Z)),
        material=metal,
        name="diopter_ring",
    )
    body.visual(
        Cylinder(radius=0.0166, length=0.008),
        origin=Origin(
            xyz=(0.082, sign * HALF_SPREAD, OPTICAL_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="objective_bezel",
    )
    body.visual(
        Cylinder(radius=EYEPIECE_RADIUS, length=0.020),
        origin=Origin(
            xyz=(-0.071, sign * HALF_SPREAD, OPTICAL_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=metal,
        name="eyepiece_sleeve",
    )
    body.visual(
        Box((0.006, 0.008, 0.004)),
        origin=Origin(xyz=(-0.061, sign * HALF_SPREAD, 0.014)),
        material=metal,
        name="eyecup_hinge_lug",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.170, 0.040, 0.040)),
        mass=0.24,
        origin=Origin(xyz=(0.000, sign * HALF_SPREAD, -0.002)),
    )
    return body


def _add_eyecup(model: ArticulatedObject, name: str, rubber, eyecup_mesh):
    eyecup = model.part(name)
    eyecup.visual(
        eyecup_mesh,
        origin=Origin(xyz=(-0.028, 0.0, -0.030)),
        material=rubber,
        name="eyecup_shell",
    )
    eyecup.visual(
        Box((0.020, 0.006, 0.004)),
        origin=Origin(xyz=(-0.014, 0.0, 0.0)),
        material=rubber,
        name="hinge_tab",
    )
    eyecup.visual(
        Box((0.006, 0.006, 0.022)),
        origin=Origin(xyz=(-0.026, 0.0, -0.012)),
        material=rubber,
        name="hinge_drop",
    )
    eyecup.inertial = Inertial.from_geometry(
        Box((0.040, 0.032, 0.040)),
        mass=0.020,
        origin=Origin(xyz=(-0.026, 0.0, -0.016)),
    )
    return eyecup


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_roof_prism_binoculars", assets=ASSETS)

    armor = model.material("armor_black", rgba=(0.12, 0.13, 0.14, 1.0))
    metal = model.material("anodized_graphite", rgba=(0.31, 0.33, 0.36, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.24, 0.25, 0.27, 1.0))
    rubber = model.material("eyecup_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    diopter_ring_mesh = _build_ring_band(
        "diopter_ring.obj",
        outer_radius=0.0174,
        inner_radius=OBJECTIVE_RADIUS - 0.0004,
        length=0.007,
    )
    eyecup_mesh = _build_eyecup_shell_mesh()
    focus_wheel_mesh = _build_focus_wheel_mesh()

    bridge_core = model.part("bridge_core")
    bridge_core.visual(
        Cylinder(radius=0.0036, length=0.050),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )
    bridge_core.visual(
        Box((0.012, 0.008, 0.014)),
        origin=Origin(xyz=(-0.018, 0.0, 0.004)),
        material=metal,
        name="bridge_spine",
    )
    bridge_core.visual(
        Box((0.022, 0.018, 0.006)),
        origin=Origin(xyz=(-0.022, 0.0, 0.012)),
        material=metal,
        name="bridge_beam",
    )
    bridge_core.visual(
        Box((0.014, 0.008, 0.016)),
        origin=Origin(xyz=(-0.030, 0.010, 0.019)),
        material=metal,
        name="focus_carrier",
    )
    bridge_core.visual(
        Cylinder(radius=0.0024, length=0.048),
        origin=Origin(
            xyz=(-0.024, 0.0, 0.028),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="focus_shaft",
    )
    bridge_core.visual(
        Cylinder(radius=0.0030, length=0.004),
        origin=Origin(
            xyz=(-0.024, -0.014, 0.028),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="left_thread_collar",
    )
    bridge_core.visual(
        Cylinder(radius=0.0030, length=0.004),
        origin=Origin(
            xyz=(-0.024, 0.014, 0.028),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="right_thread_collar",
    )
    bridge_core.inertial = Inertial.from_geometry(
        Box((0.064, 0.054, 0.038)),
        mass=0.16,
        origin=Origin(xyz=(-0.018, 0.0, 0.014)),
    )

    left_body = _add_body_half(model, "left_body", -1.0, armor, metal, diopter_ring_mesh)
    right_body = _add_body_half(model, "right_body", 1.0, armor, metal, diopter_ring_mesh)

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        focus_wheel_mesh,
        material=wheel_metal,
        name="focus_wheel",
    )
    focus_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0090, length=0.010),
        mass=0.030,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    left_eyecup = _add_eyecup(model, "left_eyecup", rubber, eyecup_mesh)
    right_eyecup = _add_eyecup(model, "right_eyecup", rubber, eyecup_mesh)

    model.articulation(
        "bridge_to_left_body",
        ArticulationType.REVOLUTE,
        parent=bridge_core,
        child=left_body,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=-0.18, upper=0.08),
    )
    model.articulation(
        "bridge_to_right_body",
        ArticulationType.REVOLUTE,
        parent=bridge_core,
        child=right_body,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.2, lower=-0.08, upper=0.18),
    )
    model.articulation(
        "bridge_to_focus_wheel",
        ArticulationType.CONTINUOUS,
        parent=bridge_core,
        child=focus_wheel,
        origin=Origin(xyz=(-0.024, 0.0, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )
    model.articulation(
        "left_body_to_eyecup",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=left_eyecup,
        origin=Origin(xyz=(-0.061, -HALF_SPREAD, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=-1.15, upper=0.0),
    )
    model.articulation(
        "right_body_to_eyecup",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=right_eyecup,
        origin=Origin(xyz=(-0.061, HALF_SPREAD, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.0, lower=-1.15, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge_core = object_model.get_part("bridge_core")
    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_wheel = object_model.get_part("focus_wheel")
    left_eyecup = object_model.get_part("left_eyecup")
    right_eyecup = object_model.get_part("right_eyecup")

    left_bridge = object_model.get_articulation("bridge_to_left_body")
    right_bridge = object_model.get_articulation("bridge_to_right_body")
    left_eyecup_hinge = object_model.get_articulation("left_body_to_eyecup")
    right_eyecup_hinge = object_model.get_articulation("right_body_to_eyecup")

    hinge_pin = bridge_core.get_visual("hinge_pin")
    bridge_beam = bridge_core.get_visual("bridge_beam")
    focus_shaft = bridge_core.get_visual("focus_shaft")
    left_objective = left_body.get_visual("objective_tube")
    right_objective = right_body.get_visual("objective_tube")
    left_diopter = left_body.get_visual("diopter_ring")
    right_diopter = right_body.get_visual("diopter_ring")
    left_bridge_yoke = left_body.get_visual("bridge_yoke")
    right_bridge_yoke = right_body.get_visual("bridge_yoke")
    left_hinge_sleeve = left_body.get_visual("hinge_sleeve")
    right_hinge_sleeve = right_body.get_visual("hinge_sleeve")
    left_eyepiece = left_body.get_visual("eyepiece_sleeve")
    right_eyepiece = right_body.get_visual("eyepiece_sleeve")
    left_eyecup_lug = left_body.get_visual("eyecup_hinge_lug")
    right_eyecup_lug = right_body.get_visual("eyecup_hinge_lug")
    wheel_body = focus_wheel.get_visual("focus_wheel")
    left_cup_shell = left_eyecup.get_visual("eyecup_shell")
    right_cup_shell = right_eyecup.get_visual("eyecup_shell")
    left_hinge_tab = left_eyecup.get_visual("hinge_tab")
    right_hinge_tab = right_eyecup.get_visual("hinge_tab")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        bridge_core,
        left_body,
        reason="central hinge pin nests within the left hinge sleeve for interpupillary adjustment",
    )
    ctx.allow_overlap(
        bridge_core,
        right_body,
        reason="central hinge pin nests within the right hinge sleeve for interpupillary adjustment",
    )
    ctx.allow_overlap(
        left_body,
        left_eyecup,
        reason="folding eyecup hinge tab nests around the rear hinge lug",
    )
    ctx.allow_overlap(
        right_body,
        right_eyecup,
        reason="folding eyecup hinge tab nests around the rear hinge lug",
    )
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        left_body,
        right_body,
        axes="xz",
        min_overlap=0.010,
        elem_a=left_objective,
        elem_b=right_objective,
    )
    ctx.expect_gap(
        right_body,
        left_body,
        axis="y",
        min_gap=0.028,
        max_gap=0.040,
        positive_elem=right_objective,
        negative_elem=left_objective,
    )
    ctx.expect_overlap(
        bridge_core,
        left_body,
        axes="xz",
        min_overlap=0.004,
        elem_a=hinge_pin,
        elem_b=left_hinge_sleeve,
    )
    ctx.expect_overlap(
        bridge_core,
        right_body,
        axes="xz",
        min_overlap=0.004,
        elem_a=hinge_pin,
        elem_b=right_hinge_sleeve,
    )
    ctx.expect_overlap(
        bridge_core,
        left_body,
        axes="xz",
        min_overlap=0.003,
        elem_a=bridge_core.get_visual("bridge_spine"),
        elem_b=left_bridge_yoke,
    )
    ctx.expect_overlap(
        bridge_core,
        right_body,
        axes="xz",
        min_overlap=0.003,
        elem_a=bridge_core.get_visual("bridge_spine"),
        elem_b=right_bridge_yoke,
    )
    ctx.expect_overlap(
        left_body,
        left_body,
        axes="yz",
        min_overlap=0.0003,
        elem_a=left_diopter,
        elem_b=left_objective,
    )
    ctx.expect_overlap(
        right_body,
        right_body,
        axes="yz",
        min_overlap=0.0003,
        elem_a=right_diopter,
        elem_b=right_objective,
    )
    ctx.expect_within(
        bridge_core,
        focus_wheel,
        axes="xz",
        inner_elem=focus_shaft,
        outer_elem=wheel_body,
    )
    ctx.expect_gap(
        focus_wheel,
        bridge_core,
        axis="z",
        min_gap=0.002,
        max_gap=0.005,
        positive_elem=wheel_body,
        negative_elem=bridge_beam,
    )
    ctx.expect_within(
        left_body,
        left_eyecup,
        axes="yz",
        inner_elem=left_eyepiece,
        outer_elem=left_cup_shell,
    )
    ctx.expect_within(
        right_body,
        right_eyecup,
        axes="yz",
        inner_elem=right_eyepiece,
        outer_elem=right_cup_shell,
    )
    ctx.expect_gap(
        left_body,
        left_eyecup,
        axis="x",
        max_gap=0.0012,
        max_penetration=0.003,
        positive_elem=left_eyecup_lug,
        negative_elem=left_hinge_tab,
    )
    ctx.expect_gap(
        right_body,
        right_eyecup,
        axis="x",
        max_gap=0.0012,
        max_penetration=0.003,
        positive_elem=right_eyecup_lug,
        negative_elem=right_hinge_tab,
    )

    with ctx.pose({left_bridge: -0.10, right_bridge: 0.10}):
        ctx.expect_gap(
            right_body,
            left_body,
            axis="y",
            min_gap=0.016,
            max_gap=0.036,
            positive_elem=right_objective,
            negative_elem=left_objective,
        )
        ctx.expect_overlap(
            left_body,
            right_body,
            axes="xz",
            min_overlap=0.010,
            elem_a=left_objective,
            elem_b=right_objective,
        )

    with ctx.pose({left_eyecup_hinge: -1.10, right_eyecup_hinge: -1.10}):
        ctx.expect_overlap(
            left_body,
            left_eyecup,
            axes="yz",
            min_overlap=0.018,
            elem_a=left_eyepiece,
            elem_b=left_cup_shell,
        )
        ctx.expect_overlap(
            right_body,
            right_eyecup,
            axes="yz",
            min_overlap=0.018,
            elem_a=right_eyepiece,
            elem_b=right_cup_shell,
        )
        ctx.expect_gap(
            left_body,
            left_eyecup,
            axis="x",
            max_gap=0.0012,
            max_penetration=0.003,
            positive_elem=left_eyecup_lug,
            negative_elem=left_hinge_tab,
        )
        ctx.expect_gap(
            right_body,
            right_eyecup,
            axis="x",
            max_gap=0.0012,
            max_penetration=0.003,
            positive_elem=right_eyecup_lug,
            negative_elem=right_hinge_tab,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
