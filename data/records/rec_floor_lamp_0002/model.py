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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _foot_positions(radius: float) -> list[tuple[float, float]]:
    return [
        (radius * math.cos(angle), radius * math.sin(angle))
        for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    ]


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_shade_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.020, 0.004),
            (0.030, 0.011),
            (0.070, 0.025),
            (0.122, 0.051),
            (0.163, 0.073),
            (0.180, 0.082),
        ],
        [
            (0.014, 0.010),
            (0.021, 0.016),
            (0.056, 0.030),
            (0.111, 0.056),
            (0.155, 0.075),
            (0.171, 0.078),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="torchiere_floor_lamp", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.15, 0.15, 0.16, 1.0))
    aged_bronze = model.material("aged_bronze", rgba=(0.24, 0.21, 0.18, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    uplight_glass = model.material("uplight_glass", rgba=(0.94, 0.89, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.165, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=cast_iron,
        name="base_disc",
    )
    body.visual(
        Cylinder(radius=0.060, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=cast_iron,
        name="base_boss",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=aged_bronze,
        name="pole_collar",
    )
    body.visual(
        Cylinder(radius=0.0115, length=1.464),
        origin=Origin(xyz=(0.0, 0.0, 0.813)),
        material=aged_bronze,
        name="pole_shaft",
    )
    body.visual(
        Cylinder(radius=0.0145, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.020)),
        material=aged_bronze,
        name="control_mount",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 1.555)),
        material=aged_bronze,
        name="pole_tip",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.010),
        origin=Origin(xyz=(0.005, 0.0, 1.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_bronze,
        name="stub_shoulder",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.012),
        origin=Origin(xyz=(0.016, 0.0, 1.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_bronze,
        name="thread_core",
    )
    for index, x_pos in enumerate((0.0115, 0.0155, 0.0195), start=1):
        body.visual(
            Cylinder(radius=0.0053, length=0.002),
            origin=Origin(xyz=(x_pos, 0.0, 1.020), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aged_bronze,
            name=f"thread_ridge_{index}",
        )
    body.visual(
        Cylinder(radius=0.005, length=0.002),
        origin=Origin(xyz=(0.023, 0.0, 1.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_bronze,
        name="thread_tip",
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=uplight_glass,
        name="shade_neck",
    )
    shade.visual(
        _save_mesh("torchiere_shade_bowl.obj", _build_shade_shell_mesh()),
        material=uplight_glass,
        name="shade_bowl",
    )

    knob = model.part("dimmer_knob")
    knob.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="knob_face",
    )
    knob.visual(
        Box((0.0015, 0.010, 0.002)),
        origin=Origin(xyz=(0.0174, 0.0, 0.011)),
        material=uplight_glass,
        name="knob_indicator",
    )

    foot_names = ("foot_front", "foot_left", "foot_right")
    for foot_name in foot_names:
        foot = model.part(foot_name)
        foot.visual(
            Cylinder(radius=0.016, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, 0.0035)),
            material=rubber,
            name="foot_pad",
        )

    model.articulation(
        "body_to_shade",
        ArticulationType.FIXED,
        parent=body,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, 1.565)),
    )
    model.articulation(
        "body_to_dimmer_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.024, 0.0, 1.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )
    for foot_name, (x_pos, y_pos) in zip(foot_names, _foot_positions(0.112)):
        model.articulation(
            f"body_to_{foot_name}",
            ArticulationType.FIXED,
            parent=body,
            child=foot_name,
            origin=Origin(xyz=(x_pos, y_pos, 0.0)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    shade = object_model.get_part("shade")
    knob = object_model.get_part("dimmer_knob")
    foot_front = object_model.get_part("foot_front")
    foot_left = object_model.get_part("foot_left")
    foot_right = object_model.get_part("foot_right")
    knob_spin = object_model.get_articulation("body_to_dimmer_knob")

    base_disc = body.get_visual("base_disc")
    control_mount = body.get_visual("control_mount")
    pole_tip = body.get_visual("pole_tip")
    thread_tip = body.get_visual("thread_tip")
    shade_bowl = shade.get_visual("shade_bowl")
    knob_body = knob.get_visual("knob_body")

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

    ctx.expect_contact(shade, body, elem_b=pole_tip)
    ctx.expect_gap(
        shade,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        negative_elem=pole_tip,
    )
    ctx.expect_origin_distance(shade, body, axes="xy", max_dist=0.01)
    ctx.expect_overlap(shade, body, axes="xy", min_overlap=0.085, elem_a=shade_bowl, elem_b=base_disc)
    ctx.expect_gap(shade, foot_front, axis="z", min_gap=1.45)

    ctx.expect_contact(knob, body, elem_a=knob_body, elem_b=thread_tip)
    ctx.expect_gap(
        knob,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=knob_body,
        negative_elem=thread_tip,
    )
    ctx.expect_overlap(
        knob,
        body,
        axes="yz",
        min_overlap=0.0003,
        elem_a=knob_body,
        elem_b=control_mount,
    )
    ctx.expect_gap(knob, foot_front, axis="z", min_gap=0.95, max_gap=1.08)

    for foot in (foot_front, foot_left, foot_right):
        foot_pad = foot.get_visual("foot_pad")
        ctx.expect_contact(foot, body, elem_a=foot_pad, elem_b=base_disc)
        ctx.expect_gap(
            body,
            foot,
            axis="z",
            max_gap=0.001,
            max_penetration=1e-6,
            positive_elem=base_disc,
            negative_elem=foot_pad,
        )
        ctx.expect_within(foot, body, axes="xy", inner_elem=foot_pad, outer_elem=base_disc)

    with ctx.pose({knob_spin: 2.2}):
        ctx.expect_contact(knob, body, elem_a=knob_body, elem_b=thread_tip)
        ctx.expect_gap(
            knob,
            body,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=knob_body,
            negative_elem=thread_tip,
        )
        ctx.expect_overlap(
            knob,
            body,
            axes="yz",
            min_overlap=0.0003,
            elem_a=knob_body,
            elem_b=control_mount,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
