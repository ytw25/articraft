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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

FACE_BUTTON_LAYOUT = {
    "button_x": ((0.045, 0.019, 0.022), "button_blue"),
    "button_y": ((0.058, 0.032, 0.022), "button_yellow"),
    "button_a": ((0.058, 0.006, 0.022), "button_green"),
    "button_b": ((0.071, 0.019, 0.022), "button_red"),
}


def _rounded_section(width: float, height: float, y: float, z_center: float) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(
        width,
        height,
        radius=min(width, height) * 0.22,
        corner_segments=10,
    )
    return [(x, y, z + z_center) for x, z in profile]


def _build_shell_mesh():
    shell_sections = [
        _rounded_section(0.112, 0.022, 0.066, 0.010),
        _rounded_section(0.132, 0.028, 0.040, 0.009),
        _rounded_section(0.146, 0.032, 0.008, 0.008),
        _rounded_section(0.136, 0.034, -0.026, 0.005),
        _rounded_section(0.108, 0.026, -0.052, 0.001),
    ]
    shell_geom = repair_loft(section_loft(shell_sections))
    return mesh_from_geometry(shell_geom, ASSETS.mesh_path("controller_shell.obj"))


def _build_grip_mesh():
    grip_sections = [
        _rounded_section(0.046, 0.034, 0.006, 0.002),
        _rounded_section(0.052, 0.042, -0.030, -0.004),
        _rounded_section(0.044, 0.038, -0.066, -0.012),
    ]
    grip_geom = repair_loft(section_loft(grip_sections))
    return mesh_from_geometry(grip_geom, ASSETS.mesh_path("controller_grip.obj"))


def _add_thumbstick(
    model: ArticulatedObject,
    name: str,
    joint_origin: tuple[float, float, float],
    shaft_material,
    cap_material,
) -> None:
    stick = model.part(name)
    stick.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=shaft_material,
    )
    stick.visual(
        Cylinder(radius=0.0160, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=cap_material,
    )
    stick.visual(
        Cylinder(radius=0.0130, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=cap_material,
    )
    stick.visual(
        Cylinder(radius=0.0095, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=cap_material,
    )
    stick.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.024),
        mass=0.024,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    model.articulation(
        f"{name}_tilt",
        ArticulationType.REVOLUTE,
        parent="body",
        child=name,
        origin=Origin(xyz=joint_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.5,
            velocity=3.0,
            lower=-0.28,
            upper=0.28,
        ),
    )


def _add_face_button(
    model: ArticulatedObject,
    name: str,
    joint_origin: tuple[float, float, float],
    button_material,
) -> None:
    button = model.part(name)
    button.visual(
        Cylinder(radius=0.0072, length=0.0034),
        origin=Origin(xyz=(0.0, 0.0, 0.0017)),
        material=button_material,
    )
    button.visual(
        Cylinder(radius=0.0060, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0041)),
        material=button_material,
    )
    button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0075, length=0.004),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    model.articulation(
        f"{name}_press",
        ArticulationType.PRISMATIC,
        parent="body",
        child=name,
        origin=Origin(xyz=joint_origin),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0035,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_controller", assets=ASSETS)

    shell_material = model.material("shell", rgba=(0.14, 0.15, 0.17, 1.0))
    trim_material = model.material("trim", rgba=(0.22, 0.23, 0.26, 1.0))
    panel_material = model.material("panel", rgba=(0.05, 0.06, 0.07, 1.0))
    rubber_material = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    shaft_material = model.material("shaft", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("button_green", rgba=(0.20, 0.72, 0.33, 1.0))
    model.material("button_red", rgba=(0.78, 0.22, 0.20, 1.0))
    model.material("button_blue", rgba=(0.19, 0.45, 0.84, 1.0))
    model.material("button_yellow", rgba=(0.89, 0.76, 0.22, 1.0))

    shell_mesh = _build_shell_mesh()
    grip_mesh = _build_grip_mesh()
    body = model.part("body")
    body.visual(shell_mesh, material=shell_material)
    body.visual(
        grip_mesh,
        origin=Origin(xyz=(-0.064, -0.014, 0.0)),
        material=shell_material,
    )
    body.visual(
        grip_mesh,
        origin=Origin(xyz=(0.064, -0.014, 0.0)),
        material=shell_material,
    )
    body.visual(
        Box((0.050, 0.028, 0.003)),
        origin=Origin(xyz=(0.0, 0.004, 0.0235)),
        material=panel_material,
    )
    body.visual(
        Box((0.024, 0.008, 0.004)),
        origin=Origin(xyz=(-0.056, -0.018, 0.023)),
        material=trim_material,
    )
    body.visual(
        Box((0.008, 0.024, 0.004)),
        origin=Origin(xyz=(-0.056, -0.018, 0.023)),
        material=trim_material,
    )
    body.visual(
        Box((0.040, 0.018, 0.007)),
        origin=Origin(xyz=(-0.045, 0.059, 0.0215)),
        material=trim_material,
    )
    body.visual(
        Box((0.040, 0.018, 0.007)),
        origin=Origin(xyz=(0.045, 0.059, 0.0215)),
        material=trim_material,
    )
    body.visual(
        Cylinder(radius=0.017, length=0.002),
        origin=Origin(xyz=(-0.046, 0.011, 0.022)),
        material=trim_material,
    )
    body.visual(
        Cylinder(radius=0.017, length=0.002),
        origin=Origin(xyz=(0.029, -0.022, 0.022)),
        material=trim_material,
    )
    body.visual(
        Box((0.010, 0.006, 0.002)),
        origin=Origin(xyz=(-0.014, 0.004, 0.0235)),
        material=trim_material,
    )
    body.visual(
        Box((0.010, 0.006, 0.002)),
        origin=Origin(xyz=(0.014, 0.004, 0.0235)),
        material=trim_material,
    )
    body.visual(
        Cylinder(radius=0.007, length=0.002),
        origin=Origin(xyz=(0.0, 0.011, 0.0235)),
        material=trim_material,
    )
    body.inertial = Inertial.from_geometry(
        Box((0.180, 0.150, 0.055)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )

    _add_thumbstick(
        model,
        "left_stick",
        (-0.046, 0.011, 0.0245),
        shaft_material,
        rubber_material,
    )
    _add_thumbstick(
        model,
        "right_stick",
        (0.029, -0.022, 0.0245),
        shaft_material,
        rubber_material,
    )

    for button_name, (joint_origin, material_name) in FACE_BUTTON_LAYOUT.items():
        _add_face_button(
            model,
            button_name,
            joint_origin,
            material_name,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected(use="visual")
    for button_name in FACE_BUTTON_LAYOUT:
        ctx.allow_overlap("body", button_name, reason="button cap is allowed to travel into the shell during a press")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_overlap("left_stick", "body", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("left_stick", "body", axis="z", max_gap=0.002, max_penetration=0.001)
    ctx.expect_joint_motion_axis("left_stick_tilt", "left_stick", world_axis="y", direction="negative", min_delta=0.006)

    ctx.expect_aabb_overlap("right_stick", "body", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("right_stick", "body", axis="z", max_gap=0.002, max_penetration=0.001)
    ctx.expect_joint_motion_axis("right_stick_tilt", "right_stick", world_axis="y", direction="negative", min_delta=0.006)

    for button_name in FACE_BUTTON_LAYOUT:
        ctx.expect_aabb_overlap(button_name, "body", axes="xy", min_overlap=0.009)
        ctx.expect_aabb_gap(button_name, "body", axis="z", max_gap=0.001, max_penetration=0.004)
        ctx.expect_joint_motion_axis(
            f"{button_name}_press",
            button_name,
            world_axis="z",
            direction="negative",
            min_delta=0.0015,
        )

    left_stick_pos = ctx.part_world_position("left_stick")
    right_stick_pos = ctx.part_world_position("right_stick")
    button_x_pos = ctx.part_world_position("button_x")
    button_y_pos = ctx.part_world_position("button_y")
    button_a_pos = ctx.part_world_position("button_a")
    button_b_pos = ctx.part_world_position("button_b")

    assert left_stick_pos[0] < -0.030, "left thumbstick should sit on the left half of the controller"
    assert left_stick_pos[1] > 0.0, "left thumbstick should sit in the upper-left play position"
    assert right_stick_pos[0] > 0.010, "right thumbstick should sit on the right half of the controller"
    assert right_stick_pos[1] < -0.005, "right thumbstick should sit lower than the face-button cluster"
    assert button_y_pos[1] > button_a_pos[1] + 0.020, "face buttons should form a vertical diamond"
    assert button_b_pos[0] > button_x_pos[0] + 0.020, "face buttons should form a horizontal diamond"

    with ctx.pose(button_a_press=0.0035):
        pressed_a_pos = ctx.part_world_position("button_a")
    assert pressed_a_pos[2] < button_a_pos[2] - 0.002, "button A should visibly travel downward when pressed"

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
