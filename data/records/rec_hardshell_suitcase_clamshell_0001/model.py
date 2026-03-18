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
    ExtrudeWithHolesGeometry,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = HERE / "meshes"
MESH_DIR.mkdir(parents=True, exist_ok=True)

BODY_W = 0.43
BODY_L = 0.67
LOWER_H = 0.118
UPPER_H = 0.103
CORNER_R = 0.066

HANDLE_SPAN = 0.175
HANDLE_Y = BODY_L * 0.56
HANDLE_AXIS_Z = UPPER_H - 0.001

WHEEL_R = 0.036
WHEEL_W = 0.026
WHEEL_Y = -BODY_L / 2.0 + 0.002
LEFT_WHEEL_X = -0.158
RIGHT_WHEEL_X = 0.158
WHEEL_Z = 0.031


def _rounded_profile_xyz(
    width: float,
    length: float,
    radius: float,
    z: float,
    *,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + center_y, z)
        for x, y in rounded_rect_profile(
            width,
            length,
            radius=max(0.001, radius),
            corner_segments=10,
        )
    ]


def _mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, MESH_DIR / f"{name}.obj")


def _shell_mesh(
    name: str,
    sections: list[tuple[float, float, float, float]],
    *,
    center_y: float = 0.0,
):
    profiles = [
        _rounded_profile_xyz(width, length, radius, z, center_y=center_y)
        for width, length, radius, z in sections
    ]
    return _mesh(name, LoftGeometry(profiles, cap=True, closed=True))


def _ring_mesh(
    name: str,
    *,
    outer_w: float,
    outer_l: float,
    outer_r: float,
    inner_w: float,
    inner_l: float,
    inner_r: float,
    height: float,
):
    outer = rounded_rect_profile(outer_w, outer_l, outer_r, corner_segments=10)
    inner = rounded_rect_profile(inner_w, inner_l, inner_r, corner_segments=10)
    return _mesh(
        name,
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            height=height,
            cap=True,
            center=False,
            closed=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hard_shell_suitcase", assets=ASSETS)

    shell = model.material("polycarbonate_graphite", rgba=(0.19, 0.21, 0.24, 1.0))
    trim = model.material("zipper_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber = model.material("wheel_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    plastic = model.material("black_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("hardware_steel", rgba=(0.58, 0.60, 0.63, 1.0))

    rear_shell = model.part("rear_shell")
    rear_shell.visual(
        _shell_mesh(
            "rear_shell_body",
            [
                (BODY_W * 0.83, BODY_L * 0.87, CORNER_R * 0.78, 0.0),
                (BODY_W * 0.93, BODY_L * 0.96, CORNER_R * 0.90, LOWER_H * 0.46),
                (BODY_W * 1.00, BODY_L * 1.00, CORNER_R * 1.00, LOWER_H * 0.84),
                (BODY_W * 0.985, BODY_L * 0.985, CORNER_R * 0.96, LOWER_H),
            ],
        ),
        material=shell,
    )
    rear_shell.visual(
        _ring_mesh(
            "rear_seam_band",
            outer_w=BODY_W * 0.985,
            outer_l=BODY_L * 0.985,
            outer_r=CORNER_R * 0.96,
            inner_w=BODY_W * 0.86,
            inner_l=BODY_L * 0.88,
            inner_r=CORNER_R * 0.78,
            height=0.006,
        ),
        origin=Origin(xyz=(0.0, 0.0, LOWER_H - 0.006)),
        material=trim,
    )
    for x in (-0.115, 0.0, 0.115):
        rear_shell.visual(
            Box((0.055, BODY_L * 0.52, 0.010)),
            origin=Origin(xyz=(x, -0.008, LOWER_H - 0.008)),
            material=shell,
        )
    for x in (LEFT_WHEEL_X, RIGHT_WHEEL_X):
        rear_shell.visual(
            Box((0.060, 0.082, 0.050)),
            origin=Origin(xyz=(x, -BODY_L / 2.0 + 0.041, 0.025)),
            material=plastic,
        )
        rear_shell.visual(
            Box((0.024, 0.016, 0.024)),
            origin=Origin(xyz=(x, WHEEL_Y, WHEEL_Z)),
            material=steel,
        )
    rear_shell.visual(
        Box((0.058, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, BODY_L / 2.0 - 0.013, LOWER_H - 0.015)),
        material=plastic,
    )
    rear_shell.visual(
        Box((0.036, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, BODY_L / 2.0 - 0.013, LOWER_H - 0.003)),
        material=aluminum,
    )
    for x in (-0.13, 0.13):
        rear_shell.visual(
            Cylinder(radius=0.006, length=0.072),
            origin=Origin(
                xyz=(x, -BODY_L / 2.0 + 0.006, LOWER_H - 0.005),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
        )
    for x in (-0.075, 0.075):
        rear_shell.visual(
            Box((0.018, 0.170, 0.006)),
            origin=Origin(xyz=(x, -0.215, LOWER_H - 0.010)),
            material=aluminum,
        )
    rear_shell.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_L, LOWER_H)),
        mass=3.3,
        origin=Origin(xyz=(0.0, 0.0, LOWER_H / 2.0)),
    )

    front_shell = model.part("front_shell")
    front_shell.visual(
        _shell_mesh(
            "front_shell_body",
            [
                (BODY_W * 0.985, BODY_L * 1.00, CORNER_R * 0.96, 0.0),
                (BODY_W * 0.97, BODY_L * 0.97, CORNER_R * 0.93, UPPER_H * 0.36),
                (BODY_W * 0.92, BODY_L * 0.93, CORNER_R * 0.88, UPPER_H * 0.78),
                (BODY_W * 0.86, BODY_L * 0.88, CORNER_R * 0.81, UPPER_H),
            ],
            center_y=BODY_L / 2.0,
        ),
        material=shell,
    )
    front_shell.visual(
        _ring_mesh(
            "front_gasket_band",
            outer_w=BODY_W * 0.982,
            outer_l=BODY_L * 0.995,
            outer_r=CORNER_R * 0.95,
            inner_w=BODY_W * 0.89,
            inner_l=BODY_L * 0.90,
            inner_r=CORNER_R * 0.80,
            height=0.004,
        ),
        origin=Origin(xyz=(0.0, BODY_L / 2.0, -0.004)),
        material=trim,
    )
    for x in (-0.105, 0.0, 0.105):
        front_shell.visual(
            Box((0.050, BODY_L * 0.47, 0.009)),
            origin=Origin(xyz=(x, BODY_L * 0.56, UPPER_H - 0.006)),
            material=shell,
        )
    front_shell.visual(
        Box((0.046, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, BODY_L - 0.014, -0.003)),
        material=aluminum,
    )
    for x in (-0.13, 0.13):
        front_shell.visual(
            Cylinder(radius=0.0055, length=0.068),
            origin=Origin(
                xyz=(x, 0.006, -0.0035),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
        )
    for x in (-HANDLE_SPAN / 2.0, HANDLE_SPAN / 2.0):
        front_shell.visual(
            Box((0.028, 0.018, 0.012)),
            origin=Origin(xyz=(x, HANDLE_Y, UPPER_H - 0.004)),
            material=plastic,
        )
    front_shell.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_L, UPPER_H)),
        mass=2.5,
        origin=Origin(xyz=(0.0, BODY_L / 2.0, UPPER_H / 2.0)),
    )

    carry_handle = model.part("carry_handle")
    carry_handle.visual(
        _mesh(
            "carry_handle_arch",
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.0),
                    (HANDLE_SPAN * 0.18, 0.018, 0.005),
                    (HANDLE_SPAN * 0.50, 0.028, 0.010),
                    (HANDLE_SPAN * 0.82, 0.018, 0.005),
                    (HANDLE_SPAN, 0.0, 0.0),
                ],
                radius=0.0075,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
        ),
        material=plastic,
    )
    carry_handle.visual(
        Box((0.105, 0.022, 0.012)),
        origin=Origin(xyz=(HANDLE_SPAN / 2.0, 0.019, 0.007)),
        material=aluminum,
    )
    carry_handle.inertial = Inertial.from_geometry(
        Box((HANDLE_SPAN, 0.035, 0.026)),
        mass=0.28,
        origin=Origin(xyz=(HANDLE_SPAN / 2.0, 0.016, 0.008)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_W),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
    )
    left_wheel.visual(
        Cylinder(radius=0.018, length=WHEEL_W + 0.002),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
    )
    left_wheel.inertial = Inertial.from_geometry(
        Box((WHEEL_W, WHEEL_R * 2.0, WHEEL_R * 2.0)),
        mass=0.18,
        origin=Origin(),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=WHEEL_R, length=WHEEL_W),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
    )
    right_wheel.visual(
        Cylinder(radius=0.018, length=WHEEL_W + 0.002),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
    )
    right_wheel.inertial = Inertial.from_geometry(
        Box((WHEEL_W, WHEEL_R * 2.0, WHEEL_R * 2.0)),
        mass=0.18,
        origin=Origin(),
    )

    model.articulation(
        "shell_hinge",
        ArticulationType.REVOLUTE,
        parent="rear_shell",
        child="front_shell",
        origin=Origin(xyz=(0.0, -BODY_L / 2.0, LOWER_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "carry_handle_hinge",
        ArticulationType.REVOLUTE,
        parent="front_shell",
        child="carry_handle",
        origin=Origin(xyz=(-HANDLE_SPAN / 2.0, HANDLE_Y, HANDLE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=4.0,
            lower=0.0,
            upper=1.2,
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="rear_shell",
        child="left_wheel",
        origin=Origin(xyz=(LEFT_WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=18.0,
        ),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="rear_shell",
        child="right_wheel",
        origin=Origin(xyz=(RIGHT_WHEEL_X, WHEEL_Y, WHEEL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=18.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.allow_overlap(
        "rear_shell",
        "front_shell",
        reason="closed clamshell seam uses slightly nested generated collision hulls",
    )
    ctx.allow_overlap(
        "rear_shell",
        "left_wheel",
        reason="wheel is partially recessed into the molded wheel housing",
    )
    ctx.allow_overlap(
        "rear_shell",
        "right_wheel",
        reason="wheel is partially recessed into the molded wheel housing",
    )
    ctx.allow_overlap(
        "front_shell",
        "carry_handle",
        reason="handle nests into a shallow recessed grip pocket when stowed",
    )
    ctx.check_no_overlaps(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )
    ctx.check_no_overlaps(
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("front_shell", "rear_shell", axes="xy", min_overlap=0.22)
    ctx.expect_aabb_contact("front_shell", "rear_shell")
    ctx.expect_joint_motion_axis(
        "shell_hinge",
        "front_shell",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )

    ctx.expect_aabb_overlap("carry_handle", "front_shell", axes="x", min_overlap=0.12)
    ctx.expect_aabb_contact("carry_handle", "front_shell")
    ctx.expect_joint_motion_axis(
        "carry_handle_hinge",
        "carry_handle",
        world_axis="z",
        direction="positive",
        min_delta=0.009,
    )

    ctx.expect_aabb_contact("left_wheel", "rear_shell")
    ctx.expect_aabb_contact("right_wheel", "rear_shell")

    with ctx.pose(shell_hinge=1.32):
        ctx.expect_aabb_overlap("front_shell", "rear_shell", axes="x", min_overlap=0.30)
        ctx.expect_aabb_contact("carry_handle", "front_shell")

    with ctx.pose(carry_handle_hinge=1.05):
        ctx.expect_aabb_overlap("carry_handle", "front_shell", axes="x", min_overlap=0.12)
        ctx.expect_aabb_contact("carry_handle", "front_shell")

    with ctx.pose(left_wheel_spin=math.pi / 2.0, right_wheel_spin=0.8):
        ctx.expect_aabb_contact("left_wheel", "rear_shell")
        ctx.expect_aabb_contact("right_wheel", "rear_shell")

    with ctx.pose(shell_hinge=1.10, carry_handle_hinge=0.95):
        ctx.expect_aabb_overlap("carry_handle", "front_shell", axes="x", min_overlap=0.12)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
