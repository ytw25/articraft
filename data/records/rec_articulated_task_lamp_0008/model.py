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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)

BASE_RADIUS = 0.11
BASE_HEIGHT = 0.024
STEM_X = -0.03
PIVOT_Z = 0.136

LOWER_ARM_LENGTH = 0.24
UPPER_ARM_LENGTH = 0.22

BAR_SPAN_Y = 0.060
BAR_WIDTH = 0.008
BAR_HEIGHT = 0.012
ROD_RADIUS = 0.0035

LOWER_YOKE_HALF_SEP = 0.041
UPPER_YOKE_HALF_SEP = 0.038
SHADE_YOKE_HALF_SEP = 0.035
YOKE_THICKNESS = 0.010

STEEL_DARK = "steel_dark"
STEEL_LIGHT = "steel_light"
ENAMEL_BLACK = "enamel_black"
BULB_GLASS = "bulb_glass"


def _add_yoke_pair(
    part,
    *,
    x: float,
    z: float,
    depth: float,
    height: float,
    half_sep: float,
    thickness: float,
    material,
    name_prefix: str,
) -> None:
    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        part.visual(
            Box((depth, thickness, height)),
            origin=Origin(xyz=(x, sign * half_sep, z)),
            material=material,
            name=f"{name_prefix}_{side_name}",
        )


def _hub(
    radius: float,
    length: float,
    *,
    xyz: tuple[float, float, float],
    material,
    name: str,
):
    return {
        "geometry": Cylinder(radius=radius, length=length),
        "origin": Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        "material": material,
        "name": name,
    }


def _rod(length: float, *, xyz: tuple[float, float, float], material, name: str):
    return {
        "geometry": Cylinder(radius=ROD_RADIUS, length=length),
        "origin": Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        "material": material,
        "name": name,
    }


def _build_shade_mesh():
    shade_geom = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.018, 0.000),
            (0.023, 0.014),
            (0.032, 0.040),
            (0.044, 0.078),
            (0.055, 0.118),
        ],
        inner_profile=[
            (0.0, 0.004),
            (0.013, 0.004),
            (0.018, 0.014),
            (0.024, 0.040),
            (0.036, 0.077),
            (0.048, 0.114),
        ],
        segments=52,
        start_cap="flat",
        end_cap="flat",
    )
    shade_geom.rotate_y(math.pi / 2.0)
    shade_geom.translate(0.028, 0.0, 0.0)
    return mesh_from_geometry(shade_geom, ASSETS.mesh_path("architect_lamp_shade.obj"))


def _build_tension_spring_mesh(
    filename: str,
    *,
    span: float,
    hook_y: float,
    coil_radius: float,
    wire_radius: float,
    turns: int,
):
    coil_span = span * 0.60
    coil_start = -coil_span / 2.0
    coil_end = coil_span / 2.0
    hook_x = span / 2.0

    points = [
        (-hook_x, hook_y, 0.0),
        (-hook_x + 0.010, hook_y * 0.88, 0.0025),
        (coil_start - 0.012, hook_y * 0.35, 0.0015),
    ]
    helix_samples = max(48, turns * 16)
    for idx in range(helix_samples + 1):
        t = turns * 2.0 * math.pi * idx / helix_samples
        x = coil_start + (coil_end - coil_start) * idx / helix_samples
        points.append((x, coil_radius * math.cos(t), coil_radius * math.sin(t)))
    points.extend(
        [
            (coil_end + 0.012, hook_y * 0.35, -0.0015),
            (hook_x - 0.010, hook_y * 0.88, -0.0025),
            (hook_x, hook_y, 0.0),
        ]
    )
    spring_geom = wire_from_points(
        points,
        radius=wire_radius,
        radial_segments=14,
        cap_ends=True,
        corner_mode="miter",
    )
    return mesh_from_geometry(spring_geom, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="architect_desk_lamp", assets=ASSETS)

    steel_dark = model.material(STEEL_DARK, rgba=(0.20, 0.20, 0.22, 1.0))
    steel_light = model.material(STEEL_LIGHT, rgba=(0.72, 0.73, 0.75, 1.0))
    enamel_black = model.material(ENAMEL_BLACK, rgba=(0.10, 0.10, 0.11, 1.0))
    bulb_glass = model.material(BULB_GLASS, rgba=(0.96, 0.91, 0.78, 0.65))

    shade_mesh = _build_shade_mesh()
    lower_spring_mesh = _build_tension_spring_mesh(
        "architect_lamp_lower_spring.obj",
        span=0.205,
        hook_y=0.010,
        coil_radius=0.0038,
        wire_radius=0.0015,
        turns=9,
    )
    upper_spring_mesh = _build_tension_spring_mesh(
        "architect_lamp_upper_spring.obj",
        span=0.191,
        hook_y=0.010,
        coil_radius=0.0036,
        wire_radius=0.0014,
        turns=8,
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        material=steel_dark,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.014, length=PIVOT_Z - BASE_HEIGHT - 0.012),
        origin=Origin(xyz=(STEM_X - 0.028, 0.0, (PIVOT_Z + BASE_HEIGHT - 0.012) / 2.0)),
        material=steel_dark,
        name="pedestal",
    )
    base.visual(
        Box((0.016, 0.060, 0.018)),
        origin=Origin(xyz=(STEM_X - 0.013, 0.0, PIVOT_Z - 0.012)),
        material=steel_dark,
        name="yoke_bridge",
    )
    _add_yoke_pair(
        base,
        x=STEM_X - 0.005,
        z=PIVOT_Z - 0.012,
        depth=0.008,
        height=0.048,
        half_sep=0.038,
        thickness=0.016,
        material=steel_dark,
        name_prefix="lower_yoke",
    )
    base.visual(
        Cylinder(radius=0.0042, length=0.092),
        origin=Origin(xyz=(STEM_X - 0.001, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_light,
        name="base_pivot_pin",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(**_hub(0.011, 0.060, xyz=(0.0, 0.0, 0.0), material=steel_light, name="rear_hub"))
    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        lower_arm.visual(
            **_rod(
                0.205,
                xyz=(0.1135, sign * 0.030, 0.0),
                material=steel_light,
                name=f"link_{side_name}",
            )
        )
    for side_name, spring_y in (("left", 0.018), ("right", -0.018)):
        lower_arm.visual(
            lower_spring_mesh,
            origin=Origin(
                xyz=(0.113, spring_y, 0.0),
            ),
            material=steel_dark,
            name=f"tension_spring_{side_name}",
        )
    _add_yoke_pair(
        lower_arm,
        x=0.2225,
        z=0.0,
        depth=0.016,
        height=0.040,
        half_sep=0.036,
        thickness=0.016,
        material=steel_light,
        name_prefix="front_yoke",
    )
    lower_arm.visual(
        Cylinder(radius=0.0040, length=0.088),
        origin=Origin(xyz=(0.2305, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="front_pivot_pin",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_ARM_LENGTH, 0.09, 0.03)),
        mass=0.45,
        origin=Origin(xyz=(LOWER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(**_hub(0.0105, 0.056, xyz=(0.0, 0.0, 0.0), material=steel_light, name="rear_hub"))
    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        upper_arm.visual(
            **_rod(
                0.190,
                xyz=(0.105, sign * 0.029, 0.0),
                material=steel_light,
                name=f"link_{side_name}",
            )
        )
    for side_name, spring_y in (("left", 0.017), ("right", -0.017)):
        upper_arm.visual(
            upper_spring_mesh,
            origin=Origin(
                xyz=(0.105, spring_y, 0.0),
            ),
            material=steel_dark,
            name=f"tension_spring_{side_name}",
        )
    _add_yoke_pair(
        upper_arm,
        x=0.204,
        z=0.0,
        depth=0.008,
        height=0.040,
        half_sep=0.033,
        thickness=0.014,
        material=steel_light,
        name_prefix="front_yoke",
    )
    upper_arm.visual(
        Cylinder(radius=0.0038, length=0.080),
        origin=Origin(xyz=(0.208, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="front_pivot_pin",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.08, 0.03)),
        mass=0.34,
        origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    shade = model.part("shade")
    shade.visual(**_hub(0.012, 0.052, xyz=(0.0, 0.0, 0.0), material=enamel_black, name="rear_hub"))
    shade.visual(
        Cylinder(radius=0.016, length=0.038),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel_black,
        name="neck",
    )
    shade.visual(
        shade_mesh,
        origin=Origin(),
        material=enamel_black,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.017, length=0.026),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_dark,
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
        material=bulb_glass,
        name="bulb",
    )
    shade.inertial = Inertial.from_geometry(
        Box((0.14, 0.11, 0.10)),
        mass=0.28,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_arm,
        origin=Origin(xyz=(STEM_X + 0.010, 0.0, PIVOT_Z), rpy=(0.0, math.radians(-45.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=math.radians(-18.0),
            upper=math.radians(10.0),
        ),
    )
    model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, math.radians(80.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=math.radians(-18.0),
            upper=math.radians(10.0),
        ),
    )
    model.articulation(
        "upper_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0), rpy=(0.0, math.radians(80.0), 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=math.radians(-40.0),
            upper=math.radians(20.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_upper = object_model.get_articulation("lower_to_upper")
    upper_to_shade = object_model.get_articulation("upper_to_shade")

    base_disc = base.get_visual("base_disc")
    lower_yoke_left = base.get_visual("lower_yoke_left")
    lower_yoke_right = base.get_visual("lower_yoke_right")
    lower_rear_hub = lower_arm.get_visual("rear_hub")
    lower_front_yoke_left = lower_arm.get_visual("front_yoke_left")
    lower_front_yoke_right = lower_arm.get_visual("front_yoke_right")
    lower_link_left = lower_arm.get_visual("link_left")
    lower_link_right = lower_arm.get_visual("link_right")
    lower_spring_left = lower_arm.get_visual("tension_spring_left")
    lower_spring_right = lower_arm.get_visual("tension_spring_right")
    upper_rear_hub = upper_arm.get_visual("rear_hub")
    upper_front_yoke_left = upper_arm.get_visual("front_yoke_left")
    upper_front_yoke_right = upper_arm.get_visual("front_yoke_right")
    upper_link_left = upper_arm.get_visual("link_left")
    upper_link_right = upper_arm.get_visual("link_right")
    upper_spring_left = upper_arm.get_visual("tension_spring_left")
    upper_spring_right = upper_arm.get_visual("tension_spring_right")
    shade_rear_hub = shade.get_visual("rear_hub")
    shade_shell = shade.get_visual("shade_shell")
    bulb = shade.get_visual("bulb")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        base,
        lower_arm,
        reason="base pivot pin intentionally passes through the lower arm hub between the fork cheeks",
    )
    ctx.allow_overlap(
        upper_arm,
        lower_arm,
        reason="elbow hinge pin and hub intentionally nest inside the lower arm fork",
    )
    ctx.allow_overlap(
        upper_arm,
        shade,
        reason="shade pivot pin intentionally passes through the shade hub between the fork cheeks",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.expect_contact(lower_arm, base, elem_a=lower_rear_hub, elem_b=lower_yoke_left, name="lower_hub_contacts_left_base_yoke")
    ctx.expect_contact(lower_arm, base, elem_a=lower_rear_hub, elem_b=lower_yoke_right, name="lower_hub_contacts_right_base_yoke")
    ctx.expect_contact(upper_arm, lower_arm, elem_a=upper_rear_hub, elem_b=lower_front_yoke_left, name="upper_hub_contacts_left_lower_yoke")
    ctx.expect_contact(upper_arm, lower_arm, elem_a=upper_rear_hub, elem_b=lower_front_yoke_right, name="upper_hub_contacts_right_lower_yoke")
    ctx.expect_contact(shade, upper_arm, elem_a=shade_rear_hub, elem_b=upper_front_yoke_left, name="shade_hub_contacts_left_upper_yoke")
    ctx.expect_contact(shade, upper_arm, elem_a=shade_rear_hub, elem_b=upper_front_yoke_right, name="shade_hub_contacts_right_upper_yoke")
    ctx.expect_contact(lower_arm, lower_arm, elem_a=lower_spring_left, elem_b=lower_rear_hub, name="lower_left_spring_hooks_to_rear_hub")
    ctx.expect_contact(lower_arm, lower_arm, elem_a=lower_spring_left, elem_b=lower_front_yoke_left, name="lower_left_spring_hooks_to_front_yoke")
    ctx.expect_contact(upper_arm, upper_arm, elem_a=upper_spring_left, elem_b=upper_rear_hub, name="upper_left_spring_hooks_to_rear_hub")
    ctx.expect_contact(upper_arm, upper_arm, elem_a=upper_spring_left, elem_b=upper_front_yoke_left, name="upper_left_spring_hooks_to_front_yoke")
    ctx.expect_gap(
        lower_arm,
        lower_arm,
        axis="y",
        min_gap=0.050,
        positive_elem=lower_link_left,
        negative_elem=lower_link_right,
        name="lower_pair_of_links_reads_as_parallel_double_arm",
    )
    ctx.expect_gap(
        upper_arm,
        upper_arm,
        axis="y",
        min_gap=0.048,
        positive_elem=upper_link_left,
        negative_elem=upper_link_right,
        name="upper_pair_of_links_reads_as_parallel_double_arm",
    )
    ctx.expect_gap(
        lower_arm,
        lower_arm,
        axis="y",
        min_gap=0.010,
        positive_elem=lower_spring_left,
        negative_elem=lower_spring_right,
        name="lower_tension_springs_remain_visibly_separated",
    )
    ctx.expect_gap(
        upper_arm,
        upper_arm,
        axis="y",
        min_gap=0.010,
        positive_elem=upper_spring_left,
        negative_elem=upper_spring_right,
        name="upper_tension_springs_remain_visibly_separated",
    )
    ctx.expect_gap(
        shade,
        base,
        axis="x",
        min_gap=0.10,
        positive_elem=shade_shell,
        negative_elem=base_disc,
        name="shade_projects_forward_of_weighted_base",
    )
    ctx.expect_within(
        shade,
        shade,
        axes="yz",
        inner_elem=bulb,
        outer_elem=shade_shell,
        name="bulb_sits_inside_shade_aperture",
    )
    ctx.expect_origin_distance(shade, base, axes="y", max_dist=0.03, name="lamp_stays_centered_over_base")

    with ctx.pose(
        {
            base_to_lower: math.radians(0.0),
            lower_to_upper: math.radians(-18.0),
            upper_to_shade: math.radians(-20.0),
        }
    ):
        ctx.expect_contact(lower_arm, base, elem_a=lower_rear_hub, elem_b=lower_yoke_left, name="posed_lower_hub_keeps_left_contact")
        ctx.expect_contact(upper_arm, lower_arm, elem_a=upper_rear_hub, elem_b=lower_front_yoke_right, name="posed_upper_hub_keeps_right_contact")
        ctx.expect_gap(
            shade,
            base,
            axis="z",
            min_gap=0.05,
            positive_elem=shade_shell,
            negative_elem=base_disc,
            name="shade_clears_base_in_reading_pose",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
