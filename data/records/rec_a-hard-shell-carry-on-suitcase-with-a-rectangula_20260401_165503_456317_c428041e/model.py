from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xz_rounded_rect_loop(
    width: float,
    height: float,
    radius: float,
    *,
    y: float,
    z_center: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for x, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def _shell_cap_mesh(
    name: str,
    *,
    width: float,
    height: float,
    corner_radius: float,
    seam_y: float,
    outer_y: float,
    z_center: float,
) -> object:
    mid_y = (seam_y + outer_y) * 0.5
    geom = section_loft(
        [
            _xz_rounded_rect_loop(
                width * 0.992,
                height * 0.992,
                corner_radius * 0.95,
                y=seam_y,
                z_center=z_center,
            ),
            _xz_rounded_rect_loop(
                width * 1.01,
                height * 1.004,
                corner_radius * 1.10,
                y=mid_y,
                z_center=z_center,
            ),
            _xz_rounded_rect_loop(
                width * 0.974,
                height * 0.972,
                corner_radius * 1.22,
                y=outer_y,
                z_center=z_center,
            ),
        ]
    )
    return mesh_from_geometry(geom, name)


def _add_spinner_corner(
    model: ArticulatedObject,
    shell,
    *,
    prefix: str,
    mount_xyz: tuple[float, float, float],
    mount_pad_name: str,
    pod_material,
    metal_material,
    rubber_material,
) -> tuple[str, str]:
    pod = model.part(f"{prefix}_wheel_pod")
    pod.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=pod_material,
        name="top_cap",
    )
    pod.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=metal_material,
        name="swivel_stem",
    )
    pod.visual(
        Box((0.030, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=pod_material,
        name="fork_bridge",
    )
    pod.visual(
        Box((0.004, 0.016, 0.034)),
        origin=Origin(xyz=(-0.012, 0.0, -0.046)),
        material=pod_material,
        name="left_fork",
    )
    pod.visual(
        Box((0.004, 0.016, 0.034)),
        origin=Origin(xyz=(0.012, 0.0, -0.046)),
        material=pod_material,
        name="right_fork",
    )
    pod.inertial = Inertial.from_geometry(
        Box((0.032, 0.020, 0.052)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
    )

    wheel = model.part(f"{prefix}_wheel")
    wheel.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber_material,
        name="wheel_tire",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.020),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=metal_material,
        name="wheel_hub",
    )
    wheel.visual(
        Cylinder(radius=0.008, length=0.012),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=pod_material,
        name="hub_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.030, length=0.020),
        mass=0.22,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    swivel_name = f"{prefix}_swivel"
    spin_name = f"{prefix}_wheel_spin"

    model.articulation(
        swivel_name,
        ArticulationType.REVOLUTE,
        parent=shell,
        child=pod,
        origin=Origin(xyz=mount_xyz),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.5,
            lower=-3.10,
            upper=3.10,
        ),
        meta={"mount_pad": mount_pad_name},
    )
    model.articulation(
        spin_name,
        ArticulationType.CONTINUOUS,
        parent=pod,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=25.0),
    )
    return swivel_name, spin_name


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="carry_on_spinner_suitcase")

    shell_blue = model.material("shell_blue", rgba=(0.17, 0.24, 0.39, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.10, 0.11, 1.0))
    pod_black = model.material("pod_black", rgba=(0.13, 0.13, 0.14, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.79, 0.82, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body_w = 0.360
    body_d = 0.230
    body_h = 0.520
    corner_r = 0.042
    band_depth = 0.134
    band_half = band_depth * 0.5
    z_center = body_h * 0.5

    shell = model.part("shell")
    shell.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, z_center)),
    )

    band_outer = rounded_rect_profile(body_w, body_h, corner_r, corner_segments=8)
    band_inner = rounded_rect_profile(body_w - 0.038, body_h - 0.038, corner_r - 0.013, corner_segments=8)
    shell.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                band_outer,
                [band_inner],
                band_depth,
                center=True,
            ),
            "suitcase_shell_band",
        ),
        origin=Origin(xyz=(0.0, 0.0, z_center), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shell_blue,
        name="shell_band",
    )
    shell.visual(
        _shell_cap_mesh(
            "suitcase_front_shell",
            width=body_w,
            height=body_h,
            corner_radius=corner_r,
            seam_y=band_half,
            outer_y=body_d * 0.5,
            z_center=z_center,
        ),
        material=shell_blue,
        name="front_shell",
    )
    shell.visual(
        _shell_cap_mesh(
            "suitcase_rear_shell",
            width=body_w,
            height=body_h,
            corner_radius=corner_r,
            seam_y=-band_half,
            outer_y=-body_d * 0.5,
            z_center=z_center,
        ),
        material=shell_blue,
        name="rear_shell",
    )

    for rib_name, x_center, rib_width in (
        ("front_center_rib", 0.0, 0.020),
        ("front_left_rib", -0.085, 0.016),
        ("front_right_rib", 0.085, 0.016),
    ):
        shell.visual(
            Box((rib_width, 0.006, 0.400)),
            origin=Origin(xyz=(x_center, body_d * 0.5 + 0.003, 0.260)),
            material=trim_black,
            name=rib_name,
        )
    for rib_name, x_center, rib_width in (
        ("rear_center_rib", 0.0, 0.018),
        ("rear_left_rib", -0.085, 0.014),
        ("rear_right_rib", 0.085, 0.014),
    ):
        shell.visual(
            Box((rib_width, 0.005, 0.350)),
            origin=Origin(xyz=(x_center, -body_d * 0.5 - 0.0025, 0.240)),
            material=trim_black,
            name=rib_name,
        )

    shell.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.210, 0.250, 0.020, corner_segments=8),
                [rounded_rect_profile(0.144, 0.214, 0.015, corner_segments=8)],
                0.008,
                center=True,
            ),
            "suitcase_handle_recess_frame",
        ),
        origin=Origin(
            xyz=(0.0, -body_d * 0.5 - 0.004, 0.345),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=trim_black,
        name="handle_recess_frame",
    )

    post_spacing = 0.156
    guide_depth = 0.024
    guide_width = 0.028
    guide_height = 0.180
    guide_center_y = -body_d * 0.5 - guide_depth * 0.5
    guide_center_z = 0.350
    for guide_name, x_pos in (("left_guide", -post_spacing * 0.5), ("right_guide", post_spacing * 0.5)):
        shell.visual(
            Box((guide_width, guide_depth, guide_height)),
            origin=Origin(xyz=(x_pos, guide_center_y, guide_center_z)),
            material=trim_black,
            name=guide_name,
        )

    mount_positions = {
        "front_left_mount_pad": (-0.152, 0.091, 0.009),
        "front_right_mount_pad": (0.152, 0.091, 0.009),
        "rear_left_mount_pad": (-0.152, -0.091, 0.009),
        "rear_right_mount_pad": (0.152, -0.091, 0.009),
    }
    for pad_name, (x_pos, y_pos, z_pos) in mount_positions.items():
        shell.visual(
            Box((0.056, 0.042, 0.018)),
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            material=pod_black,
            name=pad_name,
        )

    pull_handle = model.part("pull_handle")
    pull_handle.visual(
        Cylinder(radius=0.008, length=0.340),
        origin=Origin(xyz=(-post_spacing * 0.5, 0.0, -0.110)),
        material=aluminum,
        name="left_post",
    )
    pull_handle.visual(
        Cylinder(radius=0.008, length=0.340),
        origin=Origin(xyz=(post_spacing * 0.5, 0.0, -0.110)),
        material=aluminum,
        name="right_post",
    )
    pull_handle.visual(
        Box((0.200, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=trim_black,
        name="grip",
    )
    pull_handle.visual(
        Box((0.044, 0.020, 0.030)),
        origin=Origin(xyz=(-post_spacing * 0.5, 0.0, 0.074)),
        material=trim_black,
        name="left_grip_mount",
    )
    pull_handle.visual(
        Box((0.044, 0.020, 0.030)),
        origin=Origin(xyz=(post_spacing * 0.5, 0.0, 0.074)),
        material=trim_black,
        name="right_grip_mount",
    )
    pull_handle.inertial = Inertial.from_geometry(
        Box((0.200, 0.030, 0.360)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=pull_handle,
        origin=Origin(xyz=(0.0, guide_center_y, guide_center_z + guide_height * 0.5)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.35,
            lower=0.0,
            upper=0.220,
        ),
    )

    _add_spinner_corner(
        model,
        shell,
        prefix="front_left",
        mount_xyz=(-0.152, 0.091, 0.0),
        mount_pad_name="front_left_mount_pad",
        pod_material=pod_black,
        metal_material=aluminum,
        rubber_material=wheel_rubber,
    )
    _add_spinner_corner(
        model,
        shell,
        prefix="front_right",
        mount_xyz=(0.152, 0.091, 0.0),
        mount_pad_name="front_right_mount_pad",
        pod_material=pod_black,
        metal_material=aluminum,
        rubber_material=wheel_rubber,
    )
    _add_spinner_corner(
        model,
        shell,
        prefix="rear_left",
        mount_xyz=(-0.152, -0.091, 0.0),
        mount_pad_name="rear_left_mount_pad",
        pod_material=pod_black,
        metal_material=aluminum,
        rubber_material=wheel_rubber,
    )
    _add_spinner_corner(
        model,
        shell,
        prefix="rear_right",
        mount_xyz=(0.152, -0.091, 0.0),
        mount_pad_name="rear_right_mount_pad",
        pod_material=pod_black,
        metal_material=aluminum,
        rubber_material=wheel_rubber,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    pull_handle = object_model.get_part("pull_handle")
    handle_slide = object_model.get_articulation("handle_slide")

    wheel_prefixes = ("front_left", "front_right", "rear_left", "rear_right")
    wheel_pods = {prefix: object_model.get_part(f"{prefix}_wheel_pod") for prefix in wheel_prefixes}
    wheels = {prefix: object_model.get_part(f"{prefix}_wheel") for prefix in wheel_prefixes}
    swivels = {prefix: object_model.get_articulation(f"{prefix}_swivel") for prefix in wheel_prefixes}
    spins = {prefix: object_model.get_articulation(f"{prefix}_wheel_spin") for prefix in wheel_prefixes}

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        shell,
        pull_handle,
        elem_a="left_guide",
        elem_b="left_post",
        reason="The left telescoping post is intentionally represented sliding within its guide sleeve proxy.",
    )
    ctx.allow_overlap(
        shell,
        pull_handle,
        elem_a="right_guide",
        elem_b="right_post",
        reason="The right telescoping post is intentionally represented sliding within its guide sleeve proxy.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "handle uses upward prismatic motion",
        handle_slide.articulation_type == ArticulationType.PRISMATIC
        and handle_slide.axis == (0.0, 0.0, 1.0)
        and handle_slide.motion_limits is not None
        and handle_slide.motion_limits.lower == 0.0
        and handle_slide.motion_limits.upper is not None
        and handle_slide.motion_limits.upper >= 0.20,
        details=str(
            {
                "type": str(handle_slide.articulation_type),
                "axis": handle_slide.axis,
                "limits": (
                    None if handle_slide.motion_limits is None else
                    (handle_slide.motion_limits.lower, handle_slide.motion_limits.upper)
                ),
            }
        ),
    )

    with ctx.pose({handle_slide: 0.0}):
        ctx.expect_within(
            pull_handle,
            shell,
            axes="xy",
            inner_elem="left_post",
            outer_elem="left_guide",
            margin=0.002,
            name="left post stays centered in left guide at rest",
        )
        ctx.expect_within(
            pull_handle,
            shell,
            axes="xy",
            inner_elem="right_post",
            outer_elem="right_guide",
            margin=0.002,
            name="right post stays centered in right guide at rest",
        )
        ctx.expect_overlap(
            pull_handle,
            shell,
            axes="z",
            elem_a="left_post",
            elem_b="left_guide",
            min_overlap=0.170,
            name="left post remains deeply inserted when collapsed",
        )
        ctx.expect_overlap(
            pull_handle,
            shell,
            axes="z",
            elem_a="right_post",
            elem_b="right_guide",
            min_overlap=0.170,
            name="right post remains deeply inserted when collapsed",
        )
        ctx.expect_gap(
            pull_handle,
            shell,
            axis="z",
            positive_elem="grip",
            negative_elem="shell_band",
            min_gap=0.0,
            max_gap=0.020,
            name="collapsed grip sits just above the shell top line",
        )

    handle_rest = ctx.part_world_position(pull_handle)
    handle_upper = handle_slide.motion_limits.upper if handle_slide.motion_limits is not None else 0.0
    with ctx.pose({handle_slide: handle_upper}):
        ctx.expect_within(
            pull_handle,
            shell,
            axes="xy",
            inner_elem="left_post",
            outer_elem="left_guide",
            margin=0.002,
            name="left post stays centered in left guide when extended",
        )
        ctx.expect_within(
            pull_handle,
            shell,
            axes="xy",
            inner_elem="right_post",
            outer_elem="right_guide",
            margin=0.002,
            name="right post stays centered in right guide when extended",
        )
        ctx.expect_overlap(
            pull_handle,
            shell,
            axes="z",
            elem_a="left_post",
            elem_b="left_guide",
            min_overlap=0.050,
            name="left post keeps retained insertion at full extension",
        )
        ctx.expect_overlap(
            pull_handle,
            shell,
            axes="z",
            elem_a="right_post",
            elem_b="right_guide",
            min_overlap=0.050,
            name="right post keeps retained insertion at full extension",
        )
        handle_extended = ctx.part_world_position(pull_handle)
    ctx.check(
        "handle extends upward in upper pose",
        handle_rest is not None
        and handle_extended is not None
        and handle_extended[2] > handle_rest[2] + 0.18,
        details=f"rest={handle_rest}, extended={handle_extended}",
    )

    mount_pad_by_prefix = {
        "front_left": "front_left_mount_pad",
        "front_right": "front_right_mount_pad",
        "rear_left": "rear_left_mount_pad",
        "rear_right": "rear_right_mount_pad",
    }
    for prefix in wheel_prefixes:
        swivel = swivels[prefix]
        spin = spins[prefix]
        pod = wheel_pods[prefix]
        wheel = wheels[prefix]
        ctx.check(
            f"{prefix} pod swivels on vertical revolute axis",
            swivel.articulation_type == ArticulationType.REVOLUTE
            and swivel.axis == (0.0, 0.0, 1.0),
            details=str({"type": str(swivel.articulation_type), "axis": swivel.axis}),
        )
        ctx.check(
            f"{prefix} wheel spins on continuous axle axis",
            spin.articulation_type == ArticulationType.CONTINUOUS
            and spin.axis == (1.0, 0.0, 0.0)
            and spin.motion_limits is not None
            and spin.motion_limits.lower is None
            and spin.motion_limits.upper is None,
            details=str(
                {
                    "type": str(spin.articulation_type),
                    "axis": spin.axis,
                    "limits": None if spin.motion_limits is None else (spin.motion_limits.lower, spin.motion_limits.upper),
                }
            ),
        )
        ctx.expect_contact(
            pod,
            shell,
            elem_a="top_cap",
            elem_b=mount_pad_by_prefix[prefix],
            name=f"{prefix} pod mounts directly to the shell corner pad",
        )
        ctx.expect_contact(
            wheel,
            pod,
            elem_a="wheel_tire",
            elem_b="left_fork",
            name=f"{prefix} wheel is supported by its fork",
        )

    ctx.warn_if_articulation_overlaps(max_pose_samples=20)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
