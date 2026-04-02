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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _section_xy(profile, z: float, *, center_x: float = 0.0, center_y: float = 0.0):
    return [(x + center_x, y + center_y, z) for x, y in profile]


def _section_yz(profile, x: float, *, center_y: float = 0.0, center_z: float = 0.0):
    return [(x, y + center_y, z + center_z) for y, z in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_home_mixer")

    body = model.material("body_cream", rgba=(0.92, 0.89, 0.82, 1.0))
    metal = model.material("brushed_steel", rgba=(0.77, 0.79, 0.82, 1.0))
    trim = model.material("trim_dark", rgba=(0.21, 0.22, 0.24, 1.0))
    rubber = model.material("rubber_dark", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")

    base_shell = section_loft(
        [
            _section_xy(rounded_rect_profile(0.30, 0.22, 0.045), 0.0, center_x=0.0),
            _section_xy(rounded_rect_profile(0.29, 0.21, 0.040), 0.04, center_x=0.008),
            _section_xy(rounded_rect_profile(0.24, 0.18, 0.034), 0.08, center_x=0.014),
        ]
    )
    base.visual(mesh_from_geometry(base_shell, "base_shell"), material=body, name="base_shell")

    rear_column = section_loft(
        [
            _section_xy(rounded_rect_profile(0.090, 0.115, 0.020), 0.070, center_x=-0.090),
            _section_xy(rounded_rect_profile(0.078, 0.095, 0.018), 0.160, center_x=-0.094),
            _section_xy(rounded_rect_profile(0.056, 0.082, 0.016), 0.217, center_x=-0.095),
        ]
    )
    base.visual(
        mesh_from_geometry(rear_column, "rear_column"),
        material=body,
        name="rear_column",
    )

    base.visual(
        Cylinder(radius=0.056, length=0.012),
        origin=Origin(xyz=(0.060, 0.0, 0.086)),
        material=trim,
        name="bowl_pedestal",
    )

    for idx, y in enumerate((-0.036, 0.036), start=1):
        base.visual(
            Box((0.024, 0.028, 0.018)),
            origin=Origin(xyz=(-0.095, y, 0.226)),
            material=body,
            name=f"hinge_yoke_{idx}",
        )

    for name, y in (("hinge_left_barrel", -0.036), ("hinge_right_barrel", 0.036)):
        base.visual(
            Cylinder(radius=0.016, length=0.036),
            origin=Origin(xyz=(-0.095, y, 0.236), rpy=(pi / 2.0, 0.0, 0.0)),
            material=trim,
            name=name,
        )

    for idx, x in enumerate((-0.090, 0.080), start=1):
        for side, y in (("l", -0.070), ("r", 0.070)):
            base.visual(
                Cylinder(radius=0.010, length=0.010),
                origin=Origin(xyz=(x, y, 0.005)),
                material=rubber,
                name=f"foot_{idx}_{side}",
            )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [(0.050, 0.000), (0.060, 0.010), (0.082, 0.064), (0.093, 0.108)],
        [(0.038, 0.012), (0.052, 0.020), (0.074, 0.060), (0.084, 0.102)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )
    bowl.visual(mesh_from_geometry(bowl_shell, "mixer_bowl"), material=metal, name="bowl_shell")

    head = model.part("head")

    head_shell = section_loft(
        [
            _section_yz(superellipse_profile(0.086, 0.064, exponent=2.6, segments=36), 0.026, center_z=0.024),
            _section_yz(superellipse_profile(0.112, 0.100, exponent=2.6, segments=36), 0.070, center_z=0.026),
            _section_yz(superellipse_profile(0.118, 0.114, exponent=2.8, segments=36), 0.128, center_z=0.028),
            _section_yz(superellipse_profile(0.094, 0.088, exponent=2.8, segments=36), 0.180, center_z=0.022),
            _section_yz(superellipse_profile(0.046, 0.052, exponent=2.4, segments=36), 0.220, center_z=0.013),
        ]
    )
    head.visual(mesh_from_geometry(head_shell, "head_shell"), material=body, name="head_shell")

    head.visual(
        Box((0.032, 0.072, 0.040)),
        origin=Origin(xyz=(0.040, 0.0, 0.020)),
        material=body,
        name="head_neck",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="head_hinge_barrel",
    )
    head.visual(
        Box((0.040, 0.028, 0.024)),
        origin=Origin(xyz=(0.020, 0.0, 0.008)),
        material=body,
        name="head_hinge_bridge",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.155, 0.0, -0.022)),
        material=trim,
        name="drive_socket",
    )
    head.visual(
        Cylinder(radius=0.023, length=0.012),
        origin=Origin(xyz=(0.218, 0.0, 0.001), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim,
        name="nose_socket",
    )

    attachment = model.part("attachment")
    attachment.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=metal,
        name="shaft",
    )
    hook_geom = tube_from_spline_points(
        [
            (0.000, 0.000, -0.028),
            (0.010, 0.000, -0.040),
            (0.018, 0.004, -0.055),
            (0.015, 0.008, -0.070),
            (0.004, 0.010, -0.084),
            (-0.010, 0.006, -0.078),
            (-0.017, 0.000, -0.063),
            (-0.008, -0.004, -0.050),
        ],
        radius=0.0045,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    attachment.visual(
        mesh_from_geometry(hook_geom, "dough_hook"),
        material=metal,
        name="hook",
    )
    attachment.visual(
        Cylinder(radius=0.0036, length=0.012),
        origin=Origin(xyz=(-0.014, 0.0, -0.064), rpy=(0.0, pi / 2.0, 0.0)),
        material=metal,
        name="hook_tip",
    )

    accessory_cap = model.part("accessory_cap")
    accessory_cap.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(xyz=(0.004, 0.0, -0.021), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim,
        name="cap_cover",
    )
    accessory_cap.visual(
        Box((0.010, 0.012, 0.006)),
        origin=Origin(xyz=(0.008, 0.0, -0.036)),
        material=trim,
        name="cap_grip",
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.060, 0.0, 0.092)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.095, 0.0, 0.236)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.05),
    )
    model.articulation(
        "head_to_attachment",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=attachment,
        origin=Origin(xyz=(0.155, 0.0, -0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "head_to_accessory_cap",
        ArticulationType.REVOLUTE,
        parent=head,
        child=accessory_cap,
        origin=Origin(xyz=(0.224, 0.0, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    attachment = object_model.get_part("attachment")
    accessory_cap = object_model.get_part("accessory_cap")

    head_hinge = object_model.get_articulation("base_to_head")
    attachment_spin = object_model.get_articulation("head_to_attachment")
    cap_hinge = object_model.get_articulation("head_to_accessory_cap")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        bowl,
        base,
        elem_a="bowl_shell",
        elem_b="bowl_pedestal",
        name="bowl seats on the pedestal",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="head_hinge_barrel",
        elem_b="hinge_left_barrel",
        name="head hinge barrel engages the left yoke barrel",
    )
    ctx.expect_contact(
        attachment,
        head,
        elem_a="shaft",
        elem_b="drive_socket",
        name="attachment couples to the drive socket",
    )
    ctx.expect_contact(
        accessory_cap,
        head,
        elem_a="cap_cover",
        elem_b="nose_socket",
        name="accessory cap closes against the nose socket",
    )
    ctx.expect_overlap(
        attachment,
        bowl,
        axes="xy",
        elem_a="hook",
        elem_b="bowl_shell",
        min_overlap=0.02,
        name="attachment hangs within the bowl footprint",
    )

    def elem_center(part, elem_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    closed_socket = elem_center(head, "drive_socket")
    with ctx.pose({head_hinge: 0.95}):
        raised_socket = elem_center(head, "drive_socket")
    ctx.check(
        "head tilts upward from the rear hinge",
        closed_socket is not None
        and raised_socket is not None
        and raised_socket[2] > closed_socket[2] + 0.05
        and raised_socket[0] < closed_socket[0] - 0.03,
        details=f"closed={closed_socket}, raised={raised_socket}",
    )

    closed_hook = elem_center(attachment, "hook_tip")
    with ctx.pose({attachment_spin: pi / 2.0}):
        rotated_hook = elem_center(attachment, "hook_tip")
    ctx.check(
        "attachment spins around a vertical axis",
        closed_hook is not None
        and rotated_hook is not None
        and abs(rotated_hook[0] - closed_hook[0]) > 0.010
        and abs(rotated_hook[1] - closed_hook[1]) > 0.010,
        details=f"closed={closed_hook}, rotated={rotated_hook}",
    )

    closed_cap = elem_center(accessory_cap, "cap_cover")
    with ctx.pose({cap_hinge: 1.0}):
        open_cap = elem_center(accessory_cap, "cap_cover")
    ctx.check(
        "accessory cap flips upward at the nose",
        closed_cap is not None
        and open_cap is not None
        and open_cap[2] > closed_cap[2] + 0.010
        and open_cap[0] > closed_cap[0] + 0.010,
        details=f"closed={closed_cap}, open={open_cap}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
