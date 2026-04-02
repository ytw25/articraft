from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shift_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jobsite_speaker")

    frame_black = model.material("frame_black", rgba=(0.12, 0.12, 0.12, 1.0))
    tool_yellow = model.material("tool_yellow", rgba=(0.80, 0.67, 0.14, 1.0))
    grille_gray = model.material("grille_gray", rgba=(0.22, 0.23, 0.24, 1.0))
    pod_black = model.material("pod_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_gray = model.material("rubber_gray", rgba=(0.19, 0.19, 0.20, 1.0))
    dial_orange = model.material("dial_orange", rgba=(0.86, 0.36, 0.11, 1.0))

    body = model.part("speaker_body")
    body.inertial = Inertial.from_geometry(
        Box((0.40, 0.22, 0.34)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    body.visual(
        Box((0.30, 0.15, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=tool_yellow,
        name="cabinet_shell",
    )
    body.visual(
        Box((0.26, 0.010, 0.20)),
        origin=Origin(xyz=(0.0, 0.071, 0.14)),
        material=pod_black,
        name="front_baffle",
    )

    for x_pos in (-0.17, 0.17):
        body.visual(
            Box((0.04, 0.21, 0.33)),
            origin=Origin(xyz=(x_pos, 0.0, 0.165)),
            material=frame_black,
            name=f"side_guard_{'left' if x_pos < 0.0 else 'right'}",
        )

    for y_pos, name in ((0.103, "front"), (-0.103, "rear")):
        body.visual(
            Box((0.30, 0.040, 0.026)),
            origin=Origin(xyz=(0.0, y_pos, 0.317)),
            material=frame_black,
            name=f"top_rail_{name}",
        )
        body.visual(
            Box((0.30, 0.040, 0.032)),
            origin=Origin(xyz=(0.0, y_pos, 0.016)),
            material=frame_black,
            name=f"bottom_rail_{name}",
        )

    body.visual(
        Box((0.018, 0.024, 0.036)),
        origin=Origin(xyz=(-0.199, -0.117, 0.314)),
        material=frame_black,
        name="left_handle_mount",
    )
    body.visual(
        Box((0.018, 0.024, 0.036)),
        origin=Origin(xyz=(0.199, -0.117, 0.314)),
        material=frame_black,
        name="right_handle_mount",
    )

    grille_outer = rounded_rect_profile(0.24, 0.18, 0.012, corner_segments=8)
    slot_profile = rounded_rect_profile(0.060, 0.030, 0.006, corner_segments=6)
    grille_holes = []
    for x_pos in (-0.070, 0.0, 0.070):
        for z_pos in (-0.048, 0.0, 0.048):
            grille_holes.append(_shift_profile(slot_profile, x_pos, z_pos))

    grille_geom = ExtrudeWithHolesGeometry(
        grille_outer,
        grille_holes,
        height=0.006,
        center=True,
    ).rotate_x(-math.pi / 2.0)
    body.visual(
        _save_mesh("front_grille", grille_geom),
        origin=Origin(xyz=(0.0, 0.078, 0.14)),
        material=grille_gray,
        name="front_grille",
    )

    control_pod = model.part("control_pod")
    control_pod.inertial = Inertial.from_geometry(
        Box((0.12, 0.09, 0.05)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )
    control_pod.visual(
        Box((0.11, 0.085, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=pod_black,
        name="pod_shell",
    )
    control_pod.visual(
        Box((0.11, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.031, 0.040)),
        material=frame_black,
        name="control_ridge",
    )

    knob = model.part("volume_knob")
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.022),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
    )
    knob.visual(
        Cylinder(radius=0.032, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=rubber_gray,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dial_orange,
        name="knob_cap",
    )
    knob.visual(
        Box((0.006, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.022, 0.023)),
        material=tool_yellow,
        name="knob_pointer",
    )

    handle = model.part("carry_handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.44, 0.12, 0.14)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.010, 0.066)),
    )
    handle_loop = tube_from_spline_points(
        [
            (-0.190, 0.010, 0.050),
            (-0.134, 0.022, 0.086),
            (0.0, 0.036, 0.136),
            (0.134, 0.022, 0.086),
            (0.190, 0.010, 0.050),
        ],
        radius=0.0095,
        samples_per_segment=18,
        radial_segments=18,
    )
    left_handle_leg = tube_from_spline_points(
        [
            (-0.224, 0.0, 0.012),
            (-0.216, 0.004, 0.034),
            (-0.190, 0.010, 0.050),
        ],
        radius=0.007,
        samples_per_segment=14,
        radial_segments=16,
    )
    right_handle_leg = tube_from_spline_points(
        [
            (0.224, 0.0, 0.012),
            (0.216, 0.004, 0.034),
            (0.190, 0.010, 0.050),
        ],
        radius=0.007,
        samples_per_segment=14,
        radial_segments=16,
    )
    handle.visual(
        _save_mesh("carry_handle_loop", handle_loop),
        material=frame_black,
        name="handle_loop",
    )
    handle.visual(
        _save_mesh("left_handle_leg", left_handle_leg),
        material=frame_black,
        name="left_handle_leg",
    )
    handle.visual(
        _save_mesh("right_handle_leg", right_handle_leg),
        material=frame_black,
        name="right_handle_leg",
    )
    handle.visual(
        Cylinder(radius=0.0135, length=0.016),
        origin=Origin(xyz=(-0.216, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_black,
        name="left_pivot_collar",
    )
    handle.visual(
        Cylinder(radius=0.0135, length=0.016),
        origin=Origin(xyz=(0.216, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_black,
        name="right_pivot_collar",
    )
    handle.visual(
        Cylinder(radius=0.016, length=0.17),
        origin=Origin(xyz=(0.0, 0.037, 0.133), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_gray,
        name="handle_grip",
    )

    model.articulation(
        "body_to_control_pod",
        ArticulationType.FIXED,
        parent=body,
        child=control_pod,
        origin=Origin(xyz=(0.095, 0.028, 0.26)),
    )
    model.articulation(
        "pod_to_knob",
        ArticulationType.REVOLUTE,
        parent=control_pod,
        child=knob,
        origin=Origin(xyz=(0.022, 0.010, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, -0.117, 0.314)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("speaker_body")
    control_pod = object_model.get_part("control_pod")
    knob = object_model.get_part("volume_knob")
    handle = object_model.get_part("carry_handle")
    handle_joint = object_model.get_articulation("body_to_handle")
    knob_joint = object_model.get_articulation("pod_to_knob")

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
        control_pod,
        body,
        elem_a="pod_shell",
        elem_b="cabinet_shell",
        name="control pod sits on cabinet top",
    )
    ctx.expect_contact(
        knob,
        control_pod,
        elem_a="knob_body",
        elem_b="pod_shell",
        name="volume knob seats on control pod",
    )
    ctx.expect_contact(
        handle,
        body,
        elem_a="left_pivot_collar",
        elem_b="left_handle_mount",
        name="left handle pivot seats on the frame mount",
    )
    ctx.expect_contact(
        handle,
        body,
        elem_a="right_pivot_collar",
        elem_b="right_handle_mount",
        name="right handle pivot seats on the frame mount",
    )
    ctx.expect_origin_gap(
        control_pod,
        body,
        axis="x",
        min_gap=0.07,
        name="control pod stays on the right side",
    )
    ctx.expect_origin_gap(
        control_pod,
        body,
        axis="z",
        min_gap=0.24,
        name="control pod stays on the top deck",
    )
    ctx.check(
        "handle hinge axis runs across the speaker",
        handle_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )
    ctx.check(
        "volume knob rotates around a vertical axis",
        knob_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={knob_joint.axis}",
    )

    body_aabb = ctx.part_world_aabb(body)
    upright_handle_aabb = ctx.part_world_aabb(handle)
    knob_rest_pos = ctx.part_world_position(knob)

    with ctx.pose({handle_joint: 0.95}):
        folded_handle_aabb = ctx.part_world_aabb(handle)
        ctx.fail_if_parts_overlap_in_current_pose(name="folded handle pose stays clear")

    with ctx.pose({knob_joint: 1.6}):
        knob_turned_pos = ctx.part_world_position(knob)

    handle_reads_as_carry_loop = (
        body_aabb is not None
        and upright_handle_aabb is not None
        and upright_handle_aabb[1][2] > body_aabb[1][2] + 0.05
    )
    ctx.check(
        "handle stands proud above the protective frame",
        handle_reads_as_carry_loop,
        details=f"body_aabb={body_aabb}, handle_aabb={upright_handle_aabb}",
    )

    handle_folds_down = (
        upright_handle_aabb is not None
        and folded_handle_aabb is not None
        and ((folded_handle_aabb[0][1] + folded_handle_aabb[1][1]) * 0.5)
        < ((upright_handle_aabb[0][1] + upright_handle_aabb[1][1]) * 0.5) - 0.04
        and folded_handle_aabb[1][2] < upright_handle_aabb[1][2] - 0.015
    )
    ctx.check(
        "handle folds toward the top of the speaker",
        handle_folds_down,
        details=f"upright={upright_handle_aabb}, folded={folded_handle_aabb}",
    )

    knob_spins_in_place = (
        knob_rest_pos is not None
        and knob_turned_pos is not None
        and max(abs(a - b) for a, b in zip(knob_rest_pos, knob_turned_pos)) < 1e-6
    )
    ctx.check(
        "volume knob rotates without translating off the pod",
        knob_spins_in_place,
        details=f"rest={knob_rest_pos}, turned={knob_turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
