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
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.052
BODY_DEPTH = 0.022
BODY_HEIGHT = 0.048
BODY_TOP = BODY_HEIGHT

PIVOT_X = -0.0135
RELEASE_X = 0.0135
JOINT_Z = 0.0435

SHACKLE_RADIUS = 0.0034
PIVOT_BARREL_RADIUS = 0.0044
RIGHT_BORE_DEPTH = 0.0094
RIGHT_BORE_RADIUS = SHACKLE_RADIUS - 0.0002
LEFT_BORE_DEPTH = 0.0115
LEFT_BORE_RADIUS = PIVOT_BARREL_RADIUS - 0.0002
PIVOT_POCKET_RADIUS = PIVOT_BARREL_RADIUS + 0.0008
SHACKLE_SPAN = RELEASE_X - PIVOT_X
LEG_EMERGE_Z = 0.0105


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_body_mesh():
    shell = BoxGeometry((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)).translate(0.0, 0.0, BODY_HEIGHT * 0.5)

    left_bore = CylinderGeometry(
        radius=LEFT_BORE_RADIUS,
        height=LEFT_BORE_DEPTH + 0.003,
        radial_segments=30,
    ).translate(
        PIVOT_X,
        0.0,
        BODY_TOP - LEFT_BORE_DEPTH * 0.5 + 0.0010,
    )
    shell = boolean_difference(shell, left_bore)

    right_bore = CylinderGeometry(
        radius=RIGHT_BORE_RADIUS,
        height=RIGHT_BORE_DEPTH + 0.003,
        radial_segments=30,
    ).translate(
        RELEASE_X,
        0.0,
        BODY_TOP - RIGHT_BORE_DEPTH * 0.5 + 0.0010,
    )
    shell = boolean_difference(shell, right_bore)

    pivot_pocket = CylinderGeometry(
        radius=PIVOT_POCKET_RADIUS,
        height=BODY_DEPTH + 0.006,
        radial_segments=30,
    ).rotate_x(math.pi * 0.5).translate(PIVOT_X, 0.0, JOINT_Z)
    shell = boolean_difference(shell, pivot_pocket)

    pivot_slot_height = 0.013
    pivot_slot = BoxGeometry((0.020, BODY_DEPTH + 0.006, pivot_slot_height)).translate(
        PIVOT_X + 0.003,
        0.0,
        BODY_TOP - pivot_slot_height * 0.5 + 0.001,
    )
    shell = boolean_difference(shell, pivot_slot)

    keyway_recess = BoxGeometry((0.015, 0.005, 0.017)).translate(0.0, BODY_DEPTH * 0.5, 0.020)
    shell = boolean_difference(shell, keyway_recess)

    return shell


def _build_shackle_mesh():
    return tube_from_spline_points(
        [
            (0.0, 0.0, LEG_EMERGE_Z),
            (0.0, 0.0, 0.028),
            (0.004, 0.0, 0.047),
            (0.010, 0.0, 0.060),
            (0.0135, 0.0, 0.064),
            (0.017, 0.0, 0.060),
            (0.023, 0.0, 0.047),
            (SHACKLE_SPAN, 0.0, 0.028),
            (SHACKLE_SPAN, 0.0, LEG_EMERGE_Z),
        ],
        radius=SHACKLE_RADIUS,
        samples_per_segment=18,
        radial_segments=22,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classic_padlock", assets=ASSETS)

    brass = model.material("brass", rgba=(0.76, 0.61, 0.18, 1.0))
    dark_brass = model.material("dark_brass", rgba=(0.56, 0.42, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.75, 0.78, 0.82, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.35, 0.38, 0.42, 1.0))

    body = model.part("body")
    body.visual(_save_mesh(_build_body_mesh(), "padlock_body.obj"), material=brass, name="body_shell")
    body.visual(
        Box((BODY_WIDTH * 0.86, 0.0018, BODY_HEIGHT * 0.72)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 + 0.0009, BODY_HEIGHT * 0.45)),
        material=dark_brass,
        name="front_plate",
    )
    body.visual(
        Cylinder(radius=0.0052, length=0.0032),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 + 0.0016, 0.0195), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="key_cylinder",
    )
    body.visual(
        Box((0.0025, 0.0036, 0.0095)),
        origin=Origin(xyz=(0.0, BODY_DEPTH * 0.5 + 0.0012, 0.0150)),
        material=dark_steel,
        name="key_slot",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    shackle = model.part("shackle")
    shackle.visual(
        _save_mesh(_build_shackle_mesh(), "padlock_shackle.obj"),
        material=steel,
        name="shackle_shell",
    )
    shackle.visual(
        Cylinder(radius=SHACKLE_RADIUS * 0.985, length=0.008),
        origin=Origin(xyz=(SHACKLE_SPAN, 0.0, 0.0065)),
        material=steel,
        name="release_leg",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((SHACKLE_SPAN + 0.014, BODY_DEPTH * 0.78, 0.072)),
        mass=0.11,
        origin=Origin(xyz=(SHACKLE_SPAN * 0.5, 0.0, 0.026)),
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(PIVOT_X, 0.0, JOINT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(60.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    hinge = object_model.get_articulation("body_to_shackle")

    body_shell = body.get_visual("body_shell")
    shackle_shell = shackle.get_visual("shackle_shell")
    release_leg = shackle.get_visual("release_leg")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.allow_overlap(
        body,
        shackle,
        elem_a=body_shell,
        elem_b=release_leg,
        reason="The release leg is modeled as a light press-fit into the body bore in the locked pose so the shackle reads positively captured instead of floating.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_within(
        shackle,
        body,
        axes="xy",
        inner_elem=release_leg,
        outer_elem=body_shell,
        margin=0.0015,
        name="release_tip_aligns_with_body_footprint",
    )
    ctx.expect_gap(
        shackle,
        body,
        axis="z",
        positive_elem=release_leg,
        negative_elem=body_shell,
        min_gap=-0.012,
        max_gap=-0.002,
        name="release_tip_is_seated_in_body_at_rest",
    )

    body_aabb = ctx.part_world_aabb(body)
    shackle_rest_pos = ctx.part_world_position(shackle)
    rest_tip_aabb = ctx.part_element_world_aabb(shackle, elem=release_leg)
    assert body_aabb is not None
    assert shackle_rest_pos is not None
    assert rest_tip_aabb is not None
    body_min = body_aabb[0]
    body_max = body_aabb[1]
    body_top = body_aabb[1][2]
    rest_tip_center = tuple((lo + hi) * 0.5 for lo, hi in zip(rest_tip_aabb[0], rest_tip_aabb[1]))
    ctx.check(
        "pivot_origin_stays_inside_body_footprint_at_rest",
        body_min[0] <= shackle_rest_pos[0] <= body_max[0]
        and body_min[1] <= shackle_rest_pos[1] <= body_max[1]
        and body_top - 0.010 <= shackle_rest_pos[2] <= body_top,
        details=(
            f"body_x=({body_min[0]:.4f},{body_max[0]:.4f}) "
            f"body_y=({body_min[1]:.4f},{body_max[1]:.4f}) "
            f"pivot={tuple(round(v, 4) for v in shackle_rest_pos)}"
        ),
    )

    with ctx.pose({hinge: math.radians(60.0)}):
        ctx.expect_gap(
            shackle,
            body,
            axis="z",
            positive_elem=release_leg,
            negative_elem=body_shell,
            min_gap=0.010,
            name="release_tip_lifts_clear_of_body_when_open",
        )

        open_tip_aabb = ctx.part_element_world_aabb(shackle, elem=release_leg)
        open_shackle_pos = ctx.part_world_position(shackle)
        assert open_tip_aabb is not None
        assert open_shackle_pos is not None
        open_tip_center = tuple((lo + hi) * 0.5 for lo, hi in zip(open_tip_aabb[0], open_tip_aabb[1]))
        ctx.check(
            "pivot_origin_remains_captive_when_open",
            body_min[0] <= open_shackle_pos[0] <= body_max[0]
            and body_min[1] <= open_shackle_pos[1] <= body_max[1]
            and abs(open_shackle_pos[0] - shackle_rest_pos[0]) < 1e-6
            and abs(open_shackle_pos[1] - shackle_rest_pos[1]) < 1e-6
            and abs(open_shackle_pos[2] - shackle_rest_pos[2]) < 1e-6,
            details=(
                f"rest={tuple(round(v, 4) for v in shackle_rest_pos)} "
                f"open={tuple(round(v, 4) for v in open_shackle_pos)}"
            ),
        )
        ctx.check(
            "release_side_swings_upward",
            open_tip_center[2] > rest_tip_center[2] + 0.018,
            details=f"rest_z={rest_tip_center[2]:.4f}, open_z={open_tip_center[2]:.4f}",
        )
        ctx.check(
            "release_side_rotates_away_from_bore_centerline",
            open_tip_center[0] < rest_tip_center[0] - 0.008,
            details=f"rest_x={rest_tip_center[0]:.4f}, open_x={open_tip_center[0]:.4f}",
        )
        ctx.check(
            "open_tip_clears_body_top",
            open_tip_aabb[0][2] > body_top + 0.010,
            details=f"body_top={body_top:.4f}, open_tip_min_z={open_tip_aabb[0][2]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
