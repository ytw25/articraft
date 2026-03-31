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
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _stitch_loops(
    geometry: MeshGeometry,
    loop_a: list[int],
    loop_b: list[int],
) -> None:
    count = len(loop_a)
    for index in range(count):
        next_index = (index + 1) % count
        _add_quad(
            geometry,
            loop_a[index],
            loop_a[next_index],
            loop_b[next_index],
            loop_b[index],
        )


def _superellipse_loop(
    width: float,
    depth: float,
    z: float,
    *,
    exponent: float = 2.8,
    segments: int = 40,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in superellipse_profile(
            width,
            depth,
            exponent=exponent,
            segments=segments,
        )
    ]


def _shell_from_sections(
    outer_sections: list[list[tuple[float, float, float]]],
    inner_sections: list[list[tuple[float, float, float]]],
) -> MeshGeometry:
    geometry = MeshGeometry()

    outer_ids = [
        [geometry.add_vertex(*point) for point in section] for section in outer_sections
    ]
    inner_ids = [
        [geometry.add_vertex(*point) for point in section] for section in inner_sections
    ]

    for index in range(len(outer_ids) - 1):
        _stitch_loops(geometry, outer_ids[index], outer_ids[index + 1])
    for index in range(len(inner_ids) - 1):
        _stitch_loops(geometry, inner_ids[index + 1], inner_ids[index])

    _stitch_loops(geometry, outer_ids[0], inner_ids[0])
    _stitch_loops(geometry, inner_ids[-1], outer_ids[-1])
    return geometry


def _yz_rounded_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_center: float,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for z, y in rounded_rect_profile(
            height,
            width,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _offset_section(
    section: list[tuple[float, float, float]],
    *,
    dy: float = 0.0,
    dz: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y + dy, z + dz) for x, y, z in section]


def _side_panel_mesh(
    profile: list[tuple[float, float]],
    *,
    thickness: float,
) -> MeshGeometry:
    return ExtrudeGeometry.centered(profile, thickness).rotate_x(math.pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_trigger_spray_bottle")

    painted_metal = model.material("painted_metal", rgba=(0.77, 0.81, 0.84, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.70, 0.73, 0.76, 1.0))
    graphite_polymer = model.material("graphite_polymer", rgba=(0.18, 0.19, 0.21, 1.0))
    warm_gray_polymer = model.material("warm_gray_polymer", rgba=(0.50, 0.52, 0.55, 1.0))
    charcoal_elastomer = model.material("charcoal_elastomer", rgba=(0.12, 0.13, 0.14, 1.0))

    bottle = model.part("bottle")
    bottle_shell = _shell_from_sections(
        [
            _superellipse_loop(0.088, 0.058, 0.000, exponent=3.2),
            _superellipse_loop(0.092, 0.060, 0.012, exponent=3.0),
            _superellipse_loop(0.096, 0.064, 0.148, exponent=2.9),
            _superellipse_loop(0.090, 0.060, 0.182, exponent=2.8),
            _superellipse_loop(0.060, 0.046, 0.198, exponent=2.5),
            _superellipse_loop(0.038, 0.038, 0.206, exponent=2.2),
            _superellipse_loop(0.032, 0.032, 0.209, exponent=2.0),
        ],
        [
            _superellipse_loop(0.082, 0.052, 0.004, exponent=3.2),
            _superellipse_loop(0.086, 0.054, 0.016, exponent=3.0),
            _superellipse_loop(0.090, 0.058, 0.144, exponent=2.9),
            _superellipse_loop(0.084, 0.054, 0.178, exponent=2.8),
            _superellipse_loop(0.054, 0.040, 0.194, exponent=2.5),
            _superellipse_loop(0.026, 0.026, 0.199, exponent=2.0),
        ],
    )
    bottle.visual(
        mesh_from_geometry(bottle_shell, "spray_bottle_shell"),
        material=painted_metal,
        name="shell",
    )
    bottle.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.204)),
        material=satin_metal,
        name="neck_collar",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.096, 0.064, 0.209)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.1045)),
    )

    head = model.part("head")
    left_rail_mesh = section_loft(
        [
            _offset_section(
                _yz_rounded_section(0.002, 0.008, 0.018, 0.003, z_center=0.024),
                dy=0.011,
            ),
            _offset_section(
                _yz_rounded_section(0.028, 0.009, 0.020, 0.003, z_center=0.034),
                dy=0.012,
            ),
            _offset_section(
                _yz_rounded_section(0.052, 0.008, 0.014, 0.003, z_center=0.038),
                dy=0.010,
            ),
            _offset_section(
                _yz_rounded_section(0.068, 0.006, 0.010, 0.0025, z_center=0.038),
                dy=0.008,
            ),
        ]
    )
    head.visual(
        mesh_from_geometry(left_rail_mesh, "sprayer_head_left_rail"),
        material=graphite_polymer,
        name="left_rail",
    )
    right_rail_mesh = section_loft(
        [
            _offset_section(
                _yz_rounded_section(0.002, 0.008, 0.018, 0.003, z_center=0.024),
                dy=-0.011,
            ),
            _offset_section(
                _yz_rounded_section(0.028, 0.009, 0.020, 0.003, z_center=0.034),
                dy=-0.012,
            ),
            _offset_section(
                _yz_rounded_section(0.052, 0.008, 0.014, 0.003, z_center=0.038),
                dy=-0.010,
            ),
            _offset_section(
                _yz_rounded_section(0.068, 0.006, 0.010, 0.0025, z_center=0.038),
                dy=-0.008,
            ),
        ]
    )
    head.visual(
        mesh_from_geometry(right_rail_mesh, "sprayer_head_right_rail"),
        material=graphite_polymer,
        name="right_rail",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.036, 0.019, 0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="left_pivot_boss",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.036, -0.019, 0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite_polymer,
        name="right_pivot_boss",
    )

    side_profile = [
        (0.000, 0.004),
        (0.002, 0.018),
        (0.014, 0.032),
        (0.038, 0.043),
        (0.056, 0.041),
        (0.050, 0.026),
        (0.020, 0.014),
        (0.004, 0.008),
    ]
    side_mesh = _side_panel_mesh(side_profile, thickness=0.004)
    head.visual(
        mesh_from_geometry(side_mesh, "sprayer_left_shroud"),
        origin=Origin(xyz=(0.0, 0.018, 0.0)),
        material=graphite_polymer,
        name="left_shroud",
    )
    head.visual(
        mesh_from_geometry(side_mesh, "sprayer_right_shroud"),
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
        material=graphite_polymer,
        name="right_shroud",
    )
    head.visual(
        Cylinder(radius=0.019, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=graphite_polymer,
        name="collar",
    )
    head.visual(
        Box((0.006, 0.004, 0.020)),
        origin=Origin(xyz=(0.024, 0.006, 0.031)),
        material=warm_gray_polymer,
        name="left_guide_tab",
    )
    head.visual(
        Box((0.006, 0.004, 0.020)),
        origin=Origin(xyz=(0.024, -0.006, 0.031)),
        material=warm_gray_polymer,
        name="right_guide_tab",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.062, 0.0, 0.038), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray_polymer,
        name="nozzle_core",
    )
    head.visual(
        Cylinder(radius=0.009, length=0.004),
        origin=Origin(xyz=(0.071, 0.0, 0.038), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="nozzle_bezel",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.074, 0.042, 0.050)),
        mass=0.16,
        origin=Origin(xyz=(0.032, 0.0, 0.025)),
    )

    trigger = model.part("trigger")
    trigger_profile = [
        (-0.016, 0.010),
        (-0.012, 0.015),
        (-0.004, 0.012),
        (0.004, 0.006),
        (0.014, -0.010),
        (0.032, -0.046),
        (0.024, -0.056),
        (0.010, -0.030),
        (0.000, -0.010),
        (-0.006, -0.004),
    ]
    arm_mesh = _side_panel_mesh(trigger_profile, thickness=0.004)
    trigger.visual(
        mesh_from_geometry(arm_mesh, "trigger_left_arm"),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=warm_gray_polymer,
        name="left_arm",
    )
    trigger.visual(
        mesh_from_geometry(arm_mesh, "trigger_right_arm"),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=warm_gray_polymer,
        name="right_arm",
    )
    trigger.visual(
        Cylinder(radius=0.0035, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_gray_polymer,
        name="axle",
    )
    trigger.visual(
        mesh_from_geometry(
            _side_panel_mesh(
                rounded_rect_profile(0.014, 0.024, 0.004),
                thickness=0.028,
            ),
            "trigger_finger_body",
        ),
        origin=Origin(xyz=(0.022, 0.0, -0.040)),
        material=warm_gray_polymer,
        name="finger_body",
    )
    trigger.visual(
        Box((0.010, 0.008, 0.006)),
        origin=Origin(xyz=(-0.012, 0.0, 0.004)),
        material=warm_gray_polymer,
        name="pusher",
    )
    trigger.visual(
        Box((0.008, 0.008, 0.010)),
        origin=Origin(xyz=(-0.005, 0.0, 0.002)),
        material=warm_gray_polymer,
        name="center_web",
    )
    trigger.visual(
        mesh_from_geometry(
            _side_panel_mesh(
                rounded_rect_profile(0.006, 0.018, 0.0025),
                thickness=0.020,
            ),
            "trigger_pad",
        ),
        origin=Origin(xyz=(0.028, 0.0, -0.040)),
        material=charcoal_elastomer,
        name="pad",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.052, 0.032, 0.072)),
        mass=0.08,
        origin=Origin(xyz=(0.008, 0.0, -0.020)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.004, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=warm_gray_polymer,
        name="stem",
    )
    plunger.visual(
        Box((0.012, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=warm_gray_polymer,
        name="tail",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.012, 0.014, 0.024)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "bottle_to_head",
        ArticulationType.FIXED,
        parent=bottle,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.209)),
    )
    model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head,
        child=trigger,
        origin=Origin(xyz=(0.036, 0.0, 0.014)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=0.42,
        ),
    )
    model.articulation(
        "head_to_plunger",
        ArticulationType.PRISMATIC,
        parent=head,
        child=plunger,
        origin=Origin(xyz=(0.024, 0.0, 0.023)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.005,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    head = object_model.get_part("head")
    trigger = object_model.get_part("trigger")
    plunger = object_model.get_part("plunger")
    trigger_joint = object_model.get_articulation("head_to_trigger")
    plunger_joint = object_model.get_articulation("head_to_plunger")

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

    ctx.expect_contact(head, bottle, elem_a="collar", elem_b="neck_collar", name="head_seats_on_bottle")
    ctx.expect_contact(trigger, head, elem_a="axle", elem_b="left_pivot_boss", name="trigger_left_pivot_supported")
    ctx.expect_contact(trigger, head, elem_a="axle", elem_b="right_pivot_boss", name="trigger_right_pivot_supported")
    ctx.expect_contact(plunger, head, elem_a="stem", elem_b="left_guide_tab", name="plunger_left_guide_support")
    ctx.expect_contact(plunger, head, elem_a="stem", elem_b="right_guide_tab", name="plunger_right_guide_support")

    with ctx.pose({trigger_joint: 0.0, plunger_joint: 0.0}):
        ctx.expect_contact(
            trigger,
            plunger,
            elem_a="pusher",
            elem_b="tail",
            contact_tol=0.001,
            name="rest_trigger_contacts_plunger",
        )
        rest_pad_aabb = ctx.part_element_world_aabb(trigger, elem="pad")
        rest_stem_aabb = ctx.part_element_world_aabb(plunger, elem="stem")

    with ctx.pose({trigger_joint: 0.40, plunger_joint: 0.005}):
        ctx.expect_contact(
            trigger,
            plunger,
            elem_a="pusher",
            elem_b="tail",
            contact_tol=0.0015,
            name="pulled_trigger_stays_on_plunger",
        )
        pulled_pad_aabb = ctx.part_element_world_aabb(trigger, elem="pad")
        pulled_stem_aabb = ctx.part_element_world_aabb(plunger, elem="stem")

    pad_pulls_back = (
        rest_pad_aabb is not None
        and pulled_pad_aabb is not None
        and pulled_pad_aabb[0][0] < rest_pad_aabb[0][0] - 0.008
    )
    stem_rises = (
        rest_stem_aabb is not None
        and pulled_stem_aabb is not None
        and pulled_stem_aabb[0][2] > rest_stem_aabb[0][2] + 0.004
    )
    ctx.check(
        "trigger_pull_moves_finger_rearward",
        pad_pulls_back,
        details=f"rest_pad={rest_pad_aabb}, pulled_pad={pulled_pad_aabb}",
    )
    ctx.check(
        "trigger_pull_lifts_visible_pump_stem",
        stem_rises,
        details=f"rest_stem={rest_stem_aabb}, pulled_stem={pulled_stem_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
