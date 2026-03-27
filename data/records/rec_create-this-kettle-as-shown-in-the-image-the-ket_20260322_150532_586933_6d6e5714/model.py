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
    CapsuleGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _yz_loop(x_coord: float, points: list[tuple[float, float]]) -> list[tuple[float, float, float]]:
    return [(x_coord, y_val, z_val) for y_val, z_val in points]


def _xy_loop(points: list[tuple[float, float]], z_val: float) -> list[tuple[float, float, float]]:
    return [(x_val, y_val, z_val) for x_val, y_val in points]


def _ellipse_loop(radius_x: float, radius_y: float, z_val: float, segments: int = 40) -> list[tuple[float, float, float]]:
    return [
        (
            radius_x * math.cos((2.0 * math.pi * idx) / segments),
            radius_y * math.sin((2.0 * math.pi * idx) / segments),
            z_val,
        )
        for idx in range(segments)
    ]


def _scaled_plan(
    points: list[tuple[float, float]],
    scale_x: float,
    scale_y: float,
    *,
    shift_x: float = 0.0,
    shift_y: float = 0.0,
) -> list[tuple[float, float]]:
    return [
        (x_val * scale_x + shift_x, y_val * scale_y + shift_y)
        for x_val, y_val in points
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_kettle", assets=ASSETS)

    body_gloss = model.material("body_gloss", rgba=(0.08, 0.08, 0.09, 1.0))
    trim_black = model.material("trim_black", rgba=(0.12, 0.12, 0.13, 1.0))
    interior_black = model.material("interior_black", rgba=(0.03, 0.03, 0.04, 1.0))
    smoked_window = model.material("smoked_window", rgba=(0.82, 0.84, 0.87, 0.45))
    button_grey = model.material("button_grey", rgba=(0.48, 0.49, 0.51, 1.0))
    switch_clear = model.material("switch_clear", rgba=(0.86, 0.88, 0.92, 0.72))

    body = model.part("body")
    body_shell = repair_loft(
        [
            _ellipse_loop(0.058, 0.056, 0.000),
            _ellipse_loop(0.061, 0.059, 0.016),
            _ellipse_loop(0.062, 0.060, 0.075),
            _ellipse_loop(0.061, 0.059, 0.132),
            _ellipse_loop(0.056, 0.055, 0.170),
            _ellipse_loop(0.047, 0.046, 0.184),
        ]
    )
    body.visual(_mesh("kettle_body.obj", body_shell), material=body_gloss)
    body.visual(
        Cylinder(radius=0.063, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=trim_black,
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.061, length=0.184),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
    )

    spout = model.part("spout")
    spout_outer = repair_loft(
        [
            _yz_loop(
                -0.050,
                [
                    (0.0, 0.137),
                    (0.015, 0.140),
                    (0.021, 0.149),
                    (0.018, 0.163),
                    (0.0, 0.171),
                    (-0.018, 0.163),
                    (-0.021, 0.149),
                    (-0.015, 0.140),
                ],
            ),
            _yz_loop(
                -0.074,
                [
                    (0.0, 0.139),
                    (0.017, 0.143),
                    (0.024, 0.152),
                    (0.021, 0.166),
                    (0.0, 0.176),
                    (-0.021, 0.166),
                    (-0.024, 0.152),
                    (-0.017, 0.143),
                ],
            ),
            _yz_loop(
                -0.094,
                [
                    (0.0, 0.141),
                    (0.010, 0.145),
                    (0.013, 0.153),
                    (0.011, 0.165),
                    (0.0, 0.172),
                    (-0.011, 0.165),
                    (-0.013, 0.153),
                    (-0.010, 0.145),
                ],
            ),
        ]
    )
    spout_inner = repair_loft(
        [
            _yz_loop(
                -0.060,
                [
                    (0.0, 0.145),
                    (0.009, 0.147),
                    (0.013, 0.152),
                    (0.011, 0.160),
                    (0.0, 0.165),
                    (-0.011, 0.160),
                    (-0.013, 0.152),
                    (-0.009, 0.147),
                ],
            ),
            _yz_loop(
                -0.079,
                [
                    (0.0, 0.146),
                    (0.010, 0.149),
                    (0.015, 0.155),
                    (0.013, 0.163),
                    (0.0, 0.168),
                    (-0.013, 0.163),
                    (-0.015, 0.155),
                    (-0.010, 0.149),
                ],
            ),
            _yz_loop(
                -0.092,
                [
                    (0.0, 0.147),
                    (0.008, 0.150),
                    (0.010, 0.155),
                    (0.009, 0.161),
                    (0.0, 0.165),
                    (-0.009, 0.161),
                    (-0.010, 0.155),
                    (-0.008, 0.150),
                ],
            ),
        ]
    )
    spout.visual(_mesh("kettle_spout_outer.obj", spout_outer), material=body_gloss)
    spout.visual(_mesh("kettle_spout_inner.obj", spout_inner), material=interior_black)
    spout.inertial = Inertial.from_geometry(
        Box((0.048, 0.040, 0.040)),
        mass=0.05,
        origin=Origin(xyz=(-0.073, 0.0, 0.156)),
    )

    handle = model.part("handle")
    handle_pod = repair_loft(
        [
            _yz_loop(
                0.046,
                [
                    (0.0, 0.164),
                    (0.012, 0.168),
                    (0.016, 0.180),
                    (0.013, 0.192),
                    (0.0, 0.196),
                    (-0.013, 0.192),
                    (-0.016, 0.180),
                    (-0.012, 0.168),
                ],
            ),
            _yz_loop(
                0.082,
                [
                    (0.0, 0.168),
                    (0.014, 0.172),
                    (0.019, 0.184),
                    (0.016, 0.196),
                    (0.0, 0.200),
                    (-0.016, 0.196),
                    (-0.019, 0.184),
                    (-0.014, 0.172),
                ],
            ),
            _yz_loop(
                0.118,
                [
                    (0.0, 0.166),
                    (0.016, 0.170),
                    (0.022, 0.182),
                    (0.018, 0.193),
                    (0.0, 0.197),
                    (-0.018, 0.193),
                    (-0.022, 0.182),
                    (-0.016, 0.170),
                ],
            ),
        ]
    )
    handle_arch = sweep_profile_along_spline(
        [
            (0.107, 0.0, 0.183),
            (0.124, 0.0, 0.158),
            (0.132, 0.0, 0.112),
            (0.131, 0.0, 0.064),
            (0.114, 0.0, 0.030),
        ],
        profile=rounded_rect_profile(0.012, 0.020, radius=0.003, corner_segments=8),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    handle.visual(_mesh("kettle_handle_pod.obj", handle_pod), material=trim_black)
    handle.visual(_mesh("kettle_handle_arch.obj", handle_arch), material=body_gloss)
    handle.visual(
        Box((0.024, 0.028, 0.036)),
        origin=Origin(xyz=(0.055, 0.0, 0.176)),
        material=trim_black,
    )
    handle.visual(
        Box((0.058, 0.026, 0.032)),
        origin=Origin(xyz=(0.087, 0.0, 0.018)),
        material=trim_black,
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.090, 0.032, 0.180)),
        mass=0.16,
        origin=Origin(xyz=(0.090, 0.0, 0.094)),
    )

    window = model.part("window")
    window_geom = CapsuleGeometry(radius=0.011, length=0.105, radial_segments=24, height_segments=8)
    window_geom.scale(1.25, 0.32, 1.0)
    window.visual(
        _mesh("kettle_window.obj", window_geom),
        origin=Origin(xyz=(0.0, -0.056, 0.093)),
        material=smoked_window,
    )
    window.inertial = Inertial.from_geometry(
        Box((0.030, 0.008, 0.128)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.056, 0.093)),
    )

    lid = model.part("lid")
    lid_plan = [
        (-0.108, 0.0),
        (-0.095, -0.033),
        (-0.060, -0.050),
        (-0.018, -0.048),
        (-0.002, -0.028),
        (-0.002, 0.028),
        (-0.018, 0.048),
        (-0.060, 0.050),
        (-0.095, 0.033),
    ]
    lid_geom = repair_loft(
        [
            _xy_loop(lid_plan, 0.0),
            _xy_loop(_scaled_plan(lid_plan, 0.93, 0.90, shift_x=0.004), 0.010),
            _xy_loop(_scaled_plan(lid_plan, 0.78, 0.72, shift_x=0.011), 0.018),
        ]
    )
    lid.visual(_mesh("kettle_lid.obj", lid_geom), material=trim_black)
    lid.visual(
        Box((0.008, 0.060, 0.008)),
        origin=Origin(xyz=(-0.004, 0.0, 0.004)),
        material=trim_black,
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.110, 0.105, 0.020)),
        mass=0.12,
        origin=Origin(xyz=(-0.055, 0.0, 0.009)),
    )

    lid_button = model.part("lid_button")
    lid_button_geom = CapsuleGeometry(radius=0.007, length=0.011, radial_segments=24, height_segments=8)
    lid_button_geom.scale(1.0, 0.76, 0.92)
    lid_button.visual(
        _mesh("kettle_lid_button.obj", lid_button_geom),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_grey,
    )
    lid_button.inertial = Inertial.from_geometry(
        Box((0.025, 0.013, 0.014)),
        mass=0.02,
        origin=Origin(),
    )

    power_switch = model.part("power_switch")
    switch_geom = CapsuleGeometry(radius=0.005, length=0.012, radial_segments=24, height_segments=8)
    switch_geom.scale(1.15, 0.70, 0.88)
    power_switch.visual(
        _mesh("kettle_power_switch.obj", switch_geom),
        origin=Origin(xyz=(0.014, 0.0, -0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=switch_clear,
    )
    power_switch.inertial = Inertial.from_geometry(
        Box((0.024, 0.012, 0.010)),
        mass=0.025,
        origin=Origin(xyz=(0.014, 0.0, -0.003)),
    )

    model.articulation(
        "body_to_spout",
        ArticulationType.FIXED,
        parent="body",
        child="spout",
        origin=Origin(),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.FIXED,
        parent="body",
        child="handle",
        origin=Origin(),
    )
    model.articulation(
        "body_to_window",
        ArticulationType.FIXED,
        parent="body",
        child="window",
        origin=Origin(),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="lid",
        origin=Origin(xyz=(0.049, 0.0, 0.186)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.18),
    )
    model.articulation(
        "lid_button_slide",
        ArticulationType.PRISMATIC,
        parent="handle",
        child="lid_button",
        origin=Origin(xyz=(0.093, 0.0, 0.203)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=-0.004, upper=0.0),
    )
    model.articulation(
        "power_switch_joint",
        ArticulationType.REVOLUTE,
        parent="handle",
        child="power_switch",
        origin=Origin(xyz=(0.111, 0.0, 0.029)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.146)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected(use="visual")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.allow_overlap("handle", "lid", reason="the closed lid tucks under the rear handle bridge")
    ctx.allow_overlap("lid_button", "handle", reason="the lid-release button sinks into a shallow handle recess")
    ctx.allow_overlap("power_switch", "handle", reason="the rocker nests at its pivot pocket in the lower handle")
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("spout", "body")
    ctx.expect_aabb_overlap("spout", "body", axes="yz", min_overlap=0.020)
    ctx.expect_origin_distance("spout", "body", axes="y", max_dist=0.010)

    ctx.expect_aabb_contact("handle", "body")
    ctx.expect_aabb_gap("handle", "body", axis="x", max_gap=0.002, max_penetration=0.030)
    ctx.expect_aabb_overlap("handle", "body", axes="yz", min_overlap=0.040)
    ctx.expect_origin_distance("handle", "body", axes="y", max_dist=0.010)

    ctx.expect_aabb_contact("window", "body")
    ctx.expect_aabb_overlap("window", "body", axes="xz", min_overlap=0.025)
    ctx.expect_origin_distance("window", "body", axes="x", max_dist=0.010)

    ctx.expect_aabb_overlap("lid", "body", axes="xy", min_overlap=0.080)
    ctx.expect_aabb_gap("lid", "body", axis="z", max_gap=0.008, max_penetration=0.0)
    ctx.expect_joint_motion_axis("lid_hinge", "lid", world_axis="z", direction="positive", min_delta=0.030)

    ctx.expect_aabb_overlap("lid_button", "handle", axes="xy", min_overlap=0.010)
    ctx.expect_aabb_gap("lid_button", "handle", axis="z", max_gap=0.012, max_penetration=0.005)
    ctx.expect_joint_motion_axis(
        "lid_button_slide",
        "lid_button",
        world_axis="z",
        direction="positive",
        min_delta=0.002,
    )

    ctx.expect_aabb_contact("power_switch", "handle")
    ctx.expect_aabb_overlap("power_switch", "handle", axes="yz", min_overlap=0.006)
    ctx.expect_joint_motion_axis(
        "power_switch_joint",
        "power_switch",
        world_axis="z",
        direction="negative",
        min_delta=0.003,
    )

    with ctx.pose(lid_hinge=1.12):
        ctx.expect_aabb_overlap("lid", "handle", axes="yz", min_overlap=0.010)

    with ctx.pose(lid_button_slide=-0.004):
        ctx.expect_aabb_gap("lid_button", "handle", axis="z", max_gap=0.008, max_penetration=0.009)

    with ctx.pose(power_switch_joint=0.40):
        ctx.expect_aabb_contact("power_switch", "handle")
        ctx.expect_aabb_overlap("power_switch", "handle", axes="yz", min_overlap=0.006)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
