from __future__ import annotations

import math

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
)


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (center_x + x, center_y + y, z)
        for x, y in rounded_rect_profile(width, depth, radius)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_vacuum")

    body_red = model.material("body_red", rgba=(0.73, 0.12, 0.12, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.20, 0.21, 0.23, 1.0))
    translucent_bin = model.material("translucent_bin", rgba=(0.62, 0.78, 0.88, 0.35))
    wheel_silver = model.material("wheel_silver", rgba=(0.72, 0.74, 0.77, 1.0))

    head = model.part("head")
    head_shell = section_loft(
        [
            _xy_section(0.310, 0.118, 0.034, 0.000, center_x=0.024),
            _xy_section(0.320, 0.124, 0.038, 0.018, center_x=0.018),
            _xy_section(0.252, 0.108, 0.032, 0.038, center_x=-0.020),
            _xy_section(0.116, 0.074, 0.020, 0.056, center_x=-0.090),
        ]
    )
    head.visual(
        mesh_from_geometry(head_shell, "vacuum_head_shell"),
        material=charcoal,
        name="head_shell",
    )
    head.visual(
        Box((0.052, 0.064, 0.020)),
        origin=Origin(xyz=(-0.101, 0.0, 0.060)),
        material=dark_grey,
        name="neck_tower",
    )

    body = model.part("body")

    body_shell = section_loft(
        [
            _xy_section(0.084, 0.090, 0.020, 0.014, center_x=0.010),
            _xy_section(0.154, 0.114, 0.030, 0.120, center_x=-0.028),
            _xy_section(0.142, 0.108, 0.028, 0.320, center_x=-0.028),
            _xy_section(0.086, 0.080, 0.020, 0.580, center_x=-0.062),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell, "vacuum_body_shell"),
        material=body_red,
        name="body_shell",
    )
    body.visual(
        Box((0.044, 0.046, 0.040)),
        origin=Origin(xyz=(0.012, 0.0, 0.010)),
        material=dark_grey,
        name="pivot_block",
    )
    body.visual(
        Box((0.124, 0.078, 0.018)),
        origin=Origin(xyz=(-0.070, 0.0, 0.006)),
        material=dark_grey,
        name="axle_bridge",
    )
    for index, y_offset in enumerate((-0.048, 0.048)):
        body.visual(
            Box((0.066, 0.018, 0.086)),
            origin=Origin(xyz=(-0.102, y_offset, 0.002)),
            material=dark_grey,
            name=f"wheel_shroud_{index}",
        )
    body.visual(
        Box((0.038, 0.050, 0.115)),
        origin=Origin(xyz=(0.014, 0.0, 0.110)),
        material=body_red,
        name="dust_dock",
    )

    dust_cup_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.040, 0.000),
            (0.050, 0.018),
            (0.056, 0.060),
            (0.054, 0.205),
            (0.040, 0.248),
        ],
        inner_profile=[
            (0.000, 0.004),
            (0.045, 0.022),
            (0.051, 0.060),
            (0.049, 0.200),
            (0.035, 0.240),
        ],
        segments=40,
    )
    body.visual(
        mesh_from_geometry(dust_cup_shell, "vacuum_dust_cup"),
        origin=Origin(xyz=(0.080, 0.0, 0.076)),
        material=translucent_bin,
        name="dust_cup_shell",
    )
    body.visual(
        Cylinder(radius=0.048, length=0.034),
        origin=Origin(xyz=(0.080, 0.0, 0.233)),
        material=dark_grey,
        name="dust_cup_cap",
    )
    body.visual(
        Box((0.036, 0.034, 0.060)),
        origin=Origin(xyz=(0.056, 0.0, 0.160)),
        material=body_red,
        name="cyclone_spine",
    )

    handle_angle = -0.30
    handle_length = 0.400
    for index, y_offset in enumerate((-0.036, 0.036)):
        body.visual(
            Cylinder(radius=0.013, length=handle_length),
            origin=Origin(
                xyz=(-0.110, y_offset, 0.730),
                rpy=(0.0, handle_angle, 0.0),
            ),
            material=dark_grey,
            name=f"handle_tube_{index}",
        )
    body.visual(
        Cylinder(radius=0.016, length=0.150),
        origin=Origin(
            xyz=(-0.170, 0.0, 0.922),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=charcoal,
        name="grip_bar",
    )
    body.visual(
        Box((0.055, 0.090, 0.020)),
        origin=Origin(xyz=(-0.090, 0.0, 0.605)),
        material=body_red,
        name="handle_bridge",
    )

    recline = model.articulation(
        "head_to_body",
        ArticulationType.REVOLUTE,
        parent=head,
        child=body,
        origin=Origin(xyz=(-0.092, 0.0, 0.080)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(65.0),
        ),
    )

    for index, y_offset in enumerate((-0.069, 0.069)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.062, length=0.024),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.038, length=0.026),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_silver,
            name="hub",
        )
        model.articulation(
            f"body_to_wheel_{index}",
            ArticulationType.FIXED,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.118, y_offset, -0.020)),
        )

    body.meta["recline_joint"] = recline.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    head = object_model.get_part("head")
    body = object_model.get_part("body")
    recline = object_model.get_articulation("head_to_body")

    with ctx.pose({recline: 0.0}):
        ctx.expect_gap(
            body,
            head,
            axis="z",
            positive_elem="pivot_block",
            negative_elem="neck_tower",
            min_gap=0.0,
            max_gap=0.004,
            name="body clears the nozzle neck in parked pose",
        )
        ctx.expect_overlap(
            body,
            head,
            axes="y",
            min_overlap=0.060,
            name="body stays laterally centered over the nozzle",
        )

        parked_aabb = ctx.part_world_aabb(body)

    with ctx.pose({recline: math.radians(60.0)}):
        reclined_aabb = ctx.part_world_aabb(body)

    parked_ok = parked_aabb is not None
    reclined_ok = reclined_aabb is not None
    reclines_back = False
    if parked_ok and reclined_ok:
        reclines_back = (
            reclined_aabb[0][0] < parked_aabb[0][0] - 0.080
            and reclined_aabb[1][2] < parked_aabb[1][2] - 0.180
        )
    ctx.check(
        "body reclines backward from the floor head",
        reclines_back,
        details=f"parked_aabb={parked_aabb}, reclined_aabb={reclined_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
