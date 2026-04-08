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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_two_hole_punch")

    body_metal = model.material("body_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.24, 0.25, 0.28, 1.0))
    guide_metal = model.material("guide_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    dark_insert = model.material("dark_insert", rgba=(0.05, 0.05, 0.06, 1.0))

    def xz_section(
        *,
        width: float,
        height: float,
        radius: float,
        y: float,
        z0: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x, y, z0 + height * 0.5 + z)
            for x, z in rounded_rect_profile(width, height, radius, corner_segments=8)
        ]

    body = model.part("body")

    body.visual(
        Box((0.128, 0.188, 0.012)),
        origin=Origin(xyz=(0.0, 0.020, 0.006)),
        material=body_metal,
        name="rear_deck",
    )
    body.visual(
        Box((0.128, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, -0.098, 0.006)),
        material=body_metal,
        name="front_strip",
    )
    body.visual(
        Box((0.016, 0.008, 0.012)),
        origin=Origin(xyz=(-0.056, -0.078, 0.006)),
        material=body_metal,
        name="left_slot_bridge",
    )
    body.visual(
        Box((0.016, 0.008, 0.012)),
        origin=Origin(xyz=(0.056, -0.078, 0.006)),
        material=body_metal,
        name="right_slot_bridge",
    )
    body.visual(
        Box((0.090, 0.058, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, 0.005)),
        material=body_metal,
        name="underside_pad",
    )

    housing_sections = [
        xz_section(width=0.110, height=0.018, radius=0.006, y=-0.008, z0=0.012),
        xz_section(width=0.104, height=0.032, radius=0.007, y=0.022, z0=0.012),
        xz_section(width=0.096, height=0.046, radius=0.008, y=0.058, z0=0.012),
        xz_section(width=0.084, height=0.056, radius=0.008, y=0.094, z0=0.012),
    ]
    body.visual(
        mesh_from_geometry(section_loft(housing_sections), "two_hole_punch_housing"),
        material=body_metal,
        name="housing_shell",
    )
    body.visual(
        Box((0.108, 0.050, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, 0.014)),
        material=body_metal,
        name="paper_table",
    )
    body.visual(
        Box((0.050, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, 0.021)),
        material=body_metal,
        name="die_block",
    )
    for index, x in enumerate((-0.020, 0.020)):
        body.visual(
            Cylinder(radius=0.0065, length=0.024),
            origin=Origin(xyz=(x, 0.012, 0.026)),
            material=dark_insert,
            name=f"punch_sleeve_{index}",
        )
        body.visual(
            Cylinder(radius=0.0085, length=0.004),
            origin=Origin(xyz=(x, -0.002, 0.014)),
            material=dark_insert,
            name=f"die_opening_{index}",
        )

    body.visual(
        Box((0.018, 0.020, 0.022)),
        origin=Origin(xyz=(-0.048, 0.096, 0.063)),
        material=body_metal,
        name="left_hinge_support",
    )
    body.visual(
        Box((0.018, 0.020, 0.022)),
        origin=Origin(xyz=(0.048, 0.096, 0.063)),
        material=body_metal,
        name="right_hinge_support",
    )
    body.visual(
        Box((0.034, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.097, 0.062)),
        material=body_metal,
        name="center_hinge_bridge",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.022),
        origin=Origin(
            xyz=(-0.044, 0.106, 0.074),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_metal,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.024),
        origin=Origin(
            xyz=(0.0, 0.106, 0.074),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_metal,
        name="center_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.0075, length=0.022),
        origin=Origin(
            xyz=(0.044, 0.106, 0.074),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=body_metal,
        name="right_hinge_barrel",
    )

    for x, y, name in (
        (-0.045, -0.086, "left_front_foot"),
        (0.045, -0.086, "right_front_foot"),
        (-0.045, 0.086, "left_rear_foot"),
        (0.045, 0.086, "right_rear_foot"),
    ):
        body.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(x, y, -0.002)),
            material=rubber,
            name=name,
        )

    body.inertial = Inertial.from_geometry(
        Box((0.128, 0.228, 0.086)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0075, length=0.021),
        origin=Origin(
            xyz=(-0.0225, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_metal,
        name="left_handle_barrel",
    )
    handle.visual(
        Cylinder(radius=0.0075, length=0.021),
        origin=Origin(
            xyz=(0.0225, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_metal,
        name="right_handle_barrel",
    )
    handle.visual(
        Box((0.020, 0.050, 0.016)),
        origin=Origin(xyz=(-0.034, -0.028, 0.008)),
        material=handle_metal,
        name="left_web",
    )
    handle.visual(
        Box((0.020, 0.050, 0.016)),
        origin=Origin(xyz=(0.034, -0.028, 0.008)),
        material=handle_metal,
        name="right_web",
    )
    handle.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.112, 0.176, 0.014, corner_segments=8),
                0.014,
                cap=True,
                center=True,
                closed=True,
            ),
            "two_hole_punch_handle_grip",
        ),
        origin=Origin(xyz=(0.0, -0.094, 0.015)),
        material=handle_metal,
        name="handle_grip",
    )
    handle.visual(
        Box((0.094, 0.032, 0.008)),
        origin=Origin(xyz=(0.0, -0.156, 0.004)),
        material=handle_metal,
        name="front_press_bar",
    )
    handle.visual(
        Box((0.054, 0.022, 0.004)),
        origin=Origin(xyz=(0.0, -0.124, 0.022)),
        material=dark_insert,
        name="finger_pad",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.112, 0.182, 0.032)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.092, 0.012)),
    )

    guide = model.part("paper_guide")
    guide.visual(
        Box((0.032, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=guide_metal,
        name="guide_underplate",
    )
    guide.visual(
        Box((0.014, 0.007, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=guide_metal,
        name="guide_stem",
    )
    guide.visual(
        Box((0.030, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=guide_metal,
        name="guide_cap",
    )
    guide.visual(
        Box((0.004, 0.034, 0.022)),
        origin=Origin(xyz=(0.0, 0.017, 0.024)),
        material=guide_metal,
        name="guide_fence",
    )
    guide.inertial = Inertial.from_geometry(
        Box((0.032, 0.034, 0.030)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.010, 0.013)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.106, 0.074)),
        # The closed handle extends forward along local -Y from the rear hinge.
        # Using -X makes positive q raise the front of the handle upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "body_to_paper_guide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=guide,
        origin=Origin(xyz=(0.0, -0.078, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.12,
            lower=-0.038,
            upper=0.038,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    guide = object_model.get_part("paper_guide")
    handle_joint = object_model.get_articulation("body_to_handle")
    guide_joint = object_model.get_articulation("body_to_paper_guide")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.expect_overlap(
        handle,
        body,
        axes="x",
        min_overlap=0.090,
        name="handle spans the punch body width",
    )
    ctx.expect_within(
        guide,
        body,
        axes="x",
        margin=0.0,
        name="paper guide stays within the body width at rest",
    )

    rest_front = aabb_center(ctx.part_element_world_aabb(handle, elem="front_press_bar"))
    with ctx.pose({handle_joint: math.radians(55.0)}):
        open_front = aabb_center(ctx.part_element_world_aabb(handle, elem="front_press_bar"))
    ctx.check(
        "handle front lifts upward when opened",
        rest_front is not None
        and open_front is not None
        and open_front[2] > rest_front[2] + 0.075
        and open_front[1] > rest_front[1] + 0.020,
        details=f"rest_front={rest_front}, open_front={open_front}",
    )

    lower_limit = guide_joint.motion_limits.lower if guide_joint.motion_limits else None
    upper_limit = guide_joint.motion_limits.upper if guide_joint.motion_limits else None
    lower_pos = None
    upper_pos = None
    if lower_limit is not None:
        with ctx.pose({guide_joint: lower_limit}):
            ctx.expect_within(
                guide,
                body,
                axes="x",
                margin=0.0,
                name="paper guide stays within the body width at the left stop",
            )
            lower_pos = ctx.part_world_position(guide)
    if upper_limit is not None:
        with ctx.pose({guide_joint: upper_limit}):
            ctx.expect_within(
                guide,
                body,
                axes="x",
                margin=0.0,
                name="paper guide stays within the body width at the right stop",
            )
            upper_pos = ctx.part_world_position(guide)

    ctx.check(
        "paper guide slides laterally across the front slot",
        lower_pos is not None
        and upper_pos is not None
        and upper_pos[0] > lower_pos[0] + 0.070
        and abs(upper_pos[1] - lower_pos[1]) < 1e-6
        and abs(upper_pos[2] - lower_pos[2]) < 1e-6,
        details=f"lower_pos={lower_pos}, upper_pos={upper_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
