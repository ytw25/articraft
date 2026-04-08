from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEN = 0.290
BASE_WIDTH = 0.220
BASE_T = 0.012
BASE_CORNER = 0.040

SIDE_Y = 0.088
SUPPORT_OFFSET = 0.018
REAR_PIVOT_X = -0.082
LOWER_PIVOT_Z = 0.032
UPPER_PIVOT_Z = 0.076
PIVOT_SPACING = UPPER_PIVOT_Z - LOWER_PIVOT_Z

LINK_LENGTH = 0.185
LINK_THICK = 0.010
HOUSING_LEN = 0.012
HOUSING_R = 0.017
HOLE_R = 0.0062
CROSS_TUBE_R = 0.0065
SIDE_PLATE_T = 0.008
SIDE_PLATE_Y = SIDE_Y + HOUSING_LEN / 2.0 + SIDE_PLATE_T / 2.0
PIVOT_BOSS_R = 0.020

PLATFORM_LEN = 0.310
PLATFORM_WIDTH = 0.245
PLATFORM_T = 0.008
PLATFORM_CORNER = 0.017
PLATFORM_REAR_OVERHANG = 0.028
PLATE_Z = 0.020
LOWER_PIN_LOCAL_Z = -PIVOT_SPACING
VENT_LEN = 0.175
VENT_WIDTH = 0.105
VENT_CENTER_X = 0.108
VENT_CORNER = 0.016
LIP_T = 0.008
LIP_WIDTH = 0.205
LIP_H = 0.014

RAISED_ANGLE = 0.65


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _base_plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_LEN, BASE_WIDTH, BASE_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(BASE_CORNER)
    )


def _base_bracket_shape() -> cq.Workplane:
    bracket_height = UPPER_PIVOT_Z - BASE_T + 0.030
    center_z = BASE_T + bracket_height / 2.0
    supports = []
    for side in (-1.0, 1.0):
        side_plate = (
            cq.Workplane("XZ")
            .center(REAR_PIVOT_X - 0.004, center_z)
            .rect(0.026, bracket_height)
            .extrude(SIDE_PLATE_T / 2.0, both=True)
            .translate((0.0, side * SIDE_PLATE_Y, 0.0))
        )
        lower_boss = (
            cq.Workplane("XZ")
            .center(REAR_PIVOT_X, LOWER_PIVOT_Z)
            .circle(PIVOT_BOSS_R)
            .extrude(SIDE_PLATE_T / 2.0, both=True)
            .translate((0.0, side * SIDE_PLATE_Y, 0.0))
        )
        upper_boss = (
            cq.Workplane("XZ")
            .center(REAR_PIVOT_X, UPPER_PIVOT_Z)
            .circle(PIVOT_BOSS_R)
            .extrude(SIDE_PLATE_T / 2.0, both=True)
            .translate((0.0, side * SIDE_PLATE_Y, 0.0))
        )
        foot = (
            cq.Workplane("XY")
            .box(0.042, SIDE_PLATE_T, 0.014, centered=(True, True, False))
            .translate((REAR_PIVOT_X - 0.010, side * SIDE_PLATE_Y, BASE_T))
        )
        supports.append(side_plate.union(lower_boss).union(upper_boss).union(foot))
    return _union_all(supports)


def _platform_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATFORM_LEN, PLATFORM_WIDTH, PLATFORM_T)
        .translate(
            (
                PLATFORM_LEN / 2.0 - PLATFORM_REAR_OVERHANG,
                0.0,
                PLATE_Z + PLATFORM_T / 2.0,
            )
        )
        .edges("|Z")
        .fillet(PLATFORM_CORNER)
    )
    vent = (
        cq.Workplane("XY")
        .box(VENT_LEN, VENT_WIDTH, PLATFORM_T + 0.020)
        .edges("|Z")
        .fillet(VENT_CORNER)
        .translate((VENT_CENTER_X, 0.0, PLATE_Z + PLATFORM_T / 2.0))
    )
    return plate.cut(vent)


def _platform_lip_shape() -> cq.Workplane:
    front_x = PLATFORM_LEN - PLATFORM_REAR_OVERHANG - LIP_T / 2.0
    return (
        cq.Workplane("XY")
        .box(LIP_T, LIP_WIDTH, LIP_H, centered=(True, True, False))
        .translate((front_x, 0.0, PLATE_Z + PLATFORM_T))
    )


def _platform_bracket_shape() -> cq.Workplane:
    bracket_bottom = LOWER_PIN_LOCAL_Z - 0.008
    bracket_top = PLATE_Z + PLATFORM_T + 0.004
    bracket_height = bracket_top - bracket_bottom
    bracket_center_z = bracket_bottom + bracket_height / 2.0
    brackets = []
    for side in (-1.0, 1.0):
        side_plate = (
            cq.Workplane("XZ")
            .center(0.008, bracket_center_z)
            .rect(0.022, bracket_height)
            .extrude(SIDE_PLATE_T / 2.0, both=True)
            .translate((0.0, side * SIDE_PLATE_Y, 0.0))
        )
        upper_boss = (
            cq.Workplane("XZ")
            .center(0.0, 0.0)
            .circle(PIVOT_BOSS_R)
            .extrude(SIDE_PLATE_T / 2.0, both=True)
            .translate((0.0, side * SIDE_PLATE_Y, 0.0))
        )
        lower_boss = (
            cq.Workplane("XZ")
            .center(0.0, LOWER_PIN_LOCAL_Z)
            .circle(PIVOT_BOSS_R)
            .extrude(SIDE_PLATE_T / 2.0, both=True)
            .translate((0.0, side * SIDE_PLATE_Y, 0.0))
        )
        deck_tab = (
            cq.Workplane("XY")
            .box(0.048, SIDE_PLATE_T, 0.012, centered=(True, True, False))
            .translate((0.020, side * SIDE_PLATE_Y, PLATE_Z + PLATFORM_T - 0.004))
        )
        brackets.append(side_plate.union(upper_boss).union(lower_boss).union(deck_tab))
    return _union_all(brackets)


def _single_link_bar() -> cq.Workplane:
    web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.008, -0.009),
                (0.045, -0.010),
                (LINK_LENGTH - 0.045, -0.010),
                (LINK_LENGTH - 0.008, -0.009),
                (LINK_LENGTH - 0.008, 0.009),
                (LINK_LENGTH - 0.045, 0.010),
                (0.045, 0.010),
                (0.008, 0.009),
            ]
        )
        .close()
        .extrude(LINK_THICK / 2.0, both=True)
    )
    front_housing = cq.Workplane("XZ").center(LINK_LENGTH, 0.0).circle(HOUSING_R).extrude(HOUSING_LEN / 2.0, both=True)
    rear_housing = cq.Workplane("XZ").center(0.0, 0.0).circle(HOUSING_R).extrude(HOUSING_LEN / 2.0, both=True)
    body = web.union(front_housing).union(rear_housing)
    front_hole = cq.Workplane("XZ").center(LINK_LENGTH, 0.0).circle(HOLE_R).extrude((HOUSING_LEN + 0.008) / 2.0, both=True)
    rear_hole = cq.Workplane("XZ").center(0.0, 0.0).circle(HOLE_R).extrude((HOUSING_LEN + 0.008) / 2.0, both=True)
    return body.cut(front_hole).cut(rear_hole)


def _link_pair_shape(cross_x: float, cross_z: float = 0.0) -> cq.Workplane:
    left = _single_link_bar().translate((0.0, SIDE_Y, 0.0))
    right = _single_link_bar().translate((0.0, -SIDE_Y, 0.0))
    cross_tube = (
        cq.Workplane("XZ")
        .center(cross_x, cross_z)
        .circle(CROSS_TUBE_R)
        .extrude(SIDE_Y, both=True)
    )
    return left.union(right).union(cross_tube)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_adjustable_laptop_stand")

    model.material("graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("graphite_dark", rgba=(0.15, 0.17, 0.19, 1.0))
    model.material("silver", rgba=(0.78, 0.80, 0.84, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_plate_shape(), "base_plate"), material="graphite_dark", name="base_plate")
    base.visual(mesh_from_cadquery(_base_bracket_shape(), "rear_brackets"), material="graphite", name="rear_brackets")
    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_WIDTH, 0.110)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    upper_links = model.part("upper_links")
    upper_links.visual(
        mesh_from_cadquery(_link_pair_shape(cross_x=LINK_LENGTH * 0.56, cross_z=0.002), "upper_links_body"),
        material="graphite",
        name="upper_links_body",
    )
    upper_links.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH, 2.0 * SIDE_Y + 0.020, 0.040)),
        mass=0.34,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    lower_links = model.part("lower_links")
    lower_links.visual(
        mesh_from_cadquery(_link_pair_shape(cross_x=LINK_LENGTH * 0.48, cross_z=-0.002), "lower_links_body"),
        material="graphite",
        name="lower_links_body",
    )
    lower_links.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH, 2.0 * SIDE_Y + 0.020, 0.040)),
        mass=0.36,
        origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
    )

    platform = model.part("platform")
    platform.visual(mesh_from_cadquery(_platform_plate_shape(), "platform_plate"), material="graphite_dark", name="platform_plate")
    platform.visual(mesh_from_cadquery(_platform_lip_shape(), "platform_lip"), material="graphite", name="platform_lip")
    platform.visual(
        mesh_from_cadquery(_platform_bracket_shape(), "platform_brackets"),
        material="graphite",
        name="platform_brackets",
    )
    platform.inertial = Inertial.from_geometry(
        Box((PLATFORM_LEN, PLATFORM_WIDTH, 0.090)),
        mass=0.88,
        origin=Origin(xyz=(PLATFORM_LEN / 2.0 - PLATFORM_REAR_OVERHANG, 0.0, 0.010)),
    )

    model.articulation(
        "base_to_upper_links",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_links,
        origin=Origin(xyz=(REAR_PIVOT_X, 0.0, UPPER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.90, effort=20.0, velocity=1.4),
    )
    model.articulation(
        "upper_links_to_platform",
        ArticulationType.REVOLUTE,
        parent=upper_links,
        child=platform,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.00, upper=0.20, effort=12.0, velocity=1.8),
    )
    model.articulation(
        "base_to_lower_links",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_links,
        origin=Origin(xyz=(REAR_PIVOT_X, 0.0, LOWER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.90, effort=20.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_links = object_model.get_part("lower_links")
    platform = object_model.get_part("platform")
    upper_joint = object_model.get_articulation("base_to_upper_links")
    lower_joint = object_model.get_articulation("base_to_lower_links")
    level_joint = object_model.get_articulation("upper_links_to_platform")

    ctx.expect_gap(
        platform,
        base,
        axis="z",
        positive_elem="platform_plate",
        negative_elem="base_plate",
        min_gap=0.080,
        name="platform plate clears the oval base at rest",
    )
    ctx.expect_overlap(
        platform,
        base,
        axes="y",
        elem_a="platform_plate",
        elem_b="base_plate",
        min_overlap=0.160,
        name="platform stays laterally centered over the base",
    )
    ctx.expect_contact(
        lower_links,
        platform,
        contact_tol=0.0012,
        name="lower passive pivots stay engaged at rest",
    )

    rest_platform_pos = ctx.part_world_position(platform)
    rest_plate_aabb = ctx.part_element_world_aabb(platform, elem="platform_plate")
    rest_plate_z_size = None if rest_plate_aabb is None else rest_plate_aabb[1][2] - rest_plate_aabb[0][2]
    ctx.check(
        "rest platform plate stays nearly level",
        rest_plate_z_size is not None and rest_plate_z_size < 0.015,
        details=f"plate z-size={rest_plate_z_size}",
    )

    with ctx.pose(
        {
            upper_joint: RAISED_ANGLE,
            lower_joint: RAISED_ANGLE,
            level_joint: -RAISED_ANGLE,
        }
    ):
        ctx.expect_gap(
            platform,
            base,
            axis="z",
            positive_elem="platform_plate",
            negative_elem="base_plate",
            min_gap=0.185,
            name="platform rises well above the base when opened",
        )
        ctx.expect_contact(
            lower_links,
            platform,
            contact_tol=0.0012,
            name="lower passive pivots stay engaged when opened",
        )
        raised_platform_pos = ctx.part_world_position(platform)
        raised_plate_aabb = ctx.part_element_world_aabb(platform, elem="platform_plate")
        raised_plate_z_size = None if raised_plate_aabb is None else raised_plate_aabb[1][2] - raised_plate_aabb[0][2]
        ctx.check(
            "opened platform plate stays nearly level",
            raised_plate_z_size is not None and raised_plate_z_size < 0.018,
            details=f"plate z-size={raised_plate_z_size}",
        )

    ctx.check(
        "platform origin rises between rest and opened poses",
        rest_platform_pos is not None
        and raised_platform_pos is not None
        and raised_platform_pos[2] > rest_platform_pos[2] + 0.090,
        details=f"rest={rest_platform_pos}, raised={raised_platform_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
