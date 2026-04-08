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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 0.94
BODY_D = 0.23
BODY_H = 0.32

UPPER_SHELL_H = 0.21
UPPER_SHELL_Z = BODY_H - UPPER_SHELL_H / 2.0

OPENING_W = 0.78
OPENING_SIDE_W = (BODY_W - OPENING_W) / 2.0
LOWER_SECTION_H = 0.11
LOWER_SECTION_Z = LOWER_SECTION_H / 2.0

SERVICE_HATCH_T = 0.008
SERVICE_HATCH_D = 0.115
SERVICE_HATCH_H = 0.17
SERVICE_HATCH_FRONT_Y = 0.068
SERVICE_HATCH_Z = 0.18
SERVICE_HATCH_X = BODY_W / 2.0

FLAP_W = OPENING_W
FLAP_T = 0.010
FLAP_H = 0.058
FLAP_Y = 0.109
FLAP_Z = 0.084

LOUVER_Y = 0.064
LOUVER_Z = 0.055
LOUVER_W = 0.010
LOUVER_D = 0.048
LOUVER_H = 0.046
LOUVER_ROD_H = 0.050
LOUVER_XS = (-0.26, -0.13, 0.0, 0.13, 0.26)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_air_conditioner")

    housing_white = model.material("housing_white", rgba=(0.93, 0.94, 0.95, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    vent_dark = model.material("vent_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    accent_dark = model.material("accent_dark", rgba=(0.34, 0.36, 0.39, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, UPPER_SHELL_H)),
        origin=Origin(xyz=(0.0, 0.0, UPPER_SHELL_Z)),
        material=housing_white,
        name="upper_shell",
    )
    body.visual(
        Box((OPENING_SIDE_W, BODY_D, LOWER_SECTION_H)),
        origin=Origin(xyz=(-(OPENING_W / 2.0 + OPENING_SIDE_W / 2.0), 0.0, LOWER_SECTION_Z)),
        material=housing_white,
        name="left_lower_cheek",
    )
    body.visual(
        Box((OPENING_SIDE_W, BODY_D, LOWER_SECTION_H)),
        origin=Origin(xyz=((OPENING_W / 2.0 + OPENING_SIDE_W / 2.0), 0.0, LOWER_SECTION_Z)),
        material=housing_white,
        name="right_lower_cheek",
    )
    body.visual(
        Box((OPENING_W, 0.13, LOWER_SECTION_H)),
        origin=Origin(xyz=(0.0, -0.05, LOWER_SECTION_Z)),
        material=housing_white,
        name="rear_plenum",
    )
    body.visual(
        Box((OPENING_W, 0.022, 0.022)),
        origin=Origin(xyz=(0.0, 0.104, 0.099)),
        material=housing_white,
        name="opening_header",
    )
    body.visual(
        Box((OPENING_W, 0.038, 0.024)),
        origin=Origin(xyz=(0.0, 0.085, 0.012)),
        material=housing_white,
        name="opening_sill",
    )
    body.visual(
        Box((OPENING_W, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, LOUVER_Y, 0.084)),
        material=vent_dark,
        name="upper_track",
    )
    body.visual(
        Box((OPENING_W, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, LOUVER_Y, 0.026)),
        material=vent_dark,
        name="lower_track",
    )
    body.visual(
        Box((BODY_W - 0.10, 0.006, 0.10)),
        origin=Origin(xyz=(0.0, -(BODY_D / 2.0), 0.16)),
        material=trim_gray,
        name="rear_mount_plate",
    )

    hatch = model.part("service_hatch")
    hatch.visual(
        Box((SERVICE_HATCH_T, SERVICE_HATCH_D, SERVICE_HATCH_H)),
        origin=Origin(xyz=(SERVICE_HATCH_T / 2.0, -(SERVICE_HATCH_D / 2.0), 0.0)),
        material=housing_white,
        name="hatch_panel",
    )
    hatch.visual(
        Cylinder(radius=0.0045, length=SERVICE_HATCH_H),
        origin=Origin(xyz=(SERVICE_HATCH_T / 2.0, 0.0, 0.0)),
        material=trim_gray,
        name="hatch_barrel",
    )
    hatch.visual(
        Box((0.005, 0.020, 0.040)),
        origin=Origin(xyz=(0.006, -(SERVICE_HATCH_D - 0.010), 0.0)),
        material=accent_dark,
        name="hatch_pull",
    )
    model.articulation(
        "body_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(SERVICE_HATCH_X, SERVICE_HATCH_FRONT_Y, SERVICE_HATCH_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    flap = model.part("outlet_flap")
    flap.visual(
        Cylinder(radius=0.004, length=FLAP_W),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_gray,
        name="flap_rod",
    )
    flap.visual(
        Box((FLAP_W, FLAP_T, FLAP_H)),
        origin=Origin(xyz=(0.0, -0.003, -(FLAP_H / 2.0))),
        material=trim_gray,
        name="flap_panel",
    )
    flap.visual(
        Box((FLAP_W, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.001, -(FLAP_H - 0.005))),
        material=accent_dark,
        name="flap_edge",
    )
    model.articulation(
        "body_to_outlet_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, FLAP_Y, FLAP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    for index, louver_x in enumerate(LOUVER_XS, start=1):
        louver = model.part(f"louver_{index}")
        louver.visual(
            Cylinder(radius=0.0025, length=LOUVER_ROD_H),
            origin=Origin(),
            material=accent_dark,
            name="pivot_rod",
        )
        louver.visual(
            Box((LOUVER_W, LOUVER_D, LOUVER_H)),
            origin=Origin(),
            material=vent_dark,
            name="vane_panel",
        )
        louver.visual(
            Box((0.005, 0.010, 0.018)),
            origin=Origin(xyz=(0.0, LOUVER_D / 2.0 - 0.005, 0.0)),
            material=accent_dark,
            name="vane_spine",
        )
        model.articulation(
            f"body_to_louver_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=louver,
            origin=Origin(xyz=(louver_x, LOUVER_Y, LOUVER_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.75, upper=0.75),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    hatch = object_model.get_part("service_hatch")
    flap = object_model.get_part("outlet_flap")
    louver_1 = object_model.get_part("louver_1")
    louver_2 = object_model.get_part("louver_2")
    louver_3 = object_model.get_part("louver_3")

    hatch_hinge = object_model.get_articulation("body_to_service_hatch")
    flap_hinge = object_model.get_articulation("body_to_outlet_flap")
    louver_1_joint = object_model.get_articulation("body_to_louver_1")
    louver_2_joint = object_model.get_articulation("body_to_louver_2")
    louver_3_joint = object_model.get_articulation("body_to_louver_3")

    def span(aabb, axis: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis] - aabb[0][axis]

    ctx.expect_gap(
        hatch,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="hatch_panel",
        name="service hatch sits just proud of the side shell",
    )
    ctx.expect_overlap(
        hatch,
        body,
        axes="yz",
        min_overlap=0.10,
        elem_a="hatch_panel",
        name="service hatch covers a substantial side-face footprint",
    )

    ctx.expect_gap(
        body,
        flap,
        axis="z",
        min_gap=0.002,
        max_gap=0.006,
        positive_elem="opening_header",
        negative_elem="flap_panel",
        name="closed outlet flap tucks below the opening header",
    )
    ctx.expect_gap(
        flap,
        body,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="flap_panel",
        negative_elem="opening_sill",
        name="closed outlet flap clears the lower sill",
    )
    ctx.expect_overlap(
        flap,
        body,
        axes="x",
        min_overlap=0.70,
        elem_a="flap_panel",
        elem_b="opening_sill",
        name="outlet flap spans the long discharge opening",
    )

    ctx.expect_gap(
        body,
        louver_3,
        axis="z",
        min_gap=0.002,
        max_gap=0.012,
        positive_elem="upper_track",
        negative_elem="vane_panel",
        name="center louver clears the upper track",
    )
    ctx.expect_gap(
        louver_3,
        body,
        axis="z",
        min_gap=0.002,
        max_gap=0.012,
        positive_elem="vane_panel",
        negative_elem="lower_track",
        name="center louver clears the lower track",
    )
    ctx.expect_gap(
        flap,
        louver_3,
        axis="y",
        min_gap=0.008,
        max_gap=0.030,
        positive_elem="flap_panel",
        negative_elem="vane_panel",
        name="louvers sit behind the outlet flap",
    )

    hatch_rest = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({hatch_hinge: 1.0}):
        hatch_open = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    ctx.check(
        "service hatch swings outward",
        hatch_rest is not None
        and hatch_open is not None
        and hatch_open[1][0] > hatch_rest[1][0] + 0.07,
        details=f"closed={hatch_rest}, open={hatch_open}",
    )

    flap_rest = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_hinge: 0.95}):
        flap_open = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "outlet flap swings forward",
        flap_rest is not None
        and flap_open is not None
        and flap_open[1][1] > flap_rest[1][1] + 0.04,
        details=f"closed={flap_rest}, open={flap_open}",
    )

    louver_1_rest = ctx.part_element_world_aabb(louver_1, elem="vane_panel")
    louver_2_rest = ctx.part_element_world_aabb(louver_2, elem="vane_panel")
    with ctx.pose({louver_1_joint: 0.65, louver_2_joint: 0.0, louver_3_joint: -0.45}):
        louver_1_turned = ctx.part_element_world_aabb(louver_1, elem="vane_panel")
        louver_2_still = ctx.part_element_world_aabb(louver_2, elem="vane_panel")
        louver_3_turned = ctx.part_element_world_aabb(louver_3, elem="vane_panel")
    ctx.check(
        "louvers articulate independently",
        span(louver_1_rest, 0) is not None
        and span(louver_2_rest, 0) is not None
        and span(louver_1_turned, 0) is not None
        and span(louver_2_still, 0) is not None
        and span(louver_1_turned, 0) > span(louver_1_rest, 0) + 0.015
        and abs(span(louver_2_still, 0) - span(louver_2_rest, 0)) < 0.003,
        details=(
            f"l1_rest={louver_1_rest}, l1_turned={louver_1_turned}, "
            f"l2_rest={louver_2_rest}, l2_still={louver_2_still}, "
            f"l3_turned={louver_3_turned}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
