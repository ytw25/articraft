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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_split_air_conditioner")

    housing = model.material("housing_white", rgba=(0.94, 0.95, 0.94, 1.0))
    housing_shadow = model.material("housing_shadow", rgba=(0.86, 0.88, 0.88, 1.0))
    interior = model.material("interior_dark", rgba=(0.26, 0.29, 0.31, 1.0))
    outlet = model.material("outlet_dark", rgba=(0.19, 0.21, 0.23, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.98, 0.006, 0.30)),
        origin=Origin(xyz=(0.0, 0.003, 0.150)),
        material=housing_shadow,
        name="back_panel",
    )
    body.visual(
        Box((0.98, 0.140, 0.070)),
        origin=Origin(xyz=(0.0, 0.070, 0.275)),
        material=housing,
        name="top_shell",
    )
    body.visual(
        Box((0.018, 0.142, 0.210)),
        origin=Origin(xyz=(-0.481, 0.071, 0.170)),
        material=housing,
        name="left_side_shell",
    )
    body.visual(
        Box((0.018, 0.142, 0.210)),
        origin=Origin(xyz=(0.481, 0.071, 0.170)),
        material=housing,
        name="right_side_shell",
    )
    body.visual(
        Box((0.94, 0.115, 0.040)),
        origin=Origin(xyz=(0.0, 0.0575, 0.020)),
        material=housing_shadow,
        name="bottom_tray",
    )
    body.visual(
        Box((0.90, 0.014, 0.058)),
        origin=Origin(xyz=(0.0, 0.083, 0.069)),
        material=interior,
        name="outlet_rear_wall",
    )
    body.visual(
        Box((0.90, 0.070, 0.014)),
        origin=Origin(xyz=(0.0, 0.104, 0.105)),
        material=interior,
        name="outlet_roof",
    )
    body.visual(
        Box((0.90, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.149, 0.040)),
        material=housing,
        name="front_lip",
    )
    body.visual(
        Box((0.020, 0.080, 0.072)),
        origin=Origin(xyz=(-0.450, 0.112, 0.076)),
        material=housing,
        name="left_outlet_end",
    )
    body.visual(
        Box((0.020, 0.080, 0.072)),
        origin=Origin(xyz=(0.450, 0.112, 0.076)),
        material=housing,
        name="right_outlet_end",
    )
    body.visual(
        Box((0.90, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.144, 0.257)),
        material=housing,
        name="front_frame_top",
    )
    body.visual(
        Box((0.018, 0.022, 0.135)),
        origin=Origin(xyz=(-0.451, 0.144, 0.1795)),
        material=housing,
        name="left_front_frame",
    )
    body.visual(
        Box((0.018, 0.022, 0.135)),
        origin=Origin(xyz=(0.451, 0.144, 0.1795)),
        material=housing,
        name="right_front_frame",
    )
    body.visual(
        Box((0.88, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.130, 0.088)),
        material=outlet,
        name="outlet_brow",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.98, 0.23, 0.31)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.115, 0.155)),
    )

    cover = model.part("service_cover")
    cover.visual(
        Box((0.94, 0.014, 0.156)),
        origin=Origin(xyz=(0.0, 0.026, -0.082)),
        material=housing,
        name="cover_panel",
    )
    cover.visual(
        Box((0.94, 0.026, 0.052)),
        origin=Origin(xyz=(0.0, 0.036, -0.160)),
        material=housing_shadow,
        name="cover_lower_bulge",
    )
    cover.visual(
        Box((0.94, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.022, -0.010)),
        material=housing,
        name="cover_top_spine",
    )
    cover.visual(
        Box((0.88, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.007, -0.004)),
        material=housing_shadow,
        name="cover_hinge_lip",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.94, 0.040, 0.190)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.020, -0.095)),
    )
    model.articulation(
        "body_to_service_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, 0.148, 0.274)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    flap = model.part("discharge_flap")
    flap.visual(
        Box((0.872, 0.006, 0.048)),
        origin=Origin(xyz=(0.0, 0.003, 0.035)),
        material=housing,
        name="flap_panel",
    )
    flap.inertial = Inertial.from_geometry(
        Box((0.872, 0.010, 0.050)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.003, 0.035)),
    )
    model.articulation(
        "body_to_discharge_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, 0.146, 0.038)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.00,
        ),
    )

    louver_x_positions = (-0.33, -0.22, -0.11, 0.0, 0.11, 0.22, 0.33)
    for index, x_pos in enumerate(louver_x_positions, start=1):
        louver = model.part(f"louver_{index}")
        louver.visual(
            Box((0.008, 0.024, 0.044)),
            origin=Origin(),
            material=outlet,
            name="louver_blade",
        )
        louver.visual(
            Cylinder(radius=0.003, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, 0.0255)),
            material=outlet,
            name="top_pivot_pin",
        )
        louver.visual(
            Cylinder(radius=0.003, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, -0.0255)),
            material=outlet,
            name="bottom_pivot_pin",
        )
        louver.inertial = Inertial.from_geometry(
            Box((0.012, 0.024, 0.058)),
            mass=0.06,
            origin=Origin(),
        )
        model.articulation(
            f"body_to_louver_{index}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=louver,
            origin=Origin(xyz=(x_pos, 0.104, 0.069)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.0,
                lower=-0.65,
                upper=0.65,
            ),
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
    cover = object_model.get_part("service_cover")
    flap = object_model.get_part("discharge_flap")
    cover_joint = object_model.get_articulation("body_to_service_cover")
    flap_joint = object_model.get_articulation("body_to_discharge_flap")
    center_louver = object_model.get_part("louver_4")
    center_louver_joint = object_model.get_articulation("body_to_louver_4")
    neighbor_louver = object_model.get_part("louver_5")

    ctx.expect_overlap(
        cover,
        body,
        axes="x",
        elem_a="cover_panel",
        elem_b="front_frame_top",
        min_overlap=0.84,
        name="service cover spans the front opening width",
    )
    ctx.expect_overlap(
        flap,
        body,
        axes="x",
        elem_a="flap_panel",
        elem_b="front_lip",
        min_overlap=0.82,
        name="discharge flap spans the outlet width",
    )
    ctx.expect_gap(
        flap,
        body,
        axis="y",
        positive_elem="flap_panel",
        negative_elem="outlet_rear_wall",
        min_gap=0.020,
        max_gap=0.080,
        name="flap sits in front of the outlet interior",
    )

    closed_cover = ctx.part_element_world_aabb(cover, elem="cover_panel")
    with ctx.pose({cover_joint: 1.10}):
        opened_cover = ctx.part_element_world_aabb(cover, elem="cover_panel")
    ctx.check(
        "service cover opens upward",
        closed_cover is not None
        and opened_cover is not None
        and opened_cover[0][2] > closed_cover[0][2] + 0.08,
        details=f"closed={closed_cover}, opened={opened_cover}",
    )

    closed_flap = ctx.part_element_world_aabb(flap, elem="flap_panel")
    with ctx.pose({flap_joint: 0.80}):
        opened_flap = ctx.part_element_world_aabb(flap, elem="flap_panel")
    ctx.check(
        "discharge flap swings down and forward",
        closed_flap is not None
        and opened_flap is not None
        and opened_flap[0][2] < closed_flap[0][2] - 0.004
        and opened_flap[1][1] > closed_flap[1][1] + 0.02,
        details=f"closed={closed_flap}, opened={opened_flap}",
    )

    closed_center_louver = ctx.part_world_aabb(center_louver)
    closed_neighbor_louver = ctx.part_world_aabb(neighbor_louver)
    with ctx.pose({center_louver_joint: 0.45}):
        turned_center_louver = ctx.part_world_aabb(center_louver)
        turned_neighbor_louver = ctx.part_world_aabb(neighbor_louver)
    ctx.check(
        "center louver rotates on its own vertical pivot",
        closed_center_louver is not None
        and turned_center_louver is not None
        and (turned_center_louver[1][0] - turned_center_louver[0][0])
        > (closed_center_louver[1][0] - closed_center_louver[0][0]) + 0.004,
        details=f"closed={closed_center_louver}, turned={turned_center_louver}",
    )
    ctx.check(
        "neighbor louver stays put when center louver turns",
        closed_neighbor_louver == turned_neighbor_louver,
        details=f"closed={closed_neighbor_louver}, turned={turned_neighbor_louver}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
