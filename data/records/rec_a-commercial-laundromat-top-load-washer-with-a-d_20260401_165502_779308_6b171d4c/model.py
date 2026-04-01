from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_top_load_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.90, 0.91, 0.92, 1.0))
    lid_steel = model.material("lid_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    basket_steel = model.material("basket_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    tub_gray = model.material("tub_gray", rgba=(0.66, 0.69, 0.72, 1.0))
    console_dark = model.material("console_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.08, 0.08, 0.09, 1.0))

    cabinet_width = 0.69
    cabinet_depth = 0.74
    base_height = 0.06
    body_height = 0.88
    top_height = base_height + body_height
    opening_width = 0.56
    opening_depth = 0.50
    opening_center_y = -0.02

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((cabinet_width, cabinet_depth, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=cabinet_white,
        name="base_pan",
    )
    cabinet.visual(
        Cylinder(radius=0.090, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.1725)),
        material=trim_dark,
        name="center_support",
    )
    cabinet.visual(
        Box((0.02, cabinet_depth, body_height)),
        origin=Origin(xyz=(-0.335, 0.0, base_height + body_height * 0.5)),
        material=cabinet_white,
        name="left_wall",
    )
    cabinet.visual(
        Box((0.02, cabinet_depth, body_height)),
        origin=Origin(xyz=(0.335, 0.0, base_height + body_height * 0.5)),
        material=cabinet_white,
        name="right_wall",
    )
    cabinet.visual(
        Box((cabinet_width - 0.04, 0.02, body_height)),
        origin=Origin(xyz=(0.0, -0.36, base_height + body_height * 0.5)),
        material=cabinet_white,
        name="front_wall",
    )
    cabinet.visual(
        Box((cabinet_width - 0.04, 0.02, 0.76)),
        origin=Origin(xyz=(0.0, 0.36, base_height + 0.38)),
        material=cabinet_white,
        name="rear_lower_wall",
    )
    cabinet.visual(
        Box((0.065, opening_depth, 0.03)),
        origin=Origin(xyz=(-0.3125, opening_center_y, top_height - 0.015)),
        material=cabinet_white,
        name="top_left_rail",
    )
    cabinet.visual(
        Box((0.065, opening_depth, 0.03)),
        origin=Origin(xyz=(0.3125, opening_center_y, top_height - 0.015)),
        material=cabinet_white,
        name="top_right_rail",
    )
    cabinet.visual(
        Box((opening_width, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, -0.32, top_height - 0.015)),
        material=cabinet_white,
        name="top_front_rail",
    )
    cabinet.visual(
        Box((cabinet_width - 0.04, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, 0.25, top_height - 0.015)),
        material=cabinet_white,
        name="rear_hinge_deck",
    )
    cabinet.visual(
        Box((0.40, 0.015, 0.008)),
        origin=Origin(xyz=(0.0, -0.27, top_height - 0.004)),
        material=rubber_dark,
        name="lid_strike",
    )
    cabinet.visual(
        Box((cabinet_width - 0.04, 0.08, 0.18)),
        origin=Origin(xyz=(0.0, 0.29, 1.03)),
        material=cabinet_white,
        name="console_front",
    )
    cabinet.visual(
        Box((cabinet_width - 0.02, 0.13, 0.025)),
        origin=Origin(xyz=(0.0, 0.335, 1.125)),
        material=cabinet_white,
        name="console_cap",
    )
    cabinet.visual(
        Box((0.46, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.275, 1.09), rpy=(-0.16, 0.0, 0.0)),
        material=console_dark,
        name="control_strip",
    )
    cabinet.visual(
        Box((0.14, 0.003, 0.10)),
        origin=Origin(xyz=(0.195, 0.249, 0.99)),
        material=trim_dark,
        name="coin_module_backplate",
    )
    cabinet.visual(
        Box((0.118, 0.002, 0.012)),
        origin=Origin(xyz=(0.195, 0.249, 1.031)),
        material=trim_dark,
        name="coin_bezel_top",
    )
    cabinet.visual(
        Box((0.118, 0.002, 0.012)),
        origin=Origin(xyz=(0.195, 0.249, 0.949)),
        material=trim_dark,
        name="coin_bezel_bottom",
    )
    cabinet.visual(
        Box((0.012, 0.002, 0.070)),
        origin=Origin(xyz=(0.141, 0.249, 0.99)),
        material=trim_dark,
        name="coin_bezel_left",
    )
    cabinet.visual(
        Box((0.012, 0.002, 0.070)),
        origin=Origin(xyz=(0.249, 0.249, 0.99)),
        material=trim_dark,
        name="coin_bezel_right",
    )
    cabinet.visual(
        Box((0.050, 0.005, 0.008)),
        origin=Origin(xyz=(0.195, 0.250, 1.055)),
        material=trim_dark,
        name="coin_slot",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, 1.16)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.58)),
    )

    tub = model.part("tub")
    tub_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.065, 0.00),
                (0.110, 0.025),
                (0.185, 0.070),
                (0.255, 0.130),
                (0.295, 0.220),
                (0.300, 0.580),
                (0.292, 0.620),
            ],
            [
                (0.000, 0.025),
                (0.060, 0.045),
                (0.130, 0.075),
                (0.205, 0.110),
                (0.255, 0.185),
                (0.258, 0.570),
                (0.258, 0.600),
            ],
            segments=56,
        ),
        "washer_outer_tub",
    )
    tub.visual(tub_mesh, material=tub_gray, name="tub_shell")
    tub.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=trim_dark,
        name="drive_support",
    )
    tub.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=trim_dark,
        name="drive_shaft",
    )
    tub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.30, length=0.63),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.FIXED,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, -0.015, 0.285)),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.208, tube=0.010, radial_segments=18, tubular_segments=48).translate(
                0.0, 0.0, 0.448
            ),
            "washer_basket_top_rim",
        ),
        material=basket_steel,
        name="basket_top_rim",
    )
    basket.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.186, tube=0.007, radial_segments=16, tubular_segments=44).translate(
                0.0, 0.0, 0.112
            ),
            "washer_basket_lower_rim",
        ),
        material=basket_steel,
        name="basket_lower_rim",
    )
    for index in range(24):
        angle = (2.0 * math.pi * index) / 24.0
        basket.visual(
            Box((0.020, 0.014, 0.336)),
            origin=Origin(
                xyz=(0.190 * math.cos(angle), 0.190 * math.sin(angle), 0.280),
                rpy=(0.0, 0.0, angle),
            ),
            material=basket_steel,
            name=f"basket_slat_{index}",
        )
    basket.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
        material=basket_steel,
        name="basket_hub_mount",
    )
    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        basket.visual(
            Box((0.358, 0.012, 0.010)),
            origin=Origin(
                xyz=(0.0, 0.0, 0.112),
                rpy=(0.0, 0.0, angle),
            ),
            material=basket_steel,
            name=f"basket_spoke_{index}",
        )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.235, length=0.47),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
    )
    model.articulation(
        "tub_to_basket",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=20.0),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.680, 0.600, 0.028)),
        origin=Origin(xyz=(0.0, -0.300, -0.002)),
        material=lid_steel,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.013, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=lid_steel,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.180, 0.020, 0.016)),
        origin=Origin(xyz=(0.0, -0.555, 0.006)),
        material=trim_dark,
        name="lid_handle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.700, 0.600, 0.060)),
        mass=6.2,
        origin=Origin(xyz=(0.0, -0.290, -0.002)),
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.228, 0.956)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(76.0),
        ),
    )

    coin_flap = model.part("coin_flap")
    coin_flap.visual(
        Box((0.092, 0.004, 0.070)),
        origin=Origin(xyz=(0.046, 0.0, 0.035)),
        material=console_dark,
        name="coin_flap_panel",
    )
    coin_flap.visual(
        Cylinder(radius=0.004, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=trim_dark,
        name="coin_hinge_barrel",
    )
    coin_flap.visual(
        Box((0.018, 0.014, 0.020)),
        origin=Origin(xyz=(0.082, -0.009, 0.035)),
        material=trim_dark,
        name="coin_handle",
    )
    coin_flap.inertial = Inertial.from_geometry(
        Box((0.100, 0.020, 0.070)),
        mass=0.25,
        origin=Origin(xyz=(0.050, -0.004, 0.035)),
    )
    model.articulation(
        "cabinet_to_coin_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=coin_flap,
        origin=Origin(xyz=(0.145, 0.246, 0.955)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(95.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    tub = object_model.get_part("tub")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    coin_flap = object_model.get_part("coin_flap")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    basket_spin = object_model.get_articulation("tub_to_basket")
    coin_hinge = object_model.get_articulation("cabinet_to_coin_flap")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_distance(
        basket,
        tub,
        axes="xy",
        max_dist=0.002,
        name="basket stays centered on the tub axis",
    )
    ctx.expect_within(
        basket,
        tub,
        axes="xy",
        inner_elem="basket_top_rim",
        outer_elem="tub_shell",
        margin=0.05,
        name="basket remains inside the tub footprint",
    )
    ctx.expect_gap(
        lid,
        basket,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="basket_top_rim",
        min_gap=0.14,
        name="basket sits low below the closed lid",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="lid_strike",
            min_gap=-0.001,
            max_gap=0.004,
            name="closed lid seats on the cabinet strike rail",
        )
        ctx.expect_overlap(
            lid,
            cabinet,
            axes="x",
            elem_a="lid_panel",
            elem_b="top_front_rail",
            min_overlap=0.50,
            name="lid spans the opening front edge",
        )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    lid_closed_center = _center_from_aabb(ctx.part_element_world_aabb(lid, elem="lid_handle"))
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_open_center = _center_from_aabb(ctx.part_element_world_aabb(lid, elem="lid_handle"))
    ctx.check(
        "lid opens upward from the rear hinge",
        lid_closed_center is not None
        and lid_open_center is not None
        and lid_open_center[2] > lid_closed_center[2] + 0.18
        and lid_open_center[1] > lid_closed_center[1] + 0.18,
        details=f"closed={lid_closed_center}, open={lid_open_center}",
    )

    flap_closed_center = _center_from_aabb(ctx.part_element_world_aabb(coin_flap, elem="coin_handle"))
    with ctx.pose({coin_hinge: coin_hinge.motion_limits.upper}):
        flap_open_center = _center_from_aabb(ctx.part_element_world_aabb(coin_flap, elem="coin_handle"))
    ctx.check(
        "coin flap swings outward from its side hinge",
        flap_closed_center is not None
        and flap_open_center is not None
        and flap_open_center[1] < flap_closed_center[1] - 0.03,
        details=f"closed={flap_closed_center}, open={flap_open_center}",
    )

    ctx.check(
        "lid hinge is a full-width rear revolute joint",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(abs(lid_hinge.axis[0]) - 1.0) < 1e-6
        and abs(lid_hinge.axis[1]) < 1e-6
        and abs(lid_hinge.axis[2]) < 1e-6,
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}",
    )
    ctx.check(
        "basket is configured for vertical continuous spin",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(basket_spin.axis[0]) < 1e-6
        and abs(basket_spin.axis[1]) < 1e-6
        and abs(abs(basket_spin.axis[2]) - 1.0) < 1e-6,
        details=f"type={basket_spin.articulation_type}, axis={basket_spin.axis}",
    )
    ctx.check(
        "coin flap uses a small vertical side hinge",
        coin_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(coin_hinge.axis[0]) < 1e-6
        and abs(coin_hinge.axis[1]) < 1e-6
        and abs(abs(coin_hinge.axis[2]) - 1.0) < 1e-6,
        details=f"type={coin_hinge.articulation_type}, axis={coin_hinge.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
