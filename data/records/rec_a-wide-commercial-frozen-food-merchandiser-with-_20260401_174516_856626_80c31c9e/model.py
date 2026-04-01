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


def _add_basket(
    model: ArticulatedObject,
    *,
    name: str,
    x_center: float,
    z_center: float,
    material,
) -> None:
    basket = model.part(name)
    basket_width = 0.50
    basket_depth = 0.54
    basket_height = 0.22
    hanger_span = basket_width + 0.05
    hanger_y = 0.310
    upper_side_depth = 0.620

    basket.visual(
        Box((hanger_span, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, hanger_y, 0.115)),
        material=material,
        name="front_hanger",
    )
    basket.visual(
        Box((hanger_span, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -hanger_y, 0.115)),
        material=material,
        name="rear_hanger",
    )
    basket.visual(
        Box((0.014, upper_side_depth, 0.010)),
        origin=Origin(xyz=(-basket_width / 2.0, 0.0, 0.105)),
        material=material,
        name="left_top_rail",
    )
    basket.visual(
        Box((0.014, upper_side_depth, 0.010)),
        origin=Origin(xyz=(basket_width / 2.0, 0.0, 0.105)),
        material=material,
        name="right_top_rail",
    )

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            basket.visual(
                Box((0.010, 0.010, basket_height - 0.010)),
                origin=Origin(
                    xyz=(
                        x_sign * basket_width / 2.0,
                        y_sign * 0.250,
                        0.0,
                    )
                ),
                material=material,
                name=f"corner_post_{'l' if x_sign < 0.0 else 'r'}_{'r' if y_sign < 0.0 else 'f'}",
            )

    basket.visual(
        Box((basket_width + 0.004, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.250, -0.105)),
        material=material,
        name="front_bottom_rail",
    )
    basket.visual(
        Box((basket_width + 0.004, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.250, -0.105)),
        material=material,
        name="rear_bottom_rail",
    )
    basket.visual(
        Box((0.010, basket_depth - 0.040, 0.010)),
        origin=Origin(xyz=(-basket_width / 2.0, 0.0, -0.105)),
        material=material,
        name="left_bottom_rail",
    )
    basket.visual(
        Box((0.010, basket_depth - 0.040, 0.010)),
        origin=Origin(xyz=(basket_width / 2.0, 0.0, -0.105)),
        material=material,
        name="right_bottom_rail",
    )
    basket.visual(
        Box((basket_width - 0.040, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.100, -0.105)),
        material=material,
        name="center_floor_front",
    )
    basket.visual(
        Box((basket_width - 0.040, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -0.100, -0.105)),
        material=material,
        name="center_floor_rear",
    )
    basket.visual(
        Box((0.008, basket_depth - 0.040, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.105)),
        material=material,
        name="center_floor_longitudinal",
    )
    basket.inertial = Inertial.from_geometry(
        Box((hanger_span, 0.64, 0.23)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        f"cabinet_to_{name}",
        ArticulationType.FIXED,
        parent="cabinet",
        child=basket,
        origin=Origin(xyz=(x_center, 0.0, z_center)),
    )


def _add_lid(
    model: ArticulatedObject,
    *,
    name: str,
    joint_name: str,
    x_center: float,
    track_z: float,
    axis: tuple[float, float, float],
    upper: float,
    lane_index: int,
    material_frame,
    material_glass,
    handle_x: float,
) -> None:
    lid = model.part(name)
    lid_length = 0.70
    lid_width = 0.68

    lid.visual(
        Box((lid_length, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.332, 0.003)),
        material=material_frame,
        name="front_runner",
    )
    lid.visual(
        Box((lid_length, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.332, 0.003)),
        material=material_frame,
        name="rear_runner",
    )
    lid.visual(
        Box((0.018, lid_width - 0.012, 0.006)),
        origin=Origin(xyz=(-0.341, 0.0, 0.003)),
        material=material_frame,
        name="left_end_frame",
    )
    lid.visual(
        Box((0.018, lid_width - 0.012, 0.006)),
        origin=Origin(xyz=(0.341, 0.0, 0.003)),
        material=material_frame,
        name="right_end_frame",
    )
    lid.visual(
        Box((lid_length - 0.030, lid_width - 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=material_glass,
        name="glass_panel",
    )
    lid.visual(
        Box((0.120, 0.028, 0.006)),
        origin=Origin(xyz=(handle_x, 0.0, 0.006)),
        material=material_frame,
        name="pull_strip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, 0.014)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent="cabinet",
        child=lid,
        origin=Origin(xyz=(x_center, 0.0, track_z)),
        axis=axis,
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.30,
            lower=0.0,
            upper=upper,
        ),
        meta={"lane": lane_index},
    )


def _add_caster(
    model: ArticulatedObject,
    *,
    suffix: str,
    x: float,
    y: float,
    steel,
    rubber,
) -> None:
    caster = model.part(f"caster_{suffix}")
    caster.visual(
        Box((0.070, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=steel,
        name="mount_plate",
    )
    caster.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=steel,
        name="swivel_stem",
    )
    caster.visual(
        Box((0.046, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=steel,
        name="fork_crown",
    )
    caster.visual(
        Box((0.010, 0.024, 0.050)),
        origin=Origin(xyz=(-0.018, 0.0, -0.061)),
        material=steel,
        name="left_fork_arm",
    )
    caster.visual(
        Box((0.010, 0.024, 0.050)),
        origin=Origin(xyz=(0.018, 0.0, -0.061)),
        material=steel,
        name="right_fork_arm",
    )
    caster.inertial = Inertial.from_geometry(
        Box((0.070, 0.060, 0.100)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
    )

    wheel = model.part(f"wheel_{suffix}")
    wheel.visual(
        Cylinder(radius=0.033, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hub",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=0.026),
        mass=0.55,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        f"cabinet_to_caster_{suffix}",
        ArticulationType.CONTINUOUS,
        parent="cabinet",
        child=caster,
        origin=Origin(xyz=(x, y, 0.100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )
    model.articulation(
        f"caster_{suffix}_to_wheel_{suffix}",
        ArticulationType.CONTINUOUS,
        parent=caster,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.076)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="frozen_food_merchandiser")

    exterior = model.material("exterior_charcoal", rgba=(0.22, 0.25, 0.27, 1.0))
    liner = model.material("liner_white", rgba=(0.94, 0.95, 0.97, 1.0))
    aluminum = model.material("aluminum_trim", rgba=(0.73, 0.75, 0.78, 1.0))
    glass = model.material("glass_tint", rgba=(0.74, 0.84, 0.90, 0.34))
    basket_wire = model.material("basket_wire", rgba=(0.92, 0.93, 0.95, 1.0))
    bumper = model.material("bumper_black", rgba=(0.10, 0.10, 0.11, 1.0))
    caster_steel = model.material("caster_steel", rgba=(0.52, 0.54, 0.57, 1.0))
    caster_rubber = model.material("caster_rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((1.840, 0.920, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=exterior,
        name="base_plinth",
    )
    cabinet.visual(
        Box((1.800, 0.030, 0.640)),
        origin=Origin(xyz=(0.0, 0.455, 0.500)),
        material=exterior,
        name="front_outer_wall",
    )
    cabinet.visual(
        Box((1.800, 0.030, 0.640)),
        origin=Origin(xyz=(0.0, -0.455, 0.500)),
        material=exterior,
        name="rear_outer_wall",
    )
    cabinet.visual(
        Box((0.030, 0.880, 0.640)),
        origin=Origin(xyz=(-0.915, 0.0, 0.500)),
        material=exterior,
        name="left_outer_wall",
    )
    cabinet.visual(
        Box((0.030, 0.880, 0.640)),
        origin=Origin(xyz=(0.915, 0.0, 0.500)),
        material=exterior,
        name="right_outer_wall",
    )
    cabinet.visual(
        Box((1.560, 0.018, 0.140)),
        origin=Origin(xyz=(0.0, 0.461, 0.250)),
        material=bumper,
        name="front_bumper_strip",
    )
    cabinet.visual(
        Box((1.640, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.452, 0.320)),
        material=aluminum,
        name="front_badge_rail",
    )

    cabinet.visual(
        Box((1.840, 0.090, 0.050)),
        origin=Origin(xyz=(0.0, 0.425, 0.845)),
        material=aluminum,
        name="front_top_rim",
    )
    cabinet.visual(
        Box((1.840, 0.090, 0.050)),
        origin=Origin(xyz=(0.0, -0.425, 0.845)),
        material=aluminum,
        name="rear_top_rim",
    )
    cabinet.visual(
        Box((0.090, 0.920, 0.050)),
        origin=Origin(xyz=(-0.885, 0.0, 0.845)),
        material=aluminum,
        name="left_top_rim",
    )
    cabinet.visual(
        Box((0.090, 0.920, 0.050)),
        origin=Origin(xyz=(0.885, 0.0, 0.845)),
        material=aluminum,
        name="right_top_rim",
    )
    cabinet.visual(
        Box((1.680, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.365, 0.845)),
        material=aluminum,
        name="front_track_bridge",
    )
    cabinet.visual(
        Box((1.680, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, -0.365, 0.845)),
        material=aluminum,
        name="rear_track_bridge",
    )
    cabinet.visual(
        Box((1.600, 0.005, 0.022)),
        origin=Origin(xyz=(0.0, 0.3475, 0.827)),
        material=aluminum,
        name="front_track_riser_stack",
    )
    cabinet.visual(
        Box((1.600, 0.005, 0.022)),
        origin=Origin(xyz=(0.0, -0.3475, 0.827)),
        material=aluminum,
        name="rear_track_riser_stack",
    )

    cabinet.visual(
        Box((1.680, 0.700, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.183)),
        material=liner,
        name="liner_floor",
    )
    cabinet.visual(
        Box((1.680, 0.005, 0.620)),
        origin=Origin(xyz=(0.0, 0.3475, 0.490)),
        material=liner,
        name="front_liner_wall",
    )
    cabinet.visual(
        Box((1.680, 0.005, 0.620)),
        origin=Origin(xyz=(0.0, -0.3475, 0.490)),
        material=liner,
        name="rear_liner_wall",
    )
    cabinet.visual(
        Box((0.005, 0.700, 0.620)),
        origin=Origin(xyz=(-0.8425, 0.0, 0.490)),
        material=liner,
        name="left_liner_wall",
    )
    cabinet.visual(
        Box((0.005, 0.700, 0.620)),
        origin=Origin(xyz=(0.8425, 0.0, 0.490)),
        material=liner,
        name="right_liner_wall",
    )

    cabinet.visual(
        Box((1.600, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.340, 0.803)),
        material=aluminum,
        name="front_track_lane_0",
    )
    cabinet.visual(
        Box((1.600, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.340, 0.819)),
        material=aluminum,
        name="front_track_lane_1",
    )
    cabinet.visual(
        Box((1.600, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.340, 0.835)),
        material=aluminum,
        name="front_track_lane_2",
    )
    cabinet.visual(
        Box((1.600, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.340, 0.803)),
        material=aluminum,
        name="rear_track_lane_0",
    )
    cabinet.visual(
        Box((1.600, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.340, 0.819)),
        material=aluminum,
        name="rear_track_lane_1",
    )
    cabinet.visual(
        Box((1.600, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.340, 0.835)),
        material=aluminum,
        name="rear_track_lane_2",
    )

    cabinet.visual(
        Box((1.600, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, 0.331, 0.748)),
        material=aluminum,
        name="front_basket_support",
    )
    cabinet.visual(
        Box((1.600, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, -0.331, 0.748)),
        material=aluminum,
        name="rear_basket_support",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((1.860, 0.940, 0.880)),
        mass=165.0,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
    )

    _add_basket(
        model,
        name="basket_left",
        x_center=-0.560,
        z_center=0.644,
        material=basket_wire,
    )
    _add_basket(
        model,
        name="basket_center",
        x_center=0.000,
        z_center=0.644,
        material=basket_wire,
    )
    _add_basket(
        model,
        name="basket_right",
        x_center=0.560,
        z_center=0.644,
        material=basket_wire,
    )

    _add_lid(
        model,
        name="lid_left",
        joint_name="cabinet_to_lid_left",
        x_center=-0.490,
        track_z=0.806,
        axis=(1.0, 0.0, 0.0),
        upper=0.340,
        lane_index=0,
        material_frame=aluminum,
        material_glass=glass,
        handle_x=-0.210,
    )
    _add_lid(
        model,
        name="lid_center",
        joint_name="cabinet_to_lid_center",
        x_center=0.000,
        track_z=0.822,
        axis=(1.0, 0.0, 0.0),
        upper=0.340,
        lane_index=1,
        material_frame=aluminum,
        material_glass=glass,
        handle_x=0.000,
    )
    _add_lid(
        model,
        name="lid_right",
        joint_name="cabinet_to_lid_right",
        x_center=0.490,
        track_z=0.838,
        axis=(-1.0, 0.0, 0.0),
        upper=0.340,
        lane_index=2,
        material_frame=aluminum,
        material_glass=glass,
        handle_x=0.210,
    )

    _add_caster(
        model,
        suffix="front_left",
        x=-0.820,
        y=0.390,
        steel=caster_steel,
        rubber=caster_rubber,
    )
    _add_caster(
        model,
        suffix="front_right",
        x=0.820,
        y=0.390,
        steel=caster_steel,
        rubber=caster_rubber,
    )
    _add_caster(
        model,
        suffix="rear_left",
        x=-0.820,
        y=-0.390,
        steel=caster_steel,
        rubber=caster_rubber,
    )
    _add_caster(
        model,
        suffix="rear_right",
        x=0.820,
        y=-0.390,
        steel=caster_steel,
        rubber=caster_rubber,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    basket_left = object_model.get_part("basket_left")
    basket_center = object_model.get_part("basket_center")
    basket_right = object_model.get_part("basket_right")
    lid_left = object_model.get_part("lid_left")
    lid_center = object_model.get_part("lid_center")
    lid_right = object_model.get_part("lid_right")
    caster_front_left = object_model.get_part("caster_front_left")
    wheel_front_left = object_model.get_part("wheel_front_left")

    lid_left_joint = object_model.get_articulation("cabinet_to_lid_left")
    lid_center_joint = object_model.get_articulation("cabinet_to_lid_center")
    lid_right_joint = object_model.get_articulation("cabinet_to_lid_right")
    caster_front_left_joint = object_model.get_articulation("cabinet_to_caster_front_left")
    wheel_front_left_joint = object_model.get_articulation("caster_front_left_to_wheel_front_left")

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

    for required_part in (
        cabinet,
        basket_left,
        basket_center,
        basket_right,
        lid_left,
        lid_center,
        lid_right,
        caster_front_left,
        wheel_front_left,
    ):
        ctx.check(
            f"part present: {required_part.name}",
            required_part is not None,
            details=f"missing {required_part.name}",
        )

    for basket in (basket_left, basket_center, basket_right):
        ctx.expect_contact(
            basket,
            cabinet,
            elem_a="front_hanger",
            elem_b="front_basket_support",
            name=f"{basket.name} front hanger sits on support rail",
        )
        ctx.expect_contact(
            basket,
            cabinet,
            elem_a="rear_hanger",
            elem_b="rear_basket_support",
            name=f"{basket.name} rear hanger sits on support rail",
        )

    ctx.expect_contact(
        lid_left,
        cabinet,
        elem_a="front_runner",
        elem_b="front_track_lane_0",
        name="left lid front runner sits on lower track",
    )
    ctx.expect_contact(
        lid_left,
        cabinet,
        elem_a="rear_runner",
        elem_b="rear_track_lane_0",
        name="left lid rear runner sits on lower track",
    )
    ctx.expect_contact(
        lid_center,
        cabinet,
        elem_a="front_runner",
        elem_b="front_track_lane_1",
        name="center lid front runner sits on middle track",
    )
    ctx.expect_contact(
        lid_center,
        cabinet,
        elem_a="rear_runner",
        elem_b="rear_track_lane_1",
        name="center lid rear runner sits on middle track",
    )
    ctx.expect_contact(
        lid_right,
        cabinet,
        elem_a="front_runner",
        elem_b="front_track_lane_2",
        name="right lid front runner sits on upper track",
    )
    ctx.expect_contact(
        lid_right,
        cabinet,
        elem_a="rear_runner",
        elem_b="rear_track_lane_2",
        name="right lid rear runner sits on upper track",
    )

    ctx.expect_contact(
        caster_front_left,
        cabinet,
        elem_a="mount_plate",
        elem_b="base_plinth",
        name="front left caster mount plate contacts cabinet base",
    )
    ctx.expect_contact(
        wheel_front_left,
        caster_front_left,
        elem_a="hub",
        elem_b="left_fork_arm",
        name="front left wheel is carried by the caster fork",
    )

    ctx.check(
        "lid joints are prismatic",
        lid_left_joint.articulation_type == ArticulationType.PRISMATIC
        and lid_center_joint.articulation_type == ArticulationType.PRISMATIC
        and lid_right_joint.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"left={lid_left_joint.articulation_type}, "
            f"center={lid_center_joint.articulation_type}, "
            f"right={lid_right_joint.articulation_type}"
        ),
    )
    ctx.check(
        "lid slide axes follow cabinet length",
        tuple(lid_left_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(lid_center_joint.axis) == (1.0, 0.0, 0.0)
        and tuple(lid_right_joint.axis) == (-1.0, 0.0, 0.0),
        details=(
            f"left={lid_left_joint.axis}, center={lid_center_joint.axis}, right={lid_right_joint.axis}"
        ),
    )
    ctx.check(
        "caster and wheel joints are continuous with vertical swivel and horizontal axle",
        caster_front_left_joint.articulation_type == ArticulationType.CONTINUOUS
        and wheel_front_left_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(caster_front_left_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(wheel_front_left_joint.axis) == (1.0, 0.0, 0.0),
        details=(
            f"caster={caster_front_left_joint.articulation_type}/{caster_front_left_joint.axis}, "
            f"wheel={wheel_front_left_joint.articulation_type}/{wheel_front_left_joint.axis}"
        ),
    )

    left_rest = ctx.part_world_position(lid_left)
    center_rest = ctx.part_world_position(lid_center)
    right_rest = ctx.part_world_position(lid_right)
    with ctx.pose({lid_left_joint: lid_left_joint.motion_limits.upper}):
        left_open = ctx.part_world_position(lid_left)
        ctx.expect_contact(
            lid_left,
            cabinet,
            elem_a="front_runner",
            elem_b="front_track_lane_0",
            name="left lid remains supported when opened",
        )
    with ctx.pose({lid_center_joint: lid_center_joint.motion_limits.upper}):
        center_open = ctx.part_world_position(lid_center)
        ctx.expect_contact(
            lid_center,
            cabinet,
            elem_a="front_runner",
            elem_b="front_track_lane_1",
            name="center lid remains supported when opened",
        )
    with ctx.pose({lid_right_joint: lid_right_joint.motion_limits.upper}):
        right_open = ctx.part_world_position(lid_right)
        ctx.expect_contact(
            lid_right,
            cabinet,
            elem_a="front_runner",
            elem_b="front_track_lane_2",
            name="right lid remains supported when opened",
        )

    ctx.check(
        "left and center lids slide right while right lid slides left",
        left_rest is not None
        and left_open is not None
        and center_rest is not None
        and center_open is not None
        and right_rest is not None
        and right_open is not None
        and left_open[0] > left_rest[0] + 0.20
        and center_open[0] > center_rest[0] + 0.20
        and right_open[0] < right_rest[0] - 0.20,
        details=(
            f"left rest/open={left_rest}/{left_open}, "
            f"center rest/open={center_rest}/{center_open}, "
            f"right rest/open={right_rest}/{right_open}"
        ),
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

    caster_rest = _aabb_center(
        ctx.part_element_world_aabb(caster_front_left, elem="left_fork_arm")
    )
    with ctx.pose({caster_front_left_joint: 1.2}):
        caster_swiveled = _aabb_center(
            ctx.part_element_world_aabb(caster_front_left, elem="left_fork_arm")
        )
        ctx.expect_contact(
            wheel_front_left,
            caster_front_left,
            elem_a="hub",
            elem_b="left_fork_arm",
            name="front left wheel stays carried after swivel",
        )
    ctx.check(
        "front left caster swivels around a vertical pivot",
        caster_rest is not None
        and caster_swiveled is not None
        and math.hypot(
            caster_rest[0] - caster_swiveled[0],
            caster_rest[1] - caster_swiveled[1],
        )
        > 0.015
        and abs(caster_rest[2] - caster_swiveled[2]) < 0.002,
        details=f"rest={caster_rest}, swiveled={caster_swiveled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
