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
    mesh_from_geometry,
    wire_from_points,
)


OUTER_LENGTH = 1.76
OUTER_DEPTH = 0.92
BODY_UNDERSIDE_Z = 0.16
RIM_TOP_Z = 0.83
INNER_LENGTH = 1.62
INNER_DEPTH = 0.74
LID_LENGTH = 0.84
LID_DEPTH = 0.73
LEFT_LID_CLOSED_X = -0.40
RIGHT_LID_CLOSED_X = 0.40
LID_TRAVEL = 0.40


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_basket_geometry():
    rod_radius = 0.004
    geom = wire_from_points(
        [(-0.76, 0.285, 0.724), (0.76, 0.285, 0.724)],
        radius=rod_radius,
        cap_ends=True,
        corner_mode="miter",
    )
    geom.merge(
        wire_from_points(
            [(-0.76, -0.285, 0.724), (0.76, -0.285, 0.724)],
            radius=rod_radius,
            cap_ends=True,
            corner_mode="miter",
        )
    )
    for y in (-0.25, 0.0, 0.25):
        geom.merge(
            wire_from_points(
                [(-0.72, y, 0.325), (0.72, y, 0.325)],
                radius=rod_radius,
                cap_ends=True,
                corner_mode="miter",
            )
        )
    for x in (-0.72, -0.24, 0.24, 0.72):
        geom.merge(
            wire_from_points(
                [
                    (x, 0.285, 0.724),
                    (x, 0.285, 0.325),
                    (x, -0.285, 0.325),
                    (x, -0.285, 0.724),
                ],
                radius=rod_radius,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.05,
                corner_segments=8,
            )
        )
    geom.merge(
        wire_from_points(
            [(-0.72, 0.285, 0.724), (-0.72, -0.285, 0.724)],
            radius=rod_radius,
            cap_ends=True,
            corner_mode="miter",
        )
    )
    geom.merge(
        wire_from_points(
            [(0.72, 0.285, 0.724), (0.72, -0.285, 0.724)],
            radius=rod_radius,
            cap_ends=True,
            corner_mode="miter",
        )
    )
    for x in (-0.72, 0.72):
        for y0, y1 in ((0.285, 0.325), (-0.285, -0.325)):
            geom.merge(
                wire_from_points(
                    [(x, y0, 0.724), (x, y1, 0.724)],
                    radius=rod_radius,
                    cap_ends=True,
                    corner_mode="miter",
                )
            )
    return geom


def _add_lid_visuals(part, *, handle_sign: float, frame_material, glass_material) -> None:
    part.visual(
        Box((0.80, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.365, 0.005)),
        material=frame_material,
        name="front_runner",
    )
    part.visual(
        Box((0.80, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.365, 0.005)),
        material=frame_material,
        name="rear_runner",
    )
    part.visual(
        Box((0.022, 0.712, 0.018)),
        origin=Origin(xyz=(-0.379, 0.0, 0.009)),
        material=frame_material,
        name="left_frame_bar",
    )
    part.visual(
        Box((0.022, 0.712, 0.018)),
        origin=Origin(xyz=(0.379, 0.0, 0.009)),
        material=frame_material,
        name="right_frame_bar",
    )
    part.visual(
        Box((0.736, 0.664, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=glass_material,
        name="glass_panel",
    )
    part.visual(
        Box((0.17, 0.032, 0.006)),
        origin=Origin(xyz=(handle_sign * 0.18, 0.0, 0.013)),
        material=frame_material,
        name="handle_mount",
    )
    for offset in (-0.04, 0.04):
        part.visual(
            Box((0.012, 0.016, 0.016)),
            origin=Origin(xyz=(handle_sign * 0.18 + offset, 0.0, 0.024)),
            material=frame_material,
            name=f"handle_post_{'inner' if offset < 0.0 else 'outer'}",
        )
    part.visual(
        Box((0.14, 0.012, 0.008)),
        origin=Origin(xyz=(handle_sign * 0.18, 0.0, 0.034)),
        material=frame_material,
        name="handle_grip",
    )


def _add_caster(
    model: ArticulatedObject,
    cabinet,
    *,
    name: str,
    x: float,
    y: float,
    metal_material,
    rubber_material,
):
    fork = model.part(f"{name}_fork")
    fork.visual(
        Box((0.064, 0.064, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=metal_material,
        name="top_plate",
    )
    fork.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=metal_material,
        name="swivel_stem",
    )
    fork.visual(
        Box((0.040, 0.052, 0.010)),
        origin=Origin(xyz=(0.020, 0.0, -0.033)),
        material=metal_material,
        name="yoke_bridge",
    )
    for side_name, side_y in (("left", 0.021), ("right", -0.021)):
        fork.visual(
            Box((0.018, 0.010, 0.096)),
            origin=Origin(xyz=(0.040, side_y, -0.076)),
            material=metal_material,
            name=f"{side_name}_leg",
        )
    for side_name, side_y in (("left", 0.024), ("right", -0.024)):
        fork.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(
                xyz=(0.040, side_y, -0.100),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_material,
            name=f"{side_name}_axle_pin",
        )
    fork.inertial = Inertial.from_geometry(
        Box((0.082, 0.070, 0.126)),
        mass=2.2,
        origin=Origin(xyz=(0.024, 0.0, -0.052)),
    )

    wheel = model.part(f"{name}_wheel")
    wheel.visual(
        Cylinder(radius=0.055, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_material,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.022, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_material,
        name="hub_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.034),
        mass=1.7,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    swivel = model.articulation(
        f"{name}_swivel",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=fork,
        origin=Origin(xyz=(x, y, BODY_UNDERSIDE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    spin = model.articulation(
        f"{name}_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.040, 0.0, -0.100)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=20.0),
    )
    return swivel, spin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="supermarket_island_display_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    liner_white = model.material("liner_white", rgba=(0.97, 0.98, 0.98, 1.0))
    base_grey = model.material("base_grey", rgba=(0.33, 0.35, 0.38, 1.0))
    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.72, 0.84, 0.90, 0.33))
    wire_steel = model.material("wire_steel", rgba=(0.75, 0.77, 0.80, 1.0))
    caster_rubber = model.material("caster_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((1.70, 0.86, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=base_grey,
        name="base_skirt",
    )
    cabinet.visual(
        Box((OUTER_LENGTH, 0.03, 0.56)),
        origin=Origin(xyz=(0.0, 0.445, 0.48)),
        material=cabinet_white,
        name="front_shell",
    )
    cabinet.visual(
        Box((OUTER_LENGTH, 0.03, 0.56)),
        origin=Origin(xyz=(0.0, -0.445, 0.48)),
        material=cabinet_white,
        name="rear_shell",
    )
    cabinet.visual(
        Box((0.03, 0.86, 0.56)),
        origin=Origin(xyz=(0.865, 0.0, 0.48)),
        material=cabinet_white,
        name="right_shell",
    )
    cabinet.visual(
        Box((0.03, 0.86, 0.56)),
        origin=Origin(xyz=(-0.865, 0.0, 0.48)),
        material=cabinet_white,
        name="left_shell",
    )
    cabinet.visual(
        Box((INNER_LENGTH, INNER_DEPTH, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
        material=liner_white,
        name="liner_floor",
    )
    cabinet.visual(
        Box((INNER_LENGTH, 0.02, 0.46)),
        origin=Origin(xyz=(0.0, 0.37, 0.53)),
        material=liner_white,
        name="liner_front_wall",
    )
    cabinet.visual(
        Box((INNER_LENGTH, 0.02, 0.46)),
        origin=Origin(xyz=(0.0, -0.37, 0.53)),
        material=liner_white,
        name="liner_rear_wall",
    )
    cabinet.visual(
        Box((0.02, 0.70, 0.46)),
        origin=Origin(xyz=(0.80, 0.0, 0.53)),
        material=liner_white,
        name="liner_right_wall",
    )
    cabinet.visual(
        Box((0.02, 0.70, 0.46)),
        origin=Origin(xyz=(-0.80, 0.0, 0.53)),
        material=liner_white,
        name="liner_left_wall",
    )
    cabinet.visual(
        Box((1.70, 0.07, 0.07)),
        origin=Origin(xyz=(0.0, 0.425, 0.795)),
        material=anodized_aluminum,
        name="front_rim",
    )
    cabinet.visual(
        Box((1.70, 0.07, 0.07)),
        origin=Origin(xyz=(0.0, -0.425, 0.795)),
        material=anodized_aluminum,
        name="rear_rim",
    )
    cabinet.visual(
        Box((0.09, 0.78, 0.07)),
        origin=Origin(xyz=(0.845, 0.0, 0.795)),
        material=anodized_aluminum,
        name="right_rim",
    )
    cabinet.visual(
        Box((0.09, 0.78, 0.07)),
        origin=Origin(xyz=(-0.845, 0.0, 0.795)),
        material=anodized_aluminum,
        name="left_rim",
    )
    cabinet.visual(
        Box((1.56, 0.07, 0.01)),
        origin=Origin(xyz=(0.0, 0.325, 0.715)),
        material=anodized_aluminum,
        name="front_basket_ledge",
    )
    cabinet.visual(
        Box((1.56, 0.07, 0.01)),
        origin=Origin(xyz=(0.0, -0.325, 0.715)),
        material=anodized_aluminum,
        name="rear_basket_ledge",
    )
    for track_name, y, z in (
        ("upper_front_track", 0.365, 0.804),
        ("upper_rear_track", -0.365, 0.804),
        ("lower_front_track", 0.365, 0.776),
        ("lower_rear_track", -0.365, 0.776),
    ):
        cabinet.visual(
            Box((1.70, 0.020, 0.012)),
            origin=Origin(xyz=(0.0, y, z)),
            material=anodized_aluminum,
            name=track_name,
        )
    for x_sign in (-1.0, 1.0):
        for y_sign, name_prefix in ((1.0, "front"), (-1.0, "rear")):
            cabinet.visual(
                Box((0.03, 0.045, 0.06)),
                origin=Origin(xyz=(x_sign * 0.85, y_sign * 0.3875, 0.79)),
                material=anodized_aluminum,
                name=f"{name_prefix}_{'left' if x_sign < 0.0 else 'right'}_track_bracket",
            )
    caster_pad_positions = {
        "front_left_caster_pad": (-0.70, 0.33),
        "front_right_caster_pad": (0.70, 0.33),
        "rear_left_caster_pad": (-0.70, -0.33),
        "rear_right_caster_pad": (0.70, -0.33),
    }
    for pad_name, (pad_x, pad_y) in caster_pad_positions.items():
        cabinet.visual(
            Box((0.09, 0.09, 0.012)),
            origin=Origin(xyz=(pad_x, pad_y, 0.166)),
            material=base_grey,
            name=pad_name,
        )
    cabinet.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_DEPTH, RIM_TOP_Z)),
        mass=165.0,
        origin=Origin(xyz=(0.0, 0.0, RIM_TOP_Z * 0.5)),
    )

    basket_divider_assembly = model.part("basket_divider_assembly")
    basket_divider_assembly.visual(
        _mesh("basket_divider_assembly", _build_basket_geometry()),
        material=wire_steel,
        name="basket_wires",
    )
    basket_divider_assembly.inertial = Inertial.from_geometry(
        Box((1.56, 0.64, 0.42)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
    )
    model.articulation(
        "cabinet_to_basket_divider_assembly",
        ArticulationType.FIXED,
        parent=cabinet,
        child=basket_divider_assembly,
        origin=Origin(),
    )

    left_lid = model.part("left_lid")
    _add_lid_visuals(
        left_lid,
        handle_sign=-1.0,
        frame_material=anodized_aluminum,
        glass_material=glass_tint,
    )
    left_lid.inertial = Inertial.from_geometry(
        Box((LID_LENGTH, LID_DEPTH, 0.04)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )
    left_lid_joint = model.articulation(
        "cabinet_to_left_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=left_lid,
        origin=Origin(xyz=(LEFT_LID_CLOSED_X, 0.0, 0.810)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=LID_TRAVEL,
        ),
    )

    right_lid = model.part("right_lid")
    _add_lid_visuals(
        right_lid,
        handle_sign=1.0,
        frame_material=anodized_aluminum,
        glass_material=glass_tint,
    )
    right_lid.inertial = Inertial.from_geometry(
        Box((LID_LENGTH, LID_DEPTH, 0.04)),
        mass=13.0,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )
    right_lid_joint = model.articulation(
        "cabinet_to_right_lid",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=right_lid,
        origin=Origin(xyz=(RIGHT_LID_CLOSED_X, 0.0, 0.782)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.25,
            lower=0.0,
            upper=LID_TRAVEL,
        ),
    )

    _add_caster(
        model,
        cabinet,
        name="front_left",
        x=-0.70,
        y=0.33,
        metal_material=anodized_aluminum,
        rubber_material=caster_rubber,
    )
    _add_caster(
        model,
        cabinet,
        name="front_right",
        x=0.70,
        y=0.33,
        metal_material=anodized_aluminum,
        rubber_material=caster_rubber,
    )
    _add_caster(
        model,
        cabinet,
        name="rear_left",
        x=-0.70,
        y=-0.33,
        metal_material=anodized_aluminum,
        rubber_material=caster_rubber,
    )
    _add_caster(
        model,
        cabinet,
        name="rear_right",
        x=0.70,
        y=-0.33,
        metal_material=anodized_aluminum,
        rubber_material=caster_rubber,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    basket_divider_assembly = object_model.get_part("basket_divider_assembly")
    left_lid = object_model.get_part("left_lid")
    right_lid = object_model.get_part("right_lid")
    left_lid_joint = object_model.get_articulation("cabinet_to_left_lid")
    right_lid_joint = object_model.get_articulation("cabinet_to_right_lid")

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

    ctx.expect_contact(
        basket_divider_assembly,
        cabinet,
        name="basket divider assembly is physically supported by the liner ledges",
    )

    with ctx.pose({left_lid_joint: 0.0, right_lid_joint: 0.0}):
        ctx.expect_gap(
            left_lid,
            cabinet,
            axis="z",
            positive_elem="front_runner",
            negative_elem="upper_front_track",
            max_gap=0.001,
            max_penetration=0.0,
            name="left lid front runner sits on the upper rail",
        )
        ctx.expect_gap(
            right_lid,
            cabinet,
            axis="z",
            positive_elem="front_runner",
            negative_elem="lower_front_track",
            max_gap=0.001,
            max_penetration=0.0,
            name="right lid front runner sits on the lower rail",
        )

    left_closed = ctx.part_world_position(left_lid)
    right_closed = ctx.part_world_position(right_lid)
    with ctx.pose({left_lid_joint: LID_TRAVEL, right_lid_joint: right_lid_joint.motion_limits.upper}):
        ctx.expect_overlap(
            left_lid,
            cabinet,
            axes="x",
            elem_a="front_runner",
            elem_b="upper_front_track",
            min_overlap=0.78,
            name="left lid remains captured by the upper track when opened",
        )
        ctx.expect_overlap(
            right_lid,
            cabinet,
            axes="x",
            elem_a="front_runner",
            elem_b="lower_front_track",
            min_overlap=0.78,
            name="right lid remains captured by the lower track when opened",
        )
        left_open = ctx.part_world_position(left_lid)
        right_open = ctx.part_world_position(right_lid)

    ctx.check(
        "left lid slides toward the center of the freezer",
        left_closed is not None
        and left_open is not None
        and left_open[0] > left_closed[0] + 0.30,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right lid slides toward the center of the freezer",
        right_closed is not None
        and right_open is not None
        and right_open[0] < right_closed[0] - 0.30,
        details=f"closed={right_closed}, open={right_open}",
    )

    for caster_name, pad_name in (
        ("front_left", "front_left_caster_pad"),
        ("front_right", "front_right_caster_pad"),
        ("rear_left", "rear_left_caster_pad"),
        ("rear_right", "rear_right_caster_pad"),
    ):
        fork = object_model.get_part(f"{caster_name}_fork")
        wheel = object_model.get_part(f"{caster_name}_wheel")
        swivel = object_model.get_articulation(f"{caster_name}_swivel")
        spin = object_model.get_articulation(f"{caster_name}_wheel_spin")

        ctx.expect_contact(
            fork,
            cabinet,
            elem_a="top_plate",
            elem_b=pad_name,
            name=f"{caster_name} fork is mounted to the cabinet pad",
        )
        ctx.expect_contact(
            wheel,
            fork,
            elem_a="hub",
            elem_b="left_axle_pin",
            name=f"{caster_name} wheel hub is supported on the axle pin",
        )
        ctx.check(
            f"{caster_name} swivel axis is vertical",
            swivel.axis == (0.0, 0.0, 1.0),
            details=f"axis={swivel.axis}",
        )
        ctx.check(
            f"{caster_name} wheel spin axis is lateral",
            spin.axis == (0.0, 1.0, 0.0),
            details=f"axis={spin.axis}",
        )

    front_left_wheel = object_model.get_part("front_left_wheel")
    front_left_swivel = object_model.get_articulation("front_left_swivel")
    front_left_rest = ctx.part_world_position(front_left_wheel)
    with ctx.pose({front_left_swivel: 0.6}):
        front_left_swiveled = ctx.part_world_position(front_left_wheel)
    ctx.check(
        "front left caster fork swivels the wheel around the vertical pivot",
        front_left_rest is not None
        and front_left_swiveled is not None
        and abs(front_left_swiveled[1] - front_left_rest[1]) > 0.01,
        details=f"rest={front_left_rest}, swiveled={front_left_swiveled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
