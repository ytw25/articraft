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
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_width = width * 0.5
    half_depth = depth * 0.5
    return [
        (-half_width, -half_depth),
        (half_width, -half_depth),
        (half_width, half_depth),
        (-half_width, half_depth),
    ]


def _circle_profile(
    center_x: float,
    center_y: float,
    radius: float,
    *,
    segments: int = 28,
) -> list[tuple[float, float]]:
    return [
        (
            center_x + radius * math.cos(2.0 * math.pi * index / segments),
            center_y + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_gas_cooktop", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.78, 0.79, 0.80, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.20, 0.20, 0.21, 1.0))
    burner_iron = model.material("burner_iron", rgba=(0.10, 0.10, 0.10, 1.0))
    counter_stone = model.material("counter_stone", rgba=(0.73, 0.72, 0.70, 1.0))
    control_black = model.material("control_black", rgba=(0.08, 0.08, 0.08, 1.0))
    control_grey = model.material("control_grey", rgba=(0.32, 0.33, 0.34, 1.0))

    cooktop_width = 0.600
    cooktop_depth = 0.510
    cooktop_thickness = 0.004
    counter_width = 0.960
    counter_depth = 0.660
    counter_thickness = 0.040
    counter_top_z = counter_thickness * 0.5
    plate_center_z = counter_top_z - cooktop_thickness * 0.5

    counter = model.part("counter")
    side_strip_width = (counter_width - cooktop_width) * 0.5
    end_strip_depth = (counter_depth - cooktop_depth) * 0.5
    counter.visual(
        Box((side_strip_width, counter_depth, counter_thickness)),
        origin=Origin(xyz=(-(cooktop_width + side_strip_width) * 0.5, 0.0, 0.0)),
        material=counter_stone,
        name="left_counter_strip",
    )
    counter.visual(
        Box((side_strip_width, counter_depth, counter_thickness)),
        origin=Origin(xyz=((cooktop_width + side_strip_width) * 0.5, 0.0, 0.0)),
        material=counter_stone,
        name="right_counter_strip",
    )
    counter.visual(
        Box((cooktop_width, end_strip_depth, counter_thickness)),
        origin=Origin(xyz=(0.0, -((cooktop_depth + end_strip_depth) * 0.5), 0.0)),
        material=counter_stone,
        name="front_counter_strip",
    )
    counter.visual(
        Box((cooktop_width, end_strip_depth, counter_thickness)),
        origin=Origin(xyz=(0.0, ((cooktop_depth + end_strip_depth) * 0.5), 0.0)),
        material=counter_stone,
        name="rear_counter_strip",
    )

    cooktop_plate = model.part("cooktop_plate")
    plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(cooktop_width, cooktop_depth, 0.018, corner_segments=8),
            [
                _circle_profile(-0.275, -0.170, 0.0155),
                _circle_profile(-0.165, -0.170, 0.0155),
            ],
            height=cooktop_thickness,
            center=True,
        ),
        ASSETS.asset_root / "cooktop_plate.obj",
    )
    cooktop_plate.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, plate_center_z)),
        material=stainless,
        name="plate_surface",
    )
    cooktop_plate.visual(
        Box((0.155, 0.135, 0.0006)),
        origin=Origin(xyz=(-0.220, -0.132, counter_top_z - 0.0003)),
        material=control_grey,
        name="control_zone_shadow",
    )

    model.articulation(
        "counter_to_cooktop_plate",
        ArticulationType.FIXED,
        parent=counter,
        child=cooktop_plate,
        origin=Origin(),
    )

    def add_burner(
        name: str,
        *,
        center_x: float,
        center_y: float,
        base_radius: float,
        cap_radius: float,
        grate_span: float,
    ) -> None:
        burner = model.part(name)
        burner.visual(
            Cylinder(radius=base_radius, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=dark_iron,
            name="burner_base",
        )
        burner.visual(
            Cylinder(radius=cap_radius, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=burner_iron,
            name="burner_cap",
        )
        burner.visual(
            Box((grate_span, 0.012, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.018)),
            material=burner_iron,
            name="grate_bar_x",
        )
        burner.visual(
            Box((0.012, grate_span, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.018)),
            material=burner_iron,
            name="grate_bar_y",
        )
        model.articulation(
            f"cooktop_plate_to_{name}",
            ArticulationType.FIXED,
            parent=cooktop_plate,
            child=burner,
            origin=Origin(xyz=(center_x, center_y, counter_top_z)),
        )

    add_burner(
        "rear_left_burner",
        center_x=-0.040,
        center_y=0.115,
        base_radius=0.049,
        cap_radius=0.026,
        grate_span=0.110,
    )
    add_burner(
        "rear_right_burner",
        center_x=0.175,
        center_y=0.115,
        base_radius=0.056,
        cap_radius=0.031,
        grate_span=0.126,
    )
    add_burner(
        "front_left_burner",
        center_x=0.020,
        center_y=-0.095,
        base_radius=0.043,
        cap_radius=0.023,
        grate_span=0.098,
    )
    add_burner(
        "front_right_burner",
        center_x=0.190,
        center_y=-0.095,
        base_radius=0.051,
        cap_radius=0.028,
        grate_span=0.116,
    )

    def add_knob(name: str, joint_name: str, *, center_x: float, center_y: float) -> None:
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.032, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=control_grey,
            name="knob_skirt",
        )
        knob.visual(
            Cylinder(radius=0.028, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=control_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.005, 0.020, 0.004)),
            origin=Origin(xyz=(0.0, 0.015, 0.018)),
            material=stainless,
            name="knob_pointer",
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=cooktop_plate,
            child=knob,
            origin=Origin(xyz=(center_x, center_y, counter_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=10.0),
        )

    add_knob("upper_knob", "cooktop_plate_to_upper_knob", center_x=-0.220, center_y=-0.095)
    add_knob("lower_knob", "cooktop_plate_to_lower_knob", center_x=-0.220, center_y=-0.170)

    def add_button(name: str, joint_name: str, *, center_x: float, center_y: float) -> None:
        button = model.part(name)
        button.visual(
            Cylinder(radius=0.013, length=0.007),
            origin=Origin(xyz=(0.0, 0.0, 0.0035)),
            material=control_black,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=stainless,
            name="button_top_disc",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=cooktop_plate,
            child=button,
            origin=Origin(xyz=(center_x, center_y, counter_top_z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.06,
                lower=0.0,
                upper=0.004,
            ),
        )

    add_button(
        "left_button",
        "cooktop_plate_to_left_button",
        center_x=-0.275,
        center_y=-0.170,
    )
    add_button(
        "right_button",
        "cooktop_plate_to_right_button",
        center_x=-0.165,
        center_y=-0.170,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    counter = object_model.get_part("counter")
    cooktop_plate = object_model.get_part("cooktop_plate")
    rear_left_burner = object_model.get_part("rear_left_burner")
    rear_right_burner = object_model.get_part("rear_right_burner")
    front_left_burner = object_model.get_part("front_left_burner")
    front_right_burner = object_model.get_part("front_right_burner")
    upper_knob = object_model.get_part("upper_knob")
    lower_knob = object_model.get_part("lower_knob")
    left_button = object_model.get_part("left_button")
    right_button = object_model.get_part("right_button")

    upper_knob_joint = object_model.get_articulation("cooktop_plate_to_upper_knob")
    lower_knob_joint = object_model.get_articulation("cooktop_plate_to_lower_knob")
    left_button_joint = object_model.get_articulation("cooktop_plate_to_left_button")
    right_button_joint = object_model.get_articulation("cooktop_plate_to_right_button")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    non_fixed_joints = {
        articulation.name
        for articulation in object_model.articulations
        if articulation.articulation_type != ArticulationType.FIXED
    }
    ctx.check(
        "only_controls_articulate",
        non_fixed_joints
        == {
            "cooktop_plate_to_upper_knob",
            "cooktop_plate_to_lower_knob",
            "cooktop_plate_to_left_button",
            "cooktop_plate_to_right_button",
        },
        f"Unexpected moving joints: {sorted(non_fixed_joints)}",
    )

    plate_aabb = ctx.part_world_aabb(cooktop_plate)
    counter_aabb = ctx.part_world_aabb(counter)
    if plate_aabb is not None:
        plate_width = plate_aabb[1][0] - plate_aabb[0][0]
        plate_depth = plate_aabb[1][1] - plate_aabb[0][1]
        ctx.check(
            "cooktop_realistic_size",
            0.58 <= plate_width <= 0.62 and 0.49 <= plate_depth <= 0.53,
            f"Cooktop plate size was {(plate_width, plate_depth)}",
        )
    if counter_aabb is not None:
        counter_width = counter_aabb[1][0] - counter_aabb[0][0]
        ctx.check(
            "counter_context_wider_than_cooktop",
            counter_width >= 0.90,
            f"Counter width was {counter_width}",
        )

    ctx.expect_contact(cooktop_plate, counter, name="cooktop_plate_contacts_counter")

    for burner in (
        rear_left_burner,
        rear_right_burner,
        front_left_burner,
        front_right_burner,
    ):
        ctx.expect_gap(
            burner,
            cooktop_plate,
            axis="z",
            max_gap=1e-6,
            max_penetration=1e-6,
            negative_elem="plate_surface",
            name=f"{burner.name}_seats_on_plate",
        )

    for knob in (upper_knob, lower_knob):
        ctx.expect_gap(
            knob,
            cooktop_plate,
            axis="z",
            max_gap=1e-6,
            max_penetration=1e-6,
            negative_elem="plate_surface",
            name=f"{knob.name}_rests_on_plate",
        )

    for button in (left_button, right_button):
        ctx.expect_gap(
            button,
            cooktop_plate,
            axis="z",
            max_gap=1e-6,
            max_penetration=1e-6,
            negative_elem="plate_surface",
            name=f"{button.name}_starts_flush_with_plate",
        )

    ctx.expect_origin_distance(
        upper_knob,
        lower_knob,
        axes="x",
        max_dist=0.003,
        name="stacked_knobs_share_same_column",
    )
    ctx.expect_origin_gap(
        upper_knob,
        lower_knob,
        axis="y",
        min_gap=0.06,
        max_gap=0.09,
        name="stacked_knobs_have_vertical_spacing",
    )
    ctx.expect_origin_gap(
        lower_knob,
        left_button,
        axis="x",
        min_gap=0.04,
        max_gap=0.07,
        name="left_button_sits_left_of_lower_knob",
    )
    ctx.expect_origin_gap(
        right_button,
        lower_knob,
        axis="x",
        min_gap=0.04,
        max_gap=0.07,
        name="right_button_sits_right_of_lower_knob",
    )

    for knob_joint in (upper_knob_joint, lower_knob_joint):
        ctx.check(
            f"{knob_joint.name}_is_continuous",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS,
            f"{knob_joint.name} type was {knob_joint.articulation_type}",
        )
        ctx.check(
            f"{knob_joint.name}_axis_is_vertical",
            tuple(knob_joint.axis) == (0.0, 0.0, 1.0),
            f"{knob_joint.name} axis was {knob_joint.axis}",
        )

    for button_joint in (left_button_joint, right_button_joint):
        ctx.check(
            f"{button_joint.name}_is_prismatic",
            button_joint.articulation_type == ArticulationType.PRISMATIC,
            f"{button_joint.name} type was {button_joint.articulation_type}",
        )
        ctx.check(
            f"{button_joint.name}_axis_points_inward",
            tuple(button_joint.axis) == (0.0, 0.0, -1.0),
            f"{button_joint.name} axis was {button_joint.axis}",
        )

    with ctx.pose({upper_knob_joint: 1.7, lower_knob_joint: -2.3}):
        ctx.fail_if_parts_overlap_in_current_pose(name="rotated_knobs_no_overlap")
        ctx.fail_if_isolated_parts(name="rotated_knobs_no_floating")
        ctx.expect_gap(
            upper_knob,
            cooktop_plate,
            axis="z",
            max_gap=1e-6,
            max_penetration=1e-6,
            negative_elem="plate_surface",
            name="upper_knob_stays_seated_when_rotated",
        )
        ctx.expect_gap(
            lower_knob,
            cooktop_plate,
            axis="z",
            max_gap=1e-6,
            max_penetration=1e-6,
            negative_elem="plate_surface",
            name="lower_knob_stays_seated_when_rotated",
        )

    for button, button_joint in (
        (left_button, left_button_joint),
        (right_button, right_button_joint),
    ):
        limits = button_joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({button_joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"{button_joint.name}_upper_no_overlap"
                )
                ctx.fail_if_isolated_parts(name=f"{button_joint.name}_upper_no_floating")
                plate_bounds = ctx.part_element_world_aabb(cooktop_plate, elem="plate_surface")
                button_bounds = ctx.part_world_aabb(button)
                if plate_bounds is not None and button_bounds is not None:
                    ctx.check(
                        f"{button.name}_remains_visible_when_pressed",
                        button_bounds[1][2] > plate_bounds[1][2] + 0.001,
                        (
                            f"{button.name} top z was {button_bounds[1][2]} while plate top z "
                            f"was {plate_bounds[1][2]}"
                        ),
                    )
                    ctx.check(
                        f"{button.name}_does_not_drop_below_plate_bottom",
                        button_bounds[0][2] >= plate_bounds[0][2] - 1e-6,
                        (
                            f"{button.name} bottom z was {button_bounds[0][2]} while plate bottom z "
                            f"was {plate_bounds[0][2]}"
                        ),
                    )

    with ctx.pose(
        {
            upper_knob_joint: 0.8,
            lower_knob_joint: -1.4,
            left_button_joint: 0.004,
            right_button_joint: 0.004,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_controls_operated_no_overlap")
        ctx.fail_if_isolated_parts(name="all_controls_operated_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
