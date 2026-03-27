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
    Inertial,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_rear_axle_module", assets=ASSETS)

    housing_mat = model.material("cast_aluminum", rgba=(0.59, 0.61, 0.64, 1.0))
    steel_mat = model.material("machined_steel", rgba=(0.30, 0.31, 0.34, 1.0))
    flange_mat = model.material("coated_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    rubber_mat = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    def circle_profile(
        radius: float,
        *,
        segments: int = 28,
        center: tuple[float, float] = (0.0, 0.0),
    ) -> list[tuple[float, float]]:
        cx, cy = center
        return [
            (
                cx + radius * math.cos(2.0 * math.pi * i / segments),
                cy + radius * math.sin(2.0 * math.pi * i / segments),
            )
            for i in range(segments)
        ]

    def rect_profile(width: float, height: float) -> list[tuple[float, float]]:
        hw = width / 2.0
        hh = height / 2.0
        return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]

    def x_axis_cylinder(
        part,
        *,
        radius: float,
        length: float,
        x: float,
        y: float,
        z: float,
        material,
        name: str,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    housing = model.part("housing")
    housing.visual(
        Box((0.32, 0.23, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=housing_mat,
        name="lower_body",
    )
    housing.visual(
        Box((0.26, 0.17, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=housing_mat,
        name="upper_motor_case",
    )
    housing.visual(
        Box((0.18, 0.13, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=housing_mat,
        name="bottom_sump",
    )
    housing.visual(
        Box((0.09, 0.18, 0.07)),
        origin=Origin(xyz=(-0.115, 0.0, 0.062)),
        material=housing_mat,
        name="left_web",
    )
    housing.visual(
        Box((0.09, 0.18, 0.07)),
        origin=Origin(xyz=(0.115, 0.0, 0.062)),
        material=housing_mat,
        name="right_web",
    )
    x_axis_cylinder(
        housing,
        radius=0.048,
        length=0.048,
        x=-0.184,
        y=0.0,
        z=0.06,
        material=housing_mat,
        name="left_output_boss",
    )
    x_axis_cylinder(
        housing,
        radius=0.048,
        length=0.048,
        x=0.184,
        y=0.0,
        z=0.06,
        material=housing_mat,
        name="right_output_boss",
    )
    housing.visual(
        Box((0.22, 0.14, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.162)),
        material=housing_mat,
        name="fin_pad",
    )
    fin_x_positions = (-0.085, -0.051, -0.017, 0.017, 0.051, 0.085)
    for index, x_pos in enumerate(fin_x_positions, start=1):
        housing.visual(
            Box((0.018, 0.14, 0.025)),
            origin=Origin(xyz=(x_pos, 0.0, 0.1765)),
            material=housing_mat,
            name=f"fin_{index}",
        )
    housing.inertial = Inertial.from_geometry(
        Box((0.32, 0.23, 0.19)),
        mass=54.0,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    flange_geometry = ExtrudeWithHolesGeometry(
        rect_profile(0.28, 0.10),
        [
            circle_profile(0.012, center=(-0.095, -0.028)),
            circle_profile(0.012, center=(0.095, -0.028)),
            circle_profile(0.012, center=(-0.095, 0.028)),
            circle_profile(0.012, center=(0.095, 0.028)),
        ],
        height=0.012,
        center=True,
    )
    mounting_flange = model.part("mounting_flange")
    mounting_flange.visual(
        mesh_from_geometry(flange_geometry, ASSETS.mesh_path("mounting_flange.obj")),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=housing_mat,
        name="front_mount_plate",
    )
    mounting_flange.inertial = Inertial.from_geometry(
        Box((0.28, 0.012, 0.10)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
    )

    def build_axle_side(part_name: str) -> None:
        axle = model.part(part_name)
        x_axis_cylinder(
            axle,
            radius=0.028,
            length=0.036,
            x=0.018,
            y=0.0,
            z=0.0,
            material=steel_mat,
            name="inboard_stub",
        )
        x_axis_cylinder(
            axle,
            radius=0.015,
            length=0.110,
            x=0.091,
            y=0.0,
            z=0.0,
            material=steel_mat,
            name="half_shaft",
        )
        x_axis_cylinder(
            axle,
            radius=0.022,
            length=0.010,
            x=0.151,
            y=0.0,
            z=0.0,
            material=steel_mat,
            name="boot_inner_collar",
        )
        boot_segment_centers = (0.162, 0.174, 0.186, 0.198)
        boot_segment_radii = (0.027, 0.031, 0.029, 0.025)
        for index, (center_x, radius) in enumerate(
            zip(boot_segment_centers, boot_segment_radii), start=1
        ):
            x_axis_cylinder(
                axle,
                radius=radius,
                length=0.014,
                x=center_x,
                y=0.0,
                z=0.0,
                material=rubber_mat,
                name=f"cv_boot_{index}",
            )
        x_axis_cylinder(
            axle,
            radius=0.020,
            length=0.012,
            x=0.211,
            y=0.0,
            z=0.0,
            material=steel_mat,
            name="boot_outer_collar",
        )
        x_axis_cylinder(
            axle,
            radius=0.038,
            length=0.016,
            x=0.225,
            y=0.0,
            z=0.0,
            material=flange_mat,
            name="wheel_flange",
        )
        x_axis_cylinder(
            axle,
            radius=0.016,
            length=0.012,
            x=0.236,
            y=0.0,
            z=0.0,
            material=steel_mat,
            name="hub_pilot",
        )
        for stud_index in range(6):
            angle = stud_index * math.tau / 6.0
            y_pos = 0.023 * math.cos(angle)
            z_pos = 0.023 * math.sin(angle)
            x_axis_cylinder(
                axle,
                radius=0.0035,
                length=0.014,
                x=0.232,
                y=y_pos,
                z=z_pos,
                material=steel_mat,
                name=f"lug_stud_{stud_index + 1}",
            )
        axle.inertial = Inertial.from_geometry(
            Cylinder(radius=0.03, length=0.25),
            mass=6.5,
            origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        )

    build_axle_side("left_axle")
    build_axle_side("right_axle")

    model.articulation(
        "housing_to_mounting_flange",
        ArticulationType.FIXED,
        parent=housing,
        child=mounting_flange,
        origin=Origin(xyz=(0.0, 0.115, 0.076)),
    )
    model.articulation(
        "housing_to_left_axle",
        ArticulationType.FIXED,
        parent=housing,
        child="left_axle",
        origin=Origin(xyz=(-0.208, 0.0, 0.06), rpy=(0.0, 0.0, math.pi)),
    )
    model.articulation(
        "housing_to_right_axle",
        ArticulationType.FIXED,
        parent=housing,
        child="right_axle",
        origin=Origin(xyz=(0.208, 0.0, 0.06)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    mounting_flange = object_model.get_part("mounting_flange")
    left_axle = object_model.get_part("left_axle")
    right_axle = object_model.get_part("right_axle")

    lower_body = housing.get_visual("lower_body")
    upper_motor_case = housing.get_visual("upper_motor_case")
    fin_pad = housing.get_visual("fin_pad")
    center_fin = housing.get_visual("fin_3")
    left_output_boss = housing.get_visual("left_output_boss")
    right_output_boss = housing.get_visual("right_output_boss")
    front_mount_plate = mounting_flange.get_visual("front_mount_plate")
    left_stub = left_axle.get_visual("inboard_stub")
    right_stub = right_axle.get_visual("inboard_stub")
    left_flange = left_axle.get_visual("wheel_flange")
    right_flange = right_axle.get_visual("wheel_flange")
    left_boot_outer = left_axle.get_visual("boot_outer_collar")
    right_boot_outer = right_axle.get_visual("boot_outer_collar")
    left_hub_pilot = left_axle.get_visual("hub_pilot")
    right_hub_pilot = right_axle.get_visual("hub_pilot")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        housing,
        housing,
        axes="xy",
        min_overlap=0.12,
        elem_a=fin_pad,
        elem_b=upper_motor_case,
        name="fin_pad_spans_motor_case",
    )
    ctx.expect_gap(
        housing,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=fin_pad,
        negative_elem=upper_motor_case,
        name="fin_pad_seated_on_motor_case",
    )
    ctx.expect_gap(
        housing,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem=center_fin,
        negative_elem=fin_pad,
        name="center_fin_cast_into_pad",
    )
    ctx.expect_within(
        housing,
        housing,
        axes="xy",
        inner_elem=fin_pad,
        outer_elem=upper_motor_case,
        name="cooling_fins_stay_on_top_face",
    )

    ctx.expect_overlap(mounting_flange, housing, axes="xz", min_overlap=0.018)
    ctx.expect_gap(
        mounting_flange,
        housing,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.003,
        positive_elem=front_mount_plate,
        negative_elem=lower_body,
    )

    ctx.expect_origin_distance(left_axle, right_axle, axes="yz", max_dist=0.002)
    ctx.expect_gap(
        housing,
        left_axle,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.003,
        positive_elem=left_output_boss,
        negative_elem=left_stub,
    )
    ctx.expect_gap(
        right_axle,
        housing,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.003,
        positive_elem=right_stub,
        negative_elem=right_output_boss,
    )
    ctx.expect_gap(
        housing,
        left_axle,
        axis="x",
        min_gap=0.21,
        max_gap=0.225,
        positive_elem=left_output_boss,
        negative_elem=left_flange,
        name="left_halfshaft_reach",
    )
    ctx.expect_gap(
        right_axle,
        housing,
        axis="x",
        min_gap=0.21,
        max_gap=0.225,
        positive_elem=right_flange,
        negative_elem=right_output_boss,
        name="right_halfshaft_reach",
    )
    ctx.expect_overlap(left_axle, right_axle, axes="yz", min_overlap=0.06)
    ctx.expect_gap(
        left_axle,
        left_axle,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_boot_outer,
        negative_elem=left_flange,
        name="left_boot_terminates_at_wheel_flange",
    )
    ctx.expect_gap(
        right_axle,
        right_axle,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_flange,
        negative_elem=right_boot_outer,
        name="right_boot_terminates_at_wheel_flange",
    )
    ctx.expect_within(
        left_axle,
        left_axle,
        axes="yz",
        inner_elem=left_hub_pilot,
        outer_elem=left_flange,
        name="left_hub_pilot_centered_in_flange",
    )
    ctx.expect_within(
        right_axle,
        right_axle,
        axes="yz",
        inner_elem=right_hub_pilot,
        outer_elem=right_flange,
        name="right_hub_pilot_centered_in_flange",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
