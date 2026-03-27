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
    repair_loft,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

BASE_W = 0.70
BASE_D = 0.50
BASE_H = 0.08

COLUMN_R = 0.040
COLUMN_H = 0.96

ARM_CARRIER_Z0 = 0.52
ARM_CARRIER_H = 0.20

GUIDE_LEN = 0.46
GUIDE_TOP_W = 0.080
GUIDE_BOTTOM_W = 0.048
GUIDE_H = 0.028

TABLE_SUPPORT_Z = 0.38
TABLE_PIVOT_Y = -0.28
TABLE_PIVOT_Z = -0.11


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_profile(radius: float, segments: int = 28):
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _ring_plate_mesh(outer_radius: float, inner_radius: float, thickness: float):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        height=thickness,
        center=False,
        closed=True,
    )


def _dovetail_guide_mesh(length: float, top_width: float, bottom_width: float, height: float):
    x0 = -length / 2.0
    x1 = length / 2.0
    sections = [
        [
            (x0, -top_width / 2.0, 0.0),
            (x0, top_width / 2.0, 0.0),
            (x0, bottom_width / 2.0, -height),
            (x0, -bottom_width / 2.0, -height),
        ],
        [
            (x1, -top_width / 2.0, 0.0),
            (x1, top_width / 2.0, 0.0),
            (x1, bottom_width / 2.0, -height),
            (x1, -bottom_width / 2.0, -height),
        ],
    ]
    return repair_loft(section_loft(sections))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_arm_drill_press", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.28, 0.29, 0.31, 1.0))
    machine_green = model.material("machine_green", rgba=(0.34, 0.43, 0.37, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.65, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    table_gray = model.material("table_gray", rgba=(0.38, 0.40, 0.42, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_W, BASE_D, BASE_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H / 2.0)),
        material=cast_iron,
        name="base_plinth",
    )
    base.visual(
        Box((0.22, 0.20, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, BASE_H + 0.01)),
        material=cast_iron,
        name="column_pad",
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=0.065, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="column_collar",
    )
    column.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_H / 2.0)),
        material=steel,
        name="column_shaft",
    )

    arm_carrier = model.part("arm_carrier")
    arm_carrier.visual(
        Box((0.036, 0.150, ARM_CARRIER_H)),
        origin=Origin(xyz=(-0.068, 0.0, ARM_CARRIER_H / 2.0)),
        material=machine_green,
        name="sleeve_left",
    )
    arm_carrier.visual(
        Box((0.036, 0.150, ARM_CARRIER_H)),
        origin=Origin(xyz=(0.068, 0.0, ARM_CARRIER_H / 2.0)),
        material=machine_green,
        name="sleeve_right",
    )
    arm_carrier.visual(
        Box((0.106, 0.026, ARM_CARRIER_H)),
        origin=Origin(xyz=(0.0, -0.062, ARM_CARRIER_H / 2.0)),
        material=machine_green,
        name="sleeve_back",
    )
    arm_carrier.visual(
        Box((0.106, 0.026, ARM_CARRIER_H)),
        origin=Origin(xyz=(0.0, 0.062, ARM_CARRIER_H / 2.0)),
        material=machine_green,
        name="sleeve_front",
    )
    arm_carrier.visual(
        _save_mesh(
            "arm_carrier_turntable_plate.obj",
            _ring_plate_mesh(outer_radius=0.084, inner_radius=0.051, thickness=0.010),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=machined_steel,
        name="turntable_plate",
    )
    arm_carrier.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(
            xyz=(0.098, 0.0, 0.115),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined_steel,
        name="lift_handwheel",
    )

    radial_arm = model.part("radial_arm")
    radial_arm.visual(
        _save_mesh(
            "radial_arm_mount_plate.obj",
            _ring_plate_mesh(outer_radius=0.084, inner_radius=0.051, thickness=0.010),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=machined_steel,
        name="arm_mount_plate",
    )
    radial_arm.visual(
        Box((0.110, 0.130, 0.100)),
        origin=Origin(xyz=(0.105, 0.0, 0.060)),
        material=machine_green,
        name="arm_root",
    )
    radial_arm.visual(
        Box((0.600, 0.120, 0.070)),
        origin=Origin(xyz=(0.360, 0.0, 0.055)),
        material=machine_green,
        name="arm_beam",
    )
    radial_arm.visual(
        Box((0.460, 0.060, 0.020)),
        origin=Origin(xyz=(0.390, 0.0, 0.087)),
        material=machine_green,
        name="arm_top_rib",
    )
    radial_arm.visual(
        _save_mesh(
            "radial_arm_dovetail_guide.obj",
            _dovetail_guide_mesh(GUIDE_LEN, GUIDE_TOP_W, GUIDE_BOTTOM_W, GUIDE_H),
        ),
        origin=Origin(xyz=(0.390, 0.0, 0.020)),
        material=machined_steel,
        name="dovetail_guide",
    )
    radial_arm.visual(
        Box((0.040, 0.100, 0.050)),
        origin=Origin(xyz=(0.650, 0.0, 0.055)),
        material=machine_green,
        name="arm_nose",
    )

    drill_head = model.part("drill_head")
    drill_head.visual(
        Box((0.112, 0.080, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=machined_steel,
        name="guide_pad",
    )
    drill_head.visual(
        Box((0.100, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, -0.034, -0.031)),
        material=machined_steel,
        name="carriage_jaw_left",
    )
    drill_head.visual(
        Box((0.100, 0.024, 0.050)),
        origin=Origin(xyz=(0.0, 0.034, -0.031)),
        material=machined_steel,
        name="carriage_jaw_right",
    )
    drill_head.visual(
        Box((0.140, 0.160, 0.170)),
        origin=Origin(xyz=(0.0, 0.0, -0.115)),
        material=cast_iron,
        name="head_casting",
    )
    drill_head.visual(
        Cylinder(radius=0.042, length=0.110),
        origin=Origin(
            xyz=(0.0, 0.0, -0.100),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=machine_green,
        name="motor_housing",
    )
    drill_head.visual(
        Box((0.070, 0.090, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
        material=cast_iron,
        name="spindle_box",
    )
    drill_head.visual(
        Cylinder(radius=0.022, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, -0.230)),
        material=steel,
        name="spindle_quill",
    )
    drill_head.visual(
        Cylinder(radius=0.014, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.290)),
        material=machined_steel,
        name="chuck",
    )

    table_support = model.part("table_support")
    table_support.visual(
        _save_mesh(
            "table_support_clamp.obj",
            _ring_plate_mesh(outer_radius=0.072, inner_radius=0.043, thickness=0.070),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cast_iron,
        name="support_clamp",
    )
    table_support.visual(
        Box((0.070, 0.220, 0.026)),
        origin=Origin(xyz=(0.0, -0.135, 0.020)),
        material=cast_iron,
        name="support_arm",
    )
    table_support.visual(
        Box((0.012, 0.024, 0.120)),
        origin=Origin(xyz=(0.0, -0.280, -0.050)),
        material=cast_iron,
        name="support_drop",
    )
    table_support.visual(
        Box((0.020, 0.080, 0.160)),
        origin=Origin(xyz=(0.0, -0.255, -0.050)),
        material=cast_iron,
        name="support_knuckle",
    )
    table_support.visual(
        Box((0.010, 0.040, 0.022)),
        origin=Origin(xyz=(0.0, TABLE_PIVOT_Y, TABLE_PIVOT_Z)),
        material=machined_steel,
        name="table_yoke",
    )

    work_table = model.part("work_table")
    work_table.visual(
        Cylinder(radius=0.160, length=0.022),
        origin=Origin(xyz=(0.0, -0.255, 0.090)),
        material=table_gray,
        name="table_disk",
    )
    work_table.visual(
        Box((0.096, 0.240, 0.018)),
        origin=Origin(xyz=(0.0, -0.110, 0.048)),
        material=cast_iron,
        name="table_web",
    )
    work_table.visual(
        Box((0.055, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, -0.095, 0.022)),
        material=cast_iron,
        name="table_hub",
    )
    work_table.visual(
        Box((0.080, 0.240, 0.090)),
        origin=Origin(xyz=(0.0, -0.120, 0.045)),
        material=cast_iron,
        name="table_trunnion_block",
    )
    work_table.visual(
        Box((0.014, 0.022, 0.060)),
        origin=Origin(xyz=(-0.028, 0.0, 0.0)),
        material=cast_iron,
        name="left_ear",
    )
    work_table.visual(
        Box((0.014, 0.022, 0.060)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=cast_iron,
        name="right_ear",
    )

    table_pin = model.part("table_pin")
    table_pin.visual(
        Cylinder(radius=0.006, length=0.066),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="tilt_pin",
    )

    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, BASE_H)),
    )
    model.articulation(
        "arm_lift",
        ArticulationType.PRISMATIC,
        parent=column,
        child=arm_carrier,
        origin=Origin(xyz=(0.0, 0.0, ARM_CARRIER_Z0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.28,
            lower=-0.06,
            upper=0.18,
        ),
    )
    model.articulation(
        "arm_azimuth",
        ArticulationType.CONTINUOUS,
        parent=arm_carrier,
        child=radial_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.5),
    )
    model.articulation(
        "head_slide",
        ArticulationType.PRISMATIC,
        parent=radial_arm,
        child=drill_head,
        origin=Origin(xyz=(0.200, 0.0, -0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=65.0,
            velocity=0.30,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "column_to_table_support",
        ArticulationType.FIXED,
        parent=column,
        child=table_support,
        origin=Origin(xyz=(0.0, 0.0, TABLE_SUPPORT_Z)),
    )
    model.articulation(
        "table_support_to_pin",
        ArticulationType.FIXED,
        parent=table_support,
        child=table_pin,
        origin=Origin(xyz=(0.0, TABLE_PIVOT_Y, TABLE_PIVOT_Z)),
    )
    model.articulation(
        "table_tilt",
        ArticulationType.REVOLUTE,
        parent=table_pin,
        child=work_table,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    column = object_model.get_part("column")
    arm_carrier = object_model.get_part("arm_carrier")
    radial_arm = object_model.get_part("radial_arm")
    drill_head = object_model.get_part("drill_head")
    table_support = object_model.get_part("table_support")
    table_pin = object_model.get_part("table_pin")
    work_table = object_model.get_part("work_table")

    arm_lift = object_model.get_articulation("arm_lift")
    arm_azimuth = object_model.get_articulation("arm_azimuth")
    head_slide = object_model.get_articulation("head_slide")
    table_tilt = object_model.get_articulation("table_tilt")

    base_plinth = base.get_visual("base_plinth")
    column_collar = column.get_visual("column_collar")
    column_shaft = column.get_visual("column_shaft")

    sleeve_left = arm_carrier.get_visual("sleeve_left")
    sleeve_right = arm_carrier.get_visual("sleeve_right")
    sleeve_back = arm_carrier.get_visual("sleeve_back")
    sleeve_front = arm_carrier.get_visual("sleeve_front")
    turntable_plate = arm_carrier.get_visual("turntable_plate")

    arm_mount_plate = radial_arm.get_visual("arm_mount_plate")
    arm_beam = radial_arm.get_visual("arm_beam")
    dovetail_guide = radial_arm.get_visual("dovetail_guide")

    guide_pad = drill_head.get_visual("guide_pad")
    carriage_jaw_left = drill_head.get_visual("carriage_jaw_left")
    head_casting = drill_head.get_visual("head_casting")
    chuck = drill_head.get_visual("chuck")

    support_clamp = table_support.get_visual("support_clamp")
    support_arm = table_support.get_visual("support_arm")
    support_drop = table_support.get_visual("support_drop")
    table_yoke = table_support.get_visual("table_yoke")
    tilt_pin = table_pin.get_visual("tilt_pin")

    table_disk = work_table.get_visual("table_disk")
    table_web = work_table.get_visual("table_web")
    table_hub = work_table.get_visual("table_hub")
    left_ear = work_table.get_visual("left_ear")
    right_ear = work_table.get_visual("right_ear")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(table_pin, work_table, reason="tilt pin passes through the welded ear brackets below the table")
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(base, column, elem_a=base_plinth, elem_b=column_collar)
    ctx.expect_within(column, table_support, axes="xy", inner_elem=column_shaft, outer_elem=support_clamp)
    ctx.expect_gap(
        table_support,
        work_table,
        axis="z",
        min_gap=0.012,
        max_gap=0.020,
        positive_elem=support_arm,
        negative_elem=table_disk,
    )
    ctx.expect_origin_distance(arm_carrier, column, axes="xy", max_dist=0.001)
    ctx.expect_gap(
        arm_carrier,
        column,
        axis="x",
        min_gap=0.009,
        max_gap=0.012,
        positive_elem=sleeve_right,
        negative_elem=column_shaft,
    )
    ctx.expect_gap(
        column,
        arm_carrier,
        axis="x",
        min_gap=0.009,
        max_gap=0.012,
        positive_elem=column_shaft,
        negative_elem=sleeve_left,
    )
    ctx.expect_gap(
        arm_carrier,
        column,
        axis="y",
        min_gap=0.008,
        max_gap=0.011,
        positive_elem=sleeve_front,
        negative_elem=column_shaft,
    )
    ctx.expect_gap(
        column,
        arm_carrier,
        axis="y",
        min_gap=0.008,
        max_gap=0.011,
        positive_elem=column_shaft,
        negative_elem=sleeve_back,
    )
    ctx.expect_contact(arm_carrier, radial_arm, elem_a=turntable_plate, elem_b=arm_mount_plate)
    ctx.expect_gap(
        radial_arm,
        base,
        axis="z",
        min_gap=0.72,
        positive_elem=arm_beam,
        negative_elem=base_plinth,
    )
    ctx.expect_gap(
        radial_arm,
        drill_head,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=dovetail_guide,
        negative_elem=guide_pad,
    )
    ctx.expect_overlap(
        drill_head,
        radial_arm,
        axes="xy",
        min_overlap=0.04,
        elem_a=guide_pad,
        elem_b=dovetail_guide,
    )
    ctx.expect_overlap(
        drill_head,
        radial_arm,
        axes="xy",
        min_overlap=0.01,
        elem_a=carriage_jaw_left,
        elem_b=dovetail_guide,
    )
    ctx.expect_gap(
        drill_head,
        column,
        axis="x",
        min_gap=0.09,
        positive_elem=head_casting,
        negative_elem=column_shaft,
    )
    ctx.expect_overlap(table_pin, work_table, axes="xy", min_overlap=0.01, elem_a=tilt_pin, elem_b=left_ear)
    ctx.expect_overlap(table_pin, work_table, axes="xy", min_overlap=0.01, elem_a=tilt_pin, elem_b=right_ear)
    ctx.expect_gap(
        work_table,
        table_support,
        axis="x",
        min_gap=0.016,
        max_gap=0.018,
        positive_elem=right_ear,
        negative_elem=table_yoke,
    )
    ctx.expect_gap(
        table_support,
        work_table,
        axis="x",
        min_gap=0.016,
        max_gap=0.018,
        positive_elem=table_yoke,
        negative_elem=left_ear,
    )

    with ctx.pose({arm_azimuth: math.pi / 2.0}):
        ctx.expect_contact(arm_carrier, radial_arm, elem_a=turntable_plate, elem_b=arm_mount_plate)
        ctx.expect_gap(
            radial_arm,
            base,
            axis="z",
            min_gap=0.72,
            positive_elem=arm_beam,
            negative_elem=base_plinth,
        )

    with ctx.pose({arm_lift: 0.16}):
        ctx.expect_origin_distance(arm_carrier, column, axes="xy", max_dist=0.001)
        ctx.expect_gap(
            radial_arm,
            base,
            axis="z",
            min_gap=0.88,
            positive_elem=arm_beam,
            negative_elem=base_plinth,
        )

    with ctx.pose({head_slide: 0.24}):
        ctx.expect_gap(
            drill_head,
            column,
            axis="x",
            min_gap=0.33,
            positive_elem=head_casting,
            negative_elem=column_shaft,
        )

    with ctx.pose({arm_azimuth: -math.pi / 2.0, head_slide: 0.22}):
        ctx.expect_overlap(
            drill_head,
            work_table,
            axes="xy",
            min_overlap=0.02,
            elem_a=chuck,
            elem_b=table_disk,
        )
        ctx.expect_gap(
            drill_head,
            work_table,
            axis="z",
            min_gap=0.02,
            max_gap=0.07,
            positive_elem=chuck,
            negative_elem=table_disk,
        )

    with ctx.pose({table_tilt: 0.40}):
        ctx.expect_gap(
            work_table,
            table_support,
            axis="x",
            min_gap=0.016,
            max_gap=0.018,
            positive_elem=right_ear,
            negative_elem=table_yoke,
        )
        ctx.expect_gap(
            table_support,
            work_table,
            axis="x",
            min_gap=0.016,
            max_gap=0.018,
            positive_elem=table_yoke,
            negative_elem=left_ear,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
