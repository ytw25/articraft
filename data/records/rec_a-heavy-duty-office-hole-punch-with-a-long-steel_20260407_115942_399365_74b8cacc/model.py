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
    sweep_profile_along_spline,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_office_hole_punch")

    painted_steel = model.material("painted_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.14, 0.15, 0.17, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.73, 0.75, 0.78, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, name)

    body = model.part("body")

    base_plate = save_mesh(
        "hole_punch_base_plate",
        ExtrudeGeometry(rounded_rect_profile(0.35, 0.125, 0.020), 0.014),
    )
    body.visual(
        base_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=painted_steel,
        name="base_plate",
    )
    body.visual(
        Box((0.175, 0.106, 0.016)),
        origin=Origin(xyz=(-0.075, 0.0, 0.021)),
        material=painted_steel,
        name="top_deck",
    )
    body.visual(
        Box((0.110, 0.024, 0.056)),
        origin=Origin(xyz=(-0.108, 0.048, 0.045)),
        material=painted_steel,
        name="left_cheek",
    )
    body.visual(
        Box((0.110, 0.024, 0.056)),
        origin=Origin(xyz=(-0.108, -0.048, 0.045)),
        material=painted_steel,
        name="right_cheek",
    )
    body.visual(
        Box((0.032, 0.100, 0.016)),
        origin=Origin(xyz=(-0.145, 0.0, 0.045)),
        material=painted_steel,
        name="rear_pivot_block",
    )
    body.visual(
        Box((0.088, 0.082, 0.026)),
        origin=Origin(xyz=(-0.056, 0.0, 0.060)),
        material=dark_steel,
        name="punch_bridge",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.026),
        origin=Origin(xyz=(-0.145, 0.030, 0.067), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_knuckle_left",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.026),
        origin=Origin(xyz=(-0.145, -0.030, 0.067), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_knuckle_right",
    )
    body.visual(
        Cylinder(radius=0.0085, length=0.006),
        origin=Origin(xyz=(-0.068, 0.020, 0.016)),
        material=satin_metal,
        name="die_ring_left",
    )
    body.visual(
        Cylinder(radius=0.0085, length=0.006),
        origin=Origin(xyz=(-0.036, -0.020, 0.016)),
        material=satin_metal,
        name="die_ring_right",
    )
    body.visual(
        Box((0.094, 0.004, 0.016)),
        origin=Origin(xyz=(-0.005, 0.032, 0.000)),
        material=dark_steel,
        name="tray_side_left",
    )
    body.visual(
        Box((0.094, 0.004, 0.016)),
        origin=Origin(xyz=(-0.005, -0.032, 0.000)),
        material=dark_steel,
        name="tray_side_right",
    )
    body.visual(
        Box((0.004, 0.064, 0.016)),
        origin=Origin(xyz=(0.044, 0.0, 0.000)),
        material=dark_steel,
        name="tray_front_wall",
    )
    body.visual(
        Box((0.012, 0.012, 0.014)),
        origin=Origin(xyz=(-0.054, 0.019, -0.001)),
        material=dark_steel,
        name="tray_bracket_left",
    )
    body.visual(
        Box((0.012, 0.012, 0.014)),
        origin=Origin(xyz=(-0.054, -0.019, -0.001)),
        material=dark_steel,
        name="tray_bracket_right",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(-0.055, 0.019, -0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="tray_hinge_left",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(-0.055, -0.019, -0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="tray_hinge_right",
    )
    for sx in (-0.130, 0.130):
        for sy in (-0.045, 0.045):
            body.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(sx, sy, -0.001)),
                material=rubber_black,
                name=f"foot_{'rear' if sx < 0.0 else 'front'}_{'left' if sy > 0.0 else 'right'}",
            )
    body.inertial = Inertial.from_geometry(
        Box((0.350, 0.125, 0.080)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    handle = model.part("handle")
    lever_beam = save_mesh(
        "hole_punch_handle_beam",
        sweep_profile_along_spline(
            [
                (0.0, 0.0, 0.029),
                (0.070, 0.0, 0.040),
                (0.190, 0.0, 0.050),
                (0.305, 0.0, 0.054),
            ],
            profile=rounded_rect_profile(0.022, 0.042, radius=0.006),
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    handle.visual(lever_beam, material=painted_steel, name="lever_beam")
    handle.visual(
        Cylinder(radius=0.0105, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.036, 0.030, 0.014)),
        origin=Origin(xyz=(0.030, 0.0, 0.013)),
        material=painted_steel,
        name="hinge_web",
    )
    handle.visual(
        Box((0.086, 0.090, 0.016)),
        origin=Origin(xyz=(0.082, 0.0, 0.021)),
        material=dark_steel,
        name="press_block",
    )
    handle.visual(
        Box((0.086, 0.078, 0.016)),
        origin=Origin(xyz=(0.273, 0.0, 0.040)),
        material=rubber_black,
        name="grip_pad",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.310, 0.090, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(0.155, 0.0, 0.030)),
    )

    tray_door = model.part("tray_door")
    tray_door.visual(
        Cylinder(radius=0.0040, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_barrel",
    )
    tray_door.visual(
        Box((0.014, 0.050, 0.008)),
        origin=Origin(xyz=(0.007, 0.0, -0.003)),
        material=dark_steel,
        name="rear_tab",
    )
    tray_door.visual(
        Box((0.098, 0.060, 0.003)),
        origin=Origin(xyz=(0.052, 0.0, -0.004)),
        material=painted_steel,
        name="door_panel",
    )
    tray_door.visual(
        Box((0.086, 0.003, 0.010)),
        origin=Origin(xyz=(0.050, 0.028, -0.0005)),
        material=painted_steel,
        name="door_side_left",
    )
    tray_door.visual(
        Box((0.086, 0.003, 0.010)),
        origin=Origin(xyz=(0.050, -0.028, -0.0005)),
        material=painted_steel,
        name="door_side_right",
    )
    tray_door.visual(
        Box((0.010, 0.022, 0.006)),
        origin=Origin(xyz=(0.101, 0.0, -0.006)),
        material=rubber_black,
        name="pull_tab",
    )
    tray_door.inertial = Inertial.from_geometry(
        Box((0.102, 0.060, 0.016)),
        mass=0.08,
        origin=Origin(xyz=(0.051, 0.0, -0.002)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.145, 0.0, 0.067)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "body_to_tray_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray_door,
        origin=Origin(xyz=(-0.055, 0.0, -0.004)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    tray_door = object_model.get_part("tray_door")
    handle_hinge = object_model.get_articulation("body_to_handle")
    tray_hinge = object_model.get_articulation("body_to_tray_door")

    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="press_block",
        negative_elem="punch_bridge",
        min_gap=0.003,
        max_gap=0.012,
        name="handle press block clears the punch bridge when closed",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="xy",
        elem_a="press_block",
        elem_b="punch_bridge",
        min_overlap=0.060,
        name="handle press block stays aligned over the punch bridge",
    )
    ctx.expect_gap(
        body,
        tray_door,
        axis="z",
        positive_elem="base_plate",
        negative_elem="door_panel",
        min_gap=0.001,
        max_gap=0.008,
        name="chip tray door sits just beneath the underside opening when closed",
    )

    closed_grip_aabb = ctx.part_element_world_aabb(handle, elem="grip_pad")
    with ctx.pose({handle_hinge: math.radians(60.0)}):
        open_grip_aabb = ctx.part_element_world_aabb(handle, elem="grip_pad")
    ctx.check(
        "handle opens upward from the rear hinge",
        closed_grip_aabb is not None
        and open_grip_aabb is not None
        and open_grip_aabb[1][2] > closed_grip_aabb[1][2] + 0.10,
        details=f"closed={closed_grip_aabb}, open={open_grip_aabb}",
    )

    closed_door_aabb = ctx.part_element_world_aabb(tray_door, elem="door_panel")
    with ctx.pose({tray_hinge: math.radians(95.0)}):
        open_door_aabb = ctx.part_element_world_aabb(tray_door, elem="door_panel")
    ctx.check(
        "chip tray door swings downward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][2] < closed_door_aabb[0][2] - 0.035,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
