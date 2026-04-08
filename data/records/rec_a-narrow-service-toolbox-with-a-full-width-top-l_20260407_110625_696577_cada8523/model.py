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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


BODY_WIDTH = 0.56
BODY_DEPTH = 0.22
BODY_HEIGHT = 0.24
WALL_THICKNESS = 0.008

LID_THICKNESS = 0.006
LID_FLANGE_DEPTH = 0.028
LID_CLEARANCE = 0.002
LID_WIDTH = BODY_WIDTH + 2.0 * (LID_CLEARANCE + LID_THICKNESS)
LID_DEPTH = BODY_DEPTH + 2.0 * (LID_CLEARANCE + LID_THICKNESS)

HANDLE_PIVOT_Z = 0.165
HANDLE_SPAN = 0.110
HANDLE_DROP = 0.090
HANDLE_OUTSET = 0.030
HANDLE_RADIUS = 0.0045


def _handle_mesh(side_sign: float, mesh_name: str):
    return mesh_from_geometry(
        wire_from_points(
            [
                (0.0, -HANDLE_SPAN * 0.5, 0.0),
                (side_sign * HANDLE_OUTSET * 0.45, -HANDLE_SPAN * 0.5, -0.016),
                (side_sign * HANDLE_OUTSET, -HANDLE_SPAN * 0.5, -HANDLE_DROP),
                (side_sign * HANDLE_OUTSET, HANDLE_SPAN * 0.5, -HANDLE_DROP),
                (side_sign * HANDLE_OUTSET * 0.45, HANDLE_SPAN * 0.5, -0.016),
                (0.0, HANDLE_SPAN * 0.5, 0.0),
            ],
            radius=HANDLE_RADIUS,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.014,
            corner_segments=10,
        ),
        mesh_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_toolbox")

    shell_blue = model.material("shell_blue", rgba=(0.13, 0.25, 0.47, 1.0))
    lid_blue = model.material("lid_blue", rgba=(0.16, 0.29, 0.52, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    plated_steel = model.material("plated_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        Box((BODY_WIDTH, BODY_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, WALL_THICKNESS * 0.5)),
        material=shell_blue,
        name="body_bottom",
    )
    lower_shell.visual(
        Box((BODY_WIDTH, WALL_THICKNESS, BODY_HEIGHT - WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_DEPTH * 0.5 - WALL_THICKNESS * 0.5),
                WALL_THICKNESS + (BODY_HEIGHT - WALL_THICKNESS) * 0.5,
            )
        ),
        material=shell_blue,
        name="front_wall",
    )
    lower_shell.visual(
        Box((BODY_WIDTH, WALL_THICKNESS, BODY_HEIGHT - WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 - WALL_THICKNESS * 0.5,
                WALL_THICKNESS + (BODY_HEIGHT - WALL_THICKNESS) * 0.5,
            )
        ),
        material=shell_blue,
        name="back_wall",
    )
    lower_shell.visual(
        Box((WALL_THICKNESS, BODY_DEPTH - 2.0 * WALL_THICKNESS, BODY_HEIGHT - WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                -(BODY_WIDTH * 0.5 - WALL_THICKNESS * 0.5),
                0.0,
                WALL_THICKNESS + (BODY_HEIGHT - WALL_THICKNESS) * 0.5,
            )
        ),
        material=shell_blue,
        name="left_wall",
    )
    lower_shell.visual(
        Box((WALL_THICKNESS, BODY_DEPTH - 2.0 * WALL_THICKNESS, BODY_HEIGHT - WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                BODY_WIDTH * 0.5 - WALL_THICKNESS * 0.5,
                0.0,
                WALL_THICKNESS + (BODY_HEIGHT - WALL_THICKNESS) * 0.5,
            )
        ),
        material=shell_blue,
        name="right_wall",
    )
    lower_shell.visual(
        Box((0.050, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -(BODY_DEPTH * 0.5 + 0.0085), 0.196)),
        material=plated_steel,
        name="front_keeper",
    )
    lower_shell.visual(
        Box((0.002561, 0.132, 0.036)),
        origin=Origin(xyz=(-(BODY_WIDTH * 0.5 + 0.0012805), 0.0, HANDLE_PIVOT_Z)),
        material=dark_steel,
        name="left_mount_plate",
    )
    lower_shell.visual(
        Box((0.002561, 0.132, 0.036)),
        origin=Origin(xyz=(BODY_WIDTH * 0.5 + 0.0012805, 0.0, HANDLE_PIVOT_Z)),
        material=dark_steel,
        name="right_mount_plate",
    )
    lower_shell.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=5.6,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_WIDTH, LID_DEPTH, LID_THICKNESS)),
        origin=Origin(xyz=(0.0, -LID_DEPTH * 0.5, LID_THICKNESS * 0.5)),
        material=lid_blue,
        name="lid_top",
    )
    lid.visual(
        Box((LID_WIDTH, LID_THICKNESS, LID_FLANGE_DEPTH)),
        origin=Origin(
            xyz=(
                0.0,
                -(LID_DEPTH - LID_THICKNESS * 0.5),
                -(LID_FLANGE_DEPTH * 0.5 - 0.003),
            )
        ),
        material=lid_blue,
        name="lid_front_flange",
    )
    lid.visual(
        Box((LID_THICKNESS, LID_DEPTH - 2.0 * LID_THICKNESS, LID_FLANGE_DEPTH)),
        origin=Origin(
            xyz=(
                -(LID_WIDTH * 0.5 - LID_THICKNESS * 0.5),
                -(LID_DEPTH * 0.5 - LID_THICKNESS),
                -(LID_FLANGE_DEPTH * 0.5 - 0.003),
            )
        ),
        material=lid_blue,
        name="lid_left_flange",
    )
    lid.visual(
        Box((LID_THICKNESS, LID_DEPTH - 2.0 * LID_THICKNESS, LID_FLANGE_DEPTH)),
        origin=Origin(
            xyz=(
                LID_WIDTH * 0.5 - LID_THICKNESS * 0.5,
                -(LID_DEPTH * 0.5 - LID_THICKNESS),
                -(LID_FLANGE_DEPTH * 0.5 - 0.003),
            )
        ),
        material=lid_blue,
        name="lid_right_flange",
    )
    lid.visual(
        Box((0.012, 0.010, 0.014)),
        origin=Origin(xyz=(-0.046, -(LID_DEPTH + 0.0045), -0.003)),
        material=dark_steel,
        name="hasp_left_tab",
    )
    lid.visual(
        Box((0.012, 0.010, 0.014)),
        origin=Origin(xyz=(0.046, -(LID_DEPTH + 0.0045), -0.003)),
        material=dark_steel,
        name="hasp_right_tab",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_DEPTH, 0.032)),
        mass=1.7,
        origin=Origin(xyz=(0.0, -LID_DEPTH * 0.5, -0.008)),
    )

    hasp = model.part("hasp")
    hasp.visual(
        Cylinder(radius=0.006, length=0.080),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=plated_steel,
        name="hasp_barrel",
    )
    hasp.visual(
        Box((0.072, 0.004, 0.066)),
        origin=Origin(xyz=(0.0, -0.007, -0.033)),
        material=plated_steel,
        name="hasp_plate",
    )
    hasp.visual(
        Box((0.028, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.010, -0.063)),
        material=dark_steel,
        name="hasp_hook",
    )
    hasp.inertial = Inertial.from_geometry(
        Box((0.080, 0.014, 0.070)),
        mass=0.10,
        origin=Origin(xyz=(0.0, -0.006, -0.032)),
    )

    left_handle = model.part("left_handle")
    left_handle.visual(_handle_mesh(-1.0, "left_handle_loop"), material=grip_black, name="left_handle_loop")
    left_handle.inertial = Inertial.from_geometry(
        Box((0.034, 0.120, 0.100)),
        mass=0.22,
        origin=Origin(xyz=(-0.017, 0.0, -0.045)),
    )

    right_handle = model.part("right_handle")
    right_handle.visual(_handle_mesh(1.0, "right_handle_loop"), material=grip_black, name="right_handle_loop")
    right_handle.inertial = Inertial.from_geometry(
        Box((0.034, 0.120, 0.100)),
        mass=0.22,
        origin=Origin(xyz=(0.017, 0.0, -0.045)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, LID_DEPTH * 0.5, BODY_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.45),
    )
    model.articulation(
        "lid_to_hasp",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp,
        origin=Origin(xyz=(0.0, -LID_DEPTH - 0.007, 0.001)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=1.30),
    )
    model.articulation(
        "left_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=left_handle,
        origin=Origin(xyz=(-(BODY_WIDTH * 0.5 + 0.006), 0.0, HANDLE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "right_handle_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=right_handle,
        origin=Origin(xyz=(BODY_WIDTH * 0.5 + 0.006, 0.0, HANDLE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    hasp = object_model.get_part("hasp")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")

    lid_hinge = object_model.get_articulation("lid_hinge")
    hasp_joint = object_model.get_articulation("lid_to_hasp")
    left_joint = object_model.get_articulation("left_handle_pivot")
    right_joint = object_model.get_articulation("right_handle_pivot")

    ctx.expect_overlap(
        lid,
        lower_shell,
        axes="xy",
        min_overlap=0.20,
        elem_a="lid_top",
        name="lid covers the lower shell opening when closed",
    )
    ctx.expect_overlap(
        hasp,
        lower_shell,
        axes="x",
        min_overlap=0.045,
        elem_a="hasp_plate",
        elem_b="front_keeper",
        name="hasp stays centered on the front keeper",
    )
    ctx.expect_gap(
        lower_shell,
        left_handle,
        axis="x",
        positive_elem="left_mount_plate",
        negative_elem="left_handle_loop",
        max_penetration=1e-5,
        max_gap=0.0035,
        name="left handle rests close to the left wall hardware",
    )
    ctx.expect_gap(
        right_handle,
        lower_shell,
        axis="x",
        positive_elem="right_handle_loop",
        negative_elem="right_mount_plate",
        max_penetration=1e-5,
        max_gap=0.0035,
        name="right handle rests close to the right wall hardware",
    )

    closed_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_flange")
    closed_hasp = ctx.part_element_world_aabb(hasp, elem="hasp_hook")
    closed_left = ctx.part_element_world_aabb(left_handle, elem="left_handle_loop")
    closed_right = ctx.part_element_world_aabb(right_handle, elem="right_handle_loop")

    with ctx.pose({lid_hinge: 1.20}):
        open_lid_front = ctx.part_element_world_aabb(lid, elem="lid_front_flange")

    with ctx.pose({hasp_joint: 1.00}):
        open_hasp = ctx.part_element_world_aabb(hasp, elem="hasp_hook")

    with ctx.pose({left_joint: 1.00}):
        open_left = ctx.part_element_world_aabb(left_handle, elem="left_handle_loop")

    with ctx.pose({right_joint: 1.00}):
        open_right = ctx.part_element_world_aabb(right_handle, elem="right_handle_loop")

    ctx.check(
        "lid front edge rises when the rear hinge opens",
        closed_lid_front is not None
        and open_lid_front is not None
        and open_lid_front[0][2] > closed_lid_front[1][2] + 0.12,
        details=f"closed={closed_lid_front}, open={open_lid_front}",
    )
    ctx.check(
        "hasp hook lifts away from the keeper when opened",
        closed_hasp is not None
        and open_hasp is not None
        and open_hasp[0][2] > closed_hasp[0][2] + 0.025,
        details=f"closed={closed_hasp}, open={open_hasp}",
    )
    ctx.check(
        "left handle swings outward and upward",
        closed_left is not None
        and open_left is not None
        and open_left[1][2] > closed_left[1][2] + 0.020
        and open_left[0][0] < closed_left[0][0] - 0.040,
        details=f"closed={closed_left}, open={open_left}",
    )
    ctx.check(
        "right handle swings outward and upward",
        closed_right is not None
        and open_right is not None
        and open_right[1][2] > closed_right[1][2] + 0.020
        and open_right[1][0] > closed_right[1][0] + 0.040,
        details=f"closed={closed_right}, open={open_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
