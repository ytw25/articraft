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


def _add_arc_panel(
    part,
    *,
    radius: float,
    angle_deg: float,
    width: float,
    height: float,
    bottom_z: float,
    thickness: float,
    material,
    name: str | None = None,
) -> None:
    angle = math.radians(angle_deg)
    part.visual(
        Box((thickness, width, height)),
        origin=Origin(
            xyz=(radius * math.cos(angle), radius * math.sin(angle), bottom_z + height * 0.5),
            rpy=(0.0, 0.0, angle),
        ),
        material=material,
        name=name,
    )


def _add_rotor_wing(
    part,
    *,
    yaw: float,
    frame_material,
    glass_material,
    outer_edge_name: str | None = None,
) -> None:
    inner_stile_x = 0.22
    outer_stile_x = 1.62
    stile_height = 2.56
    stile_center_z = 1.32
    rail_center_x = 0.93
    rail_length = 1.48

    part.visual(
        Box((0.05, 0.07, stile_height)),
        origin=Origin(xyz=(inner_stile_x, 0.0, stile_center_z), rpy=(0.0, 0.0, yaw)),
        material=frame_material,
    )
    part.visual(
        Box((0.04, 0.08, stile_height)),
        origin=Origin(xyz=(outer_stile_x, 0.0, stile_center_z), rpy=(0.0, 0.0, yaw)),
        material=frame_material,
        name=outer_edge_name,
    )
    part.visual(
        Box((1.33, 0.024, 2.38)),
        origin=Origin(xyz=(0.93, 0.0, 1.31), rpy=(0.0, 0.0, yaw)),
        material=glass_material,
    )
    part.visual(
        Box((rail_length, 0.07, 0.08)),
        origin=Origin(xyz=(rail_center_x, 0.0, 0.06), rpy=(0.0, 0.0, yaw)),
        material=frame_material,
    )
    part.visual(
        Box((rail_length, 0.07, 0.10)),
        origin=Origin(xyz=(rail_center_x, 0.0, 2.54), rpy=(0.0, 0.0, yaw)),
        material=frame_material,
    )
    part.visual(
        Box((0.02, 0.035, 2.40)),
        origin=Origin(xyz=(1.65, 0.0, 1.31), rpy=(0.0, 0.0, yaw)),
        material=frame_material,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="museum_revolving_door")

    bronze = model.material("bronze", rgba=(0.40, 0.31, 0.22, 1.0))
    dark_bronze = model.material("dark_bronze", rgba=(0.20, 0.16, 0.12, 1.0))
    threshold_stone = model.material("threshold_stone", rgba=(0.25, 0.25, 0.27, 1.0))
    clear_glass = model.material("clear_glass", rgba=(0.70, 0.84, 0.92, 0.28))
    tinted_glass = model.material("tinted_glass", rgba=(0.62, 0.78, 0.86, 0.24))

    drum = model.part("drum")
    drum.visual(
        Cylinder(radius=1.90, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=threshold_stone,
        name="drum_floor",
    )
    drum.visual(
        Cylinder(radius=1.90, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 2.80)),
        material=dark_bronze,
        name="canopy_disk",
    )
    wall_radius = 1.73
    panel_height = 2.64
    panel_width = 0.26
    panel_thickness = 0.03
    for angle_deg in (-48.0, -32.0, -16.0, 16.0, 32.0, 48.0, 132.0, 148.0, 164.0, 196.0, 212.0, 228.0):
        _add_arc_panel(
            drum,
            radius=wall_radius,
            angle_deg=angle_deg,
            width=panel_width,
            height=panel_height,
            bottom_z=0.06,
            thickness=panel_thickness,
            material=tinted_glass,
        )

    for angle_deg in (-58.0, 58.0, 122.0, 238.0):
        _add_arc_panel(
            drum,
            radius=1.72,
            angle_deg=angle_deg,
            width=0.12,
            height=2.64,
            bottom_z=0.06,
            thickness=0.08,
            material=bronze,
        )

    drum.visual(
        Box((0.18, 1.20, 0.12)),
        origin=Origin(xyz=(1.81, 0.0, 2.76)),
        material=bronze,
    )
    drum.visual(
        Box((0.18, 1.20, 0.12)),
        origin=Origin(xyz=(-1.81, 0.0, 2.76)),
        material=bronze,
    )
    drum.inertial = Inertial.from_geometry(
        Box((3.80, 3.80, 2.90)),
        mass=900.0,
        origin=Origin(xyz=(0.0, 0.0, 1.45)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.08, length=2.64),
        origin=Origin(xyz=(0.0, 0.0, 1.32)),
        material=bronze,
        name="center_pillar",
    )
    rotor.visual(
        Cylinder(radius=0.15, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_bronze,
        name="bottom_bearing",
    )
    rotor.visual(
        Cylinder(radius=0.15, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 2.56)),
        material=dark_bronze,
        name="top_bearing",
    )
    rotor.visual(
        Box((0.42, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_bronze,
    )
    rotor.visual(
        Box((0.08, 0.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_bronze,
    )
    rotor.visual(
        Box((0.42, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.54)),
        material=dark_bronze,
    )
    rotor.visual(
        Box((0.08, 0.42, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 2.54)),
        material=dark_bronze,
    )

    _add_rotor_wing(
        rotor,
        yaw=0.0,
        frame_material=bronze,
        glass_material=clear_glass,
        outer_edge_name="wing_east_edge",
    )
    _add_rotor_wing(
        rotor,
        yaw=math.pi * 0.5,
        frame_material=bronze,
        glass_material=clear_glass,
    )
    _add_rotor_wing(
        rotor,
        yaw=math.pi,
        frame_material=bronze,
        glass_material=clear_glass,
    )
    _add_rotor_wing(
        rotor,
        yaw=-math.pi * 0.5,
        frame_material=bronze,
        glass_material=clear_glass,
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=1.66, length=2.64),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, 1.32)),
    )

    model.articulation(
        "revolving_rotation",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    drum = object_model.get_part("drum")
    rotor = object_model.get_part("rotor")
    rotation = object_model.get_articulation("revolving_rotation")

    limits = rotation.motion_limits
    ctx.check(
        "wing assembly uses continuous rotation",
        rotation.articulation_type == ArticulationType.CONTINUOUS
        and limits is not None
        and limits.lower is None
        and limits.upper is None,
        details=f"type={rotation.articulation_type}, limits={limits}",
    )

    drum_aabb = ctx.part_world_aabb(drum)
    drum_dims_ok = False
    if drum_aabb is not None:
        drum_dims_ok = (
            drum_aabb[1][0] - drum_aabb[0][0] > 3.5
            and drum_aabb[1][1] - drum_aabb[0][1] > 3.5
            and drum_aabb[1][2] - drum_aabb[0][2] > 2.8
        )
    ctx.check(
        "drum reads as a large museum-scale enclosure",
        drum_dims_ok,
        details=f"aabb={drum_aabb}",
    )

    with ctx.pose({rotation: 0.0}):
        ctx.expect_within(
            rotor,
            drum,
            axes="xy",
            margin=0.02,
            name="rotor stays inside the drum footprint at rest",
        )
        ctx.expect_contact(
            rotor,
            drum,
            elem_a="bottom_bearing",
            elem_b="drum_floor",
            name="rotor seats on the floor bearing",
        )
        ctx.expect_contact(
            rotor,
            drum,
            elem_a="top_bearing",
            elem_b="canopy_disk",
            name="rotor reaches the overhead bearing",
        )

    rest_edge = ctx.part_element_world_aabb(rotor, elem="wing_east_edge")
    with ctx.pose({rotation: math.pi * 0.25}):
        ctx.expect_within(
            rotor,
            drum,
            axes="xy",
            margin=0.02,
            name="rotor stays inside the drum footprint while turning",
        )
    with ctx.pose({rotation: math.pi * 0.5}):
        turned_edge = ctx.part_element_world_aabb(rotor, elem="wing_east_edge")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_center = _aabb_center(rest_edge)
    turned_center = _aabb_center(turned_edge)
    rotates_correctly = (
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 1.45
        and abs(rest_center[1]) < 0.08
        and turned_center[1] > 1.45
        and abs(turned_center[0]) < 0.08
        and abs(turned_center[2] - rest_center[2]) < 0.02
    )
    ctx.check(
        "east wing rotates around the vertical pillar axis",
        rotates_correctly,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
