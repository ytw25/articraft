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
    section_loft,
)


OUTER_RADIUS = 0.82
THRESHOLD_HEIGHT = 0.05
CLEAR_HEIGHT = 2.28
CANOPY_HEIGHT = 0.18
GLASS_RADIUS = 0.785
GLASS_THICKNESS = 0.012
JAMB_DEPTH = 0.055
JAMB_WIDTH = 0.06
SIDE_ARC_HALF_ANGLE = math.radians(50.0)
POST_RADIUS = 0.042
WING_LENGTH = 0.68
WING_GLASS_LENGTH = 0.60
WING_GLASS_THICKNESS = 0.012
WING_RAIL_CENTER_Z = 0.06
WING_RAIL_HEIGHT = 0.035
WING_PANEL_HEIGHT = CLEAR_HEIGHT - 0.155
WING_STILE_HEIGHT = CLEAR_HEIGHT - 0.085


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _wall_section(
    radius_mid: float,
    thickness: float,
    z0: float,
    z1: float,
    angle: float,
) -> list[tuple[float, float, float]]:
    outer_r = radius_mid + (thickness * 0.5)
    inner_r = radius_mid - (thickness * 0.5)
    ox, oy = _polar_xy(outer_r, angle)
    ix, iy = _polar_xy(inner_r, angle)
    return [
        (ox, oy, z0),
        (ix, iy, z0),
        (ix, iy, z1),
        (ox, oy, z1),
    ]


def _curved_wall_mesh(
    *,
    name: str,
    radius_mid: float,
    thickness: float,
    z0: float,
    z1: float,
    start_angle: float,
    end_angle: float,
    sections: int = 8,
):
    loops = []
    for index in range(sections):
        t = index / (sections - 1)
        angle = start_angle + (end_angle - start_angle) * t
        loops.append(_wall_section(radius_mid, thickness, z0, z1, angle))
    return mesh_from_geometry(section_loft(loops), name)


def _add_sidewall(
    model: ArticulatedObject,
    *,
    name: str,
    start_angle: float,
    end_angle: float,
    frame_material,
    glass_material,
) -> None:
    sidewall = model.part(name)
    glass_mesh = _curved_wall_mesh(
        name=f"{name}_glass",
        radius_mid=GLASS_RADIUS,
        thickness=GLASS_THICKNESS,
        z0=0.055,
        z1=CLEAR_HEIGHT - 0.055,
        start_angle=start_angle,
        end_angle=end_angle,
    )
    sidewall.visual(
        glass_mesh,
        material=glass_material,
        name="curved_glass",
    )

    for suffix, angle in (("front_jamb", start_angle), ("rear_jamb", end_angle)):
        x, y = _polar_xy(GLASS_RADIUS, angle)
        sidewall.visual(
            Box((JAMB_DEPTH, JAMB_WIDTH, CLEAR_HEIGHT)),
            origin=Origin(
                xyz=(x, y, CLEAR_HEIGHT * 0.5),
                rpy=(0.0, 0.0, angle + (math.pi * 0.5)),
            ),
            material=frame_material,
            name=suffix,
        )

    sidewall.inertial = Inertial.from_geometry(
        Box((0.26, 1.35, CLEAR_HEIGHT)),
        mass=35.0,
        origin=Origin(xyz=(0.0, 0.0, CLEAR_HEIGHT * 0.5)),
    )

    model.articulation(
        f"threshold_to_{name}",
        ArticulationType.FIXED,
        parent="threshold",
        child=sidewall,
        origin=Origin(xyz=(0.0, 0.0, THRESHOLD_HEIGHT)),
    )


def _add_wing(rotor, *, angle: float, glass_material, rail_material, index: int) -> None:
    wing_origin = Origin(rpy=(0.0, 0.0, angle))
    rail_center_x = POST_RADIUS + 0.5 * WING_LENGTH
    glass_center_x = POST_RADIUS + 0.06 + (WING_GLASS_LENGTH * 0.5)

    rotor.visual(
        Box((0.05, 0.05, CLEAR_HEIGHT - 0.28)),
        origin=Origin(
            xyz=(POST_RADIUS + 0.024, 0.0, CLEAR_HEIGHT * 0.5),
            rpy=wing_origin.rpy,
        ),
        material=rail_material,
        name=f"inner_stile_{index}",
    )
    rotor.visual(
        Box((WING_LENGTH, 0.038, 0.035)),
        origin=Origin(
            xyz=(rail_center_x, 0.0, WING_RAIL_CENTER_Z),
            rpy=wing_origin.rpy,
        ),
        material=rail_material,
        name=f"bottom_rail_{index}",
    )
    rotor.visual(
        Box((WING_LENGTH, 0.038, 0.035)),
        origin=Origin(
            xyz=(rail_center_x, 0.0, CLEAR_HEIGHT - WING_RAIL_CENTER_Z),
            rpy=wing_origin.rpy,
        ),
        material=rail_material,
        name=f"top_rail_{index}",
    )
    rotor.visual(
        Box((0.035, 0.045, WING_STILE_HEIGHT)),
        origin=Origin(
            xyz=(POST_RADIUS + WING_LENGTH - 0.018, 0.0, CLEAR_HEIGHT * 0.5),
            rpy=wing_origin.rpy,
        ),
        material=rail_material,
        name=f"outer_stile_{index}",
    )
    rotor.visual(
        Box((WING_GLASS_LENGTH, WING_GLASS_THICKNESS, WING_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(glass_center_x, 0.0, CLEAR_HEIGHT * 0.5),
            rpy=wing_origin.rpy,
        ),
        material=glass_material,
        name=f"wing_panel_{index}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wing_revolving_door")

    anodized_aluminum = model.material(
        "anodized_aluminum",
        rgba=(0.43, 0.46, 0.49, 1.0),
    )
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    storefront_glass = model.material(
        "storefront_glass",
        rgba=(0.66, 0.80, 0.87, 0.28),
    )
    threshold_stone = model.material(
        "threshold_stone",
        rgba=(0.48, 0.49, 0.47, 1.0),
    )

    threshold = model.part("threshold")
    threshold.visual(
        Cylinder(radius=OUTER_RADIUS, length=THRESHOLD_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, THRESHOLD_HEIGHT * 0.5)),
        material=threshold_stone,
        name="threshold_drum",
    )
    threshold.visual(
        Cylinder(radius=0.22, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, THRESHOLD_HEIGHT - 0.009)),
        material=dark_metal,
        name="center_bearing_plate",
    )
    threshold.inertial = Inertial.from_geometry(
        Cylinder(radius=OUTER_RADIUS, length=THRESHOLD_HEIGHT),
        mass=55.0,
        origin=Origin(xyz=(0.0, 0.0, THRESHOLD_HEIGHT * 0.5)),
    )

    canopy = model.part("canopy")
    canopy.visual(
        Cylinder(radius=OUTER_RADIUS, length=CANOPY_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CANOPY_HEIGHT * 0.5)),
        material=anodized_aluminum,
        name="canopy_drum",
    )
    canopy.visual(
        Cylinder(radius=0.18, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_metal,
        name="ceiling_bearing_collar",
    )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=OUTER_RADIUS, length=CANOPY_HEIGHT),
        mass=60.0,
    )
    model.articulation(
        "threshold_to_canopy",
        ArticulationType.FIXED,
        parent=threshold,
        child=canopy,
        origin=Origin(xyz=(0.0, 0.0, THRESHOLD_HEIGHT + CLEAR_HEIGHT)),
    )

    _add_sidewall(
        model,
        name="right_sidewall",
        start_angle=-SIDE_ARC_HALF_ANGLE,
        end_angle=SIDE_ARC_HALF_ANGLE,
        frame_material=anodized_aluminum,
        glass_material=storefront_glass,
    )
    _add_sidewall(
        model,
        name="left_sidewall",
        start_angle=math.pi - SIDE_ARC_HALF_ANGLE,
        end_angle=math.pi + SIDE_ARC_HALF_ANGLE,
        frame_material=anodized_aluminum,
        glass_material=storefront_glass,
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=POST_RADIUS, length=CLEAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CLEAR_HEIGHT * 0.5)),
        material=dark_metal,
        name="central_post",
    )
    rotor.visual(
        Cylinder(radius=0.078, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_metal,
        name="bottom_pivot_collar",
    )
    rotor.visual(
        Cylinder(radius=0.078, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, CLEAR_HEIGHT - 0.012)),
        material=dark_metal,
        name="top_pivot_collar",
    )
    rotor.visual(
        Cylinder(radius=0.018, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, CLEAR_HEIGHT - 0.015)),
        material=dark_metal,
        name="top_spindle",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        _add_wing(
            rotor,
            angle=angle,
            glass_material=storefront_glass,
            rail_material=anodized_aluminum,
            index=index,
        )

    rotor.inertial = Inertial.from_geometry(
        Box((1.48, 1.48, CLEAR_HEIGHT)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, CLEAR_HEIGHT * 0.5)),
    )
    model.articulation(
        "threshold_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=threshold,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, THRESHOLD_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2),
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

    threshold = object_model.get_part("threshold")
    canopy = object_model.get_part("canopy")
    left_sidewall = object_model.get_part("left_sidewall")
    right_sidewall = object_model.get_part("right_sidewall")
    rotor = object_model.get_part("rotor")
    rotor_joint = object_model.get_articulation("threshold_to_rotor")

    ctx.expect_contact(
        right_sidewall,
        threshold,
        name="right sidewall sits on the threshold",
    )
    ctx.expect_contact(
        left_sidewall,
        threshold,
        name="left sidewall sits on the threshold",
    )
    ctx.expect_contact(
        right_sidewall,
        canopy,
        name="right sidewall reaches the canopy",
    )
    ctx.expect_contact(
        left_sidewall,
        canopy,
        name="left sidewall reaches the canopy",
    )
    ctx.expect_contact(
        rotor,
        threshold,
        name="rotor is supported by the threshold pivot",
    )
    ctx.expect_contact(
        rotor,
        canopy,
        name="rotor reaches the canopy pivot collar",
    )
    ctx.expect_within(
        rotor,
        canopy,
        axes="xy",
        margin=0.0,
        name="rotor footprint stays inside the drum canopy",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    rest_aabb = ctx.part_element_world_aabb(rotor, elem="wing_panel_0")
    with ctx.pose({rotor_joint: math.pi * 0.5}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="wing_panel_0")
        ctx.expect_within(
            rotor,
            canopy,
            axes="xy",
            margin=0.0,
            name="rotor stays within the drum while rotating",
        )

    rest_center = _aabb_center(rest_aabb)
    turned_center = _aabb_center(turned_aabb)
    ctx.check(
        "continuous rotation turns the wing around the central post",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.20
        and abs(rest_center[1]) < 0.05
        and turned_center[1] > 0.20
        and abs(turned_center[0]) < 0.05,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )
    ctx.check(
        "rotor uses an unrestricted continuous joint",
        rotor_joint.motion_limits is not None
        and rotor_joint.motion_limits.lower is None
        and rotor_joint.motion_limits.upper is None,
        details=f"limits={rotor_joint.motion_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
