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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    wrap_profile_onto_surface,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _annulus_mesh(*, outer_radius: float, inner_radius: float, height: float, name: str):
    return _mesh(
        name,
        ExtrudeWithHolesGeometry(
            superellipse_profile(outer_radius * 2.0, outer_radius * 2.0, exponent=2.0, segments=72),
            [superellipse_profile(inner_radius * 2.0, inner_radius * 2.0, exponent=2.0, segments=56)],
            height=height,
            center=True,
        ),
    )


def _curved_panel_mesh(
    *,
    name: str,
    radius: float,
    arc_length: float,
    height: float,
    thickness: float,
    direction: tuple[float, float, float],
    z_center: float,
    border: float | None = None,
):
    outer = rounded_rect_profile(arc_length, height, radius=min(0.045, arc_length * 0.08, height * 0.04), corner_segments=8)
    hole_profiles = []
    if border is not None:
        inner_width = arc_length - 2.0 * border
        inner_height = height - 2.0 * border
        hole_profiles = [
            rounded_rect_profile(
                inner_width,
                inner_height,
                radius=min(0.028, inner_width * 0.08, inner_height * 0.04),
                corner_segments=8,
            )
        ]

    geom = wrap_profile_onto_surface(
        outer,
        Cylinder(radius=radius, length=height),
        thickness=thickness,
        hole_profiles=hole_profiles,
        direction=direction,
        up_hint=(0.0, 0.0, 1.0),
    )
    geom.translate(0.0, 0.0, z_center)
    return _mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retail_revolving_door")

    aluminum = model.material("aluminum", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.23, 0.24, 0.26, 1.0))
    threshold_gray = model.material("threshold_gray", rgba=(0.42, 0.42, 0.44, 1.0))
    floor_black = model.material("floor_black", rgba=(0.10, 0.11, 0.12, 1.0))
    glass = model.material("glass", rgba=(0.62, 0.78, 0.88, 0.34))

    drum_radius = 0.92
    threshold_radius = 0.95
    canopy_radius = 0.98
    threshold_height = 0.045
    clear_height = 2.22
    canopy_height = 0.22
    canopy_bottom_z = threshold_height + clear_height
    wall_arc_length = 1.52
    wall_frame_thickness = 0.050
    wall_border = 0.060
    wing_length = 0.79
    wing_depth = 0.044
    wing_bottom_z = 0.06
    wing_top_z = clear_height - 0.06
    wing_height = wing_top_z - wing_bottom_z
    rail_height = 0.070
    stile_width = 0.060
    glass_overlap = 0.010

    drum = model.part("drum")
    drum.inertial = Inertial.from_geometry(
        Box((threshold_radius * 2.0, threshold_radius * 2.0, threshold_height)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, threshold_height * 0.5)),
    )

    threshold_ring = _annulus_mesh(
        outer_radius=threshold_radius,
        inner_radius=0.14,
        height=threshold_height,
        name="threshold_ring",
    )
    canopy_ring = _annulus_mesh(
        outer_radius=canopy_radius,
        inner_radius=0.17,
        height=canopy_height,
        name="canopy_ring",
    )
    floor_bearing_ring = _annulus_mesh(
        outer_radius=0.18,
        inner_radius=0.085,
        height=0.050,
        name="floor_bearing_ring",
    )
    bearing_ring = _annulus_mesh(
        outer_radius=0.19,
        inner_radius=0.050,
        height=0.050,
        name="overhead_bearing_housing",
    )
    drum.visual(threshold_ring, origin=Origin(xyz=(0.0, 0.0, threshold_height * 0.5)), material=threshold_gray, name="threshold_ring")
    drum.visual(floor_bearing_ring, origin=Origin(xyz=(0.0, 0.0, 0.025)), material=floor_black, name="floor_bearing_ring")
    drum.visual(canopy_ring, origin=Origin(xyz=(0.0, 0.0, canopy_bottom_z + canopy_height * 0.5)), material=aluminum, name="canopy_ring")
    drum.visual(bearing_ring, origin=Origin(xyz=(0.0, 0.0, canopy_bottom_z + 0.025)), material=floor_black, name="overhead_bearing_housing")

    left_wall_frame = _curved_panel_mesh(
        name="left_wall_frame",
        radius=drum_radius,
        arc_length=wall_arc_length,
        height=clear_height,
        thickness=wall_frame_thickness,
        direction=(-1.0, 0.0, 0.0),
        z_center=clear_height * 0.5,
        border=wall_border,
    )
    right_wall_frame = _curved_panel_mesh(
        name="right_wall_frame",
        radius=drum_radius,
        arc_length=wall_arc_length,
        height=clear_height,
        thickness=wall_frame_thickness,
        direction=(1.0, 0.0, 0.0),
        z_center=clear_height * 0.5,
        border=wall_border,
    )
    glass_arc_length = wall_arc_length - 2.0 * (wall_border - 0.008)
    glass_height = clear_height - 2.0 * (wall_border - 0.008)
    left_wall_glass = _curved_panel_mesh(
        name="left_wall_glass",
        radius=drum_radius - wall_frame_thickness * 0.55,
        arc_length=glass_arc_length,
        height=glass_height,
        thickness=0.012,
        direction=(-1.0, 0.0, 0.0),
        z_center=clear_height * 0.5,
    )
    right_wall_glass = _curved_panel_mesh(
        name="right_wall_glass",
        radius=drum_radius - wall_frame_thickness * 0.55,
        arc_length=glass_arc_length,
        height=glass_height,
        thickness=0.012,
        direction=(1.0, 0.0, 0.0),
        z_center=clear_height * 0.5,
    )

    drum.visual(left_wall_frame, origin=Origin(xyz=(0.0, 0.0, threshold_height)), material=aluminum, name="left_wall_frame")
    drum.visual(right_wall_frame, origin=Origin(xyz=(0.0, 0.0, threshold_height)), material=aluminum, name="right_wall_frame")
    drum.visual(left_wall_glass, origin=Origin(xyz=(0.0, 0.0, threshold_height)), material=glass, name="left_wall_glass")
    drum.visual(right_wall_glass, origin=Origin(xyz=(0.0, 0.0, threshold_height)), material=glass, name="right_wall_glass")

    mullion_radius = 0.045
    mullion_length = clear_height + 0.040
    mullion_z = threshold_height + clear_height * 0.5
    mullion_x = 0.360
    mullion_y = 0.820
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            drum.visual(
                Cylinder(radius=mullion_radius, length=mullion_length),
                origin=Origin(
                    xyz=(
                        x_sign * mullion_x,
                        y_sign * mullion_y,
                        mullion_z,
                    )
                ),
                material=aluminum,
                name=f"mullion_{'left' if x_sign < 0 else 'right'}_{'back' if y_sign < 0 else 'front'}",
            )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Box((wing_length * 2.0, wing_length * 2.0, clear_height)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, clear_height * 0.5)),
    )

    rotor.visual(Cylinder(radius=0.078, length=1.08), origin=Origin(xyz=(0.0, 0.0, 0.54)), material=dark_metal, name="central_short_post")
    rotor.visual(Cylinder(radius=0.028, length=1.15), origin=Origin(xyz=(0.0, 0.0, 1.655)), material=dark_metal, name="upper_spindle")
    rotor.visual(Cylinder(radius=0.095, length=0.060), origin=Origin(xyz=(0.0, 0.0, 0.080)), material=aluminum, name="lower_hub_collar")
    rotor.visual(Cylinder(radius=0.095, length=0.060), origin=Origin(xyz=(0.0, 0.0, 2.235)), material=aluminum, name="upper_hub_collar")

    for sign, wing_name in ((1.0, "forward"), (-1.0, "back")):
        y_center = sign * (wing_length * 0.5)
        tip_center = sign * (wing_length - stile_width * 0.5)
        glass_center = sign * ((wing_length - stile_width - 0.050) * 0.5 + 0.025)
        push_bar_center = sign * (wing_length * 0.46)

        rotor.visual(
            Box((wing_depth, wing_length, rail_height)),
            origin=Origin(xyz=(0.0, y_center, wing_bottom_z + rail_height * 0.5)),
            material=aluminum,
            name=f"wing_{wing_name}_bottom_rail",
        )
        rotor.visual(
            Box((wing_depth, wing_length, rail_height)),
            origin=Origin(xyz=(0.0, y_center, wing_top_z - rail_height * 0.5)),
            material=aluminum,
            name=f"wing_{wing_name}_top_rail",
        )
        rotor.visual(
            Box((wing_depth, stile_width, wing_height)),
            origin=Origin(xyz=(0.0, tip_center, wing_bottom_z + wing_height * 0.5)),
            material=aluminum,
            name=f"wing_{wing_name}_tip_stile",
        )
        rotor.visual(
            Box((0.065, 0.135, 0.340)),
            origin=Origin(xyz=(0.0, sign * 0.068, 0.72)),
            material=aluminum,
            name=f"wing_{wing_name}_mount_block",
        )
        rotor.visual(
            Box((0.024, wing_length - stile_width - 0.050 + glass_overlap * 2.0, wing_height - rail_height * 2.0 + glass_overlap * 2.0)),
            origin=Origin(
                xyz=(
                    0.0,
                    glass_center,
                    wing_bottom_z + rail_height + (wing_height - rail_height * 2.0) * 0.5,
                )
            ),
            material=glass,
            name=f"wing_{wing_name}_glass",
        )
        rotor.visual(
            Box((0.026, 0.320, 0.052)),
            origin=Origin(xyz=(0.0, push_bar_center, wing_bottom_z + wing_height * 0.53)),
            material=dark_metal,
            name=f"wing_{wing_name}_push_rail",
        )

    model.articulation(
        "drum_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=drum,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    drum = object_model.get_part("drum")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("drum_to_rotor")

    ctx.check(
        "revolving door uses a continuous rotor articulation",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )
    ctx.check(
        "rotor spins about the vertical axis",
        spin.axis == (0.0, 0.0, 1.0),
        details=f"axis={spin.axis}",
    )
    ctx.expect_origin_distance(drum, rotor, axes="xy", max_dist=0.001, name="rotor stays centered in the drum")

    rest_tip = None
    quarter_tip = None
    with ctx.pose({spin: 0.0}):
        rest_tip = ctx.part_element_world_aabb(rotor, elem="wing_forward_tip_stile")
    with ctx.pose({spin: pi * 0.5}):
        quarter_tip = ctx.part_element_world_aabb(rotor, elem="wing_forward_tip_stile")

    if rest_tip is not None and quarter_tip is not None:
        rest_center = (
            (rest_tip[0][0] + rest_tip[1][0]) * 0.5,
            (rest_tip[0][1] + rest_tip[1][1]) * 0.5,
        )
        quarter_center = (
            (quarter_tip[0][0] + quarter_tip[1][0]) * 0.5,
            (quarter_tip[0][1] + quarter_tip[1][1]) * 0.5,
        )
        ctx.check(
            "forward wing rotates from the front opening to a side position",
            abs(rest_center[0]) < 0.08 and rest_center[1] > 0.68 and abs(quarter_center[0]) > 0.68 and abs(quarter_center[1]) < 0.08,
            details=f"rest_center={rest_center}, quarter_center={quarter_center}",
        )
        ctx.check(
            "quarter-turn wing tip remains inside the shallow drum radius",
            quarter_tip[1][0] < 0.86,
            details=f"quarter_tip_aabb={quarter_tip}",
        )
    else:
        ctx.fail("wing tip AABB available in key poses", f"rest_tip={rest_tip}, quarter_tip={quarter_tip}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
