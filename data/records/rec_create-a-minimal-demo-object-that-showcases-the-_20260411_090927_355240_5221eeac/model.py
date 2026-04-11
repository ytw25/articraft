from __future__ import annotations

from sdk import (
    ArticulatedObject,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    FanRotorShroud,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

BASE_SIZE = (0.74, 0.34, 0.012)
BASE_TOP_Z = BASE_SIZE[2]
STAND_HEIGHT = 0.022
MOUNT_Z = BASE_TOP_Z + STAND_HEIGHT - 0.0006

TOP_ROW_Y = 0.092
BOTTOM_ROW_Y = -0.075
TOP_ROW_XS = (-0.24, -0.08, 0.08, 0.24)
BOTTOM_ROW_XS = (-0.12, 0.12)


def _add_showcase_mesh(
    part,
    geometry,
    logical_name: str,
    visual_name: str,
    material,
    xyz: tuple[float, float, float],
) -> None:
    part.visual(
        mesh_from_geometry(geometry, logical_name),
        origin=Origin(xyz=xyz),
        material=material,
        name=visual_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fan_geometry_showcase")

    base_finish = model.material("base_finish", rgba=(0.13, 0.14, 0.16, 1.0))
    stand_finish = model.material("stand_finish", rgba=(0.28, 0.30, 0.34, 1.0))
    rotor_blue = model.material("rotor_blue", rgba=(0.33, 0.48, 0.72, 1.0))
    rotor_red = model.material("rotor_red", rgba=(0.72, 0.37, 0.29, 1.0))
    rotor_teal = model.material("rotor_teal", rgba=(0.19, 0.56, 0.54, 1.0))
    rotor_gold = model.material("rotor_gold", rgba=(0.72, 0.62, 0.26, 1.0))
    blower_dark = model.material("blower_dark", rgba=(0.38, 0.40, 0.45, 1.0))
    blower_light = model.material("blower_light", rgba=(0.60, 0.63, 0.67, 1.0))

    showcase = model.part("showcase")
    showcase.visual(
        Box(BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] * 0.5)),
        material=base_finish,
        name="display_base",
    )

    top_positions = [(x, TOP_ROW_Y) for x in TOP_ROW_XS]
    bottom_positions = [(x, BOTTOM_ROW_Y) for x in BOTTOM_ROW_XS]
    for index, (x_pos, y_pos) in enumerate(top_positions + bottom_positions):
        showcase.visual(
            Cylinder(radius=0.013 if index < 4 else 0.016, length=STAND_HEIGHT),
            origin=Origin(xyz=(x_pos, y_pos, BASE_TOP_Z + STAND_HEIGHT * 0.5)),
            material=stand_finish,
            name=f"stand_{index}",
        )

    axial_variants = [
        (
            "axial_straight",
            FanRotorGeometry(
                0.064,
                0.020,
                7,
                thickness=0.010,
                blade_pitch_deg=26.0,
                blade_sweep_deg=10.0,
                blade=FanRotorBlade(
                    shape="straight",
                    tip_pitch_deg=15.0,
                    camber=0.08,
                ),
                hub=FanRotorHub(
                    style="flat",
                    bore_diameter=0.0045,
                ),
                center=False,
            ),
            rotor_blue,
            (TOP_ROW_XS[0], TOP_ROW_Y, MOUNT_Z),
        ),
        (
            "axial_scimitar_shrouded",
            FanRotorGeometry(
                0.070,
                0.020,
                7,
                thickness=0.010,
                blade_pitch_deg=31.0,
                blade_sweep_deg=24.0,
                blade=FanRotorBlade(
                    shape="scimitar",
                    tip_pitch_deg=12.0,
                    camber=0.16,
                ),
                hub=FanRotorHub(
                    style="spinner",
                    bore_diameter=0.005,
                ),
                shroud=FanRotorShroud(
                    thickness=0.004,
                    depth=0.012,
                    clearance=0.0015,
                    lip_depth=0.002,
                ),
                center=False,
            ),
            rotor_red,
            (TOP_ROW_XS[1], TOP_ROW_Y, MOUNT_Z),
        ),
        (
            "axial_broad",
            FanRotorGeometry(
                0.074,
                0.022,
                5,
                thickness=0.012,
                blade_pitch_deg=35.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(
                    shape="broad",
                    tip_pitch_deg=18.0,
                    camber=0.10,
                ),
                hub=FanRotorHub(style="domed"),
                center=False,
            ),
            rotor_teal,
            (TOP_ROW_XS[2], TOP_ROW_Y, MOUNT_Z),
        ),
        (
            "axial_narrow",
            FanRotorGeometry(
                0.058,
                0.016,
                9,
                thickness=0.008,
                blade_pitch_deg=29.0,
                blade_sweep_deg=20.0,
                blade=FanRotorBlade(
                    shape="narrow",
                    tip_pitch_deg=10.0,
                    camber=0.12,
                    tip_clearance=0.002,
                ),
                hub=FanRotorHub(
                    style="capped",
                    rear_collar_height=0.0015,
                    bore_diameter=0.0035,
                ),
                center=False,
            ),
            rotor_gold,
            (TOP_ROW_XS[3], TOP_ROW_Y, MOUNT_Z),
        ),
    ]

    blower_variants = [
        (
            "blower_dense",
            BlowerWheelGeometry(
                0.082,
                0.040,
                0.055,
                20,
                blade_thickness=0.0035,
                blade_sweep_deg=28.0,
                center=False,
            ),
            blower_dark,
            (BOTTOM_ROW_XS[0], BOTTOM_ROW_Y, MOUNT_Z),
        ),
        (
            "blower_light",
            BlowerWheelGeometry(
                0.074,
                0.036,
                0.048,
                14,
                blade_thickness=0.0040,
                blade_sweep_deg=18.0,
                shroud=False,
                center=False,
            ),
            blower_light,
            (BOTTOM_ROW_XS[1], BOTTOM_ROW_Y, MOUNT_Z),
        ),
    ]

    for logical_name, geometry, material, xyz in axial_variants + blower_variants:
        _add_showcase_mesh(
            showcase,
            geometry,
            logical_name=logical_name,
            visual_name=logical_name,
            material=material,
            xyz=xyz,
        )

    showcase.inertial = Inertial.from_geometry(
        Box((0.74, 0.34, 0.14)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    showcase = object_model.get_part("showcase")
    ctx.check("showcase_part_present", showcase is not None, "Expected a single showcase part.")
    if showcase is None:
        return ctx.report()

    expected_visuals = [
        "display_base",
        "axial_straight",
        "axial_scimitar_shrouded",
        "axial_broad",
        "axial_narrow",
        "blower_dense",
        "blower_light",
    ]
    for visual_name in expected_visuals:
        ctx.check(
            f"{visual_name}_present",
            showcase.get_visual(visual_name) is not None,
            f"Expected showcase visual {visual_name}.",
        )
    return ctx.report()


object_model = build_object_model()
