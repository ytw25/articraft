from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Cylinder,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleMounts,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)

BASE_SIZE = (0.84, 0.44, 0.014)
BASE_TOP_Z = BASE_SIZE[2]
STAND_HEIGHT = 0.018
MOUNT_Z = BASE_TOP_Z + STAND_HEIGHT

TOP_ROW_Y = 0.115
BOTTOM_ROW_Y = -0.105
TOP_ROW_XS = (-0.24, 0.0, 0.24)
BOTTOM_ROW_XS = (-0.24, 0.0, 0.24)


def _add_grille(part, logical_name: str, geometry, material, xyz: tuple[float, float, float]) -> None:
    part.visual(
        mesh_from_geometry(geometry, logical_name),
        origin=Origin(xyz=xyz),
        material=material,
        name=logical_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vent_grille_showcase")

    base_finish = model.material("base_finish", rgba=(0.14, 0.15, 0.17, 1.0))
    stand_finish = model.material("stand_finish", rgba=(0.30, 0.32, 0.36, 1.0))
    warm_white = model.material("warm_white", rgba=(0.86, 0.84, 0.80, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.68, 0.71, 0.74, 1.0))
    olive = model.material("olive", rgba=(0.47, 0.53, 0.39, 1.0))
    brass = model.material("brass", rgba=(0.68, 0.58, 0.30, 1.0))
    cool_white = model.material("cool_white", rgba=(0.82, 0.86, 0.89, 1.0))
    charcoal = model.material("charcoal", rgba=(0.34, 0.36, 0.39, 1.0))

    showcase = model.part("showcase")
    showcase.visual(
        Box(BASE_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_SIZE[2] * 0.5)),
        material=base_finish,
        name="display_base",
    )

    stand_positions = [(x, TOP_ROW_Y) for x in TOP_ROW_XS] + [(x, BOTTOM_ROW_Y) for x in BOTTOM_ROW_XS]
    for index, (x_pos, y_pos) in enumerate(stand_positions):
        showcase.visual(
            Cylinder(radius=0.015, length=STAND_HEIGHT),
            origin=Origin(xyz=(x_pos, y_pos, BASE_TOP_Z + STAND_HEIGHT * 0.5)),
            material=stand_finish,
            name=f"stand_{index}",
        )

    variants = [
        (
            "register_flush",
            VentGrilleGeometry(
                (0.18, 0.10),
                frame=0.012,
                face_thickness=0.004,
                duct_depth=0.026,
                duct_wall=0.003,
                slat_pitch=0.018,
                slat_width=0.009,
                slat_angle_deg=32.0,
                corner_radius=0.006,
                center=False,
            ),
            warm_white,
            (TOP_ROW_XS[0], TOP_ROW_Y, MOUNT_Z),
        ),
        (
            "register_airfoil",
            VentGrilleGeometry(
                (0.18, 0.10),
                frame=0.012,
                face_thickness=0.004,
                duct_depth=0.026,
                duct_wall=0.003,
                slat_pitch=0.018,
                slat_width=0.009,
                slat_angle_deg=28.0,
                corner_radius=0.006,
                slats=VentGrilleSlats(
                    profile="airfoil",
                    direction="down",
                    divider_count=2,
                    divider_width=0.004,
                ),
                frame_profile=VentGrilleFrame(style="beveled", depth=0.0012),
                mounts=VentGrilleMounts(style="holes", inset=0.010, hole_diameter=0.0032),
                sleeve=VentGrilleSleeve(style="full"),
                center=False,
            ),
            soft_gray,
            (TOP_ROW_XS[1], TOP_ROW_Y, MOUNT_Z),
        ),
        (
            "register_face_only",
            VentGrilleGeometry(
                (0.17, 0.11),
                frame=0.011,
                face_thickness=0.004,
                duct_depth=0.024,
                duct_wall=0.003,
                slat_pitch=0.017,
                slat_width=0.008,
                slat_angle_deg=18.0,
                corner_radius=0.008,
                slats=VentGrilleSlats(
                    profile="boxed",
                    direction="up",
                    inset=0.0008,
                ),
                frame_profile=VentGrilleFrame(style="radiused", depth=0.0010),
                sleeve=VentGrilleSleeve(style="none"),
                center=False,
            ),
            olive,
            (TOP_ROW_XS[2], TOP_ROW_Y, MOUNT_Z),
        ),
        (
            "register_divided",
            VentGrilleGeometry(
                (0.21, 0.10),
                frame=0.014,
                face_thickness=0.004,
                duct_depth=0.022,
                duct_wall=0.003,
                slat_pitch=0.016,
                slat_width=0.0075,
                slat_angle_deg=26.0,
                slats=VentGrilleSlats(
                    profile="flat",
                    direction="down",
                    divider_count=3,
                    divider_width=0.005,
                ),
                frame_profile=VentGrilleFrame(style="beveled", depth=0.0011),
                sleeve=VentGrilleSleeve(style="short"),
                center=False,
            ),
            brass,
            (BOTTOM_ROW_XS[0], BOTTOM_ROW_Y, MOUNT_Z),
        ),
        (
            "tower_vent",
            VentGrilleGeometry(
                (0.10, 0.24),
                frame=0.010,
                face_thickness=0.0036,
                duct_depth=0.018,
                duct_wall=0.0026,
                slat_pitch=0.014,
                slat_width=0.0068,
                slat_angle_deg=22.0,
                corner_radius=0.012,
                slats=VentGrilleSlats(
                    profile="airfoil",
                    direction="down",
                    divider_count=1,
                    divider_width=0.0035,
                ),
                frame_profile=VentGrilleFrame(style="flush"),
                mounts=VentGrilleMounts(style="holes", inset=0.009, hole_diameter=0.0030),
                sleeve=VentGrilleSleeve(style="short", depth=0.012),
                center=False,
            ),
            cool_white,
            (BOTTOM_ROW_XS[1], BOTTOM_ROW_Y, MOUNT_Z),
        ),
        (
            "industrial_grille",
            VentGrilleGeometry(
                (0.18, 0.12),
                frame=0.014,
                face_thickness=0.005,
                duct_depth=0.030,
                duct_wall=0.0035,
                slat_pitch=0.020,
                slat_width=0.010,
                slat_angle_deg=35.0,
                corner_radius=0.004,
                slats=VentGrilleSlats(
                    profile="boxed",
                    direction="down",
                    inset=0.0015,
                    divider_count=2,
                    divider_width=0.006,
                ),
                frame_profile=VentGrilleFrame(style="beveled", depth=0.0015),
                mounts=VentGrilleMounts(style="holes", inset=0.012, hole_diameter=0.004),
                sleeve=VentGrilleSleeve(style="full", depth=0.032, wall=0.004),
                center=False,
            ),
            charcoal,
            (BOTTOM_ROW_XS[2], BOTTOM_ROW_Y, MOUNT_Z),
        ),
    ]

    for logical_name, geometry, material, xyz in variants:
        _add_grille(showcase, logical_name, geometry, material, xyz)

    showcase.inertial = Inertial.from_geometry(
        Box((0.84, 0.44, 0.16)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
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
        "register_flush",
        "register_airfoil",
        "register_face_only",
        "register_divided",
        "tower_vent",
        "industrial_grille",
    ]
    for visual_name in expected_visuals:
        ctx.check(
            f"{visual_name}_present",
            showcase.get_visual(visual_name) is not None,
            f"Expected showcase visual {visual_name}.",
        )
    return ctx.report()


object_model = build_object_model()
