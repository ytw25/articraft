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
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _circle_loop(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _annulus_mesh(name: str, *, outer_radius: float, inner_radius: float, height: float):
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            _circle_loop(outer_radius, segments=72),
            [_circle_loop(inner_radius, segments=72)],
            height=height,
            center=True,
        ),
    )


def _build_canister_shell_mesh():
    canister_geom = LatheGeometry.from_shell_profiles(
        [
            (0.091, 0.008),
            (0.091, 0.028),
            (0.090, 0.072),
            (0.089, 0.120),
            (0.088, 0.154),
        ],
        [
            (0.084, 0.008),
            (0.084, 0.028),
            (0.083, 0.072),
            (0.082, 0.120),
            (0.081, 0.154),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    canister_geom.merge(
        CylinderGeometry(radius=0.082, height=0.004, radial_segments=72).translate(0.0, 0.0, 0.154)
    )
    canister_geom.merge(BoxGeometry((0.030, 0.008, 0.020)).translate(0.0, 0.081, 0.032))
    canister_geom.merge(BoxGeometry((0.030, 0.008, 0.020)).translate(0.0, -0.081, 0.032))
    return canister_geom


def _build_trim_face_mesh():
    trim_face_geom = ExtrudeWithHolesGeometry(
        _circle_loop(0.110, segments=72),
        [_circle_loop(0.090, segments=72)],
        height=0.004,
        center=True,
    )
    trim_face_geom.translate(0.0, 0.0, -0.010)
    trim_face_geom.merge(
        TorusGeometry(radius=0.105, tube=0.004, radial_segments=18, tubular_segments=72).translate(
            0.0,
            0.0,
            -0.010,
        )
    )
    return trim_face_geom


def _build_trim_inner_mesh():
    trim_inner_geom = LatheGeometry.from_shell_profiles(
        [
            (0.090, -0.008),
            (0.087, -0.002),
            (0.083, 0.003),
            (0.078, 0.008),
            (0.073, 0.013),
            (0.068, 0.017),
            (0.064, 0.021),
        ],
        [
            (0.084, -0.008),
            (0.081, -0.002),
            (0.077, 0.003),
            (0.072, 0.008),
            (0.067, 0.013),
            (0.062, 0.017),
            (0.058, 0.021),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    trim_inner_geom.merge(BoxGeometry((0.016, 0.006, 0.004)).translate(0.0, 0.080, 0.020))
    trim_inner_geom.merge(BoxGeometry((0.016, 0.006, 0.004)).translate(0.0, -0.080, 0.020))
    return trim_inner_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_can_light_housing", assets=ASSETS)

    ceiling_white = model.material("ceiling_white", rgba=(0.95, 0.95, 0.93, 1.0))
    trim_white = model.material("trim_white", rgba=(0.97, 0.97, 0.96, 1.0))
    reflector_aluminum = model.material("reflector_aluminum", rgba=(0.83, 0.85, 0.87, 1.0))
    galvanized = model.material("galvanized", rgba=(0.63, 0.66, 0.69, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    ceramic_gray = model.material("ceramic_gray", rgba=(0.72, 0.71, 0.68, 1.0))

    ceiling = model.part("ceiling")
    ceiling.visual(
        _save_mesh(
            "ceiling_cutout_panel.obj",
            ExtrudeWithHolesGeometry(
                [
                    (-0.190, -0.190),
                    (0.190, -0.190),
                    (0.190, 0.190),
                    (-0.190, 0.190),
                ],
                [_circle_loop(0.095, segments=72)],
                height=0.016,
                center=True,
            ),
        ),
        material=ceiling_white,
        name="ceiling_panel",
    )
    ceiling.inertial = Inertial.from_geometry(
        Box((0.38, 0.38, 0.016)),
        mass=2.8,
        origin=Origin(),
    )

    canister = model.part("canister_body")
    canister.visual(
        _save_mesh("canister_shell.obj", _build_canister_shell_mesh()),
        material=galvanized,
        name="canister_shell",
    )
    canister.inertial = Inertial.from_geometry(
        Cylinder(radius=0.091, length=0.148),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.082)),
    )

    trim = model.part("trim_ring")
    trim.visual(
        _save_mesh("trim_face.obj", _build_trim_face_mesh()),
        material=trim_white,
        name="trim_face",
    )
    trim.visual(
        _save_mesh("trim_inner_assembly.obj", _build_trim_inner_mesh()),
        material=reflector_aluminum,
        name="trim_inner_assembly",
    )
    trim.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.034),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    reflector = model.part("reflector_insert")
    reflector.visual(
        _save_mesh(
            "reflector_insert_shell.obj",
            LatheGeometry.from_shell_profiles(
                [
                    (0.072, 0.018),
                    (0.070, 0.036),
                    (0.064, 0.060),
                    (0.053, 0.090),
                    (0.040, 0.116),
                    (0.034, 0.124),
                ],
                [
                    (0.068, 0.018),
                    (0.066, 0.036),
                    (0.060, 0.060),
                    (0.049, 0.090),
                    (0.036, 0.116),
                    (0.030, 0.124),
                ],
                segments=72,
                start_cap="flat",
                end_cap="flat",
            ),
        ),
        material=reflector_aluminum,
        name="reflector_shell",
    )
    reflector.visual(
        Cylinder(radius=0.032, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=reflector_aluminum,
        name="socket_seat",
    )
    reflector.inertial = Inertial.from_geometry(
        Cylinder(radius=0.076, length=0.110),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, 0.072)),
    )

    socket = model.part("socket_shell")
    socket.visual(
        Cylinder(radius=0.021, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
        material=ceramic_gray,
        name="socket_body",
    )
    socket.visual(
        Cylinder(radius=0.032, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=ceramic_gray,
        name="socket_flange",
    )
    socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.034),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
    )

    left_bracket = model.part("left_spring_bracket")
    left_bracket.visual(
        Box((0.028, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.084, 0.032)),
        material=spring_steel,
        name="mount_pad",
    )
    left_bracket.visual(
        Box((0.014, 0.022, 0.002)),
        origin=Origin(xyz=(0.0, 0.081, 0.024), rpy=(0.55, 0.0, 0.0)),
        material=spring_steel,
        name="spring_leaf",
    )
    left_bracket.visual(
        Box((0.016, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.080, 0.020)),
        material=spring_steel,
        name="hook_pad",
    )
    left_bracket.inertial = Inertial.from_geometry(
        Box((0.028, 0.022, 0.030)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.076, 0.024)),
    )

    right_bracket = model.part("right_spring_bracket")
    right_bracket.visual(
        Box((0.028, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.084, 0.032)),
        material=spring_steel,
        name="mount_pad",
    )
    right_bracket.visual(
        Box((0.014, 0.022, 0.002)),
        origin=Origin(xyz=(0.0, -0.081, 0.024), rpy=(-0.55, 0.0, 0.0)),
        material=spring_steel,
        name="spring_leaf",
    )
    right_bracket.visual(
        Box((0.016, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, -0.080, 0.020)),
        material=spring_steel,
        name="hook_pad",
    )
    right_bracket.inertial = Inertial.from_geometry(
        Box((0.028, 0.022, 0.030)),
        mass=0.05,
        origin=Origin(xyz=(0.0, -0.076, 0.024)),
    )

    junction_box = model.part("junction_box_cover")
    junction_box.visual(
        Box((0.022, 0.026, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=galvanized,
        name="mount_plate",
    )
    junction_box.visual(
        Box((0.066, 0.046, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=galvanized,
        name="junction_box_body",
    )
    junction_box.visual(
        Box((0.074, 0.054, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=galvanized,
        name="junction_box_cover_plate",
    )
    junction_box.inertial = Inertial.from_geometry(
        Box((0.074, 0.054, 0.031)),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
    )

    model.articulation(
        "canister_to_ceiling",
        ArticulationType.FIXED,
        parent=canister,
        child=ceiling,
        origin=Origin(),
    )
    model.articulation(
        "canister_to_trim",
        ArticulationType.FIXED,
        parent=canister,
        child=trim,
        origin=Origin(),
    )
    model.articulation(
        "canister_to_reflector",
        ArticulationType.FIXED,
        parent=canister,
        child=reflector,
        origin=Origin(),
    )
    model.articulation(
        "reflector_to_socket",
        ArticulationType.FIXED,
        parent=reflector,
        child=socket,
        origin=Origin(),
    )
    model.articulation(
        "canister_to_left_bracket",
        ArticulationType.FIXED,
        parent=canister,
        child=left_bracket,
        origin=Origin(),
    )
    model.articulation(
        "canister_to_right_bracket",
        ArticulationType.FIXED,
        parent=canister,
        child=right_bracket,
        origin=Origin(),
    )
    model.articulation(
        "canister_to_junction_box",
        ArticulationType.FIXED,
        parent=canister,
        child=junction_box,
        origin=Origin(xyz=(0.060, 0.0, 0.156)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canister = object_model.get_part("canister_body")
    ceiling = object_model.get_part("ceiling")
    trim = object_model.get_part("trim_ring")
    reflector = object_model.get_part("reflector_insert")
    socket = object_model.get_part("socket_shell")
    left_bracket = object_model.get_part("left_spring_bracket")
    right_bracket = object_model.get_part("right_spring_bracket")
    junction_box = object_model.get_part("junction_box_cover")

    ceiling_panel = ceiling.get_visual("ceiling_panel")
    trim_face = trim.get_visual("trim_face")
    trim_inner = trim.get_visual("trim_inner_assembly")
    reflector_shell = reflector.get_visual("reflector_shell")
    socket_seat = reflector.get_visual("socket_seat")
    socket_flange = socket.get_visual("socket_flange")
    left_mount_pad = left_bracket.get_visual("mount_pad")
    right_mount_pad = right_bracket.get_visual("mount_pad")
    left_hook = left_bracket.get_visual("hook_pad")
    right_hook = right_bracket.get_visual("hook_pad")
    canister_shell = canister.get_visual("canister_shell")
    junction_mount = junction_box.get_visual("mount_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.15)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.allow_overlap(
        socket,
        canister,
        reason="socket shell sits inside the rear cavity of the thin-walled canister; broad shell overlap sensing is conservative here",
    )
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance(canister, ceiling, axes="xy", max_dist=0.001)
    ctx.expect_contact(trim, ceiling, elem_a=trim_face, elem_b=ceiling_panel)
    ctx.expect_gap(
        ceiling,
        trim,
        axis="z",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem=ceiling_panel,
        negative_elem=trim_face,
    )
    ctx.expect_overlap(trim, ceiling, axes="xy", min_overlap=0.04, elem_a=trim_face, elem_b=ceiling_panel)
    ctx.expect_contact(left_bracket, canister, elem_a=left_mount_pad, elem_b=canister_shell)
    ctx.expect_contact(right_bracket, canister, elem_a=right_mount_pad, elem_b=canister_shell)
    ctx.expect_contact(trim, left_bracket, elem_a=trim_inner, elem_b=left_hook)
    ctx.expect_contact(trim, right_bracket, elem_a=trim_inner, elem_b=right_hook)
    ctx.expect_within(trim, canister, axes="xy", inner_elem=trim_inner, outer_elem=canister_shell)
    ctx.expect_within(
        reflector,
        canister,
        axes="xy",
        inner_elem=reflector_shell,
        outer_elem=canister_shell,
    )
    ctx.expect_gap(
        socket,
        reflector,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=socket_flange,
        negative_elem=socket_seat,
    )
    ctx.expect_overlap(socket, reflector, axes="xy", min_overlap=0.02, elem_a=socket_flange, elem_b=socket_seat)
    ctx.expect_gap(
        junction_box,
        canister,
        axis="z",
        max_gap=0.001,
        max_penetration=0.01,
        positive_elem=junction_mount,
        negative_elem=canister_shell,
    )
    ctx.expect_overlap(
        junction_box,
        canister,
        axes="xy",
        min_overlap=0.02,
        elem_a=junction_mount,
        elem_b=canister_shell,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
