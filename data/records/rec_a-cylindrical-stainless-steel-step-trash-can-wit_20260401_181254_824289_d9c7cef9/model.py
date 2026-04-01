from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _revolve_closed_profile(
    profile: list[tuple[float, float]],
    *,
    segments: int = 72,
) -> MeshGeometry:
    points = profile[:-1] if profile and profile[0] == profile[-1] else profile[:]
    if len(points) < 3:
        raise ValueError("Revolved profiles need at least three points.")

    geometry = MeshGeometry()
    rings: list[tuple[str, int | list[int]]] = []

    for radius, z_pos in points:
        if radius <= 1e-6:
            rings.append(("axis", geometry.add_vertex(0.0, 0.0, z_pos)))
            continue

        ring: list[int] = []
        for segment in range(segments):
            angle = math.tau * segment / segments
            ring.append(
                geometry.add_vertex(
                    radius * math.cos(angle),
                    radius * math.sin(angle),
                    z_pos,
                )
            )
        rings.append(("ring", ring))

    for index, current in enumerate(rings):
        nxt = rings[(index + 1) % len(rings)]

        if current[0] == "axis" and nxt[0] == "axis":
            continue

        if current[0] == "axis":
            center_index = int(current[1])
            next_ring = list(nxt[1])
            for segment in range(segments):
                segment_next = (segment + 1) % segments
                geometry.add_face(center_index, next_ring[segment_next], next_ring[segment])
            continue

        if nxt[0] == "axis":
            current_ring = list(current[1])
            center_index = int(nxt[1])
            for segment in range(segments):
                segment_next = (segment + 1) % segments
                geometry.add_face(current_ring[segment], current_ring[segment_next], center_index)
            continue

        current_ring = list(current[1])
        next_ring = list(nxt[1])
        for segment in range(segments):
            segment_next = (segment + 1) % segments
            a0 = current_ring[segment]
            a1 = current_ring[segment_next]
            b0 = next_ring[segment]
            b1 = next_ring[segment_next]
            geometry.add_face(a0, b1, b0)
            geometry.add_face(a0, a1, b1)

    return geometry


def _body_shell_mesh() -> MeshGeometry:
    shell = _revolve_closed_profile(
        [
            (0.0, 0.0),
            (0.122, 0.0),
            (0.150, 0.002),
            (0.158, 0.011),
            (0.155, 0.045),
            (0.155, 0.566),
            (0.156, 0.574),
            (0.155, 0.580),
            (0.1515, 0.579),
            (0.1510, 0.572),
            (0.1510, 0.020),
            (0.146, 0.006),
            (0.0, 0.006),
        ],
        segments=88,
    )
    shell.merge(BoxGeometry((0.012, 0.140, 0.028)).translate(-0.152, 0.0, 0.552))
    shell.merge(BoxGeometry((0.018, 0.020, 0.028)).translate(0.149, 0.062, 0.060))
    shell.merge(BoxGeometry((0.018, 0.020, 0.028)).translate(0.149, -0.062, 0.060))
    return shell


def _lid_shell_mesh() -> MeshGeometry:
    return _revolve_closed_profile(
        [
            (0.0, 0.058),
            (0.046, 0.054),
            (0.108, 0.039),
            (0.148, 0.022),
            (0.160, 0.006),
            (0.160, -0.022),
            (0.152, -0.028),
            (0.152, -0.011),
            (0.101, 0.014),
            (0.043, 0.028),
            (0.010, 0.036),
            (0.008, 0.054),
        ],
        segments=88,
    )


def build_object_model() -> ArticulatedObject:
    body_radius = 0.155
    body_height = 0.580
    hinge_span = 0.138

    model = ArticulatedObject(name="step_trash_can")

    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    pedal_metal = model.material("pedal_metal", rgba=(0.28, 0.29, 0.31, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_shell_mesh(), "body_shell"),
        material=stainless,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.586),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.0, 0.293)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_lid_shell_mesh(), "lid_shell"),
        origin=Origin(xyz=(0.161, 0.0, 0.031)),
        material=stainless,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.007, length=hinge_span),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.018, 0.140, 0.008)),
        origin=Origin(xyz=(0.008, 0.0, 0.004)),
        material=stainless,
        name="hinge_leaf",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.162, length=0.070),
        mass=1.2,
        origin=Origin(xyz=(0.161, 0.0, 0.040)),
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.004, length=0.126),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pedal_metal,
        name="pivot_rod",
    )
    pedal.visual(
        Box((0.050, 0.010, 0.012)),
        origin=Origin(xyz=(0.025, 0.055, -0.005)),
        material=pedal_metal,
        name="left_arm",
    )
    pedal.visual(
        Box((0.050, 0.010, 0.012)),
        origin=Origin(xyz=(0.025, -0.055, -0.005)),
        material=pedal_metal,
        name="right_arm",
    )
    pedal.visual(
        Cylinder(radius=0.006, length=0.132),
        origin=Origin(xyz=(0.052, 0.0, -0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="pedal_bar",
    )
    pedal.inertial = Inertial.from_geometry(
        Box((0.110, 0.140, 0.022)),
        mass=0.18,
        origin=Origin(xyz=(0.040, 0.0, -0.006)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-body_radius - 0.011, 0.0, body_height - 0.002)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(body_radius + 0.007, 0.0, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=0.42,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_joint = object_model.get_articulation("body_to_pedal")
    lid_shell = lid.get_visual("lid_shell")
    body_shell = body.get_visual("body_shell")
    hinge_barrel = lid.get_visual("hinge_barrel")
    pivot_rod = pedal.get_visual("pivot_rod")
    pedal_bar = pedal.get_visual("pedal_bar")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_overlap(
        lid,
        body,
        elem_a=lid_shell,
        elem_b=body_shell,
        axes="xy",
        min_overlap=0.28,
        name="closed lid covers the cylindrical body",
    )
    ctx.expect_gap(
        body,
        lid,
        axis="x",
        min_gap=0.0,
        max_gap=0.012,
        positive_elem=body_shell,
        negative_elem=hinge_barrel,
        name="lid hinge sits just behind the rear rim",
    )
    ctx.expect_origin_gap(
        pedal,
        body,
        axis="x",
        min_gap=0.145,
        max_gap=0.170,
        name="pedal pivot sits at the front of the can",
    )
    ctx.expect_origin_gap(
        pedal,
        body,
        axis="z",
        min_gap=0.045,
        max_gap=0.075,
        name="pedal pivot is mounted low on the body",
    )
    ctx.expect_gap(
        pedal,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem=pivot_rod,
        negative_elem=body_shell,
        name="pedal pivot seats against the front body brackets",
    )

    lid_closed_aabb = ctx.part_element_world_aabb(lid, elem=lid_shell)
    with ctx.pose({lid_hinge: 1.10}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem=lid_shell)
    ctx.check(
        "lid opens upward from the rear hinge",
        lid_closed_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_closed_aabb[1][2] + 0.09,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    pedal_rest_aabb = ctx.part_element_world_aabb(pedal, elem=pedal_bar)
    with ctx.pose({pedal_joint: 0.36}):
        pedal_pressed_aabb = ctx.part_element_world_aabb(pedal, elem=pedal_bar)
    ctx.check(
        "pedal bar rotates downward when pressed",
        pedal_rest_aabb is not None
        and pedal_pressed_aabb is not None
        and pedal_pressed_aabb[0][2] < pedal_rest_aabb[0][2] - 0.012,
        details=f"rest={pedal_rest_aabb}, pressed={pedal_pressed_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
