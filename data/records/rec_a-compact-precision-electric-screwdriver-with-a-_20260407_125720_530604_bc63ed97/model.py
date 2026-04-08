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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hex_profile(flat_diameter: float) -> list[tuple[float, float]]:
    radius = flat_diameter / math.sqrt(3.0)
    return [
        (radius * math.cos(index * math.pi / 3.0), radius * math.sin(index * math.pi / 3.0))
        for index in range(6)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_electric_screwdriver")

    body_aluminum = model.material("body_aluminum", rgba=(0.66, 0.69, 0.73, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.82, 1.0))

    body = model.part("body")

    body_length = 0.144
    body_radius = 0.0086
    body_profile = [
        (0.0, 0.0000),
        (0.0038, 0.0014),
        (0.0068, 0.0042),
        (body_radius, 0.0085),
        (body_radius, 0.1080),
        (0.0080, 0.1220),
        (0.0070, 0.1300),
        (0.0060, 0.1360),
        (0.0054, body_length),
    ]
    body_mesh = mesh_from_geometry(
        LatheGeometry(body_profile, segments=56).rotate_y(math.pi / 2.0),
        "precision_driver_body_shell",
    )
    body.visual(
        body_mesh,
        origin=Origin(xyz=(-body_length * 0.5, 0.0, 0.0)),
        material=body_aluminum,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.0054, length=0.014),
        origin=Origin(xyz=(0.079, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="nose_sleeve",
    )
    body.visual(
        Box((0.042, 0.0042, 0.014)),
        origin=Origin(xyz=(0.010, 0.0095, 0.0)),
        material=dark_trim,
        name="control_pad",
    )
    body.visual(
        Box((0.008, 0.003, 0.006)),
        origin=Origin(xyz=(-0.067, 0.0078, 0.0056)),
        material=dark_trim,
        name="rear_hinge_block",
    )
    body.visual(
        Box((0.0032, 0.0112, 0.0094)),
        origin=Origin(xyz=(-0.0704, 0.0, -0.0004)),
        material=dark_trim,
        name="charging_port_surround",
    )
    body.visual(
        Cylinder(radius=0.00155, length=0.0026),
        origin=Origin(xyz=(-0.0728, -0.0037, 0.0051), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="charging_hinge_lug_left",
    )
    body.visual(
        Cylinder(radius=0.00155, length=0.0026),
        origin=Origin(xyz=(-0.0728, 0.0037, 0.0051), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="charging_hinge_lug_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.158, 0.022, 0.022)),
        mass=0.19,
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
    )

    bit_socket = model.part("bit_socket")
    socket_hex = mesh_from_geometry(
        ExtrudeGeometry.centered(_hex_profile(0.0054), 0.010).rotate_y(math.pi / 2.0),
        "precision_driver_bit_socket_hex",
    )
    bit_socket.visual(
        Cylinder(radius=0.0022, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="drive_shaft",
    )
    bit_socket.visual(
        socket_hex,
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material=steel,
        name="socket_collet",
    )
    bit_socket.visual(
        Cylinder(radius=0.0032, length=0.006),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="socket_tip",
    )
    bit_socket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0035, length=0.024),
        mass=0.018,
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    control_ring = model.part("control_ring")
    ring_center_x = 0.028
    ring_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0110, 0.0000),
                (0.0116, 0.0016),
                (0.0119, 0.0184),
                (0.0110, 0.0200),
            ],
            [
                (0.00915, 0.0000),
                (0.00915, 0.0200),
            ],
            segments=56,
        ).rotate_y(math.pi / 2.0),
        "precision_driver_control_ring_shell",
    )
    control_ring.visual(
        ring_shell,
        origin=Origin(xyz=(ring_center_x - 0.010, 0.0, 0.0)),
        material=dark_trim,
        name="ring_shell",
    )
    ring_ridge_radius = 0.0116
    for ridge_index in range(18):
        angle = ridge_index * (2.0 * math.pi / 18.0)
        control_ring.visual(
            Box((0.017, 0.0011, 0.0016)),
            origin=Origin(
                xyz=(
                    ring_center_x,
                    ring_ridge_radius * math.cos(angle),
                    ring_ridge_radius * math.sin(angle),
                ),
                rpy=(angle, 0.0, 0.0),
            ),
            material=steel,
            name=f"knurl_ridge_{ridge_index}",
        )
    control_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0118, length=0.020),
        mass=0.014,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    forward_button = model.part("forward_button")
    forward_button.visual(
        Cylinder(radius=0.0022, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="forward_button_stem",
    )
    forward_button.visual(
        Cylinder(radius=0.0042, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="forward_button_cap",
    )
    forward_button.visual(
        Box((0.0036, 0.0008, 0.0012)),
        origin=Origin(xyz=(0.0, 0.00325, 0.00155)),
        material=steel,
        name="forward_button_mark",
    )
    forward_button.inertial = Inertial.from_geometry(
        Box((0.009, 0.0048, 0.009)),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0020, 0.0)),
    )

    reverse_button = model.part("reverse_button")
    reverse_button.visual(
        Cylinder(radius=0.0022, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="reverse_button_stem",
    )
    reverse_button.visual(
        Cylinder(radius=0.0042, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="reverse_button_cap",
    )
    reverse_button.visual(
        Box((0.0036, 0.0008, 0.0012)),
        origin=Origin(xyz=(0.0, 0.00325, 0.00155)),
        material=steel,
        name="reverse_button_mark_a",
    )
    reverse_button.visual(
        Box((0.0036, 0.0008, 0.0012)),
        origin=Origin(xyz=(0.0, 0.00325, -0.00155)),
        material=steel,
        name="reverse_button_mark_b",
    )
    reverse_button.inertial = Inertial.from_geometry(
        Box((0.009, 0.0048, 0.009)),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0020, 0.0)),
    )

    charging_cover = model.part("charging_cover")
    charging_cover.visual(
        Cylinder(radius=0.0013, length=0.0048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cover_hinge_barrel",
    )
    charging_cover.visual(
        Box((0.0024, 0.0102, 0.0128)),
        origin=Origin(xyz=(-0.0012, 0.0, -0.0064)),
        material=dark_trim,
        name="cover_panel",
    )
    charging_cover.visual(
        Box((0.0038, 0.0048, 0.0016)),
        origin=Origin(xyz=(-0.0016, 0.0, -0.0130)),
        material=steel,
        name="cover_grip_lip",
    )
    charging_cover.inertial = Inertial.from_geometry(
        Box((0.005, 0.011, 0.015)),
        mass=0.004,
        origin=Origin(xyz=(-0.0012, 0.0, -0.0060)),
    )

    model.articulation(
        "body_to_bit_socket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=bit_socket,
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=20.0),
    )
    model.articulation(
        "body_to_control_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=control_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )
    model.articulation(
        "body_to_forward_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=forward_button,
        origin=Origin(xyz=(0.017, 0.0116, 0.0039)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=-0.0012, upper=0.0),
    )
    model.articulation(
        "body_to_reverse_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=reverse_button,
        origin=Origin(xyz=(-0.001, 0.0116, -0.0039)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.04, lower=-0.0012, upper=0.0),
    )
    model.articulation(
        "body_to_charging_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=charging_cover,
        origin=Origin(xyz=(-0.0728, 0.0, 0.0051)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.12,
            velocity=2.0,
            lower=0.0,
            upper=1.7,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    bit_socket = object_model.get_part("bit_socket")
    control_ring = object_model.get_part("control_ring")
    forward_button = object_model.get_part("forward_button")
    reverse_button = object_model.get_part("reverse_button")
    charging_cover = object_model.get_part("charging_cover")
    socket_joint = object_model.get_articulation("body_to_bit_socket")
    forward_button_joint = object_model.get_articulation("body_to_forward_button")
    reverse_button_joint = object_model.get_articulation("body_to_reverse_button")
    charging_cover_joint = object_model.get_articulation("body_to_charging_cover")

    ctx.expect_overlap(
        control_ring,
        body,
        axes="xz",
        elem_a="ring_shell",
        elem_b="body_shell",
        min_overlap=0.010,
        name="control ring sits concentrically around the body",
    )
    ctx.expect_gap(
        bit_socket,
        body,
        axis="x",
        min_gap=-0.001,
        max_gap=0.001,
        positive_elem="drive_shaft",
        negative_elem="nose_sleeve",
        name="bit socket emerges directly from the nose sleeve",
    )

    rest_pos = ctx.part_world_position(bit_socket)
    with ctx.pose({socket_joint: 1.4}):
        spun_pos = ctx.part_world_position(bit_socket)
    ctx.check(
        "bit socket stays centered while spinning",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    ctx.expect_gap(
        forward_button,
        body,
        axis="y",
        min_gap=-0.00005,
        max_gap=0.00005,
        positive_elem="forward_button_stem",
        negative_elem="control_pad",
        name="forward button stem is seated against the side pad",
    )
    ctx.expect_gap(
        reverse_button,
        body,
        axis="y",
        min_gap=-0.00005,
        max_gap=0.00005,
        positive_elem="reverse_button_stem",
        negative_elem="control_pad",
        name="reverse button stem is seated against the side pad",
    )

    forward_rest = ctx.part_world_position(forward_button)
    with ctx.pose({forward_button_joint: -0.0012}):
        forward_pressed = ctx.part_world_position(forward_button)
    ctx.check(
        "forward button depresses inward",
        forward_rest is not None
        and forward_pressed is not None
        and forward_pressed[1] < forward_rest[1] - 0.0010,
        details=f"rest={forward_rest}, pressed={forward_pressed}",
    )

    reverse_rest = ctx.part_world_position(reverse_button)
    with ctx.pose({reverse_button_joint: -0.0012}):
        reverse_pressed = ctx.part_world_position(reverse_button)
    ctx.check(
        "reverse button depresses inward",
        reverse_rest is not None
        and reverse_pressed is not None
        and reverse_pressed[1] < reverse_rest[1] - 0.0010,
        details=f"rest={reverse_rest}, pressed={reverse_pressed}",
    )

    ctx.expect_overlap(
        charging_cover,
        body,
        axes="yz",
        elem_a="cover_panel",
        elem_b="charging_port_surround",
        min_overlap=0.008,
        name="charging cap covers the rear charging surround when closed",
    )
    cover_rest = ctx.part_element_world_aabb(charging_cover, elem="cover_panel")
    with ctx.pose({charging_cover_joint: 1.2}):
        cover_open = ctx.part_element_world_aabb(charging_cover, elem="cover_panel")
    ctx.check(
        "charging cap swings away from the rear face when opened",
        cover_rest is not None
        and cover_open is not None
        and cover_open[1][0] > cover_rest[1][0] + 0.008,
        details=f"rest={cover_rest}, open={cover_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
