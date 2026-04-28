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
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)
GPU_LENGTH = 0.312
GPU_HEIGHT = 0.114
GPU_THICKNESS = 0.038
FAN_CENTERS_X = (0.086, 0.169, 0.252)
FAN_JOINT_Z = 0.008


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    radial_segments: int = 56,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -0.5 * length), (outer_radius, 0.5 * length)],
        [(inner_radius, -0.5 * length), (inner_radius, 0.5 * length)],
        segments=radial_segments,
        start_cap="flat",
        end_cap="flat",
    )


def _section_loop(
    x_pos: float,
    half_y: float,
    z_back: float,
    z_front: float,
    bevel_y: float,
    bevel_z: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, -half_y + bevel_y, z_back),
        (x_pos, half_y - bevel_y, z_back),
        (x_pos, half_y, z_back + bevel_z),
        (x_pos, half_y, z_front - bevel_z),
        (x_pos, half_y - bevel_y, z_front),
        (x_pos, -half_y + bevel_y, z_front),
        (x_pos, -half_y, z_front - bevel_z),
        (x_pos, -half_y, z_back + bevel_z),
    ]


def _build_body_shell_mesh() -> MeshGeometry:
    shell = repair_loft(
        section_loft(
            [
                _section_loop(0.010, 0.048, -0.006, 0.013, 0.010, 0.003),
                _section_loop(0.060, 0.054, -0.006, 0.015, 0.010, 0.004),
                _section_loop(0.160, 0.056, -0.0065, 0.016, 0.011, 0.004),
                _section_loop(0.262, 0.054, -0.006, 0.0155, 0.011, 0.004),
                _section_loop(0.300, 0.046, -0.005, 0.013, 0.008, 0.003),
            ]
        )
    )

    return shell


def _build_bracket_mesh() -> MeshGeometry:
    plate = BoxGeometry((0.0022, 0.116, 0.036))
    screw_ear = BoxGeometry((0.0022, 0.012, 0.018)).translate(0.0, 0.062, 0.0)
    retention_tab = BoxGeometry((0.0022, 0.010, 0.016)).translate(0.0, -0.062, 0.0)
    bracket = boolean_union(plate, screw_ear)
    bracket = boolean_union(bracket, retention_tab)

    for port_center_y in (-0.026, 0.0, 0.026):
        display_port = BoxGeometry((0.006, 0.018, 0.0085)).translate(0.0, port_center_y, 0.0045)
        bracket = boolean_difference(bracket, display_port)

    for vent_center_y in (-0.036, -0.026, -0.016, -0.006, 0.004, 0.014):
        vent_slot = BoxGeometry((0.006, 0.0035, 0.018)).translate(0.0, vent_center_y, -0.006)
        bracket = boolean_difference(bracket, vent_slot)

    return bracket


def _fan_blade_section(
    radius: float,
    center_y: float,
    center_z: float,
    chord: float,
    thickness: float,
) -> list[tuple[float, float, float]]:
    return [
        (radius, center_y - 0.54 * chord, center_z - 0.34 * thickness),
        (radius, center_y - 0.10 * chord, center_z - 0.62 * thickness),
        (radius, center_y + 0.40 * chord, center_z - 0.16 * thickness),
        (radius, center_y + 0.56 * chord, center_z + 0.20 * thickness),
        (radius, center_y + 0.14 * chord, center_z + 0.58 * thickness),
        (radius, center_y - 0.46 * chord, center_z + 0.18 * thickness),
    ]


def _radial_pattern(base_geometry: MeshGeometry, count: int, *, angle_offset: float = 0.0) -> MeshGeometry:
    patterned = MeshGeometry()
    for index in range(count):
        patterned.merge(base_geometry.copy().rotate_z(angle_offset + index * math.tau / count))
    return patterned


def _build_fan_blades_mesh() -> MeshGeometry:
    blade = repair_loft(
        section_loft(
            [
                _fan_blade_section(0.014, 0.000, 0.0002, 0.017, 0.0042),
                _fan_blade_section(0.022, 0.004, 0.0008, 0.021, 0.0036),
                _fan_blade_section(0.030, 0.007, 0.0014, 0.018, 0.0030),
                _fan_blade_section(0.0365, 0.008, 0.0018, 0.010, 0.0022),
            ]
        )
    )
    blade.rotate_z(0.28)
    return _radial_pattern(blade, 9, angle_offset=math.tau / 18.0)


def _build_fan_ring_mesh() -> MeshGeometry:
    return _ring_band(outer_radius=0.040, inner_radius=0.0375, length=0.0045, radial_segments=64).translate(
        0.0,
        0.0,
        0.0018,
    )


def _build_fan_hub_mesh() -> MeshGeometry:
    hub = CylinderGeometry(radius=0.0165, height=0.0042, radial_segments=56).translate(0.0, 0.0, 0.0018)
    cap = CylinderGeometry(radius=0.0098, height=0.0050, radial_segments=40).translate(0.0, 0.0, 0.0022)
    return _merge_geometries([hub, cap])


def _build_fan_sleeve_mesh() -> MeshGeometry:
    return CylinderGeometry(radius=0.0065, height=0.0060, radial_segments=36).translate(0.0, 0.0, -0.0030)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="graphics_card", assets=ASSETS)

    shroud_black = model.material("shroud_black", rgba=(0.09, 0.10, 0.11, 1.0))
    carbon_gray = model.material("carbon_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    heatsink_aluminum = model.material("heatsink_aluminum", rgba=(0.56, 0.59, 0.63, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.66, 0.68, 0.70, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.08, 0.20, 0.12, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.78, 0.66, 0.30, 1.0))

    shell_mesh = _save_mesh(_build_body_shell_mesh(), "gpu_body_shell.obj")
    bracket_mesh = _save_mesh(_build_bracket_mesh(), "gpu_bracket.obj")
    fan_blades_mesh = _save_mesh(_build_fan_blades_mesh(), "gpu_fan_blades.obj")
    fan_ring_mesh = _save_mesh(_build_fan_ring_mesh(), "gpu_fan_ring.obj")
    fan_hub_mesh = _save_mesh(_build_fan_hub_mesh(), "gpu_fan_hub.obj")
    fan_sleeve_mesh = _save_mesh(_build_fan_sleeve_mesh(), "gpu_fan_sleeve.obj")

    body = model.part("card_body")
    body.visual(shell_mesh, material=shroud_black, name="body_shell")
    body.visual(
        Box((0.242, 0.092, 0.018)),
        origin=Origin(xyz=(0.174, 0.0, -0.002)),
        material=heatsink_aluminum,
        name="heatsink_core",
    )
    body.visual(
        Box((0.286, 0.102, 0.0022)),
        origin=Origin(xyz=(0.156, 0.0, -0.0119)),
        material=pcb_green,
        name="pcb",
    )
    body.visual(
        Box((0.076, 0.010, 0.0016)),
        origin=Origin(xyz=(0.112, -0.054, -0.0132)),
        material=connector_gold,
        name="pcie_fingers",
    )
    body.visual(
        Box((0.030, 0.012, 0.010)),
        origin=Origin(xyz=(0.252, 0.049, 0.009)),
        material=carbon_gray,
        name="power_connector",
    )
    body.visual(
        Box((0.012, 0.062, 0.024)),
        origin=Origin(xyz=(0.006, 0.0, 0.001)),
        material=heatsink_aluminum,
        name="bracket_mount_block",
    )
    for label, fan_center_x in zip(("left", "center", "right"), FAN_CENTERS_X):
        body.visual(
            _save_mesh(
                _ring_band(outer_radius=0.052, inner_radius=0.0465, length=0.0060).translate(
                    fan_center_x,
                    0.0,
                    0.011,
                ),
                f"gpu_fan_well_{label}.obj",
            ),
            material=carbon_gray,
            name=f"fan_well_{label}",
        )
        body.visual(
            _save_mesh(
                _ring_band(outer_radius=0.048, inner_radius=0.0415, length=0.0060).translate(
                    fan_center_x,
                    0.0,
                    0.011,
                ),
                f"gpu_fan_trim_{label}.obj",
            ),
            material=carbon_gray,
            name=f"fan_trim_{label}",
        )
        body.visual(
            Cylinder(radius=0.017, length=0.004),
            origin=Origin(xyz=(fan_center_x, 0.0, 0.0)),
            material=heatsink_aluminum,
            name=f"hub_seat_{label}",
        )
    body.inertial = Inertial.from_geometry(
        Box((GPU_LENGTH, GPU_HEIGHT, GPU_THICKNESS)),
        mass=1.35,
        origin=Origin(xyz=(GPU_LENGTH * 0.5, 0.0, 0.0)),
    )

    bracket = model.part("io_bracket")
    bracket.visual(bracket_mesh, material=bracket_metal, name="bracket_plate")
    bracket.inertial = Inertial.from_geometry(
        Box((0.0022, 0.124, 0.038)),
        mass=0.10,
        origin=Origin(),
    )

    for part_name in ("fan_left", "fan_center", "fan_right"):
        fan = model.part(part_name)
        fan.visual(fan_ring_mesh, material=shroud_black, name="rotor_ring")
        fan.visual(fan_hub_mesh, material=carbon_gray, name="hub_shell")
        fan.visual(fan_blades_mesh, material=shroud_black, name="blade_pack")
        fan.visual(fan_sleeve_mesh, material=carbon_gray, name="hub_sleeve")
        fan.inertial = Inertial.from_geometry(
            Cylinder(radius=0.040, length=0.012),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

    model.articulation(
        "body_to_bracket",
        ArticulationType.FIXED,
        parent=body,
        child=bracket,
        origin=Origin(xyz=(-0.0011, 0.0, 0.0)),
    )
    for part_name, joint_name, fan_center_x in zip(
        ("fan_left", "fan_center", "fan_right"),
        ("left_fan_spin", "center_fan_spin", "right_fan_spin"),
        FAN_CENTERS_X,
    ):
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=part_name,
            origin=Origin(xyz=(fan_center_x, 0.0, FAN_JOINT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.08, velocity=240.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("card_body")
    bracket = object_model.get_part("io_bracket")
    fan_left = object_model.get_part("fan_left")
    fan_center = object_model.get_part("fan_center")
    fan_right = object_model.get_part("fan_right")
    left_fan_spin = object_model.get_articulation("left_fan_spin")
    center_fan_spin = object_model.get_articulation("center_fan_spin")
    right_fan_spin = object_model.get_articulation("right_fan_spin")

    body_shell = body.get_visual("body_shell")
    bracket_mount_block = body.get_visual("bracket_mount_block")
    bracket_plate = bracket.get_visual("bracket_plate")
    fan_well_left = body.get_visual("fan_well_left")
    fan_well_center = body.get_visual("fan_well_center")
    fan_well_right = body.get_visual("fan_well_right")
    hub_seat_left = body.get_visual("hub_seat_left")
    hub_seat_center = body.get_visual("hub_seat_center")
    hub_seat_right = body.get_visual("hub_seat_right")
    pcb = body.get_visual("pcb")
    pcie_fingers = body.get_visual("pcie_fingers")
    power_connector = body.get_visual("power_connector")

    left_ring = fan_left.get_visual("rotor_ring")
    center_ring = fan_center.get_visual("rotor_ring")
    right_ring = fan_right.get_visual("rotor_ring")
    left_sleeve = fan_left.get_visual("hub_sleeve")
    center_sleeve = fan_center.get_visual("hub_sleeve")
    right_sleeve = fan_right.get_visual("hub_sleeve")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual connectivity gate for floating/disconnected subassemblies inside one part.
    ctx.fail_if_part_contains_disconnected_geometry_islands()

    # Encode the actual visual/mechanical claims with prompt-specific exact checks.
    # If you add a warning-tier heuristic and it fires, investigate it with
    # `probe_model` before editing geometry or relaxing thresholds.
    # Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is
    # genuinely uncertain or mechanically important.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # For ctx.expect_* helpers, keep the first body/link arguments as Part objects.
    # Named Visuals belong only in elem_a/elem_b/positive_elem/negative_elem/inner_elem/outer_elem.
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # If the object has a mounted subassembly, prefer exact `expect_contact(...)`,
    # `expect_gap(...)`, `expect_overlap(...)`, and `expect_within(...)` checks on
    # named local features over broad warning-tier heuristics.
    ctx.expect_overlap(
        bracket,
        body,
        axes="yz",
        min_overlap=0.022,
        elem_a=bracket_plate,
        elem_b=bracket_mount_block,
        name="bracket_overlaps_mount_block_in_yz",
    )
    ctx.expect_gap(
        body,
        bracket,
        axis="x",
        max_gap=0.0002,
        max_penetration=0.0,
        positive_elem=bracket_mount_block,
        negative_elem=bracket_plate,
        name="bracket_seats_against_mount_block",
    )
    ctx.expect_contact(
        bracket,
        body,
        elem_a=bracket_plate,
        elem_b=bracket_mount_block,
        name="bracket_contacts_body_mount",
    )
    ctx.expect_overlap(
        body,
        bracket,
        axes="yz",
        min_overlap=0.020,
        elem_a=body_shell,
        elem_b=bracket_plate,
        name="body_shell_reaches_bracket_zone",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="xy",
        min_overlap=0.0015,
        elem_a=pcie_fingers,
        elem_b=pcb,
        name="pcie_fingers_overlap_pcb_mount_zone",
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        max_gap=0.0002,
        max_penetration=0.0008,
        positive_elem=pcb,
        negative_elem=pcie_fingers,
        name="pcie_fingers_are_seated_on_pcb",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="xy",
        min_overlap=0.010,
        elem_a=power_connector,
        elem_b=body_shell,
        name="power_connector_is_integrated_into_top_shroud",
    )

    fan_checks = [
        (fan_left, left_fan_spin, left_ring, left_sleeve, fan_well_left, hub_seat_left, "left"),
        (
            fan_center,
            center_fan_spin,
            center_ring,
            center_sleeve,
            fan_well_center,
            hub_seat_center,
            "center",
        ),
        (fan_right, right_fan_spin, right_ring, right_sleeve, fan_well_right, hub_seat_right, "right"),
    ]
    for fan_part, fan_joint, ring, sleeve, fan_well, hub_seat, label in fan_checks:
        ctx.expect_overlap(
            fan_part,
            body,
            axes="xy",
            min_overlap=0.078,
            elem_a=ring,
            elem_b=fan_well,
            name=f"{label}_fan_centered_in_well",
        )
        ctx.expect_gap(
            fan_part,
            body,
            axis="z",
            max_gap=0.0010,
            max_penetration=0.0,
            positive_elem=sleeve,
            negative_elem=hub_seat,
            name=f"{label}_fan_hub_is_seated",
        )
        with ctx.pose({fan_joint: math.pi / 3.0}):
            ctx.expect_overlap(
                fan_part,
                body,
                axes="xy",
                min_overlap=0.078,
                elem_a=ring,
                elem_b=fan_well,
                name=f"{label}_fan_stays_centered_when_spun",
            )
            ctx.expect_gap(
                fan_part,
                body,
                axis="z",
                max_gap=0.0010,
                max_penetration=0.0,
                positive_elem=sleeve,
                negative_elem=hub_seat,
                name=f"{label}_fan_hub_stays_seated_when_spun",
            )
    ctx.expect_gap(
        fan_center,
        fan_left,
        axis="x",
        min_gap=0.002,
        positive_elem=center_ring,
        negative_elem=left_ring,
        name="left_and_center_fans_clear_each_other",
    )
    ctx.expect_gap(
        fan_right,
        fan_center,
        axis="x",
        min_gap=0.002,
        positive_elem=right_ring,
        negative_elem=center_ring,
        name="center_and_right_fans_clear_each_other",
    )

    # Add prompt-specific exact visual checks below; optional warning heuristics are not enough.
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
