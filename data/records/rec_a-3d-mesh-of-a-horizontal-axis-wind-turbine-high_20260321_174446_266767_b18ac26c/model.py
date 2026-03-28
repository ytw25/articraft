from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

TOWER_HEIGHT = 2.62
TOWER_BASE_RADIUS = 0.12
TOWER_TOP_RADIUS = 0.055
NACELLE_LENGTH = 0.56
NACELLE_WIDTH = 0.19
NACELLE_HEIGHT = 0.22
HUB_RADIUS = 0.078
HUB_ATTACH_X = 0.070
BLADE_ATTACH_RADIUS = 0.058
BLADE_LENGTH = 0.98


def _signed_superellipse(value: float, exponent: float) -> float:
    if value == 0.0:
        return 0.0
    return math.copysign(abs(value) ** (2.0 / exponent), value)


def _save_mesh(name: str, geometry):
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / f"{name}.obj")


def _tower_geometry():
    return LatheGeometry(
        [
            (0.155, 0.0),
            (0.132, 0.035),
            (TOWER_BASE_RADIUS, 0.10),
            (0.102, 0.90),
            (0.084, 1.65),
            (0.068, 2.18),
            (TOWER_TOP_RADIUS, 2.45),
            (TOWER_TOP_RADIUS, 2.54),
            (0.072, 2.57),
            (0.072, TOWER_HEIGHT - 0.018),
            (0.060, TOWER_HEIGHT),
        ],
        segments=72,
    )


def _nacelle_section(x: float, half_width: float, z_bottom: float, z_top: float, samples: int = 56):
    center_z = 0.5 * (z_bottom + z_top)
    upper = z_top - center_z
    lower = center_z - z_bottom
    points = []
    for idx in range(samples):
        angle = 2.0 * math.pi * idx / samples
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        y = half_width * _signed_superellipse(cos_a, 2.8)
        if sin_a >= 0.0:
            z = center_z + upper * (sin_a ** (2.0 / 2.6))
        else:
            z = center_z - lower * ((-sin_a) ** (2.0 / 3.2))
        points.append((x, y, z))
    return points


def _nacelle_geometry():
    sections = [
        _nacelle_section(-0.18, 0.012, 0.058, 0.100),
        _nacelle_section(-0.10, 0.060, 0.020, 0.156),
        _nacelle_section(0.02, 0.090, 0.004, 0.206),
        _nacelle_section(0.18, 0.095, 0.000, 0.220),
        _nacelle_section(0.30, 0.068, 0.014, 0.170),
        _nacelle_section(0.40, 0.042, 0.044, 0.158),
        _nacelle_section(0.475, 0.022, 0.084, 0.148),
        _nacelle_section(0.505, 0.010, 0.104, 0.132),
    ]
    return repair_loft(
        SectionLoftSpec(
            sections=tuple(tuple(section) for section in sections),
            cap=True,
            solid=True,
            repair="auto",
        )
    )


def _hub_geometry():
    geometry = LatheGeometry(
        [
            (0.030, 0.000),
            (0.048, 0.014),
            (0.066, 0.048),
            (HUB_RADIUS, 0.095),
            (0.068, 0.152),
            (0.040, 0.196),
            (0.0, 0.228),
        ],
        segments=64,
    )
    geometry.rotate_y(math.pi / 2.0)
    return geometry


def _airfoil_loop(chord: float, thickness_ratio: float, samples: int = 28):
    def thickness_at(u: float) -> float:
        return 5.0 * thickness_ratio * chord * (
            0.2969 * math.sqrt(max(u, 0.0))
            - 0.1260 * u
            - 0.3516 * (u**2)
            + 0.2843 * (u**3)
            - 0.1036 * (u**4)
        )

    profile = []
    for idx in range(samples, -1, -1):
        u = idx / samples
        profile.append(((u - 0.25) * chord, thickness_at(u)))
    for idx in range(1, samples + 1):
        u = idx / samples
        profile.append(((u - 0.25) * chord, -thickness_at(u)))
    return profile


def _blade_geometry():
    span_positions = [0.0, 0.08, 0.22, 0.46, 0.72, 1.0]
    sections = []
    for s in span_positions:
        chord = 0.190 * (1.0 - s) ** 0.78 + 0.050 * s
        thickness_ratio = 0.20 - 0.11 * s
        twist = math.radians(16.0 - 14.0 * s)
        sweep = -0.030 * (s**1.4)
        prebend = 0.010 * (s**1.8)
        z_span = -0.012 + BLADE_LENGTH * s
        section = []
        for x, y in _airfoil_loop(chord, thickness_ratio):
            x_rot = x * math.cos(twist) - y * math.sin(twist)
            y_rot = x * math.sin(twist) + y * math.cos(twist)
            section.append((x_rot + sweep, y_rot + prebend, z_span))
        sections.append(section)
    return repair_loft(
        SectionLoftSpec(
            sections=tuple(tuple(section) for section in sections),
            cap=True,
            solid=True,
            repair="auto",
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="horizontal_axis_wind_turbine", assets=ASSETS)

    tower_paint = model.material("tower_paint", rgba=(0.90, 0.91, 0.93, 1.0))
    nacelle_paint = model.material("nacelle_paint", rgba=(0.93, 0.94, 0.95, 1.0))
    blade_paint = model.material("blade_composite", rgba=(0.96, 0.96, 0.97, 1.0))
    hub_metal = model.material("hub_metal", rgba=(0.66, 0.69, 0.72, 1.0))

    tower = model.part("tower")
    tower.visual(_save_mesh("tower", _tower_geometry()), material=tower_paint)
    tower.inertial = None

    nacelle = model.part("nacelle")
    nacelle.visual(_save_mesh("nacelle", _nacelle_geometry()), material=nacelle_paint)
    nacelle.inertial = None

    hub = model.part("hub")
    hub.visual(_save_mesh("hub", _hub_geometry()), material=hub_metal)
    hub.inertial = None

    blade_mesh = _save_mesh("blade", _blade_geometry())
    for idx in range(3):
        blade = model.part(f"blade_{idx}")
        blade.visual(blade_mesh, material=blade_paint)
        blade.inertial = None

    model.articulation(
        "tower_to_nacelle",
        ArticulationType.FIXED,
        parent="tower",
        child="nacelle",
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT + 0.003)),
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent="nacelle",
        child="hub",
        origin=Origin(xyz=(0.505, 0.0, 0.118)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=8.0),
    )

    blade_angles = [math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0]
    for idx, angle in enumerate(blade_angles):
        model.articulation(
            f"hub_to_blade_{idx}",
            ArticulationType.FIXED,
            parent="hub",
            child=f"blade_{idx}",
            origin=Origin(
                xyz=(
                    HUB_ATTACH_X,
                    BLADE_ATTACH_RADIUS * math.cos(angle),
                    BLADE_ATTACH_RADIUS * math.sin(angle),
                ),
                rpy=(angle - math.pi / 2.0, 0.0, 0.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual", ignore_adjacent=True, ignore_fixed=True)
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("tower", "nacelle", axes="xy", min_overlap=0.09)
    ctx.expect_aabb_gap("nacelle", "tower", axis="z", max_gap=0.02, max_penetration=0.02)
    ctx.expect_aabb_overlap("hub", "nacelle", axes="yz", min_overlap=0.08)
    ctx.expect_aabb_gap("hub", "nacelle", axis="x", max_gap=0.05, max_penetration=0.03)
    for idx in range(3):
        ctx.expect_aabb_contact(f"blade_{idx}", "hub")

    blade_names = [f"blade_{idx}" for idx in range(3)]
    if len([part for part in object_model.parts if part.name.startswith("blade_")]) != 3:
        raise AssertionError("Wind turbine must expose exactly three blades.")

    tower_pos = ctx.part_world_position("tower")
    nacelle_pos = ctx.part_world_position("nacelle")
    hub_pos = ctx.part_world_position("hub")
    if abs(nacelle_pos[0] - tower_pos[0]) > 1e-6 or abs(nacelle_pos[1] - tower_pos[1]) > 1e-6:
        raise AssertionError("Nacelle should sit centered on the tower axis.")
    if hub_pos[0] <= 0.24:
        raise AssertionError("Rotor hub should project clearly in front of the tower.")
    if hub_pos[2] <= nacelle_pos[2] + 0.09:
        raise AssertionError("Rotor axis should sit high in the nacelle body.")

    rest_blade_positions = {name: ctx.part_world_position(name) for name in blade_names}
    radial_distances = []
    for name, pos in rest_blade_positions.items():
        radial = math.hypot(pos[1] - hub_pos[1], pos[2] - hub_pos[2])
        radial_distances.append(radial)
        if abs(pos[0] - (hub_pos[0] + HUB_ATTACH_X)) > 0.01:
            raise AssertionError(f"{name} root should sit on the hub face, not drift fore-aft.")
        if not (0.045 <= radial <= 0.075):
            raise AssertionError(f"{name} root should be distributed evenly around the hub circumference.")

    for i in range(3):
        a = rest_blade_positions[blade_names[i]]
        b = rest_blade_positions[blade_names[(i + 1) % 3]]
        yz_pair_dist = math.hypot(a[1] - b[1], a[2] - b[2])
        if yz_pair_dist <= 0.09:
            raise AssertionError("Blade roots should form a clear three-spoke pattern around the hub.")

    if max(pos[2] for pos in rest_blade_positions.values()) - min(pos[2] for pos in rest_blade_positions.values()) <= 0.08:
        raise AssertionError("The rotor should read as a vertical three-blade arrangement in its rest pose.")

    with ctx.pose(rotor_spin=math.pi / 3.0):
        spun_hub = ctx.part_world_position("hub")
        for name in blade_names:
            spun_pos = ctx.part_world_position(name)
            radial = math.hypot(spun_pos[1] - spun_hub[1], spun_pos[2] - spun_hub[2])
            if abs(radial - BLADE_ATTACH_RADIUS) > 0.01:
                raise AssertionError(f"{name} should stay on the hub pitch circle while spinning.")
        if abs(ctx.part_world_position("blade_0")[2] - rest_blade_positions["blade_0"][2]) <= 0.02:
            raise AssertionError("Rotor articulation should move the blades around the hub axis.")

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
