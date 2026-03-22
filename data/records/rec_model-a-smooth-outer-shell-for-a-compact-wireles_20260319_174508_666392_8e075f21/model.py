from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    Box,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    superellipse_profile,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MOUSE_MESH_PATH = MESH_DIR / "mouse_body_shell.obj"

MODEL_LENGTH = 0.118
MODEL_WIDTH = 0.064
MODEL_HEIGHT = 0.040


def _section_loop(y: float, width: float, z_min: float, z_max: float, exponent: float) -> list[tuple[float, float, float]]:
    profile = superellipse_profile(width, z_max - z_min, exponent=exponent, segments=48)
    z_center = 0.5 * (z_min + z_max)
    return [(x, y, z_center + z) for x, z in profile]


def _build_mouse_shell_mesh():
    sections = [
        _section_loop(-0.059, 0.012, 0.0075, 0.0105, 2.0),
        _section_loop(-0.052, 0.032, 0.0030, 0.0130, 2.2),
        _section_loop(-0.044, 0.046, 0.0015, 0.0165, 2.4),
        _section_loop(-0.034, 0.056, 0.0005, 0.0205, 2.6),
        _section_loop(-0.022, 0.061, 0.0000, 0.0240, 2.8),
        _section_loop(-0.008, 0.064, 0.0000, 0.0285, 2.9),
        _section_loop(0.010, 0.064, 0.0000, 0.0335, 3.0),
        _section_loop(0.026, 0.062, 0.0000, 0.0400, 3.15),
        _section_loop(0.038, 0.056, 0.0005, 0.0375, 3.0),
        _section_loop(0.047, 0.045, 0.0015, 0.0330, 2.8),
        _section_loop(0.054, 0.033, 0.0030, 0.0275, 2.6),
        _section_loop(0.059, 0.018, 0.0050, 0.0210, 2.3),
    ]
    return repair_loft(sections, repair="auto")


def _load_mesh_vertices(path: Path) -> list[tuple[float, float, float]]:
    vertices: list[tuple[float, float, float]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.startswith("v "):
            continue
        _, xs, ys, zs, *_ = line.split()
        vertices.append((float(xs), float(ys), float(zs)))
    return vertices


def _band_stats(vertices: list[tuple[float, float, float]], y_min: float, y_max: float) -> dict[str, float]:
    band = [(x, y, z) for x, y, z in vertices if y_min <= y <= y_max]
    if not band:
        raise AssertionError(f"no mesh vertices found in longitudinal band {y_min:.3f}..{y_max:.3f}")
    xs = [x for x, _, _ in band]
    zs = [z for _, _, z in band]
    return {
        "width": max(xs) - min(xs),
        "z_min": min(zs),
        "z_max": max(zs),
        "height": max(zs) - min(zs),
    }


def _assert_between(value: float, lower: float, upper: float, label: str) -> None:
    if not (lower <= value <= upper):
        raise AssertionError(f"{label}={value:.4f} outside expected range [{lower:.4f}, {upper:.4f}]")


def build_object_model() -> ArticulatedObject:
    MESH_DIR.mkdir(parents=True, exist_ok=True)

    model = ArticulatedObject(name="compact_wireless_mouse", assets=ASSETS)
    graphite = model.material("graphite_plastic", rgba=(0.20, 0.21, 0.23, 1.0))

    shell_mesh = mesh_from_geometry(_build_mouse_shell_mesh(), MOUSE_MESH_PATH)

    body = model.part("mouse_body")
    body.visual(shell_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0)), material=graphite, name="outer_shell")
    body.inertial = Inertial.from_geometry(
        Box((MODEL_WIDTH, MODEL_LENGTH, MODEL_HEIGHT)),
        mass=0.095,
        origin=Origin(xyz=(0.0, 0.0, MODEL_HEIGHT * 0.5)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.015)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(
        max_pose_samples=16,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    vertices = _load_mesh_vertices(MOUSE_MESH_PATH)
    xs = [x for x, _, _ in vertices]
    ys = [y for _, y, _ in vertices]
    zs = [z for _, _, z in vertices]

    min_y, max_y = min(ys), max(ys)
    length = max_y - min_y
    width = max(xs) - min(xs)
    height = max(zs) - min(zs)

    _assert_between(length, 0.114, 0.122, "mouse length")
    _assert_between(width, 0.061, 0.067, "mouse width")
    _assert_between(height, 0.038, 0.0425, "mouse height")
    _assert_between(0.5 * (max(xs) + min(xs)), -0.0015, 0.0015, "lateral centering")

    front_band = _band_stats(vertices, min_y + 0.012, min_y + 0.032)
    mid_band = _band_stats(vertices, min_y + 0.046, min_y + 0.074)
    hump_band = _band_stats(vertices, max_y - 0.040, max_y - 0.018)
    tail_band = _band_stats(vertices, max_y - 0.018, max_y - 0.006)

    if front_band["width"] < 0.046:
        raise AssertionError(f"front deck should stay wide; got {front_band['width']:.4f} m")
    if front_band["z_max"] > 0.024:
        raise AssertionError(f"nose should stay low; got front z_max={front_band['z_max']:.4f} m")
    if front_band["z_min"] > 0.0035:
        raise AssertionError(f"front underside should stay close to the desk plane; got z_min={front_band['z_min']:.4f} m")

    if hump_band["z_max"] < 0.038:
        raise AssertionError(f"rear hump should be full and high; got hump z_max={hump_band['z_max']:.4f} m")
    if hump_band["z_max"] < front_band["z_max"] + 0.013:
        raise AssertionError("rear hump is not visibly fuller than the low nose")
    if hump_band["z_max"] < mid_band["z_max"] + 0.004:
        raise AssertionError("rear hump should crest behind the midpoint, not flatten into the center")

    if mid_band["width"] < front_band["width"]:
        raise AssertionError("body should broaden gently from the nose into the main shell")
    if tail_band["width"] > hump_band["width"] - 0.008:
        raise AssertionError("rear taper should tighten slightly behind the main hump")
    if front_band["width"] < 0.72 * width:
        raise AssertionError("nose deck should read as low and broad, not pinched")
    if tail_band["width"] < 0.42 * width:
        raise AssertionError("rear taper is too abrupt for a plausible compact mouse shell")

    highest_vertex_y = max(vertices, key=lambda vertex: vertex[2])[1]
    if highest_vertex_y < 0.010:
        raise AssertionError("highest point should sit in the rear half to create the hump-backed silhouette")

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
