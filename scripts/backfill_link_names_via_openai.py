from __future__ import annotations

import argparse
import ast
import copy
import glob
import hashlib
import json
import os
import re
import sys
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from threading import Lock, local
from typing import Any, Sequence

from storage.repo import StorageRepo

DEFAULT_SYSTEM_PROMPT = Path(__file__).with_name("system_prompt_backfill_link_names.md")
DEFAULT_FAILURES_OUTPUT_NAME = "link_name_backfill_openai_failures.txt"
SUGGESTED_NAME_PATTERN = re.compile(r"^[a-z0-9]+(?:_[a-z0-9]+){0,4}$")
FINAL_SINGLE_LETTER_TOKEN_PATTERN = re.compile(r"^[a-z]$")
WORD_PATTERN = re.compile(r"\b[\w'-]+\b")
RELEVANT_CONTEXT_RADIUS_LINES = 6
MAX_FULL_FUNCTION_EXCERPT_LINES = 140
BANNED_STATE_TOKENS = frozenset(
    {
        "open",
        "opened",
        "closed",
        "ajar",
        "extended",
        "retracted",
        "pulled",
        "pulledout",
        "tilted",
        "rotated",
        "folded",
        "unfolded",
        "deployed",
        "stowed",
        "raised",
        "lowered",
    }
)
FRAME_STATUSES = frozenset({"clear", "partial", "ambiguous", "not_applicable"})
LINK_STATUSES = frozenset({"compliant", "rename", "review"})


@dataclass(frozen=True)
class PartCall:
    lineno: int
    name: str | None
    expression: str


@dataclass(frozen=True)
class SourceAnalysis:
    model_py_sha256: str
    model_line_count: int
    part_call_count: int
    literal_part_names: tuple[str, ...]
    dynamic_part_call_expressions: tuple[str, ...]
    get_part_names: tuple[str, ...]
    object_name_hint: str | None


@dataclass(frozen=True)
class RecordTask:
    record_dir: Path
    record_id: str
    output_path: Path


@dataclass(frozen=True)
class PreparedRecord:
    task: RecordTask
    model_path: Path
    title: str | None
    prompt_preview: str | None
    category_slug: str | None
    model_source: str
    extracted_model_context: str
    available_part_names: tuple[str, ...]
    available_part_names_source: str
    available_part_names_complete: bool
    source_analysis: SourceAnalysis


@dataclass(frozen=True)
class UsageTotals:
    input_tokens: int = 0
    cached_input_tokens: int = 0
    output_tokens: int = 0
    reasoning_tokens: int = 0

    def __add__(self, other: UsageTotals) -> UsageTotals:
        return UsageTotals(
            input_tokens=self.input_tokens + other.input_tokens,
            cached_input_tokens=self.cached_input_tokens + other.cached_input_tokens,
            output_tokens=self.output_tokens + other.output_tokens,
            reasoning_tokens=self.reasoning_tokens + other.reasoning_tokens,
        )


@dataclass(frozen=True)
class TaskResult:
    record_id: str
    status: str
    detail: str | None = None
    usage: UsageTotals | None = None


class SkipTaskError(RuntimeError):
    """Raised when a record should be skipped without counting as a failure."""


class ResponseProcessingError(RuntimeError):
    def __init__(self, message: str, usage: UsageTotals | None = None):
        super().__init__(message)
        self.usage = usage or UsageTotals()


class _SourceAnalyzer(ast.NodeVisitor):
    def __init__(self, source: str):
        self.source = source
        self.part_calls: list[PartCall] = []
        self.get_part_names: list[str] = []
        self.object_name_hint: str | None = None

    def visit_Call(self, node: ast.Call) -> Any:
        func = node.func
        if isinstance(func, ast.Attribute) and func.attr == "part":
            self.part_calls.append(self._extract_part_call(node))
        elif isinstance(func, ast.Attribute) and func.attr == "get_part":
            literal_name = _string_literal_value(node.args[0]) if node.args else None
            if literal_name:
                self.get_part_names.append(literal_name)
        elif isinstance(func, ast.Name) and func.id == "ArticulatedObject":
            for keyword in node.keywords:
                if keyword.arg != "name":
                    continue
                literal_name = _string_literal_value(keyword.value)
                if literal_name:
                    self.object_name_hint = literal_name
                    break
        self.generic_visit(node)

    def _extract_part_call(self, node: ast.Call) -> PartCall:
        arg = node.args[0] if node.args else None
        literal_name = _string_literal_value(arg)
        expression = _node_to_source(self.source, arg).strip() if arg is not None else ""
        return PartCall(
            lineno=getattr(node, "lineno", 0),
            name=literal_name,
            expression=expression,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Backfill articulated link-name rename manifests by sending each record's metadata "
            "and articulated part-name list to the OpenAI Responses API. This script is "
            "text-only and does not use rendered images."
        )
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path("."),
        help="Repository root containing data/records (default: current directory).",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help=(
            "Directory for per-record JSON manifests. Defaults to "
            "data/cache/manifests/link_name_backfill_openai under --repo-root."
        ),
    )
    parser.add_argument(
        "--system-prompt",
        type=Path,
        default=DEFAULT_SYSTEM_PROMPT,
        help=(f"System prompt markdown file (default: {DEFAULT_SYSTEM_PROMPT.as_posix()})."),
    )
    parser.add_argument(
        "--model",
        type=str,
        default="gpt-5.4-mini",
        help="OpenAI model name (default: gpt-5.4-mini).",
    )
    parser.add_argument(
        "--reasoning-effort",
        type=str,
        choices=("low", "medium", "high"),
        default="medium",
        help="OpenAI reasoning effort (default: medium).",
    )
    parser.add_argument(
        "--record-dir",
        type=str,
        default=None,
        help=(
            "Optional record selector or quoted glob pattern like rec_ring_light_on_stand_0002, "
            "rec_ring_light_*, or ./data/records/rec_ring_light_on_stand_0002."
        ),
    )
    parser.add_argument(
        "--show-payload-only",
        action="store_true",
        help=(
            "When used with --record-dir and exactly one matching record, print the request "
            "payload and exit without sending it."
        ),
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip records whose output JSON already exists.",
    )
    parser.add_argument(
        "--n-concurrency",
        type=int,
        default=8,
        help="Number of concurrent OpenAI requests (default: 8).",
    )
    parser.add_argument(
        "--api-key-env",
        type=str,
        default="OPENAI_API_KEY",
        help=(
            "Preferred env var containing the OpenAI API key. The script also falls back to "
            "OPENAI_API_KEY and OPENAI_API_KEYS."
        ),
    )
    parser.add_argument(
        "--failures-output",
        type=Path,
        default=None,
        help=(f"Path to save failures. Defaults to <output-root>/{DEFAULT_FAILURES_OUTPUT_NAME}."),
    )
    return parser.parse_args()


def load_system_prompt(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(f"System prompt file not found: {path}")
    content = path.read_text(encoding="utf-8").strip()
    if not content:
        raise ValueError(f"System prompt file is empty: {path}")
    return content


def atomic_write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_suffix(path.suffix + ".tmp")
    temp_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    temp_path.replace(path)


def write_lines(path: Path, lines: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(f"{line}\n" for line in lines), encoding="utf-8")


def _string_literal_value(node: ast.AST | None) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _node_to_source(source: str, node: ast.AST | None) -> str:
    if node is None:
        return ""
    source_segment = ast.get_source_segment(source, node)
    if source_segment is not None:
        return source_segment
    try:
        return ast.unparse(node)
    except Exception:
        return ""


def _line_numbered_source(source: str) -> str:
    lines = source.splitlines()
    width = max(3, len(str(max(1, len(lines)))))
    return "\n".join(f"{index:0{width}d}: {line}" for index, line in enumerate(lines, start=1))


def analyze_model_source(source: str) -> SourceAnalysis:
    literal_part_names: list[str] = []
    dynamic_part_call_expressions: list[str] = []
    get_part_names: list[str] = []
    object_name_hint: str | None = None
    part_call_count = 0

    try:
        tree = ast.parse(source)
    except SyntaxError:
        literal_part_names = re.findall(r"""\.part\(\s*["']([^"']+)["']\s*\)""", source)
        get_part_names = re.findall(r"""get_part\(\s*["']([^"']+)["']\s*\)""", source)
        part_call_count = len(re.findall(r"\.part\(", source))
    else:
        analyzer = _SourceAnalyzer(source)
        analyzer.visit(tree)
        part_call_count = len(analyzer.part_calls)
        for call in analyzer.part_calls:
            if call.name is not None:
                literal_part_names.append(call.name)
            else:
                dynamic_part_call_expressions.append(call.expression or "<unknown>")
        get_part_names = analyzer.get_part_names
        object_name_hint = analyzer.object_name_hint

    return SourceAnalysis(
        model_py_sha256=hashlib.sha256(source.encode("utf-8")).hexdigest(),
        model_line_count=len(source.splitlines()),
        part_call_count=part_call_count,
        literal_part_names=tuple(literal_part_names),
        dynamic_part_call_expressions=tuple(dynamic_part_call_expressions),
        get_part_names=tuple(get_part_names),
        object_name_hint=object_name_hint,
    )


def _record_title_from_metadata(metadata: object) -> str | None:
    if not isinstance(metadata, dict):
        return None
    display = metadata.get("display")
    if not isinstance(display, dict):
        return None
    title = display.get("title")
    return title if isinstance(title, str) and title.strip() else None


def _record_category_from_metadata(metadata: object) -> str | None:
    if not isinstance(metadata, dict):
        return None
    category_slug = metadata.get("category_slug")
    return category_slug if isinstance(category_slug, str) and category_slug.strip() else None


def _record_prompt_preview_from_metadata(metadata: object) -> str | None:
    if not isinstance(metadata, dict):
        return None
    display = metadata.get("display")
    if not isinstance(display, dict):
        return None
    prompt_preview = display.get("prompt_preview")
    return prompt_preview if isinstance(prompt_preview, str) and prompt_preview.strip() else None


def _dedupe_preserve_order(values: Sequence[str]) -> tuple[str, ...]:
    seen: set[str] = set()
    ordered: list[str] = []
    for value in values:
        normalized = value.strip()
        if not normalized or normalized in seen:
            continue
        seen.add(normalized)
        ordered.append(normalized)
    return tuple(ordered)


def _available_part_names_from_materialized_urdf(
    record_dir: Path, record_id: str
) -> tuple[str, ...] | None:
    if len(record_dir.parents) < 3:
        return None
    repo_root = record_dir.parents[2]
    urdf_path = StorageRepo(repo_root).layout.record_materialization_urdf_path(record_id)
    if not urdf_path.exists():
        return None
    try:
        root = ET.fromstring(urdf_path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return _dedupe_preserve_order(
        [
            str(link.attrib.get("name", "")).strip()
            for link in root.findall(".//link")
            if str(link.attrib.get("name", "")).strip()
        ]
    )


def _fallback_available_part_names(source_analysis: SourceAnalysis) -> tuple[str, ...]:
    return _dedupe_preserve_order(
        [*source_analysis.literal_part_names, *source_analysis.get_part_names]
    )


def resolve_available_part_names(
    *,
    record_dir: Path,
    record_id: str,
    source_analysis: SourceAnalysis,
) -> tuple[tuple[str, ...], str, bool]:
    urdf_names = _available_part_names_from_materialized_urdf(record_dir, record_id)
    if urdf_names:
        return urdf_names, "materialized_urdf", True

    fallback_names = _fallback_available_part_names(source_analysis)
    is_complete = bool(fallback_names) and not source_analysis.dynamic_part_call_expressions
    return fallback_names, "source_fallback", is_complete


def _relevant_source_call(node: ast.Call) -> bool:
    func = node.func
    if isinstance(func, ast.Attribute) and func.attr in {"part", "get_part", "articulation"}:
        return True
    return isinstance(func, ast.Name) and func.id == "ArticulatedObject"


def _node_line_span(node: ast.AST) -> tuple[int, int]:
    start = getattr(node, "lineno", 0) or 0
    end = getattr(node, "end_lineno", start) or start
    return start, max(start, end)


def _merge_line_ranges(ranges: Sequence[tuple[int, int]]) -> list[tuple[int, int]]:
    merged: list[tuple[int, int]] = []
    for start, end in sorted(ranges):
        if not merged or start > merged[-1][1] + 1:
            merged.append((start, end))
            continue
        merged[-1] = (merged[-1][0], max(merged[-1][1], end))
    return merged


def _format_line_ranges(source: str, ranges: Sequence[tuple[int, int]]) -> str:
    lines = source.splitlines()
    if not lines:
        return ""
    width = max(3, len(str(len(lines))))
    chunks: list[str] = []
    previous_end = 0
    for start, end in ranges:
        start = max(1, start)
        end = min(len(lines), end)
        if previous_end and start > previous_end + 1:
            chunks.append(f"... omitted lines {previous_end + 1}-{start - 1} ...")
        for line_no in range(start, end + 1):
            chunks.append(f"{line_no:0{width}d}: {lines[line_no - 1]}")
        previous_end = end
    return "\n".join(chunks)


def extract_relevant_model_context(source: str) -> str:
    lines = source.splitlines()
    if not lines:
        return ""

    try:
        tree = ast.parse(source)
    except SyntaxError:
        fallback_ranges: list[tuple[int, int]] = []
        for index, line in enumerate(lines, start=1):
            if any(
                token in line
                for token in (".part(", "get_part(", ".articulation(", "ArticulatedObject(")
            ):
                fallback_ranges.append(
                    (
                        max(1, index - RELEVANT_CONTEXT_RADIUS_LINES),
                        min(len(lines), index + RELEVANT_CONTEXT_RADIUS_LINES),
                    )
                )
        if not fallback_ranges:
            fallback_ranges = [(1, min(len(lines), 80))]
        return _format_line_ranges(source, _merge_line_ranges(fallback_ranges))

    relevant_ranges: list[tuple[int, int]] = []
    function_nodes = sorted(
        (
            node
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        ),
        key=lambda item: getattr(item, "lineno", 0),
    )

    for function_node in function_nodes:
        relevant_calls = sorted(
            (
                node
                for node in ast.walk(function_node)
                if isinstance(node, ast.Call) and _relevant_source_call(node)
            ),
            key=lambda item: getattr(item, "lineno", 0),
        )
        if not relevant_calls:
            continue

        function_start, function_end = _node_line_span(function_node)
        function_length = function_end - function_start + 1
        if function_length <= MAX_FULL_FUNCTION_EXCERPT_LINES and function_node.name not in {
            "build_object_model",
            "run_tests",
        }:
            relevant_ranges.append((function_start, function_end))
            continue

        relevant_ranges.append((function_start, min(function_end, function_start + 2)))
        for call_node in relevant_calls:
            call_start, call_end = _node_line_span(call_node)
            relevant_ranges.append(
                (
                    max(function_start, call_start - RELEVANT_CONTEXT_RADIUS_LINES),
                    min(function_end, call_end + RELEVANT_CONTEXT_RADIUS_LINES),
                )
            )

    if not relevant_ranges:
        relevant_ranges.append((1, min(len(lines), 80)))

    return _format_line_ranges(source, _merge_line_ranges(relevant_ranges))


def prepare_record(task: RecordTask) -> PreparedRecord:
    model_path = task.record_dir / "model.py"
    if not model_path.exists():
        raise SkipTaskError("record is missing model.py")
    model_source = model_path.read_text(encoding="utf-8")
    source_analysis = analyze_model_source(model_source)
    if source_analysis.part_call_count <= 0:
        raise SkipTaskError("record has no model.part(...) calls")
    record_json_path = task.record_dir / "record.json"
    metadata = (
        json.loads(record_json_path.read_text(encoding="utf-8"))
        if record_json_path.exists()
        else None
    )
    (
        available_part_names,
        available_part_names_source,
        available_part_names_complete,
    ) = resolve_available_part_names(
        record_dir=task.record_dir,
        record_id=task.record_id,
        source_analysis=source_analysis,
    )
    return PreparedRecord(
        task=task,
        model_path=model_path,
        title=_record_title_from_metadata(metadata),
        prompt_preview=_record_prompt_preview_from_metadata(metadata),
        category_slug=_record_category_from_metadata(metadata),
        model_source=model_source,
        extracted_model_context=extract_relevant_model_context(model_source),
        available_part_names=available_part_names,
        available_part_names_source=available_part_names_source,
        available_part_names_complete=available_part_names_complete,
        source_analysis=source_analysis,
    )


def build_output_json_schema() -> dict[str, Any]:
    return {
        "type": "json_schema",
        "name": "link_name_backfill_manifest",
        "description": "A per-link rename manifest for one articulated object record.",
        "strict": True,
        "schema": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "record_id": {"type": "string"},
                "global_shape_context": {"type": "string"},
                "intrinsic_frame": {
                    "type": "object",
                    "additionalProperties": False,
                    "properties": {
                        axis_name: {
                            "type": "object",
                            "additionalProperties": False,
                            "properties": {
                                "status": {
                                    "type": "string",
                                    "enum": sorted(FRAME_STATUSES),
                                },
                                "reason": {"type": "string"},
                            },
                            "required": ["status", "reason"],
                        }
                        for axis_name in ("front_back", "left_right")
                    },
                    "required": ["front_back", "left_right"],
                },
                "enumeration_complete": {"type": "boolean"},
                "general_notes": {"type": "string"},
                "links": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "additionalProperties": False,
                        "properties": {
                            "current_name": {"type": "string"},
                            "status": {
                                "type": "string",
                                "enum": sorted(LINK_STATUSES),
                            },
                            "suggested_name": {"type": ["string", "null"]},
                            "reason": {"type": "string"},
                        },
                        "required": ["current_name", "status", "suggested_name", "reason"],
                    },
                },
            },
            "required": [
                "record_id",
                "global_shape_context",
                "intrinsic_frame",
                "enumeration_complete",
                "general_notes",
                "links",
            ],
        },
    }


def build_user_prompt(prepared: PreparedRecord) -> str:
    available_part_lines = [f"- {name}" for name in prepared.available_part_names] or ["- <none>"]

    title_line = prepared.title if prepared.title is not None else "<unknown>"
    prompt_preview_line = (
        prepared.prompt_preview if prepared.prompt_preview is not None else "<unknown>"
    )
    category_line = prepared.category_slug if prepared.category_slug is not None else "<unknown>"
    object_name_line = (
        prepared.source_analysis.object_name_hint
        if prepared.source_analysis.object_name_hint is not None
        else "<unknown>"
    )

    return (
        f"Record ID: {prepared.task.record_id}\n"
        f"Record title: {title_line}\n"
        f"Prompt preview: {prompt_preview_line}\n"
        f"Category slug: {category_line}\n"
        f"ArticulatedObject name hint: {object_name_line}\n"
        f"Available articulated part count: {len(prepared.available_part_names)}\n"
        f"Available part-name source: {prepared.available_part_names_source}\n"
        f"Available part-name list is authoritative: {str(prepared.available_part_names_complete).lower()}\n"
        "\n"
        "Important scope:\n"
        "- First reason about the whole object as a global 3D shape/mechanism, not just isolated part names.\n"
        "- You are not receiving source code. You are receiving only record metadata and an articulated part-name list.\n"
        "- Review exactly the provided available articulated part names. Do not invent extra part names that are not listed.\n"
        "- If the available part-name list is not authoritative and the missing context could materially change a naming decision, use review instead of guessing.\n"
        "- It is possible, and in some cases highly likely, that the original link names' left/right or front/rear labeling is wrong, arbitrary, or should not be present.\n"
        "- Do not trust the existing link names as evidence that a directional distinction is valid; use the provided metadata, part-name list, and represented whole-object shape to decide that independently.\n"
        "- Only include distinctions that are grounded in the object's physical structure, articulated layout, or clearly implied visible grouping from the provided metadata and part-name list.\n"
        "- Do not include functional labels, legends, control mappings, or task-specific roles in the link name when they are not physically grounded in the represented object shape.\n"
        "- For keyboards, keypads, button panels, and similar control surfaces, do not name repeated physical controls after their printed legend or logical function such as escape, shift, ctrl, arrow_left, or numpad_7 unless that distinction is itself physically grounded; prefer generic physical names such as key, button, or knob plus numbering when the parts are just repeated controls.\n"
        "- Whenever semantically identical parts repeat, prefer numbering with _0, _1, ... rather than left/right or front/rear distinctions.\n"
        "- If repeated parts are semantically identical, collapse them to a numbered family instead of preserving or correcting directional labels.\n"
        "- When you switch a repeated-part family to numbering, replace the old numbering or labeling format with the new one instead of preserving both.\n"
        "- Do not carry legacy row/column indices, side labels, letter suffixes, or other old enumeration fragments into the new numbered name unless that exact structure is itself the intended new numbering scheme.\n"
        "- Reason carefully about whether the object has a meaningful canonical pose, only a partial intrinsic frame, or no reliable intrinsic frame at all.\n"
        "- Explicitly consider whether front/back, left/right, top/bottom, inner/outer, or similar distinctions are truly object-intrinsic rather than arbitrary.\n"
        "- Use the object's geometry, support structure, articulation layout, and typical use to judge whether directional naming is justified.\n"
        "- Do not use left/right or front/rear merely to distinguish repeated semantically identical parts, even if the object has a meaningful global intrinsic frame.\n"
        "- For cabinets, stoves, ovens, appliances, desks, and similar objects, repeated semantically identical doors, drawers, knobs, handles, buttons, or supports should usually be numbered rather than labeled with left/right or front/rear.\n"
        "- Even if the object has a meaningful global front/back axis, do not assume every repeated peripheral part inherits intrinsic directional labels.\n"
        "- For repeated radial or perimeter supports around a central hub or column, such as tripod legs or similar evenly distributed supports, prefer numbering unless the source gives a genuinely intrinsic distinction.\n"
        "- Do not propagate an object-level front/back or left/right frame onto repeated radial supports unless those supports are themselves intrinsically distinguishable.\n"
        "- Review only articulated link/part names created by model.part(...).\n"
        "- Do not review visual names, mesh filenames, material names, helper variables, or the object name.\n"
        "- Return every provided current link name exactly once.\n"
        "- All final link names after applying renames must be unique across the object; no two links may share the same final name.\n"
        "- Keep compliant names unchanged.\n"
        "- If the provided part-name list does not seem sufficient to support a confident decision, set enumeration_complete=false and use review for uncertain items.\n"
        "- Populate global_shape_context with a short description of the object's overall shape, layout, and articulated structure.\n"
        "\n"
        "Available articulated part names:\n"
        f"{chr(10).join(available_part_lines)}\n"
        "\n"
        "Return the links in order of first appearance / natural construction order.\n"
        "Use review rather than guessing when the rename or the intrinsic frame is uncertain.\n"
    )


def build_response_request(
    *,
    prepared: PreparedRecord,
    system_prompt: str,
    model: str,
    reasoning_effort: str,
) -> dict[str, Any]:
    return {
        "model": model,
        "reasoning": {"effort": reasoning_effort},
        "text": {"format": build_output_json_schema()},
        "input": [
            {
                "role": "system",
                "content": [{"type": "input_text", "text": system_prompt}],
            },
            {
                "role": "user",
                "content": [{"type": "input_text", "text": build_user_prompt(prepared)}],
            },
        ],
    }


def get_usage_field(value: Any, *path: str) -> int:
    current = value
    for key in path:
        if current is None:
            return 0
        if isinstance(current, dict):
            current = current.get(key)
        else:
            current = getattr(current, key, None)
    if isinstance(current, bool):
        return 0
    if isinstance(current, int):
        return current
    if isinstance(current, float) and current.is_integer():
        return int(current)
    return 0


def extract_usage_totals(response: Any) -> UsageTotals:
    usage = getattr(response, "usage", None)
    return UsageTotals(
        input_tokens=get_usage_field(usage, "input_tokens"),
        cached_input_tokens=get_usage_field(usage, "input_tokens_details", "cached_tokens"),
        output_tokens=get_usage_field(usage, "output_tokens"),
        reasoning_tokens=get_usage_field(usage, "output_tokens_details", "reasoning_tokens"),
    )


def clean_response_text(text: str) -> str:
    cleaned = text.strip()
    if cleaned.startswith("```"):
        lines = cleaned.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        cleaned = "\n".join(lines).strip()
    return cleaned


def extract_json_text(text: str) -> str:
    cleaned = clean_response_text(text)
    if cleaned.startswith("{") and cleaned.endswith("}"):
        return cleaned
    start = cleaned.find("{")
    end = cleaned.rfind("}")
    if start != -1 and end > start:
        return cleaned[start : end + 1]
    raise ValueError("Response does not contain a JSON object")


def parse_response_as_json(text: str) -> dict[str, Any]:
    json_text = extract_json_text(text)
    parsed = json.loads(json_text)
    if not isinstance(parsed, dict):
        raise ValueError("Expected a JSON object response")
    return parsed


def _validate_frame_axis(value: Any, *, axis_name: str) -> dict[str, str]:
    if not isinstance(value, dict):
        raise ValueError(f"intrinsic_frame.{axis_name} must be an object")
    status = value.get("status")
    reason = value.get("reason")
    if status not in FRAME_STATUSES:
        raise ValueError(
            f"intrinsic_frame.{axis_name}.status must be one of {sorted(FRAME_STATUSES)}"
        )
    if not isinstance(reason, str) or not reason.strip():
        raise ValueError(f"intrinsic_frame.{axis_name}.reason must be a non-empty string")
    return {"status": str(status), "reason": reason.strip()}


def _validate_suggested_name(name: str) -> str:
    normalized = "_".join(name.strip().split())
    if not normalized:
        raise ValueError("suggested_name must not be empty")
    if not SUGGESTED_NAME_PATTERN.fullmatch(normalized):
        raise ValueError(
            "suggested_name must be lowercase snake_case with at most 5 underscore-separated words"
        )
    tokens = normalized.split("_")
    if len(tokens) > 5:
        raise ValueError("suggested_name must contain at most 5 words")
    if FINAL_SINGLE_LETTER_TOKEN_PATTERN.fullmatch(tokens[-1]):
        raise ValueError("suggested_name must use numeric suffixes instead of letter suffixes")
    banned_tokens = sorted(token for token in tokens if token in BANNED_STATE_TOKENS)
    if banned_tokens:
        raise ValueError(
            f"suggested_name contains articulation-state token(s): {', '.join(banned_tokens)}"
        )
    return normalized


def validate_manifest_payload(
    manifest: dict[str, Any],
    *,
    prepared: PreparedRecord,
) -> dict[str, Any]:
    record_id = manifest.get("record_id")
    if record_id != prepared.task.record_id:
        raise ValueError(
            f"record_id mismatch: expected {prepared.task.record_id!r}, got {record_id!r}"
        )

    global_shape_context = manifest.get("global_shape_context")
    if not isinstance(global_shape_context, str) or not global_shape_context.strip():
        raise ValueError("global_shape_context must be a non-empty string")

    intrinsic_frame = manifest.get("intrinsic_frame")
    if not isinstance(intrinsic_frame, dict):
        raise ValueError("intrinsic_frame must be an object")
    normalized_intrinsic_frame = {
        "front_back": _validate_frame_axis(
            intrinsic_frame.get("front_back"),
            axis_name="front_back",
        ),
        "left_right": _validate_frame_axis(
            intrinsic_frame.get("left_right"),
            axis_name="left_right",
        ),
    }

    enumeration_complete = manifest.get("enumeration_complete")
    if not isinstance(enumeration_complete, bool):
        raise ValueError("enumeration_complete must be a boolean")

    general_notes = manifest.get("general_notes")
    if not isinstance(general_notes, str):
        raise ValueError("general_notes must be a string")

    links = manifest.get("links")
    if not isinstance(links, list) or not links:
        raise ValueError("links must be a non-empty list")

    normalized_links: list[dict[str, Any]] = []
    current_names_seen: set[str] = set()
    final_names_seen: set[str] = set()

    for index, raw_link in enumerate(links):
        if not isinstance(raw_link, dict):
            raise ValueError(f"links[{index}] must be an object")
        current_name = raw_link.get("current_name")
        if not isinstance(current_name, str) or not current_name.strip():
            raise ValueError(f"links[{index}].current_name must be a non-empty string")
        current_name = current_name.strip()
        if current_name in current_names_seen:
            raise ValueError(f"Duplicate current_name in response: {current_name}")
        current_names_seen.add(current_name)

        status = raw_link.get("status")
        if status not in LINK_STATUSES:
            raise ValueError(f"links[{index}].status must be one of {sorted(LINK_STATUSES)}")

        reason = raw_link.get("reason")
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError(f"links[{index}].reason must be a non-empty string")
        reason = reason.strip()

        suggested_name_raw = raw_link.get("suggested_name")
        suggested_name: str | None
        if status == "rename":
            if not isinstance(suggested_name_raw, str):
                raise ValueError(f"links[{index}].suggested_name must be a string for rename")
            suggested_name = _validate_suggested_name(suggested_name_raw)
            if suggested_name == current_name:
                raise ValueError(
                    f"links[{index}] rename target matches current_name: {current_name}"
                )
            final_name = suggested_name
        else:
            if suggested_name_raw is not None:
                raise ValueError(
                    f"links[{index}].suggested_name must be null when status is {status}"
                )
            suggested_name = None
            final_name = current_name

        if final_name in final_names_seen:
            raise ValueError(f"Duplicate final link name in response: {final_name}")
        final_names_seen.add(final_name)

        normalized_links.append(
            {
                "current_name": current_name,
                "status": str(status),
                "suggested_name": suggested_name,
                "reason": reason,
            }
        )

    available_names = set(prepared.available_part_names)
    if available_names:
        missing_available_names = sorted(available_names - current_names_seen)
        if missing_available_names:
            raise ValueError(
                "Response omitted provided available part names: "
                + ", ".join(missing_available_names)
            )
        extra_names = sorted(current_names_seen - available_names)
        if extra_names:
            raise ValueError(
                "Response included unexpected link names outside the provided part-name list: "
                + ", ".join(extra_names)
            )
        if len(current_names_seen) != len(prepared.available_part_names):
            raise ValueError("Response link count does not match the provided part-name count")
        if prepared.available_part_names_complete and not enumeration_complete:
            raise ValueError(
                "enumeration_complete must be true when the provided part-name list is authoritative"
            )
    else:
        literal_names = set(prepared.source_analysis.literal_part_names)
        missing_literal_names = sorted(literal_names - current_names_seen)
        if missing_literal_names:
            raise ValueError(
                "Response omitted statically extracted literal part names: "
                + ", ".join(missing_literal_names)
            )
        if not prepared.source_analysis.dynamic_part_call_expressions:
            extra_names = sorted(current_names_seen - literal_names)
            if extra_names:
                raise ValueError(
                    "Response included unexpected link names for a literal-only record: "
                    + ", ".join(extra_names)
                )
            if len(current_names_seen) != len(prepared.source_analysis.literal_part_names):
                raise ValueError("Response link count does not match the literal part-name count")
            if not enumeration_complete:
                raise ValueError(
                    "enumeration_complete must be true when all part names are literal"
                )

    return {
        "record_id": prepared.task.record_id,
        "global_shape_context": global_shape_context.strip(),
        "intrinsic_frame": normalized_intrinsic_frame,
        "enumeration_complete": enumeration_complete,
        "general_notes": general_notes.strip(),
        "links": normalized_links,
    }


def summarize_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    links = manifest["links"]
    rename_count = sum(1 for link in links if link["status"] == "rename")
    review_count = sum(1 for link in links if link["status"] == "review")
    compliant_count = sum(1 for link in links if link["status"] == "compliant")
    return {
        "link_count": len(links),
        "compliant": compliant_count,
        "rename": rename_count,
        "review": review_count,
        "manual_review_required": review_count > 0 or not manifest["enumeration_complete"],
    }


def renames_from_manifest(manifest: dict[str, Any]) -> dict[str, str]:
    return {
        link["current_name"]: link["suggested_name"]
        for link in manifest["links"]
        if link["status"] == "rename"
    }


def save_manifest_result(
    *,
    prepared: PreparedRecord,
    manifest: dict[str, Any],
    usage: UsageTotals,
    output_path: Path,
    model: str,
    reasoning_effort: str,
) -> None:
    payload = {
        "record_id": prepared.task.record_id,
        "record_path": str(prepared.task.record_dir),
        "model_path": str(prepared.model_path),
        "title": prepared.title,
        "category_slug": prepared.category_slug,
        "object_name_hint": prepared.source_analysis.object_name_hint,
        "source_analysis": {
            "model_py_sha256": prepared.source_analysis.model_py_sha256,
            "model_line_count": prepared.source_analysis.model_line_count,
            "part_call_count": prepared.source_analysis.part_call_count,
            "literal_part_names": list(prepared.source_analysis.literal_part_names),
            "dynamic_part_call_expressions": list(
                prepared.source_analysis.dynamic_part_call_expressions
            ),
            "get_part_names": list(prepared.source_analysis.get_part_names),
        },
        "manifest": manifest,
        "renames": renames_from_manifest(manifest),
        "summary": summarize_manifest(manifest),
        "usage": {
            "input_tokens": usage.input_tokens,
            "cached_input_tokens": usage.cached_input_tokens,
            "output_tokens": usage.output_tokens,
            "reasoning_tokens": usage.reasoning_tokens,
        },
        "request": {
            "model": model,
            "reasoning_effort": reasoning_effort,
        },
    }
    atomic_write_json(output_path, payload)


def annotate_one_record(
    *,
    task: RecordTask,
    client: Any,
    system_prompt: str,
    model: str,
    reasoning_effort: str,
) -> TaskResult:
    try:
        prepared = prepare_record(task)
    except SkipTaskError as exc:
        return TaskResult(
            record_id=task.record_id, status="skipped", detail=str(exc), usage=UsageTotals()
        )

    request_payload = build_response_request(
        prepared=prepared,
        system_prompt=system_prompt,
        model=model,
        reasoning_effort=reasoning_effort,
    )
    response = client.responses.create(**request_payload)
    usage = extract_usage_totals(response)
    output_text = (getattr(response, "output_text", None) or "").strip()
    if not output_text:
        raise ResponseProcessingError(f"Empty model response for {task.record_id}", usage=usage)

    try:
        manifest = validate_manifest_payload(
            parse_response_as_json(output_text),
            prepared=prepared,
        )
    except Exception as exc:
        raise ResponseProcessingError(str(exc), usage=usage) from exc

    save_manifest_result(
        prepared=prepared,
        manifest=manifest,
        usage=usage,
        output_path=task.output_path,
        model=model,
        reasoning_effort=reasoning_effort,
    )
    return TaskResult(
        record_id=task.record_id,
        status="saved",
        usage=usage,
    )


def materialize_printable_payload(request_payload: dict[str, Any]) -> dict[str, Any]:
    return copy.deepcopy(request_payload)


def validate_record_dir(record_dir: Path) -> bool:
    return record_dir.is_dir() and (record_dir / "model.py").exists()


def resolve_record_selector_paths(record_selector: str, records_root: Path) -> list[Path]:
    cleaned = record_selector.strip()
    if not cleaned:
        raise ValueError("--record-dir must not be empty")

    if glob.has_magic(cleaned):
        raw_patterns: list[str] = []
        candidate = Path(cleaned)
        if candidate.is_absolute():
            raw_patterns.append(cleaned)
        else:
            raw_patterns.append(str(records_root / cleaned.strip("/")))
            raw_patterns.append(str((Path.cwd() / candidate).resolve()))

        matches: list[Path] = []
        seen: set[Path] = set()
        for raw_pattern in raw_patterns:
            for match in glob.glob(raw_pattern):
                path = Path(match).resolve()
                if path.is_dir() and path not in seen:
                    seen.add(path)
                    matches.append(path)
        return sorted(matches)

    candidate = Path(cleaned)
    candidate_paths: list[Path] = []
    if candidate.is_absolute():
        candidate_paths.append(candidate.resolve())
    else:
        candidate_paths.append((Path.cwd() / candidate).resolve())
        candidate_paths.append((records_root / cleaned.strip("/")).resolve())

    seen_candidates: set[Path] = set()
    for path in candidate_paths:
        if path in seen_candidates:
            continue
        seen_candidates.add(path)
        if path.exists():
            if not path.is_dir():
                raise ValueError(f"Resolved path is not a record directory: {path}")
            return [path]
    return [candidate_paths[-1]]


def collect_tasks(
    *,
    repo_root: Path,
    output_root: Path,
    record_dir: str | None,
    skip_existing: bool,
) -> tuple[list[RecordTask], int]:
    repo = StorageRepo(repo_root)
    records_root = repo.layout.records_root
    if not records_root.exists():
        raise FileNotFoundError(f"Records root not found: {records_root}")

    skipped_existing = 0
    if record_dir is not None:
        record_paths = resolve_record_selector_paths(record_dir, records_root)
        if not record_paths:
            raise FileNotFoundError(f"No record directories matched selector: {record_dir}")
        tasks: list[RecordTask] = []
        for path in record_paths:
            if not path.exists():
                raise FileNotFoundError(f"Record directory not found: {path}")
            if not validate_record_dir(path):
                raise ValueError(f"Record directory is missing model.py: {path}")
            task = RecordTask(
                record_dir=path,
                record_id=path.name,
                output_path=output_root / f"{path.name}.json",
            )
            if skip_existing and task.output_path.exists():
                skipped_existing += 1
                continue
            tasks.append(task)
        return sorted(tasks, key=lambda item: item.record_id), skipped_existing

    tasks = []
    for path in sorted(child for child in records_root.iterdir() if child.is_dir()):
        if not validate_record_dir(path):
            continue
        task = RecordTask(
            record_dir=path,
            record_id=path.name,
            output_path=output_root / f"{path.name}.json",
        )
        if skip_existing and task.output_path.exists():
            skipped_existing += 1
            continue
        tasks.append(task)
    return tasks, skipped_existing


def resolve_api_key(preferred_env_name: str) -> str:
    checked: list[str] = []
    for env_name in (preferred_env_name, "OPENAI_API_KEY", "OPENAI_API_KEYS"):
        if env_name in checked:
            continue
        checked.append(env_name)
        raw = os.environ.get(env_name, "").strip()
        if not raw:
            continue
        if env_name == "OPENAI_API_KEYS":
            keys = [token.strip() for token in raw.replace("\n", ",").split(",") if token.strip()]
            if keys:
                return keys[0]
            continue
        return raw
    raise EnvironmentError(
        "Missing OpenAI API key. Set one of " + ", ".join(f"${name}" for name in checked) + "."
    )


def build_openai_client(api_key: str) -> Any:
    try:
        from openai import OpenAI  # type: ignore
    except Exception as exc:
        raise RuntimeError(
            "The `openai` package is required for this script. Install it, then try again."
        ) from exc
    return OpenAI(api_key=api_key)


def run_parallel_backfill(
    *,
    tasks: Sequence[RecordTask],
    api_key: str,
    system_prompt: str,
    model: str,
    reasoning_effort: str,
    n_concurrency: int,
) -> tuple[int, int, list[str], UsageTotals]:
    saved_count = 0
    skipped_count = 0
    failures: list[str] = []
    usage_totals = UsageTotals()
    usage_lock = Lock()
    client_local = local()

    def get_client() -> Any:
        client = getattr(client_local, "client", None)
        if client is None:
            client = build_openai_client(api_key)
            client_local.client = client
        return client

    def worker(task: RecordTask) -> TaskResult:
        return annotate_one_record(
            task=task,
            client=get_client(),
            system_prompt=system_prompt,
            model=model,
            reasoning_effort=reasoning_effort,
        )

    with ThreadPoolExecutor(max_workers=max(1, n_concurrency)) as executor:
        future_to_task = {executor.submit(worker, task): task for task in tasks}
        for future in as_completed(future_to_task):
            task = future_to_task[future]
            try:
                result = future.result()
            except ResponseProcessingError as exc:
                failures.append(f"{task.record_id}: {exc}")
                with usage_lock:
                    usage_totals = usage_totals + exc.usage
                continue
            except Exception as exc:
                failures.append(f"{task.record_id}: {exc}")
                continue

            with usage_lock:
                usage_totals = usage_totals + (result.usage or UsageTotals())
            if result.status == "saved":
                saved_count += 1
                print(f"saved {result.record_id}", flush=True)
            elif result.status == "skipped":
                skipped_count += 1
                if result.detail:
                    print(f"skipped {result.record_id}: {result.detail}", flush=True)
            else:
                failures.append(f"{result.record_id}: unexpected status {result.status}")
    return saved_count, skipped_count, failures, usage_totals


def show_payload_for_task(
    *,
    task: RecordTask,
    system_prompt: str,
    model: str,
    reasoning_effort: str,
) -> None:
    prepared = prepare_record(task)
    request_payload = build_response_request(
        prepared=prepared,
        system_prompt=system_prompt,
        model=model,
        reasoning_effort=reasoning_effort,
    )
    printable_payload = materialize_printable_payload(request_payload)
    print(
        json.dumps(
            {
                "record_id": task.record_id,
                "title": prepared.title,
                "prompt_preview": prepared.prompt_preview,
                "category_slug": prepared.category_slug,
                "object_name_hint": prepared.source_analysis.object_name_hint,
                "available_part_names": list(prepared.available_part_names),
                "available_part_names_source": prepared.available_part_names_source,
                "available_part_names_complete": prepared.available_part_names_complete,
                "source_analysis": {
                    "model_py_sha256": prepared.source_analysis.model_py_sha256,
                    "model_line_count": prepared.source_analysis.model_line_count,
                    "part_call_count": prepared.source_analysis.part_call_count,
                    "literal_part_names": list(prepared.source_analysis.literal_part_names),
                    "dynamic_part_call_expressions": list(
                        prepared.source_analysis.dynamic_part_call_expressions
                    ),
                    "get_part_names": list(prepared.source_analysis.get_part_names),
                },
                "payload": printable_payload,
            },
            indent=2,
        )
    )


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    repo = StorageRepo(repo_root)
    output_root = (
        args.output_root.resolve()
        if args.output_root is not None
        else (repo.layout.manifests_root / "link_name_backfill_openai").resolve()
    )
    failures_output = (
        args.failures_output.resolve()
        if args.failures_output is not None
        else (output_root / DEFAULT_FAILURES_OUTPUT_NAME).resolve()
    )

    system_prompt = load_system_prompt(args.system_prompt)
    tasks, skipped_existing = collect_tasks(
        repo_root=repo_root,
        output_root=output_root,
        record_dir=args.record_dir,
        skip_existing=args.skip_existing,
    )

    print(
        f"found_tasks={len(tasks)} skipped_existing={skipped_existing} "
        f"repo_root={repo_root} output_root={output_root}",
        flush=True,
        file=sys.stderr,
    )

    if args.show_payload_only:
        if args.record_dir is None:
            print("[ERROR] --show-payload-only requires --record-dir", file=sys.stderr)
            return 2
        if len(tasks) != 1:
            print("[ERROR] --show-payload-only requires exactly one record", file=sys.stderr)
            return 2
        show_payload_for_task(
            task=tasks[0],
            system_prompt=system_prompt,
            model=args.model,
            reasoning_effort=args.reasoning_effort,
        )
        return 0

    api_key = resolve_api_key(args.api_key_env)
    (
        saved_count,
        skipped_count_direct,
        failures,
        usage_totals,
    ) = run_parallel_backfill(
        tasks=tasks,
        api_key=api_key,
        system_prompt=system_prompt,
        model=args.model,
        reasoning_effort=args.reasoning_effort,
        n_concurrency=args.n_concurrency,
    )
    skipped_count = skipped_existing + skipped_count_direct

    print(
        json.dumps(
            {
                "saved": saved_count,
                "skipped": skipped_count,
                "failed": len(failures),
                "usage": {
                    "input_tokens": usage_totals.input_tokens,
                    "cached_input_tokens": usage_totals.cached_input_tokens,
                    "output_tokens": usage_totals.output_tokens,
                    "reasoning_tokens": usage_totals.reasoning_tokens,
                },
            },
            indent=2,
        ),
        flush=True,
    )

    if failures:
        write_lines(failures_output, failures)
        print(f"wrote failures to {failures_output}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
