from __future__ import annotations

import json
import sys

import pytest

from scripts.backfill_link_names_via_openai import (
    RecordTask,
    SourceAnalysis,
    UsageTotals,
    analyze_model_source,
    annotate_one_record,
    build_user_prompt,
    parse_args,
    resolve_available_part_names,
    validate_manifest_payload,
)


def test_parse_args_defaults_to_gpt54_mini_medium_and_supports_skip_existing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(sys, "argv", ["backfill_link_names_via_openai.py"])
    args = parse_args()

    assert args.model == "gpt-5.4-mini"
    assert args.reasoning_effort == "medium"
    assert args.skip_existing is False

    monkeypatch.setattr(
        sys,
        "argv",
        ["backfill_link_names_via_openai.py", "--skip-existing"],
    )
    args_with_flag = parse_args()
    assert args_with_flag.skip_existing is True


def test_analyze_model_source_extracts_literal_and_dynamic_part_calls() -> None:
    source = """
from sdk import ArticulatedObject

def build_object_model():
    model = ArticulatedObject(name="demo_tripod")
    base = model.part("base")
    helper = object_model.get_part("base")
    leg = model.part(name)
"""

    analysis = analyze_model_source(source)

    assert analysis.object_name_hint == "demo_tripod"
    assert analysis.part_call_count == 2
    assert analysis.literal_part_names == ("base",)
    assert analysis.dynamic_part_call_expressions == ("name",)
    assert analysis.get_part_names == ("base",)


def test_validate_manifest_payload_rejects_letter_suffix_rename(tmp_path) -> None:
    record_dir = tmp_path / "data" / "records" / "rec_demo"
    record_dir.mkdir(parents=True)
    (record_dir / "model.py").write_text(
        """
from sdk import ArticulatedObject

def build_object_model():
    model = ArticulatedObject(name="demo")
    model.part("base")
    model.part("leg")
""".strip()
        + "\n",
        encoding="utf-8",
    )
    task = RecordTask(
        record_dir=record_dir,
        record_id="rec_demo",
        output_path=tmp_path / "out.json",
    )
    prepared_analysis = SourceAnalysis(
        model_py_sha256="sha",
        model_line_count=6,
        part_call_count=2,
        literal_part_names=("base", "leg"),
        dynamic_part_call_expressions=(),
        get_part_names=(),
        object_name_hint="demo",
    )

    class Prepared:
        def __init__(self) -> None:
            self.task = task
            self.model_path = record_dir / "model.py"
            self.title = None
            self.prompt_preview = None
            self.category_slug = None
            self.model_source = ""
            self.extracted_model_context = ""
            self.available_part_names = ("base", "leg")
            self.available_part_names_source = "materialized_urdf"
            self.available_part_names_complete = True
            self.source_analysis = prepared_analysis

    manifest = {
        "record_id": "rec_demo",
        "global_shape_context": "A small object with a base and one repeated leg part.",
        "intrinsic_frame": {
            "front_back": {"status": "ambiguous", "reason": "No consistent front."},
            "left_right": {"status": "ambiguous", "reason": "No consistent left/right."},
        },
        "enumeration_complete": True,
        "general_notes": "",
        "links": [
            {
                "current_name": "base",
                "status": "compliant",
                "suggested_name": None,
                "reason": "Already concise.",
            },
            {
                "current_name": "leg",
                "status": "rename",
                "suggested_name": "leg_a",
                "reason": "Needs explicit indexing.",
            },
        ],
    }

    with pytest.raises(ValueError, match="numeric suffixes instead of letter suffixes"):
        validate_manifest_payload(manifest, prepared=Prepared())


def test_build_user_prompt_mentions_canonical_pose_reasoning(tmp_path) -> None:
    record_dir = tmp_path / "data" / "records" / "rec_prompt"
    record_dir.mkdir(parents=True)
    model_path = record_dir / "model.py"
    model_path.write_text(
        """
from sdk import ArticulatedObject

def build_object_model():
    model = ArticulatedObject(name="demo")
    model.part("base")
""".strip()
        + "\n",
        encoding="utf-8",
    )

    task = RecordTask(
        record_dir=record_dir,
        record_id="rec_prompt",
        output_path=tmp_path / "out.json",
    )

    class Prepared:
        def __init__(self) -> None:
            self.task = task
            self.model_path = model_path
            self.title = "Prompt Demo"
            self.prompt_preview = "A compact demo object with a simple base."
            self.category_slug = "demo"
            self.model_source = model_path.read_text(encoding="utf-8")
            self.extracted_model_context = '001: def build_object_model():\n002:     model = ArticulatedObject(name="demo")\n003:     model.part("base")'
            self.available_part_names = ("base",)
            self.available_part_names_source = "materialized_urdf"
            self.available_part_names_complete = True
            self.source_analysis = SourceAnalysis(
                model_py_sha256="sha",
                model_line_count=5,
                part_call_count=1,
                literal_part_names=("base",),
                dynamic_part_call_expressions=(),
                get_part_names=(),
                object_name_hint="demo",
            )

    prompt = build_user_prompt(Prepared())

    assert "canonical pose" in prompt
    assert "partial intrinsic frame" in prompt
    assert "front/back, left/right, top/bottom, inner/outer" in prompt
    assert "You are not receiving source code." in prompt
    assert "Review exactly the provided available articulated part names." in prompt
    assert "Available articulated part names:" in prompt
    assert "tripod legs" in prompt
    assert (
        "Do not propagate an object-level front/back or left/right frame onto repeated radial supports"
        in prompt
    )
    assert "original link names' left/right or front/rear labeling is wrong" in prompt
    assert "Do not trust the existing link names as evidence" in prompt
    assert (
        "Only include distinctions that are grounded in the object's physical structure" in prompt
    )
    assert "Do not include functional labels, legends, control mappings" in prompt
    assert "unless that distinction is itself physically grounded" in prompt
    assert "All final link names after applying renames must be unique across the object" in prompt
    assert "Whenever semantically identical parts repeat, prefer numbering" in prompt
    assert "collapse them to a numbered family" in prompt
    assert "replace the old numbering or labeling format with the new one" in prompt
    assert "Do not carry legacy row/column indices" in prompt
    assert (
        "repeated semantically identical doors, drawers, knobs, handles, buttons, or supports should usually be numbered"
        in prompt
    )
    assert "Relevant extracted model.py context follows" not in prompt


def test_resolve_available_part_names_prefers_materialized_urdf(tmp_path) -> None:
    record_dir = tmp_path / "data" / "records" / "rec_demo"
    record_dir.mkdir(parents=True)
    urdf_path = tmp_path / "data" / "cache" / "record_materialization" / "rec_demo" / "model.urdf"
    urdf_path.parent.mkdir(parents=True)
    urdf_path.write_text(
        """
<robot name="demo">
  <link name="base" />
  <link name="key_0" />
  <link name="key_1" />
</robot>
""".strip()
        + "\n",
        encoding="utf-8",
    )

    names, source, complete = resolve_available_part_names(
        record_dir=record_dir,
        record_id="rec_demo",
        source_analysis=SourceAnalysis(
            model_py_sha256="sha",
            model_line_count=1,
            part_call_count=1,
            literal_part_names=("base",),
            dynamic_part_call_expressions=("name",),
            get_part_names=(),
            object_name_hint="demo",
        ),
    )

    assert names == ("base", "key_0", "key_1")
    assert source == "materialized_urdf"
    assert complete is True


def test_annotate_one_record_saves_validated_manifest(tmp_path) -> None:
    record_dir = tmp_path / "data" / "records" / "rec_tripod"
    record_dir.mkdir(parents=True)
    (record_dir / "model.py").write_text(
        """
from sdk import ArticulatedObject

def build_object_model():
    model = ArticulatedObject(name="tripod")
    model.part("base")
    model.part("leg_a")
    model.part("leg_b")
""".strip()
        + "\n",
        encoding="utf-8",
    )
    (record_dir / "record.json").write_text(
        json.dumps({"display": {"title": "Tripod"}, "category_slug": "tripod"}) + "\n",
        encoding="utf-8",
    )

    task = RecordTask(
        record_dir=record_dir,
        record_id="rec_tripod",
        output_path=tmp_path / "manifests" / "rec_tripod.json",
    )

    response_payload = {
        "record_id": "rec_tripod",
        "global_shape_context": "A tripod stand with one central base and two repeated legs.",
        "intrinsic_frame": {
            "front_back": {"status": "ambiguous", "reason": "The stand has no true front."},
            "left_right": {
                "status": "ambiguous",
                "reason": "The legs are symmetric around the hub.",
            },
        },
        "enumeration_complete": True,
        "general_notes": "Number identical tripod legs.",
        "links": [
            {
                "current_name": "base",
                "status": "compliant",
                "suggested_name": None,
                "reason": "Already concise.",
            },
            {
                "current_name": "leg_a",
                "status": "rename",
                "suggested_name": "leg_0",
                "reason": "Identical repeated leg; use numeric suffix.",
            },
            {
                "current_name": "leg_b",
                "status": "rename",
                "suggested_name": "leg_1",
                "reason": "Identical repeated leg; use numeric suffix.",
            },
        ],
    }

    class FakeResponse:
        def __init__(self) -> None:
            self.output_text = json.dumps(response_payload)
            self.usage = {
                "input_tokens": 120,
                "input_tokens_details": {"cached_tokens": 20},
                "output_tokens": 50,
                "output_tokens_details": {"reasoning_tokens": 12},
            }

    class FakeResponses:
        def create(self, **kwargs):
            for message in kwargs["input"]:
                for item in message["content"]:
                    assert item["type"] == "input_text"
            assert kwargs["model"] == "gpt-5.4"
            return FakeResponse()

    class FakeClient:
        responses = FakeResponses()

    result = annotate_one_record(
        task=task,
        client=FakeClient(),
        system_prompt="System prompt.",
        model="gpt-5.4",
        reasoning_effort="high",
    )

    assert result.status == "saved"
    assert result.usage == UsageTotals(
        input_tokens=120,
        cached_input_tokens=20,
        output_tokens=50,
        reasoning_tokens=12,
    )

    saved = json.loads(task.output_path.read_text(encoding="utf-8"))
    assert saved["record_id"] == "rec_tripod"
    assert saved["title"] == "Tripod"
    assert saved["category_slug"] == "tripod"
    assert saved["renames"] == {"leg_a": "leg_0", "leg_b": "leg_1"}
    assert saved["summary"] == {
        "link_count": 3,
        "compliant": 1,
        "rename": 2,
        "review": 0,
        "manual_review_required": False,
    }
    assert saved["manifest"]["global_shape_context"].startswith("A tripod stand")
    assert "apply" not in saved
    assert "compile_check" not in saved
