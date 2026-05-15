from __future__ import annotations

import hashlib
import json
from pathlib import Path

from storage.collections import CollectionStore
from storage.data_validation import validate_data_format
from storage.migrations import migrate_records_to_v3
from storage.repo import StorageRepo


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _write_system_prompt(repo: StorageRepo) -> str:
    text = "system prompt\n"
    digest = hashlib.sha256(text.encode("utf-8")).hexdigest()
    repo.layout.system_prompts_root.mkdir(parents=True, exist_ok=True)
    repo.layout.system_prompt_path(digest).write_text(text, encoding="utf-8")
    return digest


def _write_v2_dataset_record(repo: StorageRepo, record_id: str = "rec_hinge_0001") -> None:
    prompt = "Make a hinge\n"
    model = "# model\n"
    prompt_sha = _write_system_prompt(repo)
    _write_json(
        repo.layout.category_metadata_path("hinge"),
        {
            "schema_version": 1,
            "slug": "hinge",
            "title": "Hinge",
            "description": "",
        },
    )
    record_dir = repo.layout.record_dir(record_id)
    record_dir.mkdir(parents=True, exist_ok=True)
    (record_dir / "prompt.txt").write_text(prompt, encoding="utf-8")
    (record_dir / "model.py").write_text(model, encoding="utf-8")
    (record_dir / "cost.json").write_text("{}\n", encoding="utf-8")
    inputs_dir = record_dir / "inputs"
    inputs_dir.mkdir()
    (inputs_dir / "reference.png").write_bytes(b"fake image")
    traces_dir = record_dir / "traces"
    traces_dir.mkdir()
    (traces_dir / "trajectory.jsonl").write_text(
        '{"type":"message","message":{"role":"assistant","content":"done"}}\n',
        encoding="utf-8",
    )
    _write_json(
        record_dir / "provenance.json",
        {
            "schema_version": 2,
            "record_id": record_id,
            "generation": {
                "provider": "openai",
                "model_id": "gpt-5.4",
                "thinking_level": "high",
                "max_turns": 10,
            },
            "prompting": {
                "system_prompt_file": "designer_system_prompt_openai.txt",
                "system_prompt_sha256": prompt_sha,
            },
            "sdk": {"sdk_package": "sdk", "sdk_version": "workspace", "sdk_fingerprint": None},
            "environment": {"python_version": "3.11.0", "platform": "darwin-arm64"},
            "run_summary": {
                "turn_count": 3,
                "tool_call_count": 5,
                "compile_attempt_count": 2,
                "final_status": "success",
            },
        },
    )
    _write_json(
        record_dir / "record.json",
        {
            "schema_version": 2,
            "record_id": record_id,
            "created_at": "2026-03-18T00:00:00Z",
            "updated_at": "2026-03-18T00:00:00Z",
            "rating": 5,
            "secondary_rating": None,
            "author": None,
            "rated_by": None,
            "secondary_rated_by": None,
            "kind": "generated_model",
            "prompt_kind": "single_prompt",
            "category_slug": "hinge",
            "source": {"run_id": "run_1"},
            "sdk_package": "sdk",
            "provider": "openai",
            "model_id": "gpt-5.4",
            "display": {"title": "Hinge", "prompt_preview": "Make a hinge"},
            "artifacts": {
                "prompt_txt": "prompt.txt",
                "prompt_series_json": None,
                "model_py": "model.py",
                "provenance_json": "provenance.json",
                "cost_json": "cost.json",
                "inputs_dir": "inputs",
            },
            "hashes": {
                "prompt_sha256": hashlib.sha256(prompt.encode("utf-8")).hexdigest(),
                "model_py_sha256": hashlib.sha256(model.encode("utf-8")).hexdigest(),
            },
            "collections": ["dataset", "workbench"],
        },
    )
    _write_json(
        record_dir / "dataset_entry.json",
        {
            "schema_version": 1,
            "record_id": record_id,
            "dataset_id": "ds_hinge_0001",
            "category_slug": "hinge",
            "promoted_at": "2026-03-18T00:01:00Z",
        },
    )
    _write_json(
        repo.layout.local_workbench_path(),
        {
            "schema_version": 1,
            "collection": "workbench",
            "updated_at": "2026-03-18T00:02:00Z",
            "entries": [
                {
                    "record_id": record_id,
                    "added_at": "2026-03-18T00:02:00Z",
                    "label": "draft",
                    "tags": ["migration"],
                    "archived": False,
                }
            ],
        },
    )


def test_migrate_records_to_v3_moves_flat_artifacts_and_sidecars(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_v2_dataset_record(repo)

    dry_run = migrate_records_to_v3(repo, dry_run=True)
    assert dry_run.dry_run is True
    assert dry_run.planned_record_ids == ["rec_hinge_0001"]
    assert (repo.layout.record_dir("rec_hinge_0001") / "model.py").exists()

    result = migrate_records_to_v3(repo)
    assert result.migrated_record_ids == ["rec_hinge_0001"]

    record_dir = repo.layout.record_dir("rec_hinge_0001")
    revision_dir = repo.layout.record_revision_dir("rec_hinge_0001", "rev_000001")
    assert (revision_dir / "prompt.txt").read_text(encoding="utf-8") == "Make a hinge\n"
    assert (revision_dir / "model.py").read_text(encoding="utf-8") == "# model\n"
    assert (revision_dir / "provenance.json").exists()
    assert (revision_dir / "cost.json").exists()
    assert (revision_dir / "inputs" / "reference.png").exists()
    assert (revision_dir / "traces" / "trajectory.jsonl").exists()
    assert (revision_dir / "revision.json").exists()

    for stale_name in (
        "prompt.txt",
        "model.py",
        "provenance.json",
        "cost.json",
        "inputs",
        "traces",
    ):
        assert not (record_dir / stale_name).exists()
    assert not (record_dir / "dataset_entry.json").exists()
    assert repo.layout.record_dataset_entry_path("rec_hinge_0001").exists()
    assert repo.layout.record_workbench_entry_path("rec_hinge_0001").exists()
    assert not repo.layout.local_workbench_path().exists()

    record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert record["schema_version"] == 3
    assert record["active_revision_id"] == "rev_000001"
    assert record["artifacts"]["model_py"] == "revisions/rev_000001/model.py"
    assert record["lineage"]["edit_mode"] == "root"

    validation = validate_data_format(repo)
    assert validation.errors == []


def test_workbench_load_merges_legacy_and_per_record_sidecars(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_json(
        repo.layout.local_workbench_path(),
        {
            "schema_version": 1,
            "collection": "workbench",
            "updated_at": "2026-03-18T00:02:00Z",
            "entries": [
                {
                    "record_id": "rec_legacy",
                    "added_at": "2026-03-18T00:02:00Z",
                    "label": "legacy",
                    "tags": [],
                    "archived": False,
                }
            ],
        },
    )
    _write_json(
        repo.layout.record_workbench_entry_path("rec_sidecar"),
        {
            "record_id": "rec_sidecar",
            "added_at": "2026-03-18T00:03:00Z",
            "label": "sidecar",
            "tags": ["new"],
            "archived": False,
        },
    )

    collection = CollectionStore(repo).load_workbench()

    assert collection is not None
    entries = {entry["record_id"]: entry for entry in collection["entries"]}
    assert set(entries) == {"rec_legacy", "rec_sidecar"}
    assert entries["rec_sidecar"]["tags"] == ["new"]


def test_migrate_preserves_legacy_workbench_entries_for_existing_v3_records(
    tmp_path: Path,
) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_v2_dataset_record(repo)
    existing_record_id = "rec_existing_v3"
    repo.layout.record_dir(existing_record_id).mkdir(parents=True, exist_ok=True)
    _write_json(
        repo.layout.record_metadata_path(existing_record_id),
        {
            "schema_version": 3,
            "record_id": existing_record_id,
            "active_revision_id": "rev_000001",
            "artifacts": {"model_py": "revisions/rev_000001/model.py"},
            "lineage": {
                "origin_record_id": existing_record_id,
                "parent_record_id": None,
                "parent_revision_id": None,
                "edit_mode": "root",
            },
            "collections": ["workbench"],
        },
    )
    _write_json(
        repo.layout.local_workbench_path(),
        {
            "schema_version": 1,
            "collection": "workbench",
            "updated_at": "2026-03-18T00:04:00Z",
            "entries": [
                {
                    "record_id": "rec_hinge_0001",
                    "added_at": "2026-03-18T00:02:00Z",
                    "label": "v2",
                    "tags": [],
                    "archived": False,
                },
                {
                    "record_id": existing_record_id,
                    "added_at": "2026-03-18T00:03:00Z",
                    "label": "already migrated",
                    "tags": ["keep"],
                    "archived": False,
                },
            ],
        },
    )

    result = migrate_records_to_v3(repo)

    assert result.migrated_record_ids == ["rec_hinge_0001"]
    assert existing_record_id in result.skipped_record_ids
    assert not repo.layout.local_workbench_path().exists()
    existing_entry = repo.read_json(repo.layout.record_workbench_entry_path(existing_record_id))
    assert isinstance(existing_entry, dict)
    assert existing_entry["label"] == "already migrated"
    collection = CollectionStore(repo).load_workbench()
    assert collection is not None
    assert {entry["record_id"] for entry in collection["entries"]} == {
        "rec_hinge_0001",
        existing_record_id,
    }


def test_validation_rejects_stale_flat_v2_files_on_v3_records(tmp_path: Path) -> None:
    repo = StorageRepo(tmp_path)
    repo.ensure_layout()
    _write_v2_dataset_record(repo)
    migrate_records_to_v3(repo)

    stale_model = repo.layout.record_dir("rec_hinge_0001") / "model.py"
    stale_model.write_text("# stale\n", encoding="utf-8")

    validation = validate_data_format(repo)

    assert any(
        "v3 records must not use flat v2 artifact paths" in error for error in validation.errors
    )
