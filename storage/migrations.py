from __future__ import annotations

import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from storage.repo import StorageRepo
from storage.revisions import (
    INITIAL_REVISION_ID,
    build_revision_payload,
    revision_artifacts_payload,
    sha256_file,
)


@dataclass(slots=True, frozen=True)
class RecordsV3MigrationResult:
    migrated_record_ids: list[str] = field(default_factory=list)
    skipped_record_ids: list[str] = field(default_factory=list)
    planned_record_ids: list[str] = field(default_factory=list)
    removed_paths: list[Path] = field(default_factory=list)
    dry_run: bool = False

    @property
    def migrated_count(self) -> int:
        return len(self.migrated_record_ids)

    @property
    def skipped_count(self) -> int:
        return len(self.skipped_record_ids)


def _read_workbench_entries(repo: StorageRepo) -> dict[str, dict[str, Any]]:
    payload = repo.read_json(repo.layout.local_workbench_path(), default={}) or {}
    entries = payload.get("entries") if isinstance(payload, dict) else None
    if not isinstance(entries, list):
        return {}
    result: dict[str, dict[str, Any]] = {}
    for item in entries:
        if not isinstance(item, dict):
            continue
        record_id = str(item.get("record_id") or "")
        if record_id:
            result[record_id] = item
    return result


def _move_if_exists(source: Path, destination: Path, *, dry_run: bool) -> bool:
    if not source.exists():
        return False
    if dry_run:
        return True
    before = _path_fingerprint(source)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise FileExistsError(f"Destination already exists: {destination}")
    shutil.move(str(source), str(destination))
    after = _path_fingerprint(destination)
    if before != after:
        raise RuntimeError(f"Hash verification failed while moving {source} to {destination}")
    return True


def _path_fingerprint(path: Path) -> list[tuple[str, str | None]]:
    if path.is_file():
        return [(".", sha256_file(path))]
    if path.is_dir():
        return [
            (item.relative_to(path).as_posix(), sha256_file(item))
            for item in sorted(path.rglob("*"))
            if item.is_file()
        ]
    return []


def _unlink_if_exists(path: Path, *, dry_run: bool) -> bool:
    if not path.exists():
        return False
    if not dry_run:
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
    return True


def _write_workbench_sidecars(
    repo: StorageRepo,
    entries: dict[str, dict[str, Any]],
) -> None:
    records_root = repo.layout.records_root
    for record_id, entry in entries.items():
        record_dir = records_root / record_id
        if not record_dir.is_dir():
            continue
        repo.write_json(repo.layout.record_workbench_entry_path(record_id), entry)


def migrate_records_to_v3(repo: StorageRepo, *, dry_run: bool = False) -> RecordsV3MigrationResult:
    repo.ensure_layout()
    workbench_entries = _read_workbench_entries(repo)
    migrated: list[str] = []
    skipped: list[str] = []
    planned: list[str] = []
    removed: list[Path] = []

    records_root = repo.layout.records_root
    if not records_root.exists():
        return RecordsV3MigrationResult(dry_run=dry_run)

    for record_dir in sorted(path for path in records_root.iterdir() if path.is_dir()):
        record_id = record_dir.name
        record_path = repo.layout.record_metadata_path(record_id)
        record = repo.read_json(record_path)
        if not isinstance(record, dict):
            skipped.append(record_id)
            continue
        if record.get("schema_version") == 3:
            skipped.append(record_id)
            continue
        planned.append(record_id)
        if dry_run:
            continue

        revision_id = INITIAL_REVISION_ID
        revision_dir = repo.layout.record_revision_dir(record_id, revision_id)
        revision_dir.mkdir(parents=True, exist_ok=True)

        moved_cost = _move_if_exists(
            record_dir / "cost.json", revision_dir / "cost.json", dry_run=False
        )
        _move_if_exists(record_dir / "prompt.txt", revision_dir / "prompt.txt", dry_run=False)
        _move_if_exists(record_dir / "model.py", revision_dir / "model.py", dry_run=False)
        _move_if_exists(
            record_dir / "provenance.json", revision_dir / "provenance.json", dry_run=False
        )
        _move_if_exists(record_dir / "inputs", revision_dir / "inputs", dry_run=False)
        _move_if_exists(record_dir / "traces", revision_dir / "traces", dry_run=False)

        dataset_entry_path = repo.layout.legacy_record_dataset_entry_path(record_id)
        dataset_entry = repo.read_json(dataset_entry_path)
        if isinstance(dataset_entry, dict):
            repo.write_json(repo.layout.record_dataset_entry_path(record_id), dataset_entry)
            if _unlink_if_exists(dataset_entry_path, dry_run=False):
                removed.append(dataset_entry_path)

        workbench_entry = workbench_entries.get(record_id)
        if isinstance(workbench_entry, dict):
            repo.write_json(repo.layout.record_workbench_entry_path(record_id), workbench_entry)

        prompt_text = ""
        prompt_path = revision_dir / "prompt.txt"
        if prompt_path.exists():
            prompt_text = prompt_path.read_text(encoding="utf-8")
        provenance = repo.read_json(revision_dir / "provenance.json", default={}) or {}
        generation = provenance.get("generation") if isinstance(provenance, dict) else {}
        run_summary = provenance.get("run_summary") if isinstance(provenance, dict) else {}
        source = record.get("source") if isinstance(record.get("source"), dict) else {}
        artifacts = revision_artifacts_payload(revision_id=revision_id, has_cost_file=moved_cost)
        hashes = {
            "prompt_sha256": (record.get("hashes") or {}).get("prompt_sha256")
            if isinstance(record.get("hashes"), dict)
            else None,
            "model_py_sha256": sha256_file(revision_dir / "model.py"),
        }
        repo.write_json(
            repo.layout.record_revision_metadata_path(record_id, revision_id),
            build_revision_payload(
                record_id=record_id,
                revision_id=revision_id,
                created_at=str(record.get("created_at") or ""),
                prompt_text=prompt_text,
                prompt_kind=str(record.get("prompt_kind") or "single_prompt"),
                source=source if isinstance(source, dict) else {},
                generation=generation if isinstance(generation, dict) else {},
                artifacts=artifacts,
                hashes=hashes,
                run_summary=run_summary if isinstance(run_summary, dict) else {},
            ),
        )

        record["schema_version"] = 3
        record["active_revision_id"] = revision_id
        record["artifacts"] = artifacts
        record["hashes"] = hashes
        record["lineage"] = {
            "origin_record_id": record_id,
            "parent_record_id": None,
            "parent_revision_id": None,
            "edit_mode": "root",
        }
        repo.write_json(record_path, record)
        migrated.append(record_id)

    if not dry_run:
        _write_workbench_sidecars(repo, workbench_entries)
        local_workbench = repo.layout.local_workbench_path()
        if local_workbench.exists():
            local_workbench.unlink()
            removed.append(local_workbench)
        for cache_dir in (
            repo.layout.trajectory_unroll_root,
            repo.layout.record_materializations_root,
        ):
            if cache_dir.exists():
                shutil.rmtree(cache_dir)
                cache_dir.mkdir(parents=True, exist_ok=True)
                removed.append(cache_dir)

    return RecordsV3MigrationResult(
        migrated_record_ids=migrated,
        skipped_record_ids=skipped,
        planned_record_ids=planned,
        removed_paths=removed,
        dry_run=dry_run,
    )
