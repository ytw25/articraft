from __future__ import annotations

import sys
from pathlib import Path
from tempfile import TemporaryDirectory

from fastapi.testclient import TestClient

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from storage.collections import CollectionStore
from storage.datasets import DatasetStore
from storage.models import DisplayMetadata, Record, RecordArtifacts, RunRecord, SourceRef
from storage.records import RecordStore
from storage.repo import StorageRepo
from storage.runs import RunStore
from viewer.api.app import create_app


def main() -> None:
    with TemporaryDirectory() as tmpdir:
        repo_root = Path(tmpdir)
        repo = StorageRepo(repo_root)
        repo.ensure_layout()

        record_store = RecordStore(repo)
        record = Record(
            schema_version=1,
            record_id="rec_001",
            created_at="2026-03-17T19:24:12Z",
            updated_at="2026-03-17T19:25:24Z",
            kind="generated_model",
            prompt_kind="single_prompt",
            category_slug="hinges",
            source=SourceRef(run_id="run_001"),
            sdk_package="sdk",
            provider="openai",
            model_id="gpt-5.4",
            display=DisplayMetadata(
                title="Test hinge model",
                prompt_preview="create a hinge with two linked parts",
            ),
            artifacts=RecordArtifacts(
                prompt_txt="prompt.txt",
                prompt_series_json=None,
                model_py="model.py",
                model_urdf="model.urdf",
                compile_report_json="compile_report.json",
                provenance_json="provenance.json",
                cost_json="cost.json",
            ),
            collections=["workbench"],
        )
        record_store.write_record(record)
        repo.write_json(
            repo.layout.record_dir("rec_001") / "compile_report.json",
            {
                "schema_version": 1,
                "record_id": "rec_001",
                "status": "success",
                "metrics": {"turn_count": 3},
            },
        )
        repo.write_json(
            repo.layout.record_dir("rec_001") / "provenance.json",
            {
                "schema_version": 1,
                "record_id": "rec_001",
                "generation": {"provider": "openai", "model_id": "gpt-5.4"},
            },
        )
        repo.write_json(
            repo.layout.record_dir("rec_001") / "cost.json",
            {
                "total": {
                    "tokens": {"total_tokens": 1000},
                    "costs_usd": {"total": 0.05},
                }
            },
        )
        trace_dir = repo.layout.run_staging_dir("run_001") / "rec_001" / "traces"
        trace_dir.mkdir(parents=True, exist_ok=True)
        (trace_dir / "conversation.jsonl").write_text(
            '{"type":"message","message":{"role":"assistant","content":"done"}}\n',
            encoding="utf-8",
        )

        collections = CollectionStore(repo)
        collections.append_workbench_entry(
            record_id="rec_001",
            added_at="2026-03-17T20:20:03Z",
            label="hinge trial",
            tags=["smoke"],
        )

        datasets = DatasetStore(repo)
        datasets.promote_record(
            record_id="rec_001",
            dataset_id="hinge_dataset_001",
            promoted_at="2026-03-17T20:21:03Z",
        )

        runs = RunStore(repo)
        runs.write_run(
            RunRecord(
                schema_version=1,
                run_id="run_001",
                run_mode="workbench_single",
                collection="workbench",
                created_at="2026-03-17T19:24:12Z",
                updated_at="2026-03-17T19:25:24Z",
                provider="openai",
                model_id="gpt-5.4",
                sdk_package="sdk",
                status="success",
                prompt_count=1,
            )
        )
        runs.append_result(
            "run_001",
            {
                "record_id": "rec_001",
                "status": "success",
                "turn_count": 3,
                "tool_call_count": 2,
                "compile_attempt_count": 1,
            },
        )

        client = TestClient(create_app(repo_root=repo_root))

        assert client.get("/health").json()["status"] == "ok"

        root_response = client.get("/")
        assert root_response.status_code == 200

        bootstrap = client.get("/api/bootstrap").json()
        assert bootstrap["repo_root"] == repo_root.resolve().as_posix()
        assert len(bootstrap["workbench_entries"]) == 1
        assert len(bootstrap["dataset_entries"]) == 1
        assert len(bootstrap["runs"]) == 1

        workbench = client.get("/api/collections/workbench").json()
        assert workbench[0]["record"]["record_id"] == "rec_001"

        dataset = client.get("/api/collections/dataset").json()
        assert dataset[0]["dataset_id"] == "hinge_dataset_001"

        run_summaries = client.get("/api/runs").json()
        assert run_summaries[0]["run_id"] == "run_001"
        assert run_summaries[0]["success_count"] == 1

        run_detail = client.get("/api/runs/run_001").json()
        assert run_detail["run"]["run_id"] == "run_001"
        assert run_detail["results"][0]["record_id"] == "rec_001"
        assert run_detail["records"][0]["summary"]["title"] == "Test hinge model"

        trace_file = client.get("/api/records/rec_001/traces/conversation.jsonl")
        assert trace_file.status_code == 200
        assert '"role":"assistant"' in trace_file.text

        missing = client.get("/api/runs/run_missing")
        assert missing.status_code == 404


if __name__ == "__main__":
    main()
