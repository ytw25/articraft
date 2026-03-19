from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

from agent import runner
from cli.workbench import main as workbench_main
from tests.helpers import FakeAgent


@pytest.fixture
def fake_agent(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(runner, "ArticraftAgent", FakeAgent)


def test_workbench_rerun_record_command(
    fake_agent: None,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    repo_root = tmp_path
    exit_code = asyncio.run(
        runner.run_from_input(
            "make a cabinet hinge",
            prompt_text="make a cabinet hinge",
            display_prompt="make a cabinet hinge",
            repo_root=repo_root,
            image_path=None,
            provider="openai",
            thinking_level="high",
            max_turns=30,
            system_prompt_path="designer_system_prompt.txt",
            sdk_package="sdk",
            sdk_docs_mode="full",
            label="hinge rerun",
            tags=["hinge"],
        )
    )
    assert exit_code == 0
    capsys.readouterr()

    record_dir = next((repo_root / "data" / "records").iterdir())
    original_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    original_run_id = original_record["source"]["run_id"]

    assert (
        workbench_main(
            [
                "--repo-root",
                str(repo_root),
                "rerun-record",
                str(record_dir),
            ]
        )
        == 0
    )

    updated_record = json.loads((record_dir / "record.json").read_text(encoding="utf-8"))
    assert updated_record["record_id"] == record_dir.name
    assert updated_record["source"]["run_id"] != original_run_id
    assert (repo_root / "data" / "cache" / "search.sqlite").exists()

    captured = capsys.readouterr().out
    assert f"reran record_id={record_dir.name}" in captured
    assert "search_index=" in captured
