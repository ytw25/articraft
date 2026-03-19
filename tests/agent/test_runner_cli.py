from __future__ import annotations

import pytest

from agent import runner


def test_runner_help_text(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit, match="0"):
        runner.main(["--help"])

    help_text = capsys.readouterr().out
    assert "--prompt" in help_text
    assert "--image" in help_text
    assert "--provider {gemini,openai}" in help_text
    assert "--openai-transport {http,websocket}" in help_text
    assert "--sdk-docs-mode {core,full,none}" in help_text
    assert "--collection {workbench,dataset}" in help_text
    assert "--dataset-id DATASET_ID" in help_text
    assert "--flash" not in help_text
    assert "--hybrid-sdk" not in help_text
    assert "--openai-reasoning-summary" not in help_text
    assert "Currently supported only with --provider openai." not in help_text


def test_runner_rejects_removed_flash_flag(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit, match="2"):
        runner.main(["--prompt", "test", "--flash"])

    assert "unrecognized arguments: --flash" in capsys.readouterr().err


def test_runner_requires_dataset_id_for_dataset_collection(
    capsys: pytest.CaptureFixture[str],
) -> None:
    with pytest.raises(SystemExit, match="2"):
        runner.main(["--prompt", "test", "--collection", "dataset"])

    assert "--dataset-id is required when --collection dataset." in capsys.readouterr().err
