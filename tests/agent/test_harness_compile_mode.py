from __future__ import annotations

import asyncio
from pathlib import Path

import pytest

import agent.harness as harness
from agent.feedback import build_compile_signal_bundle
from agent.harness import ArticraftAgent
from agent.models import CompileReport


def test_compile_async_uses_timeout_wrapper(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    agent = ArticraftAgent.__new__(ArticraftAgent)
    agent.file_path = str(tmp_path / "model.py")
    agent.sdk_package = "sdk_hybrid"
    agent.runtime_limits = None

    report = CompileReport(
        urdf_xml="<robot />",
        warnings=[],
        signal_bundle=build_compile_signal_bundle(status="success"),
    )
    captured: dict[str, object] = {}

    def fake_compile(
        script_path: Path,
        *,
        sdk_package: str,
        rewrite_visual_glb: bool,
    ) -> CompileReport:
        captured["script_path"] = script_path
        captured["sdk_package"] = sdk_package
        captured["rewrite_visual_glb"] = rewrite_visual_glb
        return report

    monkeypatch.setattr(harness, "compile_urdf_report_maybe_timeout", fake_compile)

    result = asyncio.run(agent._compile_urdf_report_async())

    assert result is report
    assert captured == {
        "script_path": tmp_path / "model.py",
        "sdk_package": "sdk_hybrid",
        "rewrite_visual_glb": False,
    }
