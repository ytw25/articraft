from __future__ import annotations

import asyncio
import urllib.error
from types import SimpleNamespace

from agent.providers.anthropic import (
    ANTHROPIC_VERSION,
    DEFAULT_ANTHROPIC_MODEL,
    AnthropicAPIError,
    AnthropicLLM,
    _async_retry,
    _should_retry_anthropic_exception,
    anthropic_api_key_from_env,
    anthropic_api_keys_from_env,
)


def test_anthropic_api_keys_from_env_prefers_primary_key_and_dedupes() -> None:
    keys = anthropic_api_keys_from_env(
        {
            "ANTHROPIC_API_KEY": "sk-ant-primary",
            "ANTHROPIC_API_KEYS": "sk-ant-primary,\nsk-ant-secondary, sk-ant-secondary",
        }
    )

    assert keys == ["sk-ant-primary", "sk-ant-secondary"]


def test_anthropic_api_key_from_env_uses_key_pool_when_primary_missing() -> None:
    key = anthropic_api_key_from_env({"ANTHROPIC_API_KEYS": "sk-ant-first,sk-ant-second"})

    assert key in {"sk-ant-first", "sk-ant-second"}


def test_anthropic_provider_loads_dotenv_over_exported_key(monkeypatch, tmp_path) -> None:
    env_name = "ANTHROPIC_API_" + "KEY"
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv(env_name, "test-exported")
    (tmp_path / ".env").write_text(f"{env_name}=test-dotenv\n", encoding="utf-8")

    try:
        AnthropicLLM()
    except Exception:
        pass

    assert anthropic_api_key_from_env() == "test-dotenv"


def test_anthropic_default_model_is_current_opus() -> None:
    provider = AnthropicLLM(dry_run=True)

    assert provider.model_id == DEFAULT_ANTHROPIC_MODEL


def test_anthropic_request_preview_uses_messages_shape() -> None:
    provider = AnthropicLLM(dry_run=True)
    payload = provider.build_request_preview(
        system_prompt="system",
        messages=[{"role": "user", "content": "task"}],
        tools=[
            {
                "type": "function",
                "function": {
                    "name": "compile_model",
                    "description": "Compile",
                    "parameters": {"type": "object", "properties": {}, "required": []},
                },
            }
        ],
    )

    assert payload["base_url"] == "https://api.anthropic.com"
    assert payload["anthropic_version"] == ANTHROPIC_VERSION
    assert payload["model"] == "claude-opus-4-7"
    assert payload["cache_control"] == {"type": "ephemeral"}
    assert payload["system"] == [
        {
            "type": "text",
            "text": "system",
            "cache_control": {"type": "ephemeral"},
        }
    ]
    assert payload["thinking"] == {"type": "adaptive"}
    assert payload["output_config"] == {"effort": "high"}
    assert payload["messages"] == [{"role": "user", "content": "task"}]
    assert payload["tools"] == [
        {
            "name": "compile_model",
            "description": "Compile",
            "input_schema": {"type": "object", "properties": {}, "required": []},
            "cache_control": {"type": "ephemeral"},
        }
    ]


def test_anthropic_prompt_cache_can_be_disabled(monkeypatch) -> None:
    monkeypatch.setenv("ANTHROPIC_PROMPT_CACHE", "off")
    provider = AnthropicLLM(dry_run=True)

    payload = provider.build_request_preview(
        system_prompt="system",
        messages=[{"role": "user", "content": "task"}],
        tools=[],
    )

    assert "cache_control" not in payload
    assert payload["system"] == "system"


def test_anthropic_prompt_cache_marks_stable_workspace_docs() -> None:
    provider = AnthropicLLM(dry_run=True)

    payload = provider.build_request_preview(
        system_prompt="system",
        messages=[
            {
                "role": "user",
                "content": "\n\n# Workspace Documentation (read-only)\nshared docs",
            },
            {"role": "user", "content": "per-record prompt"},
        ],
        tools=[],
    )

    assert payload["messages"][0] == {
        "role": "user",
        "content": [
            {
                "type": "text",
                "text": "\n\n# Workspace Documentation (read-only)\nshared docs",
                "cache_control": {"type": "ephemeral"},
            }
        ],
    }
    assert payload["messages"][1] == {"role": "user", "content": "per-record prompt"}


def test_anthropic_prompt_cache_supports_one_hour_ttl(monkeypatch) -> None:
    monkeypatch.setenv("ANTHROPIC_PROMPT_CACHE_TTL", "1h")
    provider = AnthropicLLM(dry_run=True)

    payload = provider.build_request_preview(
        system_prompt="system",
        messages=[{"role": "user", "content": "task"}],
        tools=[],
    )

    assert payload["cache_control"] == {"type": "ephemeral", "ttl": "1h"}
    assert payload["system"][0]["cache_control"] == {"type": "ephemeral", "ttl": "1h"}


def test_anthropic_preview_coalesces_tool_results_and_preserves_raw_blocks() -> None:
    provider = AnthropicLLM(dry_run=True)
    payload = provider.build_request_preview(
        system_prompt="system",
        messages=[
            {"role": "user", "content": "task"},
            {
                "role": "assistant",
                "content": "I will compile.",
                "tool_calls": [
                    {
                        "id": "toolu_1",
                        "type": "function",
                        "function": {"name": "compile_model", "arguments": "{}"},
                    },
                    {
                        "id": "toolu_2",
                        "type": "function",
                        "function": {"name": "read_file", "arguments": '{"path":"model.py"}'},
                    },
                ],
                "extra_content": {
                    "anthropic": {
                        "content": [
                            {
                                "type": "thinking",
                                "thinking": "Need compile evidence.",
                                "signature": "sig",
                            },
                            {"type": "text", "text": "I will compile."},
                            {
                                "type": "tool_use",
                                "id": "toolu_1",
                                "name": "compile_model",
                                "input": {},
                            },
                            {
                                "type": "tool_use",
                                "id": "toolu_2",
                                "name": "read_file",
                                "input": {"path": "model.py"},
                            },
                        ]
                    }
                },
            },
            {
                "role": "tool",
                "tool_call_id": "toolu_1",
                "name": "compile_model",
                "content": '{"result":"Compile passed cleanly."}',
            },
            {
                "role": "tool",
                "tool_call_id": "toolu_2",
                "name": "read_file",
                "content": '{"error":"not found"}',
            },
        ],
        tools=[],
    )

    assert payload["messages"][1]["content"][0]["type"] == "thinking"
    assert payload["messages"][1]["content"][0]["signature"] == "sig"
    assert payload["messages"][2] == {
        "role": "user",
        "content": [
            {
                "type": "tool_result",
                "tool_use_id": "toolu_1",
                "content": '{"result":"Compile passed cleanly."}',
            },
            {
                "type": "tool_result",
                "tool_use_id": "toolu_2",
                "content": '{"error":"not found"}',
                "is_error": True,
            },
        ],
    }


def test_anthropic_response_converts_text_thinking_tool_use_and_usage() -> None:
    provider = AnthropicLLM(dry_run=True)
    response = {
        "content": [
            {"type": "thinking", "thinking": "Planned compile.", "signature": "sig"},
            {"type": "text", "text": "I will compile."},
            {
                "type": "tool_use",
                "id": "toolu_1",
                "name": "compile_model",
                "input": {},
            },
        ],
        "stop_reason": "tool_use",
        "usage": {
            "input_tokens": 10,
            "cache_creation_input_tokens": 3,
            "cache_read_input_tokens": 7,
            "cache_creation": {
                "ephemeral_5m_input_tokens": 1,
                "ephemeral_1h_input_tokens": 2,
            },
            "output_tokens": 5,
        },
    }

    converted = provider._convert_response(response)

    assert converted["content"] == "I will compile."
    assert converted["thought_summary"] == "Planned compile."
    assert converted["tool_calls"] == [
        {
            "id": "toolu_1",
            "type": "function",
            "function": {"name": "compile_model", "arguments": "{}"},
        }
    ]
    assert converted["extra_content"]["anthropic"]["content"][0]["signature"] == "sig"
    assert converted["usage"] == {
        "prompt_tokens": 20,
        "candidates_tokens": 5,
        "total_tokens": 25,
        "cached_tokens": 7,
        "cache_creation_input_tokens": 3,
        "cache_read_input_tokens": 7,
        "cache_creation_5m_input_tokens": 1,
        "cache_creation_1h_input_tokens": 2,
    }


def test_anthropic_thinking_rejection_does_not_retry_without_thinking() -> None:
    provider = AnthropicLLM(model_id="claude-opus-4-7", dry_run=True)
    provider.api_key = "sk-ant-test"
    provider.max_attempts = 1
    calls: list[dict] = []

    def fake_post_json(payload: dict) -> dict:
        calls.append(payload)
        raise AnthropicAPIError(
            status_code=400,
            error_type="invalid_request_error",
            message="thinking is not supported for this model",
        )

    provider._post_json = fake_post_json  # type: ignore[method-assign]

    try:
        asyncio.run(
            provider.generate_with_tools(
                system_prompt="system",
                messages=[{"role": "user", "content": "task"}],
                tools=[],
            )
        )
    except AnthropicAPIError:
        pass
    else:  # pragma: no cover - defensive assertion branch
        raise AssertionError("expected AnthropicAPIError")

    assert len(calls) == 1
    assert calls[0]["thinking"] == {"type": "adaptive"}
    assert calls[0]["output_config"] == {"effort": "high"}


def test_anthropic_retry_predicate_treats_rate_limits_as_transient() -> None:
    exc = AnthropicAPIError(
        status_code=429,
        error_type="rate_limit_error",
        message="rate limited",
        retry_after=1,
    )

    assert _should_retry_anthropic_exception(exc) is True


def test_anthropic_retry_predicate_treats_request_too_large_as_terminal() -> None:
    exc = AnthropicAPIError(
        status_code=413,
        error_type="request_too_large",
        message="too large",
    )

    assert _should_retry_anthropic_exception(exc) is False


def test_anthropic_retry_predicate_treats_url_errors_as_transient() -> None:
    assert _should_retry_anthropic_exception(urllib.error.URLError("disconnected")) is True


def test_anthropic_async_retry_honors_retry_after() -> None:
    attempts = 0
    sleeps: list[float] = []

    async def flaky() -> str:
        nonlocal attempts
        attempts += 1
        if attempts < 2:
            raise AnthropicAPIError(
                status_code=529,
                error_type="overloaded_error",
                message="overloaded",
                retry_after=3,
            )
        return "ok"

    async def fake_sleep(delay: float) -> None:
        sleeps.append(delay)

    result = asyncio.run(
        _async_retry(
            flaky,
            max_attempts=2,
            should_retry=_should_retry_anthropic_exception,
            retry_after=lambda exc: exc.retry_after if isinstance(exc, AnthropicAPIError) else None,
            base_delay=1,
            max_delay=10,
            logger=SimpleNamespace(warning=lambda *args, **kwargs: None),
            context="anthropic[test]",
            sleep_fn=fake_sleep,
            rng=lambda: 0.5,
        )
    )

    assert result == "ok"
    assert sleeps == [3]
