from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from agent.providers.anthropic import (
    DEFAULT_ANTHROPIC_MODEL,
    AnthropicLLM,
    anthropic_api_key_from_env,
)
from agent.providers.base import ProviderClient
from agent.providers.gemini import GeminiLLM, gemini_client_config_from_env
from agent.providers.openai import DEFAULT_OPENAI_MODEL, OpenAILLM, openai_api_key_from_env
from agent.providers.openrouter import (
    DEFAULT_OPENROUTER_MODEL,
    OpenRouterLLM,
    openrouter_api_key_from_env,
)

DEFAULT_GEMINI_MODEL = "gemini-3.1-pro-preview"
SUPPORTED_PROVIDERS = frozenset({"anthropic", "gemini", "openai", "openrouter"})


@dataclass(slots=True, frozen=True)
class ProviderConfig:
    provider: str = "openai"
    model_id: str | None = None
    thinking_level: str = "high"
    openai_transport: str = "http"
    openai_reasoning_summary: str | None = "auto"
    openai_prompt_cache_key: str | None = None
    openai_prompt_cache_retention: str | None = None

    @property
    def normalized_provider(self) -> str:
        return normalize_provider_name(self.provider)


@dataclass(slots=True, frozen=True)
class ProviderConstructors:
    anthropic: Callable[..., ProviderClient] = AnthropicLLM
    gemini: Callable[..., ProviderClient] = GeminiLLM
    openai: Callable[..., ProviderClient] = OpenAILLM
    openrouter: Callable[..., ProviderClient] = OpenRouterLLM


def normalize_provider_name(provider: str | None) -> str:
    value = (provider or "openai").strip().lower()
    if value not in SUPPORTED_PROVIDERS:
        raise ValueError(f"Unsupported provider: {provider}")
    return value


def infer_provider_from_model_id(model_id: str | None) -> str | None:
    model_norm = (model_id or "").strip().lower()
    if not model_norm:
        return None
    if model_norm.startswith(("gpt-", "o1", "o3", "o4")):
        return "openai"
    if model_norm.startswith("claude-"):
        return "anthropic"
    if model_norm.startswith("gemini-"):
        return "gemini"
    if "/" in model_norm or model_norm.startswith("openrouter/"):
        return "openrouter"
    return None


def default_model_id(config: ProviderConfig) -> str:
    if config.model_id:
        return config.model_id
    provider = config.normalized_provider
    if provider == "anthropic":
        return DEFAULT_ANTHROPIC_MODEL
    if provider == "gemini":
        return DEFAULT_GEMINI_MODEL
    if provider == "openrouter":
        return DEFAULT_OPENROUTER_MODEL
    if provider == "openai":
        return DEFAULT_OPENAI_MODEL
    raise ValueError(f"Unsupported provider: {config.provider}")


def create_provider_client(
    config: ProviderConfig,
    *,
    dry_run: bool = False,
    constructors: ProviderConstructors | None = None,
) -> ProviderClient:
    provider = config.normalized_provider
    model_id = default_model_id(config)
    provider_constructors = constructors or ProviderConstructors()
    if provider == "anthropic":
        return provider_constructors.anthropic(
            model_id=model_id,
            thinking_level=config.thinking_level,
            dry_run=dry_run,
        )
    if provider == "gemini":
        return provider_constructors.gemini(
            model_id=model_id,
            thinking_level=config.thinking_level,
            dry_run=dry_run,
        )
    if provider == "openrouter":
        return provider_constructors.openrouter(
            model_id=model_id,
            thinking_level=config.thinking_level,
            dry_run=dry_run,
        )
    if provider == "openai":
        return provider_constructors.openai(
            model_id=model_id,
            thinking_level=config.thinking_level,
            reasoning_summary=config.openai_reasoning_summary,
            transport=config.openai_transport,
            prompt_cache_key=config.openai_prompt_cache_key,
            prompt_cache_retention=config.openai_prompt_cache_retention,
            dry_run=dry_run,
        )
    raise ValueError(f"Unsupported provider: {config.provider}")


def validate_provider_credentials(provider: str) -> None:
    provider_norm = normalize_provider_name(provider)
    if provider_norm == "anthropic":
        if not anthropic_api_key_from_env():
            raise ValueError(
                "Anthropic credentials are required. Set ANTHROPIC_API_KEY or ANTHROPIC_API_KEYS."
            )
        return
    if provider_norm == "gemini":
        gemini_client_config_from_env()
        return
    if provider_norm == "openrouter":
        if not openrouter_api_key_from_env():
            raise ValueError(
                "OpenRouter credentials are required. "
                "Set OPENROUTER_API_KEY or OPENROUTER_API_KEYS."
            )
        return
    if provider_norm == "openai" and not openai_api_key_from_env():
        raise ValueError("OpenAI credentials are required. Set OPENAI_API_KEY or OPENAI_API_KEYS.")
