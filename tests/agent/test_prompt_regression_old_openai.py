from __future__ import annotations

from pathlib import Path

from agent.prompts.compile import compile_prompt_variant
from agent.prompts.spec import iter_prompt_variants

OLD_OPENAI_PROMPT = Path(
    "/Users/matthewzhou/particulate-v2/synth-urdf-agent/designer_system_prompt_openai.txt"
)


def test_compiled_openai_prompt_matches_old_baseline() -> None:
    assert OLD_OPENAI_PROMPT.exists(), f"Missing old prompt baseline: {OLD_OPENAI_PROMPT}"
    expected = OLD_OPENAI_PROMPT.read_text(encoding="utf-8")

    variant = next(
        variant
        for variant in iter_prompt_variants()
        if variant.output.name == "designer_system_prompt_openai.txt"
    )
    compiled = compile_prompt_variant(variant)
    generated = variant.output.read_text(encoding="utf-8")

    assert compiled == expected, "Compiled OpenAI designer prompt no longer matches old baseline"
    assert generated == expected, "Generated OpenAI designer prompt no longer matches old baseline"
