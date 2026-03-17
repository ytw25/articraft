"""
Agent runtime entrypoints and CLI.
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import logging
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional

from dotenv import load_dotenv

from agent.compiler import (
    compile_urdf,
    compile_urdf_report_maybe_timeout,
    persist_compile_success_artifacts,
    update_manifest,
)
from agent.prompts import (
    DESIGNER_PROMPT_NAME,
    GEMINI_DESIGNER_PROMPT_NAME,
    HYBRID_OPENAI_DESIGNER_PROMPT_NAME,
    HYBRID_GEMINI_DESIGNER_PROMPT_NAME,
    LEGACY_DESIGNER_PROMPT_NAME,
    OPENAI_DESIGNER_PROMPT_NAME,
    SUPPORTED_SDK_DOCS_MODES,
    load_sdk_docs_reference,
    load_system_prompt_text,
    resolve_sdk_package_flags,
    resolve_system_prompt_path,
)
from agent.providers.gemini import GeminiLLM, gemini_api_keys_from_env
from agent.providers.openai import OpenAILLM
from agent.session import UrdfAgent
from agent.tools import (
    build_first_turn_messages as _build_first_turn_messages,
    build_initial_user_content as _build_initial_user_content,
    build_tool_registry,
    provider_system_prompt_suffix,
    resolve_image_path as _resolve_image_path,
)

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

MAX_SINGLE_RUN_SLUG_LEN = 120


def _slugify(text: str) -> str:
    cleaned: list[str] = []
    last_dash = False
    for ch in text.lower():
        if ch.isalnum():
            cleaned.append(ch)
            last_dash = False
        elif not last_dash:
            cleaned.append("-")
            last_dash = True
    slug = "".join(cleaned).strip("-")
    return slug or "object"


def _build_single_run_slug(prompt: str) -> str:
    base = _slugify(prompt) or "object"
    if len(base) <= MAX_SINGLE_RUN_SLUG_LEN:
        return base

    digest = hashlib.sha1(prompt.encode("utf-8")).hexdigest()[:10]
    reserved = len("_") + len(digest)
    max_base_len = max(1, MAX_SINGLE_RUN_SLUG_LEN - reserved)
    truncated_base = base[:max_base_len].rstrip("-") or "object"
    return f"{truncated_base}_{digest}"


def _append_run_timestamp_to_out_dir(out_dir: Path, now: Optional[datetime] = None) -> Path:
    timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    if out_dir.name:
        return out_dir.with_name(f"{out_dir.name}_{timestamp}")
    return out_dir / timestamp


def build_provider_payload_preview(
    user_content: Any,
    *,
    provider: str,
    model_id: str,
    openai_transport: str = "http",
    thinking_level: str,
    system_prompt_path: str,
    sdk_package: str = "sdk",
    sdk_docs_mode: str = "full",
    openai_reasoning_summary: Optional[str] = "auto",
) -> dict:
    repo_root = Path(__file__).resolve().parents[1]

    _, system_prompt = load_system_prompt_text(
        system_prompt_path,
        provider=provider,
        sdk_package=sdk_package,
        repo_root=repo_root,
    )
    suffix = provider_system_prompt_suffix(provider, sdk_package=sdk_package)
    if suffix:
        system_prompt = f"{system_prompt.rstrip()}\n\n{suffix}\n"

    docs = load_sdk_docs_reference(
        repo_root,
        sdk_package=sdk_package,
        docs_mode=sdk_docs_mode,
    )
    conversation = _build_first_turn_messages(
        user_content,
        sdk_docs_context=docs,
    )
    tools = build_tool_registry(provider, sdk_package=sdk_package).get_tool_schemas()

    provider_norm = (provider or "").strip().lower()
    if provider_norm == "openai":
        llm = OpenAILLM(
            model_id=model_id,
            thinking_level=thinking_level,
            reasoning_summary=openai_reasoning_summary,
            transport=openai_transport,
            dry_run=True,
        )
        return llm.build_request_preview(
            system_prompt=system_prompt,
            messages=conversation,
            tools=tools,
        )
    if provider_norm == "gemini":
        llm = GeminiLLM(model_id=model_id, thinking_level=thinking_level, dry_run=True)
        return llm.build_request_preview(
            system_prompt=system_prompt,
            messages=conversation,
            tools=tools,
        )
    raise ValueError(f"Unsupported provider: {provider}")


async def run_from_input(
    user_content: Any,
    *,
    script_path: Path,
    urdf_out: Optional[Path],
    outputs_root: Optional[Path],
    provider: str,
    model_id: Optional[str] = None,
    openai_transport: str = "http",
    thinking_level: str,
    max_turns: int,
    system_prompt_path: str,
    display_enabled: Optional[bool] = None,
    on_turn_start: Optional[Callable[[int], None]] = None,
    sdk_package: str = "sdk",
    sdk_docs_mode: str = "full",
    openai_reasoning_summary: Optional[str] = "auto",
) -> int:
    trace_dir = str(script_path.parent / "traces")
    async with UrdfAgent(
        file_path=str(script_path),
        provider=provider,
        model_id=model_id,
        openai_transport=openai_transport,
        thinking_level=thinking_level,
        max_turns=max_turns,
        system_prompt_path=system_prompt_path,
        trace_dir=trace_dir,
        display_enabled=display_enabled,
        on_turn_start=on_turn_start,
        checkpoint_urdf_path=urdf_out,
        manifest_outputs_root=outputs_root,
        sdk_package=sdk_package,
        sdk_docs_mode=sdk_docs_mode,
        openai_reasoning_summary=openai_reasoning_summary,
    ) as agent:
        logger.info("Using system prompt: %s", agent.loaded_system_prompt_path)
        result = await agent.run(user_content)

    if not result.success:
        logger.error("Agent failed: %s", result.message)
        return 2

    if result.usage:
        logger.info("Total tokens: %s", result.usage)
        try:
            cost_json_path = script_path.parent / "cost.json"
            if cost_json_path.exists():
                with open(cost_json_path) as file:
                    cost_data = json.load(file)
                    total_cost = (
                        cost_data.get("total", {}).get("costs_usd", {}).get("total", 0.0)
                    )
                    logger.info("Total cost: $%.6f", total_cost)
        except Exception:
            pass

    if result.urdf_xml is not None:
        urdf_xml = result.urdf_xml
    else:
        try:
            report = await asyncio.to_thread(
                compile_urdf_report_maybe_timeout,
                script_path,
                sdk_package=sdk_package,
            )
            for warning in report.warnings:
                logger.warning("%s", warning)
            urdf_xml = report.urdf_xml
        except Exception as exc:
            logger.error("Failed to compile URDF: %s", exc)
            return 3

    if urdf_out:
        await asyncio.to_thread(
            persist_compile_success_artifacts,
            urdf_xml=urdf_xml,
            urdf_out=urdf_out,
            outputs_root=None,
        )
        logger.info("Wrote URDF to %s", urdf_out)
    else:
        print(urdf_xml)

    if outputs_root:
        try:
            update_manifest(outputs_root)
        except Exception as exc:
            logger.warning("Failed to update manifest: %s", exc)

    return 0


async def run_from_prompt(
    prompt: str,
    *,
    script_path: Path,
    urdf_out: Optional[Path],
    outputs_root: Optional[Path],
    provider: str,
    model_id: Optional[str] = None,
    openai_transport: str = "http",
    thinking_level: str,
    max_turns: int,
    system_prompt_path: str,
    display_enabled: Optional[bool] = None,
    on_turn_start: Optional[Callable[[int], None]] = None,
    sdk_package: str = "sdk",
    sdk_docs_mode: str = "full",
    openai_reasoning_summary: Optional[str] = "auto",
) -> int:
    return await run_from_input(
        prompt,
        script_path=script_path,
        urdf_out=urdf_out,
        outputs_root=outputs_root,
        provider=provider,
        model_id=model_id,
        openai_transport=openai_transport,
        thinking_level=thinking_level,
        max_turns=max_turns,
        system_prompt_path=system_prompt_path,
        display_enabled=display_enabled,
        on_turn_start=on_turn_start,
        sdk_package=sdk_package,
        sdk_docs_mode=sdk_docs_mode,
        openai_reasoning_summary=openai_reasoning_summary,
    )


def _prepare_out_dir(
    out_dir: Path,
    *,
    outputs_root: Path,
    overwrite: bool,
    in_outputs: bool,
) -> None:
    out_dir_abs = out_dir.resolve()
    outputs_root_abs = outputs_root.resolve()

    if not overwrite:
        out_dir_abs.mkdir(parents=True, exist_ok=True)
        return

    if not in_outputs:
        out_dir_abs.mkdir(parents=True, exist_ok=True)
        return

    if out_dir_abs == outputs_root_abs:
        raise ValueError(f"Refusing to overwrite outputs root: {outputs_root_abs}")

    if out_dir_abs.exists() and not out_dir_abs.is_dir():
        raise ValueError(f"Output path exists and is not a directory: {out_dir_abs}")

    if out_dir_abs.exists():
        shutil.rmtree(out_dir_abs)

    out_dir_abs.mkdir(parents=True, exist_ok=True)


def _load_qc_blurb_text(qc_blurb_path: Optional[str], *, repo_root: Path) -> Optional[str]:
    if not qc_blurb_path:
        return None

    path = Path(qc_blurb_path)
    if not path.is_absolute():
        path = (repo_root / path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"QC blurb file not found: {path}")

    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    if text and not text.endswith("\n"):
        text += "\n"
    return text


def _build_prompt_with_qc(prompt: str, qc_blurb_text: Optional[str]) -> str:
    if not qc_blurb_text:
        return prompt

    return (
        f"{prompt.rstrip()}\n\n"
        "-----\n\n"
        "The following is a QC checklist to use as a final pass before declaring the model finished:\n\n"
        f"{qc_blurb_text}"
    )


def _load_plan_text(plan_path: str, *, repo_root: Path) -> str:
    path = Path(plan_path)
    if not path.is_absolute():
        path = (repo_root / path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"Plan file not found: {path}")
    text = path.read_text(encoding="utf-8").replace("\r\n", "\n").strip()
    if not text:
        raise ValueError(f"Plan file is empty: {path}")
    return text


def _build_prompt_from_plan(plan_markdown: str) -> str:
    return (
        "Implement the object described in the following markdown plan.\n\n"
        f"{plan_markdown.strip()}"
    )


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    load_dotenv()

    parser = argparse.ArgumentParser(
        description="Generate an articulated object from a prompt or markdown plan."
    )
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument("--prompt", help="Text prompt for the object.")
    input_group.add_argument(
        "--plan",
        default=None,
        help="Path to a markdown planning brief to use instead of --prompt.",
    )
    parser.add_argument(
        "--image",
        default=None,
        help=(
            "Optional reference image to augment --prompt or --plan. "
            "Currently supported only with --provider openai."
        ),
    )
    parser.add_argument(
        "--provider",
        default="openai",
        choices=["gemini", "openai"],
        help="LLM provider.",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help=(
            "Directory to write outputs. Defaults to outputs/<slug>_YYYYMMDD_HHMMSS for --prompt, "
            "or the plan file's parent directory for --plan."
        ),
    )
    parser.add_argument(
        "--timestamp-out-dir",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Append a timestamp suffix to the output directory name. "
            "Default: enabled for --prompt runs, disabled when --plan infers an existing output directory."
        ),
    )
    parser.add_argument(
        "--script",
        default=None,
        help="Path to write the generated Python script (default: <out-dir>/<run-stem>.py).",
    )
    parser.add_argument(
        "--urdf",
        default=None,
        help="Path to write the URDF XML output (default: <out-dir>/<run-stem>.urdf).",
    )
    parser.add_argument("--model", default=None, help="Model id (provider-specific).")
    parser.add_argument(
        "--openai-transport",
        default="http",
        choices=["http", "websocket"],
        help=(
            "Transport for --provider openai. "
            "`websocket` uses Responses WebSocket mode and enables response storage."
        ),
    )
    parser.add_argument(
        "--flash",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Use gemini-3-flash-preview (Gemini only).",
    )
    parser.add_argument(
        "--thinking",
        default="high",
        choices=["low", "med", "high"],
        help="Thinking budget level.",
    )
    parser.add_argument(
        "--openai-reasoning-summary",
        default="auto",
        help=(
            "OpenAI reasoning summary mode. Use `none` to omit thought summaries and "
            "reduce output latency/cost."
        ),
    )
    parser.add_argument("--max-turns", type=int, default=30)
    parser.add_argument(
        "--system-prompt",
        default="designer_system_prompt.txt",
        help="Path to the system prompt file.",
    )
    parser.add_argument(
        "--qc-blurb",
        default=None,
        help="Path to a markdown QC checklist to append to the prompt.",
    )
    parser.add_argument(
        "--dump-provider-payload",
        action="store_true",
        help="Print the provider request payload for turn 1 and exit (no API call).",
    )
    parser.add_argument(
        "--dump-provider-payload-out",
        default=None,
        help="Write the payload JSON to this path instead of stdout.",
    )
    parser.add_argument(
        "--dump-provider-payload-indent",
        type=int,
        default=2,
        help="JSON indent for --dump-provider-payload (default: 2).",
    )
    parser.add_argument(
        "--overwrite",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "If true, clear <out-dir> before generating outputs (only applies when <out-dir> is inside outputs/). "
            "Default: true for new prompt-based outputs inside outputs/*, false for --plan runs."
        ),
    )
    sdk_group = parser.add_mutually_exclusive_group()
    sdk_group.add_argument(
        "--hybrid-sdk",
        action="store_true",
        default=False,
        help="Use `sdk_hybrid` instead of `sdk` for scaffold/docs/compile-time helpers.",
    )
    parser.add_argument(
        "--sdk-docs-mode",
        default="full",
        choices=sorted(SUPPORTED_SDK_DOCS_MODES),
        help=(
            "SDK docs injection mode for turn 1. `core` keeps only a compact subset "
            "to reduce prompt size; `none` disables injected SDK docs entirely."
        ),
    )
    args = parser.parse_args()

    sdk_package = resolve_sdk_package_flags(hybrid_sdk=args.hybrid_sdk)

    if args.provider != "openai" and args.openai_transport != "http":
        print("--openai-transport is only supported for --provider openai.", file=sys.stderr)
        return 1

    repo_root = Path(__file__).resolve().parents[1]
    try:
        qc_blurb_text = _load_qc_blurb_text(args.qc_blurb, repo_root=repo_root)
    except Exception as exc:
        print(f"Failed to load qc blurb: {exc}", file=sys.stderr)
        return 1

    try:
        image_path = _resolve_image_path(args.image, provider=args.provider)
    except Exception as exc:
        print(f"Failed to load image: {exc}", file=sys.stderr)
        return 1

    plan_path: Optional[Path] = None
    if args.plan:
        try:
            plan_path = Path(args.plan)
            if not plan_path.is_absolute():
                plan_path = (repo_root / plan_path).resolve()
            base_prompt = _build_prompt_from_plan(
                _load_plan_text(str(plan_path), repo_root=repo_root)
            )
        except Exception as exc:
            print(f"Failed to load plan: {exc}", file=sys.stderr)
            return 1
    else:
        base_prompt = args.prompt

    prompt_with_qc = _build_prompt_with_qc(base_prompt, qc_blurb_text)
    user_content = _build_initial_user_content(prompt_with_qc, image_path=image_path)

    if args.dump_provider_payload:
        if args.provider == "gemini":
            if args.flash:
                model_id = "gemini-3-flash-preview"
            elif args.model:
                model_id = args.model
            else:
                model_id = GeminiLLM(dry_run=True).model_id
        else:
            model_id = args.model if args.model else OpenAILLM(dry_run=True).model_id

        payload = build_provider_payload_preview(
            user_content,
            provider=args.provider,
            model_id=model_id,
            openai_transport=args.openai_transport,
            thinking_level=args.thinking,
            system_prompt_path=args.system_prompt,
            sdk_package=sdk_package,
            sdk_docs_mode=args.sdk_docs_mode,
            openai_reasoning_summary=args.openai_reasoning_summary,
        )
        text = json.dumps(payload, indent=args.dump_provider_payload_indent, ensure_ascii=False)
        if args.dump_provider_payload_out:
            out_path = Path(args.dump_provider_payload_out)
            if not out_path.is_absolute():
                out_path = (Path.cwd() / out_path).resolve()
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(text, encoding="utf-8")
        else:
            print(text)
        return 0

    outputs_root = Path("outputs").resolve()
    if args.out_dir:
        out_dir = Path(args.out_dir).resolve()
    elif plan_path is not None:
        out_dir = plan_path.parent.resolve()
    else:
        out_dir = (outputs_root / _build_single_run_slug(args.prompt)).resolve()

    use_timestamp_out_dir = args.timestamp_out_dir
    if plan_path is not None and not args.out_dir:
        use_timestamp_out_dir = False
    if use_timestamp_out_dir:
        out_dir = _append_run_timestamp_to_out_dir(out_dir)

    in_outputs = out_dir.is_relative_to(outputs_root)
    overwrite_default = False if plan_path is not None else in_outputs
    overwrite = args.overwrite if args.overwrite is not None else overwrite_default
    try:
        _prepare_out_dir(
            out_dir,
            outputs_root=outputs_root,
            overwrite=overwrite,
            in_outputs=in_outputs,
        )
    except Exception as exc:
        print(f"Failed to prepare output directory: {exc}", file=sys.stderr)
        return 1

    if args.prompt:
        run_stem = _build_single_run_slug(args.prompt)
    elif out_dir.name:
        run_stem = out_dir.name
    elif plan_path is not None:
        run_stem = plan_path.parent.name or plan_path.stem
    else:
        run_stem = "object"

    script_path = Path(args.script).resolve() if args.script else out_dir / f"{run_stem}.py"
    urdf_out = Path(args.urdf).resolve() if args.urdf else out_dir / f"{run_stem}.urdf"

    if args.provider == "gemini":
        if not gemini_api_keys_from_env():
            print("GEMINI_API_KEYS environment variable is required.", file=sys.stderr)
            return 1
    elif args.provider == "openai":
        if not os.environ.get("OPENAI_API_KEY"):
            print("OPENAI_API_KEY environment variable is required.", file=sys.stderr)
            return 1
        if args.flash:
            print("--flash is only supported for --provider gemini.", file=sys.stderr)
            return 1

    model_id = "gemini-3-flash-preview" if args.provider == "gemini" and args.flash else args.model

    return asyncio.run(
        run_from_input(
            user_content,
            script_path=script_path,
            urdf_out=urdf_out,
            outputs_root=outputs_root if in_outputs else None,
            provider=args.provider,
            model_id=model_id,
            openai_transport=args.openai_transport,
            thinking_level=args.thinking,
            max_turns=args.max_turns,
            system_prompt_path=args.system_prompt,
            sdk_package=sdk_package,
            sdk_docs_mode=args.sdk_docs_mode,
            openai_reasoning_summary=args.openai_reasoning_summary,
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
