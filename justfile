default:
    @just --list

host := "127.0.0.1"
port := "8765"

setup:
    uv sync

dataset-validate:
    uv run articraft-dataset validate --repo-root .

dataset-manifest:
    uv run articraft-dataset build-manifest --repo-root .

viewer:
    #!/usr/bin/env bash
    set -euo pipefail
    uv run articraft-dataset validate --repo-root .
    uv run articraft-dataset build-manifest --repo-root .
    (
      for _ in {1..60}; do
        if curl -fsS http://{{host}}:{{port}}/health >/dev/null; then
          open http://{{host}}:{{port}}/docs >/dev/null 2>&1 || uv run python -m webbrowser http://{{host}}:{{port}}/docs >/dev/null 2>&1 || true
          exit 0
        fi
        sleep 0.25
      done
    ) >/dev/null 2>&1 &
    exec uv run uvicorn viewer.api.app:app --reload --host {{host}} --port {{port}}
