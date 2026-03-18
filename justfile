default:
    @just --list

host := "127.0.0.1"
port := "8765"

setup:
    uv sync --group dev
    uv run pre-commit install --hook-type pre-commit --hook-type pre-push
    uv run articraft-dataset --repo-root . init-storage
    uv run articraft-workbench --repo-root . init-storage
    uv run articraft-dataset --repo-root . validate
    uv run articraft-dataset --repo-root . build-manifest

hooks-install:
    uv run pre-commit install --hook-type pre-commit --hook-type pre-push

format:
    uv run ruff format .

lint:
    uv run ruff check .

smoke-tests:
    find tests -name 'test_*.py' -type f | sort | xargs -I{} uv run python {}

dataset-validate:
    uv run articraft-dataset --repo-root . validate

dataset-manifest:
    uv run articraft-dataset --repo-root . build-manifest

viewer mode="prod" api_host=host api_port=port:
    #!/usr/bin/env bash
    set -euo pipefail
    if ! command -v npm >/dev/null 2>&1; then
      echo "npm is required for viewer/web. Install Node.js and npm first."
      exit 1
    fi
    if [ ! -d viewer/web/node_modules ]; then
      npm --prefix viewer/web install
    fi
    uv run articraft-dataset --repo-root . validate
    uv run articraft-dataset --repo-root . build-manifest
    case "{{mode}}" in
      prod)
        npm --prefix viewer/web run build
        (
          for _ in {1..60}; do
            if curl -fsS http://{{api_host}}:{{api_port}}/health >/dev/null; then
              open http://{{api_host}}:{{api_port}}/ >/dev/null 2>&1 || uv run python -m webbrowser http://{{api_host}}:{{api_port}}/ >/dev/null 2>&1 || true
              exit 0
            fi
            sleep 0.25
          done
        ) >/dev/null 2>&1 &
        exec uv run uvicorn viewer.api.app:app --reload --host {{api_host}} --port {{api_port}}
        ;;
      dev)
        (
          for _ in {1..60}; do
            if curl -fsS http://127.0.0.1:5173 >/dev/null; then
              open http://127.0.0.1:5173/ >/dev/null 2>&1 || uv run python -m webbrowser http://127.0.0.1:5173/ >/dev/null 2>&1 || true
              exit 0
            fi
            sleep 0.25
          done
        ) >/dev/null 2>&1 &
        trap 'if [ -n "${api_pid:-}" ]; then kill "$api_pid" >/dev/null 2>&1 || true; fi' EXIT INT TERM
        uv run uvicorn viewer.api.app:app --reload --host {{api_host}} --port {{api_port}} &
        api_pid=$!
        ARTICRAFT_VIEWER_API_HOST={{api_host}} ARTICRAFT_VIEWER_API_PORT={{api_port}} exec npm --prefix viewer/web run dev
        ;;
      *)
        echo "Unknown viewer mode: {{mode}}. Expected 'prod' or 'dev'." >&2
        exit 1
        ;;
    esac

viewer-dev api_host=host api_port=port:
    exec just viewer dev {{api_host}} {{api_port}}

viewer-api:
    exec uv run uvicorn viewer.api.app:app --reload --host {{host}} --port {{port}}

viewer-web-install:
    npm --prefix viewer/web install

viewer-web-dev:
    npm --prefix viewer/web run dev

viewer-web-build:
    npm --prefix viewer/web run build

viewer-web-lint:
    npm --prefix viewer/web run lint

viewer-web-typecheck:
    npm --prefix viewer/web run typecheck
