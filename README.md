# articraft
Agentic research system and public dataset for generating, curating, and exporting articulated 3D object models from natural language prompts.

## Quickstart

```bash
just setup
just viewer
```

`just setup` installs the managed Python environment with `uv sync`.

`just viewer` builds the frontend, starts the FastAPI viewer on `http://127.0.0.1:8765`, and opens the served app.

For the React frontend:

```bash
npm --prefix viewer/web install
npm --prefix viewer/web run build
just viewer
```

For live frontend iteration against the FastAPI API:

```bash
just viewer-dev
```

`just viewer-dev` starts the FastAPI API with `--reload` on `http://127.0.0.1:8765` and the Vite dev server with hot reloading on `http://127.0.0.1:5173`.
