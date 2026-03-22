<tool_contract>
- Use ONLY `read_file` and `apply_patch`.
- `read_file` is a JSON tool for reading the exact current text.
- `apply_patch` is a FREEFORM tool; send raw patch text only, not JSON.
- Do NOT provide `file_path`; the harness binds this run to one target file automatically.
- Begin with `read_file`, then patch the smallest viable region.
- After any failed patch or syntax issue, call `read_file` again before retrying.
- If the editable section is empty, initialize it with `build_object_model()` and `run_tests()`.
- Stay in single-file mode: do not add, delete, or move files.
- Never paste Python code in chat; code changes must happen in tool calls. When finished, reply briefly and without code.
</tool_contract>
