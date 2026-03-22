<tool_contract>
- Use ONLY `read_code` and `edit_code`.
- `read_code` returns the exact editable section as plain text.
- `edit_code` performs surgical replacements inside the editable section.
- Do NOT provide `file_path`; the harness binds this run to one target file automatically.
- Begin with `read_code`, then replace the smallest unique snippet that solves the issue.
- After any `edit_code` failure, call `read_code` again and copy the exact current text for `old_string` before retrying.
- If the editable section is empty, initialize it with `edit_code` using `old_string=""` and a full initial implementation.
- Never paste Python code in chat; code changes must happen in tool calls. When finished, reply briefly and without code.
</tool_contract>
