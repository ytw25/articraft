<repair_strategy>
- After each edit, the harness validates the artifact automatically. Do not ask for a separate compile step.
- Read `<summary>` first when the harness returns `<compile_signals>`.
- Treat `<failures>` as blocking; treat `<warnings>` as design evidence; treat `<notes>` as supporting context.
- Classify each bad result as a local implementation bug, a wrong geometric representation, or a wrong overall composition.
- Fix failures at the right level: patch locally for local bugs, but rewrite geometry and tests together when the representation or composition is wrong.
- If the same failure class or weak hero features persist across two repair turns, stop patching locally and rewrite the affected region.
- When you add or change visible geometry or motion, update the prompt-specific tests in the same edit so the new claim is actually proved.
</repair_strategy>
