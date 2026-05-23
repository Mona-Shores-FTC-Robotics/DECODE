# Stale docs

Files moved here are documentation that, as of 2026-05-16, has at least one of these problems:

- References code symbols / classes / flags that **do not exist** in the current codebase
- Is a dated **status snapshot** that's been superseded by reality
- **Duplicates** content already in `CLAUDE.md` or another doc

Nothing was deleted. Review and decide what to keep / delete / merge when you have time.

Full analysis is in `../DOCS_AUDIT.md`. Quick reference per file:

| File | Reason it's here |
|---|---|
| `TEST_BENCH_MODE.md` | Describes `Robot.testBenchMode` flag — flag does not exist in `Robot.java`. |
| `LAUNCHER_RPM_ROADMAP.md` | Describes `LaunchAtPositionCommand` + `PositionRpmConfig` — classes do not exist anywhere in the codebase. |
| `TeamCode-README.md` | Was `TeamCode/README.md`. Describes `FlywheelSubsystem`, `Constants.DEV_SIM_ENABLED`, `Constants.createPedroFollower()` — none exist. Also pins FTC Dashboard 0.4.8 (now 0.5.1). Severely outdated. |
| `CURRENT_STATUS.md` | Dated 2025-12-09. Status snapshot ~5 months stale; "recently implemented" items are now ancient. |
| `TODO_ANALYSIS.md` | Dated 2025-12-09. Same staleness. |
| `LIGHTING_IMPROVEMENTS.md` | Dated 2025-12-09. Implementation notes for features that shipped in December — git history preserves the context. |
| `AGENTS.md` | "Repository Guidelines" doc, mostly duplicates `CLAUDE.md`. |
| `CONVENTIONS.md` | "AI Coding Conventions" doc, mostly duplicates `CLAUDE.md`. |
| `DECODE_2025_Coding_Conventions.md` | Shorter conventions doc, mostly duplicates `CLAUDE.md`. |
| `logging_overview.md` | Was `doc/logging_overview.md`. Duplicates `TeamCode/.../telemetry/README.md` and claims a `PRACTICE` telemetry level that no longer exists in code (enum has only MATCH + DEBUG). |

## When you have time

For each file, options are:
1. **Delete** — git history preserves it; safest if the content is truly dead.
2. **Refresh** — rewrite to match current code, move back into `docs/` proper.
3. **Merge** — fold useful bits into another doc (e.g. convention docs → `CLAUDE.md` or a new `docs/conventions.md`).
