# Documentation Audit & Reorganization Proposal

Generated 2026-05-16. Cross-referenced against current code on `testCode` branch. **Nothing moved/deleted yet — this is a proposal for your review.**

## TL;DR

- **3 docs reference code that no longer exists.** They were accurate when written but the code has since been removed/renamed. Recommend deletion or archival.
- **3 convention/style docs at the project root duplicate each other and overlap heavily with `CLAUDE.md`.** Recommend consolidating to one.
- **3 docs claim 3-level telemetry (MATCH/PRACTICE/DEBUG)** but the actual `TelemetryLevel` enum in code has only 2 levels (MATCH/DEBUG). Need correction.
- **3 dated "snapshot" docs from December 2025** (status/todo/lighting changelog) are essentially historical. Recommend moving to an `archive/` subfolder or deleting.
- **Whiffle ball trio** (3 docs on the same topic) could be merged into one.
- **Aiming pair** (2 docs on the same topic) could be merged into one.

If you accept the recommendations, you'd go from **~21 doc files scattered across the repo** to **~11 organized files in `docs/`** with no information loss.

## Inventory by current location

### Project root (`./`)

| File | Status | Recommendation |
|---|---|---|
| `CLAUDE.md` | ✅ Canonical | **Keep in root.** Required by Claude Code; do not move. |
| `README.md` | ⚠️ Outdated | Describes the "FTC + Codex Starter" scaffold — predates current codebase. **Replace** with a real project README that points to `docs/` and `CLAUDE.md`. |
| `AGENTS.md` | 🔁 Duplicates `CLAUDE.md` | Most of its content is covered by `CLAUDE.md`. **Merge unique bits into `CLAUDE.md`, then delete.** |
| `CONVENTIONS.md` | 🔁 Duplicates `CLAUDE.md` | Same. **Merge & delete.** |
| `DECODE_2025_Coding_Conventions.md` | 🔁 Duplicates `CLAUDE.md` | Same. **Merge & delete.** |
| `CURRENT_STATUS.md` | 📅 Snapshot 2025-12-09 | Five+ months old; "recently implemented" items are now ancient history. **Move to `docs/archive/` or delete.** |
| `TODO_ANALYSIS.md` | 📅 Snapshot 2025-12-09 | Same staleness. Contents largely superseded by current state of code + `docs/saturday-practice-plan.md`. **Move to archive or delete.** |
| `LIGHTING_IMPROVEMENTS.md` | 📅 Snapshot 2025-12-09 | Implementation notes for features that shipped in December. **Move to archive or delete** — git history preserves the context. |
| `LAUNCHER_RPM_ROADMAP.md` | ❌ Wrong / dead code | References `LaunchAtPositionCommand` and `LaunchAtPositionCommand.PositionRpmConfig` — **these classes do not exist in the codebase anywhere.** The phase-based roadmap belongs to an older architecture. **Delete.** |
| `TEST_BENCH_MODE.md` | ❌ Wrong / dead code | Describes `Robot.testBenchMode` flag — **this flag does not exist in `Robot.java`.** **Delete.** |

### `docs/` (current contents)

| File | Status | Recommendation |
|---|---|---|
| `AIMING_METHODS.md` | ✅ Current; describes 3 methods | **Merge with `AIMING_TESTING_PLAN.md`** → `docs/subsystems/aiming.md`. They cover the same topic at different levels of detail. |
| `AIMING_TESTING_PLAN.md` | ✅ Current; describes 4 methods | Merge as above. (Note discrepancy: one says 3 methods, one says 4. Worth reconciling during merge.) |
| `ARTIFACT_DETECTION_TUNING.md` | ✅ Current | Config keys match current code (`lanePresenceConfig20245`, etc.). **Move to `docs/subsystems/artifact-detection.md`.** |
| `FLYWHEEL_FEEDFORWARD_TUNING.md` | ✅ Current | Config keys (`LauncherFlywheelConfig.flywheelLeft.kS` etc.) match current code. **Move to `docs/subsystems/flywheel-tuning.md`.** |
| `PATH_VISUALIZATION.md` | ✅ Current | The `:TeamCode:exportPaths` gradle task exists in `TeamCode/build.gradle:69-92`. **Move to `docs/operations/path-visualization.md`.** |
| `WHIFFLE_BALL_DETECTION.md` | ⚠️ Concept doc | Three different docs on the same topic. **Merge all three** → `docs/subsystems/whiffle-ball-detection.md`. |
| `WHIFFLE_BALL_FIX_SUMMARY.md` | ⚠️ Implementation snapshot | Merge as above. |
| `WHIFFLE_BALL_QUICK_START.md` | ⚠️ Quick-start variant | Merge as above. |
| `loop-timing-analysis.md` | 📅 Snapshot 2025-11-14 | References specific commits (`5176aea`, `b278468`). Analysis itself may still be relevant. **Move to `docs/analysis/loop-timing.md` and add a "last verified" note** — or refresh if you re-run the analysis. |
| `operator-controls-analysis.md` | ⚠️ Possibly stale | References button mappings; needs verification against current `OperatorBindings.java`. **Audit + move to `docs/operations/operator-controls.md`.** |
| `saturday-practice-plan.md` | ✅ Current (today) | **Stays in `docs/`** (or move to `docs/analysis/` with loop-timing). |

### `doc/` (singular — accidental fork of `docs/`)

| File | Status | Recommendation |
|---|---|---|
| `doc/logging_overview.md` | 🔁 Duplicates `TeamCode/.../telemetry/README.md` | Both describe the telemetry system; both incorrectly claim 3 levels (see Wrong claims below). **Merge into the telemetry README**, delete the `doc/` folder. |

### `TeamCode/`

| File | Status | Recommendation |
|---|---|---|
| `TeamCode/README.md` | ❌ Severely outdated | Describes "DECODE Command-Based Scaffold" with `FlywheelSubsystem`, `DEV_SIM_ENABLED`, `Constants.createPedroFollower()`. **None of these exist in the current codebase.** Also references FTC Dashboard 0.4.8 (current is 0.5.1). **Delete** — it's actively misleading. |
| `TeamCode/src/main/java/.../telemetry/README.md` | ⚠️ Mostly accurate, one wrong section | Claims MATCH/PRACTICE/DEBUG telemetry levels (lines 47-70). Actual code (`TelemetrySettings.java:9-24`) has only **MATCH** and **DEBUG**. **Keep where it is, but fix the levels section.** |
| `TeamCode/src/main/res/raw/readme.md` | (not checked — likely FTC SDK boilerplate) | Leave alone. |

### `codex/`

| File | Status | Recommendation |
|---|---|---|
| `codex/codex_task_template.md` | ✅ Template | Tied to an alternate AI tool (Codex). If still in use, **keep**. If retired, **delete the whole `codex/` folder** including `tasks/`. |
| `codex/tasks/flywheel_stability.md` | ✅ Sample task | Same — keep or delete the folder together. |

### `.github/`

| File | Status | Recommendation |
|---|---|---|
| `.github/CONTRIBUTING.md` | (not audited) | GitHub convention — leave in place. |
| `.github/PULL_REQUEST_TEMPLATE.md` | (not audited) | GitHub convention — leave in place. |

### FTC SDK boilerplate (do not touch)

These ship with the FTC SDK template and have no team-owned content:
- `FtcRobotController/src/main/java/.../samples/readme.md`
- `FtcRobotController/src/main/java/.../samples/sample_conventions.md`
- Anything under `*/build/` (generated)

## Wrong claims to fix or remove

These are concrete factual errors that will confuse readers:

| File | Wrong claim | Reality |
|---|---|---|
| `TEST_BENCH_MODE.md` | `Robot.testBenchMode` flag exists for path-only testing | No such flag in `Robot.java`. Feature was removed or never made it past planning. |
| `LAUNCHER_RPM_ROADMAP.md` | `LaunchAtPositionCommand` with `PositionRpmConfig.farLaunchRpm = 4200`, etc. | Class does not exist. Current launcher RPM is set via `presetRangeSpinUp(LauncherRange.SHORT_AUTO, ...)` driven by `CommandRangeConfig`. |
| `TeamCode/README.md` | `FlywheelSubsystem`, `Constants.DEV_SIM_ENABLED`, `Constants.createPedroFollower()` | None exist. Flywheel logic lives in `LauncherSubsystem` (3 lanes). Pedro is configured in `pedroPathing/Constants.java` via builders, not a `createPedroFollower()` factory. |
| `TeamCode/README.md` | FTC Dashboard 0.4.8 | Now 0.5.1 (today's upgrade). |
| `TeamCode/src/main/java/.../telemetry/README.md` lines 47-70 | "Three telemetry levels: MATCH / PRACTICE / DEBUG" | `TelemetrySettings.TelemetryLevel` enum only contains MATCH and DEBUG (`TelemetrySettings.java:9-24`). PRACTICE was removed. |
| `doc/logging_overview.md` lines 14-17 | Same MATCH/PRACTICE/DEBUG claim | Same — only 2 levels exist. |
| `CLAUDE.md` (yes, our own canonical file!) lines ~370-373 | Lists `TelemetryLevel.PRACTICE` in the "Special Considerations" section | The earlier section of `CLAUDE.md` correctly says MATCH and DEBUG. The later section contradicts itself by listing all three. **Worth fixing in `CLAUDE.md` for self-consistency.** |

## Proposed final structure

```
./
├── CLAUDE.md                              (stays in root, fix PRACTICE inconsistency)
├── README.md                              (rewrite as project intro pointing to docs/)
├── .github/                               (unchanged)
├── codex/                                 (keep or delete as a unit)
└── docs/
    ├── README.md                          (NEW — index of docs)
    ├── conventions.md                     (consolidates AGENTS + CONVENTIONS + DECODE_2025)
    ├── subsystems/
    │   ├── aiming.md                      (merged from AIMING_METHODS + AIMING_TESTING_PLAN)
    │   ├── artifact-detection.md          (from ARTIFACT_DETECTION_TUNING)
    │   ├── flywheel-tuning.md             (from FLYWHEEL_FEEDFORWARD_TUNING)
    │   └── whiffle-ball-detection.md      (merged from 3 whiffle files)
    ├── operations/
    │   ├── path-visualization.md          (from PATH_VISUALIZATION)
    │   ├── telemetry.md                   (consolidates TeamCode telemetry README + doc/logging_overview, fix level count)
    │   └── operator-controls.md           (audit-then-move from operator-controls-analysis)
    └── analysis/
        ├── loop-timing.md                 (from loop-timing-analysis, add last-verified date)
        └── saturday-practice-plan.md      (today's doc)

DELETE:
├── TEST_BENCH_MODE.md                     (describes non-existent feature)
├── LAUNCHER_RPM_ROADMAP.md                (describes non-existent classes)
├── TeamCode/README.md                     (describes dead scaffold)
├── AGENTS.md                              (after merge into CLAUDE.md and/or conventions.md)
├── CONVENTIONS.md                         (after merge)
├── DECODE_2025_Coding_Conventions.md      (after merge)
├── doc/                                   (whole folder — content merged into telemetry.md)

ARCHIVE (or delete; git history preserves them either way):
├── CURRENT_STATUS.md                      → docs/archive/2025-12-current-status.md
├── TODO_ANALYSIS.md                       → docs/archive/2025-12-todo-analysis.md
├── LIGHTING_IMPROVEMENTS.md               → docs/archive/2025-12-lighting-improvements.md
```

## What I'd like your decisions on

1. **Codex folder** — still using the Codex tool, or can the whole `codex/` folder go?
2. **December snapshots** — delete outright or archive under `docs/archive/`? Archive is gentler but adds clutter; the git history preserves them in either case.
3. **Wrong docs** — comfortable deleting `TEST_BENCH_MODE.md`, `LAUNCHER_RPM_ROADMAP.md`, and `TeamCode/README.md` since they describe code that doesn't exist? They're more misleading than useful.
4. **CLAUDE.md PRACTICE inconsistency** — fix while we're here? It's a 3-line edit.
5. **Conventions consolidation** — `AGENTS.md` + `CONVENTIONS.md` + `DECODE_2025_Coding_Conventions.md` all into one new `docs/conventions.md`, then update `CLAUDE.md` to reference it. OK?
6. **Whiffle ball merge** — merge the three files into one canonical doc? The first one is conceptual, the second is the fix changelog, the third is quick-start. A single doc with conceptual / fix-history / quick-start sections covers all bases.

## Suggested execution order if you approve

This is order-of-execution, not priority. Each step is independent and safe to revert.

1. **Quick wins (deletions of wrong content):** `TEST_BENCH_MODE.md`, `LAUNCHER_RPM_ROADMAP.md`, `TeamCode/README.md`. ~5 minutes.
2. **Fact-fix in `CLAUDE.md`:** Remove the PRACTICE inconsistency. ~2 minutes.
3. **Telemetry consolidation:** Fix the level-count claim in `TeamCode/.../telemetry/README.md`. Merge `doc/logging_overview.md` content in. Delete `doc/`. ~15 minutes.
4. **Snapshot archival:** Create `docs/archive/`, move the three December files. ~5 minutes.
5. **Conventions merge:** Consolidate three convention files to one. ~20 minutes.
6. **Subsystem reshuffle:** Move existing `docs/` files into `subsystems/` and `operations/` subfolders. ~10 minutes.
7. **Aiming + whiffle merges:** Combine the duplicate-topic docs. ~20 minutes each.
8. **New `docs/README.md` index:** Lists everything for navigation. ~10 minutes.
9. **Rewrite root `README.md`:** Brief project intro pointing to `CLAUDE.md` and `docs/`. ~10 minutes.

Total budget if you do all of it: **~2 hours.** Easy to split across sessions.

---

*Audit performed by Claude, cross-referenced against `testCode` branch at commit `b5d84bb`. If you accept any of these recommendations, tell me which numbered items to execute.*
