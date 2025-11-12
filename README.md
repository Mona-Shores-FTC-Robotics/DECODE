# FTC + Codex Starter

This bundle gives you a safe, repeatable way to run Codex on an FTC Android Studio project.

## What is inside
- `codex/run-task.sh`: Bash launcher for Codex tasks
- `codex/run-task.ps1`: PowerShell launcher for Codex tasks
- `codex/codex_task_template.md`: Prompt template with FTC guardrails
- `codex/tasks/flywheel_stability.md`: Example task file
- `.githooks/pre-commit`: Git hook that blocks Gradle and SDK version changes
- `codex/install-hook.sh`: One-time script to enable the Git hook

## Quick start
1) Copy the `codex/` folder and the `.githooks/` folder into the **root** of your FTC repo.
2) Run `bash codex/install-hook.sh` once to enable the protective pre-commit hook.
3) Open the repo in Android Studio as usual.
4) In a terminal at the repo root, run one of these:
   - `bash codex/run-task.sh codex/tasks/flywheel_stability.md`
   - `powershell -ExecutionPolicy Bypass -File codex/run-task.ps1 codex\tasks\flywheel_stability.md`
5) Review Codex diffs in Android Studio, build, test on robot, and commit approved changes.

> The hook prevents commits that modify Gradle or SDK version files. This reduces the risk of Codex changing project configuration that FTC depends on.
