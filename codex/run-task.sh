#!/usr/bin/env bash
set -euo pipefail

TASK_FILE="${1:-codex/tasks/flywheel_stability.md}"

if ! command -v codex >/dev/null 2>&1; then
  echo "Codex CLI not found. Please install and authenticate it first."
  echo "Example: npm i -g @openai/codex  (or follow the official docs)"
  exit 1
fi

if [ ! -f "$TASK_FILE" ]; then
  echo "Task file not found: $TASK_FILE"
  exit 1
fi

echo "Running Codex with task: $TASK_FILE"
codex run --task-file "$TASK_FILE"
