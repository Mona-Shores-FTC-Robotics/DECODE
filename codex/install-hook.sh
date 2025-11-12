#!/usr/bin/env bash
set -euo pipefail
git config core.hooksPath .githooks
chmod +x .githooks/pre-commit || true
echo "Git hooks path set to .githooks"
echo "Protective pre-commit hook installed."
