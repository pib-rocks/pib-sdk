#!/usr/bin/env bash
set -euo pipefail
cd /home/pib/opencode/pib-sdk
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="${CURSOR_API_KEY:-}"
test -n "${CURSOR_API_KEY:-}" || { echo "CURSOR_API_KEY not set"; exit 2; }
echo "BRANCH_BEFORE=$(git branch --show-current)"
timeout 900 cursor-agent -p "$(cat .hermes/plans/PR-1655-phase2-prompt.txt)" \
  --output-format text --force 2>&1 | tail -200
echo "CURSOR_EXIT=${PIPESTATUS[0]}"
echo "=== git status after ==="
git status --short --untracked-files=no