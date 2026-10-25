#!/usr/bin/env bash
set -euo pipefail
cd /home/pib/opencode/pib-sdk
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="${CURSOR_API_KEY:-}"
test -n "${CURSOR_API_KEY:-}" || { echo "CURSOR_API_KEY not set"; exit 2; }
echo "BRANCH_BEFORE=$(git branch --show-current)"
echo "PROMPT_BYTES=$(wc -c < .hermes/plans/PR-1687-delegation-prompt.txt)"
timeout 2400 cursor-agent --print --force --output-format stream-json --stream-partial-output \
  "$(cat .hermes/plans/PR-1687-delegation-prompt.txt)"
echo "CURSOR_EXIT=$?"
echo "=== git status after ==="
git status --short --untracked-files=no