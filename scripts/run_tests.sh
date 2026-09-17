#!/usr/bin/env bash
# Run the whole test suite: pure Python core tests + headless Blender tests.
set -euo pipefail

BLENDER="${BLENDER:-/Applications/Blender.app/Contents/MacOS/Blender}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "== core tests (no Blender)"
python3 "${REPO_ROOT}/tests/test_core.py"

echo
echo "== Blender integration tests"
"${BLENDER}" -b --factory-startup --python "${REPO_ROOT}/tests/run_blender_tests.py" 2>&1 \
  | grep -Ev "^(Blender|Read prefs|found bundled|Fra:|Saved:|Time:)"