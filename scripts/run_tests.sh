#!/usr/bin/env bash
# Run the test suite: the pure Python tests always, the headless Blender
# integration tests when Blender is available.
#
#   scripts/run_tests.sh                 # skip the Blender part if not installed
#   scripts/run_tests.sh --no-blender    # force skipping it
#   BLENDER=/path/to/blender scripts/run_tests.sh
set -euo pipefail

BLENDER="${BLENDER:-/Applications/Blender.app/Contents/MacOS/Blender}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_BLENDER=1
for arg in "$@"; do
  case "${arg}" in
    --no-blender) RUN_BLENDER=0 ;;
    *) echo "unknown option ${arg}" >&2; exit 2 ;;
  esac
done

echo "== core tests (no Blender)"
python3 "${REPO_ROOT}/tests/test_core.py"

echo
echo "== release tooling tests (no Blender)"
python3 "${REPO_ROOT}/tests/test_release_scripts.py"

echo
if [ "${RUN_BLENDER}" = "0" ]; then
  echo "== Blender integration tests: skipped (--no-blender)"
elif [ ! -x "${BLENDER}" ]; then
  echo "== Blender integration tests: skipped (no Blender at ${BLENDER})"
  echo "   set BLENDER=/path/to/blender to run them (they cover the OSL camera,"
  echo "   the render self test and the panel/menu registration)"
else
  echo "== Blender integration tests"
  "${BLENDER}" -b --factory-startup --python "${REPO_ROOT}/tests/run_blender_tests.py" 2>&1 \
    | grep -Ev "^(Blender|Read prefs|found bundled|Fra:|Saved:|Time:)"
fi