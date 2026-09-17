#!/usr/bin/env bash
# Build installable extension zips into dist/.
#
#   scripts/package.sh              # all add-ons
#   scripts/package.sh opencv_camera
set -euo pipefail

BLENDER="${BLENDER:-/Applications/Blender.app/Contents/MacOS/Blender}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DIST="${REPO_ROOT}/dist"

if [ ! -x "${BLENDER}" ]; then
  echo "Blender not found at ${BLENDER}; set the BLENDER environment variable" >&2
  exit 1
fi

mkdir -p "${DIST}"

if [ "$#" -gt 0 ]; then
  ADDONS=("$@")
else
  ADDONS=()
  for path in "${REPO_ROOT}"/addons/*/; do
    ADDONS+=("$(basename "${path}")")
  done
fi

for addon in "${ADDONS[@]}"; do
  echo "== ${addon}"
  "${BLENDER}" -b --factory-startup --command extension build \
    --source-dir "${REPO_ROOT}/addons/${addon}" --output-dir "${DIST}"
done

ls -la "${DIST}"