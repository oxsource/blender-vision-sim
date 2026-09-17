#!/usr/bin/env bash
# Build installable extension zips into dist/.
#
#   scripts/package.sh                  # every add-on
#   scripts/package.sh opencv_camera    # one add-on
#   scripts/package.sh --check          # validate manifests only
#   scripts/package.sh --blender        # OPTIONAL extra validation with the official
#                                       # `blender --command extension build`; writes
#                                       # dist-official/ so it never mixes with the
#                                       # reproducible dist/ output.  The default path
#                                       # (and CI) needs no Blender at all.
#
# Without --blender this uses the dependency-free scripts/package.py, so it works
# on machines (and CI runners) without Blender installed.  With Blender available
# the official builder is the authority: it validates the manifest and is used
# automatically when `--blender` is given.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BLENDER="${BLENDER:-/Applications/Blender.app/Contents/MacOS/Blender}"
USE_BLENDER=0
ARGS=()

for arg in "$@"; do
  case "${arg}" in
    --blender) USE_BLENDER=1 ;;
    *) ARGS+=("${arg}") ;;
  esac
done

if [ "${USE_BLENDER}" = "1" ]; then
  if [ ! -x "${BLENDER}" ]; then
    echo "Blender not found at ${BLENDER}; set the BLENDER environment variable" >&2
    exit 1
  fi
  if [ "${#ARGS[@]}" -gt 0 ]; then
    ADDONS=("${ARGS[@]}")
  else
    ADDONS=()
    for path in "${REPO_ROOT}"/addons/*/; do
      ADDONS+=("$(basename "${path}")")
    done
  fi
  OUT="${REPO_ROOT}/dist-official"
  mkdir -p "${OUT}"                     # the builder needs the directory to exist
  for addon in "${ADDONS[@]}"; do
    echo "== ${addon} (blender --command extension build -> dist-official)"
    "${BLENDER}" -b --factory-startup --command extension build \
      --source-dir "${REPO_ROOT}/addons/${addon}" --output-dir "${OUT}"
  done
  ls -la "${OUT}"
  exit 0
fi

python3 "${REPO_ROOT}/scripts/package.py" "${ARGS[@]+"${ARGS[@]}"}"