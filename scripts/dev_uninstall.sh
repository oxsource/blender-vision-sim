#!/usr/bin/env bash
# Remove the symlinks created by dev_install.sh.
set -euo pipefail

BLENDER_VERSION="${BLENDER_VERSION:-4.5}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "$(uname -s)" in
  Darwin) CONFIG="${HOME}/Library/Application Support/Blender" ;;
  Linux)  CONFIG="${HOME}/.config/blender" ;;
  *)      echo "unsupported platform: $(uname -s)" >&2; exit 1 ;;
esac

EXTENSIONS_DIR="${CONFIG}/${BLENDER_VERSION}/extensions/user_default"

for path in "${REPO_ROOT}"/addons/*/; do
  addon="$(basename "${path}")"
  target="${EXTENSIONS_DIR}/${addon}"
  if [ -L "${target}" ]; then
    rm -f "${target}"
    echo "removed ${target}"
  fi
done