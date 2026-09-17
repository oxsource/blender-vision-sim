#!/usr/bin/env bash
# Symlink the add-ons of this repository into Blender's "User Default" extension
# repository so they can be enabled in Preferences ▸ Add-ons (Extensions).
#
#   scripts/dev_install.sh                 # all add-ons, default Blender version
#   scripts/dev_install.sh -v 5.0          # target another Blender version
#   scripts/dev_install.sh opencv_camera   # a single add-on
set -euo pipefail

BLENDER_VERSION="${BLENDER_VERSION:-4.5}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

case "$(uname -s)" in
  Darwin) CONFIG="${HOME}/Library/Application Support/Blender" ;;
  Linux)  CONFIG="${HOME}/.config/blender" ;;
  *)      echo "unsupported platform: $(uname -s)" >&2; exit 1 ;;
esac

while getopts "v:" opt; do
  case "${opt}" in
    v) BLENDER_VERSION="${OPTARG}" ;;
    *) exit 2 ;;
  esac
done
shift $((OPTIND - 1))

EXTENSIONS_DIR="${CONFIG}/${BLENDER_VERSION}/extensions/user_default"
mkdir -p "${EXTENSIONS_DIR}"

if [ "$#" -gt 0 ]; then
  ADDONS=("$@")
else
  ADDONS=()
  for path in "${REPO_ROOT}"/addons/*/; do
    ADDONS+=("$(basename "${path}")")
  done
fi

for addon in "${ADDONS[@]}"; do
  source_dir="${REPO_ROOT}/addons/${addon}"
  if [ ! -f "${source_dir}/blender_manifest.toml" ]; then
    echo "skip ${addon}: no blender_manifest.toml" >&2
    continue
  fi
  target="${EXTENSIONS_DIR}/${addon}"
  rm -rf "${target}"
  ln -s "${source_dir}" "${target}"
  echo "linked ${target} -> ${source_dir}"
  echo "  enable with: bpy.ops.preferences.addon_enable(module='bl_ext.user_default.${addon}')"
done

echo
echo "Restart Blender (or use Extensions ▸ Refresh) and enable the add-on in Preferences ▸ Add-ons."