#!/usr/bin/env bash
# Version bump + tag, in the spirit of `npm version`.
#
#   scripts/version.sh patch|minor|major|X.Y.Z   # bump, commit, tag
#   scripts/version.sh patch --push              # ... and push commit + tags
#   scripts/version.sh --show                    # print the current version
#   scripts/version.sh minor --dry-run           # show what would happen
#
# The single source of truth is `version` in addons/opencv_camera/blender_manifest.toml.
# The commit is `chore(release): vX.Y.Z` and the annotated tag is `vX.Y.Z`, which is
# what .github/workflows/release.yml reacts to (build the zip, publish a Release).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="${REPO_ROOT}/addons/opencv_camera/blender_manifest.toml"
TAG_PREFIX="v"

current_version() {
  sed -n 's/^version = "\(.*\)"$/\1/p' "${MANIFEST}" | head -1
}

bump() {
  local kind="$1" version="$2"
  IFS='.' read -r major minor patch <<<"${version}"
  case "${kind}" in
    major) echo "$((major + 1)).0.0" ;;
    minor) echo "${major}.$((minor + 1)).0" ;;
    patch) echo "${major}.${minor}.$((patch + 1))" ;;
    [0-9]*.[0-9]*.[0-9]*) echo "${kind}" ;;
    *) echo "usage: $0 patch|minor|major|X.Y.Z [--push] [--dry-run]" >&2; exit 2 ;;
  esac
}

KIND=""
PUSH=0
DRY_RUN=0
for arg in "$@"; do
  case "${arg}" in
    --push) PUSH=1 ;;
    --dry-run) DRY_RUN=1 ;;
    --show) echo "$(current_version)"; exit 0 ;;
    -*) echo "unknown option ${arg}" >&2; exit 2 ;;
    *) KIND="${arg}" ;;
  esac
done

if [ -z "${KIND}" ]; then
  echo "usage: $0 patch|minor|major|X.Y.Z [--push] [--dry-run]" >&2
  exit 2
fi

CURRENT="$(current_version)"
if [ -z "${CURRENT}" ]; then
  echo "could not read the version from ${MANIFEST}" >&2
  exit 1
fi
NEW="$(bump "${KIND}" "${CURRENT}")"
TAG="${TAG_PREFIX}${NEW}"

echo "version: ${CURRENT} -> ${NEW} (tag ${TAG})"
if [ "${DRY_RUN}" = "1" ]; then
  echo "dry run: nothing written"
  exit 0
fi

if [ -n "$(git -C "${REPO_ROOT}" status --porcelain)" ]; then
  echo "working tree is dirty; commit or stash first" >&2
  exit 1
fi
if git -C "${REPO_ROOT}" rev-parse -q --verify "refs/tags/${TAG}" >/dev/null; then
  echo "tag ${TAG} already exists" >&2
  exit 1
fi

python3 - "${MANIFEST}" "${NEW}" <<'PY'
import re
import sys

path, new_version = sys.argv[1], sys.argv[2]
with open(path, encoding="utf-8") as handle:
    text = handle.read()
updated, count = re.subn(r'^version = ".*"$', f'version = "{new_version}"', text, count=1, flags=re.M)
if count != 1:
    raise SystemExit(f"could not update the version in {path}")
with open(path, "w", encoding="utf-8") as handle:
    handle.write(updated)
print(f"updated {path}")
PY

git -C "${REPO_ROOT}" add "${MANIFEST}"
git -C "${REPO_ROOT}" commit -q -m "chore(release): ${TAG}"
git -C "${REPO_ROOT}" tag -a "${TAG}" -m "Release ${TAG}"
echo "committed and tagged ${TAG}"

if [ "${PUSH}" = "1" ]; then
  git -C "${REPO_ROOT}" push
  git -C "${REPO_ROOT}" push --tags
  echo "pushed (CI will build the zip and publish the release)"
else
  echo "next: git push && git push --tags   (or re-run with --push)"
fi