#!/usr/bin/env python3
"""Build installable extension zips without needing Blender.

    python3 scripts/package.py                    # every add-on in addons/
    python3 scripts/package.py opencv_camera      # one add-on
    python3 scripts/package.py --check            # validate manifests only
    python3 scripts/package.py --output-dir /tmp/out

The zip layout is the extension layout: ``blender_manifest.toml`` and the add-on
package at the archive root.  Output goes to ``dist/`` next to a ``.sha256`` file.
Entry timestamps are fixed so the same sources produce byte-identical archives.

``blender --command extension build`` (see ``scripts/package.sh``) is still the
authority for validation; this script is the dependency-free path used by CI and by
anyone without Blender installed.
"""

from __future__ import annotations

import hashlib
import os
import sys
import tomllib
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ADDONS_DIR = os.path.join(ROOT, "addons")
DIST_DIR = os.path.join(ROOT, "dist")

#: never pack these (extension builder defaults + editor/OS noise)
EXCLUDE_DIRS = {"__pycache__", ".git", ".vscode", ".idea"}
EXCLUDE_SUFFIXES = (".pyc", ".pyo", ".zip", ".blend1", ".blend2", ".oso", ".DS_Store")
#: fixed timestamp for reproducible archives (1980-01-01, the zip epoch)
FIXED_DATE = (1980, 1, 1, 0, 0, 0)


def read_manifest(addon_dir: str) -> dict:
    path = os.path.join(addon_dir, "blender_manifest.toml")
    with open(path, "rb") as handle:
        return tomllib.load(handle)


def check_manifest(addon_dir: str) -> list:
    """Minimal sanity check of a manifest; returns a list of problems."""
    problems = []
    name = os.path.basename(addon_dir.rstrip("/"))
    try:
        manifest = read_manifest(addon_dir)
    except FileNotFoundError:
        return [f"{name}: no blender_manifest.toml"]
    except tomllib.TOMLDecodeError as exc:
        return [f"{name}: invalid TOML: {exc}"]
    identifier = manifest.get("id", "")
    if not identifier:
        problems.append(f"{name}: manifest has no 'id'")
    elif identifier != name:
        problems.append(f"{name}: manifest id {identifier!r} != directory name")
    for key in ("version", "name", "tagline", "maintainer", "type", "license"):
        if not manifest.get(key):
            problems.append(f"{name}: manifest has no '{key}'")
    if manifest.get("schema_version") != "1.0.0":
        problems.append(f"{name}: unexpected schema_version {manifest.get('schema_version')!r}")
    if not os.path.exists(os.path.join(addon_dir, "__init__.py")):
        problems.append(f"{name}: no __init__.py")
    return problems


def iter_files(addon_dir: str):
    for current, dirs, files in os.walk(addon_dir):
        dirs[:] = sorted(d for d in dirs if d not in EXCLUDE_DIRS)
        for name in sorted(files):
            if name.endswith(EXCLUDE_SUFFIXES):
                continue
            full = os.path.join(current, name)
            yield full, os.path.relpath(full, addon_dir)


def build(addon_dir: str, output_dir: str = DIST_DIR) -> str:
    manifest = read_manifest(addon_dir)
    identifier = manifest["id"]
    version = manifest["version"]
    os.makedirs(output_dir, exist_ok=True)
    zip_path = os.path.join(output_dir, f"{identifier}-{version}.zip")

    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for full, relative in iter_files(addon_dir):
            info = zipfile.ZipInfo(relative.replace(os.sep, "/"), date_time=FIXED_DATE)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            with open(full, "rb") as handle:
                archive.writestr(info, handle.read())

    digest = hashlib.sha256(open(zip_path, "rb").read()).hexdigest()
    with open(zip_path + ".sha256", "w", encoding="utf-8") as handle:
        handle.write(f"{digest}  {os.path.basename(zip_path)}\n")
    print(f"built {zip_path} ({os.path.getsize(zip_path)} bytes, sha256 {digest[:16]}…)")
    return zip_path


def addon_dirs(names) -> list:
    if names:
        return [os.path.join(ADDONS_DIR, name) for name in names]
    return [
        os.path.join(ADDONS_DIR, name)
        for name in sorted(os.listdir(ADDONS_DIR))
        if os.path.isdir(os.path.join(ADDONS_DIR, name))
    ]


def main(argv) -> int:
    check_only = "--check" in argv
    output_dir = DIST_DIR
    if "--output-dir" in argv:
        output_dir = argv[argv.index("--output-dir") + 1]
    skip = {"--check", "--output-dir", output_dir}
    names = [a for a in argv if a not in skip and not a.startswith("-")]
    directories = addon_dirs(names)
    if not directories:
        print("no add-ons found", file=sys.stderr)
        return 1

    problems = []
    for directory in directories:
        problems.extend(check_manifest(directory))
    if problems:
        for problem in problems:
            print(f"ERROR {problem}", file=sys.stderr)
        return 1
    print(f"manifests OK ({len(directories)} add-on(s))")
    if check_only:
        return 0

    for directory in directories:
        build(directory, output_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))