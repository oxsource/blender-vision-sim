#!/usr/bin/env python3
"""Tests for the release tooling (no Blender, runs in CI).

    python3 tests/test_release_scripts.py
"""

from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
import tomllib
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE = os.path.join(ROOT, "scripts", "package.py")
VERSION = os.path.join(ROOT, "scripts", "version.sh")
MANIFEST = os.path.join(ROOT, "addons", "opencv_camera", "blender_manifest.toml")

FAILURES = []


def check(name, condition, detail=""):
    status = "PASS" if condition else "FAIL"
    print(f"[{status}] {name}" + (f" - {detail}" if detail else ""))
    if not condition:
        FAILURES.append(name)


def run(args, **kwargs):
    return subprocess.run(args, capture_output=True, text=True, **kwargs)


def manifest_version() -> str:
    with open(MANIFEST, "rb") as handle:
        return tomllib.load(handle)["version"]


def test_package_check():
    result = run([sys.executable, PACKAGE, "--check"])
    check("manifest check passes", result.returncode == 0, result.stderr.strip())
    check("check reports the add-on", "manifests OK" in result.stdout, result.stdout.strip())


def test_package_build_is_reproducible():
    tmp = tempfile.mkdtemp(prefix="opencv_cam_pkg_")
    try:
        first = run([sys.executable, PACKAGE, "--output-dir", tmp])
        check("package build succeeds", first.returncode == 0, first.stderr.strip())
        version = manifest_version()
        zip_path = os.path.join(tmp, f"opencv_camera-{version}.zip")
        check("zip named after id and version", os.path.exists(zip_path), zip_path)
        check("sha256 file written", os.path.exists(zip_path + ".sha256"))

        digest = hashlib.sha256(open(zip_path, "rb").read()).hexdigest()
        recorded = open(zip_path + ".sha256").read().split()[0]
        check("recorded hash matches the zip", digest == recorded)

        with zipfile.ZipFile(zip_path) as archive:
            names = archive.namelist()
            check("manifest sits at the archive root", "blender_manifest.toml" in names)
            check("package root sits at the archive root", "__init__.py" in names)
            check("no bytecode or editor noise packed",
                  not any("__pycache__" in n or n.endswith((".pyc", ".DS_Store", ".zip"))
                          for n in names),
                  str([n for n in names if "__pycache__" in n][:3]))
            check("shaders, icons and presets are packed",
                  any(n.startswith("shaders/") for n in names)
                  and any(n.startswith("icons/") for n in names)
                  and any(n.startswith("presets/") for n in names))
            manifest = tomllib.loads(archive.read("blender_manifest.toml").decode("utf-8"))
            check("packed manifest version matches", manifest["version"] == version)

        again = run([sys.executable, PACKAGE, "--output-dir", tmp])
        digest2 = hashlib.sha256(open(zip_path, "rb").read()).hexdigest()
        check("rebuild is byte identical", again.returncode == 0 and digest == digest2)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _load_packager():
    import importlib.util
    spec = importlib.util.spec_from_file_location("package_script", PACKAGE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_package_rejects_bad_manifest():
    """check_manifest is the gate: a mismatched id or a missing file must fail."""
    packager = _load_packager()
    tmp = tempfile.mkdtemp(prefix="opencv_cam_bad_")
    try:
        addon = os.path.join(tmp, "wrong_id")
        shutil.copytree(os.path.join(ROOT, "addons", "opencv_camera"), addon)
        problems = packager.check_manifest(addon)
        check("a mismatched id is reported",
              any("directory name" in problem for problem in problems), str(problems))

        empty = os.path.join(tmp, "no_manifest")
        os.makedirs(empty)
        check("a missing manifest is reported",
              any("no blender_manifest.toml" in problem for problem in packager.check_manifest(empty)))

        # the shipped add-on itself must be clean
        check("the real add-on passes the check",
              packager.check_manifest(os.path.join(ROOT, "addons", "opencv_camera")) == [])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_version_script():
    result = run(["bash", VERSION, "--show"])
    check("--show prints the manifest version",
          result.returncode == 0 and result.stdout.strip() == manifest_version(),
          result.stdout.strip())

    for kind, expected in (("patch", None), ("minor", None), ("major", None)):
        dry = run(["bash", VERSION, kind, "--dry-run"])
        check(f"dry run {kind}", dry.returncode == 0 and "dry run" in dry.stdout, dry.stdout.strip())
    check("dry run does not touch the manifest", manifest_version() == run(
        ["bash", VERSION, "--show"]).stdout.strip())

    major, minor, patch = (int(part) for part in manifest_version().split("."))
    dry = run(["bash", VERSION, "patch", "--dry-run"])
    check("patch bump math", f"v{major}.{minor}.{patch + 1}" in dry.stdout, dry.stdout.strip())
    dry = run(["bash", VERSION, "minor", "--dry-run"])
    check("minor bump math", f"v{major}.{minor + 1}.0" in dry.stdout, dry.stdout.strip())
    dry = run(["bash", VERSION, "2.3.4", "--dry-run"])
    check("explicit version accepted", "v2.3.4" in dry.stdout, dry.stdout.strip())
    bad = run(["bash", VERSION, "nonsense"])
    check("invalid bump rejected", bad.returncode != 0)


def test_workflows_present():
    for path in (".github/workflows/ci.yml", ".github/workflows/release.yml"):
        full = os.path.join(ROOT, path)
        check(f"{path} exists", os.path.exists(full))
        if os.path.exists(full):
            text = open(full, encoding="utf-8").read()
            check(f"{path} triggers on v* tags" if "release" in path else f"{path} runs tests",
                  "tags: [\"v*\"]" in text if "release" in path else "test_core.py" in text)
    release = os.path.join(ROOT, ".github/workflows/release.yml")
    if os.path.exists(release):
        text = open(release, encoding="utf-8").read()
        check("release attaches the zip and checksum",
              "dist/*.zip" in text and "dist/*.sha256" in text)
        check("release verifies the tag against the manifest",
              "does not match the manifest version" in text)


def main() -> int:
    for test in (test_package_check, test_package_build_is_reproducible,
                 test_package_rejects_bad_manifest, test_version_script,
                 test_workflows_present):
        test()
    print()
    if FAILURES:
        print(f"{len(FAILURES)} failure(s): {', '.join(FAILURES)}")
        return 1
    print("all release tooling tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())