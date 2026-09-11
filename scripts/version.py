#!/usr/bin/env python3
"""
version

Derives ROTATOR_VERSION from the git tag, so a build always reports the
commit it was actually built from instead of a number someone has to
remember to bump - the same idea as QlockThreeW32's scripts/version.py.

Runs as a CMake custom target in main/CMakeLists.txt, ALL, with no OUTPUT
file of its own - that makes it rerun on every single build, not just once
at CMake configure time. A plain `execute_process(git describe ...)` in
CMakeLists.txt (what this project did before) only runs when CMake itself
reconfigures, which plain `idf.py build` after committing or tagging does
not trigger - the reported version then quietly goes stale until something
forces a reconfigure (deleting the build dir, touching CMakeLists.txt).

`git describe --tags --dirty --match "v[0-9]*"` yields e.g. "0.9.1" on a
tagged commit and "0.9.1-2-gdb8d03a-dirty" on work in progress after it.
ROTATOR_BUILD_VERSION_OVERRIDE (set by scripts/dev-container.sh's release
builds) takes priority over git describe when present, matching what the
old CMakeLists.txt code honoured. If neither applies (no tags reachable,
git unavailable, not a git checkout at all), the fallback compiled into
main/Version.h is read back out via regex rather than duplicated by hand -
see that file's own comment.

Writes the resolved version into a small generated header that
main/CMakeLists.txt force-includes into every translation unit in the
`main` component (only rewritten when its content actually changes, so
unrelated builds do not needlessly recompile the handful of files that use
ROTATOR_VERSION), and into a stamp file the release workflow reads rather
than working the version out a second time - two implementations of the
same rule would eventually disagree.
"""
import argparse
import os
import re
import subprocess
import sys


def git_version(source_dir):
    override = os.environ.get("ROTATOR_BUILD_VERSION_OVERRIDE", "").strip()
    if override:
        return override
    try:
        described = subprocess.check_output(
            ["git", "describe", "--tags", "--dirty", "--match", "v[0-9]*"],
            cwd=source_dir,
            stderr=subprocess.DEVNULL,
            universal_newlines=True,
        ).strip()
    except Exception:
        # No git, not a repository, or no matching tag reachable from HEAD.
        return None
    if described.startswith("v"):
        described = described[1:]
    return described if described[:1].isdigit() else None


def fallback_version(fallback_header):
    """The value compiled in when no tag applies, straight from Version.h."""
    try:
        with open(fallback_header, "r") as header:
            match = re.search(r'#define\s+ROTATOR_VERSION\s+"([^"]+)"', header.read())
            if match:
                return match.group(1)
    except Exception:
        pass
    return "0.0.0"


def write_if_changed(path, content):
    """Only touches mtime when content actually differs, so ninja does not
    treat every build as invalidating everything that includes this header."""
    try:
        with open(path, "r") as existing:
            if existing.read() == content:
                return
    except Exception:
        pass
    with open(path, "w") as out:
        out.write(content)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", required=True, help="Git working tree to describe")
    parser.add_argument("--fallback-header", required=True, help="main/Version.h")
    parser.add_argument("--generated-header", required=True, help="force-included header to (re)write")
    parser.add_argument("--stamp-file", required=True, help="plain-text version, read by the release workflow")
    args = parser.parse_args()

    version = git_version(args.source_dir)
    if version:
        print("Rotator version from git: %s" % version)
    else:
        version = fallback_version(args.fallback_header)
        print("No usable git tag, keeping the fallback version %s from %s" % (version, args.fallback_header))

    os.makedirs(os.path.dirname(args.generated_header), exist_ok=True)
    write_if_changed(args.generated_header, '#pragma once\n#define ROTATOR_VERSION "%s"\n' % version)

    os.makedirs(os.path.dirname(args.stamp_file), exist_ok=True)
    with open(args.stamp_file, "w") as stamp:
        stamp.write(version + "\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
