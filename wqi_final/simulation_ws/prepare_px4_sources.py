#!/usr/bin/env python3
"""Materialize pinned repository sources in an external PX4 build workspace."""

import argparse
from pathlib import Path
import subprocess


def git(directory, *arguments):
    return subprocess.check_output(
        ["git", "-C", str(directory), *arguments], text=True
    ).strip()


def prepare(source, destination):
    if not (source / ".git").exists():
        raise RuntimeError(f"Missing submodule {source}; run git submodule update --init --recursive")
    expected = git(source, "rev-parse", "HEAD")
    if not (destination / ".git").exists():
        destination.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["git", "clone", "--no-checkout", str(source), str(destination)], check=True)
        subprocess.run(["git", "-C", str(destination), "checkout", "--detach", expected], check=True)
    elif git(destination, "rev-parse", "HEAD") != expected:
        raise RuntimeError(
            f"Build copy {destination} differs from pinned source {expected}; "
            "preserve local changes and select a fresh WQI_BUILD_ROOT instead of overwriting it"
        )
    origin = git(source, "remote", "get-url", "origin")
    subprocess.run(["git", "-C", str(destination), "remote", "set-url", "origin", origin], check=True)
    if not (source / ".gitmodules").is_file():
        return
    entries = subprocess.run([
        "git", "-C", str(source), "config", "--file", ".gitmodules",
        "--get-regexp", r"^submodule\..*\.path$",
    ], text=True, capture_output=True)
    if entries.returncode not in (0, 1):
        raise RuntimeError(entries.stderr)
    listing = entries.stdout
    for row in listing.splitlines():
        key, path = row.split(None, 1)
        module = key.removesuffix(".path")
        url = git(source, "config", "--file", ".gitmodules", "--get", module + ".url")
        subprocess.run(["git", "-C", str(destination), "config", module + ".url", url], check=True)
        prepare(source / path, destination / path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--build-root", type=Path, required=True)
    arguments = parser.parse_args()
    for name, target in [
        ("PX4-Autopilot", "px4/PX4-Autopilot"),
        ("px4_msgs", "px4_ros_ws/src/px4_msgs"),
        ("Micro-XRCE-DDS-Agent", "px4_ros_ws/src/Micro-XRCE-DDS-Agent"),
    ]:
        prepare(arguments.source_root / name, arguments.build_root / target)
    print("External build sources match the repository's pinned submodules.")


if __name__ == "__main__":
    main()
