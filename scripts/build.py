#!/usr/bin/env python3
"""Build all CMake presets for every project under projects/ (or a single --path)."""
import argparse
import re
import subprocess
import sys
from pathlib import Path
from typing import List, Optional, Tuple, Final

REPO_ROOT:Final = Path(__file__).resolve().parent.parent
PROJECTS_DIR:Final = REPO_ROOT / "projects"

QUOTED_NAME_RE = re.compile(r'"([^"]+)"')

def find_projects(path: Optional[Path]) -> List[Path]:
    if path is not None:
        if not (path / "CMakeLists.txt").is_file():
            print(f"error: {path} has no CMakeLists.txt", file=sys.stderr)
            sys.exit(1)
        return [path]

    if not PROJECTS_DIR.is_dir():
        print(f"error: {PROJECTS_DIR} does not exist", file=sys.stderr)
        sys.exit(1)

    return sorted(
        entry for entry in PROJECTS_DIR.iterdir()
        if entry.is_dir() and (entry / "CMakeLists.txt").is_file()
    )


def list_presets(project_dir: Path) -> List[str]:
    result = subprocess.run(
        ["cmake", "--list-presets"],
        cwd=project_dir,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(result.stdout, end="")
        print(result.stderr, file=sys.stderr, end="")
        return []

    presets = []
    in_configure_section = False
    for line in result.stdout.splitlines():
        if "Available configure presets" in line:
            in_configure_section = True
            continue
        if in_configure_section:
            match = QUOTED_NAME_RE.search(line)
            if match:
                presets.append(match.group(1))
    return presets


def run_streamed(cmd: List[str], cwd: Path) -> bool:
    print(f"\n$ {' '.join(cmd)}   (in {cwd})")
    return subprocess.run(cmd, cwd=cwd).returncode == 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--path",
        type=Path,
        help="Build only this single project directory instead of scanning projects/",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    # Find projects to scan
    path = args.path.resolve() if args.path else None
    projects = find_projects(path)

    if not projects:
        print("No projects with a CMakeLists.txt found.", file=sys.stderr)
        sys.exit(1)

    results: List[Tuple[str, str, bool]] = []

    # Process all projects and compile the project
    for project_dir in projects:
        project_name = project_dir.name
        print(f"\n{'=' * 70}\n{project_name}\n{'=' * 70}")

        # Get all presets
        presets = list_presets(project_dir)
        if not presets:
            print("  no configure presets found, skipping")
            continue

        # Configure and build the specific preset
        for preset in presets:
            print(f"\n--- {project_name} :: {preset} ---")
            configure_ok = run_streamed(["cmake", "--preset", preset], project_dir)
            build_ok = configure_ok and run_streamed(
                ["cmake", "--build", "--preset", preset], project_dir
            )
            results.append((project_name, preset, build_ok))

    print(f"\n{'=' * 70}\nSummary\n{'=' * 70}")
    if not results:
        print("Nothing was built.")
        sys.exit(1)

    name_width = max(len(name) for name, _, _ in results)
    preset_width = max(len(preset) for _, preset, _ in results)
    failed = 0
    for project_name, preset, ok in results:
        status = "PASS" if ok else "FAIL"
        failed += not ok
        print(f"  {project_name:<{name_width}}  {preset:<{preset_width}}  {status}")

    total = len(results)
    print(f"\n{total - failed}/{total} builds passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
