#!/usr/bin/env python3
from __future__ import annotations

import importlib
import os
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def run(command: list[str]) -> str:
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=False)
    except OSError as exc:
        return f"MISSING ({exc})"
    output = (result.stdout or result.stderr).strip()
    return output or f"exit={result.returncode}"


def import_version(module_name: str) -> str:
    try:
        module = importlib.import_module(module_name)
    except Exception as exc:
        return f"MISSING ({exc})"
    if module_name == "PyQt5":
        from PyQt5 import QtCore  # type: ignore

        return f"{QtCore.PYQT_VERSION_STR} / Qt {QtCore.QT_VERSION_STR}"
    return str(getattr(module, "__version__", "unknown"))


def first_existing(paths: list[Path]) -> str:
    for path in paths:
        if path.exists():
            return str(path)
    return "not found"


def main() -> int:
    print("== Manual Loop Closure Tools: environment check ==")
    print(f"Repo root: {REPO_ROOT}")
    print(f"Python: {sys.executable}")
    print(f"Python version: {sys.version.split()[0]}")
    print(f"ROS_DISTRO: {os.environ.get('ROS_DISTRO', 'unset')}")
    print()

    print("[Python packages]")
    for name in ["open3d", "PyQt5", "numpy", "scipy", "matplotlib", "gtsam"]:
        print(f"  {name}: {import_version(name)}")
    print()

    print("[Toolchain]")
    print(f"  catkin: {run(['catkin', '--version']).splitlines()[0]}")
    print(f"  cmake: {run(['cmake', '--version']).splitlines()[0]}")
    print(f"  g++: {run(['g++', '--version']).splitlines()[0]}")
    print()

    print("[Common CMake package paths]")
    gtsam_candidates = [
        Path("/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake"),
        Path("/usr/lib/x86_64-linux-gnu/cmake/GTSAM/GTSAMConfig.cmake"),
    ]
    print(f"  GTSAMConfig.cmake: {first_existing(gtsam_candidates)}")
    print()

    print("[Backend build hints]")
    optimizer_candidates = [
        REPO_ROOT / "gui" / "manual_loop_closure" / "python_optimizer" / "cli.py",
        REPO_ROOT / "backend" / "catkin_ws" / "devel" / "lib" / "manual_loop_closure_backend" / "manual_loop_optimize",
        REPO_ROOT / "backend" / "catkin_ws" / "install" / "lib" / "manual_loop_closure_backend" / "manual_loop_optimize",
    ]
    print(f"  python optimizer cli: {first_existing([optimizer_candidates[0]])}")
    print(f"  legacy optimizer binary: {first_existing(optimizer_candidates[1:])}")
    print("  build command: bash scripts/build_backend_catkin.sh")
    print("  GUI launch:    python launch_gui.py --session-root /path/to/session")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
