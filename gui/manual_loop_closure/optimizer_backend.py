from __future__ import annotations

import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


BACKEND_PREFERENCE_AUTO = "auto"
BACKEND_PREFERENCE_PYTHON = "python"
BACKEND_PREFERENCE_CPP = "cpp"


# ===== BEGIN CHANGE: optimizer backend adapter =====
@dataclass(frozen=True)
class OptimizerRunOptions:
    session_root: Path
    g2o_path: Path
    tum_path: Path
    keyframe_dir: Path
    constraints_csv: Path
    output_dir: Path
    map_voxel_leaf: float

    def to_cli_args(self) -> list[str]:
        return [
            "--session-root",
            str(self.session_root),
            "--g2o",
            str(self.g2o_path),
            "--tum",
            str(self.tum_path),
            "--keyframe-dir",
            str(self.keyframe_dir),
            "--constraints-csv",
            str(self.constraints_csv),
            "--output-dir",
            str(self.output_dir),
            "--map-voxel-leaf",
            f"{self.map_voxel_leaf:.6f}",
        ]


@dataclass(frozen=True)
class OptimizerRunResult:
    output_dir: Path
    output_g2o: Path
    output_tum: Path
    output_map_pcd: Path
    output_trajectory_pcd: Path
    output_report_json: Path
    factor_count: int
    pose_count: int
    enabled_constraints: int


@dataclass(frozen=True)
class ResolvedOptimizerBackend:
    key: str
    display_name: str
    program: str
    arguments_prefix: tuple[str, ...]

    def build_process_args(self, options: OptimizerRunOptions) -> list[str]:
        return [*self.arguments_prefix, *options.to_cli_args()]


def _supports_python_optimizer(python_executable: Path) -> bool:
    try:
        result = subprocess.run(
            [
                str(python_executable),
                "-c",
                "import gtsam, numpy, scipy",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except OSError:
        return False
    return result.returncode == 0


def _candidate_python_executables(project_root: Optional[Path]) -> list[Path]:
    candidates: list[Path] = []
    current = Path(sys.executable).resolve()

    def append_candidate(path: Optional[Path]) -> None:
        if path is None:
            return
        resolved = path.expanduser().resolve()
        if not resolved.is_file():
            return
        if resolved not in candidates:
            candidates.append(resolved)

    append_candidate(Path(os.environ["MANUAL_LOOP_OPTIMIZER_PYTHON"])) if os.environ.get(
        "MANUAL_LOOP_OPTIMIZER_PYTHON"
    ) else None
    append_candidate(current)

    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        append_candidate(Path(conda_prefix) / "bin" / "python3")
        append_candidate(Path(conda_prefix) / "bin" / "python")

    if project_root is not None:
        append_candidate(project_root / ".venv" / "bin" / "python3")
        append_candidate(project_root / ".venv" / "bin" / "python")

    home = Path.home()
    for base_dir in (home / "anaconda3", home / "miniconda3"):
        append_candidate(base_dir / "bin" / "python3")
        append_candidate(base_dir / "bin" / "python")

    return candidates


def resolve_python_optimizer_backend(
    *,
    script_dir: Path,
    project_root: Optional[Path],
) -> Optional[ResolvedOptimizerBackend]:
    cli_script = script_dir / "manual_loop_closure" / "python_optimizer" / "cli.py"
    if not cli_script.is_file():
        return None

    for candidate in _candidate_python_executables(project_root):
        if not _supports_python_optimizer(candidate):
            continue
        return ResolvedOptimizerBackend(
            key=BACKEND_PREFERENCE_PYTHON,
            display_name=f"python:{candidate}",
            program=str(candidate),
            arguments_prefix=(str(cli_script),),
        )
    return None
# ===== END CHANGE: optimizer backend adapter =====

