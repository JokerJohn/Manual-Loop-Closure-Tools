#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


CURRENT_DIR = Path(__file__).resolve().parent
GUI_DIR = CURRENT_DIR.parents[1]
if str(GUI_DIR) not in sys.path:
    sys.path.insert(0, str(GUI_DIR))

from manual_loop_closure.optimizer_backend import OptimizerRunOptions  # noqa: E402
from manual_loop_closure.python_optimizer.optimizer import run_python_optimizer  # noqa: E402


# ===== BEGIN CHANGE: python optimizer cli =====
def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pure Python manual loop optimizer backend.")
    parser.add_argument("--session-root", required=True, type=Path)
    parser.add_argument("--g2o", required=True, type=Path)
    parser.add_argument("--tum", required=True, type=Path)
    parser.add_argument("--keyframe-dir", required=True, type=Path)
    parser.add_argument("--constraints-csv", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--map-voxel-leaf", required=False, type=float, default=0.2)
    parser.add_argument("--skip-map-build", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    options = OptimizerRunOptions(
        session_root=args.session_root,
        g2o_path=args.g2o,
        tum_path=args.tum,
        keyframe_dir=args.keyframe_dir,
        constraints_csv=args.constraints_csv,
        output_dir=args.output_dir,
        map_voxel_leaf=float(args.map_voxel_leaf),
        skip_map_build=bool(args.skip_map_build),
    )
    try:
        result = run_python_optimizer(options, log_fn=print)
    except Exception as exc:
        print(f"[PythonOptimizer] ERROR: {exc}", file=sys.stderr)
        return 1

    print(f"  output_dir: {result.output_dir}")
    print(f"  enabled_constraints: {result.enabled_constraints}")
    print(f"  pose_count: {result.pose_count}")
    print(f"  factor_count: {result.factor_count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
# ===== END CHANGE: python optimizer cli =====
