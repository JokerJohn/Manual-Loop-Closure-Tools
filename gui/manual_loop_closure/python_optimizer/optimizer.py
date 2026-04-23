from __future__ import annotations

import csv
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from manual_loop_closure.optimizer_backend import OptimizerRunOptions, OptimizerRunResult
from manual_loop_closure.trajectory_io import load_tum_trajectory

from .exporters import (
    MeasurementRecord,
    build_optimized_map,
    generate_pose_graph_png,
    save_report_json,
    save_trajectory_pcd,
    save_tum,
    save_xyzi_pcd,
    write_pose_graph_g2o,
)
from .graph_loader import BetweenFactorRecord, build_factor_graph, pose_key


LogFn = Optional[Callable[[str], None]]


# ===== BEGIN CHANGE: python optimizer orchestration =====
@dataclass(frozen=True)
class ManualConstraintSpec:
    enabled: bool
    source_id: int
    target_id: int
    translation_xyz: np.ndarray
    quat_xyzw: np.ndarray
    sigma_t_xyz: np.ndarray
    sigma_r_deg: np.ndarray


def _log(log_fn: LogFn, message: str) -> None:
    if log_fn is not None:
        log_fn(message)


def _import_gtsam():
    try:
        import gtsam  # type: ignore
    except Exception as exc:  # pragma: no cover - import behavior is environment-specific
        raise RuntimeError(
            "Python GTSAM is unavailable. Install the Python wrapper first "
            "(see docs/INSTALL_GTSAM_PYTHON.md)."
        ) from exc
    return gtsam


def _load_measurements(tum_path: Path, keyframe_dir: Path) -> list[MeasurementRecord]:
    trajectory = load_tum_trajectory(tum_path)
    cloud_paths = sorted(
        [
            entry
            for entry in keyframe_dir.iterdir()
            if entry.is_file() and entry.suffix.lower() == ".pcd" and entry.stem.isdigit()
        ],
        key=lambda item: int(item.stem),
    )
    if len(cloud_paths) != trajectory.size:
        raise RuntimeError(
            "Keyframe count does not match optimized_poses_tum.txt: "
            f"pcd={len(cloud_paths)} tum={trajectory.size}"
        )
    for expected_index, path in enumerate(cloud_paths):
        actual_index = int(path.stem)
        if actual_index != expected_index:
            raise RuntimeError(
                "Keyframe numbering must match 0..N-1 exactly. "
                f"Expected {expected_index}.pcd but found {path.name}"
            )
    measurements = [
        MeasurementRecord(index=index, odom_time=float(trajectory.timestamps[index]), cloud_path=cloud_paths[index])
        for index in range(trajectory.size)
    ]
    return measurements


def _parse_bool(text: str) -> bool:
    normalized = text.strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no"}:
        return False
    raise RuntimeError(f"Invalid bool value in constraints csv: {text}")


def _load_constraints_csv(path: Path) -> list[ManualConstraintSpec]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = [
            "enabled",
            "source_id",
            "target_id",
            "tx",
            "ty",
            "tz",
            "qx",
            "qy",
            "qz",
            "qw",
            "sigma_tx",
            "sigma_ty",
            "sigma_tz",
            "sigma_roll_deg",
            "sigma_pitch_deg",
            "sigma_yaw_deg",
        ]
        if reader.fieldnames is None:
            raise RuntimeError(f"Constraints CSV is empty: {path}")
        for field in required:
            if field not in reader.fieldnames:
                raise RuntimeError(f"Constraints CSV missing required column: {field}")

        constraints: list[ManualConstraintSpec] = []
        for row in reader:
            if row is None:
                continue
            constraints.append(
                ManualConstraintSpec(
                    enabled=_parse_bool(row["enabled"]),
                    source_id=int(row["source_id"]),
                    target_id=int(row["target_id"]),
                    translation_xyz=np.asarray(
                        [row["tx"], row["ty"], row["tz"]],
                        dtype=np.float64,
                    ),
                    quat_xyzw=np.asarray(
                        [row["qx"], row["qy"], row["qz"], row["qw"]],
                        dtype=np.float64,
                    ),
                    sigma_t_xyz=np.asarray(
                        [row["sigma_tx"], row["sigma_ty"], row["sigma_tz"]],
                        dtype=np.float64,
                    ),
                    sigma_r_deg=np.asarray(
                        [row["sigma_roll_deg"], row["sigma_pitch_deg"], row["sigma_yaw_deg"]],
                        dtype=np.float64,
                    ),
                )
            )
    return constraints


def _manual_constraint_information(constraint: ManualConstraintSpec) -> np.ndarray:
    sigma_r_rad = np.deg2rad(constraint.sigma_r_deg)
    sigmas = np.concatenate([sigma_r_rad, constraint.sigma_t_xyz], axis=0)
    information = np.zeros((6, 6), dtype=np.float64)
    for idx, sigma in enumerate(sigmas):
        if sigma <= 0.0:
            raise RuntimeError("Constraint sigmas must be positive.")
        information[idx, idx] = 1.0 / (sigma * sigma)
    return information


def run_python_optimizer(
    options: OptimizerRunOptions,
    log_fn: LogFn = None,
) -> OptimizerRunResult:
    total_start = time.perf_counter()
    gtsam = _import_gtsam()
    _log(log_fn, f"[PythonOptimizer] Using gtsam from {Path(gtsam.__file__).resolve()}")

    stage_start = time.perf_counter()
    measurements = _load_measurements(options.tum_path, options.keyframe_dir)
    _log(
        log_fn,
        f"[PythonOptimizer] Loaded measurements poses={len(measurements)}, elapsed={time.perf_counter() - stage_start:.2f}s",
    )

    stage_start = time.perf_counter()
    graph_result = build_factor_graph(
        session_root=options.session_root,
        g2o_path=options.g2o_path,
        gtsam_mod=gtsam,
        log_fn=log_fn,
    )
    _log(
        log_fn,
        "[PythonOptimizer] Factor graph ready "
        f"poses={graph_result.pose_count}, factors={int(graph_result.graph.size())}, "
        f"elapsed={time.perf_counter() - stage_start:.2f}s",
    )
    if graph_result.pose_count != len(measurements):
        raise RuntimeError(
            "Pose graph and measurements disagree on pose count: "
            f"graph={graph_result.pose_count} measurements={len(measurements)}"
        )

    stage_start = time.perf_counter()
    constraints = _load_constraints_csv(options.constraints_csv)
    _log(
        log_fn,
        f"[PythonOptimizer] Loaded manual constraints total={len(constraints)}, elapsed={time.perf_counter() - stage_start:.2f}s",
    )
    enabled_constraints = 0
    factor_records = list(graph_result.factor_records)
    for constraint in constraints:
        if not constraint.enabled:
            continue
        if (
            constraint.source_id < 0
            or constraint.target_id < 0
            or constraint.source_id >= len(measurements)
            or constraint.target_id >= len(measurements)
        ):
            raise RuntimeError(
                "Constraint index out of range: "
                f"target={constraint.target_id} source={constraint.source_id}"
            )
        measured_pose = gtsam.Pose3(
            gtsam.Rot3.Quaternion(
                float(constraint.quat_xyzw[3]),
                float(constraint.quat_xyzw[0]),
                float(constraint.quat_xyzw[1]),
                float(constraint.quat_xyzw[2]),
            ),
            gtsam.Point3(
                float(constraint.translation_xyz[0]),
                float(constraint.translation_xyz[1]),
                float(constraint.translation_xyz[2]),
            ),
        )
        sigma_r_rad = np.deg2rad(constraint.sigma_r_deg)
        sigmas = np.concatenate([sigma_r_rad, constraint.sigma_t_xyz], axis=0)
        noise = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        graph_result.graph.add(
            gtsam.BetweenFactorPose3(
                pose_key(gtsam, constraint.target_id),
                pose_key(gtsam, constraint.source_id),
                measured_pose,
                noise,
            )
        )
        factor_records.append(
            BetweenFactorRecord(
                node_i=constraint.target_id,
                node_j=constraint.source_id,
                translation_xyz=constraint.translation_xyz,
                quat_xyzw=constraint.quat_xyzw,
                information=_manual_constraint_information(constraint),
                origin="manual",
            )
        )
        enabled_constraints += 1

    _log(
        log_fn,
        f"[PythonOptimizer] Active manual constraints enabled={enabled_constraints}",
    )
    if enabled_constraints == 0:
        _log(
            log_fn,
            "[PythonOptimizer] No enabled manual constraints found in CSV. "
            "Proceeding with the filtered input graph only.",
        )

    params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(50)
    if hasattr(params, "setVerbosityLM"):
        params.setVerbosityLM("SUMMARY")
    initial_error = float(graph_result.graph.error(graph_result.initial_values))
    _log(
        log_fn,
        f"[PythonOptimizer] Starting LM optimize factors={int(graph_result.graph.size())}, initial_error={initial_error:.9e}",
    )
    stage_start = time.perf_counter()
    optimizer = gtsam.LevenbergMarquardtOptimizer(
        graph_result.graph,
        graph_result.initial_values,
        params,
    )
    optimized = optimizer.optimize()
    optimize_elapsed = time.perf_counter() - stage_start
    final_error = float(graph_result.graph.error(optimized))
    _log(
        log_fn,
        "[PythonOptimizer] LM finished "
        f"elapsed={optimize_elapsed:.2f}s, initial_error={initial_error:.9e}, "
        f"final_error={final_error:.9e}, improvement={initial_error - final_error:.9e}",
    )

    options.output_dir.mkdir(parents=True, exist_ok=True)
    output_g2o = options.output_dir / "pose_graph.g2o"
    output_tum = options.output_dir / "optimized_poses_tum.txt"
    output_map = options.output_dir / "global_map_manual_imu.pcd"
    output_trajectory = options.output_dir / "trajectory.pcd"
    output_png = options.output_dir / "pose_graph.png"
    output_report = options.output_dir / "manual_loop_report.json"
    copied_constraints = options.output_dir / "manual_loop_constraints.csv"
    map_built = False
    map_build_elapsed_sec = 0.0
    map_point_count = 0

    write_pose_graph_g2o(
        output_g2o,
        optimized,
        graph_result.pose_count,
        factor_records,
        gtsam,
    )
    save_tum(output_tum, measurements, optimized, gtsam)
    if options.skip_map_build:
        _log(log_fn, "[PythonOptimizer] Map rebuild deferred until export.")
        optimized_map = None
    else:
        map_stage_start = time.perf_counter()
        optimized_map = build_optimized_map(
            measurements,
            optimized,
            gtsam,
            options.map_voxel_leaf,
            log_fn=log_fn,
        )
        if optimized_map.size == 0:
            raise RuntimeError("Optimized point cloud map is empty.")
        save_xyzi_pcd(output_map, optimized_map)
        save_trajectory_pcd(output_trajectory, measurements, optimized, gtsam)
        map_built = True
        map_build_elapsed_sec = time.perf_counter() - map_stage_start
        map_point_count = int(optimized_map.shape[0])
    generate_pose_graph_png(Path(__file__).resolve().parents[3], output_g2o, output_png, log_fn=log_fn)
    save_report_json(
        output_report,
        session_root=options.session_root,
        input_g2o=options.g2o_path,
        input_tum=options.tum_path,
        input_keyframe_dir=options.keyframe_dir,
        constraints_csv=options.constraints_csv,
        output_dir=options.output_dir,
        map_voxel_leaf=options.map_voxel_leaf,
        total_constraints=len(constraints),
        enabled_constraints=enabled_constraints,
        optimized_pose_count=len(measurements),
        factor_count=int(graph_result.graph.size()),
        map_point_count=map_point_count,
        map_built=map_built,
        map_build_elapsed_sec=map_build_elapsed_sec,
    )
    if options.constraints_csv.resolve() != copied_constraints.resolve():
        shutil.copyfile(options.constraints_csv, copied_constraints)

    total_elapsed = time.perf_counter() - total_start
    _log(
        log_fn,
        "[PythonOptimizer] Finished successfully "
        f"total_elapsed={total_elapsed:.2f}s, final_error={final_error:.9e}, "
        f"map_points={map_point_count}",
    )
    return OptimizerRunResult(
        output_dir=options.output_dir,
        output_g2o=output_g2o,
        output_tum=output_tum,
        output_map_pcd=output_map,
        output_trajectory_pcd=output_trajectory,
        output_report_json=output_report,
        factor_count=int(graph_result.graph.size()),
        pose_count=len(measurements),
        enabled_constraints=enabled_constraints,
        map_built=map_built,
    )
# ===== END CHANGE: python optimizer orchestration =====
