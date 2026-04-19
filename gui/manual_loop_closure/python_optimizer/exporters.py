from __future__ import annotations

import json
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from merge_pcds import PCDCloud, read_pcd, write_pcd

from .graph_loader import ANCHOR_VERTEX_ID, BetweenFactorRecord, GnssPriorRecord, PosePriorRecord


LogFn = Optional[Callable[[str], None]]


# ===== BEGIN CHANGE: python optimizer exporters =====
@dataclass(frozen=True)
class MeasurementRecord:
    index: int
    odom_time: float
    cloud_path: Path


def _log(log_fn: LogFn, message: str) -> None:
    if log_fn is not None:
        log_fn(message)


def _pose_matrix(pose) -> np.ndarray:
    return np.asarray(pose.matrix(), dtype=np.float64)


def _pose_quat_xyzw(pose) -> np.ndarray:
    return Rotation.from_matrix(_pose_matrix(pose)[:3, :3]).as_quat()


def _make_xyzi_dtype() -> np.dtype:
    return np.dtype(
        [
            ("x", np.float32),
            ("y", np.float32),
            ("z", np.float32),
            ("intensity", np.float32),
        ]
    )


def _make_xyzi_cloud(points_xyzi: np.ndarray) -> PCDCloud:
    dtype = _make_xyzi_dtype()
    structured = np.empty(points_xyzi.shape[0], dtype=dtype)
    structured["x"] = points_xyzi[:, 0].astype(np.float32, copy=False)
    structured["y"] = points_xyzi[:, 1].astype(np.float32, copy=False)
    structured["z"] = points_xyzi[:, 2].astype(np.float32, copy=False)
    structured["intensity"] = points_xyzi[:, 3].astype(np.float32, copy=False)
    return PCDCloud(
        fields=("x", "y", "z", "intensity"),
        sizes=(4, 4, 4, 4),
        types=("F", "F", "F", "F"),
        counts=(1, 1, 1, 1),
        data_type="binary",
        version="0.7",
        viewpoint=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        comments=("# .PCD v0.7 - Point Cloud Data file format",),
        data=structured,
        height=1,
    )


def load_xyzi_points(path: Path) -> np.ndarray:
    cloud = read_pcd(path)
    if not {"x", "y", "z"}.issubset(set(cloud.fields)):
        raise RuntimeError(f"PCD is missing x/y/z fields: {path}")
    intensity = (
        cloud.data["intensity"].astype(np.float32, copy=False)
        if "intensity" in cloud.fields
        else np.zeros(cloud.num_points, dtype=np.float32)
    )
    return np.column_stack(
        [
            cloud.data["x"].astype(np.float32, copy=False),
            cloud.data["y"].astype(np.float32, copy=False),
            cloud.data["z"].astype(np.float32, copy=False),
            intensity,
        ]
    )


def _apply_transform(points_xyzi: np.ndarray, transform: np.ndarray) -> np.ndarray:
    xyz = points_xyzi[:, :3].astype(np.float64, copy=False)
    rotated = xyz @ transform[:3, :3].T + transform[:3, 3]
    transformed = np.empty_like(points_xyzi, dtype=np.float32)
    transformed[:, :3] = rotated.astype(np.float32, copy=False)
    transformed[:, 3] = points_xyzi[:, 3].astype(np.float32, copy=False)
    return transformed


def voxel_downsample_xyzi(points_xyzi: np.ndarray, voxel_leaf: float) -> np.ndarray:
    if voxel_leaf <= 0.0 or points_xyzi.size == 0:
        return points_xyzi

    coords = np.floor(points_xyzi[:, :3] / float(voxel_leaf)).astype(np.int64)
    unique_voxels, inverse = np.unique(coords, axis=0, return_inverse=True)
    downsampled = np.zeros((unique_voxels.shape[0], 4), dtype=np.float64)
    counts = np.bincount(inverse)
    for column in range(4):
        downsampled[:, column] = np.bincount(
            inverse,
            weights=points_xyzi[:, column].astype(np.float64, copy=False),
        ) / counts
    return downsampled.astype(np.float32, copy=False)


def save_tum(path: Path, measurements: list[MeasurementRecord], optimized_values, gtsam_mod) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for measurement in measurements:
            pose = optimized_values.atPose3(gtsam_mod.Symbol("x", measurement.index).key())
            matrix = _pose_matrix(pose)
            quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
            stream.write(
                f"{measurement.odom_time:.9f} "
                f"{matrix[0, 3]:.9f} {matrix[1, 3]:.9f} {matrix[2, 3]:.9f} "
                f"{quat[0]:.9f} {quat[1]:.9f} {quat[2]:.9f} {quat[3]:.9f}\n"
            )


def build_optimized_map(
    measurements: list[MeasurementRecord],
    optimized_values,
    gtsam_mod,
    voxel_leaf: float,
    log_fn: LogFn = None,
) -> np.ndarray:
    merged_parts: list[np.ndarray] = []
    for measurement in measurements:
        points_xyzi = load_xyzi_points(measurement.cloud_path)
        pose = optimized_values.atPose3(gtsam_mod.Symbol("x", measurement.index).key())
        merged_parts.append(_apply_transform(points_xyzi, _pose_matrix(pose)))
    if not merged_parts:
        return np.empty((0, 4), dtype=np.float32)
    merged = np.vstack(merged_parts)
    filtered = voxel_downsample_xyzi(merged, voxel_leaf)
    _log(
        log_fn,
        "[PythonOptimizer] Built optimized map "
        f"input_points={merged.shape[0]}, output_points={filtered.shape[0]}, voxel={voxel_leaf:.3f}",
    )
    return filtered


def save_xyzi_pcd(path: Path, points_xyzi: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    write_pcd(path, _make_xyzi_cloud(points_xyzi))


def save_trajectory_pcd(path: Path, measurements: list[MeasurementRecord], optimized_values, gtsam_mod) -> None:
    points = np.zeros((len(measurements), 4), dtype=np.float32)
    for row, measurement in enumerate(measurements):
        pose = optimized_values.atPose3(gtsam_mod.Symbol("x", measurement.index).key())
        matrix = _pose_matrix(pose)
        points[row, 0:3] = matrix[:3, 3].astype(np.float32, copy=False)
        points[row, 3] = np.float32(measurement.odom_time)
    save_xyzi_pcd(path, points)


def _format_upper_triangular_information(information: np.ndarray) -> str:
    values: list[str] = []
    for row in range(6):
        for col in range(row, 6):
            values.append(f"{information[row, col]:.15e}")
    return " ".join(values)


def write_pose_graph_g2o(
    path: Path,
    optimized_values,
    pose_count: int,
    factor_records: list[object],
    gtsam_mod,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        for node_id in range(pose_count):
            pose = optimized_values.atPose3(gtsam_mod.Symbol("x", node_id).key())
            matrix = _pose_matrix(pose)
            quat = Rotation.from_matrix(matrix[:3, :3]).as_quat()
            stream.write(
                "VERTEX_SE3:QUAT "
                f"{node_id} "
                f"{matrix[0, 3]:.15e} {matrix[1, 3]:.15e} {matrix[2, 3]:.15e} "
                f"{quat[0]:.15e} {quat[1]:.15e} {quat[2]:.15e} {quat[3]:.15e}\n"
            )

        for record in factor_records:
            if not isinstance(record, BetweenFactorRecord):
                continue
            stream.write(
                "EDGE_SE3:QUAT "
                f"{record.node_i} {record.node_j} "
                f"{record.translation_xyz[0]:.15e} {record.translation_xyz[1]:.15e} {record.translation_xyz[2]:.15e} "
                f"{record.quat_xyzw[0]:.15e} {record.quat_xyzw[1]:.15e} {record.quat_xyzw[2]:.15e} {record.quat_xyzw[3]:.15e} "
                f"{_format_upper_triangular_information(record.information)}\n"
            )

        gnss_records = [record for record in factor_records if isinstance(record, GnssPriorRecord)]
        if gnss_records:
            stream.write("# GNSS prior factors serialized by MS-Mapping\n")
            for record in gnss_records:
                if record.subtype == "XYZ":
                    stream.write(
                        "# GNSS_PRIOR XYZ "
                        f"{record.node_id} "
                        f"{record.measurement[0]:.15e} {record.measurement[1]:.15e} {record.measurement[2]:.15e} "
                        f"{record.sigmas.shape[0]} "
                        + " ".join(f"{value:.15e}" for value in record.sigmas)
                        + f" {record.robust_type} {record.robust_param:.15e}\n"
                    )
                elif record.subtype == "POSE":
                    stream.write(
                        "# GNSS_PRIOR POSE "
                        f"{record.node_id} "
                        + " ".join(f"{value:.15e}" for value in record.measurement)
                        + f" {record.sigmas.shape[0]} "
                        + " ".join(f"{value:.15e}" for value in record.sigmas)
                        + f" {record.robust_type} {record.robust_param:.15e}\n"
                    )
                elif record.subtype == "XY":
                    stream.write(
                        "# GNSS_PRIOR XY "
                        f"{record.node_id} "
                        f"{record.measurement[0]:.15e} {record.measurement[1]:.15e} "
                        f"{record.sigmas.shape[0]} "
                        + " ".join(f"{value:.15e}" for value in record.sigmas)
                        + f" {record.robust_type} {record.robust_param:.15e}\n"
                    )

        selected_prior = None
        for record in factor_records:
            if isinstance(record, PosePriorRecord) and record.node_id == 0:
                selected_prior = record
                break
        if selected_prior is not None:
            stream.write("# Anchor vertex for pose prior\n")
            stream.write(f"VERTEX_SE3:QUAT {ANCHOR_VERTEX_ID} 0 0 0 0 0 0 1\n")
            stream.write(
                "EDGE_SE3:QUAT "
                f"{ANCHOR_VERTEX_ID} {selected_prior.node_id} "
                f"{selected_prior.translation_xyz[0]:.15e} {selected_prior.translation_xyz[1]:.15e} {selected_prior.translation_xyz[2]:.15e} "
                f"{selected_prior.quat_xyzw[0]:.15e} {selected_prior.quat_xyzw[1]:.15e} {selected_prior.quat_xyzw[2]:.15e} {selected_prior.quat_xyzw[3]:.15e} "
                f"{_format_upper_triangular_information(selected_prior.information)}\n"
            )


def generate_pose_graph_png(project_root: Path, g2o_path: Path, output_path: Path, log_fn: LogFn = None) -> None:
    candidates = [
        project_root / "gui" / "visualize_pose_graph.py",
        project_root / "scripts" / "visualize_pose_graph.py",
        project_root
        / "backend"
        / "catkin_ws"
        / "src"
        / "manual_loop_closure_backend"
        / "scripts"
        / "visualize_pose_graph.py",
    ]
    script_path = next((candidate for candidate in candidates if candidate.is_file()), None)
    if script_path is None:
        raise RuntimeError("visualize_pose_graph.py not found in known repository paths.")
    env = dict(os.environ)
    env["MPLBACKEND"] = "Agg"
    result = subprocess.run(
        [
            sys.executable,
            str(script_path),
            "--g2o",
            str(g2o_path),
            "--output",
            str(output_path),
        ],
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or "pose graph png generation failed")
    _log(log_fn, f"[PythonOptimizer] Saved pose graph image: {output_path}")


def save_report_json(
    path: Path,
    *,
    session_root: Path,
    input_g2o: Path,
    input_tum: Path,
    input_keyframe_dir: Path,
    constraints_csv: Path,
    output_dir: Path,
    map_voxel_leaf: float,
    total_constraints: int,
    enabled_constraints: int,
    optimized_pose_count: int,
    factor_count: int,
    map_point_count: int,
) -> None:
    report = {
        "session_root": str(session_root),
        "input_g2o": str(input_g2o),
        "input_tum": str(input_tum),
        "input_keyframe_dir": str(input_keyframe_dir),
        "constraints_csv": str(constraints_csv),
        "output_dir": str(output_dir),
        "map_voxel_leaf": map_voxel_leaf,
        "total_constraints": total_constraints,
        "enabled_constraints": enabled_constraints,
        "optimized_pose_count": optimized_pose_count,
        "factor_count": factor_count,
        "map_point_count": map_point_count,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
# ===== END CHANGE: python optimizer exporters =====
