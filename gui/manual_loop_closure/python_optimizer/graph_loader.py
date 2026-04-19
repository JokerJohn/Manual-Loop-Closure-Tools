from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np
from scipy.spatial.transform import Rotation

LogFn = Optional[Callable[[str], None]]
ANCHOR_VERTEX_ID = 2147483646


# ===== BEGIN CHANGE: python optimizer graph loader =====
@dataclass(frozen=True)
class OfflineRuntimeConfig:
    scan_filter_size: float = 0.0
    map_saved_size: float = 0.2
    grid_map_downsample_size: float = 0.2
    save_result_body_frame: bool = True


@dataclass(frozen=True)
class BetweenFactorRecord:
    node_i: int
    node_j: int
    translation_xyz: np.ndarray
    quat_xyzw: np.ndarray
    information: np.ndarray
    origin: str


@dataclass(frozen=True)
class PosePriorRecord:
    node_id: int
    translation_xyz: np.ndarray
    quat_xyzw: np.ndarray
    information: np.ndarray
    source: str


@dataclass(frozen=True)
class GnssPriorRecord:
    subtype: str
    node_id: int
    measurement: np.ndarray
    sigmas: np.ndarray
    robust_type: str
    robust_param: float


@dataclass
class GraphBuildResult:
    graph: object
    initial_values: object
    factor_records: list[object]
    pose_count: int
    continuous_edge_count: int
    non_continuous_edge_count: int


def _log(log_fn: LogFn, message: str) -> None:
    if log_fn is not None:
        log_fn(message)


def pose_key(gtsam_mod, index: int) -> int:
    return gtsam_mod.Symbol("x", int(index)).key()


def parse_bool_text(text: str) -> Optional[bool]:
    normalized = text.strip().lower()
    if normalized in {"1", "true", "yes"}:
        return True
    if normalized in {"0", "false", "no"}:
        return False
    return None


def load_runtime_config(session_root: Path, log_fn: LogFn = None) -> OfflineRuntimeConfig:
    config = OfflineRuntimeConfig()
    candidates = [
        session_root / "mapping" / "runtime_params.yaml",
        session_root / "runtime_params.yaml",
    ]
    for candidate in candidates:
        if not candidate.is_file():
            continue
        for raw_line in candidate.read_text(encoding="utf-8").splitlines():
            if "saveResultBodyFrame" not in raw_line:
                continue
            colon = raw_line.find(":")
            if colon < 0:
                continue
            parsed = parse_bool_text(raw_line[colon + 1 :])
            if parsed is not None:
                _log(log_fn, f"[PythonOptimizer] saveResultBodyFrame={parsed} from {candidate}")
                return OfflineRuntimeConfig(
                    scan_filter_size=config.scan_filter_size,
                    map_saved_size=config.map_saved_size,
                    grid_map_downsample_size=config.grid_map_downsample_size,
                    save_result_body_frame=parsed,
                )
    _log(
        log_fn,
        "[PythonOptimizer] saveResultBodyFrame missing in runtime_params.yaml, assuming body frame keyframes.",
    )
    return config


def _make_pose3(gtsam_mod, translation_xyz: np.ndarray, quat_xyzw: np.ndarray):
    qx, qy, qz, qw = [float(value) for value in quat_xyzw]
    tx, ty, tz = [float(value) for value in translation_xyz]
    return gtsam_mod.Pose3(
        gtsam_mod.Rot3.Quaternion(qw, qx, qy, qz),
        gtsam_mod.Point3(tx, ty, tz),
    )


def _diag_information_from_sigmas(sigmas: np.ndarray) -> np.ndarray:
    variances = np.square(sigmas)
    information = np.zeros((sigmas.size, sigmas.size), dtype=np.float64)
    for idx, variance in enumerate(variances):
        if variance <= 0.0:
            raise ValueError("Noise sigma must be positive.")
        information[idx, idx] = 1.0 / variance
    return information


def _make_gnss_noise(gtsam_mod, sigmas: np.ndarray, robust_type: str, robust_param: float):
    base = gtsam_mod.noiseModel.Diagonal.Sigmas(np.asarray(sigmas, dtype=np.float64))
    robust_type_upper = robust_type.upper()
    if robust_type_upper == "CAUCHY":
        kernel = gtsam_mod.noiseModel.mEstimator.Cauchy.Create(float(robust_param))
        return gtsam_mod.noiseModel.Robust.Create(kernel, base)
    if robust_type_upper == "HUBER":
        kernel = gtsam_mod.noiseModel.mEstimator.Huber.Create(float(robust_param))
        return gtsam_mod.noiseModel.Robust.Create(kernel, base)
    if robust_type_upper == "TUKEY":
        kernel = gtsam_mod.noiseModel.mEstimator.Tukey.Create(float(robust_param))
        return gtsam_mod.noiseModel.Robust.Create(kernel, base)
    return base


def _make_xy_prior_factor(gtsam_mod, key: int, measurement_xy: np.ndarray, noise):
    measured = np.asarray(measurement_xy, dtype=np.float64)

    def error_func(this, values, jacobians=None):
        pose = values.atPose3(this.keys()[0])
        translation = np.asarray(pose.translation(), dtype=np.float64)
        rotation = np.asarray(pose.rotation().matrix(), dtype=np.float64)
        if jacobians is not None:
            J = np.zeros((2, 6), dtype=np.float64, order="F")
            J[0, 3:6] = rotation[0, :]
            J[1, 3:6] = rotation[1, :]
            jacobians[0] = J
        return np.asarray(
            [
                translation[0] - measured[0],
                translation[1] - measured[1],
            ],
            dtype=np.float64,
        )

    return gtsam_mod.CustomFactor(noise, [int(key)], error_func)


def _parse_information_matrix(tokens: list[str], start_index: int) -> np.ndarray:
    information = np.zeros((6, 6), dtype=np.float64)
    token_index = start_index
    for row in range(6):
        for col in range(row, 6):
            information[row, col] = float(tokens[token_index])
            information[col, row] = information[row, col]
            token_index += 1
    return information


def build_factor_graph(
    *,
    session_root: Path,
    g2o_path: Path,
    gtsam_mod,
    log_fn: LogFn = None,
) -> GraphBuildResult:
    runtime_config = load_runtime_config(session_root, log_fn)
    graph = gtsam_mod.NonlinearFactorGraph()
    initial_values = gtsam_mod.Values()
    factor_records: list[object] = []
    continuous_sigmas = np.asarray([1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3], dtype=np.float64)
    continuous_noise = gtsam_mod.noiseModel.Diagonal.Sigmas(continuous_sigmas)

    id_map: dict[int, int] = {}
    next_id = 0
    existing_edges: set[tuple[int, int]] = set()
    continuous_edge_count = 0
    non_continuous_edge_count = 0
    anchor_vertex_present = False
    anchor_prior_loaded = False
    first_vertex_pose = None
    first_vertex_key = None

    with g2o_path.open("r", encoding="utf-8") as stream:
        for raw_line in stream:
            line = raw_line.strip()
            if not line:
                continue

            tokens = line.split()
            tag = tokens[0]
            cursor = 1
            if tag.startswith("#"):
                if len(tokens) < 2:
                    continue
                tag = tokens[1]
                cursor = 2

            if tag == "VERTEX_SE3:QUAT":
                original_id = int(tokens[cursor])
                if original_id == ANCHOR_VERTEX_ID:
                    anchor_vertex_present = True
                    continue
                translation_xyz = np.asarray(tokens[cursor + 1 : cursor + 4], dtype=np.float64)
                quat_xyzw = np.asarray(tokens[cursor + 4 : cursor + 8], dtype=np.float64)
                key = pose_key(gtsam_mod, next_id)
                pose = _make_pose3(gtsam_mod, translation_xyz, quat_xyzw)
                id_map[original_id] = next_id
                initial_values.insert(key, pose)
                if first_vertex_pose is None:
                    first_vertex_pose = pose
                    first_vertex_key = key
                next_id += 1
                continue

            if tag == "GNSS_PRIOR":
                subtype = tokens[cursor]
                original_id = int(tokens[cursor + 1])
                mapped_id = id_map.get(original_id)
                if mapped_id is None:
                    continue
                key = pose_key(gtsam_mod, mapped_id)
                robust_type = "NONE"
                robust_param = 0.0

                if subtype == "XYZ":
                    px, py, pz = map(float, tokens[cursor + 2 : cursor + 5])
                    sigma_dim = int(tokens[cursor + 5])
                    sigmas = np.asarray(
                        tokens[cursor + 6 : cursor + 6 + sigma_dim],
                        dtype=np.float64,
                    )
                    if len(tokens) > cursor + 6 + sigma_dim:
                        robust_type = tokens[cursor + 6 + sigma_dim]
                        robust_param = float(tokens[cursor + 7 + sigma_dim])
                    noise = _make_gnss_noise(gtsam_mod, sigmas, robust_type, robust_param)
                    graph.add(gtsam_mod.GPSFactor(key, gtsam_mod.Point3(px, py, pz), noise))
                    factor_records.append(
                        GnssPriorRecord(
                            subtype="XYZ",
                            node_id=mapped_id,
                            measurement=np.asarray([px, py, pz], dtype=np.float64),
                            sigmas=sigmas,
                            robust_type=robust_type,
                            robust_param=robust_param,
                        )
                    )
                elif subtype == "POSE":
                    px, py, pz = map(float, tokens[cursor + 2 : cursor + 5])
                    qx, qy, qz, qw = map(float, tokens[cursor + 5 : cursor + 9])
                    sigma_dim = int(tokens[cursor + 9])
                    sigmas = np.asarray(
                        tokens[cursor + 10 : cursor + 10 + sigma_dim],
                        dtype=np.float64,
                    )
                    if len(tokens) > cursor + 10 + sigma_dim:
                        robust_type = tokens[cursor + 10 + sigma_dim]
                        robust_param = float(tokens[cursor + 11 + sigma_dim])
                    noise = _make_gnss_noise(gtsam_mod, sigmas, robust_type, robust_param)
                    pose = _make_pose3(
                        gtsam_mod,
                        np.asarray([px, py, pz], dtype=np.float64),
                        np.asarray([qx, qy, qz, qw], dtype=np.float64),
                    )
                    graph.add(gtsam_mod.PriorFactorPose3(key, pose, noise))
                    factor_records.append(
                        PosePriorRecord(
                            node_id=mapped_id,
                            translation_xyz=np.asarray([px, py, pz], dtype=np.float64),
                            quat_xyzw=np.asarray([qx, qy, qz, qw], dtype=np.float64),
                            information=_diag_information_from_sigmas(sigmas),
                            source="gnss_pose",
                        )
                    )
                    factor_records.append(
                        GnssPriorRecord(
                            subtype="POSE",
                            node_id=mapped_id,
                            measurement=np.asarray([px, py, pz, qx, qy, qz, qw], dtype=np.float64),
                            sigmas=sigmas,
                            robust_type=robust_type,
                            robust_param=robust_param,
                        )
                    )
                elif subtype == "XY":
                    px, py = map(float, tokens[cursor + 2 : cursor + 4])
                    sigma_dim = int(tokens[cursor + 4])
                    sigmas = np.asarray(
                        tokens[cursor + 5 : cursor + 5 + sigma_dim],
                        dtype=np.float64,
                    )
                    if len(tokens) > cursor + 5 + sigma_dim:
                        robust_type = tokens[cursor + 5 + sigma_dim]
                        robust_param = float(tokens[cursor + 6 + sigma_dim])
                    noise = _make_gnss_noise(gtsam_mod, sigmas, robust_type, robust_param)
                    graph.add(_make_xy_prior_factor(gtsam_mod, key, np.asarray([px, py]), noise))
                    factor_records.append(
                        GnssPriorRecord(
                            subtype="XY",
                            node_id=mapped_id,
                            measurement=np.asarray([px, py], dtype=np.float64),
                            sigmas=sigmas,
                            robust_type=robust_type,
                            robust_param=robust_param,
                        )
                    )
                continue

            if tag != "EDGE_SE3:QUAT":
                continue

            id1 = int(tokens[cursor])
            id2 = int(tokens[cursor + 1])
            translation_xyz = np.asarray(tokens[cursor + 2 : cursor + 5], dtype=np.float64)
            quat_xyzw = np.asarray(tokens[cursor + 5 : cursor + 9], dtype=np.float64)
            information = _parse_information_matrix(tokens, cursor + 9)

            id1_is_anchor = anchor_vertex_present and id1 == ANCHOR_VERTEX_ID
            id2_is_anchor = anchor_vertex_present and id2 == ANCHOR_VERTEX_ID

            if id1_is_anchor or id2_is_anchor:
                pose_original_id = id2 if id1_is_anchor else id1
                mapped_id = id_map.get(pose_original_id)
                if mapped_id is None:
                    continue
                key = pose_key(gtsam_mod, mapped_id)
                relative_pose = _make_pose3(gtsam_mod, translation_xyz, quat_xyzw)
                prior_pose = relative_pose if id1_is_anchor else relative_pose.inverse()
                graph.add(
                    gtsam_mod.PriorFactorPose3(
                        key,
                        prior_pose,
                        gtsam_mod.noiseModel.Gaussian.Information(information),
                    )
                )
                factor_records.append(
                    PosePriorRecord(
                        node_id=mapped_id,
                        translation_xyz=np.asarray(prior_pose.translation(), dtype=np.float64),
                        quat_xyzw=Rotation.from_matrix(
                            np.asarray(prior_pose.rotation().matrix(), dtype=np.float64)
                        ).as_quat(),
                        information=information,
                        source="anchor",
                    )
                )
                anchor_prior_loaded = True
                continue

            mapped1 = id_map.get(id1)
            mapped2 = id_map.get(id2)
            if mapped1 is None or mapped2 is None:
                continue

            edge_key = (id1, id2)
            if edge_key in existing_edges:
                continue
            existing_edges.add(edge_key)

            noise = gtsam_mod.noiseModel.Gaussian.Covariance(np.linalg.inv(information))
            relative_pose = _make_pose3(gtsam_mod, translation_xyz, quat_xyzw)
            graph.add(
                gtsam_mod.BetweenFactorPose3(
                    pose_key(gtsam_mod, mapped1),
                    pose_key(gtsam_mod, mapped2),
                    relative_pose,
                    noise,
                )
            )
            factor_records.append(
                BetweenFactorRecord(
                    node_i=mapped1,
                    node_j=mapped2,
                    translation_xyz=translation_xyz,
                    quat_xyzw=quat_xyzw,
                    information=information,
                    origin="g2o",
                )
            )
            if mapped1 + 1 == mapped2:
                continuous_edge_count += 1
            else:
                non_continuous_edge_count += 1

    if not anchor_prior_loaded and first_vertex_pose is not None and first_vertex_key is not None:
        graph.add(gtsam_mod.PriorFactorPose3(first_vertex_key, first_vertex_pose, continuous_noise))
        factor_records.append(
            PosePriorRecord(
                node_id=0,
                translation_xyz=np.asarray(first_vertex_pose.translation(), dtype=np.float64),
                quat_xyzw=Rotation.from_matrix(
                    np.asarray(first_vertex_pose.rotation().matrix(), dtype=np.float64)
                ).as_quat(),
                information=_diag_information_from_sigmas(continuous_sigmas),
                source="fallback_first_pose",
            )
        )

    _log(
        log_fn,
        "[PythonOptimizer] Loaded graph "
        f"poses={next_id}, continuous_edges={continuous_edge_count}, "
        f"non_continuous_edges={non_continuous_edge_count}, "
        f"saveResultBodyFrame={runtime_config.save_result_body_frame}",
    )
    return GraphBuildResult(
        graph=graph,
        initial_values=initial_values,
        factor_records=factor_records,
        pose_count=next_id,
        continuous_edge_count=continuous_edge_count,
        non_continuous_edge_count=non_continuous_edge_count,
    )
# ===== END CHANGE: python optimizer graph loader =====
