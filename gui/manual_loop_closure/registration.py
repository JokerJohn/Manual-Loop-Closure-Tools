from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from .pcd_io import load_xyz_points
from .trajectory_io import TrajectoryData

TARGET_CLOUD_MODE_TEMPORAL_WINDOW = "temporal_window"
TARGET_CLOUD_MODE_RS_SPATIAL_SUBMAP = "rs_spatial_submap"
TARGET_CLOUD_MODE_CHOICES = (
    TARGET_CLOUD_MODE_TEMPORAL_WINDOW,
    TARGET_CLOUD_MODE_RS_SPATIAL_SUBMAP,
)

OFFICE_DEFAULT_TARGET_NEIGHBORS = 100
OFFICE_DEFAULT_TARGET_MIN_TIME_GAP_SEC = 30.0
OFFICE_DEFAULT_TARGET_MAP_VOXEL_SIZE = 0.1
OFFICE_DEFAULT_TARGET_WINDOW = OFFICE_DEFAULT_TARGET_NEIGHBORS
OFFICE_DEFAULT_TARGET_CLOUD_MODE = TARGET_CLOUD_MODE_TEMPORAL_WINDOW
OFFICE_DEFAULT_VOXEL_SIZE = 0.0
OFFICE_DEFAULT_MAX_CORRESPONDENCE_DISTANCE = 2.0
OFFICE_DEFAULT_MAX_ITERATIONS = 30
# loop noise: 5 cm and 0.3 deg per axis (variances); the former 0.1 rad^2 (18 deg) left
# the rotation drift almost uncorrected
OFFICE_DEFAULT_VARIANCE_T = (0.0025, 0.0025, 0.0025)
OFFICE_DEFAULT_VARIANCE_R_RAD2 = (2.7416e-05, 2.7416e-05, 2.7416e-05)


@dataclass(frozen=True)
class RegistrationConfig:
    target_cloud_mode: str = OFFICE_DEFAULT_TARGET_CLOUD_MODE
    target_neighbors: int = OFFICE_DEFAULT_TARGET_NEIGHBORS
    min_time_gap_sec: float = OFFICE_DEFAULT_TARGET_MIN_TIME_GAP_SEC
    target_map_voxel_size: float = OFFICE_DEFAULT_TARGET_MAP_VOXEL_SIZE
    voxel_size: float = OFFICE_DEFAULT_VOXEL_SIZE
    max_correspondence_distance: float = OFFICE_DEFAULT_MAX_CORRESPONDENCE_DISTANCE
    max_iterations: int = OFFICE_DEFAULT_MAX_ITERATIONS


@dataclass(frozen=True)
class RegistrationPreview:
    source_id: int
    target_id: int
    target_cloud_mode: str
    target_neighbors: int
    min_time_gap_sec: float
    target_map_voxel_size: float
    target_frame_indices: tuple[int, ...]
    time_gap_filter_enabled: bool
    time_gap_filter_applied: bool
    target_points_world: np.ndarray
    source_points_local: np.ndarray
    source_points_world_initial: np.ndarray
    source_points_world_adjusted: np.ndarray
    transform_world_source_initial: np.ndarray
    transform_world_source_adjusted: np.ndarray
    transform_world_target: np.ndarray

    @property
    def target_frame_count(self) -> int:
        return len(self.target_frame_indices)

    @property
    def target_point_count(self) -> int:
        return int(self.target_points_world.shape[0])

    @property
    def target_frame_range(self) -> tuple[int, int] | None:
        if not self.target_frame_indices:
            return None
        return int(self.target_frame_indices[0]), int(self.target_frame_indices[-1])

    @property
    def target_window_clipped(self) -> bool:
        if self.target_cloud_mode != TARGET_CLOUD_MODE_TEMPORAL_WINDOW:
            return False
        expected_count = max(2 * int(self.target_neighbors) + 1, 1)
        return self.target_frame_count < expected_count


@dataclass(frozen=True)
class RegistrationResult:
    preview: RegistrationPreview
    transform_world_source_final: np.ndarray
    transform_target_source_final: np.ndarray
    source_points_world_final: np.ndarray
    fitness: float
    inlier_rmse: float


def build_delta_transform(
    x: float,
    y: float,
    z: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = Rotation.from_euler(
        "xyz",
        [roll_deg, pitch_deg, yaw_deg],
        degrees=True,
    ).as_matrix()
    transform[:3, 3] = np.asarray([x, y, z], dtype=np.float64)
    return transform


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points.copy()
    rotated = points @ transform[:3, :3].T
    return rotated + transform[:3, 3]


def numpy_to_open3d(points: np.ndarray) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    if points.size:
        cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64, copy=False))
    return cloud


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.size == 0 or voxel_size <= 0.0:
        return points.copy()
    cloud = numpy_to_open3d(points)
    cloud = cloud.voxel_down_sample(voxel_size)
    return np.asarray(cloud.points, dtype=np.float64)


class RegistrationWorkspace:
    def __init__(self, keyframe_dir: Path, trajectory: TrajectoryData) -> None:
        self._keyframe_dir = keyframe_dir
        self._trajectory = trajectory
        self._local_cache: Dict[int, np.ndarray] = {}
        self._target_cache: Dict[
            Tuple[str, int, int, int, float, float],
            tuple[tuple[int, ...], bool, np.ndarray],
        ] = {}

    @property
    def trajectory(self) -> TrajectoryData:
        return self._trajectory

    def load_local_points(self, index: int) -> np.ndarray:
        if index not in self._local_cache:
            path = self._keyframe_dir / f"{index}.pcd"
            self._local_cache[index] = load_xyz_points(path)
        return self._local_cache[index]

    def _target_cache_key(
        self,
        *,
        target_cloud_mode: str,
        source_id: int,
        target_id: int,
        target_neighbors: int,
        min_time_gap_sec: float,
        target_map_voxel_size: float,
    ) -> tuple[str, int, int, int, float, float]:
        return (
            str(target_cloud_mode),
            int(source_id),
            int(target_id),
            int(target_neighbors),
            round(float(min_time_gap_sec), 6),
            round(float(target_map_voxel_size), 6),
        )

    def select_temporal_window_frame_indices(
        self,
        *,
        target_id: int,
        target_window_radius: int,
    ) -> tuple[tuple[int, ...], bool]:
        window_radius = max(int(target_window_radius), 0)
        start_index = max(int(target_id) - window_radius, 0)
        end_index = min(int(target_id) + window_radius, self._trajectory.size - 1)
        selected = tuple(range(start_index, end_index + 1))
        if not selected:
            raise RuntimeError("Temporal target window does not contain any keyframes.")
        return selected, False

    def select_rs_spatial_frame_indices(
        self,
        *,
        source_id: int,
        target_id: int,
        target_neighbors: int,
        min_time_gap_sec: float,
    ) -> tuple[tuple[int, ...], bool]:
        target_xy = self._trajectory.positions_xyz[target_id, :2]
        source_timestamp = float(self._trajectory.timestamps[source_id])
        candidates: list[tuple[float, int]] = []
        filtered_due_to_time_gap = 0

        for index in range(self._trajectory.size):
            time_difference = abs(float(self._trajectory.timestamps[index]) - source_timestamp)
            if min_time_gap_sec > 0.0 and time_difference < min_time_gap_sec:
                filtered_due_to_time_gap += 1
                continue

            pose_xy = self._trajectory.positions_xyz[index, :2]
            distance = float(np.linalg.norm(pose_xy - target_xy))
            candidates.append((distance, index))

        candidates.sort(key=lambda item: (item[0], item[1]))
        selected = tuple(index for _, index in candidates[: max(int(target_neighbors), 1)])
        if not selected:
            raise RuntimeError(
                "No target frames remain after applying the RS loop-closure time-gap filter."
            )
        return selected, filtered_due_to_time_gap > 0

    def select_target_frame_indices(
        self,
        *,
        target_cloud_mode: str,
        source_id: int,
        target_id: int,
        target_neighbors: int,
        min_time_gap_sec: float,
    ) -> tuple[tuple[int, ...], bool]:
        if target_cloud_mode == TARGET_CLOUD_MODE_TEMPORAL_WINDOW:
            return self.select_temporal_window_frame_indices(
                target_id=target_id,
                target_window_radius=target_neighbors,
            )
        if target_cloud_mode == TARGET_CLOUD_MODE_RS_SPATIAL_SUBMAP:
            return self.select_rs_spatial_frame_indices(
                source_id=source_id,
                target_id=target_id,
                target_neighbors=target_neighbors,
                min_time_gap_sec=min_time_gap_sec,
            )
        raise RuntimeError(f"Unsupported target cloud mode: {target_cloud_mode}")

    def build_target_submap(
        self,
        *,
        target_cloud_mode: str,
        source_id: int,
        target_id: int,
        target_neighbors: int,
        min_time_gap_sec: float,
        target_map_voxel_size: float,
    ) -> tuple[tuple[int, ...], bool, np.ndarray]:
        key = self._target_cache_key(
            target_cloud_mode=target_cloud_mode,
            source_id=source_id,
            target_id=target_id,
            target_neighbors=target_neighbors,
            min_time_gap_sec=min_time_gap_sec,
            target_map_voxel_size=target_map_voxel_size,
        )
        if key in self._target_cache:
            return self._target_cache[key]

        selected_indices, time_gap_filter_applied = self.select_target_frame_indices(
            target_cloud_mode=target_cloud_mode,
            source_id=source_id,
            target_id=target_id,
            target_neighbors=target_neighbors,
            min_time_gap_sec=min_time_gap_sec,
        )

        merged_chunks = []
        for index in selected_indices:
            local_points = self.load_local_points(index)
            world_points = transform_points(
                local_points,
                self._trajectory.transforms_world_sensor[index],
            )
            merged_chunks.append(world_points)

        merged = (
            np.concatenate(merged_chunks, axis=0)
            if merged_chunks
            else np.empty((0, 3), dtype=np.float64)
        )
        merged = voxel_downsample(merged, target_map_voxel_size)
        result = (selected_indices, time_gap_filter_applied, merged)
        self._target_cache[key] = result
        return result

    def build_preview(
        self,
        *,
        source_id: int,
        target_id: int,
        delta_transform_local: np.ndarray,
        target_cloud_mode: str,
        target_neighbors: int,
        min_time_gap_sec: float,
        target_map_voxel_size: float,
    ) -> RegistrationPreview:
        source_local = self.load_local_points(source_id)
        target_frame_indices, time_gap_filter_applied, target_points_world = self.build_target_submap(
            target_cloud_mode=target_cloud_mode,
            source_id=source_id,
            target_id=target_id,
            target_neighbors=target_neighbors,
            min_time_gap_sec=min_time_gap_sec,
            target_map_voxel_size=target_map_voxel_size,
        )
        transform_world_source_initial = self._trajectory.transforms_world_sensor[source_id]
        # Apply the manual delta in the current source local frame, then move the
        # adjusted cloud into map coordinates.
        transform_world_source_adjusted = transform_world_source_initial @ delta_transform_local
        transform_world_target = self._trajectory.transforms_world_sensor[target_id]

        return RegistrationPreview(
            source_id=source_id,
            target_id=target_id,
            target_cloud_mode=str(target_cloud_mode),
            target_neighbors=int(target_neighbors),
            min_time_gap_sec=float(min_time_gap_sec),
            target_map_voxel_size=float(target_map_voxel_size),
            target_frame_indices=target_frame_indices,
            time_gap_filter_enabled=(
                target_cloud_mode == TARGET_CLOUD_MODE_RS_SPATIAL_SUBMAP
                and min_time_gap_sec > 0.0
            ),
            time_gap_filter_applied=time_gap_filter_applied,
            target_points_world=target_points_world,
            source_points_local=source_local,
            source_points_world_initial=transform_points(
                source_local,
                transform_world_source_initial,
            ),
            source_points_world_adjusted=transform_points(
                source_local,
                transform_world_source_adjusted,
            ),
            transform_world_source_initial=transform_world_source_initial,
            transform_world_source_adjusted=transform_world_source_adjusted,
            transform_world_target=transform_world_target,
        )

    def run_gicp(
        self,
        *,
        source_id: int,
        target_id: int,
        delta_transform_local: np.ndarray,
        config: RegistrationConfig,
    ) -> RegistrationResult:
        preview = self.build_preview(
            source_id=source_id,
            target_id=target_id,
            delta_transform_local=delta_transform_local,
            target_cloud_mode=config.target_cloud_mode,
            target_neighbors=config.target_neighbors,
            min_time_gap_sec=config.min_time_gap_sec,
            target_map_voxel_size=config.target_map_voxel_size,
        )

        source_down = voxel_downsample(preview.source_points_local, config.voxel_size)
        target_down = voxel_downsample(preview.target_points_world, config.voxel_size)
        if source_down.size == 0 or target_down.size == 0:
            raise RuntimeError("Source or target point cloud is empty after downsampling.")

        source_cloud = numpy_to_open3d(source_down)
        target_cloud = numpy_to_open3d(target_down)

        result = o3d.pipelines.registration.registration_generalized_icp(
            source_cloud,
            target_cloud,
            config.max_correspondence_distance,
            preview.transform_world_source_adjusted,
            o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=int(config.max_iterations),
            ),
        )

        transform_world_source_final = np.asarray(result.transformation, dtype=np.float64)
        transform_target_source_final = (
            np.linalg.inv(preview.transform_world_target) @ transform_world_source_final
        )

        return RegistrationResult(
            preview=preview,
            transform_world_source_final=transform_world_source_final,
            transform_target_source_final=transform_target_source_final,
            source_points_world_final=transform_points(
                preview.source_points_local,
                transform_world_source_final,
            ),
            fitness=float(result.fitness),
            inlier_rmse=float(result.inlier_rmse),
        )
