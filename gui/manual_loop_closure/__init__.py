"""Utilities for the offline manual loop-closure tool."""

from .session_resolver import SessionPaths, SessionResolutionError, resolve_session_paths
from .pose_graph_io import (
    EdgeRecord,
    PoseGraphData,
    PoseGraphValidationError,
    align_pose_graph_to_frame_count,
    load_pose_graph,
    write_filtered_pose_graph,
)
from .trajectory_io import (
    TrajectoryData,
    TrajectoryValidationError,
    load_tum_trajectory,
    matrix_to_quat_xyzw,
    matrix_to_xyz_rpy_deg,
)
from .pcd_io import PcdValidationError, load_xyz_points, list_numbered_pcds
from .registration import (
    OFFICE_DEFAULT_MAX_CORRESPONDENCE_DISTANCE,
    OFFICE_DEFAULT_MAX_ITERATIONS,
    OFFICE_DEFAULT_TARGET_CLOUD_MODE,
    OFFICE_DEFAULT_TARGET_MAP_VOXEL_SIZE,
    OFFICE_DEFAULT_TARGET_MIN_TIME_GAP_SEC,
    OFFICE_DEFAULT_TARGET_NEIGHBORS,
    OFFICE_DEFAULT_TARGET_WINDOW,
    OFFICE_DEFAULT_VARIANCE_R_RAD2,
    OFFICE_DEFAULT_VARIANCE_T,
    OFFICE_DEFAULT_VOXEL_SIZE,
    RegistrationConfig,
    RegistrationPreview,
    RegistrationResult,
    RegistrationWorkspace,
    TARGET_CLOUD_MODE_CHOICES,
    TARGET_CLOUD_MODE_RS_SPATIAL_SUBMAP,
    TARGET_CLOUD_MODE_TEMPORAL_WINDOW,
)
