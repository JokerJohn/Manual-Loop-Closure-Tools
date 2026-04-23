#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import csv
import importlib.util
import json
import math
import os
import subprocess
import shutil
import sys
import time
import traceback
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional


def _python_supports_manual_loop_dependencies(python_executable: Path) -> bool:
    try:
        result = subprocess.run(
            [
                str(python_executable),
                "-c",
                "import open3d, PyQt5, numpy, scipy",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except OSError:
        return False
    return result.returncode == 0


def _candidate_python_executables() -> list[Path]:
    current = Path(sys.executable).resolve()
    candidates: list[Path] = []

    def append_candidate(path: Path) -> None:
        resolved = path.expanduser().resolve()
        if resolved == current or not resolved.is_file():
            return
        if resolved not in candidates:
            candidates.append(resolved)

    conda_prefix = os.environ.get("CONDA_PREFIX")
    if conda_prefix:
        append_candidate(Path(conda_prefix) / "bin" / "python3")
        append_candidate(Path(conda_prefix) / "bin" / "python")

    home = Path.home()
    for base_dir in (home / "anaconda3", home / "miniconda3"):
        append_candidate(base_dir / "bin" / "python3")
        append_candidate(base_dir / "bin" / "python")

    repo_venv = Path(__file__).resolve().parent.parent / ".venv"
    append_candidate(repo_venv / "bin" / "python3")
    append_candidate(repo_venv / "bin" / "python")

    return candidates


def _bootstrap_python_environment() -> None:
    if importlib.util.find_spec("open3d") is not None:
        return
    if os.environ.get("MS_MANUAL_LOOP_REEXEC") == "1":
        return

    for candidate in _candidate_python_executables():
        if not _python_supports_manual_loop_dependencies(candidate):
            continue
        os.environ["MS_MANUAL_LOOP_REEXEC"] = "1"
        print(
            f"[manual_loop_closure_tool] Re-launching with {candidate} because "
            f"{sys.executable} cannot import open3d.",
            file=sys.stderr,
        )
        os.execv(str(candidate), [str(candidate), str(Path(__file__).resolve()), *sys.argv[1:]])


_bootstrap_python_environment()

import numpy as np
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as FigureCanvas,
    NavigationToolbar2QT as NavigationToolbar,
)
from matplotlib.figure import Figure
from PyQt5 import QtCore, QtGui, QtWidgets

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

try:
    from manual_loop_closure import (  # noqa: E402
        EdgeRecord,
        OFFICE_DEFAULT_MAX_CORRESPONDENCE_DISTANCE,
        OFFICE_DEFAULT_MAX_ITERATIONS,
        OFFICE_DEFAULT_TARGET_CLOUD_MODE,
        OFFICE_DEFAULT_TARGET_MAP_VOXEL_SIZE,
        OFFICE_DEFAULT_TARGET_MIN_TIME_GAP_SEC,
        OFFICE_DEFAULT_TARGET_NEIGHBORS,
        OFFICE_DEFAULT_VARIANCE_R_RAD2,
        OFFICE_DEFAULT_VARIANCE_T,
        OFFICE_DEFAULT_VOXEL_SIZE,
        PoseGraphData,
        PoseGraphValidationError,
        RegistrationConfig,
        RegistrationPreview,
        RegistrationResult,
        RegistrationWorkspace,
        SessionPaths,
        SessionResolutionError,
        TARGET_CLOUD_MODE_RS_SPATIAL_SUBMAP,
        TARGET_CLOUD_MODE_TEMPORAL_WINDOW,
        TrajectoryData,
        TrajectoryValidationError,
        align_pose_graph_to_frame_count,
        load_pose_graph,
        load_tum_trajectory,
        list_numbered_pcds,
        matrix_to_quat_xyzw,
        matrix_to_xyz_rpy_deg,
        resolve_session_paths,
        write_filtered_pose_graph,
    )
    from manual_loop_closure.open3d_viewer import (  # noqa: E402
        INTERACTION_MODE_CAMERA,
        INTERACTION_MODE_EDIT_SOURCE,
        EDIT_OPERATION_ROTATE,
        EDIT_OPERATION_TRANSLATE,
        LOCK_MODE_XY_YAW,
        LOCK_MODE_Z_ONLY,
        LOCK_MODE_XZ_PITCH,
        LOCK_MODE_YZ_ROLL,
        ManualAlignUpdate,
        EmbeddedOpen3DWidget,
        PreviewScene,
        VIEW_PRESET_SIDE_X,
        VIEW_PRESET_SIDE_Y,
        VIEW_PRESET_TOP,
    )
    # ===== BEGIN CHANGE: optimizer backend imports =====
    from manual_loop_closure.optimizer_backend import (  # noqa: E402
        BACKEND_PREFERENCE_CPP,
        BACKEND_PREFERENCE_PYTHON,
        OptimizerRunOptions,
        resolve_python_optimizer_backend,
    )
    # ===== END CHANGE: optimizer backend imports =====
    from manual_loop_closure.pcd_io import (  # noqa: E402
        PcdValidationError,
        load_xyz_points,
        validate_keyframe_numbering,
    )
    from manual_loop_closure.registration import (  # noqa: E402
        build_delta_transform,
        transform_points,
    )
    from manual_loop_closure.python_optimizer.exporters import (  # noqa: E402
        build_map_and_trajectory_from_tum,
        update_report_map_fields,
    )
except ModuleNotFoundError as exc:
    if exc.name != "open3d":
        raise
    raise SystemExit(
        "Failed to import Python package 'open3d'. The tool already tried to re-launch "
        f"with a compatible Python, but none worked. Current interpreter: {sys.executable}\n"
        "Install the tested dependencies first, then try again. Example:\n"
        "  conda env create -f environment.yml\n"
        "  conda activate manual-loop-closure\n"
        "  python launch_gui.py --session-root /path/to/session"
    ) from exc


@dataclass(frozen=True)
class SelectedEdgeRef:
    edge_kind: str
    edge_uid: int


@dataclass
class ManualConstraint:
    manual_uid: int
    enabled: bool
    source_id: int
    target_id: int
    target_cloud_mode: str
    target_neighbors: int
    min_time_gap_sec: float
    target_map_voxel_size: float
    transform_world_source_final: np.ndarray
    transform_target_source_final: np.ndarray
    source_points_world_final: np.ndarray
    fitness: float
    inlier_rmse: float
    variance_t_m2: tuple[float, float, float]
    variance_r_rad2: tuple[float, float, float]
    replaces_edge_uid: Optional[int] = None
    accepted_rev: int = 0
    applied_rev: Optional[int] = None
    note: str = ""

    def csv_row(self) -> list[str]:
        translation = self.transform_target_source_final[:3, 3]
        quat_xyzw = matrix_to_quat_xyzw(self.transform_target_source_final)
        sigma_t = [math.sqrt(max(value, 0.0)) for value in self.variance_t_m2]
        sigma_r = [math.sqrt(max(value, 0.0)) for value in self.variance_r_rad2]
        return [
            "1" if self.enabled else "0",
            str(self.source_id),
            str(self.target_id),
            f"{translation[0]:.12f}",
            f"{translation[1]:.12f}",
            f"{translation[2]:.12f}",
            f"{quat_xyzw[0]:.12f}",
            f"{quat_xyzw[1]:.12f}",
            f"{quat_xyzw[2]:.12f}",
            f"{quat_xyzw[3]:.12f}",
            f"{sigma_t[0]:.6f}",
            f"{sigma_t[1]:.6f}",
            f"{sigma_t[2]:.6f}",
            f"{sigma_r[0]:.6f}",
            f"{sigma_r[1]:.6f}",
            f"{sigma_r[2]:.6f}",
        ]

    def as_edge_record(self) -> EdgeRecord:
        return EdgeRecord(
            edge_uid=self.manual_uid,
            source_id=self.source_id,
            target_id=self.target_id,
            edge_type="manual_added",
            tag="MANUAL",
            line_index=None,
            raw_line=None,
            enabled=self.enabled,
            deletable=True,
        )


@dataclass
class ExistingLoopChange:
    edge_uid: int
    enabled: bool = True
    accepted_rev: int = 0
    applied_rev: Optional[int] = None
    note: str = ""


@dataclass
class UndoSnapshot:
    pose_graph: Optional[PoseGraphData]
    trajectory: Optional[TrajectoryData]
    constraints: list[ManualConstraint]
    disabled_loop_changes: dict[int, ExistingLoopChange]
    source_id: Optional[int]
    target_id: Optional[int]
    selected_edge_ref: Optional[SelectedEdgeRef]
    candidate_replace_edge_uid: Optional[int]
    working_revision: int
    session_dirty: bool
    last_output_dir: Optional[Path]
    pick_mode: str


class TrajectoryCanvas(FigureCanvas):
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        self._figure = Figure(figsize=(8.0, 6.0))
        self._axes = self._figure.add_subplot(111)
        self._figure.subplots_adjust(left=0.08, right=0.99, bottom=0.09, top=0.95)
        super().__init__(self._figure)
        self.setParent(parent)

        self._positions_xy: Optional[np.ndarray] = None
        self._ghost_positions_xy: Optional[np.ndarray] = None
        self._selected_target_positions_xy: Optional[np.ndarray] = None
        self._pose_graph: Optional[PoseGraphData] = None
        self._constraints: list[ManualConstraint] = []
        self._source_id: Optional[int] = None
        self._target_id: Optional[int] = None
        self._selected_edge_ref: Optional[SelectedEdgeRef] = None
        self._interaction_mode = "nodes"
        self._select_callback = None
        self._hover_callback = None
        self._toolbar: Optional[NavigationToolbar] = None

        self.mpl_connect("button_press_event", self._on_click)
        self.mpl_connect("motion_notify_event", self._on_motion)

    def set_toolbar(self, toolbar: NavigationToolbar) -> None:
        self._toolbar = toolbar

    def set_callbacks(self, select_callback, hover_callback) -> None:
        self._select_callback = select_callback
        self._hover_callback = hover_callback

    def set_interaction_mode(self, mode: str) -> None:
        self._interaction_mode = mode

    def set_plot_data(
        self,
        *,
        positions_xy: Optional[np.ndarray],
        ghost_positions_xy: Optional[np.ndarray],
        selected_target_positions_xy: Optional[np.ndarray],
        pose_graph: Optional[PoseGraphData],
        constraints: list[ManualConstraint],
        source_id: Optional[int],
        target_id: Optional[int],
        selected_edge_ref: Optional[SelectedEdgeRef],
        preserve_view: bool,
    ) -> None:
        self._positions_xy = positions_xy
        self._ghost_positions_xy = ghost_positions_xy
        self._selected_target_positions_xy = selected_target_positions_xy
        self._pose_graph = pose_graph
        self._constraints = constraints
        self._source_id = source_id
        self._target_id = target_id
        self._selected_edge_ref = selected_edge_ref
        self.redraw(preserve_view=preserve_view)

    def fit_view(self) -> None:
        self.redraw(preserve_view=False)

    def redraw(self, *, preserve_view: bool) -> None:
        previous_limits = None
        if preserve_view and self._axes.has_data():
            previous_limits = (self._axes.get_xlim(), self._axes.get_ylim())

        self._axes.clear()
        self._axes.set_title("Pose Graph")
        self._axes.set_xlabel("X [m]")
        self._axes.set_ylabel("Y [m]")
        self._axes.grid(True, linestyle="--", alpha=0.3)
        self._axes.set_aspect("equal", adjustable="box")

        if self._positions_xy is None or self._pose_graph is None:
            self.draw_idle()
            return

        positions = self._positions_xy
        if self._ghost_positions_xy is not None and len(self._ghost_positions_xy) > 1:
            self._axes.plot(
                self._ghost_positions_xy[:, 0],
                self._ghost_positions_xy[:, 1],
                color="#94a3b8",
                linewidth=1.2,
                alpha=0.55,
                linestyle="--",
                label="Ghost trajectory",
            )
        self._axes.scatter(
            positions[:, 0],
            positions[:, 1],
            s=8,
            c="#666666",
            alpha=0.4,
        )

        if len(positions) > 1:
            self._axes.plot(
                positions[:, 0],
                positions[:, 1],
                color="#4c78a8",
                linewidth=1.0,
                alpha=0.75,
                label="Trajectory",
            )

        if (
            self._selected_target_positions_xy is not None
            and len(self._selected_target_positions_xy) > 0
        ):
            selected = self._selected_target_positions_xy
            self._axes.scatter(
                selected[:, 0],
                selected[:, 1],
                s=26,
                c="#f6d32d",
                alpha=0.95,
                edgecolors="black",
                linewidths=0.3,
                label="Selected target frames",
                zorder=4,
            )

        enabled_loops = [edge for edge in self._pose_graph.loop_edges if edge.enabled]
        disabled_loops = [edge for edge in self._pose_graph.loop_edges if not edge.enabled]
        for edge in enabled_loops:
            self._draw_edge(edge, "#d62728", linewidth=1.3, alpha=0.85)
        if enabled_loops:
            self._axes.plot([], [], color="#d62728", linewidth=1.3, label="Existing loop")

        for edge in disabled_loops:
            self._draw_edge(edge, "#8a8a8a", linewidth=1.2, alpha=0.7, linestyle="--")
        if disabled_loops:
            self._axes.plot([], [], color="#8a8a8a", linewidth=1.2, linestyle="--", label="Disabled loop")

        enabled_manual = [constraint for constraint in self._constraints if constraint.enabled]
        disabled_manual = [constraint for constraint in self._constraints if not constraint.enabled]
        for constraint in enabled_manual:
            self._draw_edge(constraint.as_edge_record(), "#2ca02c", linewidth=1.8, alpha=0.95)
        if enabled_manual:
            self._axes.plot([], [], color="#2ca02c", linewidth=1.8, label="Manual loop")

        for constraint in disabled_manual:
            self._draw_edge(constraint.as_edge_record(), "#7f7f7f", linewidth=1.4, alpha=0.7, linestyle="--")
        if disabled_manual:
            self._axes.plot([], [], color="#7f7f7f", linewidth=1.4, linestyle="--", label="Disabled manual")

        if self._source_id is not None and self._target_id is not None:
            self._axes.plot(
                [positions[self._source_id, 0], positions[self._target_id, 0]],
                [positions[self._source_id, 1], positions[self._target_id, 1]],
                color="#bc5090",
                linewidth=2.0,
                alpha=0.95,
                label="Active pair",
            )

        selected_edge = self._edge_from_ref(self._selected_edge_ref)
        if selected_edge is not None:
            highlight_color = "#ffd166"
            if selected_edge.edge_type == "odom":
                highlight_color = "#ff9f1c"
            elif selected_edge.edge_type == "manual_added":
                highlight_color = "#00f5d4"
            self._draw_edge(
                selected_edge,
                highlight_color,
                linewidth=3.2,
                alpha=1.0,
                linestyle="--" if not selected_edge.enabled else "-",
            )

        if self._source_id is not None:
            source = positions[self._source_id]
            self._axes.scatter(
                [source[0]],
                [source[1]],
                c="#ff7f0e",
                s=90,
                marker="*",
                edgecolors="black",
                label="Source",
                zorder=5,
            )
        if self._target_id is not None:
            target = positions[self._target_id]
            self._axes.scatter(
                [target[0]],
                [target[1]],
                c="#17becf",
                s=64,
                marker="o",
                edgecolors="black",
                label="Target",
                zorder=5,
            )

        if previous_limits is not None:
            self._axes.set_xlim(previous_limits[0])
            self._axes.set_ylim(previous_limits[1])
        self.draw_idle()

    def _draw_edge(
        self,
        edge: EdgeRecord,
        color: str,
        *,
        linewidth: float,
        alpha: float,
        linestyle: str = "-",
    ) -> None:
        if self._positions_xy is None:
            return
        positions = self._positions_xy
        self._axes.plot(
            [positions[edge.source_id, 0], positions[edge.target_id, 0]],
            [positions[edge.source_id, 1], positions[edge.target_id, 1]],
            color=color,
            linewidth=linewidth,
            alpha=alpha,
            linestyle=linestyle,
        )

    def _manual_edges(self) -> list[EdgeRecord]:
        return [constraint.as_edge_record() for constraint in self._constraints]

    def _edge_from_ref(self, edge_ref: Optional[SelectedEdgeRef]) -> Optional[EdgeRecord]:
        if edge_ref is None or self._pose_graph is None:
            return None
        if edge_ref.edge_kind == "existing":
            return self._pose_graph.edge_index_by_uid.get(edge_ref.edge_uid)
        for edge in self._manual_edges():
            if edge.edge_uid == edge_ref.edge_uid:
                return edge
        return None

    def _nearest_node(self, event) -> Optional[int]:
        if self._positions_xy is None or event.xdata is None or event.ydata is None:
            return None

        positions = self._positions_xy
        click = np.asarray([event.xdata, event.ydata], dtype=np.float64)
        deltas = positions - click
        distances = np.linalg.norm(deltas, axis=1)
        index = int(np.argmin(distances))

        global_span = max(float(np.ptp(positions[:, 0])), float(np.ptp(positions[:, 1])), 1.0)
        x_limits = self._axes.get_xlim()
        y_limits = self._axes.get_ylim()
        visible_span = max(
            abs(float(x_limits[1] - x_limits[0])),
            abs(float(y_limits[1] - y_limits[0])),
            1.0,
        )
        max_distance = min(0.02 * global_span, max(0.08, 0.02 * visible_span))
        if float(distances[index]) > max_distance:
            return None
        return index

    def _nearest_edge(self, event) -> Optional[SelectedEdgeRef]:
        if self._positions_xy is None or self._pose_graph is None or event.xdata is None or event.ydata is None:
            return None

        point = np.asarray([event.xdata, event.ydata], dtype=np.float64)
        best_ref: Optional[SelectedEdgeRef] = None
        best_distance = float("inf")

        all_edges: list[tuple[SelectedEdgeRef, EdgeRecord]] = [
            (SelectedEdgeRef("existing", edge.edge_uid), edge)
            for edge in self._pose_graph.edge_records
        ]
        all_edges.extend(
            (SelectedEdgeRef("manual_added", edge.edge_uid), edge)
            for edge in self._manual_edges()
        )

        for edge_ref, edge in all_edges:
            segment_start = self._positions_xy[edge.source_id]
            segment_end = self._positions_xy[edge.target_id]
            distance = _point_to_segment_distance(point, segment_start, segment_end)
            if distance < best_distance:
                best_distance = distance
                best_ref = edge_ref

        x_limits = self._axes.get_xlim()
        y_limits = self._axes.get_ylim()
        visible_span = max(
            abs(float(x_limits[1] - x_limits[0])),
            abs(float(y_limits[1] - y_limits[0])),
            1.0,
        )
        if best_distance > max(0.12, 0.015 * visible_span):
            return None
        return best_ref

    def _on_click(self, event) -> None:
        if self._toolbar is not None and self._toolbar.mode:
            return
        if event.inaxes != self._axes or self._select_callback is None:
            return

        if self._interaction_mode == "edges":
            edge_ref = self._nearest_edge(event)
            if edge_ref is not None:
                self._select_callback("edge", edge_ref)
            return

        node_id = self._nearest_node(event)
        if node_id is not None:
            self._select_callback("node", node_id)

    def _on_motion(self, event) -> None:
        if self._hover_callback is None or event.inaxes != self._axes:
            return

        if self._interaction_mode == "edges":
            self._hover_callback("edge", self._nearest_edge(event))
        else:
            self._hover_callback("node", self._nearest_node(event))


def _point_to_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    segment = end - start
    denom = float(np.dot(segment, segment))
    if denom <= 1e-12:
        return float(np.linalg.norm(point - start))
    projection = float(np.dot(point - start, segment) / denom)
    projection = max(0.0, min(1.0, projection))
    nearest = start + projection * segment
    return float(np.linalg.norm(point - nearest))


class ManualLoopClosureWindow(QtWidgets.QMainWindow):
    def __init__(
        self,
        *,
        initial_session_root: Optional[Path] = None,
        initial_g2o_path: Optional[Path] = None,
    ) -> None:
        super().__init__()
        self.setWindowTitle("Manual Loop Closure Tool")
        self.setMinimumSize(760, 560)
        self._settings = QtCore.QSettings("JokerJohn", "ManualLoopClosureTools")

        self.session_paths: Optional[SessionPaths] = None
        self.original_pose_graph: Optional[PoseGraphData] = None
        self.original_trajectory: Optional[TrajectoryData] = None
        self.pose_graph: Optional[PoseGraphData] = None
        self.trajectory: Optional[TrajectoryData] = None
        self.workspace: Optional[RegistrationWorkspace] = None
        self.keyframe_paths: list[Path] = []

        self.source_id: Optional[int] = None
        self.target_id: Optional[int] = None
        self.selected_edge_ref: Optional[SelectedEdgeRef] = None
        self.constraints: list[ManualConstraint] = []
        self.disabled_loop_changes: dict[int, ExistingLoopChange] = {}
        self.current_preview: Optional[RegistrationPreview] = None
        self.last_result: Optional[RegistrationResult] = None
        self.pick_mode = "nodes"
        self._table_updating = False
        self._optimizer_process: Optional[QtCore.QProcess] = None
        self._last_output_dir: Optional[Path] = None
        self._preview_scene_key: Optional[tuple] = None
        self._next_manual_uid = 1
        self._pending_preview_reset_camera = False
        self._working_revision = 0
        self._session_dirty = False
        self._pending_export_after_optimize = False
        self._undo_stack: list[UndoSnapshot] = []
        self._graph_change_rows: list[tuple[str, int]] = []
        self._pre_optimize_snapshot: Optional[UndoSnapshot] = None
        self._graph_change_status_filter = "All Status"
        self._graph_change_type_filter = "All Types"
        self._candidate_replace_edge_uid: Optional[int] = None
        self._active_optimizer_backend_name = ""
        self._active_optimizer_backend_key = BACKEND_PREFERENCE_PYTHON
        self._optimizer_retry_attempted = False
        self._pending_optimizer_options: Optional[OptimizerRunOptions] = None
        self._selected_optimizer_preference = BACKEND_PREFERENCE_PYTHON
        self._cpp_optimizer_command_cache: Optional[list[str]] = None
        self._show_legacy_backend_controls = False
        self._optimizer_started_at: Optional[float] = None
        self._project_dir: Optional[Path] = None
        self._project_state_path: Optional[Path] = None
        self._project_log_path: Optional[Path] = None
        self._project_ops_path: Optional[Path] = None
        self._project_id: Optional[str] = None
        self._latest_export_dir: Optional[Path] = None
        self._requested_project_dir: Optional[Path] = None

        self._preview_timer = QtCore.QTimer(self)
        self._preview_timer.setSingleShot(True)
        self._preview_timer.setInterval(120)
        self._preview_timer.timeout.connect(self.refresh_preview)

        self._optimizer_heartbeat_timer = QtCore.QTimer(self)
        self._optimizer_heartbeat_timer.setInterval(5000)
        self._optimizer_heartbeat_timer.timeout.connect(self._on_optimizer_heartbeat)

        self._build_ui()
        self._apply_initial_window_geometry()
        self._apply_styles()
        self._configure_plot_toolbar()
        self._configure_button_cursors()

        self._restore_input_history(
            initial_session_root=initial_session_root,
            initial_g2o_path=initial_g2o_path,
        )

        self._update_plot_help_state()
        self._update_trajectory_legend_label()
        self._update_cloud_display_controls()

    def _build_ui(self) -> None:
        central_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(central_widget)
        root_layout = QtWidgets.QVBoxLayout(central_widget)
        root_layout.setContentsMargins(8, 8, 8, 8)
        root_layout.setSpacing(8)

        root_layout.addWidget(self._build_loader_group())

        self.content_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.content_splitter.setChildrenCollapsible(False)
        self.content_splitter.setHandleWidth(10)
        self.plot_panel = self._build_plot_panel()
        self.cloud_panel = self._build_cloud_panel()
        self.control_scroll_area = self._build_control_scroll_area()
        self.plot_panel.setMinimumWidth(500)
        self.cloud_panel.setMinimumWidth(360)
        self.content_splitter.addWidget(self.plot_panel)
        self.content_splitter.addWidget(self.cloud_panel)
        self.content_splitter.addWidget(self.control_scroll_area)
        self.content_splitter.setStretchFactor(0, 6)
        self.content_splitter.setStretchFactor(1, 5)
        self.content_splitter.setStretchFactor(2, 0)
        self.content_splitter.setSizes([560, 460, 320])

        bottom_tabs = QtWidgets.QTabWidget()
        bottom_tabs.addTab(self._build_constraint_tab(), "Graph Changes")
        bottom_tabs.addTab(self._build_log_tab(), "Execution Log")

        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        main_splitter.setChildrenCollapsible(False)
        main_splitter.addWidget(self.content_splitter)
        main_splitter.addWidget(bottom_tabs)
        main_splitter.setStretchFactor(0, 7)
        main_splitter.setStretchFactor(1, 2)
        main_splitter.setSizes([820, 240])
        root_layout.addWidget(main_splitter, stretch=1)

    def _apply_styles(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #f4f7fb;
                color: #1f2937;
                font-size: 12px;
            }
            QGroupBox {
                background: #ffffff;
                border: 1px solid #d9e1ea;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QPlainTextEdit, QTableWidget {
                background: #ffffff;
                border: 1px solid #ccd6e2;
                border-radius: 6px;
                padding: 2px 6px;
                min-height: 24px;
            }
            QPushButton, QToolButton {
                background: #ffffff;
                border: 1px solid #ccd6e2;
                border-radius: 6px;
                padding: 4px 8px;
                min-height: 24px;
            }
            QPushButton:hover, QToolButton:hover {
                background: #eef4ff;
                border-color: #9db6d8;
            }
            QPushButton:pressed, QToolButton:pressed {
                background: #dbeafe;
                border-color: #60a5fa;
                color: #0f172a;
            }
            QPushButton:checked, QToolButton:checked {
                background: #dbeafe;
                border-color: #60a5fa;
                color: #1d4ed8;
                font-weight: 600;
            }
            QPushButton:disabled, QToolButton:disabled {
                color: #98a4b3;
                background: #f8fafc;
            }
            QPushButton[buttonRole="primary"], QToolButton[buttonRole="primary"] {
                background: #eff6ff;
                border-color: #93c5fd;
                color: #1d4ed8;
                font-weight: 600;
            }
            QPushButton[buttonRole="primary"]:hover, QToolButton[buttonRole="primary"]:hover {
                background: #dbeafe;
                border-color: #60a5fa;
            }
            QPushButton[buttonRole="primary"]:pressed, QToolButton[buttonRole="primary"]:pressed {
                background: #bfdbfe;
                border-color: #3b82f6;
            }
            QToolBar {
                background: #ffffff;
                border: 1px solid #d9e1ea;
                border-radius: 12px;
                spacing: 3px;
                padding: 4px;
            }
            QToolBar QToolButton {
                background: #f8fafc;
                border: 1px solid #e2e8f0;
                color: #334155;
                qproperty-iconSize: 17px;
                border-radius: 9px;
                padding: 5px;
                min-width: 30px;
                min-height: 30px;
            }
            QToolBar QToolButton:hover {
                background: #eef4ff;
                border-color: #bfdbfe;
            }
            QToolBar QToolButton:pressed, QToolBar QToolButton:checked {
                background: #dbeafe;
                border-color: #60a5fa;
            }
            QToolBar::separator {
                width: 8px;
                background: transparent;
            }
            QToolBar QToolButton[toolbarRole="quiet"] {
                background: transparent;
                border: 1px solid transparent;
                color: #64748b;
            }
            QFrame#SegmentedControl {
                background: #eef2f7;
                border: 1px solid #d9e1ea;
                border-radius: 10px;
            }
            QToolButton[segmentRole="segmented"] {
                background: transparent;
                border: 1px solid transparent;
                border-radius: 8px;
                padding: 4px 10px;
                min-height: 24px;
                font-weight: 600;
                color: #475569;
            }
            QToolButton[segmentRole="segmented"]:hover {
                background: #e2e8f0;
                border-color: #cbd5e1;
            }
            QToolButton[segmentRole="segmented"]:checked {
                background: #ffffff;
                border-color: #93c5fd;
                color: #1d4ed8;
            }
            QFrame#PrimarySegmentedControl {
                background: #eef6ff;
                border: 1px solid #bfd5f7;
                border-radius: 10px;
            }
            QToolButton[segmentRole="segmentedPrimary"] {
                background: transparent;
                border: 1px solid transparent;
                border-radius: 8px;
                padding: 5px 12px;
                min-height: 26px;
                font-weight: 700;
                color: #31527c;
            }
            QToolButton[segmentRole="segmentedPrimary"]:hover {
                background: #dbeafe;
                border-color: #93c5fd;
            }
            QToolButton[segmentRole="segmentedPrimary"]:checked {
                background: #ffffff;
                border-color: #60a5fa;
                color: #1d4ed8;
            }
            QHeaderView::section {
                background: #edf3f9;
                border: none;
                border-bottom: 1px solid #d9e1ea;
                padding: 4px;
            }
            QLabel#StatusBadge {
                background: #eef2f7;
                border: 1px solid #d6dde6;
                border-radius: 7px;
                padding: 1px 5px;
                font-weight: 600;
                font-size: 10px;
            }
            QLabel#PanelLegend {
                background: #f8fafc;
                border: 1px solid #e2e8f0;
                border-radius: 6px;
                padding: 3px 6px;
                color: #334155;
                font-size: 11px;
            }
            QLabel#SubtleText {
                color: #64748b;
                font-size: 10px;
            }
            QLabel#CompactValue {
                color: #0f172a;
                padding: 1px 0;
            }
            QFrame#SummaryCard {
                background: #f8fafc;
                border: 1px solid #e2e8f0;
                border-radius: 7px;
            }
            QLabel#SummaryCardTitle {
                color: #475569;
                background: #eef2f7;
                border: 1px solid #d6dde6;
                border-radius: 6px;
                padding: 0px 5px;
                font-size: 9px;
                font-weight: 600;
                letter-spacing: 0.1px;
            }
            QLabel#SummaryCardBody {
                color: #0f172a;
                font-size: 10px;
            }
            QFrame#ActionSection {
                background: #f8fafc;
                border: 1px solid #e2e8f0;
                border-radius: 8px;
            }
            QLabel#ActionSectionTitle {
                color: #475569;
                font-size: 11px;
                font-weight: 700;
                letter-spacing: 0.3px;
            }
            QSplitter::handle {
                background: #d9e1ea;
            }
            QTabWidget::pane {
                border: 1px solid #d9e1ea;
                background: #ffffff;
            }
            QTabBar::tab {
                background: #f8fafc;
                border: 1px solid #d9e1ea;
                border-bottom-color: #cfd8e3;
                border-top-left-radius: 8px;
                border-top-right-radius: 8px;
                padding: 4px 10px;
                min-width: 72px;
                color: #475569;
                font-weight: 600;
            }
            QTabBar::tab:selected {
                background: #ffffff;
                color: #1d4ed8;
                border-color: #93c5fd;
                border-bottom-color: #ffffff;
            }
            QTabBar::tab:hover:!selected {
                background: #f1f5f9;
                border-color: #cbd5e1;
            }
            """
        )

    def _configure_button_cursors(self) -> None:
        pointing_hand = QtGui.QCursor(QtCore.Qt.PointingHandCursor)
        for button in self.findChildren(QtWidgets.QPushButton):
            button.setCursor(pointing_hand)
        for button in self.findChildren(QtWidgets.QToolButton):
            button.setCursor(pointing_hand)

    def _configure_plot_toolbar(self) -> None:
        if not hasattr(self, "toolbar") or self.toolbar is None:
            return

        self.toolbar.setIconSize(QtCore.QSize(17, 17))
        self.toolbar.setToolButtonStyle(QtCore.Qt.ToolButtonIconOnly)
        self.toolbar.setMovable(False)
        self.toolbar.setFloatable(False)
        removable_actions = {"Subplots", "Customize"}
        for action in list(self.toolbar.actions()):
            text = action.text().strip()
            if text in removable_actions:
                self.toolbar.removeAction(action)
                continue
            action.triggered.connect(self._on_plot_toolbar_action_triggered)

        for button in self.toolbar.findChildren(QtWidgets.QToolButton):
            button.setAutoRaise(False)
            text = button.defaultAction().text().strip() if button.defaultAction() is not None else ""
            if text in {"Clear selection", "Fit trajectory view"}:
                button.setProperty("toolbarRole", "quiet")
                button.style().unpolish(button)
                button.style().polish(button)

    def _on_plot_toolbar_action_triggered(self, *_args) -> None:
        QtCore.QTimer.singleShot(0, self._update_plot_help_state)

    def _restore_input_history(
        self,
        *,
        initial_session_root: Optional[Path],
        initial_g2o_path: Optional[Path],
    ) -> None:
        session_root = initial_session_root or self._settings_existing_path("browser/last_session_root")
        if session_root is not None:
            self.session_root_edit.setText(str(session_root))

        g2o_path = initial_g2o_path or self._settings_existing_path("browser/last_g2o_path")
        if g2o_path is not None:
            self.g2o_edit.setText(str(g2o_path))

    def _settings_existing_path(self, key: str) -> Optional[Path]:
        raw_value = self._settings.value(key, "", type=str)
        if not raw_value:
            return None
        path = Path(raw_value).expanduser()
        if path.exists():
            return path
        return None

    def _existing_dialog_directory(self, text: str) -> Optional[Path]:
        text = text.strip()
        if not text:
            return None
        path = Path(text).expanduser()
        if path.is_dir():
            return path
        if path.is_file():
            return path.parent
        return None

    def _session_root_dialog_directory(self) -> str:
        for candidate in (
            self._existing_dialog_directory(self.session_root_edit.text()),
            self._settings_existing_path("browser/last_session_root"),
            self._settings_existing_path("browser/last_g2o_path"),
        ):
            if candidate is None:
                continue
            return str(candidate if candidate.is_dir() else candidate.parent)
        return str(Path.home())

    def _g2o_dialog_directory(self) -> str:
        for candidate in (
            self._existing_dialog_directory(self.g2o_edit.text()),
            self._existing_dialog_directory(self.session_root_edit.text()),
            self._settings_existing_path("browser/last_g2o_path"),
            self._settings_existing_path("browser/last_session_root"),
        ):
            if candidate is None:
                continue
            return str(candidate if candidate.is_dir() else candidate.parent)
        return str(Path.home())

    def _project_state_dialog_directory(self) -> str:
        session_root_dir = self._existing_dialog_directory(self.session_root_edit.text())
        if session_root_dir is not None:
            projects_dir = session_root_dir / "manual_loop_projects"
            if projects_dir.is_dir():
                return str(projects_dir)
            return str(session_root_dir)
        for candidate in (
            self._settings_existing_path("browser/last_project_state"),
            self._settings_existing_path("browser/last_session_root"),
            Path.home(),
        ):
            if candidate is None:
                continue
            return str(candidate if candidate.is_dir() else candidate.parent)
        return str(Path.home())

    def _is_g2o_under_session_root(self, session_root: Path, g2o_path: Path) -> bool:
        try:
            g2o_path.resolve().relative_to(session_root.resolve())
            return True
        except ValueError:
            return False

    def _remember_loaded_paths(self, paths: SessionPaths) -> None:
        self._settings.setValue("browser/last_session_root", str(paths.session_root))
        self._settings.setValue("browser/last_g2o_path", str(paths.g2o_path))
        self._settings.sync()

    def _remember_project_state_path(self, project_state_path: Path) -> None:
        self._settings.setValue("browser/last_project_state", str(project_state_path))
        self._settings.sync()

    def _release_plot_toolbar_navigation(self) -> bool:
        if not hasattr(self, "toolbar") or self.toolbar is None or not self.toolbar.mode:
            return False

        mode = str(self.toolbar.mode).lower()
        if "pan" in mode:
            self.toolbar.pan()
        elif "zoom" in mode:
            self.toolbar.zoom()
        else:
            return False
        self._update_plot_help_state()
        return True

    def _update_plot_help_state(self) -> None:
        if not self._is_working_view():
            self.plot_help_label.setText(
                "Original view is read-only. Toggle Ghost to compare against working."
            )
            return

        toolbar_mode = str(self.toolbar.mode).lower() if hasattr(self, "toolbar") and self.toolbar is not None else ""
        if "zoom" in toolbar_mode:
            self.plot_help_label.setText(
                "Zoom active · click Pick Nodes or Pick Edges to resume selection."
            )
            return
        if "pan" in toolbar_mode:
            self.plot_help_label.setText(
                "Pan active · click Pick Nodes or Pick Edges to resume selection."
            )
            return

        if self.pick_mode == "edges":
            self.plot_help_label.setText(
                "Pick edges to inspect, disable, or replace."
            )
            return

        self.plot_help_label.setText(
            "Pick nodes to add closures, or pick edges to inspect/replace."
        )

    def _apply_initial_window_geometry(self) -> None:
        screen = QtWidgets.QApplication.primaryScreen()
        if screen is None:
            self.resize(1400, 900)
            return
        available = screen.availableGeometry()
        self.setMinimumSize(
            min(self.minimumWidth(), max(640, available.width() - 24)),
            min(self.minimumHeight(), max(520, available.height() - 24)),
        )
        target_width = min(available.width(), max(self.minimumWidth(), int(available.width() * 0.96)))
        target_height = min(available.height(), max(self.minimumHeight(), int(available.height() * 0.94)))
        self.resize(min(target_width, available.width()), min(target_height, available.height()))
        frame = self.frameGeometry()
        frame.moveCenter(available.center())
        self.move(frame.topLeft())

    def _build_loader_group(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Session Input")
        layout = QtWidgets.QGridLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)

        self.session_root_edit = QtWidgets.QLineEdit()
        self.g2o_edit = QtWidgets.QLineEdit()
        browse_root_button = QtWidgets.QPushButton("Browse Root")
        browse_root_button.setToolTip("Open the session-root browser from the most recent folder.")
        browse_root_button.clicked.connect(self._browse_session_root)
        browse_g2o_button = QtWidgets.QPushButton("Browse G2O")
        browse_g2o_button.setToolTip("Open the g2o file browser from the most recent folder.")
        browse_g2o_button.clicked.connect(self._browse_g2o)
        open_project_button = QtWidgets.QPushButton("Open Project")
        open_project_button.setToolTip("Open a previous edit project by selecting its project_state.json.")
        open_project_button.clicked.connect(self._browse_project_state)
        load_button = QtWidgets.QPushButton("Load Session")
        load_button.setProperty("buttonRole", "primary")
        load_button.clicked.connect(self.load_session)

        layout.addWidget(QtWidgets.QLabel("Session Root"), 0, 0)
        layout.addWidget(self.session_root_edit, 0, 1, 1, 2)
        layout.addWidget(browse_root_button, 0, 2)
        layout.addWidget(QtWidgets.QLabel("G2O File"), 1, 0)
        layout.addWidget(self.g2o_edit, 1, 1)
        layout.addWidget(browse_g2o_button, 1, 2)
        layout.addWidget(open_project_button, 1, 3)
        layout.addWidget(load_button, 1, 4)
        layout.setColumnStretch(1, 1)

        self.session_info_label = QtWidgets.QLabel("No session loaded.")
        self.session_info_label.setWordWrap(True)
        layout.addWidget(self.session_info_label, 2, 0, 1, 5)
        return group

    def _build_plot_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Trajectory")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(7, 8, 7, 7)
        layout.setSpacing(5)
        self.trajectory_canvas = TrajectoryCanvas(group)
        self.toolbar = NavigationToolbar(self.trajectory_canvas, group)
        self.trajectory_canvas.set_toolbar(self.toolbar)
        self.trajectory_canvas.set_callbacks(self._handle_plot_selection, self._handle_plot_hover)
        style = self.style()
        self.toolbar.addSeparator()
        clear_icon = style.standardIcon(QtWidgets.QStyle.SP_DialogResetButton)
        fit_icon = style.standardIcon(QtWidgets.QStyle.SP_TitleBarMaxButton)
        self.clear_selection_action = self.toolbar.addAction(clear_icon, "Clear selection", self._clear_selection)
        self.clear_selection_action.setToolTip("Clear the current node or edge selection.")
        self.fit_view_action = self.toolbar.addAction(
            fit_icon,
            "Fit trajectory view",
            lambda: self._refresh_plot(preserve_view=False),
        )
        self.fit_view_action.setToolTip("Reset the trajectory view to the full pose graph.")
        layout.addWidget(self.toolbar)

        header_row = QtWidgets.QHBoxLayout()
        header_row.setSpacing(5)
        header_row.addWidget(QtWidgets.QLabel("View"))
        self.trajectory_view_combo = QtWidgets.QComboBox()
        self.trajectory_view_combo.addItems(["Working", "Original"])
        self.trajectory_view_combo.currentIndexChanged.connect(self._on_trajectory_view_changed)
        header_row.addWidget(self.trajectory_view_combo)
        self.show_ghost_check = QtWidgets.QCheckBox("Ghost")
        self.show_ghost_check.setToolTip("Overlay the other trajectory view as a ghost reference.")
        self.show_ghost_check.setChecked(True)
        self.show_ghost_check.stateChanged.connect(self._on_show_ghost_changed)
        header_row.addWidget(self.show_ghost_check)
        self.pick_nodes_button = QtWidgets.QToolButton()
        self.pick_nodes_button.setText("Nodes")
        self.pick_nodes_button.setCheckable(True)
        self.pick_nodes_button.setChecked(True)
        self.pick_nodes_button.setProperty("segmentRole", "segmentedPrimary")
        self.pick_nodes_button.setToolTip("Exit pan/zoom mode and pick a source-target node pair.")
        self.pick_nodes_button.clicked.connect(lambda: self._set_pick_mode("nodes"))

        self.pick_edges_button = QtWidgets.QToolButton()
        self.pick_edges_button.setText("Edges")
        self.pick_edges_button.setCheckable(True)
        self.pick_edges_button.setProperty("segmentRole", "segmentedPrimary")
        self.pick_edges_button.setToolTip("Exit pan/zoom mode and inspect an existing or manual edge.")
        self.pick_edges_button.clicked.connect(lambda: self._set_pick_mode("edges"))

        mode_group = QtWidgets.QButtonGroup(self)
        mode_group.setExclusive(True)
        mode_group.addButton(self.pick_nodes_button)
        mode_group.addButton(self.pick_edges_button)

        segmented_control = QtWidgets.QFrame()
        segmented_control.setObjectName("PrimarySegmentedControl")
        segmented_layout = QtWidgets.QHBoxLayout(segmented_control)
        segmented_layout.setContentsMargins(2, 2, 2, 2)
        segmented_layout.setSpacing(2)
        segmented_layout.addWidget(self.pick_nodes_button)
        segmented_layout.addWidget(self.pick_edges_button)
        header_row.addWidget(segmented_control)
        header_row.addStretch(1)
        self.working_rev_badge = QtWidgets.QLabel("R0")
        self.working_rev_badge.setObjectName("StatusBadge")
        header_row.addWidget(self.working_rev_badge)
        self.session_state_badge = QtWidgets.QLabel("OK")
        self.session_state_badge.setObjectName("StatusBadge")
        header_row.addWidget(self.session_state_badge)
        self.change_summary_badge = QtWidgets.QLabel("M0 D0")
        self.change_summary_badge.setObjectName("StatusBadge")
        header_row.addWidget(self.change_summary_badge)
        self.trajectory_view_info_label = QtWidgets.QLabel("P0 · L0 · edit")
        self.trajectory_view_info_label.setObjectName("SubtleText")
        header_row.addWidget(self.trajectory_view_info_label)
        layout.addLayout(header_row)

        self.plot_help_label = QtWidgets.QLabel("Working edits. Original compares.")
        self.plot_help_label.setObjectName("SubtleText")
        self.plot_help_label.setWordWrap(True)
        layout.addWidget(self.plot_help_label)
        self.trajectory_legend_label = QtWidgets.QLabel()
        self.trajectory_legend_label.setWordWrap(False)
        self.trajectory_legend_label.setTextFormat(QtCore.Qt.RichText)
        self.trajectory_legend_label.setObjectName("PanelLegend")
        layout.addWidget(self.trajectory_legend_label)
        layout.addWidget(self.trajectory_canvas, stretch=1)

        self.hover_label = QtWidgets.QLabel("Hover: none")
        layout.addWidget(self.hover_label)
        return group

    def _build_cloud_panel(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Point Cloud Review")
        layout = QtWidgets.QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        # ===== BEGIN CHANGE: manual align cloud toolbar =====
        self.cloud_view = EmbeddedOpen3DWidget(group)
        self.cloud_view.manual_align_changed.connect(self._on_manual_align_changed)
        self.cloud_view.manual_align_status.connect(self._on_manual_align_status)
        self.cloud_view.camera_preset_changed.connect(self._on_camera_preset_changed)

        controls_row = QtWidgets.QHBoxLayout()
        controls_row.setSpacing(5)
        controls_row.addWidget(QtWidgets.QLabel("Display"))
        self.display_mode_combo = QtWidgets.QComboBox()
        self.display_mode_combo.addItems(["Preview", "Final", "Compare"])
        self.display_mode_combo.currentIndexChanged.connect(self._update_cloud_display_controls)
        self.display_mode_combo.setMaximumWidth(92)
        controls_row.addWidget(self.display_mode_combo)
        controls_row.addWidget(QtWidgets.QLabel("Traj"))
        self.trajectory_point_size_spin = QtWidgets.QSpinBox()
        self.trajectory_point_size_spin.setRange(1, 32)
        self.trajectory_point_size_spin.setValue(8)
        self.trajectory_point_size_spin.setMaximumWidth(56)
        self.trajectory_point_size_spin.valueChanged.connect(self._update_cloud_display_controls)
        controls_row.addWidget(self.trajectory_point_size_spin)
        self.target_point_size_spin = self._make_double_spin(
            step=0.5,
            minimum=0.5,
            maximum=20.0,
            value=1.0,
        )
        self.target_point_size_spin.valueChanged.connect(self._update_cloud_display_controls)
        self.source_point_size_spin = self._make_double_spin(
            step=0.5,
            minimum=0.5,
            maximum=20.0,
            value=3.0,
        )
        self.source_point_size_spin.valueChanged.connect(self._update_cloud_display_controls)
        self.show_world_axis_check = QtWidgets.QCheckBox("World Axis")
        self.show_world_axis_check.stateChanged.connect(self._update_cloud_display_controls)
        controls_row.addWidget(self.show_world_axis_check)
        controls_row.addWidget(QtWidgets.QLabel("Edit"))
        self.cloud_interaction_mode_combo = QtWidgets.QComboBox()
        self.cloud_interaction_mode_combo.addItem("View", INTERACTION_MODE_CAMERA)
        self.cloud_interaction_mode_combo.addItem("Edit", INTERACTION_MODE_EDIT_SOURCE)
        self.cloud_interaction_mode_combo.setMaximumWidth(84)
        self.cloud_interaction_mode_combo.currentIndexChanged.connect(self._on_cloud_interaction_changed)
        controls_row.addWidget(self.cloud_interaction_mode_combo)
        controls_row.addWidget(QtWidgets.QLabel("Align"))
        self.cloud_lock_mode_combo = QtWidgets.QComboBox()
        self.cloud_lock_mode_combo.addItem("XY+Yaw", LOCK_MODE_XY_YAW)
        self.cloud_lock_mode_combo.addItem("Z", LOCK_MODE_Z_ONLY)
        self.cloud_lock_mode_combo.setMaximumWidth(96)
        self.cloud_lock_mode_combo.currentIndexChanged.connect(self._on_cloud_interaction_changed)
        controls_row.addWidget(self.cloud_lock_mode_combo)
        controls_row.addWidget(QtWidgets.QLabel("Drag"))
        self.cloud_drag_mode_combo = QtWidgets.QComboBox()
        self.cloud_drag_mode_combo.addItem("XY", EDIT_OPERATION_TRANSLATE)
        self.cloud_drag_mode_combo.addItem("Yaw", EDIT_OPERATION_ROTATE)
        self.cloud_drag_mode_combo.setMaximumWidth(72)
        self.cloud_drag_mode_combo.currentIndexChanged.connect(self._on_cloud_interaction_changed)
        controls_row.addWidget(self.cloud_drag_mode_combo)
        controls_row.addStretch(1)
        controls_row.addWidget(QtWidgets.QLabel("View"))
        self.camera_preset_buttons: dict[str, QtWidgets.QToolButton] = {}
        camera_preset_frame = QtWidgets.QFrame()
        camera_preset_frame.setObjectName("SegmentedControl")
        camera_preset_layout = QtWidgets.QHBoxLayout(camera_preset_frame)
        camera_preset_layout.setContentsMargins(2, 2, 2, 2)
        camera_preset_layout.setSpacing(2)
        for text, preset in (
            ("Top", VIEW_PRESET_TOP),
            ("Side-Y", VIEW_PRESET_SIDE_Y),
            ("Side-X", VIEW_PRESET_SIDE_X),
        ):
            button = QtWidgets.QToolButton()
            button.setText(text)
            button.setCheckable(True)
            button.setProperty("segmentRole", "segmented")
            button.clicked.connect(lambda _checked=False, value=preset: self.cloud_view.set_view_preset(value))
            camera_preset_layout.addWidget(button)
            self.camera_preset_buttons[preset] = button
        controls_row.addWidget(camera_preset_frame)
        self.manual_align_translation_step_spin = self._make_double_spin(
            step=0.01,
            minimum=0.01,
            maximum=5.0,
            value=0.05,
        )
        self.manual_align_rotation_step_spin = self._make_double_spin(
            step=0.5,
            minimum=0.1,
            maximum=30.0,
            value=1.0,
        )
        self.manual_align_translation_step_spin.hide()
        self.manual_align_rotation_step_spin.hide()
        self.manual_align_translation_step_spin.valueChanged.connect(self._on_cloud_interaction_changed)
        self.manual_align_rotation_step_spin.valueChanged.connect(self._on_cloud_interaction_changed)
        self.manual_align_snap_view_check = QtWidgets.QCheckBox("Snap")
        self.manual_align_snap_view_check.setChecked(True)
        self.manual_align_snap_view_check.stateChanged.connect(self._on_cloud_interaction_changed)
        controls_row.addWidget(self.manual_align_snap_view_check)
        self.reset_manual_align_button = QtWidgets.QPushButton("Reset Align")
        self.reset_manual_align_button.clicked.connect(self._reset_manual_align)
        controls_row.addWidget(self.reset_manual_align_button)
        self.reset_camera_button = QtWidgets.QPushButton("Reset Camera")
        self.reset_camera_button.clicked.connect(self.cloud_view.reset_camera)
        controls_row.addWidget(self.reset_camera_button)
        layout.addLayout(controls_row)
        self._on_camera_preset_changed(VIEW_PRESET_TOP)

        self.manual_align_status_label = QtWidgets.QLabel("View mode · wheel zoom")
        self.manual_align_status_label.setObjectName("SubtleText")
        self.manual_align_status_label.setWordWrap(False)
        layout.addWidget(self.manual_align_status_label)
        # ===== END CHANGE: manual align cloud toolbar =====

        self.cloud_legend_label = QtWidgets.QLabel(
            "Gray target · yellow selected frames · orange/cyan source · green final"
        )
        self.cloud_legend_label.setObjectName("SubtleText")
        self.cloud_legend_label.setWordWrap(False)
        layout.addWidget(self.cloud_legend_label)

        layout.addWidget(self.cloud_view, stretch=1)
        return group

    def _build_control_scroll_area(self) -> QtWidgets.QWidget:
        panel = self._build_control_panel()
        panel.setMinimumWidth(300)
        panel.setMaximumWidth(360)
        return panel

    def _wrap_control_tab(self, content: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
        scroll_area = QtWidgets.QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll_area.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        scroll_area.setWidget(content)
        return scroll_area

    def _build_summary_card(self, title: str, body_label: QtWidgets.QLabel) -> QtWidgets.QFrame:
        card = QtWidgets.QFrame()
        card.setObjectName("SummaryCard")
        card_layout = QtWidgets.QVBoxLayout(card)
        card_layout.setContentsMargins(6, 4, 6, 4)
        card_layout.setSpacing(1)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("SummaryCardTitle")
        title_label.setSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        card_layout.addWidget(title_label)
        card_layout.addWidget(body_label)
        return card

    def _build_action_section(
        self,
        title: str,
        widgets: list[tuple[QtWidgets.QWidget, int, int, int, int]],
    ) -> QtWidgets.QFrame:
        section = QtWidgets.QFrame()
        section.setObjectName("ActionSection")
        section_layout = QtWidgets.QVBoxLayout(section)
        section_layout.setContentsMargins(8, 6, 8, 6)
        section_layout.setSpacing(6)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("ActionSectionTitle")
        section_layout.addWidget(title_label)

        grid = QtWidgets.QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)
        for widget, row, column, row_span, column_span in widgets:
            grid.addWidget(widget, row, column, row_span, column_span)
        section_layout.addLayout(grid)
        return section

    def _build_control_panel(self) -> QtWidgets.QWidget:
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        control_tabs = QtWidgets.QTabWidget()
        control_tabs.setDocumentMode(True)
        control_tabs.setElideMode(QtCore.Qt.ElideRight)
        summary_tab = QtWidgets.QWidget()
        summary_layout = QtWidgets.QVBoxLayout(summary_tab)
        summary_layout.setContentsMargins(0, 0, 0, 0)
        summary_layout.setSpacing(6)
        advanced_tab = QtWidgets.QWidget()
        advanced_layout = QtWidgets.QVBoxLayout(advanced_tab)
        advanced_layout.setContentsMargins(0, 0, 0, 0)
        advanced_layout.setSpacing(6)

        selection_group = QtWidgets.QGroupBox("Summary")
        selection_layout = QtWidgets.QGridLayout(selection_group)
        selection_layout.setContentsMargins(6, 6, 6, 6)
        selection_layout.setHorizontalSpacing(4)
        selection_layout.setVerticalSpacing(4)
        self.pair_label = QtWidgets.QLabel("none")
        self.pair_label.setWordWrap(True)
        self.pair_label.setTextFormat(QtCore.Qt.RichText)
        self.pair_label.setObjectName("SummaryCardBody")
        self.source_label = QtWidgets.QLabel("none")
        self.source_label.setWordWrap(True)
        self.source_label.setObjectName("CompactValue")
        self.source_label.setTextFormat(QtCore.Qt.RichText)
        self.source_label.setObjectName("SummaryCardBody")
        self.target_label = QtWidgets.QLabel("none")
        self.target_label.setWordWrap(True)
        self.target_label.setTextFormat(QtCore.Qt.RichText)
        self.target_label.setObjectName("SummaryCardBody")
        self.selected_edge_label = QtWidgets.QLabel("none")
        self.selected_edge_label.setWordWrap(True)
        self.selected_edge_label.setTextFormat(QtCore.Qt.RichText)
        self.selected_edge_label.setObjectName("SummaryCardBody")
        self.working_graph_label = QtWidgets.QLabel("Rev 0 · no working graph")
        self.working_graph_label.setWordWrap(True)
        self.working_graph_label.setTextFormat(QtCore.Qt.RichText)
        self.working_graph_label.setObjectName("SummaryCardBody")
        self.target_map_label = QtWidgets.QLabel("No target submap.")
        self.target_map_label.setWordWrap(True)
        self.target_map_label.setTextFormat(QtCore.Qt.RichText)
        self.target_map_label.setObjectName("SummaryCardBody")
        self.gicp_metrics_label = QtWidgets.QLabel("No GICP result.")
        self.gicp_metrics_label.setWordWrap(True)
        self.gicp_metrics_label.setTextFormat(QtCore.Qt.RichText)
        self.gicp_metrics_label.setObjectName("SummaryCardBody")
        selection_layout.addWidget(self._build_summary_card("Graph", self.working_graph_label), 0, 0)
        selection_layout.addWidget(self._build_summary_card("State", self.gicp_metrics_label), 0, 1)
        selection_layout.addWidget(self._build_summary_card("Pair", self.pair_label), 1, 0, 1, 2)
        selection_layout.addWidget(self._build_summary_card("Edge", self.selected_edge_label), 2, 0)
        selection_layout.addWidget(self._build_summary_card("Map", self.target_map_label), 2, 1)
        selection_layout.setColumnStretch(0, 1)
        selection_layout.setColumnStretch(1, 1)
        summary_layout.addWidget(selection_group)

        delta_group = QtWidgets.QGroupBox("Delta")
        delta_group.setToolTip("Source-local seed. Viewer drag edits the same values.")
        delta_layout = QtWidgets.QGridLayout(delta_group)
        delta_layout.setContentsMargins(7, 7, 7, 7)
        delta_layout.setHorizontalSpacing(5)
        delta_layout.setVerticalSpacing(3)
        self.delta_spins = {
            "x": self._make_double_spin(step=0.1, minimum=-1000.0, maximum=1000.0, value=0.0),
            "y": self._make_double_spin(step=0.1, minimum=-1000.0, maximum=1000.0, value=0.0),
            "z": self._make_double_spin(step=0.1, minimum=-1000.0, maximum=1000.0, value=0.0),
            "roll": self._make_double_spin(step=1.0, minimum=-180.0, maximum=180.0, value=0.0),
            "pitch": self._make_double_spin(step=1.0, minimum=-180.0, maximum=180.0, value=0.0),
            "yaw": self._make_double_spin(step=1.0, minimum=-180.0, maximum=180.0, value=0.0),
        }
        for spin in self.delta_spins.values():
            spin.valueChanged.connect(self._on_delta_spin_changed)
        for column, key in enumerate(("x", "y", "z")):
            delta_layout.addWidget(QtWidgets.QLabel(key), 0, column)
            delta_layout.addWidget(self.delta_spins[key], 1, column)
        for column, (label_text, key) in enumerate((("r", "roll"), ("p", "pitch"), ("y", "yaw"))):
            delta_layout.addWidget(QtWidgets.QLabel(label_text), 2, column)
            delta_layout.addWidget(self.delta_spins[key], 3, column)
        reset_button = QtWidgets.QPushButton("Reset Delta")
        reset_button.clicked.connect(self._reset_delta)
        refresh_button = QtWidgets.QPushButton("Refresh Preview")
        refresh_button.clicked.connect(lambda: self.schedule_preview_refresh(reset_camera=False))
        self.auto_yaw_steps_spin = QtWidgets.QSpinBox()
        self.auto_yaw_steps_spin.setRange(2, 72)
        self.auto_yaw_steps_spin.setValue(12)
        self.auto_yaw_steps_spin.setToolTip("Sweep evenly-spaced yaw seeds across 0-360 degrees.")
        self.auto_yaw_button = QtWidgets.QPushButton("Auto Yaw Sweep")
        self.auto_yaw_button.clicked.connect(self.run_auto_yaw_sweep)
        delta_layout.addWidget(reset_button, 4, 0, 1, 2)
        refresh_button.setText("Preview")
        delta_layout.addWidget(refresh_button, 4, 2)
        summary_layout.addWidget(delta_group)

        registration_group = QtWidgets.QGroupBox("Registration")
        registration_layout = QtWidgets.QGridLayout(registration_group)
        registration_layout.setContentsMargins(7, 7, 7, 7)
        registration_layout.setHorizontalSpacing(5)
        registration_layout.setVerticalSpacing(4)
        self.target_cloud_mode_combo = QtWidgets.QComboBox()
        self.target_cloud_mode_combo.addItem("Temporal Window", TARGET_CLOUD_MODE_TEMPORAL_WINDOW)
        self.target_cloud_mode_combo.addItem("RS Spatial Submap", TARGET_CLOUD_MODE_RS_SPATIAL_SUBMAP)
        default_mode_index = self.target_cloud_mode_combo.findData(OFFICE_DEFAULT_TARGET_CLOUD_MODE)
        if default_mode_index >= 0:
            self.target_cloud_mode_combo.setCurrentIndex(default_mode_index)
        self.target_cloud_mode_combo.currentIndexChanged.connect(self._on_target_cloud_mode_changed)
        self.target_neighbors_spin = QtWidgets.QSpinBox()
        self.target_neighbors_spin.setRange(1, 500)
        self.target_neighbors_spin.setValue(OFFICE_DEFAULT_TARGET_NEIGHBORS)
        self.target_neighbors_spin.valueChanged.connect(self.schedule_preview_refresh)
        self.target_min_gap_spin = self._make_double_spin(
            step=1.0,
            minimum=0.0,
            maximum=10000.0,
            value=OFFICE_DEFAULT_TARGET_MIN_TIME_GAP_SEC,
        )
        self.target_min_gap_spin.valueChanged.connect(self.schedule_preview_refresh)
        self.target_map_voxel_spin = self._make_double_spin(
            step=0.05,
            minimum=0.0,
            maximum=5.0,
            value=OFFICE_DEFAULT_TARGET_MAP_VOXEL_SIZE,
        )
        self.target_map_voxel_spin.valueChanged.connect(self.schedule_preview_refresh)
        self.voxel_spin = self._make_double_spin(
            step=0.05,
            minimum=0.0,
            maximum=5.0,
            value=OFFICE_DEFAULT_VOXEL_SIZE,
        )
        self.max_corr_spin = self._make_double_spin(
            step=0.1,
            minimum=0.1,
            maximum=20.0,
            value=OFFICE_DEFAULT_MAX_CORRESPONDENCE_DISTANCE,
        )
        self.max_iter_spin = QtWidgets.QSpinBox()
        self.max_iter_spin.setRange(1, 500)
        self.max_iter_spin.setValue(OFFICE_DEFAULT_MAX_ITERATIONS)
        self.target_neighbors_label = QtWidgets.QLabel("Nbr")
        self.target_neighbors_label.setToolTip("Temporal window radius or RS spatial neighbor count.")
        self.target_min_gap_label = QtWidgets.QLabel("Gap[s]")
        self.target_min_gap_label.setToolTip("Minimum source-target time gap, only used by RS Spatial Submap.")
        mode_label = QtWidgets.QLabel("Mode")
        mode_label.setToolTip("Target cloud construction mode.")
        target_map_voxel_label = QtWidgets.QLabel("TgtVoxel")
        target_map_voxel_label.setToolTip("Downsample size for the target map preview.")
        voxel_label = QtWidgets.QLabel("Voxel")
        voxel_label.setToolTip("Source and target voxel size for GICP.")
        radius_label = QtWidgets.QLabel("Radius")
        radius_label.setToolTip("Maximum search radius for correspondence lookup.")
        iter_label = QtWidgets.QLabel("Iter")
        iter_label.setToolTip("Maximum GICP iterations.")
        registration_layout.addWidget(mode_label, 0, 0)
        registration_layout.addWidget(self.target_cloud_mode_combo, 0, 1, 1, 3)
        registration_layout.addWidget(self.target_neighbors_label, 1, 0)
        registration_layout.addWidget(self.target_neighbors_spin, 1, 1)
        registration_layout.addWidget(self.target_min_gap_label, 1, 2)
        registration_layout.addWidget(self.target_min_gap_spin, 1, 3)
        registration_layout.addWidget(target_map_voxel_label, 2, 0)
        registration_layout.addWidget(self.target_map_voxel_spin, 2, 1)
        registration_layout.addWidget(voxel_label, 2, 2)
        registration_layout.addWidget(self.voxel_spin, 2, 3)
        registration_layout.addWidget(radius_label, 3, 0)
        registration_layout.addWidget(self.max_corr_spin, 3, 1)
        registration_layout.addWidget(iter_label, 3, 2)
        registration_layout.addWidget(self.max_iter_spin, 3, 3)
        registration_layout.setColumnStretch(1, 1)
        registration_layout.setColumnStretch(3, 1)

        self._update_target_cloud_mode_controls()
        summary_layout.addWidget(registration_group)

        advanced_group = QtWidgets.QGroupBox("Advanced")
        advanced_group.setToolTip("Optional expert settings. The default values already match the validated parity runs.")
        advanced_layout_grid = QtWidgets.QGridLayout(advanced_group)
        advanced_layout_grid.setContentsMargins(8, 8, 8, 8)
        advanced_layout_grid.setHorizontalSpacing(6)
        advanced_layout_grid.setVerticalSpacing(4)
        advanced_note = QtWidgets.QLabel("Python is the default backend. C++ is optional and only used for fallback or parity checks.")
        advanced_note.setObjectName("SubtleText")
        advanced_note.setWordWrap(True)
        advanced_layout_grid.addWidget(advanced_note, 0, 0, 1, 2)
        self.variance_t_shared_spin = self._make_double_spin(
            step=0.01,
            minimum=0.0,
            maximum=100.0,
            value=OFFICE_DEFAULT_VARIANCE_T[0],
        )
        self.variance_r_shared_spin = self._make_double_spin(
            step=0.01,
            minimum=0.0,
            maximum=100.0,
            value=OFFICE_DEFAULT_VARIANCE_R_RAD2[0],
        )
        self.export_map_voxel_spin = self._make_double_spin(
            step=0.05,
            minimum=0.0,
            maximum=5.0,
            value=0.0,
        )
        advanced_layout_grid.addWidget(QtWidgets.QLabel("Variance T [m^2]"), 1, 0)
        advanced_layout_grid.addWidget(self.variance_t_shared_spin, 1, 1)
        advanced_layout_grid.addWidget(QtWidgets.QLabel("Variance R [rad^2]"), 2, 0)
        advanced_layout_grid.addWidget(self.variance_r_shared_spin, 2, 1)
        map_voxel_label = QtWidgets.QLabel("MapVoxel")
        map_voxel_label.setToolTip("Final map export voxel size.")
        advanced_layout_grid.addWidget(map_voxel_label, 3, 0)
        advanced_layout_grid.addWidget(self.export_map_voxel_spin, 3, 1)

        self._cpp_optimizer_command_cache = self._resolve_cpp_optimizer_command()
        self._show_legacy_backend_controls = (
            self._cpp_optimizer_command_cache is not None
            and os.environ.get("MLCT_SHOW_LEGACY_CPP_BACKEND", "").strip().lower() in {"1", "true", "yes"}
        )
        backend_label = QtWidgets.QLabel("Backend")
        backend_label.setObjectName("SubtleText")
        advanced_layout_grid.addWidget(backend_label, 4, 0)
        if self._show_legacy_backend_controls:
            self.optimizer_backend_combo = QtWidgets.QComboBox()
            self.optimizer_backend_combo.addItem("Python", BACKEND_PREFERENCE_PYTHON)
            self.optimizer_backend_combo.addItem("C++", BACKEND_PREFERENCE_CPP)
            backend_preference = os.environ.get(
                "MANUAL_LOOP_OPTIMIZER_BACKEND",
                BACKEND_PREFERENCE_PYTHON,
            ).lower()
            backend_index = {
                BACKEND_PREFERENCE_PYTHON: 0,
                BACKEND_PREFERENCE_CPP: 1,
            }.get(backend_preference, 0)
            self.optimizer_backend_combo.setCurrentIndex(backend_index)
            advanced_layout_grid.addWidget(self.optimizer_backend_combo, 4, 1)
            backend_hint = QtWidgets.QLabel(
                "Developer mode: legacy C++ backend selector is visible."
            )
        else:
            self.optimizer_backend_label = QtWidgets.QLabel("Python")
            self.optimizer_backend_label.setObjectName("SubtleText")
            advanced_layout_grid.addWidget(self.optimizer_backend_label, 4, 1)
            backend_hint = QtWidgets.QLabel(
                "Python-first path. Legacy C++ stays hidden and is only used as an automatic fallback when available."
            )
        backend_hint.setObjectName("SubtleText")
        backend_hint.setWordWrap(True)
        advanced_layout_grid.addWidget(backend_hint, 5, 0, 1, 2)
        advanced_layout.addWidget(advanced_group)

        action_group = QtWidgets.QGroupBox("Actions")
        action_layout = QtWidgets.QVBoxLayout(action_group)
        action_layout.setContentsMargins(7, 7, 7, 7)
        action_layout.setSpacing(5)
        self.run_gicp_button = QtWidgets.QPushButton("GICP")
        self.run_gicp_button.setProperty("buttonRole", "primary")
        self.run_gicp_button.clicked.connect(self.run_gicp)
        self.accept_button = QtWidgets.QPushButton("Add")
        self.accept_button.clicked.connect(self.accept_constraint)
        self.accept_button.setEnabled(False)
        self.replace_edge_button = QtWidgets.QPushButton("Replace")
        self.replace_edge_button.clicked.connect(self.replace_selected_edge)
        self.replace_edge_button.setEnabled(False)
        self.disable_edge_button = QtWidgets.QPushButton("Disable")
        self.disable_edge_button.clicked.connect(self.disable_selected_edge)
        self.restore_edge_button = QtWidgets.QPushButton("Restore")
        self.restore_edge_button.clicked.connect(self.restore_selected_edge)
        self.remove_manual_button = QtWidgets.QPushButton("Remove")
        self.remove_manual_button.clicked.connect(self.remove_selected_manual_constraint)
        self.optimize_button = QtWidgets.QPushButton("Optim")
        self.optimize_button.setProperty("buttonRole", "primary")
        self.optimize_button.clicked.connect(self.run_optimization)
        self.export_button = QtWidgets.QPushButton("Export")
        self.export_button.setProperty("buttonRole", "primary")
        self.export_button.clicked.connect(self.export_final_result)
        self.undo_button = QtWidgets.QPushButton("Undo")
        self.undo_button.clicked.connect(self.undo_last_change)
        self.undo_button.setEnabled(False)
        yaw_steps_label = QtWidgets.QLabel("Yaw")
        yaw_steps_label.setObjectName("SubtleText")
        auto_yaw_row = QtWidgets.QWidget()
        auto_yaw_row_layout = QtWidgets.QHBoxLayout(auto_yaw_row)
        auto_yaw_row_layout.setContentsMargins(0, 0, 0, 0)
        auto_yaw_row_layout.setSpacing(6)
        auto_yaw_row_layout.addWidget(yaw_steps_label)
        auto_yaw_row_layout.addWidget(self.auto_yaw_steps_spin)
        self.auto_yaw_button.setText("Auto Yaw")
        auto_yaw_row_layout.addWidget(self.auto_yaw_button, 1)
        action_layout.addWidget(
            self._build_action_section(
                "Match",
                [
                    (self.run_gicp_button, 0, 0, 1, 2),
                    (auto_yaw_row, 1, 0, 1, 2),
                ],
            )
        )
        action_layout.addWidget(
            self._build_action_section(
                "Edit Graph",
                [
                    (self.accept_button, 0, 0, 1, 1),
                    (self.replace_edge_button, 0, 1, 1, 1),
                    (self.disable_edge_button, 1, 0, 1, 1),
                    (self.restore_edge_button, 1, 1, 1, 1),
                    (self.remove_manual_button, 2, 0, 1, 2),
                ],
            )
        )
        action_layout.addWidget(
            self._build_action_section(
                "Commit",
                [
                    (self.optimize_button, 0, 0, 1, 1),
                    (self.undo_button, 0, 1, 1, 1),
                    (self.export_button, 1, 0, 1, 2),
                ],
            )
        )
        summary_layout.addWidget(action_group)

        summary_layout.addStretch(1)
        advanced_layout.addStretch(1)
        control_tabs.addTab(self._wrap_control_tab(summary_tab), "Summary")
        control_tabs.addTab(self._wrap_control_tab(advanced_tab), "Advanced")
        layout.addWidget(control_tabs)
        self._update_edge_action_buttons()
        return container

    def _build_constraint_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        self.change_legend_label = QtWidgets.QLabel(
            "Green=Applied · Amber=Accepted · Gray=Disabled · Note supports double-click edit"
        )
        self.change_legend_label.setObjectName("SubtleText")
        self.change_legend_label.setWordWrap(True)
        layout.addWidget(self.change_legend_label)
        filter_row = QtWidgets.QHBoxLayout()
        filter_row.addWidget(QtWidgets.QLabel("Status"))
        self.change_status_filter_combo = QtWidgets.QComboBox()
        self.change_status_filter_combo.addItems(
            ["All Status", "Accepted", "Applied", "Disabled"]
        )
        self.change_status_filter_combo.currentTextChanged.connect(self._on_graph_change_filter_changed)
        filter_row.addWidget(self.change_status_filter_combo)
        filter_row.addWidget(QtWidgets.QLabel("Type"))
        self.change_type_filter_combo = QtWidgets.QComboBox()
        self.change_type_filter_combo.addItems(
            ["All Types", "Manual Add", "Replace Existing Loop", "Disable Existing Loop"]
        )
        self.change_type_filter_combo.currentTextChanged.connect(self._on_graph_change_filter_changed)
        filter_row.addWidget(self.change_type_filter_combo)
        filter_row.addStretch(1)
        layout.addLayout(filter_row)
        self.constraint_table = QtWidgets.QTableWidget(0, 13)
        self.constraint_table.setHorizontalHeaderLabels(
            [
                "Use",
                "Type",
                "Status",
                "Src",
                "Tgt",
                "Submap",
                "Range",
                "Fitness",
                "RMSE",
                "Noise",
                "Accepted@",
                "Applied@",
                "Note",
            ]
        )
        self.constraint_table.horizontalHeader().setStretchLastSection(True)
        self.constraint_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.constraint_table.setEditTriggers(
            QtWidgets.QAbstractItemView.DoubleClicked
            | QtWidgets.QAbstractItemView.EditKeyPressed
            | QtWidgets.QAbstractItemView.SelectedClicked
        )
        self.constraint_table.itemChanged.connect(self._on_constraint_item_changed)
        self.constraint_table.itemSelectionChanged.connect(self._on_constraint_selection_changed)
        layout.addWidget(self.constraint_table)
        return widget

    def _build_log_tab(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(widget)
        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)
        return widget

    def _make_double_spin(
        self,
        *,
        step: float,
        minimum: float,
        maximum: float,
        value: float,
    ) -> QtWidgets.QDoubleSpinBox:
        spin = QtWidgets.QDoubleSpinBox()
        spin.setDecimals(6)
        spin.setSingleStep(step)
        spin.setRange(minimum, maximum)
        spin.setValue(value)
        spin.setMinimumWidth(72)
        spin.setMaximumWidth(96)
        return spin

    def _pack_row(
        self,
        labels: tuple[str, ...],
        spins: tuple[QtWidgets.QDoubleSpinBox, ...],
    ) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        for label, spin in zip(labels, spins):
            layout.addWidget(QtWidgets.QLabel(label))
            layout.addWidget(spin)
        layout.addStretch(1)
        return widget

    def _pack_vector_grid(
        self,
        labels: tuple[str, ...],
        spins: tuple[QtWidgets.QDoubleSpinBox, ...],
    ) -> QtWidgets.QWidget:
        widget = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(4)
        layout.setVerticalSpacing(2)
        for column, (label, spin) in enumerate(zip(labels, spins)):
            layout.addWidget(QtWidgets.QLabel(label), 0, column)
            layout.addWidget(spin, 1, column)
        return widget

    def _current_target_cloud_mode(self) -> str:
        mode = self.target_cloud_mode_combo.currentData()
        return str(mode if mode is not None else OFFICE_DEFAULT_TARGET_CLOUD_MODE)

    def _current_view_kind(self) -> str:
        return self.trajectory_view_combo.currentText().strip().lower()

    def _is_working_view(self) -> bool:
        return self._current_view_kind() == "working"

    def _capture_undo_snapshot(self) -> UndoSnapshot:
        return UndoSnapshot(
            pose_graph=copy.deepcopy(self.pose_graph),
            trajectory=copy.deepcopy(self.trajectory),
            constraints=copy.deepcopy(self.constraints),
            disabled_loop_changes=copy.deepcopy(self.disabled_loop_changes),
            source_id=self.source_id,
            target_id=self.target_id,
            selected_edge_ref=copy.deepcopy(self.selected_edge_ref),
            candidate_replace_edge_uid=self._candidate_replace_edge_uid,
            working_revision=self._working_revision,
            session_dirty=self._session_dirty,
            last_output_dir=self._last_output_dir,
            pick_mode=self.pick_mode,
        )

    def _push_undo_snapshot(self) -> None:
        self._undo_stack.append(self._capture_undo_snapshot())
        self.undo_button.setEnabled(True)

    def _restore_snapshot(self, snapshot: UndoSnapshot) -> None:
        self.pose_graph = snapshot.pose_graph
        self.trajectory = snapshot.trajectory
        self.constraints = copy.deepcopy(snapshot.constraints)
        self.disabled_loop_changes = copy.deepcopy(snapshot.disabled_loop_changes)
        self.source_id = snapshot.source_id
        self.target_id = snapshot.target_id
        self.selected_edge_ref = copy.deepcopy(snapshot.selected_edge_ref)
        self._candidate_replace_edge_uid = snapshot.candidate_replace_edge_uid
        self._working_revision = snapshot.working_revision
        self._session_dirty = snapshot.session_dirty
        self._last_output_dir = snapshot.last_output_dir
        self.pick_mode = snapshot.pick_mode
        self.current_preview = None
        self.last_result = None
        self._preview_scene_key = None
        self.accept_button.setEnabled(False)
        self.gicp_metrics_label.setText("Undo restored the previous working state.")
        if self.trajectory is not None and self.session_paths is not None:
            self.workspace = RegistrationWorkspace(self.session_paths.keyframe_dir, self.trajectory)
        else:
            self.workspace = None
        self.cloud_view.clear_scene()
        self._set_pick_mode(self.pick_mode)
        self._update_preview_summary(None)
        self._update_selection_labels()
        self._update_edge_action_buttons()
        self._rebuild_constraint_table()
        self._sync_constraint_table_selection()
        self._update_session_status_widgets()
        self._refresh_plot(preserve_view=False)
        self._update_cloud_interaction_controls()

    def _status_text(self, enabled: bool, applied_rev: Optional[int]) -> str:
        if not enabled:
            return "Disabled"
        if applied_rev is not None:
            return "Applied"
        return "Accepted"

    def _active_disabled_loop_changes(self) -> list[ExistingLoopChange]:
        return [change for change in self.disabled_loop_changes.values() if change.enabled]

    def _recompute_session_dirty(self) -> None:
        dirty = False
        for constraint in self.constraints:
            if constraint.enabled and constraint.applied_rev is None:
                dirty = True
                break
            if not constraint.enabled and constraint.applied_rev is not None:
                dirty = True
                break
        if not dirty:
            for change in self.disabled_loop_changes.values():
                if change.enabled and change.applied_rev is None:
                    dirty = True
                    break
                if not change.enabled and change.applied_rev is not None:
                    dirty = True
                    break
        self._session_dirty = dirty

    def _update_session_status_widgets(self) -> None:
        manual_active = sum(1 for constraint in self.constraints if constraint.enabled)
        disabled_active = sum(1 for change in self.disabled_loop_changes.values() if change.enabled)
        self.working_rev_badge.setText(f"R{self._working_revision}")
        self.session_state_badge.setText("Dirty" if self._session_dirty else "OK")
        self.change_summary_badge.setText(f"M{manual_active} D{disabled_active}")
        if self._session_dirty:
            self.session_state_badge.setStyleSheet(
                "background:#fff4db;border:1px solid #f2c46d;border-radius:10px;padding:2px 8px;font-weight:600;color:#8a5300;"
            )
        else:
            self.session_state_badge.setStyleSheet(
                "background:#e7f8ee;border:1px solid #74c69d;border-radius:10px;padding:2px 8px;font-weight:600;color:#1f6f43;"
            )
        self.working_rev_badge.setStyleSheet(
            "background:#e8f1ff;border:1px solid #8ab4f8;border-radius:10px;padding:2px 8px;font-weight:600;color:#1e4f91;"
        )
        self.change_summary_badge.setStyleSheet(
            "background:#eef2f7;border:1px solid #d6dde6;border-radius:10px;padding:2px 8px;font-weight:600;color:#334155;"
        )
        if self.pose_graph is None or self.trajectory is None:
            self.working_graph_label.setText("No working graph loaded.")
            self.working_graph_label.setToolTip("No working graph loaded.")
        else:
            loop_count = len([edge for edge in self.pose_graph.loop_edges if edge.enabled])
            self.working_graph_label.setText(
                f"Rev {self._working_revision} · P{self.trajectory.size} · "
                f"L{loop_count} · {'Dirty' if self._session_dirty else 'Clean'}"
            )
            self.working_graph_label.setToolTip(
                f"revision={self._working_revision}\nposes={self.trajectory.size}\n"
                f"enabled_loops={loop_count}\nstate={'dirty pending changes' if self._session_dirty else 'clean'}"
            )
        export_enabled = self._last_output_dir is not None or self._session_dirty
        self.export_button.setEnabled(export_enabled)
        self.undo_button.setEnabled(bool(self._undo_stack))

    def _update_target_cloud_mode_controls(self) -> None:
        mode = self._current_target_cloud_mode()
        temporal_mode = mode == TARGET_CLOUD_MODE_TEMPORAL_WINDOW
        self.target_neighbors_label.setText(
            "Win"
            if temporal_mode
            else "Nbr"
        )
        self.target_min_gap_spin.setEnabled(not temporal_mode)
        self.target_min_gap_label.setEnabled(not temporal_mode)
        self.target_min_gap_spin.setToolTip(
            "Only used by RS Spatial Submap mode."
            if temporal_mode
            else "Reject target candidates whose timestamps are too close to source."
        )

    def _on_target_cloud_mode_changed(self) -> None:
        self._update_target_cloud_mode_controls()
        self.schedule_preview_refresh(reset_camera=False)

    # ===== BEGIN CHANGE: manual align viewer controls =====
    def _manual_align_editable(self) -> bool:
        return bool(
            self._is_working_view()
            and self.workspace is not None
            and self.source_id is not None
            and self.target_id is not None
            and (self.selected_edge_ref is None or self.selected_edge_ref.edge_kind == "existing")
        )

    def _update_cloud_interaction_controls(self, *, apply_view_preset: bool = False) -> None:
        editable = self._manual_align_editable()
        if not editable and self.cloud_interaction_mode_combo.currentData() != INTERACTION_MODE_CAMERA:
            with QtCore.QSignalBlocker(self.cloud_interaction_mode_combo):
                self.cloud_interaction_mode_combo.setCurrentIndex(
                    self.cloud_interaction_mode_combo.findData(INTERACTION_MODE_CAMERA)
                )
        mode = str(self.cloud_interaction_mode_combo.currentData() or INTERACTION_MODE_CAMERA)
        lock_mode = str(self.cloud_lock_mode_combo.currentData() or LOCK_MODE_XY_YAW)
        drag_mode = str(self.cloud_drag_mode_combo.currentData() or EDIT_OPERATION_TRANSLATE)
        translation_step = float(self.manual_align_translation_step_spin.value())
        rotation_step = float(self.manual_align_rotation_step_spin.value())
        snap_view = self.manual_align_snap_view_check.isChecked()
        xy_lock = lock_mode == LOCK_MODE_XY_YAW
        self.cloud_drag_mode_combo.setEnabled(editable and xy_lock)
        if not xy_lock and self.cloud_drag_mode_combo.currentData() != EDIT_OPERATION_TRANSLATE:
            with QtCore.QSignalBlocker(self.cloud_drag_mode_combo):
                self.cloud_drag_mode_combo.setCurrentIndex(
                    self.cloud_drag_mode_combo.findData(EDIT_OPERATION_TRANSLATE)
                )
            drag_mode = EDIT_OPERATION_TRANSLATE
        self.cloud_view.set_interaction_mode(
            mode if editable else INTERACTION_MODE_CAMERA
        )
        self.cloud_view.set_manual_align_options(
            lock_mode=lock_mode,
            edit_operation=drag_mode,
            translation_step_m=translation_step,
            rotation_step_deg=rotation_step,
            snap_view=snap_view,
        )
        interaction_widgets = (
            self.cloud_interaction_mode_combo,
            self.cloud_lock_mode_combo,
            self.cloud_drag_mode_combo,
            self.manual_align_translation_step_spin,
            self.manual_align_rotation_step_spin,
            self.manual_align_snap_view_check,
            self.reset_manual_align_button,
        )
        for widget in interaction_widgets:
            widget.setEnabled(editable)
        if editable and mode == INTERACTION_MODE_EDIT_SOURCE and apply_view_preset and snap_view:
            self.cloud_view.apply_lock_view_preset()
        elif not editable:
            self.manual_align_status_label.setText("Camera mode")

    def _on_cloud_interaction_changed(self) -> None:
        if str(self.cloud_interaction_mode_combo.currentData() or INTERACTION_MODE_CAMERA) == INTERACTION_MODE_EDIT_SOURCE:
            self._set_display_mode("Preview")
        self._update_cloud_interaction_controls(apply_view_preset=True)

    def _on_manual_align_status(self, text: str) -> None:
        self.manual_align_status_label.setText(text)
        self.manual_align_status_label.setVisible("Edit" in text)

    def _on_camera_preset_changed(self, preset: str) -> None:
        for preset_name, button in self.camera_preset_buttons.items():
            with QtCore.QSignalBlocker(button):
                button.setChecked(preset_name == preset)

    def _set_delta_spin_values(self, values: np.ndarray) -> None:
        blockers = [QtCore.QSignalBlocker(spin) for spin in self.delta_spins.values()]
        try:
            for key, value in zip(
                ("x", "y", "z", "roll", "pitch", "yaw"),
                np.asarray(values, dtype=np.float64).tolist(),
            ):
                self.delta_spins[key].setValue(float(value))
        finally:
            del blockers

    def _set_delta_from_world_source_transform(self, transform_world_source: np.ndarray) -> None:
        if self.current_preview is None or self.current_preview.transform_world_source_initial is None:
            return
        base_transform = self.current_preview.transform_world_source_initial
        delta_local = np.linalg.inv(base_transform) @ np.asarray(transform_world_source, dtype=np.float64)
        self._set_delta_spin_values(matrix_to_xyz_rpy_deg(delta_local))

    def _invalidate_last_result_for_manual_align(self) -> None:
        if self.last_result is None and self.gicp_metrics_label.text().startswith("Manual align updated"):
            return
        self.last_result = None
        self.accept_button.setEnabled(False)
        self._set_display_mode("Preview")
        self.gicp_metrics_label.setText(
            "Manual align updated · rerun GICP."
            "<br><span style='color:#64748b;font-size:11px'>Viewer drag edits the current source seed.</span>"
        )
        self._update_edge_action_buttons()

    def _on_manual_align_changed(self, payload: ManualAlignUpdate) -> None:
        if not self._manual_align_editable():
            return
        self._set_delta_from_world_source_transform(payload.transform_world_source)
        self._invalidate_last_result_for_manual_align()
        self.schedule_preview_refresh(reset_camera=False)

    def _on_delta_spin_changed(self) -> None:
        if self.workspace is None or self.source_id is None or self.target_id is None:
            return
        self._invalidate_last_result_for_manual_align()
        self.schedule_preview_refresh(reset_camera=False)

    def _reset_manual_align(self) -> None:
        self._reset_delta()
        if self.manual_align_snap_view_check.isChecked():
            self.cloud_view.apply_lock_view_preset()
    # ===== END CHANGE: manual align viewer controls =====

    def _on_show_ghost_changed(self, *_args) -> None:
        self._update_trajectory_legend_label()
        self._refresh_plot(preserve_view=True)

    def _update_trajectory_legend_label(self) -> None:
        current_text = "Working" if self._is_working_view() else "Original"
        ghost_text = " · Ghost" if self.show_ghost_check.isChecked() else ""
        self.trajectory_legend_label.setText(
            "<span style='color:#4c78a8;font-weight:600'>Traj</span> · "
            "<span style='color:#d62728;font-weight:600'>Loop</span> · "
            "<span style='color:#8a8a8a;font-weight:600'>Disabled</span> · "
            "<span style='color:#2ca02c;font-weight:600'>Manual</span> · "
            "<span style='color:#f6d32d;font-weight:600'>Target</span> · "
            "<span style='color:#bc5090;font-weight:600'>Pair</span> · "
            "<span style='color:#ff7f0e;font-weight:600'>Src</span>/<span style='color:#17becf;font-weight:600'>Tgt</span>"
            f" <span style='color:#64748b'>| {current_text}{ghost_text}</span>"
        )

    def _update_cloud_display_controls(self) -> None:
        display_mode = self.display_mode_combo.currentText().strip().lower()
        self.cloud_view.set_display_options(
            display_mode=display_mode,
            trajectory_point_size=float(self.trajectory_point_size_spin.value()),
            target_point_size=float(self.target_point_size_spin.value()),
            source_point_size=float(self.source_point_size_spin.value()),
            show_world_axis=self.show_world_axis_check.isChecked(),
            reset_camera=False,
        )
        self._update_cloud_interaction_controls()
        if display_mode == "preview":
            legend = "Preview: gray target · yellow subset · orange/cyan source"
        elif display_mode == "final":
            legend = "Final: gray target · yellow subset · green source"
        else:
            legend = "Compare: gray target · yellow subset · source states"
        if self.show_world_axis_check.isChecked():
            legend += " · world axis"
        self.cloud_legend_label.setText(legend)

    def _current_cloud_point_size_summary(self) -> str:
        return (
            f"viewer_size(target={self.target_point_size_spin.value():.1f}, "
            f"source={self.source_point_size_spin.value():.1f}, "
            f"traj={self.trajectory_point_size_spin.value():.0f})"
        )

    def _gicp_log_cloud_summary(self, preview: RegistrationPreview) -> str:
        return (
            f"{self._current_cloud_point_size_summary()} | "
            f"points(target={preview.target_point_count}, "
            f"source={preview.source_points_local.shape[0]})"
        )

    def _trajectory_scene_payload(self) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if self.trajectory is None or self.trajectory.size == 0:
            return None, None
        return (
            self.trajectory.positions_xyz,
            np.arange(self.trajectory.size, dtype=np.float64),
        )

    def _make_preview_scene(
        self,
        *,
        preview: RegistrationPreview,
        initial_source_points: Optional[np.ndarray] = None,
        adjusted_source_points: Optional[np.ndarray] = None,
        final_source_points: Optional[np.ndarray] = None,
        transform_world_source_initial: Optional[np.ndarray] = None,
        transform_world_source_adjusted: Optional[np.ndarray] = None,
        transform_world_source_final: Optional[np.ndarray] = None,
    ) -> PreviewScene:
        trajectory_points, trajectory_values = self._trajectory_scene_payload()
        selected_target_trajectory_points = None
        if self.trajectory is not None and preview.target_frame_indices:
            selected_indices = np.asarray(preview.target_frame_indices, dtype=np.int64)
            selected_target_trajectory_points = self.trajectory.positions_xyz[selected_indices]
        return PreviewScene(
            target_points=preview.target_points_world,
            trajectory_points=trajectory_points,
            trajectory_values=trajectory_values,
            selected_target_trajectory_points=selected_target_trajectory_points,
            editable_source_points_local=preview.source_points_local,
            editable_source_transform=(
                transform_world_source_adjusted
                if transform_world_source_adjusted is not None
                else transform_world_source_initial
            ),
            initial_source_points=initial_source_points,
            adjusted_source_points=adjusted_source_points,
            final_source_points=final_source_points,
            transform_world_target=preview.transform_world_target,
            transform_world_source_initial=transform_world_source_initial,
            transform_world_source_adjusted=transform_world_source_adjusted,
            transform_world_source_final=transform_world_source_final,
        )

    def _projects_root(self) -> Optional[Path]:
        if self.session_paths is None:
            return None
        return self.session_paths.session_root / "manual_loop_projects"

    def _latest_project_pointer_path(self) -> Optional[Path]:
        root = self._projects_root()
        if root is None:
            return None
        return root / "latest_project.json"

    def _new_project_id(self) -> str:
        return datetime.now().strftime("%Y%m%d_%H%M%S")

    def _project_payload_summary(self) -> dict:
        return {
            "project_id": self._project_id,
            "session_root": str(self.session_paths.session_root) if self.session_paths is not None else None,
            "g2o_path": str(self.session_paths.g2o_path) if self.session_paths is not None else None,
            "tum_path": str(self.session_paths.tum_path) if self.session_paths is not None else None,
            "keyframe_dir": str(self.session_paths.keyframe_dir) if self.session_paths is not None else None,
            "working_revision": self._working_revision,
            "session_dirty": self._session_dirty,
            "last_output_dir": str(self._last_output_dir) if self._last_output_dir is not None else None,
            "latest_export_dir": str(self._latest_export_dir) if self._latest_export_dir is not None else None,
            "pick_mode": self.pick_mode,
            "source_id": self.source_id,
            "target_id": self.target_id,
            "selected_edge_ref": (
                {
                    "edge_kind": self.selected_edge_ref.edge_kind,
                    "edge_uid": self.selected_edge_ref.edge_uid,
                }
                if self.selected_edge_ref is not None
                else None
            ),
            "candidate_replace_edge_uid": self._candidate_replace_edge_uid,
            "constraints": [self._serialize_manual_constraint(constraint) for constraint in self.constraints],
            "disabled_loop_changes": {
                str(edge_uid): {
                    "edge_uid": change.edge_uid,
                    "enabled": change.enabled,
                    "accepted_rev": change.accepted_rev,
                    "applied_rev": change.applied_rev,
                    "note": change.note,
                }
                for edge_uid, change in self.disabled_loop_changes.items()
            },
        }

    def _serialize_manual_constraint(self, constraint: ManualConstraint) -> dict:
        return {
            "manual_uid": constraint.manual_uid,
            "enabled": constraint.enabled,
            "source_id": constraint.source_id,
            "target_id": constraint.target_id,
            "target_cloud_mode": constraint.target_cloud_mode,
            "target_neighbors": constraint.target_neighbors,
            "min_time_gap_sec": constraint.min_time_gap_sec,
            "target_map_voxel_size": constraint.target_map_voxel_size,
            "transform_world_source_final": constraint.transform_world_source_final.tolist(),
            "transform_target_source_final": constraint.transform_target_source_final.tolist(),
            "fitness": constraint.fitness,
            "inlier_rmse": constraint.inlier_rmse,
            "variance_t_m2": list(constraint.variance_t_m2),
            "variance_r_rad2": list(constraint.variance_r_rad2),
            "replaces_edge_uid": constraint.replaces_edge_uid,
            "accepted_rev": constraint.accepted_rev,
            "applied_rev": constraint.applied_rev,
            "note": constraint.note,
        }

    def _deserialize_manual_constraint(self, payload: dict) -> ManualConstraint:
        transform_world_source_final = np.asarray(
            payload["transform_world_source_final"], dtype=np.float64
        )
        transform_target_source_final = np.asarray(
            payload["transform_target_source_final"], dtype=np.float64
        )
        source_local = load_xyz_points(self.session_paths.keyframe_dir / f"{int(payload['source_id'])}.pcd")
        source_points_world_final = transform_points(source_local, transform_world_source_final)
        return ManualConstraint(
            manual_uid=int(payload["manual_uid"]),
            enabled=bool(payload["enabled"]),
            source_id=int(payload["source_id"]),
            target_id=int(payload["target_id"]),
            target_cloud_mode=str(payload["target_cloud_mode"]),
            target_neighbors=int(payload["target_neighbors"]),
            min_time_gap_sec=float(payload["min_time_gap_sec"]),
            target_map_voxel_size=float(payload["target_map_voxel_size"]),
            transform_world_source_final=transform_world_source_final,
            transform_target_source_final=transform_target_source_final,
            source_points_world_final=source_points_world_final,
            fitness=float(payload["fitness"]),
            inlier_rmse=float(payload["inlier_rmse"]),
            variance_t_m2=tuple(float(v) for v in payload["variance_t_m2"]),
            variance_r_rad2=tuple(float(v) for v in payload["variance_r_rad2"]),
            replaces_edge_uid=(
                None if payload.get("replaces_edge_uid") is None else int(payload["replaces_edge_uid"])
            ),
            accepted_rev=int(payload.get("accepted_rev", 0)),
            applied_rev=(
                None if payload.get("applied_rev") is None else int(payload["applied_rev"])
            ),
            note=str(payload.get("note", "")),
        )

    def _write_json(self, path: Optional[Path], payload: dict) -> None:
        if path is None:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    def _append_operation_entry(
        self,
        *,
        message: str,
        event: str = "log",
        payload: Optional[dict] = None,
    ) -> None:
        if self._project_ops_path is None:
            return
        record = {
            "ts": datetime.now().isoformat(timespec="seconds"),
            "event": event,
            "message": message,
        }
        if payload:
            record["payload"] = payload
        with self._project_ops_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _save_project_state(self) -> None:
        if self._project_state_path is None or self.session_paths is None:
            return
        payload = {
            "version": 1,
            "updated_at": datetime.now().isoformat(timespec="seconds"),
            **self._project_payload_summary(),
        }
        self._write_json(self._project_state_path, payload)
        latest_pointer = self._latest_project_pointer_path()
        if latest_pointer is not None and self._project_dir is not None:
            self._write_json(
                latest_pointer,
                {
                    "project_id": self._project_id,
                    "project_dir": str(self._project_dir),
                    "updated_at": datetime.now().isoformat(timespec="seconds"),
                },
            )

    def _write_run_context(self, output_dir: Path) -> None:
        payload = {
            "project_id": self._project_id,
            "project_dir": str(self._project_dir) if self._project_dir is not None else None,
            "project_state": str(self._project_state_path) if self._project_state_path is not None else None,
            "execution_log": str(self._project_log_path) if self._project_log_path is not None else None,
            "operations_log": str(self._project_ops_path) if self._project_ops_path is not None else None,
            "created_at": datetime.now().isoformat(timespec="seconds"),
        }
        self._write_json(output_dir / "run_context.json", payload)

    def _write_export_manifest(self, export_dir: Path, run_dir: Path) -> None:
        manifest = {
            "exported_at": datetime.now().isoformat(timespec="seconds"),
            "project_id": self._project_id,
            "project_dir": str(self._project_dir) if self._project_dir is not None else None,
            "run_dir": str(run_dir),
            "run_context": str(run_dir / "run_context.json"),
            "report_json": str(run_dir / "manual_loop_report.json"),
            "pose_graph_g2o": str(run_dir / "pose_graph.g2o"),
            "optimized_tum": str(run_dir / "optimized_poses_tum.txt"),
            "global_map_pcd": str(run_dir / "global_map_manual_imu.pcd"),
            "trajectory_pcd": str(run_dir / "trajectory.pcd"),
        }
        self._write_json(export_dir / "export_manifest.json", manifest)
        (export_dir / "selected_run.txt").write_text(str(run_dir) + "\n", encoding="utf-8")
        symlink_path = export_dir / "run"
        try:
            if symlink_path.exists() or symlink_path.is_symlink():
                symlink_path.unlink()
            symlink_path.symlink_to(run_dir)
        except OSError:
            pass
        exports_root = export_dir.parent
        self._write_json(
            exports_root / "latest_export.json",
            {
                "export_dir": str(export_dir),
                "run_dir": str(run_dir),
                "project_id": self._project_id,
                "updated_at": datetime.now().isoformat(timespec="seconds"),
            },
        )

    def _ensure_run_map_outputs(self, run_dir: Path) -> None:
        output_map = run_dir / "global_map_manual_imu.pcd"
        output_trajectory = run_dir / "trajectory.pcd"
        if output_map.is_file() and output_trajectory.is_file():
            return
        if self.session_paths is None:
            raise RuntimeError("Session paths are unavailable for map export.")
        optimized_tum = run_dir / "optimized_poses_tum.txt"
        if not optimized_tum.is_file():
            raise RuntimeError(f"Missing optimized TUM for export: {optimized_tum}")

        self.append_log(
            f"Export building final map from {optimized_tum.name} with voxel={self.export_map_voxel_spin.value():.3f} m"
        )
        map_point_count, trajectory_count, elapsed = build_map_and_trajectory_from_tum(
            tum_path=optimized_tum,
            keyframe_dir=self.session_paths.keyframe_dir,
            output_map=output_map,
            output_trajectory=output_trajectory,
            voxel_leaf=float(self.export_map_voxel_spin.value()),
            log_fn=self.append_log,
        )
        report_path = run_dir / "manual_loop_report.json"
        update_report_map_fields(
            report_path,
            map_point_count=map_point_count,
            map_build_elapsed_sec=elapsed,
        )
        self.append_log(
            f"Export map build finished for {run_dir.name}: map_points={map_point_count}, "
            f"trajectory_points={trajectory_count}, elapsed={elapsed:.2f}s"
        )

    def _load_project_state_from_dir(self, project_dir: Path, paths: SessionPaths) -> bool:
        try:
            state_path = project_dir / "project_state.json"
            if not state_path.is_file():
                return False
            payload = json.loads(state_path.read_text(encoding="utf-8"))
            if str(payload.get("session_root", "")) != str(paths.session_root):
                return False
            self._project_dir = project_dir
            self._project_state_path = state_path
            self._project_log_path = project_dir / "execution.log"
            self._project_ops_path = project_dir / "operations.jsonl"
            self._project_id = str(payload.get("project_id", project_dir.name))
            self._remember_project_state_path(state_path)
            if self._project_log_path.is_file():
                self.log_text.setPlainText(self._project_log_path.read_text(encoding="utf-8"))
                self.log_text.moveCursor(QtGui.QTextCursor.End)
            self.constraints = [
                self._deserialize_manual_constraint(item)
                for item in payload.get("constraints", [])
            ]
            self.disabled_loop_changes = {
                int(edge_uid): ExistingLoopChange(
                    edge_uid=int(change["edge_uid"]),
                    enabled=bool(change["enabled"]),
                    accepted_rev=int(change.get("accepted_rev", 0)),
                    applied_rev=(
                        None if change.get("applied_rev") is None else int(change["applied_rev"])
                    ),
                    note=str(change.get("note", "")),
                )
                for edge_uid, change in payload.get("disabled_loop_changes", {}).items()
            }
            self._working_revision = int(payload.get("working_revision", 0))
            self._session_dirty = bool(payload.get("session_dirty", False))
            self._last_output_dir = (
                Path(payload["last_output_dir"]).expanduser()
                if payload.get("last_output_dir")
                else None
            )
            self._latest_export_dir = (
                Path(payload["latest_export_dir"]).expanduser()
                if payload.get("latest_export_dir")
                else None
            )
            self.pick_mode = str(payload.get("pick_mode", "nodes"))
            self.source_id = payload.get("source_id")
            self.target_id = payload.get("target_id")
            edge_ref_payload = payload.get("selected_edge_ref")
            self.selected_edge_ref = (
                SelectedEdgeRef(
                    edge_kind=str(edge_ref_payload["edge_kind"]),
                    edge_uid=int(edge_ref_payload["edge_uid"]),
                )
                if edge_ref_payload is not None
                else None
            )
            self._candidate_replace_edge_uid = payload.get("candidate_replace_edge_uid")

            self.pose_graph = copy.deepcopy(self.original_pose_graph)
            for edge_uid, change in self.disabled_loop_changes.items():
                edge = self.pose_graph.edge_index_by_uid.get(edge_uid)
                if edge is not None:
                    edge.enabled = not change.enabled

            if self._last_output_dir is not None:
                restored_tum = self._last_output_dir / "optimized_poses_tum.txt"
                if restored_tum.is_file():
                    self.trajectory = load_tum_trajectory(restored_tum)
                    self.workspace = RegistrationWorkspace(paths.keyframe_dir, self.trajectory)
            self._next_manual_uid = (
                max((constraint.manual_uid for constraint in self.constraints), default=0) + 1
            )
            return True
        except Exception:
            return False

    def _load_latest_project_state(self, paths: SessionPaths) -> bool:
        pointer_path = paths.session_root / "manual_loop_projects" / "latest_project.json"
        if not pointer_path.is_file():
            return False
        try:
            pointer = json.loads(pointer_path.read_text(encoding="utf-8"))
            project_dir = Path(pointer["project_dir"]).expanduser()
        except Exception:
            return False
        return self._load_project_state_from_dir(project_dir, paths)

    def _start_or_resume_project(self, paths: SessionPaths) -> None:
        self.log_text.clear()
        if self._requested_project_dir is not None:
            requested_project_dir = self._requested_project_dir
            self._requested_project_dir = None
            if self._load_project_state_from_dir(requested_project_dir, paths):
                self.append_log(
                    f"Opened edit project {self._project_id} from {self._project_dir}",
                    event="project_open",
                )
                return
            self.append_log(
                f"Requested project {requested_project_dir} could not be restored. Falling back to latest project or new project.",
                event="project_open_failed",
            )
        if self._load_latest_project_state(paths):
            self.append_log(
                f"Resumed edit project {self._project_id} from {self._project_dir}",
                event="project_resume",
            )
            return

        project_root = paths.session_root / "manual_loop_projects"
        project_root.mkdir(parents=True, exist_ok=True)
        project_id = self._new_project_id()
        project_dir = project_root / project_id
        project_dir.mkdir(parents=True, exist_ok=True)
        self._project_dir = project_dir
        self._project_state_path = project_dir / "project_state.json"
        self._project_log_path = project_dir / "execution.log"
        self._project_ops_path = project_dir / "operations.jsonl"
        self._project_id = project_id
        self._write_json(
            project_root / "latest_project.json",
            {
                "project_id": project_id,
                "project_dir": str(project_dir),
                "updated_at": datetime.now().isoformat(timespec="seconds"),
            },
        )
        self._save_project_state()
        self.append_log(
            f"Started new edit project {project_id} at {project_dir}",
            event="project_start",
        )

    def append_log(
        self,
        message: str,
        *,
        event: str = "log",
        payload: Optional[dict] = None,
    ) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{timestamp}] {message}"
        self.log_text.appendPlainText(line)
        if self._project_log_path is not None:
            self._project_log_path.parent.mkdir(parents=True, exist_ok=True)
            with self._project_log_path.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")
        self._append_operation_entry(message=message, event=event, payload=payload)

    def _browse_session_root(self) -> None:
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "Select Session Root",
            self._session_root_dialog_directory(),
        )
        if directory:
            self.session_root_edit.setText(directory)
            current_g2o_text = self.g2o_edit.text().strip()
            if current_g2o_text:
                current_g2o = Path(current_g2o_text).expanduser()
                if current_g2o.exists() and not self._is_g2o_under_session_root(Path(directory), current_g2o):
                    self.g2o_edit.clear()
            self._settings.setValue("browser/last_session_root", directory)
            self._settings.sync()

    def _browse_g2o(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Select pose_graph.g2o",
            self._g2o_dialog_directory(),
            filter="G2O Files (*.g2o);;All Files (*)",
        )
        if path:
            self.g2o_edit.setText(path)
            self._settings.setValue("browser/last_g2o_path", path)
            self._settings.sync()

    def _browse_project_state(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Open Edit Project",
            self._project_state_dialog_directory(),
            filter="Project State (project_state.json);;JSON (*.json);;All Files (*)",
        )
        if not path:
            return
        project_state_path = Path(path).expanduser()
        try:
            payload = json.loads(project_state_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self._show_error("Open Project Failed", f"Failed to read project state:\n{exc}")
            return
        session_root = payload.get("session_root")
        g2o_path = payload.get("g2o_path")
        if not session_root:
            self._show_error("Open Project Failed", "project_state.json does not contain session_root.")
            return
        self.session_root_edit.setText(str(session_root))
        self.g2o_edit.setText(str(g2o_path or ""))
        self._remember_project_state_path(project_state_path)
        self._requested_project_dir = project_state_path.parent
        self.load_session()

    def _set_pick_mode(self, mode: str) -> None:
        self._release_plot_toolbar_navigation()
        self.pick_mode = mode
        self.trajectory_canvas.set_interaction_mode(mode)
        self.pick_nodes_button.setChecked(mode == "nodes")
        self.pick_edges_button.setChecked(mode == "edges")
        self._update_plot_help_state()

    def _handle_plot_hover(self, kind: str, payload) -> None:
        if kind == "node" and payload is not None and self.trajectory is not None:
            pose = self.trajectory.positions_xyz[payload]
            timestamp = self.trajectory.timestamps[payload]
            self.hover_label.setText(
                f"Hover: node {payload} | t={timestamp:.3f} | xyz=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})"
            )
            return

        if kind == "edge" and payload is not None:
            edge = self._edge_from_ref(payload)
            if edge is not None:
                self.hover_label.setText(f"Hover: {edge.summary()}")
                return

        self.hover_label.setText("Hover: none")

    def _handle_plot_selection(self, kind: str, payload) -> None:
        if not self._is_working_view():
            return
        if kind == "node":
            self._handle_node_pick(int(payload))
            return
        if kind == "edge":
            self._select_edge(payload, reset_camera=True)

    def _handle_node_pick(self, node_id: int) -> None:
        if self.pose_graph is None:
            return

        if self.source_id is None or self.target_id is not None or self.selected_edge_ref is not None:
            self.source_id = node_id
            self.target_id = None
            self.selected_edge_ref = None
            self._candidate_replace_edge_uid = None
            self.current_preview = None
            self.last_result = None
            self.accept_button.setEnabled(False)
            self._set_display_mode("Preview")
            self.gicp_metrics_label.setText("Src selected · pick target.")
            self.append_log(f"Picked first node {node_id}")
        else:
            if node_id == self.source_id:
                self.append_log("Source and target cannot be the same node.")
                return
            first_node = self.source_id
            self.source_id = max(first_node, node_id)
            self.target_id = min(first_node, node_id)
            self.selected_edge_ref = None
            self._candidate_replace_edge_uid = None
            self.current_preview = None
            self.last_result = None
            self.accept_button.setEnabled(False)
            self._set_display_mode("Preview")
            self.gicp_metrics_label.setText("Pair ready · preview refreshed.")
            self.append_log(
                f"Picked node pair target={self.target_id}, source={self.source_id}"
            )
            self.schedule_preview_refresh(reset_camera=True)

        self._update_selection_labels()
        self._update_edge_action_buttons()
        self._sync_constraint_table_selection()
        self._refresh_plot(preserve_view=True)
        self._save_project_state()

    def _select_edge(self, edge_ref: SelectedEdgeRef, *, reset_camera: bool) -> None:
        edge = self._edge_from_ref(edge_ref)
        if edge is None:
            return

        self.selected_edge_ref = edge_ref
        self.source_id = edge.source_id
        self.target_id = edge.target_id
        self._candidate_replace_edge_uid = edge.edge_uid if edge.edge_type == "loop_existing" else None
        self.last_result = None
        self.accept_button.setEnabled(False)
        self._update_selection_labels()
        self._update_edge_action_buttons()
        self._refresh_plot(preserve_view=True)
        self._sync_constraint_table_selection()

        if edge_ref.edge_kind == "manual_added":
            constraint = self._constraint_by_uid(edge_ref.edge_uid)
            if constraint is not None:
                self._preview_manual_constraint(constraint, reset_camera=reset_camera)
            return

        self._preview_existing_edge(edge, reset_camera=reset_camera)

    def _preview_existing_edge(self, edge: EdgeRecord, *, reset_camera: bool) -> None:
        if self.workspace is None:
            return
        try:
            self._set_display_mode("Preview")
            config = self._current_registration_config()
            delta_transform_local = self._current_delta_transform()
            preview = self.workspace.build_preview(
                source_id=edge.source_id,
                target_id=edge.target_id,
                delta_transform_local=delta_transform_local,
                target_cloud_mode=config.target_cloud_mode,
                target_neighbors=config.target_neighbors,
                min_time_gap_sec=config.min_time_gap_sec,
                target_map_voxel_size=config.target_map_voxel_size,
            )
            self.last_result = None
            self.accept_button.setEnabled(False)
            self.current_preview = preview
            self._update_preview_summary(preview)
            if edge.edge_type == "loop_existing":
                self.gicp_metrics_label.setText(
                    f"Loop selected · {edge.target_id}->{edge.source_id}"
                    "<br><span style='color:#64748b;font-size:11px'>Adjust delta, run GICP, then replace.</span>"
                )
            else:
                self.gicp_metrics_label.setText(
                    f"Odom selected · {edge.target_id}->{edge.source_id}"
                    "<br><span style='color:#64748b;font-size:11px'>Inspect only.</span>"
                )
            self._show_scene(
                self._make_preview_scene(
                    preview=preview,
                    initial_source_points=preview.source_points_world_initial,
                    adjusted_source_points=preview.source_points_world_adjusted,
                    transform_world_source_initial=preview.transform_world_source_initial,
                    transform_world_source_adjusted=preview.transform_world_source_adjusted,
                ),
                scene_key=(
                    "existing",
                    edge.edge_uid,
                    preview.target_cloud_mode,
                    preview.target_neighbors,
                    round(preview.min_time_gap_sec, 6),
                    round(preview.target_map_voxel_size, 6),
                ),
                reset_camera=reset_camera,
            )
            self._refresh_plot(preserve_view=True)
            self.append_log(f"Selected edge {edge.summary()}")
            self._save_project_state()
        except Exception as exc:
            self._show_error("Preview Edge Failed", f"{exc}\n\n{traceback.format_exc()}")

    def _preview_manual_constraint(self, constraint: ManualConstraint, *, reset_camera: bool) -> None:
        if self.workspace is None:
            return
        try:
            self._set_display_mode("Final")
            preview = self.workspace.build_preview(
                source_id=constraint.source_id,
                target_id=constraint.target_id,
                delta_transform_local=np.eye(4, dtype=np.float64),
                target_cloud_mode=constraint.target_cloud_mode,
                target_neighbors=constraint.target_neighbors,
                min_time_gap_sec=constraint.min_time_gap_sec,
                target_map_voxel_size=constraint.target_map_voxel_size,
            )
            self.current_preview = preview
            self._update_preview_summary(preview)
            self.gicp_metrics_label.setText(
                f"Manual edge · fit {constraint.fitness:.4f} · rmse {constraint.inlier_rmse:.4f}"
            )
            self._show_scene(
                self._make_preview_scene(
                    preview=preview,
                    initial_source_points=preview.source_points_world_initial,
                    final_source_points=constraint.source_points_world_final,
                    transform_world_source_initial=preview.transform_world_source_initial,
                    transform_world_source_final=constraint.transform_world_source_final,
                ),
                scene_key=(
                    "manual",
                    constraint.manual_uid,
                    constraint.target_cloud_mode,
                    constraint.target_neighbors,
                    round(constraint.min_time_gap_sec, 6),
                    round(constraint.target_map_voxel_size, 6),
                ),
                reset_camera=reset_camera,
            )
            self._refresh_plot(preserve_view=True)
            self.append_log(
                f"Selected manual constraint {constraint.target_id}->{constraint.source_id}"
            )
            self._save_project_state()
        except Exception as exc:
            self._show_error("Preview Manual Constraint Failed", f"{exc}\n\n{traceback.format_exc()}")

    def _show_scene(
        self,
        scene: PreviewScene,
        *,
        scene_key: tuple,
        reset_camera: bool,
        force_preserve_camera: bool = False,
    ) -> None:
        if force_preserve_camera:
            should_reset = bool(reset_camera)
        else:
            should_reset = reset_camera or scene_key != self._preview_scene_key
        previous_camera = None if should_reset else self.cloud_view.capture_camera_state()
        self.cloud_view.update_scene(
            scene,
            reset_camera=should_reset,
            camera_state=previous_camera,
        )
        self._preview_scene_key = scene_key

    def _update_selection_labels(self) -> None:
        self.source_label.setText(self._format_node_info(self.source_id))
        self.source_label.setToolTip(
            "none" if self.source_id is None or self.trajectory is None else
            f"node={self.source_id}\nt={self.trajectory.timestamps[self.source_id]:.6f}s\n"
            f"xyz={self.trajectory.positions_xyz[self.source_id, 0]:.3f}, "
            f"{self.trajectory.positions_xyz[self.source_id, 1]:.3f}, "
            f"{self.trajectory.positions_xyz[self.source_id, 2]:.3f}"
        )
        self.target_label.setText(self._format_node_info(self.target_id))
        self.target_label.setToolTip(
            "none" if self.target_id is None or self.trajectory is None else
            f"node={self.target_id}\nt={self.trajectory.timestamps[self.target_id]:.6f}s\n"
            f"xyz={self.trajectory.positions_xyz[self.target_id, 0]:.3f}, "
            f"{self.trajectory.positions_xyz[self.target_id, 1]:.3f}, "
            f"{self.trajectory.positions_xyz[self.target_id, 2]:.3f}"
        )
        if self.source_id is None or self.target_id is None or self.trajectory is None:
            self.pair_label.setText("none")
            self.pair_label.setToolTip("none")
        else:
            self.pair_label.setText(
                f"T {self.target_id} · S {self.source_id}"
                f"<br><span style='color:#64748b;font-size:11px'>"
                f"{self.trajectory.positions_xyz[self.target_id, 0]:.2f}, {self.trajectory.positions_xyz[self.target_id, 1]:.2f} → "
                f"{self.trajectory.positions_xyz[self.source_id, 0]:.2f}, {self.trajectory.positions_xyz[self.source_id, 1]:.2f}"
                "</span>"
            )
            self.pair_label.setToolTip(
                "target:\n"
                f"node={self.target_id}\nt={self.trajectory.timestamps[self.target_id]:.6f}s\n"
                f"xyz={self.trajectory.positions_xyz[self.target_id, 0]:.3f}, "
                f"{self.trajectory.positions_xyz[self.target_id, 1]:.3f}, "
                f"{self.trajectory.positions_xyz[self.target_id, 2]:.3f}\n\n"
                "source:\n"
                f"node={self.source_id}\nt={self.trajectory.timestamps[self.source_id]:.6f}s\n"
                f"xyz={self.trajectory.positions_xyz[self.source_id, 0]:.3f}, "
                f"{self.trajectory.positions_xyz[self.source_id, 1]:.3f}, "
                f"{self.trajectory.positions_xyz[self.source_id, 2]:.3f}"
            )
        if self.selected_edge_ref is None:
            self.selected_edge_label.setText("none")
            self.selected_edge_label.setToolTip("none")
        else:
            edge = self._edge_from_ref(self.selected_edge_ref)
            if edge is None:
                self.selected_edge_label.setText("none")
                self.selected_edge_label.setToolTip("none")
            else:
                state = "on" if edge.enabled else "off"
                type_text = (
                    "Loop" if edge.edge_type == "loop_existing"
                    else "Odom" if edge.edge_type == "odom"
                    else "Manual"
                )
                self.selected_edge_label.setText(f"{type_text} · {edge.target_id}->{edge.source_id} · {state}")
                self.selected_edge_label.setToolTip(edge.summary())

    def _update_preview_summary(self, preview: Optional[RegistrationPreview]) -> None:
        self.current_preview = preview
        if preview is None:
            self.target_map_label.setText("No target submap.")
            return

        if preview.target_cloud_mode == TARGET_CLOUD_MODE_TEMPORAL_WINDOW:
            mode_text = f"TW±{preview.target_neighbors}"
            filter_text = "off"
        elif preview.time_gap_filter_enabled:
            mode_text = f"RS{preview.target_neighbors}"
            filter_text = "on" if preview.time_gap_filter_applied else "idle"
        else:
            mode_text = f"RS{preview.target_neighbors}"
            filter_text = "off"
        frame_range = preview.target_frame_range
        range_text = "n/a" if frame_range is None else f"{frame_range[0]}..{frame_range[1]}"
        if preview.target_cloud_mode == TARGET_CLOUD_MODE_TEMPORAL_WINDOW:
            clip_text = "clipped" if preview.target_window_clipped else "full"
        else:
            clip_text = "n/a"
        self.target_map_label.setText(
            f"{mode_text} · {preview.target_frame_count}f · {preview.target_point_count} pts"
            f"<br><span style='color:#64748b;font-size:11px'>r {range_text} · b {clip_text} · g {filter_text}</span>"
        )
        self.target_map_label.setToolTip(
            f"mode={mode_text}\nframes={preview.target_frame_count}\nrange={range_text}\n"
            f"boundary={clip_text}\ntime_gap={filter_text}\npoints={preview.target_point_count}"
        )

    def _set_display_mode(self, text: str) -> None:
        if self.display_mode_combo.currentText() == text:
            return
        with QtCore.QSignalBlocker(self.display_mode_combo):
            self.display_mode_combo.setCurrentText(text)
        self._update_cloud_display_controls()

    def _format_node_info(self, node_id: Optional[int]) -> str:
        if node_id is None or self.trajectory is None:
            return "none"
        pose = self.trajectory.positions_xyz[node_id]
        timestamp = self.trajectory.timestamps[node_id]
        return (
            f"{node_id} · ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}) · t={timestamp:.3f}s"
        )

    def _refresh_plot(self, *, preserve_view: bool) -> None:
        plot_pose_graph = self.pose_graph if self._is_working_view() else self.original_pose_graph
        plot_trajectory = self.trajectory if self._is_working_view() else self.original_trajectory
        plot_constraints = self.constraints if self._is_working_view() else []
        selected_edge_ref = self.selected_edge_ref if self._is_working_view() else None

        positions_xy = (
            plot_trajectory.positions_xyz[:, :2]
            if plot_trajectory is not None
            else None
        )
        ghost_positions_xy = None
        if self.show_ghost_check.isChecked():
            ghost_trajectory = self.original_trajectory if self._is_working_view() else self.trajectory
            if ghost_trajectory is not None:
                ghost_positions_xy = ghost_trajectory.positions_xyz[:, :2]
        self.trajectory_canvas.set_interaction_mode(self.pick_mode)
        self.trajectory_canvas.set_plot_data(
            positions_xy=positions_xy,
            ghost_positions_xy=ghost_positions_xy,
            selected_target_positions_xy=self._selected_target_positions_xy(),
            pose_graph=plot_pose_graph,
            constraints=plot_constraints,
            source_id=self.source_id,
            target_id=self.target_id,
            selected_edge_ref=selected_edge_ref,
            preserve_view=preserve_view,
        )

    def _on_trajectory_view_changed(self, *_args) -> None:
        if self._is_working_view():
            loop_count = len(self.pose_graph.loop_edges) if self.pose_graph is not None else 0
            pose_count = self.trajectory.size if self.trajectory is not None else 0
            self.trajectory_view_info_label.setText(
                f"P{pose_count} · L{loop_count} · edit"
            )
            self.pick_nodes_button.setEnabled(True)
            self.pick_edges_button.setEnabled(True)
        else:
            loop_count = len(self.original_pose_graph.loop_edges) if self.original_pose_graph is not None else 0
            pose_count = self.original_trajectory.size if self.original_trajectory is not None else 0
            self.trajectory_view_info_label.setText(
                f"P{pose_count} · L{loop_count} · read only"
            )
            self.pick_nodes_button.setEnabled(False)
            self.pick_edges_button.setEnabled(False)
            self._release_plot_toolbar_navigation()
        self._update_plot_help_state()
        self._update_trajectory_legend_label()
        self._update_edge_action_buttons()
        self._refresh_plot(preserve_view=False)

    def _edge_from_ref(self, edge_ref: Optional[SelectedEdgeRef]) -> Optional[EdgeRecord]:
        if edge_ref is None:
            return None
        if edge_ref.edge_kind == "existing" and self.pose_graph is not None:
            return self.pose_graph.edge_index_by_uid.get(edge_ref.edge_uid)
        constraint = self._constraint_by_uid(edge_ref.edge_uid)
        if constraint is not None:
            return constraint.as_edge_record()
        return None

    def _constraint_by_uid(self, manual_uid: int) -> Optional[ManualConstraint]:
        for constraint in self.constraints:
            if constraint.manual_uid == manual_uid:
                return constraint
        return None

    def _constraint_by_pair(self, source_id: int, target_id: int) -> Optional[ManualConstraint]:
        for constraint in self.constraints:
            if constraint.source_id == source_id and constraint.target_id == target_id:
                return constraint
        return None

    def _disabled_loop_change(self, edge_uid: int) -> Optional[ExistingLoopChange]:
        return self.disabled_loop_changes.get(edge_uid)

    def _matches_graph_change_filters(self, type_text: str, status_text: str) -> bool:
        status_filter = self._graph_change_status_filter
        type_filter = self._graph_change_type_filter
        if status_filter not in {"", "All Status"} and status_text != status_filter:
            return False
        if type_filter not in {"", "All Types"} and type_text != type_filter:
            return False
        return True

    def _graph_change_table_rows(self) -> list[tuple[str, int, dict[str, str]]]:
        rows: list[tuple[str, int, dict[str, str]]] = []
        replaced_edge_uids = {
            constraint.replaces_edge_uid
            for constraint in self.constraints
            if constraint.replaces_edge_uid is not None
        }
        for constraint in self.constraints:
            status = self._status_text(constraint.enabled, constraint.applied_rev)
            frame_range = "-"
            if constraint.target_cloud_mode == TARGET_CLOUD_MODE_TEMPORAL_WINDOW:
                frame_range = f"+/-{constraint.target_neighbors}"
            type_text = (
                "Replace Existing Loop"
                if constraint.replaces_edge_uid is not None
                else "Manual Add"
            )
            if not self._matches_graph_change_filters(type_text, status):
                continue
            noise_text = (
                f"T:{'/'.join(f'{value:.2f}' for value in constraint.variance_t_m2)} "
                f"R:{'/'.join(f'{value:.2f}' for value in constraint.variance_r_rad2)}"
            )
            rows.append(
                (
                    "manual",
                    constraint.manual_uid,
                    {
                        "use": "1" if constraint.enabled else "0",
                        "type": type_text,
                        "status": status,
                        "src": str(constraint.source_id),
                        "tgt": str(constraint.target_id),
                        "submap": (
                            f"TW±{constraint.target_neighbors}"
                            if constraint.target_cloud_mode == TARGET_CLOUD_MODE_TEMPORAL_WINDOW
                            else f"RS{constraint.target_neighbors}"
                        ),
                        "range": frame_range,
                        "fitness": f"{constraint.fitness:.4f}",
                        "rmse": f"{constraint.inlier_rmse:.4f}",
                        "noise": noise_text,
                        "accepted": f"Rev {constraint.accepted_rev}",
                        "applied": "-" if constraint.applied_rev is None else f"Rev {constraint.applied_rev}",
                        "note": constraint.note,
                    },
                )
            )

        for edge_uid, change in sorted(self.disabled_loop_changes.items()):
            if edge_uid in replaced_edge_uids:
                continue
            edge = self.pose_graph.edge_index_by_uid.get(edge_uid) if self.pose_graph is not None else None
            if edge is None:
                continue
            status = self._status_text(change.enabled, change.applied_rev)
            type_text = "Disable Existing Loop"
            if not self._matches_graph_change_filters(type_text, status):
                continue
            rows.append(
                (
                    "disable_loop",
                    edge_uid,
                    {
                        "use": "1" if change.enabled else "0",
                        "type": type_text,
                        "status": status,
                        "src": str(edge.source_id),
                        "tgt": str(edge.target_id),
                        "submap": "-",
                        "range": "-",
                        "fitness": "-",
                        "rmse": "-",
                        "noise": "-",
                        "accepted": f"Rev {change.accepted_rev}",
                        "applied": "-" if change.applied_rev is None else f"Rev {change.applied_rev}",
                        "note": change.note,
                    },
                )
            )
        return rows

    def _status_brushes(self, status: str) -> tuple[QtGui.QBrush, QtGui.QBrush]:
        if status == "Applied":
            return (
                QtGui.QBrush(QtGui.QColor("#e7f8ee")),
                QtGui.QBrush(QtGui.QColor("#1f6f43")),
            )
        if status == "Accepted":
            return (
                QtGui.QBrush(QtGui.QColor("#fff4db")),
                QtGui.QBrush(QtGui.QColor("#8a5300")),
            )
        return (
            QtGui.QBrush(QtGui.QColor("#f1f5f9")),
            QtGui.QBrush(QtGui.QColor("#64748b")),
        )

    def _current_delta_transform(self) -> np.ndarray:
        return build_delta_transform(
            self.delta_spins["x"].value(),
            self.delta_spins["y"].value(),
            self.delta_spins["z"].value(),
            self.delta_spins["roll"].value(),
            self.delta_spins["pitch"].value(),
            self.delta_spins["yaw"].value(),
        )

    def _current_registration_config(self) -> RegistrationConfig:
        return RegistrationConfig(
            target_cloud_mode=self._current_target_cloud_mode(),
            target_neighbors=self.target_neighbors_spin.value(),
            min_time_gap_sec=self.target_min_gap_spin.value(),
            target_map_voxel_size=self.target_map_voxel_spin.value(),
            voxel_size=self.voxel_spin.value(),
            max_correspondence_distance=self.max_corr_spin.value(),
            max_iterations=self.max_iter_spin.value(),
        )

    def _current_variances(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        translation_value = float(self.variance_t_shared_spin.value())
        rotation_value = float(self.variance_r_shared_spin.value())
        return (
            (translation_value, translation_value, translation_value),
            (rotation_value, rotation_value, rotation_value),
        )

    def _selected_target_positions_xy(self) -> Optional[np.ndarray]:
        if (
            not self._is_working_view()
            or self.current_preview is None
            or self.trajectory is None
            or not self.current_preview.target_frame_indices
        ):
            return None
        indices = np.asarray(self.current_preview.target_frame_indices, dtype=np.int64)
        indices = indices[(indices >= 0) & (indices < self.trajectory.size)]
        if indices.size == 0:
            return None
        return self.trajectory.positions_xyz[indices, :2]

    def _on_graph_change_filter_changed(self) -> None:
        self._graph_change_status_filter = self.change_status_filter_combo.currentText().strip()
        self._graph_change_type_filter = self.change_type_filter_combo.currentText().strip()
        self._rebuild_constraint_table()
        self._sync_constraint_table_selection()

    def _sync_constraint_table_selection(self) -> None:
        if not hasattr(self, "constraint_table"):
            return
        with QtCore.QSignalBlocker(self.constraint_table.selectionModel()):
            self.constraint_table.clearSelection()
            if self.selected_edge_ref is None:
                return
            if self.selected_edge_ref.edge_kind == "existing":
                for row, (row_kind, row_uid) in enumerate(self._graph_change_rows):
                    if row_kind != "manual":
                        continue
                    constraint = self._constraint_by_uid(row_uid)
                    if constraint is not None and constraint.replaces_edge_uid == self.selected_edge_ref.edge_uid:
                        self.constraint_table.selectRow(row)
                        return
            for row, (row_kind, row_uid) in enumerate(self._graph_change_rows):
                if self.selected_edge_ref.edge_kind == "manual_added" and row_kind == "manual" and row_uid == self.selected_edge_ref.edge_uid:
                    self.constraint_table.selectRow(row)
                    break
                if self.selected_edge_ref.edge_kind == "existing" and row_kind == "disable_loop" and row_uid == self.selected_edge_ref.edge_uid:
                    self.constraint_table.selectRow(row)
                    break

    def _update_edge_action_buttons(self) -> None:
        edge = self._edge_from_ref(self.selected_edge_ref)
        editable = self._is_working_view()
        self.disable_edge_button.setEnabled(bool(editable and edge and edge.edge_type == "loop_existing" and edge.enabled))
        self.restore_edge_button.setEnabled(bool(editable and edge and edge.edge_type == "loop_existing" and not edge.enabled))
        self.remove_manual_button.setEnabled(bool(editable and edge and edge.edge_type == "manual_added"))
        pair_ready = bool(editable and self.workspace is not None and self.source_id is not None and self.target_id is not None)
        self.run_gicp_button.setEnabled(pair_ready)
        self.auto_yaw_button.setEnabled(pair_ready)
        self.accept_button.setEnabled(bool(editable and self.last_result is not None))
        replacement_edge = (
            self.pose_graph.edge_index_by_uid.get(self._candidate_replace_edge_uid)
            if editable and self.pose_graph is not None and self._candidate_replace_edge_uid is not None
            else None
        )
        self.replace_edge_button.setEnabled(
            bool(
                editable
                and self.last_result is not None
                and replacement_edge is not None
                and replacement_edge.edge_type == "loop_existing"
            )
        )
        self._update_cloud_interaction_controls()

    def _clear_selection(self) -> None:
        had_selection = (
            self.source_id is not None
            or self.target_id is not None
            or self.selected_edge_ref is not None
        )
        self.source_id = None
        self.target_id = None
        self.selected_edge_ref = None
        self._candidate_replace_edge_uid = None
        self.current_preview = None
        self.last_result = None
        self.accept_button.setEnabled(False)
        self.gicp_metrics_label.setText("No GICP result.")
        self._preview_scene_key = None
        self._update_preview_summary(None)
        self.cloud_view.clear_scene()
        self._update_selection_labels()
        self._update_edge_action_buttons()
        self._sync_constraint_table_selection()
        self._refresh_plot(preserve_view=True)
        self._save_project_state()
        if had_selection:
            self.append_log("Cleared node/edge selection.")

    def _reset_delta(self) -> None:
        blockers = [QtCore.QSignalBlocker(spin) for spin in self.delta_spins.values()]
        try:
            for spin in self.delta_spins.values():
                spin.setValue(0.0)
        finally:
            del blockers
        self._invalidate_last_result_for_manual_align()
        self.schedule_preview_refresh(reset_camera=False)

    def load_session(self) -> None:
        try:
            session_root_text = self.session_root_edit.text().strip()
            g2o_text = self.g2o_edit.text().strip()
            session_root_path = Path(session_root_text) if session_root_text else None
            g2o_path = Path(g2o_text) if g2o_text else None
            paths = None
            stale_g2o_note = None
            if session_root_path is not None and g2o_path is not None:
                resolved_root = session_root_path.expanduser().resolve()
                resolved_g2o = g2o_path.expanduser().resolve()
                if (
                    resolved_root.is_dir()
                    and resolved_g2o.is_file()
                    and not self._is_g2o_under_session_root(resolved_root, resolved_g2o)
                ):
                    paths = resolve_session_paths(session_root=session_root_path)
                    stale_g2o_note = (
                        f"Ignored stale g2o outside session root: {resolved_g2o}. "
                        f"Using {paths.g2o_path} from {paths.session_root}."
                    )
            if paths is None:
                paths = resolve_session_paths(
                    session_root=session_root_path,
                    g2o_path=g2o_path,
                )
            pose_graph = load_pose_graph(paths.g2o_path)
            trajectory = load_tum_trajectory(paths.tum_path)
            keyframe_paths = list_numbered_pcds(paths.keyframe_dir)
            if trajectory.size != len(keyframe_paths):
                raise PcdValidationError(
                    "Keyframe PCD count does not match TUM pose count: "
                    f"{len(keyframe_paths)} vs {trajectory.size}"
                )
            pose_graph, trim_note = align_pose_graph_to_frame_count(pose_graph, trajectory.size)
            validate_keyframe_numbering(keyframe_paths, len(pose_graph.vertex_ids))

            self.session_paths = paths
            self.session_root_edit.setText(str(paths.session_root))
            self.g2o_edit.setText(str(paths.g2o_path))
            self._remember_loaded_paths(paths)
            self.original_pose_graph = pose_graph
            self.original_trajectory = trajectory
            self.pose_graph = copy.deepcopy(pose_graph)
            self.trajectory = copy.deepcopy(trajectory)
            self.workspace = RegistrationWorkspace(paths.keyframe_dir, self.trajectory)
            self.keyframe_paths = keyframe_paths
            self.source_id = None
            self.target_id = None
            self.selected_edge_ref = None
            self._candidate_replace_edge_uid = None
            self.constraints.clear()
            self.disabled_loop_changes.clear()
            self.current_preview = None
            self.last_result = None
            self._next_manual_uid = 1
            self._preview_scene_key = None
            self._working_revision = 0
            self._session_dirty = False
            self._pending_export_after_optimize = False
            self._undo_stack.clear()
            self._last_output_dir = None
            self._latest_export_dir = None
            self.accept_button.setEnabled(False)
            self.gicp_metrics_label.setText("No GICP result.")
            self._set_pick_mode("nodes")
            self._set_display_mode("Preview")
            with QtCore.QSignalBlocker(self.show_world_axis_check):
                self.show_world_axis_check.setChecked(False)
            with QtCore.QSignalBlocker(self.cloud_interaction_mode_combo):
                self.cloud_interaction_mode_combo.setCurrentIndex(
                    self.cloud_interaction_mode_combo.findData(INTERACTION_MODE_CAMERA)
                )
            with QtCore.QSignalBlocker(self.cloud_lock_mode_combo):
                self.cloud_lock_mode_combo.setCurrentIndex(
                    self.cloud_lock_mode_combo.findData(LOCK_MODE_XY_YAW)
                )
            with QtCore.QSignalBlocker(self.cloud_drag_mode_combo):
                self.cloud_drag_mode_combo.setCurrentIndex(
                    self.cloud_drag_mode_combo.findData(EDIT_OPERATION_TRANSLATE)
                )
            with QtCore.QSignalBlocker(self.manual_align_snap_view_check):
                self.manual_align_snap_view_check.setChecked(True)
            with QtCore.QSignalBlocker(self.trajectory_view_combo):
                self.trajectory_view_combo.clear()
                self.trajectory_view_combo.addItems(["Working", "Original"])
                self.trajectory_view_combo.setCurrentText("Working")
            with QtCore.QSignalBlocker(self.change_status_filter_combo):
                self.change_status_filter_combo.setCurrentText("All Status")
            with QtCore.QSignalBlocker(self.change_type_filter_combo):
                self.change_type_filter_combo.setCurrentText("All Types")
            self._graph_change_status_filter = "All Status"
            self._graph_change_type_filter = "All Types"
            self._update_cloud_display_controls()
            self._start_or_resume_project(paths)
            self._update_trajectory_legend_label()
            self.trajectory_view_info_label.setText(
                f"P{self.trajectory.size} · L{len(self.pose_graph.loop_edges)} · edit"
            )
            self.pick_edges_button.setEnabled(True)
            self._release_plot_toolbar_navigation()
            self._update_plot_help_state()
            self.cloud_view.clear_scene()
            self._update_preview_summary(None)
            self._update_selection_labels()
            self._update_edge_action_buttons()
            self._rebuild_constraint_table()
            self._update_session_status_widgets()
            self._refresh_plot(preserve_view=False)
            self._update_cloud_interaction_controls()
            self._save_project_state()

            self.session_info_label.setText(
                f"Loaded session: {paths.session_root} | "
                f"g2o={paths.g2o_path.name} | tum={paths.tum_path.name} | "
                f"keyframes={len(self.keyframe_paths)} | loops={len(self.pose_graph.loop_edges)} | "
                f"project={self._project_id}"
            )
            if stale_g2o_note:
                self.append_log(stale_g2o_note)
            if trim_note:
                self.append_log(trim_note)
            self.append_log(
                f"Loaded session root={paths.session_root}, g2o={paths.g2o_path}, "
                f"tum={paths.tum_path}, keyframes={len(self.keyframe_paths)}, "
                f"existing_loops={len(self.pose_graph.loop_edges)}"
            )
            self._save_project_state()
        except (
            SessionResolutionError,
            PoseGraphValidationError,
            TrajectoryValidationError,
            PcdValidationError,
            OSError,
        ) as exc:
            self._show_error("Load Session Failed", str(exc))
        except Exception as exc:
            self._show_error("Load Session Failed", f"{exc}\n\n{traceback.format_exc()}")

    def schedule_preview_refresh(self, reset_camera: bool = False) -> None:
        if self.workspace is None or self.source_id is None or self.target_id is None:
            return
        self._pending_preview_reset_camera = self._pending_preview_reset_camera or reset_camera
        self._preview_timer.start()

    def refresh_preview(self) -> None:
        if self.workspace is None or self.source_id is None or self.target_id is None:
            return
        reset_camera = self._pending_preview_reset_camera
        self._pending_preview_reset_camera = False
        if self.selected_edge_ref is not None:
            edge = self._edge_from_ref(self.selected_edge_ref)
            if edge is None:
                return
            if self.selected_edge_ref.edge_kind == "manual_added":
                constraint = self._constraint_by_uid(self.selected_edge_ref.edge_uid)
                if constraint is not None:
                    self._preview_manual_constraint(constraint, reset_camera=reset_camera)
            else:
                self._preview_existing_edge(edge, reset_camera=reset_camera)
            return
        try:
            self._set_display_mode("Preview")
            config = self._current_registration_config()
            preview = self.workspace.build_preview(
                source_id=self.source_id,
                target_id=self.target_id,
                delta_transform_local=self._current_delta_transform(),
                target_cloud_mode=config.target_cloud_mode,
                target_neighbors=config.target_neighbors,
                min_time_gap_sec=config.min_time_gap_sec,
                target_map_voxel_size=config.target_map_voxel_size,
            )
            self.current_preview = preview
            self._update_preview_summary(preview)
            self.last_result = None
            self.accept_button.setEnabled(False)
            self.gicp_metrics_label.setText("Preview ready · run GICP.")
            self._show_scene(
                self._make_preview_scene(
                    preview=preview,
                    initial_source_points=preview.source_points_world_initial,
                    adjusted_source_points=preview.source_points_world_adjusted,
                    transform_world_source_initial=preview.transform_world_source_initial,
                    transform_world_source_adjusted=preview.transform_world_source_adjusted,
                ),
                scene_key=(
                    "pair",
                    self.source_id,
                    self.target_id,
                    preview.target_cloud_mode,
                    preview.target_neighbors,
                    round(preview.min_time_gap_sec, 6),
                    round(preview.target_map_voxel_size, 6),
                ),
                reset_camera=reset_camera,
            )
            self._refresh_plot(preserve_view=True)
        except Exception as exc:
            self._show_error("Preview Failed", f"{exc}\n\n{traceback.format_exc()}")

    def run_gicp(self) -> None:
        if self.workspace is None or self.source_id is None or self.target_id is None:
            self._show_error("Run GICP", "Select source and target nodes first.")
            return

        replace_edge_uid = None
        if self.selected_edge_ref is not None and self.selected_edge_ref.edge_kind == "existing":
            edge = self._edge_from_ref(self.selected_edge_ref)
            if edge is not None and edge.edge_type == "loop_existing":
                replace_edge_uid = edge.edge_uid

        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            self.selected_edge_ref = None
            self._candidate_replace_edge_uid = replace_edge_uid
            self._update_selection_labels()
            self._sync_constraint_table_selection()
            if self.current_preview is not None:
                self.append_log(
                    f"Run GICP target={self.target_id}, source={self.source_id} | "
                    f"{self._gicp_log_cloud_summary(self.current_preview)}"
                )
            result = self.workspace.run_gicp(
                source_id=self.source_id,
                target_id=self.target_id,
                delta_transform_local=self._current_delta_transform(),
                config=self._current_registration_config(),
            )
            self._set_display_mode("Final")
            self.current_preview = result.preview
            self._update_preview_summary(result.preview)
            self.last_result = result
            self._show_scene(
                self._make_preview_scene(
                    preview=result.preview,
                    initial_source_points=result.preview.source_points_world_initial,
                    adjusted_source_points=result.preview.source_points_world_adjusted,
                    final_source_points=result.source_points_world_final,
                    transform_world_source_initial=result.preview.transform_world_source_initial,
                    transform_world_source_adjusted=result.preview.transform_world_source_adjusted,
                    transform_world_source_final=result.transform_world_source_final,
                ),
                scene_key=(
                    "pair",
                    self.source_id,
                    self.target_id,
                    result.preview.target_cloud_mode,
                    result.preview.target_neighbors,
                    round(result.preview.min_time_gap_sec, 6),
                    round(result.preview.target_map_voxel_size, 6),
                    "gicp",
                ),
                reset_camera=False,
            )
            self.gicp_metrics_label.setText(
                (
                    f"GICP ready · fit {result.fitness:.4f} · rmse {result.inlier_rmse:.4f}"
                    "<br><span style='color:#64748b;font-size:11px'>Replace selected loop when confirmed.</span>"
                )
                if replace_edge_uid is not None
                else f"GICP ready · fit {result.fitness:.4f} · rmse {result.inlier_rmse:.4f}"
            )
            self.accept_button.setEnabled(True)
            self._update_edge_action_buttons()
            self._refresh_plot(preserve_view=True)
            self.append_log(
                f"GICP completed for target={self.target_id}, source={self.source_id}, "
                f"fitness={result.fitness:.4f}, rmse={result.inlier_rmse:.4f} | "
                f"{self._gicp_log_cloud_summary(result.preview)}"
            )
        except Exception as exc:
            self._show_error("Run GICP Failed", f"{exc}\n\n{traceback.format_exc()}")
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()

    def run_auto_yaw_sweep(self) -> None:
        if self.workspace is None or self.source_id is None or self.target_id is None:
            self._show_error("Auto Yaw Sweep", "Select source and target nodes first.")
            return

        replace_edge_uid = None
        if self.selected_edge_ref is not None and self.selected_edge_ref.edge_kind == "existing":
            edge = self._edge_from_ref(self.selected_edge_ref)
            if edge is not None and edge.edge_type == "loop_existing":
                replace_edge_uid = edge.edge_uid

        base_delta = {key: spin.value() for key, spin in self.delta_spins.items()}
        step_count = max(int(self.auto_yaw_steps_spin.value()), 2)
        yaw_candidates = [360.0 * float(index) / float(step_count) for index in range(step_count)]
        config = self._current_registration_config()
        candidate_results: list[tuple[float, RegistrationResult]] = []
        self.selected_edge_ref = None
        self._candidate_replace_edge_uid = replace_edge_uid
        self._update_selection_labels()
        self._sync_constraint_table_selection()
        self._set_display_mode("Preview")
        self.accept_button.setEnabled(False)

        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            for yaw_index, yaw_seed in enumerate(yaw_candidates, start=1):
                delta_transform = build_delta_transform(
                    base_delta["x"],
                    base_delta["y"],
                    base_delta["z"],
                    base_delta["roll"],
                    base_delta["pitch"],
                    yaw_seed,
                )
                preview = self.workspace.build_preview(
                    source_id=self.source_id,
                    target_id=self.target_id,
                    delta_transform_local=delta_transform,
                    target_cloud_mode=config.target_cloud_mode,
                    target_neighbors=config.target_neighbors,
                    min_time_gap_sec=config.min_time_gap_sec,
                    target_map_voxel_size=config.target_map_voxel_size,
                )
                self.current_preview = preview
                self._update_preview_summary(preview)
                self.gicp_metrics_label.setText(
                    f"Yaw sweep {yaw_index}/{step_count} · seed {yaw_seed:.1f}°"
                )
                self._show_scene(
                    self._make_preview_scene(
                        preview=preview,
                        initial_source_points=preview.source_points_world_initial,
                        adjusted_source_points=preview.source_points_world_adjusted,
                        transform_world_source_initial=preview.transform_world_source_initial,
                        transform_world_source_adjusted=preview.transform_world_source_adjusted,
                    ),
                    scene_key=(
                        "auto_yaw_preview",
                        self.source_id,
                        self.target_id,
                        preview.target_cloud_mode,
                        preview.target_neighbors,
                        round(preview.min_time_gap_sec, 6),
                        round(preview.target_map_voxel_size, 6),
                    ),
                    reset_camera=False,
                    force_preserve_camera=True,
                )
                self._refresh_plot(preserve_view=True)
                self.append_log(
                    f"Auto yaw preview {yaw_index}/{step_count} seed={yaw_seed:.1f} deg | "
                    f"{self._gicp_log_cloud_summary(preview)}"
                )
                QtWidgets.QApplication.processEvents(QtCore.QEventLoop.AllEvents)
                result = self.workspace.run_gicp(
                    source_id=self.source_id,
                    target_id=self.target_id,
                    delta_transform_local=delta_transform,
                    config=config,
                )
                candidate_results.append((yaw_seed, result))
                self.append_log(
                    f"Auto yaw seed {yaw_seed:.1f} deg -> "
                    f"fitness={result.fitness:.4f}, rmse={result.inlier_rmse:.4f} | "
                    f"{self._gicp_log_cloud_summary(result.preview)}"
                )
                QtWidgets.QApplication.processEvents(QtCore.QEventLoop.ExcludeUserInputEvents)

            if not candidate_results:
                raise RuntimeError("Auto yaw sweep produced no valid GICP results.")

            max_fitness = max(result.fitness for _, result in candidate_results)
            shortlist = [
                (yaw_seed, result)
                for yaw_seed, result in candidate_results
                if result.fitness >= max(0.0, max_fitness - 0.05)
            ]
            best_yaw_seed, best_result = min(
                shortlist,
                key=lambda item: (item[1].inlier_rmse, -item[1].fitness, abs(item[0])),
            )

            blockers = [QtCore.QSignalBlocker(spin) for spin in self.delta_spins.values()]
            try:
                self.delta_spins["x"].setValue(base_delta["x"])
                self.delta_spins["y"].setValue(base_delta["y"])
                self.delta_spins["z"].setValue(base_delta["z"])
                self.delta_spins["roll"].setValue(base_delta["roll"])
                self.delta_spins["pitch"].setValue(base_delta["pitch"])
                self.delta_spins["yaw"].setValue(best_yaw_seed)
            finally:
                del blockers

            self._set_display_mode("Final")
            self.current_preview = best_result.preview
            self._update_preview_summary(best_result.preview)
            self.last_result = best_result
            self._show_scene(
                self._make_preview_scene(
                    preview=best_result.preview,
                    initial_source_points=best_result.preview.source_points_world_initial,
                    adjusted_source_points=best_result.preview.source_points_world_adjusted,
                    final_source_points=best_result.source_points_world_final,
                    transform_world_source_initial=best_result.preview.transform_world_source_initial,
                    transform_world_source_adjusted=best_result.preview.transform_world_source_adjusted,
                    transform_world_source_final=best_result.transform_world_source_final,
                ),
                scene_key=(
                    "pair",
                    self.source_id,
                    self.target_id,
                    best_result.preview.target_cloud_mode,
                    best_result.preview.target_neighbors,
                    round(best_result.preview.min_time_gap_sec, 6),
                    round(best_result.preview.target_map_voxel_size, 6),
                    "auto_yaw",
                    round(best_yaw_seed, 6),
                ),
                reset_camera=False,
                force_preserve_camera=True,
            )
            self.gicp_metrics_label.setText(
                (
                    f"Auto yaw · {best_yaw_seed:.1f}° · fit {best_result.fitness:.4f} · "
                    f"rmse {best_result.inlier_rmse:.4f}"
                    "<br><span style='color:#64748b;font-size:11px'>Replace selected loop when confirmed.</span>"
                )
                if replace_edge_uid is not None
                else f"Auto yaw · {best_yaw_seed:.1f}° · fit {best_result.fitness:.4f} · rmse {best_result.inlier_rmse:.4f}"
            )
            self.accept_button.setEnabled(True)
            self._update_edge_action_buttons()
            self._refresh_plot(preserve_view=True)
            self.append_log(
                f"Auto yaw selected seed {best_yaw_seed:.1f} deg for "
                f"target={self.target_id}, source={self.source_id}."
            )
        except Exception as exc:
            self._show_error("Auto Yaw Sweep Failed", f"{exc}\n\n{traceback.format_exc()}")
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()

    def accept_constraint(self) -> None:
        if self.last_result is None:
            self._show_error("Accept Constraint", "Run GICP and review the preview before accepting.")
            return

        self._push_undo_snapshot()
        variance_t, variance_r = self._current_variances()
        constraint, replaced = self._upsert_manual_constraint(
            self.last_result,
            variance_t=variance_t,
            variance_r=variance_r,
            replaces_edge_uid=None,
        )

        self.selected_edge_ref = SelectedEdgeRef("manual_added", constraint.manual_uid)
        self._candidate_replace_edge_uid = None
        self.current_preview = self.last_result.preview
        self._update_preview_summary(self.current_preview)
        self._rebuild_constraint_table()
        self._update_selection_labels()
        self._update_edge_action_buttons()
        self._sync_constraint_table_selection()
        self._recompute_session_dirty()
        self._update_session_status_widgets()
        self._refresh_plot(preserve_view=True)
        self.accept_button.setEnabled(False)
        self.append_log(
            f"{'Updated' if replaced else 'Accepted'} new manual edge "
            f"{constraint.target_id}->{constraint.source_id}"
        )
        self._save_project_state()
        self._save_project_state()

    def _upsert_manual_constraint(
        self,
        result: RegistrationResult,
        *,
        variance_t: tuple[float, float, float],
        variance_r: tuple[float, float, float],
        replaces_edge_uid: Optional[int],
    ) -> tuple[ManualConstraint, bool]:
        existing_constraint = self._constraint_by_pair(
            result.preview.source_id,
            result.preview.target_id,
        )
        manual_uid = existing_constraint.manual_uid if existing_constraint is not None else self._next_manual_uid
        preserved_note = existing_constraint.note if existing_constraint is not None else ""
        constraint = ManualConstraint(
            manual_uid=manual_uid,
            enabled=True,
            source_id=result.preview.source_id,
            target_id=result.preview.target_id,
            target_cloud_mode=result.preview.target_cloud_mode,
            target_neighbors=result.preview.target_neighbors,
            min_time_gap_sec=result.preview.min_time_gap_sec,
            target_map_voxel_size=result.preview.target_map_voxel_size,
            transform_world_source_final=result.transform_world_source_final.copy(),
            transform_target_source_final=result.transform_target_source_final.copy(),
            source_points_world_final=result.source_points_world_final.copy(),
            fitness=result.fitness,
            inlier_rmse=result.inlier_rmse,
            variance_t_m2=variance_t,
            variance_r_rad2=variance_r,
            replaces_edge_uid=replaces_edge_uid,
            accepted_rev=self._working_revision,
            applied_rev=None,
            note=preserved_note,
        )

        replaced = False
        for index, existing in enumerate(self.constraints):
            if existing.manual_uid == manual_uid:
                self.constraints[index] = constraint
                replaced = True
                break
        if not replaced:
            self.constraints.append(constraint)
            self._next_manual_uid += 1
        return constraint, replaced

    def replace_selected_edge(self) -> None:
        if self.last_result is None or self._candidate_replace_edge_uid is None or self.pose_graph is None:
            self._show_error(
                "Replace Selected Edge",
                "Run GICP from a selected existing loop edge first.",
            )
            return

        edge = self.pose_graph.edge_index_by_uid.get(self._candidate_replace_edge_uid)
        if edge is None or edge.edge_type != "loop_existing":
            self._show_error(
                "Replace Selected Edge",
                "The current GICP result is not tied to an existing loop edge.",
            )
            return

        answer = QtWidgets.QMessageBox.question(
            self,
            "Replace Selected Edge",
            "This will disable the selected existing loop edge in the working graph "
            "and replace it with the current manual GICP result. "
            "Original graph will not be modified.\n\nContinue?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel,
            QtWidgets.QMessageBox.Yes,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return

        self._push_undo_snapshot()
        variance_t, variance_r = self._current_variances()
        edge.enabled = False
        change = self.disabled_loop_changes.get(edge.edge_uid)
        if change is None:
            change = ExistingLoopChange(
                edge_uid=edge.edge_uid,
                enabled=True,
                accepted_rev=self._working_revision,
                applied_rev=None,
            )
            self.disabled_loop_changes[edge.edge_uid] = change
        else:
            change.enabled = True
            change.accepted_rev = self._working_revision
            change.applied_rev = None

        constraint, replaced = self._upsert_manual_constraint(
            self.last_result,
            variance_t=variance_t,
            variance_r=variance_r,
            replaces_edge_uid=edge.edge_uid,
        )

        self.selected_edge_ref = SelectedEdgeRef("manual_added", constraint.manual_uid)
        self._candidate_replace_edge_uid = None
        self.current_preview = self.last_result.preview
        self._update_preview_summary(self.current_preview)
        self._recompute_session_dirty()
        self._rebuild_constraint_table()
        self._update_selection_labels()
        self._update_edge_action_buttons()
        self._sync_constraint_table_selection()
        self._update_session_status_widgets()
        self._refresh_plot(preserve_view=True)
        self.accept_button.setEnabled(False)
        self.append_log(
            f"{'Updated' if replaced else 'Created'} replacement for existing loop "
            f"{edge.target_id}->{edge.source_id}; working graph now uses the new manual result."
        )
        self._save_project_state()

    def _rebuild_constraint_table(self) -> None:
        self._table_updating = True
        try:
            rows = self._graph_change_table_rows()
            self._graph_change_rows = [(kind, uid) for kind, uid, _ in rows]
            self.constraint_table.setRowCount(len(rows))
            for row, (_kind, _uid, values_map) in enumerate(rows):
                enabled_item = QtWidgets.QTableWidgetItem()
                enabled_item.setFlags(
                    QtCore.Qt.ItemIsEnabled
                    | QtCore.Qt.ItemIsSelectable
                    | QtCore.Qt.ItemIsUserCheckable
                )
                enabled_item.setCheckState(QtCore.Qt.Checked if values_map["use"] == "1" else QtCore.Qt.Unchecked)
                self.constraint_table.setItem(row, 0, enabled_item)

                values = [
                    values_map["type"],
                    values_map["status"],
                    values_map["src"],
                    values_map["tgt"],
                    values_map["submap"],
                    values_map["range"],
                    values_map["fitness"],
                    values_map["rmse"],
                    values_map["noise"],
                    values_map["accepted"],
                    values_map["applied"],
                    values_map["note"],
                ]
                background, foreground = self._status_brushes(values_map["status"])
                enabled_item.setBackground(background)
                enabled_item.setForeground(foreground)
                for column, value in enumerate(values, start=1):
                    item = QtWidgets.QTableWidgetItem(value)
                    flags = QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable
                    if column == 12:
                        flags |= QtCore.Qt.ItemIsEditable
                        item.setToolTip("Double-click to edit the note for this graph change.")
                    item.setFlags(flags)
                    item.setBackground(background)
                    item.setForeground(foreground)
                    if column == 12:
                        item.setData(QtCore.Qt.UserRole, value)
                    self.constraint_table.setItem(row, column, item)
        finally:
            self._table_updating = False

    def _on_constraint_item_changed(self, item: QtWidgets.QTableWidgetItem) -> None:
        if self._table_updating:
            return
        row = item.row()
        if row < 0 or row >= len(self._graph_change_rows):
            return
        row_kind, row_uid = self._graph_change_rows[row]
        if item.column() == 0:
            self._push_undo_snapshot()
            enabled = item.checkState() == QtCore.Qt.Checked
            if row_kind == "manual":
                constraint = self._constraint_by_uid(row_uid)
                if constraint is not None:
                    constraint.enabled = enabled
                    if constraint.replaces_edge_uid is not None and self.pose_graph is not None:
                        change = self._disabled_loop_change(constraint.replaces_edge_uid)
                        edge = self.pose_graph.edge_index_by_uid.get(constraint.replaces_edge_uid)
                        if change is not None and edge is not None:
                            change.enabled = enabled
                            edge.enabled = not enabled
            elif row_kind == "disable_loop":
                change = self._disabled_loop_change(row_uid)
                edge = self.pose_graph.edge_index_by_uid.get(row_uid) if self.pose_graph is not None else None
                if change is not None and edge is not None:
                    change.enabled = enabled
                    edge.enabled = not enabled
                    replacement_constraint = next(
                        (
                            constraint
                            for constraint in self.constraints
                            if constraint.replaces_edge_uid == row_uid
                        ),
                        None,
                    )
                    if replacement_constraint is not None:
                        replacement_constraint.enabled = enabled
            self._recompute_session_dirty()
            self._update_edge_action_buttons()
            self._update_session_status_widgets()
            self._rebuild_constraint_table()
            self._refresh_plot(preserve_view=True)
            self._save_project_state()
            return

        if item.column() != 12:
            return

        new_note = item.text().strip()
        previous_note = str(item.data(QtCore.Qt.UserRole) or "").strip()
        if new_note == previous_note:
            return
        self._push_undo_snapshot()
        if row_kind == "manual":
            constraint = self._constraint_by_uid(row_uid)
            if constraint is not None:
                constraint.note = new_note
        elif row_kind == "disable_loop":
            change = self._disabled_loop_change(row_uid)
            if change is not None:
                change.note = new_note
        self.append_log(
            f"Updated graph change note for {row_kind}:{row_uid} to "
            f"'{new_note or '(empty)'}'."
        )
        with QtCore.QSignalBlocker(self.constraint_table):
            item.setData(QtCore.Qt.UserRole, new_note)
        self._save_project_state()

    def _on_constraint_selection_changed(self) -> None:
        if self._table_updating:
            return
        row = self.constraint_table.currentRow()
        if row < 0 or row >= len(self._graph_change_rows):
            return
        row_kind, row_uid = self._graph_change_rows[row]
        if row_kind == "manual":
            self._select_edge(SelectedEdgeRef("manual_added", row_uid), reset_camera=True)
        else:
            self._select_edge(SelectedEdgeRef("existing", row_uid), reset_camera=True)

    def disable_selected_edge(self) -> None:
        edge = self._edge_from_ref(self.selected_edge_ref)
        if edge is None or edge.edge_type != "loop_existing" or not edge.enabled:
            self._show_error("Disable Edge", "Select an enabled existing loop edge first.")
            return
        self._push_undo_snapshot()
        edge.enabled = False
        self.disabled_loop_changes[edge.edge_uid] = ExistingLoopChange(
            edge_uid=edge.edge_uid,
            enabled=True,
            accepted_rev=self._working_revision,
            applied_rev=None,
        )
        self._recompute_session_dirty()
        self._update_selection_labels()
        self._update_edge_action_buttons()
        self._rebuild_constraint_table()
        self._update_session_status_widgets()
        self._refresh_plot(preserve_view=True)
        self.append_log(f"Disabled existing loop edge {edge.target_id}->{edge.source_id}")
        self._save_project_state()

    def restore_selected_edge(self) -> None:
        edge = self._edge_from_ref(self.selected_edge_ref)
        if edge is None or edge.edge_type != "loop_existing" or edge.enabled:
            self._show_error("Restore Edge", "Select a disabled existing loop edge first.")
            return
        self._push_undo_snapshot()
        replacement_constraint = None
        for constraint in self.constraints:
            if constraint.replaces_edge_uid == edge.edge_uid and constraint.enabled:
                replacement_constraint = constraint
                break
        edge.enabled = True
        change = self.disabled_loop_changes.get(edge.edge_uid)
        if change is not None:
            if change.applied_rev is None:
                self.disabled_loop_changes.pop(edge.edge_uid, None)
            else:
                change.enabled = False
        if replacement_constraint is not None:
            if replacement_constraint.applied_rev is None:
                self.constraints = [
                    constraint
                    for constraint in self.constraints
                    if constraint.manual_uid != replacement_constraint.manual_uid
                ]
            else:
                replacement_constraint.enabled = False
        self._recompute_session_dirty()
        self._update_selection_labels()
        self._update_edge_action_buttons()
        self._rebuild_constraint_table()
        self._update_session_status_widgets()
        self._refresh_plot(preserve_view=True)
        if replacement_constraint is not None:
            self.append_log(
                f"Restored existing loop edge {edge.target_id}->{edge.source_id} and removed its replacement manual edge."
            )
        else:
            self.append_log(f"Restored existing loop edge {edge.target_id}->{edge.source_id}")
        self._save_project_state()

    def remove_selected_manual_constraint(self) -> None:
        edge = self._edge_from_ref(self.selected_edge_ref)
        if edge is None or edge.edge_type != "manual_added":
            self._show_error("Remove Manual Constraint", "Select a manual constraint first.")
            return
        self._push_undo_snapshot()

        removed = None
        for index, constraint in enumerate(self.constraints):
            if constraint.manual_uid == edge.edge_uid:
                if constraint.applied_rev is None:
                    removed = self.constraints.pop(index)
                else:
                    constraint.enabled = False
                    removed = constraint
                break
        if removed is None:
            return

        if removed.replaces_edge_uid is not None and self.pose_graph is not None:
            restored_edge = self.pose_graph.edge_index_by_uid.get(removed.replaces_edge_uid)
            change = self.disabled_loop_changes.get(removed.replaces_edge_uid)
            if restored_edge is not None:
                restored_edge.enabled = True
            if change is not None:
                if change.applied_rev is None:
                    self.disabled_loop_changes.pop(removed.replaces_edge_uid, None)
                else:
                    change.enabled = False

        self.selected_edge_ref = None
        self._candidate_replace_edge_uid = None
        self._recompute_session_dirty()
        self._rebuild_constraint_table()
        self._update_selection_labels()
        self._update_edge_action_buttons()
        self._update_session_status_widgets()
        self._refresh_plot(preserve_view=True)
        if removed.replaces_edge_uid is not None:
            self.append_log(
                f"Removed replacement manual constraint {removed.target_id}->{removed.source_id} and restored the original loop edge."
            )
        else:
            self.append_log(f"Removed manual constraint {removed.target_id}->{removed.source_id}")
        self._save_project_state()

        if self.workspace is not None and self.source_id is not None and self.target_id is not None:
            self.schedule_preview_refresh(reset_camera=False)
        else:
            self.cloud_view.clear_scene()
            self._preview_scene_key = None

    # ===== BEGIN CHANGE: optimizer backend resolution =====
    def _resolve_cpp_optimizer_command(self) -> Optional[list[str]]:
        env_command = os.environ.get("MANUAL_LOOP_OPTIMIZER_BIN")
        if env_command:
            env_path = Path(env_command).expanduser().resolve()
            if env_path.is_file():
                return [str(env_path)]

        command = shutil.which("manual_loop_optimize")
        if command:
            return [command]

        candidates = [
            PROJECT_ROOT
            / "backend"
            / "catkin_ws"
            / "devel"
            / "lib"
            / "manual_loop_closure_backend"
            / "manual_loop_optimize",
            PROJECT_ROOT
            / "backend"
            / "catkin_ws"
            / "install"
            / "lib"
            / "manual_loop_closure_backend"
            / "manual_loop_optimize",
        ]
        for candidate in candidates:
            if candidate.is_file():
                return [str(candidate)]

        if shutil.which("rosrun"):
            return ["rosrun", "manual_loop_closure_backend", "manual_loop_optimize"]
        return None

    def _resolve_optimizer_backend(self):
        preference = str(
            self.optimizer_backend_combo.currentData()
            if hasattr(self, "optimizer_backend_combo")
            else os.environ.get("MANUAL_LOOP_OPTIMIZER_BACKEND", BACKEND_PREFERENCE_PYTHON).lower()
        )
        python_backend = resolve_python_optimizer_backend(
            script_dir=SCRIPT_DIR,
            project_root=PROJECT_ROOT,
        )
        cpp_command = self._cpp_optimizer_command_cache or self._resolve_cpp_optimizer_command()
        self._cpp_optimizer_command_cache = cpp_command

        if preference == BACKEND_PREFERENCE_PYTHON:
            if python_backend is not None:
                return python_backend
            if cpp_command is None:
                return None
            return {
                "key": BACKEND_PREFERENCE_CPP,
                "display_name": "cpp-fallback",
                "program": cpp_command[0],
                "args_prefix": cpp_command[1:],
                "fallback_from": BACKEND_PREFERENCE_PYTHON,
            }
        if preference == BACKEND_PREFERENCE_CPP:
            if cpp_command is None:
                return None
            return {
                "key": BACKEND_PREFERENCE_CPP,
                "display_name": "cpp-fallback",
                "program": cpp_command[0],
                "args_prefix": cpp_command[1:],
            }

        return python_backend
    # ===== END CHANGE: optimizer backend resolution =====

    def _start_optimizer_backend(
        self,
        backend,
        options: OptimizerRunOptions,
        *,
        fallback_note: Optional[str] = None,
    ) -> None:
        if isinstance(backend, dict):
            program = backend["program"]
            args = [*backend["args_prefix"], *options.to_cli_args()]
            backend_name = backend["display_name"]
            backend_key = backend.get("key", BACKEND_PREFERENCE_CPP)
        else:
            program = backend.program
            args = backend.build_process_args(options)
            backend_name = backend.display_name
            backend_key = backend.key

        self._optimizer_process = QtCore.QProcess(self)
        process_env = QtCore.QProcessEnvironment.systemEnvironment()
        process_env.insert("PYTHONUNBUFFERED", "1")
        self._optimizer_process.setProcessEnvironment(process_env)
        self._optimizer_process.setProgram(program)
        self._optimizer_process.setArguments(args)
        self._optimizer_process.readyReadStandardOutput.connect(self._read_optimizer_stdout)
        self._optimizer_process.readyReadStandardError.connect(self._read_optimizer_stderr)
        self._optimizer_process.finished.connect(self._optimizer_finished)
        self.optimize_button.setEnabled(False)
        self.export_button.setEnabled(False)
        self._active_optimizer_backend_name = backend_name
        self._active_optimizer_backend_key = backend_key
        self._optimizer_started_at = time.perf_counter()
        self._optimizer_heartbeat_timer.start()
        if fallback_note:
            self.append_log(fallback_note)
        self.append_log(
            "Starting Optimize Working Graph with "
            f"{sum(1 for constraint in self.constraints if constraint.enabled)} manual constraints, "
            f"{len(self._active_disabled_loop_changes())} disabled loop edges, "
            f"export_map_voxel={self.export_map_voxel_spin.value():.3f} m, "
            f"optimizer backend={backend_name}. "
            "Full map rebuild is deferred until Export."
        )
        self._optimizer_process.start()

    def _on_optimizer_heartbeat(self) -> None:
        if self._optimizer_process is None or self._optimizer_started_at is None:
            return
        elapsed = time.perf_counter() - self._optimizer_started_at
        self.append_log(
            f"Optimizer running ({self._active_optimizer_backend_name}) · elapsed={elapsed:.1f}s"
        )

    def run_optimization(self) -> None:
        if self._optimizer_process is not None:
            self._show_error("Optimize Working Graph", "An optimization process is already running.")
            return
        if self.session_paths is None or self.pose_graph is None or self.original_pose_graph is None or self.original_trajectory is None:
            self._show_error("Optimize Working Graph", "Load a session first.")
            return
        if not self._session_dirty:
            self._show_error("Optimize Working Graph", "Working graph is already clean. Add, remove, or toggle a change first.")
            return

        enabled_constraints = [constraint for constraint in self.constraints if constraint.enabled]
        active_disabled_changes = self._active_disabled_loop_changes()
        if not enabled_constraints and not active_disabled_changes:
            self._show_error(
                "Optimize Working Graph",
                "No enabled manual constraints or disabled existing loop edges to export.",
            )
            return

        self._selected_optimizer_preference = str(
            self.optimizer_backend_combo.currentData()
            if hasattr(self, "optimizer_backend_combo")
            else os.environ.get("MANUAL_LOOP_OPTIMIZER_BACKEND", BACKEND_PREFERENCE_PYTHON).lower()
        )
        backend = self._resolve_optimizer_backend()
        if backend is None:
            self._show_error(
                "Optimize Working Graph",
                "No optimizer backend is available. Install Python GTSAM for the default Python backend. "
                "The legacy C++ backend is optional and only needed for fallback or parity checks.",
            )
            return

        output_dir = (
            self.session_paths.session_root
            / "manual_loop_runs"
            / datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        output_dir.mkdir(parents=True, exist_ok=True)

        edited_g2o_path = output_dir / "edited_input_pose_graph.g2o"
        baseline_pose_graph = copy.deepcopy(self.original_pose_graph)
        for edge in baseline_pose_graph.loop_edges:
            if edge.edge_uid in self.disabled_loop_changes:
                edge.enabled = not self.disabled_loop_changes[edge.edge_uid].enabled
        write_filtered_pose_graph(baseline_pose_graph, edited_g2o_path)

        csv_path = output_dir / "manual_loop_constraints.csv"
        with csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
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
            )
            for constraint in enabled_constraints:
                writer.writerow(constraint.csv_row())

        options = OptimizerRunOptions(
            session_root=self.session_paths.session_root,
            g2o_path=edited_g2o_path,
            tum_path=self.original_trajectory.path,
            keyframe_dir=self.session_paths.keyframe_dir,
            constraints_csv=csv_path,
            output_dir=output_dir,
            map_voxel_leaf=float(self.export_map_voxel_spin.value()),
            skip_map_build=True,
        )
        self._last_output_dir = output_dir
        self._write_run_context(output_dir)
        self._pre_optimize_snapshot = self._capture_undo_snapshot()
        self._pending_optimizer_options = options
        self._optimizer_retry_attempted = False
        fallback_note = None
        if isinstance(backend, dict) and backend.get("fallback_from") == BACKEND_PREFERENCE_PYTHON:
            fallback_note = "Python backend is unavailable in the current environment. Falling back to the optional C++ backend."
        self._start_optimizer_backend(backend, options, fallback_note=fallback_note)

    def _read_optimizer_stdout(self) -> None:
        if self._optimizer_process is None:
            return
        data = bytes(self._optimizer_process.readAllStandardOutput()).decode("utf-8", errors="ignore")
        for line in data.splitlines():
            self.append_log(line)

    def _read_optimizer_stderr(self) -> None:
        if self._optimizer_process is None:
            return
        data = bytes(self._optimizer_process.readAllStandardError()).decode("utf-8", errors="ignore")
        for line in data.splitlines():
            self.append_log(f"[stderr] {line}")

    def _optimizer_finished(self, exit_code: int, exit_status: QtCore.QProcess.ExitStatus) -> None:
        self.optimize_button.setEnabled(True)
        self._update_session_status_widgets()
        self._optimizer_heartbeat_timer.stop()
        elapsed_text = ""
        if self._optimizer_started_at is not None:
            elapsed_text = f", elapsed={time.perf_counter() - self._optimizer_started_at:.2f}s"
        if exit_status == QtCore.QProcess.NormalExit and exit_code == 0:
            self.append_log(
                f"Optimizer finished successfully ({self._active_optimizer_backend_name}). "
                f"Output: {self._last_output_dir}{elapsed_text}"
            )
            self.append_log("Working trajectory updated. Final map rebuild will run during Export.")
            if self._pre_optimize_snapshot is not None:
                self._undo_stack.append(self._pre_optimize_snapshot)
                self.undo_button.setEnabled(True)
            self._apply_working_optimization_result()
            self._save_project_state()
            if self._pending_export_after_optimize:
                self._pending_export_after_optimize = False
                self.export_final_result()
        else:
            if (
                not self._optimizer_retry_attempted
                and self._selected_optimizer_preference == BACKEND_PREFERENCE_PYTHON
                and self._active_optimizer_backend_key == BACKEND_PREFERENCE_PYTHON
                and self._pending_optimizer_options is not None
            ):
                cpp_command = self._cpp_optimizer_command_cache or self._resolve_cpp_optimizer_command()
                self._cpp_optimizer_command_cache = cpp_command
                if cpp_command is not None:
                    self.append_log(
                        "Python optimizer failed. Retrying once with the optional C++ backend."
                    )
                    self._optimizer_retry_attempted = True
                    self._optimizer_heartbeat_timer.stop()
                    self._optimizer_process = None
                    backend = {
                        "key": BACKEND_PREFERENCE_CPP,
                        "display_name": "cpp-fallback",
                        "program": cpp_command[0],
                        "args_prefix": cpp_command[1:],
                    }
                    self._start_optimizer_backend(backend, self._pending_optimizer_options)
                    return
            self.append_log(
                f"Optimizer failed ({self._active_optimizer_backend_name}). "
                f"exit_code={exit_code}, exit_status={int(exit_status)}{elapsed_text}"
            )
            self._pending_export_after_optimize = False
        self._pre_optimize_snapshot = None
        self._pending_optimizer_options = None
        self._optimizer_retry_attempted = False
        self._optimizer_started_at = None
        self._optimizer_process = None

    def _apply_working_optimization_result(self) -> None:
        if self._last_output_dir is None:
            return
        tum_path = self._last_output_dir / "optimized_poses_tum.txt"
        if not tum_path.is_file():
            self.append_log("Working graph update skipped: optimized TUM output is missing.")
            return
        try:
            self.trajectory = load_tum_trajectory(tum_path)
            self.workspace = RegistrationWorkspace(self.session_paths.keyframe_dir, self.trajectory)
            self._working_revision += 1
            for constraint in self.constraints:
                if constraint.enabled:
                    constraint.applied_rev = self._working_revision
            for change in self.disabled_loop_changes.values():
                if change.enabled:
                    change.applied_rev = self._working_revision
            self._session_dirty = False
            self.current_preview = None
            self.last_result = None
            self._candidate_replace_edge_uid = None
            self.accept_button.setEnabled(False)
            self.gicp_metrics_label.setText("Working graph updated · continue from the new trajectory.")
            with QtCore.QSignalBlocker(self.trajectory_view_combo):
                self.trajectory_view_combo.setCurrentText("Working")
            self._rebuild_constraint_table()
            self._update_selection_labels()
            self._update_edge_action_buttons()
            self._update_session_status_widgets()
            self._refresh_plot(preserve_view=False)
            if self.source_id is not None and self.target_id is not None:
                self.schedule_preview_refresh(reset_camera=False)
            self._save_project_state()
        except Exception as exc:
            self.append_log(f"Failed to update working graph after optimization: {exc}")

    def export_final_result(self) -> None:
        if self.session_paths is None:
            self._show_error("Export Final Result", "Load a session first.")
            return
        if self._session_dirty:
            answer = QtWidgets.QMessageBox.question(
                self,
                "Export Final Result",
                "Working graph has unapplied changes. Optimize before export?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.Cancel,
                QtWidgets.QMessageBox.Yes,
            )
            if answer == QtWidgets.QMessageBox.Yes:
                self._pending_export_after_optimize = True
                self.run_optimization()
            return
        if self._last_output_dir is None or not self._last_output_dir.exists():
            self._show_error(
                "Export Final Result",
                "No optimized working result is available yet. Run Optimize Working Graph first.",
            )
            return
        try:
            QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            self._ensure_run_map_outputs(self._last_output_dir)
        except Exception as exc:
            self._show_error("Export Final Result", f"Failed to build final map during export:\n{exc}")
            return
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()

        export_dir = (
            self.session_paths.session_root
            / "manual_loop_exports"
            / datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        export_dir.parent.mkdir(parents=True, exist_ok=True)
        export_dir.mkdir(parents=True, exist_ok=True)
        self._write_export_manifest(export_dir, self._last_output_dir)
        self._latest_export_dir = export_dir
        self.append_log(
            f"Exported final clean working manifest to {export_dir} -> {self._last_output_dir}",
            event="export_final",
            payload={"export_dir": str(export_dir), "run_dir": str(self._last_output_dir)},
        )
        self._save_project_state()
        QtWidgets.QMessageBox.information(
            self,
            "Export Final Result",
            f"Exported final clean working manifest to:\n{export_dir}\n\nSelected run:\n{self._last_output_dir}",
        )

    def undo_last_change(self) -> None:
        if not self._undo_stack:
            return
        snapshot = self._undo_stack.pop()
        self._restore_snapshot(snapshot)
        self.append_log("Undo restored the previous working-session state.")
        self._save_project_state()

    def _show_error(self, title: str, message: str) -> None:
        self.append_log(f"{title}: {message}")
        QtWidgets.QMessageBox.critical(self, title, message)

    def closeEvent(self, event) -> None:  # noqa: N802
        if self._optimizer_process is not None:
            self._optimizer_process.kill()
            self._optimizer_process.waitForFinished(1000)
            self._optimizer_process = None
        self.cloud_view.shutdown()
        super().closeEvent(event)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline manual loop closure GUI.")
    parser.add_argument("--session-root", type=Path, help="Session root directory")
    parser.add_argument("--g2o", type=Path, help="Explicit pose_graph.g2o path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    window = ManualLoopClosureWindow(
        initial_session_root=args.session_root,
        initial_g2o_path=args.g2o,
    )
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
