from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MS_MANUAL_LOOP_REEXEC", "1")

REPO_ROOT = Path(__file__).resolve().parents[1]
GUI_DIR = REPO_ROOT / "gui"
if str(GUI_DIR) not in sys.path:
    sys.path.insert(0, str(GUI_DIR))

from PyQt5 import QtWidgets  # noqa: E402

import manual_loop_closure_tool as tool  # noqa: E402
from manual_loop_closure.trajectory_io import (  # noqa: E402
    TrajectoryData,
    build_transform,
    matrix_to_xyz_rpy_deg,
)


DELTA_KEYS = ("x", "y", "z", "roll", "pitch", "yaw")


class ManualEdgePreviewSeedTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def _make_window(self) -> tool.ManualLoopClosureWindow:
        window = tool.ManualLoopClosureWindow.__new__(tool.ManualLoopClosureWindow)
        window.delta_spins = {}
        for key in DELTA_KEYS:
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(9)
            spin.setRange(-1.0e6, 1.0e6)
            window.delta_spins[key] = spin
        window.constraints = []
        window.appended_logs = []
        window.append_log = window.appended_logs.append
        return window

    def _spin_values(self, window: tool.ManualLoopClosureWindow) -> np.ndarray:
        return np.asarray([window.delta_spins[key].value() for key in DELTA_KEYS], dtype=np.float64)

    def _set_spin_values(self, window: tool.ManualLoopClosureWindow, values: tuple[float, ...]) -> None:
        for key, value in zip(DELTA_KEYS, values):
            window.delta_spins[key].setValue(value)

    def _trajectory_with_source_base(self, source_id: int, base_transform: np.ndarray) -> TrajectoryData:
        transforms = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], source_id + 1, axis=0)
        transforms[source_id] = base_transform
        return TrajectoryData(
            path=Path("dummy_tum.txt"),
            timestamps=np.arange(source_id + 1, dtype=np.float64),
            positions_xyz=transforms[:, :3, 3].copy(),
            quats_xyzw=np.tile(np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64), (source_id + 1, 1)),
            transforms_world_sensor=transforms,
        )

    def _manual_constraint(
        self,
        *,
        source_id: int,
        target_id: int,
        transform_world_source_final: np.ndarray,
        enabled: bool = True,
        applied_rev: int | None = None,
    ) -> tool.ManualConstraint:
        return tool.ManualConstraint(
            manual_uid=7,
            enabled=enabled,
            source_id=source_id,
            target_id=target_id,
            target_cloud_mode=tool.TARGET_CLOUD_MODE_TEMPORAL_WINDOW,
            target_neighbors=100,
            min_time_gap_sec=30.0,
            target_map_voxel_size=0.1,
            transform_world_source_final=transform_world_source_final,
            transform_target_source_final=np.eye(4, dtype=np.float64),
            source_points_world_final=np.empty((0, 3), dtype=np.float64),
            fitness=1.0,
            inlier_rmse=0.01,
            variance_t_m2=(0.1, 0.1, 0.1),
            variance_r_rad2=(0.1, 0.1, 0.1),
            applied_rev=applied_rev,
        )

    def test_unapplied_manual_edge_seeds_node_pair_delta(self) -> None:
        source_id = 2
        target_id = 0
        base_transform = build_transform(
            [2.0, -1.0, 0.5],
            Rotation.from_euler("xyz", [0.0, 0.0, 20.0], degrees=True).as_quat(),
        )
        expected_delta_transform = build_transform(
            [0.8, -0.2, 0.15],
            Rotation.from_euler("xyz", [1.0, -2.0, 15.0], degrees=True).as_quat(),
        )
        final_source_transform = base_transform @ expected_delta_transform

        window = self._make_window()
        window.workspace = SimpleNamespace(
            trajectory=self._trajectory_with_source_base(source_id, base_transform)
        )
        window.constraints = [
            self._manual_constraint(
                source_id=source_id,
                target_id=target_id,
                transform_world_source_final=final_source_transform,
                applied_rev=None,
            )
        ]

        window._prepare_delta_for_pair_preview(source_id, target_id)

        np.testing.assert_allclose(
            self._spin_values(window),
            matrix_to_xyz_rpy_deg(expected_delta_transform),
            atol=1.0e-8,
        )
        self.assertIn("Using accepted manual edge as preview seed", window.appended_logs[-1])

    def test_applied_or_missing_manual_edge_clears_stale_delta(self) -> None:
        source_id = 1
        target_id = 0
        base_transform = np.eye(4, dtype=np.float64)

        window = self._make_window()
        window.workspace = SimpleNamespace(
            trajectory=self._trajectory_with_source_base(source_id, base_transform)
        )
        window.constraints = [
            self._manual_constraint(
                source_id=source_id,
                target_id=target_id,
                transform_world_source_final=np.eye(4, dtype=np.float64),
                applied_rev=3,
            )
        ]
        self._set_spin_values(window, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))

        window._prepare_delta_for_pair_preview(source_id, target_id)

        np.testing.assert_allclose(self._spin_values(window), np.zeros(6), atol=1.0e-12)
        self.assertEqual(window.appended_logs, [])

    def test_working_graph_update_clears_delta_after_optimization(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            output_dir = Path(tmp_dir) / "manual_loop_runs" / "run_001"
            output_dir.mkdir(parents=True)
            (output_dir / "optimized_poses_tum.txt").write_text(
                "0.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0\n"
                "1.0 1.0 0.0 0.0 0.0 0.0 0.0 1.0\n",
                encoding="utf-8",
            )

            window = self._make_window()
            window._last_output_dir = output_dir
            window.session_paths = SimpleNamespace(keyframe_dir=Path(tmp_dir) / "key_point_frame")
            window.constraints = [
                self._manual_constraint(
                    source_id=1,
                    target_id=0,
                    transform_world_source_final=np.eye(4, dtype=np.float64),
                    applied_rev=None,
                )
            ]
            window.disabled_loop_changes = {}
            window._working_revision = 0
            window._session_dirty = True
            window.current_preview = object()
            window.last_result = object()
            window._candidate_replace_edge_uid = 42
            window.source_id = None
            window.target_id = None
            window.accept_button = QtWidgets.QPushButton()
            window.gicp_metrics_label = QtWidgets.QLabel()
            window.trajectory_view_combo = QtWidgets.QComboBox()
            window.trajectory_view_combo.addItems(["Original", "Working"])
            window._rebuild_constraint_table = lambda: None
            window._update_selection_labels = lambda: None
            window._update_edge_action_buttons = lambda: None
            window._update_session_status_widgets = lambda: None
            window._refresh_plot = lambda preserve_view=False: None
            window._save_project_state = lambda: None
            self._set_spin_values(window, (1.0, -2.0, 3.0, -4.0, 5.0, -6.0))

            window._apply_working_optimization_result()

            np.testing.assert_allclose(self._spin_values(window), np.zeros(6), atol=1.0e-12)
            self.assertEqual(window._working_revision, 1)
            self.assertEqual(window.constraints[0].applied_rev, 1)
            self.assertIsNone(window.current_preview)
            self.assertIsNone(window.last_result)
            self.assertIsNone(window._candidate_replace_edge_uid)
            self.assertFalse(window.accept_button.isEnabled())


if __name__ == "__main__":
    unittest.main()
