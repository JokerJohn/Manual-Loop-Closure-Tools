"""Rotation sigmas of manual loop constraints are written in degrees.

The GUI stores the rotation variance in rad^2 (spin box "Variance R [rad^2]");
the constraints CSV has columns sigma_{roll,pitch,yaw}_deg, which both the Python
and the C++ optimizer convert with deg -> rad. The value written to those columns
must therefore be the standard deviation in degrees.
"""
from __future__ import annotations

import csv
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MS_MANUAL_LOOP_REEXEC", "1")

REPO_ROOT = Path(__file__).resolve().parents[1]
GUI_DIR = REPO_ROOT / "gui"
if str(GUI_DIR) not in sys.path:
    sys.path.insert(0, str(GUI_DIR))

import manual_loop_closure_tool as tool  # noqa: E402
from manual_loop_closure.python_optimizer import optimizer as py_optimizer  # noqa: E402

CSV_HEADER = [
    "enabled", "source_id", "target_id", "tx", "ty", "tz", "qx", "qy", "qz", "qw",
    "sigma_tx", "sigma_ty", "sigma_tz", "sigma_roll_deg", "sigma_pitch_deg", "sigma_yaw_deg",
]

SIGMA_T_M = 0.05
SIGMA_R_RAD = 0.02  # 1.146 deg


def _make_constraint() -> tool.ManualConstraint:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = [0.1, 0.0, 0.0]
    return tool.ManualConstraint(
        manual_uid=0,
        enabled=True,
        source_id=1,
        target_id=0,
        target_cloud_mode="temporal_window",
        target_neighbors=10,
        min_time_gap_sec=0.0,
        target_map_voxel_size=0.1,
        transform_world_source_final=transform.copy(),
        transform_target_source_final=transform.copy(),
        source_points_world_final=np.zeros((0, 3), dtype=np.float64),
        fitness=1.0,
        inlier_rmse=0.01,
        variance_t_m2=(SIGMA_T_M**2,) * 3,
        variance_r_rad2=(SIGMA_R_RAD**2,) * 3,
    )


class ManualConstraintSigmaUnitsTest(unittest.TestCase):
    def test_csv_row_writes_rotation_sigma_in_degrees(self) -> None:
        row = dict(zip(CSV_HEADER, _make_constraint().csv_row()))
        for axis in ("roll", "pitch", "yaw"):
            self.assertAlmostEqual(float(row[f"sigma_{axis}_deg"]), math.degrees(SIGMA_R_RAD), places=5)
        for axis in ("tx", "ty", "tz"):
            self.assertAlmostEqual(float(row[f"sigma_{axis}"]), SIGMA_T_M, places=6)

    def test_optimizer_recovers_rad2_variance_from_gui_csv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            csv_path = Path(tmp) / "manual_loop_constraints.csv"
            with csv_path.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(CSV_HEADER)
                writer.writerow(_make_constraint().csv_row())
            specs = py_optimizer._load_constraints_csv(csv_path)
        self.assertEqual(len(specs), 1)
        information = py_optimizer._manual_constraint_information(specs[0])
        # GTSAM Pose3 order: rotation block first.
        np.testing.assert_allclose(np.diag(information)[:3], 1.0 / SIGMA_R_RAD**2, rtol=1e-4)
        np.testing.assert_allclose(np.diag(information)[3:], 1.0 / SIGMA_T_M**2, rtol=1e-4)


if __name__ == "__main__":
    unittest.main()
