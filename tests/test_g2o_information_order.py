"""EDGE_SE3:QUAT information blocks follow the g2o convention (translation first).

GTSAM's own writeG2o/readG2o permute the 6x6 information between the g2o order
[x y z | rx ry rz] and the Pose3 tangent order [rx ry rz | x y z]. The GUI's
Python loader and writer must do the same, otherwise every anisotropic edge
read from (or written to) a g2o file has its rotation and translation weights
swapped.
"""
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
GUI_DIR = REPO_ROOT / "gui"
if str(GUI_DIR) not in sys.path:
    sys.path.insert(0, str(GUI_DIR))

try:
    import gtsam  # noqa: E402
except Exception:  # pragma: no cover - environment specific
    gtsam = None

from manual_loop_closure.python_optimizer.graph_loader import (  # noqa: E402
    ANCHOR_VERTEX_ID,
    BetweenFactorRecord,
    PosePriorRecord,
    build_factor_graph,
)
from manual_loop_closure.python_optimizer.exporters import write_pose_graph_g2o  # noqa: E402

# Rotation sigma 0.1 rad (information 100), translation sigma 1 m (information 1).
INFO_GTSAM_ORDER = np.diag([100.0, 100.0, 100.0, 1.0, 1.0, 1.0])
# The same information in the g2o file order (translation block first).
INFO_G2O_ORDER = np.diag([1.0, 1.0, 1.0, 100.0, 100.0, 100.0])


def _upper(info: np.ndarray) -> str:
    return " ".join(f"{info[r, c]:g}" for r in range(6) for c in range(r, 6))


@unittest.skipIf(gtsam is None, "Python GTSAM is unavailable")
class G2oInformationOrderTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def _write(self, text: str) -> Path:
        path = self.root / "pose_graph.g2o"
        path.write_text(text, encoding="utf-8")
        return path

    def test_between_edge_written_by_gtsam_is_read_in_pose3_order(self) -> None:
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        values.insert(0, gtsam.Pose3())
        values.insert(1, gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)))
        graph.add(
            gtsam.BetweenFactorPose3(
                0, 1, gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)),
                gtsam.noiseModel.Gaussian.Information(INFO_GTSAM_ORDER),
            )
        )
        g2o_path = self.root / "pose_graph.g2o"
        gtsam.writeG2o(graph, values, str(g2o_path))

        result = build_factor_graph(session_root=self.root, g2o_path=g2o_path, gtsam_mod=gtsam)
        between = [f for f in (result.graph.at(i) for i in range(result.graph.size()))
                   if isinstance(f, gtsam.BetweenFactorPose3)]
        self.assertEqual(len(between), 1)
        np.testing.assert_allclose(between[0].noiseModel().information(), INFO_GTSAM_ORDER, rtol=1e-9)
        records = [r for r in result.factor_records if isinstance(r, BetweenFactorRecord)]
        np.testing.assert_allclose(records[0].information, INFO_GTSAM_ORDER, rtol=1e-9)

    def test_anisotropic_edge_weights_rotation_and_translation_correctly(self) -> None:
        # Two contradicting edges 0->1: edge A is tight in rotation (0.1 rad) and loose in
        # translation (1 m); edge B the opposite. The optimum must take A's rotation and
        # B's translation.
        info_b_g2o = np.diag([100.0, 100.0, 100.0, 1.0, 1.0, 1.0])  # g2o order: tight t, loose R
        qz = np.sin(np.deg2rad(10.0) / 2.0)
        qw = np.cos(np.deg2rad(10.0) / 2.0)
        text = (
            "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\n"
            "VERTEX_SE3:QUAT 1 0 0 0 0 0 0 1\n"
            f"EDGE_SE3:QUAT 0 1 0 0 0 0 0 {qz:.12f} {qw:.12f} {_upper(INFO_G2O_ORDER)}\n"
            f"EDGE_SE3:QUAT 1 0 1 0 0 0 0 0 1 {_upper(info_b_g2o)}\n"
        )
        # Edge B is written as 1->0 so the loader's duplicate-edge filter keeps both.
        result = build_factor_graph(session_root=self.root, g2o_path=self._write(text), gtsam_mod=gtsam)
        params = gtsam.LevenbergMarquardtParams()
        optimized = gtsam.LevenbergMarquardtOptimizer(result.graph, result.initial_values, params).optimize()
        relative = optimized.atPose3(gtsam.Symbol("x", 0).key()).between(
            optimized.atPose3(gtsam.Symbol("x", 1).key())
        )
        yaw_deg = np.rad2deg(relative.rotation().yaw())
        self.assertAlmostEqual(yaw_deg, 10.0, delta=0.5)
        # Edge B fixes T_10 = (I, [1, 0, 0]) in translation, so t_01 = -R_01 [1, 0, 0].
        expected_t = -np.asarray(relative.rotation().matrix())[:, 0]
        np.testing.assert_allclose(np.asarray(relative.translation()), expected_t, atol=0.02)

    def test_anchor_prior_is_read_in_pose3_order(self) -> None:
        text = (
            "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\n"
            f"VERTEX_SE3:QUAT {ANCHOR_VERTEX_ID} 0 0 0 0 0 0 1\n"
            f"EDGE_SE3:QUAT {ANCHOR_VERTEX_ID} 0 0 0 0 0 0 0 1 {_upper(INFO_G2O_ORDER)}\n"
        )
        result = build_factor_graph(session_root=self.root, g2o_path=self._write(text), gtsam_mod=gtsam)
        priors = [r for r in result.factor_records if isinstance(r, PosePriorRecord)]
        self.assertEqual(len(priors), 1)
        self.assertEqual(priors[0].source, "anchor")
        np.testing.assert_allclose(priors[0].information, INFO_GTSAM_ORDER, rtol=1e-9)
        factor = result.graph.at(0)
        np.testing.assert_allclose(factor.noiseModel().information(), INFO_GTSAM_ORDER, rtol=1e-9)

    def test_written_g2o_round_trips_through_gtsam_reader(self) -> None:
        values = gtsam.Values()
        values.insert(gtsam.Symbol("x", 0).key(), gtsam.Pose3())
        values.insert(gtsam.Symbol("x", 1).key(), gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)))
        record = BetweenFactorRecord(
            node_i=0,
            node_j=1,
            translation_xyz=np.asarray([1.0, 0.0, 0.0]),
            quat_xyzw=np.asarray([0.0, 0.0, 0.0, 1.0]),
            information=INFO_GTSAM_ORDER.copy(),
            origin="manual",
        )
        out = self.root / "out.g2o"
        write_pose_graph_g2o(out, values, 2, [record], gtsam)
        graph, _ = gtsam.readG2o(str(out), True)
        np.testing.assert_allclose(graph.at(0).noiseModel().information(), INFO_GTSAM_ORDER, rtol=1e-9)


if __name__ == "__main__":
    unittest.main()
