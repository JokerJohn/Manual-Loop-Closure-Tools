"""Loop audit: per-edge evidence and a keep/check/remove suggestion for every loop.

A loop constraint is judged by evidence that does not depend on its noise model.

Map consistency.  The keyframe scans are stitched under a solution of the graph and
every planar point is matched to the nearest point recorded at least ``min_dt``
seconds earlier or later (a different visit of the place).  Of the points that
overlap such a visit within ``overlap_radius``, the normal agreement ratio (NAR) is
the fraction whose surface normals agree within ``normal_deg``.  Two visits of the
same surface agree; a map folded by a wrong loop overlays different structures, whose
normals do not.  The NAR is computed without any reference trajectory.

Stage 1 (single-loop test).  Each loop is added alone to the graph without loops.
A correct loop raises the NAR of that graph; a wrong loop lowers it.
Stage 2 (leave-one-out).  Among the loops that pass Stage 1, the graph is solved
with all of them and once without each; the loop whose removal raises the NAR most
(beyond ``tol``) is removed and the remaining loops are tested again, until no
removal raises the NAR.  This finds loops that are plausible alone but conflict
with the others.
Repeatability.  The source scan is registered again to the target window, coarse to
fine (1.0, 0.3, 0.1 m), from the stored measurement; the overlap at 0.1 m and the
change of the relative pose are reported, and a measurement that does not reproduce
is marked for checking.

The module has no Qt dependency; the GUI calls :func:`audit_loops` from a worker and
``python -m manual_loop_closure.loop_audit`` runs it headless.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import tempfile
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Optional, Sequence

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

LogFn = Optional[Callable[[str], None]]


# ----------------------------------------------------------------------------- data
@dataclass
class AuditLoop:
    uid: str                      # 'existing:<k>' or 'manual:<row>'
    source_id: int
    target_id: int
    Z: np.ndarray                 # 4x4 pose of the source in the target frame
    information: np.ndarray       # 6x6, GTSAM Pose3 order (rotation first)
    origin: str = 'manual'


@dataclass
class AuditConfig:
    voxel: float = 0.2            # scan voxel for the consistency score [m]
    points_per_scan: int = 600    # stitched points kept per keyframe
    queries: int = 30000          # planar query points
    min_dt: float = 30.0          # time gap between two visits [s]
    overlap_radius: float = 0.5   # cross-visit neighbour counted as overlap [m]
    normal_deg: float = 25.0      # normals agree within this angle
    tol: float = 0.02             # NAR change counted as an effect (absolute)
    recheck: bool = True
    recheck_window: int = 5       # target window +- keyframes for the re-registration
    recheck_fitness: float = 0.3  # overlap at 0.1 m below which a measurement is weak
    recheck_t: float = 0.2        # [m]  re-measured pose change beyond which it is weak
    recheck_r: float = 2.0        # [deg]
    seed: int = 0


@dataclass
class LoopEvidence:
    uid: str
    source_id: int
    target_id: int
    origin: str
    nar_single: float = float('nan')        # NAR with only this loop
    d_single: float = float('nan')          # nar_single - NAR without loops
    nar_without: float = float('nan')       # NAR of the accepted set without this loop
    d_loo: float = float('nan')             # nar_without - NAR of the accepted set (> 0: removal helps)
    residual_t: float = float('nan')        # residual in the solution with all loops [m]
    residual_r: float = float('nan')        # [deg]
    recheck_fitness: float = float('nan')   # overlap at 0.1 m after re-registration
    recheck_rmse: float = float('nan')
    recheck_dt: float = float('nan')        # re-measured minus stored pose [m]
    recheck_dr: float = float('nan')        # [deg]
    suggestion: str = 'keep'                # keep | redundant | check | remove
    reason: str = ''
    stage: int = 0                          # 1 or 2: stage that removed it


@dataclass
class AuditResult:
    nar_none: float
    nar_all: float
    nar_final: float
    loops: list[LoopEvidence] = field(default_factory=list)
    removed: list[str] = field(default_factory=list)
    elapsed_sec: float = 0.0
    config: dict = field(default_factory=dict)


def _log(log_fn: LogFn, msg: str) -> None:
    if log_fn is not None:
        log_fn(msg)


def _rel_err(A: np.ndarray, B: np.ndarray) -> tuple[float, float]:
    D = np.linalg.inv(A) @ B
    return float(np.linalg.norm(D[:3, 3])), float(np.degrees(Rotation.from_matrix(D[:3, :3]).magnitude()))


# ----------------------------------------------------------------------------- graph
class LoopGraph:
    """Archived graph without its loop edges, plus a list of loops to add."""

    def __init__(self, session_root: Path, g2o_path: Path, gtsam_mod, log_fn: LogFn = None):
        from .python_optimizer.graph_loader import _parse_information_matrix, build_factor_graph
        self.gtsam = gtsam_mod
        lines = g2o_path.read_text(encoding='utf-8').splitlines(keepends=True)
        keep, self.existing = [], []
        for k, raw in enumerate(lines):
            tok = raw.split()
            if tok and tok[0] == 'EDGE_SE3:QUAT':
                i, j = int(tok[1]), int(tok[2])
                anchor = 2147483646 in (i, j)
                if not anchor and abs(i - j) > 1:
                    Zij = np.eye(4)
                    Zij[:3, :3] = Rotation.from_quat([float(v) for v in tok[6:10]]).as_matrix()
                    Zij[:3, 3] = [float(v) for v in tok[3:6]]
                    info = _parse_information_matrix(tok, 10)
                    # tool convention: target = smaller id, Z = T_target^-1 T_source
                    if i < j:
                        Z, src, tgt = Zij, j, i
                    else:
                        Z, src, tgt = np.linalg.inv(Zij), i, j
                        Ad = _adjoint(Z)                # information of the inverted measurement
                        info = Ad.T @ info @ Ad
                    self.existing.append(dict(line=k, source_id=src, target_id=tgt, Z=Z, information=info))
                    continue
            keep.append(raw)
        with tempfile.NamedTemporaryFile('w', suffix='.g2o', delete=False) as f:
            f.writelines(keep)
            self._base_g2o = Path(f.name)
        res = build_factor_graph(session_root=session_root, g2o_path=self._base_g2o,
                                 gtsam_mod=gtsam_mod, log_fn=None)
        self.base_graph, self.initial = res.graph, res.initial_values
        self.n = res.pose_count
        _log(log_fn, f'[LoopAudit] base graph: {self.n} poses, {int(self.base_graph.size())} factors, '
                     f'{len(self.existing)} loop edges in the g2o')

    def solve(self, loops: Sequence[AuditLoop], initial=None) -> np.ndarray:
        g = self.gtsam
        graph = self.base_graph.clone()
        for L in loops:
            graph.add(g.BetweenFactorPose3(g.Symbol('x', L.target_id).key(), g.Symbol('x', L.source_id).key(),
                                           g.Pose3(L.Z), g.noiseModel.Gaussian.Information(L.information)))
        p = g.LevenbergMarquardtParams()
        p.setMaxIterations(100)
        out = g.LevenbergMarquardtOptimizer(graph, initial if initial is not None else self.initial, p).optimize()
        return np.stack([out.atPose3(g.Symbol('x', i).key()).matrix() for i in range(self.n)])


def _adjoint(T: np.ndarray) -> np.ndarray:
    """Adjoint of SE(3) in GTSAM order (rotation first)."""
    R, t = T[:3, :3], T[:3, 3]
    tx = np.array([[0, -t[2], t[1]], [t[2], 0, -t[0]], [-t[1], t[0], 0]])
    A = np.zeros((6, 6))
    A[:3, :3] = R
    A[3:, 3:] = R
    A[3:, :3] = tx @ R
    return A


# ----------------------------------------------------------------------------- map score
class ConsistencyScore:
    """Normal agreement ratio of cross-visit overlaps in the stitched keyframe scans."""

    def __init__(self, load_points: Callable[[int], np.ndarray], timestamps: np.ndarray,
                 cfg: AuditConfig, log_fn: LogFn = None):
        import open3d as o3d
        self.cfg = cfg
        rng = np.random.default_rng(cfg.seed)
        P, N, PL, S = [], [], [], []
        for i in range(len(timestamps)):
            pts = load_points(i)
            pts = pts[np.isfinite(pts).all(1)]
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts)).voxel_down_sample(cfg.voxel)
            pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=2.5 * cfg.voxel, max_nn=20))
            p = np.asarray(pc.points)
            if len(p) < 10:
                continue
            _, nn = cKDTree(p).query(p, 10)
            Q = p[nn] - p[nn].mean(1, keepdims=True)
            ev = np.linalg.eigvalsh(np.einsum('nki,nkj->nij', Q, Q))
            planar = ev[:, 0] / np.maximum(ev.sum(1), 1e-12) < 0.02
            sel = rng.choice(len(p), min(cfg.points_per_scan, len(p)), replace=False)
            P.append(p[sel]); N.append(np.asarray(pc.normals)[sel]); PL.append(planar[sel])
            S.append(np.full(len(sel), i))
        self.P, self.N, self.PL, self.S = map(np.concatenate, (P, N, PL, S))
        self.t = np.asarray(timestamps)
        q = np.nonzero(self.PL)[0]
        if len(q) > cfg.queries:
            q = np.sort(rng.choice(q, cfg.queries, replace=False))
        self.query = q
        self.cos_min = float(np.cos(np.deg2rad(cfg.normal_deg)))
        _log(log_fn, f'[LoopAudit] consistency score: {len(self.P)} points, {len(q)} planar queries')

    def __call__(self, T: np.ndarray) -> tuple[float, int]:
        """(NAR, number of overlapping queries) of the map stitched under T."""
        R, t = T[self.S, :3, :3], T[self.S, :3, 3]
        W = np.einsum('nij,nj->ni', R, self.P) + t
        Nw = np.einsum('nij,nj->ni', R, self.N)
        ids = self.query
        d, nn = cKDTree(W).query(W[ids], 16)
        far = np.abs(self.t[self.S[nn]] - self.t[self.S[ids]][:, None]) >= self.cfg.min_dt
        first = np.argmax(far, 1)
        rows = np.arange(len(ids))
        j, dj = nn[rows, first], d[rows, first]
        ov = far.any(1) & (dj < self.cfg.overlap_radius)
        if ov.sum() == 0:
            return float('nan'), 0
        cos = np.abs(np.einsum('ni,ni->n', Nw[ids[ov]], Nw[j[ov]]))
        return float((cos >= self.cos_min).mean()), int(ov.sum())


def recheck_loop(load_points, T: np.ndarray, L: AuditLoop, cfg: AuditConfig):
    """Re-register the source scan to the target window from the stored measurement."""
    import open3d as o3d
    j = L.target_id
    tgt = []
    for k in range(max(0, j - cfg.recheck_window), min(len(T), j + cfg.recheck_window + 1)):
        Tjk = np.linalg.inv(T[j]) @ T[k]
        p = load_points(k)
        tgt.append(p @ Tjk[:3, :3].T + Tjk[:3, 3])
    b = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(tgt))).voxel_down_sample(0.1)
    a = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(load_points(L.source_id))).voxel_down_sample(0.1)
    reg = o3d.pipelines.registration
    M = L.Z.copy()
    for d in (1.0, 0.3, 0.1):
        M = np.asarray(reg.registration_generalized_icp(a, b, d, M).transformation)
    ev = reg.evaluate_registration(a, b, 0.1, M)
    dt, dr = _rel_err(L.Z, M)
    return float(ev.fitness), float(ev.inlier_rmse), dt, dr


# ----------------------------------------------------------------------------- audit
def audit_loops(*, graph: LoopGraph, loops: Sequence[AuditLoop], load_points, timestamps,
                cfg: Optional[AuditConfig] = None, log_fn: LogFn = None,
                progress: Optional[Callable[[int, int], None]] = None) -> AuditResult:
    cfg = cfg or AuditConfig()
    t0 = time.perf_counter()
    loops = list(loops)
    score = ConsistencyScore(load_points, timestamps, cfg, log_fn)
    ev = {L.uid: LoopEvidence(L.uid, L.source_id, L.target_id, L.origin) for L in loops}
    n_steps, step = 2 * len(loops) + 2, 0

    def tick():
        nonlocal step
        step += 1
        if progress is not None:
            progress(step, n_steps)

    nar_none, _ = score(graph.solve([]))
    T_all = graph.solve(loops)
    nar_all, _ = score(T_all)
    tick()
    _log(log_fn, f'[LoopAudit] {len(loops)} loops: NAR without loops {nar_none:.3f}, with all loops {nar_all:.3f}')
    for L in loops:
        e = ev[L.uid]
        e.residual_t, e.residual_r = _rel_err(np.linalg.inv(T_all[L.target_id]) @ T_all[L.source_id], L.Z)
        if cfg.recheck:
            e.recheck_fitness, e.recheck_rmse, e.recheck_dt, e.recheck_dr = recheck_loop(load_points, T_all, L, cfg)
    # Stage 1: each loop alone against the graph without loops
    passed = []
    for L in loops:
        e = ev[L.uid]
        e.nar_single, _ = score(graph.solve([L]))
        e.d_single = e.nar_single - nar_none
        tick()
        if e.d_single < -cfg.tol:
            e.stage = 1
        else:
            passed.append(L)
    _log(log_fn, f'[LoopAudit] stage 1: {len(loops) - len(passed)} of {len(loops)} loops lower the NAR alone')
    # Stage 2: greedy leave-one-out within the loops that passed
    active = list(passed)
    nar_cur = score(graph.solve(active))[0] if active else nar_none
    rnd = 0
    while len(active) > 1:
        rnd += 1
        best, best_d = None, -np.inf
        for L in active:
            e = ev[L.uid]
            e.nar_without, _ = score(graph.solve([M for M in active if M.uid != L.uid]))
            e.d_loo = e.nar_without - nar_cur
            if e.d_loo > best_d:
                best, best_d = L, e.d_loo
            tick()
        _log(log_fn, f'[LoopAudit] stage 2 round {rnd}: NAR {nar_cur:.3f}, best removal {best.uid} ({best_d:+.3f})')
        if best_d <= cfg.tol:
            break
        ev[best.uid].stage = 2
        active = [M for M in active if M.uid != best.uid]
        nar_cur = score(graph.solve(active))[0]
    removed = [L.uid for L in loops if ev[L.uid].stage > 0]
    for e in ev.values():
        weak = cfg.recheck and (e.recheck_fitness < cfg.recheck_fitness or e.recheck_dt > cfg.recheck_t
                                or e.recheck_dr > cfg.recheck_r)
        if e.stage == 1:
            e.suggestion = 'remove'
            e.reason = f'alone, it lowers the NAR by {-e.d_single:.3f}'
        elif e.stage == 2:
            e.suggestion = 'remove'
            e.reason = f'conflicts with the other loops: removing it raises the NAR by {e.d_loo:.3f}'
        elif weak:
            e.suggestion, e.reason = 'check', 'the measurement does not reproduce'
        elif e.d_single > cfg.tol:
            e.suggestion, e.reason = 'keep', f'alone, it raises the NAR by {e.d_single:.3f}'
        else:
            e.suggestion, e.reason = 'redundant', 'no measurable effect on the NAR'
    res = AuditResult(nar_none=nar_none, nar_all=nar_all, nar_final=nar_cur,
                      loops=[ev[L.uid] for L in loops], removed=removed,
                      elapsed_sec=time.perf_counter() - t0, config=asdict(cfg))
    _log(log_fn, f'[LoopAudit] done in {res.elapsed_sec:.1f} s: suggest removing {len(removed)}, '
                 f'NAR {nar_all:.3f} -> {nar_cur:.3f}')
    return res


def loops_from_session(graph: LoopGraph, constraints_csv: Optional[Path]) -> list[AuditLoop]:
    out = [AuditLoop(f"existing:{k}", e['source_id'], e['target_id'], e['Z'], e['information'], 'existing')
           for k, e in enumerate(graph.existing)]
    if constraints_csv is not None and constraints_csv.is_file():
        for k, r in enumerate(csv.DictReader(constraints_csv.open())):
            if r['enabled'].strip().lower() not in ('1', 'true', 'yes'):
                continue
            Z = np.eye(4)
            Z[:3, :3] = Rotation.from_quat([float(r[c]) for c in ('qx', 'qy', 'qz', 'qw')]).as_matrix()
            Z[:3, 3] = [float(r[c]) for c in ('tx', 'ty', 'tz')]
            sr = np.deg2rad([float(r[c]) for c in ('sigma_roll_deg', 'sigma_pitch_deg', 'sigma_yaw_deg')])
            st = [float(r[c]) for c in ('sigma_tx', 'sigma_ty', 'sigma_tz')]
            out.append(AuditLoop(f"manual:{k}", int(r['source_id']), int(r['target_id']), Z,
                                 np.diag(1.0 / np.square(np.r_[sr, st])), 'manual'))
    return out


def save_result(res: AuditResult, path: Path) -> None:
    d = asdict(res)
    path.write_text(json.dumps(d, indent=1), encoding='utf-8')


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description='Audit the loop constraints of a session.')
    ap.add_argument('--session-root', type=Path, required=True)
    ap.add_argument('--g2o', type=Path, default=None)
    ap.add_argument('--constraints-csv', type=Path, default=None)
    ap.add_argument('--output', type=Path, default=None)
    ap.add_argument('--no-recheck', action='store_true')
    a = ap.parse_args(argv)
    import gtsam
    from .pcd_io import load_xyz_points
    from .session_resolver import resolve_session_paths
    from .trajectory_io import load_tum_trajectory
    paths = resolve_session_paths(session_root=a.session_root, g2o_path=a.g2o)
    traj = load_tum_trajectory(paths.tum_path)
    graph = LoopGraph(paths.session_root, paths.g2o_path, gtsam, log_fn=print)
    loops = loops_from_session(graph, a.constraints_csv)
    cache: dict[int, np.ndarray] = {}

    def load_points(i: int) -> np.ndarray:
        if i not in cache:
            cache[i] = load_xyz_points(paths.keyframe_dir / f'{i}.pcd')
        return cache[i]

    cfg = AuditConfig(recheck=not a.no_recheck)
    res = audit_loops(graph=graph, loops=loops, load_points=load_points, timestamps=traj.timestamps,
                      cfg=cfg, log_fn=print)
    for e in res.loops:
        print(f'{e.uid:14s} {e.source_id:5d}->{e.target_id:5d} {e.suggestion:9s} single {e.d_single:+.3f} '
              f'loo {e.d_loo:+.3f} res {100 * e.residual_t:6.1f} cm '
              f'refit {e.recheck_fitness:.2f} d {100 * e.recheck_dt:5.1f} cm {e.recheck_dr:.2f} deg | {e.reason}')
    if a.output:
        save_result(res, a.output)
    return 0


if __name__ == '__main__':
    sys.exit(main())
