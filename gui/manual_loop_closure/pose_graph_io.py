from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set

import numpy as np

ANCHOR_VERTEX_ID = (2 ** 31) - 2


class PoseGraphValidationError(RuntimeError):
    """Raised when the g2o content does not match the expected tool constraints."""


@dataclass
class EdgeRecord:
    edge_uid: int
    source_id: int
    target_id: int
    edge_type: str
    tag: str
    line_index: Optional[int]
    raw_line: Optional[str]
    enabled: bool = True
    deletable: bool = False

    @property
    def is_loop(self) -> bool:
        return self.edge_type == "loop_existing"

    @property
    def is_odom(self) -> bool:
        return self.edge_type == "odom"

    @property
    def is_manual(self) -> bool:
        return self.edge_type == "manual_added"

    def summary(self) -> str:
        state = "enabled" if self.enabled else "disabled"
        return f"{self.edge_type} {self.target_id}->{self.source_id} ({state})"


@dataclass
class PoseGraphData:
    path: Path
    vertex_ids: List[int]
    positions_xyz: np.ndarray
    edge_records: List[EdgeRecord]
    odom_edges: List[EdgeRecord]
    loop_edges: List[EdgeRecord]
    prior_nodes: Set[int]
    gnss_nodes: Dict[int, str]
    raw_lines: List[str] = field(default_factory=list)
    vertex_line_indices: Dict[int, int] = field(default_factory=dict)
    edge_index_by_uid: Dict[int, EdgeRecord] = field(default_factory=dict)
    excluded_line_indices: Set[int] = field(default_factory=set)

    def get_edge(self, edge_uid: int) -> EdgeRecord:
        return self.edge_index_by_uid[edge_uid]


def _parse_g2o_rich(
    path: Path,
    odom_threshold: int,
) -> tuple[
    Dict[int, tuple[float, float, float]],
    Dict[int, int],
    List[EdgeRecord],
    Set[int],
    Dict[int, str],
    List[str],
]:
    vertices: Dict[int, tuple[float, float, float]] = {}
    vertex_line_indices: Dict[int, int] = {}
    prior_nodes: Set[int] = set()
    gnss_nodes: Dict[int, str] = {}
    raw_lines = path.read_text(encoding="utf-8").splitlines(keepends=True)

    pending_edges: List[tuple[int, int, str, int, str]] = []
    for line_index, raw_line in enumerate(raw_lines):
        line = raw_line.strip()
        if not line:
            continue

        if line.startswith("#"):
            tokens = line.split()
            if len(tokens) >= 4 and tokens[1] == "GNSS_PRIOR":
                try:
                    gnss_nodes[int(tokens[3])] = tokens[2]
                except ValueError:
                    pass
            continue

        tokens = line.split()
        tag = tokens[0]

        if tag.startswith("VERTEX"):
            idx = int(tokens[1])
            if idx == ANCHOR_VERTEX_ID:
                continue
            if tag == "VERTEX_SE3:QUAT":
                x, y, z = map(float, tokens[2:5])
            elif tag in {"VERTEX_SE2", "VERTEX_XY"}:
                x, y = map(float, tokens[2:4])
                z = 0.0
            else:
                continue
            vertices[idx] = (x, y, z)
            vertex_line_indices[idx] = line_index
            continue

        if not tag.startswith("EDGE") or len(tokens) < 3:
            continue

        raw_i, raw_j = int(tokens[1]), int(tokens[2])
        if raw_i == ANCHOR_VERTEX_ID or raw_j == ANCHOR_VERTEX_ID:
            prior_nodes.add(raw_j if raw_i == ANCHOR_VERTEX_ID else raw_i)
            continue

        pending_edges.append((raw_i, raw_j, tag, line_index, raw_line))

    edge_records: List[EdgeRecord] = []
    for edge_uid, (raw_i, raw_j, tag, line_index, raw_line) in enumerate(pending_edges):
        source_id = max(raw_i, raw_j)
        target_id = min(raw_i, raw_j)
        is_odom = abs(raw_i - raw_j) <= odom_threshold
        edge_type = "odom" if is_odom else "loop_existing"
        edge_records.append(
            EdgeRecord(
                edge_uid=edge_uid,
                source_id=source_id,
                target_id=target_id,
                edge_type=edge_type,
                tag=tag,
                line_index=line_index,
                raw_line=raw_line,
                enabled=True,
                deletable=not is_odom,
            )
        )

    return vertices, vertex_line_indices, edge_records, prior_nodes, gnss_nodes, raw_lines


def load_pose_graph(path: Path, odom_threshold: int = 1) -> PoseGraphData:
    vertices, vertex_line_indices, edge_records, prior_nodes, gnss_nodes, raw_lines = _parse_g2o_rich(
        path,
        odom_threshold,
    )
    vertex_ids = sorted(vertices)
    if not vertex_ids:
        raise PoseGraphValidationError(f"No vertices found in pose graph: {path}")

    expected_ids = list(range(len(vertex_ids)))
    if vertex_ids != expected_ids:
        raise PoseGraphValidationError(
            "v1 requires contiguous g2o vertex ids starting from 0. "
            f"Got ids [{vertex_ids[0]}..{vertex_ids[-1]}] with {len(vertex_ids)} vertices."
        )

    positions_xyz = np.array([vertices[idx] for idx in vertex_ids], dtype=np.float64)
    odom_edges = [edge for edge in edge_records if edge.edge_type == "odom"]
    loop_edges = [edge for edge in edge_records if edge.edge_type == "loop_existing"]
    edge_index_by_uid = {edge.edge_uid: edge for edge in edge_records}

    return PoseGraphData(
        path=path,
        vertex_ids=vertex_ids,
        positions_xyz=positions_xyz,
        edge_records=edge_records,
        odom_edges=odom_edges,
        loop_edges=loop_edges,
        prior_nodes=prior_nodes,
        gnss_nodes=gnss_nodes,
        raw_lines=raw_lines,
        vertex_line_indices=vertex_line_indices,
        edge_index_by_uid=edge_index_by_uid,
    )


def align_pose_graph_to_frame_count(
    pose_graph: PoseGraphData,
    frame_count: int,
) -> tuple[PoseGraphData, Optional[str]]:
    current_count = len(pose_graph.vertex_ids)
    if current_count == frame_count:
        return pose_graph, None
    if current_count < frame_count:
        raise PoseGraphValidationError(
            "g2o vertex count is smaller than the available TUM/PCD frame count: "
            f"{current_count} vs {frame_count}"
        )

    trailing_ids = pose_graph.vertex_ids[frame_count:]
    expected_trailing_ids = list(range(frame_count, current_count))
    if trailing_ids != expected_trailing_ids:
        raise PoseGraphValidationError(
            "g2o contains extra vertices that are not a simple trailing range. "
            f"Expected trailing ids {expected_trailing_ids[:3]}... but got {trailing_ids[:3]}..."
        )

    kept_ids = pose_graph.vertex_ids[:frame_count]
    removed_ids = set(trailing_ids)
    kept_edges = [
        edge
        for edge in pose_graph.edge_records
        if edge.source_id not in removed_ids and edge.target_id not in removed_ids
    ]
    removed_line_indices = set(pose_graph.excluded_line_indices)
    removed_line_indices.update(
        pose_graph.vertex_line_indices[vertex_id]
        for vertex_id in trailing_ids
        if vertex_id in pose_graph.vertex_line_indices
    )
    removed_line_indices.update(
        edge.line_index
        for edge in pose_graph.edge_records
        if (edge.source_id in removed_ids or edge.target_id in removed_ids) and edge.line_index is not None
    )

    odom_edges = [edge for edge in kept_edges if edge.edge_type == "odom"]
    loop_edges = [edge for edge in kept_edges if edge.edge_type == "loop_existing"]
    aligned = PoseGraphData(
        path=pose_graph.path,
        vertex_ids=kept_ids,
        positions_xyz=pose_graph.positions_xyz[:frame_count].copy(),
        edge_records=kept_edges,
        odom_edges=odom_edges,
        loop_edges=loop_edges,
        prior_nodes={node_id for node_id in pose_graph.prior_nodes if node_id < frame_count},
        gnss_nodes={node_id: kind for node_id, kind in pose_graph.gnss_nodes.items() if node_id < frame_count},
        raw_lines=pose_graph.raw_lines,
        vertex_line_indices={
            node_id: line_index
            for node_id, line_index in pose_graph.vertex_line_indices.items()
            if node_id < frame_count
        },
        edge_index_by_uid={edge.edge_uid: edge for edge in kept_edges},
        excluded_line_indices=removed_line_indices,
    )
    note = (
        "Trimmed "
        f"{current_count - frame_count} trailing g2o vertex/vertices without matching TUM/PCD frames."
    )
    return aligned, note


def write_filtered_pose_graph(pose_graph: PoseGraphData, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    disabled_line_indices = {
        edge.line_index
        for edge in pose_graph.loop_edges
        if not edge.enabled and edge.line_index is not None
    }

    with output_path.open("w", encoding="utf-8") as stream:
        for line_index, raw_line in enumerate(pose_graph.raw_lines):
            if line_index in disabled_line_indices or line_index in pose_graph.excluded_line_indices:
                continue
            stream.write(raw_line)
