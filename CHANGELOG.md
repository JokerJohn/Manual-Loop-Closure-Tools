# Changelog

All notable changes to this project will be documented in this file.

本项目的重要变更会记录在本文件中。

## [0.1.0] - 2026-04-18

### Added

- Standalone bilingual open-source project layout for the manual loop closure tool.
- PyQt GUI entrypoint and extracted backend catkin workspace.
- Detailed English / Chinese installation and tool documentation.
- Version-pinned `requirements.txt` and optional conda environment.
- One-command helper scripts for venv creation, environment checking, backend building, and Ubuntu dependency installation.
- README assets, screenshots, and workflow illustrations.
- GitHub Actions smoke-check workflow.

### Changed

- Adapted GUI path discovery and optimizer lookup for the standalone repository.
- Simplified backend build dependencies so the offline optimizer no longer requires Open3D CMake integration.

### Validated

- `python3 -m py_compile`
- `python launch_gui.py --help`
- `python scripts/check_env.py`
- `bash scripts/build_backend_catkin.sh`

## [Unreleased]

### Changed

- Made the standalone GUI Python-first by default while keeping the legacy C++ optimizer as an optional fallback path.
- Added validated Python/C++ parity reporting for multiple sessions and documented the observed pose, graph, map-point, and runtime differences.
- Refined the trajectory panel to emphasize `Nodes` / `Edges` selection, reduce status-badge prominence, and compact the toolbar and control rows.
- Reworked the right-side control area into persistent `Summary` / `Advanced` tabs with per-tab scrolling so the tab header remains visible while browsing long panels.
- Compressed the `Summary`, `Delta`, and `Registration` layouts, moved `MapVoxel` into `Advanced`, and simplified action labels for a denser but clearer control column.
- Improved the point-cloud viewer defaults with a top-down camera preset, active preset highlighting, and synchronized local/repo UI behavior.
- Added animated README demos for adding, replacing, and disabling loop edges.
- Added a repository-hosted wiki content set under `wiki/` with a GitHub-Wiki-ready page structure.
- Enlarged the main README screenshot and switched feature demos from a three-column layout to full-width single-column sections for clearer viewing.
- Updated the wiki content so it can be published directly to GitHub Wiki with sidebar navigation and GitHub-safe asset links.

### Validated

- `python3 -m py_compile gui/manual_loop_closure_tool.py gui/manual_loop_closure/open3d_viewer.py`
- `QT_QPA_PLATFORM=offscreen python3 launch_gui.py --help`
- Offscreen widget instantiation for the updated trajectory, tabbed control panel, and camera preset controls
