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

- Compressed the homepage hero card again by renaming `Graph Changes` to `Graph Ops` and shortening the export call-to-action.
- Hid the legacy C++ backend selector from the normal GUI path and kept it only for explicit developer mode while preserving automatic fallback behavior.
- Added a one-command `make gtsam-python` / `scripts/install_gtsam_python.sh` path for installing the GTSAM 4.3 Python wrapper into the active virtual environment.
- Added first-party Docker support with a repository `Dockerfile`, `.dockerignore`, and `docs/DOCKER.md`.
- Updated installation docs and both README variants to emphasize Docker and Python-first setup while keeping the C++ backend as a developer-only reference path.
- Made the standalone GUI Python-first by default while keeping the legacy C++ optimizer as an optional fallback path.
- Added validated Python/C++ parity reporting for multiple sessions and documented the observed pose, graph, map-point, and runtime differences.
- Refined the trajectory panel to emphasize `Nodes` / `Edges` selection, reduce status-badge prominence, and compact the toolbar and control rows.
- Reworked the right-side control area into persistent `Summary` / `Advanced` tabs with per-tab scrolling so the tab header remains visible while browsing long panels.
- Compressed the `Summary`, `Delta`, and `Registration` layouts, moved `MapVoxel` into `Advanced`, and simplified action labels for a denser but clearer control column.
- Improved the point-cloud viewer defaults with a top-down camera preset, active preset highlighting, and synchronized local/repo UI behavior.
- Improved PALoc-style session loading by ignoring stale explicit g2o paths outside the selected session root and trimming simple trailing unmatched g2o vertices.
- Added animated README demos for adding, replacing, and disabling loop edges.
- Added a repository-hosted wiki content set under `wiki/` with a GitHub-Wiki-ready page structure.
- Enlarged the main README screenshot and switched feature demos from a three-column layout to full-width single-column sections for clearer viewing.
- Updated the wiki content so it can be published directly to GitHub Wiki with sidebar navigation and GitHub-safe asset links.
- Added a prominent author/contact block to the README header.
- Linked the author names in the README header to their GitHub profiles and corrected the `Xieyuanli Chen` spelling.
- Switched the repository front page to English-first README files with a dedicated `README.zh.md` language counterpart.
- Reworked the GitHub Wiki content into English-first pages with matching Chinese switch pages instead of line-by-line bilingual mixing.
- Simplified the hero banner and tutorial card visuals to avoid text overflow on the homepage assets.
- Updated the contact email to `xhubd@connect.ust.hk`.
- Added a beginner-friendly Docker FAQ and linked it directly from both README quick-start sections.

### Validated

- `python3 -m py_compile gui/manual_loop_closure_tool.py gui/manual_loop_closure/open3d_viewer.py`
- `QT_QPA_PLATFORM=offscreen python3 launch_gui.py --help`
- Offscreen widget instantiation for the updated trajectory, tabbed control panel, and camera preset controls
