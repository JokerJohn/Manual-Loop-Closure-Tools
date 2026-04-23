# Quick Start

[English](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Quick-Start) | [中文](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Quick-Start-zh)

## Recommended Path

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
make venv
source .venv/bin/activate
python launch_gui.py --session-root /path/to/mapping_session
```

For normal usage, the Python backend is the default path.

## Minimum Input

Your session should already contain:

- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `key_point_frame/*.pcd`

## Typical Flow

1. Load a session.
2. Inspect the trajectory.
3. Pick node pairs or an existing loop edge.
4. Preview source and target clouds.
5. Run GICP.
6. Add / replace / disable graph changes.
7. Optimize the working graph.
8. Export the final result.

## Resume an Old Edit Project

- `Load Session` resumes the latest edit project under the selected session root.
- `Open Project` lets you restore a specific historical edit project by picking its `project_state.json`.

Session input example:

![Session Input](https://raw.githubusercontent.com/JokerJohn/Manual-Loop-Closure-Tools/main/assets/screenshots/session-loaded.png)

## Storage Layout

- `manual_loop_projects/<project_id>/`
  - edit history and resume state
  - `project_state.json`, `execution.log`, `operations.jsonl`
- `manual_loop_runs/<run_id>/`
  - one optimization result
  - edited g2o, constraints CSV, optimized TUM, report, plot, run context
- `manual_loop_exports/<export_id>/`
  - lightweight final-export manifest
  - no full duplicate copy of the selected run

`Optimize` updates the working graph quickly. `Export` is the stage that builds the final map and trajectory files.

## Helpful Links

- [GUI Workflow](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/GUI-Workflow)
- [Graph Editing](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Graph-Editing)
- [Troubleshooting](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Troubleshooting)
