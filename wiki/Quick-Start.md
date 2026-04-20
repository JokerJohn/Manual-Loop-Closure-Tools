# Quick Start

[English](Quick-Start.md) | [中文](Quick-Start-zh.md)

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

## Helpful Links

- [GUI Workflow](GUI-Workflow.md)
- [Graph Editing](Graph-Editing.md)
- [Troubleshooting](Troubleshooting.md)
