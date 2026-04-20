# Manual Loop Closure Tools Wiki

[English](Home.md) | [中文](Home-zh.md)

This wiki is a practical operator guide for the standalone manual loop closure workflow. It complements the repository README with focused pages on usage, graph-editing logic, backend behavior, and troubleshooting.

**Authors**: [Xiangcheng HU](https://github.com/JokerJohn), [Jin Wu](https://github.com/zarathustr), [Xieyuanli Chen](https://github.com/Chen-Xieyuanli)  
**Contact**: [xhubd@connect.ust.hk](mailto:xhubd@connect.ust.hk)

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/screenshots/edge-selected.png" alt="Manual Loop Closure Tools screenshot" width="82%" />
</p>

## Start Here

- [Quick Start](Quick-Start.md)
- [GUI Workflow](GUI-Workflow.md)
- [Graph Editing](Graph-Editing.md)
- [Optimization Backends](Optimization-Backends.md)
- [Troubleshooting](Troubleshooting.md)
- [FAQ](FAQ.md)

## What This Tool Does

The project is an offline loop-closure editor for LiDAR mapping sessions that already export:

- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `key_point_frame/*.pcd`

It lets you:

- inspect `Original` and `Working` trajectories
- pick node pairs or existing loop edges
- preview source / target point clouds in map coordinates
- run GICP and validate loop proposals
- add, replace, disable, restore, and export graph changes

## Core Editing Actions

### Add Manual Loop

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/add_loopsx3.gif" alt="Add loop demo" width="82%" />
</p>

### Replace Existing Loop

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/replace_loopx3.gif" alt="Replace loop demo" width="82%" />
</p>

### Disable Existing Loop

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/disable_loop.gif" alt="Disable loop demo" width="82%" />
</p>

## Suggested Reading Order

1. [Quick Start](Quick-Start.md)
2. [GUI Workflow](GUI-Workflow.md)
3. [Graph Editing](Graph-Editing.md)
4. [Optimization Backends](Optimization-Backends.md)
5. [Troubleshooting](Troubleshooting.md)

## Repository Links

- [Main README](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/README.md)
- [Chinese README](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/README.zh.md)
- [Installation Guide](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/docs/INSTALL.md)
- [Python GTSAM 4.3](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/docs/INSTALL_GTSAM_PYTHON.md)
- [Tool Manual](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/docs/TOOL_README.md)
- [Changelog](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/CHANGELOG.md)
