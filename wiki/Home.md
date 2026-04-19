# Manual Loop Closure Tools Wiki

欢迎来到 **Manual Loop Closure Tools** 的项目 Wiki。

This wiki is designed as a practical operator guide for the standalone manual loop closure workflow. It complements the repository `README.md` by focusing on usage, graph-editing logic, backend behavior, and troubleshooting.

本 Wiki 面向实际使用者，重点介绍独立手动闭环工具的操作流程、图编辑逻辑、优化后端行为和常见问题。

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/screenshots/edge-selected.png" alt="Manual Loop Closure Tools screenshot" width="82%" />
</p>

## Start Here | 从这里开始

- [Quick Start](Quick-Start.md)
- [GUI Workflow](GUI-Workflow.md)
- [Graph Editing](Graph-Editing.md)
- [Optimization Backends](Optimization-Backends.md)
- [Troubleshooting](Troubleshooting.md)
- [FAQ](FAQ.md)

## What This Tool Does | 工具定位

The project is an offline loop-closure editor for LiDAR mapping sessions that already export:

本项目是一个面向激光雷达建图结果的离线闭环编辑工具，输入通常已经包含：

- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `key_point_frame/*.pcd`

It lets you:

它可以帮助你：

- inspect `Original` and `Working` trajectories
- pick node pairs or existing loop edges
- preview source / target point clouds in map coordinates
- run GICP and validate loop proposals
- add, replace, disable, restore, and export graph changes

- 对比 `Original` 和 `Working` 两套轨迹
- 选中节点对或已有闭环边
- 在地图坐标系下预览 source / target 点云
- 运行 GICP 并验证闭环候选
- 新增、替换、禁用、恢复并导出图改动

## Core Editing Actions | 核心编辑动作

### Add Manual Loop | 新增手工闭环边

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/add_loopsx3.gif" alt="Add loop demo" width="82%" />
</p>

### Replace Existing Loop | 替换已有闭环边

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/replace_loopx3.gif" alt="Replace loop demo" width="82%" />
</p>

### Disable Existing Loop | 禁用已有闭环边

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/disable_loop.gif" alt="Disable loop demo" width="82%" />
</p>

## Suggested Reading Order | 建议阅读顺序

1. [Quick Start](Quick-Start.md)
2. [GUI Workflow](GUI-Workflow.md)
3. [Graph Editing](Graph-Editing.md)
4. [Optimization Backends](Optimization-Backends.md)
5. [Troubleshooting](Troubleshooting.md)

## Related Repository Links | 仓库链接

- [Main README](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/README.md)
- [Installation Guide](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/docs/INSTALL.md)
- [Python GTSAM 4.3](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/docs/INSTALL_GTSAM_PYTHON.md)
- [Tool Manual](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/docs/TOOL_README.md)
