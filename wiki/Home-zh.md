# Manual Loop Closure Tools Wiki

[English](Home.md) | [中文](Home-zh.md)

本 Wiki 是独立手动闭环工作流的实用使用手册，重点介绍工具操作、图编辑逻辑、优化后端行为和常见问题。

**作者**：[Xiangcheng HU](https://github.com/JokerJohn)、[Jin Wu](https://github.com/zarathustr)、[Xieyuanli Chen](https://github.com/Chen-Xieyuanli)  
**联系邮箱**：[xhubd@connect.ust.hk](mailto:xhubd@connect.ust.hk)

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/screenshots/edge-selected.png" alt="Manual Loop Closure Tools screenshot" width="82%" />
</p>

## 从这里开始

- [快速开始](Quick-Start-zh.md)
- [GUI 工作流](GUI-Workflow-zh.md)
- [图编辑逻辑](Graph-Editing-zh.md)
- [优化后端](Optimization-Backends-zh.md)
- [常见问题排查](Troubleshooting-zh.md)
- [FAQ](FAQ-zh.md)

## 工具定位

本项目是一个面向激光雷达建图结果的离线闭环编辑工具，输入通常已经包含：

- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `key_point_frame/*.pcd`

它可以帮助你：

- 对比 `Original` 和 `Working` 两套轨迹
- 选中节点对或已有闭环边
- 在地图坐标系下预览 source / target 点云
- 运行 GICP 并验证闭环候选
- 新增、替换、禁用、恢复并导出图改动

## 核心编辑动作

### 新增手工闭环边

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/add_loopsx3.gif" alt="Add loop demo" width="82%" />
</p>

### 替换已有闭环边

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/replace_loopx3.gif" alt="Replace loop demo" width="82%" />
</p>

### 禁用已有闭环边

<p align="center">
  <img src="https://raw.githubusercontent.com/JokerJohn/Mannual-Loop-Closure-Tools/main/assets/disable_loop.gif" alt="Disable loop demo" width="82%" />
</p>

## 建议阅读顺序

1. [快速开始](Quick-Start-zh.md)
2. [GUI 工作流](GUI-Workflow-zh.md)
3. [图编辑逻辑](Graph-Editing-zh.md)
4. [优化后端](Optimization-Backends-zh.md)
5. [常见问题排查](Troubleshooting-zh.md)

## 仓库链接

- [English README](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/README.md)
- [中文 README](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/README.zh.md)
- [安装说明](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/docs/INSTALL.md)
- [Python GTSAM 4.3 安装](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/docs/INSTALL_GTSAM_PYTHON.md)
- [工具说明](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/docs/TOOL_README.md)
- [版本记录](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/blob/main/CHANGELOG.md)
