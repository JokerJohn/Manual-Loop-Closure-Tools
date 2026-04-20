# GUI 工作流

[English](GUI-Workflow.md) | [中文](GUI-Workflow-zh.md)

## 主界面结构

界面主要分为四块：

- 轨迹面板
- 点云预览面板
- 右侧控制页签（`Summary` / `Advanced`）
- 底部页签（`Graph Changes` / `Execution Log`）

## 轨迹面板

- `Nodes` 用于选择 source / target 节点对。
- `Edges` 用于检查或修订已有闭环边。
- `Working` 与 `Original` 可对比当前编辑图与原始图。
- `Ghost` 会以轻量叠加方式显示另一套轨迹。

## 点云预览

- `Preview`、`Final`、`Compare` 控制点云显示模式。
- `Top`、`Side-Y`、`Side-X` 是相机预设。
- 在编辑模式下，只有 source 可编辑，target 始终固定。

## 右侧控制页

### Summary

- summary 卡片
- delta
- registration
- actions

### Advanced

- 共享协方差参数
- backend 选择
- 导出地图体素参数

## 底部页签

- `Graph Changes` 记录已接受的手工改动及禁用/恢复动作。
- `Execution Log` 用于查看优化与 GICP 日志。
