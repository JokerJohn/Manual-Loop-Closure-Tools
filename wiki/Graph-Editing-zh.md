# 图编辑逻辑

[English](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Graph-Editing) | [中文](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Graph-Editing-zh)

## 编辑逻辑

工具明确区分以下几种图编辑动作：

- 新增手工闭环边
- 替换已有闭环边
- 禁用已有闭环边
- 恢复已禁用边

## 新增手工闭环边

适用于图中原本不存在你想加入的那条约束。

流程：

1. 选择 source / target 节点对
2. 如有必要，先调整初始位姿
3. 运行 GICP
4. 在点云视图中确认结果
5. 点击 `Add`

## 替换已有闭环边

适用于已有闭环边存在，但你认为约束不够可靠的情况。

流程：

1. 切换到 `Edges`
2. 选择已有闭环边
3. 从当前预览初值运行 GICP
4. 点击 `Replace`

内部实现上不会改写原始图，而是在 working graph 中禁用原边，再应用新的手工约束结果。

## 禁用 / 恢复

- `Disable` 只在 working graph 中移除已有闭环边。
- `Restore` 会把被禁用的边重新加入 working graph。

## 导出语义

导出结果基于优化后的 working graph 状态。

这意味着：

- 被禁用的闭环边不会进入最终导出
- 替换后的闭环以 working graph 的接受状态导出
- 原始输入文件不会被覆盖

## 项目日志与复盘

每个编辑项目现在都会写出：

- `project_state.json`
- `execution.log`
- `operations.jsonl`

这样可以：

- 回看一次手动编辑过程中做过哪些操作
- 用 `Load Session` 恢复最近一次项目
- 用 `Open Project` 恢复某个历史项目

## Run 与 Export 的区别

- `manual_loop_runs/` 保存真实优化输出
- `manual_loop_exports/` 保存指向某次 run 的轻量级导出清单

因此 `Export` 不会再重复复制整包 run 数据。
