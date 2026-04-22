# FAQ

[English](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/FAQ) | [中文](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/FAQ-zh)

## 正常使用需要 ROS 吗？

不需要。

独立工具默认采用 Python-first 工作流。只有在你希望保留 legacy C++ optimizer 作为 fallback 时，才需要 ROS / catkin。

## 为什么同时有 `Original` 和 `Working` 两套轨迹？

`Original` 是不可变的基线结果。

`Working` 是当前被编辑和重新优化的图。

## `Replace` 会覆盖原始边吗？

不会改写输入图本身。

它只会在 working session 的语义中替换当前边。

## 为什么 parity 测试里 Python 比 C++ 慢？

Python backend 的重点是更低的部署门槛和更自然的 GUI 集成。当前 parity 结果表明，它通常比 C++ 更慢，但安装和维护明显更简单。

## 为什么 PALoc 的 session 会出现 g2o 比 TUM 或 PCD 多一个顶点？

这通常是 PALoc 导出链路的问题，而不是 GUI 本身的问题。

一些较早的 PALoc 导出会把完整位姿图写入 `pose_graph.g2o`，但在保存 `optimized_poses_tum.txt` 和 `key_point_frame/*.pcd` 时漏掉最后一个关键帧。

当前 GUI 已经支持自动裁掉这种“末尾多出一个且无对应 TUM/PCD”的简单 trailing g2o 顶点，因此旧结果也能继续加载；但更推荐在修复 PALoc 保存逻辑后重新导出结果。

## 应该从哪里开始阅读？

建议阅读顺序：

1. [快速开始](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Quick-Start-zh)
2. [GUI 工作流](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/GUI-Workflow-zh)
3. [图编辑逻辑](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Graph-Editing-zh)
4. [常见问题排查](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Troubleshooting-zh)
