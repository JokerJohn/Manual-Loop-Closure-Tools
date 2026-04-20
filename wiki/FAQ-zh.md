# FAQ

[English](FAQ.md) | [中文](FAQ-zh.md)

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

## 应该从哪里开始阅读？

建议阅读顺序：

1. [快速开始](Quick-Start-zh.md)
2. [GUI 工作流](GUI-Workflow-zh.md)
3. [图编辑逻辑](Graph-Editing-zh.md)
4. [常见问题排查](Troubleshooting-zh.md)
