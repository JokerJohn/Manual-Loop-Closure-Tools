# 常见问题排查

[English](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Troubleshooting) | [中文](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Troubleshooting-zh)

## GUI 能打开，但点云显示异常

请检查：

- 当前 session 是否包含正确的 `g2o`、`TUM` 和 `key_point_frame`
- `source` / `target` 是否按预期顺序选择
- 当前点云模式是否为 `Temporal Window` 或 `RS Spatial Submap`

## Python Optimizer 不可用

运行：

```bash
python scripts/check_env.py
```

并确认 Python GTSAM 能正常导入。

## C++ Backend 不存在

对于 Python-only 使用路径，这属于正常情况。

只有在你明确需要 fallback 或 parity 对比时，才需要安装 C++ backend。

## 导出的地图太稀疏

请检查 `Advanced` 中的导出地图体素参数。

- `0.0` 表示不做导出下采样
- 大于 `0.0` 的值会降低地图密度

## Session Browser 总是回到 Home

GUI 会记住最近一次成功加载的 session 路径。如果仍然回到 home，请先确认上一次 session 是否真的加载成功。

## PALoc Session 因为 `g2o` 多一个顶点而无法加载

常见现象是：

- `pose_graph.g2o` 有 `N+1` 个顶点
- `optimized_poses_tum.txt` 只有 `N` 行
- `key_point_frame/*.pcd` 只有 `N` 个文件

当前 GUI 已经能处理“仅在末尾多出一个且无对应 TUM/PCD”的简单 g2o trailing 顶点。

如果你仍然遇到加载失败，请检查：

- 当前是否误用了来自其他 session 的陈旧 `g2o` 路径
- 是否直接重新加载了 PALoc 的 session root
- 如果条件允许，建议在修复 PALoc 保存逻辑后重新导出一次结果，让 `g2o`、`TUM` 和 PCD 数量完全一致
