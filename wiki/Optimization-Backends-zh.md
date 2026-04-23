# 优化后端

[English](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Optimization-Backends) | [中文](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Optimization-Backends-zh)

## Python 优先

独立项目默认使用 Python 优化后端。

原因：

- 安装更简单
- 不再强制依赖 ROS / catkin
- 与 PyQt + Open3D GUI 的集成更自然
- 已与 legacy C++ backend 做过一致性验证

## Legacy C++ 回退后端

C++ backend 仍保留为可选 fallback。

适用情况：

- 你已经有 legacy backend 环境
- 你想和历史结果做直接 parity 对比
- Python GTSAM 暂时不可用时需要回退路径

## 参数一致性

Python backend 遵循与 legacy optimizer 相同的参数优先级：

1. 显式 CLI / GUI 参数
2. `runtime_params.yaml`
3. 已验证的离线默认值

## 导出文件

两种 backend 都会导出：

- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `pose_graph.png`
- `manual_loop_report.json`

完整目录结构是：

- `manual_loop_projects/<project_id>/`
  - 编辑状态与恢复文件
- `manual_loop_runs/<run_id>/`
  - 真正的优化输出
- `manual_loop_exports/<export_id>/`
  - 指向某次 run 的最终导出清单

默认情况下，`global_map_manual_imu.pcd` 和 `trajectory.pcd` 会在 `Export` 阶段生成，这样多轮图编辑时更流畅。
