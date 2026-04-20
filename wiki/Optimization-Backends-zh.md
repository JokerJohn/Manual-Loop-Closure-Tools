# 优化后端

[English](Optimization-Backends.md) | [中文](Optimization-Backends-zh.md)

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
- `global_map_manual_imu.pcd`
- `trajectory.pcd`
- `pose_graph.png`
- `manual_loop_report.json`
