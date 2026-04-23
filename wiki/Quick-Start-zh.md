# 快速开始

[English](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Quick-Start) | [中文](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Quick-Start-zh)

## 推荐路径

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
make venv
source .venv/bin/activate
python launch_gui.py --session-root /path/to/mapping_session
```

默认情况下，工具优先使用 Python backend。

## 最小输入

你的 session 通常需要已经包含：

- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `key_point_frame/*.pcd`

## 典型流程

1. 加载 session。
2. 检查轨迹。
3. 选择节点对或已有闭环边。
4. 预览 source 和 target 点云。
5. 运行 GICP。
6. 新增 / 替换 / 禁用图改动。
7. 优化 working graph。
8. 导出最终结果。

## 恢复旧编辑项目

- `Load Session` 会恢复当前 session root 下最近一次编辑项目。
- `Open Project` 可以通过选择某个 `project_state.json` 恢复指定历史项目。

界面示意：

![Session Input](https://raw.githubusercontent.com/JokerJohn/Manual-Loop-Closure-Tools/main/assets/screenshots/session-loaded.png)

## 目录结构

- `manual_loop_projects/<project_id>/`
  - 编辑历史和恢复状态
  - `project_state.json`, `execution.log`, `operations.jsonl`
- `manual_loop_runs/<run_id>/`
  - 一次优化输出
  - 编辑后的 g2o、constraints CSV、优化后 TUM、报告、图像、run context
- `manual_loop_exports/<export_id>/`
  - 轻量级最终导出清单
  - 不再重复复制整包 run 数据

`Optimize` 会快速更新 working graph；真正生成最终地图和轨迹文件的是 `Export` 阶段。

## 快速链接

- [GUI 工作流](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/GUI-Workflow-zh)
- [图编辑逻辑](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Graph-Editing-zh)
- [常见问题排查](https://github.com/JokerJohn/Manual-Loop-Closure-Tools/wiki/Troubleshooting-zh)
