# Manual Loop Closure Tools

<p align="center">
  <img src="assets/hero.svg" alt="Manual Loop Closure Tools hero" width="100%" />
</p>

<p align="center">
  <a href="https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/actions/workflows/ci.yml">
    <img src="https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/actions/workflows/ci.yml/badge.svg" alt="CI" />
  </a>
  <img src="https://img.shields.io/badge/python-3.10-blue.svg" alt="Python 3.10" />
  <img src="https://img.shields.io/badge/optimizer-Python%20first-2E8B57.svg" alt="Python-first optimizer" />
  <img src="https://img.shields.io/badge/license-GPLv3-blue.svg" alt="GNU GPL v3.0 License" />
</p>

Offline manual loop closure editing and optimization tools for LiDAR mapping pose graphs.

用于激光雷达建图位姿图的离线手动闭环编辑与优化工具。

## Watch the Tutorial | 视频教程

<p align="center">
  <a href="https://youtu.be/lemd4XfPSYY">
    <img src="assets/video-tutorial-card.png" alt="Watch the tutorial on YouTube" width="100%" />
  </a>
</p>

<p align="center">
  Click the card to watch the end-to-end workflow demo on YouTube.<br/>
  点击上方卡片可观看完整工具演示视频。
</p>

## Overview | 项目简介

This repository packages the manual loop-closure workflow into a standalone open-source project with:

本仓库将手动闭环工作流整理为一个独立的开源项目，包含：

- a PyQt GUI for trajectory inspection and point-cloud-assisted loop editing
- a Python-first offline optimizer backend for exporting new pose graphs and maps
- helper scripts for virtual environments, Python GTSAM checks, legacy backend build, environment checks, and screenshot generation

- 用于轨迹检查和点云辅助闭环编辑的 PyQt 图形界面
- 用于导出新位姿图和地图的 Python 优先离线优化后端
- 用于虚拟环境、Python GTSAM 检查、legacy 后端构建、环境检查和截图生成的辅助脚本

It is designed for mapping results that already contain:

它面向已经导出以下结果的建图任务：

- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `key_point_frame/*.pcd`

## Screenshots | 界面截图

<p align="center">
  <img src="assets/screenshots/session-loaded.png" alt="Session loaded screenshot" width="48%" />
  <img src="assets/screenshots/edge-selected.png" alt="Edge selected screenshot" width="48%" />
</p>

## Workflow | 工作流

<p align="center">
  <img src="assets/workflow.png" alt="Workflow diagram" width="100%" />
</p>

The GUI lets you inspect trajectories, select node pairs or existing loop edges, preview target/source point clouds, run GICP, add or replace loop constraints, manage a working graph session, and export a new optimized map.

图形界面支持轨迹检查、节点对和已有闭环边选择、source/target 点云预览、GICP 配准、手工新增或替换闭环约束、工作态位姿图管理，以及新优化地图导出。

## Key Features | 主要功能

| Feature | 功能 |
|---|---|
| Embedded PyQt + Open3D viewer | 内嵌式 PyQt + Open3D 点云查看器 |
| `Working` / `Original` trajectory comparison | `Working` / `Original` 双轨迹对比 |
| Manual edge add, replace, disable, restore | 手工新增、替换、禁用、恢复闭环边 |
| Interactive source alignment in point-cloud view | 在点云视图中直接调整 source 初值 |
| Auto yaw sweep for ground robots | 面向地面机器人的自动 yaw 遍历 |
| Offline optimizer exporting new `g2o`, `TUM`, map, and trajectory PCD | 离线优化器可导出新的 `g2o`、`TUM`、地图和轨迹点云 |
| Session-based graph editing with undo and change tracking | 带撤销和改动跟踪的工作会话式图编辑 |

## Quick Start | 快速开始

### Recommended path: `requirements.txt` + `.venv` | 推荐方式：`requirements.txt` + `.venv`

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
make venv
source .venv/bin/activate
python launch_gui.py --session-root /path/to/mapping_session
```

For normal use, you can stop here. ROS, catkin, and the legacy C++ optimizer are optional.

对于大多数用户，到这里就够了。ROS、catkin 和 legacy C++ optimizer 都是可选项。

Python GTSAM 4.3 wrapper installation is documented here:

Python GTSAM 4.3 wrapper 的安装说明见：

- [docs/INSTALL_GTSAM_PYTHON.md](docs/INSTALL_GTSAM_PYTHON.md)

You can also point directly to a `g2o` file:

也可以直接指定某个 `g2o` 文件：

```bash
python launch_gui.py --g2o /path/to/pose_graph.g2o
```

### Alternative path: conda | 备选方式：conda

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
conda env create -f environment.yml
conda activate manual-loop-closure
python launch_gui.py --session-root /path/to/mapping_session
```

## Legacy C++ Fallback Backend | Legacy C++ 回退后端

The GUI now defaults to the Python backend. The C++ optimizer is only kept as an optional fallback and parity reference:

GUI 现在默认使用 Python backend。C++ optimizer 仅保留为可选 fallback 和 parity 对照路径：

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
make backend
```

If the C++ backend is not installed, the GUI still works normally with the Python path.

如果没有安装 C++ backend，GUI 仍可通过 Python 路径正常工作。

## Test Data | 测试数据

You can download a sample mapping session for quick validation here:

你可以通过下面的链接下载示例建图结果，快速验证工具流程：

- Google Drive: https://drive.google.com/file/d/1iu3wO5YsiIl9ZuWlSlXz2fmu6ujTBw5J/view?usp=drive_link

## Tested Environment | 当前测试环境

The current repository content was tested with the following dependency versions on Ubuntu 20.04. Python-only usage is the primary path; the ROS/catkin backend is kept as a fallback.

当前仓库内容在 Ubuntu 20.04 环境下使用如下依赖版本进行了测试。Python-only 是主路径，ROS/catkin backend 保留为 fallback。

| Ubuntu | ROS | Python | catkin_tools | CMake | GCC / G++ |
|---|---|---|---|---|---|
| 20.04 | Noetic (fallback) | 3.10.16 | 0.9.4 | 3.25.0 | 9.4.0 |

| Open3D | PyQt5 | Qt | NumPy | SciPy | Matplotlib | OpenCV | PCL | GeographicLib | GTSAM |
|---|---|---|---|---|---|---|---|---|---|
| 0.19.0 | 5.15.10 | 5.15.2 | 1.24.4 | 1.14.1 | 3.10.8 | 4.2.0 | 1.10.0 | 1.50.1 | 4.3.0 |

## Output Artifacts | 导出结果

After optimization, the tool exports a new run directory under the input session:

优化完成后，工具会在输入 session 下生成新的运行目录：

- `edited_input_pose_graph.g2o`
- `manual_loop_constraints.csv`
- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `global_map_manual_imu.pcd`
- `trajectory.pcd`
- `pose_graph.png`
- `manual_loop_report.json`

## Python vs C++ Parity Validation | Python 与 C++ 一致性验证

The Python backend was validated against the legacy C++ optimizer on multiple real sessions. The GUI now defaults to Python and only falls back to C++ when Python optimization fails and a legacy binary is available.

Python backend 已在多组真实 session 上与 legacy C++ optimizer 做过一致性验证。GUI 现在默认走 Python，只有在 Python 优化失败且本地存在 legacy 二进制时才会隐藏回退到 C++。

| Session | Constraints | Python time [s] | C++ time [s] | TUM max t err [m] | TUM max r err [rad] | g2o max t err [m] | g2o max r err [rad] | Map points Py / C++ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `office_fs_fastlio_saved` | 1 | 10.193 | 2.096 | 5.20e-09 | 3.11e-06 | 6.85e-05 | 3.27e-06 | 4,850,749 / 4,850,749 |
| `floor34-1_fs_fastlio_saved` | 1 | 16.335 | 3.827 | 1.00e-09 | 3.56e-06 | 6.83e-05 | 4.50e-06 | 13,771,605 / 13,771,605 |
| `dr_tunnel_2026_01_24_145439` | 0 | 12.112 | 2.909 | 2.49e-08 | 2.87e-09 | 4.99e-04 | 1.05e-06 | 2,139,789 / 2,139,789 |

Notes:

- `optimized_poses_tum.txt` is already numerically aligned at the `1e-9 m` to `1e-8 m` translation level and `1e-6 rad` rotation level.
- The remaining `pose_graph.g2o` difference mainly comes from export text precision and quaternion sign-equivalent representations, not from optimizer mismatch.

说明：

- `optimized_poses_tum.txt` 已在 `1e-9 m` 到 `1e-8 m` 的平移量级和 `1e-6 rad` 的旋转量级上与 C++ 对齐。
- `pose_graph.g2o` 的残余差异主要来自导出文本精度和四元数符号等价表示，而不是优化结果失配。

## Repository Layout | 仓库结构

```text
Mannual-Loop-Closure-Tools/
├── README.md
├── CHANGELOG.md
├── CONTRIBUTING.md
├── LICENSE
├── Makefile
├── requirements.txt
├── environment.yml
├── launch_gui.py
├── assets/
├── docs/
├── gui/
├── backend/
│   └── catkin_ws/
│       └── src/
│           ├── CMakeLists.txt
│           └── manual_loop_closure_backend/
└── scripts/
```

## Documentation | 文档

- [Installation Guide / 安装说明](docs/INSTALL.md)
- [Python GTSAM 4.3 / Python GTSAM 4.3 安装](docs/INSTALL_GTSAM_PYTHON.md)
- [Tool Manual / 工具说明](docs/TOOL_README.md)
- [Contributing / 贡献说明](CONTRIBUTING.md)
- [Changelog / 版本记录](CHANGELOG.md)

## Developer Utilities | 开发辅助

```bash
make help
make check
make env-check
make optimizer-help
make backend
make assets SESSION_ROOT=/path/to/session
```

## Open-Source Notes | 开源说明

This repository focuses on the standalone manual-loop-closure workflow only. It does not include the full online mapping stack.

本仓库聚焦于独立的手动闭环工具链，不包含完整的在线建图系统。

This project is derived from and complements the broader **MS-Mapping** research and codebase:

本项目源自并服务于更完整的 **MS-Mapping** 研究与代码体系：

- Project URL / 项目地址: https://github.com/JokerJohn/MS-Mapping

If you use this repository in academic work, please also cite the MS-Mapping paper:

如果你在学术工作中使用了本仓库，也请同时引用 MS-Mapping 论文：

```bibtex
@misc{hu2024msmapping,
      title={MS-Mapping: An Uncertainty-Aware Large-Scale Multi-Session LiDAR Mapping System},
      author={Xiangcheng Hu, Jin Wu, Jianhao Jiao, Binqian Jiang, Wei Zhang, Wenshuo Wang and Ping Tan},
      year={2024},
      eprint={2408.03723},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2408.03723},
}
```

## License | 许可

This standalone repository is released under the GNU General Public License v3.0 (GPLv3).

本独立仓库采用 GNU General Public License v3.0（GPLv3）。
