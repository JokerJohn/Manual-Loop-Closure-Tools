# Installation Guide | 安装说明

## Scope | 适用范围

This guide targets Ubuntu 20.04 and matches the versions currently used to test this repository. The Python-only optimizer path is the primary recommendation, while the ROS/catkin backend is now a legacy fallback.

本说明面向 Ubuntu 20.04，并与当前仓库已经验证过的版本保持一致。当前主推荐路径是 Python-only optimizer，ROS/catkin backend 已降级为 legacy fallback。

For most users, ROS/catkin can be skipped entirely. If Python GTSAM 4.3 is installed, the GUI works normally without any C++ build.

对大多数用户来说，可以完全跳过 ROS/catkin。只要 Python GTSAM 4.3 已安装，GUI 就能在不编译任何 C++ 代码的情况下正常使用。

## Tested Versions | 已测试版本

| Dependency | Version | Notes |
|---|---:|---|
| Python | 3.10.16 | GUI environment |
| Open3D | 0.19.0 | GUI point-cloud viewer |
| PyQt5 | 5.15.10 | GUI |
| NumPy | 1.24.4 | GUI |
| SciPy | 1.14.1 | GUI / transforms |
| Matplotlib | 3.10.8 | Trajectory view |
| ROS | Noetic (fallback) | Legacy backend build/runtime |
| catkin_tools | 0.9.4 | Backend build |
| GCC | 9.4.0 | Backend build |
| CMake | 3.25.0 | Backend build |
| OpenCV | 4.2.0 | Backend |
| PCL | 1.10.0 | Backend |
| GeographicLib | 1.50.1 | Backend |
| GTSAM | 4.3.0 | Backend |

## 1. Clone the Repository | 1. 克隆仓库

```bash
cd ~/my_git
git clone git@github.com:JokerJohn/Mannual-Loop-Closure-Tools.git
cd Mannual-Loop-Closure-Tools
```

## 2. Install System Dependencies | 2. 安装系统依赖

Run the helper script only if you also want the legacy C++ fallback backend.

仅当你还需要 legacy C++ fallback backend 时，才需要在 ROS Noetic 软件源已经配置好的前提下运行辅助脚本安装系统依赖。

```bash
bash scripts/install_ubuntu20.sh
```

What the script installs:

脚本会安装以下依赖：

- build tools: `build-essential`, `cmake`, `git`, `pkg-config`
- ROS packages: `roscpp`, `rospy`, `rosbag`, `tf`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `image_transport`, `cv_bridge`, `pcl_conversions`, `visualization_msgs`, `std_srvs`
- system libraries: `libpcl-dev`, `libopencv-dev`, `libboost-all-dev`, `libgeographic-dev`, `libzstd-dev`
- utilities: `python3-pip`, `python3-venv`, `python3-catkin-tools`

## 3. Create the Python Environment | 3. 创建 Python 环境

Recommended default path:

默认推荐方式：

```bash
make venv
source .venv/bin/activate
```

This uses the version-pinned [requirements.txt](../requirements.txt) and creates a local virtual environment under `.venv/`.

这条路径会使用带版本号约束的 [requirements.txt](../requirements.txt)，并在仓库根目录创建 `.venv/` 本地虚拟环境。

Manual equivalent:

手动等价命令：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

Alternative with conda:

也可使用 conda：

```bash
conda env create -f environment.yml
conda activate manual-loop-closure
```

## 4. Install Python GTSAM 4.3 | 4. 安装 Python GTSAM 4.3

The Python optimizer is designed to match the current C++ backend semantics and was tested against the `GTSAM 4.3` line.

Python 优化器按当前 C++ backend 语义对齐设计，并以 `GTSAM 4.3` 线为目标进行了测试。

Use the dedicated installation notes here:

请参考这里的专用安装说明：

- [INSTALL_GTSAM_PYTHON.md](INSTALL_GTSAM_PYTHON.md)

## 5. Launch the GUI (Python-first path) | 5. 启动 GUI（Python 主路径）

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
source .venv/bin/activate
python launch_gui.py --session-root /path/to/mapping_session
```

Or:

或者：

```bash
python launch_gui.py --g2o /path/to/pose_graph.g2o
```

## 6. Optional: Build the Legacy C++ Backend | 6. 可选：编译 Legacy C++ Backend

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
make backend
```

This builds:

该步骤会编译：

- package: `manual_loop_closure_backend`
- binary: `manual_loop_optimize`

You do not need this step for the default Python workflow. Keep it only if you want a local fallback backend or parity benchmarking.

默认 Python 工作流不需要这一步。只有当你希望保留本地 fallback backend 或做 parity 对照测试时，才需要编译它。

Expected binary path after a successful build:

编译成功后的二进制路径通常为：

```bash
backend/catkin_ws/devel/lib/manual_loop_closure_backend/manual_loop_optimize
```

## 7. Verify the Environment | 7. 检查环境

```bash
make env-check
```

This script prints Python package versions, Python GTSAM availability, ROS / catkin presence, common GTSAM CMake paths, and backend build hints.

该脚本会打印 Python 包版本、Python GTSAM 可用性、ROS / catkin 状态、常见 GTSAM CMake 路径以及后端构建提示。

## 8. Expected Input Layout | 8. 输入目录结构

```text
mapping_session/
├── key_point_frame/
│   ├── 0.pcd
│   ├── 1.pcd
│   └── ...
├── pose_graph.g2o
└── optimized_poses_tum.txt
```

The tool also supports sessions where `pose_graph.g2o` and `optimized_poses_tum.txt` are stored under the latest timestamp subdirectory, while `key_point_frame/` remains at the session root.

工具也支持这样的 session：`pose_graph.g2o` 和 `optimized_poses_tum.txt` 位于最新时间戳子目录下，而 `key_point_frame/` 仍位于 session 根目录。

## Troubleshooting | 故障排查

### Open3D cannot be imported | 无法导入 Open3D

- Make sure you launched the GUI from the tested conda or venv environment.
- The GUI will try to re-launch itself with a compatible Python interpreter if possible.

- 请确认 GUI 在已安装依赖的 conda 或 venv 环境中启动。
- GUI 会尽量自动切换到可导入依赖的 Python 解释器。

### Backend optimizer not found | 找不到后端优化器

- Install Python GTSAM 4.3 first if you want the Python backend.
- Re-run `bash scripts/build_backend_catkin.sh` only when you need the legacy fallback backend.
- Or set `MANUAL_LOOP_OPTIMIZER_BIN=/absolute/path/to/manual_loop_optimize`

- 如果你希望走 Python backend，先安装 Python GTSAM 4.3。
- 仅当你需要 legacy fallback backend 时，再重新执行 `bash scripts/build_backend_catkin.sh`
- 或手动设置 `MANUAL_LOOP_OPTIMIZER_BIN=/absolute/path/to/manual_loop_optimize`

### GTSAM not found by CMake | CMake 找不到 GTSAM

Export a CMake prefix before building:

构建前先导出 CMake 前缀：

```bash
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
```

## Related Documentation | 相关文档

- [Tool Manual / 工具说明](TOOL_README.md)
- [Python GTSAM 4.3 安装 / Python GTSAM 4.3](INSTALL_GTSAM_PYTHON.md)
- [Project Overview / 项目总览](../README.md)
