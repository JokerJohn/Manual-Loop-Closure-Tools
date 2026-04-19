# Python GTSAM 4.3 Installation | Python GTSAM 4.3 安装

## Goal | 目标

The standalone GUI now prefers a pure Python optimizer backend. To keep the parameter behavior and numerical output aligned with the current C++ optimizer, this repository targets the **GTSAM 4.3 Python wrapper** instead of `pip gtsam 4.2`.

独立 GUI 现在优先使用纯 Python 优化后端。为了与当前 C++ 优化器在参数行为和数值输出上保持一致，本仓库默认目标是 **GTSAM 4.3 Python wrapper**，而不是 `pip gtsam 4.2`。

If this Python wrapper is installed, you can use the GUI normally without ROS, catkin, or the legacy C++ optimizer.

只要这个 Python wrapper 已安装，你就可以在不依赖 ROS、catkin 和 legacy C++ optimizer 的情况下正常使用 GUI。

## Recommended Build Path | 推荐构建路径

### 1. Install build prerequisites | 安装编译依赖

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git pkg-config \
  libboost-all-dev libtbb-dev python3-dev python3-venv python3-pip
```

### 2. Create or activate the project Python environment | 创建或激活项目 Python 环境

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip setuptools wheel
pip install -r requirements.txt
pip install pybind11-stubgen
```

### 3. Build GTSAM 4.3 Python wrappers from source | 从源码构建 GTSAM 4.3 Python wrappers

```bash
cd ~/third_party
git clone https://github.com/borglab/gtsam.git
cd gtsam
git checkout 4.3a0
mkdir -p build && cd build
cmake .. \
  -DGTSAM_BUILD_PYTHON=ON \
  -DGTSAM_USE_SYSTEM_EIGEN=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DPYTHON_EXECUTABLE=$(which python) \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$VIRTUAL_ENV
cmake --build . -j$(nproc)
cmake --build . --target python-install
```

If your local GTSAM checkout uses slightly different CMake flags for Python, keep the wrapper enabled and make sure the install prefix points to the active virtual environment.

如果你的本地 GTSAM 分支在 Python 相关 CMake 选项上略有差异，保持 Python wrapper 开启即可，同时确保安装前缀指向当前激活的虚拟环境。

`python-install` will generate Python type stubs before packaging the wheel. If it fails with `No module named pybind11_stubgen`, install `pybind11-stubgen` into the active virtual environment first, then rerun `cmake --build . --target python-install`.

`python-install` 在打包 wheel 前会先生成 Python 类型桩。如果出现 `No module named pybind11_stubgen`，先在当前虚拟环境里安装 `pybind11-stubgen`，然后重新执行 `cmake --build . --target python-install`。

### 4. Verify the wrapper | 验证 Python wrapper

```bash
source ~/my_git/Mannual-Loop-Closure-Tools/.venv/bin/activate
python - <<'PY'
import gtsam
print("gtsam module:", gtsam.__file__)
print("has LM:", hasattr(gtsam, "LevenbergMarquardtOptimizer"))
print("has BetweenFactorPose3:", hasattr(gtsam, "BetweenFactorPose3"))
print("has PriorFactorPose3:", hasattr(gtsam, "PriorFactorPose3"))
print("has GPSFactor:", hasattr(gtsam, "GPSFactor"))
print("has writeG2o:", hasattr(gtsam, "writeG2o"))
PY
```

## Repository Validation | 仓库侧验证

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
source .venv/bin/activate
python scripts/check_env.py
python gui/manual_loop_closure/python_optimizer/cli.py --help
python launch_gui.py --help
```

## Backend Preference | 后端选择

The GUI now defaults to the Python backend. If Python optimization fails at runtime and a legacy C++ binary is available, it can still fall back automatically.

GUI 现在默认使用 Python backend。若运行时 Python 优化失败且本地存在 legacy C++ 二进制，仍可自动回退。

You can override the preference with:

也可以手动指定优先级：

```bash
export MANUAL_LOOP_OPTIMIZER_BACKEND=python   # or cpp
export MANUAL_LOOP_OPTIMIZER_PYTHON=/abs/path/to/python
```

## Parity Validation Snapshot | 一致性验证快照

The current Python backend was benchmarked against the legacy C++ optimizer on multiple real sessions:

当前 Python backend 已在多组真实 session 上与 legacy C++ optimizer 做过基准对照：

| Session | Constraints | Python time [s] | C++ time [s] | TUM max t err [m] | TUM max r err [rad] | g2o max t err [m] | g2o max r err [rad] | Map points Py / C++ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `office_fs_fastlio_saved` | 1 | 10.193 | 2.096 | 5.20e-09 | 3.11e-06 | 6.85e-05 | 3.27e-06 | 4,850,749 / 4,850,749 |
| `floor34-1_fs_fastlio_saved` | 1 | 16.335 | 3.827 | 1.00e-09 | 3.56e-06 | 6.83e-05 | 4.50e-06 | 13,771,605 / 13,771,605 |
| `dr_tunnel_2026_01_24_145439` | 0 | 12.112 | 2.909 | 2.49e-08 | 2.87e-09 | 4.99e-04 | 1.05e-06 | 2,139,789 / 2,139,789 |

The residual `g2o` difference is mainly due to export text precision and quaternion sign-equivalent representations. The optimized TUM trajectories and exported map point counts already match closely enough for the validated workflow.

残余的 `g2o` 差异主要来自导出文本精度和四元数符号等价表示。对当前验证过的工作流来说，优化后的 TUM 轨迹和导出地图点数已经足够一致。

## Notes | 说明

- This repository keeps the legacy C++ optimizer only as a fallback / parity reference during the transition.
- The Python backend still exports the same result files:
  - `pose_graph.g2o`
  - `optimized_poses_tum.txt`
  - `global_map_manual_imu.pcd`
  - `trajectory.pcd`
  - `pose_graph.png`
  - `manual_loop_report.json`

- 当前仓库保留 legacy C++ optimizer，仅作为过渡期 fallback / parity 对照。
- Python backend 仍会导出相同的结果文件：
  - `pose_graph.g2o`
  - `optimized_poses_tum.txt`
  - `global_map_manual_imu.pcd`
  - `trajectory.pcd`
  - `pose_graph.png`
  - `manual_loop_report.json`
