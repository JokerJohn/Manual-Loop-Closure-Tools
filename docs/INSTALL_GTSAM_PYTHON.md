# Python GTSAM 4.3 Installation | Python GTSAM 4.3 安装

## Goal | 目标

The standalone GUI now prefers a pure Python optimizer backend. To keep the parameter behavior and numerical output aligned with the current C++ optimizer, this repository targets the **GTSAM 4.3 Python wrapper** instead of `pip gtsam 4.2`.

独立 GUI 现在优先使用纯 Python 优化后端。为了与当前 C++ 优化器在参数行为和数值输出上保持一致，本仓库默认目标是 **GTSAM 4.3 Python wrapper**，而不是 `pip gtsam 4.2`。

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
cmake --install .
```

If your local GTSAM checkout uses slightly different CMake flags for Python, keep the wrapper enabled and make sure the install prefix points to the active virtual environment.

如果你的本地 GTSAM 分支在 Python 相关 CMake 选项上略有差异，保持 Python wrapper 开启即可，同时确保安装前缀指向当前激活的虚拟环境。

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

The GUI now prefers the Python backend first. If Python GTSAM is missing, it can still fall back to the legacy C++ backend.

GUI 现在会优先选择 Python backend。若 Python GTSAM 缺失，仍可回退到 legacy C++ backend。

You can override the preference with:

也可以手动指定优先级：

```bash
export MANUAL_LOOP_OPTIMIZER_BACKEND=python   # or cpp / auto
export MANUAL_LOOP_OPTIMIZER_PYTHON=/abs/path/to/python
```

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
