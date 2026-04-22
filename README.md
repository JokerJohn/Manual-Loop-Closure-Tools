# Manual Loop Closure Tools

<p align="center">
  <a href="README.md"><strong>English</strong></a> | <a href="README.zh.md">中文</a>
</p>

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

<p align="center">
  <strong>Authors</strong>:
  <a href="https://github.com/JokerJohn">Xiangcheng HU</a>,
  <a href="https://github.com/zarathustr">Jin Wu</a> and
  <a href="https://github.com/Chen-Xieyuanli">Xieyuanli Chen</a><br/>
  <strong>Contact</strong>: <a href="mailto:xhubd@connect.ust.hk">xhubd@connect.ust.hk</a>
</p>

Offline manual loop-closure editing and optimization tools for LiDAR mapping pose graphs.

## Watch the Tutorial

<p align="center">
  <a href="https://youtu.be/lemd4XfPSYY">
    <img src="assets/video-tutorial-card.png" alt="YouTube demo" width="100%" />
  </a>
</p>

<p align="center">
  Click the card to watch the end-to-end YouTube demo.
</p>

## Overview

This repository packages the manual loop-closure workflow into a standalone project with:

- a PyQt GUI for trajectory inspection and point-cloud-assisted loop editing
- a Python-first offline optimizer backend for exporting new pose graphs and maps
- helper scripts for virtual environments, Python GTSAM checks, legacy backend build, environment checks, and screenshot generation

It is designed for mapping results that already contain:

- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `key_point_frame/*.pcd`

## Screenshot

<p align="center">
  <img src="assets/screenshots/edge-selected.png" alt="Edge selected screenshot" width="82%" />
</p>

## Workflow

<p align="center">
  <img src="assets/workflow.png" alt="Workflow diagram" width="100%" />
</p>

The GUI lets you inspect trajectories, select node pairs or existing loop edges, preview target/source point clouds, run GICP, add or replace loop constraints, manage a working graph session, and export a new optimized map.

Compatibility note:

- Some older PALoc exports may contain `pose_graph.g2o = N+1` while `optimized_poses_tum.txt` and `key_point_frame/*.pcd` stay at `N`.
- The GUI now trims a simple trailing unmatched g2o vertex automatically so these sessions can still be opened.
- Regenerating the PALoc export after fixing the PALoc save path is still the recommended long-term solution.

## Feature Demos

### Add Loop

<p align="center">
  <img src="assets/add_loopsx3.gif" alt="Add loop demo" width="82%" />
</p>

Add a new manual loop after validating a GICP result.

### Replace Loop

<p align="center">
  <img src="assets/replace_loopx3.gif" alt="Replace loop demo" width="82%" />
</p>

Replace an existing loop edge with a better manual registration result.

### Disable Loop

<p align="center">
  <img src="assets/disable_loop.gif" alt="Disable loop demo" width="82%" />
</p>

Temporarily disable an existing loop edge before re-optimization.

## Key Features

| Feature | Description |
|---|---|
| Embedded PyQt + Open3D viewer | Inspect trajectories and point clouds in one workflow |
| `Working` / `Original` trajectory comparison | Compare the edited graph against the baseline graph |
| Manual edge add, replace, disable, restore | Control loop constraints explicitly in a working session |
| Interactive source alignment in point-cloud view | Refine source initialization before GICP |
| Auto yaw sweep for ground robots | Search yaw seeds before final registration |
| Offline export of `g2o`, `TUM`, map, and trajectory PCD | Produce clean optimized outputs after validation |
| Session-based graph editing with undo and change tracking | Keep a controlled workflow while revising loop edges |

## Quick Start

### Fastest path: Docker

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
make docker-build
xhost +local:docker
docker run --rm -it \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /path/to/mapping_session:/data/session \
  manual-loop-closure-tools:latest \
  python launch_gui.py --session-root /data/session
```

Docker FAQ for first-time users:

- Need a display permission first: `xhost +local:docker`
- Mount your host session to `/data/session`
- All exported outputs stay on the host under `manual_loop_runs/`
- Headless usage is supported through the Python optimizer CLI
- Full details: [docs/DOCKER.md](docs/DOCKER.md)

### Recommended local path: `requirements.txt` + `.venv`

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
make venv
source .venv/bin/activate
make gtsam-python
python launch_gui.py --session-root /path/to/mapping_session
```

For normal use, you can stop here. ROS, catkin, and the legacy C++ optimizer are optional.

The Python GTSAM 4.3 wrapper now has a repository helper path:

- `make gtsam-python`
- or `bash scripts/install_gtsam_python.sh`
- details: [docs/INSTALL_GTSAM_PYTHON.md](docs/INSTALL_GTSAM_PYTHON.md)

You can also point directly to a `g2o` file:

```bash
python launch_gui.py --g2o /path/to/pose_graph.g2o
```

### Alternative path: conda

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
conda env create -f environment.yml
conda activate manual-loop-closure
python launch_gui.py --session-root /path/to/mapping_session
```

## Python-First Optimizer Path

After reviewing the pose-graph optimization path against the current C++ implementation, this repository now treats the Python backend as the normal runtime path for the validated manual loop-closure workflow.

The legacy C++ optimizer is retained only as:

- a developer fallback
- a parity reference
- a regression benchmark path

It is no longer required for normal installation or GUI usage.

If you still want the legacy backend locally:

```bash
cd ~/my_git/Mannual-Loop-Closure-Tools
make backend
```

By default, the GUI now exposes only the Python-first path. The legacy C++ selector stays hidden unless a developer explicitly enables it.

## Test Data

You can download a sample mapping session for quick validation here:

- Google Drive: https://drive.google.com/file/d/1iu3wO5YsiIl9ZuWlSlXz2fmu6ujTBw5J/view?usp=drive_link

## Tested Environment

The current repository content was tested with the following dependency versions on Ubuntu 20.04. Python-only usage is the primary path; the ROS/catkin backend is kept as a fallback.

| Ubuntu | ROS | Python | catkin_tools | CMake | GCC / G++ |
|---|---|---|---|---|---|
| 20.04 | Noetic (fallback) | 3.10.16 | 0.9.4 | 3.25.0 | 9.4.0 |

| Open3D | PyQt5 | Qt | NumPy | SciPy | Matplotlib | OpenCV | PCL | GeographicLib | GTSAM |
|---|---|---|---|---|---|---|---|---|---|
| 0.19.0 | 5.15.10 | 5.15.2 | 1.24.4 | 1.14.1 | 3.10.8 | 4.2.0 | 1.10.0 | 1.50.1 | 4.3.0 |

## Output Artifacts

After optimization, the tool exports a new run directory under the input session:

- `edited_input_pose_graph.g2o`
- `manual_loop_constraints.csv`
- `pose_graph.g2o`
- `optimized_poses_tum.txt`
- `global_map_manual_imu.pcd`
- `trajectory.pcd`
- `pose_graph.png`
- `manual_loop_report.json`

## Python vs C++ Parity Validation

The Python backend was validated against the legacy C++ optimizer on multiple real sessions. The GUI now defaults to Python and only falls back to C++ when Python optimization fails and a legacy binary is available.

| Session | Constraints | Python time [s] | C++ time [s] | TUM max t err [m] | TUM max r err [rad] | g2o max t err [m] | g2o max r err [rad] | Map points Py / C++ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `office_fs_fastlio_saved` | 1 | 10.193 | 2.096 | 5.20e-09 | 3.11e-06 | 6.85e-05 | 3.27e-06 | 4,850,749 / 4,850,749 |
| `floor34-1_fs_fastlio_saved` | 1 | 16.335 | 3.827 | 1.00e-09 | 3.56e-06 | 6.83e-05 | 4.50e-06 | 13,771,605 / 13,771,605 |
| `dr_tunnel_2026_01_24_145439` | 0 | 12.112 | 2.909 | 2.49e-08 | 2.87e-09 | 4.99e-04 | 1.05e-06 | 2,139,789 / 2,139,789 |

Notes:

- `optimized_poses_tum.txt` is already numerically aligned at the `1e-9 m` to `1e-8 m` translation level and `1e-6 rad` rotation level.
- The remaining `pose_graph.g2o` difference mainly comes from export text precision and quaternion sign-equivalent representations, not from optimizer mismatch.

## Repository Layout

```text
Mannual-Loop-Closure-Tools/
├── README.md
├── README.zh.md
├── CHANGELOG.md
├── CONTRIBUTING.md
├── LICENSE
├── Makefile
├── Dockerfile
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
├── scripts/
└── wiki/
```

## Documentation

- [中文 README](README.zh.md)
- [Installation Guide](docs/INSTALL.md)
- [Python GTSAM 4.3](docs/INSTALL_GTSAM_PYTHON.md)
- [Docker Guide](docs/DOCKER.md)
- [Tool Manual](docs/TOOL_README.md)
- [GitHub Wiki](https://github.com/JokerJohn/Mannual-Loop-Closure-Tools/wiki)
- [Contributing](CONTRIBUTING.md)
- [Changelog](CHANGELOG.md)

## Developer Utilities

```bash
make help
make gtsam-python
make check
make env-check
make optimizer-help
make docker-build
make backend
make assets SESSION_ROOT=/path/to/session
```

## Open-Source Notes

This repository focuses on the standalone manual-loop-closure workflow only. It does not include the full online mapping stack.

This project is derived from and complements the broader **MS-Mapping** research and codebase:

- Project URL: https://github.com/JokerJohn/MS-Mapping

If you use this repository in academic work, please also cite the MS-Mapping paper:

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

## License

This standalone repository is released under the GNU General Public License v3.0 (GPLv3).
