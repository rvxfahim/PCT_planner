# PCT Planner

## Overview

This is an implementation of paper **Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography** (accepted by TMECH).
It provides a highly efficient and extensible global navigation framework based on a tomographic understanding of the environment to navigate ground robots in multi-layer structures.

**Demonstrations**: [pct_planner](https://byangw.github.io/projects/tmech2024/)

![demo](rsc/docs/demo.png)

## Citing

If you use PCT Planner, please cite the following paper:

[Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography](https://ieeexplore.ieee.org/document/10531813)

```bibtex
@ARTICLE{yang2024efficient,
  author={Yang, Bowen and Cheng, Jie and Xue, Bohuan and Jiao, Jianhao and Liu, Ming},
  journal={IEEE/ASME Transactions on Mechatronics},
  title={Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography},
  year={2024},
  volume={},
  number={},
  pages={1-12}
}
```

## Prerequisites

### Environment

- Ubuntu 22.04
- **ROS2 Humble** (ros-humble-desktop-full) — see [ROS2 install guide](https://docs.ros.org/en/humble/Installation.html)
- CUDA 12.x (tested with 12.8)
- CMake >= 3.22, GCC >= 11, Eigen3

> **Note:** The original codebase targeted ROS1 Noetic. This fork has been fully ported to **ROS2 Humble** and tested on Ubuntu 22.04 + CUDA 12.8.

### Python

- Python >= 3.10
- [CuPy](https://docs.cupy.dev/en/stable/install.html) matching your CUDA version (e.g. `cupy-cuda12x`)
- Open3D
- NumPy >= 2.x, SciPy

```bash
pip install cupy-cuda12x open3d numpy scipy
```

## Build & Install

Inside the package, there are two modules: the point cloud tomography module for tomogram reconstruction (in **tomography/**) and the planner module for path planning and optimization (in **planner/**).

Build the planner module:

```bash
cd planner/
./build_thirdparty.sh   # builds GTSAM 4.1.1 and OSQP from source (~5–10 min)
./build.sh              # builds the pybind11 .so modules
```

> See **SETUP.md** for a full step-by-step guide including common pitfalls and compatibility notes.

## Run Examples — Original Scenes (ROS2)

Three example scenarios are provided: **"Spiral"**, **"Building"**, and **"Plaza"**.
- **"Spiral"**: A spiral overpass scenario released in the [3D2M planner](https://github.com/ZJU-FAST-Lab/3D2M-planner).
- **"Building"**: A multi-layer indoor scenario with various stairs, slopes, overhangs and obstacles.
- **"Plaza"**: A complex outdoor plaza for repeated trajectory generation evaluation.

### Tomogram Construction

- Unzip the pcd files in **rsc/pcd/pcd_files.zip** to **rsc/pcd/**.
- Run the standalone tomography script (no ROS required):

```bash
cd tomography/scripts/
python3 run_standalone.py --scene Building
```

The generated tomogram is saved to **rsc/tomogram/**.

### Trajectory Generation

```bash
cd planner/scripts/
python3 plan_standalone.py --scene Building
```

---

## Running on a Custom PCD — Clinic Scene (ROS2 Interactive)

This fork adds a fully interactive ROS2 workflow where you click start/end points in **RViz2** and the planned path is published live.

### 1. Place your PCD file

```bash
cp /path/to/clinic.pcd rsc/pcd/clinic.pcd
```

### 2. Run tomography (once, or when scene config changes)

```bash
cd tomography/scripts/
python3 run_standalone.py --scene Clinic
```

Output: `rsc/tomogram/clinic.pickle`

### 3. Launch the interactive node + RViz2

**Option A — two terminals:**

```bash
# Terminal 1: planner node
source /opt/ros/humble/setup.bash
python3 run_ros2_interactive.py --skip-tomo

# Terminal 2: RViz2
source /opt/ros/humble/setup.bash
rviz2 -d rsc/rviz/pct_ros2.rviz
```

**Option B — single launcher (RViz2 opens automatically):**

```bash
./launch_ros2.sh --skip-tomo
```

### 4. Pick start and end points in RViz2

1. Select the **"Publish Point"** tool from the toolbar.
2. **Click** a start location → green sphere appears.
3. **Click** an end location → red sphere appears and planning runs automatically.
4. The planned path appears as a green line on the `/pct_path` topic.

> The z coordinate of each click is used to automatically select the correct floor/slice. Click directly on the coloured tomogram layer for the floor you want.

### ROS2 Topics

| Topic | Type | Content |
|-------|------|---------|
| `/global_points` | `sensor_msgs/PointCloud2` | Raw point cloud |
| `/tomogram` | `sensor_msgs/PointCloud2` | Traversability layers (intensity = cost) |
| `/pct_path` | `nav_msgs/Path` | Planned trajectory |
| `/pct_marker` | `visualization_msgs/Marker` | Start/end spheres, path waypoints |

---

## Scripts Reference

| Script | Description |
|--------|-------------|
| `run_ros2_interactive.py` | Main interactive ROS2 node. Subscribes to `/clicked_point`, plans on each start+end pair, publishes path and markers. |
| `launch_ros2.sh` | Launches `run_ros2_interactive.py` in the background and opens RViz2. Cleans up both processes on exit. |
| `tomography/scripts/run_standalone.py` | Runs tomography without ROS. Saves pickle to `rsc/tomogram/`. |
| `planner/scripts/plan_standalone.py` | Runs the planner without ROS on a saved tomogram. |

---

## Tunable Parameters

See **PARAMETERS.md** for a full reference of all tunable parameters including:
- Agent dimensions (footprint, collision radius, clearance height)
- Climb and step limits (max slope angle, max step height)
- Map resolution and floor-separation settings
- Planner trajectory style and optimizer weights

---

## Compatibility Fixes Applied

The following issues were found and fixed relative to the original repo:

| Issue | Fix |
|-------|-----|
| CUDA 12/13 NVRTC rejects `float16` in kernel preamble | Changed to `float` in `tomography/scripts/kernels.py` |
| ROS2 rejects positional `PointField` constructor args | Changed to keyword args in `tomography/config/prototype.py` |
| Bundled pybind11 2.11 segfaults with NumPy 2.x | Replaced headers with pybind11 3.0.2 in `planner/lib/3rdparty/pybind11/` |
| `tomography/config` and `planner/config` both named `config` (Python import collision) | Loaded via `importlib.util` under unique names in `run_ros2_interactive.py` |
| `libmetis-gtsam.so` / `libgtsam.so` not found at runtime | Preloaded with `ctypes.CDLL(..., RTLD_GLOBAL)` before imports |

---

## License

The source code is released under [GPLv2](http://www.gnu.org/licenses/) license.

For commercial use, please contact Bowen Yang [byangar@connect.ust.hk](mailto:byangar@connect.ust.hk).
