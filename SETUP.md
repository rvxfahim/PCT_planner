# PCT Planner — Setup Guide (ROS2 / clinic.pcd)

Follow these steps after a fresh `git clone` on a machine that has never run this repo before.

---

## System requirements

| Requirement | Version tested |
|-------------|---------------|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| CUDA | 12.x |
| Python | 3.10 |
| CMake | ≥ 3.22 |
| GCC | ≥ 11 |
| Eigen3 | system package |

---

## Step 1 — Install system dependencies

```bash
sudo apt update
sudo apt install -y \
    libeigen3-dev \
    libboost-all-dev \
    cmake build-essential
```

---

## Step 2 — Install Python packages

```bash
# GPU-accelerated point cloud processing
pip install cupy-cuda12x          # match your CUDA major version (11.x → cupy-cuda11x)

# Point cloud I/O
pip install open3d

# Scientific computing
pip install numpy scipy

# pybind11 >= 2.12 is required for numpy 2.x compatibility.
# The repo ships pybind11 headers that have already been upgraded,
# but if you ever regenerate them:
pip install "pybind11>=3.0"
```

> **Note:** `transforms3d` is **not** required by any script in this repo.

---

## Step 3 — Source ROS2

Add this to your `~/.bashrc` (if not already present) and re-open the terminal:

```bash
source /opt/ros/humble/setup.bash
```

---

## Step 4 — Copy the PCD file

The PCD file must live under `rsc/pcd/`.

```bash
# If you have clinic.pcd at the repo root (or anywhere else):
cp /path/to/clinic.pcd /path/to/PCT_planner/rsc/pcd/clinic.pcd
```

---

## Step 5 — Build third-party C++ libraries

This compiles **GTSAM 4.1.1** and **OSQP** from source.
It only needs to be done once per machine.

```bash
cd planner
bash build_thirdparty.sh
```

Expected duration: ~5–10 minutes depending on core count.

---

## Step 6 — Build the planner pybind11 modules

```bash
cd planner        # (stay here from step 5, or cd into it again)
bash build.sh
```

This produces `.so` files under `planner/lib/`:
`a_star*.so`, `traj_opt*.so`, `ele_planner*.so`, `py_map_manager*.so`, `libcommon_smoothing.so`

---

## Step 7 — Run tomography (first time only)

This generates the traversability tomogram from the PCD and saves it as a pickle.
Re-run only when you change a parameter in `tomography/config/scene_clinic.py`.

```bash
cd tomography/scripts
python3 run_standalone.py --scene Clinic
```

Output is saved to `rsc/tomogram/clinic.pickle`.
Expected duration: a few seconds (GPU-accelerated).

---

## Step 8 — Launch the interactive ROS2 node + RViz2

Open **two terminals**, both with ROS2 sourced.

**Terminal 1 — planner node:**
```bash
cd /path/to/PCT_planner
python3 run_ros2_interactive.py --skip-tomo
```

**Terminal 2 — RViz2:**
```bash
source /opt/ros/humble/setup.bash
cd /path/to/PCT_planner
rviz2 -d rsc/rviz/pct_ros2.rviz
```

Or use the convenience launcher (opens RViz2 in the same terminal, kills the node on exit):

```bash
./launch_ros2.sh --skip-tomo
```

---

## Usage in RViz2

1. Select the **"Publish Point"** tool from the toolbar (crosshair icon).
2. **Click** a start point on the map → a green sphere appears.
3. **Click** an end point → a red sphere appears and planning begins automatically.
4. The planned path is shown as a green line on `/pct_path`.

> The z coordinate from clicked points is used to select the correct floor.
> Click on the tomogram layer (coloured by traversability) at the floor you want to navigate on for accurate floor selection.

---

## Topics published by the node

| Topic | Type | Content |
|-------|------|---------|
| `/global_points` | `sensor_msgs/PointCloud2` | Raw clinic point cloud |
| `/tomogram` | `sensor_msgs/PointCloud2` | Traversability layers (intensity = cost) |
| `/pct_path` | `nav_msgs/Path` | Planned trajectory |
| `/pct_marker` | `visualization_msgs/Marker` | Start/end spheres, path waypoints |

---

## Rebuild after code changes

| What changed | What to re-run |
|-------------|---------------|
| C++ source under `planner/lib/src/` | `cd planner && bash build.sh` |
| GTSAM or OSQP source | `cd planner && bash build_thirdparty.sh && bash build.sh` |
| `tomography/config/scene_clinic.py` | `cd tomography/scripts && python3 run_standalone.py --scene Clinic` |
| `run_ros2_interactive.py` or Python scripts | No rebuild — just restart the node |

---

## Known compatibility fixes already applied to this repo

These issues were already fixed in the committed code — listed here for reference in case the same problems appear on a new machine.

| Issue | Fix applied |
|-------|------------|
| CUDA 12/13 rejects `float16` in NVRTC kernels | `tomography/scripts/kernels.py` — changed `float16` → `float` in device function signatures |
| ROS2 rejects positional `PointField` constructor args | `tomography/config/prototype.py` — changed to keyword args |
| pybind11 2.11 segfaults with numpy 2.x | `planner/lib/3rdparty/pybind11/` headers replaced with pybind11 3.0.2 |
| Both `tomography/config` and `planner/config` are named `config` (Python import collision) | `run_ros2_interactive.py` loads them via `importlib.util` under unique module names |
| `libmetis-gtsam.so` / `libgtsam.so` not found at runtime | `run_ros2_interactive.py` preloads them with `ctypes.CDLL(..., RTLD_GLOBAL)` before any import |
