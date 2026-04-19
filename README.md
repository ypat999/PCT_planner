# PCT Planner

## 概述 / Overview

本项目是论文 **Efficient Global Navigational Planning in 3-D Structures Based on Point Cloud Tomography** (TMECH收录) 的实现。
基于点云断层摄影的环境理解，提供高效可扩展的全局导航框架，用于多层结构中的地面机器人导航。

**演示视频 / Demonstrations**: [pct_planner](https://byangw.github.io/projects/tmech2024/)

![demo](rsc/docs/demo.png)

---

## 中文说明

### 功能特点

- **多楼层路径规划**：支持楼梯、坡道、过桥等多层结构导航
- **GPU加速处理**：使用CuPy进行CUDA加速，实时生成环境断层图
- **3D轨迹输出**：输出包含Z坐标的完整3D路径，可直接用于机器狗控制
- **ROS2原生支持**：完整的ROS2 Humble接口，支持RViz2交互式规划

### 系统架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     PCT Planner 系统架构                                 │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐   │
│  │   Tomography    │     │    Planner      │     │    ROS2接口     │   │
│  │   断层图生成    │────▶│   路径规划      │────▶│   话题发布      │   │
│  │   (GPU加速)     │     │   (C++核心)     │     │   (Python)      │   │
│  └─────────────────┘     └─────────────────┘     └─────────────────┘   │
│         │                        │                       │              │
│         ▼                        ▼                       ▼              │
│  ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐   │
│  │ 点云 → 多层切片 │     │ A*搜索 + GPMP   │     │ /pct_path       │   │
│  │ 可通行性分析    │     │ 轨迹优化        │     │ nav_msgs/Path   │   │
│  │ 楼梯/坡道检测   │     │ 高度平滑        │     │ 3D坐标输出      │   │
│  └─────────────────┘     └─────────────────┘     └─────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### 快速开始

#### 1. 环境要求

- Ubuntu 22.04
- ROS2 Humble
- CUDA 11.x 或 12.x
- Python 3.10+

#### 2. 安装依赖

```bash
# Python依赖
pip install cupy-cuda11x open3d numpy scipy

# 注意：根据你的CUDA版本选择CuPy
# CUDA 11.x: pip install cupy-cuda11x
# CUDA 12.x: pip install cupy-cuda12x
```

#### 3. 编译

```bash
cd planner/
./build_thirdparty.sh   # 编译GTSAM和OSQP（约5-10分钟）
./build.sh              # 编译Python绑定库
```

#### 4. 设置环境变量

```bash
# 添加到 ~/.bashrc
export LD_LIBRARY_PATH=/path/to/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib:/path/to/pct_planner/planner/lib/build/src/common/smoothing:$LD_LIBRARY_PATH
export PYTHONPATH=/path/to/pct_planner/planner/lib:$PYTHONPATH
```

#### 5. 运行示例

```bash
# 解压示例PCD文件
cd rsc/pcd/
unzip pcd_files.zip

# 生成断层图
cd tomography/scripts/
python3 run_standalone.py --scene Building

# 路径规划
cd planner/scripts/
python3 plan_standalone.py --tomo building2_9 --start -5 -3 --end 5 3
```

### 输出格式

规划结果输出为 `nav_msgs/Path` 消息，包含：

```python
path_msg.poses[i].pose.position.x  # X坐标
path_msg.poses[i].pose.position.y  # Y坐标
path_msg.poses[i].pose.position.z  # Z高度（支持多楼层）
```

### 与机器狗系统集成

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   PCT Planner   │     │  EGO Planner    │     │   机器狗控制    │
│   (全局路径)    │────▶│   (局部优化)    │────▶│   (执行)        │
│   多楼层支持    │     │   3D避障        │     │   /cmd_vel      │
└─────────────────┘     └─────────────────┘     └─────────────────┘
        │                       │
        ▼                       ▼
   /pct_path              /position_cmd
   (nav_msgs/Path)        → 转换为/cmd_vel
```

### 性能指标

| 指标 | 数值 |
|------|------|
| 断层图生成 | ~40ms (85万点) |
| 路径搜索 | ~20ms |
| 轨迹优化 | ~375ms |
| 输出航点 | 1000+ |

---

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
- CUDA 11.x or 12.x
- CMake >= 3.22, GCC >= 11, Eigen3

> **Note:** The original codebase targeted ROS1 Noetic. This fork has been fully ported to **ROS2 Humble** and tested on Ubuntu 22.04.

### Python

- Python >= 3.10
- [CuPy](https://docs.cupy.dev/en/stable/install.html) matching your CUDA version
- Open3D
- NumPy >= 2.x, SciPy

```bash
pip install cupy-cuda11x open3d numpy scipy
```

## Build & Install

Inside the package, there are two modules: the point cloud tomography module for tomogram reconstruction (in **tomography/**) and the planner module for path planning and optimization (in **planner/**).

Build the planner module:

```bash
cd planner/
./build_thirdparty.sh   # builds GTSAM 4.1.1 and OSQP from source (~5–10 min)
./build.sh              # builds the pybind11 .so modules
```

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
python3 plan_standalone.py --tomo building2_9 --start -5 -3 --end 5 3
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
