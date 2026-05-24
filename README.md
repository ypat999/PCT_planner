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

**方式一：ROS2交互模式（推荐）**

```bash
# 终端1：启动规划节点
source /opt/ros/humble/setup.bash
cd /home/ywj/git/3d_dog_navi_ros2/src/pct_planner
python3 run_ros2_interactive.py --scene Building --skip-tomo

# 终端2：启动RViz2可视化
source /opt/ros/humble/setup.bash
cd /home/ywj/git/3d_dog_navi_ros2/src/pct_planner
rviz2 -d rsc/rviz/pct_ros2.rviz
```

在RViz2中：
1. 点击工具栏的 "Publish Point" 按钮
2. 点击地图选择起点（绿色球体）
3. 再次点击选择终点（红色球体）
4. 自动规划并显示路径（绿色线条）

**方式二：独立运行模式**

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

**完整运行指令（已测试通过）：**

```bash
# 终端1：启动规划节点（在项目根目录运行）
source /opt/ros/humble/setup.bash
cd /home/ywj/git/3d_dog_navi_ros2/src/pct_planner
python3 run_ros2_interactive.py --scene Building --skip-tomo

# 终端2：启动RViz2可视化
source /opt/ros/humble/setup.bash
cd /home/ywj/git/3d_dog_navi_ros2/src/pct_planner
rviz2 -d rsc/rviz/pct_ros2.rviz
```

**单命令启动（自动打开RViz2）：**

```bash
cd /home/ywj/git/3d_dog_navi_ros2/src/pct_planner
./launch_ros2.sh --scene Building --skip-tomo
```

**Available scenes:**
- `Building` - 多层室内场景 (building2_9.pcd) **[默认]**
- `Clinic` - 自定义场景 (clinic.pcd)
- `Plaza` - 室外广场场景 (plaza3_10.pcd)

**启动后RViz2显示内容：**
- `/global_points` - 原始点云（白色，半透明）
- `/tomogram` - 断层图（绿色=可通行，红色=障碍）
- TF树: `world -> map`

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
| `run_ros2_interactive.py` | Main interactive ROS2 node (项目根目录). Subscribes to `/clicked_point`, plans on each start+end pair, publishes path and markers. |
| `launch_ros2.sh` | Launches `run_ros2_interactive.py` in the background and opens RViz2. Cleans up both processes on exit. |
| `tomography/scripts/run_standalone.py` | Runs tomography without ROS. Saves pickle to `rsc/tomogram/`. |
| `planner/scripts/plan_standalone.py` | Runs the planner without ROS on a saved tomogram. |
| `planner/scripts/plan_ros2.py` | ROS2 planner node, publishes path to `/pct_path`. |

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

## 与 Nav2 2D 规划器的配置对齐

PCT Planner 作为3D全局规划器，需要与 Nav2 2D 规划器（SmacPlannerHybrid + MPPI）的约束保持一致，确保全局路径和局部避障的安全边界匹配。以下是基于 `nav2_params_zg_2d.yaml` 的逐项对比和同步建议。

### 机器人几何约束

| 2D Nav2 参数 | 当前值 | PCT Planner 对应 | 当前值 | 同步建议 |
|---|---|---|---|---|
| `footprint` (local) | `[[-1.0,-0.3],[0.1,0.3],...]` | 无直接对应 | — | PCT膨胀应覆盖footprint外接圆 |
| `footprint` (global) | `[[-0.8,-0.25],[0.05,0.25],...]` | 无直接对应 | — | 同上 |
| `footprint_padding` | 0.05 | `safe_margin` | 0.4 | 含义不同，见下方说明 |
| — | — | `inflation` | 0.2 | 见下方说明 |

**说明**：2D Nav2 通过 `footprint` 精确定义机器人轮廓，PCT 使用固定距离膨胀。当前 `safe_margin=0.4 + inflation=0.2 = 0.6m` 总膨胀，而 2D Nav2 的 `inflation_radius=1.0m`。

**配置文件**：`tomography/config/scene_building_sim.py`

```python
# 当前值
trav.safe_margin = 0.4
trav.inflation = 0.2

# 建议值（与2D Nav2 inflation_radius=1.0对齐）
trav.safe_margin = 0.6   # 覆盖机器人外接圆半径 ~0.55m
trav.inflation = 0.4     # 额外安全膨胀，总膨胀半径=1.0m
```

### 速度约束

| 2D Nav2 参数 | 当前值 | PCT Planner 对应 | 当前值 | 同步建议 |
|---|---|---|---|---|
| `vx_max` | 2.0 | 无直接对应 | — | 轨迹优化不限制线速度 |
| `vx_min` | -0.5 | 无 | — | — |
| `vy_max` | 0.2 | 无（3D规划无横移概念） | — | — |
| `wz_max` | 0.8 | `max_heading_rate` | 10 | ⚠️ 需调整 |
| `ax_max/min` | ±1.0 | 无 | — | — |

**说明**：PCT 的 `max_heading_rate` 是轨迹优化中的航向变化率约束（rad/step），不是角速度。当前值 10 允许路径急转弯，与 `wz_max=0.8 rad/s` 不匹配。

**配置文件**：`planner/config/param.py`

```python
# 当前值
class ConfigPlanner():
    max_heading_rate = 10

# 建议值（与wz_max=0.8对齐，限制急转弯）
class ConfigPlanner():
    max_heading_rate = 4
```

### 代价地图 / 可通行性参数

| 2D Nav2 参数 | 当前值 | PCT Planner 对应 | 当前值 | 同步建议 |
|---|---|---|---|---|
| `resolution` (local/global) | 0.05 | `map.resolution` | 0.10 | ⚠️ 不同，见下方说明 |
| `inflation_radius` (local) | 1.0 | `safe_margin + inflation` | 0.6 | ⚠️ 不同，见上方 |
| `inflation_radius` (global) | 1.0 | 同上 | 0.6 | ⚠️ 不同 |
| `cost_scaling_factor` (local) | 20.0 | 膨胀核线性衰减 | — | 策略差异 |
| `cost_scaling_factor` (global) | 5.0 | 同上 | — | 策略差异 |
| `track_unknown_space` | true | 无 | — | — |

**分辨率说明**：PCT 用 0.10m 分辨率（2D Nav2 用 0.05m），可能导致窄通道（<0.2m）无法被检测到。将分辨率降至 0.05m 会使地图尺寸增大4倍，GPU显存需求显著增加，需评估硬件是否支持。

### 路径规划约束

| 2D Nav2 参数 | 当前值 | PCT Planner 对应 | 当前值 | 同步建议 |
|---|---|---|---|---|
| `cost_penalty` | 2.0 | `step_cost_weight` | 0.2 | ⚠️ 需调整 |
| `non_straight_penalty` | 1.2 | 无直接对应 | — | — |
| `tolerance` | 0.5 | 无（A*精确到格点） | — | — |
| `allow_unknown` | true | `cost_threshold` | 45 | 语义不同 |
| `planner_timeout` | 5.0s | 无 | — | ⚠️ 缺失 |
| `max_iterations` | 100000 | 无 | — | ⚠️ 缺失 |

**代价权重说明**：2D Nav2 的 `cost_penalty=2.0` 使高代价区域被强烈避让；PCT 的 `step_cost_weight` 通过 `init_map(cost_threshold, safe_cost_margin, resolution, n_slice, step_cost_weight, ...)` 传入，当前值 0.2 对代价的避让力度弱。

**配置文件**：`planner/scripts/planner_wrapper.py`

```python
# 当前值（init_map的第2个参数是safe_cost_margin，第5个是step_cost_weight）
self.planner.init_map(
    45, 15, self.resolution, self.n_slice, 0.2,  # step_cost_weight=0.2
    ...
)

# 建议值（增强对高代价区域的避让）
self.planner.init_map(
    45, 15, self.resolution, self.n_slice, 0.5,  # step_cost_weight=0.5
    ...
)
```

### 可通行性评估参数（PCT 特有）

这些参数是 PCT 3D 规划器特有的，2D Nav2 没有对应项，但需与机器人实际能力匹配：

| PCT 参数 | 当前值 | 含义 | 建议值 | 说明 |
|---|---|---|---|---|
| `interval_min` | 0.10 | 天花板-地面间距<0.1m视为不可通行 | 0.10 | 合理 |
| `interval_free` | 0.65 | 间距>0.65m为自由空间 | **0.80** | 机器狗站立+传感器高度可能超过0.65m |
| `step_max` | 0.30 | 最大可跨越台阶高度 | 0.30 | 与机器狗越障能力匹配 |
| `slope_max` | 0.80 | 最大可通行坡度（rad） | 0.80 | 约46°，合理 |
| `cost_barrier` | 50.0 | 不可通行代价 | 50.0 | 对应2D Nav2的lethal cost |
| `standable_ratio` | 0.20 | 可站立区域最小比例 | 0.20 | 合理 |
| `kernel_size` | 5 | 可通行性评估窗口 | 5 | 合理 |

**配置文件**：`tomography/config/scene_building_sim.py`

### 轨迹优化参数

| 2D Nav2 (MPPI) 参数 | 当前值 | PCT (GPMP) 参数 | 当前值 | 同步建议 |
|---|---|---|---|---|
| `time_steps × model_dt` | 56×0.05=2.8s | `T` (优化时间) | 200 | 预测时域 |
| `temperature` | 0.3 | `kQc` (GP先验噪声) | 0.01 | 探索vs平滑 |
| `ObstaclesCritic.cost_weight` | 50 | `safe_cost_margin` | 15 | ⚠️ 避障力度不一致 |
| `PathFollowCritic.cost_weight` | 1.0 | GP先验约束 | — | 路径跟踪 |
| `GoalCritic.cost_weight` | 50 | 无 | — | ⚠️ 缺失目标点因子 |
| `PreferForwardCritic` | 20 | `max_heading_rate` | 10 | 前进偏好 |

**避障力度说明**：2D Nav2 的 `ObstaclesCritic.cost_weight=50` 强烈避障，PCT 的 `safe_cost_margin=15`（通过 `init_map` 第二个参数传入）避让力度相对较弱。

### 修改优先级总结

#### 🔴 高优先级（影响安全性）

1. **`safe_margin` 0.4 → 0.6**，**`inflation` 0.2 → 0.4** — 与2D Nav2 `inflation_radius=1.0` 对齐
2. **`step_cost_weight` 0.2 → 0.5** — 与2D Nav2 `cost_penalty=2.0` 对齐
3. **`max_heading_rate` 10 → 4** — 与2D Nav2 `wz_max=0.8` 对齐

#### 🟡 中优先级（影响规划质量）

4. **`interval_free` 0.65 → 0.80** — 匹配机器狗站立高度
5. **A* 添加 `max_iterations` 限制** — 与2D Nav2 `max_iterations=100000` 对齐
6. **`map.resolution` 0.10 → 0.05** — 与2D Nav2分辨率对齐（⚠️ 计算量增大4倍）

#### 🟢 低优先级（优化项）

7. **膨胀衰减策略**：从线性衰减改为指数衰减，与2D Nav2 `cost_scaling_factor` 对齐
8. **轨迹优化添加目标点因子**：与2D Nav2 `GoalCritic` 对齐
9. **A* 添加搜索超时**：与2D Nav2 `planner_timeout=5.0s` 对齐

### 配置文件索引

| 配置项 | 文件 | 位置 |
|--------|------|------|
| 可通行性参数 (resolution, slope, step, inflation...) | `tomography/config/scene_building_sim.py` | `SceneTrav` / `SceneMap` |
| 轨迹优化参数 (max_heading_rate) | `planner/config/param.py` | `ConfigPlanner` |
| A*参数 (cost_threshold, step_cost_weight, safe_cost_margin) | `planner/scripts/planner_wrapper.py` | `init_map()` 调用 |
| A*搜索参数 (search_layer_depth, heuristic) | `planner/lib/src/a_star/a_star_search.h` | `Astar` 类成员 |
| GPMP优化参数 (kQc, sample_interval, max_iterations) | `planner/lib/src/trajectory_optimization/gpmp_optimizer/gpmp_optimizer.cc` | 常量/成员 |
| 膨胀核衰减函数 | `tomography/scripts/tomogram.py` | `inf_table` 计算 |

---

## License

The source code is released under [GPLv2](http://www.gnu.org/licenses/) license.

For commercial use, please contact Bowen Yang [byangar@connect.ust.hk](mailto:byangar@connect.ust.hk).
