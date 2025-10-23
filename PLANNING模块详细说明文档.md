# Autoware Planning 模块详细说明文档

## 目录
1. [模块概述](#模块概述)
2. [整体架构](#整体架构)
3. [核心子模块详解](#核心子模块详解)
4. [数据流分析](#数据流分析)
5. [关键算法](#关键算法)
6. [配置参数说明](#配置参数说明)
7. [调试指南](#调试指南)

---

## 模块概述

### 什么是 Planning 模块?

Planning (规划) 模块是 Autoware 自动驾驶系统的核心组件,负责根据感知信息、地图数据和目标位置,生成安全、舒适且符合交通规则的行驶轨迹。

### 主要功能

- **任务规划 (Mission Planning)**: 从起点到目标点的全局路线规划
- **路径生成 (Path Generation)**: 根据路线生成可行驶路径
- **行为规划 (Behavior Planning)**: 根据交通规则调整行驶行为
- **速度规划 (Velocity Planning)**: 规划符合约束的速度曲线
- **轨迹优化 (Trajectory Optimization)**: 平滑和优化最终输出轨迹

---

## 整体架构

### Planning 模块层次结构

```
Planning Module
├── Mission Planning (任务规划层)
│   └── autoware_mission_planner - 全局路线规划
├── Path Generation (路径生成层)
│   ├── autoware_path_generator - 简单路径生成器
│   └── autoware_route_handler - 路由处理库
├── Behavior Planning (行为规划层)
│   └── behavior_velocity_planner - 基于交通规则的速度调整
│       ├── Stop Line Module - 停止线模块
│       ├── Traffic Light Module - 红绿灯模块
│       ├── Crosswalk Module - 人行横道模块
│       ├── Intersection Module - 路口模块
│       └── ... (其他交通场景模块)
├── Motion Planning (运动规划层)
│   └── motion_velocity_planner - 基于障碍物的速度规划
│       └── Obstacle Stop Module - 障碍物停止模块
├── Trajectory Smoothing (轨迹平滑层)
│   └── autoware_velocity_smoother - 速度平滑器
└── Utilities (辅助工具)
    ├── autoware_planning_factor_interface - 规划因子接口
    ├── autoware_planning_topic_converter - 话题转换器
    └── autoware_objects_of_interest_marker_interface - 兴趣对象标记接口
```

### 数据流向图

```
┌─────────────────────────────────────────────────────────────┐
│                      Perception & Localization               │
│  (感知层: 动态对象、点云、交通信号、自车定位等)                 │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  Mission Planner (任务规划器)                                 │
│  - 输入: 目标位置 Pose、Lanelet2 地图                          │
│  - 输出: LaneletRoute (全局路线)                               │
└────────────────────────┬────────────────────────────────────┘
                         │ LaneletRoute
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  Path Generator (路径生成器)                                  │
│  - 输入: LaneletRoute、自车位置                                │
│  - 输出: PathWithLaneId (带车道ID的路径)                       │
└────────────────────────┬────────────────────────────────────┘
                         │ PathWithLaneId
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  Behavior Velocity Planner (行为速度规划器)                   │
│  - 输入: PathWithLaneId、交通信号、动态对象                     │
│  - 功能: 根据交通规则(红绿灯、停止线、人行横道等)插入速度限制    │
│  - 输出: Path (调整后的路径)                                   │
└────────────────────────┬────────────────────────────────────┘
                         │ Path → Trajectory
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  Motion Velocity Planner (运动速度规划器)                     │
│  - 输入: Trajectory、动态对象、点云                            │
│  - 功能: 根据周围障碍物调整速度                                │
│  - 输出: Trajectory (调整后的轨迹)                             │
└────────────────────────┬────────────────────────────────────┘
                         │ Trajectory
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  Velocity Smoother (速度平滑器)                               │
│  - 输入: Trajectory                                           │
│  - 功能: 平滑速度曲线,满足加速度、加加速度约束                  │
│  - 输出: Trajectory (最终平滑轨迹)                             │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
                    Control Module (控制模块)
```

---

## 核心子模块详解

### 1. Mission Planner (任务规划器)

#### 功能描述
Mission Planner 负责计算从当前车辆位置到目标位置的全局路线。路线由 Lanelet2 地图中的一系列车道(lanelets)组成。

#### 主要特点
- **静态规划**: 不考虑动态障碍物和动态地图信息
- **插件架构**: 支持不同地图格式的规划算法插件
- **路由图**: 使用 Lanelet2 的路由图进行路径搜索

#### 关键类和文件

**1. MissionPlanner 类** (`mission_planner.hpp/cpp`)
```cpp
// 主要成员变量:
- planner_: 规划插件实例 (默认 DefaultPlanner)
- route_handler: 路由处理器
- arrival_checker_: 到达检查器
- odometry_: 车辆里程计信息
- map_ptr_: Lanelet2 地图指针

// 主要方法:
- on_set_lanelet_route(): 处理路线设置请求
- create_route(): 创建路线
- check_reroute_safety(): 检查重新规划的安全性
```

**2. ArrivalChecker 类** (`arrival_checker.hpp/cpp`)
```cpp
// 功能: 检查车辆是否到达目标点
// 检查条件:
- 距离阈值: arrival_check_distance
- 角度阈值: arrival_check_angle_deg
- 停止时长: arrival_check_duration
```

#### 输入输出

**输入**:
- `/input/vector_map` (LaneletMapBin): Lanelet2 矢量地图
- `/input/odometry` (Odometry): 车辆里程计
- `/input/operation_mode_state` (OperationModeState): 运行模式状态

**服务接口**:
- `~/set_waypoint_route`: 设置基于路点的路线
- `~/set_lanelet_route`: 设置基于车道的路线
- `~/clear_route`: 清除当前路线

**输出**:
- `~/route` (LaneletRoute): 规划的路线
- `~/state` (RouteState): 路线状态
- `~/debug/route_marker`: 调试可视化标记

#### 核心算法

**1. 路线规划流程**:
```
1. 验证目标点有效性
2. 计算起点和目标点最近的车道
3. 使用 Lanelet2 路由图搜索最短路径
4. 初始化 route_lanelets (可变道车道)
5. 提取优选车道 (preferred_lanelets)
6. 创建路线段 (route sections)
```

**2. 重新规划安全检查**:
```cpp
// check_reroute_safety() 函数逻辑:
1. 找到原路线和新路线的公共部分
2. 计算从当前位置到路线分叉点的距离
3. 计算安全距离 = max(当前速度 × 时间阈值, 最小距离)
4. 如果公共路线长度 > 安全距离,则允许重新规划
```

#### 重要参数
```yaml
map_frame: "map"                          # 地图坐标系
arrival_check_distance: 2.0               # 到达检查距离 [m]
arrival_check_angle_deg: 45.0             # 到达检查角度 [度]
arrival_check_duration: 1.0               # 到达检查停止时长 [s]
goal_angle_threshold: 45.0                # 目标点角度阈值 [度]
reroute_time_threshold: 10.0              # 重新规划时间阈值 [s]
minimum_reroute_length: 30.0              # 最小重新规划距离 [m]
allow_reroute_in_autonomous_mode: true    # 自动驾驶模式下允许重新规划
```

---

### 2. Route Handler (路由处理器)

#### 功能描述
Route Handler 是一个库,提供在 Lanelet2 地图上处理路线的各种工具函数。它被其他规划模块广泛使用。

#### 主要功能
- 查询车道的前后邻接关系
- 查询可变道的左右邻居车道
- 判断车道是否在路线内
- 计算路线上的参考路径

#### 关键类

**RouteHandler 类** (`route_handler.hpp`)
```cpp
// 主要成员:
- lanelet_map_ptr_: Lanelet2 地图指针
- routing_graph_ptr_: 路由图指针
- route_lanelets_: 路线上的所有车道
- preferred_lanelets_: 优选车道
- goal_lanelet_: 目标车道

// 关键方法分类:

// 1. 路线查询
- isInGoalRouteSection(): 判断车道是否在目标路段
- getGoalLaneId(): 获取目标车道ID
- getGoalPose(): 获取目标位姿

// 2. 车道邻接关系
- getNextLaneletsWithinRoute(): 获取路线内的下一个车道
- getPreviousLaneletsWithinRoute(): 获取路线内的前一个车道
- getRightLanelet(): 获取右侧车道
- getLeftLanelet(): 获取左侧车道

// 3. 变道查询
- getLaneChangeableNeighbors(): 获取可变道的邻居车道
- isDeadEndLanelet(): 判断是否为死胡同车道

// 4. 路径生成
- getCenterLinePath(): 获取中心线路径
- getReferencePathFromLaneletPrimitive(): 从车道基元获取参考路径
```

---

### 3. Path Generator (路径生成器)

#### 功能描述
Path Generator 接收 Mission Planner 的路线,将其转换为带有车道ID的路径。这是 behavior_path_planner 的简化版本。

#### 主要流程

```cpp
// 路径生成流程 (generate_path):
1. 搜索距离车辆最近的车道
2. 获取前方 path_length.forward 和后方 path_length.backward 范围内的车道
3. 拼接这些车道的中心线生成路径
4. 如果路线中存在路点,则替换重叠部分
5. 路径裁剪 (处理自相交和边界相交)
6. 生成转向信号
```

#### 路径裁剪算法

**问题**: 当路径存在自相交或折返时,需要裁剪路径避免碰撞

**解决方案**:
```cpp
// 检查三种相交类型:
1. 路径边界自相交 (self-intersection)
2. 左右边界互相交 (mutual intersection)
3. 起始边与路径边界相交 (start edge intersection)

// 在第一个相交点前的指定距离处裁剪路径
```

#### 转向信号生成

**规则**:
- 在指定距离前开启转向灯
- 当行驶方向改变到指定角度后关闭
- 连续转向时,优先显示必需或最后的转向信号

#### 输入输出

**输入**:
- `~/input/odometry` (Odometry): 车辆位姿
- `~/input/vector_map` (LaneletMapBin): 矢量地图
- `~/input/route` (LaneletRoute): 路线

**输出**:
- `~/output/path` (PathWithLaneId): 生成的路径
- `~/output/turn_indicators_cmd` (TurnIndicatorsCommand): 转向信号
- `~/output/hazard_lights_cmd` (HazardLightsCommand): 危险警示灯

#### 关键参数
```yaml
path_length:
  forward: 200.0          # 前方路径长度 [m]
  backward: 5.0           # 后方路径长度 [m]
turn_signal:
  search_distance: 30.0   # 转向信号搜索距离 [m]
  shift_length: 0.5       # 转向信号偏移长度 [m]
```

---

### 4. Behavior Velocity Planner (行为速度规划器)

#### 功能描述
Behavior Velocity Planner 根据交通规则调整路径速度。它采用插件架构,每个交通场景(如停止线、红绿灯、人行横道等)都作为独立模块实现。

#### 插件架构

```cpp
// 模块管理器 (PlannerManager):
- 加载各个场景模块插件
- 管理模块的生命周期
- 调用各模块修改路径速度

// 场景模块接口 (SceneModuleInterface):
class SceneModuleInterface {
  virtual bool modifyPathVelocity(PathWithLaneId * path) = 0;
  virtual visualization_msgs::msg::MarkerArray createDebugMarkerArray() = 0;
};
```

#### 内置模块

| 模块名 | 功能描述 |
|--------|---------|
| **Stop Line** | 在停止线前停车 |
| **Traffic Light** | 根据红绿灯状态决定通过或停止 |
| **Crosswalk** | 在人行横道前礼让行人 |
| **Intersection** | 路口安全通过逻辑 |
| **Blind Spot** | 盲区检测和减速 |
| **Detection Area** | 检测区域减速或停止 |
| **Virtual Traffic Light** | 虚拟红绿灯 |
| **Occlusion Spot** | 遮挡区域减速 |
| **No Stopping Area** | 禁停区域处理 |
| **Speed Bump** | 减速带减速 |

#### Stop Line Module 详解

**功能**: 在地图中标记的停止线前停车

**关键逻辑**:
```cpp
// 1. 检测停止线
- 从地图中查询 "stop_line" regulatory element
- 计算停止线与路径的交点

// 2. 设置停止速度
- 考虑车辆的 base_link 到前轴距离
- 在停止点插入速度为 0 的点

// 3. 状态机
APPROACH → STOPPED → START → PASS
```

**参数**:
```yaml
stop_line:
  stop_margin: 0.0                # 停止线前的停止距离 [m]
  stop_duration_sec: 1.0          # 停止持续时间 [s]
```

#### 速度设置原理

所有模块基于 `base_link` (后轴中心) 位置规划速度:

```
Vehicle:
  ├─── base_link (后轴中心,速度规划参考点)
  │
  └─── front (车辆前端)
       └── 距离 = wheelbase + front_overhang

若要在停止线前让车头停止:
stop_point_for_base_link = stop_line_position - (wheelbase + front_overhang)
```

#### 输入输出

**输入**:
- `~/input/path_with_lane_id` (PathWithLaneId): 带车道ID的路径
- `~/input/vector_map` (LaneletMapBin): 矢量地图
- `~/input/vehicle_odometry` (Odometry): 车辆速度
- `~/input/dynamic_objects` (PredictedObjects): 动态对象
- `~/input/traffic_signals` (TrafficLightGroupArray): 交通信号

**输出**:
- `~/output/path` (Path): 调整后的路径
- `~/output/stop_reasons` (StopReasonArray): 停止原因

#### 关键参数
```yaml
forward_path_length: 1000.0       # 前方路径长度 [m]
backward_path_length: 5.0         # 后方路径长度 [m]
max_accel: 2.0                    # 最大加速度 [m/s²]
system_delay: 0.3                 # 系统延迟 [s]
delay_response_time: 1.3          # 延迟响应时间 [s]
```

---

### 5. Motion Velocity Planner (运动速度规划器)

#### 功能描述
Motion Velocity Planner 基于周围障碍物调整轨迹速度。与 Behavior Velocity Planner 不同,它主要处理动态障碍物避障。

#### 主要模块

**Obstacle Stop Module**:
- 检测轨迹前方的障碍物
- 在安全距离前插入停止点或减速区间

#### 工作流程

```cpp
// 1. 障碍物检测
- 使用点云或预测对象数据
- 创建碰撞检查器 (CollisionChecker)

// 2. 碰撞检查
- 对轨迹上的每个点进行碰撞检查
- 考虑车辆尺寸和安全裕度

// 3. 插入速度限制
if (检测到障碍物) {
  if (需要停止) {
    插入停止点(stop_point)
  } else if (需要减速) {
    插入减速区间(slowdown_interval)
  }
}
```

#### 输入输出

**输入**:
- `~/input/trajectory` (Trajectory): 输入轨迹
- `~/input/vector_map` (LaneletMapBin): 矢量地图
- `~/input/vehicle_odometry` (Odometry): 车辆位姿和速度
- `~/input/dynamic_objects` (PredictedObjects): 动态对象
- `~/input/no_ground_pointcloud` (PointCloud2): 非地面点云

**输出**:
- `~/output/trajectory` (Trajectory): 调整后的轨迹
- `~/output/planning_factors/<MODULE_NAME>` (PlanningFactorsArray): 规划因子

#### 关键参数
```yaml
obstacle_stop:
  stop_margin: 5.0                # 停止裕度 [m]
  slow_down_margin: 10.0          # 减速裕度 [m]
  slow_down_velocity: 3.0         # 减速速度 [m/s]
```

---

### 6. Velocity Smoother (速度平滑器)

#### 功能描述
Velocity Smoother 对轨迹的速度曲线进行平滑处理,确保满足车辆的加速度、减速度和加加速度(jerk)约束,同时最大化速度以提高行驶效率。

#### 工作流程

```
输入轨迹
  ↓
1. 提取轨迹段 (extract_ahead_dist 前方, extract_behind_dist 后方)
  ↓
2. 应用外部速度限制
  ↓
3. 应用停止接近速度
  ↓
4. 应用横向加速度限制 (弯道减速)
  ↓
5. 应用转向角速率限制
  ↓
6. 轨迹重采样 (时间间隔采样)
  ↓
7. 计算初始状态 (速度、加速度)
  ↓
8. 速度平滑优化
   - JerkFiltered: 最小化加加速度平方和
   - L2: 最小化伪加加速度L2范数
   - Linf: 最小化伪加加速度L∞范数
   - Analytical: 解析解算法
  ↓
9. 后处理
   - 停止点后设置零速度
   - 限制最大速度
   - 后重采样
  ↓
输出平滑轨迹
```

#### 优化算法

**1. JerkFiltered 算法**:
```cpp
// 目标函数:
minimize: -Σ(v²) + w_jerk·Σ(jerk²) + w_over_v·Σ(over_v²) 
          + w_over_a·Σ(over_a²) + w_over_j·Σ(over_j²)

// 约束:
- velocity_min ≤ v ≤ velocity_max
- accel_min ≤ a ≤ accel_max
- jerk_min ≤ j ≤ jerk_max
```

**2. L2 Pseudo Jerk 算法**:
```cpp
// 伪加加速度定义:
pseudo_jerk = (a[i] - a[i-1]) / dt

// 目标函数:
minimize: -Σ(v²) + w_jerk·Σ(pseudo_jerk²) + w_over_v·Σ(over_v²) + w_over_a·Σ(over_a²)
```

**3. Analytical Jerk Constrained 算法**:
- 基于解析解的方法
- 不依赖于优化求解器
- 速度更快但可能不如优化方法精确

#### 横向加速度限制

```cpp
// 根据曲率计算速度限制:
v_max_curve = sqrt(max_lateral_accel / curvature)
v_max_curve = max(v_max_curve, min_curve_velocity)

// 在曲线前后设置减速区:
- decel_distance_before_curve: 曲线前减速距离
- decel_distance_after_curve: 曲线后减速距离
```

#### 初始状态计算

| 情况 | 初始速度 | 初始加速度 |
|------|---------|-----------|
| 首次计算 | 当前速度 | 0.0 |
| 启动中 (Engaging) | engage_velocity | engage_acceleration |
| 速度偏差大 | 当前速度 | 上一次规划值 |
| 正常 | 上一次规划值 | 上一次规划值 |

#### 输入输出

**输入**:
- `~/input/trajectory` (Trajectory): 输入轨迹
- `/planning/scenario_planning/max_velocity` (Float32): 外部速度限制
- `/localization/kinematic_state` (Odometry): 当前里程计

**输出**:
- `~/output/trajectory` (Trajectory): 平滑后的轨迹
- `/planning/scenario_planning/current_max_velocity` (Float32): 当前速度限制

#### 关键参数

**约束参数**:
```yaml
max_velocity: 20.0              # 最大速度 [m/s]
max_accel: 1.0                  # 最大加速度 [m/s²]
min_decel: -0.5                 # 最小减速度 [m/s²]
max_jerk: 1.0                   # 最大加加速度 [m/s³]
min_jerk: -0.5                  # 最小加加速度 [m/s³]
```

**弯道参数**:
```yaml
enable_lateral_acc_limit: true          # 启用横向加速度限制
max_lateral_accel: 0.5                  # 最大横向加速度 [m/s²]
min_curve_velocity: 2.74                # 弯道最小速度 [m/s]
decel_distance_before_curve: 3.5        # 弯道前减速距离 [m]
decel_distance_after_curve: 2.0         # 弯道后减速距离 [m]
```

**启动参数**:
```yaml
engage_velocity: 0.25                   # 启动速度阈值 [m/s]
engage_acceleration: 0.1                # 启动加速度 [m/s²]
engage_exit_ratio: 0.5                  # 退出启动序列的速度比例
```

**重采样参数**:
```yaml
resample_time: 10.0                     # 重采样总时间 [s]
dense_dt: 0.1                           # 密集采样时间间隔 [s]
dense_min_interval_distance: 0.1        # 密集采样最小间隔 [m]
sparse_dt: 0.5                          # 稀疏采样时间间隔 [s]
sparse_min_interval_distance: 4.0       # 稀疏采样最小间隔 [m]
```

**优化权重 (JerkFiltered)**:
```yaml
jerk_weight: 10.0                       # 平滑性权重
over_v_weight: 100000.0                 # 超速惩罚权重
over_a_weight: 5000.0                   # 超加速度惩罚权重
over_j_weight: 1000.0                   # 超加加速度惩罚权重
```

---

## 数据流分析

### 主要消息类型

#### 1. LaneletRoute
```cpp
// 路线消息,包含车道段序列
autoware_planning_msgs::msg::LaneletRoute
├── header: Header
├── start_pose: Pose                    // 起始位姿
├── goal_pose: Pose                     // 目标位姿
├── segments: LaneletSegment[]          // 路线段数组
│   ├── preferred_primitive: LaneletPrimitive  // 优选车道
│   └── primitives: LaneletPrimitive[]         // 所有可用车道
├── uuid: UUID                          // 唯一标识符
└── allow_modification: bool            // 是否允许修改
```

#### 2. PathWithLaneId
```cpp
// 带车道ID的路径
autoware_internal_planning_msgs::msg::PathWithLaneId
├── header: Header
├── points: PathPointWithLaneId[]       // 路径点数组
│   ├── point: Point                    // 位置
│   ├── longitudinal_velocity_mps: float // 纵向速度 [m/s]
│   ├── lateral_velocity_mps: float     // 横向速度 [m/s]
│   ├── heading_rate_rps: float         // 航向角速率 [rad/s]
│   ├── is_final: bool                  // 是否为最终点
│   └── lane_ids: int64[]               // 车道ID列表
├── left_bound: Point[]                 // 左边界
└── right_bound: Point[]                // 右边界
```

#### 3. Trajectory
```cpp
// 轨迹消息
autoware_planning_msgs::msg::Trajectory
├── header: Header
└── points: TrajectoryPoint[]           // 轨迹点数组
    ├── pose: Pose                      // 位姿
    ├── longitudinal_velocity_mps: float // 纵向速度 [m/s]
    ├── lateral_velocity_mps: float     // 横向速度 [m/s]
    ├── acceleration_mps2: float        // 加速度 [m/s²]
    ├── heading_rate_rps: float         // 航向角速率 [rad/s]
    ├── front_wheel_angle_rad: float    // 前轮转角 [rad]
    └── rear_wheel_angle_rad: float     // 后轮转角 [rad]
```

### 消息转换关系

```
LaneletRoute (Mission Planner)
  ↓ [Path Generator]
PathWithLaneId (带车道信息的路径)
  ↓ [Behavior Velocity Planner]
Path (基本路径)
  ↓ [Path to Trajectory Converter]
Trajectory (轨迹)
  ↓ [Motion Velocity Planner]
Trajectory (调整后的轨迹)
  ↓ [Velocity Smoother]
Trajectory (平滑轨迹) → Control Module
```

---

## 关键算法

### 1. 最短路径搜索 (Lanelet2 Routing Graph)

```cpp
// Dijkstra 算法在 Lanelet2 路由图上搜索
lanelet::routing::Route route = routing_graph->getRoute(
  start_lanelet,      // 起始车道
  goal_lanelet,       // 目标车道
  routing_cost_id     // 路由成本ID
);
```

### 2. 速度优化 (OSQP Solver)

```cpp
// 二次规划问题:
// minimize: (1/2) x^T P x + q^T x
// subject to: l ≤ Ax ≤ u

// 状态变量:
// x = [v0, v1, ..., vn, a0, a1, ..., an, j0, j1, ..., jn]

// 约束:
// - 速度约束: v_min ≤ v ≤ v_max
// - 加速度约束: a_min ≤ a ≤ a_max
// - 加加速度约束: j_min ≤ j ≤ j_max
// - 运动学约束: v[i+1] = v[i] + a[i] * dt
//              a[i+1] = a[i] + j[i] * dt
```

### 3. 碰撞检查

```cpp
// 使用 Boost.Geometry 进行多边形相交检查
bool hasCollision(const Polygon & vehicle_polygon, 
                   const Polygon & object_polygon) {
  return boost::geometry::intersects(vehicle_polygon, object_polygon);
}
```

### 4. 轨迹插值和重采样

```cpp
// 线性插值
Point interpolate(const Point & p1, const Point & p2, double ratio) {
  return p1 + (p2 - p1) * ratio;
}

// Arc-length 参数化重采样
std::vector<Point> resampleByArcLength(
  const std::vector<Point> & points, double interval);
```

---

## 配置参数说明

### 参数文件位置

```
autoware_launch/
└── config/
    └── planning/
        ├── mission_planning/
        │   └── mission_planner/
        │       └── mission_planner.param.yaml
        ├── scenario_planning/
        │   ├── lane_driving/
        │   │   ├── behavior_planning/
        │   │   │   └── behavior_velocity_planner/
        │   │   │       └── *.param.yaml
        │   │   └── motion_planning/
        │   │       └── motion_velocity_planner/
        │   │           └── *.param.yaml
        │   └── common/
        │       ├── common.param.yaml
        │       ├── nearest_search.param.yaml
        │       └── vehicle_info.param.yaml
        └── preset/
            └── default_preset.yaml
```

### 通用参数

```yaml
# common.param.yaml
common:
  max_vel: 20.0                    # 最大速度 [m/s]
  max_acc: 2.0                     # 最大加速度 [m/s²]
  min_acc: -3.0                    # 最小加速度 [m/s²]
  max_jerk: 2.0                    # 最大加加速度 [m/s³]
  min_jerk: -3.0                   # 最小加加速度 [m/s³]
  
# vehicle_info.param.yaml
vehicle_info:
  vehicle_length: 4.5              # 车辆长度 [m]
  vehicle_width: 1.8               # 车辆宽度 [m]
  wheel_base: 2.7                  # 轴距 [m]
  front_overhang: 0.9              # 前悬 [m]
  rear_overhang: 0.9               # 后悬 [m]
  left_overhang: 0.0               # 左悬 [m]
  right_overhang: 0.0              # 右悬 [m]
  vehicle_height: 1.5              # 车辆高度 [m]
```

### 调试参数

```yaml
# 启用调试输出
is_publish_debug_path: true        # 发布调试路径
output_debug_markers: true         # 输出调试标记
```

---

## 调试指南

### 1. 可视化工具

使用 RViz 可视化以下话题:

```bash
# 路线可视化
/planning/mission_planning/mission_planner/debug/route_marker

# 路径可视化
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id

# 轨迹可视化
/planning/scenario_planning/trajectory

# 停止原因可视化
/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/virtual_wall/*

# 调试标记
/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/*
```

### 2. 日志分析

```bash
# 查看规划模块日志
ros2 topic echo /planning/mission_planning/state

# 查看速度调试信息
ros2 topic echo /planning/scenario_planning/velocity_smoother/debug/closest_velocity
ros2 topic echo /planning/scenario_planning/velocity_smoother/debug/closest_acceleration
ros2 topic echo /planning/scenario_planning/velocity_smoother/debug/closest_jerk
```

### 3. 常见问题排查

#### 问题 1: 车辆速度为零

**可能原因**:
1. Behavior Velocity Planner 某个模块插入了停止点
2. Motion Velocity Planner 检测到障碍物
3. Velocity Smoother 的约束过于严格

**排查步骤**:
```bash
# 1. 检查停止原因
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/output/stop_reasons

# 2. 检查各模块输出速度
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/output/path
ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/output/trajectory

# 3. 检查速度平滑器输入输出
ros2 topic echo /planning/scenario_planning/velocity_smoother/input/trajectory
ros2 topic echo /planning/scenario_planning/velocity_smoother/output/trajectory
```

#### 问题 2: 路径规划失败

**可能原因**:
1. 起点或目标点不在地图车道上
2. 起点和目标点之间没有连通路径
3. 目标点角度偏差过大

**排查步骤**:
```bash
# 检查路线状态
ros2 topic echo /planning/mission_planning/state

# 检查日志
ros2 log info /planning/mission_planning/mission_planner
```

#### 问题 3: 轨迹不平滑

**可能原因**:
1. 速度平滑器权重参数不合理
2. 重采样参数设置不当
3. 约束条件过于宽松

**调整方法**:
```yaml
# 增加平滑性权重
jerk_weight: 100.0  # 增大此值

# 调整重采样参数
dense_dt: 0.05      # 减小以获得更密集的采样
```

### 4. 性能优化

```bash
# 查看处理时间
ros2 topic echo /planning/scenario_planning/velocity_smoother/debug/processing_time_ms
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/processing_time_detail_ms
```

**优化建议**:
1. 减少 `forward_path_length` 以降低计算量
2. 增大重采样间隔以减少优化变量数量
3. 选择计算效率高的平滑算法 (如 Analytical)

### 5. 参数调优流程

```
1. 确定场景 (城市/高速/停车场等)
   ↓
2. 调整速度和加速度约束
   ↓
3. 调整各模块的安全裕度参数
   ↓
4. 调整平滑器权重参数
   ↓
5. 实车测试验证
   ↓
6. 根据反馈迭代优化
```

---

## 总结

Planning 模块是 Autoware 的核心,通过层次化的架构实现了从全局路线规划到局部轨迹优化的完整流程:

1. **Mission Planner**: 全局路线规划
2. **Path Generator**: 生成可行驶路径
3. **Behavior Velocity Planner**: 遵守交通规则
4. **Motion Velocity Planner**: 避让动态障碍物
5. **Velocity Smoother**: 平滑和优化轨迹

每个模块都有清晰的接口和可配置的参数,支持灵活的定制和扩展。理解这些模块的工作原理和参数配置,是进行自动驾驶系统开发和调试的关键。

---

## 附录

### 相关文档
- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/)
- [Lanelet2 Documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [OSQP Solver](https://osqp.org/)

### 参考论文
1. Y. Zhang, et al., "Toward a More Complete, Flexible, and Safer Speed Planning for Autonomous Driving via Convex Optimization", Sensors, 2018
2. B. Stellato, et al., "OSQP: an operator splitting solver for quadratic programs", Mathematical Programming Computation, 2020

### 技术支持
- [Autoware GitHub Issues](https://github.com/autowarefoundation/autoware/issues)
- [Autoware Discourse Forum](https://discourse.ros.org/c/autoware)

