# Lane Following 参考路径生成详解

## 概述

当 `start_planner` 不激活时（例如车辆已在车道中间），Behavior Path Planner 会使用 **参考路径（Reference Path）** 进行规划，这就是所谓的 **Lane Following**（车道跟随）功能。

本文档详细说明 Lane Following 的工作机制、输入输出和代码实现。

---

## 1. Lane Following 的概念

### 1.1 什么是 Lane Following？

**Lane Following** 不是一个独立的模块，而是 Behavior Path Planner 的**基础功能**：

- 📍 **核心功能**: 从 lanelet 地图的中心线生成参考路径
- 🎯 **基准路径**: 作为所有其他场景模块的基准
- 🔄 **默认行为**: 当没有其他场景模块激活时使用

### 1.2 与 Start Planner 的关系

```
车辆状态判断
    ↓
┌─────────────────────────────────────┐
│ 车辆在路边/停车位？                  │
│ (远离车道中心线)                     │
└─────────────────────────────────────┘
    ├─ 是 → 激活 Start Planner
    │       └─ 生成起步路径（横向位移/弧线）
    │
    └─ 否 → 使用 Lane Following
            └─ 生成参考路径（沿车道中心线）
```

**关键判断条件**（`start_planner_module.cpp:370-380`）:
```cpp
bool StartPlannerModule::isCurrentPoseOnEgoCenterline() const
{
  // 计算车辆到车道中心线的横向距离
  const double lateral_distance_to_center_lane =
    lanelet::utils::getArcCoordinates(current_lanes, current_pose, lanelet_map_ptr)
      .distance;

  // 如果距离小于阈值（默认0.5m），认为在中心线上
  return std::abs(lateral_distance_to_center_lane) < 
         parameters_->th_distance_to_middle_of_the_road;  // 默认 0.5m
}
```

**判断逻辑**:
- ✅ 如果 `|横向偏移| < 0.5m` → 在车道中间 → 不激活 Start Planner → 使用 Lane Following
- ⚠️ 如果 `|横向偏移| >= 0.5m` → 偏离车道 → 可能激活 Start Planner

---

## 2. Lane Following 的工作机制

### 2.1 执行流程

```
behavior_path_planner_node
    ↓
planner_manager.run()
    ↓
判断是否有激活的场景模块（start/goal/avoidance等）
    ├─ 有激活模块 → 使用该模块的输出路径
    └─ 无激活模块 → 使用参考路径（Lane Following）
        ↓
    getReferencePath()  ← ⭐ 核心函数
        ↓
    从 lanelet 中心线生成路径
        ↓
    返回 BehaviorModuleOutput
```

### 2.2 核心函数调用链

```cpp
// 1. planner_manager.cpp - 主入口
BehaviorModuleOutput PlannerManager::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  // 调用工具函数生成参考路径
  const auto reference_path = utils::getReferencePath(
    current_route_lanelet_->value(), data);
  
  return reference_path;
}

// 2. path_utils.cpp - 核心实现
BehaviorModuleOutput utils::getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  // 生成中心线路径
  reference_path = getCenterLinePath(...);
  
  // 生成可行驶区域
  drivable_lanes = generateDrivableLanes(...);
  
  // 返回输出
  return output;
}

// 3. route_handler - 底层实现
PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanes,
  const double s_start, const double s_end)
{
  // 从 lanelet 的中心线点生成路径
  // 包含位置、朝向、速度限制等信息
}
```

---

## 3. 详细输入和输出

### 3.1 输入数据

#### A. 核心输入：PlannerData

**函数签名**:
```cpp
BehaviorModuleOutput getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data
)
```

**输入参数详解**:

##### 1) `current_lane` - 当前车道

**类型**: `lanelet::ConstLanelet`

**内容**:
- 当前车辆所在的车道（lanelet）
- 包含车道几何信息（中心线、边界）
- 包含车道属性（速度限制、车道类型等）

**获取方式**（`planner_manager.cpp:236-270`）:
```cpp
void PlannerManager::updateCurrentRouteLanelet(
  const std::shared_ptr<PlannerData> & data,
  const bool is_any_approved_module_running)
{
  const auto & route_handler = data->route_handler;
  const auto & pose = data->self_odometry->pose.pose;
  
  lanelet::ConstLanelet closest_lane{};
  
  // 方法1: 从上一次的车道更新
  if (route_handler->getClosestRouteLaneletFromLanelet(
        pose, current_route_lanelet_->value(), &closest_lane, ...)) {
    *current_route_lanelet_ = closest_lane;
    return;
  }
  
  // 方法2: 从车道序列中查找
  const auto lanelet_sequence = route_handler->getLaneletSequence(...);
  lanelet::utils::query::getClosestLanelet(lanelet_sequence, pose, &closest_lane);
  *current_route_lanelet_ = closest_lane;
}
```

##### 2) `planner_data` - 规划数据

**类型**: `std::shared_ptr<const PlannerData>`

**包含信息**:
```cpp
struct PlannerData {
  // 车辆状态
  Odometry::ConstSharedPtr self_odometry;  // 位姿、速度
  
  // 路由和地图
  std::shared_ptr<RouteHandler> route_handler;  // 路由处理器
    ├── LaneletMapPtr lanelet_map;              // Lanelet地图
    ├── LaneletRoute route;                     // 当前路由
    └── 各种地图查询方法
  
  // 规划参数
  BehaviorPathPlannerParameters parameters;
    ├── forward_path_length: 前方路径长度（默认300m）
    ├── backward_path_length: 后方路径长度（默认5m）
    ├── input_path_interval: 路径点间隔（默认1m）
    └── ego_nearest_dist_threshold: 最近距离阈值
  
  // 动态障碍物（Lane Following不直接使用，但存在）
  PredictedObjects::ConstSharedPtr dynamic_object;
  
  // 其他
  // ...
};
```

### 3.2 输出数据

#### A. 输出类型：BehaviorModuleOutput

**函数签名**:
```cpp
BehaviorModuleOutput getReferencePath(...)
```

**返回结构**:
```cpp
struct BehaviorModuleOutput {
  PathWithLaneId path;                // 主要输出路径
  PathWithLaneId reference_path;      // 参考路径（通常与path相同）
  TurnSignalInfo turn_signal_info;    // 转向信号（Lane Following为空）
  DrivableAreaInfo drivable_area_info; // 可行驶区域信息
  std::optional<PoseWithUuidStamped> modified_goal; // 修改后的目标（通常为空）
};
```

#### B. 核心输出：path (PathWithLaneId)

**结构详解**:
```cpp
struct PathWithLaneId {
  Header header;
  std::vector<PathPointWithLaneId> points;
  // 可行驶区域边界（由 generateDrivableArea() 填充）
  PathFootprint left_bound;
  PathFootprint right_bound;
};

struct PathPointWithLaneId {
  PathPoint point;              // 路径点信息
    ├── Pose pose;              // 位姿（位置+朝向）
    ├── float longitudinal_velocity_mps;  // ⭐ 纵向速度 [m/s]
    ├── float lateral_velocity_mps;       // 横向速度 [m/s]
    ├── float heading_rate_rps;           // 航向角速率 [rad/s]
    └── bool is_final;                    // 是否为最终点
  std::vector<int64_t> lane_ids;  // 该点所属的车道ID列表
};
```

**路径特点**:
- 📏 **长度**: `backward_length + forward_length`（通常 5m + 300m = 305m）
- 📍 **采样间隔**: `input_path_interval`（默认 1m）
- 🎯 **点数**: 约 305 个点
- 🚗 **位置**: 沿车道中心线
- ⚡ **速度**: 从 lanelet 地图的 `speed_limit` 属性获取

#### C. 速度信息来源

**关键点**: 速度从哪里来？

**代码流程**（`route_handler` 内部）:
```cpp
PathWithLaneId RouteHandler::getCenterLinePath(...) {
  // 1. 从 lanelet 获取中心线点
  for (const auto & lane : lanes) {
    const auto & centerline = lane.centerline();
    for (const auto & point : centerline) {
      path_point.point.pose.position = point;
      
      // 2. ⭐ 获取速度限制
      const auto speed_limit_opt = 
        lane.attributeOr("speed_limit", std::nullopt);
      
      if (speed_limit_opt.has_value()) {
        // 从地图属性获取速度限制（单位：km/h）
        const double speed_kmph = std::stod(speed_limit_opt.value());
        
        // 转换为 m/s
        path_point.point.longitudinal_velocity_mps = 
          static_cast<float>(speed_kmph / 3.6);
      }
      
      // 3. 添加车道ID
      path_point.lane_ids.push_back(lane.id());
      
      path.points.push_back(path_point);
    }
  }
  
  // 4. 样条插值平滑
  path = resamplePathWithSpline(path, interval);
  
  return path;
}
```

**速度设置流程**:
```
Lanelet 地图
  ├── 车道属性: speed_limit = "50"  (km/h)
  └── 中心线点: centerline points

        ↓

RouteHandler::getCenterLinePath()
  ├── 读取 speed_limit 属性
  ├── 转换单位: 50 km/h → 13.89 m/s
  └── 设置到路径点

        ↓

参考路径 (Reference Path)
  └── 每个点的 longitudinal_velocity_mps = 13.89
```

#### D. 可行驶区域信息 (drivable_area_info)

```cpp
struct DrivableAreaInfo {
  std::vector<DrivableLanes> drivable_lanes;  // 可行驶车道列表
  bool enable_expanding_hatched_road_markings;  // 是否扩展斜线区域
  bool enable_expanding_intersection_areas;     // 是否扩展交叉口
  bool enable_expanding_freespace_areas;        // 是否扩展自由空间
  double drivable_margin;                       // 可行驶边界余量
  bool is_already_expanded;                     // 是否已经扩展
  std::vector<DrivableAreaInfo::Obstacle> obstacles; // 障碍物
};
```

**可行驶车道生成**（`path_utils.cpp:494-502`）:
```cpp
// 1. 从路径获取相关的 lanelets
const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);

// 2. 生成可行驶车道结构
const auto drivable_lanes = generateDrivableLanes(drivable_lanelets);

// 3. 裁剪重叠车道
const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);

// 4. 扩展车道边界
const auto expanded_lanes = expandLanelets(
  shorten_lanes,
  dp.drivable_area_left_bound_offset,   // 左边界偏移
  dp.drivable_area_right_bound_offset,  // 右边界偏移
  dp.drivable_area_types_to_skip        // 跳过的类型
);

// 5. 设置到输出
output.drivable_area_info.drivable_lanes = drivable_lanes;
```

---

## 4. 代码位置和实现

### 4.1 核心文件位置

#### A. 主要实现文件

| 文件 | 路径 | 功能 |
|------|------|------|
| **planner_manager.cpp** | `autoware_behavior_path_planner/src/planner_manager.cpp` | 管理器主逻辑 |
| **path_utils.cpp** | `autoware_behavior_path_planner_common/src/utils/path_utils.cpp` | 路径生成工具 |
| **route_handler.cpp** | `autoware_core/common/autoware_route_handler/...` | 路由和地图处理 |

#### B. 头文件

| 文件 | 功能 |
|------|------|
| `planner_manager.hpp` | 管理器接口定义 |
| `path_utils.hpp` | 路径工具函数声明 |
| `data_manager.hpp` | 数据结构定义 (PlannerData, BehaviorModuleOutput) |

### 4.2 关键函数实现

#### 函数1: PlannerManager::getReferencePath()

**位置**: `planner_manager.cpp:272-295`

```cpp
BehaviorModuleOutput PlannerManager::getReferencePath(
  const std::shared_ptr<PlannerData> & data) const
{
  // 调用工具函数生成参考路径
  const auto reference_path = utils::getReferencePath(
    current_route_lanelet_->value(), data);

  // 检查路径是否为空
  if (reference_path.path.points.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 5000,
      "Empty reference path detected. Using last valid reference path if available.");

    // 尝试使用上一次有效的参考路径
    if (last_valid_reference_path_.has_value()) {
      return last_valid_reference_path_.value();
    }

    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 5000, 
      "No valid previous reference path available. Creating empty path.");
    return BehaviorModuleOutput{};
  }

  // 缓存有效路径
  last_valid_reference_path_ = reference_path;

  // 发布调试信息
  publishDebugRootReferencePath(reference_path);
  
  return reference_path;
}
```

**作用**:
- 作为 planner_manager 的接口函数
- 处理空路径情况（使用缓存）
- 发布调试信息

#### 函数2: utils::getReferencePath()

**位置**: `path_utils.cpp:451-510`

```cpp
BehaviorModuleOutput getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  PathWithLaneId reference_path{};

  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_odometry->pose.pose;
  const auto p = planner_data->parameters;

  // ⭐ 步骤1: 设置路径头信息
  reference_path.header = route_handler->getRouteHeader();

  // ⭐ 步骤2: 计算前后范围（带额外余量避免插值不稳定）
  constexpr double extra_margin = 10.0;
  const double backward_length = p.backward_path_length + extra_margin;
  
  // 获取车道序列（当前车道前后的车道）
  const auto current_lanes_with_backward_margin =
    route_handler->getLaneletSequence(
      current_lane, backward_length, p.forward_path_length);

  // ⭐ 步骤3: 计算最近的中心线位姿（无偏移）
  const auto no_shift_pose =
    lanelet::utils::getClosestCenterPose(current_lane, current_pose.position);
  
  // ⭐ 步骤4: 生成中心线路径
  reference_path = getCenterLinePath(
    *route_handler, current_lanes_with_backward_margin, no_shift_pose,
    backward_length, p.forward_path_length, p);

  // ⭐ 步骤5: 检查路径是否为空
  if (reference_path.points.empty()) {
    RCLCPP_WARN_THROTTLE(..., "Empty reference path detected.");
    return BehaviorModuleOutput{};
  }

  // ⭐ 步骤6: 裁剪路径到指定长度
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      reference_path.points, no_shift_pose, ...);
  
  reference_path.points = autoware::motion_utils::cropPoints(
    reference_path.points, no_shift_pose.position, current_seg_idx,
    p.forward_path_length,
    p.backward_path_length + p.input_path_interval);

  // ⭐ 步骤7: 生成可行驶区域
  const auto drivable_lanelets = getLaneletsFromPath(reference_path, route_handler);
  const auto drivable_lanes = generateDrivableLanes(drivable_lanelets);

  const auto & dp = planner_data->drivable_area_expansion_parameters;
  const auto shorten_lanes = cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset,
    dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  // ⭐ 步骤8: 构建输出
  BehaviorModuleOutput output;
  output.path = reference_path;
  output.reference_path = reference_path;
  output.drivable_area_info.drivable_lanes = drivable_lanes;

  return output;
}
```

**关键步骤**:
1. 设置路径头信息（时间戳、坐标系）
2. 获取前后车道序列（考虑车道变化）
3. 计算无偏移的中心线位姿
4. **生成中心线路径**（包含速度）
5. 检查路径有效性
6. 裁剪路径到合适长度
7. 生成可行驶区域
8. 构建并返回输出

#### 函数3: getCenterLinePath()

**位置**: `path_utils.cpp` 或 `route_handler`

```cpp
PathWithLaneId getCenterLinePath(
  const RouteHandler & route_handler,
  const lanelet::ConstLanelets & lanes,
  const Pose & pose,
  const double backward_length,
  const double forward_length,
  const BehaviorPathPlannerParameters & params)
{
  // 实际调用 route_handler 的方法
  auto path = route_handler.getCenterLinePath(
    lanes, 
    arc_start, 
    arc_end,
    use_exact_arc);
  
  // 重采样路径（样条插值）
  path = resamplePathWithSpline(path, params.input_path_interval);
  
  return path;
}
```

---

## 5. 使用场景和示例

### 5.1 场景1: 正常车道行驶

**初始状态**:
- 车辆在车道中心线附近（偏移 < 0.5m）
- 车辆正在行驶（速度 > 0.01 m/s）
- 无其他场景模块激活

**处理流程**:
```
behavior_path_planner_node::run()
    ↓
planner_manager::run(data)
    ↓
isExecutionRequested() 各模块
  ├─ start_planner: false  (车辆在中心线上)
  ├─ goal_planner: false   (未接近目标)
  ├─ avoidance: false      (无需避让)
  └─ lane_change: false    (无需变道)
    ↓
无激活模块 → 使用参考路径
    ↓
getReferencePath(current_lane, planner_data)
    ↓
生成输出:
  └─ path: 沿车道中心线的路径
      ├─ 长度: 305m (5m backward + 300m forward)
      ├─ 点数: ~305个点 (1m间隔)
      ├─ 速度: 13.89 m/s (50 km/h, 来自地图)
      └─ 车道ID: [当前车道ID, 前方车道ID]
```

**输出示例**:
```yaml
path:
  header:
    frame_id: "map"
    stamp: {sec: 1234, nanosec: 567890000}
  points:
    - point:
        pose:
          position: {x: 100.0, y: 50.0, z: 0.0}
          orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
        longitudinal_velocity_mps: 13.89  # 50 km/h
        lateral_velocity_mps: 0.0
        heading_rate_rps: 0.0
      lane_ids: [12345]
    - point:
        pose:
          position: {x: 101.0, y: 50.0, z: 0.0}  # 1m后
          orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
        longitudinal_velocity_mps: 13.89
        lateral_velocity_mps: 0.0
      lane_ids: [12345]
    # ... 约303个点
```

### 5.2 场景2: 从 Start Planner 过渡到 Lane Following

**阶段1: Start Planner 激活**
```
初始: 车辆在路边 (偏移 = 2m)
    ↓
start_planner 激活
    ├─ 生成起步路径（横向位移）
    └─ 逐渐靠近车道中心
```

**阶段2: 过渡时刻**
```
车辆位置: 偏移 = 0.6m → 0.4m
    ↓
isCurrentPoseOnEgoCenterline() = false → true
    ↓
start_planner 不再请求执行
    ↓
start_planner 完成任务
```

**阶段3: Lane Following 接管**
```
无激活模块
    ↓
使用 getReferencePath()
    ↓
输出车道跟随路径
```

**完整流程**:
```
[Start Planner 激活]
  └─ 输出: 起步路径
      ├─ 起点: (x=100, y=52, 偏移=2m)
      ├─ 终点: (x=120, y=50, 偏移=0m)
      └─ 速度: 0 → 5 m/s (加速)

[过渡] 车辆到达 (x=120, y=50.4, 偏移=0.4m)

[Lane Following 接管]
  └─ 输出: 参考路径
      ├─ 起点: (x=115, y=50, backward 5m)
      ├─ 终点: (x=420, y=50, forward 300m)
      └─ 速度: 13.89 m/s (来自地图)
```

---

## 6. 与其他模块的协作

### 6.1 作为其他模块的基准

**参考路径的作用**:

```cpp
// 在各个场景模块中使用
BehaviorModuleOutput MyModule::plan() {
  // 获取参考路径作为基准
  const auto reference_path = getPreviousModuleOutput().reference_path;
  
  // 基于参考路径进行修改
  auto modified_path = modifyPath(reference_path);
  
  BehaviorModuleOutput output;
  output.path = modified_path;
  output.reference_path = reference_path;  // 保留原始参考
  
  return output;
}
```

**模块链示例**:
```
getReferencePath()
  └─ 生成基准路径
      ↓
Avoidance Module
  └─ 基于参考路径生成避让路径
      ↓
Lane Change Module
  └─ 基于避让路径生成变道路径
      ↓
最终输出
```

### 6.2 速度传递链

```
Lanelet 地图
  └─ speed_limit: 50 km/h
      ↓
getReferencePath()
  └─ longitudinal_velocity_mps: 13.89 m/s
      ↓
其他模块处理
  ├─ Avoidance: 可能降低速度（避让时）
  ├─ Lane Change: 保持或降低速度
  └─ Goal Planner: 逐渐减速到 0
      ↓
最终输出路径
  └─ 每个点都有速度信息
```

---

## 7. 调试和可视化

### 7.1 查看参考路径

#### A. RViz 可视化

在 RViz 中添加以下话题：
```
/planning/scenario_planning/lane_driving/behavior_planning/path
/planning/scenario_planning/lane_driving/behavior_planning/path_reference
```

**查看内容**:
- 路径点位置（绿色线条）
- 可行驶区域（蓝色多边形）
- 路径方向（箭头）

#### B. 命令行查看

```bash
# 查看参考路径话题
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_reference

# 查看路径点数量
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_reference --once | grep -c "pose:"

# 查看速度信息
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_reference | grep longitudinal_velocity_mps
```

### 7.2 调试日志

#### A. 启用调试信息

修改配置文件：
```yaml
# behavior_path_planner.param.yaml
/**:
  ros__parameters:
    behavior_path_planner:
      verbose: true  # 启用详细日志
```

#### B. 查看日志输出

```bash
# 查看规划管理器日志
ros2 run rqt_console rqt_console

# 过滤关键字
- "reference_path"
- "getReferencePath"
- "Empty reference path"
- "Lane Following"
```

**关键日志示例**:
```
[INFO] [planner_manager]: Using reference path (no active modules)
[DEBUG] [path_utils]: Generated reference path with 305 points
[DEBUG] [path_utils]: Reference path length: 305.0m
[WARN] [path_utils]: Empty reference path detected. Using cached path.
```

### 7.3 性能监控

```bash
# 查看参考路径生成时间
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/processing_time

# 典型值:
# - getReferencePath: 2-5 ms
# - generateDrivableArea: 1-3 ms
# - total_time: 5-10 ms
```

---

## 8. 常见问题

### Q1: 为什么参考路径的速度全是 0？

**可能原因**:
1. ❌ **Lanelet 地图没有 speed_limit 属性**
   - 检查地图文件: `<tag k="speed_limit" v="50"/>`
   - 解决: 在地图中添加速度限制属性

2. ❌ **某个模块将速度设为 0**
   - 检查是否有其他模块在后面处理路径
   - 查看 `createGoalAroundPath()` 是否被调用（会将速度设为0）

3. ❌ **系统未就绪**
   - 检查 `is_ready` 标志
   - 确认所有必要的输入数据都已接收

**调试方法**:
```bash
# 1. 检查地图速度限制
ros2 topic echo /map/vector_map | grep speed_limit

# 2. 检查参考路径速度
ros2 topic echo /planning/.../path_reference | grep longitudinal_velocity_mps | head -5

# 3. 查看日志
ros2 run rqt_console rqt_console
# 搜索: "Empty reference path" 或 "speed_limit"
```

### Q2: 参考路径为什么有时候是空的？

**可能原因**:
1. ❌ **无法找到当前车道**
   - 车辆不在路由范围内
   - 地图加载失败

2. ❌ **车道序列获取失败**
   - 路由中断
   - 前方无可用车道

**解决方法**:
- 检查路由是否有效: `/planning/mission_planning/route`
- 检查车辆位置是否在地图内
- 使用缓存的上一次有效路径（自动处理）

### Q3: Lane Following 和 Start Planner 冲突？

**不会冲突！**

它们是互斥的：
- Start Planner 激活 → Lane Following 不使用
- Start Planner 未激活 → 使用 Lane Following

**判断依据**:
```cpp
// start_planner 的激活条件（之一）
if (isCurrentPoseOnEgoCenterline()) {
  return false;  // 不激活 start_planner
  // 系统将使用 lane following
}
```

---

## 9. 参数配置

### 9.1 关键参数

**配置文件**: `behavior_path_planner.param.yaml`

```yaml
/**:
  ros__parameters:
    behavior_path_planner:
      # 路径长度
      forward_path_length: 300.0      # 前方路径长度 [m]
      backward_path_length: 5.0       # 后方路径长度 [m]
      
      # 路径采样
      input_path_interval: 1.0        # 路径点间隔 [m]
      
      # 阈值
      ego_nearest_dist_threshold: 3.0 # 最近距离阈值 [m]
      ego_nearest_yaw_threshold: 1.046  # 最近朝向阈值 [rad] (~60°)
      
      # 可行驶区域扩展
      drivable_area_left_bound_offset: 0.0   # 左边界偏移 [m]
      drivable_area_right_bound_offset: 0.0  # 右边界偏移 [m]
      
      # Start Planner 激活阈值
      start_planner:
        th_distance_to_middle_of_the_road: 0.5  # 到中心线距离阈值 [m]
```

### 9.2 参数调优建议

#### 增加路径长度

```yaml
forward_path_length: 500.0  # 从 300m 增加到 500m
```

**影响**:
- ✅ 更远的前瞻距离
- ⚠️ 计算时间增加
- ⚠️ 内存使用增加

#### 调整采样间隔

```yaml
input_path_interval: 0.5  # 从 1.0m 减小到 0.5m
```

**影响**:
- ✅ 路径更平滑
- ✅ 控制更精确
- ⚠️ 路径点数翻倍
- ⚠️ 计算和传输开销增加

#### 调整中心线阈值

```yaml
start_planner:
  th_distance_to_middle_of_the_road: 1.0  # 从 0.5m 增加到 1.0m
```

**影响**:
- ✅ 更容易使用 Lane Following
- ⚠️ Start Planner 激活机会减少
- ⚠️ 可能在偏离较大时仍使用 Lane Following

---

## 10. 总结

### 关键要点

1. **Lane Following 是基础功能**
   - 不是独立模块，而是参考路径生成
   - 当无其他模块激活时使用
   - 作为所有其他模块的基准

2. **自动切换机制**
   - 车辆偏离中心线 → Start Planner 激活
   - 车辆在中心线上 → 使用 Lane Following
   - 无缝过渡，无需人工干预

3. **速度信息来源**
   - 从 Lanelet 地图的 `speed_limit` 属性
   - 自动转换单位（km/h → m/s）
   - 保持在整个路径中

4. **核心函数**
   - `PlannerManager::getReferencePath()` - 管理器接口
   - `utils::getReferencePath()` - 核心实现
   - `RouteHandler::getCenterLinePath()` - 底层实现

5. **输入输出清晰**
   - 输入: 当前车道 + 规划数据
   - 输出: 带速度的路径 + 可行驶区域

6. **性能优秀**
   - 生成时间: 2-5 ms
   - 路径平滑: 样条插值
   - 缓存机制: 避免空路径问题

---

**相关文件**:
- 主实现: `planner_manager.cpp`, `path_utils.cpp`
- 头文件: `planner_manager.hpp`, `path_utils.hpp`, `data_manager.hpp`
- 配置: `behavior_path_planner.param.yaml`

**相关文档**:
- StartPlanner 激活条件说明
- StartPlanner 模块输入输出说明
- longitudinal_velocity_mps 代码总结

**文档生成时间**: 2025-10-24

