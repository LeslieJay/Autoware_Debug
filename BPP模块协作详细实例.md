# Behavior Path Planner 模块协作详细实例

## 🎬 场景设定

### 初始状态
```
道路情况:
┌─────────────────────────────────────────────────────────────┐
│  Lane 1 (左侧车道)                                           │
│  ════════════════════════════════════════════════════════   │
│                                                              │
├─────────────────────────────────────────────────────────────┤
│  Lane 2 (当前车道 - Ego在此)                                 │
│  ════════════════════════╗ 🚗停车 ╔═══════════════════════  │
│                Ego→     ╚════════╝                          │
├─────────────────────────────────────────────────────────────┤
│  Lane 3 (右侧车道)                                           │
│  ════════════════════════════════════════════════════════   │
└─────────────────────────────────────────────────────────────┘

距离关系:
- Ego当前位置: 0m
- 停车车辆位置: 50m (在当前车道)
- 车道宽度: 3.5m
- 车辆速度: 8.33 m/s (30 km/h)
```

---

## 📊 Slot 2 处理过程详解

### 初始输入（来自Slot 1）

**输入路径：**
```cpp
struct BehaviorModuleOutput {
  PathWithLaneId path;
  DrivableAreaInfo drivable_area_info;
  TurnSignalInfo turn_signal_info;
  // ... 其他字段
};

// 初始路径内容
path.points = [
  // 点0: 当前位置
  {
    lane_ids: [200],  // Lane 2
    point: {
      pose: { position: {x: 0, y: 0, z: 0}, orientation: ... },
      longitudinal_velocity_mps: 8.333,  // 30 km/h
      lateral_velocity_mps: 0.0
    }
  },
  // 点1: 10m
  {
    lane_ids: [200],
    point: {
      pose: { position: {x: 10, y: 0, z: 0}, ... },
      longitudinal_velocity_mps: 8.333
    }
  },
  // 点2: 20m
  {
    lane_ids: [200],
    point: {
      pose: { position: {x: 20, y: 0, z: 0}, ... },
      longitudinal_velocity_mps: 8.333
    }
  },
  // ... 点3-4: 30m-40m ...
  
  // 点5: 50m (障碍物位置)
  {
    lane_ids: [200],
    point: {
      pose: { position: {x: 50, y: 0, z: 0}, ... },
      longitudinal_velocity_mps: 8.333  // 还未感知到障碍物
    }
  },
  // ... 后续点 60m-100m ...
]

drivable_area_info = {
  drivable_lanes: [Lane 200],  // 当前可行驶车道
  obstacles: [],  // 暂无障碍物信息
}
```

---

## 阶段1️⃣：所有模块检查请求（Request Phase）

### Step 1.1: Static Obstacle Avoidance 检查

**代码位置：** `static_obstacle_avoidance_module.cpp::isExecutionRequested()`

```cpp
bool StaticObstacleAvoidanceModule::isExecutionRequested() {
  // 1. 检测路径上的静态障碍物
  const auto obstacles = detectStaticObstacles(input_path);
  
  // 检测结果：
  // obstacles = [
  //   {
  //     position: {x: 50, y: 0, z: 0},
  //     size: {length: 4.5, width: 1.8},
  //     type: "PARKED_VEHICLE"
  //   }
  // ]
  
  // 2. 判断是否需要避障
  if (!obstacles.empty()) {
    const auto & obstacle = obstacles.front();
    const double lateral_margin = 1.0;  // 需要1米横向间距
    
    if (obstacle.distance_to_path < lateral_margin) {
      RCLCPP_INFO(get_logger(), "Static obstacle detected at 50m, requesting avoidance");
      return true;  // ✅ 请求执行
    }
  }
  
  return false;
}
```

**结果：** ✅ **请求执行**（检测到障碍物需要避障）

---

### Step 1.2: Lane Change Left 检查

**代码位置：** `lane_change_module.cpp::isExecutionRequested()`

```cpp
bool LaneChangeLeftModule::isExecutionRequested() {
  // 1. 检查是否有左侧车道
  const auto left_lanes = route_handler->getLeftLanelet(current_lane);
  if (left_lanes.empty()) {
    return false;
  }
  
  // 2. 检查换道条件
  const auto reason = checkLaneChangeCondition();
  
  // 条件检查：
  // - 检测到前方有障碍物 → YES (静态避障模块已检测到)
  // - 左侧车道可用 → YES
  // - 左侧车道安全 → 需要检查
  
  // 3. 检查左侧车道安全性
  const auto left_lane_objects = getLeftLaneObjects();
  if (isSafeToChangeLane(left_lane_objects)) {
    RCLCPP_INFO(get_logger(), "Safe to change lane left, requesting lane change");
    return true;  // ✅ 请求执行
  }
  
  return false;
}
```

**结果：** ✅ **请求执行**（可以换道避开障碍物）

---

### Step 1.3: Lane Change Right 检查

```cpp
bool LaneChangeRightModule::isExecutionRequested() {
  // 检查右侧车道
  const auto right_lanes = route_handler->getRight Lanelet(current_lane);
  if (right_lanes.empty()) {
    return false;  // ❌ 右侧无车道
  }
  
  // ... 其他检查
  
  return false;
}
```

**结果：** ❌ **不请求**（右侧没有车道或不需要右换道）

---

### Step 1.4: Side Shift 检查

```cpp
bool SideShiftModule::isExecutionRequested() {
  // Side Shift 用于小幅度横向调整，通常不主动请求
  return false;
}
```

**结果：** ❌ **不请求**

---

**请求汇总：**
```
Request Modules:
  ✅ Static Obstacle Avoidance
  ✅ Lane Change Left
  ❌ Lane Change Right
  ❌ Side Shift
```

---

## 阶段2️⃣：Candidate Modules 并行运行

### Module A: Static Obstacle Avoidance (Candidate)

**输入：**
```cpp
Input: approved_modules_output (来自Slot 1)
  path.points = [0m至100m，速度8.33 m/s]
  drivable_area = Lane 200
```

**处理过程：**
```cpp
BehaviorModuleOutput StaticObstacleAvoidanceModule::run(
  const BehaviorModuleOutput & input
) {
  // 1. 生成避障路径
  auto avoid_path = generateAvoidancePath(input.path);
  
  // 2. 计算横向偏移量
  const double obstacle_position = 50.0;  // m
  const double obstacle_width = 1.8;  // m
  const double safety_margin = 1.0;  // m
  const double required_offset = (obstacle_width / 2) + safety_margin;  // 1.9m
  
  // 3. 生成平滑的横向偏移轨迹
  // 从30m开始偏移，50m处最大偏移，70m回归中心
  
  for (auto & point : avoid_path.points) {
    double s = point.s;  // 沿路径的距离
    double lateral_offset = 0.0;
    
    if (s >= 30.0 && s < 50.0) {
      // 30-50m: 平滑增加偏移 (0 → 1.9m)
      lateral_offset = required_offset * (s - 30.0) / 20.0;
    } else if (s >= 50.0 && s < 70.0) {
      // 50-70m: 平滑减少偏移 (1.9m → 0)
      lateral_offset = required_offset * (70.0 - s) / 20.0;
    }
    
    // 应用横向偏移
    point.point.pose.position.y += lateral_offset;
    
    // 4. 调整速度（接近障碍物时减速）
    if (s >= 40.0 && s <= 60.0) {
      // 在障碍物附近减速到6.0 m/s
      double speed_ratio = 0.72;  // 6.0 / 8.33
      point.point.longitudinal_velocity_mps = 8.333 * speed_ratio;
    }
  }
  
  // 5. 设置转向灯信息
  output.turn_signal_info.turn_signal.command = TurnSignal::LEFT;
  output.turn_signal_info.desired_start_point = getPoint(30.0);
  output.turn_signal_info.required_start_point = getPoint(25.0);
  
  return output;
}
```

**输出（Candidate Path）：**
```cpp
CandidateOutput_StaticAvoidance = {
  path.points = [
    // 0-30m: 保持中心线
    {s: 0m,  y: 0.0,    velocity: 8.333 m/s},
    {s: 10m, y: 0.0,    velocity: 8.333 m/s},
    {s: 20m, y: 0.0,    velocity: 8.333 m/s},
    {s: 30m, y: 0.0,    velocity: 8.333 m/s},
    
    // 30-50m: 开始横向偏移，接近障碍物
    {s: 35m, y: +0.475, velocity: 8.333 m/s},
    {s: 40m, y: +0.95,  velocity: 7.200 m/s},  // 开始减速
    {s: 45m, y: +1.425, velocity: 6.000 m/s},
    {s: 50m, y: +1.900, velocity: 6.000 m/s},  // 最大偏移，通过障碍物
    
    // 50-70m: 回归中心线
    {s: 55m, y: +1.425, velocity: 6.000 m/s},
    {s: 60m, y: +0.95,  velocity: 7.200 m/s},  // 开始加速
    {s: 65m, y: +0.475, velocity: 8.333 m/s},
    {s: 70m, y: 0.0,    velocity: 8.333 m/s},  // 回归正常
    
    // 70m+: 正常行驶
    {s: 80m, y: 0.0, velocity: 8.333 m/s},
    ...
  ],
  
  turn_signal_info = {
    turn_signal: LEFT,
    start_position: 30m
  },
  
  status: WAITING_APPROVAL
}
```

---

### Module B: Lane Change Left (Candidate)

**输入：**
```cpp
Input: approved_modules_output (来自Slot 1，同Static Avoidance)
  path.points = [0m至100m，速度8.33 m/s，lane_ids=[200]]
```

**处理过程：**
```cpp
BehaviorModuleOutput LaneChangeLeftModule::run(
  const BehaviorModuleOutput & input
) {
  // 1. 检查换道安全性
  const auto left_lane = route_handler->getLeftLanelet(current_lane);
  const auto safety_check = checkSafety(left_lane);
  
  if (!safety_check.is_safe) {
    return {};  // 不安全，不生成候选路径
  }
  
  // 2. 计算换道参数
  const double lane_change_distance = 30.0;  // 换道距离
  const double lane_width = 3.5;  // 车道宽度
  const double preparation_distance = 10.0;  // 准备距离
  
  // 3. 生成换道路径
  const double lc_start = 20.0;  // 从20m开始换道
  const double lc_end = lc_start + lane_change_distance;  // 50m完成换道
  
  auto lane_change_path = input.path;
  
  for (auto & point : lane_change_path.points) {
    double s = point.s;
    
    if (s >= lc_start && s <= lc_end) {
      // 20-50m: 平滑换道 (Lane 2 → Lane 1)
      double progress = (s - lc_start) / lane_change_distance;
      double lateral_shift = lane_width * progress;  // 0 → 3.5m
      
      point.point.pose.position.y += lateral_shift;
      
      // 更新lane_ids
      if (progress < 0.5) {
        point.lane_ids = {200, 100};  // 跨越两个车道
      } else {
        point.lane_ids = {100};  // 进入左侧车道
      }
      
      // 4. 换道时根据曲率调整速度
      double curvature = calculateCurvature(point);
      double speed_limit = calculateSpeedForCurvature(curvature);
      point.point.longitudinal_velocity_mps = std::min(8.333, speed_limit);
    } else if (s > lc_end) {
      // 50m后: 在左侧车道行驶
      point.point.pose.position.y += lane_width;
      point.lane_ids = {100};  // Lane 1
      point.point.longitudinal_velocity_mps = 8.333;
    }
  }
  
  // 5. 设置转向灯
  output.turn_signal_info.turn_signal.command = TurnSignal::LEFT;
  output.turn_signal_info.desired_start_point = getPoint(15.0);
  
  return output;
}
```

**输出（Candidate Path）：**
```cpp
CandidateOutput_LaneChangeLeft = {
  path.points = [
    // 0-20m: Lane 2, 准备换道
    {s: 0m,  y: 0.0,  lane_ids: [200], velocity: 8.333 m/s},
    {s: 10m, y: 0.0,  lane_ids: [200], velocity: 8.333 m/s},
    {s: 20m, y: 0.0,  lane_ids: [200], velocity: 8.333 m/s},
    
    // 20-50m: 换道过程 (Lane 2 → Lane 1)
    {s: 25m, y: +0.583, lane_ids: [200,100], velocity: 8.333 m/s},
    {s: 30m, y: +1.167, lane_ids: [200,100], velocity: 8.333 m/s},
    {s: 35m, y: +1.750, lane_ids: [200,100], velocity: 8.000 m/s},  // 轻微减速
    {s: 40m, y: +2.333, lane_ids: [200,100], velocity: 8.000 m/s},
    {s: 45m, y: +2.917, lane_ids: [100], velocity: 8.000 m/s},
    {s: 50m, y: +3.500, lane_ids: [100], velocity: 8.333 m/s},  // 完成换道
    
    // 50m+: Lane 1, 正常行驶（已避开障碍物）
    {s: 60m, y: +3.500, lane_ids: [100], velocity: 8.333 m/s},
    {s: 70m, y: +3.500, lane_ids: [100], velocity: 8.333 m/s},
    {s: 80m, y: +3.500, lane_ids: [100], velocity: 8.333 m/s},
    ...
  ],
  
  turn_signal_info = {
    turn_signal: LEFT,
    start_position: 15m
  },
  
  status: WAITING_APPROVAL
}
```

---

**Candidate阶段输出对比：**
```
┌─────────────────────────────────────────────────────────────┐
│ 方案A: Static Obstacle Avoidance (横向偏移避障)              │
│ ════════════════════════╗ 🚗 ╔════════════════════════      │
│              Ego→      ╚═══╝                                 │
│            ↗ ↗ ↗ ↗ → → → → → ↘ ↘ ↘ ↘                       │
├─────────────────────────────────────────────────────────────┤
│ 方案B: Lane Change Left (换道避障)                           │
│              ↗ ↗ ↗ ↗ ↗ ↗ → → → → → → → →                   │
│ ════════════════════════╗ 🚗 ╔════════════════════════      │
│              Ego→      ╚═══╝                                 │
└─────────────────────────────────────────────────────────────┘

两个候选路径都可视化显示，等待决策
```

---

## 阶段3️⃣：决策和批准（Approval Phase）

### Step 3.1: 评估候选方案

```cpp
// Manager评估两个候选方案
struct CandidateEvaluation {
  double safety_score;
  double comfort_score;
  double efficiency_score;
  double total_score;
};

// 方案A评估
CandidateEvaluation eval_avoidance = {
  safety_score: 0.8,     // 较安全，但接近障碍物
  comfort_score: 0.7,    // 需要减速和横向移动
  efficiency_score: 0.6, // 需要减速，效率较低
  total_score: 0.7
};

// 方案B评估
CandidateEvaluation eval_lane_change = {
  safety_score: 0.9,     // 很安全，完全避开障碍物
  comfort_score: 0.9,    // 平滑换道
  efficiency_score: 0.9, // 速度保持，效率高
  total_score: 0.9       // 🏆 更高分数
};

// 决策：选择Lane Change Left
if (enable_rtc == false) {
  // 自动批准
  approveModule(lane_change_left_module);
}
```

**结果：** ✅ **Lane Change Left 被批准**

---

## 阶段4️⃣：Approved Module 串行运行

### 现在 Approved Stack 中只有 Lane Change Left

**输入：**
```cpp
Input: approved_modules_output (Slot 1的输出)
```

**处理：**
```cpp
BehaviorModuleOutput output = input;

// Lane Change Left (Approved) 运行
output = lane_change_left_module->run(output);
```

**输出：**
```cpp
ApprovedOutput = {
  path.points = [
    // 换道路径（已在Candidate阶段计算）
    {s: 0m,  y: 0.0,    lane_ids: [200], velocity: 8.333 m/s},
    {s: 20m, y: 0.0,    lane_ids: [200], velocity: 8.333 m/s},
    {s: 30m, y: +1.167, lane_ids: [200,100], velocity: 8.333 m/s},
    {s: 50m, y: +3.500, lane_ids: [100], velocity: 8.333 m/s},
    {s: 60m, y: +3.500, lane_ids: [100], velocity: 8.333 m/s},
    ...
  ],
  
  turn_signal_info = {
    turn_signal: LEFT,
    start_position: 15m,
    end_position: 50m
  },
  
  drivable_area_info = {
    drivable_lanes: [Lane 100, Lane 200],  // 换道时两个车道都可用
  }
}
```

---

## 阶段5️⃣：Static Avoidance 检测换道后不再需要

### 下一个Planning周期

```cpp
bool StaticObstacleAvoidanceModule::isExecutionRequested() {
  // 输入路径已经在Lane 1（左侧车道）
  // 障碍物在Lane 2（右侧车道）
  
  const auto obstacles = detectStaticObstacles(input_path);
  
  // 检查障碍物距离当前路径的距离
  for (const auto & obs : obstacles) {
    double lateral_distance = calculateLateralDistance(obs, path);
    // lateral_distance = 3.5m (整个车道宽度)
    
    if (lateral_distance > safe_margin) {
      // 障碍物已经不在路径上
      return false;  // ❌ 不再请求
    }
  }
  
  return false;
}
```

**结果：** ❌ Static Avoidance 不再请求执行（问题已解决）

---

## 📊 完整数据流总结

### 时序图

```
时间 | Approved Stack              | Candidate Stack              | 输出路径
────┼────────────────────────────┼─────────────────────────────┼──────────────
T0  │ []                         │ []                          │ Reference Path
    │                            │                             │ (Lane 2中心线)
────┼────────────────────────────┼─────────────────────────────┼──────────────
T1  │ []                         │ [Static Avoidance]          │ Reference Path
    │                            │ [Lane Change Left]          │ + 2个候选路径
    │                            │                             │ (可视化显示)
────┼────────────────────────────┼─────────────────────────────┼──────────────
T2  │ [Lane Change Left]         │ []                          │ 换道路径
    │ (已批准，开始执行)          │                             │ (Lane 1)
────┼────────────────────────────┼─────────────────────────────┼──────────────
T3  │ []                         │ []                          │ 换道路径
    │ (换道完成，模块SUCCESS)     │                             │ (继续在Lane 1)
────┴────────────────────────────┴─────────────────────────────┴──────────────
```

---

## 🔍 详细输入输出对照表

### Module: Static Obstacle Avoidance

| 阶段 | 输入 | 处理 | 输出 |
|-----|------|------|------|
| **Request** | PlannerData<br>- ego_pose: (0, 0)<br>- obstacles: [{pos:(50,0), type:PARKED}]<br>- current_path: Lane2中心线 | 检测到障碍物<br>需要避障 | return **true** |
| **Candidate** | BehaviorModuleOutput<br>- path: Lane2中心线<br>- velocity: 8.33 m/s | 生成避障路径：<br>- 30-70m横向偏移1.9m<br>- 40-60m减速至6.0m/s<br>- 转向灯：LEFT | CandidatePath<br>- 横向偏移轨迹<br>- 速度调整<br>- status: WAITING |
| **Approved** | *(未被批准)* | *(未执行)* | *(无输出)* |

### Module: Lane Change Left

| 阶段 | 输入 | 处理 | 输出 |
|-----|------|------|------|
| **Request** | PlannerData<br>- ego_pose: (0, 0)<br>- left_lane: Lane1可用<br>- left_lane_objects: 无障碍 | 检测左侧可换道<br>换道安全 | return **true** |
| **Candidate** | BehaviorModuleOutput<br>- path: Lane2中心线<br>- velocity: 8.33 m/s | 生成换道路径：<br>- 20-50m换到Lane1<br>- y偏移0→3.5m<br>- lane_ids: 200→100<br>- 转向灯：LEFT | CandidatePath<br>- 换道轨迹<br>- 多车道ID<br>- status: WAITING |
| **Approved** | BehaviorModuleOutput<br>- path: Lane2中心线 | 执行换道：<br>- 应用换道路径<br>- 更新drivable_area<br>- 激活转向灯 | ApprovedOutput<br>- path: 换道路径<br>- turn_signal: LEFT<br>- status: RUNNING |

---

## 🎯 关键数据结构详解

### BehaviorModuleOutput 完整结构

```cpp
struct BehaviorModuleOutput {
  // 1. 路径信息
  PathWithLaneId path;
  // path.points = [
  //   PathPointWithLaneId {
  //     lane_ids: vector<int64_t>,        // 所属车道ID列表
  //     point: {
  //       pose: {
  //         position: {x, y, z},          // 位置
  //         orientation: {x, y, z, w}      // 姿态（四元数）
  //       },
  //       longitudinal_velocity_mps: float, // 纵向速度 [m/s]
  //       lateral_velocity_mps: float,      // 横向速度 [m/s]
  //       acceleration_mps2: float,         // 加速度 [m/s²]
  //       heading_rate_rps: float,          // 航向角速度 [rad/s]
  //       front_wheel_angle_rad: float,     // 前轮转角 [rad]
  //       rear_wheel_angle_rad: float       // 后轮转角 [rad]
  //     }
  //   },
  //   ... 更多点
  // ]
  
  // 2. 参考路径（仅用于可视化）
  PathWithLaneId reference_path;
  
  // 3. 可行驶区域信息
  DrivableAreaInfo drivable_area_info;
  // drivable_area_info = {
  //   drivable_lanes: vector<DrivableLanes>,
  //   obstacles: vector<DrivableAreaInfo::Obstacle>,
  //   enable_expanding_hatched_road_markings: bool,
  //   enable_expanding_intersection_areas: bool,
  //   enable_expanding_freespace_areas: bool
  // }
  
  // 4. 转向灯信息
  TurnSignalInfo turn_signal_info;
  // turn_signal_info = {
  //   turn_signal: {
  //     command: LEFT/RIGHT/HAZARD,  // 转向灯命令
  //   },
  //   desired_start_point: Pose,      // 期望开启位置
  //   required_start_point: Pose,     // 必需开启位置
  //   desired_end_point: Pose,        // 期望关闭位置
  //   required_end_point: Pose        // 必需关闭位置
  // }
  
  // 5. 目标点修改信息（仅Goal Planner使用）
  std::optional<PoseWithUuidStamped> modified_goal;
  
  // 6. 停车路径信息
  std::optional<ParkingPathWithTime> parking_path;
};
```

### PlannerData 完整结构

```cpp
struct PlannerData {
  // 1. 车辆状态
  Odometry::ConstSharedPtr self_odometry;
  // self_odometry = {
  //   pose: {position: {x, y, z}, orientation: {x, y, z, w}},
  //   twist: {linear: {x, y, z}, angular: {x, y, z}}
  // }
  
  AccelWithCovarianceStamped::ConstSharedPtr self_acceleration;
  
  // 2. 地图和路线
  std::shared_ptr<RouteHandler> route_handler;
  // route_handler提供：
  //   - getLeftLanelet()
  //   - getRightLanelet()
  //   - getCenterLinePath()
  //   - getClosestLaneletWithinRoute()
  
  // 3. 障碍物信息
  PredictedObjects::ConstSharedPtr dynamic_object;
  // dynamic_object.objects = [
  //   {
  //     object_id: uuid,
  //     classification: CAR/PEDESTRIAN/...,
  //     kinematics: {pose, twist, ...},
  //     shape: {dimensions, footprint},
  //     predicted_paths: [...]  // 预测轨迹
  //   },
  //   ...
  // ]
  
  // 4. 交通灯状态
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map;
  
  // 5. 占用栅格地图
  OccupancyGrid::ConstSharedPtr occupancy_grid;
  
  // 6. 代价地图
  OccupancyGrid::ConstSharedPtr costmap;
  
  // 7. 操作模式
  OperationModeState::ConstSharedPtr operation_mode;
  
  // 8. 上一次输出路径
  std::shared_ptr<PathWithLaneId const> prev_output_path;
  
  // 9. 参数
  BehaviorPathPlannerParameters parameters;
  // parameters = {
  //   backward_path_length: 5.0,
  //   forward_path_length: 300.0,
  //   output_path_interval: 2.0,
  //   ego_nearest_dist_threshold: 3.0,
  //   ...
  // }
};
```

---

## 💡 模块间协作关键点

### 1. **并行运行的独立性**

```cpp
// Candidate阶段：两个模块完全独立
parallel_for_each(candidate_modules, [&](auto module) {
  // 每个模块都从相同的输入开始
  auto result = module->run(approved_output);
  results[module->name()] = result;
});

// 关键：输入完全相同
// - Static Avoidance输入：Lane2中心线路径
// - Lane Change输入：Lane2中心线路径（相同）

// 输出完全独立
// - Static Avoidance输出：横向偏移路径（仍在Lane2）
// - Lane Change输出：换道路径（移到Lane1）
```

### 2. **串行运行的依赖性**

```cpp
// Approved阶段：模块串行处理
BehaviorModuleOutput output = input;

for (auto & module : approved_modules) {
  output = module->run(output);  // 输出作为下一个的输入
}

// 示例：如果两个模块都批准
// 1. Input: Reference Path (Lane2)
// 2. Lane Change执行：Output1 = 换道路径 (Lane1)
// 3. Static Avoidance执行：Output2 = 在Lane1上避障（如果需要）
// 4. Final Output: Output2
```

### 3. **状态转换**

```
Candidate (WAITING_APPROVAL)
  ↓ 
  【批准决策】
  ↓
Approved (RUNNING)
  ↓
  【执行一段时间】
  ↓
SUCCESS (任务完成) / FAILURE (任务失败)
  ↓
  【模块移除】
```

---

## 📈 性能和时序

### 单个Planning周期（100ms @ 10Hz）

```
Timeline (ms):
0    10   20   30   40   50   60   70   80   90   100
├────┼────┼────┼────┼────┼────┼────┼────┼────┼────┤
│    │    │    │    │    │    │    │    │    │    │
│ 生成参考路径 (5ms)                                 │
│    ├────┤                                         │
│         │ Request检查 (10ms)                      │
│         ├──────────┤                              │
│                    │ Candidate并行运行 (30ms)     │
│                    ├───────────────────────┤      │
│                                           │ 决策 (5ms)
│                                           ├──┤    │
│                                              │ Approved运行 (20ms)
│                                              ├─────────┤
│                                                        │ 后处理 (10ms)
│                                                        ├───────┤
│                                                                │ 发布 (5ms)
│                                                                └──┤

总计：85ms（留有15ms余量）
```

---

## 🔧 调试命令

### 查看Candidate和Approved状态

```bash
# 查看所有模块状态
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/scene_module_status

# 输出示例：
# - module_name: 'static_obstacle_avoidance'
#   status: 'WAITING_APPROVAL'    # Candidate
#   is_waiting_approval: true
# 
# - module_name: 'lane_change_left'
#   status: 'RUNNING'              # Approved
#   is_waiting_approval: false

# 查看候选路径（可视化）
ros2 topic echo /planning/.../path_candidate/static_obstacle_avoidance
ros2 topic echo /planning/.../path_candidate/lane_change_left

# 查看最终输出路径
ros2 topic echo /planning/behavior_planning/path_with_lane_id
```

### 使用脚本诊断

```bash
./debug_bpp_velocity.sh

# 会显示：
# [3] 激活的场景模块
#   lane_change_left: RUNNING (Approved)
#   static_obstacle_avoidance: IDLE
```

---

## 📝 总结

### Candidate vs Approved 对比

| 特性 | Candidate Modules | Approved Modules |
|-----|------------------|------------------|
| **运行方式** | 并行 | 串行 |
| **输入** | 相同（Approved Stack输出） | 上一个模块的输出 |
| **输出** | 候选路径（仅可视化） | 实际执行路径 |
| **状态** | WAITING_APPROVAL | RUNNING |
| **影响车辆** | 不影响 | 直接影响 |
| **数量** | 可以有多个 | 可以有多个（串行） |
| **批准** | 需要批准才能进入Approved | 已批准 |

### 模块协作原则

1. **独立评估**：每个模块独立判断是否需要执行
2. **并行生成**：Candidate阶段并行生成多个方案
3. **最优选择**：从多个候选方案中选择最优方案
4. **串行执行**：Approved模块串行修改路径，确保连贯性
5. **动态调整**：每个周期重新评估，灵活应对变化

### 输入输出流转

```
Reference Path
  ↓
[Candidate并行生成] → 方案A, 方案B, 方案C...
  ↓
[批准最优方案] → 方案B
  ↓
[Approved串行执行] → Module1(Input) → Output1
                                    ↓
                    → Module2(Output1) → Output2
                                        ↓
                    → Final Output
```

这就是Behavior Path Planner中各个模块的协作机制！每个模块都是一个独立的"专家"，在自己擅长的场景下提供解决方案，最终由Manager协调选择和执行最优方案。
