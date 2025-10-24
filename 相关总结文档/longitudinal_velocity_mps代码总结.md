# behavior_path_planner 模块中 longitudinal_velocity_mps 代码总结

本文档总结了 `behavior_path_planner` 文件夹下所有涉及 `longitudinal_velocity_mps` 的代码行，包括代码位置、功能和作用。

---

## 目录

1. [主节点 (behavior_path_planner_node.cpp)](#1-主节点)
2. [Start Planner 模块](#2-start-planner-模块)
3. [Goal Planner 模块](#3-goal-planner-模块)
4. [Static Obstacle Avoidance 模块](#4-static-obstacle-avoidance-模块)
5. [Dynamic Obstacle Avoidance 模块](#5-dynamic-obstacle-avoidance-模块)
6. [Lane Change 模块](#6-lane-change-模块)
7. [通用工具函数](#7-通用工具函数)

---

## 1. 主节点

### 文件: `autoware_behavior_path_planner/src/behavior_path_planner_node.cpp`

#### 1.1 输出路径速度打印（调试日志）
**位置**: 第 405-412 行
```cpp
const auto & last_point = path->points.back().point;
RCLCPP_INFO_THROTTLE(
  get_logger(), *get_clock(), 1000,
  "[VEL_DEBUG][BPP][PUBLISH] Output path: points=%zu, first_vel=%.3f m/s, last_vel=%.3f m/s",
  path->points.size(),
  first_point.longitudinal_velocity_mps,
  last_point.longitudinal_velocity_mps
);
```
**作用**: 
- 打印发布路径的首尾速度信息
- 用于调试，每1000ms打印一次
- 帮助追踪速度传递问题

#### 1.2 未准备好时将速度设为0
**位置**: 第 714-718 行
```cpp
if (!is_ready) {
  for (auto & point : output.points) {
    point.longitudinal_velocity_mps = 0.0;
  }
}
```
**作用**: 
- 当规划器未准备好时，强制将所有路径点速度设为0
- 确保在系统未就绪时不会产生危险的速度指令
- **这是导致速度为0的关键原因之一**

---

## 2. Start Planner 模块

### 2.1 文件: `autoware_behavior_path_start_planner_module/src/start_planner_module.cpp`

#### 2.1.1 停止路径速度设为0
**位置**: 第 830-832 行
```cpp
for (auto & p : stop_path.points) {
  p.point.longitudinal_velocity_mps = 0.0;
}
```
**作用**: 
- 生成停止路径时，将所有点速度设为0
- 用于起步前的等待状态
- **这是 start_planner 输出速度为0的主要原因**

#### 2.1.2 初始化路径点速度为0
**位置**: 第 1026 行
```cpp
auto toPathPointWithLaneId = [this](const Pose & pose) {
  PathPointWithLaneId p{};
  p.point.pose = pose;
  p.point.longitudinal_velocity_mps = 0.0;
  // ...
}
```
**作用**: 
- 创建新路径点时，初始速度设为0
- 作为默认初始值，后续可能被修改

### 2.2 文件: `autoware_behavior_path_start_planner_module/src/geometric_pull_out.cpp`

#### 2.2.1 第一段弧线路径末端停止
**位置**: 第 104 行
```cpp
output.partial_paths.front().points.back().point.longitudinal_velocity_mps = 0.0;
```
**作用**: 
- 在几何起步规划中，第一段弧线路径的末端设置停止点
- 用于分段起步轨迹的中间停止点

### 2.3 文件: `autoware_behavior_path_start_planner_module/src/shift_pull_out.cpp`

#### 2.3.1 获取道路速度限制
**位置**: 第 305-306 行
```cpp
const double road_velocity =
  road_lane_reference_path.points.at(shift_start_idx).point.longitudinal_velocity_mps;
```
**作用**: 
- 从参考路径中读取道路速度限制
- 用于计算起步时的目标速度

#### 2.3.2 限制起步速度
**位置**: 第 450-451 行
```cpp
point.point.longitudinal_velocity_mps =
  std::min(point.point.longitudinal_velocity_mps, static_cast<float>(terminal_velocity));
```
**作用**: 
- 在起步结束点之前，限制速度不超过终端速度
- 确保平滑加速

#### 2.3.3 终点速度设为0
**位置**: 第 456 行
```cpp
if (path_terminal_is_goal) {
  shifted_path.path.points.back().point.longitudinal_velocity_mps = 0.0;
}
```
**作用**: 
- 如果路径终点是目标点，设置速度为0
- 确保在目标位置停止

---

## 3. Goal Planner 模块

### 3.1 文件: `autoware_behavior_path_goal_planner_module/src/goal_planner_module.cpp`

#### 3.1.1 限制路径速度（含调试日志）
**位置**: 第 1452-1457 行
```cpp
const auto old_vel = p.point.longitudinal_velocity_mps;
p.point.longitudinal_velocity_mps = std::min(p.point.longitudinal_velocity_mps, vel);
if (std::abs(old_vel - p.point.longitudinal_velocity_mps) > 0.01) {
  modified_count++;
}
```
**作用**: 
- 限制路径速度不超过指定值
- 记录修改次数用于调试
- 添加了调试日志以追踪速度修改

#### 3.1.2 靠边停车速度限制
**位置**: 第 1926-1927 行
```cpp
p.point.longitudinal_velocity_mps =
  std::min(p.point.longitudinal_velocity_mps, static_cast<float>(pull_over_velocity));
```
**作用**: 
- 在靠边停车时，限制速度为pull_over_velocity参数值
- 确保低速安全停车

#### 3.1.3 保持停止状态
**位置**: 第 2175 行
```cpp
for (auto & p : path.points) {
  p.point.longitudinal_velocity_mps = 0.0;
}
```
**作用**: 
- 在需要保持停止状态时，强制所有点速度为0
- 用于停车后的保持阶段

#### 3.1.4 靠边停车路径减速
**位置**: 第 2223-2224 行
```cpp
p.point.longitudinal_velocity_mps = std::min(
  p.point.longitudinal_velocity_mps, static_cast<float>(parameters_.pull_over_velocity));
```
**作用**: 
- 限制靠边停车路径的速度
- 使用配置参数中的靠边停车速度

#### 3.1.5 转向信号减速计算
**位置**: 第 2239 行
```cpp
const float decel_vel =
  std::min(point.point.longitudinal_velocity_mps, static_cast<float>(distance_to_stop / time));
```
**作用**: 
- 计算转向信号时的减速速度
- 根据停止距离和时间计算

#### 3.1.6 应用转向信号减速
**位置**: 第 2253 行
```cpp
point.point.longitudinal_velocity_mps = decel_vel;
```
**作用**: 
- 应用计算出的减速速度
- 确保在转向信号激活时安全减速

### 3.2 文件: `autoware_behavior_path_goal_planner_module/src/pull_over_planner/bezier_pull_over.cpp`

#### 3.2.1 目标点速度设为0
**位置**: 第 290 行
```cpp
PathPointWithLaneId p{};
p.point.longitudinal_velocity_mps = 0.0;
p.point.pose = goal_pose;
```
**作用**: 
- 创建目标点时，速度设为0
- 确保在目标位置完全停止

#### 3.2.2 贝塞尔靠边停车速度限制
**位置**: 第 305-306 行
```cpp
point.point.longitudinal_velocity_mps =
  std::min(point.point.longitudinal_velocity_mps, static_cast<float>(pull_over_velocity));
```
**作用**: 
- 在贝塞尔曲线靠边停车路径中限制速度
- 从换道开始点到路径结束

#### 3.2.3 贝塞尔靠边停车减速
**位置**: 第 385-386 行
```cpp
point.point.longitudinal_velocity_mps =
  std::min(point.point.longitudinal_velocity_mps, decelerated_velocity);
```
**作用**: 
- 在减速区间内平滑减速
- 计算考虑了距离和时间因素

### 3.3 文件: `autoware_behavior_path_goal_planner_module/src/pull_over_planner/freespace_pull_over.cpp`

#### 3.3.1 自由空间规划速度设置
**位置**: 第 278-280 行
```cpp
for (auto & point : partial_path.points) {
  point.point.longitudinal_velocity_mps = velocity;
}
```
**作用**: 
- 为自由空间规划的路径点设置统一速度
- 用于低速泊车场景

#### 3.3.2 自由空间停止点
**位置**: 第 285 行
```cpp
if (set_stop_end) {
  partial_path.points.back().point.longitudinal_velocity_mps = 0.0;
}
```
**作用**: 
- 如果需要，在路径末端设置停止点
- 用于分段路径的中间停止

---

## 4. Static Obstacle Avoidance 模块

### 文件: `autoware_behavior_path_static_obstacle_avoidance_module/src/scene.cpp`

#### 4.1 向后路径速度覆盖
**位置**: 第 961-962 行
```cpp
p.point.longitudinal_velocity_mps =
  original_path.points.at(orig_ego_idx).point.longitudinal_velocity_mps;
```
**作用**: 
- 用最新的速度覆盖向后扩展的路径速度
- 确保回退路径速度一致性

#### 4.2 道路速度计算换道时间
**位置**: 第 1341 行
```cpp
const double road_velocity = avoid_data_.reference_path.points.at(front_new_shift_line.start_idx)
                               .point.longitudinal_velocity_mps;
```
**作用**: 
- 从参考路径获取道路速度
- 用于计算避障换道的时间和加速度

#### 4.3 避障减速（前方障碍物）
**位置**: 第 1737-1741 行
```cpp
const double v_original = shifted_path.path.points.at(i).point.longitudinal_velocity_mps;
const double v_insert =
  std::max(v_target - parameters_->buf_slow_down_speed, parameters_->min_slow_down_speed);

shifted_path.path.points.at(i).point.longitudinal_velocity_mps = std::min(v_original, v_insert);
```
**作用**: 
- 在障碍物前方区域减速
- 根据横向加速度（jerk）限制计算目标速度
- 确保安全避让

#### 4.4 避障减速（等待点）
**位置**: 第 1952-1955 行
```cpp
const double v_original = shifted_path.path.points.at(i).point.longitudinal_velocity_mps;
const double v_insert = std::max(v_target - parameters_->buf_slow_down_speed, lower_speed);

shifted_path.path.points.at(i).point.longitudinal_velocity_mps = std::min(v_original, v_insert);
```
**作用**: 
- 在等待点插入减速
- 类似前方障碍物减速，但有最低速度限制

#### 4.5 避障减速（动态调整 + 调试日志）
**位置**: 第 2035-2050 行
```cpp
const double v_target = std::max(getEgoSpeed(), std::sqrt(v_target_square));
const double v_original = shifted_path.path.points.at(i).point.longitudinal_velocity_mps;
shifted_path.path.points.at(i).point.longitudinal_velocity_mps = std::min(v_original, v_target);

// [VEL_DEBUG] 每20个点打印一次
if (i % 20 == start_idx && std::abs(v_original - shifted_path.path.points.at(i).point.longitudinal_velocity_mps) > 0.01) {
  RCLCPP_DEBUG(
    getLogger(),
    "[VEL_DEBUG][Avoidance][POINT_%zu] %.3f -> %.3f m/s (ego=%.3f, target=%.3f, dist=%.2f)",
    i, v_original, shifted_path.path.points.at(i).point.longitudinal_velocity_mps,
    getEgoSpeed(), v_target, distance_from_ego
  );
}

if (std::abs(v_original - shifted_path.path.points.at(i).point.longitudinal_velocity_mps) > 0.01) {
  modified_count++;
}
```
**作用**: 
- 根据横向加速度限制动态调整速度
- 详细的调试日志记录速度修改
- 计数修改的点数用于诊断

---

## 5. Dynamic Obstacle Avoidance 模块

### 文件: `autoware_behavior_path_dynamic_obstacle_avoidance_module/src/scene.cpp`

#### 5.1 动态减速速度计算
**位置**: 第 532-533 行
```cpp
const double original_vel = path_points.at(i).point.longitudinal_velocity_mps;
const double filtered_vel = longitudinal_velocity_mps_vec.at(i);
```
**作用**: 
- 获取原始速度和滤波后的速度
- 用于速度平滑处理

#### 5.2 应用动态减速
**位置**: 第 546-547 行
```cpp
path_points.at(i).point.longitudinal_velocity_mps = std::max(
  static_cast<float>(filtered_vel - additional_vel_to_stop), 0.0f);
```
**作用**: 
- 应用滤波后的减速速度
- 减去额外的停止速度余量
- 确保速度不为负

---

## 6. Lane Change 模块

### 6.1 文件: `autoware_behavior_path_lane_change_module/src/utils/utils.cpp`

#### 6.1.1 获取参考路径速度
**位置**: 第 1018 行
```cpp
const auto v0 = path.points.at(idx).point.longitudinal_velocity_mps;
```
**作用**: 
- 从参考路径获取当前速度
- 用于换道速度计算

#### 6.1.2 换道速度插值
**位置**: 第 1066-1069 行
```cpp
const auto v1 = ref_path.points.at(seg_idx).point.longitudinal_velocity_mps;
const auto v2 = ref_path.points.at(seg_idx + 1).point.longitudinal_velocity_mps;
lc_path.points.at(p_idx).point.longitudinal_velocity_mps =
  static_cast<float>(v1 + (v2 - v1) * ratio);
```
**作用**: 
- 在换道路径点间进行速度插值
- 确保速度平滑过渡

### 6.2 文件: `autoware_behavior_path_lane_change_module/src/scene.cpp`

#### 6.2.1 换道准备阶段减速
**位置**: 第 632-633 行
```cpp
point.point.longitudinal_velocity_mps = std::min(
  point.point.longitudinal_velocity_mps, static_cast<float>(terminal_velocity));
```
**作用**: 
- 在换道准备阶段限制速度
- 确保安全换道条件

#### 6.2.2 换道终点速度为0
**位置**: 第 638 行
```cpp
if (path_terminal_is_goal) {
  lane_changing_path.path.points.back().point.longitudinal_velocity_mps = 0.0;
}
```
**作用**: 
- 如果换道终点是目标点，速度设为0
- 确保在目标位置停止

---

## 7. 通用工具函数

### 7.1 文件: `autoware_behavior_path_planner_common/src/utils/parking_departure/geometric_parallel_parking.cpp`

#### 7.1.1 弧线路径速度设置
**位置**: 第 101-103 行
```cpp
if (i == path.points.size() - 1 && set_stop_end) {
  path.points.at(i).point.longitudinal_velocity_mps = 0.0;
} else {
  path.points.at(i).point.longitudinal_velocity_mps = velocity;
}
```
**作用**: 
- 为平行泊车的弧线路径设置速度
- 末端可选择性设置停止点

#### 7.1.2 泊车目标点速度为0
**位置**: 第 307 行
```cpp
if (path_terminal_is_goal) {
  paths.back().points.back().point.longitudinal_velocity_mps = 0.0;
}
```
**作用**: 
- 泊车路径终点速度为0
- 确保完全停止

#### 7.1.3 后退路径末端停止
**位置**: 第 370 行
```cpp
if (!path.points.empty()) {
  path.points.back().point.longitudinal_velocity_mps = 0;
}
```
**作用**: 
- 后退路径的末端设置停止
- 用于泊车后退阶段

### 7.2 文件: `autoware_behavior_path_planner_common/src/utils/path_safety_checker/safety_check.cpp`

#### 7.2.1 预测路径速度设置
**位置**: 第 370-380 行
```cpp
object_path_point.longitudinal_velocity_mps = static_cast<float>(obj_vel);
// ...
object_path_point.longitudinal_velocity_mps = std::max(
  object_path_point.longitudinal_velocity_mps, stop_velocity);
```
**作用**: 
- 为预测对象路径设置速度
- 用于安全性检查
- 确保最低速度不低于停止速度

### 7.3 文件: `autoware_behavior_path_planner_common/src/utils/path_utils.cpp`

#### 7.3.1 路径点速度复制
**位置**: 多处
```cpp
point.point.longitudinal_velocity_mps = velocity;
```
**作用**: 
- 在各种路径生成函数中设置速度
- 用于路径重采样和生成

#### 7.3.2 减速点插入
**位置**: 第 208-209 行
```cpp
inserted_point.point.longitudinal_velocity_mps =
  std::min(inserted_point.point.longitudinal_velocity_mps, velocity);
```
**作用**: 
- 在路径中插入减速点
- 限制速度不超过指定值

---

## 关键发现总结

### 1. 速度为0的主要原因

根据代码分析，`longitudinal_velocity_mps` 被设为0主要有以下几种情况：

#### a. 系统未就绪
- **位置**: `behavior_path_planner_node.cpp:716`
- **触发条件**: `is_ready` 标志为false
- **影响**: 整个输出路径所有点速度都为0
- **这是最常见的速度为0的根本原因**

#### b. Start Planner 停止路径
- **位置**: `start_planner_module.cpp:831`
- **触发条件**: 生成停止路径
- **影响**: start_planner输出的路径速度为0

#### c. Goal Planner 保持停止
- **位置**: `goal_planner_module.cpp:2175`
- **触发条件**: 需要保持停止状态
- **影响**: 目标规划器输出路径速度为0

#### d. 路径终点是目标点
- **位置**: 多个模块
- **触发条件**: `path_terminal_is_goal` 为true
- **影响**: 路径最后一点速度为0

### 2. 速度限制和修改场景

#### a. 避障减速
- 在障碍物前方区域根据横向加速度限制减速
- 使用jerk限制计算可行速度

#### b. 靠边停车减速
- 使用配置参数 `pull_over_velocity`
- 在靠边停车路径上限制最大速度

#### c. 换道速度调整
- 在换道准备和执行阶段限制速度
- 使用插值确保速度平滑过渡

#### d. 转向信号减速
- 根据距离和时间计算减速速度
- 在转向信号激活时应用

### 3. 速度继承和传递

速度信息的传递链：

```
Lanelet地图速度限制 (speed_limit)
    ↓
参考路径 (reference_path) longitudinal_velocity_mps
    ↓
各模块处理 (start/goal/avoidance/lane_change)
    ↓
最终输出路径 (path_with_lane_id)
```

每个模块都可能修改速度：
- 读取上游路径的速度
- 根据自身逻辑调整
- 输出修改后的速度

### 4. 调试日志位置

已添加的速度调试日志：
- `behavior_path_planner_node.cpp:407-412` - 输出路径速度
- `goal_planner_module.cpp:1452-1456` - Goal Planner速度修改
- `scene.cpp:2039-2046` - Avoidance速度修改

---

## 排查建议

当遇到速度为0的问题时，建议按以下顺序检查：

1. **检查 `is_ready` 标志**
   - 位置: `behavior_path_planner_node.cpp:714`
   - 确认系统是否准备就绪

2. **检查 Start Planner 状态**
   - 位置: `start_planner_module.cpp:831`
   - 确认是否生成了停止路径

3. **检查参考路径速度**
   - 查看 `reference_path` 话题
   - 确认地图速度限制是否正确加载

4. **检查各模块速度修改日志**
   - 启用DEBUG日志级别
   - 查看速度修改的详细信息

5. **检查路径终点标志**
   - 确认 `path_terminal_is_goal` 是否错误设置

---

## 相关配置参数

### Start Planner 参数
- `max_back_distance`: 最大后退距离
- `backward_path_length`: 后向路径长度

### Goal Planner 参数
- `pull_over_velocity`: 靠边停车速度
- `maximum_deceleration`: 最大减速度
- `maximum_jerk`: 最大加速度变化率

### Avoidance 参数
- `buf_slow_down_speed`: 减速缓冲速度
- `min_slow_down_speed`: 最小减速速度
- `max_acceleration`: 最大加速度
- `lateral_max_jerk_limit`: 横向最大jerk限制
- `lateral_max_accel_limit`: 横向最大加速度限制

---

## 总结

`longitudinal_velocity_mps` 在 behavior_path_planner 模块中的处理非常复杂，涉及多个子模块和多种场景。主要的处理逻辑包括：

1. **初始化和传递**: 从地图读取速度限制，通过参考路径传递
2. **安全限制**: 各模块根据安全需求限制速度
3. **场景适应**: 根据不同场景（起步、停车、避障、换道）调整速度
4. **强制停止**: 在特定条件下强制速度为0

理解这些代码的作用对于解决速度为0的问题至关重要。建议结合调试日志和配置参数，系统性地排查问题根源。

---

**文档生成时间**: 2025-10-24  
**代码路径**: `/home/huanyue/weicanming/github_programs/autoware/Autoware_Debug/src/universe/autoware_universe/planning/behavior_path_planner/`

