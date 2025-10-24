# PullOutPath 详解

## 概述

`PullOutPath` 是 StartPlanner 模块中的核心数据结构，用于表示车辆从静止状态（路边、停车位等）起步的完整路径信息。它封装了起步过程中所需的所有路径规划数据。

---

## 1. 数据结构定义

### 1.1 完整定义

**文件位置**: `autoware_behavior_path_start_planner_module/include/autoware/behavior_path_start_planner_module/pull_out_path.hpp`

```cpp
struct PullOutPath
{
  // 分段路径数组 - 起步路径可能包含多个阶段
  std::vector<PathWithLaneId> partial_paths{};
  
  // 终端速度和加速度对 - 每个分段对应一个速度/加速度配置
  // 用于以恒定加速度加速到目标速度
  std::vector<std::pair<double, double>> pairs_terminal_velocity_and_accel{};
  
  // 起步起点位姿
  Pose start_pose{};
  
  // 起步终点位姿
  Pose end_pose{};
};
```

### 1.2 各字段详细说明

#### a) `partial_paths` - 分段路径数组

**类型**: `std::vector<PathWithLaneId>`

**含义**: 
- 存储起步路径的多个分段
- 每个分段是一个完整的 `PathWithLaneId`，包含多个路径点
- 分段是按顺序执行的

**为什么需要分段？**

起步过程可能包含多个不同的运动阶段，例如：

1. **简单路边起步**（Shift Pull Out）
   - 通常只有 **1个分段**：横向位移 + 前进

2. **几何起步**（Geometric Pull Out）
   - 可能有 **2个分段**：
     - 第1段：弧线路径（转弯出库）
     - 第2段：直线路径（并入车道）

3. **需要后退的起步**（复杂场景）
   - 可能有 **3个或更多分段**：
     - 第1段：后退
     - 第2段：前进转向
     - 第3段：并入车道

4. **自由空间起步**（Freespace Pull Out）
   - 根据障碍物环境可能有 **多个分段**
   - 包含前进和后退的组合

**示例**:
```cpp
// 简单起步 - 1个分段
PullOutPath simple_path;
simple_path.partial_paths.push_back(shifted_path);  // 只有1个

// 几何起步 - 2个分段（如果配置了 divide_pull_out_path）
PullOutPath geometric_path;
geometric_path.partial_paths.push_back(arc_path_1);   // 第1段弧线
geometric_path.partial_paths.push_back(arc_path_2);   // 第2段弧线
```

#### b) `pairs_terminal_velocity_and_accel` - 速度和加速度对

**类型**: `std::vector<std::pair<double, double>>`

**含义**: 
- 每个分段对应一对终端速度和加速度值
- `pair.first`: 该分段的目标终端速度 [m/s]
- `pair.second`: 达到该速度所需的加速度 [m/s²]

**用途**:
- 为每个分段设置合适的速度规划
- 确保平滑加速到目标速度
- 在分段交接处保持速度连续性

**示例**:
```cpp
// Shift Pull Out 示例
candidate_path.pairs_terminal_velocity_and_accel.push_back(
  std::make_pair(terminal_velocity, longitudinal_acc));
// terminal_velocity: 例如 3.0 m/s
// longitudinal_acc: 例如 0.5 m/s²

// Geometric Pull Out 示例（2个分段）
// 第1段：加速到平均速度
output.pairs_terminal_velocity_and_accel.push_back(
  std::make_pair(average_velocity, average_acceleration));

// 第2段：达到目标速度
output.pairs_terminal_velocity_and_accel.push_back(
  std::make_pair(velocity, required_acceleration));
```

#### c) `start_pose` - 起点位姿

**类型**: `Pose`（包含 position 和 orientation）

**含义**: 起步路径的起始位姿

**用途**:
- 标记起步开始的位置
- 用于碰撞检查的起点
- 用于计算起步距离
- 用于可视化调试

**设置示例**:
```cpp
// Shift Pull Out
candidate_path.start_pose = shift_line.start;

// Geometric Pull Out
output.start_pose = planner_.getArcPaths().at(0).points.front().point.pose;
```

#### d) `end_pose` - 终点位姿

**类型**: `Pose`

**含义**: 起步路径的结束位姿

**用途**:
- 标记起步完成的位置
- 判断是否完成起步过程
- 用于与下一段路径的连接
- 用于RTC（Request To Cooperate）状态更新

**设置示例**:
```cpp
// Shift Pull Out
candidate_path.end_pose = shift_line.end;

// Geometric Pull Out
output.end_pose = planner_.getArcPaths().at(1).points.back().point.pose;
```

---

## 2. PullOutPath 的生成过程

### 2.1 生成流程图

```
StartPlannerModule::updateData()
    ↓
planWithPriority()
    ↓
遍历规划器优先级（Shift/Geometric/Freespace）
    ↓
findPullOutPath()
    ↓
调用具体规划器的 plan() 方法
    ↓
  ┌──────────────┬───────────────┬─────────────────┐
  ↓              ↓               ↓                 ↓
ShiftPullOut  GeometricPullOut  FreespacePullOut  ...
    ↓              ↓               ↓
生成 PullOutPath 结构体
    ↓
返回到 StartPlannerModule
    ↓
更新 status_.pull_out_path
```

### 2.2 各规划器生成方式

#### A. Shift Pull Out（位移起步）

**特点**: 
- 简单的横向位移 + 前进
- 通常只有 **1个分段**
- 适用于路边起步

**生成代码**（`shift_pull_out.cpp:460-466`）:
```cpp
// 添加位移后的路径到候选路径
PullOutPath candidate_path;
candidate_path.partial_paths.push_back(shifted_path.path);  // 只有1个分段
candidate_path.start_pose = shift_line.start;
candidate_path.end_pose = shift_line.end;
candidate_path.pairs_terminal_velocity_and_accel.push_back(
  std::make_pair(terminal_velocity, longitudinal_acc));
candidate_paths.push_back(candidate_path);
```

**路径特点**:
- `partial_paths.size() = 1`
- 路径包含横向位移和纵向前进
- 速度从0逐渐加速到终端速度

#### B. Geometric Pull Out（几何起步）

**特点**: 
- 使用弧线路径规划
- 可以有 **1个或2个分段**（取决于 `divide_pull_out_path` 参数）
- 适用于停车位出库

**生成代码**（`geometric_pull_out.cpp:86-126`）:

**情况1: 分段模式**（`divide_pull_out_path = true`）
```cpp
if (parameters_.divide_pull_out_path) {
  // 获取2段弧线路径
  output.partial_paths = planner_.getPaths();  // 2个分段
  
  // 第1段：插入停止速度到第一段弧线末端
  output.partial_paths.front().points.back().point.longitudinal_velocity_mps = 0.0;
  
  // 计算第1段的加速度
  const double arc_length_on_first_arc_path =
    autoware::motion_utils::calcArcLength(output.partial_paths.front().points);
  const double time_to_center = arc_length_on_first_arc_path / (2 * velocity);
  const double average_velocity = arc_length_on_first_arc_path / (time_to_center * 2);
  const double average_acceleration = average_velocity / (time_to_center * 2);
  output.pairs_terminal_velocity_and_accel.push_back(
    std::make_pair(average_velocity, average_acceleration));
  
  // 计算第2段的加速度
  const double arc_length_on_second_arc_path =
    autoware::motion_utils::calcArcLength(planner_.getArcPaths().at(1).points);
  output.pairs_terminal_velocity_and_accel.push_back(
    std::make_pair(velocity, velocity * velocity / (2 * arc_length_on_second_arc_path)));
}
```

**路径示意**:
```
分段1：第1个弧线（转弯开始）
    起点速度: 0 m/s
    终点速度: 0 m/s（停止）
    加速度: average_acceleration

分段2：第2个弧线（并入车道）
    起点速度: 0 m/s
    终点速度: target_velocity
    加速度: calculated_acceleration
```

**情况2: 组合模式**（`divide_pull_out_path = false`）
```cpp
else {
  // 将2段弧线路径合并为1个
  const auto partial_paths = planner_.getPaths();
  const auto combined_path = utils::combinePath(partial_paths.at(0), partial_paths.at(1));
  output.partial_paths.push_back(combined_path);  // 只有1个分段
  
  // 计算整体加速度
  const double arc_length_on_path = autoware::motion_utils::calcArcLength(combined_path.points);
  output.pairs_terminal_velocity_and_accel.push_back(
    std::make_pair(velocity, velocity * velocity / 2 * arc_length_on_path));
}
```

**设置起止位姿**:
```cpp
output.start_pose = planner_.getArcPaths().at(0).points.front().point.pose;
output.end_pose = planner_.getArcPaths().at(1).points.back().point.pose;
```

#### C. Freespace Pull Out（自由空间起步）

**特点**: 
- 使用RRT*等算法在自由空间规划
- 可能有 **多个分段**（包含前进和后退）
- 适用于复杂拥挤环境

**生成代码**（`freespace_pull_out.cpp:76-95`）:
```cpp
// 将路径点转换为带车道ID的路径
const PathWithLaneId path =
  utils::convertWayPointsToPathWithLaneId(planner_->getWaypoints(), velocity_, lanes);

// 获取需要倒车的索引
const auto reverse_indices = utils::getReversingIndices(path);

// 根据倒车索引分割路径
std::vector<PathWithLaneId> partial_paths = utils::dividePath(path, reverse_indices);

// 移除靠近终点的点
PathWithLaneId & last_path = partial_paths.back();
const double th_end_distance = 1.0;
for (auto it = last_path.points.begin(); it != last_path.points.end(); ++it) {
  const size_t index = std::distance(last_path.points.begin(), it);
  if (index == 0) continue;
  const double distance =
    autoware_utils::calc_distance2d(end_pose.position, it->point.pose.position);
  if (distance < th_end_distance) {
    last_path.points.erase(it, last_path.points.end());
    break;
  }
}
```

**路径特点**:
- `partial_paths.size()` 可能为 2, 3, 4... 等
- 包含前进段和后退段的组合
- 每次方向改变会产生一个新的分段

---

## 3. PullOutPath 的使用

### 3.1 存储在状态中

```cpp
struct PullOutStatus
{
  PullOutPath pull_out_path{};     // ⭐ 存储在这里
  size_t current_path_idx{0};      // 当前执行的分段索引
  PlannerType planner_type{PlannerType::NONE};
  PathWithLaneId backward_path{};
  bool found_pull_out_path{false};
  // ...
};
```

**更新时机**（`start_planner_module.cpp:1004-1014`）:
```cpp
void StartPlannerModule::updateStatusWithCurrentPath(
  const PullOutPath & path, const Pose & start_pose,
  const PlannerType & planner_type)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  status_.driving_forward = true;
  status_.found_pull_out_path = true;
  status_.pull_out_path = path;           // ⭐ 存储生成的路径
  status_.pull_out_start_pose = start_pose;
  status_.planner_type = planner_type;
}
```

### 3.2 获取当前分段路径

```cpp
PathWithLaneId StartPlannerModule::getCurrentPath() const
{
  // 检查索引有效性
  if (status_.pull_out_path.partial_paths.size() <= status_.current_path_idx) {
    return PathWithLaneId{};
  }
  
  // 返回当前分段
  return status_.pull_out_path.partial_paths.at(status_.current_path_idx);
}
```

### 3.3 递增路径索引

当完成当前分段时，切换到下一个分段：

```cpp
void StartPlannerModule::incrementPathIndex()
{
  // 递增索引，但不超过最大值
  status_.current_path_idx =
    std::min(status_.current_path_idx + 1, 
             status_.pull_out_path.partial_paths.size() - 1);
}
```

**调用时机**（`start_planner_module.cpp:708-711`）:
```cpp
// 如果当前路径已完成，递增路径索引
if (hasFinishedCurrentPath()) {
  RCLCPP_INFO(getLogger(), "Increment path index");
  incrementPathIndex();
}
```

### 3.4 判断是否完成当前分段

```cpp
bool StartPlannerModule::hasFinishedCurrentPath()
{
  const auto current_path = getCurrentPath();
  if (current_path.points.empty()) return false;
  
  // 获取当前位置
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  
  // 计算到路径终点的距离
  const auto & goal_pose = current_path.points.back().point.pose;
  const double distance = autoware_utils::calc_distance2d(
    current_pose.position, goal_pose.position);
  
  // 如果距离小于阈值，认为已完成
  return distance < parameters_->th_arrived_distance;
}
```

### 3.5 在 plan() 方法中的使用

```cpp
BehaviorModuleOutput StartPlannerModule::plan()
{
  // 检查是否找到起步路径
  if (!status_.found_pull_out_path) {
    return generateStopOutput();  // 未找到，返回停止路径
  }

  // 选择合适的路径
  const auto path = std::invoke([&]() {
    // 情况1: 后退阶段
    if (!status_.driving_forward && !status_.backward_driving_complete) {
      return status_.backward_path;
    }

    // 情况2: 递增路径索引（切换到下一分段）
    if (hasFinishedCurrentPath()) {
      RCLCPP_INFO(getLogger(), "Increment path index");
      incrementPathIndex();
    }

    // 情况3: 返回当前分段路径
    return getCurrentPath();  // ⭐ 使用 pull_out_path
  });

  // 构建输出
  BehaviorModuleOutput output;
  output.path = path;
  // ...
  
  return output;
}
```

---

## 4. 实际应用示例

### 示例1: 简单路边起步（1个分段）

**场景**: 车辆停在路边，需要并入车道

```cpp
PullOutPath simple_path;

// 只有1个分段
simple_path.partial_paths.push_back(shifted_lane_path);

// 速度配置：加速到 5 m/s，加速度 0.5 m/s²
simple_path.pairs_terminal_velocity_and_accel.push_back(
  std::make_pair(5.0, 0.5));

// 起止位姿
simple_path.start_pose = current_pose;
simple_path.end_pose = lane_center_pose;
```

**执行过程**:
1. 开始执行 `partial_paths[0]`
2. 横向位移 + 纵向加速
3. 到达终点，起步完成

### 示例2: 停车位出库（2个分段）

**场景**: 车辆在停车位中，需要通过弧线路径出库

```cpp
PullOutPath parking_path;

// 2个分段
parking_path.partial_paths.push_back(first_arc_path);   // 第1段弧线
parking_path.partial_paths.push_back(second_arc_path);  // 第2段弧线

// 第1段：加速到平均速度，然后停止
parking_path.pairs_terminal_velocity_and_accel.push_back(
  std::make_pair(1.5, 0.3));

// 第2段：加速到目标速度
parking_path.pairs_terminal_velocity_and_accel.push_back(
  std::make_pair(3.0, 0.5));

// 起止位姿
parking_path.start_pose = parking_spot_pose;
parking_path.end_pose = lane_entry_pose;
```

**执行过程**:
1. `current_path_idx = 0`，执行 `partial_paths[0]`（第1段弧线）
2. 完成第1段，调用 `incrementPathIndex()`
3. `current_path_idx = 1`，执行 `partial_paths[1]`（第2段弧线）
4. 完成第2段，起步完成

### 示例3: 复杂环境起步（多个分段，包含后退）

**场景**: 拥挤环境，需要先后退再前进

```cpp
PullOutPath complex_path;

// 3个分段
complex_path.partial_paths.push_back(backward_path);      // 第1段：后退
complex_path.partial_paths.push_back(turn_forward_path);  // 第2段：转向前进
complex_path.partial_paths.push_back(merge_path);         // 第3段：并入车道

// 各段速度配置
complex_path.pairs_terminal_velocity_and_accel.push_back(
  std::make_pair(-1.0, -0.2));  // 后退，负速度
complex_path.pairs_terminal_velocity_and_accel.push_back(
  std::make_pair(2.0, 0.3));    // 低速前进
complex_path.pairs_terminal_velocity_and_accel.push_back(
  std::make_pair(5.0, 0.5));    // 加速到正常速度

// 起止位姿
complex_path.start_pose = tight_spot_pose;
complex_path.end_pose = lane_center_pose;
```

**执行过程**:
1. `current_path_idx = 0`，执行后退路径
2. 完成后退，递增索引
3. `current_path_idx = 1`，执行转向前进路径
4. 完成转向，递增索引
5. `current_path_idx = 2`，执行并入车道路径
6. 完成并入，起步完成

---

## 5. 分段切换机制

### 5.1 切换流程

```
执行 partial_paths[0]
    ↓
hasFinishedCurrentPath() == true?
    ↓ (是)
incrementPathIndex()
    ↓
current_path_idx = 1
    ↓
执行 partial_paths[1]
    ↓
hasFinishedCurrentPath() == true?
    ↓ (是)
incrementPathIndex()
    ↓
current_path_idx = 2
    ↓
... 继续直到最后一个分段
```

### 5.2 完成判定

**位置判定**:
```cpp
// 计算到当前分段终点的距离
const double distance = calc_distance_to_segment_end();

// 如果距离小于阈值，认为已完成
if (distance < parameters_->th_arrived_distance) {
  return true;  // 已完成当前分段
}
```

**速度判定**（某些情况）:
```cpp
// 如果需要停止，检查速度是否接近0
if (need_stop && current_velocity < 0.1) {
  return true;  // 已停止，分段完成
}
```

---

## 6. 关键特性总结

### 6.1 灵活性

- **支持多种起步场景**：简单起步、停车位出库、复杂环境起步
- **可变分段数量**：1个、2个或多个分段
- **支持前进和后退**：通过多个分段组合实现

### 6.2 平滑性

- **速度规划**：每个分段都有明确的终端速度和加速度
- **连续性**：分段之间速度平滑过渡
- **加速度限制**：考虑车辆动力学约束

### 6.3 安全性

- **碰撞检查**：生成路径后进行碰撞检测
- **动态调整**：可以在执行过程中插入停止点
- **分段执行**：可以在任何分段停止或重新规划

---

## 7. 与速度为0的关系

### 问题场景

当 `status_.found_pull_out_path = false` 时：

```cpp
if (!status_.found_pull_out_path) {
  RCLCPP_WARN_THROTTLE(
    getLogger(), *clock_, 5000, 
    "Not found safe pull out path, publish stop path");
  const auto output = generateStopOutput();  // ⭐ 生成停止路径
  // 停止路径中所有点的速度都是 0.0
  return output;
}
```

### 原因分析

1. **未找到安全的 PullOutPath**
   - 所有规划器都未能生成可行路径
   - 碰撞检查失败
   - 几何约束不满足

2. **结果**
   - `status_.pull_out_path` 为空或无效
   - `getCurrentPath()` 返回空路径
   - 生成停止路径，速度全部为0

### 解决方法

- 检查规划器是否正确启用
- 调整碰撞检查参数
- 检查起步位姿是否合理
- 查看规划器评估表（planner_evaluation_table）

---

## 8. 调试建议

### 8.1 检查 PullOutPath 是否生成

```bash
# 查看日志
ros2 run rqt_console rqt_console

# 搜索关键字
- "Not found safe pull out path"
- "found_pull_out_path"
- "Increment path index"
```

### 8.2 可视化分段路径

在 RViz 中添加标记：
- 每个分段使用不同颜色
- 显示起止位姿
- 显示当前执行的分段索引

### 8.3 打印调试信息

```cpp
RCLCPP_INFO(getLogger(), 
  "PullOutPath: segments=%zu, current_idx=%zu, planner=%s",
  status_.pull_out_path.partial_paths.size(),
  status_.current_path_idx,
  magic_enum::enum_name(status_.planner_type).data());

for (size_t i = 0; i < status_.pull_out_path.pairs_terminal_velocity_and_accel.size(); ++i) {
  const auto & pair = status_.pull_out_path.pairs_terminal_velocity_and_accel[i];
  RCLCPP_INFO(getLogger(), 
    "Segment[%zu]: target_vel=%.2f m/s, accel=%.2f m/s²",
    i, pair.first, pair.second);
}
```

---

## 9. 总结

### 关键要点

1. **PullOutPath 是起步路径的容器**
   - 包含1个或多个分段路径
   - 每个分段有独立的速度和加速度配置
   - 记录起止位姿

2. **分段设计支持复杂起步**
   - 简单场景：1个分段
   - 停车位出库：2个分段
   - 复杂环境：多个分段（含后退）

3. **分段按顺序执行**
   - 通过 `current_path_idx` 跟踪进度
   - 完成一个分段后自动切换到下一个
   - 支持动态停止和重新规划

4. **速度规划精细**
   - 每个分段独立的速度目标
   - 考虑加速度限制
   - 确保平滑过渡

5. **与速度为0的关系**
   - 如果未生成有效的 PullOutPath，输出停止路径
   - 停止路径中所有点速度为0

---

**相关文件**:
- 定义: `pull_out_path.hpp`
- 使用: `start_planner_module.cpp`
- Shift规划器: `shift_pull_out.cpp`
- Geometric规划器: `geometric_pull_out.cpp`
- Freespace规划器: `freespace_pull_out.cpp`

**文档生成时间**: 2025-10-24

