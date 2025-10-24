# 车辆状态对Planning速度的影响分析

## 📋 概述

**是的，车辆状态会对 Planning 模块中的速度值产生重要影响！** 车辆的当前速度、加速度和运动状态是规划模块计算目标速度的关键输入。

---

## 🚗 关键车辆状态话题

### 1. 主要车辆状态话题

根据代码分析，以下是 Planning 模块订阅的车辆状态相关话题：

#### 📍 **`/localization/kinematic_state` (最重要)**
**话题类型**: `nav_msgs::msg::Odometry`

这是**最核心**的车辆状态话题，包含：
- **位置** (position): x, y, z
- **姿态** (orientation): 四元数表示的航向
- **线速度** (linear velocity): vx, vy, vz
- **角速度** (angular velocity): 绕x, y, z轴的旋转速度

**在 BPP 中的使用**:
```cpp
// 文件: behavior_path_planner_node.cpp, 第182-188行
// velocity
{
  const auto msg = velocity_subscriber_.take_data();
  if (msg) {
    planner_data_->self_odometry = msg;  // 存储到规划数据中
  }
}
```

#### 📊 **`/localization/acceleration`**
**话题类型**: `geometry_msgs::msg::AccelWithCovarianceStamped`

包含车辆的加速度信息：
- **线加速度** (linear): ax, ay, az
- **角加速度** (angular): 绕x, y, z轴的旋转加速度
- **协方差**: 加速度的不确定性

**在 BPP 中的使用**:
```cpp
// 文件: behavior_path_planner_node.cpp, 第189-195行
// acceleration
{
  const auto msg = acceleration_subscriber_.take_data();
  if (msg) {
    planner_data_->self_acceleration = msg;
  }
}
```

---

## 💡 车辆状态如何影响规划速度

### 1. **当前速度作为速度规划的基准**

在 Goal Planner 中，当前速度用于设置最小速度：

```cpp
// 文件: goal_planner_module.cpp, 第1425-1438行
void GoalPlannerModule::decideVelocity(PullOverPath & pull_over_path)
{
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;
  
  // 使用当前速度和最小速度中的较大值
  const auto vel = static_cast<float>(
    std::max(current_vel, parameters_.pull_over_minimum_velocity)
  );
  
  for (auto & p : first_path.points) {
    // 限制路径点的速度不超过计算的目标速度
    p.point.longitudinal_velocity_mps = std::min(
      p.point.longitudinal_velocity_mps, vel
    );
  }
}
```

**影响**: 
- 如果车辆当前速度高于设定的最小速度，会保持当前速度
- 避免突然减速造成的不舒适感

---

### 2. **车辆停止判断**

通过 `odometry` 判断车辆是否已停止：

```cpp
// 文件: goal_planner_module.cpp, 第793-796行
isStopped(
  odometry_buffer_stopped_, 
  planner_data_->self_odometry, 
  parameters_.th_stopped_time,
  parameters_.th_stopped_velocity
)
```

**判断条件**:
- 速度 < `th_stopped_velocity` (停止速度阈值，通常 0.01-0.05 m/s)
- 持续时间 > `th_stopped_time` (停止时间阈值，通常 1-2 秒)

**影响**:
- 决定是否切换到停车模式
- 影响路径规划的决策状态

---

### 3. **避障速度计算**

在障碍物避让模块中，使用当前速度计算避障轨迹：

```cpp
// 文件: static_obstacle_avoidance_module.cpp, 第1996-2005行
// 根据当前速度和加速度计算目标速度
const double v_target_square = 
  v_max * v_max - 2.0 * parameters_->max_acceleration * accel_distance;

// 目标速度取当前速度和计算值中的较大值
const double v_target = std::max(getEgoSpeed(), std::sqrt(v_target_square));

// 应用到路径点
const double v_original = shifted_path.path.points.at(i).point.longitudinal_velocity_mps;
shifted_path.path.points.at(i).point.longitudinal_velocity_mps = 
  std::min(v_original, v_target);
```

**影响**:
- 避免速度突变
- 确保平滑减速
- 基于当前速度计算制动距离

---

### 4. **自车轨迹预测**

使用当前速度和加速度预测未来轨迹（用于安全检查）：

```cpp
// 文件: objects_filtering.cpp, 第303-320行
for (double t = 0.0; t < time_horizon; t += time_resolution) {
  double velocity = 0.0;
  double length = 0.0;
  
  if (t >= delay_until_departure) {
    double t_with_delay = t - delay_until_departure;
    // 基于当前速度和加速度预测未来速度
    velocity = std::clamp(
      current_velocity + acceleration * t_with_delay, 
      min_velocity, 
      max_velocity
    );
    // 计算预测位置
    length = current_velocity * t_with_delay + 
             0.5 * acceleration * t_with_delay * t_with_delay;
  }
  
  const auto pose = autoware::motion_utils::calcInterpolatedPose(
    path_points, vehicle_pose_frenet.length + length
  );
  predicted_path.emplace_back(t, pose, velocity);
}
```

**影响**:
- 碰撞检测
- 安全距离计算
- 跟随距离调整

---

## 📊 Planning 模块订阅的完整话题列表

### Behavior Path Planner (BPP)

| 话题名称 | 类型 | 用途 | 对速度的影响 |
|---------|------|------|-------------|
| `/localization/kinematic_state` | `nav_msgs::msg::Odometry` | **车辆速度和位置** | ⭐⭐⭐ 直接影响 |
| `/localization/acceleration` | `AccelWithCovarianceStamped` | **车辆加速度** | ⭐⭐⭐ 直接影响 |
| `/perception/object_recognition/objects` | `PredictedObjects` | 动态障碍物 | ⭐⭐ 间接影响（避障） |
| `/planning/mission_planning/route` | `LaneletRoute` | 路径规划 | ⭐⭐ 路径速度限制 |
| `/perception/occupancy_grid_map/map` | `OccupancyGrid` | 占据栅格地图 | ⭐ 间接影响（避障） |
| `/system/operation_mode/state` | `OperationModeState` | 运行模式 | ⭐ 模式切换 |
| `/planning/scenario_planning/scenario` | `Scenario` | 场景类型 | ⭐ 场景切换 |

### Obstacle Cruise Planner

| 话题名称 | 类型 | 用途 | 对速度的影响 |
|---------|------|------|-------------|
| `/localization/kinematic_state` | `Odometry` | **车辆速度** | ⭐⭐⭐ 巡航速度计算 |
| `/perception/object_recognition/objects` | `PredictedObjects` | 动态障碍物 | ⭐⭐⭐ 跟随速度 |
| `/planning/scenario_planning/trajectory` | `Trajectory` | 输入轨迹 | ⭐⭐ 速度修正 |

### Obstacle Stop Planner

| 话题名称 | 类型 | 用途 | 对速度的影响 |
|---------|------|------|-------------|
| `/localization/kinematic_state` | `Odometry` | **车辆速度** | ⭐⭐⭐ 停止判断 |
| `/localization/acceleration` | `AccelWithCovarianceStamped` | **车辆加速度** | ⭐⭐ 制动距离计算 |

---

## 🔍 如何查看车辆状态话题

### 1. 查看当前速度

```bash
# 方法1: 查看完整的 Odometry 消息
ros2 topic echo /localization/kinematic_state

# 方法2: 仅查看线速度（更简洁）
ros2 topic echo /localization/kinematic_state/twist/twist/linear

# 方法3: 实时监控速度值
ros2 topic echo /localization/kinematic_state | grep -A 3 "linear:"
```

**输出示例**:
```yaml
twist:
  twist:
    linear:
      x: 2.5    # 前进速度 (m/s)
      y: 0.0    # 横向速度
      z: 0.0    # 垂直速度
    angular:
      x: 0.0
      y: 0.0
      z: 0.1    # 转向角速度 (rad/s)
```

### 2. 查看加速度

```bash
# 查看加速度消息
ros2 topic echo /localization/acceleration

# 仅查看线加速度
ros2 topic echo /localization/acceleration/accel/accel/linear
```

**输出示例**:
```yaml
accel:
  accel:
    linear:
      x: 0.5    # 前进加速度 (m/s²)
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
```

### 3. 查看话题频率

```bash
# 查看 odometry 发布频率
ros2 topic hz /localization/kinematic_state

# 查看加速度发布频率
ros2 topic hz /localization/acceleration
```

**期望频率**:
- Odometry: 30-50 Hz
- Acceleration: 30-50 Hz

### 4. 绘制速度曲线（RViz）

在 RViz 中添加 `Odometry` 插件：
1. Add → By topic → `/localization/kinematic_state` → Odometry
2. 设置 Arrow Length 和 Arrow Color
3. 可以看到车辆的速度矢量可视化

### 5. 使用 `plotjuggler` 实时绘图

```bash
# 安装 plotjuggler（如果未安装）
sudo apt install ros-humble-plotjuggler-ros

# 启动 plotjuggler
ros2 run plotjuggler plotjuggler

# 在界面中订阅话题并绘制速度曲线
```

---

## 🛠️ 车辆状态对速度的具体影响场景

### 场景1: 车辆从静止启动

**车辆状态**:
- 当前速度: 0 m/s
- 目标速度: 1.5 m/s

**Planning 行为**:
1. BPP 检测到车辆静止 (`isStopped` 返回 true)
2. 根据最大加速度限制生成加速轨迹
3. 速度逐渐从 0 增加到目标速度

**关键参数**:
```yaml
max_acceleration: 0.5 m/s²    # 最大加速度
max_jerk: 0.5 m/s³            # 最大加加速度
```

---

### 场景2: 车辆接近障碍物

**车辆状态**:
- 当前速度: 1.2 m/s
- 检测到前方障碍物，距离 5m

**Planning 行为**:
1. Obstacle Cruise Planner 接收当前速度
2. 计算制动距离: `d = v² / (2 * a)` = 1.2² / (2 * 0.8) ≈ 0.9m
3. 由于距离足够 (5m > 0.9m)，逐渐减速
4. 速度从 1.2 m/s 平滑降低到巡航速度或停止

**代码逻辑**:
```cpp
// 基于当前速度计算目标速度，避免突然减速
const double v_target = std::max(getEgoSpeed(), calculated_target_speed);
```

---

### 场景3: 车辆高速行驶需要急停

**车辆状态**:
- 当前速度: 3.0 m/s
- 前方紧急障碍物，距离 3m

**Planning 行为**:
1. 计算所需制动距离: 3.0² / (2 * 0.8) ≈ 5.6m
2. 发现距离不足 (3m < 5.6m)
3. 应用最大减速度 (可能超过舒适性限制)
4. 发布紧急停止命令

**安全机制**:
- 如果规划模块来不及响应，`vehicle_cmd_gate` 会介入
- 触发紧急制动系统

---

### 场景4: 目标点在当前位置附近

**车辆状态**:
- 当前速度: 1.0 m/s
- 目标点距离: 2m

**Planning 行为**:
1. 检测到目标点很近
2. **考虑当前速度**，计算减速轨迹
3. 如果距离太短无法安全停止，可能输出速度为 0

**这就是您遇到的"近距离目标点速度为零"问题！**

参考您的文档 `近距离目标点规划问题分析.md`：
```markdown
当目标点距离 < 制动距离时，BPP 可能直接输出速度为 0
```

---

## 🔧 AGV 中的调优建议

### 1. 速度平滑参数

为了让规划速度更好地响应车辆状态，调整以下参数：

**文件**: `velocity_smoother.param.yaml`
```yaml
smoother_type: "JerkFiltered"

dynamics:
  max_velocity: 1.5          # 最大速度
  max_acceleration: 0.5      # 最大加速度（考虑当前速度）
  max_deceleration: 0.8      # 最大减速度
  max_jerk: 0.5              # 最大Jerk（平滑性）

JerkFiltered:
  jerk_filter_bandwidth: 1.0
  jerk_limit_scaling: 0.9    # Jerk限制缩放因子
```

### 2. 停止检测参数

**文件**: `goal_planner.param.yaml` 或 `behavior_path_planner.param.yaml`
```yaml
# 停止判断阈值
th_stopped_velocity: 0.05   # 停止速度阈值 (m/s)
                            # AGV建议: 0.01-0.05
th_stopped_time: 1.0        # 持续时间阈值 (秒)
                            # AGV建议: 0.5-2.0

# 避免频繁启停
pull_over_minimum_velocity: 0.3  # 最小前进速度
```

### 3. 基于当前速度的避障参数

**文件**: `obstacle_cruise_planner.param.yaml`
```yaml
cruise_mode:
  # 使用自适应策略，根据当前速度调整
  strategy: "adaptive"
  
  # 速度响应参数
  velocity_response_time: 1.0      # 速度响应时间
  time_headway: 1.5                # 时间车距（基于当前速度）
  
  # 当前速度低于此阈值时切换到跟随模式
  follow_speed_threshold: 0.2      # m/s
```

### 4. 避免速度跳变

**文件**: `behavior_velocity_planner.param.yaml`
```yaml
velocity_smoothing:
  enable: true
  
  # 速度变化率限制（避免因状态突变导致速度跳变）
  max_velocity_change_rate: 0.5    # m/s per planning cycle
  
  # 基于当前速度的缓冲区
  velocity_buffer_ratio: 1.1       # 当前速度的110%作为上限
```

---

## 📈 监控和调试工具

### 1. 创建速度监控脚本

```bash
#!/bin/bash
# 文件名: monitor_vehicle_speed.sh

echo "=== 车辆速度监控 ==="
echo ""

# 监控当前速度
echo "1. 当前车辆速度:"
ros2 topic echo --once /localization/kinematic_state/twist/twist/linear/x

echo ""

# 监控规划速度
echo "2. 规划目标速度:"
ros2 topic echo --once /planning/scenario_planning/trajectory | grep -A 1 "longitudinal_velocity_mps" | head -2

echo ""

# 监控控制命令速度
echo "3. 控制命令速度:"
ros2 topic echo --once /control/command/control_cmd/longitudinal/velocity

echo ""

# 监控速度限制
echo "4. 当前速度限制:"
ros2 topic echo --once /planning/scenario_planning/max_velocity_default
```

**使用方法**:
```bash
chmod +x monitor_vehicle_speed.sh
./monitor_vehicle_speed.sh
```

### 2. 实时对比脚本

```bash
#!/bin/bash
# 文件名: compare_velocities.sh

echo "持续监控速度差异 (Ctrl+C 退出)"
echo "=================================================="
echo ""

while true; do
  # 获取当前速度
  current_vel=$(ros2 topic echo --once /localization/kinematic_state | grep "x:" | head -1 | awk '{print $2}')
  
  # 获取规划速度
  planned_vel=$(ros2 topic echo --once /planning/scenario_planning/trajectory | grep "longitudinal_velocity_mps" | head -1 | awk '{print $2}')
  
  # 计算差异
  echo "时间: $(date +%H:%M:%S)"
  echo "  当前速度: ${current_vel} m/s"
  echo "  规划速度: ${planned_vel} m/s"
  echo "  差值: $(echo "$planned_vel - $current_vel" | bc) m/s"
  echo "---"
  
  sleep 1
done
```

---

## 🎯 总结

### 关键要点

1. ✅ **车辆状态对 Planning 速度有直接影响**
   - 当前速度用于计算目标速度的基准
   - 加速度用于预测未来轨迹和制动距离
   - 停止状态影响路径规划决策

2. ✅ **核心话题**
   - `/localization/kinematic_state` - 最重要的速度和位置信息
   - `/localization/acceleration` - 加速度信息
   - 两者都是 Planning 模块的必需输入

3. ✅ **影响机制**
   - 速度平滑: 避免突然加减速
   - 安全检查: 基于当前状态预测碰撞
   - 避障决策: 根据当前速度调整避障策略
   - 停止判断: 决定是否进入停车模式

4. ✅ **AGV 优化建议**
   - 降低停止速度阈值 (0.01-0.05 m/s)
   - 缩短停止时间阈值 (0.5-1.0 秒)
   - 启用速度平滑和 Jerk 限制
   - 使用自适应巡航策略

### 调试流程

当遇到速度问题时：
1. 检查 `/localization/kinematic_state` 是否正常发布
2. 确认当前速度值是否准确
3. 对比当前速度和规划速度
4. 检查是否触发停止判断
5. 查看加速度数据是否合理

**祝您调试顺利！** 🚀

