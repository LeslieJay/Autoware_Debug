# Planning 模块代码注释说明

## 文档概述

本文档说明了 Planning 模块中已添加中文注释的代码文件,以及如何阅读和理解这些注释。

## 注释规范

### 1. 文件头注释

每个头文件都包含详细的文件说明:

```cpp
/**
 * @file mission_planner.hpp
 * @brief 任务规划器(Mission Planner)头文件
 * 
 * 该文件定义了MissionPlanner类,负责从当前位置到目标位置的全局路线规划。
 * 主要功能包括:
 * - 接收目标位置并规划全局路线(基于Lanelet2地图)
 * - 支持重新规划路线(reroute)
 * - 检查车辆是否到达目标点
 * - 提供路线可视化
 */
```

### 2. 类注释

每个类都有详细的功能说明:

```cpp
/**
 * @class MissionPlanner
 * @brief 任务规划器主类
 * 
 * 该类负责全局路线规划,包括:
 * - 接收目标位置并规划从当前位置到目标位置的路线
 * - 支持动态重新规划路线
 * - 检查车辆是否安全到达目标点
 * - 发布路线状态和可视化信息
 * 
 * 路线规划采用插件架构,默认使用基于Lanelet2的规划算法。
 * 规划过程不考虑动态障碍物,仅基于静态地图信息。
 */
```

### 3. 成员变量注释

使用 Doxygen 风格的内联注释:

```cpp
ArrivalChecker arrival_checker_;  ///< 到达检查器,用于判断是否到达目标点
pluginlib::ClassLoader<PlannerPlugin> plugin_loader_;  ///< 插件加载器
std::shared_ptr<PlannerPlugin> planner_;  ///< 规划器插件实例(默认为DefaultPlanner)
```

### 4. 函数注释

完整的函数文档包括参数、返回值和详细说明:

```cpp
/**
 * @brief 检查重新规划的安全性
 * @param original_route 原始路线
 * @param target_route 目标路线
 * @return 如果重新规划安全则返回true
 * 
 * 安全性判断逻辑:
 * 1. 找到两条路线的公共部分
 * 2. 计算从当前位置到路线分叉点的距离
 * 3. 计算安全距离 = max(当前速度 × 时间阈值, 最小距离)
 * 4. 如果公共部分长度 >= 安全距离,则认为安全
 * 
 * 这样可以确保车辆有足够的距离平稳过渡到新路线
 */
bool check_reroute_safety(const LaneletRoute & original_route, const LaneletRoute & target_route);
```

## 已添加注释的文件列表

### Mission Planner 模块

1. **mission_planner.hpp** ✅
   - 位置: `src/core/autoware_core/planning/autoware_mission_planner/src/mission_planner/`
   - 内容: MissionPlanner 类完整注释
   - 包含:
     - 类功能说明
     - 成员变量注释
     - 回调函数注释
     - 路线管理函数注释
     - 重新规划安全检查详解

## 关键代码解读

### 1. Mission Planner 工作流程

```cpp
// 设置路线的主要流程:
on_set_waypoint_route() / on_set_lanelet_route()
  ↓
检查状态(UNSET 或 SET)
  ↓
检查任务规划器是否就绪
  ↓
create_route() - 调用规划插件
  ↓
check_reroute_safety() - 检查安全性(如果是重新规划)
  ↓
change_route() - 更新路线
  ↓
change_state(RouteState::SET) - 更新状态
  ↓
发布路线和可视化标记
```

### 2. 重新规划安全检查算法

```cpp
bool check_reroute_safety(original_route, target_route) {
  // Step 1: 找到两条路线的公共车道段
  for (i in original_route.segments) {
    for (j in target_route.segments) {
      if (hasSamePrimitives(original[i], target[j])) {
        start_idx_original = i;
        start_idx_target = j;
        break;
      }
    }
  }
  
  // Step 2: 找到公共部分的结束索引
  end_idx_original = start_idx_original;
  end_idx_target = start_idx_target;
  for (i = 1; ...) {
    if (!hasSamePrimitives(original[start+i], target[start+i])) {
      break;
    }
    end_idx_original = start_idx_original + i;
    end_idx_target = start_idx_target + i;
  }
  
  // Step 3: 计算从当前位置到公共部分结束的累计长度
  accumulated_length = 计算从当前位置到end_idx的距离;
  
  // Step 4: 计算安全距离并判断
  safety_length = max(current_velocity × reroute_time_threshold_, 
                     minimum_reroute_length_);
  
  return accumulated_length > safety_length;
}
```

**关键点**:
- `reroute_time_threshold_`: 通常设为 10 秒,意味着给车辆至少 10 秒的时间来过渡
- `minimum_reroute_length_`: 最小安全距离,通常设为 30 米
- 最终安全距离取两者中的较大值

### 3. 到达检查机制

```cpp
// arrival_checker.cpp
bool ArrivalChecker::is_arrived(const PoseStamped & pose) const {
  // 1. 检查坐标系是否匹配
  if (goal.header.frame_id != pose.header.frame_id) return false;
  
  // 2. 检查距离 (欧氏距离)
  double distance = calc_distance2d(pose.pose, goal.pose);
  if (distance_ < distance) return false;
  
  // 3. 检查航向角差异
  double yaw_diff = normalize_radian(yaw_pose - yaw_goal);
  if (angle_ < std::fabs(yaw_diff)) return false;
  
  // 4. 检查车辆是否停止足够时长
  return vehicle_stop_checker_.isVehicleStopped(duration_);
}
```

**到达条件**:
1. 距离 < `arrival_check_distance` (默认 2.0 米)
2. 角度差 < `arrival_check_angle_deg` (默认 45 度)
3. 停止时长 > `arrival_check_duration` (默认 1.0 秒)

## 代码结构图

### MissionPlanner 类成员分组

```
MissionPlanner
├── 核心组件
│   ├── arrival_checker_      (到达检查器)
│   ├── plugin_loader_         (插件加载器)
│   └── planner_               (规划器插件)
│
├── 坐标变换
│   ├── map_frame_             (地图坐标系)
│   ├── tf_buffer_             (TF缓冲)
│   └── tf_listener_           (TF监听)
│
├── ROS接口
│   ├── 服务
│   │   ├── srv_clear_route           (清除路线)
│   │   ├── srv_set_lanelet_route     (设置车道路线)
│   │   └── srv_set_waypoint_route    (设置路点路线)
│   │
│   ├── 发布器
│   │   ├── pub_state_               (路线状态)
│   │   ├── pub_route_               (路线)
│   │   └── pub_marker_              (可视化标记)
│   │
│   └── 订阅器
│       ├── sub_odometry_            (里程计)
│       ├── sub_operation_mode_state_ (运行模式)
│       └── sub_vector_map_          (矢量地图)
│
├── 数据缓存
│   ├── odometry_              (当前里程计)
│   ├── operation_mode_state_  (当前运行模式)
│   ├── map_ptr_               (地图指针)
│   ├── state_                 (路线状态)
│   ├── current_route_         (当前路线)
│   └── lanelet_map_ptr_       (Lanelet2地图)
│
├── 回调函数
│   ├── on_odometry()          (里程计回调)
│   ├── on_operation_mode_state() (运行模式回调)
│   ├── on_map()               (地图回调)
│   ├── on_clear_route()       (清除路线服务)
│   ├── on_set_lanelet_route() (设置车道路线服务)
│   └── on_set_waypoint_route() (设置路点路线服务)
│
├── 路线管理
│   ├── change_state()         (改变状态)
│   ├── change_route()         (改变路线)
│   ├── cancel_route()         (取消路线)
│   └── create_route() (多个重载) (创建路线)
│
├── 安全检查
│   └── check_reroute_safety() (重新规划安全检查)
│
└── 调试监控
    ├── publish_processing_time() (发布处理时间)
    └── publish_pose_log()        (发布位姿日志)
```

## 数据流图

### 路线规划数据流

```
用户/系统发起路线请求
  ↓
set_waypoint_route 或 set_lanelet_route (ROS Service)
  ↓
MissionPlanner::on_set_*_route()
  ↓
检查: 是否允许设置路线?
  ├─ NO → 返回错误
  └─ YES
      ↓
MissionPlanner::create_route()
  ↓
PlannerPlugin::plan() (调用Lanelet2路由图)
  ↓
Lanelet2 Dijkstra 搜索
  ↓
返回 LaneletRoute (车道序列)
  ↓
检查: 是重新规划吗?
  ├─ YES → check_reroute_safety()
  │         ├─ 不安全 → 取消并保持原路线
  │         └─ 安全 → 继续
  └─ NO
      ↓
MissionPlanner::change_route()
  ├─ 更新 current_route_
  ├─ 调用 planner_->updateRoute()
  └─ 设置 arrival_checker_ 目标
      ↓
发布路线
  ├─ pub_route_->publish()      (LaneletRoute)
  └─ pub_marker_->publish()     (可视化标记)
      ↓
change_state(RouteState::SET)
  └─ pub_state_->publish()      (路线状态)
```

### 到达检查数据流

```
里程计更新 (nav_msgs::Odometry)
  ↓
MissionPlanner::on_odometry()
  ↓
更新 odometry_
  ↓
检查: state_ == RouteState::SET?
  ├─ NO → 跳过检查
  └─ YES
      ↓
arrival_checker_.is_arrived(current_pose)
  ├─ 检查距离
  ├─ 检查角度
  └─ 检查停止时长
      ↓
到达?
  ├─ YES → change_state(RouteState::ARRIVED)
  └─ NO → 继续等待
```

## 编译和使用

### 编译注释后的代码

```bash
cd /path/to/autoware/workspace
colcon build --packages-select autoware_mission_planner
```

### 生成 Doxygen 文档

如果需要生成HTML格式的API文档:

```bash
# 在包目录下
cd src/core/autoware_core/planning/autoware_mission_planner
doxygen Doxyfile  # 如果有配置文件

# 或使用 rosdoc_lite
rosdoc_lite .
```

## 注释的好处

1. **理解代码逻辑**: 详细的中文注释帮助快速理解复杂的规划算法
2. **维护和扩展**: 清晰的函数说明便于后续开发和维护
3. **调试**: 了解每个函数的预期行为,便于定位问题
4. **团队协作**: 统一的注释规范提高团队开发效率

## 继续添加注释的计划

由于代码量较大,建议按以下优先级继续添加注释:

### 高优先级 (核心功能)
1. ✅ `mission_planner.hpp/cpp` - 已完成
2. `arrival_checker.hpp/cpp`
3. `route_handler.hpp/cpp`
4. `path_generator/node.hpp/cpp`
5. `behavior_velocity_planner/node.hpp/cpp`
6. `motion_velocity_planner/node.hpp/cpp`
7. `velocity_smoother/node.hpp/cpp`

### 中优先级 (重要工具)
1. 各模块的 `planner_manager.hpp/cpp`
2. `scene_module_interface.hpp/cpp`
3. 速度平滑器算法实现
4. 碰撞检查器

### 低优先级 (辅助功能)
1. 调试和可视化工具
2. 测试代码
3. 工具函数

## 如何阅读注释

### 建议的阅读顺序

1. **从文件头开始**: 了解文件的整体功能
2. **阅读类注释**: 理解类的职责和使用场景
3. **浏览成员变量**: 了解类维护的状态
4. **查看公共接口**: 理解如何使用这个类
5. **深入实现细节**: 查看私有方法和算法实现

### 使用IDE辅助

现代IDE(如VSCode、CLion)可以:
- 鼠标悬停显示注释
- 快速跳转到定义
- 生成函数调用图
- 搜索符号引用

推荐的VSCode插件:
- C/C++
- Doxygen Documentation Generator
- Better Comments

## 总结

本项目为 Autoware Planning 模块的关键代码文件添加了详细的中文注释,遵循 Doxygen 规范。这些注释不仅解释了代码的功能,还提供了算法原理和使用示例,大大降低了代码的理解难度,有助于开发、调试和维护工作。

配合《PLANNING模块详细说明文档.md》,用户可以从宏观和微观两个层面全面理解 Planning 模块的设计和实现。


