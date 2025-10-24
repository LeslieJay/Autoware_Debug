# Behavior Path Planner 节点输出路径速度为0的原因分析

## 问题描述

`behavior_path_planner` 节点输出的话题 `/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` 中，路径点的 `longitudinal_velocity_mps` 值为0。
@PIC_1 TOPIC话题输出

仿真环境下，使用 planning_simulator.launch.xml 启动文件，地图文件相同，对应话题输出z中 longitudinal_velocity_mps 为 lanelet地图中的speed_limit参数值

@PIC_2 仿真环境下topic输出

## 代码流程分析

### 1. 路径生成流程

```
BehaviorPathPlannerNode::run()
  └─> planner_manager_->run(planner_data_)
       └─> getReferencePath(data)  // 获取参考路径
            └─> utils::getReferencePath(current_route_lanelet, data)
                 └─> getCenterLinePath(...)
                      └─> route_handler.getCenterLinePath(...)
                           └─> 从lanelet地图读取速度限制
```

### 2. 速度设置的关键代码

**位置：** `src/core/autoware_core/planning/autoware_route_handler/src/route_handler.cpp` (行1588-1596)

```cpp
for (size_t lanelet_idx = 0; lanelet_idx < lanelet_sequence.size(); ++lanelet_idx) {
  const auto & lanelet = lanelet_sequence.at(lanelet_idx);
  
  // 从地图读取速度限制
  const float speed_limit =
    static_cast<float>(traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value());
  
  const auto & piecewise_ref_points = piecewise_ref_points_vec.at(lanelet_idx);
  for (const auto & ref_point : piecewise_ref_points) {
    PathPointWithLaneId p{};
    p.point.pose.position = ref_point.point;
    p.lane_ids.push_back(lanelet.id());
    p.point.longitudinal_velocity_mps = speed_limit;  // 设置速度
    reference_path.points.push_back(p);
  }
}
```

## 可能导致速度为0的原因

### 原因1：Lanelet地图中未设置速度限制 ⭐⭐⭐⭐⭐

**最常见的原因**

#### 问题描述
- Lanelet地图的OSM文件中，相关lanelet未定义 `speed_limit` 标签
- 或者 `speed_limit` 标签的值为0

#### 检查方法
```bash
# 检查地图文件中的速度限制定义
grep "speed_limit" /path/to/your_map.osm

# 查看具体的lanelet定义
grep -A 10 -B 5 "speed_limit" /path/to/your_map.osm

# 查看当前车辆所在的lanelet ID
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id --once | grep "lane_ids" | head -n 5
```

#### 地图示例（正确）
```xml
<relation id="2621">
  <member type="way" role="left" ref="2593"/>
  <member type="way" role="right" ref="2620"/>
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="road"/>
  <tag k="speed_limit" v="30.00"/>  <!-- 必须设置，单位：km/h -->
  <tag k="location" v="urban"/>
  <tag k="one_way" v="yes"/>
  <tag k="participant:vehicle" v="yes"/>
</relation>
```

#### 解决方案
1. 在JOSM中打开地图文件
2. 选择对应的lanelet
3. 添加 `speed_limit` 标签（单位：km/h）
4. 保存地图并重新加载

---

### 原因2：Traffic Rules未正确初始化 ⭐⭐⭐⭐

#### 问题描述
- `traffic_rules_ptr_` 为空或未正确初始化
- traffic_rules无法从lanelet读取速度限制

#### 检查方法
```bash
# 查看日志中是否有地图加载错误
ros2 topic echo /rosout | grep -i "traffic_rules\|lanelet\|map"

# 检查地图是否正确加载
ros2 topic info /map/vector_map

# 检查地图话题是否有数据
ros2 topic echo /map/vector_map --once
```

#### 相关代码位置
**位置：** `src/core/autoware_lanelet2_extension/autoware_lanelet2_extension/lib/message_conversion.cpp` (行88-90)

```cpp
void fromBinMsg(
  const autoware_map_msgs::msg::LaneletMapBin & msg, 
  lanelet::LaneletMapPtr map,
  lanelet::traffic_rules::TrafficRulesPtr * traffic_rules,
  lanelet::routing::RoutingGraphPtr * routing_graph)
{
  fromBinMsg(msg, map);
  // 创建交通规则对象
  *traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany,      // 使用德国交通规则
    lanelet::Participants::Vehicle);  // 车辆参与者
  *routing_graph = lanelet::routing::RoutingGraph::build(*map, **traffic_rules);
}
```

#### 解决方案
1. 确认地图文件格式正确（OSM格式）
2. 检查Autoware启动日志，确认地图加载成功
3. 重启Autoware，确保地图正确加载

---

### 原因3：SpeedLimit返回无效值 ⭐⭐⭐

#### 问题描述
- `traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value()` 返回0或无效值
- Lanelet类型不支持速度限制（如人行道、停车区等）

#### 检查lanelet类型
```bash
# 检查当前lanelet的subtype
grep -A 5 "lane_ids" /path/to/your_map.osm
```

#### 不支持速度限制的lanelet类型
- `subtype="sidewalk"` - 人行道
- `subtype="crosswalk"` - 人行横道
- `subtype="parking"` - 停车区
- 非 `participant:vehicle="yes"` 的lanelet

#### 地图检查
```xml
<!-- 错误示例：人行道没有速度限制 -->
<relation id="1234">
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="sidewalk"/>  <!-- 人行道 -->
  <tag k="participant:pedestrian" v="yes"/>
  <!-- 缺少 speed_limit 标签 -->
</relation>

<!-- 正确示例：道路有速度限制 -->
<relation id="1235">
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="road"/>
  <tag k="participant:vehicle" v="yes"/>
  <tag k="speed_limit" v="50.00"/>
</relation>
```

#### 解决方案
1. 确认车辆在正确的车道上（road类型）
2. 确认lanelet有 `participant:vehicle="yes"` 属性
3. 为所有道路类型的lanelet添加speed_limit标签

---

### 原因4：车辆在Goal附近 ⭐⭐⭐

#### 问题描述
当车辆接近目标点时，`createGoalAroundPath` 函数会被调用，将路径速度设置为0。

#### 相关代码
**位置：** `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/utils/path_utils.cpp` (行547-550)

```cpp
BehaviorModuleOutput createGoalAroundPath(const std::shared_ptr<const PlannerData> & planner_data)
{
  // ... 生成goal周围的路径 ...
  
  // Insert zero velocity to each point in the path.
  for (auto & point : reference_path.points) {
    point.point.longitudinal_velocity_mps = 0.0;  // 强制设为0
  }
  
  // ...
}
```

#### 触发条件
**位置：** `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp` (行146-157)

```cpp
const bool is_out_of_route = utils::isEgoOutOfRoute(
  data->self_odometry->pose.pose, current_route_lanelet_->value(), 
  data->prev_modified_goal, data->route_handler);

if (!is_any_module_running && is_out_of_route) {
  BehaviorModuleOutput result_output = utils::createGoalAroundPath(data);
  RCLCPP_WARN_THROTTLE(
    logger_, clock_, 5000,
    "Ego is out of route, no module is running. Skip running scene modules.");
  generateCombinedDrivableArea(result_output, data);
  return result_output;  // 返回速度为0的路径
}
```

#### 检查方法
```bash
# 查看日志中是否有 "out of route" 警告
ros2 topic echo /rosout | grep -i "out of route"

# 检查车辆是否接近目标点
ros2 topic echo /planning/mission_planning/route --once | grep "goal_pose" -A 10

# 查看当前车辆位置
ros2 topic echo /localization/kinematic_state --once
```

#### 解决方案
1. 如果车辆确实到达目标点，这是正常行为
2. 如果车辆未到达目标点但被判定为"out of route"：
   - 检查route是否正确
   - 检查lanelet连接是否正确
   - 重新规划路线

---

### 原因5：场景模块修改了速度 ⭐⭐⭐

#### 问题描述
某些场景模块（Scene Modules）可能会修改路径速度，包括：
- Goal Planner
- Start Planner  
- Avoidance Module
- Lane Change Module
- 停车模块

#### 可能清零速度的场景
1. **停车场景**：车辆准备停车时
2. **避障场景**：检测到障碍物需要停车
3. **交通灯**：红灯停车
4. **人行横道**：有行人通过

#### 检查方法
```bash
# 查看当前激活的场景模块
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/scene_module_status

# 查看停止原因
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/stop_reasons

# 查看规划因子（planning factors）
ros2 topic list | grep planning_factors
```

#### 相关模块速度设置位置
**Goal Planner示例：**
```cpp
// 在goal planner中可能设置停止速度
for (auto & p : path.points) {
  p.point.longitudinal_velocity_mps = 0.0;  // 停车
}
```

#### 解决方案
1. 检查是否有场景模块正在运行
2. 查看停止原因（stop_reasons）
3. 如果是预期行为（如交通灯停车），则正常
4. 如果是误触发，检查对应模块的参数配置

---

### 原因6：路径重采样问题 ⭐⭐

#### 问题描述
在路径重采样过程中，速度信息可能丢失或被清零。

#### 相关代码
**位置：** `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` (行724-726)

```cpp
const auto resampled_path = utils::resamplePathWithSpline(
  *path, planner_data->parameters.output_path_interval, 
  keepInputPoints(module_status_ptr_vec));
return std::make_shared<PathWithLaneId>(resampled_path);
```

**位置：** `autoware::motion_utils::resamplePath` 的实现可能影响速度插值

#### 检查方法
```bash
# 检查重采样参数
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner output_path_interval

# 查看原始路径和重采样后的路径
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/reference_path
```

#### 解决方案
1. 检查 `output_path_interval` 参数是否合理
2. 确认重采样算法正确处理速度插值
3. 如果怀疑是重采样问题，可以尝试调整间隔参数

---

### 原因7：初始化时路径为空 ⭐

#### 问题描述
系统刚启动时，或者没有有效路径时，可能输出空路径或零速度路径。

#### 检查方法
```bash
# 检查route是否已设置
ros2 topic echo /planning/mission_planning/route --once

# 检查当前位置是否在路线上
ros2 topic echo /localization/kinematic_state --once

# 查看是否有错误日志
ros2 topic echo /rosout | grep -i "error\|empty\|invalid"
```

#### 解决方案
1. 确保已设置有效的route（使用RViz设置目标点）
2. 确认车辆位置在地图覆盖范围内
3. 等待系统完全初始化

---

## 调试步骤总结

### 快速诊断流程

#### 步骤1：检查地图速度限制（最重要）
```bash
# 查看当前车道的速度限制
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id --once | grep -A 2 "lane_ids" | head -n 10

# 在地图文件中查找对应lane_id的speed_limit
grep "id='<lane_id>'" /path/to/map.osm -A 10 | grep speed_limit
```

#### 步骤2：检查系统状态
```bash
# 运行监控脚本
./check_speed_limit.sh

# 查看behavior_planning输出的路径速度
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once | \
  grep "longitudinal_velocity_mps" | head -n 10
```

#### 步骤3：检查场景模块状态
```bash
# 查看激活的场景模块
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/scene_module_status --once

# 查看停止原因
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/stop_reasons --once
```

#### 步骤4：查看日志
```bash
# 查看相关错误和警告
ros2 topic echo /rosout | grep -i "behavior_path\|speed\|velocity"
```

### 完整调试脚本

创建 `debug_bpp_velocity.sh`：

```bash
#!/bin/bash

echo "======================================"
echo "  Behavior Path Planner 速度调试"
echo "======================================"
echo ""

# 1. 检查路径速度
echo "[1] 当前路径速度分布"
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once 2>/dev/null | \
  grep "longitudinal_velocity_mps" | \
  awk '{print $2}' | sort -n | uniq -c | \
  awk '{printf "  %3d 个点: %.2f m/s (%.1f km/h)\n", $1, $2, $2*3.6}'
echo ""

# 2. 检查lane_ids
echo "[2] 当前路径的lane IDs"
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once 2>/dev/null | \
  grep "lane_ids" | head -n 5
echo ""

# 3. 检查场景模块状态
echo "[3] 激活的场景模块"
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/scene_module_status --once 2>/dev/null | \
  grep "module_name" | head -n 10
echo ""

# 4. 检查停止原因
echo "[4] 停止原因"
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/stop_reasons --once 2>/dev/null
echo ""

# 5. 检查日志错误
echo "[5] 最近的相关日志"
timeout 2 ros2 topic echo /rosout 2>/dev/null | \
  grep -i "behavior_path\|out of route\|empty.*path" | tail -n 5
echo ""

# 6. 检查是否接近goal
echo "[6] Route状态"
ros2 topic echo /planning/mission_planning/route --once 2>/dev/null | grep -E "goal_pose|segments" | head -n 5
echo ""

echo "======================================"
echo "调试完成"
echo "======================================"
```

## 解决方案优先级

### ⭐⭐⭐⭐⭐ 最优先检查

1. **检查Lanelet地图的speed_limit标签**
   - 这是最常见的原因
   - 直接检查地图文件
   - 为所有道路lanelet添加speed_limit

### ⭐⭐⭐⭐ 高优先级

2. **确认traffic_rules正确初始化**
   - 检查地图是否正确加载
   - 查看启动日志

3. **检查车辆是否接近goal**
   - 查看"out of route"日志
   - 确认车辆位置

### ⭐⭐⭐ 中优先级

4. **检查场景模块状态**
   - 查看激活的模块
   - 检查停止原因

5. **确认lanelet类型正确**
   - 车辆应该在road类型的lanelet上
   - 不应该在人行道或停车区

### ⭐⭐ 低优先级

6. **检查路径重采样参数**
7. **检查系统初始化状态**

## 参考文档

- [Lanelet地图速度限制参数说明](./lanelet地图速度限制参数说明.md)
- [如何实时查看速度限制 - 快速指南](./如何实时查看速度限制_快速指南.md)
- [PLANNING_VELOCITY_CODE_REFERENCE.md](./PLANNING_VELOCITY_CODE_REFERENCE.md)

## 关键代码位置

| 功能 | 文件路径 | 行号 |
|-----|---------|------|
| 速度从地图读取 | `src/core/autoware_core/planning/autoware_route_handler/src/route_handler.cpp` | 1588-1596 |
| 参考路径生成 | `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/utils/path_utils.cpp` | 451-510 |
| Goal附近速度清零 | `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/utils/path_utils.cpp` | 547-550 |
| PlannerManager主流程 | `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/planner_manager.cpp` | 103-191 |
| 路径发布 | `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner/src/behavior_path_planner_node.cpp` | 402 |
| Traffic Rules初始化 | `src/core/autoware_lanelet2_extension/autoware_lanelet2_extension/lib/message_conversion.cpp` | 88-90 |

