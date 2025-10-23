# Autoware运行时Lanelet地图导入的速度限制参数说明

## 概述

Autoware使用Lanelet2格式的高精度地图，速度限制信息通过地图标签（tag）定义，并在运行时通过traffic_rules模块读取和应用。

## 1. 地图中的速度限制定义

### 1.1 基本格式

在Lanelet2地图的OSM格式中，速度限制通过`speed_limit`标签定义在lanelet关系（relation）中：

```xml
<relation id="2621">
    <member type="way" role="left" ref="2593"/>
    <member type="way" role="right" ref="2620"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
    <tag k="speed_limit" v="30.00"/>  <!-- 速度限制，单位：km/h -->
    <tag k="location" v="urban"/>
    <tag k="one_way" v="yes"/>
    <tag k="participant:vehicle" v="yes"/>
</relation>
```

**关键说明：**
- `speed_limit`标签的值单位为 **km/h（公里/小时）**
- 该标签定义在lanelet级别，每个lanelet可以有独立的速度限制
- 文档位置：`src/core/autoware_lanelet2_extension/autoware_lanelet2_extension/docs/lanelet2_format_extension.md`

### 1.2 特殊区域的速度限制

#### 人行横道（Crosswalk）
```xml
<relation id="1556">
  <member type="way" role="left" ref="1449"/>
  <member type="way" role="right" ref="1450"/>
  <member type="relation" role="regulatory_element" ref="179750"/>
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="crosswalk"/>
  <tag k="speed_limit" v="10"/>  <!-- 人行横道速度限制 -->
  <tag k="location" v="urban"/>
</relation>
```

人行横道还可以定义安全减速参数：
```xml
<relation id='34378'>
  <tag k='subtype' v='crosswalk' />
  <tag k='safety_slow_down_speed' v='3.0' />      <!-- 通过人行横道时的安全速度 m/s -->
  <tag k='safety_slow_down_distance' v='2.0' />   <!-- 减速开始距离 m -->
  <tag k='type' v='lanelet' />
</relation>
```

#### 减速带（Speed Bump）
减速带可以指定减速速度：
```xml
<way id='5'>
  <nd ref='1' />
  <nd ref='2' />
  <tag k='area' v='yes' />
  <tag k='height' v='0.15' />                    <!-- 减速带高度 m -->
  <tag k='type' v='speed_bump' />
  <tag k='slow_down_speed' v='10.0' />           <!-- 通过减速带的速度 km/h (可选) -->
</way>
```

## 2. 运行时加载机制

### 2.1 地图加载过程

地图加载时，通过`fromBinMsg`函数初始化traffic_rules：

```cpp
// 位置：src/core/autoware_lanelet2_extension/autoware_lanelet2_extension/lib/message_conversion.cpp
void fromBinMsg(
  const autoware_map_msgs::msg::LaneletMapBin & msg, 
  lanelet::LaneletMapPtr map,
  lanelet::traffic_rules::TrafficRulesPtr * traffic_rules,
  lanelet::routing::RoutingGraphPtr * routing_graph)
{
  fromBinMsg(msg, map);
  // 创建交通规则对象 - 使用德国交通规则和车辆参与者类型
  *traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany,      // 国家/地区
    lanelet::Participants::Vehicle);  // 参与者类型
  *routing_graph = lanelet::routing::RoutingGraph::build(*map, **traffic_rules);
}
```

**注意：** Autoware默认使用`Germany`交通规则，这是Lanelet2库的标准配置。

### 2.2 速度限制读取

在planning模块中，通过`traffic_rules_ptr_->speedLimit()`方法读取速度限制：

```cpp
// 位置：src/core/autoware_core/planning/autoware_route_handler/src/route_handler.cpp (行1588-1596)
const lanelet::ConstLanelet & lanelet = lanelet_sequence.at(lanelet_idx);

// 获取当前lanelet的速度限制信息
const float speed_limit = static_cast<float>(
    traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value()
);

// 设置路径点的纵向速度
p.point.longitudinal_velocity_mps = speed_limit;
```

返回的`SpeedLimitInformation`结构包含：
- `speedLimit`：速度限制值（单位转换为 **m/s**）
- 其他相关交通规则信息

### 2.3 在Path Generator中的应用

```cpp
// 位置：src/core/autoware_core/planning/autoware_path_generator/src/node.cpp (行342-343)
path_point_with_lane_id.point.longitudinal_velocity_mps =
    planner_data_.traffic_rules_ptr->speedLimit(lanelet).speedLimit.value();
```

## 3. 速度限制的实际应用

### 3.1 Map-Based Prediction模块

在障碍物预测中使用速度限制：

```cpp
// 位置：src/universe/autoware_universe/perception/autoware_map_based_prediction/src/map_based_prediction_node.cpp (行926-929)
const lanelet::traffic_rules::SpeedLimitInformation limit =
    traffic_rules_ptr_->speedLimit(current_lanelet_data.lanelet);
const double legal_speed_limit = static_cast<double>(limit.speedLimit.value());
target_speed_limit = legal_speed_limit * speed_limit_multiplier_;
```

相关配置参数：
```yaml
# 位置：src/launcher/autoware_launch/autoware_launch/config/perception/object_recognition/prediction/map_based_prediction.param.yaml
/**:
  ros__parameters:
    speed_limit_multiplier: 1.5  # 速度限制倍数：实际使用时 = 地图速度限制 × 1.5
```

### 3.2 速度限制的层级关系

Autoware中速度限制的优先级从高到低：

1. **外部速度限制** (`external_velocity_limit`)
   - 来自外部系统的动态速度限制
   - 在behavior_velocity_planner中处理

2. **地图速度限制** (`map speed_limit`)
   - 从lanelet地图读取的静态速度限制
   - 通过traffic_rules获取

3. **车辆速度限制** (`vehicle_cmd_gate vel_lim`)
   ```yaml
   # 位置：src/launcher/autoware_launch/autoware_launch/config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml
   nominal:
     vel_lim: 25.0  # 车辆最大速度限制 m/s (90 km/h)
   ```

4. **规划器最大速度** (`max_vel`)
   ```yaml
   # 位置：src/core/autoware_core/planning/autoware_core_planning/config/common.param.yaml
   /**:
     ros__parameters:
       max_vel: 4.17  # 最大速度限制 m/s (15 km/h)
   ```

## 4. 单位转换说明

**重要：** 地图中定义的速度使用不同单位，需要注意：

| 参数类型 | 地图中单位 | 运行时单位 | 转换 |
|---------|----------|-----------|------|
| `speed_limit` (lanelet标签) | km/h | m/s | 自动转换 |
| `safety_slow_down_speed` | m/s | m/s | 无需转换 |
| `slow_down_speed` (减速带) | km/h | m/s | 需除以3.6 |

代码中的转换示例：
```cpp
// 减速带速度转换
// 位置：src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/src/scene.cpp (行54-55)
if (speed_bump_reg_elem_.speedBump().hasAttribute("slow_down_speed")) {
  speed_bump_slow_down_speed_ = static_cast<float>(
    speed_bump_reg_elem_.speedBump().attribute("slow_down_speed").asDouble().get() / 3.6
  );  // km/h -> m/s
}
```

## 5. 🔍 实时查看速度限制参数（运行状态）

### 5.1 通过ROS话题查看当前最大速度限制

**话题名称：** `/planning/scenario_planning/current_max_velocity`  
**消息类型：** `autoware_internal_planning_msgs/msg/VelocityLimit`

#### 方法1：实时监听速度限制

```bash
# 查看当前速度限制
ros2 topic echo /planning/scenario_planning/current_max_velocity

# 输出示例：
# stamp:
#   sec: 1729425432
#   nanosec: 123456789
# max_velocity: 15.0        # 最大速度 [m/s] (54 km/h)
# use_constraints: false
# constraints:
#   max_acceleration: 0.0
#   max_jerk: 0.0
# sender: 'velocity_smoother'
```

**消息字段说明：**
- `max_velocity`：当前应用的最大速度限制（m/s）
- `sender`：速度限制的来源模块（如velocity_smoother、cruise等）
- `stamp`：时间戳

#### 方法2：检查话题频率和数据

```bash
# 查看话题发布频率
ros2 topic hz /planning/scenario_planning/current_max_velocity

# 查看话题信息
ros2 topic info /planning/scenario_planning/current_max_velocity

# 只查看速度值（更简洁）
ros2 topic echo /planning/scenario_planning/current_max_velocity --once | grep max_velocity
```

#### 方法3：持续监控速度限制变化

```bash
# 持续监控速度限制变化（每0.5秒更新）
watch -n 0.5 "ros2 topic echo /planning/scenario_planning/current_max_velocity --once | grep 'max_velocity'"

# 或者使用简单脚本
while true; do
  echo "$(date '+%H:%M:%S') - Speed Limit: $(ros2 topic echo /planning/scenario_planning/current_max_velocity --once 2>/dev/null | grep 'max_velocity' | awk '{print $2}') m/s"
  sleep 0.5
done
```

### 5.2 通过RViz可视化查看速度限制

#### 在RViz中启用SpeedLimitDisplay

1. **启动Autoware后打开RViz**
   - RViz会自动加载，或者手动运行：
   ```bash
   rviz2 -d src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz
   ```

2. **查看SignalDisplay/OverlayDisplay**
   - 在RViz窗口左上角会显示仪表盘
   - **红色圆圈标志** - 显示当前速度限制（单位：km/h）
   - 蓝色数字 - 显示当前车速
   - 当车速接近限速时，圆圈会变成橙色或红色警告

3. **配置速度限制话题**
   - SpeedLimitDisplay订阅：`/planning/scenario_planning/current_max_velocity`
   - 如需修改，在RViz左侧面板找到对应Display进行配置

**RViz配置位置：**
```
src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz
src/universe/autoware_universe/visualization/autoware_overlay_rviz_plugin/autoware_overlay_rviz_plugin/src/speed_limit_display.cpp
```

### 5.3 查看路径中每个路点的速度（来自地图）

#### 方法1：查看behavior planning输出的路径速度

```bash
# 查看behavior planning输出的路径（包含从地图读取的速度）
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path

# 输出中每个点的字段：
# points:
#   - lane_ids: [123, 124]
#     point:
#       pose: ...
#       longitudinal_velocity_mps: 8.333333  # 这是从lanelet地图读取的速度限制 (30 km/h)
#       lateral_velocity_mps: 0.0
```

#### 方法2：查看最终规划轨迹的速度

```bash
# 查看完整轨迹的速度信息
ros2 topic echo /planning/scenario_planning/trajectory

# 只看速度值（纵向速度）
ros2 topic echo /planning/scenario_planning/trajectory --once | \
  grep -A 1 "longitudinal_velocity_mps"

# 输出示例：
#     longitudinal_velocity_mps: 8.333333  # 30 km/h
#     lateral_velocity_mps: 0.0
#   --
#     longitudinal_velocity_mps: 8.333333
#     lateral_velocity_mps: 0.0
```

#### 方法3：统计轨迹速度分布

```bash
# 统计当前轨迹中不同速度值的数量
ros2 topic echo /planning/scenario_planning/trajectory --once | \
  grep "longitudinal_velocity_mps" | \
  awk '{print $2}' | \
  sort -n | uniq -c

# 输出示例：
#     45 0.0          # 45个点速度为0 (停止点)
#     120 8.333333    # 120个点速度为8.33 m/s (30 km/h)
#     35 13.888889    # 35个点速度为13.89 m/s (50 km/h)
```

**速度值与地图的对应关系：**
- `8.333333 m/s` = 30 km/h（地图中定义的速度限制）
- `13.888889 m/s` = 50 km/h
- `4.166667 m/s` = 15 km/h

#### 方法4：查找零速度点（停止点）

```bash
# 查找轨迹中速度为0的点（停止点）
ros2 topic echo /planning/scenario_planning/trajectory --once | \
  grep -B 2 "longitudinal_velocity_mps: 0.0" | head -n 20

# 统计停止点数量
ZERO_COUNT=$(ros2 topic echo /planning/scenario_planning/trajectory --once 2>/dev/null | \
  grep "longitudinal_velocity_mps: 0.0" | wc -l)
echo "检测到 ${ZERO_COUNT} 个停止点"
```

### 5.4 创建便捷的监控脚本

创建一个调试脚本 `check_speed_limit.sh`：

```bash
#!/bin/bash

echo "======================================"
echo "  Autoware 速度限制实时监控"
echo "======================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# 1. 检查当前速度限制
echo -e "${YELLOW}[1] 当前最大速度限制${NC}"
timeout 2 ros2 topic echo /planning/scenario_planning/current_max_velocity --once 2>/dev/null | \
  grep -E "max_velocity|sender"
echo ""

# 2. 检查实际车速
echo -e "${YELLOW}[2] 当前车速${NC}"
timeout 2 ros2 topic echo /vehicle/status/velocity_status --once 2>/dev/null | \
  grep "longitudinal_velocity"
echo ""

# 3. 检查轨迹速度统计
echo -e "${YELLOW}[3] 当前轨迹速度分布${NC}"
timeout 3 ros2 topic echo /planning/scenario_planning/trajectory --once 2>/dev/null | \
  grep "longitudinal_velocity_mps" | \
  awk '{print $2}' | \
  sort -n | uniq -c | \
  awk '{printf "  %3d 个点: %.2f m/s (%.1f km/h)\n", $1, $2, $2*3.6}'
echo ""

# 4. 检查是否有停止点
echo -e "${YELLOW}[4] 停止点检查${NC}"
ZERO_COUNT=$(timeout 3 ros2 topic echo /planning/scenario_planning/trajectory --once 2>/dev/null | \
  grep "longitudinal_velocity_mps: 0.0" | wc -l)
if [ "$ZERO_COUNT" -gt 0 ]; then
    echo -e "${CYAN}  ✓ 检测到 ${ZERO_COUNT} 个停止点${NC}"
else
    echo -e "${GREEN}  ✓ 无停止点，车辆保持运动${NC}"
fi
echo ""

# 5. 检查behavior planning路径速度
echo -e "${YELLOW}[5] Behavior Planning路径速度（来自地图）${NC}"
timeout 3 ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once 2>/dev/null | \
  grep "longitudinal_velocity_mps" | head -n 5 | \
  awk '{printf "  点速度: %.2f m/s (%.1f km/h)\n", $2, $2*3.6}'
echo ""

echo "======================================"
echo "监控完成"
echo "======================================"
```

使用方法：
```bash
chmod +x check_speed_limit.sh
./check_speed_limit.sh
```

### 5.5 调试常用话题列表

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/planning/scenario_planning/current_max_velocity` | `VelocityLimit` | **当前最大速度限制（推荐）** |
| `/planning/scenario_planning/trajectory` | `Trajectory` | 最终输出轨迹（含速度） |
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | `Path` | behavior planning路径（含地图速度） |
| `/planning/scenario_planning/lane_driving/trajectory` | `Trajectory` | lane driving轨迹 |
| `/vehicle/status/velocity_status` | `VelocityReport` | 实际车速 |
| `/planning/scenario_planning/max_velocity` | `VelocityLimit` | 外部速度限制输入 |

### 5.6 检查地图文件中的速度限制

```bash
# 在OSM地图文件中查找速度限制标签
grep "speed_limit" /path/to/your_map.osm

# 查看完整的lanelet定义（包含周围上下文）
grep -A 10 -B 5 "speed_limit" /path/to/your_map.osm

# 统计地图中不同速度限制的数量
grep "speed_limit" /path/to/your_map.osm | \
  grep -o 'v="[0-9.]*"' | \
  sort | uniq -c
```

### 5.7 地图验证

使用autoware_lanelet2_validation节点验证地图的有效性：
```bash
ros2 run autoware_lanelet2_extension autoware_lanelet2_validation
```

### 5.8 速度限制来源追踪

速度限制可能来自多个来源，按优先级从高到低检查：

```bash
# 1. 检查外部API设置的速度限制（最高优先级）
ros2 topic echo /planning/scenario_planning/max_velocity

# 2. 检查地图中的速度限制（通过路径点）
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once | \
  grep "longitudinal_velocity_mps" | head -n 10

# 3. 检查velocity smoother输出的当前限制
ros2 topic echo /planning/scenario_planning/current_max_velocity

# 4. 检查vehicle_cmd_gate的硬限制
ros2 param get /control/vehicle_cmd_gate nominal.vel_lim

# 5. 检查planning模块的最大速度配置
ros2 param get /planning/scenario_planning/lane_driving/motion_planning/path_optimizer max_vel
```

## 6. 主要代码文件位置

| 功能 | 文件路径 |
|-----|---------|
| 地图格式文档 | `src/core/autoware_lanelet2_extension/autoware_lanelet2_extension/docs/lanelet2_format_extension.md` |
| 消息转换（地图加载） | `src/core/autoware_lanelet2_extension/autoware_lanelet2_extension/lib/message_conversion.cpp` |
| 路径规划器速度设置 | `src/core/autoware_core/planning/autoware_route_handler/src/route_handler.cpp` (行1588) |
| 路径生成器速度设置 | `src/core/autoware_core/planning/autoware_path_generator/src/node.cpp` (行343) |
| 预测模块速度使用 | `src/universe/autoware_universe/perception/autoware_map_based_prediction/src/map_based_prediction_node.cpp` (行927) |
| 减速带模块 | `src/universe/autoware_universe/planning/behavior_velocity_planner/autoware_behavior_velocity_speed_bump_module/` |

## 7. 常见问题

### Q1: 速度限制为0或异常怎么办？
**A:** 检查lanelet地图中是否正确设置了`speed_limit`标签。如果未设置，traffic_rules可能返回默认值或无效值。

### Q2: 车辆没有遵守地图中的速度限制？
**A:** 检查以下配置：
1. `vehicle_cmd_gate`的`vel_lim`参数
2. planning模块的`max_vel`参数
3. 是否有外部速度限制覆盖了地图速度限制

### Q3: 如何修改默认的交通规则？
**A:** 目前Autoware硬编码使用`Locations::Germany`和`Participants::Vehicle`。如需修改，需要在`message_conversion.cpp`中更改`TrafficRulesFactory::create()`的参数。

## 8. 总结

- Lanelet地图中的速度限制通过`speed_limit`标签定义（单位：km/h）
- 运行时通过`traffic_rules_ptr_->speedLimit()`读取（自动转换为m/s）
- 默认使用Germany交通规则
- 实际速度受多个参数限制，以最严格的为准
- 特殊区域（人行横道、减速带）可以有额外的速度限制配置

