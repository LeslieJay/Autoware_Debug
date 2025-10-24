# Autoware Planning 快速参考指南

## 📋 5分钟速查表

### Core vs Universe 快速对比

| 问题 | 回答 |
|------|------|
| **能否只用Core不用Universe?** | ❌ 不行，缺少关键功能模块（如behavior_path_planner） |
| **能否只用Universe不用Core?** | ❌ 不行，Universe依赖Core的基础组件 |
| **两者是否会同时运行?** | ✅ 是的，它们在运行时协同工作 |
| **修改配置需要重新编译吗?** | 部分需要，模块启用/禁用需要；参数调整不需要 |
| **AGV需要全部模块吗?** | ❌ 不需要，可以禁用大部分Universe模块 |

---

## 🔑 关键模块速查

### Core - 不可删除的模块 ⭐⭐⭐

```
必须保留 (删除会导致编译失败):
1. autoware_route_handler          - 所有模块都依赖
2. autoware_velocity_smoother      - 最终速度平滑
3. autoware_mission_planner        - 全局路由
4. autoware_planning_factor_interface - 接口定义
5. behavior_velocity_planner_common - 通用功能
6. motion_velocity_planner_common  - 通用功能
```

### Universe - 可选择性使用 ⭐⭐

```
室内AGV推荐启用:
✓ behavior_path_planner (简化配置)
✓ freespace_planner (停车/倒车)
✓ obstacle_cruise_planner (自适应巡航)
✓ scenario_selector (场景切换)
✓ planning_validator (轨迹验证)

室内AGV推荐禁用:
✗ traffic_light_module (交通灯)
✗ crosswalk_module (人行横道)
✗ intersection_module (十字路口)
✗ lane_change_module (换道)
✗ blind_spot_module (盲区)
```

---

## 📁 文件位置速查

### 配置文件位置

```bash
# 模块启用/禁用配置
autoware_launch/config/planning/preset/default_preset.yaml

# 各模块参数配置
autoware_launch/config/planning/scenario_planning/
├── common/                          # 通用配置
│   ├── common.param.yaml
│   ├── nearest_search.param.yaml
│   └── autoware_velocity_smoother/
├── lane_driving/
│   ├── behavior_planning/
│   │   ├── behavior_path_planner/
│   │   └── behavior_velocity_planner/
│   └── motion_planning/
│       └── motion_velocity_planner/
└── parking/
    └── freespace_planner/

# Launch文件
autoware_launch/launch/components/tier4_planning_component.launch.xml
tier4_planning_launch/launch/planning.launch.xml
```

### 源代码位置

```bash
# Core源码
src/core/autoware_core/planning/
├── autoware_mission_planner/
├── autoware_route_handler/
├── autoware_velocity_smoother/
└── behavior_velocity_planner/

# Universe源码
src/universe/autoware_universe/planning/
├── behavior_path_planner/
├── behavior_velocity_planner/
├── motion_velocity_planner/
└── autoware_freespace_planner/
```

---

## 🔧 常用调试命令

### 1. 检查模块状态

```bash
# 列出所有planning节点
ros2 node list | grep planning

# 检查特定节点是否运行
ros2 node list | grep behavior_path_planner

# 查看节点信息
ros2 node info /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner
```

### 2. 检查话题数据

```bash
# 查看最终轨迹输出
ros2 topic echo /planning/scenario_planning/trajectory --once

# 检查速度是否为0
ros2 topic echo /planning/scenario_planning/trajectory --once | grep longitudinal_velocity_mps

# 查看中间路径
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once
```

### 3. 检查参数配置

```bash
# 列出节点的所有参数
ros2 param list /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner

# 获取特定参数值
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner wheel_base

# 查看vehicle_info相关参数
ros2 param dump /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner | grep -E "wheel|vehicle"
```

### 4. 查看停止原因

```bash
# 查看为什么速度为0
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/stop_reasons

# 查看planning factors
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/stop_reason
```

### 5. 性能分析

```bash
# 查看处理时间
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/processing_time

# 查看话题频率
ros2 topic hz /planning/scenario_planning/trajectory

# 查看话题带宽
ros2 topic bw /planning/scenario_planning/trajectory
```

---

## 🚀 快速启动配置

### 标准Autoware启动

```bash
# 完整启动（包含所有模块）
ros2 launch autoware_launch autoware.launch.xml \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    map_path:=/path/to/map

# 仅启动planning模块（调试用）
ros2 launch tier4_planning_launch planning.launch.xml \
    vehicle_param_file:=/path/to/vehicle_info.param.yaml
```

### AGV最小化配置启动

创建 `agv_preset.yaml`:
```yaml
# 禁用不需要的模块
- arg:
    name: launch_traffic_light_module
    default: "false"
- arg:
    name: launch_crosswalk_module
    default: "false"
- arg:
    name: launch_intersection_module
    default: "false"
- arg:
    name: launch_lane_change_right_module
    default: "false"
- arg:
    name: launch_lane_change_left_module
    default: "false"
- arg:
    name: launch_blind_spot_module
    default: "false"
- arg:
    name: launch_occlusion_spot_module
    default: "false"
- arg:
    name: launch_run_out_module
    default: "false"

# 保持启用AGV需要的模块
- arg:
    name: launch_static_obstacle_avoidance
    default: "true"
- arg:
    name: launch_goal_planner_module
    default: "true"
- arg:
    name: launch_start_planner_module
    default: "true"
```

启动命令:
```bash
ros2 launch autoware_launch autoware.launch.xml \
    vehicle_model:=my_agv \
    planning_module_preset:=agv_preset \
    map_path:=/path/to/warehouse/map
```

---

## 🐛 常见问题速查

### Q1: 轨迹速度全为0

**可能原因排查顺序:**

```bash
# 1. 检查是否有route
ros2 topic echo /planning/mission_planning/route --once
# 无输出 → 需要设置goal点

# 2. 检查停止原因
ros2 topic echo /planning/.../debug/stop_reasons
# 查看是否有停止线、交通灯等

# 3. 检查定位数据
ros2 topic hz /localization/kinematic_state
# 频率为0 → 定位模块问题

# 4. 检查操作模式
ros2 topic echo /system/operation_mode/state
# mode不是AUTONOMOUS → 需要切换模式

# 5. 检查vehicle参数
ros2 param get /planning/.../behavior_path_planner wheel_base
# 参数异常 → 检查vehicle_info.param.yaml
```

### Q2: 编译失败 - 找不到autoware_route_handler

**解决方案:**
```bash
# 先编译Core
colcon build --packages-select autoware_route_handler

# 再编译依赖它的包
colcon build --packages-select autoware_behavior_path_planner
```

### Q3: 模块没有加载

**检查步骤:**
```bash
# 1. 确认配置文件中启用了模块
cat autoware_launch/config/planning/preset/default_preset.yaml | grep launch_avoidance_module

# 2. 检查launch文件是否引用
grep avoidance tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml

# 3. 查看运行时日志
ros2 topic echo /rosout | grep -i avoidance
```

### Q4: 参数修改不生效

**原因和解决:**
```bash
# 原因1: 修改了错误的配置文件
# 确保修改的是autoware_launch中的配置，不是包内默认配置

# 原因2: 需要重启节点
ros2 lifecycle set /planning/scenario_planning/.../behavior_path_planner shutdown
ros2 launch ...  # 重新启动

# 原因3: 参数被launch文件覆盖
# 检查launch文件中是否有硬编码的参数值
```

---

## 📊 依赖关系速查表

### Universe → Core 依赖

| Universe模块 | 依赖的Core模块 |
|-------------|---------------|
| behavior_path_planner | route_handler, planning_factor_interface |
| behavior_velocity_planner | route_handler, velocity_smoother |
| motion_velocity_planner | motion_velocity_planner_common |
| freespace_planner | route_handler, vehicle_info_utils |
| scenario_selector | route_handler |
| planning_validator | vehicle_info_utils |

### 关键接口依赖

```cpp
// 所有Universe planning模块都会使用:

#include "autoware/route_handler/route_handler.hpp"
// 用途: 路由信息查询、lanelet操作

#include "autoware/planning_factor_interface/planning_factor_interface.hpp"
// 用途: 添加停车原因、规划因子

#include "autoware/vehicle_info_utils/vehicle_info_utils.hpp"
// 用途: 获取车辆尺寸、转向参数

#include "autoware/motion_utils/motion_utils.hpp"
// 用途: 轨迹计算、几何运算
```

---

## 🎯 AGV定制建议速查

### 最小化配置（仅保留必需功能）

```yaml
Planning模块精简配置:

Core (全部保留):
  ✓ mission_planner
  ✓ route_handler
  ✓ velocity_smoother
  ✓ planning_factor_interface
  ✓ behavior_velocity_planner_common
  ✓ motion_velocity_planner_common

Universe (精选):
  ✓ scenario_selector               # 场景切换
  ✓ behavior_path_planner           # 基础路径规划
      ├─ start_planner_module       # 起步
      ├─ goal_planner_module        # 停车（可选）
      └─ static_obstacle_avoidance  # 静态避障
  
  ✓ freespace_planner               # 停车场规划
  
  ✓ motion_velocity_planner
      └─ obstacle_stop_module       # 障碍物停止
  
  ✓ planning_validator              # 轨迹验证

  ✗ 所有traffic/crosswalk/intersection相关
  ✗ lane_change (如果是单车道)
  ✗ blind_spot, occlusion_spot
```

### 性能优化建议

```bash
# 1. 降低规划频率（如果不需要高频）
# 在velocity_smoother配置中:
update_rate: 5.0  # 从默认10Hz降到5Hz

# 2. 减少轨迹点数
# 在behavior_path_planner配置中:
output_path_interval: 2.0  # 增大间隔

# 3. 简化碰撞检测
# 在obstacle_cruise_planner配置中:
prediction_time_horizon: 5.0  # 从默认8s减少到5s

# 4. 禁用不必要的debug输出
# 设置环境变量:
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
```

---

## 📝 快速检查清单

启动Autoware Planning之前检查:

```
□ 地图文件存在且正确
  └─ ls /path/to/map/*.pcd *.osm

□ vehicle_info.param.yaml配置正确
  └─ cat vehicle_info.param.yaml

□ default_preset.yaml配置符合需求
  └─ cat default_preset.yaml | grep launch_

□ Core模块已编译
  └─ ls install/autoware_route_handler

□ 选择的Universe模块已编译
  └─ ls install/autoware_behavior_path_planner

□ ROS2环境已source
  └─ echo $ROS_DISTRO

□ Autoware环境已source
  └─ echo $AUTOWARE_PATH
```

---

## 🔗 相关资源链接

- **Autoware官方文档**: https://autowarefoundation.github.io/autoware-documentation/
- **Planning设计文档**: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/planning/
- **Core vs Universe概念**: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/
- **Planning & Control工作组**: https://github.com/orgs/autowarefoundation/discussions?discussions_q=label%3Aplanning-control-wg

---

## 💡 实用技巧

### 1. 快速禁用所有Universe扩展模块

```bash
# 创建minimal_preset.yaml
cat > minimal_preset.yaml << 'EOF'
# 所有扩展模块设为false
$(grep "launch_.*_module" default_preset.yaml | sed 's/true/false/')
EOF
```

### 2. 实时监控planning状态

```bash
# 使用tmux分屏监控
tmux new-session \; \
  send-keys 'ros2 topic hz /planning/scenario_planning/trajectory' C-m \; \
  split-window -v \; \
  send-keys 'ros2 topic echo /planning/.../debug/stop_reasons' C-m \; \
  split-window -h \; \
  send-keys 'ros2 topic echo /diagnostics | grep planning' C-m
```

### 3. 批量检查参数

```bash
# 创建参数检查脚本
cat > check_planning_params.sh << 'EOF'
#!/bin/bash
nodes=(
  "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner"
  "/planning/scenario_planning/motion_velocity_smoother"
)
for node in "${nodes[@]}"; do
  echo "=== $node ==="
  ros2 param list $node | grep -E "wheel|vehicle|max"
done
EOF
chmod +x check_planning_params.sh
```

---

**最后更新**: 2025-01-17  
**适用版本**: Autoware main分支

