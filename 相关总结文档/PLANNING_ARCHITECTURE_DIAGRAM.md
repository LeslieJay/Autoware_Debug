# Autoware Planning架构可视化图表

## 1. 整体架构层次图

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Autoware Planning Stack                      │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    │                               │
    ┌───────────────▼──────────────┐   ┌───────────▼──────────────┐
    │  autoware_core/planning       │   │ autoware_universe/planning│
    │  (基础层 - Foundation)        │   │  (应用层 - Application)   │
    │                               │   │                           │
    │  • 核心接口和数据结构          │   │  • 复杂场景处理            │
    │  • 基础算法实现               │   │  • 行为决策               │
    │  • 通用工具库                 │   │  • 高级规划模块            │
    │  • 稳定可靠                   │   │  • 可插拔模块             │
    └───────────────┬──────────────┘   └───────────┬──────────────┘
                    │                               │
                    │      提供基础组件              │
                    └───────────────────────────────┘
                                    │
                            依赖Core组件
```

## 2. Core Planning 模块关系图

```
                    ┌──────────────────────────────┐
                    │   Planning Input Sources      │
                    │  (外部输入)                   │
                    │  • /map/vector_map           │
                    │  • /planning/mission_planning/route │
                    │  • /localization/kinematic_state    │
                    └─────────┬────────────────────┘
                              │
                ┌─────────────▼─────────────┐
                │  autoware_mission_planner │
                │  (全局路径规划)            │
                │  • 起点到终点的路由        │
                │  • Lanelet序列生成        │
                └─────────────┬─────────────┘
                              │
                ┌─────────────▼─────────────┐
                │  autoware_route_handler   │◄────┐
                │  (路由管理核心)            │     │
                │  • 路由信息存储            │     │
                │  • Lanelet查询服务        │     │ 所有其他模块
                │  • 前后车道关系查询        │     │ 都需要依赖
                └─────────────┬─────────────┘     │ route_handler
                              │                   │
                ┌─────────────▼─────────────┐     │
                │  autoware_path_generator  │─────┘
                │  (基础路径生成)            │
                │  • 跟随路由生成路径        │
                │  • 转向灯控制              │
                └─────────────┬─────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
┌───────▼────────┐  ┌─────────▼────────┐  ┌────────▼────────┐
│ Behavior       │  │ Motion           │  │ Velocity        │
│ Velocity       │  │ Velocity         │  │ Smoother        │
│ Planner        │  │ Planner          │  │                 │
│ (Common)       │  │ (Common)         │  │ • 速度平滑      │
│                │  │                  │  │ • 加加速度约束  │
│ • 停止线       │  │ • 障碍物停止     │  │ • 横向加速度    │
└────────────────┘  └──────────────────┘  └─────────────────┘
        │                     │                     │
        └─────────────────────┼─────────────────────┘
                              │
                    ┌─────────▼─────────┐
                    │ Planning Output    │
                    │ /planning/.../trajectory │
                    └────────────────────┘
```

## 3. Universe Planning 扩展模块树

```
autoware_universe/planning/
│
├─── 🎭 场景控制层
│    └── autoware_scenario_selector
│         ├─→ LANE_DRIVING ─────┐
│         └─→ PARKING ───────────┼─→ 选择不同的planning pipeline
│                                 │
├─── 🚗 行为决策层 (Lane Driving)  │
│    │                            │
│    ├── behavior_path_planner ◄──┘
│    │   ├── 📍 场景模块:
│    │   │   ├─ start_planner_module        (起步规划)
│    │   │   ├─ goal_planner_module         (目标/泊车规划)
│    │   │   ├─ lane_change_module          (换道)
│    │   │   ├─ side_shift_module           (侧移)
│    │   │   ├─ static_obstacle_avoidance   (静态避障)
│    │   │   ├─ dynamic_obstacle_avoidance  (动态避障)
│    │   │   └─ sampling_planner_module     (采样规划)
│    │   │
│    │   └── 依赖Core: route_handler, planning_factor_interface
│    │
│    └── behavior_velocity_planner (扩展版)
│        ├── 📌 交通规则模块:
│        │   ├─ traffic_light_module        (交通灯)
│        │   ├─ crosswalk_module            (人行横道)
│        │   ├─ intersection_module         (路口)
│        │   ├─ blind_spot_module           (盲区)
│        │   ├─ stop_line_module            (停止线 - 来自Core)
│        │   ├─ detection_area_module       (检测区域)
│        │   ├─ no_stopping_area_module     (禁停区)
│        │   ├─ virtual_traffic_light       (虚拟交通灯)
│        │   ├─ speed_bump_module           (减速带)
│        │   ├─ run_out_module              (冲出检测)
│        │   └─ occlusion_spot_module       (遮挡点)
│        │
│        └── 依赖Core: velocity_smoother, route_handler
│
├─── 🎯 运动规划层
│    │
│    ├── motion_velocity_planner (扩展版)
│    │   ├── 🚧 障碍物处理:
│    │   │   ├─ obstacle_cruise_module      (障碍物巡航)
│    │   │   ├─ obstacle_slow_down_module   (障碍物减速)
│    │   │   ├─ obstacle_stop_module        (障碍物停止 - 来自Core)
│    │   │   ├─ dynamic_obstacle_stop       (动态障碍物)
│    │   │   ├─ obstacle_velocity_limiter   (速度限制)
│    │   │   └─ out_of_lane_module          (车道外检测)
│    │   │
│    │   └── 依赖Core: motion_velocity_planner_common
│    │
│    ├── sampling_based_planner/
│    │   └── autoware_path_sampler          (路径采样)
│    │        └── 依赖Core: vehicle_info_utils
│    │
│    ├── autoware_path_optimizer            (路径优化)
│    │   └── 功能: 避障优化、路径平滑
│    │
│    └── autoware_path_smoother             (路径平滑)
│        └── 功能: 弹性带平滑
│
├─── 🅿️ 停车规划层 (Parking)
│    │
│    └── autoware_freespace_planner
│         ├── 📐 算法:
│         │   ├─ A* / Hybrid A*
│         │   ├─ RRT*
│         │   └─ Reed-Shepp曲线
│         │
│         └── 依赖Core: route_handler, vehicle_info_utils
│
├─── 🔍 辅助功能层
│    │
│    ├── autoware_obstacle_cruise_planner   (自适应巡航)
│    ├── autoware_surround_obstacle_checker (周边障碍物检查)
│    ├── autoware_planning_validator        (轨迹验证)
│    ├── autoware_costmap_generator         (代价地图)
│    ├── autoware_external_velocity_limit_selector (外部速度限制)
│    └── autoware_remaining_distance_time_calculator (剩余距离时间)
│
└─── 🔧 工具和接口
     └── autoware_rtc_interface             (RTC接口 - 请求控制)
```

## 4. 数据流向图

```
                        ┌──────────────────────┐
                        │   外部传感器和定位    │
                        │  • Map, Localization │
                        │  • Perception        │
                        └──────────┬───────────┘
                                   │
                        ┌──────────▼───────────┐
                        │  Mission Planner     │
                        │  (Core)              │
                        │  全局路由规划        │
                        └──────────┬───────────┘
                                   │ LaneletRoute
                        ┌──────────▼───────────┐
                        │  Route Handler       │
                        │  (Core - 核心依赖)   │
                        │  路由管理服务         │
                        └──────────┬───────────┘
                                   │
                 ┌─────────────────┴─────────────────┐
                 │                                   │
      ┌──────────▼──────────┐         ┌─────────────▼─────────────┐
      │  Scenario Selector   │         │  Freespace Planner        │
      │  (Universe)          │         │  (Universe - Parking)     │
      │  场景选择            │         │  停车场规划               │
      └──────────┬──────────┘         └─────────────┬─────────────┘
                 │                                   │
    ┌────────────▼────────────┐                     │
    │  Lane Driving Pipeline  │                     │
    └────────────┬────────────┘                     │
                 │                                   │
    ┌────────────▼──────────────────────────┐       │
    │  Behavior Path Planner (Universe)     │       │
    │  • 换道、避障等行为决策                │       │
    │  • 使用Core的route_handler            │       │
    └────────────┬──────────────────────────┘       │
                 │ Path                             │
    ┌────────────▼──────────────────────────┐       │
    │  Behavior Velocity Planner            │       │
    │  (Core + Universe混合)                │       │
    │  • 交通规则速度调整                    │       │
    └────────────┬──────────────────────────┘       │
                 │ Path with Velocity               │
    ┌────────────▼──────────────────────────┐       │
    │  Motion Velocity Planner              │       │
    │  (Core + Universe混合)                │       │
    │  • 障碍物速度规划                      │       │
    └────────────┬──────────────────────────┘       │
                 │ Trajectory (rough)               │
                 └──────────────┬───────────────────┘
                                │
                  ┌─────────────▼──────────────┐
                  │  Velocity Smoother (Core)   │
                  │  最终速度平滑               │
                  └─────────────┬──────────────┘
                                │ Trajectory (smoothed)
                  ┌─────────────▼──────────────┐
                  │  Planning Validator         │
                  │  (Universe)                 │
                  │  轨迹合法性检查             │
                  └─────────────┬──────────────┘
                                │
                  ┌─────────────▼──────────────┐
                  │  /planning/scenario_planning│
                  │         /trajectory         │
                  │  (最终输出给控制模块)       │
                  └─────────────────────────────┘
```

## 5. 模块启用/禁用配置图

```
default_preset.yaml (配置入口)
│
├─ launch_avoidance_module: true/false
│   └─→ behavior_path_planner
│       └─→ static_obstacle_avoidance_module ✓/✗
│
├─ launch_lane_change_right_module: true/false
│   └─→ behavior_path_planner
│       └─→ lane_change_right_module ✓/✗
│
├─ launch_traffic_light_module: true/false
│   └─→ behavior_velocity_planner
│       └─→ traffic_light_module ✓/✗
│
├─ launch_crosswalk_module: true/false
│   └─→ behavior_velocity_planner
│       └─→ crosswalk_module ✓/✗
│
├─ launch_intersection_module: true/false
│   └─→ behavior_velocity_planner
│       └─→ intersection_module ✓/✗
│
└─ motion_stop_planner_type: "obstacle_cruise_planner"
    └─→ motion_velocity_planner
        ├─→ obstacle_cruise_planner (if selected) ✓
        └─→ obstacle_stop_planner (alternative) ✗

AGV室内应用推荐配置:
✓ 保持启用: static_obstacle_avoidance, side_shift, freespace_planner
✗ 建议禁用: traffic_light, crosswalk, intersection, lane_change
```

## 6. AGV定制架构建议

```
┌─────────────────────────────────────────────────────────┐
│                   AGV Planning Stack                     │
│                  (基于Autoware定制)                      │
└─────────────────────────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
┌───────▼────────┐ ┌──────▼──────┐ ┌───────▼────────┐
│ Core Modules   │ │ Selected    │ │ Custom         │
│ (必须保留)      │ │ Universe    │ │ AGV Modules   │
│                │ │ Modules     │ │ (自定义)       │
│ • mission_     │ │             │ │                │
│   planner      │ │ • behavior_ │ │ • agv_specific_│
│ • route_       │ │   path_     │ │   planner      │
│   handler      │ │   planner   │ │ • warehouse_   │
│ • velocity_    │ │   (简化版)  │ │   navigation   │
│   smoother     │ │ • freespace_│ │ • pallet_      │
│ • planning_    │ │   planner   │ │   approach     │
│   factor_      │ │ • obstacle_ │ │                │
│   interface    │ │   stop      │ │                │
└────────────────┘ └─────────────┘ └────────────────┘
        │                 │                 │
        └─────────────────┼─────────────────┘
                          │
              ┌───────────▼───────────┐
              │  Common Dependencies  │
              │  • vehicle_info_utils │
              │  • motion_utils       │
              │  • route_handler      │
              └───────────────────────┘
```

## 7. 编译依赖关系图

```
编译顺序 (必须按此顺序):

Level 1: Core基础组件
  ├─ autoware_vehicle_info_utils
  ├─ autoware_motion_utils
  ├─ autoware_planning_msgs
  └─ autoware_internal_planning_msgs
          │
          ▼
Level 2: Core Planning基础
  ├─ autoware_route_handler ◄───────┐ (核心依赖)
  ├─ autoware_planning_factor_interface
  ├─ autoware_velocity_smoother      │
  └─ autoware_mission_planner        │
          │                          │
          ▼                          │
Level 3: Universe Planning扩展       │
  ├─ behavior_path_planner ──────────┘
  ├─ behavior_velocity_planner ──────┘
  ├─ motion_velocity_planner ────────┘
  ├─ freespace_planner ──────────────┘
  └─ scenario_selector ──────────────┘

编译命令示例:
  # 先编译Core
  colcon build --packages-up-to autoware_route_handler
  
  # 再编译Universe (自动处理依赖)
  colcon build --packages-select autoware_behavior_path_planner
```

## 8. 调试和监控架构

```
调试工具栈:

┌─────────────────────────────────────────┐
│         RViz2 可视化                     │
│  • Planning Markers                     │
│  • Debug Trajectory                     │
│  • Virtual Walls                        │
└─────────────┬───────────────────────────┘
              │
┌─────────────▼───────────────────────────┐
│      Planning Debug Topics               │
│  /planning/.../debug/*                  │
│  • stop_reasons                         │
│  • processing_time                      │
│  • planning_factors                     │
└─────────────┬───────────────────────────┘
              │
┌─────────────▼───────────────────────────┐
│      Planning Diagnostics                │
│  /diagnostics                           │
│  • Module Status                        │
│  • Performance Metrics                  │
└─────────────┬───────────────────────────┘
              │
┌─────────────▼───────────────────────────┐
│      ROS2 Tools                         │
│  • ros2 topic echo                      │
│  • ros2 param get/set                   │
│  • ros2 node info                       │
└─────────────────────────────────────────┘
```

---

## 使用建议

### 1. 查看实时模块状态
```bash
# 查看所有planning节点
ros2 node list | grep planning

# 查看behavior_path_planner状态
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/stop_reason
```

### 2. 分析性能瓶颈
```bash
# 查看各模块处理时间
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/processing_time

# 使用ros2 bag记录并分析
ros2 bag record /planning/.../debug/processing_time
```

### 3. 自定义模块集成
参考Universe的插件机制，在behavior_path_planner或behavior_velocity_planner中添加自定义模块。

---

**提示**: 
- 使用 `rqt_graph` 可以可视化节点和话题连接
- 使用 `ros2 run tf2_tools view_frames` 查看坐标变换关系
- 在RViz2中启用Planning相关的Marker可以直观看到规划过程

