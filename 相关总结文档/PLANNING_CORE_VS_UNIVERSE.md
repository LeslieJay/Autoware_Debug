# Autoware Planning: Core vs Universe 详细对比

## 📊 概览对比

| 维度 | autoware_core/planning | autoware_universe/planning |
|------|------------------------|----------------------------|
| **定位** | 核心基础组件 | 扩展功能模块 |
| **稳定性** | 高质量、高稳定性 | 实验性、持续演进 |
| **依赖关系** | 独立的基础库 | 依赖Core模块 |
| **更新频率** | 谨慎、经过充分测试 | 快速迭代、社区驱动 |
| **功能范围** | 基础规划能力 | 高级场景应用 |

---

## 🏗️ **1. autoware_core/planning - 核心基础层**

### **设计理念**
- **稳定的基础组件集**: 提供经过充分测试的、高质量的基础ROS包
- **最小依赖**: 作为其他模块的基础，尽量减少外部依赖
- **接口标准化**: 定义规划系统的核心接口和数据结构

### **核心模块列表**

```
/src/core/autoware_core/planning/
├── autoware_mission_planner              # 任务规划器（全局路径）
│   └── 功能：从起点到终点的高层路由规划
│
├── autoware_route_handler                # 路由处理器 ⭐核心依赖
│   └── 功能：路由信息管理、lanelet查询
│
├── autoware_velocity_smoother            # 速度平滑器 ⭐核心依赖
│   └── 功能：速度轮廓优化、加加速度约束
│
├── autoware_path_generator               # 路径生成器
│   └── 功能：基础路径生成、转向灯控制
│
├── autoware_planning_factor_interface    # 规划因子接口
│   └── 功能：统一的规划因子（停车原因等）接口
│
├── autoware_planning_topic_converter     # 话题转换器
│   └── 功能：Path和Trajectory的格式转换
│
├── behavior_velocity_planner/            # 行为速度规划器（核心版）
│   ├── autoware_behavior_velocity_planner
│   ├── autoware_behavior_velocity_planner_common  ⭐通用功能
│   └── autoware_behavior_velocity_stop_line_module
│
└── motion_velocity_planner/              # 运动速度规划器（核心版）
    ├── autoware_motion_velocity_planner
    ├── autoware_motion_velocity_planner_common     ⭐通用功能
    └── autoware_motion_velocity_obstacle_stop_module
```

### **关键特点**
1. ✅ **高度通用**: 适用于各种自动驾驶场景
2. ✅ **充分测试**: 经过大量实车测试验证
3. ✅ **文档完善**: 有详细的设计文档和API说明
4. ✅ **向后兼容**: API变更非常谨慎

---

## 🌌 **2. autoware_universe/planning - 扩展功能层**

### **设计理念**
- **功能扩展**: 在Core基础上添加高级规划功能
- **场景丰富**: 支持复杂的驾驶场景（换道、泊车、避障等）
- **社区驱动**: 快速集成最新研究成果和实用功能
- **模块化设计**: 用户可根据需求选择启用的模块

### **扩展模块列表**

```
/src/universe/autoware_universe/planning/
├── behavior_path_planner/                # 行为路径规划器 ⭐核心功能
│   ├── autoware_behavior_path_planner
│   ├── autoware_behavior_path_planner_common
│   ├── lane_change_module               # 换道模块
│   ├── goal_planner_module              # 目标规划（泊车）
│   ├── start_planner_module             # 起步规划
│   ├── static_obstacle_avoidance_module # 静态避障
│   ├── dynamic_obstacle_avoidance_module# 动态避障
│   ├── side_shift_module                # 侧向移动
│   └── sampling_planner_module          # 采样规划器
│
├── behavior_velocity_planner/            # 行为速度规划器（扩展版）
│   ├── crosswalk_module                 # 人行横道
│   ├── traffic_light_module             # 交通灯
│   ├── intersection_module              # 路口
│   ├── blind_spot_module                # 盲区
│   ├── detection_area_module            # 检测区域
│   ├── no_stopping_area_module          # 禁停区
│   ├── virtual_traffic_light_module     # 虚拟交通灯
│   ├── speed_bump_module                # 减速带
│   ├── run_out_module                   # 冲出检测
│   └── occlusion_spot_module            # 遮挡点
│
├── motion_velocity_planner/              # 运动速度规划器（扩展版）
│   ├── obstacle_cruise_module           # 障碍物巡航
│   ├── obstacle_slow_down_module        # 障碍物减速
│   ├── dynamic_obstacle_stop_module     # 动态障碍物停止
│   ├── obstacle_velocity_limiter_module # 障碍物速度限制
│   └── out_of_lane_module               # 车道外检测
│
├── autoware_freespace_planner            # 自由空间规划器
│   └── 功能：停车场等低速场景的路径规划
│
├── autoware_freespace_planning_algorithms# 自由空间算法库
│   └── A*/Hybrid A*/RRT*等算法
│
├── autoware_obstacle_cruise_planner      # 障碍物巡航规划器
│   └── 功能：自适应巡航控制(ACC)
│
├── autoware_path_optimizer               # 路径优化器
│   └── 功能：路径平滑、避障优化
│
├── autoware_path_smoother                # 路径平滑器
│   └── 功能：弹性带平滑等算法
│
├── sampling_based_planner/               # 采样规划器
│   └── autoware_path_sampler            # 路径采样器
│
├── autoware_scenario_selector            # 场景选择器
│   └── 功能：lane_driving/parking场景切换
│
├── autoware_planning_validator           # 规划验证器
│   └── 功能：轨迹合法性检查
│
└── autoware_surround_obstacle_checker    # 周边障碍物检查器
    └── 功能：低速时的近距离障碍物检测
```

### **关键特点**
1. 🚀 **功能丰富**: 涵盖各种复杂驾驶场景
2. 🔧 **可配置**: 模块可灵活启用/禁用
3. 🔬 **实验性**: 包含最新研究成果
4. 🌐 **社区活跃**: 持续更新和改进

---

## 🔗 **3. 依赖关系和交互**

### **依赖方向**

```
┌─────────────────────────────────────┐
│   autoware_core/planning            │
│   (基础层 - 不依赖Universe)          │
│                                     │
│  - route_handler                   │
│  - velocity_smoother                │
│  - mission_planner                  │
│  - planning_factor_interface        │
│  - behavior_velocity_planner_common│
│  - motion_velocity_planner_common  │
└─────────────────────────────────────┘
              ↑ 依赖
              │
┌─────────────────────────────────────┐
│  autoware_universe/planning          │
│  (扩展层 - 依赖Core的基础组件)       │
│                                     │
│  - behavior_path_planner ───────┐  │
│  - behavior_velocity_modules    │  │
│  - motion_velocity_modules      │  │
│  - freespace_planner ───────────┤  │
│  - obstacle_cruise_planner      │  │
│  - path_optimizer               │  │
│  - scenario_selector ───────────┘  │
│                                     │
│  这些模块都依赖Core提供的：          │
│  • autoware_route_handler           │
│  • autoware_planning_factor_interface│
│  • autoware_velocity_smoother (部分)│
└─────────────────────────────────────┘
```

### **实际依赖示例**

以`autoware_freespace_planner`为例（Universe模块）：

```xml
<!-- package.xml -->
<depend>autoware_route_handler</depend>  <!-- 来自Core -->
<depend>autoware_planning_factor_interface</depend>  <!-- 来自Core -->
<depend>autoware_vehicle_info_utils</depend>  <!-- 来自Core common -->
```

---

## 🔄 **4. 数据流和交互模式**

### **典型的Planning Pipeline**

```
1. Mission Planning (Core)
   └─→ 生成全局路由
        ↓
2. Scenario Selector (Universe)
   └─→ 选择场景：lane_driving / parking
        ↓
3a. Lane Driving分支:
    ├─→ Behavior Path Planner (Universe)
    │   └─→ 换道、避障等行为决策
    │        ↓
    ├─→ Behavior Velocity Planner (Core + Universe混合)
    │   └─→ 交通规则速度控制
    │        ↓
    ├─→ Motion Velocity Planner (Core + Universe混合)
    │   └─→ 障碍物速度规划
    │        ↓
    └─→ Velocity Smoother (Core)
        └─→ 最终速度平滑

3b. Parking分支:
    └─→ Freespace Planner (Universe)
        └─→ 停车场路径规划
             ↓
        └─→ Velocity Smoother (Core)

4. Planning Validator (Universe)
   └─→ 最终轨迹验证
```

### **关键交互点**

#### **1. Route Handler (Core提供，全局使用)**
```cpp
// Universe模块使用Core的route_handler
#include "autoware/route_handler/route_handler.hpp"

// 在behavior_path_planner中
planner_data->route_handler->getRouteLanelets();
planner_data->route_handler->getPreviousLanelets();
```

#### **2. Planning Factor Interface (Core定义，Universe实现)**
```cpp
// Core定义接口
namespace autoware::planning_factor_interface {
  class PlanningFactorInterface;
}

// Universe各模块使用
planning_factor_interface_->add(
  distance[0], distance[1], pose[0], pose[1], 
  PlanningFactor::STOP, SafetyFactorArray{});
```

#### **3. Velocity Smoother (Core提供，Universe调用)**
```cpp
// Universe的behavior_velocity_planner使用Core的velocity_smoother
planner_data->velocity_smoother_->applyLateralAccelerationFilter(...);
```

---

## 🎯 **5. 针对AGV的建议**

### **5.1 需要保留的Core模块**
```yaml
必需模块（不可删除）:
  - autoware_mission_planner          # 全局路由
  - autoware_route_handler            # 路由管理（其他模块依赖）
  - autoware_velocity_smoother        # 速度平滑（必需）
  - autoware_planning_factor_interface# 接口定义
  - autoware_planning_topic_converter # 格式转换
```

### **5.2 可选择性使用的Universe模块**

#### **室内AGV推荐配置**
```yaml
推荐启用:
  - behavior_path_planner:
      - static_obstacle_avoidance_module   # 静态避障
      - side_shift_module                  # 侧移避让
  
  - freespace_planner                      # 停车/倒车
  
  - motion_velocity_planner:
      - obstacle_stop_module               # 障碍物停止
      - obstacle_cruise_module             # 自适应巡航
  
  - scenario_selector                      # 场景切换
  - planning_validator                     # 轨迹验证

推荐禁用（室内不需要）:
  - behavior_velocity_planner:
      - traffic_light_module               # 交通灯
      - crosswalk_module                   # 人行横道
      - intersection_module                # 十字路口
      - blind_spot_module                  # 盲区检测
  
  - behavior_path_planner:
      - lane_change_module                 # 换道（室内无车道）
      - goal_planner_module                # 路边停车（如果不需要）
```

### **5.3 配置方法**

**修改 `default_preset.yaml`:**
```yaml
# autoware_launch/config/planning/preset/default_preset.yaml

- arg:
    name: launch_traffic_light_module
    default: "false"  # 室内关闭

- arg:
    name: launch_crosswalk_module
    default: "false"  # 室内关闭

- arg:
    name: launch_lane_change_right_module
    default: "false"  # 单车道关闭

- arg:
    name: launch_static_obstacle_avoidance
    default: "true"   # 保持开启
```

---

## 📈 **6. 版本演进策略**

### **Core的演进（保守策略）**
```
版本N → 版本N+1:
1. 充分的社区讨论和设计审查
2. 多轮测试验证
3. 向后兼容性保证
4. API变更需要deprecation周期
```

### **Universe的演进（激进策略）**
```
版本N → 版本N+1:
1. 快速集成新功能
2. 持续重构和优化
3. Breaking changes可以更频繁
4. 实验性功能可以先放入Universe
```

### **迁移路径**
```
Universe新功能 → 充分验证 → 稳定后考虑迁移到Core
                ↓
         (如果是基础功能且稳定)
```

---

## 🔍 **7. 实际代码依赖分析**

### **查看模块依赖**
```bash
# 查看Universe模块对Core的依赖
grep -r "autoware_route_handler" \
  src/universe/autoware_universe/planning/*/package.xml

# 输出示例：
# autoware_freespace_planner/package.xml:<depend>autoware_route_handler</depend>
# autoware_scenario_selector/package.xml:<depend>autoware_route_handler</depend>
# behavior_velocity_planner/.../package.xml:<depend>autoware_route_handler</depend>
```

### **编译时依赖关系**
```bash
# 必须先编译Core
colcon build --packages-up-to autoware_route_handler
colcon build --packages-up-to autoware_velocity_smoother

# 然后才能编译依赖它们的Universe模块
colcon build --packages-select autoware_behavior_path_planner
colcon build --packages-select autoware_freespace_planner
```

---

## 📝 **总结**

### **核心区别**
| 特性 | Core | Universe |
|------|------|----------|
| **角色** | 提供基础设施 | 构建应用功能 |
| **依赖** | 自给自足 | 依赖Core |
| **稳定性** | 生产级稳定 | 持续演进 |
| **更新** | 谨慎保守 | 快速迭代 |
| **选择** | 必须使用 | 按需选择 |

### **两者关系**
```
Core是地基 → Universe是建筑
Core是标准库 → Universe是第三方库
Core是内核 → Universe是应用层
```

### **对AGV开发者的建议**
1. ✅ **完整保留Core**: 所有Core模块都应该保留
2. ✅ **精选Universe**: 根据AGV场景选择需要的Universe模块
3. ✅ **理解依赖**: 了解你使用的Universe模块依赖哪些Core组件
4. ✅ **配置优化**: 禁用不需要的模块以提高性能
5. ✅ **自定义扩展**: 新功能可以参考Universe的架构进行扩展

---

## 🔧 **快速诊断命令**

```bash
# 1. 查看Core planning模块
ls src/core/autoware_core/planning/

# 2. 查看Universe planning模块
ls src/universe/autoware_universe/planning/

# 3. 查看某个Universe模块的依赖
cat src/universe/autoware_universe/planning/autoware_freespace_planner/package.xml | grep depend

# 4. 检查运行时加载的模块
ros2 node list | grep planning

# 5. 查看某个模块的参数
ros2 param list /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner
```

---

**文档版本**: 1.0  
**适用Autoware版本**: main分支  
**最后更新**: 2025-01-17

