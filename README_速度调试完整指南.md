# Autoware Planning 速度调试完整指南

## 📋 文档目录

本仓库包含完整的 Autoware Planning 模块速度调试资源，包括代码修改、调试脚本、文档说明等。

---

## 📚 核心文档列表

### 1. 速度调试相关

| 文档 | 说明 | 重要性 |
|------|------|--------|
| `速度调试日志已添加总结.md` | 已添加的调试日志总结，包含代码位置和使用方法 | ⭐⭐⭐ |
| `添加速度调试日志说明.md` | 详细的日志添加指南，包含所有模块的日志位置 | ⭐⭐⭐ |
| `车辆状态对Planning速度的影响分析.md` | 分析车辆状态（速度、加速度）如何影响规划速度 | ⭐⭐⭐ |
| `BPP速度为零_快速诊断.md` | BPP 速度为零问题的快速诊断流程 | ⭐⭐⭐ |
| `behavior_path_planner速度为零的原因分析.md` | 深度分析 BPP 速度为零的各种原因 | ⭐⭐ |

### 2. 消息和架构

| 文档 | 说明 | 重要性 |
|------|------|--------|
| `LaneletRoute消息详解.md` | 详细解释 LaneletRoute 消息及其作用 | ⭐⭐⭐ |
| `LaneletRoute快速参考.md` | LaneletRoute 的快速查询参考 | ⭐⭐ |
| `PLANNING_ARCHITECTURE_DIAGRAM.md` | Planning 模块的完整架构图 | ⭐⭐⭐ |

### 3. 配置相关

| 文档 | 说明 | 重要性 |
|------|------|--------|
| `lanelet地图速度限制参数说明.md` | Lanelet2 地图中速度限制的配置方法 | ⭐⭐⭐ |
| `AGV配置快速指南.md` | 室内 AGV 的完整配置指南 | ⭐⭐⭐ |
| `E2E仿真器AGV使用指南.md` | e2e_simulator.launch.xml 的使用说明 | ⭐⭐ |

### 4. 模块详解

| 文档 | 说明 | 重要性 |
|------|------|--------|
| `behavior_path_planner场景模块工作原理详解.md` | BPP 各场景模块的详细工作原理 | ⭐⭐ |
| `BPP模块协作详细实例.md` | BPP 模块协作的实例分析 | ⭐⭐ |
| `近距离目标点规划问题分析.md` | 近距离目标点导致速度为零的分析 | ⭐⭐ |

---

## 🛠️ 调试脚本

### 已创建的脚本

| 脚本 | 功能 | 使用场景 |
|------|------|---------|
| `monitor_velocity_debug.sh` | 实时监控速度调试日志 | 查看各模块的速度日志 |
| `compare_velocity_chain.sh` | 对比速度传递链 | 追踪速度在各模块间的变化 |
| `rebuild_velocity_debug.sh` | 重新编译修改的模块 | 应用代码修改 |
| `check_speed_limit.sh` | 检查速度限制配置 | 诊断速度限制问题 |
| `debug_bpp_velocity.sh` | BPP 速度调试专用脚本 | 快速诊断 BPP 速度 |

### 使用方法

```bash
# 1. 监控所有速度日志
./monitor_velocity_debug.sh

# 2. 对比速度传递链
./compare_velocity_chain.sh

# 3. 检查速度限制
./check_speed_limit.sh

# 4. 调试 BPP 速度
./debug_bpp_velocity.sh

# 5. 重新编译
./rebuild_velocity_debug.sh
```

---

## 🔧 代码修改位置

### 已添加调试日志的文件

| 文件 | 模块 | 修改内容 |
|------|------|---------|
| `behavior_path_planner_node.cpp` | BPP 主节点 | 输出路径发布日志 |
| `goal_planner_module.cpp` | Goal Planner | 速度决策日志 |
| `static_obstacle_avoidance_module.cpp` | 避障模块 | 避障速度插入日志 |
| `autoware_obstacle_cruise_planner/src/node.cpp` | 障碍物巡航 | 输入输出速度日志 |

### 日志标记格式

所有日志都使用统一格式：
```
[VEL_DEBUG][模块名][阶段] 描述信息
```

示例：
```
[VEL_DEBUG][BPP][PUBLISH] Output path: points=150, first_vel=1.234 m/s
[VEL_DEBUG][GoalPlanner][DECIDE] current_vel=1.200, decided_vel=1.200
[VEL_DEBUG][Avoidance][INSERT] v_max=1.500, ego_speed=1.200
[VEL_DEBUG][ObstacleCruise][PUBLISH] Input_vel=1.500 -> Output_vel=0.800
```

---

## 🚀 快速开始

### 1. 准备工作

```bash
# 进入 Autoware 根目录
cd /path/to/autoware

# 确保环境已 source
source install/setup.bash
```

### 2. 重新编译（如果修改了代码）

```bash
# 使用提供的脚本
./rebuild_velocity_debug.sh

# 或手动编译
colcon build --packages-select \
  autoware_behavior_path_planner \
  autoware_behavior_path_goal_planner_module \
  autoware_behavior_path_static_obstacle_avoidance_module \
  autoware_obstacle_cruise_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 重新 source
source install/setup.bash
```

### 3. 启动 Autoware

```bash
# 根据您的环境启动
ros2 launch autoware_launch ...
```

### 4. 开始调试

**终端1 - 监控日志**:
```bash
./monitor_velocity_debug.sh
```

**终端2 - 速度链对比**:
```bash
./compare_velocity_chain.sh
```

**终端3 - 设置目标点**:
```bash
# 在 RViz 中设置目标点
# 或使用命令行
ros2 topic pub /planning/mission_planning/goal ...
```

---

## 🔍 调试流程

### 标准调试流程

```
1. 观察现象
   └→ 速度为零？速度过低？速度异常？

2. 查看速度传递链
   └→ ./compare_velocity_chain.sh
   └→ 找出哪个模块导致速度变化

3. 查看详细日志
   └→ ./monitor_velocity_debug.sh
   └→ 选择对应模块查看详细信息

4. 检查配置
   └→ 检查 lanelet 速度限制
   └→ 检查 planning 参数配置
   └→ 检查车辆状态

5. 定位根本原因
   └→ 参考相关文档分析
   └→ 修改配置或代码

6. 验证修复
   └→ 重新测试
   └→ 确认速度正常
```

---

## 📊 典型问题诊断

### 问题1: 速度始终为零

**诊断步骤**:

1. 检查 LaneletRoute
   ```bash
   ros2 topic echo --once /planning/mission_planning/route
   ```

2. 查看 BPP 输出
   ```bash
   ros2 topic echo /rosout | grep "\[VEL_DEBUG\]\[BPP\]"
   ```

3. 检查速度限制
   ```bash
   ./check_speed_limit.sh
   ```

4. 查看详细分析
   - 参考: `BPP速度为零_快速诊断.md`
   - 参考: `近距离目标点规划问题分析.md`

### 问题2: 速度突然降低

**诊断步骤**:

1. 查看障碍物巡航日志
   ```bash
   ros2 topic echo /rosout | grep "\[VEL_DEBUG\]\[ObstacleCruise\]"
   ```

2. 检查避障日志
   ```bash
   ros2 topic echo /rosout | grep "\[VEL_DEBUG\]\[Avoidance\]"
   ```

3. 查看速度链对比
   ```bash
   ./compare_velocity_chain.sh
   ```

### 问题3: 速度不符合预期

**诊断步骤**:

1. 检查 lanelet 速度限制
   ```bash
   grep "speed_limit" /path/to/lanelet2_map.osm
   ```

2. 查看规划参数
   ```bash
   ros2 param list /behavior_path_planner | grep velocity
   ros2 param list /obstacle_cruise_planner | grep velocity
   ```

3. 参考配置文档
   - `lanelet地图速度限制参数说明.md`
   - `AGV配置快速指南.md`

---

## 📖 学习路径

### 新手入门

1. **了解架构** → `PLANNING_ARCHITECTURE_DIAGRAM.md`
2. **理解 Route** → `LaneletRoute消息详解.md`
3. **快速配置** → `AGV配置快速指南.md`

### 深入调试

1. **添加日志** → `添加速度调试日志说明.md`
2. **速度影响** → `车辆状态对Planning速度的影响分析.md`
3. **问题诊断** → `BPP速度为零_快速诊断.md`

### 高级优化

1. **模块协作** → `behavior_path_planner场景模块工作原理详解.md`
2. **参数调优** → `AGV配置快速指南.md`
3. **地图配置** → `lanelet地图速度限制参数说明.md`

---

## 🎯 关键话题列表

### Planning 相关

| 话题 | 类型 | 说明 |
|------|------|------|
| `/planning/mission_planning/route` | LaneletRoute | 全局路径 |
| `/planning/scenario_planning/lane_driving/behavior_planning/path` | PathWithLaneId | BPP 输出路径 |
| `/planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner/trajectory` | Trajectory | 巡航规划输出 |
| `/planning/scenario_planning/trajectory` | Trajectory | 最终轨迹 |

### 车辆状态

| 话题 | 类型 | 说明 |
|------|------|------|
| `/localization/kinematic_state` | Odometry | 车辆速度和位置 |
| `/localization/acceleration` | AccelWithCovarianceStamped | 车辆加速度 |

### 控制命令

| 话题 | 类型 | 说明 |
|------|------|------|
| `/control/command/control_cmd` | Control | 控制命令 |

---

## 🔬 调试技巧

### 实时监控命令

```bash
# 1. 监控所有速度调试日志
ros2 topic echo /rosout | grep "\[VEL_DEBUG\]"

# 2. 查看车辆当前速度
ros2 topic echo /localization/kinematic_state/twist/twist/linear/x

# 3. 查看规划速度
ros2 topic echo /planning/scenario_planning/trajectory | grep "longitudinal_velocity_mps" | head -5

# 4. 查看控制命令速度
ros2 topic echo /control/command/control_cmd/longitudinal/velocity

# 5. 监控话题频率
ros2 topic hz /planning/scenario_planning/trajectory
```

### 日志过滤技巧

```bash
# 仅查看 BPP 日志
ros2 topic echo /rosout | grep "\[VEL_DEBUG\]\[BPP\]"

# 仅查看速度修改日志
ros2 topic echo /rosout | grep "\[VEL_DEBUG\].*MODIFY"

# 查看所有 Planning 节点的错误
ros2 topic echo /rosout | grep "planning.*ERROR"

# 保存日志到文件
ros2 topic echo /rosout | grep "\[VEL_DEBUG\]" > velocity_debug_$(date +%Y%m%d_%H%M%S).log
```

---

## 📝 调试记录

### 建议记录内容

创建 `调试过程及结果记录.md`，记录：

1. **问题描述**
   - 现象
   - 复现步骤
   - 预期行为

2. **调试过程**
   - 执行的命令
   - 观察到的日志
   - 尝试的解决方案

3. **解决方案**
   - 根本原因
   - 修改内容
   - 验证结果

4. **经验总结**
   - 学到的知识
   - 避免的坑
   - 优化建议

---

## ✅ 检查清单

### 环境检查

- [ ] Autoware 已正确安装
- [ ] 所有依赖包已安装
- [ ] 环境已 source (`source install/setup.bash`)
- [ ] ROS 2 正常运行

### 代码检查

- [ ] 调试日志已添加到关键位置
- [ ] 代码已重新编译
- [ ] 编译无错误和警告
- [ ] 日志格式统一

### 配置检查

- [ ] Lanelet2 地图已准备
- [ ] 地图包含速度限制属性
- [ ] Planning 参数已配置
- [ ] 车辆和传感器参数已设置

### 运行检查

- [ ] Autoware 成功启动
- [ ] 所有必需节点正在运行
- [ ] 调试日志正常输出
- [ ] 监控脚本可用

---

## 🆘 获取帮助

### 问题排查顺序

1. 查看相关文档
2. 检查日志输出
3. 运行诊断脚本
4. 查阅源代码
5. 搜索 GitHub Issues
6. 询问社区

### 有用的资源

- **Autoware 官方文档**: https://autowarefoundation.github.io/autoware-documentation/
- **Autoware GitHub**: https://github.com/autowarefoundation/autoware.universe
- **ROS 2 文档**: https://docs.ros.org/en/humble/

---

## 🎉 总结

本指南提供了：

✅ **完整的调试文档** - 涵盖各个方面  
✅ **实用的调试脚本** - 快速定位问题  
✅ **详细的代码修改** - 添加调试日志  
✅ **系统的诊断流程** - 结构化排查  
✅ **丰富的参考资料** - 深入学习

**祝您调试顺利，速度问题迎刃而解！** 🚀

---

**最后更新**: 2025-10-22  
**维护者**: LESLIE  
**版本**: v1.0

