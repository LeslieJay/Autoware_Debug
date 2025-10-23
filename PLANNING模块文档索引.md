# Autoware Planning 模块文档索引

欢迎查阅 Autoware Planning 模块的完整技术文档。本索引帮助您快速找到所需的信息。

## 📚 文档结构

```
Planning 模块文档
├── 📖 主文档
│   ├── PLANNING模块详细说明文档.md          (核心文档,必读)
│   ├── PLANNING模块代码注释说明.md          (注释规范和使用指南)
│   ├── PLANNING模块注释工作总结.md          (项目总结)
│   └── PLANNING模块文档索引.md              (本文件)
│
├── 📝 已有文档 (原有的调试文档)
│   ├── behavior_path_planner场景模块工作原理详解.md
│   ├── behavior_path_planner速度为零的原因分析.md
│   ├── BPP模块协作快速参考.md
│   ├── BPP模块协作详细实例.md
│   ├── BPP路径生成流程图.md
│   ├── BPP速度为零_快速诊断.md
│   ├── 速度调试日志已添加总结.md
│   └── 车辆状态对Planning速度的影响分析.md
│
└── 💻 已注释代码文件
    ├── mission_planner.hpp                  (任务规划器主类)
    └── arrival_checker.hpp                  (到达检查器)
```

---

## 🎯 快速导航

### 新手入门
如果您是第一次接触 Autoware Planning 模块,建议按以下顺序阅读:

1. **[PLANNING模块详细说明文档.md](./PLANNING模块详细说明文档.md)** 👈 **从这里开始!**
   - 第1章: 模块概述
   - 第2章: 整体架构
   
2. **[PLANNING模块代码注释说明.md](./PLANNING模块代码注释说明.md)**
   - 了解代码结构
   - 学习注释规范

3. **阅读已注释的源代码**
   - `mission_planner.hpp`
   - `arrival_checker.hpp`

### 调试和排错
如果您遇到了具体问题,可以参考:

1. **速度相关问题**
   - [PLANNING模块详细说明文档.md](./PLANNING模块详细说明文档.md) - 第7章 调试指南
   - [behavior_path_planner速度为零的原因分析.md](./behavior_path_planner速度为零的原因分析.md)
   - [BPP速度为零_快速诊断.md](./BPP速度为零_快速诊断.md)
   - [车辆状态对Planning速度的影响分析.md](./车辆状态对Planning速度的影响分析.md)

2. **参数配置**
   - [PLANNING模块详细说明文档.md](./PLANNING模块详细说明文档.md) - 第6章 配置参数说明

3. **模块协作**
   - [BPP模块协作快速参考.md](./BPP模块协作快速参考.md)
   - [BPP模块协作详细实例.md](./BPP模块协作详细实例.md)

### 深入研究
如果您想深入了解某个子模块:

1. **Mission Planner**
   - [PLANNING模块详细说明文档.md](./PLANNING模块详细说明文档.md) - 第3.1节
   - 源码: `src/core/autoware_core/planning/autoware_mission_planner/`

2. **Behavior Velocity Planner**
   - [PLANNING模块详细说明文档.md](./PLANNING模块详细说明文档.md) - 第3.4节
   - [behavior_path_planner场景模块工作原理详解.md](./behavior_path_planner场景模块工作原理详解.md)
   - 源码: `src/core/autoware_core/planning/behavior_velocity_planner/`

3. **Velocity Smoother**
   - [PLANNING模块详细说明文档.md](./PLANNING模块详细说明文档.md) - 第3.6节
   - 源码: `src/core/autoware_core/planning/autoware_velocity_smoother/`

---

## 📖 文档详细介绍

### 1. PLANNING模块详细说明文档.md

**文档类型**: 核心技术文档  
**字数**: 约 15,000 字  
**适合人群**: 所有 Planning 模块使用者

#### 章节概览

| 章节 | 内容 | 重要程度 |
|------|------|---------|
| 1. 模块概述 | Planning模块的功能定位 | ⭐⭐⭐⭐⭐ |
| 2. 整体架构 | 层次结构和数据流 | ⭐⭐⭐⭐⭐ |
| 3. 核心子模块详解 | 7个子模块的详细说明 | ⭐⭐⭐⭐⭐ |
| 4. 数据流分析 | 消息类型和转换关系 | ⭐⭐⭐⭐ |
| 5. 关键算法 | 核心算法原理 | ⭐⭐⭐⭐ |
| 6. 配置参数说明 | 所有参数的详细说明 | ⭐⭐⭐⭐⭐ |
| 7. 调试指南 | 实用调试技巧 | ⭐⭐⭐⭐⭐ |

#### 重点内容

- ✅ **完整的架构图**: 清晰展示模块间关系
- ✅ **算法详解**: 重新规划安全检查、速度优化等
- ✅ **参数表格**: 所有参数的含义和默认值
- ✅ **调试流程**: 常见问题的排查步骤
- ✅ **代码示例**: 关键算法的伪代码

#### 使用场景

```
场景1: 系统集成
  → 阅读 第1、2章 了解整体架构
  → 阅读 第6章 配置参数
  
场景2: 功能开发
  → 阅读 第3章 相关子模块
  → 阅读 第5章 理解算法
  → 查看 已注释的源代码
  
场景3: 问题调试
  → 阅读 第7章 调试指南
  → 参考 其他调试文档
```

---

### 2. PLANNING模块代码注释说明.md

**文档类型**: 注释规范和指南  
**字数**: 约 5,000 字  
**适合人群**: 代码贡献者和深度开发者

#### 主要内容

1. **注释规范**
   - Doxygen 格式说明
   - 文件头、类、函数、变量注释规范
   - 示例代码

2. **已注释文件列表**
   - `mission_planner.hpp` ✅
   - `arrival_checker.hpp` ✅

3. **关键代码解读**
   - Mission Planner 工作流程
   - 重新规划安全检查算法
   - 到达检查机制

4. **代码结构图**
   - MissionPlanner 类成员分组
   - 数据流图

5. **编译和使用**
   - 编译命令
   - Doxygen 文档生成

#### 使用场景

```
场景1: 阅读已注释代码
  → 了解注释格式
  → 查看类结构图
  → 理解算法流程
  
场景2: 添加新注释
  → 学习注释规范
  → 参考已有示例
  → 遵循命名规则
  
场景3: 生成API文档
  → 查看编译说明
  → 配置 Doxygen
  → 生成 HTML 文档
```

---

### 3. PLANNING模块注释工作总结.md

**文档类型**: 项目总结报告  
**字数**: 约 5,000 字  
**适合人群**: 项目管理者和后续维护者

#### 主要内容

1. **项目概述**
   - 工作目标和范围
   - 代码统计

2. **工作内容**
   - 模块架构分析 ✅
   - 文档编写 ✅
   - 代码注释 ✅

3. **交付成果**
   - 3份完整文档
   - 2个已注释文件
   - 15个图表

4. **技术亮点**
   - 深度技术分析
   - 实用调试指南
   - 参数调优指导

5. **后续建议**
   - 继续注释的优先级
   - 文档改进方向
   - 工具开发建议

#### 价值

- ✅ 记录项目完成情况
- ✅ 提供后续工作指引
- ✅ 总结经验和方法

---

## 🔍 常见问题快速查找

### Q1: 车辆不动或速度为零?

**解决方案路径**:
```
步骤1: 阅读 
  → PLANNING模块详细说明文档.md - 第7.3节 "问题1: 车辆速度为零"
  
步骤2: 运行诊断脚本
  → debug_zero_velocity.sh
  
步骤3: 查看详细分析
  → behavior_path_planner速度为零的原因分析.md
  → BPP速度为零_快速诊断.md
```

### Q2: 如何调整规划参数?

**解决方案路径**:
```
步骤1: 了解参数体系
  → PLANNING模块详细说明文档.md - 第6章 "配置参数说明"
  
步骤2: 查看参数文件位置
  → autoware_launch/config/planning/...
  
步骤3: 阅读调优指南
  → PLANNING模块详细说明文档.md - 第7.5节 "参数调优流程"
```

### Q3: 如何理解重新规划(reroute)机制?

**解决方案路径**:
```
步骤1: 阅读功能说明
  → PLANNING模块详细说明文档.md - 第3.1节 "Mission Planner"
  
步骤2: 查看算法详解
  → PLANNING模块代码注释说明.md - "重新规划安全检查算法"
  
步骤3: 阅读源码注释
  → mission_planner.hpp - check_reroute_safety() 函数
```

### Q4: 路径规划失败怎么办?

**解决方案路径**:
```
步骤1: 查看诊断步骤
  → PLANNING模块详细说明文档.md - 第7.3节 "问题2: 路径规划失败"
  
步骤2: 检查日志
  → ros2 topic echo /planning/mission_planning/state
  
步骤3: 验证地图和目标点
  → 确认目标点在地图车道上
```

### Q5: 如何添加新的场景模块?

**解决方案路径**:
```
步骤1: 理解插件架构
  → PLANNING模块详细说明文档.md - 第3.4节 "Behavior Velocity Planner"
  
步骤2: 参考现有模块
  → behavior_velocity_planner/autoware_behavior_velocity_stop_line_module
  
步骤3: 实现接口
  → 继承 SceneModuleInterface
  → 实现 modifyPathVelocity()
```

---

## 💡 学习路径建议

### 路径1: 快速上手 (3小时)

```
Hour 1: 理解整体架构
  ✓ PLANNING模块详细说明文档.md - 第1、2章
  ✓ 浏览架构图和数据流图
  
Hour 2: 了解核心模块
  ✓ PLANNING模块详细说明文档.md - 第3章(浏览)
  ✓ 重点关注自己需要的子模块
  
Hour 3: 实践和配置
  ✓ PLANNING模块详细说明文档.md - 第6章
  ✓ 运行调试脚本
  ✓ 查看 RViz 可视化
```

### 路径2: 深入学习 (1-2天)

```
Day 1 上午: 理论学习
  ✓ 完整阅读 PLANNING模块详细说明文档.md
  ✓ 理解所有子模块的功能
  
Day 1 下午: 源码阅读
  ✓ 阅读 mission_planner.hpp (已注释)
  ✓ 阅读 arrival_checker.hpp (已注释)
  ✓ 参考 PLANNING模块代码注释说明.md
  
Day 2 上午: 算法研究
  ✓ 重新规划安全检查算法
  ✓ 速度优化算法(OSQP)
  ✓ 碰撞检查算法
  
Day 2 下午: 实践调试
  ✓ 配置参数并测试
  ✓ 使用调试工具
  ✓ 解决实际问题
```

### 路径3: 专家级 (1-2周)

```
Week 1: 全面学习
  Day 1-2: 阅读所有文档
  Day 3-4: 阅读核心源码(Planning目录下主要文件)
  Day 5: 运行和调试实验

Week 2: 深度研究
  Day 1-2: 研究感兴趣的算法
  Day 3-4: 尝试修改和扩展
  Day 5: 总结和分享
```

---

## 🛠️ 工具和资源

### 调试脚本
```bash
# 速度调试
./debug_zero_velocity.sh        # 诊断速度为零问题
./debug_bpp_velocity.sh          # BPP速度调试
./monitor_velocity_debug.sh      # 监控速度变化

# 速度限制检查
./check_speed_limit.sh           # 检查速度限制设置
./compare_velocity_chain.sh      # 比较速度链变化
```

### RViz 可视化
```bash
# 路线可视化
/planning/mission_planning/mission_planner/debug/route_marker

# 路径可视化  
/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id

# 轨迹可视化
/planning/scenario_planning/trajectory

# 虚拟墙(停止原因)
/planning/.../virtual_wall/*
```

### ROS 命令
```bash
# 查看Planning状态
ros2 topic echo /planning/mission_planning/state

# 查看速度信息
ros2 topic echo /planning/scenario_planning/velocity_smoother/debug/closest_velocity

# 设置路线(示例)
ros2 service call /planning/mission_planning/mission_planner/set_waypoint_route ...
```

---

## 📞 获取帮助

### 文档相关
- 📖 首先查阅本文档索引,找到相关章节
- 🔍 使用文档内搜索功能(Ctrl+F)
- 💬 参考"常见问题快速查找"章节

### 技术问题
- 📚 查阅 [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/)
- 💻 查看 [Autoware GitHub](https://github.com/autowarefoundation/autoware)
- 💬 访问 [Autoware Discourse Forum](https://discourse.ros.org/c/autoware)

### 代码贡献
- 🔧 遵循本项目的注释规范
- 📝 参考已注释的代码示例
- 🚀 提交 Pull Request 前阅读 CONTRIBUTING.md

---

## 📊 文档更新记录

| 日期 | 版本 | 更新内容 | 作者 |
|------|------|---------|------|
| 2025-10-23 | 1.0 | 初始版本,完成核心文档和注释 | - |

---

## 🎉 总结

本套文档为 Autoware Planning 模块提供了:

✅ **15,000+ 字**的详细技术文档  
✅ **完整的架构**说明和数据流分析  
✅ **实用的调试**指南和参数配置说明  
✅ **高质量的代码注释**示例  
✅ **清晰的学习路径**和快速导航  

无论您是新手还是专家,都能从这些文档中获益。

祝您学习愉快! 🚗💨

---

**注**: 如果您在使用过程中发现文档有误或需要补充,欢迎提出反馈!


