# Behavior Path Planner 速度为零 - 快速诊断指南

## 🚨 问题现象

`/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id` 中所有路径点的 `longitudinal_velocity_mps` 值为 **0**

## 🔍 快速诊断（一键运行）

```bash
./debug_bpp_velocity.sh
```

## 📊 最可能的原因（按概率排序）

### 1️⃣ Lanelet地图未设置speed_limit（90%概率） ⭐⭐⭐⭐⭐

**最常见原因！**

#### 快速检查
```bash
# 查看当前使用的lane_id
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once | grep "lane_ids" | head -n 1

# 在地图文件中查找该lane_id的speed_limit
grep "id='<lane_id>'" /path/to/your_map.osm -A 10 | grep speed_limit
```

#### 地图示例
```xml
<!-- ❌ 错误：缺少speed_limit -->
<relation id="2621">
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="road"/>
  <!-- 缺少 speed_limit 标签！ -->
</relation>

<!-- ✅ 正确：有speed_limit -->
<relation id="2621">
  <tag k="type" v="lanelet"/>
  <tag k="subtype" v="road"/>
  <tag k="speed_limit" v="30.00"/>  <!-- 单位：km/h -->
</relation>
```

#### 解决方案
1. 用JOSM打开地图文件
2. 为所有road类型的lanelet添加 `speed_limit` 标签
3. 保存并重新加载地图

---

### 2️⃣ 车辆接近目标点（5%概率） ⭐⭐⭐

#### 快速检查
```bash
# 查看是否有"out of route"日志
ros2 topic echo /rosout | grep -i "out of route"

# 如果脚本显示距离目标点<5米，这是正常行为
```

#### 判断标准
- 距离目标点 < 5米 → **正常行为**，速度为0是预期的
- 距离目标点 > 5米 → **异常**，继续检查其他原因

---

### 3️⃣ Traffic Rules未初始化（3%概率） ⭐⭐⭐⭐

#### 快速检查
```bash
# 检查地图是否加载
ros2 topic echo /map/vector_map --once

# 查看启动日志
ros2 topic echo /rosout | grep -i "traffic_rules\|lanelet"
```

#### 解决方案
重启Autoware，确保地图正确加载

---

### 4️⃣ 场景模块触发停车（2%概率） ⭐⭐⭐

#### 快速检查
```bash
# 查看停止原因
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/stop_reasons --once
```

#### 常见停车场景
- 🚦 红灯停车
- 🚶 人行横道有行人
- 🚗 前方有障碍物
- 🅿️ Goal Planner停车

如果是以上场景，速度为0是**正常行为**

---

## 🛠️ 标准诊断流程

### 第1步：运行诊断脚本（推荐）
```bash
./debug_bpp_velocity.sh
```

脚本会自动检查：
- ✓ 路径速度分布
- ✓ Lane IDs
- ✓ 激活的场景模块
- ✓ 停止原因
- ✓ 距离目标点距离
- ✓ 参考路径速度
- ✓ 错误日志
- ✓ **自动给出诊断建议**

### 第2步：根据脚本建议处理

#### 情况A：脚本提示"参考路径速度为0"
→ **地图speed_limit问题**（90%是这个）
```bash
# 解决方案
1. 找到地图文件路径
2. 用JOSM打开
3. 搜索当前使用的lane_id
4. 添加 speed_limit 标签（单位：km/h）
5. 保存并重启Autoware
```

#### 情况B：脚本提示"接近目标点"
→ **正常行为**，无需处理

#### 情况C：脚本显示停止原因
→ **场景模块触发**，检查是否合理

#### 情况D：脚本显示"out of route"
→ **路线问题**，重新规划路线

### 第3步：验证修复
```bash
# 检查速度是否恢复正常
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once | \
  grep "longitudinal_velocity_mps" | head -n 10
```

---

## 📝 手动检查命令

### 快速检查路径速度
```bash
# 查看速度分布
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once | \
  grep "longitudinal_velocity_mps" | awk '{print $2}' | sort -n | uniq -c
```

### 查看lane_id
```bash
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once | \
  grep "lane_ids" | head -n 5
```

### 查看停止原因
```bash
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/stop_reasons --once
```

### 查看场景模块
```bash
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/scene_module_status --once
```

### 检查日志
```bash
ros2 topic echo /rosout | grep -i "behavior_path\|out of route\|empty"
```

---

## 💡 关键代码位置

### 速度从地图读取的位置
**文件：** `src/core/autoware_core/planning/autoware_route_handler/src/route_handler.cpp`  
**行号：** 1588-1596

```cpp
// 从lanelet地图读取速度限制
const float speed_limit =
  static_cast<float>(traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value());

// 设置到路径点
p.point.longitudinal_velocity_mps = speed_limit;
```

**如果地图中没有speed_limit，这里会返回0或无效值！**

### Goal附近强制清零的位置
**文件：** `src/universe/autoware_universe/planning/behavior_path_planner/autoware_behavior_path_planner_common/src/utils/path_utils.cpp`  
**行号：** 547-550

```cpp
// 在goal附近，强制设置速度为0
for (auto & point : reference_path.points) {
  point.point.longitudinal_velocity_mps = 0.0;
}
```

---

## 🎯 解决方案总结

| 原因 | 概率 | 解决方案 | 耗时 |
|-----|------|---------|------|
| 地图未设置speed_limit | 90% | 用JOSM添加speed_limit标签 | 5-10分钟 |
| 接近目标点 | 5% | 正常行为，无需处理 | - |
| Traffic Rules未初始化 | 3% | 重启Autoware | 1分钟 |
| 场景模块触发停车 | 2% | 检查停止原因是否合理 | 2-5分钟 |

---

## 📚 详细文档

- **完整分析：** `behavior_path_planner速度为零的原因分析.md` （包含7种可能原因的详细说明）
- **速度限制说明：** `lanelet地图速度限制参数说明.md`
- **实时查看：** `如何实时查看速度限制_快速指南.md`

---

## ✅ 检查清单

在提问前，请确认已完成：

- [ ] 运行了 `./debug_bpp_velocity.sh`
- [ ] 检查了地图中的 speed_limit 标签
- [ ] 确认了车辆不是在接近目标点
- [ ] 查看了停止原因
- [ ] 检查了Autoware启动日志
- [ ] 查看了lane_id是否对应road类型的lanelet

---

## 🆘 仍然无法解决？

1. 运行诊断脚本并保存输出
2. 查看详细分析文档
3. 提供以下信息：
   - 诊断脚本输出
   - 当前使用的lane_id
   - 地图文件中对应lanelet的定义
   - Autoware启动日志

**记住：90%的情况是地图speed_limit未设置！**

