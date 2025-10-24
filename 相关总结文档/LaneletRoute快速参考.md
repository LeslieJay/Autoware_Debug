# LaneletRoute 快速参考

## 📋 消息结构一览

```
LaneletRoute
├── header (时间戳和坐标系)
├── start_pose (起点位姿)
├── goal_pose (终点位姿)
├── segments[] ⭐ (路径段序列 - 最重要)
│   └── LaneletSegment
│       ├── preferred_primitive (推荐车道)
│       │   ├── id (Lanelet ID)
│       │   └── primitive_type (类型)
│       └── primitives[] (所有可选车道)
├── uuid (唯一标识)
└── allow_modification (是否允许修改)
```

---

## 🎯 核心作用

| 功能 | 说明 |
|------|------|
| **全局路径定义** | 从起点到终点的车道序列 |
| **速度限制来源** | 每个 lanelet 包含速度限制 |
| **路径约束** | 限制车辆在指定车道内行驶 |
| **换道信息** | 提供可选的相邻车道 |

---

## 📊 关键字段速查

### segments (最重要！)

```yaml
segments:
  - preferred_primitive:
      id: 1001              # Lanelet ID
      primitive_type: "lane"
    primitives:
      - id: 1001
      - id: 1002           # 可选车道
```

**用途**:
- ✅ 定义行驶路径
- ✅ 提供速度限制 (从 lanelet 属性)
- ✅ 支持换道决策

### allow_modification

```yaml
allow_modification: true   # 允许避障/换道
allow_modification: false  # 严格按路径行驶
```

**场景**:
- `true`: 普通道路，开阔区域
- `false`: 窄通道，停车入库，货架区

---

## 🔄 消息流转

```
用户设置目标点 (RViz)
         ↓
Mission Planner (全局路径规划)
         ↓
    LaneletRoute ← 发布到话题
         ↓
Behavior Path Planner (订阅并使用)
         ↓
生成详细路径 (PathWithLaneId)
```

**话题名称**: `/planning/mission_planning/route`

---

## 🛠️ 常用命令

### 查看当前路径

```bash
# 完整消息
ros2 topic echo /planning/mission_planning/route

# 仅查看 segments
ros2 topic echo /planning/mission_planning/route | grep -A10 segments

# 查看起点和终点
ros2 topic echo /planning/mission_planning/route | grep -A3 "start_pose\|goal_pose"
```

### 检查路径有效性

```bash
# 检查是否有 segments
ros2 topic echo --once /planning/mission_planning/route | grep -c "id:"

# 监控路径更新频率
ros2 topic hz /planning/mission_planning/route

# 查看 UUID (路径是否更新)
ros2 topic echo --once /planning/mission_planning/route | grep -A1 uuid
```

---

## 🐛 速度相关故障排查

### 问题: 车辆速度为零

**检查步骤**:

1. **检查 Route 是否存在**
   ```bash
   ros2 topic echo --once /planning/mission_planning/route
   ```
   - 如果没有输出 → Mission Planner 未生成路径

2. **检查 segments 是否为空**
   ```bash
   ros2 topic echo --once /planning/mission_planning/route | grep -c "id:"
   ```
   - 如果结果为 0 → 起点/终点不在有效 lanelet 上

3. **检查 lanelet 速度限制**
   ```bash
   # 查看地图中 lanelet 的属性
   # 在 lanelet2_map.osm 中搜索:
   grep "speed_limit" lanelet2_map.osm
   ```

4. **查看日志中的路径信息**
   ```bash
   ros2 topic echo /rosout | grep -i "route\|lanelet"
   ```

---

## 📍 与其他消息的关系

```
LaneletRoute (全局路径)
    ↓
PathWithLaneId (详细路径点)
    ↓
Trajectory (带速度的轨迹)
    ↓
Control Command (控制命令)
```

---

## 🎨 RViz 可视化

**添加显示**:
1. Add → By topic
2. 选择 `/planning/mission_planning/route`
3. 类型: `LaneletRoute`

**显示内容**:
- 路径的 lanelet 序列
- 起点和终点标记
- 可选车道（如果有）

---

## 💡 AGV 使用建议

### 室内 AGV 配置

```yaml
# 开阔区域
allow_modification: true    # 允许动态避障

# 货架区/窄通道
allow_modification: false   # 禁止偏离路径

# 速度设置
# 在 lanelet 地图中为每个区域设置合适的速度:
# - 主通道: 1.5 m/s
# - 转弯区: 0.8 m/s
# - 货架区: 0.5 m/s
# - 停车区: 0.3 m/s
```

### 调试技巧

```bash
# 1. 检查当前是否有路径
ros2 topic echo --once /planning/mission_planning/route > route.txt

# 2. 提取所有 lanelet ID
grep "id:" route.txt | awk '{print $2}'

# 3. 在地图中查找这些 lanelet
for id in $(grep "id:" route.txt | awk '{print $2}'); do
  grep "id=\"$id\"" lanelet2_map.osm
done
```

---

## 📚 详细文档

- **完整解释**: `LaneletRoute消息详解.md`
- **速度限制配置**: `lanelet地图速度限制参数说明.md`
- **Planning 架构**: `PLANNING_ARCHITECTURE_DIAGRAM.md`

---

## ✅ 检查清单

调试速度问题时，确认：

- [ ] Route 消息存在且有效
- [ ] segments 数组非空
- [ ] 每个 lanelet ID 在地图中存在
- [ ] Lanelet 有速度限制属性
- [ ] 起点和终点在有效 lanelet 上
- [ ] 路径连通（相邻 lanelet 连接）
- [ ] allow_modification 设置正确

---

## 🚀 快速测试

```bash
# 完整测试脚本
echo "=== LaneletRoute 健康检查 ==="
echo ""

echo "1. 检查 Route 是否存在..."
ros2 topic echo --once /planning/mission_planning/route &> /dev/null && echo "✅ Route 存在" || echo "❌ Route 不存在"

echo ""
echo "2. 检查 segments 数量..."
count=$(ros2 topic echo --once /planning/mission_planning/route 2>/dev/null | grep -c "id:")
echo "   Segments: $count"

echo ""
echo "3. 检查 allow_modification..."
ros2 topic echo --once /planning/mission_planning/route 2>/dev/null | grep "allow_modification"

echo ""
echo "=== 检查完成 ==="
```

保存为 `check_route.sh` 并运行：
```bash
chmod +x check_route.sh
./check_route.sh
```

---

祝调试顺利！🎯

