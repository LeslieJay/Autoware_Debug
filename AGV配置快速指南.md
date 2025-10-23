# 室内AGV配置快速指南

## 🎯 核心问题解答

### Q1: 目标点太近会有影响吗？

**答：是的，有重大影响！**

```
┌─────────────────────────────────────────────────────────┐
│ 问题                                                     │
├─────────────────────────────────────────────────────────┤
│ ❌ 路径规划失败（距离 < minimum_request_length）        │
│ ❌ 无法安全减速（距离 < 减速所需距离）                  │
│ ❌ 目标点附近速度强制为0                                │
│ ❌ Goal Planner无法激活                                 │
└─────────────────────────────────────────────────────────┘
```

### Q2: 会导致速度为0吗？

**答：会！主要原因：**

1. **目标点本身速度必须为0** （代码强制）
2. **目标点附近处于减速区**
3. **距离太短，整个路径都在减速**
4. **没有有效车道信息时返回零速度路径**

### Q3: 最小安全距离是多少？

```
┌────────────────┬──────────────┬──────────────┐
│ 速度           │ 最小距离     │ 推荐距离     │
├────────────────┼──────────────┼──────────────┤
│ 静止（0 m/s）  │ 3 m          │ 5 m          │
│ 慢速（1 m/s）  │ 5 m          │ 8 m          │
│ 中速（2 m/s）  │ 8 m          │ 12 m         │
│ 快速（5 m/s）  │ 25 m         │ 35 m         │
└────────────────┴──────────────┴──────────────┘

计算公式：
  最小距离 = v² / (2×a) + 缓冲距离
  
示例（AGV: v=2m/s, a=0.5m/s²）:
  最小距离 = 2² / (2×0.5) + 2 = 4 + 2 = 6m
  推荐距离 = 6 × 2 = 12m
```

---

## ⚡ 快速配置（3分钟）

### 步骤1: 修改路径长度 ⚠️ 最重要

**文件：** `behavior_path_planner.param.yaml`

```yaml
forward_path_length: 30.0      # 默认300.0 → 30.0
backward_path_length: 2.0      # 默认5.0 → 2.0
```

### 步骤2: 设置最小距离 ⚠️ 必须设置

**文件：** `goal_planner.param.yaml`

```yaml
pull_over:
  minimum_request_length: 5.0  # 默认0.0 → 5.0 ⚠️ 关键！
  pull_over_prepare_length: 15.0  # 默认100.0 → 15.0
  decide_path_distance: 3.0    # 默认10.0 → 3.0
```

### 步骤3: 降低速度

**文件：** `goal_planner.param.yaml`

```yaml
pull_over:
  pull_over_velocity: 0.8       # 默认3.0 → 0.8
  pull_over_minimum_velocity: 0.5  # 默认1.38 → 0.5
```

### 步骤4: 提高精度

**文件：** `goal_planner.param.yaml`

```yaml
th_arrived_distance: 0.3        # 默认1.0 → 0.3
```

### 步骤5: 禁用不需要的模块

**文件：** `default_preset.yaml`

```yaml
launch_static_obstacle_avoidance: "false"
launch_lane_change_left: "false"
launch_lane_change_right: "false"
launch_dynamic_obstacle_avoidance_module: "false"
```

---

## 📊 参数速查表

### 核心参数（必改）

| 参数 | 默认值 | AGV推荐 | 说明 |
|-----|--------|---------|------|
| `forward_path_length` | 300.0 m | **30.0 m** | 向前路径长度 |
| `minimum_request_length` | 0.0 m | **5.0 m** | 最小请求长度 ⚠️ |
| `th_arrived_distance` | 1.0 m | **0.3 m** | 到达判断距离 |
| `pull_over_velocity` | 3.0 m/s | **0.8 m/s** | 接近速度 |

### 距离参数（重要）

| 参数 | 默认值 | AGV推荐 | 影响 |
|-----|--------|---------|------|
| `backward_path_length` | 5.0 m | 2.0 m | 向后路径 |
| `pull_over_prepare_length` | 100.0 m | 15.0 m | 准备距离 |
| `decide_path_distance` | 10.0 m | 3.0 m | 决定距离 |
| `forward_goal_search_length` | 40.0 m | 10.0 m | 目标搜索前向 |
| `backward_goal_search_length` | 20.0 m | 5.0 m | 目标搜索后向 |

### 动力学参数

| 参数 | 默认值 | AGV推荐 | 说明 |
|-----|--------|---------|------|
| `maximum_deceleration` | 1.0 m/s² | 0.5 m/s² | 最大减速度 |
| `maximum_jerk` | 1.0 m/s³ | 0.5 m/s³ | 最大加加速度 |
| `pull_over_minimum_velocity` | 1.38 m/s | 0.5 m/s | 最小速度 |

### 精度参数

| 参数 | 默认值 | AGV推荐 | 说明 |
|-----|--------|---------|------|
| `goal_search_interval` | 2.0 m | 0.5 m | 搜索间隔 |
| `input_path_interval` | 2.0 m | 0.5 m | 输入路径间隔 |
| `output_path_interval` | 2.0 m | 0.5 m | 输出路径间隔 |
| `lateral_offset_interval` | 0.5 m | 0.2 m | 横向偏移间隔 |

---

## 🔍 问题诊断

### 现象1: "Goal Planner不执行"

```bash
# 检查
ros2 topic echo /.../scene_module_status | grep goal_planner

# 可能原因：
❌ 距离太近 (< minimum_request_length)
❌ 距离太远 (> forward_goal_search_length)
❌ 目标点在当前位置后方
❌ 目标点不在可达车道

# 解决：
✅ 增加 minimum_request_length: 5.0
✅ 确保目标点距离 5-40m
✅ 检查目标点方向
```

### 现象2: "路径规划失败"

```bash
# 检查日志
ros2 topic echo /planning/behavior_planning/path_with_lane_id

# 可能原因：
❌ forward_path_length 太长（300m）
❌ 减速距离不足
❌ 无有效车道信息

# 解决：
✅ 缩短 forward_path_length: 30.0
✅ 降低速度或增加目标距离
✅ 检查Lanelet地图
```

### 现象3: "速度为0"

```bash
# 检查速度
ros2 topic echo /planning/behavior_planning/path_with_lane_id | \
  grep "longitudinal_velocity_mps" | head -20

# 可能原因：
❌ 目标点附近（< 5m）
❌ 地图无速度限制
❌ 路径太短，全程减速
❌ Goal Planner设置了停车路径

# 解决：
✅ 增加目标距离
✅ 检查地图speed_limit
✅ 增加 forward_path_length
✅ 查看 pull_over_velocity 设置
```

### 现象4: "车辆不停止/超过目标点"

```bash
# 可能原因：
❌ th_arrived_distance 太小（< 定位精度）
❌ th_stopped_velocity 太小
❌ th_stopped_time 太长

# 解决：
✅ 增加 th_arrived_distance: 0.3-0.5
✅ 检查定位精度（建议 ≥ th_arrived_distance × 0.5）
✅ 减少 th_stopped_time: 1.0
```

---

## 🧪 测试流程

### 1. 基础测试（静止起步）

```bash
# 设置：车辆静止
# 目标：10m前方

# 预期：
✅ Goal Planner激活
✅ 路径平滑
✅ 速度从0加速到pull_over_velocity
✅ 在目标点附近减速到0
✅ 停止在th_arrived_distance范围内
```

### 2. 动态测试（移动中）

```bash
# 设置：车辆2 m/s移动
# 目标：15m前方

# 预期：
✅ 路径实时更新
✅ 平滑减速
✅ 无突变
✅ 准确停止
```

### 3. 近距离测试

```bash
# 设置：车辆静止
# 目标：逐步缩短距离

测试距离：15m → 10m → 7m → 5m → 3m

记录最小成功距离：_____m

# 如果 < 5m失败：
→ 检查 minimum_request_length 设置
→ 降低 pull_over_velocity
→ 减小 maximum_deceleration
```

### 4. 精度测试

```bash
# 多次测试，记录停止位置误差

目标位置：(x, y) = (10.0, 0.0)

测试10次，记录实际停止位置：
1. (x, y) = (_____, _____)  误差 = _____
2. (x, y) = (_____, _____)  误差 = _____
...

平均误差：_____ m
最大误差：_____ m

# 如果误差 > th_arrived_distance:
→ 增加 th_arrived_distance
→ 检查定位系统精度
→ 考虑动态调整（根据定位协方差）
```

---

## 📝 配置模板（复制使用）

### 小型AGV（体积小，低速）

```yaml
# 适用：体积 < 1m², 速度 < 1 m/s
forward_path_length: 20.0
backward_path_length: 1.5
minimum_request_length: 3.0
th_arrived_distance: 0.2
pull_over_velocity: 0.5
pull_over_minimum_velocity: 0.3
maximum_deceleration: 0.3
maximum_jerk: 0.3
forward_goal_search_length: 8.0
backward_goal_search_length: 3.0
```

### 中型AGV（标准尺寸，中速）

```yaml
# 适用：体积 1-2m², 速度 1-2 m/s
forward_path_length: 30.0
backward_path_length: 2.0
minimum_request_length: 5.0
th_arrived_distance: 0.3
pull_over_velocity: 0.8
pull_over_minimum_velocity: 0.5
maximum_deceleration: 0.5
maximum_jerk: 0.5
forward_goal_search_length: 10.0
backward_goal_search_length: 5.0
```

### 大型AGV（体积大，正常速度）

```yaml
# 适用：体积 > 2m², 速度 > 2 m/s
forward_path_length: 50.0
backward_path_length: 3.0
minimum_request_length: 10.0
th_arrived_distance: 0.5
pull_over_velocity: 1.2
pull_over_minimum_velocity: 0.7
maximum_deceleration: 0.7
maximum_jerk: 0.7
forward_goal_search_length: 15.0
backward_goal_search_length: 8.0
```

---

## 🛠️ 实用脚本

### 一键检查配置

```bash
#!/bin/bash
# agv_config_check.sh

echo "========== AGV配置检查 =========="

# 获取ROS参数
NS="/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner"

echo "[1] 路径长度"
forward=$(ros2 param get $NS forward_path_length)
echo "  forward_path_length: $forward"
[ "${forward#*: }" -gt 100 ] && echo "  ⚠️  建议 < 50m（AGV）"

echo "[2] 最小距离"
min_req=$(ros2 param get $NS/goal_planner pull_over.minimum_request_length)
echo "  minimum_request_length: $min_req"
[ "${min_req#*: }" -lt 3 ] && echo "  ❌ 建议 ≥ 5m"

echo "[3] 到达精度"
arrived=$(ros2 param get $NS/goal_planner th_arrived_distance)
echo "  th_arrived_distance: $arrived"
[ "${arrived#*: }" -gt 0.5 ] && echo "  ⚠️  AGV建议 < 0.3m"

echo "[4] 速度"
vel=$(ros2 param get $NS/goal_planner pull_over.pull_over_velocity)
echo "  pull_over_velocity: $vel"
[ "${vel#*: }" -gt 1.5 ] && echo "  ⚠️  AGV建议 < 1.0m/s"

echo "================================"
```

### 实时监控目标距离

```bash
#!/bin/bash
# monitor_goal_distance.sh

echo "实时监控到目标点的距离..."

ros2 topic echo /planning/mission_planning/goal | \
  grep -E "(position:|orientation:)" -A 3 &

ros2 topic echo /localization/kinematic_state | \
  grep -E "position:" -A 3 | \
  while read line; do
    echo "当前位置: $line"
    # 这里可以添加距离计算逻辑
  done
```

### 速度剖面可视化

```bash
#!/bin/bash
# visualize_velocity_profile.sh

echo "提取路径速度剖面..."

ros2 topic echo /planning/behavior_planning/path_with_lane_id | \
  grep "longitudinal_velocity_mps" | \
  awk '{print NR, $2}' | \
  gnuplot -persist -e "
    set title 'Velocity Profile';
    set xlabel 'Point Index';
    set ylabel 'Velocity (m/s)';
    plot '-' with lines title 'Velocity'
  "
```

---

## 🚀 快速启动检查清单

```
启动前检查：
□ 已修改 forward_path_length (< 50m)
□ 已设置 minimum_request_length (≥ 5m)
□ 已调整 th_arrived_distance (0.2-0.3m)
□ 已降低 pull_over_velocity (< 1.0 m/s)
□ 已禁用不需要的模块
□ 车辆参数已更新（vehicle_info.param.yaml）

首次测试：
□ 目标距离 ≥ 10m
□ 起始速度 = 0
□ 观察Goal Planner是否激活
□ 检查路径是否生成
□ 验证速度剖面

调优步骤：
□ 记录最小成功距离
□ 测试多个速度下的表现
□ 测试不同距离的精度
□ 根据结果微调参数

验收标准：
□ 目标距离 ≥ 5m时成功规划
□ 停止误差 < 0.5m
□ 速度剖面平滑
□ 无路径跳变
□ 计算时间 < 100ms
```

---

## 📚 相关文档

- **详细分析：** [近距离目标点规划问题分析.md](./近距离目标点规划问题分析.md)
- **完整配置：** [AGV完整配置模板.yaml](./AGV完整配置模板.yaml)
- **速度问题：** [behavior_path_planner速度为零的原因分析.md](./behavior_path_planner速度为零的原因分析.md)
- **模块原理：** [behavior_path_planner场景模块工作原理详解.md](./behavior_path_planner场景模块工作原理详解.md)

---

## 💡 常见错误及解决

```
┌──────────────────────────────────────────────────────┐
│ 错误1: "No valid path found"                         │
├──────────────────────────────────────────────────────┤
│ 原因：forward_path_length太长或目标点太近            │
│ 解决：缩短forward_path_length到20-30m                │
│      确保目标距离 ≥ minimum_request_length          │
└──────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────┐
│ 错误2: "Goal planner not activated"                  │
├──────────────────────────────────────────────────────┤
│ 原因：目标点太近或minimum_request_length=0           │
│ 解决：设置minimum_request_length=5.0                 │
│      目标距离 ≥ 5m                                   │
└──────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────┐
│ 错误3: "Velocity is zero along path"                 │
├──────────────────────────────────────────────────────┤
│ 原因：整个路径在减速区或无速度限制                   │
│ 解决：增加目标距离到 ≥ 10m                          │
│      检查Lanelet地图speed_limit                      │
│      增加pull_over_velocity                          │
└──────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────┐
│ 错误4: "Vehicle doesn't stop at goal"                │
├──────────────────────────────────────────────────────┤
│ 原因：th_arrived_distance太小，小于定位误差          │
│ 解决：增加th_arrived_distance到0.3-0.5m              │
│      检查定位系统精度                                │
└──────────────────────────────────────────────────────┘
```

---

**最后提醒：**
- ✅ 从保守参数开始（距离更长、速度更慢）
- ✅ 逐步调优，每次只改一个参数
- ✅ 详细记录测试结果
- ✅ 考虑定位系统的实际精度
- ✅ 在真实环境测试前先仿真验证

