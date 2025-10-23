# 室内AGV自动驾驶配置文件

## 📋 目录

- [概述](#概述)
- [文件结构](#文件结构)
- [快速开始](#快速开始)
- [模块详细说明](#模块详细说明)
  - [Localization 定位模块](#localization-定位模块)
  - [Perception 感知模块](#perception-感知模块)
  - [Planning 规划模块](#planning-规划模块)
  - [Control 控制模块](#control-控制模块)
  - [Map 地图模块](#map-地图模块)
  - [Vehicle 车辆模块](#vehicle-车辆模块)
- [参数调优指南](#参数调优指南)
- [常见问题](#常见问题)
- [故障排查](#故障排查)

---

## 概述

本配置包专为**室内环境下的AGV（自动导引车）**设计，针对室内场景的特点进行了全面优化。

### 🎯 适用场景

- 🏭 **工厂车间**：物料运输、产线对接
- 📦 **仓库物流**：货物搬运、自动分拣
- 🏥 **医院配送**：药品配送、物资运输
- 🏬 **商场服务**：迎宾导引、物品配送

### ✨ 主要特点

1. **低速优化**：针对0-2 m/s速度范围优化
2. **高精度定位**：基于2D激光雷达+IMU融合
3. **实时避障**：高频障碍物检测和动态规避
4. **平稳控制**：舒适的加减速和转向控制
5. **窄通道适应**：优化的路径规划和跟踪算法

### 📊 性能指标

| 指标 | 数值 | 说明 |
|------|------|------|
| 最大速度 | 1.5 m/s | 可调整，建议0.8-2.0 m/s |
| 定位精度 | ±5 cm | 横向误差（室内NDT） |
| 控制频率 | 30-50 Hz | 保证低速控制稳定性 |
| 障碍物检测范围 | 15 m | 可根据传感器调整 |
| 最小转弯半径 | 0.5 m | 根据车辆规格设置 |

---

## 文件结构

```
indoor_agv/
├── README.md                          # 本文档
├── localization/                      # 定位模块配置
│   ├── ekf_localizer.param.yaml      # EKF定位器
│   ├── ndt_scan_matcher.param.yaml   # NDT扫描匹配器
│   └── pose_initializer.param.yaml   # 位姿初始化器
├── perception/                        # 感知模块配置
│   ├── occupancy_grid_map.param.yaml # 占据栅格地图
│   ├── ground_segmentation.param.yaml # 地面分割
│   └── euclidean_cluster.param.yaml  # 聚类检测
├── planning/                          # 规划模块配置
│   ├── mission_planner.param.yaml    # 任务规划器
│   ├── behavior_path_planner.param.yaml      # 行为路径规划
│   ├── behavior_velocity_planner.param.yaml  # 行为速度规划
│   ├── motion_velocity_planner.param.yaml    # 运动速度规划
│   ├── obstacle_cruise_planner.param.yaml    # 障碍物巡航
│   └── velocity_smoother.param.yaml          # 速度平滑器
├── control/                           # 控制模块配置
│   ├── trajectory_follower.param.yaml        # 轨迹跟踪器
│   ├── lateral_controller.param.yaml         # 横向控制器
│   ├── longitudinal_controller.param.yaml    # 纵向控制器
│   └── vehicle_cmd_gate.param.yaml           # 车辆命令门
├── map/                               # 地图模块配置
│   ├── lanelet2_map_loader.param.yaml        # Lanelet2地图加载
│   └── pointcloud_map_loader.param.yaml      # 点云地图加载
├── vehicle/                           # 车辆模块配置
│   └── raw_vehicle_cmd_converter.param.yaml  # 命令转换器
└── system/                            # 系统配置（可选）
    └── diagnostics.yaml               # 诊断配置
```

---

## 快速开始

### 1. 前置条件

确保已经安装并配置好：
- ✅ Autoware.universe
- ✅ ROS 2 Humble/Galactic
- ✅ 室内地图（Lanelet2格式 + 点云地图）
- ✅ 激光雷达驱动和标定
- ✅ IMU驱动和标定

### 2. 启动命令

```bash
# 启动室内AGV自动驾驶系统
ros2 launch autoware_launch indoor_agv.launch.xml \
  map_path:=/path/to/your/indoor/map \
  vehicle_model:=your_agv_model \
  sensor_model:=your_sensor_kit

# 示例：使用默认模型
ros2 launch autoware_launch indoor_agv.launch.xml \
  map_path:=/home/user/maps/warehouse_floor1
```

### 3. 参数说明

主要启动参数：

| 参数 | 说明 | 默认值 | 示例 |
|------|------|--------|------|
| `map_path` | 地图文件路径 | 必填 | `/home/user/maps/indoor` |
| `vehicle_model` | 车辆模型 | `agv_vehicle` | `forklift_agv` |
| `sensor_model` | 传感器配置 | `indoor_agv_sensor` | `sick_lidar_kit` |
| `max_velocity` | 最大速度(m/s) | `1.5` | `1.0` |
| `use_sim_time` | 使用仿真时间 | `false` | `true` |

### 4. 运行检查

启动后检查以下话题是否正常：

```bash
# 检查定位
ros2 topic echo /localization/kinematic_state

# 检查感知
ros2 topic echo /perception/object_recognition/objects

# 检查规划
ros2 topic echo /planning/scenario_planning/trajectory

# 检查控制
ros2 topic echo /control/command/control_cmd
```

---

## 模块详细说明

### Localization 定位模块

#### 功能概述
提供高精度的车辆位姿估计，融合2D激光雷达NDT匹配和IMU数据。

#### 核心配置文件

**1. ekf_localizer.param.yaml**
- **功能**：扩展卡尔曼滤波器，融合多源定位数据
- **关键参数**：
  ```yaml
  predict_frequency: 50.0        # 预测频率50Hz，室内精度要求高
  enable_yaw_bias_estimation: true  # 补偿IMU偏航角漂移
  proc_stddev_yaw_c: 0.005      # 偏航角过程噪声
  ```
- **调优建议**：
  - 如果定位抖动：降低 `proc_stddev_*` 参数
  - 如果响应慢：提高 `predict_frequency`
  - 如果有明显漂移：启用 `enable_yaw_bias_estimation`

**2. ndt_scan_matcher.param.yaml**
- **功能**：基于NDT算法的激光雷达定位
- **关键参数**：
  ```yaml
  resolution: 0.5                # 体素分辨率，室内0.3-0.5m
  max_iterations: 30             # 最大迭代次数
  trans_epsilon: 0.005           # 收敛阈值
  ```
- **调优建议**：
  - 定位精度不够：减小 `resolution` (0.3m)
  - 计算太慢：增大 `resolution` (0.5-1.0m)
  - 匹配失败：增加 `max_iterations` (50-100)

**3. pose_initializer.param.yaml**
- **功能**：设置车辆初始位姿
- **使用方法**：
  1. 在RViz中点击"2D Pose Estimate"
  2. 点击车辆实际位置并拖动设置方向
  3. 系统自动初始化定位

#### 故障诊断

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 定位跳变 | NDT匹配失败 | 检查地图质量，减小resolution |
| 定位漂移 | IMU偏差累积 | 启用yaw_bias_estimation |
| 无法初始化 | 初始位姿偏差大 | 手动设置更准确的初始位姿 |
| 定位延迟大 | 计算资源不足 | 减少max_iterations，增大resolution |

---

### Perception 感知模块

#### 功能概述
实时检测室内环境中的障碍物，生成占据栅格地图用于规划避障。

#### 核心配置文件

**1. occupancy_grid_map.param.yaml**
- **功能**：生成占据栅格地图
- **关键参数**：
  ```yaml
  map_length: 50.0              # 地图范围50m×50m
  map_resolution: 0.1           # 栅格分辨率10cm
  height_filter:
    min_height: -0.5            # 过滤地面
    max_height: 2.5             # 过滤天花板
  ```
- **调优建议**：
  - 需要更大范围：增加 `map_length` (但会增加计算量)
  - 需要更精细：减小 `map_resolution` (0.05m)
  - 过滤天花板：调整 `max_height` 为实际天花板高度

**2. ground_segmentation.param.yaml**
- **功能**：分离地面和障碍物点云
- **关键参数**：
  ```yaml
  algorithm_type: "ransac"      # RANSAC平面拟合
  distance_threshold: 0.05      # 地面点距离阈值5cm
  ```
- **调优建议**：
  - 地面不平整：增大 `distance_threshold` (0.08-0.1)
  - 误检地面为障碍：减小 `distance_threshold` (0.03-0.05)

**3. euclidean_cluster.param.yaml**
- **功能**：将障碍物点云聚类成独立对象
- **关键参数**：
  ```yaml
  cluster_tolerance: 0.2        # 聚类距离阈值20cm
  min_cluster_size: 15          # 最小点数
  max_cluster_size: 10000       # 最大点数
  ```
- **调优建议**：
  - 障碍物分割过细：增大 `cluster_tolerance` (0.3-0.5)
  - 多个障碍物粘连：减小 `cluster_tolerance` (0.1-0.15)
  - 噪声太多：增大 `min_cluster_size` (30-50)

#### 故障诊断

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 栅格地图空白 | 激光数据异常 | 检查激光话题和坐标变换 |
| 地面被识别为障碍 | 地面分割失败 | 调整height_filter参数 |
| 小障碍物检测不到 | 聚类参数过严 | 减小min_cluster_size |
| 噪声点太多 | 过滤不足 | 增大min_cluster_size |

---

### Planning 规划模块

#### 功能概述
提供全局路径规划、局部避障、速度规划等功能，生成平滑可执行的轨迹。

#### 核心配置文件

**1. mission_planner.param.yaml**
- **功能**：全局路径规划（A*搜索）
- **关键参数**：
  ```yaml
  algorithm: "a_star"           # A*算法
  goal_tolerance: 0.3           # 目标点容差30cm
  safety_margin: 0.3            # 安全边界30cm
  ```
- **使用方法**：
  1. 在RViz中点击"2D Nav Goal"
  2. 点击目标位置并拖动设置方向
  3. 系统自动规划全局路径

**2. behavior_path_planner.param.yaml**
- **功能**：局部路径规划和避障
- **关键参数**：
  ```yaml
  planning_hz: 10.0             # 规划频率10Hz
  forward_path_length: 50.0     # 前瞻距离50m
  avoidance:
    lateral_avoidance_distance: 0.5   # 横向避障距离
    min_avoidance_gap: 0.8            # 最小通过间隙
  ```
- **调优建议**：
  - 通道较窄：减小 `lateral_avoidance_distance` (0.3-0.4)
  - 避障不够灵敏：提高 `planning_hz` (15-20)

**3. behavior_velocity_planner.param.yaml**
- **功能**：基于场景的速度规划（停止线、障碍物）
- **关键参数**：
  ```yaml
  obstacle_stop:
    stop_distance: 1.0          # 障碍物前停止距离1m
    decel_distance: 3.0         # 开始减速距离3m
  curvature_speed_limit:
    max_lateral_accel: 0.5      # 最大横向加速度
  ```

**4. motion_velocity_planner.param.yaml**
- **功能**：轨迹级速度优化
- **关键参数**：
  ```yaml
  max_velocity: 1.5             # 最大速度1.5m/s
  max_acceleration: 0.5         # 最大加速度0.5m/s²
  max_jerk: 0.5                 # 最大急动度0.5m/s³
  ```

**5. obstacle_cruise_planner.param.yaml**
- **功能**：动态障碍物跟随和避让
- **关键参数**：
  ```yaml
  strategy: "adaptive"          # 自适应策略
  time_headway: 1.5             # 时间车距1.5s
  target_following_distance: 2.0  # 跟随距离2m
  ```

**6. velocity_smoother.param.yaml**
- **功能**：平滑速度曲线
- **关键参数**：
  ```yaml
  smoother_type: "JerkFiltered"  # 急动度滤波
  max_jerk: 0.5                  # 最大急动度限制
  ```

#### 调优建议

**场景1：通道较窄（<1.5m）**
```yaml
# behavior_path_planner.param.yaml
lateral_avoidance_distance: 0.3
min_avoidance_gap: 0.8

# behavior_velocity_planner.param.yaml
corridor_width_speed_adjustment:
  enable: true
  min_corridor_width: 1.2
```

**场景2：人员密集区域**
```yaml
# behavior_velocity_planner.param.yaml
pedestrian_detection:
  enable: true
  pedestrian_speed_limit: 0.8
  pedestrian_stop_distance: 2.0

# motion_velocity_planner.param.yaml
max_velocity: 1.0
```

**场景3：高精度对接**
```yaml
# mission_planner.param.yaml
goal_tolerance: 0.2

# behavior_path_planner.param.yaml
enable_cog_on_centerline: true
output_path_interval: 0.5
```

---

### Control 控制模块

#### 功能概述
实现精确的轨迹跟踪控制，包括横向控制（转向）和纵向控制（速度）。

#### 核心配置文件

**1. trajectory_follower.param.yaml**
- **功能**：轨迹跟踪主控制器
- **关键参数**：
  ```yaml
  ctrl_period: 0.03             # 控制周期30ms (33Hz)
  timeout_thr_sec: 0.5          # 轨迹超时阈值
  ```

**2. lateral_controller.param.yaml**
- **功能**：横向控制（转向控制）
- **控制器类型**：Pure Pursuit（纯跟踪）
- **关键参数**：
  ```yaml
  controller_type: "pure_pursuit"
  base_lookahead_distance: 2.0  # 基础前瞻距离2m
  min_lookahead_distance: 1.0   # 最小前瞻1m
  velocity_lookahead_ratio: 1.5 # 速度系数
  max_steering_angle: 0.524     # 最大转向角30度
  ```
- **调优建议**：
  - 转弯过冲：减小 `base_lookahead_distance` (1.5m)
  - 转弯滞后：增大 `base_lookahead_distance` (2.5-3.0m)
  - 抖动严重：增大 `min_lookahead_distance` (1.5m)

**3. longitudinal_controller.param.yaml**
- **功能**：纵向控制（速度控制）
- **控制器类型**：PID控制器
- **关键参数**：
  ```yaml
  kp: 1.0                       # 比例增益
  ki: 0.1                       # 积分增益
  kd: 0.05                      # 微分增益
  max_acceleration: 0.5         # 最大加速度
  max_deceleration: 0.8         # 最大减速度
  ```
- **PID调优方法**：
  1. 先调P：从小到大增加kp，观察响应速度
  2. 再调I：消除稳态误差，通常kp/10
  3. 最后调D：减少超调，通常kp/20

**4. vehicle_cmd_gate.param.yaml**
- **功能**：车辆命令安全门控
- **关键参数**：
  ```yaml
  update_rate: 30.0             # 更新频率30Hz
  command_limits:
    velocity:
      max: 1.5                  # 速度限制
      max_rate: 0.8             # 速度变化率限制
  ```

#### PID参数调优实例

**案例1：响应慢、有稳态误差**
```yaml
# 问题：车辆跟不上目标速度，总是慢一点
# 解决：增加P和I增益
kp: 1.5  # 原来1.0
ki: 0.2  # 原来0.1
kd: 0.05
```

**案例2：超调严重、震荡**
```yaml
# 问题：速度超过目标值后又降下来，反复震荡
# 解决：增加D增益，降低P增益
kp: 0.8  # 原来1.0
ki: 0.1
kd: 0.1  # 原来0.05
```

**案例3：加速过猛、不舒适**
```yaml
# 问题：启动时加速度太大
# 解决：限制加速度和急动度
max_acceleration: 0.3  # 原来0.5
max_jerk: 0.3          # 原来0.5
```

---

### Map 地图模块

#### 功能概述
加载和管理室内地图，包括Lanelet2路网地图和点云地图。

#### 核心配置文件

**1. lanelet2_map_loader.param.yaml**
- **功能**：加载Lanelet2格式路网地图
- **关键参数**：
  ```yaml
  projection_type: "local"      # 本地坐标系
  default_lane_width: 1.5       # 默认车道宽度
  default_speed_limit: 1.0      # 默认速度限制
  ```

**2. pointcloud_map_loader.param.yaml**
- **功能**：加载NDT定位用点云地图
- **关键参数**：
  ```yaml
  loading_mode: "whole"         # 全地图加载
  enable_downsampling: false    # 不降采样
  ```

#### 地图制作要求

**Lanelet2地图**：
1. 车道宽度：0.8-3.0m
2. 必须包含的元素：
   - Lanelets（车道）
   - Areas（区域，如停车区）
   - Regulatory Elements（停止线等）

**点云地图**：
1. 格式：PCD格式
2. 密度：室内环境建议点间距5-10cm
3. 覆盖范围：完整的室内区域
4. 特征：包含墙壁、柱子等显著特征

---

### Vehicle 车辆模块

#### 功能概述
将Autoware标准控制命令转换为底层车辆命令。

#### 核心配置文件

**raw_vehicle_cmd_converter.param.yaml**
- **功能**：命令转换和标定
- **关键参数**：
  ```yaml
  velocity_conversion:
    conversion_factor: 1.0      # 速度转换系数
    deadzone: 0.05              # 死区
  steering_conversion:
    steering_ratio: 15.0        # 转向比
    max_steering_angle: 0.524   # 最大转向角
  ```

#### 车辆参数标定

**步骤1：速度标定**
1. 发送固定速度命令（如0.5 m/s）
2. 测量实际车辆速度
3. 计算转换因子：`factor = 实际速度 / 命令速度`
4. 更新 `velocity_conversion.conversion_factor`

**步骤2：转向标定**
1. 发送最大转向命令
2. 测量实际前轮转角
3. 更新 `max_steering_angle`
4. 测试不同转向角度，建立标定表

---

## 参数调优指南

### 基本调优流程

```
1. 基础功能验证
   ├── 定位是否稳定？
   ├── 障碍物能否检测？
   ├── 路径是否合理？
   └── 控制是否跟踪？

2. 分模块调优
   ├── Localization：定位精度和稳定性
   ├── Perception：检测范围和准确性
   ├── Planning：路径质量和避障效果
   └── Control：跟踪精度和舒适性

3. 整体性能优化
   ├── 端到端延迟
   ├── 计算资源占用
   └── 实际运行效果
```

### 常见调优场景

#### 场景1：定位不稳定

**现象**：车辆位置跳变、抖动

**排查步骤**：
1. 检查NDT匹配得分
   ```bash
   ros2 topic echo /localization/pose_estimator/ndt_scan_matcher/debug
   ```
2. 检查地图质量（是否有足够特征）
3. 调整NDT参数

**解决方案**：
```yaml
# ndt_scan_matcher.param.yaml
resolution: 0.4  # 减小分辨率提高精度
max_iterations: 50  # 增加迭代次数
converged_param_nearest_voxel_transformation_likelihood: 2.0  # 降低阈值
```

#### 场景2：避障不及时

**现象**：快撞到障碍物才开始减速

**排查步骤**：
1. 检查障碍物检测距离
2. 检查规划频率
3. 检查减速参数

**解决方案**：
```yaml
# behavior_velocity_planner.param.yaml
obstacle_stop:
  stop_distance: 1.5  # 增加停止距离
  decel_distance: 5.0  # 增加减速距离

# obstacle_cruise_planner.param.yaml
time_headway: 2.0  # 增加时间车距
```

#### 场景3：转弯过冲

**现象**：转弯时偏离路径较多

**排查步骤**：
1. 检查转向响应延迟
2. 检查前瞻距离设置
3. 检查控制增益

**解决方案**：
```yaml
# lateral_controller.param.yaml
base_lookahead_distance: 1.5  # 减小前瞻距离
lateral_error_gain: 1.2  # 增加误差增益

# trajectory_follower.param.yaml
ctrl_period: 0.025  # 提高控制频率到40Hz
```

#### 场景4：速度控制不稳定

**现象**：速度忽快忽慢，不平滑

**排查步骤**：
1. 检查速度反馈质量
2. 检查PID参数设置
3. 检查命令滤波

**解决方案**：
```yaml
# longitudinal_controller.param.yaml
kp: 0.8  # 降低P增益
kd: 0.1  # 增加D增益
enable_velocity_smoothing: true
velocity_smoothing_window: 7  # 增加平滑窗口

# vehicle_cmd_gate.param.yaml
command_filtering:
  enable: true
  cutoff_frequency: 3.0  # 降低截止频率
```

---

## 常见问题

### Q1: 如何调整AGV最大速度？

**A:** 需要在多个文件中同步修改：

```bash
# 1. Launch文件
# indoor_agv.launch.xml
max_velocity: 2.0

# 2. 规划模块
# planning/motion_velocity_planner.param.yaml
max_velocity: 2.0

# 3. 控制模块
# control/vehicle_cmd_gate.param.yaml
velocity:
  max: 2.0
```

### Q2: 如何设置不同区域的速度限制？

**A:** 在Lanelet2地图中设置速度限制标签：

```xml
<!-- 地图文件 indoor_map.osm -->
<way id="100">
  <tag k="type" v="lanelet"/>
  <tag k="speed_limit" v="0.8"/>  <!-- 低速区 -->
  <nd ref="1"/>
  <nd ref="2"/>
</way>
```

### Q3: 如何调整安全距离？

**A:** 根据场景调整多个参数：

```yaml
# 全局安全距离
# mission_planner.param.yaml
safety_margin: 0.4

# 障碍物停止距离
# behavior_velocity_planner.param.yaml
obstacle_stop:
  stop_distance: 1.2

# 跟随距离
# obstacle_cruise_planner.param.yaml
target_following_distance: 2.5
```

### Q4: 如何处理窄通道？

**A:** 专门优化窄通道参数：

```yaml
# behavior_path_planner.param.yaml
avoidance:
  lateral_avoidance_distance: 0.2
  min_avoidance_gap: 0.6

# behavior_velocity_planner.param.yaml
corridor_width_speed_adjustment:
  enable: true
  min_corridor_width: 1.0
  width_to_speed_scaling:
    - [0.8, 0.2]  # 80cm宽通道20%速度
    - [1.2, 0.5]
```

### Q5: 如何提高定位精度？

**A:** 优化NDT和EKF参数：

```yaml
# ndt_scan_matcher.param.yaml
resolution: 0.3  # 更小分辨率
max_iterations: 50
trans_epsilon: 0.003

# ekf_localizer.param.yaml
predict_frequency: 100.0  # 更高频率
pose_measure_uncertainty_time: 0.005
```

### Q6: 如何处理动态障碍物？

**A:** 启用并调整动态障碍物处理：

```yaml
# behavior_path_planner.param.yaml
avoidance:
  strategy: "dynamic"

# obstacle_cruise_planner.param.yaml
strategy: "adaptive"
collision_prediction:
  time_horizon: 3.0
  consider_velocity_uncertainty: true
```

### Q7: 如何优化计算性能？

**A:** 降低计算密集参数：

```yaml
# ndt_scan_matcher.param.yaml
resolution: 0.6  # 增大体素尺寸
max_iterations: 20  # 减少迭代

# occupancy_grid_map.param.yaml
map_resolution: 0.15  # 降低栅格精度
enable_single_frame_mode: true

# behavior_path_planner.param.yaml
planning_hz: 5.0  # 降低规划频率
```

### Q8: 如何调试某个模块？

**A:** 使用ROS 2调试工具：

```bash
# 1. 查看话题列表
ros2 topic list

# 2. 查看特定话题
ros2 topic echo /localization/kinematic_state

# 3. 查看话题频率
ros2 topic hz /planning/scenario_planning/trajectory

# 4. 查看节点参数
ros2 param list /localization/ekf_localizer

# 5. 动态修改参数（临时）
ros2 param set /control/trajectory_follower ctrl_period 0.02

# 6. 记录数据包
ros2 bag record -a
```

---

## 故障排查

### 启动失败

**症状**：Launch文件无法启动或报错

**排查步骤**：
1. 检查地图路径是否正确
   ```bash
   ls -la /path/to/map/
   # 应该看到 lanelet2_map.osm 和 pointcloud_map.pcd
   ```

2. 检查TF树是否完整
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```

3. 检查传感器驱动是否启动
   ```bash
   ros2 topic list | grep lidar
   ros2 topic echo /sensing/lidar/points
   ```

**常见错误**：
- `Map file not found`: 检查map_path参数
- `Transform timeout`: 检查传感器标定和TF发布
- `Sensor data timeout`: 检查传感器驱动

### 定位异常

**症状1：定位不动**

**排查**：
```bash
# 检查激光数据
ros2 topic hz /sensing/lidar/points
# 应该有10-20Hz

# 检查地图加载
ros2 topic echo /map/pointcloud_map --once
# 应该有点云数据

# 检查NDT状态
ros2 topic echo /localization/pose_estimator/ndt_scan_matcher/debug
# 查看transform_probability或nvtl_score
```

**症状2：定位跳变**

**排查**：
```bash
# 查看NDT匹配得分
ros2 topic echo /localization/pose_estimator/ndt_scan_matcher/debug

# 如果得分低于阈值，调整参数
# ndt_scan_matcher.param.yaml
converged_param_nearest_voxel_transformation_likelihood: 2.0
```

### 感知异常

**症状：检测不到障碍物**

**排查**：
```bash
# 1. 检查激光数据
ros2 topic echo /sensing/lidar/points --once

# 2. 查看地面分割结果
ros2 topic echo /perception/obstacle_segmentation/pointcloud

# 3. 查看聚类结果
ros2 topic echo /perception/object_recognition/objects

# 4. 在RViz中可视化
rviz2 -d $(ros2 pkg prefix autoware_launch)/share/autoware_launch/config/indoor_agv/rviz/indoor_agv.rviz
```

**解决方案**：
```yaml
# 降低检测阈值
# euclidean_cluster.param.yaml
min_cluster_size: 10  # 原来15
cluster_tolerance: 0.3  # 原来0.2
```

### 规划异常

**症状：不生成路径**

**排查**：
```bash
# 检查目标点是否在地图内
ros2 topic echo /planning/mission_planning/goal

# 检查全局路径
ros2 topic echo /planning/mission_planning/route

# 检查局部路径
ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path
```

**症状：路径不平滑**

**解决**：
```yaml
# velocity_smoother.param.yaml
smoother_type: "JerkFiltered"
jerk_filter_bandwidth: 0.5  # 降低带宽
```

### 控制异常

**症状：车辆不动**

**排查**：
```bash
# 1. 检查控制命令
ros2 topic echo /control/command/control_cmd

# 2. 检查车辆状态
ros2 topic echo /vehicle/status/velocity_status

# 3. 检查engage状态
ros2 topic echo /vehicle/engage
# 应该是true

# 4. 如果是false，手动engage
ros2 topic pub /vehicle/engage std_msgs/msg/Bool "{data: true}"
```

**症状：控制抖动**

**解决**：
```yaml
# lateral_controller.param.yaml
min_lookahead_distance: 2.0  # 增大
path_smoothing_window: 7  # 增大平滑窗口

# longitudinal_controller.param.yaml
kp: 0.7  # 降低P增益
enable_velocity_smoothing: true
```

### 性能问题

**症状：CPU占用率过高**

**排查**：
```bash
# 查看节点CPU占用
top -p $(pgrep -d',' -f ros)

# 或使用ros2工具
ros2 run rqt_top rqt_top
```

**优化方案**：
```yaml
# 降低各模块频率
# ndt_scan_matcher.param.yaml
max_iterations: 15  # 原来30

# behavior_path_planner.param.yaml
planning_hz: 5.0  # 原来10.0

# occupancy_grid_map.param.yaml
map_resolution: 0.2  # 原来0.1
```

**症状：延迟大**

**排查**：
```bash
# 检查各模块延迟
ros2 topic echo /diagnostics | grep -A5 "control"
```

**优化**：
1. 提高控制频率
2. 降低其他模块频率
3. 启用多线程
4. 使用更强大的计算平台

---

## 技术支持

### 日志记录

启用详细日志以便调试：

```bash
# 设置日志级别
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_LOGGING_BUFFERED_STREAM=1

# 启动时记录日志
ros2 launch autoware_launch indoor_agv.launch.xml 2>&1 | tee agv_run.log
```

### 数据包记录

记录关键话题用于离线分析：

```bash
# 记录所有话题（文件会很大）
ros2 bag record -a

# 仅记录关键话题
ros2 bag record \
  /localization/kinematic_state \
  /perception/object_recognition/objects \
  /planning/scenario_planning/trajectory \
  /control/command/control_cmd \
  /vehicle/status/velocity_status
```

### 参数导出

导出当前运行参数：

```bash
# 导出所有节点参数
ros2 param dump /localization/ekf_localizer > ekf_running.yaml
ros2 param dump /control/trajectory_follower > control_running.yaml
```

---

## 版本信息

- **配置版本**: v1.0.0
- **创建日期**: 2025-10-21
- **Autoware版本**: universe (ROS 2 Humble)
- **适用车辆**: 室内AGV（差速/阿克曼转向）
- **测试环境**: 仓库、工厂车间

## 更新日志

### v1.0.0 (2025-10-21)
- ✨ 初始版本发布
- 📝 完整的配置文件和文档
- 🎯 针对室内场景优化
- 🔧 包含详细的调优指南

---

## 贡献指南

如果您在使用过程中发现问题或有改进建议，欢迎：
1. 提交Issue
2. 提交Pull Request
3. 分享您的调优经验

---

## 许可证

本配置文件遵循Apache 2.0许可证。

---

**祝您使用愉快！🚀**

