# E2E仿真器AGV使用指南

## 📋 概述

本指南详细说明如何使用 `e2e_simulator.launch.xml` 在仿真环境中运行室内AGV。该启动文件是 Autoware 的端到端仿真入口，支持 AWSIM 和 CARLA 仿真器。

---

## 🎯 文件位置

```
src/launcher/autoware_launch/autoware_launch/launch/e2e_simulator.launch.xml
```

---

## 🚀 快速启动

### 基本启动命令

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=/path/to/your/indoor_map \
  vehicle_model:=indoor_agv \
  sensor_model:=agv_sensor_kit \
  use_sim_time:=true \
  simulator_type:=awsim
```

### 必需参数说明

| 参数名 | 说明 | AGV示例 |
|--------|------|---------|
| `map_path` | 地图文件夹路径（包含.osm和.pcd文件） | `/home/user/maps/warehouse_floor1` |
| `vehicle_model` | 车辆模型名称 | `indoor_agv` 或 `warehouse_robot` |
| `sensor_model` | 传感器配置名称 | `agv_sensor_kit` 或 `2d_lidar_kit` |
| `use_sim_time` | 使用仿真时间（仿真必须设为true） | `true` |

---

## ⚙️ 核心参数详解

### 1. 仿真器配置

#### `simulator_type` (默认: `awsim`)
选择仿真器类型：

- **`awsim`**: Unity引擎，轻量级，推荐用于AGV开发
- **`carla`**: 虚幻引擎，高逼真度，适合需要复杂视觉的场景

**AGV建议**: 使用 `awsim`，资源消耗更低，启动更快。

```bash
# 使用AWSIM（推荐）
simulator_type:=awsim

# 使用CARLA
simulator_type:=carla
```

#### `use_sim_time` (默认: `false`)
时间源控制：

- **`false`**: 使用系统实际时间（实车测试）
- **`true`**: 使用仿真器时间（仿真测试）

**AGV建议**: 仿真环境必须设为 `true`，实车测试设为 `false`。

---

### 2. 模块启动开关

所有模块默认都是启动的（`true`），可根据调试需要关闭特定模块：

| 参数名 | 默认值 | 功能 | 建议 |
|--------|--------|------|------|
| `vehicle` | true | 车辆接口模块 | 保持true |
| `system` | true | 系统监控 | 保持true |
| `map` | true | 地图加载 | 保持true |
| `sensing` | true | 传感器处理 | 保持true |
| `localization` | true | 定位模块 | 保持true（核心） |
| `perception` | true | 感知模块 | 保持true（动态避障） |
| `planning` | true | 规划模块 | 保持true（核心） |
| `control` | true | 控制模块 | 保持true（核心） |

#### 调试示例：仅测试定位

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=/path/to/map \
  vehicle_model:=agv \
  sensor_model:=sensors \
  perception:=false \
  planning:=false \
  control:=false \
  use_sim_time:=true
```

---

### 3. 地图配置

#### `map_path` (必需参数)
地图文件夹路径，必须包含以下文件：

```
your_map_folder/
├── lanelet2_map.osm        # Lanelet2路网地图
└── pointcloud_map.pcd      # 点云地图（用于NDT定位）
```

**AGV建议**:
- 点云地图分辨率: 0.1-0.5m
- Lanelet2地图包含: 车道中心线、速度限制、停止线
- 确保地图坐标系一致

#### `lanelet2_map_file` (默认: `lanelet2_map.osm`)
如果你的Lanelet2文件名不是默认的，可以修改：

```bash
lanelet2_map_file:=warehouse_map.osm
```

#### `pointcloud_map_file` (默认: `pointcloud_map.pcd`)
如果你的点云文件名不是默认的，可以修改：

```bash
pointcloud_map_file:=floor1_pointcloud.pcd
```

---

### 4. 车辆和传感器配置

#### `vehicle_model`
车辆物理参数配置，定义在:
```
src/vehicle/sample_vehicle_launch/config/<vehicle_model>/
```

**AGV需要配置的参数**:
- 轴距 (wheelbase)
- 最大转向角
- 车辆尺寸（长宽高）
- 重量和惯性

**创建自定义车辆模型**:
```bash
# 复制示例配置
cd src/vehicle/sample_vehicle_launch/config/
cp -r sample_vehicle indoor_agv

# 编辑参数
nano indoor_agv/vehicle_info.param.yaml
```

#### `sensor_model`
传感器布局和外参配置，定义在:
```
src/sensor_kit/<sensor_model>_description/
```

**AGV典型传感器配置**:
- 2D/3D激光雷达（前后各一个）
- IMU（可选，提升定位精度）
- 轮速计（通过车辆接口）
- 相机（可选，用于障碍物识别）

---

### 5. 模块预设配置

#### `planning_module_preset` (默认: `default`)
规划模块参数预设：

- **`default`**: 标准道路场景
- **`indoor_agv`**: 室内AGV优化（需自定义创建）

**创建indoor_agv预设**:
```bash
cd src/launcher/autoware_launch/autoware_launch/config/planning/preset/
cp -r default indoor_agv

# 修改速度限制
nano indoor_agv/common.param.yaml
# 设置: max_velocity: 1.5 m/s
```

#### `control_module_preset` (默认: `default`)
控制模块参数预设，同样可创建自定义配置。

---

### 6. 硬件接口开关

#### `launch_vehicle_interface` (默认: `false`)
是否启动真实车辆硬件接口：

- **`false`**: 仿真模式，使用仿真器提供的车辆控制
- **`true`**: 实车模式，启动CAN/串口等硬件驱动

**AGV建议**: 仿真时保持 `false`。

#### `launch_sensing_driver` (默认: `false`)
是否启动真实传感器驱动：

- **`false`**: 仿真模式，使用仿真器提供的传感器数据
- **`true`**: 实车模式，启动激光雷达、相机等驱动

**AGV建议**: 仿真时保持 `false`。

---

### 7. 系统监控

#### `launch_system_monitor` (默认: `false`)
是否启动系统资源监控（CPU、内存、温度等）：

**AGV建议**: 开发阶段设为 `true`，有助于性能调优和资源评估。

```bash
launch_system_monitor:=true
```

---

### 8. 可视化配置

#### `rviz` (默认: `true`)
是否启动RViz可视化界面：

**AGV建议**: 
- 开发调试: `true`
- 嵌入式部署: `false`（节省资源）

#### `rviz_config_name` (默认: `autoware.rviz`)
RViz配置文件名，可创建专门的AGV配置：

```bash
rviz_config_name:=indoor_agv.rviz
```

---

## 📝 完整启动示例

### 示例1: 基本室内AGV仿真

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=/home/user/maps/warehouse_floor1 \
  vehicle_model:=indoor_agv \
  sensor_model:=agv_2d_lidar \
  use_sim_time:=true \
  simulator_type:=awsim \
  rviz:=true
```

### 示例2: 使用自定义预设和监控

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=/home/user/maps/factory_indoor \
  vehicle_model:=warehouse_robot \
  sensor_model:=dual_lidar_kit \
  planning_module_preset:=indoor_agv \
  control_module_preset:=indoor_agv \
  launch_system_monitor:=true \
  use_sim_time:=true \
  simulator_type:=awsim \
  rviz_config_name:=indoor_agv.rviz
```

### 示例3: 定位调试模式

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=/home/user/maps/test_area \
  vehicle_model:=test_agv \
  sensor_model:=test_sensors \
  perception:=false \
  planning:=false \
  control:=false \
  use_sim_time:=true \
  rviz:=true
```

### 示例4: 感知调试模式

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=/home/user/maps/obstacle_test \
  vehicle_model:=agv \
  sensor_model:=3d_lidar \
  planning:=false \
  control:=false \
  use_sim_time:=true \
  rviz:=true
```

---

## 🔧 AGV专用参数调优

### 1. 速度限制调优

**文件位置**: `config/planning/preset/<preset_name>/common.param.yaml`

```yaml
# 最大速度限制 (m/s)
max_velocity: 1.5           # AGV推荐: 1.0-1.5

# 最大加速度 (m/s²)
max_acceleration: 0.5       # AGV推荐: 0.3-0.5

# 最大减速度 (m/s²)
max_deceleration: 0.8       # AGV推荐: 0.5-0.8

# 最大Jerk (m/s³)
max_jerk: 0.5              # AGV推荐: 0.3-0.5
```

### 2. 定位参数调优

**NDT扫描匹配** (`config/localization/ndt_scan_matcher.param.yaml`):

```yaml
ndt:
  resolution: 0.5           # AGV推荐: 0.3-0.5 (室内环境)
  max_iterations: 30        # AGV推荐: 30-50
  step_size: 0.05          # AGV推荐: 0.05-0.1
```

**EKF定位器** (`config/localization/ekf_localizer.param.yaml`):

```yaml
predict_frequency: 50.0     # AGV推荐: 30-50 Hz
tf_rate: 50.0              # AGV推荐: 30-50 Hz
```

### 3. 障碍物检测调优

**占据栅格地图** (`config/perception/occupancy_grid_map.param.yaml`):

```yaml
map_length: 50.0            # AGV推荐: 20-50 m (室内)
map_resolution: 0.1         # AGV推荐: 0.05-0.2 m

height_filter:
  min_height: -0.5          # AGV推荐: 根据地面高度调整
  max_height: 2.5           # AGV推荐: 室内天花板高度
```

### 4. 路径规划调优

**行为路径规划** (`config/planning/behavior_path_planner.param.yaml`):

```yaml
planning_hz: 10.0           # AGV推荐: 10-20 Hz
backward_path_length: 3.0   # AGV推荐: 3-5 m
forward_path_length: 50.0   # AGV推荐: 30-50 m (室内)

avoidance:
  lateral_avoidance_distance: 0.5    # AGV推荐: 0.3-0.5 m
  min_avoidance_gap: 0.8            # AGV推荐: 0.5-1.0 m
```

### 5. 控制参数调优

**纵向控制器** (`config/control/longitudinal_controller.param.yaml`):

```yaml
pid:
  kp: 1.0                   # 根据实车响应调整
  ki: 0.1                   # 消除稳态误差
  kd: 0.05                  # 抑制震荡

acceleration_limits:
  max_acceleration: 0.5     # AGV推荐: 0.3-0.5 m/s²
  max_deceleration: 0.8     # AGV推荐: 0.5-0.8 m/s²
```

**横向控制器** (`config/control/lateral_controller.param.yaml`):

```yaml
pure_pursuit:
  base_lookahead_distance: 2.0      # AGV推荐: 1.5-3.0 m
  min_lookahead_distance: 1.0       # AGV推荐: 1.0-1.5 m
  max_steering_angle: 0.524         # 30度，根据实车调整
```

---

## 🐛 常见问题排查

### 1. 仿真器连接失败

**症状**: 启动后无传感器数据，车辆不动

**排查步骤**:
```bash
# 1. 确认仿真器已启动
# 先启动AWSIM或CARLA仿真器

# 2. 检查ROS话题
ros2 topic list | grep -E '(lidar|imu|vehicle)'

# 3. 检查仿真时间
ros2 topic echo /clock

# 4. 确认use_sim_time设置
# 必须设为true
```

**解决方案**:
- 先启动仿真器，等待完全加载后再启动Autoware
- 确认 `use_sim_time:=true`
- 检查仿真器配置是否正确

### 2. 定位失败或精度差

**症状**: 车辆位置不准确或持续漂移

**排查步骤**:
```bash
# 1. 查看定位协方差
ros2 topic echo /localization/pose_estimator/pose_with_covariance

# 2. 查看NDT得分
ros2 topic echo /localization/pose_estimator/ndt_score

# 3. 检查点云地图质量
ros2 topic echo /map/pointcloud_map
```

**解决方案**:
- 检查点云地图分辨率和质量
- 调整NDT参数（resolution, max_iterations）
- 确保传感器数据正常
- 在RViz中手动设置初始位姿（2D Pose Estimate）

### 3. 车辆速度为零

**症状**: 设置目标点后车辆不动

**排查步骤**:
```bash
# 1. 查看规划输出
ros2 topic echo /planning/scenario_planning/trajectory

# 2. 查看速度限制
ros2 topic echo /planning/scenario_planning/max_velocity_default

# 3. 查看控制命令
ros2 topic echo /control/command/control_cmd

# 4. 检查诊断信息
ros2 topic echo /diagnostics
```

**解决方案**:
- 参考 `BPP速度为零_快速诊断.md` 文档
- 检查地图速度限制配置
- 确认目标点在可达路径上
- 检查障碍物是否阻挡

### 4. 路径规划失败

**症状**: 无法到达目标点，或路径异常

**排查步骤**:
```bash
# 1. 查看路径规划状态
ros2 topic echo /planning/mission_planning/route

# 2. 检查Lanelet2地图
# 在RViz中显示Lanelet2图层

# 3. 查看规划诊断
ros2 topic echo /diagnostics | grep planning
```

**解决方案**:
- 确认目标点在Lanelet2路网上
- 检查路网连通性
- 调整规划参数（goal_tolerance等）
- 确认起点和终点在同一路网内

### 5. 控制抖动或振荡

**症状**: 车辆行驶不平滑，左右摆动

**排查步骤**:
```bash
# 1. 查看横向误差
ros2 topic echo /control/trajectory_follower/lateral/diagnostic

# 2. 查看控制命令
ros2 topic echo /control/command/control_cmd

# 3. 检查定位精度
ros2 topic echo /localization/pose_estimator/pose_with_covariance
```

**解决方案**:
- 降低PID增益（特别是kp和kd）
- 增加前瞻距离
- 启用速度平滑
- 提高定位精度
- 检查转向延迟补偿参数

### 6. 感知误检测

**症状**: 将地面、墙壁识别为障碍物

**排查步骤**:
```bash
# 1. 查看原始点云
ros2 topic echo /sensing/lidar/concatenated/pointcloud

# 2. 查看地面分割结果
ros2 topic echo /perception/obstacle_segmentation/pointcloud

# 3. 查看障碍物聚类
ros2 topic echo /perception/object_recognition/objects
```

**解决方案**:
- 调整地面分割参数（height filter）
- 调整聚类参数（cluster_tolerance）
- 设置合理的高度过滤范围
- 使用点云降采样减少计算量

---

## 📊 性能监控

### 实时监控命令

```bash
# 1. 查看所有话题频率
ros2 topic hz /localization/pose_estimator/pose    # 定位频率
ros2 topic hz /planning/scenario_planning/trajectory  # 规划频率
ros2 topic hz /control/command/control_cmd         # 控制频率

# 2. 查看系统资源
top -p $(pgrep -d',' -f autoware)                  # CPU/内存使用

# 3. 查看延迟
ros2 topic delay /control/command/control_cmd      # 控制延迟

# 4. 查看TF树
ros2 run tf2_tools view_frames                     # 生成TF图
```

### 性能优化建议

**低性能设备优化**:
```bash
# 关闭不必要的模块
perception:=false              # 如果环境静态
launch_system_monitor:=false   # 节省资源
rviz:=false                    # 无GUI环境

# 降低频率
# 修改配置文件中的频率参数
planning_hz: 5.0               # 降低规划频率
ctrl_period: 0.05              # 降低控制频率（20Hz）
```

---

## 📚 相关文档

- **室内AGV完整配置**: `src/launcher/autoware_launch/autoware_launch/config/indoor_agv/README.md`
- **配置快速参考**: `src/launcher/autoware_launch/autoware_launch/config/indoor_agv/配置快速参考.md`
- **BPP速度诊断**: `BPP速度为零_快速诊断.md`
- **近距离目标点分析**: `近距离目标点规划问题分析.md`
- **Autoware官方文档**: https://autowarefoundation.github.io/autoware-documentation/

---

## 🎯 推荐工作流程

### 1. 准备阶段
```bash
# 1. 准备地图
# 确保有lanelet2_map.osm和pointcloud_map.pcd

# 2. 创建车辆模型
# 复制并修改sample_vehicle配置

# 3. 配置传感器
# 设置传感器外参和类型
```

### 2. 首次测试
```bash
# 仅启动定位模块进行测试
ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=/path/to/map \
  vehicle_model:=agv \
  sensor_model:=sensors \
  perception:=false \
  planning:=false \
  control:=false \
  use_sim_time:=true

# 在RViz中设置初始位姿，观察定位效果
```

### 3. 逐步启用模块
```bash
# 启用感知
perception:=true

# 启用规划
planning:=true

# 启用控制
control:=true
```

### 4. 参数调优
```bash
# 根据实际表现调整参数
# 速度、加速度、PID增益等
```

### 5. 完整测试
```bash
# 所有模块启用，进行端到端测试
# 设置多个目标点，测试各种场景
```

---

## ✅ 总结

`e2e_simulator.launch.xml` 是 Autoware 仿真的核心启动文件，通过详细的参数配置，可以灵活适配各种室内AGV场景。

**关键要点**:
1. ✅ 仿真时必须设置 `use_sim_time:=true`
2. ✅ 先启动仿真器，再启动Autoware
3. ✅ 根据实际AGV创建自定义车辆和传感器配置
4. ✅ 使用模块开关进行分步调试
5. ✅ 参考配置文件进行参数调优
6. ✅ 使用RViz实时监控系统状态

祝您使用愉快！🚀

