# Vehicle Info 配置文件说明

## ❓ 问题：哪个 vehicle_info.param.yaml 会被实际使用？

### 📁 两个文件的位置

```
1. autoware_launch/vehicle/sample_vehicle_launch/
   sample_vehicle_description/config/vehicle_info.param.yaml
   ⭐ 这个会被实际使用

2. autoware_core/common/autoware_vehicle_info_utils/
   config/vehicle_info.param.yaml
   ❌ 这个是默认模板，不会被使用
```

---

## ✅ 答案：**第1个会被实际使用**

### 详细说明

**实际使用的文件：**
```
main/src/launcher/autoware_launch/vehicle/
    sample_vehicle_launch/
        sample_vehicle_description/
            config/
                vehicle_info.param.yaml  ⭐⭐⭐
```

**不会被使用的文件（仅作为模板）：**
```
main/src/core/autoware_core/common/
    autoware_vehicle_info_utils/
        config/
            vehicle_info.param.yaml  ❌
```

---

## 🔍 原理解析

### 1. Launch文件配置

在 `tier4_planning_component.launch.xml` 第10行：

```xml
<arg name="vehicle_param_file" 
     value="$(find-pkg-share $(var vehicle_model)_description)/config/vehicle_info.param.yaml"/>
```

**关键点：**
- 使用 `$(var vehicle_model)_description` 来动态构造包名
- 默认 `vehicle_model` = `sample_vehicle`
- 因此实际路径是：`sample_vehicle_description/config/vehicle_info.param.yaml`

### 2. 完整的参数传递链

```
autoware.launch.xml (顶层)
    ↓ vehicle_model: "sample_vehicle" (默认值，第5行)
    ↓
tier4_planning_component.launch.xml
    ↓ 构造路径: $(find-pkg-share sample_vehicle_description)/config/vehicle_info.param.yaml
    ↓ 传递给: vehicle_param_file
    ↓
tier4_planning_launch/planning.launch.xml
    ↓ 传递给各子模块
    ↓
behavior_planning.launch.xml / motion_planning.launch.xml
    ↓ <param from="$(var vehicle_param_file)"/>
    ↓
各Planning节点 (behavior_path_planner, velocity_smoother等)
    ↓ VehicleInfoUtils(*this).getVehicleInfo()
    ↓
从ROS参数服务器读取参数并使用
```

---

## 📊 不同车型的配置

Autoware支持多种车型，每种车型都有自己的配置：

```
vehicle/
├── sample_vehicle_launch/
│   └── sample_vehicle_description/
│       └── config/vehicle_info.param.yaml  ← vehicle_model=sample_vehicle时使用
│
├── byd_vehicle_launch/
│   └── byd_vehicle_description/
│       └── config/vehicle_info.param.yaml  ← vehicle_model=byd_vehicle时使用
│
└── awsim_labs_vehicle_launch/
    └── awsim_labs_vehicle_description/
        └── config/vehicle_info.param.yaml  ← vehicle_model=awsim_labs_vehicle时使用
```

### 切换车型的方法

**方法1: 启动时指定**
```bash
ros2 launch autoware_launch autoware.launch.xml \
    vehicle_model:=byd_vehicle \
    map_path:=/path/to/map
```

**方法2: 修改默认值**
```xml
<!-- autoware.launch.xml 第5行 -->
<arg name="vehicle_model" default="my_custom_vehicle" description="vehicle model name"/>
```

---

## 🎯 autoware_vehicle_info_utils/config/vehicle_info.param.yaml 的作用

这个文件**不会被Autoware运行时使用**，它的作用是：

### 1. 作为参考模板
- 新建车型时可以复制这个文件作为起点
- 包含所有必需的参数字段
- 提供参数说明和单位

### 2. 用于测试
```cpp
// 在unit test中可能会引用
TEST(VehicleInfoTest, DefaultParams) {
    // 使用默认配置进行测试
}
```

### 3. 文档参考
- README.md中的示例
- 开发文档的参考

---

## 🔧 为AGV创建自定义配置

### 步骤1: 复制模板

```bash
cd src/launcher/autoware_launch/vehicle/
cp -r sample_vehicle_launch my_agv_launch
cd my_agv_launch
mv sample_vehicle_description my_agv_description
```

### 步骤2: 修改配置文件

编辑 `my_agv_description/config/vehicle_info.param.yaml`:

```yaml
/**:
  ros__parameters:
    wheel_radius: 0.15        # AGV轮子半径 [m]
    wheel_width: 0.08         # 轮胎宽度 [m]
    wheel_base: 0.6           # 前后轮轴距 [m]
    wheel_tread: 0.5          # 左右轮轮距 [m]
    front_overhang: 0.2       # 前悬 [m]
    rear_overhang: 0.2        # 后悬 [m]
    left_overhang: 0.05       # 左悬 [m]
    right_overhang: 0.05      # 右悬 [m]
    vehicle_height: 0.8       # 车高 [m]
    max_steer_angle: 1.57     # 最大转向角 [rad] (差速驱动可设大值)
```

### 步骤3: 修改package.xml

编辑 `my_agv_description/package.xml`:
```xml
<package format="3">
  <name>my_agv_description</name>
  <version>1.0.0</version>
  <description>AGV vehicle description</description>
  ...
</package>
```

### 步骤4: 启动时使用

```bash
# 先编译
colcon build --packages-select my_agv_description

# source环境
source install/setup.bash

# 启动时指定
ros2 launch autoware_launch autoware.launch.xml \
    vehicle_model:=my_agv \
    map_path:=/path/to/warehouse/map
```

---

## 🔍 验证方法

### 方法1: 检查launch文件解析结果

```bash
# 查看vehicle_model变量
ros2 launch autoware_launch autoware.launch.xml --show-args | grep vehicle_model

# 输出应该显示:
#   vehicle_model:
#       Vehicle model name (default: 'sample_vehicle')
```

### 方法2: 检查运行时参数

```bash
# 启动Autoware后
ros2 param get /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner wheel_base

# 输出应该是 sample_vehicle_description/config/vehicle_info.param.yaml 中的值
# Type: double
# Value: 2.79
```

### 方法3: 打印加载的包

```bash
# 查看ROS包路径
ros2 pkg prefix sample_vehicle_description

# 输出类似:
# /home/user/autoware/install/sample_vehicle_description
```

### 方法4: 代码层面验证

在C++代码中添加调试输出：

```cpp
// 在VehicleInfoUtils构造函数中
VehicleInfoUtils::VehicleInfoUtils(rclcpp::Node & node)
{
    const auto wheel_base_m = getParameter<double>(node, "wheel_base");
    
    RCLCPP_INFO(node.get_logger(), 
        "Loaded wheel_base: %.3f from vehicle_info parameters", 
        wheel_base_m);
    
    // 这里会打印出实际加载的值
}
```

---

## 📋 参数对比

### Sample Vehicle（实际使用的）
```yaml
wheel_radius: 0.383
wheel_base: 2.79
wheel_tread: 1.64
front_overhang: 1.0
rear_overhang: 1.1
```

### Vehicle Info Utils（模板，不使用）
```yaml
wheel_radius: 0.39
wheel_base: 2.74
wheel_tread: 1.63
front_overhang: 1.0
rear_overhang: 1.03
```

**可以看到参数值不同，证明它们是独立的文件。**

---

## ⚠️ 常见错误

### 错误1: 修改了错误的文件

```bash
# ❌ 错误：修改了autoware_vehicle_info_utils中的配置
nano src/core/autoware_core/common/autoware_vehicle_info_utils/config/vehicle_info.param.yaml

# ✅ 正确：应该修改vehicle launch包中的配置
nano src/launcher/autoware_launch/vehicle/sample_vehicle_launch/sample_vehicle_description/config/vehicle_info.param.yaml
```

### 错误2: 忘记重新编译

```bash
# 修改配置文件后需要重新编译（如果是新建的包）
colcon build --packages-select my_agv_description

# 如果只是修改参数值，不需要重新编译，但需要重启节点
```

### 错误3: 包名不匹配

```bash
# ❌ 错误：包名和vehicle_model不一致
vehicle_model:=my_agv
但包名是: my_agv_vehicle_description

# ✅ 正确：包名应该是 {vehicle_model}_description
vehicle_model:=my_agv
包名应该是: my_agv_description
```

---

## 📚 总结

| 项目 | 说明 |
|------|------|
| **实际使用的配置** | `autoware_launch/vehicle/{vehicle_model}_launch/{vehicle_model}_description/config/vehicle_info.param.yaml` |
| **不使用的配置** | `autoware_vehicle_info_utils/config/vehicle_info.param.yaml` |
| **选择机制** | 通过 `vehicle_model` 变量动态选择 |
| **默认车型** | `sample_vehicle` |
| **切换方法** | 启动时指定 `vehicle_model:=xxx` |
| **自定义方法** | 复制现有车型并修改配置 |

---

## 🎓 理解要点

1. ✅ **autoware_vehicle_info_utils中的配置只是模板**
   - 用于参考和测试
   - 不会被运行时使用

2. ✅ **实际使用的是vehicle launch包中的配置**
   - 根据`vehicle_model`变量选择
   - 支持多车型切换

3. ✅ **每种车型都有独立的配置**
   - 便于管理不同车型参数
   - 便于团队协作开发

4. ✅ **修改配置要找对文件**
   - 修改`autoware_launch/vehicle/xxx_launch`下的文件
   - 不要修改`autoware_vehicle_info_utils`中的文件

---

**最后更新**: 2025-01-17  
**文档版本**: 1.0

