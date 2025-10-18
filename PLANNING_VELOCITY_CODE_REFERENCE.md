# Autoware Planning模块速度相关代码详解

## 📋 目录

1. [速度处理流程概述](#速度处理流程概述)
2. [Core模块速度代码](#core模块速度代码)
3. [Universe模块速度代码](#universe模块速度代码)
4. [关键数据结构](#关键数据结构)
5. [速度计算算法](#速度计算算法)
6. [实用代码示例](#实用代码示例)

---

## 🔄 速度处理流程概述

```
输入轨迹(带初始速度)
    ↓
┌──────────────────────────────────────────┐
│  Behavior Path Planner (Universe)        │
│  • 设置参考速度                           │
│  • 考虑换道、避障等行为                   │
└──────────────┬───────────────────────────┘
               ↓ Path with reference velocity
┌──────────────────────────────────────────┐
│  Behavior Velocity Planner (Core+Universe)│
│  • 交通规则速度约束（停止线、红灯等）      │
│  • 设置停止点速度为0                      │
└──────────────┬───────────────────────────┘
               ↓ Path with velocity constraints
┌──────────────────────────────────────────┐
│  Motion Velocity Planner (Core+Universe) │
│  • 障碍物速度规划                         │
│  • 动态调整速度                           │
└──────────────┬───────────────────────────┘
               ↓ Trajectory (rough velocity)
┌──────────────────────────────────────────┐
│  Velocity Smoother (Core) ⭐核心          │
│  • 速度平滑优化                           │
│  • 加速度/加加速度约束                    │
│  • 横向加速度限制                         │
│  • 转向角速率限制                         │
└──────────────┬───────────────────────────┘
               ↓
输出平滑轨迹(最终速度)
```

---

## 🎯 Core模块速度代码

### 1. Velocity Smoother (核心速度平滑模块)

#### **1.1 主节点代码**

**文件**: `autoware_velocity_smoother/src/node.cpp`

```cpp
// 主回调函数 - 接收轨迹并处理速度
void VelocitySmootherNode::onCurrentTrajectory(
    const Trajectory::ConstSharedPtr msg)
{
    // 1. 接收输入数据
    base_traj_raw_ptr_ = msg;
    current_odometry_ptr_ = sub_current_odometry_.take_data();
    current_acceleration_ptr_ = sub_current_acceleration_.take_data();
    
    // 2. 转换为轨迹点数组
    auto input_points = autoware::motion_utils::convertToTrajectoryPointArray(*msg);
    
    // 3. 移除重叠点
    input_points = autoware::motion_utils::removeOverlapPoints(input_points);
    
    // 4. 设置终点速度为0
    input_points.back().longitudinal_velocity_mps = 0.0;
    
    // 5. 处理反向（倒车）
    is_reverse_ = isReverse(input_points);
    if (is_reverse_) {
        flipVelocity(input_points);  // 速度取反
    }
    
    // 6. 计算轨迹速度（核心）
    const auto output = calcTrajectoryVelocity(input_points);
    
    // 7. 重采样输出
    auto output_resampled = resampling::resampleTrajectory(
        output, current_odometry_ptr_->twist.twist.linear.x,
        current_odometry_ptr_->pose.pose);
    
    // 8. 发布结果
    pub_trajectory_->publish(toTrajectoryMsg(output_resampled));
}

// 核心速度计算函数
TrajectoryPoints VelocitySmootherNode::calcTrajectoryVelocity(
    const TrajectoryPoints & traj_input) const
{
    TrajectoryPoints output{};
    
    // 1. 提取车辆周围的轨迹
    const size_t input_closest = findNearestIndexFromEgo(traj_input);
    auto traj_extracted = trajectory_utils::extractPathAroundIndex(
        traj_input, input_closest, 
        node_param_.extract_ahead_dist,   // 前方200m
        node_param_.extract_behind_dist); // 后方5m
    
    // 2. 应用外部速度限制
    applyExternalVelocityLimit(traj_extracted);
    
    // 3. 应用停止接近速度
    applyStopApproachingVelocity(traj_extracted);
    
    // 4. 速度平滑（核心算法）
    if (!smoothVelocity(traj_extracted, input_closest, output)) {
        return prev_output_;  // 失败则返回上一次结果
    }
    
    return output;
}

// 速度平滑核心算法
bool VelocitySmootherNode::smoothVelocity(
    const TrajectoryPoints & input, 
    const size_t input_closest,
    TrajectoryPoints & traj_smoothed) const
{
    // 1. 计算初始运动状态（速度、加速度）
    const auto [initial_motion, type] = calcInitialMotion(input, input_closest);
    
    // 2. 应用横向加速度限制（弯道减速）
    const auto traj_lateral_acc_filtered = 
        node_param_.enable_lateral_acc_limit
            ? smoother_->applyLateralAccelerationFilter(
                input, initial_motion.vel, initial_motion.acc,
                enable_smooth_limit, use_resampling)
            : input;
    
    // 3. 应用转向角速率限制
    const auto traj_steering_rate_limited =
        node_param_.enable_steering_rate_limit
            ? smoother_->applySteeringRateLimit(traj_lateral_acc_filtered, false)
            : traj_lateral_acc_filtered;
    
    // 4. 基于车速的重采样
    auto traj_resampled = smoother_->resampleTrajectory(
        traj_steering_rate_limited, 
        current_odometry_ptr_->twist.twist.linear.x,  // 当前车速
        current_odometry_ptr_->pose.pose);
    
    // 5. 设置终点速度为0
    if (!traj_resampled.empty()) {
        traj_resampled.back().longitudinal_velocity_mps = 0.0;
    }
    
    // 6. 裁剪轨迹（从最近点开始）
    TrajectoryPoints clipped;
    clipped.insert(clipped.end(), 
        traj_resampled.begin() + traj_resampled_closest, 
        traj_resampled.end());
    
    // 7. 应用平滑器（QP优化或解析法）
    std::vector<TrajectoryPoints> debug_trajectories;
    if (!smoother_->apply(
            initial_motion.vel, initial_motion.acc, 
            clipped, traj_smoothed, debug_trajectories)) {
        RCLCPP_WARN(get_logger(), "Fail to solve optimization.");
        return false;
    }
    
    // 8. 覆写停止点后的速度为0
    overwriteStopPoint(clipped, traj_smoothed);
    
    // 9. 应用最大速度限制
    trajectory_utils::applyMaximumVelocityLimit(
        traj_resampled_closest, traj_smoothed.size(), 
        node_param_.max_velocity, traj_smoothed);
    
    return true;
}
```

#### **1.2 速度平滑算法实现**

**文件**: `autoware_velocity_smoother/src/smoother/jerk_filtered_smoother.cpp`

```cpp
// 解析法加加速度约束平滑器
bool AnalyticalJerkConstrainedSmoother::apply(
    const double initial_vel, const double initial_acc,
    const TrajectoryPoints & input, TrajectoryPoints & output,
    std::vector<TrajectoryPoints> & debug_trajectories)
{
    // 1. 前向积分（考虑加速度和加加速度约束）
    std::vector<double> forward_velocity;
    if (!forwardJerkFilter(
            initial_vel, initial_acc, input, 
            forward_velocity)) {
        return false;
    }
    
    // 2. 后向积分（从停止点反向计算）
    std::vector<double> backward_velocity;
    if (!backwardJerkFilter(
            input, forward_velocity, 
            backward_velocity)) {
        return false;
    }
    
    // 3. 合并前向和后向结果（取最小值）
    output = input;
    for (size_t i = 0; i < output.size(); ++i) {
        output.at(i).longitudinal_velocity_mps = 
            std::min(forward_velocity.at(i), backward_velocity.at(i));
    }
    
    return true;
}

// 前向加加速度滤波
bool AnalyticalJerkConstrainedSmoother::forwardJerkFilter(
    const double v0, const double a0,
    const TrajectoryPoints & input,
    std::vector<double> & forward_velocity)
{
    forward_velocity.clear();
    forward_velocity.resize(input.size());
    
    double current_vel = v0;
    double current_acc = a0;
    
    for (size_t i = 0; i < input.size(); ++i) {
        // 计算距离增量
        const double ds = (i == 0) ? 0.0 : 
            autoware::motion_utils::calcSignedArcLength(
                input, i - 1, i);
        
        // 考虑加加速度约束的加速度更新
        const double dt = ds / std::max(current_vel, 0.1);
        const double next_acc = std::clamp(
            current_acc + max_jerk_ * dt,
            min_decel_, max_accel_);
        
        // 速度更新: v² = v₀² + 2as
        const double next_vel_squared = 
            current_vel * current_vel + 2.0 * next_acc * ds;
        const double next_vel = 
            std::sqrt(std::max(next_vel_squared, 0.0));
        
        // 限制在参考速度之下
        forward_velocity.at(i) = std::min(
            next_vel, 
            input.at(i).longitudinal_velocity_mps);
        
        current_vel = forward_velocity.at(i);
        current_acc = next_acc;
    }
    
    return true;
}
```

#### **1.3 横向加速度限制**

**文件**: `autoware_velocity_smoother/src/smoother/smoother_base.cpp`

```cpp
// 应用横向加速度限制（弯道减速）
TrajectoryPoints SmootherBase::applyLateralAccelerationFilter(
    const TrajectoryPoints & input,
    [[maybe_unused]] const double v0,
    [[maybe_unused]] const double a0,
    const bool enable_smooth_limit,
    const bool use_resampling) const
{
    if (input.size() < 3) {
        return input;
    }
    
    auto output = input;
    
    // 计算每个点的曲率
    std::vector<double> curvature_v = 
        trajectory_utils::calcTrajectoryCurvatureFrom3Points(output);
    
    for (size_t i = 0; i < output.size(); ++i) {
        // 根据曲率和横向加速度限制计算最大速度
        // v_max = sqrt(a_lat_max / |curvature|)
        const double curvature = std::abs(curvature_v.at(i));
        
        if (curvature < 1e-6) {
            continue;  // 直线段，无需限制
        }
        
        const double v_curvature_max = std::sqrt(
            base_param_.max_lateral_accel / curvature);
        
        // 限制速度不低于最小弯道速度
        const double v_target = std::max(
            v_curvature_max, 
            base_param_.min_curve_velocity);
        
        // 应用速度限制
        output.at(i).longitudinal_velocity_mps = std::min(
            output.at(i).longitudinal_velocity_mps,
            v_target);
    }
    
    return output;
}

// 应用转向角速率限制
TrajectoryPoints SmootherBase::applySteeringRateLimit(
    const TrajectoryPoints & input,
    const bool use_resampling) const
{
    // 计算每个点的转向角
    std::vector<double> steer_angles = 
        calcSteerAngles(input);
    
    for (size_t i = 1; i < input.size(); ++i) {
        // 计算转向角变化率
        const double ds = autoware::motion_utils::calcSignedArcLength(
            input, i - 1, i);
        const double d_steer = steer_angles.at(i) - steer_angles.at(i - 1);
        const double current_v = input.at(i).longitudinal_velocity_mps;
        
        // steer_rate = d_steer / dt = d_steer * v / ds
        const double steer_rate = std::abs(d_steer * current_v / ds);
        
        // 如果转向角速率超限，降低速度
        if (steer_rate > base_param_.max_steering_angle_rate) {
            const double v_limited = 
                base_param_.max_steering_angle_rate * ds / std::abs(d_steer);
            
            output.at(i).longitudinal_velocity_mps = std::min(
                output.at(i).longitudinal_velocity_mps,
                v_limited);
        }
    }
    
    return output;
}
```

---

### 2. Behavior Velocity Planner (行为速度规划器)

#### **2.1 主节点代码**

**文件**: `behavior_velocity_planner/autoware_behavior_velocity_planner/src/node.cpp`

```cpp
// 生成带速度约束的路径
autoware_planning_msgs::msg::Path BehaviorVelocityPlannerNode::generatePath(
    const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg,
    const PlannerData & planner_data)
{
    autoware_planning_msgs::msg::Path output_path_msg;
    
    // 1. 检查是否为前进路径
    const auto is_driving_forward = 
        autoware::motion_utils::isDrivingForward(input_path_msg->points);
    
    if (!is_driving_forward_) {
        // 后退路径不支持，直接转换
        return to_path(*input_path_msg);
    }
    
    // 2. 规划路径速度（调用各个场景模块）
    const auto velocity_planned_path = 
        planner_manager_.planPathVelocity(
            std::make_shared<const PlannerData>(planner_data), 
            *input_path_msg);
    
    // 3. 过滤微小路径点
    const auto filtered_path = 
        autoware::behavior_velocity_planner::filterLitterPathPoint(
            to_path(velocity_planned_path));
    
    // 4. 插值路径
    const auto interpolated_path_msg = 
        autoware::behavior_velocity_planner::interpolatePath(
            filtered_path, 
            forward_path_length_, 
            behavior_output_path_interval_);
    
    // 5. 检查并过滤停止点
    output_path_msg = 
        autoware::behavior_velocity_planner::filterStopPathPoint(
            interpolated_path_msg);
    
    return output_path_msg;
}
```

#### **2.2 停止线模块（设置速度为0）**

**文件**: `behavior_velocity_planner/autoware_behavior_velocity_stop_line_module/src/scene.cpp`

```cpp
// 停止线模块 - 在停止线位置设置速度为0
bool StopLineModule::modifyPathVelocity(PathWithLaneId * path)
{
    // 1. 构建轨迹对象
    auto trajectory = Trajectory::Builder{}.build(path->points);
    
    if (!trajectory) {
        return true;
    }
    
    // 2. 获取ego位置和停止点
    auto [ego_s, stop_point] = 
        getEgoAndStopPoint(*trajectory, *path, 
                          planner_data_->current_odometry->pose, 
                          state_);
    
    if (!stop_point) {
        return true;  // 没有停止点
    }
    
    // 3. 设置停止点之后的所有速度为0 ⭐核心操作
    trajectory->longitudinal_velocity_mps()
        .range(*stop_point, trajectory->length())
        .set(0.0);
    
    // 4. 恢复到path格式
    path->points = trajectory->restore();
    
    // 5. 添加planning factor（停止原因）
    planning_factor_interface_->add(
        path->points, 
        planner_data_->current_odometry->pose,
        trajectory->compute(*stop_point).point.pose,
        autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
        autoware_internal_planning_msgs::msg::SafetyFactorArray{},
        true, 0.0, 0.0, "stopline");
    
    return true;
}
```

#### **2.3 PlannerData结构（速度相关数据）**

**文件**: `behavior_velocity_planner/autoware_behavior_velocity_planner_common/include/planner_data.hpp`

```cpp
struct PlannerData
{
    // 当前速度信息
    geometry_msgs::msg::TwistStamped::ConstSharedPtr current_velocity;
    
    // 速度历史缓冲（10秒）
    static constexpr double velocity_buffer_time_sec = 10.0;
    std::deque<geometry_msgs::msg::TwistStamped> velocity_buffer;
    
    // 外部速度限制
    std::optional<autoware_internal_planning_msgs::msg::VelocityLimit> 
        external_velocity_limit;
    
    // 速度平滑器接口
    std::shared_ptr<autoware::velocity_smoother::SmootherBase> 
        velocity_smoother_;
    
    // 车辆信息（用于速度计算）
    autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
    
    // 最大停车加速度和加加速度阈值
    double max_stop_acceleration_threshold;
    double max_stop_jerk_threshold;
    
    // 系统延迟和响应时间（影响速度规划）
    double system_delay;
    double delay_response_time;
    
    // 判断车辆是否停止
    bool isVehicleStopped(const double stop_duration = 0.0) const;
};
```

---

### 3. Motion Velocity Planner (运动速度规划器)

#### **3.1 主节点代码**

**文件**: `motion_velocity_planner/autoware_motion_velocity_planner/src/node.cpp`

```cpp
// 生成轨迹（带障碍物速度规划）
autoware_planning_msgs::msg::Trajectory 
MotionVelocityPlannerNode::generate_trajectory(
    const autoware::motion_velocity_planner::TrajectoryPoints & 
        input_trajectory_points,
    std::map<std::string, double> & processing_times)
{
    autoware_planning_msgs::msg::Trajectory output_trajectory_msg;
    output_trajectory_msg.points = {
        input_trajectory_points.begin(), 
        input_trajectory_points.end()
    };
    
    // 1. 速度平滑（可选，在规划前）
    const auto smoothed_trajectory_points = [&]() {
        if (smooth_velocity_before_planning_) {
            return smooth_trajectory(input_trajectory_points, planner_data_);
        }
        return input_trajectory_points;
    }();
    
    // 2. 重采样
    TrajectoryPoints resampled_smoothed_trajectory_points;
    if (!smoothed_trajectory_points.empty()) {
        resampled_smoothed_trajectory_points.push_back(
            smoothed_trajectory_points.front());
        
        constexpr auto min_interval_squared = 0.5 * 0.5;
        for (auto i = 1UL; i < smoothed_trajectory_points.size(); ++i) {
            const auto & p = smoothed_trajectory_points[i];
            const auto dist_to_prev_point = 
                autoware_utils::calc_squared_distance2d(
                    resampled_smoothed_trajectory_points.back(), p);
            
            if (dist_to_prev_point > min_interval_squared) {
                resampled_smoothed_trajectory_points.push_back(p);
            }
        }
    }
    
    // 3. 计算从起点的时间
    motion_utils::calculate_time_from_start(
        resampled_smoothed_trajectory_points, 
        planner_data_.current_odometry.pose.pose.position);
    
    // 4. 规划速度（调用各个速度模块）⭐核心
    const auto planning_results = planner_manager_.plan_velocities(
        resampled_smoothed_trajectory_points, planner_data_);
    
    // 5. 应用速度规划结果
    auto output_trajectory_points = resampled_smoothed_trajectory_points;
    for (const auto & result : planning_results) {
        if (!result.stop_points.empty()) {
            // 在停止点设置速度为0
            for (const auto & stop_point : result.stop_points) {
                const size_t stop_idx = stop_point.index;
                for (size_t idx = stop_idx; 
                     idx < output_trajectory_points.size(); ++idx) {
                    output_trajectory_points[idx].longitudinal_velocity_mps = 0.0;
                }
            }
        }
        
        if (!result.slowdown_intervals.empty()) {
            // 应用减速区间
            for (const auto & interval : result.slowdown_intervals) {
                for (size_t idx = interval.from_idx; 
                     idx <= interval.to_idx && idx < output_trajectory_points.size(); 
                     ++idx) {
                    output_trajectory_points[idx].longitudinal_velocity_mps = 
                        std::min(
                            output_trajectory_points[idx].longitudinal_velocity_mps,
                            interval.velocity);
                }
            }
        }
    }
    
    return output_trajectory_msg;
}
```

---

## 🌌 Universe模块速度代码

### 1. Behavior Path Planner（行为路径规划器）

#### **1.1 速度设置示例 - Goal Planner**

**文件**: `behavior_path_planner/autoware_behavior_path_goal_planner_module/src/goal_planner_module.cpp`

```cpp
// 决定pull-over路径的速度
void GoalPlannerModule::decideVelocity(PullOverPath & pull_over_path)
{
    const double current_vel = 
        planner_data_->self_odometry->twist.twist.linear.x;
    
    // 获取第一段路径
    auto & first_path = pull_over_path.partial_paths().front();
    
    // 设置最小速度（不低于pull_over_minimum_velocity）
    const auto vel = static_cast<float>(
        std::max(current_vel, parameters_.pull_over_minimum_velocity));
    
    // 应用速度到路径点
    for (auto & p : first_path.points) {
        p.point.longitudinal_velocity_mps = 
            std::min(p.point.longitudinal_velocity_mps, vel);
    }
}
```

#### **1.2 速度设置示例 - Start Planner**

**文件**: `behavior_path_planner/autoware_behavior_path_start_planner_module/src/start_planner_module.cpp`

```cpp
// 起步时的速度规划
void StartPlannerModule::generateStartPath()
{
    // 计算加速度曲线
    for (size_t i = 0; i < path_points.size(); ++i) {
        double current_velocity;
        
        if (i == 0) {
            // 第一个点使用初始速度
            current_velocity = initial_velocity;
        } else {
            // 累积距离
            double accumulated_distance = 0.0;
            for (size_t j = 0; j < i; ++j) {
                const double dx = path_points[j + 1].x - path_points[j].x;
                const double dy = path_points[j + 1].y - path_points[j].y;
                accumulated_distance += std::sqrt(dx * dx + dy * dy);
            }
            
            // 匀加速公式: v² = v₀² + 2as
            double calculated_velocity = std::sqrt(
                initial_velocity * initial_velocity + 
                2.0 * acceleration * accumulated_distance);
            
            // 限制不超过目标速度
            current_velocity = std::min(calculated_velocity, target_velocity);
        }
        
        // 设置速度
        path_point.point.longitudinal_velocity_mps = current_velocity;
    }
}
```

### 2. Freespace Planner（自由空间规划器）

**文件**: `autoware_freespace_planner/src/autoware_freespace_planner/utils.cpp`

```cpp
// 创建freespace轨迹（恒定速度）
Trajectory create_trajectory(
    const PoseStamped & current_pose, 
    const PlannerWaypoints & planner_waypoints,
    const double & velocity)
{
    Trajectory trajectory;
    trajectory.header = planner_waypoints.header;
    
    for (const auto & awp : planner_waypoints.waypoints) {
        TrajectoryPoint point;
        point.pose = awp.pose.pose;
        point.pose.position.z = current_pose.pose.position.z;
        
        // 速度转换（km/h → m/s）
        point.longitudinal_velocity_mps = velocity / 3.6;
        
        // 根据前进/后退切换速度符号
        point.longitudinal_velocity_mps = 
            (awp.is_back ? -1 : 1) * point.longitudinal_velocity_mps;
        
        trajectory.points.push_back(point);
    }
    
    return trajectory;
}

// 获取部分轨迹（修改起止点速度）
Trajectory get_partial_trajectory(
    const Trajectory & trajectory, 
    const size_t start_index, 
    const size_t end_index)
{
    Trajectory partial_trajectory;
    
    // 复制轨迹点
    for (size_t i = start_index; i <= end_index; ++i) {
        partial_trajectory.points.push_back(trajectory.points.at(i));
    }
    
    // 修改起点速度（使用第二个点的速度）
    if (partial_trajectory.points.size() >= 2) {
        partial_trajectory.points.front().longitudinal_velocity_mps =
            partial_trajectory.points.at(1).longitudinal_velocity_mps;
    }
    
    // 终点速度设为0
    if (!partial_trajectory.points.empty()) {
        partial_trajectory.points.back().longitudinal_velocity_mps = 0;
    }
    
    return partial_trajectory;
}
```

### 3. Obstacle Cruise Planner（障碍物巡航规划器）

**文件**: `autoware_obstacle_cruise_planner/src/pid_based_planner/pid_based_planner.cpp`

```cpp
// 计算巡航轨迹速度
std::vector<TrajectoryPoint> PIDBasedPlanner::generateCruiseTrajectory(
    const std::vector<TrajectoryPoint> & stop_traj_points,
    const PlannerData & planner_data)
{
    // 1. 获取上一次轨迹的最近点速度和加速度
    const auto prev_traj_closest_point = 
        ego_nearest_param_.calcInterpolatedPoint(
            prev_traj_points_, planner_data.ego_pose);
    
    const double v0 = prev_traj_closest_point.longitudinal_velocity_mps;
    const double a0 = prev_traj_closest_point.acceleration_mps2;
    
    // 2. 计算目标加速度（PID控制器）
    const double target_acc = pid_controller_.calc(
        desired_velocity, current_velocity, dt);
    
    // 3. 应用加速度限制生成轨迹
    auto cruise_traj_points = getAccelerationLimitedTrajectory(
        stop_traj_points, planner_data.ego_pose, 
        v0, a0, target_acc, target_jerk_ratio);
    
    // 4. 找到零速度点
    const auto zero_vel_idx_opt = 
        autoware::motion_utils::searchZeroVelocityIndex(cruise_traj_points);
    
    // 5. 零速度点之后全部设为0
    if (zero_vel_idx_opt) {
        for (size_t i = zero_vel_idx_opt.value(); 
             i < cruise_traj_points.size(); ++i) {
            cruise_traj_points.at(i).longitudinal_velocity_mps = 0.0;
        }
    }
    
    return cruise_traj_points;
}
```

---

## 📊 关键数据结构

### 1. TrajectoryPoint（轨迹点）

```cpp
// 定义在 autoware_planning_msgs/msg/TrajectoryPoint.msg
struct TrajectoryPoint {
    geometry_msgs::msg::Pose pose;                // 位置和姿态
    
    // 速度信息 ⭐
    float longitudinal_velocity_mps;              // 纵向速度 [m/s]
    float lateral_velocity_mps;                   // 横向速度 [m/s]
    float heading_rate_rps;                       // 航向角变化率 [rad/s]
    
    // 加速度信息
    float acceleration_mps2;                      // 加速度 [m/s²]
    
    // 其他
    float front_wheel_angle_rad;                  // 前轮转角 [rad]
    float rear_wheel_angle_rad;                   // 后轮转角 [rad]
    float time_from_start;                        // 从起点的时间 [s]
};
```

### 2. VelocityLimit（速度限制）

```cpp
// 定义在 autoware_internal_planning_msgs/msg/VelocityLimit.msg
struct VelocityLimit {
    std_msgs::msg::Header header;
    
    float max_velocity;          // 最大速度限制 [m/s]
    float min_velocity;          // 最小速度限制 [m/s]
    
    bool use_constraints;        // 是否使用加速度约束
    float max_acceleration;      // 最大加速度 [m/s²]
    float max_jerk;              // 最大加加速度 [m/s³]
    
    string sender;               // 发送者标识
};
```

---

## 🧮 速度计算算法

### 1. 基于加加速度约束的速度规划

**核心公式:**

```
给定初始状态 (v₀, a₀) 和约束条件:
  - 最大加速度: a_max
  - 最小加速度: a_min
  - 最大加加速度: j_max

前向传播:
  for each point i:
    dt = ds / v_i                    // 时间步长
    a_i+1 = clip(a_i + j_max * dt, a_min, a_max)
    v_i+1² = v_i² + 2 * a_i+1 * ds   // 匀加速运动
    v_i+1 = min(sqrt(v_i+1²), v_ref_i+1)

后向传播（从停止点）:
  v_end = 0
  for each point i (from end to start):
    a = (v_i² - v_i+1²) / (2 * ds)
    if a < a_min:
      v_i = sqrt(v_i+1² + 2 * a_min * ds)

最终速度:
  v_final_i = min(v_forward_i, v_backward_i)
```

### 2. 横向加速度限制

**核心公式:**

```
横向加速度: a_lat = v² * κ
其中 κ 是曲率

速度限制: v_max = sqrt(a_lat_max / |κ|)

应用:
  if |κ| > threshold:
    v_limited = sqrt(a_lat_max / |κ|)
    v_point = max(v_limited, v_min_curve)
```

### 3. 转向角速率限制

**核心公式:**

```
转向角: δ = atan(L * κ)
其中 L 是轴距, κ 是曲率

转向角速率: δ̇ = dδ/dt = (dδ/ds) * v

速度限制:
  if |δ̇| > δ̇_max:
    v_limited = δ̇_max / |dδ/ds|
```

---

## 💻 实用代码示例

### 1. 自定义速度设置

```cpp
// 在behavior_path_planner模块中设置自定义速度
void MyCustomModule::setVelocityProfile(PathWithLaneId & path)
{
    // 方法1: 直接设置每个点的速度
    for (auto & point : path.points) {
        point.point.longitudinal_velocity_mps = target_velocity_;
    }
    
    // 方法2: 根据曲率设置速度
    for (size_t i = 0; i < path.points.size(); ++i) {
        const double curvature = calcCurvature(path, i);
        const double v_curve = std::sqrt(max_lat_acc_ / std::abs(curvature));
        
        path.points[i].point.longitudinal_velocity_mps = 
            std::min(target_velocity_, v_curve);
    }
    
    // 方法3: 使用工具函数设置停止点
    const size_t stop_idx = findStopIndex(path);
    for (size_t i = stop_idx; i < path.points.size(); ++i) {
        path.points[i].point.longitudinal_velocity_mps = 0.0;
    }
}
```

### 2. 检查和调试速度

```bash
# 1. 实时监控轨迹速度
ros2 topic echo /planning/scenario_planning/trajectory | \
  grep -A 1 "longitudinal_velocity_mps"

# 2. 统计速度分布
ros2 topic echo /planning/scenario_planning/trajectory --once | \
  grep "longitudinal_velocity_mps" | \
  awk '{print $2}' | \
  sort -n | uniq -c

# 3. 检查零速度点
ros2 topic echo /planning/scenario_planning/trajectory --once | \
  grep -B 2 "longitudinal_velocity_mps: 0.0"
```

### 3. 参数调优

```yaml
# velocity_smoother.param.yaml
/**:
  ros__parameters:
    # 加速度约束
    normal.max_acc: 1.0          # 最大加速度 [m/s²]
    normal.min_acc: -2.0         # 最大减速度 [m/s²]
    
    # 加加速度约束
    normal.max_jerk: 1.5         # 最大加加速度 [m/s³]
    normal.min_jerk: -1.5        # 最大减加加速度 [m/s³]
    
    # 横向约束
    max_lateral_accel: 1.0       # 最大横向加速度 [m/s²]
    min_curve_velocity: 2.0      # 最小弯道速度 [m/s]
    
    # 转向约束
    max_steering_angle_rate: 11.5  # 最大转向角速率 [deg/s]
    curvature_threshold: 0.02      # 曲率阈值 [1/m]
```

---

## 🔍 速度调试工具

### 1. RViz2可视化

```python
# 在RViz2中添加以下显示项:
# 1. /planning/scenario_planning/trajectory (Path)
# 2. /planning/.../debug/trajectory_raw (原始轨迹)
# 3. /planning/.../debug/trajectory_vel_lim (速度限制后)
# 4. /planning/.../debug/trajectory_lat_acc_filtered (横向加速度滤波后)
```

### 2. 速度分析脚本

```python
#!/usr/bin/env python3
import rclpy
from autoware_planning_msgs.msg import Trajectory

class VelocityAnalyzer:
    def __init__(self):
        self.node = rclpy.create_node('velocity_analyzer')
        self.sub = self.node.create_subscription(
            Trajectory,
            '/planning/scenario_planning/trajectory',
            self.callback,
            10)
    
    def callback(self, msg):
        velocities = [p.longitudinal_velocity_mps for p in msg.points]
        
        print(f"Velocity Statistics:")
        print(f"  Min: {min(velocities):.2f} m/s")
        print(f"  Max: {max(velocities):.2f} m/s")
        print(f"  Avg: {sum(velocities)/len(velocities):.2f} m/s")
        print(f"  Zero count: {velocities.count(0.0)}")
        
        # 检查速度跳变
        for i in range(1, len(velocities)):
            dv = abs(velocities[i] - velocities[i-1])
            if dv > 1.0:  # 速度变化超过1m/s
                print(f"  Warning: Large velocity change at {i}: {dv:.2f} m/s")

if __name__ == '__main__':
    rclpy.init()
    analyzer = VelocityAnalyzer()
    rclpy.spin(analyzer.node)
```

---

## 📚 相关文件索引

### Core模块

```
autoware_core/planning/
├── autoware_velocity_smoother/
│   ├── src/node.cpp                          ⭐ 主节点
│   ├── src/smoother/
│   │   ├── analytical_jerk_constrained_smoother.cpp  ⭐ 解析法平滑器
│   │   ├── jerk_filtered_smoother.cpp        ⭐ 加加速度滤波
│   │   └── smoother_base.cpp                 ⭐ 基类（横向加速度限制）
│   └── src/trajectory_utils.cpp              ⭐ 轨迹工具函数
│
├── behavior_velocity_planner/
│   ├── autoware_behavior_velocity_planner/
│   │   └── src/node.cpp                      ⭐ 主节点
│   ├── autoware_behavior_velocity_planner_common/
│   │   ├── include/planner_data.hpp          ⭐ 数据结构
│   │   └── src/planner_data.cpp
│   └── autoware_behavior_velocity_stop_line_module/
│       └── src/scene.cpp                     ⭐ 停止线速度设置
│
└── motion_velocity_planner/
    ├── autoware_motion_velocity_planner/
    │   └── src/node.cpp                      ⭐ 主节点
    └── autoware_motion_velocity_planner_common/
        ├── src/polygon_utils.cpp             ⭐ 多边形碰撞检测
        └── src/utils.cpp                     ⭐ 工具函数
```

### Universe模块

```
autoware_universe/planning/
├── behavior_path_planner/
│   ├── autoware_behavior_path_goal_planner_module/
│   │   └── src/goal_planner_module.cpp       ⭐ 泊车速度
│   └── autoware_behavior_path_start_planner_module/
│       └── src/start_planner_module.cpp      ⭐ 起步速度
│
├── autoware_freespace_planner/
│   └── src/utils.cpp                         ⭐ freespace速度
│
└── autoware_obstacle_cruise_planner/
    └── src/pid_based_planner/
        └── pid_based_planner.cpp             ⭐ 巡航速度
```

---

**文档版本**: 1.0  
**最后更新**: 2025-01-17  
**适用版本**: Autoware main分支

