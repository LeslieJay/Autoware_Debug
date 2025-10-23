// Copyright 2022 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file arrival_checker.hpp
 * @brief 到达检查器(Arrival Checker)头文件
 * 
 * 该文件定义了 ArrivalChecker 类,用于判断车辆是否已经安全到达目标点。
 * 到达判断综合考虑以下因素:
 * - 与目标点的距离
 * - 与目标航向的角度差
 * - 车辆停止的持续时间
 * 
 * 只有同时满足所有条件时,才认为车辆已到达目标。
 */

#ifndef MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
#define MISSION_PLANNER__ARRIVAL_CHECKER_HPP_

#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace autoware::mission_planner
{

/**
 * @class ArrivalChecker
 * @brief 到达目标点检查器
 * 
 * 该类负责判断车辆是否已经安全到达目标点。采用多重条件判断策略:
 * 
 * 1. **距离检查**: 车辆与目标点的欧氏距离是否小于阈值
 * 2. **角度检查**: 车辆航向与目标航向的差异是否小于阈值
 * 3. **停止检查**: 车辆是否已停止并保持足够时长
 * 
 * 这种多重检查机制确保车辆不仅到达了目标位置附近,而且朝向正确且已完全停止。
 * 
 * 使用示例:
 * @code
 * ArrivalChecker checker(node);
 * checker.set_goal(goal_pose);
 * if (checker.is_arrived(current_pose)) {
 *   // 车辆已到达目标
 * }
 * @endcode
 */
class ArrivalChecker
{
public:
  using PoseWithUuidStamped = autoware_planning_msgs::msg::PoseWithUuidStamped;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  
  /**
   * @brief 构造函数
   * @param node ROS节点指针,用于声明参数
   * 
   * 从节点参数中加载:
   * - arrival_check_distance: 到达检查距离阈值 [m]
   * - arrival_check_angle_deg: 到达检查角度阈值 [度]
   * - arrival_check_duration: 到达检查停止时长 [秒]
   */
  explicit ArrivalChecker(rclcpp::Node * node);
  
  /**
   * @brief 清除目标点设置
   * 
   * 用于路线清除后重置检查器状态。
   * 调用后,is_arrived() 将始终返回 false,直到重新设置目标。
   */
  void set_goal();
  
  /**
   * @brief 设置新的目标点
   * @param goal 带UUID的目标位姿
   * 
   * UUID 用于区分不同的路线。只有当 UUID 匹配时,
   * 才会执行到达检查,这样可以避免检查旧路线的目标点。
   */
  void set_goal(const PoseWithUuidStamped & goal);
  
  /**
   * @brief 检查车辆是否已到达目标点
   * @param pose 当前车辆位姿
   * @return 如果满足所有到达条件则返回 true
   * 
   * 检查逻辑:
   * 1. 检查坐标系是否匹配
   * 2. 计算与目标点的距离,判断是否 < distance_
   * 3. 计算航向角差异,判断是否 < angle_
   * 4. 检查车辆是否已停止并保持 duration_ 时长
   * 
   * 所有条件必须同时满足才返回 true。
   */
  bool is_arrived(const PoseStamped & pose) const;

private:
  double distance_;  ///< 到达判定距离阈值 [m] (默认: 2.0m)
  double angle_;     ///< 到达判定角度阈值 [rad] (默认: 45度)
  double duration_;  ///< 停止持续时长要求 [s] (默认: 1.0s)
  
  std::optional<PoseWithUuidStamped> goal_with_uuid_;  ///< 当前目标位姿(带UUID)
  rclcpp::Subscription<PoseWithUuidStamped>::SharedPtr sub_goal_;  ///< 目标修改订阅器
  autoware::motion_utils::VehicleStopChecker vehicle_stop_checker_;  ///< 车辆停止检查器
};

}  // namespace autoware::mission_planner

#endif  // MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
