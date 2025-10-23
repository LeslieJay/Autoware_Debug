// Copyright 2019 Autoware Foundation
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
 * @file mission_planner.hpp
 * @brief 任务规划器(Mission Planner)头文件
 * 
 * 该文件定义了MissionPlanner类,负责从当前位置到目标位置的全局路线规划。
 * 主要功能包括:
 * - 接收目标位置并规划全局路线(基于Lanelet2地图)
 * - 支持重新规划路线(reroute)
 * - 检查车辆是否到达目标点
 * - 提供路线可视化
 */

#ifndef MISSION_PLANNER__MISSION_PLANNER_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_HPP_

#include "arrival_checker.hpp"

#include <autoware/mission_planner/mission_planner_plugin.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/ros/logger_level_configure.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/route_state.hpp>
#include <autoware_internal_planning_msgs/srv/clear_route.hpp>
#include <autoware_internal_planning_msgs/srv/set_lanelet_route.hpp>
#include <autoware_internal_planning_msgs/srv/set_waypoint_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::mission_planner
{

// 消息类型别名定义
using autoware_adapi_v1_msgs::msg::OperationModeState;  // 运行模式状态
using autoware_internal_planning_msgs::msg::RouteState;  // 路线状态
using autoware_internal_planning_msgs::srv::ClearRoute;  // 清除路线服务
using autoware_internal_planning_msgs::srv::SetLaneletRoute;  // 设置基于车道的路线服务
using autoware_internal_planning_msgs::srv::SetWaypointRoute;  // 设置基于路点的路线服务
using autoware_map_msgs::msg::LaneletMapBin;  // Lanelet2地图二进制消息
using autoware_planning_msgs::msg::LaneletPrimitive;  // 车道基元(最小车道单元)
using autoware_planning_msgs::msg::LaneletRoute;  // 车道路线
using autoware_planning_msgs::msg::LaneletSegment;  // 车道段
using autoware_planning_msgs::msg::PoseWithUuidStamped;  // 带UUID的位姿
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Header;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::MarkerArray;

/**
 * @class MissionPlanner
 * @brief 任务规划器主类
 * 
 * 该类负责全局路线规划,包括:
 * - 接收目标位置并规划从当前位置到目标位置的路线
 * - 支持动态重新规划路线
 * - 检查车辆是否安全到达目标点
 * - 发布路线状态和可视化信息
 * 
 * 路线规划采用插件架构,默认使用基于Lanelet2的规划算法。
 * 规划过程不考虑动态障碍物,仅基于静态地图信息。
 */
class MissionPlanner : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   * @param options ROS2节点选项
   */
  explicit MissionPlanner(const rclcpp::NodeOptions & options);
  
  /**
   * @brief 发布规划处理时间
   * @param stop_watch 计时器
   */
  void publish_processing_time(autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch);

private:
  // ========== 核心组件 ==========
  ArrivalChecker arrival_checker_;  ///< 到达检查器,用于判断是否到达目标点
  pluginlib::ClassLoader<PlannerPlugin> plugin_loader_;  ///< 插件加载器
  std::shared_ptr<PlannerPlugin> planner_;  ///< 规划器插件实例(默认为DefaultPlanner)

  // ========== 坐标变换 ==========
  std::string map_frame_;  ///< 地图坐标系名称(通常为"map")
  tf2_ros::Buffer tf_buffer_;  ///< TF2缓冲区
  tf2_ros::TransformListener tf_listener_;  ///< TF2监听器
  /**
   * @brief 将位姿变换到地图坐标系
   * @param pose 原始位姿
   * @param header 包含源坐标系信息的消息头
   * @return 变换后的位姿(地图坐标系)
   */
  Pose transform_pose(const Pose & pose, const Header & header);

  // ========== 服务接口 ==========
  rclcpp::Service<ClearRoute>::SharedPtr srv_clear_route;  ///< 清除当前路线服务
  rclcpp::Service<SetLaneletRoute>::SharedPtr srv_set_lanelet_route;  ///< 设置基于车道段的路线服务
  rclcpp::Service<SetWaypointRoute>::SharedPtr srv_set_waypoint_route;  ///< 设置基于路点的路线服务
  
  // ========== 发布器 ==========
  rclcpp::Publisher<RouteState>::SharedPtr pub_state_;  ///< 路线状态发布器
  rclcpp::Publisher<LaneletRoute>::SharedPtr pub_route_;  ///< 路线发布器
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;  ///< 可视化标记发布器

  // ========== 订阅器 ==========
  rclcpp::Subscription<PoseWithUuidStamped>::SharedPtr sub_modified_goal_;  ///< 修改后的目标点订阅器
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;  ///< 车辆里程计订阅器
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_state_;  ///< 运行模式订阅器
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_vector_map_;  ///< 矢量地图订阅器

  // ========== 缓存数据 ==========
  Odometry::ConstSharedPtr odometry_;  ///< 最新的车辆里程计数据
  OperationModeState::ConstSharedPtr operation_mode_state_;  ///< 最新的运行模式状态
  LaneletMapBin::ConstSharedPtr map_ptr_;  ///< 地图消息指针
  RouteState state_;  ///< 当前路线状态
  LaneletRoute::ConstSharedPtr current_route_;  ///< 当前活动路线
  lanelet::LaneletMapPtr lanelet_map_ptr_{nullptr};  ///< Lanelet2地图对象指针

  // ========== 回调函数 ==========
  /**
   * @brief 里程计消息回调函数
   * @param msg 里程计消息
   * 
   * 功能:
   * - 更新最新的车辆位姿和速度
   * - 检查是否到达目标点(仅在路线已设置状态下)
   */
  void on_odometry(const Odometry::ConstSharedPtr msg);
  
  /**
   * @brief 运行模式状态回调函数
   * @param msg 运行模式状态消息
   */
  void on_operation_mode_state(const OperationModeState::ConstSharedPtr msg);
  
  /**
   * @brief 地图消息回调函数
   * @param msg Lanelet2地图二进制消息
   * 
   * 功能:将二进制地图消息转换为Lanelet2地图对象
   */
  void on_map(const LaneletMapBin::ConstSharedPtr msg);

  /**
   * @brief 清除路线服务回调函数
   * @param req 服务请求
   * @param res 服务响应
   */
  void on_clear_route(
    const ClearRoute::Request::SharedPtr req, const ClearRoute::Response::SharedPtr res);
  
  /**
   * @brief 设置基于车道的路线服务回调函数
   * @param req 服务请求,包含车道段序列和目标位姿
   * @param res 服务响应
   * 
   * 主要步骤:
   * 1. 检查是否允许重新规划
   * 2. 创建路线
   * 3. 检查重新规划安全性(自动驾驶模式下)
   * 4. 发布新路线
   */
  void on_set_lanelet_route(
    const SetLaneletRoute::Request::SharedPtr req, const SetLaneletRoute::Response::SharedPtr res);
  
  /**
   * @brief 设置基于路点的路线服务回调函数
   * @param req 服务请求,包含路点序列和目标位姿
   * @param res 服务响应
   */
  void on_set_waypoint_route(
    const SetWaypointRoute::Request::SharedPtr req,
    const SetWaypointRoute::Response::SharedPtr res);

  // ========== 路线管理 ==========
  /**
   * @brief 改变路线状态并发布
   * @param state 新的路线状态
   */
  void change_state(RouteState::_state_type state);
  
  /**
   * @brief 清除当前路线
   */
  void change_route();
  
  /**
   * @brief 设置新路线
   * @param route 新路线
   */
  void change_route(const LaneletRoute & route);
  
  /**
   * @brief 取消路线规划(恢复到之前的路线)
   */
  void cancel_route();
  
  /**
   * @brief 从车道路线请求创建路线
   * @param req 设置车道路线请求
   * @return 创建的路线
   */
  LaneletRoute create_route(const SetLaneletRoute::Request & req);
  
  /**
   * @brief 从路点路线请求创建路线
   * @param req 设置路点路线请求
   * @return 创建的路线
   */
  LaneletRoute create_route(const SetWaypointRoute::Request & req);
  
  /**
   * @brief 从车道段创建路线
   * @param header 消息头
   * @param segments 车道段序列
   * @param goal_pose 目标位姿
   * @param uuid 唯一标识符
   * @param allow_goal_modification 是否允许修改目标点
   * @return 创建的路线
   */
  LaneletRoute create_route(
    const Header & header, const std::vector<LaneletSegment> & segments, const Pose & goal_pose,
    const UUID & uuid, const bool allow_goal_modification);
  
  /**
   * @brief 从路点序列创建路线
   * @param header 消息头
   * @param waypoints 路点序列
   * @param start_pose 起始位姿
   * @param goal_pose 目标位姿
   * @param uuid 唯一标识符
   * @param allow_goal_modification 是否允许修改目标点
   * @return 创建的路线
   * 
   * 该函数调用planner插件的plan方法进行路径搜索
   */
  LaneletRoute create_route(
    const Header & header, const std::vector<Pose> & waypoints, const Pose & start_pose,
    const Pose & goal_pose, const UUID & uuid, const bool allow_goal_modification);

  /**
   * @brief 发布位姿日志信息
   * @param pose 位姿
   * @param pose_type 位姿类型("initial"或"goal")
   */
  void publish_pose_log(const Pose & pose, const std::string & pose_type);

  // ========== 初始化检查 ==========
  rclcpp::TimerBase::SharedPtr data_check_timer_;  ///< 数据检查定时器
  /**
   * @brief 检查初始化是否完成
   * 
   * 检查项目:
   * - 规划器是否就绪(地图已加载)
   * - 里程计数据是否已接收
   * 
   * 当所有数据就绪后,停止定时器并发布UNSET状态
   */
  void check_initialization();
  bool is_mission_planner_ready_;  ///< 任务规划器是否就绪标志

  // ========== 重新规划参数 ==========
  double reroute_time_threshold_;  ///< 重新规划时间阈值[s],用于计算安全距离
  double minimum_reroute_length_;  ///< 最小重新规划长度[m]
  bool allow_reroute_in_autonomous_mode_;  ///< 自动驾驶模式下是否允许重新规划
  
  /**
   * @brief 检查重新规划的安全性
   * @param original_route 原始路线
   * @param target_route 目标路线
   * @return 如果重新规划安全则返回true
   * 
   * 安全性判断逻辑:
   * 1. 找到两条路线的公共部分
   * 2. 计算从当前位置到路线分叉点的距离
   * 3. 计算安全距离 = max(当前速度 × 时间阈值, 最小距离)
   * 4. 如果公共部分长度 >= 安全距离,则认为安全
   * 
   * 这样可以确保车辆有足够的距离平稳过渡到新路线
   */
  bool check_reroute_safety(const LaneletRoute & original_route, const LaneletRoute & target_route);

  // ========== 调试和监控 ==========
  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;  ///< 日志级别配置器
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;  ///< 处理时间发布器(用于性能监控)
};

}  // namespace autoware::mission_planner

#endif  // MISSION_PLANNER__MISSION_PLANNER_HPP_
