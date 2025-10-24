// Copyright 2023 TIER IV, Inc.
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

#include "manager.hpp"

#include "autoware/behavior_path_static_obstacle_avoidance_module/parameter_helper.hpp"
#include "autoware_utils/ros/parameter.hpp"
#include "autoware_utils/ros/update_param.hpp"
#include "data_structs.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::getParameter;
using autoware::behavior_path_planner::ObjectParameter;

/**
 * @brief 初始化通过变道避障模块管理器
 * 
 * 该函数从ROS2参数服务器加载所有必要的配置参数,包括:
 * 1. 基础变道参数
 * 2. 通过变道避障的专用参数
 * 3. 针对不同类型障碍物的参数(汽车、卡车、行人等)
 * 4. 目标筛选参数
 * 5. 安全检查参数
 * 
 * @param node ROS2节点指针,用于访问参数服务器
 */

void AvoidanceByLaneChangeModuleManager::init(rclcpp::Node * node)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_utils::get_or_declare_parameter;

  // 初始化管理器接口,支持左右两个方向的变道
  initInterface(node, {"left", "right"});

  // 初始化变道管理器的基础参数
  LaneChangeModuleManager::initParams(node);

  // 获取基础避障参数
  const auto avoidance_params = getParameter(node);
  AvoidanceByLCParameters p(avoidance_params);

  // 加载通过变道避障的专用参数
  {
    const std::string ns = "avoidance_by_lane_change.";
    // 执行变道避障所需的最小纵向距离裕度
    p.execute_object_longitudinal_margin =
      get_or_declare_parameter<double>(*node, ns + "execute_object_longitudinal_margin");
    // 是否要求变道必须在障碍物之前完成
    p.execute_only_when_lane_change_finish_before_object = get_or_declare_parameter<bool>(
      *node, ns + "execute_only_when_lane_change_finish_before_object");
  }

  // 加载通用参数
  {
    const std::string ns = "avoidance.";
    // 规划时的路径重采样间隔
    p.resample_interval_for_planning =
      get_or_declare_parameter<double>(*node, ns + "resample_interval_for_planning");
    // 输出时的路径重采样间隔
    p.resample_interval_for_output =
      get_or_declare_parameter<double>(*node, ns + "resample_interval_for_output");
  }

  // 加载目标障碍物参数
  {
    // Lambda函数:根据命名空间加载对象参数
    const auto get_object_param = [&](std::string && ns) {
      ObjectParameter param{};
      // 判断物体是否移动的速度阈值
      param.moving_speed_threshold =
        get_or_declare_parameter<double>(*node, ns + "th_moving_speed");
      // 判断物体是否移动的时间阈值
      param.moving_time_threshold = get_or_declare_parameter<double>(*node, ns + "th_moving_time");
      // 物体多边形最大扩展比例
      param.max_expand_ratio = get_or_declare_parameter<double>(*node, ns + "max_expand_ratio");
      // 物体包络线的缓冲裕度
      param.envelope_buffer_margin =
        get_or_declare_parameter<double>(*node, ns + "envelope_buffer_margin");
      // 横向软裕度(警告距离)
      param.lateral_soft_margin =
        get_or_declare_parameter<double>(*node, ns + "lateral_margin.soft_margin");
      // 横向硬裕度(最小安全距离)
      param.lateral_hard_margin =
        get_or_declare_parameter<double>(*node, ns + "lateral_margin.hard_margin");
      // 停放车辆的横向硬裕度
      param.lateral_hard_margin_for_parked_vehicle = get_or_declare_parameter<double>(
        *node, ns + "lateral_margin.hard_margin_for_parked_vehicle");
      return param;
    };

    const std::string ns = "avoidance_by_lane_change.target_object.";
    // 为不同类型的障碍物加载专用参数
    p.object_parameters.emplace(
      ObjectClassification::MOTORCYCLE, get_object_param(ns + "motorcycle."));  // 摩托车
    p.object_parameters.emplace(ObjectClassification::CAR, get_object_param(ns + "car."));  // 汽车
    p.object_parameters.emplace(ObjectClassification::TRUCK, get_object_param(ns + "truck."));  // 卡车
    p.object_parameters.emplace(ObjectClassification::TRAILER, get_object_param(ns + "trailer."));  // 拖车
    p.object_parameters.emplace(ObjectClassification::BUS, get_object_param(ns + "bus."));  // 公交车
    p.object_parameters.emplace(
      ObjectClassification::PEDESTRIAN, get_object_param(ns + "pedestrian."));  // 行人
    p.object_parameters.emplace(ObjectClassification::BICYCLE, get_object_param(ns + "bicycle."));  // 自行车
    p.object_parameters.emplace(ObjectClassification::UNKNOWN, get_object_param(ns + "unknown."));  // 未知类型

    // 障碍物多边形扩展的距离范围
    p.lower_distance_for_polygon_expansion =
      get_or_declare_parameter<double>(*node, ns + "lower_distance_for_polygon_expansion");
    p.upper_distance_for_polygon_expansion =
      get_or_declare_parameter<double>(*node, ns + "upper_distance_for_polygon_expansion");
  }

  // 加载目标筛选参数
  {
    // Lambda函数:设置特定类型对象是否为避障目标
    const auto set_target_flag = [&](const uint8_t & object_type, const std::string & ns) {
      if (p.object_parameters.count(object_type) == 0) {
        return;
      }
      p.object_parameters.at(object_type).is_avoidance_target =
        get_or_declare_parameter<bool>(*node, ns);
    };

    const std::string ns = "avoidance.target_filtering.";
    // 为每种类型的障碍物设置是否需要避障
    set_target_flag(ObjectClassification::CAR, ns + "target_type.car");
    set_target_flag(ObjectClassification::TRUCK, ns + "target_type.truck");
    set_target_flag(ObjectClassification::TRAILER, ns + "target_type.trailer");
    set_target_flag(ObjectClassification::BUS, ns + "target_type.bus");
    set_target_flag(ObjectClassification::PEDESTRIAN, ns + "target_type.pedestrian");
    set_target_flag(ObjectClassification::BICYCLE, ns + "target_type.bicycle");
    set_target_flag(ObjectClassification::MOTORCYCLE, ns + "target_type.motorcycle");
    set_target_flag(ObjectClassification::UNKNOWN, ns + "target_type.unknown");

    // 检查障碍物的最大距离(相对于目标点)
    p.object_check_goal_distance =
      get_or_declare_parameter<double>(*node, ns + "object_check_goal_distance");
    // 障碍物最后一次被观察到的时间阈值
    p.object_last_seen_threshold =
      get_or_declare_parameter<double>(*node, ns + "max_compensation_time");
  }

  // 加载停放车辆的参数
  {
    const std::string ns = "avoidance.target_filtering.parked_vehicle.";
    // 判断物体是否在中心线上的距离阈值
    p.threshold_distance_object_is_on_center =
      get_or_declare_parameter<double>(*node, ns + "th_offset_from_centerline");
    // 判断车道是否可横向移动的比例阈值
    p.object_check_shiftable_ratio =
      get_or_declare_parameter<double>(*node, ns + "th_shiftable_ratio");
    // 最小路肩宽度
    p.object_check_min_road_shoulder_width =
      get_or_declare_parameter<double>(*node, ns + "min_road_shoulder_width");
  }

  // 加载模糊车辆(行为不明确的车辆)的避障参数
  {
    const std::string ns = "avoidance.target_filtering.avoidance_for_ambiguous_vehicle.";
    // 处理模糊车辆的策略
    p.policy_ambiguous_vehicle = get_or_declare_parameter<std::string>(*node, ns + "policy");
    // 等待观察的目标行为类型列表
    p.wait_and_see_target_behaviors = get_or_declare_parameter<std::vector<std::string>>(
      *node, ns + "wait_and_see.target_behaviors");
    // 等待观察时的最近距离阈值
    p.wait_and_see_th_closest_distance =
      get_or_declare_parameter<double>(*node, ns + "wait_and_see.th_closest_distance");
    // 判断车辆为模糊状态的停止时间阈值
    p.time_threshold_for_ambiguous_vehicle =
      get_or_declare_parameter<double>(*node, ns + "condition.th_stopped_time");
    // 判断车辆为模糊状态的移动距离阈值
    p.distance_threshold_for_ambiguous_vehicle =
      get_or_declare_parameter<double>(*node, ns + "condition.th_moving_distance");
    // 忽略交通信号灯前方区域的距离
    p.object_ignore_section_traffic_light_in_front_distance =
      get_or_declare_parameter<double>(*node, ns + "ignore_area.traffic_light.front_distance");
    // 忽略人行横道前方区域的距离
    p.object_ignore_section_crosswalk_in_front_distance =
      get_or_declare_parameter<double>(*node, ns + "ignore_area.crosswalk.front_distance");
    // 忽略人行横道后方区域的距离
    p.object_ignore_section_crosswalk_behind_distance =
      get_or_declare_parameter<double>(*node, ns + "ignore_area.crosswalk.behind_distance");
  }

  // 加载避障机动的纵向参数
  {
    const std::string ns = "avoidance.avoidance.longitudinal.";
    // 最小准备时间
    p.min_prepare_time = get_or_declare_parameter<double>(*node, ns + "min_prepare_time");
    // 最大准备时间
    p.max_prepare_time = get_or_declare_parameter<double>(*node, ns + "max_prepare_time");
    // 最小准备距离
    p.min_prepare_distance = get_or_declare_parameter<double>(*node, ns + "min_prepare_distance");
    // 最小减速速度
    p.min_slow_down_speed = get_or_declare_parameter<double>(*node, ns + "min_slow_down_speed");
    // 减速缓冲速度
    p.buf_slow_down_speed = get_or_declare_parameter<double>(*node, ns + "buf_slow_down_speed");
    // 标称避障速度
    p.nominal_avoidance_speed =
      get_or_declare_parameter<double>(*node, ns + "nominal_avoidance_speed");
  }

  // 加载检测区域参数
  {
    const std::string ns = "avoidance.target_filtering.detection_area.";
    // 是否使用静态检测区域
    p.use_static_detection_area = get_or_declare_parameter<bool>(*node, ns + "static");
    // 前方最小检测距离
    p.object_check_min_forward_distance =
      get_or_declare_parameter<double>(*node, ns + "min_forward_distance");
    // 前方最大检测距离
    p.object_check_max_forward_distance =
      get_or_declare_parameter<double>(*node, ns + "max_forward_distance");
    // 后方检测距离
    p.object_check_backward_distance =
      get_or_declare_parameter<double>(*node, ns + "backward_distance");
  }

  // 加载安全检查参数
  {
    const std::string ns = "avoidance.safety_check.";
    // 滞后因子扩展率(用于避免频繁切换状态)
    p.hysteresis_factor_expand_rate =
      get_or_declare_parameter<double>(*node, ns + "hysteresis_factor_expand_rate");
  }

  // 保存加载的参数
  avoidance_parameters_ = std::make_shared<AvoidanceByLCParameters>(p);
}

/**
 * @brief 创建新的场景模块实例
 * 
 * 创建并返回一个AvoidanceByLaneChangeInterface实例,
 * 该实例将使用初始化时加载的所有参数。
 * 
 * @return SMIPtr 指向新创建的场景模块接口的唯一指针
 */
SMIPtr AvoidanceByLaneChangeModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<AvoidanceByLaneChangeInterface>(
    name_, *node_, parameters_, avoidance_parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_, planning_factor_interface_);
}

}  // namespace autoware::behavior_path_planner

// 将该管理器类导出为ROS2插件,使其可以被behavior_path_planner动态加载
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::AvoidanceByLaneChangeModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
