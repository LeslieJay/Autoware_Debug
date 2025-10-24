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

#ifndef INTERFACE_HPP_
#define INTERFACE_HPP_

#include "autoware/behavior_path_lane_change_module/interface.hpp"
#include "data_structs.hpp"
#include "scene.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::LaneChangeInterface;
using autoware::behavior_path_planner::ObjectsOfInterestMarkerInterface;
using autoware::behavior_path_planner::RTCInterface;

/**
 * @brief 通过变道避障的接口类
 * 
 * 该类继承自LaneChangeInterface,实现了通过变道来避开静态障碍物的功能。
 * 当遇到前方有障碍物时,该模块会评估是否可以通过变换车道来避开障碍物,
 * 而不是减速等待或在当前车道内规避。
 */
class AvoidanceByLaneChangeInterface : public LaneChangeInterface
{
public:
  /**
   * @brief 构造函数
   * 
   * @param name 模块名称
   * @param node ROS2节点引用
   * @param parameters 变道参数配置
   * @param avoidance_by_lane_change_parameters 通过变道避障的专用参数
   * @param rtc_interface_ptr_map RTC(运行时协调)接口映射表
   * @param objects_of_interest_marker_interface_ptr_map 兴趣对象标记接口映射表
   * @param planning_factor_interface 规划因子接口
   */
  AvoidanceByLaneChangeInterface(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<LaneChangeParameters> & parameters,
    const std::shared_ptr<AvoidanceByLCParameters> & avoidance_by_lane_change_parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface);

  /**
   * @brief 检查是否需要执行该模块
   * 
   * 判断当前场景是否需要通过变道来避障。
   * 
   * @return true 需要执行变道避障
   * @return false 不需要执行
   */
  bool isExecutionRequested() const override;

  /**
   * @brief 模块激活时的入口处理函数
   * 
   * 当模块被激活时调用,执行初始化操作,如等待审批等。
   */
  void processOnEntry() override;

protected:
  /**
   * @brief 更新RTC(运行时协调)状态
   * 
   * 更新与外部系统的协调状态,包括变道的开始和结束距离。
   * 
   * @param start_distance 变道开始距离(相对于当前车辆位置)
   * @param finish_distance 变道结束距离(相对于当前车辆位置)
   */
  void updateRTCStatus(const double start_distance, const double finish_distance) override;
};
}  // namespace autoware::behavior_path_planner

#endif  // INTERFACE_HPP_
