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

#include "interface.hpp"

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::State;
using autoware::route_handler::Direction;

/**
 * @brief 通过变道避障接口的构造函数实现
 * 
 * 初始化变道避障模块,创建AvoidanceByLaneChange场景对象并传递给基类。
 */
AvoidanceByLaneChangeInterface::AvoidanceByLaneChangeInterface(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<LaneChangeParameters> & parameters,
  const std::shared_ptr<AvoidanceByLCParameters> & avoidance_by_lane_change_parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface)
: LaneChangeInterface{
    name,
    node,
    parameters,
    rtc_interface_ptr_map,
    objects_of_interest_marker_interface_ptr_map,
    planning_factor_interface,
    std::make_unique<AvoidanceByLaneChange>(parameters, avoidance_by_lane_change_parameters)}
{
}

/**
 * @brief 判断是否需要执行变道避障
 * 
 * 通过以下条件判断是否需要执行:
 * 1. isLaneChangeRequired(): 是否需要进行变道
 * 2. specialRequiredCheck(): 通过变道避障的特殊检查(如是否有障碍物需要避让)
 * 3. isValidPath(): 生成的路径是否有效
 * 
 * @return true 满足所有条件,需要执行变道避障
 * @return false 不满足条件,不需要执行
 */
bool AvoidanceByLaneChangeInterface::isExecutionRequested() const
{
  return module_type_->isLaneChangeRequired() && module_type_->specialRequiredCheck() &&
         module_type_->isValidPath();
}

/**
 * @brief 模块激活时的入口处理
 * 
 * 当模块首次被激活时调用此函数。
 * 设置模块状态为等待审批(WAITING_FOR_EXECUTION),
 * 需要外部批准后才能真正执行变道动作。
 */
void AvoidanceByLaneChangeInterface::processOnEntry()
{
  waitApproval();
}

/**
 * @brief 更新RTC(运行时协调)状态
 * 
 * 根据变道方向(左或右)更新对应的RTC状态。
 * RTC用于与外部系统(如远程操作员或其他车辆)协调动作。
 * 
 * 状态包括:
 * - 变道方向(左/右)
 * - 执行状态(等待审批/正在运行)
 * - 开始和结束距离
 * - 当前时间戳
 * 
 * @param start_distance 变道开始点距离当前车辆的距离(米)
 * @param finish_distance 变道结束点距离当前车辆的距离(米)
 */
void AvoidanceByLaneChangeInterface::updateRTCStatus(
  const double start_distance, const double finish_distance)
{
  // 获取变道方向(左或右),并转换为字符串
  const auto direction = std::invoke([&]() -> std::string {
    const auto dir = module_type_->getDirection();
    return (dir == Direction::LEFT) ? "left" : "right";
  });

  // 根据是否在等待审批来确定状态
  const auto state = isWaitingApproval() ? State::WAITING_FOR_EXECUTION : State::RUNNING;

  // 更新对应方向的RTC协调状态
  rtc_interface_ptr_map_.at(direction)->updateCooperateStatus(
    uuid_map_.at(direction), isExecutionReady(), state, start_distance, finish_distance,
    clock_->now());
}
}  // namespace autoware::behavior_path_planner
