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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "autoware/behavior_path_lane_change_module/manager.hpp"
#include "data_structs.hpp"
#include "interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::LaneChangeModuleManager;
using autoware::behavior_path_planner::LaneChangeModuleType;
using autoware::behavior_path_planner::SceneModuleInterface;

using SMIPtr = std::unique_ptr<SceneModuleInterface>;

/**
 * @brief 通过变道避障模块的管理器类
 * 
 * 该类继承自LaneChangeModuleManager,负责管理通过变道来避障的模块实例。
 * 它处理模块的初始化、参数加载和场景模块实例的创建。
 * 
 * 该管理器作为ROS2插件被加载,为behavior_path_planner提供变道避障功能。
 */
class AvoidanceByLaneChangeModuleManager : public LaneChangeModuleManager
{
public:
  /**
   * @brief 构造函数
   * 
   * 初始化管理器,设置模块名称为"avoidance_by_lane_change",
   * 方向为NONE(因为可以向左或向右变道),
   * 模块类型为AVOIDANCE_BY_LANE_CHANGE。
   */
  AvoidanceByLaneChangeModuleManager()
  : LaneChangeModuleManager(
      "avoidance_by_lane_change", autoware::route_handler::Direction::NONE,
      LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE)
  {
  }

  /**
   * @brief 初始化模块管理器
   * 
   * 从ROS2参数服务器加载所有相关参数,包括:
   * - 变道参数
   * - 避障参数
   * - 目标对象参数(针对不同类型的障碍物)
   * - 安全检查参数
   * 
   * @param node ROS2节点指针,用于访问参数服务器
   */
  void init(rclcpp::Node * node) override;

  /**
   * @brief 创建新的场景模块实例
   * 
   * 创建一个AvoidanceByLaneChangeInterface实例,
   * 该实例将处理具体的变道避障逻辑。
   * 
   * @return SMIPtr 场景模块接口的唯一指针
   */
  SMIPtr createNewSceneModuleInstance() override;

private:
  /**
   * @brief 避障参数的共享指针
   * 
   * 存储从配置文件加载的避障相关参数,
   * 这些参数将被传递给创建的场景模块实例。
   */
  std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters_;
};
}  // namespace autoware::behavior_path_planner

#endif  // MANAGER_HPP_
