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

#ifndef AUTOWARE__BEHAVIOR_PATH_DYNAMIC_OBSTACLE_AVOIDANCE_MODULE__MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DYNAMIC_OBSTACLE_AVOIDANCE_MODULE__MANAGER_HPP_

#include "autoware/behavior_path_dynamic_obstacle_avoidance_module/scene.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{

/**
 * @brief 动态障碍物避障模块管理器
 * 
 * 该模块负责避让运动中的障碍物,包括:
 * - 切入车辆(从相邻车道切入自车道的车辆)
 * - 切出车辆(从自车道切出到相邻车道的车辆)
 * - 横穿对象(行人、自行车等横穿道路的对象)
 * - 迎面驶来的车辆
 */
class DynamicObstacleAvoidanceModuleManager : public SceneModuleManagerInterface
{
public:
  DynamicObstacleAvoidanceModuleManager()
  : SceneModuleManagerInterface{"dynamic_obstacle_avoidance"}
  {
  }

  /**
   * @brief 初始化模块,加载动态避障参数
   * @param node ROS2节点指针
   */
  void init(rclcpp::Node * node) override;

  /**
   * @brief 创建新的场景模块实例
   * @return 动态障碍物避障模块的唯一指针
   */
  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override
  {
    return std::make_unique<DynamicObstacleAvoidanceModule>(
      name_, *node_, parameters_, rtc_interface_ptr_map_,
      objects_of_interest_marker_interface_ptr_map_, planning_factor_interface_);
  }

  /**
   * @brief 更新模块参数
   * @param parameters 参数列表
   */
  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) override;

private:
  std::shared_ptr<DynamicAvoidanceParameters> parameters_;  ///< 动态避障参数
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DYNAMIC_OBSTACLE_AVOIDANCE_MODULE__MANAGER_HPP_
