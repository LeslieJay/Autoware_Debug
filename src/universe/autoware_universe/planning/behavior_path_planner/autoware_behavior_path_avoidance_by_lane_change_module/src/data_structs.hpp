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
#ifndef DATA_STRUCTS_HPP_
#define DATA_STRUCTS_HPP_

#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::AvoidanceParameters;

/**
 * @brief 通过变道避障的参数结构体
 * 
 * 继承自AvoidanceParameters,增加了变道避障专用的参数。
 * 这些参数用于控制何时通过变道来避开障碍物,以及变道的执行条件。
 */
struct AvoidanceByLCParameters : public AvoidanceParameters
{
  /**
   * @brief 执行变道避障的最小纵向距离裕度(米)
   * 
   * 只有当目标障碍物与车辆的纵向距离大于此参数时,才会执行变道避障。
   * 这确保了车辆有足够的距离来完成变道动作。
   */
  double execute_object_longitudinal_margin{0.0};

  /**
   * @brief 是否要求变道在障碍物之前完成
   * 
   * 如果为true,则只有当变道终点在障碍物之前时才执行变道避障。
   * 这样可以确保变道完成后不会与障碍物发生冲突。
   */
  bool execute_only_when_lane_change_finish_before_object{false};

  /**
   * @brief 构造函数
   * 
   * @param param 基础避障参数
   */
  explicit AvoidanceByLCParameters(const AvoidanceParameters & param) : AvoidanceParameters(param)
  {
  }
};
}  // namespace autoware::behavior_path_planner

#endif  // DATA_STRUCTS_HPP_
