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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include "autoware/behavior_path_lane_change_module/scene.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/helper.hpp"
#include "data_structs.hpp"

#include <memory>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::DebugData;
using AvoidanceDebugData = DebugData;
using autoware::behavior_path_planner::AvoidancePlanningData;
using autoware::behavior_path_planner::LaneChangeParameters;
using autoware::behavior_path_planner::NormalLaneChange;
using autoware::behavior_path_planner::ObjectData;
using autoware::behavior_path_planner::ObjectDataArray;
using autoware::behavior_path_planner::PredictedObject;
using autoware::behavior_path_planner::helper::static_obstacle_avoidance::AvoidanceHelper;

/**
 * @brief 通过变道避障的场景类
 * 
 * 该类继承自NormalLaneChange,实现了通过变道来避开静态障碍物的具体逻辑。
 * 主要功能包括:
 * - 检测和评估需要避让的障碍物
 * - 判断是否需要执行变道避障
 * - 计算避障所需的最小距离和横向偏移
 * - 管理避障目标对象的状态
 */
class AvoidanceByLaneChange : public NormalLaneChange
{
public:
  /**
   * @brief 构造函数
   * 
   * @param parameters 变道参数
   * @param avoidance_by_lane_change_parameters 通过变道避障的专用参数
   */
  AvoidanceByLaneChange(
    const std::shared_ptr<LaneChangeParameters> & parameters,
    std::shared_ptr<AvoidanceByLCParameters> avoidance_by_lane_change_parameters);

  /**
   * @brief 特殊的执行条件检查
   * 
   * 检查是否满足执行变道避障的条件,包括:
   * - 是否有需要避让的目标障碍物
   * - 距离障碍物是否足够远以完成变道
   * 
   * @return true 满足执行条件
   * @return false 不满足执行条件
   */
  bool specialRequiredCheck() const override;

  /**
   * @brief 特殊的过期检查
   * 
   * 检查变道避障是否不再需要(例如障碍物消失或已通过)
   * 
   * @return true 模块应该过期/终止
   * @return false 模块仍然需要
   */
  bool specialExpiredCheck() const override;

  /**
   * @brief 更新特殊数据
   * 
   * 更新避障相关的数据,包括:
   * - 重新计算避障规划数据
   * - 更新目标对象列表
   * - 确定变道方向(左或右)
   * - 补偿丢失的目标对象
   */
  void updateSpecialData() override;

private:
  std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters_;  ///< 避障参数

  /**
   * @brief 计算避障规划数据
   * 
   * @param debug 调试数据输出
   * @return AvoidancePlanningData 避障规划数据
   */
  AvoidancePlanningData calcAvoidancePlanningData(AvoidanceDebugData & debug) const;
  
  AvoidancePlanningData avoidance_data_;  ///< 当前的避障规划数据
  mutable AvoidanceDebugData avoidance_debug_data_;  ///< 调试数据

  ObjectDataArray registered_objects_;  ///< 已注册的对象列表
  mutable ObjectDataArray stopped_objects_;  ///< 停止的对象列表
  std::shared_ptr<AvoidanceHelper> avoidance_helper_;  ///< 避障辅助工具

  /**
   * @brief 为预测对象创建对象数据
   * 
   * @param data 避障规划数据
   * @param object 预测对象
   * @return std::optional<ObjectData> 如果对象有效则返回对象数据,否则返回nullopt
   */
  std::optional<ObjectData> createObjectData(
    const AvoidancePlanningData & data, const PredictedObject & object) const;

  /**
   * @brief 填充避障目标对象
   * 
   * 从动态对象中筛选出需要避让的目标对象
   * 
   * @param data 避障规划数据(输出)
   * @param debug 调试数据(输出)
   */
  void fillAvoidanceTargetObjects(AvoidancePlanningData & data, AvoidanceDebugData & debug) const;

  /**
   * @brief 计算最小避障长度
   * 
   * @param nearest_object 最近的障碍物对象
   * @return double 最小避障长度(米)
   */
  double calcMinAvoidanceLength(const ObjectData & nearest_object) const;
  
  /**
   * @brief 计算最小距离缓冲
   * 
   * @return double 最小距离缓冲(米)
   */
  double calc_minimum_dist_buffer() const;
  
  /**
   * @brief 计算横向偏移量
   * 
   * @return double 横向偏移量(米)
   */
  double calcLateralOffset() const;
};
}  // namespace autoware::behavior_path_planner

#endif  // SCENE_HPP_
