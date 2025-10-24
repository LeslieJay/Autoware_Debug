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

#include "scene.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/utils.hpp"

#include <autoware/behavior_path_lane_change_module/utils/calculation.hpp>
#include <autoware/behavior_path_lane_change_module/utils/utils.hpp>
#include <autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/logging.hpp>

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/strategies/cartesian/centroid_bashein_detmer.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

namespace
{
/**
 * @brief 从位姿创建Point32消息
 * 
 * @param pose 输入位姿
 * @return geometry_msgs::msg::Point32 转换后的Point32消息
 */
geometry_msgs::msg::Point32 create_point32(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Point32 p;
  p.x = static_cast<float>(pose.position.x);
  p.y = static_cast<float>(pose.position.y);
  p.z = static_cast<float>(pose.position.z);
  return p;
};

/**
 * @brief 创建执行区域多边形
 * 
 * 根据车辆信息和当前位姿,创建一个矩形多边形表示车辆的执行区域。
 * 该区域用于判断是否有足够的空间执行变道避障。
 * 
 * @param vehicle_info 车辆信息
 * @param pose 当前车辆位姿
 * @param additional_lon_offset 额外的纵向偏移量(米)
 * @param additional_lat_offset 额外的横向偏移量(米)
 * @return geometry_msgs::msg::Polygon 执行区域多边形(由4个顶点组成的矩形)
 */
geometry_msgs::msg::Polygon create_execution_area(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & pose, double additional_lon_offset, double additional_lat_offset)
{
  const double & base_to_front = vehicle_info.max_longitudinal_offset_m;
  const double & width = vehicle_info.vehicle_width_m;
  const double & base_to_rear = vehicle_info.rear_overhang_m;

  // 计算前后左右的偏移量
  const double forward_lon_offset = base_to_front + additional_lon_offset;
  const double backward_lon_offset = -base_to_rear;
  const double lat_offset = width / 2.0 + additional_lat_offset;

  // 计算矩形的四个顶点
  const auto p1 = autoware_utils::calc_offset_pose(pose, forward_lon_offset, lat_offset, 0.0);
  const auto p2 = autoware_utils::calc_offset_pose(pose, forward_lon_offset, -lat_offset, 0.0);
  const auto p3 = autoware_utils::calc_offset_pose(pose, backward_lon_offset, -lat_offset, 0.0);
  const auto p4 = autoware_utils::calc_offset_pose(pose, backward_lon_offset, lat_offset, 0.0);
  geometry_msgs::msg::Polygon polygon;

  polygon.points.push_back(create_point32(p1));
  polygon.points.push_back(create_point32(p2));
  polygon.points.push_back(create_point32(p3));
  polygon.points.push_back(create_point32(p4));

  return polygon;
}
}  // namespace

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::Direction;
using autoware::behavior_path_planner::LaneChangeModuleType;
using autoware::behavior_path_planner::ObjectInfo;
using autoware::behavior_path_planner::Point2d;

/**
 * @brief AvoidanceByLaneChange类的构造函数实现
 * 
 * 初始化通过变道避障的场景对象,包括避障参数和避障辅助工具。
 */
AvoidanceByLaneChange::AvoidanceByLaneChange(
  const std::shared_ptr<LaneChangeParameters> & parameters,
  std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters)
: NormalLaneChange(parameters, LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE, Direction::NONE),
  avoidance_parameters_(std::move(avoidance_parameters)),
  avoidance_helper_{std::make_shared<AvoidanceHelper>(avoidance_parameters_)}
{
}

/**
 * @brief 检查是否需要执行变道避障
 * 
 * 该函数检查以下条件:
 * 1. 是否有需要避让的目标对象
 * 2. 目标对象是否确实需要避让
 * 3. 车辆与障碍物之间的距离是否足够完成变道
 * 
 * @return true 需要执行变道避障
 * @return false 不需要执行
 */
bool AvoidanceByLaneChange::specialRequiredCheck() const
{
  const auto & data = avoidance_data_;

  // 检查是否有目标对象
  if (data.target_objects.empty()) {
    RCLCPP_DEBUG(logger_, "no empty objects");
    return false;
  }

  const auto & object_parameters = avoidance_parameters_->object_parameters;

  const auto count_target_object = [&](const auto sum, const auto & p) {
    const auto & objects = avoidance_data_.target_objects;

    const auto is_avoidance_target = [&p](const auto & object) {
      const auto target_class = utils::getHighestProbLabel(object.object.classification) == p.first;
      return target_class && object.avoid_required;
    };

    return sum + std::count_if(objects.begin(), objects.end(), is_avoidance_target);
  };
  const auto num_of_avoidance_targets =
    std::accumulate(object_parameters.begin(), object_parameters.end(), 0UL, count_target_object);

  if (num_of_avoidance_targets < 1) {
    RCLCPP_DEBUG(logger_, "no avoidance target");
    return false;
  }

  const auto & nearest_object = data.target_objects.front();
  const auto minimum_avoid_length = calcMinAvoidanceLength(nearest_object);
  const auto minimum_lane_change_length = calc_minimum_dist_buffer();

  lane_change_debug_.execution_area = create_execution_area(
    getCommonParam().vehicle_info, getEgoPose(),
    std::max(minimum_lane_change_length, minimum_avoid_length), calcLateralOffset());

  RCLCPP_DEBUG(
    logger_, "Conditions ? %f, %f, %f", nearest_object.longitudinal, minimum_lane_change_length,
    minimum_avoid_length);
  return nearest_object.longitudinal > std::max(minimum_lane_change_length, minimum_avoid_length);
}

/**
 * @brief 检查模块是否应该过期
 * 
 * 如果不再满足执行变道避障的条件,则模块应该过期。
 * 
 * @return true 模块应该过期
 * @return false 模块仍然有效
 */
bool AvoidanceByLaneChange::specialExpiredCheck() const
{
  return !specialRequiredCheck();
}

/**
 * @brief 更新避障相关的特殊数据
 * 
 * 重新计算避障规划数据,更新目标对象列表,确定变道方向,
 * 并补偿因传感器遮挡等原因暂时丢失的目标对象。
 */
void AvoidanceByLaneChange::updateSpecialData()
{
  const auto p = std::dynamic_pointer_cast<AvoidanceParameters>(avoidance_parameters_);

  avoidance_debug_data_ = DebugData();
  avoidance_data_ = calcAvoidancePlanningData(avoidance_debug_data_);

  if (avoidance_data_.target_objects.empty()) {
    direction_ = Direction::NONE;
  } else {
    direction_ = utils::static_obstacle_avoidance::isOnRight(avoidance_data_.target_objects.front())
                   ? Direction::LEFT
                   : Direction::RIGHT;
  }

  utils::static_obstacle_avoidance::compensateLostTargetObjects(
    registered_objects_, avoidance_data_, clock_.now(), planner_data_, p);

  std::sort(
    avoidance_data_.target_objects.begin(), avoidance_data_.target_objects.end(),
    [](auto a, auto b) { return a.longitudinal < b.longitudinal; });
}

/**
 * @brief 计算避障规划数据
 * 
 * 生成用于避障规划的所有必要数据,包括参考路径、当前车道信息和目标对象列表。
 * 
 * @param debug 调试数据输出
 * @return AvoidancePlanningData 避障规划数据
 */
AvoidancePlanningData AvoidanceByLaneChange::calcAvoidancePlanningData(
  AvoidanceDebugData & debug) const
{
  AvoidancePlanningData data;

  // 设置参考位姿
  data.reference_pose = getEgoPose();

  // 获取粗略的参考路径
  data.reference_path_rough = prev_module_output_.path;

  // 使用样条曲线重采样路径以获得更平滑的路径
  const auto resample_interval = avoidance_parameters_->resample_interval_for_planning;
  data.reference_path = utils::resamplePathWithSpline(data.reference_path_rough, resample_interval);

  // 获取当前车道
  data.current_lanelets = get_current_lanes();

  // 填充需要避让的目标对象
  fillAvoidanceTargetObjects(data, debug);

  return data;
}

/**
 * @brief 填充避障目标对象
 * 
 * 从所有检测到的动态对象中筛选出需要避让的目标对象。
 * 将对象分为目标车道内的对象和目标车道外的对象。
 * 
 * @param data 避障规划数据(输出)
 * @param debug 调试数据(未使用)
 */
void AvoidanceByLaneChange::fillAvoidanceTargetObjects(
  AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug) const
{
  const auto p = std::dynamic_pointer_cast<AvoidanceParameters>(avoidance_parameters_);

  const auto [object_within_target_lane, object_outside_target_lane] =
    utils::path_safety_checker::separateObjectsByLanelets(
      *planner_data_->dynamic_object, data.current_lanelets,
      [](const auto & obj, const auto & lane, const auto yaw_threshold) {
        return utils::path_safety_checker::isPolygonOverlapLanelet(obj, lane, yaw_threshold);
      });

  // Assume that the maximum allocation for data.other object is the sum of
  // objects_within_target_lane and object_outside_target_lane. The maximum allocation for
  // data.target_objects is equal to object_within_target_lane
  {
    const auto other_objects_size =
      object_within_target_lane.objects.size() + object_outside_target_lane.objects.size();
    data.other_objects.reserve(other_objects_size);
    data.target_objects.reserve(object_within_target_lane.objects.size());
  }

  {
    const auto & objects = object_outside_target_lane.objects;
    std::transform(
      objects.cbegin(), objects.cend(), std::back_inserter(data.other_objects),
      [](const auto & object) {
        ObjectData other_object;
        other_object.object = object;
        other_object.info = ObjectInfo::OUT_OF_TARGET_AREA;
        return other_object;
      });
  }

  ObjectDataArray target_lane_objects;
  target_lane_objects.reserve(object_within_target_lane.objects.size());
  for (const auto & obj : object_within_target_lane.objects) {
    const auto target_lane_object = createObjectData(data, obj);
    if (!target_lane_object) {
      continue;
    }

    target_lane_objects.push_back(*target_lane_object);
  }

  data.target_objects = target_lane_objects;
}

/**
 * @brief 为预测对象创建对象数据
 * 
 * 为检测到的对象创建详细的对象数据,包括:
 * - 相对于中心线的位置
 * - 包络多边形
 * - 移动时间
 * - 是否需要避让
 * 
 * @param data 避障规划数据
 * @param object 预测对象
 * @return std::optional<ObjectData> 如果对象有效则返回对象数据,否则返回nullopt
 */
std::optional<ObjectData> AvoidanceByLaneChange::createObjectData(
  const AvoidancePlanningData & data, const PredictedObject & object) const
{
  using autoware::motion_utils::findNearestIndex;
  using autoware_utils::calc_distance2d;
  using autoware_utils::calc_lateral_deviation;
  using boost::geometry::return_centroid;

  const auto p = std::dynamic_pointer_cast<AvoidanceParameters>(avoidance_parameters_);

  const auto & path_points = data.reference_path.points;
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto object_closest_index = findNearestIndex(path_points, object_pose.position);
  const auto object_closest_pose = path_points.at(object_closest_index).point.pose;
  const auto t = utils::getHighestProbLabel(object.classification);
  const auto & object_parameter = avoidance_parameters_->object_parameters.at(t);

  ObjectData object_data{};
  // Calc lateral deviation from path to target object.
  object_data.to_centerline =
    lanelet::utils::getArcCoordinates(data.current_lanelets, object_pose).distance;

  if (
    std::abs(object_data.to_centerline) <
    avoidance_parameters_->threshold_distance_object_is_on_center) {
    return std::nullopt;
  }

  object_data.object = object;

  const auto lower = p->lower_distance_for_polygon_expansion;
  const auto upper = p->upper_distance_for_polygon_expansion;
  const auto clamp =
    std::clamp(calc_distance2d(getEgoPose(), object_pose) - lower, 0.0, upper) / upper;
  object_data.distance_factor = object_parameter.max_expand_ratio * clamp + 1.0;

  // Calc envelop polygon.
  utils::static_obstacle_avoidance::fillObjectEnvelopePolygon(
    object_data, registered_objects_, object_closest_pose, p);

  // calc object centroid.
  object_data.centroid = return_centroid<Point2d>(object_data.envelope_poly);

  // Calc moving time.
  utils::static_obstacle_avoidance::fillObjectMovingTime(object_data, stopped_objects_, p);

  object_data.direction = calc_lateral_deviation(object_closest_pose, object_pose.position) > 0.0
                            ? Direction::LEFT
                            : Direction::RIGHT;

  // Find the footprint point closest to the path, set to object_data.overhang_distance.
  object_data.overhang_points = utils::static_obstacle_avoidance::calcEnvelopeOverhangDistance(
    object_data, data.reference_path);

  // Check whether the the ego should avoid the object.
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;
  utils::static_obstacle_avoidance::fillAvoidanceNecessity(
    object_data, registered_objects_, vehicle_width, p);

  utils::static_obstacle_avoidance::fillLongitudinalAndLengthByClosestEnvelopeFootprint(
    data.reference_path_rough, getEgoPosition(), object_data);
  return object_data;
}

/**
 * @brief 计算最小避障长度
 * 
 * 根据最近障碍物的类型和位置,计算完成避障所需的最小纵向距离。
 * 
 * @param nearest_object 最近的障碍物对象
 * @return double 最小避障长度(米)
 */
double AvoidanceByLaneChange::calcMinAvoidanceLength(const ObjectData & nearest_object) const
{
  const auto ego_width = getCommonParam().vehicle_width;
  const auto nearest_object_type = utils::getHighestProbLabel(nearest_object.object.classification);
  const auto nearest_object_parameter =
    avoidance_parameters_->object_parameters.at(nearest_object_type);
  const auto lateral_hard_margin = std::max(
    nearest_object_parameter.lateral_hard_margin,
    nearest_object_parameter.lateral_hard_margin_for_parked_vehicle);
  const auto avoid_margin = lateral_hard_margin * nearest_object.distance_factor +
                            nearest_object_parameter.lateral_soft_margin + 0.5 * ego_width;

  avoidance_helper_->setData(planner_data_);
  const auto shift_length = avoidance_helper_->getShiftLength(
    nearest_object, utils::static_obstacle_avoidance::isOnRight(nearest_object), avoid_margin);

  return avoidance_helper_->getMinAvoidanceDistance(shift_length);
}

/**
 * @brief 计算最小距离缓冲
 * 
 * 计算变道所需的最小距离缓冲区。
 * 
 * @return double 最小距离缓冲(米)
 */
double AvoidanceByLaneChange::calc_minimum_dist_buffer() const
{
  const auto [_, dist_buffer] = utils::lane_change::calculation::calc_lc_length_and_dist_buffer(
    common_data_ptr_, get_current_lanes());
  return dist_buffer.min;
}

/**
 * @brief 计算横向偏移量
 * 
 * 计算所有对象类型中所需的最大横向偏移量,
 * 包括包络缓冲裕度、硬裕度和软裕度。
 * 
 * @return double 横向偏移量(米)
 */
double AvoidanceByLaneChange::calcLateralOffset() const
{
  auto additional_lat_offset{0.0};
  for (const auto & [type, p] : avoidance_parameters_->object_parameters) {
    const auto lateral_hard_margin =
      std::max(p.lateral_hard_margin, p.lateral_hard_margin_for_parked_vehicle);
    const auto offset =
      2.0 * p.envelope_buffer_margin + lateral_hard_margin + p.lateral_soft_margin;
    additional_lat_offset = std::max(additional_lat_offset, offset);
  }
  return additional_lat_offset;
}
}  // namespace autoware::behavior_path_planner
