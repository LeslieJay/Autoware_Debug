#!/bin/bash

# ========================================
# Autoware轨迹速度为0的调试脚本
# ========================================

echo "======================================"
echo "  Autoware 轨迹速度调试脚本"
echo "======================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. 检查关键话题是否有数据
echo -e "${YELLOW}[1] 检查关键话题...${NC}"
echo ""

echo "检查最终输出轨迹:"
timeout 2 ros2 topic hz /planning/scenario_planning/trajectory 2>&1 | head -n 5
if [ $? -eq 124 ]; then
    echo -e "${RED}✗ 话题 /planning/scenario_planning/trajectory 没有数据${NC}"
else
    echo -e "${GREEN}✓ 话题有数据${NC}"
    # 检查速度值
    echo "检查轨迹速度值 (前3个点):"
    timeout 2 ros2 topic echo /planning/scenario_planning/trajectory --once 2>/dev/null | grep -A 1 "longitudinal_velocity_mps" | head -n 6
fi
echo ""

echo "检查lane_driving轨迹:"
timeout 2 ros2 topic hz /planning/scenario_planning/lane_driving/trajectory 2>&1 | head -n 5
if [ $? -eq 124 ]; then
    echo -e "${RED}✗ 话题没有数据${NC}"
else
    echo -e "${GREEN}✓ 话题有数据${NC}"
fi
echo ""

# 2. 检查场景状态
echo -e "${YELLOW}[2] 检查当前场景...${NC}"
echo "当前场景:"
timeout 2 ros2 topic echo /planning/scenario_planning/scenario --once 2>/dev/null | grep "current_scenario"
echo ""

# 3. 检查路由状态
echo -e "${YELLOW}[3] 检查路由状态...${NC}"
timeout 2 ros2 topic hz /planning/mission_planning/route 2>&1 | head -n 5
if [ $? -eq 124 ]; then
    echo -e "${RED}✗ 没有route数据，请检查是否设置了goal点${NC}"
else
    echo -e "${GREEN}✓ route数据正常${NC}"
fi
echo ""

# 4. 检查定位数据
echo -e "${YELLOW}[4] 检查定位数据...${NC}"
timeout 2 ros2 topic hz /localization/kinematic_state 2>&1 | head -n 5
if [ $? -eq 124 ]; then
    echo -e "${RED}✗ 没有定位数据${NC}"
else
    echo -e "${GREEN}✓ 定位数据正常${NC}"
    echo "当前速度:"
    timeout 2 ros2 topic echo /localization/kinematic_state --once 2>/dev/null | grep -A 3 "linear:"
fi
echo ""

# 5. 检查behavior_velocity模块状态
echo -e "${YELLOW}[5] 检查behavior_velocity模块...${NC}"
echo "检查停止原因 (planning_factors):"
timeout 2 ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/stop_reason --once 2>/dev/null | head -n 20
echo ""

# 6. 检查障碍物信息
echo -e "${YELLOW}[6] 检查障碍物信息...${NC}"
timeout 2 ros2 topic hz /perception/object_recognition/objects 2>&1 | head -n 5
if [ $? -eq 124 ]; then
    echo -e "${YELLOW}! 没有障碍物数据 (可能未启用感知模块)${NC}"
else
    echo -e "${GREEN}✓ 障碍物数据正常${NC}"
fi
echo ""

# 7. 检查关键节点状态
echo -e "${YELLOW}[7] 检查关键节点...${NC}"
echo "Planning相关节点:"
ros2 node list 2>/dev/null | grep planning
echo ""

# 8. 检查关键参数
echo -e "${YELLOW}[8] 检查关键参数...${NC}"

# 检查velocity_smoother参数
if ros2 node list 2>/dev/null | grep -q "motion_velocity_smoother"; then
    echo "motion_velocity_smoother参数:"
    ros2 param get /planning/scenario_planning/motion_velocity_smoother engage_velocity 2>/dev/null
    ros2 param get /planning/scenario_planning/motion_velocity_smoother stopping_velocity 2>/dev/null
fi
echo ""

# 9. 检查操作模式
echo -e "${YELLOW}[9] 检查操作模式...${NC}"
timeout 2 ros2 topic echo /system/operation_mode/state --once 2>/dev/null | grep "mode:"
echo ""

# 10. 诊断总结
echo ""
echo "======================================"
echo -e "${YELLOW}诊断建议:${NC}"
echo "======================================"
echo ""
echo "如果速度全为0，常见原因:"
echo ""
echo "1. 【未设置goal点】"
echo "   解决: 在RViz2中使用2D Goal Pose工具设置目标点"
echo ""
echo "2. 【检测到停止线/交通灯】"
echo "   检查: ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/debug/stop_reasons"
echo ""
echo "3. 【障碍物阻挡】"
echo "   检查: ros2 topic echo /planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner/debug/marker"
echo ""
echo "4. 【操作模式不正确】"
echo "   需要切换到AUTONOMOUS模式"
echo "   设置: ros2 topic pub /system/operation_mode/change_operation_mode ..."
echo ""
echo "5. 【定位数据异常】"
echo "   检查TF树: ros2 run tf2_tools view_frames"
echo ""
echo "6. 【地图速度限制】"
echo "   检查lanelet地图是否设置了正确的speed_limit标签"
echo ""
echo "7. 【AGV特定问题】"
echo "   - 检查是否需要禁用某些behavior_velocity模块"
echo "   - 确认路径规划器是否适合AGV场景"
echo "   - 验证vehicle参数配置（最大速度等）"
echo ""

echo "======================================"
echo "调试完成"
echo "======================================"

