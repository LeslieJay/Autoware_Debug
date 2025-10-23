#!/bin/bash

echo "======================================"
echo "  Behavior Path Planner 速度调试"
echo "======================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

# 1. 检查路径速度
echo -e "${YELLOW}[1] 当前路径速度分布${NC}"
PATH_SPEEDS=$(timeout 3 ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once 2>/dev/null | \
  grep "longitudinal_velocity_mps" | \
  awk '{print $2}')

if [ -n "$PATH_SPEEDS" ]; then
    ZERO_COUNT=$(echo "$PATH_SPEEDS" | grep -c "^0.0*$")
    TOTAL_COUNT=$(echo "$PATH_SPEEDS" | wc -l)
    
    if [ "$ZERO_COUNT" -eq "$TOTAL_COUNT" ]; then
        echo -e "  ${RED}⚠ 所有路径点速度为0！($TOTAL_COUNT 个点)${NC}"
    elif [ "$ZERO_COUNT" -gt 0 ]; then
        echo -e "  ${YELLOW}⚠ 部分路径点速度为0 ($ZERO_COUNT/$TOTAL_COUNT 个点)${NC}"
    else
        echo -e "  ${GREEN}✓ 路径速度正常${NC}"
    fi
    
    echo "$PATH_SPEEDS" | sort -n | uniq -c | \
      awk '{printf "  %3d 个点: %.2f m/s (%.1f km/h)\n", $1, $2, $2*3.6}'
else
    echo -e "  ${RED}⚠ 无法获取路径信息${NC}"
fi
echo ""

# 2. 检查lane_ids
echo -e "${YELLOW}[2] 当前路径的lane IDs（前5个点）${NC}"
LANE_IDS=$(timeout 3 ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once 2>/dev/null | \
  grep "lane_ids" | head -n 5)

if [ -n "$LANE_IDS" ]; then
    echo "$LANE_IDS" | sed 's/^/  /'
    
    # 提取第一个lane_id供后续使用
    FIRST_LANE_ID=$(echo "$LANE_IDS" | head -n 1 | grep -oP '\d+' | head -n 1)
    if [ -n "$FIRST_LANE_ID" ]; then
        echo -e "  ${CYAN}第一个lane_id: $FIRST_LANE_ID${NC}"
    fi
else
    echo -e "  ${RED}⚠ 无法获取lane IDs${NC}"
fi
echo ""

# 3. 检查场景模块状态
echo -e "${YELLOW}[3] 激活的场景模块${NC}"
MODULE_STATUS=$(timeout 3 ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/scene_module_status --once 2>/dev/null | \
  grep -E "module_name|status" | head -n 20)

if [ -n "$MODULE_STATUS" ]; then
    # 提取模块名和状态
    echo "$MODULE_STATUS" | paste - - | \
      awk -F"'" '{printf "  %s: %s\n", $2, $4}' | head -n 10
else
    echo -e "  ${CYAN}无激活的场景模块${NC}"
fi
echo ""

# 4. 检查停止原因
echo -e "${YELLOW}[4] 停止原因${NC}"
STOP_REASONS=$(timeout 3 ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/stop_reasons --once 2>/dev/null | \
  grep -E "reason|stop_pose" | head -n 10)

if [ -n "$STOP_REASONS" ]; then
    echo "$STOP_REASONS" | sed 's/^/  /'
else
    echo -e "  ${GREEN}✓ 无停止原因${NC}"
fi
echo ""

# 5. 检查是否"out of route"
echo -e "${YELLOW}[5] 路线状态检查${NC}"
OUT_OF_ROUTE=$(timeout 2 ros2 topic echo /rosout --once 2>/dev/null | \
  grep -i "out of route" | tail -n 1)

if [ -n "$OUT_OF_ROUTE" ]; then
    echo -e "  ${RED}⚠ 检测到 'out of route' 警告${NC}"
    echo "$OUT_OF_ROUTE" | sed 's/^/  /'
else
    echo -e "  ${GREEN}✓ 车辆在路线上${NC}"
fi
echo ""

# 6. 检查Goal状态
echo -e "${YELLOW}[6] Goal距离检查${NC}"
ROUTE_INFO=$(timeout 2 ros2 topic echo /planning/mission_planning/route --once 2>/dev/null)
EGO_POSE=$(timeout 2 ros2 topic echo /localization/kinematic_state --once 2>/dev/null)

if [ -n "$ROUTE_INFO" ] && [ -n "$EGO_POSE" ]; then
    GOAL_X=$(echo "$ROUTE_INFO" | grep "goal_pose:" -A 10 | grep "x:" | awk '{print $2}' | head -n 1)
    GOAL_Y=$(echo "$ROUTE_INFO" | grep "goal_pose:" -A 10 | grep "y:" | awk '{print $2}' | head -n 1)
    EGO_X=$(echo "$EGO_POSE" | grep "position:" -A 3 | grep "x:" | awk '{print $2}' | head -n 1)
    EGO_Y=$(echo "$EGO_POSE" | grep "position:" -A 3 | grep "y:" | awk '{print $2}' | head -n 1)
    
    if [ -n "$GOAL_X" ] && [ -n "$EGO_X" ]; then
        DIST=$(echo "scale=2; sqrt(($GOAL_X - $EGO_X)^2 + ($GOAL_Y - $EGO_Y)^2)" | bc -l)
        echo -e "  ${CYAN}距离目标点: $DIST 米${NC}"
        
        if (( $(echo "$DIST < 5.0" | bc -l) )); then
            echo -e "  ${YELLOW}⚠ 接近目标点，可能触发Goal Planner（速度为0是正常的）${NC}"
        fi
    else
        echo -e "  ${CYAN}无法计算距离${NC}"
    fi
else
    echo -e "  ${CYAN}无法获取位置信息${NC}"
fi
echo ""

# 7. 检查地图速度限制（通过reference path）
echo -e "${YELLOW}[7] 参考路径速度（来自地图）${NC}"
REF_PATH_SPEED=$(timeout 3 ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/debug/reference_path --once 2>/dev/null | \
  grep "longitudinal_velocity_mps" | head -n 5)

if [ -n "$REF_PATH_SPEED" ]; then
    echo "$REF_PATH_SPEED" | awk '{printf "  点速度: %.2f m/s (%.1f km/h)\n", $2, $2*3.6}'
    
    # 检查参考路径速度是否为0
    REF_ZERO_COUNT=$(echo "$REF_PATH_SPEED" | awk '{print $2}' | grep -c "^0.0*$")
    if [ "$REF_ZERO_COUNT" -gt 0 ]; then
        echo -e "  ${RED}⚠ 参考路径速度为0，可能是地图speed_limit未设置！${NC}"
    fi
else
    echo -e "  ${YELLOW}⚠ 无法获取参考路径（可能话题不存在）${NC}"
fi
echo ""

# 8. 检查最近的错误日志
echo -e "${YELLOW}[8] 最近的相关日志${NC}"
LOGS=$(timeout 2 ros2 topic echo /rosout --once 2>/dev/null | \
  grep -i "behavior_path\|empty.*path\|speed.*limit" | tail -n 3)

if [ -n "$LOGS" ]; then
    echo "$LOGS" | sed 's/^/  /'
else
    echo -e "  ${GREEN}✓ 无相关错误日志${NC}"
fi
echo ""

# 9. 诊断建议
echo -e "${YELLOW}[9] 诊断建议${NC}"
if [ "$ZERO_COUNT" -eq "$TOTAL_COUNT" ] && [ "$TOTAL_COUNT" -gt 0 ]; then
    echo -e "  ${RED}❌ 所有路径点速度为0，请按以下优先级检查：${NC}"
    echo ""
    echo -e "  ${YELLOW}1. 【最重要】检查Lanelet地图的speed_limit标签${NC}"
    echo "     - 使用JOSM打开地图文件"
    echo "     - 检查当前lane_id的speed_limit标签"
    if [ -n "$FIRST_LANE_ID" ]; then
        echo "     - 当前lane_id: $FIRST_LANE_ID"
    fi
    echo ""
    echo -e "  ${YELLOW}2. 检查是否接近目标点${NC}"
    echo "     - 如果距离目标点<5米，速度为0是正常的"
    echo ""
    echo -e "  ${YELLOW}3. 检查traffic_rules是否正确加载${NC}"
    echo "     - 查看Autoware启动日志"
    echo "     - 确认地图正确加载"
    echo ""
    echo -e "  ${YELLOW}4. 检查场景模块状态${NC}"
    echo "     - 查看上方的激活模块和停止原因"
    echo ""
elif [ "$ZERO_COUNT" -gt 0 ]; then
    echo -e "  ${YELLOW}⚠ 部分路径点速度为0${NC}"
    echo "  这可能是正常的（如停止点、交通灯等）"
    echo "  检查停止原因了解详情"
else
    echo -e "  ${GREEN}✓ 路径速度正常${NC}"
fi
echo ""

echo "======================================"
echo "调试完成 $(date '+%Y-%m-%d %H:%M:%S')"
echo "======================================"
echo ""
echo "详细分析文档: behavior_path_planner速度为零的原因分析.md"

