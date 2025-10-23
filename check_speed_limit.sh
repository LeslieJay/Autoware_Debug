#!/bin/bash

echo "======================================"
echo "  Autoware 速度限制实时监控"
echo "======================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

# 1. 检查当前速度限制
echo -e "${YELLOW}[1] 当前最大速度限制${NC}"
MAX_VEL=$(timeout 2 ros2 topic echo /planning/scenario_planning/current_max_velocity --once 2>/dev/null | grep "max_velocity" | awk '{print $2}')
SENDER=$(timeout 2 ros2 topic echo /planning/scenario_planning/current_max_velocity --once 2>/dev/null | grep "sender" | awk '{print $2}' | tr -d "'")

if [ -n "$MAX_VEL" ]; then
    MAX_VEL_KMH=$(echo "$MAX_VEL * 3.6" | bc)
    echo -e "  ${CYAN}最大速度限制: ${MAX_VEL} m/s (${MAX_VEL_KMH} km/h)${NC}"
    echo -e "  来源: ${SENDER}"
else
    echo -e "  ${RED}⚠ 无法获取速度限制信息${NC}"
fi
echo ""

# 2. 检查实际车速
echo -e "${YELLOW}[2] 当前车速${NC}"
CURRENT_SPEED=$(timeout 2 ros2 topic echo /vehicle/status/velocity_status --once 2>/dev/null | grep "longitudinal_velocity" | awk '{print $2}')

if [ -n "$CURRENT_SPEED" ]; then
    CURRENT_SPEED_KMH=$(echo "$CURRENT_SPEED * 3.6" | bc)
    echo -e "  ${CYAN}当前速度: ${CURRENT_SPEED} m/s (${CURRENT_SPEED_KMH} km/h)${NC}"
    
    # 计算速度与限速的比例
    if [ -n "$MAX_VEL" ] && [ "$(echo "$MAX_VEL > 0" | bc)" -eq 1 ]; then
        RATIO=$(echo "scale=2; $CURRENT_SPEED / $MAX_VEL * 100" | bc)
        if [ "$(echo "$RATIO >= 90" | bc)" -eq 1 ]; then
            echo -e "  ${RED}⚠ 速度已达限速的 ${RATIO}%${NC}"
        elif [ "$(echo "$RATIO >= 60" | bc)" -eq 1 ]; then
            echo -e "  ${YELLOW}注意: 速度已达限速的 ${RATIO}%${NC}"
        else
            echo -e "  ${GREEN}✓ 速度正常 (${RATIO}% 的限速)${NC}"
        fi
    fi
else
    echo -e "  ${RED}⚠ 无法获取车速信息${NC}"
fi
echo ""

# 3. 检查轨迹速度统计
echo -e "${YELLOW}[3] 当前轨迹速度分布${NC}"
TRAJ_DATA=$(timeout 3 ros2 topic echo /planning/scenario_planning/trajectory --once 2>/dev/null | \
  grep "longitudinal_velocity_mps" | \
  awk '{print $2}')

if [ -n "$TRAJ_DATA" ]; then
    echo "$TRAJ_DATA" | \
      sort -n | uniq -c | \
      awk '{printf "  %3d 个点: %.2f m/s (%.1f km/h)\n", $1, $2, $2*3.6}'
else
    echo -e "  ${RED}⚠ 无法获取轨迹信息${NC}"
fi
echo ""

# 4. 检查是否有停止点
echo -e "${YELLOW}[4] 停止点检查${NC}"
ZERO_COUNT=$(timeout 3 ros2 topic echo /planning/scenario_planning/trajectory --once 2>/dev/null | \
  grep "longitudinal_velocity_mps: 0.0" | wc -l)

if [ -n "$ZERO_COUNT" ] && [ "$ZERO_COUNT" -gt 0 ]; then
    echo -e "${CYAN}  ✓ 检测到 ${ZERO_COUNT} 个停止点${NC}"
else
    echo -e "${GREEN}  ✓ 无停止点，车辆保持运动${NC}"
fi
echo ""

# 5. 检查behavior planning路径速度（来自地图）
echo -e "${YELLOW}[5] Behavior Planning路径速度（来自Lanelet地图）${NC}"
PATH_SPEEDS=$(timeout 3 ros2 topic echo /planning/scenario_planning/lane_driving/behavior_planning/path --once 2>/dev/null | \
  grep "longitudinal_velocity_mps" | head -n 5)

if [ -n "$PATH_SPEEDS" ]; then
    echo "$PATH_SPEEDS" | awk '{printf "  点速度: %.2f m/s (%.1f km/h)\n", $2, $2*3.6}'
else
    echo -e "  ${RED}⚠ 无法获取behavior planning路径信息${NC}"
fi
echo ""

# 6. 检查外部速度限制
echo -e "${YELLOW}[6] 外部速度限制（如有）${NC}"
EXT_VEL=$(timeout 2 ros2 topic echo /planning/scenario_planning/max_velocity --once 2>/dev/null | grep "max_velocity" | awk '{print $2}')

if [ -n "$EXT_VEL" ]; then
    EXT_VEL_KMH=$(echo "$EXT_VEL * 3.6" | bc)
    echo -e "  ${CYAN}外部设定: ${EXT_VEL} m/s (${EXT_VEL_KMH} km/h)${NC}"
else
    echo -e "  ${GREEN}✓ 无外部速度限制${NC}"
fi
echo ""

echo "======================================"
echo "监控完成 $(date '+%Y-%m-%d %H:%M:%S')"
echo "======================================"

