#!/bin/bash
# Planning 模块速度调试日志监控脚本

echo "=========================================="
echo "  Planning 模块速度调试日志监控"
echo "=========================================="
echo ""
echo "功能: 实时显示所有包含 [VEL_DEBUG] 标记的日志"
echo "来源: Behavior Path Planner, Goal Planner, Avoidance, Obstacle Cruise"
echo ""
echo "按 Ctrl+C 退出"
echo "=========================================="
echo ""

# 选择监控模式
echo "选择监控模式:"
echo "1) 所有速度调试日志 (推荐)"
echo "2) 仅 BPP 输出"
echo "3) 仅 Goal Planner"
echo "4) 仅 Avoidance"
echo "5) 仅 Obstacle Cruise"
echo "6) 仅显示速度修改 (MODIFY/RESULT)"
echo ""
read -p "请输入选项 (1-6) [默认=1]: " choice
choice=${choice:-1}

case $choice in
  1)
    echo "监控所有速度调试日志..."
    ros2 topic echo /rosout | grep --line-buffered --color=always "\[VEL_DEBUG\]"
    ;;
  2)
    echo "监控 BPP 输出日志..."
    ros2 topic echo /rosout | grep --line-buffered --color=always "\[VEL_DEBUG\]\[BPP\]"
    ;;
  3)
    echo "监控 Goal Planner 日志..."
    ros2 topic echo /rosout | grep --line-buffered --color=always "\[VEL_DEBUG\]\[GoalPlanner\]"
    ;;
  4)
    echo "监控 Avoidance 日志..."
    ros2 topic echo /rosout | grep --line-buffered --color=always "\[VEL_DEBUG\]\[Avoidance\]"
    ;;
  5)
    echo "监控 Obstacle Cruise 日志..."
    ros2 topic echo /rosout | grep --line-buffered --color=always "\[VEL_DEBUG\]\[ObstacleCruise\]"
    ;;
  6)
    echo "监控速度修改日志..."
    ros2 topic echo /rosout | grep --line-buffered --color=always "\[VEL_DEBUG\].*\(MODIFY\|RESULT\|PUBLISH\)"
    ;;
  *)
    echo "无效选项，使用默认选项 1"
    ros2 topic echo /rosout | grep --line-buffered --color=always "\[VEL_DEBUG\]"
    ;;
esac

