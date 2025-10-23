#!/bin/bash
# 速度传递链对比脚本 - 对比各模块的速度输入输出

echo "=========================================="
echo "  速度传递链实时对比"
echo "=========================================="
echo ""
echo "对比以下模块的速度:"
echo "  1. BPP 输出"
echo "  2. Obstacle Cruise 输入输出"
echo "  3. 最终控制命令"
echo ""
echo "按 Ctrl+C 退出"
echo "=========================================="
echo ""

# 创建临时文件存储数据
TMP_DIR="/tmp/vel_debug_$$"
mkdir -p "$TMP_DIR"

# 清理函数
cleanup() {
  echo ""
  echo "清理临时文件..."
  rm -rf "$TMP_DIR"
  exit 0
}

trap cleanup INT TERM

# 后台监控各个话题
echo "启动监控..."

# 监控 BPP 输出
(
  while true; do
    vel=$(ros2 topic echo --once /planning/scenario_planning/lane_driving/behavior_planning/path 2>/dev/null | grep -m1 "longitudinal_velocity_mps:" | awk '{print $2}')
    if [ -n "$vel" ]; then
      echo "$vel" > "$TMP_DIR/bpp_output_vel"
    fi
    sleep 0.5
  done
) &
BPP_PID=$!

# 监控 Obstacle Cruise 输出
(
  while true; do
    vel=$(ros2 topic echo --once /planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner/trajectory 2>/dev/null | grep -m1 "longitudinal_velocity_mps:" | awk '{print $2}')
    if [ -n "$vel" ]; then
      echo "$vel" > "$TMP_DIR/cruise_output_vel"
    fi
    sleep 0.5
  done
) &
CRUISE_PID=$!

# 监控控制命令
(
  while true; do
    vel=$(ros2 topic echo --once /control/command/control_cmd 2>/dev/null | grep -m1 "velocity:" | awk '{print $2}')
    if [ -n "$vel" ]; then
      echo "$vel" > "$TMP_DIR/control_cmd_vel"
    fi
    sleep 0.5
  done
) &
CTRL_PID=$!

# 监控车辆当前速度
(
  while true; do
    vel=$(ros2 topic echo --once /localization/kinematic_state 2>/dev/null | grep -A3 "linear:" | grep "x:" | awk '{print $2}')
    if [ -n "$vel" ]; then
      echo "$vel" > "$TMP_DIR/current_vel"
    fi
    sleep 0.5
  done
) &
ODOM_PID=$!

echo "监控已启动，开始显示速度链..."
echo ""

# 主显示循环
while true; do
  # 读取所有速度值
  bpp_vel=$(cat "$TMP_DIR/bpp_output_vel" 2>/dev/null || echo "N/A")
  cruise_vel=$(cat "$TMP_DIR/cruise_output_vel" 2>/dev/null || echo "N/A")
  ctrl_vel=$(cat "$TMP_DIR/control_cmd_vel" 2>/dev/null || echo "N/A")
  current_vel=$(cat "$TMP_DIR/current_vel" 2>/dev/null || echo "N/A")
  
  # 清屏并显示
  clear
  echo "=========================================="
  echo "  速度传递链实时监控"
  echo "=========================================="
  echo ""
  echo "时间: $(date '+%H:%M:%S')"
  echo ""
  echo "速度传递流程:"
  echo ""
  printf "  当前车速:           %8s m/s\n" "$current_vel"
  echo "         ↓"
  printf "  BPP 输出:           %8s m/s\n" "$bpp_vel"
  echo "         ↓"
  printf "  Obstacle Cruise:    %8s m/s\n" "$cruise_vel"
  echo "         ↓"
  printf "  控制命令:           %8s m/s\n" "$ctrl_vel"
  echo ""
  echo "=========================================="
  
  # 计算速度差异
  if [[ "$bpp_vel" != "N/A" && "$cruise_vel" != "N/A" ]]; then
    diff_bpp_cruise=$(echo "$cruise_vel - $bpp_vel" | bc 2>/dev/null || echo "N/A")
    echo "BPP → Cruise 差异:  $diff_bpp_cruise m/s"
  fi
  
  if [[ "$cruise_vel" != "N/A" && "$ctrl_vel" != "N/A" ]]; then
    diff_cruise_ctrl=$(echo "$ctrl_vel - $cruise_vel" | bc 2>/dev/null || echo "N/A")
    echo "Cruise → Control 差异: $diff_cruise_ctrl m/s"
  fi
  
  if [[ "$current_vel" != "N/A" && "$ctrl_vel" != "N/A" ]]; then
    diff_current_target=$(echo "$ctrl_vel - $current_vel" | bc 2>/dev/null || echo "N/A")
    echo "当前 → 目标 差异:   $diff_current_target m/s"
  fi
  
  echo ""
  echo "按 Ctrl+C 退出"
  
  sleep 1
done

# 清理（在 trap 中处理）

