#!/bin/bash
# 重新编译添加了速度调试日志的 Planning 模块

echo "=========================================="
echo "  重新编译速度调试模块"
echo "=========================================="
echo ""

# 检查是否在 Autoware 根目录
if [ ! -f "src/launcher/autoware_launch/autoware_launch/package.xml" ]; then
  echo "错误: 请在 Autoware 根目录运行此脚本"
  echo "当前目录: $(pwd)"
  exit 1
fi

echo "将编译以下包:"
echo "  1. autoware_behavior_path_planner"
echo "  2. autoware_behavior_path_goal_planner_module"
echo "  3. autoware_behavior_path_static_obstacle_avoidance_module"
echo "  4. autoware_obstacle_cruise_planner"
echo ""

# 选择编译类型
echo "选择编译类型:"
echo "1) RelWithDebInfo (推荐，包含调试信息，优化性能)"
echo "2) Debug (完整调试信息，性能较慢)"
echo "3) Release (最高性能，无调试信息)"
echo ""
read -p "请输入选项 (1-3) [默认=1]: " choice
choice=${choice:-1}

case $choice in
  1)
    BUILD_TYPE="RelWithDebInfo"
    ;;
  2)
    BUILD_TYPE="Debug"
    ;;
  3)
    BUILD_TYPE="Release"
    ;;
  *)
    echo "无效选项，使用默认 RelWithDebInfo"
    BUILD_TYPE="RelWithDebInfo"
    ;;
esac

echo ""
echo "编译类型: $BUILD_TYPE"
echo ""
echo "开始编译..."
echo "=========================================="
echo ""

# 执行编译
colcon build \
  --packages-select \
    autoware_behavior_path_planner \
    autoware_behavior_path_goal_planner_module \
    autoware_behavior_path_static_obstacle_avoidance_module \
    autoware_obstacle_cruise_planner \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  --symlink-install

# 检查编译结果
if [ $? -eq 0 ]; then
  echo ""
  echo "=========================================="
  echo "  ✅ 编译成功！"
  echo "=========================================="
  echo ""
  echo "下一步:"
  echo "  1. source install/setup.bash"
  echo "  2. 启动 Autoware"
  echo "  3. 运行监控脚本: ./monitor_velocity_debug.sh"
  echo "  或者运行对比脚本: ./compare_velocity_chain.sh"
  echo ""
else
  echo ""
  echo "=========================================="
  echo "  ❌ 编译失败"
  echo "=========================================="
  echo ""
  echo "请检查错误信息并修正"
  exit 1
fi

