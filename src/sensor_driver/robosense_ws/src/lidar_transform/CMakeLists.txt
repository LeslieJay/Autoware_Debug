cmake_minimum_required(VERSION 3.8)
project(lidar_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(autoware_point_types REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lidar_transform.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${autoware_point_types_INCLUDE_DIRS}  # 👈 显式加入
)

ament_target_dependencies(lidar_transform
    SYSTEM
    rclcpp 
    sensor_msgs
    PCL  
    tf2_geometry_msgs
    pcl_ros
    autoware_point_types
    )

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lidar_transform::LidarTransformNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install(TARGETS lidar_transform_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)


