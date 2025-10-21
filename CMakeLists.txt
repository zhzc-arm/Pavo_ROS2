cmake_minimum_required(VERSION 3.5)
project(pavo_ros)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# SDK 路径
set(PAVO_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk")

# 查找 SDK 源文件
file(GLOB PAVO_SDK_SRC
  "${PAVO_SDK_PATH}/src/*.cpp"
  "${PAVO_SDK_PATH}/src/impl/unix/*.cpp"
)

message("PAVO_SDK_SRC: ${PAVO_SDK_SRC}")

# 查找 ROS2 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# 如果使用 dynamic_reconfigure，需改为 ROS2 的参数机制或使用 rclcpp::Parameter
# ROS2 不支持 message_generation 和 dynamic_reconfigure，需重构相关代码

# 包含头文件路径
include_directories(
  include
  ${PAVO_SDK_PATH}/include
)

# 添加可执行文件
add_executable(pavo_scan_node src/pavo_scan_node.cpp ${PAVO_SDK_SRC})
ament_target_dependencies(pavo_scan_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  std_msgs
  Boost
)

add_executable(pavo_pcd_node src/pavo_pcd_node.cpp ${PAVO_SDK_SRC})
ament_target_dependencies(pavo_pcd_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  std_msgs
  Boost
)

# 安装规则
install(TARGETS
  pavo_scan_node
  pavo_pcd_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz sdk
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# 导出包
ament_package()

