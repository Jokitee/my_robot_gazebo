cmake_minimum_required(VERSION 3.8)
project(my_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# --- 编译激光雷达硬件驱动节点 (内置于 my_robot_gazebo) ---
add_executable(lidar_node src/lidar_node.cpp)
ament_target_dependencies(lidar_node rclcpp sensor_msgs visualization_msgs)
target_link_libraries(lidar_node ${OpenCV_LIBS})

install(TARGETS
  lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch models worlds rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/auto_explorer.py
    scripts/chassis_driver.py
    scripts/lidar_driver.py
    scripts/test_lidar.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()