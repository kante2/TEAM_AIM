cmake_minimum_required(VERSION 3.5)
project(mission_1_1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)


# =============================================================================
# Mission 3: CAV control
# =============================================================================
add_executable(control_cav_mission_1_1 src/control_cav_mission_1_1.cpp)
ament_target_dependencies(control_cav_mission_1_1
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  ament_index_cpp
)
install(TARGETS control_cav_mission_1_1
  DESTINATION lib/${PROJECT_NAME})


# =============================================================================
# Decision: Lane overtake ROI visualizer
# =============================================================================
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/decision/roi_visualizer.cpp)
  add_executable(roi_visualizer src/decision/roi_visualizer.cpp)
  ament_target_dependencies(roi_visualizer
    rclcpp
    geometry_msgs
    std_msgs
    visualization_msgs
    tf2
    ament_index_cpp
  )
  install(TARGETS roi_visualizer
    DESTINATION lib/${PROJECT_NAME})
endif()

# =============================================================================
# Install CSV files to share directory
# =============================================================================
install(DIRECTORY src/global_path/
  DESTINATION share/${PROJECT_NAME}/global_path
  FILES_MATCHING PATTERN "*.csv")

# Install launch files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.py"
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
