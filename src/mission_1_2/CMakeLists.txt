cmake_minimum_required(VERSION 3.5)
project(mission_1_2)

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
# Mission 2: CAV control (원본 - 에러 있음, 스킵)
# =============================================================================
# add_executable(control_cav_mission_2 
#   src/control/control_cav_mission_2.cpp
#   src/control/pure_pursuit/pure_pursuit.cpp
#   src/control/path_manager/path_manager.cpp
#   src/control/roi_visualization/roi_visualization.cpp
# )

# =============================================================================
# Mission 2 executables: prefer specific versions if present, otherwise
# fall back to the single existing `src/control_cav_mission_2.cpp` source.
# =============================================================================
add_executable(control_cav_mission_1_2 src/control_cav_mission_1_2.cpp)
ament_target_dependencies(control_cav_mission_1_2
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  visualization_msgs
)
install(TARGETS control_cav_mission_1_2
  DESTINATION lib/${PROJECT_NAME})

add_executable(control_tower_mission_1_2 src/control_tower_mission_1_2.cpp)
ament_target_dependencies(control_tower_mission_1_2
  rclcpp
  geometry_msgs
  std_msgs
  tf2
  visualization_msgs
)
install(TARGETS control_tower_mission_1_2
  DESTINATION lib/${PROJECT_NAME})
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/control/control_cav_mission_2_ver2.cpp)
  add_executable(control_cav_mission_2_ver2 src/control/control_cav_mission_2_ver2.cpp)
  ament_target_dependencies(control_cav_mission_2_ver2
    rclcpp
    geometry_msgs
    std_msgs
    tf2
    ament_index_cpp
  )
  install(TARGETS control_cav_mission_2_ver2
    DESTINATION lib/${PROJECT_NAME})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/control/control_cav_mission_2_ver2_refactored.cpp)
  add_executable(control_cav_mission_2_ver2_refactored src/control/control_cav_mission_2_ver2_refactored.cpp)
  ament_target_dependencies(control_cav_mission_2_ver2_refactored
    rclcpp
    geometry_msgs
    std_msgs
    tf2
    ament_index_cpp
  )
  install(TARGETS control_cav_mission_2_ver2_refactored
    DESTINATION lib/${PROJECT_NAME})
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/control/control_cav_mission_2_ver4_refactored.cpp)
  add_executable(control_cav_mission_2_ver4_refactored src/control/control_cav_mission_2_ver4_refactored.cpp)
  ament_target_dependencies(control_cav_mission_2_ver4_refactored
    rclcpp
    geometry_msgs
    std_msgs
    tf2
    ament_index_cpp
  )
  install(TARGETS control_cav_mission_2_ver4_refactored
    DESTINATION lib/${PROJECT_NAME})
endif()

# Fallback: if no versioned control source exists, build the plain file
if(NOT TARGET control_cav_mission_2_ver2 AND NOT TARGET control_cav_mission_2_ver2_refactored AND NOT TARGET control_cav_mission_2_ver4_refactored)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/control_cav_mission_2.cpp)
    add_executable(control_cav_mission_2 src/control_cav_mission_2.cpp)
    ament_target_dependencies(control_cav_mission_2
      rclcpp
      geometry_msgs
      std_msgs
      tf2
      ament_index_cpp
    )
    install(TARGETS control_cav_mission_2
      DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

# =============================================================================
# Test: ROI Collision Detection (Standalone C++ Program)
# =============================================================================
# add_executable(test_collision_roi
#   src/control/check_roi/test_collision_main.cpp
#   src/control/check_roi/check_collision_roi.cpp
# )
# target_include_directories(test_collision_roi PRIVATE src/control)
# install(TARGETS test_collision_roi DESTINATION lib/${PROJECT_NAME})

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
