cmake_minimum_required(VERSION 3.16)
project(vtr_test_lidar)

## Common setup for vtr packages
include("$ENV{VTRSRC}/main/src/vtr_common/vtr_include.cmake")


## Find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)

find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(vtr_common REQUIRED)
find_package(vtr_logging REQUIRED)
find_package(vtr_lidar REQUIRED)
find_package(vtr_path_planning REQUIRED)
find_package(vtr_navigation REQUIRED)

## C++ Libraries

## Executables
include_directories(
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}_preprocessing src/preprocessing.cpp)
ament_target_dependencies(${PROJECT_NAME}_preprocessing rclcpp rosbag2 rosbag2_cpp vtr_lidar)

add_executable(${PROJECT_NAME}_odometry src/odometry.cpp)
ament_target_dependencies(${PROJECT_NAME}_odometry rclcpp rosbag2 rosbag2_cpp vtr_lidar)

add_executable(${PROJECT_NAME}_localization src/localization.cpp)
ament_target_dependencies(${PROJECT_NAME}_localization rclcpp rosbag2 rosbag2_cpp vtr_lidar)

add_executable(${PROJECT_NAME}_TandR src/full_pipeline_change.cpp)
ament_target_dependencies(${PROJECT_NAME}_TandR rclcpp rosbag2 rosbag2_cpp vtr_lidar)

# add_executable(${PROJECT_NAME}_lasersam src/lasersam_experiment.cpp)
# ament_target_dependencies(${PROJECT_NAME}_lasersam rclcpp rosbag2 rosbag2_cpp vtr_lidar)


add_executable(process_sequence src/full_pipeline_sequence.cpp)
ament_target_dependencies(process_sequence rclcpp rosbag2 rosbag2_cpp vtr_lidar)

add_executable(${PROJECT_NAME}_graph_info src/graph_learning.cpp)
ament_target_dependencies(${PROJECT_NAME}_graph_info rclcpp rosbag2 rosbag2_cpp vtr_tactic vtr_lidar)


add_executable(trim_lelr_graph src/trim_pose_graph.cpp)
ament_target_dependencies(trim_lelr_graph rclcpp rosbag2 rosbag2_cpp vtr_tactic)

add_executable(${PROJECT_NAME}_intra_exp_merging src/intra_exp_merging.cpp)
ament_target_dependencies(${PROJECT_NAME}_intra_exp_merging rclcpp rosbag2 rosbag2_cpp vtr_lidar)

# add_executable(${PROJECT_NAME}_dynamic_detection src/dynamic_detection.cpp)
# ament_target_dependencies(${PROJECT_NAME}_dynamic_detection rclcpp rosbag2 rosbag2_cpp vtr_lidar)

add_executable(${PROJECT_NAME}_inter_exp_merging src/inter_exp_merging.cpp)
ament_target_dependencies(${PROJECT_NAME}_inter_exp_merging rclcpp rosbag2 rosbag2_cpp vtr_lidar)

# add_executable(${PROJECT_NAME}_change_detection src/change_detection.cpp)
# ament_target_dependencies(${PROJECT_NAME}_change_detection rclcpp rosbag2 rosbag2_cpp vtr_lidar)

# add_executable(${PROJECT_NAME}_change_detection_fake src/change_detection_fake.cpp)
# ament_target_dependencies(${PROJECT_NAME}_change_detection_fake rclcpp rosbag2 rosbag2_cpp sensor_msgs std_msgs vtr_lidar)

# add_executable(${PROJECT_NAME}_change_detection_fake_one_frame src/change_detection_fake_one_frame.cpp)
# ament_target_dependencies(${PROJECT_NAME}_change_detection_fake_one_frame rclcpp rosbag2 rosbag2_cpp sensor_msgs std_msgs vtr_lidar)

# add_executable(${PROJECT_NAME}_ground_extraction src/ground_extraction.cpp)
# ament_target_dependencies(${PROJECT_NAME}_ground_extraction rclcpp rosbag2 rosbag2_cpp vtr_lidar)

# add_executable(${PROJECT_NAME}_terrain_assessment src/terrain_assessment.cpp)
# ament_target_dependencies(${PROJECT_NAME}_terrain_assessment rclcpp rosbag2 rosbag2_cpp vtr_lidar)

add_executable(${PROJECT_NAME}_create_costmap src/planning_experiment.cpp)
ament_target_dependencies(${PROJECT_NAME}_create_costmap rclcpp rosbag2 rosbag2_cpp vtr_lidar)

add_executable(${PROJECT_NAME}_map_annotation src/map_annotation.cpp)
ament_target_dependencies(${PROJECT_NAME}_map_annotation rclcpp rosbag2 rosbag2_cpp sensor_msgs std_msgs vtr_lidar)

add_executable(${PROJECT_NAME}_plot_map_maintenance src/plot_map_maintenance.cpp)
ament_target_dependencies(${PROJECT_NAME}_plot_map_maintenance rclcpp rosbag2 rosbag2_cpp vtr_lidar)

add_executable(${PROJECT_NAME}_plot_memap_maintenance src/plot_memap_maintenance.cpp)
ament_target_dependencies(${PROJECT_NAME}_plot_memap_maintenance rclcpp rosbag2 rosbag2_cpp vtr_lidar)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}_preprocessing
    ${PROJECT_NAME}_odometry
    ${PROJECT_NAME}_localization
    ${PROJECT_NAME}_TandR
    ${PROJECT_NAME}_graph_info
    ${PROJECT_NAME}_intra_exp_merging
    # ${PROJECT_NAME}_dynamic_detection
    ${PROJECT_NAME}_inter_exp_merging
    # ${PROJECT_NAME}_change_detection
    # ${PROJECT_NAME}_change_detection_fake
    # ${PROJECT_NAME}_change_detection_fake_one_frame
    # ${PROJECT_NAME}_lasersam
    # ${PROJECT_NAME}_ground_extraction
    # ${PROJECT_NAME}_terrain_assessment
    ${PROJECT_NAME}_create_costmap
    process_sequence
    ${PROJECT_NAME}_map_annotation
    ${PROJECT_NAME}_plot_map_maintenance
    ${PROJECT_NAME}_plot_memap_maintenance
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # Lint based on linter test_depend in package.xml
endif()

ament_package()
