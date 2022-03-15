cmake_minimum_required(VERSION 3.5)

project(rviz_vtr_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(ignition-math6 REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_default_plugins REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(rviz_vtr_plugins_source_files
  src/rviz_vtr_plugins/tools/publish_select/publish_selection_tool.cpp
)

add_library(rviz_vtr_plugins SHARED
  ${rviz_vtr_plugins_source_files}
)

target_include_directories(rviz_vtr_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(rviz_vtr_plugins PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

target_link_libraries(rviz_vtr_plugins PRIVATE
  ignition-math6
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_vtr_plugins PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(rviz_vtr_plugins
  PUBLIC
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  sensor_msgs
  std_msgs
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(rviz_vtr_plugins HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_ogre_vendor
  rviz_default_plugins
  sensor_msgs
  std_msgs
)

install(
  TARGETS rviz_vtr_plugins
  EXPORT rviz_vtr_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
