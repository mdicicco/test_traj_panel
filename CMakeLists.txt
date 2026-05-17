cmake_minimum_required(VERSION 3.16)
project(test_traj_panel)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# RViz on many Ubuntu installs builds against Qt5 while CMake prefers Qt6. Mixing both breaks
# INTERFACE_QT_MAJOR_VERSION checks — disable Qt6 for this package so rviz_common uses Qt5.
set(CMAKE_DISABLE_FIND_PACKAGE_Qt6 TRUE)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(tf2_ros REQUIRED)

set(TEST_TRAJ_PANEL_QT_WIDGETS_LIB Qt5::Widgets)

set(CMAKE_AUTOMOC ON)

# Header must be listed so AUTOMOC generates moc code for Q_OBJECT (otherwise vtable is undefined).
add_library(${PROJECT_NAME} SHARED
  include/test_traj_panel/test_traj_panel.hpp
  src/test_traj_panel.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  tf2_ros::tf2_ros
  ${TEST_TRAJ_PANEL_QT_WIDGETS_LIB}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
  rviz_common
  tf2_ros
)

ament_package()
