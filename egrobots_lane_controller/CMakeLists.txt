cmake_minimum_required(VERSION 3.8)
project(egrobots_lane_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(dependencies
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs
  nav2_core nav2_costmap_2d nav2_util pluginlib
  tf2 tf2_ros tf2_geometry_msgs)
foreach(pkg ${dependencies})
  find_package(${pkg} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/lane_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

pluginlib_export_plugin_description_file(nav2_core lane_controller_plugin.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
