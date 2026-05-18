# Copyright 2025 Simon Sagmeister
cmake_minimum_required(VERSION 3.12)

set(PROJECT_NAME tum_road_plane_follower_cpp)
set(NODE_EXECUTABLE_NAME road_plane_follower_node)
set(NODE_LIBRARY_PATH src/road_plane_follower.cpp)
set(NODE_EXECUTABLE_PATH executables/road_plane_follower_node.cpp)

set(NODE_DEPENDENCIES
  rclcpp
  nav_msgs
  geometry_msgs
  tum_helpers_cpp
  tum_ros_helpers_cpp
  tum_type_conversions_ros_cpp
  tum_types_cpp
  tum_road_geometry_coupling_cpp
)

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED ${NODE_LIBRARY_PATH})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_target_dependencies(${PROJECT_NAME} ${NODE_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${NODE_DEPENDENCIES})

add_executable(${NODE_EXECUTABLE_NAME} ${NODE_EXECUTABLE_PATH})
target_link_libraries(${NODE_EXECUTABLE_NAME} ${PROJECT_NAME})
install(
  TARGETS ${NODE_EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the bundled example tracks alongside the follower so the demo launch
# can locate them via ament_index, regardless of where the workspace lives.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../example_tracks
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
