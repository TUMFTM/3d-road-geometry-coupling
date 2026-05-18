# Copyright 2025 Simon Sagmeister
cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================

set(PROJECT_NAME tum_road_geometry_coupling_nodes_cpp)
set(EXECUTABLE_NAMES
  road_geometry_coupler_node
  external_influence_generator_node
)
set(NODE_LIBRARY_PATHS
  src/road_geometry_coupler_node.cpp
  src/external_influence_generator_node.cpp
)
set(COMPONENT_PATHS
  components/road_geometry_coupler_component.cpp
  components/external_influence_generator_component.cpp
)
set(COMPONENT_NAMES
  RoadGeometryCouplerComponent
  ExternalInfluenceGeneratorComponent
)

set(NODE_DEPENDENCIES
  rclcpp
  message_filters
  std_msgs
  nav_msgs
  tum_msgs
  tum_types_cpp
  geometry_msgs
  tum_type_conversions_ros_cpp
  tum_road_geometry_coupling_cpp
  tsl_ros2_publisher_cpp
  param_management_cpp
  param_management_ros2_integration_cpp
  tum_ros_helpers_cpp
)
# track_handler_cpp is only consumed by the component wrapper (not by the
# node library itself), so users that want to drive the node from explicit
# TrackData inputs do not pull in track_handler_cpp.
set(COMPONENT_DEPENDENCIES
  track_handler_cpp
)
# ============================================
# END OF CONFIGURATION

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# section: BUILD LIBRARY TARGET TO IMPORT NODE CLASS WITHIN OTHER PKGS
# ====================================================================

add_library(${PROJECT_NAME} SHARED ${NODE_LIBRARY_PATHS})

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
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)

# section: BUILD COMPONENTS
# ====================================================================
add_library(${PROJECT_NAME}_components SHARED ${COMPONENT_PATHS})
# Find pkg the additionally required pkgs
find_package(rclcpp_components)
foreach(pkg_name ${COMPONENT_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()
# Link the additional dependencies
ament_target_dependencies(${PROJECT_NAME}_components rclcpp_components ${COMPONENT_DEPENDENCIES})
# Link against the Node lib
target_link_libraries(${PROJECT_NAME}_components ${PROJECT_NAME})

foreach(COMPONENT_NAME EXECUTABLE_NAME IN ZIP_LISTS COMPONENT_NAMES EXECUTABLE_NAMES)
  # Register component and executable for each node
  rclcpp_components_register_node(
    ${PROJECT_NAME}_components
      PLUGIN ${COMPONENT_NAME}
      EXECUTABLE ${EXECUTABLE_NAME}
  )
endforeach()

ament_export_targets(export_${PROJECT_NAME}_components)

install(TARGETS ${PROJECT_NAME}_components
        EXPORT export_${PROJECT_NAME}_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
# section: BUILD STANDALONE NODE EXECUTABLE FROM CSV (NO rclcpp_components)
# ====================================================================
# A thin executable that constructs ExternalInfluenceGeneratorNode directly
# from a TrackData CSV. Used by the bundled demo launch and by anyone who
# wants to run the node without a track_handler_cpp configuration.
add_executable(external_influence_generator_node_from_csv
  src/external_influence_generator_node_from_csv.cpp)
target_link_libraries(external_influence_generator_node_from_csv ${PROJECT_NAME})
ament_target_dependencies(external_influence_generator_node_from_csv ${NODE_DEPENDENCIES})
install(TARGETS external_influence_generator_node_from_csv
        DESTINATION lib/${PROJECT_NAME})
# section: GENERAL
# ====================================================================
ament_export_dependencies(rclcpp_components ${NODE_DEPENDENCIES} ${COMPONENT_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them
ament_package()
