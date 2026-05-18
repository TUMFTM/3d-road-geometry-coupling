// Copyright 2025 Simon Sagmeister
//
// Standalone executable that constructs `ExternalInfluenceGeneratorNode`
// directly from a TrackData CSV (no `track_handler_cpp` configuration
// needed). Bypasses the rclcpp_components wrapper and is the entry point
// used by the bundled demo launch file.
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "tum_road_geometry_coupling_cpp/track_data.hpp"
#include "tum_road_geometry_coupling_cpp/track_data_io.hpp"
#include "tum_road_geometry_coupling_nodes_cpp/external_influence_generator_node.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.arguments(std::vector<std::string>(argv, argv + argc));

  // Bootstrap node: only `Node` construction triggers rclcpp's argv / params-file
  // parsing, so we spin up a throwaway node first to read the `track_csv_path`
  // override. The actual node is constructed afterwards from the loaded
  // TrackData. The bootstrap takes the same name as the real node so that any
  // params-file entry keyed by node name is matched in both passes.
  auto bootstrap = std::make_shared<rclcpp::Node>("external_influence_generator", options);
  auto const track_csv_path =
    bootstrap->declare_parameter<std::string>("track_csv_path", std::string{});
  if (track_csv_path.empty()) {
    RCLCPP_FATAL(
      bootstrap->get_logger(),
      "external_influence_generator_node_from_csv: parameter 'track_csv_path' must be set, "
      "e.g. --ros-args -p track_csv_path:=/path/to/track.csv");
    rclcpp::shutdown();
    return 1;
  }
  bootstrap.reset();

  auto track_data = tam::road_geometry_coupling::load_track_data_from_csv(track_csv_path);
  auto node = std::make_shared<tam::road_geometry_coupling::ExternalInfluenceGeneratorNode>(
    options, track_data);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
