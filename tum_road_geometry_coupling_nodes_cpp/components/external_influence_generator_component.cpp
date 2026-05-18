// Copyright 2025 Simon Sagmeister
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "tum_road_geometry_coupling_nodes_cpp/external_influence_generator_node.hpp"
#include "track_data_loader.hpp"
// Component-shaped wrapper: rclcpp_components requires a constructor that
// takes only a NodeOptions. The track is loaded here via track_handler_cpp
// and the resulting TrackData is forwarded to the actual node constructor.
struct ExternalInfluenceGeneratorComponent : public tam::road_geometry_coupling::ExternalInfluenceGeneratorNode
{
private:
  using LoadedTracks = tam::road_geometry_coupling::component_internal::LoadedTracks;
  explicit ExternalInfluenceGeneratorComponent(
    const rclcpp::NodeOptions & options, LoadedTracks tracks)
  : tam::road_geometry_coupling::ExternalInfluenceGeneratorNode(
      options, std::move(tracks.raceline), std::move(tracks.pitlane))
  {
  }

public:
  explicit ExternalInfluenceGeneratorComponent(const rclcpp::NodeOptions & options)
  : ExternalInfluenceGeneratorComponent(
      options, tam::road_geometry_coupling::component_internal::load_tracks_from_pkg_config())
  {
  }
};
RCLCPP_COMPONENTS_REGISTER_NODE(ExternalInfluenceGeneratorComponent)
