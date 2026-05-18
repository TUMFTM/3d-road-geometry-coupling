// Copyright 2025 Simon Sagmeister
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "tum_road_geometry_coupling_nodes_cpp/road_geometry_coupler_node.hpp"
#include "track_data_loader.hpp"
// Component-shaped wrapper: rclcpp_components requires a constructor that
// takes only a NodeOptions. The track is loaded here via track_handler_cpp
// and the resulting TrackData is forwarded to the actual node constructor.
struct RoadGeometryCouplerComponent : public tam::road_geometry_coupling::RoadGeometryCouplerNode
{
private:
  using LoadedTracks = tam::road_geometry_coupling::component_internal::LoadedTracks;
  explicit RoadGeometryCouplerComponent(const rclcpp::NodeOptions & options, LoadedTracks tracks)
  : tam::road_geometry_coupling::RoadGeometryCouplerNode(
      options, std::move(tracks.raceline), std::move(tracks.pitlane))
  {
  }

public:
  explicit RoadGeometryCouplerComponent(const rclcpp::NodeOptions & options)
  : RoadGeometryCouplerComponent(
      options, tam::road_geometry_coupling::component_internal::load_tracks_from_pkg_config())
  {
  }
};
RCLCPP_COMPONENTS_REGISTER_NODE(RoadGeometryCouplerComponent)
