// Copyright 2025 Simon Sagmeister
//
// Internal helper shared by the rclcpp_components in this directory.
// Loads raceline and pitlane reference lines from the track_handler_cpp
// package configuration and converts them to the plain TrackData
// description consumed by the road-geometry-coupling library.
//
// Kept here (rather than in the library) so that consumers of the library
// itself stay free of the track_handler_cpp dependency.
#pragma once

#include <utility>
#include <vector>

#include "tum_road_geometry_coupling_cpp/track_data.hpp"
#include "track_handler_cpp/race_track_handler.hpp"
#include "track_handler_cpp/track.hpp"
namespace tam::road_geometry_coupling::component_internal
{
inline TrackData track_data_from_common_track(tam::common::Track const & track)
{
  auto to_vec = [](auto const & eigen_vec) {
    return std::vector<double>(eigen_vec.data(), eigen_vec.data() + eigen_vec.size());
  };
  TrackData data;
  data.s_m = to_vec(track.s_coord());
  data.ref_line_x_m = to_vec(track.ref_line_x());
  data.ref_line_y_m = to_vec(track.ref_line_y());
  data.ref_line_z_m = to_vec(track.ref_line_z());
  data.theta_rad = to_vec(track.theta());
  data.mu_rad = to_vec(track.mu());
  data.phi_rad = to_vec(track.phi());
  data.omega_x_radpm = to_vec(track.omega_x());
  data.omega_y_radpm = to_vec(track.omega_y());
  data.omega_z_radpm = to_vec(track.omega_z());
  data.track_width_left_m = to_vec(track.trackwidth_left());
  data.track_width_right_m = to_vec(track.trackwidth_right());
  return data;
}
struct LoadedTracks
{
  TrackData raceline;
  TrackData pitlane;
};
inline LoadedTracks load_tracks_from_pkg_config()
{
  auto track_handler = tam::common::RaceTrackHandler::from_pkg_config();
  return LoadedTracks{
    track_data_from_common_track(*track_handler->create_raceline_track()),
    track_data_from_common_track(*track_handler->create_pitlane())};
}
}  // namespace tam::road_geometry_coupling::component_internal
