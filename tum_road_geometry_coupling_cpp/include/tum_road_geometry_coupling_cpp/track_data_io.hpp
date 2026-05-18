// Copyright 2025 Simon Sagmeister
#pragma once
#include <string>

#include "tum_road_geometry_coupling_cpp/track_data.hpp"
namespace tam::road_geometry_coupling
{
/// Load a `TrackData` instance from a CSV file.
///
/// The CSV must have a header row whose column names exactly match the field
/// names of `TrackData` (`s_m`, `ref_line_x_m`, `ref_line_y_m`,
/// `ref_line_z_m`, `theta_rad`, `mu_rad`, `phi_rad`, `omega_x_radpm`,
/// `omega_y_radpm`, `omega_z_radpm`, `track_width_left_m`,
/// `track_width_right_m`). Column order is irrelevant; extra columns are
/// ignored. The example tracks shipped under `example_tracks/` follow this
/// format.
TrackData load_track_data_from_csv(std::string const & path);
}  // namespace tam::road_geometry_coupling
