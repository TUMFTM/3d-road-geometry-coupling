// Copyright 2025 Simon Sagmeister
#pragma once
#include <vector>
namespace tam::road_geometry_coupling
{
/// Plain-data description of a reference line (raceline, centerline, pitlane, ...).
///
/// All vectors must have identical length. Angles follow the convention of
/// \c tam::common::Track (theta: heading, mu: slope, phi: banking). \c omega_*
/// are the Darboux rotation rates along the reference line.
/// \c track_width_left_m and \c track_width_right_m are signed lateral offsets
/// from the reference line to the left/right track bound following the same
/// sign convention as in tam::common::Track (i.e. the right bound width is
/// typically negative).
struct TrackData
{
  std::vector<double> s_m{};
  std::vector<double> ref_line_x_m{};
  std::vector<double> ref_line_y_m{};
  std::vector<double> ref_line_z_m{};
  std::vector<double> theta_rad{};
  std::vector<double> mu_rad{};
  std::vector<double> phi_rad{};
  std::vector<double> omega_x_radpm{};
  std::vector<double> omega_y_radpm{};
  std::vector<double> omega_z_radpm{};
  std::vector<double> track_width_left_m{};
  std::vector<double> track_width_right_m{};
};
}  // namespace tam::road_geometry_coupling
