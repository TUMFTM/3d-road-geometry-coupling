// Copyright 2025 Simon Sagmeister

#include "tum_road_geometry_coupling_cpp/road_plane_transformer.hpp"
namespace tam::road_geometry_coupling
{
RoadPlaneGeometryTransformer::RoadPlaneSegment
RoadPlaneGeometryTransformer::calculate_road_plane_segment(
  CartesianPose2D const & road_plane_starting_pose, std::size_t const & root_idx,
  std::size_t const & segment_length) const
{
  // Init return value
  RoadPlaneSegment segment;
  segment.root_idx = root_idx;
  segment.num_points = segment_length;

  // Localize cosy segment starting pos on the track
  auto const & s_track = track_->s_coord();
  auto const & omega_z_track = track_->omega_z();

  // Build the reference curve in the vehicle frame
  Eigen::VectorXd rpl_x(segment_length), rpl_y(segment_length), rpl_z(segment_length),
    rpl_theta(segment_length), rpl_sigma(segment_length);
  Eigen::VectorXd rpl_tx(segment_length), rpl_ty(segment_length), rpl_tz(segment_length);

  // Init starting point in the vectors
  rpl_x(0) = road_plane_starting_pose.x_m;
  rpl_y(0) = road_plane_starting_pose.y_m;
  rpl_z(0) = 0.0;  // equal to 0
  rpl_theta(0) = road_plane_starting_pose.yaw_rad;
  rpl_tx(0) = std::cos(rpl_theta(0));
  rpl_ty(0) = std::sin(rpl_theta(0));
  rpl_tz(0) = 0.0;     // equal to 0 due to projection to 2D plane
  rpl_sigma(0) = 0.0;  // sigma is not used
  for (size_t i = 1; i < segment_length; i++) {
    size_t idx{(root_idx + i) % (s_track.size()-1)};
    size_t prev_idx = (idx == 0) ? 0 : idx - 1;  // Safe access of index
    double s_dist{0.0};
    if (idx == 0) {
      prev_idx = s_track.size() - 1;
      s_dist = s_track[prev_idx] - s_track[prev_idx - 1];
    } else {
      s_dist = s_track[idx] - s_track[prev_idx];
    }
    // Euler integration of the reference curve pose
    double delta_lat{0.0}, delta_lon{s_dist};
    double omega_z_median =
      (omega_z_track[prev_idx] + omega_z_track[idx]) * 0.5;  // Use the median curvature
    // Do a circular arc integration if the curvature is not zero
    // This avoid the problems of forward euler without
    // requiring microsteps
    if (std::abs(omega_z_median) > 1e-6) {
      delta_lat = (1 - std::cos(omega_z_median * s_dist)) / omega_z_median;
      delta_lon = std::sin(omega_z_median * s_dist) / omega_z_median;
    }
    rpl_x(i) = rpl_x(i - 1) + delta_lon * std::cos(rpl_theta(i - 1)) -
               delta_lat * std::sin(rpl_theta(i - 1));
    rpl_y(i) = rpl_y(i - 1) + delta_lon * std::sin(rpl_theta(i - 1)) +
               delta_lat * std::cos(rpl_theta(i - 1));
    rpl_theta(i) =
      tam::helpers::geometry::normalize_angle(rpl_theta(i - 1) + s_dist * omega_z_median);
    rpl_z(i) = 0.0;  // equal to 0
    // compute tangent vector based on reference curve heading
    rpl_tx(i) = cos(rpl_theta(i));
    rpl_ty(i) = sin(rpl_theta(i));
    rpl_tz(i) = 0.0;  // equal to 0 due to projection to 2D plane
    rpl_sigma(i) = rpl_sigma(i - 1) + s_dist;
  }
  segment.ref_line = tam::helpers::cosy::CurvilinearCosy::create(rpl_x, rpl_y, rpl_z)
                       ->set_tangent(rpl_tx, rpl_ty, rpl_tz)
                       ->set_s(rpl_sigma)
                       ->build();
  segment.sigma_m = rpl_sigma;

  return segment;
}
void RoadPlaneGeometryTransformer::extend_road_plane_segment(
  CartesianPose2D const & current_pose_road_plane)
{
  // Match onto the road plane ref line to get n and the index
  auto matching_result = current_segment_.ref_line->convert_to_sn_and_get_idx_global_2d(
    current_pose_road_plane.x_m, current_pose_road_plane.y_m, current_pose_road_plane.yaw_rad);
  size_t local_idx = std::get<1>(matching_result);
  double sigma_start = current_segment_.sigma_m[local_idx];

  CartesianPose2D root_pose;
  auto start_pose = current_segment_.ref_line->convert_to_cartesian(sigma_start, 0, 0);
  root_pose.x_m = start_pose[0];
  root_pose.y_m = start_pose[1];
  root_pose.yaw_rad = start_pose[2];

  current_segment_ = calculate_road_plane_segment(
    root_pose, wrap_index(current_segment_.root_idx + local_idx, track_max_idx_));
}
void RoadPlaneGeometryTransformer::calculate_curvilinear_pose(
  CartesianPose2D const & road_plane_pose, CurvilinearPoseWithDiagnostics & pose_ref)
{
  // First transform the road plane pose to curvilinear pose with respect to the segment
  auto curvilinear_pose_segment =
    CurvilinearPose(current_segment_.ref_line->convert_to_sn_and_get_idx_global(
      road_plane_pose.x_m, road_plane_pose.y_m, 0.0, road_plane_pose.yaw_rad));

  // Check if we need to rebuild the segment
  if (curvilinear_pose_segment.idx > REBUILD_RATIO * DEFAULT_SEGMENT_LENGTH) {
    // Rebuild the segment
    extend_road_plane_segment(road_plane_pose);
    // Match again to the newly calculated segment.
    curvilinear_pose_segment =
      CurvilinearPose(current_segment_.ref_line->convert_to_sn_and_get_idx_global(
        road_plane_pose.x_m, road_plane_pose.y_m, 0.0, road_plane_pose.yaw_rad));
  }
  // Directly assign the values that align in the segment and the track frame
  pose_ref.pose.n_m = curvilinear_pose_segment.n_m;
  pose_ref.pose.chi_rad = curvilinear_pose_segment.chi_rad;

  // Calculate the track index the current positions belongs to
  pose_ref.pose.idx =
    wrap_index(curvilinear_pose_segment.idx + current_segment_.root_idx, track_max_idx_);
  // Calculate the s coordinate in the global track
  pose_ref.pose.s_m =
    tam::helpers::numerical::interp_from_idx(track_->s_coord(), pose_ref.pose.idx);

  // Fill diagnostics
  pose_ref.diagnostics.current_local_index = curvilinear_pose_segment.idx;
  pose_ref.diagnostics.current_sigma = curvilinear_pose_segment.s_m;
  pose_ref.diagnostics.current_n = curvilinear_pose_segment.n_m;
  pose_ref.diagnostics.segment_root_idx = current_segment_.root_idx;

  // Fill out the point on the ref line
  auto xy_yaw =
    current_segment_.ref_line->convert_to_cartesian(curvilinear_pose_segment.s_m, 0.0, 0.0);
  pose_ref.diagnostics.ref_line_pose.x_m = xy_yaw[0];
  pose_ref.diagnostics.ref_line_pose.y_m = xy_yaw[1];
  pose_ref.diagnostics.ref_line_pose.yaw_rad = xy_yaw[2];
}
RoadPlaneGeometryTransformer::RoadPlaneGeometryTransformer(
  Track const * track, CurvilinearPose const & curvilinear_pose_on_track,
  CartesianPose2D const & current_pose_road_plane)
: track_{track}, track_max_idx_{static_cast<std::size_t>(track_->s_coord().size())}
{
  // Calculate the starting point of the roadplane segment so that it gives
  // the same curvilinear pose as on the track
  std::size_t root_index =
    static_cast<std::size_t>(curvilinear_pose_on_track.idx) % track->s_coord().size();

  CartesianPose2D root_pose_road_plane;

  // Find the offset between the current s and the last point in the raceline file
  // (Start the segment directly on a point of the ref line to align the indices)
  double delta_s = curvilinear_pose_on_track.s_m - track->s_coord()[root_index];

  // Calculate the root pose of the road plane segment
  root_pose_road_plane.yaw_rad =
    current_pose_road_plane.yaw_rad - curvilinear_pose_on_track.chi_rad;
  root_pose_road_plane.x_m = current_pose_road_plane.x_m -
                             delta_s * std::cos(root_pose_road_plane.yaw_rad) +
                             curvilinear_pose_on_track.n_m * std::sin(root_pose_road_plane.yaw_rad);
  root_pose_road_plane.y_m = current_pose_road_plane.y_m -
                             delta_s * std::sin(root_pose_road_plane.yaw_rad) -
                             curvilinear_pose_on_track.n_m * std::cos(root_pose_road_plane.yaw_rad);

  // Now calculate the segment starting from the corresponding root pose
  current_segment_ = calculate_road_plane_segment(root_pose_road_plane, root_index);
}
RoadPlaneGeometryTransformer::CurvilinearPoseWithDiagnostics
RoadPlaneGeometryTransformer::transform_to_global_curvilinear_pose(
  CartesianPose2D const & road_plane_pose)
{
  CurvilinearPoseWithDiagnostics result;

  // Calculate the curvilinear pose in the segment
  calculate_curvilinear_pose(road_plane_pose, result);

  return result;
}
}  // namespace tam::road_geometry_coupling
