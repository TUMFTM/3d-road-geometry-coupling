// Copyright 2025 Simon Sagmeister
#include <chrono>
#include <tum_road_geometry_coupling_cpp/road_geometry_coupler.hpp>
namespace tam::road_geometry_coupling
{
CartesianPose2D RoadGeometryCoupler::to_cartesian_pose_2d(
  tam::types::control::Odometry const & odometry)
{
  return CartesianPose2D{odometry.position_m.x, odometry.position_m.y, odometry.orientation_rad.z};
}
bool RoadGeometryCoupler::check_on_track_global(
  Track const & track, tam::types::control::Odometry const & odometry)
{
  return track.on_track_global(
    odometry.position_m.x, odometry.position_m.y, odometry.position_m.z,
    std::numeric_limits<double>::infinity(), 0.0);
}
void RoadGeometryCoupler::step()
{
  if (disabled_) {
    // If the module is disabled, do not do any calculations and just return the input values
    odometry_global_cartesian_ = input_odometry_road_plane_;
    acceleration_global_cartesian_ = input_acceleration_road_plane_;
    vehicle_load_ = VehicleLoad{{0, 0, 0}, {0, 0, 0}};
    return;
  }

  if (!road_plane_transformer_) {
    // Try to init everything
    bool init_success = init();
    if (!init_success) return;
  }

  auto step_start_time = std::chrono::steady_clock::now();

  // Transform the road_plane position to the global cartesian position
  auto curvilinear_pose = road_plane_transformer_->transform_to_global_curvilinear_pose(
    to_cartesian_pose_2d(input_odometry_road_plane_));

  global_curvilinear_track_position_ = curvilinear_pose.pose;
  road_plane_transformer_diagnostics_ = curvilinear_pose.diagnostics;

  // Transform the measurements from road plane to global cartesian
  measurement_transformer_->set_inputs(
    global_curvilinear_track_position_, input_odometry_road_plane_, input_acceleration_road_plane_);
  measurement_transformer_->transform_measurements();
  odometry_global_cartesian_ = measurement_transformer_->get_global_cartesian_odometry();
  acceleration_global_cartesian_ = measurement_transformer_->get_global_cartesian_acceleration();

  // Calculate the load on the vehicle
  vehicle_load_ = vehicle_load_calculator_.calculate_vehicle_load(
    input_acceleration_road_plane_, acceleration_global_cartesian_, odometry_global_cartesian_);

  auto step_duration = std::chrono::steady_clock::now() - step_start_time;
  value_logger_->log(
    "timing/transform_duration_ns",
    std::chrono::duration_cast<std::chrono::nanoseconds>(step_duration).count());

  auto match_track_windowed = [this](Track const & track, CurvilinearPose & current_pose) {
    return CurvilinearPose(track.get_cosy_handle()->convert_to_sn_and_get_idx_window(
      odometry_global_cartesian_.position_m.x, odometry_global_cartesian_.position_m.y,
      odometry_global_cartesian_.position_m.z, odometry_global_cartesian_.orientation_rad.z,
      current_pose.s_m - 50, current_pose.s_m + 50));
  };

  // Compute the current s coordinate on the raceline and pitlane
  current_raceline_pose_ = match_track_windowed(*raceline_, current_raceline_pose_);
  current_pitlane_pose_ = match_track_windowed(*pitlane_, current_pitlane_pose_);

  log_debug_information();
  // Check if a track switch has to be done for the future.
  bool track_switch_done = switch_active_track(odometry_global_cartesian_);
  if (track_switch_done) {
    // Recreate the measurement transformer
    measurement_transformer_ =
      std::make_unique<MeasurementTransformer>(active_track_, measurement_transformer_params_);

    // Recreate the Road Plane Transformer
    road_plane_transformer_ = std::make_unique<RoadPlaneGeometryTransformer>(
      active_track_, *active_track_matching_result_,
      to_cartesian_pose_2d(input_odometry_road_plane_));
  }
  additional_debug_info_.track_switch_done = track_switch_done;
}
/// Init by doing a global matching and calculating the first road plane segment
/// starting from the active race line.
/// Returns true if the init was successful, false otherwise.
bool RoadGeometryCoupler::init()
{
  // Do not init if there was not an odometry or an acceleration set
  if (!input_odometry_valid_ || !input_acceleration_valid_) return false;

  // Lambda function for checking if we are on track
  bool is_on_track = check_on_track_global(*raceline_, input_odometry_road_plane_);
  bool is_on_pitlane = check_on_track_global(*pitlane_, input_odometry_road_plane_);

  // Do not init if the vehicle is not on the track or pitlane
  if (!(is_on_track || is_on_pitlane)) return false;

  // Set the current track with preference to the racetrack
  if (is_on_track) {
    set_active_track(raceline_.get(), &current_raceline_pose_);
  } else if (is_on_pitlane) {
    set_active_track(pitlane_.get(), &current_pitlane_pose_);
  }

  auto do_initial_track_matching = [this](Track const & track) {
    // Do a global matching to the track
    return CurvilinearPose(track.get_cosy_handle()->convert_to_sn_and_get_idx_global(
      input_odometry_road_plane_.position_m.x, input_odometry_road_plane_.position_m.y,
      input_odometry_road_plane_.position_m.z, input_odometry_road_plane_.orientation_rad.z));
  };

  // Do a global matching the first time to the track
  current_raceline_pose_ = do_initial_track_matching(*raceline_);
  // Same for pitlane, do a global matching for the first time to init the position
  current_pitlane_pose_ = do_initial_track_matching(*pitlane_);

  // Create the measurement transformer
  measurement_transformer_ =
    std::make_unique<MeasurementTransformer>(active_track_, measurement_transformer_params_);

  // Create the road plane transformer
  road_plane_transformer_ = std::make_unique<RoadPlaneGeometryTransformer>(
    active_track_, *active_track_matching_result_,
    to_cartesian_pose_2d(input_odometry_road_plane_));

  // After this step, init was successful
  return true;
}
// Interface methods
void RoadGeometryCoupler::set_odometry(tam::types::control::Odometry const & odometry)
{
  input_odometry_road_plane_ = odometry;
  input_odometry_valid_ = true;
}
void RoadGeometryCoupler::set_acceleration(
  tam::types::control::AccelerationwithCovariances const & acceleration)
{
  input_acceleration_road_plane_ = acceleration;
  input_acceleration_valid_ = true;
}
void RoadGeometryCoupler::set_active_track(Track * track, CurvilinearPose * matching)
{
  active_track_ = track;
  active_track_matching_result_ = matching;
  track_max_idx_ = active_track_->s_coord().size();
}
bool RoadGeometryCoupler::switch_active_track(
  tam::types::control::Odometry const & global_cartesian_odometry)
{
  // Check the current state
  bool is_on_track = check_on_track_global(*raceline_, global_cartesian_odometry);
  bool is_on_pitlane = check_on_track_global(*pitlane_, global_cartesian_odometry);

  if (!is_on_track && !is_on_pitlane) {
    // If the vehicle is not on the track or pitlane, do not switch
    track_switch_req_count_ = 0;
    return false;
  }

  // Remember the previously active track to determine if a switch was done
  Track * desired_track = nullptr;
  CurvilinearPose * matching_for_desired_track = nullptr;

  // Prefer the track whenever possible
  if (is_on_track) {
    desired_track = raceline_.get();
    matching_for_desired_track = &current_raceline_pose_;
  } else if (is_on_pitlane) {
    desired_track = pitlane_.get();
    matching_for_desired_track = &current_pitlane_pose_;
  }

  if (is_on_track && is_on_pitlane) {
    // Vehicle is on both, just switch if the heading diff is small enough
    // Could fail in the future
    // Workaround - Switching the track is probably not intended when the jump in the heading is
    // bigger than 10 deg Then its probably an overpass or the raceline is very far off.
    if (
      (std::abs(matching_for_desired_track->chi_rad) > 0.1) ||
      std::abs(matching_for_desired_track->n_m) > 5.0) {
      track_switch_req_count_ = 0;
      return false;
    }
  }

  if (desired_track == active_track_) {
    // If no desired track is set, do not switch
    track_switch_req_count_ = 0;
    return false;
  }
  track_switch_req_count_++;
  // Block the track switch until 10 consequitve switches would have been done
  // To avoid frequent tracks switches in case of numerical instabilites
  if (track_switch_req_count_ < 10) return false;
  // IF none of the switches before blocked the switch, execute a track switch.
  set_active_track(desired_track, matching_for_desired_track);
  track_switch_req_count_ = 0;
  return true;
}
tam::types::control::Odometry RoadGeometryCoupler::get_transformed_odometry() const
{
  return odometry_global_cartesian_;
}
tam::types::control::AccelerationwithCovariances RoadGeometryCoupler::get_transformed_acceleration()
  const
{
  return acceleration_global_cartesian_;
}
VehicleLoad RoadGeometryCoupler::get_vehicle_load() const { return vehicle_load_; }
// Debug method
tam::tsl::LoggerAccessInterface::SharedPtr RoadGeometryCoupler::get_logger() const
{
  return value_logger_;
}
// Get shared pointer to param manager
tam::pmg::MgmtInterface::SharedPtr RoadGeometryCoupler::get_param_manager() const
{
  return param_manager_;
}
void RoadGeometryCoupler::log_debug_information()
{
  value_logger_->log("track_handling/pitlane_active", active_track_ == pitlane_.get());
  value_logger_->log("track_handling/raceline_active", active_track_ == raceline_.get());
  value_logger_->log("track_handling/track_switch_done", additional_debug_info_.track_switch_done);

  // Track 2D
  value_logger_->log("ref_line_road_plane", road_plane_transformer_diagnostics_.ref_line_pose);
  value_logger_->log(
    "ref_line_road_plane/active_segment/root_idx",
    road_plane_transformer_diagnostics_.segment_root_idx);
  value_logger_->log("road_plane/current_sigma", road_plane_transformer_diagnostics_.current_sigma);
  value_logger_->log(
    "road_plane/current_ref_line_index", road_plane_transformer_diagnostics_.current_local_index);
  value_logger_->log("road_plane/current_n", road_plane_transformer_diagnostics_.current_n);

  value_logger_->log("track/current_index", global_curvilinear_track_position_.idx);

  value_logger_->log(
    "track/current_banking_rad", active_track_->phi(global_curvilinear_track_position_.s_m));
  value_logger_->log(
    "track/current_slope_rad", active_track_->mu(global_curvilinear_track_position_.s_m));

  value_logger_->log("vehicle_load/force_N", vehicle_load_.force_N);
  value_logger_->log("vehicle_load/torque_Nm", vehicle_load_.torque_Nm);

  CartesianPose2D cart_pose;
  // Log the raceline
  value_logger_->log("raceline", current_raceline_pose_);
  cart_pose.x_m = raceline_->ref_line_x(current_raceline_pose_.s_m);
  cart_pose.y_m = raceline_->ref_line_y(current_raceline_pose_.s_m);
  cart_pose.yaw_rad =
    tam::helpers::geometry::normalize_angle(raceline_->theta(current_raceline_pose_.s_m));
  value_logger_->log("raceline", cart_pose);

  // Log the pitlane
  value_logger_->log("pitlane", current_pitlane_pose_);
  cart_pose.x_m = pitlane_->ref_line_x(current_pitlane_pose_.s_m);
  cart_pose.y_m = pitlane_->ref_line_y(current_pitlane_pose_.s_m);
  cart_pose.yaw_rad =
    tam::helpers::geometry::normalize_angle(pitlane_->theta(current_pitlane_pose_.s_m));
  value_logger_->log("pitlane", cart_pose);
}
}  // namespace tam::road_geometry_coupling
