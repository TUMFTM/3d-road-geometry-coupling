// Copyright 2025 Simon Sagmeister
#pragma once
#include <limits>
#include <memory>
#include <string>
#include <tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp>

#include "param_management_cpp/param_reference_manager.hpp"
#include "tum_road_geometry_coupling_cpp/logging.hpp"
#include "tum_road_geometry_coupling_cpp/measurement_transformer.hpp"
#include "tum_road_geometry_coupling_cpp/road_plane_transformer.hpp"
#include "tum_road_geometry_coupling_cpp/track.hpp"
#include "tum_road_geometry_coupling_cpp/track_data.hpp"
#include "tum_road_geometry_coupling_cpp/types.hpp"
#include "tum_road_geometry_coupling_cpp/vehicle_load_calculator.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::road_geometry_coupling
{
class RoadGeometryCoupler
{
public:
  using UniquePtr = std::unique_ptr<RoadGeometryCoupler>;
  using SharedPtr = std::shared_ptr<RoadGeometryCoupler>;

public:
  /// Construct the generator from plain \c TrackData descriptions of the
  /// raceline and pitlane reference lines. The module builds the internal
  /// \c Track wrappers from these inputs and does not require
  /// track_handler_cpp.
  RoadGeometryCoupler(TrackData raceline_data, TrackData pitlane_data)
  : raceline_(std::make_unique<Track>(std::move(raceline_data))),
    pitlane_(std::make_unique<Track>(std::move(pitlane_data)))
  {
    MeasurementTransformer::register_parameters(param_manager_, measurement_transformer_params_);
    param_manager_->declare_parameter(
      "disable", &disabled_, false, tam::pmg::ParameterType::BOOL,
      "Disable the Track Geometry Influence Module");
  }
  /// Convenience constructor for users that do not have a separate pitlane.
  /// The same \c TrackData is used for both raceline and pitlane, so the
  /// internal track-switching logic effectively becomes a no-op.
  explicit RoadGeometryCoupler(TrackData const & track_data)
  : RoadGeometryCoupler(TrackData(track_data), TrackData(track_data))
  {
  }
  // Step method
  void step();
  void set_odometry(tam::types::control::Odometry const & odometry);
  void set_acceleration(tam::types::control::AccelerationwithCovariances const & acceleration);

  tam::types::control::Odometry get_transformed_odometry() const;
  tam::types::control::AccelerationwithCovariances get_transformed_acceleration() const;
  VehicleLoad get_vehicle_load() const;
  // Debug method
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() const;
  // Get shared pointer to param manager
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const;

private:
  // Helper functions
  static CartesianPose2D to_cartesian_pose_2d(tam::types::control::Odometry const & odometry);
  static bool check_on_track_global(
    Track const & track, tam::types::control::Odometry const & odometry);
  bool init();
  void set_active_track(Track * track, CurvilinearPose * matching);
  bool switch_active_track(tam::types::control::Odometry const & global_cartesian_odometry);
  void log_debug_information();

private:
  // External inputs
  tam::types::control::Odometry input_odometry_road_plane_;
  tam::types::control::AccelerationwithCovariances input_acceleration_road_plane_;
  bool input_odometry_valid_ = false;
  bool input_acceleration_valid_ = false;

private:
  // Debug Information
  struct
  {
    bool track_switch_done{false};
    // Add any additional debug information you want to track here
  } additional_debug_info_;

private:
  // Outputs
  tam::types::control::Odometry odometry_global_cartesian_;
  tam::types::control::AccelerationwithCovariances acceleration_global_cartesian_;
  VehicleLoad vehicle_load_;

private:
  // Internal helpers
  // Logging and parameter managemment
  tam::tsl::ValueLogger::SharedPtr value_logger_ = std::make_shared<tam::tsl::ValueLogger>();
  tam::pmg::ParamReferenceManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamReferenceManager>();
  // Track and Pitlane Objects
  Track::UniquePtr raceline_;
  Track::UniquePtr pitlane_;
  // Submodules for simplyfing the calculations
  RoadPlaneGeometryTransformer::UniquePtr road_plane_transformer_ = nullptr;
  MeasurementTransformer::UniquePtr measurement_transformer_ = nullptr;
  // Set up param management for the measurement transformer
  // This has to be done centrally since this object is recreated on the fly
  MeasurementTransformer::Parameters measurement_transformer_params_;
  VehicleLoadCalculator vehicle_load_calculator_{param_manager_};
  // Curvilinear Poses to Raceline, Pitlane and Raceline
  CurvilinearPose global_curvilinear_track_position_{0, 0, 0, 0};
  CurvilinearPose current_raceline_pose_{0, 0, 0, 0};
  CurvilinearPose current_pitlane_pose_{0, 0, 0, 0};
  RoadPlaneGeometryTransformer::Diagnostics road_plane_transformer_diagnostics_{};
  // Helpers variables for track switching
  Track * active_track_ = nullptr;
  CurvilinearPose * active_track_matching_result_ = nullptr;
  std::size_t track_max_idx_ = 0;
  std::size_t track_switch_req_count_ = 0;  // Only shift after 10 times

  // Helper to disable the module.
  bool disabled_ = false;
};
}  // namespace tam::road_geometry_coupling
