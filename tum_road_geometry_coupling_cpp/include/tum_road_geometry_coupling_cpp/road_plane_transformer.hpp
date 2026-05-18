// Copyright 2025 Simon Sagmeister
#pragma once
#include <memory>
#include <tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp>

#include "tum_road_geometry_coupling_cpp/helpers.hpp"
#include "tum_road_geometry_coupling_cpp/track.hpp"
#include "tum_road_geometry_coupling_cpp/types.hpp"
#include "tum_helpers_cpp/numerical.hpp"
namespace tam::road_geometry_coupling
{
/// @brief A class creating a virtual road plane and transforming the pose
/// @brief in this virtual road plane back to global curvilinear pose
class RoadPlaneGeometryTransformer
{
public:
  using UniquePtr = std::unique_ptr<RoadPlaneGeometryTransformer>;
  struct Diagnostics
  {
    std::size_t segment_root_idx{0};
    float current_local_index{0};
    double current_sigma{0};
    double current_n{0};
    CartesianPose2D ref_line_pose{0, 0, 0};
  };
  struct CurvilinearPoseWithDiagnostics
  {
    CurvilinearPose pose;
    Diagnostics diagnostics;
  };

public:
  RoadPlaneGeometryTransformer(
    Track const * track, CurvilinearPose const & curvilinear_pose_on_track,
    CartesianPose2D const & current_pose_road_plane);
  CurvilinearPoseWithDiagnostics transform_to_global_curvilinear_pose(
    CartesianPose2D const & road_plane_pose);

private:
  inline static const std::size_t DEFAULT_SEGMENT_LENGTH = 100;
  inline static const double REBUILD_RATIO = 0.8;
  // Define a struct for storing road plane segments
  struct RoadPlaneSegment
  {
    tam::helpers::cosy::CurvilinearCosyPtr ref_line = nullptr;
    std::size_t root_idx = 0;
    std::size_t num_points = DEFAULT_SEGMENT_LENGTH;
    Eigen::VectorXd sigma_m{
      DEFAULT_SEGMENT_LENGTH};  // s coordinate points where the cosy is defined
  };

private:
  RoadPlaneSegment current_segment_{};
  Track const * track_ = nullptr;
  std::size_t track_max_idx_ = 0;

protected:  // Make this protected to be able to use this in a child class for debugging purposes
  // Calculate the the upcoming roadplane segment
  RoadPlaneSegment calculate_road_plane_segment(
    CartesianPose2D const & road_plane_starting_pose, std::size_t const & root_idx,
    std::size_t const & segment_length = DEFAULT_SEGMENT_LENGTH) const;
  void extend_road_plane_segment(CartesianPose2D const & current_pose_road_plane);
  void calculate_curvilinear_pose(
    CartesianPose2D const & road_plane_pose, CurvilinearPoseWithDiagnostics & pose_ref);

  /// Calculate the global curvilinear pose and writes it into the global pose reference
};
}  // namespace tam::road_geometry_coupling
