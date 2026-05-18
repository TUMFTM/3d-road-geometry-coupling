// Copyright 2025 Simon Sagmeister
#pragma once
#include <eigen3/Eigen/Dense>
#include <tuple>
#include <tum_types_cpp/common.hpp>
#include "tum_helpers_cpp/geometry/geometry.hpp"
namespace tam::road_geometry_coupling
{
struct CartesianPose2D
{
  double x_m;
  double y_m;
  double yaw_rad;
};
struct CurvilinearPose
{
  // longitudinal coordinate on the ref line
  double s_m{0};
  // lateral offset to the ref line
  double n_m{0};
  // heading relative to the ref line
  double chi_rad{0};
  // index to ref line matched
  double idx{0};
  // Different Constructors
  CurvilinearPose(double s, double n, double chi, double index)
  : s_m(s), n_m(n), chi_rad(tam::helpers::geometry::normalize_angle(chi)), idx(index)
  {
  }
  explicit CurvilinearPose(std::tuple<Eigen::Vector3d, float> curvilinear_cosy_matching_result)
  : s_m(std::get<0>(curvilinear_cosy_matching_result)[0]),
    n_m(std::get<0>(curvilinear_cosy_matching_result)[1]),
    chi_rad(
      tam::helpers::geometry::normalize_angle(std::get<0>(curvilinear_cosy_matching_result)[2])),
    idx(std::get<1>(curvilinear_cosy_matching_result))
  {
  }
  CurvilinearPose() = default;
};
struct VehicleLoad{
  tam::types::common::Vector3D<double> force_N{0, 0, 0};  // Force on the vehicle body
  tam::types::common::Vector3D<double> torque_Nm{0, 0, 0};  // Torque on the vehicle body
};
}  // namespace tam::road_geometry_coupling

