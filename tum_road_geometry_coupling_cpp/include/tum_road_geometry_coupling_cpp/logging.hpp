// Copyright 2025 Simon Sagmeister
#pragma once
#include <string>

#include "tum_road_geometry_coupling_cpp/types.hpp"
#include "tsl_logger_cpp/type_support.hpp"
#include "tum_types_cpp/common.hpp"
// Contains the type support for logging custom types
namespace tam::tsl::type_support
{
template <>
inline void log<tam::types::common::Vector3D<double>>(
  TypeSupportInterface * logger, std::string const & name,
  tam::types::common::Vector3D<double> const & value)
{
  // You can use the type support system to again log any type that is supported by the interface.
  type_support::log(logger, name + "/x", value.x);
  type_support::log(logger, name + "/y", value.y);
  type_support::log(logger, name + "/z", value.z);
}

template <>
inline void log<tam::road_geometry_coupling::CurvilinearPose>(
  TypeSupportInterface * logger, std::string const & name,
  tam::road_geometry_coupling::CurvilinearPose const & value)
{
  // You can use the type support system to again log any type that is supported by the interface.
  type_support::log(logger, name + "/s_m", value.s_m);
  type_support::log(logger, name + "/n_m", value.n_m);
  type_support::log(logger, name + "/chi_rad", value.chi_rad);
  type_support::log(logger, name + "/idx", value.idx);
}

template <>
inline void log<tam::road_geometry_coupling::CartesianPose2D>(
  TypeSupportInterface * logger, std::string const & name,
  tam::road_geometry_coupling::CartesianPose2D const & value)
{
  // You can use the type support system to again log any type that is supported by the interface.
  type_support::log(logger, name + "/x_m", value.x_m);
  type_support::log(logger, name + "/y_m", value.y_m);
  type_support::log(logger, name + "/yaw_rad", value.yaw_rad);
}
}  // namespace tam::tsl::type_support
