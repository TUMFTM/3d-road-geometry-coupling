// Copyright 2025 Simon Sagmeister
#pragma once

#include "param_management_cpp/param_reference_manager.hpp"
#include "tum_road_geometry_coupling_cpp/types.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::road_geometry_coupling
{
class VehicleLoadCalculator
{
public:
  VehicleLoadCalculator() = default;
  explicit VehicleLoadCalculator(tam::pmg::ParamReferenceManager::SharedPtr param_manager)
  {
    register_parameters(param_manager);
  }
  void register_parameters(tam::pmg::ParamReferenceManager::SharedPtr param_manager);
  VehicleLoad calculate_vehicle_load(
    tam::types::control::AccelerationwithCovariances const & road_plane_acceleration,
    tam::types::control::AccelerationwithCovariances const & global_cartesian_acceleration,
    tam::types::control::Odometry const & global_cartesian_odometry) const;

private:
  VehicleLoad vehicle_load{{0, 0, 0}, {0, 0, 0}};
  struct
  {
    double vehicle_mass_kg{800.0};  // Mass of the vehicle in kg
    struct
    {
      double roll{100};
      double pitch{500};
      double yaw{1000};
    } inertia_kgpm2;
  } params;
};
}  // namespace tam::road_geometry_coupling
