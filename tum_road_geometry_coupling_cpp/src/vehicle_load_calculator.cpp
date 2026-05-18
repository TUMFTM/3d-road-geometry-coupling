// Copyright 2025 Simon Sagmeister

#include "tum_road_geometry_coupling_cpp/vehicle_load_calculator.hpp"
namespace tam::road_geometry_coupling
{
void VehicleLoadCalculator::register_parameters(tam::pmg::ParamReferenceManager::SharedPtr param_manager)
{
  param_manager->declare_parameter(
    "vehicle.mass.total", &params.vehicle_mass_kg, 800.0, tam::pmg::ParameterType::DOUBLE,
    "Mass of the vehicle in kg");
  param_manager->declare_parameter(
    "vehicle.inertia.roll", &params.inertia_kgpm2.roll, 100.0, tam::pmg::ParameterType::DOUBLE,
    "Roll inertia of the vehicle in kg*m^2");
  param_manager->declare_parameter(
    "vehicle.inertia.pitch", &params.inertia_kgpm2.pitch, 500.0, tam::pmg::ParameterType::DOUBLE,
    "Pitch inertia of the vehicle in kg*m^2");
  param_manager->declare_parameter(
    "vehicle.inertia.yaw", &params.inertia_kgpm2.yaw, 1000.0, tam::pmg::ParameterType::DOUBLE,
    "Yaw inertia of the vehicle in kg*m^2");
}
VehicleLoad VehicleLoadCalculator::calculate_vehicle_load(
  tam::types::control::AccelerationwithCovariances const & road_plane_acceleration,
  tam::types::control::AccelerationwithCovariances const & global_cartesian_acceleration,
  tam::types::control::Odometry const & global_cartesian_odometry) const
{
  // Set the inputs for the vehicle load calculator
  VehicleLoad vehicle_load{{0, 0, 0}, {0, 0, 0}};

  // Calculate the forces
  vehicle_load.force_N = params.vehicle_mass_kg * (road_plane_acceleration.acceleration_mps2 -
                                                   global_cartesian_acceleration.acceleration_mps2);

  // Calculate the torques
  vehicle_load.torque_Nm.x =
    params.inertia_kgpm2.roll * (road_plane_acceleration.angular_acceleration_radps2.x -
                                 global_cartesian_acceleration.angular_acceleration_radps2.x) -
    (params.inertia_kgpm2.yaw - params.inertia_kgpm2.pitch) *
      global_cartesian_odometry.angular_velocity_radps.y *
      global_cartesian_odometry.angular_velocity_radps.z;
  vehicle_load.torque_Nm.y =
    params.inertia_kgpm2.pitch * (road_plane_acceleration.angular_acceleration_radps2.y -
                                  global_cartesian_acceleration.angular_acceleration_radps2.y) -
    (params.inertia_kgpm2.roll - params.inertia_kgpm2.yaw) *
      global_cartesian_odometry.angular_velocity_radps.z *
      global_cartesian_odometry.angular_velocity_radps.x;

  vehicle_load.torque_Nm.z =
    params.inertia_kgpm2.yaw * (road_plane_acceleration.angular_acceleration_radps2.z -
                                global_cartesian_acceleration.angular_acceleration_radps2.z) -
    (params.inertia_kgpm2.pitch - params.inertia_kgpm2.roll) *
      global_cartesian_odometry.angular_velocity_radps.x *
      global_cartesian_odometry.angular_velocity_radps.y;

  return vehicle_load;
}
}  // namespace tam::road_geometry_coupling
