// Copyright 2025 Simon Sagmeister
#pragma once
#include <memory>

#include "param_management_cpp/param_reference_manager.hpp"
#include "tum_road_geometry_coupling_cpp/track.hpp"
#include "tum_road_geometry_coupling_cpp/types.hpp"
#include "tum_helpers_cpp/numerical.hpp"
namespace tam::road_geometry_coupling
{
// Transform measurements from the road plane into the
// global cartesian frame
class MeasurementTransformer
{
public:
  struct Parameters
  {
    double h{0.0};
  };  // Parameters

public:
  using UniquePtr = std::unique_ptr<MeasurementTransformer>;
  explicit MeasurementTransformer(Track const * track, Parameters const & params)
  : track_(track), p_(params)
  {
  }
  void set_inputs(
    CurvilinearPose const & global_position,
    tam::types::control::Odometry const & road_plane_odometry,
    tam::types::control::AccelerationwithCovariances const & road_plane_acceleration);
  void transform_measurements();
  tam::types::control::Odometry get_global_cartesian_odometry() const;
  tam::types::control::AccelerationwithCovariances get_global_cartesian_acceleration() const;

  static void register_parameters(
    tam::pmg::ParamReferenceManager::SharedPtr param_manager, Parameters & p);

private:
  // Input data
  Track const * track_ = nullptr;
  CurvilinearPose global_position_{0, 0, 0, 0};
  tam::types::control::Odometry road_plane_odometry_{};
  tam::types::control::AccelerationwithCovariances road_plane_acceleration_{};
  // Store a reference to the param struct
  Parameters const & p_;
  // Intermediate helpers

  struct
  {
    // All values where it is no specified will be calculated in `calculate_intermediate_helpers()`
    double v_mps{0.0};
    double beta{0.0};
    double cos_beta{0.0};
    double sin_beta{0.0};
    double cos_phi{0.0};
    double sin_phi{0.0};
    double cos_mu{0.0};
    double sin_mu{0.0};
    double chi_vel_rad{0.0};
    double s_dot{0.0};
    double cos_chi_vel{0.0};
    double sin_chi_vel{0.0};
    double Omega_x_radpm{0.0};
    double Omega_y_radpm{0.0};
    double Omega_z_radpm{0.0};
    double Omega_prime_x_radpm2{0.0};
    double Omega_prime_y_radpm2{0.0};
    double Omega_prime_z_radpm2{0.0};
    double w{0.0};  /// Calculated in 'transform_angular_velocity()'
  } imh_;
  // Output data
  tam::types::control::Odometry global_cartesian_odometry_{};
  tam::types::control::AccelerationwithCovariances global_cartesian_acceleration_{};

private:
  // Subfunction for the individual calculations
  void calculate_intermediate_helpers();
  void transform_position();
  void transform_orientation();
  void transform_angular_velocity();
  void transform_linear_velocity();
  void transform_angular_acceleration();
  void transform_linear_acceleration();
  void add_roll_pitch();

  // Internal helper functions to transform between frames
  tam::types::common::Vector3D<double> transform_to_vehicle_frame(
    tam::types::common::Vector3D<double> const & vec_vel_frame);
  tam::types::common::Vector3D<double> transform_to_velocity_frame(
    tam::types::common::Vector3D<double> const & vec_veh_frame);

  // Parameter management
};
}  // namespace tam::road_geometry_coupling
