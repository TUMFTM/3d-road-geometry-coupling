// Copyright 2025 Simon Sagmeister
#include "tum_road_geometry_coupling_cpp/measurement_transformer.hpp"

#include <cmath>

#include "tum_helpers_cpp/constants.hpp"
namespace tam::road_geometry_coupling
{
namespace
{
// 2D global heading corresponding to a chi angle (heading relative to the
// reference line) at position s. Rotates the local velocity direction
// (cos(chi), sin(chi), 0) by the track orientation (theta, mu, phi) and
// returns the heading of the resulting vector in the xy-plane.
double calc_global_heading_from_chi(Track const & track, double s, double chi)
{
  double const ct = std::cos(track.theta(s));
  double const st = std::sin(track.theta(s));
  double const cm = std::cos(track.mu(s));
  double const sm = std::sin(track.mu(s));
  double const cp = std::cos(track.phi(s));
  double const sp = std::sin(track.phi(s));
  double const cc = std::cos(chi);
  double const sc = std::sin(chi);
  // Columns 0 and 1 of the rotation matrix applied to (cc, sc, 0).
  double const gx = (ct * cm) * cc + (ct * sm * sp - st * cp) * sc;
  double const gy = (st * cm) * cc + (st * sm * sp + ct * cp) * sc;
  return std::atan2(gy, gx);
}
}  // namespace
void MeasurementTransformer::set_inputs(
  CurvilinearPose const & global_position,
  tam::types::control::Odometry const & road_plane_odometry,
  tam::types::control::AccelerationwithCovariances const & road_plane_acceleration)
{
  global_position_ = global_position;
  road_plane_odometry_ = road_plane_odometry;
  road_plane_acceleration_ = road_plane_acceleration;
}
void MeasurementTransformer::transform_measurements()
{
  // First calc helper values for the following functions
  calculate_intermediate_helpers();
  // Transform the position
  transform_position();
  // Transform the orientation
  transform_orientation();
  // Transform the angular velocity
  transform_angular_velocity();
  // Transform the linear velocity
  transform_linear_velocity();
  // Transform the angular acceleration
  transform_angular_acceleration();
  // Transform the linear acceleration
  transform_linear_acceleration();
  // Add roll and pitch angles
  add_roll_pitch();
}
tam::types::control::Odometry MeasurementTransformer::get_global_cartesian_odometry() const
{
  return global_cartesian_odometry_;
}
tam::types::control::AccelerationwithCovariances
MeasurementTransformer::get_global_cartesian_acceleration() const
{
  return global_cartesian_acceleration_;
}
void MeasurementTransformer::calculate_intermediate_helpers()
{
  // Calculate the velocity in m/s
  imh_.v_mps = std::sqrt(
    std::pow(road_plane_odometry_.velocity_mps.x, 2) +
    std::pow(road_plane_odometry_.velocity_mps.y, 2));

  // Calc beta
  imh_.beta = std::atan2(
    road_plane_odometry_.velocity_mps.y, std::max(3.0, road_plane_odometry_.velocity_mps.x));

  // Calc cos and sin beta
  imh_.cos_beta = std::cos(imh_.beta);
  imh_.sin_beta = std::sin(imh_.beta);

  // Calc chi_vel_rad
  imh_.chi_vel_rad = global_position_.chi_rad + imh_.beta;

  // Calc cos and sin of chi
  imh_.cos_chi_vel = std::cos(imh_.chi_vel_rad);
  imh_.sin_chi_vel = std::sin(imh_.chi_vel_rad);

  // Calc cos and sin of phi and mu
  imh_.cos_phi = std::cos(track_->phi(global_position_.s_m));
  imh_.sin_phi = std::sin(track_->phi(global_position_.s_m));
  imh_.cos_mu = std::cos(track_->mu(global_position_.s_m));
  imh_.sin_mu = std::sin(track_->mu(global_position_.s_m));

  // Calc omegas
  imh_.Omega_x_radpm = track_->omega_x(global_position_.s_m);
  imh_.Omega_y_radpm = track_->omega_y(global_position_.s_m);
  imh_.Omega_z_radpm = track_->omega_z(global_position_.s_m);

  // Calc omega prime
  double s_low = std::max(0.0, global_position_.s_m - 0.1);
  double s_high = std::min(global_position_.s_m + 0.1, track_->s_coord().back());
  imh_.Omega_prime_x_radpm2 = (track_->omega_x(s_high) - track_->omega_x(s_low)) / (s_high - s_low);
  imh_.Omega_prime_y_radpm2 = (track_->omega_y(s_high) - track_->omega_y(s_low)) / (s_high - s_low);
  imh_.Omega_prime_z_radpm2 = (track_->omega_z(s_high) - track_->omega_z(s_low)) / (s_high - s_low);

  // Calc s_dot
  imh_.s_dot = imh_.v_mps * imh_.cos_chi_vel / (1 - global_position_.n_m * imh_.Omega_z_radpm);
}
void MeasurementTransformer::transform_position()
{
  double const s = global_position_.s_m;
  double const n = global_position_.n_m;
  // Replicates tam::common::Track::sn2cartesian: interpolate the per-point
  // precomputed (and per-point normalized) normal components at s, then add
  // n * normal to the reference line point.
  global_cartesian_odometry_.position_m.x = track_->ref_line_x(s) + n * track_->normal_x(s);
  global_cartesian_odometry_.position_m.y = track_->ref_line_y(s) + n * track_->normal_y(s);
  // Add z track plus road plane z to be able to consider heave dynamics
  global_cartesian_odometry_.position_m.z =
    track_->ref_line_z(s) + n * track_->normal_z(s) + road_plane_odometry_.position_m.z;
}
void MeasurementTransformer::transform_orientation()
{
  // First calculate the current banking and slope angle from the track
  double banking_angle_rad = imh_.cos_chi_vel * track_->phi(global_position_.s_m) +
                             imh_.sin_chi_vel * track_->mu(global_position_.s_m);
  double slope_angle_rad = -imh_.sin_chi_vel * track_->phi(global_position_.s_m) +
                           imh_.cos_chi_vel * track_->mu(global_position_.s_m);

  // Add this angles to the roll and pitch angles measured in the road plane
  global_cartesian_odometry_.orientation_rad.x =
    road_plane_odometry_.orientation_rad.x + slope_angle_rad;
  global_cartesian_odometry_.orientation_rad.y =
    road_plane_odometry_.orientation_rad.y + banking_angle_rad;
  global_cartesian_odometry_.orientation_rad.z =
    calc_global_heading_from_chi(*track_, global_position_.s_m, global_position_.chi_rad);
}
void MeasurementTransformer::transform_angular_velocity()
{
  // Transform to the track induced angular velocites
  tam::types::common::Vector3D<double> zeta_omega_vel_radps;

  // Calculate the transformation vector in velocity frame
  zeta_omega_vel_radps.x =
    (imh_.cos_chi_vel * imh_.Omega_x_radpm + imh_.sin_chi_vel * imh_.Omega_y_radpm) * imh_.s_dot;
  zeta_omega_vel_radps.y =
    (imh_.cos_chi_vel * imh_.Omega_y_radpm - imh_.sin_chi_vel * imh_.Omega_x_radpm) * imh_.s_dot;
  zeta_omega_vel_radps.z = 0.0;

  // Rotate transformation vector to vehicle frame
  auto zeta_omega_veh_radps = transform_to_vehicle_frame(zeta_omega_vel_radps);

  // Transform by summation of road plane values and zeta.
  tam::types::common::Vector3D<double> road_plane_input{
    0, 0, road_plane_odometry_.angular_velocity_radps.z};
  global_cartesian_odometry_.angular_velocity_radps = road_plane_input + zeta_omega_veh_radps;

  // Calculate w
  imh_.w = global_position_.n_m * global_cartesian_odometry_.angular_velocity_radps.x;
}
void MeasurementTransformer::transform_linear_velocity()
{
  tam::types::common::Vector3D<double> zeta_v_vel_frame;

  // Calculate the transformation vector in velocity frame
  auto omega_vel_radps =
    transform_to_velocity_frame(global_cartesian_odometry_.angular_velocity_radps);

  zeta_v_vel_frame.x = omega_vel_radps.y * p_.h;
  zeta_v_vel_frame.y = -omega_vel_radps.x * p_.h;
  zeta_v_vel_frame.z = imh_.w;

  // Rotate transformation vector to vehicle frame
  auto zeta_v_veh_frame = transform_to_vehicle_frame(zeta_v_vel_frame);

  // Get results by adding road plane values and zeta
  global_cartesian_odometry_.velocity_mps = road_plane_odometry_.velocity_mps + zeta_v_veh_frame;
}
void MeasurementTransformer::transform_angular_acceleration()
{
  // Transform rotiational rates to velocity frame
  auto omega_vel_radps =
    transform_to_velocity_frame(global_cartesian_odometry_.angular_velocity_radps);

  // Warning: This is a small inaccuracy since we are using the acceleration from the last time step
  // to break the algebraic loop in the calculation, but should have virtually no measurable impact.
  double v_dot_mps2 =
    transform_to_velocity_frame(global_cartesian_acceleration_.acceleration_mps2).x;

  // Calc chi_vel_dot
  double chi_vel_dot = omega_vel_radps.z - imh_.Omega_x_radpm * imh_.s_dot;

  // Calc n_dot
  double n_dot = -imh_.v_mps * imh_.sin_chi_vel;

  // Calc s dot dot
  double s_ddot = ((v_dot_mps2 * imh_.cos_chi_vel - imh_.v_mps * imh_.sin_chi_vel * chi_vel_dot) *
                     (1 - global_position_.n_m * imh_.Omega_z_radpm) -
                   imh_.v_mps * imh_.cos_chi_vel *
                     (-n_dot * imh_.Omega_z_radpm -
                      global_position_.n_m * imh_.Omega_prime_z_radpm2 * imh_.s_dot)) /
                  std::pow(1 - global_position_.n_m * imh_.Omega_z_radpm, 2);

  tam::types::common::Vector3D<double> zeta_omega_dot_vel_radps2;

  zeta_omega_dot_vel_radps2.x =
    (imh_.Omega_prime_x_radpm2 * imh_.cos_chi_vel * std::pow(imh_.s_dot, 2.0)) -
    (imh_.Omega_x_radpm * imh_.sin_chi_vel * chi_vel_dot * imh_.s_dot) +
    (imh_.Omega_x_radpm * imh_.cos_chi_vel * s_ddot) +
    (imh_.Omega_prime_y_radpm2 * imh_.sin_chi_vel * std::pow(imh_.s_dot, 2.0)) +
    (imh_.cos_chi_vel * chi_vel_dot * imh_.Omega_y_radpm * imh_.s_dot) +
    (imh_.Omega_y_radpm * imh_.sin_chi_vel * s_ddot);

  zeta_omega_dot_vel_radps2.y =
    (imh_.Omega_prime_y_radpm2 * imh_.cos_chi_vel * std::pow(imh_.s_dot, 2.0)) -
    (imh_.Omega_y_radpm * imh_.sin_chi_vel * chi_vel_dot * imh_.s_dot) +
    (imh_.Omega_y_radpm * imh_.cos_chi_vel * s_ddot) -
    (imh_.Omega_prime_x_radpm2 * imh_.sin_chi_vel * std::pow(imh_.s_dot, 2.0)) -
    (imh_.cos_chi_vel * chi_vel_dot * imh_.Omega_x_radpm * imh_.s_dot) -
    (imh_.Omega_x_radpm * imh_.sin_chi_vel * s_ddot);

  zeta_omega_dot_vel_radps2.z = 0;

  // Transform the angular acceleration to the vehicle frame

  auto zeta_omega_dot_veh_radps2 = transform_to_vehicle_frame(zeta_omega_dot_vel_radps2);

  // Apply transform to input.
  tam::types::common::Vector3D<double> road_plane_input{
    0, 0, road_plane_acceleration_.angular_acceleration_radps2.z};

  global_cartesian_acceleration_.angular_acceleration_radps2 =
    road_plane_input + zeta_omega_dot_veh_radps2;
}
void MeasurementTransformer::transform_linear_acceleration()
{
  // Transforming yaw rates and accelerations into vel frame
  auto omega_vel_radps =
    transform_to_velocity_frame(global_cartesian_odometry_.angular_velocity_radps);
  auto omega_dot_vel_radps2 =
    transform_to_velocity_frame(global_cartesian_acceleration_.angular_acceleration_radps2);

  // h dot is v_z from the vehicle
  double h_dot = road_plane_odometry_.velocity_mps.z;

  // double n_dot
  double n_dot = -imh_.v_mps * imh_.sin_chi_vel;

  // double w_dot
  double w_dot = n_dot * omega_vel_radps.x + global_position_.n_m * omega_dot_vel_radps2.x;

  // Allocate the vectors
  tam::types::common::Vector3D<double> zeta_a_vel_frame, Gamma_vel_frame;

  // Zeta: X direction
  zeta_a_vel_frame.x = omega_dot_vel_radps2.y * p_.h + omega_vel_radps.y * (imh_.w + 2.0 * h_dot) +
                       omega_vel_radps.x * omega_vel_radps.z * p_.h;
  // Zeta: Y direction
  zeta_a_vel_frame.y = -omega_dot_vel_radps2.x * p_.h - omega_vel_radps.x * (imh_.w + 2.0 * h_dot) +
                       omega_vel_radps.y * omega_vel_radps.z * p_.h;

  // Zeta: Z direction
  zeta_a_vel_frame.z = w_dot +
                       -(std::pow(omega_vel_radps.x, 2) + std::pow(omega_vel_radps.y, 2)) * p_.h -
                       omega_vel_radps.y * imh_.v_mps;

  // Gamma: X direction
  Gamma_vel_frame.x = tam::constants::g_earth * (imh_.cos_mu * imh_.sin_phi * imh_.sin_chi_vel -
                                                 imh_.sin_mu * imh_.cos_chi_vel);

  // Gamma: Y direction
  Gamma_vel_frame.y = tam::constants::g_earth * (imh_.sin_mu * imh_.sin_chi_vel +
                                                 imh_.cos_mu * imh_.sin_phi * imh_.cos_chi_vel);

  // Gamma: Z direction
  Gamma_vel_frame.z = tam::constants::g_earth * (imh_.cos_mu * imh_.cos_phi - 1);

  // Rotate everything to the vehicle frame
  auto transformation_vehicle_frame =
    transform_to_vehicle_frame(zeta_a_vel_frame + Gamma_vel_frame);

  // Add everything up
  global_cartesian_acceleration_.acceleration_mps2 =
    road_plane_acceleration_.acceleration_mps2 + transformation_vehicle_frame;
}
void MeasurementTransformer::add_roll_pitch()
{
  // Add the roll and pitch angles to the global cartesian odometry
  global_cartesian_odometry_.orientation_rad.x += road_plane_odometry_.orientation_rad.x;
  global_cartesian_odometry_.orientation_rad.y += road_plane_odometry_.orientation_rad.y;
}
tam::types::common::Vector3D<double> MeasurementTransformer::transform_to_vehicle_frame(
  tam::types::common::Vector3D<double> const & vec_vel_frame)
{
  // Transform the vector from the velocity frame to the vehicle frame
  tam::types::common::Vector3D<double> vec_vehicle_frame;
  vec_vehicle_frame.x = imh_.cos_beta * vec_vel_frame.x - imh_.sin_beta * vec_vel_frame.y;
  vec_vehicle_frame.y = imh_.sin_beta * vec_vel_frame.x + imh_.cos_beta * vec_vel_frame.y;
  vec_vehicle_frame.z = vec_vel_frame.z;
  return vec_vehicle_frame;
}
tam::types::common::Vector3D<double> MeasurementTransformer::transform_to_velocity_frame(
  tam::types::common::Vector3D<double> const & vec_veh_frame)
{
  // Transform the vector from the velocity frame to the vehicle frame
  tam::types::common::Vector3D<double> vec_vehicle_frame;
  vec_vehicle_frame.x = imh_.cos_beta * vec_veh_frame.x + imh_.sin_beta * vec_veh_frame.y;
  vec_vehicle_frame.y = -imh_.sin_beta * vec_veh_frame.x + imh_.cos_beta * vec_veh_frame.y;
  vec_vehicle_frame.z = vec_veh_frame.z;
  return vec_vehicle_frame;
}
void MeasurementTransformer::register_parameters(
  tam::pmg::ParamReferenceManager::SharedPtr param_manager, Parameters & p)
{
  param_manager->declare_parameter(
    "vehicle.dimension.cog_height", &p.h, 0.3, tam::pmg::ParameterType::DOUBLE,
    "Center of gravity height of the vehicle in m");
}
}  // namespace tam::road_geometry_coupling
