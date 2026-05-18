// Copyright 2025 Simon Sagmeister
#include "tum_road_geometry_coupling_cpp/track.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "tum_helpers_cpp/numerical.hpp"
namespace tam::road_geometry_coupling
{
namespace
{
/// Wrap s for cyclic interpolation. Mirrors \c tam::helpers::track::s_mod
/// exactly (including returning s_max for negative integer multiples of
/// s_max) without pulling in track_handler_cpp.
inline double s_mod(double s, double s_max)
{
  if (s < 0) {
    return s_max - std::fmod(std::abs(s), s_max);
  }
  return std::fmod(s, s_max);
}
}  // namespace
Track::Track(TrackData data) : data_(std::move(data))
{
  validate_data();
  build_cosy();
  compute_normals();
}
void Track::validate_data() const
{
  auto const n = data_.s_m.size();
  if (n < 2) {
    throw std::invalid_argument(
      "[tum_road_geometry_coupling_cpp]: Track requires at least two reference line points");
  }
  auto const mismatch = [n](std::vector<double> const & v) { return v.size() != n; };
  if (
    mismatch(data_.ref_line_x_m) || mismatch(data_.ref_line_y_m) ||
    mismatch(data_.ref_line_z_m) || mismatch(data_.theta_rad) || mismatch(data_.mu_rad) ||
    mismatch(data_.phi_rad) || mismatch(data_.omega_x_radpm) || mismatch(data_.omega_y_radpm) ||
    mismatch(data_.omega_z_radpm) || mismatch(data_.track_width_left_m) ||
    mismatch(data_.track_width_right_m)) {
    throw std::invalid_argument(
      "[tum_road_geometry_coupling_cpp]: All TrackData vectors must have the same size");
  }
}
void Track::build_cosy()
{
  // Match tam::common::Track behaviour: let the cosy derive its s coordinate
  // from the cartesian reference points.
  cosy_ = tam::helpers::cosy::CurvilinearCosySharedPtr{
    tam::helpers::cosy::CurvilinearCosy::create(
      data_.ref_line_x_m, data_.ref_line_y_m, data_.ref_line_z_m)
      ->build()
      .release()};
}
namespace
{
inline double interp_cyclic(
  double s, std::vector<double> const & s_coord, std::vector<double> const & values)
{
  double const s_max = s_coord.back();
  return tam::helpers::numerical::interp(s_mod(s, s_max), s_coord, values);
}
}  // namespace
double Track::ref_line_x(double s) const { return interp_cyclic(s, data_.s_m, data_.ref_line_x_m); }
double Track::ref_line_y(double s) const { return interp_cyclic(s, data_.s_m, data_.ref_line_y_m); }
double Track::ref_line_z(double s) const { return interp_cyclic(s, data_.s_m, data_.ref_line_z_m); }
double Track::theta(double s) const { return interp_cyclic(s, data_.s_m, data_.theta_rad); }
double Track::mu(double s) const { return interp_cyclic(s, data_.s_m, data_.mu_rad); }
double Track::phi(double s) const { return interp_cyclic(s, data_.s_m, data_.phi_rad); }
double Track::omega_x(double s) const { return interp_cyclic(s, data_.s_m, data_.omega_x_radpm); }
double Track::omega_y(double s) const { return interp_cyclic(s, data_.s_m, data_.omega_y_radpm); }
double Track::omega_z(double s) const { return interp_cyclic(s, data_.s_m, data_.omega_z_radpm); }
double Track::trackwidth_left(double s) const
{
  return interp_cyclic(s, data_.s_m, data_.track_width_left_m);
}
double Track::trackwidth_right(double s) const
{
  return interp_cyclic(s, data_.s_m, data_.track_width_right_m);
}
double Track::normal_x(double s) const { return interp_cyclic(s, data_.s_m, normal_x_); }
double Track::normal_y(double s) const { return interp_cyclic(s, data_.s_m, normal_y_); }
double Track::normal_z(double s) const { return interp_cyclic(s, data_.s_m, normal_z_); }
void Track::compute_normals()
{
  // Mirrors tam::common::Track::create_normal_vector: build per-point normals
  // from (theta, mu, phi) and normalize row-wise. Consumers interpolate these
  // components and accept the resulting vector as "close to unit length".
  auto const n = data_.s_m.size();
  normal_x_.resize(n);
  normal_y_.resize(n);
  normal_z_.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    double const ct = std::cos(data_.theta_rad[i]);
    double const st = std::sin(data_.theta_rad[i]);
    double const cm = std::cos(data_.mu_rad[i]);
    double const sm = std::sin(data_.mu_rad[i]);
    double const cp = std::cos(data_.phi_rad[i]);
    double const sp = std::sin(data_.phi_rad[i]);
    double nx = ct * sm * sp - st * cp;
    double ny = st * sm * sp + ct * cp;
    double nz = cm * sp;
    double const norm = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (norm > 0.0) {
      nx /= norm;
      ny /= norm;
      nz /= norm;
    }
    normal_x_[i] = nx;
    normal_y_[i] = ny;
    normal_z_[i] = nz;
  }
}
bool Track::on_track_global(double x, double y, double z, double z_range, double margin) const
{
  auto sn = cosy_->convert_to_sn_and_get_idx_global_2d(x, y, z, z_range);
  auto const & idx = std::get<1>(sn);
  auto const & sn_vec = std::get<0>(sn);
  double const w_left = tam::helpers::numerical::interp_from_idx(data_.track_width_left_m, idx);
  double const w_right = tam::helpers::numerical::interp_from_idx(data_.track_width_right_m, idx);
  double const n = sn_vec[1];
  return (n >= w_right - margin) && (n <= w_left + margin);
}
}  // namespace tam::road_geometry_coupling
