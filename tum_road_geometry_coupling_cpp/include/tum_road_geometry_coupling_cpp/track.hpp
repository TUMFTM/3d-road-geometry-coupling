// Copyright 2025 Simon Sagmeister
#pragma once
#include <memory>
#include <vector>

#include "tum_road_geometry_coupling_cpp/track_data.hpp"
#include "tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp"
namespace tam::road_geometry_coupling
{
/// Lightweight internal replacement for \c tam::common::Track that this module
/// can build from a \c TrackData value. Only exposes the subset of
/// functionality actually needed by the track geometry influence module.
class Track
{
public:
  using UniquePtr = std::unique_ptr<Track>;
  using SharedPtr = std::shared_ptr<Track>;

  explicit Track(TrackData data);

  // Data Access (full vectors)
  std::vector<double> const & s_coord() const { return data_.s_m; }
  std::vector<double> const & ref_line_x() const { return data_.ref_line_x_m; }
  std::vector<double> const & ref_line_y() const { return data_.ref_line_y_m; }
  std::vector<double> const & ref_line_z() const { return data_.ref_line_z_m; }
  std::vector<double> const & theta() const { return data_.theta_rad; }
  std::vector<double> const & mu() const { return data_.mu_rad; }
  std::vector<double> const & phi() const { return data_.phi_rad; }
  std::vector<double> const & omega_x() const { return data_.omega_x_radpm; }
  std::vector<double> const & omega_y() const { return data_.omega_y_radpm; }
  std::vector<double> const & omega_z() const { return data_.omega_z_radpm; }
  std::vector<double> const & trackwidth_left() const { return data_.track_width_left_m; }
  std::vector<double> const & trackwidth_right() const { return data_.track_width_right_m; }
  std::vector<double> const & normal_x() const { return normal_x_; }
  std::vector<double> const & normal_y() const { return normal_y_; }
  std::vector<double> const & normal_z() const { return normal_z_; }

  // Scalar interpolated access along the reference line (s wraps with track length)
  double ref_line_x(double s) const;
  double ref_line_y(double s) const;
  double ref_line_z(double s) const;
  double theta(double s) const;
  double mu(double s) const;
  double phi(double s) const;
  double omega_x(double s) const;
  double omega_y(double s) const;
  double omega_z(double s) const;
  double trackwidth_left(double s) const;
  double trackwidth_right(double s) const;
  // Components of the unit surface normal. Normals are precomputed per-point
  // (normalized individually) and then linearly interpolated between points -
  // the result is therefore not exactly unit length between sample points, but
  // this mirrors the behaviour of \c tam::common::Track::sn2cartesian.
  double normal_x(double s) const;
  double normal_y(double s) const;
  double normal_z(double s) const;

  /// Check whether a point (x, y, z) is on the track via 2D projection onto
  /// the reference line. \c z_range is the altitude band for matching, \c
  /// margin is added to the track widths on both sides.
  bool on_track_global(double x, double y, double z, double z_range, double margin) const;

  /// Non-owning handle to the underlying curvilinear coordinate system. This
  /// is kept stable so client code can invoke projection routines directly.
  tam::helpers::cosy::CurvilinearCosySharedPtr get_cosy_handle() const { return cosy_; }

  double track_length() const { return data_.s_m.empty() ? 0.0 : data_.s_m.back(); }

private:
  TrackData data_;
  tam::helpers::cosy::CurvilinearCosySharedPtr cosy_{};
  std::vector<double> normal_x_;
  std::vector<double> normal_y_;
  std::vector<double> normal_z_;

  void validate_data() const;
  void build_cosy();
  void compute_normals();
};
}  // namespace tam::road_geometry_coupling
