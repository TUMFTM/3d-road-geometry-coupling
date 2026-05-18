// Copyright 2025 Simon Sagmeister
#include "tum_road_plane_follower_cpp/road_plane_follower.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "tum_helpers_cpp/geometry/geometry.hpp"
#include "tum_road_geometry_coupling_cpp/track_data.hpp"
#include "tum_road_geometry_coupling_cpp/track_data_io.hpp"
namespace tam::road_geometry_coupling::demo
{
namespace
{
struct RoadPlaneReference
{
  Eigen::VectorXd sigma_m;
  Eigen::VectorXd x_road_plane;
  Eigen::VectorXd y_road_plane;
  Eigen::VectorXd yaw_road_plane;
};

/// Build a full-track road-plane reference using the same circular-arc
/// integration as `RoadPlaneGeometryTransformer::calculate_road_plane_segment`.
/// Logic copied verbatim from the lib so the integrated points are stored
/// directly (no second pass through `CurvilinearCosy::convert_to_cartesian`).
RoadPlaneReference build_full_road_plane_reference(
  Track const & track, CartesianPose2D const & starting_pose)
{
  auto const & s_track = track.s_coord();
  auto const & omega_z_track = track.omega_z();
  auto const segment_length = static_cast<std::size_t>(s_track.size());

  RoadPlaneReference ref;
  ref.sigma_m.resize(segment_length);
  ref.x_road_plane.resize(segment_length);
  ref.y_road_plane.resize(segment_length);
  ref.yaw_road_plane.resize(segment_length);

  ref.x_road_plane(0) = starting_pose.x_m;
  ref.y_road_plane(0) = starting_pose.y_m;
  ref.yaw_road_plane(0) = starting_pose.yaw_rad;
  ref.sigma_m(0) = 0.0;

  std::size_t const root_idx = 0;
  for (std::size_t i = 1; i < segment_length; ++i) {
    std::size_t idx{(root_idx + i) % (s_track.size() - 1)};
    std::size_t prev_idx = (idx == 0) ? 0 : idx - 1;
    double s_dist{0.0};
    if (idx == 0) {
      prev_idx = s_track.size() - 1;
      s_dist = s_track[prev_idx] - s_track[prev_idx - 1];
    } else {
      s_dist = s_track[idx] - s_track[prev_idx];
    }
    double delta_lat{0.0}, delta_lon{s_dist};
    double const omega_z_median = (omega_z_track[prev_idx] + omega_z_track[idx]) * 0.5;
    if (std::abs(omega_z_median) > 1e-6) {
      delta_lat = (1 - std::cos(omega_z_median * s_dist)) / omega_z_median;
      delta_lon = std::sin(omega_z_median * s_dist) / omega_z_median;
    }
    ref.x_road_plane(i) = ref.x_road_plane(i - 1) +
                          delta_lon * std::cos(ref.yaw_road_plane(i - 1)) -
                          delta_lat * std::sin(ref.yaw_road_plane(i - 1));
    ref.y_road_plane(i) = ref.y_road_plane(i - 1) +
                          delta_lon * std::sin(ref.yaw_road_plane(i - 1)) +
                          delta_lat * std::cos(ref.yaw_road_plane(i - 1));
    ref.yaw_road_plane(i) = tam::helpers::geometry::normalize_angle(
      ref.yaw_road_plane(i - 1) + s_dist * omega_z_median);
    ref.sigma_m(i) = ref.sigma_m(i - 1) + s_dist;
  }
  return ref;
}

void dump_road_plane_to_csv(
  std::string const & path, RoadPlaneReference const & ref, Track const & track)
{
  std::ofstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("RoadPlaneFollower: could not open '" + path + "' for writing");
  }
  file << "sigma_m,x_track,y_track,yaw_track,x_road_plane,y_road_plane,yaw_road_plane\n";
  for (Eigen::Index i = 0; i < ref.sigma_m.size(); ++i) {
    auto const sigma = ref.sigma_m[i];
    file << sigma << ',' << track.ref_line_x(sigma) << ',' << track.ref_line_y(sigma) << ','
         << track.theta(sigma) << ',' << ref.x_road_plane[i] << ',' << ref.y_road_plane[i] << ','
         << ref.yaw_road_plane[i] << '\n';
  }
}
}  // namespace

RoadPlaneFollower::RoadPlaneFollower(const rclcpp::NodeOptions & options)
: Node("RoadPlaneFollower", options)
{
  track_csv_path_ = this->declare_parameter<std::string>("track_csv_path", "");
  velocity_mps_ = this->declare_parameter<double>("velocity_mps", 10.0 * std::sqrt(2.0));
  export_road_plane_csv_ = this->declare_parameter<bool>("export_road_plane_csv", false);

  // Default export path: timestamped file in the working directory the
  // executable was launched from (consistent with the demo bag default).
  auto const now = std::chrono::system_clock::now();
  auto const time_now = std::chrono::system_clock::to_time_t(now);
  std::ostringstream default_export_path;
  default_export_path << "road_plane_ref_"
                      << std::put_time(std::localtime(&time_now), "%Y%m%d_%H%M%S") << ".csv";
  export_path_ =
    this->declare_parameter<std::string>("export_path", default_export_path.str());

  if (track_csv_path_.empty()) {
    throw std::runtime_error(
      "RoadPlaneFollower: parameter 'track_csv_path' must point at a TrackData CSV file.");
  }

  build_road_plane_reference();

  odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
    "/simulation/road_plane/odometry", tam::ros::get_qos());
  acceleration_publisher_ =
    this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "/simulation/road_plane/acceleration", tam::ros::get_qos());

  timer_ =
    tam::create_timer(this, std::chrono::milliseconds(TIMER_STEP_MS), [this]() { step(); });
}

void RoadPlaneFollower::build_road_plane_reference()
{
  auto track_data = load_track_data_from_csv(track_csv_path_);
  track_ = std::make_unique<Track>(std::move(track_data));

  CartesianPose2D const starting_pose{
    track_->ref_line_x().front(), track_->ref_line_y().front(), track_->theta().front()};

  auto ref = build_full_road_plane_reference(*track_, starting_pose);

  current_s_ = -track_->ref_line_x().front();

  road_plane_cosy_ = tam::helpers::cosy::CurvilinearCosy::create(
                       ref.x_road_plane, ref.y_road_plane,
                       Eigen::VectorXd::Zero(ref.sigma_m.size()))
                       ->set_tangent(
                         ref.yaw_road_plane.array().cos(), ref.yaw_road_plane.array().sin(),
                         Eigen::VectorXd::Zero(ref.sigma_m.size()))
                       ->set_s(ref.sigma_m)
                       ->build();
  road_plane_length_m_ = ref.sigma_m[ref.sigma_m.size() - 1];

  if (export_road_plane_csv_) {
    dump_road_plane_to_csv(export_path_, ref, *track_);
    RCLCPP_INFO(
      this->get_logger(), "RoadPlaneFollower: wrote road-plane reference to '%s'.",
      export_path_.c_str());
  }
}

void RoadPlaneFollower::step()
{
  if (warmup_iterations_left_ > 0) {
    warmup_iterations_left_--;
  } else {
    current_s_ += velocity_mps_ * TIMER_STEP_MS * 1e-3;
  }

  if (current_s_ > road_plane_length_m_) {
    RCLCPP_INFO(this->get_logger(), "RoadPlaneFollower: reached end of track, shutting down.");
    rclcpp::shutdown();
    return;
  }

  auto const xy_yaw = road_plane_cosy_->convert_to_cartesian(current_s_, 0.0, 0.0);
  auto const stamp = this->now();

  tam::types::control::Odometry odometry;
  odometry.position_m.x = xy_yaw[0];
  odometry.position_m.y = xy_yaw[1];
  odometry.position_m.z = 0.0;
  odometry.orientation_rad.x = 0.0;
  odometry.orientation_rad.y = 0.0;
  odometry.orientation_rad.z = xy_yaw[2];
  odometry.velocity_mps.x = velocity_mps_;
  odometry.velocity_mps.y = 0.0;
  odometry.velocity_mps.z = 0.0;
  odometry.angular_velocity_radps.x = 0.0;
  odometry.angular_velocity_radps.y = 0.0;
  odometry.angular_velocity_radps.z = track_->omega_z(current_s_) * velocity_mps_;

  auto odometry_msg = tam::type_conversions::odometry_msg_from_type(odometry);
  odometry_msg.header.stamp = stamp;
  odometry_msg.header.frame_id = "road_plane_frame";
  odometry_msg.child_frame_id = "road_plane_child_frame";

  tam::types::control::AccelerationwithCovariances acceleration;
  auto const lat_acc = track_->omega_z(current_s_) * velocity_mps_ * velocity_mps_;
  acceleration.acceleration_mps2.x = 0.0;
  acceleration.acceleration_mps2.y = lat_acc;
  acceleration.acceleration_mps2.z = 9.81;

  auto acceleration_msg =
    tam::type_conversions::accel_with_covariance_stamped_msg_from_type(acceleration);
  acceleration_msg.header.stamp = stamp;

  odometry_publisher_->publish(odometry_msg);
  acceleration_publisher_->publish(acceleration_msg);
}
}  // namespace tam::road_geometry_coupling::demo
