// Copyright 2025 Simon Sagmeister
#pragma once
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp"
#include "tum_road_geometry_coupling_cpp/road_plane_transformer.hpp"
#include "tum_road_geometry_coupling_cpp/track.hpp"
#include "tum_road_geometry_coupling_cpp/types.hpp"
#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_ros_helpers_cpp/timer.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::road_geometry_coupling::demo
{
/// @brief A demo node that drives a virtual vehicle around the road-plane
/// reference of a 3D track at a constant speed and publishes the resulting
/// road-plane odometry / acceleration. Intended to be paired with the
/// `RoadGeometryCouplerNode` to demonstrate the coupling end-to-end.
class RoadPlaneFollower : public rclcpp::Node
{
public:
  explicit RoadPlaneFollower(const rclcpp::NodeOptions & options);

private:
  std::string track_csv_path_;
  double velocity_mps_ = 0.0;
  bool export_road_plane_csv_ = false;
  std::string export_path_;

  double current_s_ = 0.0;
  std::size_t warmup_iterations_left_ = 100;
  static constexpr int TIMER_STEP_MS = 10;

  std::unique_ptr<Track> track_;
  tam::helpers::cosy::CurvilinearCosyPtr road_plane_cosy_;
  double road_plane_length_m_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    acceleration_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void build_road_plane_reference();
  void step();
};
}  // namespace tam::road_geometry_coupling::demo
