// Copyright 2025 Simon Sagmeister

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "tum_road_geometry_coupling_cpp/track_data.hpp"
#include "tum_road_geometry_coupling_cpp/road_geometry_coupler.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"
#include "tum_msgs/msg/tum_external_vehicle_influences.hpp"
#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::road_geometry_coupling
{
/// Base node that couples a planar vehicle model running in the road plane
/// with a 3D track. Subscribes to road-plane odometry and acceleration,
/// publishes the corresponding global-cartesian quantities and an
/// \c ExternalVehicleInfluences message containing the road-geometry-induced
/// vehicle load. Friction modifiers default to \c {1, 1, 1, 1}; derived nodes
/// can override \c compute_friction_modifiers to inject external influences.
class RoadGeometryCouplerNode : public rclcpp::Node
{
public:
  /// Construct with explicit raceline and pitlane reference lines.
  RoadGeometryCouplerNode(
    rclcpp::NodeOptions options, TrackData raceline_data, TrackData pitlane_data,
    std::string node_name = std::string(default_node_name_));
  /// Convenience overload for users that do not have a separate pitlane.
  /// The same \c TrackData is used for both raceline and pitlane.
  RoadGeometryCouplerNode(
    rclcpp::NodeOptions options, TrackData const & track_data,
    std::string node_name = std::string(default_node_name_));

protected:
  static constexpr std::string_view default_node_name_ = "RoadGeometryCoupler";
  static constexpr std::string_view ros_namespace_ = "/simulation";
  static constexpr std::string_view sub_topic_odom_ = "/simulation/road_plane/odometry";
  static constexpr std::string_view sub_topic_accel_ = "/simulation/road_plane/acceleration";
  static constexpr std::string_view pub_topic_odom_ = "/simulation/odometry";
  static constexpr std::string_view pub_topic_accel_ = "/simulation/acceleration";
  static constexpr std::string_view pub_topic_external_influence_ =
    "/simulation/road_plane/external_influences";

  /// Hook for derived classes to override the per-wheel friction modifiers
  /// that are written into the published \c ExternalVehicleInfluences message.
  /// The base implementation returns \c {1, 1, 1, 1}.
  virtual tam::types::common::DataPerWheel<double> compute_friction_modifiers() const;

  /// Model owned by the base class so derived classes can read its outputs.
  RoadGeometryCoupler::UniquePtr model_;

private:
  void init_after_model();
  void on_sub_inputs(
    nav_msgs::msg::Odometry::ConstSharedPtr odom,
    geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr accel);
  void step();

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
  message_filters::Subscriber<geometry_msgs::msg::AccelWithCovarianceStamped> accel_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<
    nav_msgs::msg::Odometry, geometry_msgs::msg::AccelWithCovarianceStamped>>
    time_synchronizer_{};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_{};
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_{};
  rclcpp::Publisher<tum_msgs::msg::TUMExternalVehicleInfluences>::SharedPtr
    external_influence_pub_{};

  std::unique_ptr<tam::tsl::TSLPublisher> debug_publisher_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
}  // namespace tam::road_geometry_coupling
