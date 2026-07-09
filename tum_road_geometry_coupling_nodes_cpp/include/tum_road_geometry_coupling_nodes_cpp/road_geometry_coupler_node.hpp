// Copyright 2025 Simon Sagmeister

#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "tum_road_geometry_coupling_cpp/track_data.hpp"
#include "tum_road_geometry_coupling_cpp/road_geometry_coupler.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"
#include "tum_ros_helpers_cpp/qos.hpp"
namespace tam::road_geometry_coupling
{
/// Node that couples a planar vehicle model running in the road plane with a
/// 3D track. Subscribes to road-plane odometry and acceleration, publishes the
/// corresponding global-cartesian quantities and the road-geometry-induced
/// vehicle load as a \c geometry_msgs/WrenchStamped. The wrench is one source
/// consumed by the external-influence aggregator, which combines it with any
/// other force, grip and road-height sources.
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
  static constexpr std::string_view pub_topic_wrench_ =
    "/simulation/external_influences/wrench/road_geometry";

  /// Model owned by the node.
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
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_{};

  std::unique_ptr<tam::tsl::TSLPublisher> debug_publisher_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};
}  // namespace tam::road_geometry_coupling
