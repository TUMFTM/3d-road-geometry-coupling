// Copyright 2025 Simon Sagmeister

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "tum_road_geometry_coupling_nodes_cpp/road_geometry_coupler_node.hpp"
#include "tum_msgs/msg/tum_float32_stamped.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::road_geometry_coupling
{
/// Adds external friction modifiers (tire temperature scaling and a global
/// track grip scale) on top of the pure road-geometry coupling. The base
/// node is responsible for the road-geometry coupling itself; this class
/// only contributes the \c lambda_mue values that are written into the
/// published \c ExternalVehicleInfluences message.
class ExternalInfluenceGeneratorNode : public RoadGeometryCouplerNode
{
public:
  ExternalInfluenceGeneratorNode(
    rclcpp::NodeOptions options, TrackData raceline_data, TrackData pitlane_data);
  ExternalInfluenceGeneratorNode(
    rclcpp::NodeOptions options, TrackData const & track_data);

protected:
  tam::types::common::DataPerWheel<double> compute_friction_modifiers() const override;

private:
  static constexpr std::string_view derived_node_name_ = "ExternalInfluenceGenerator";

  void init_external_influences();
  void tire_temp_callback(const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
  void track_grip_scale_callback(const tum_msgs::msg::TUMFloat32Stamped::SharedPtr msg);

  bool enable_tire_temperature_grip_scaling{false};
  bool enable_track_grip_scale{false};
  tam::types::common::DataPerWheel<double> tire_temp_scale_{1.0, 1.0, 1.0, 1.0};
  tam::types::common::DataPerWheel<double> track_grip_scale_{1.0, 1.0, 1.0, 1.0};
  rclcpp::Subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr tire_temp_scale_sub_{};
  rclcpp::Subscription<tum_msgs::msg::TUMFloat32Stamped>::SharedPtr track_grip_scale_sub_{};
};
}  // namespace tam::road_geometry_coupling
