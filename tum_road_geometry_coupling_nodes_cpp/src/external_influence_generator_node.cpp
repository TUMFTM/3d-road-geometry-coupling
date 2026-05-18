// Copyright 2025 Simon Sagmeister
#include "tum_road_geometry_coupling_nodes_cpp/external_influence_generator_node.hpp"

#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
namespace tam::road_geometry_coupling
{
ExternalInfluenceGeneratorNode::ExternalInfluenceGeneratorNode(
  rclcpp::NodeOptions options, TrackData raceline_data, TrackData pitlane_data)
: RoadGeometryCouplerNode(
    options, std::move(raceline_data), std::move(pitlane_data),
    std::string(derived_node_name_))
{
  init_external_influences();
}
ExternalInfluenceGeneratorNode::ExternalInfluenceGeneratorNode(
  rclcpp::NodeOptions options, TrackData const & track_data)
: RoadGeometryCouplerNode(options, track_data, std::string(derived_node_name_))
{
  init_external_influences();
}
void ExternalInfluenceGeneratorNode::init_external_influences()
{
  this->declare_parameter<bool>("enable_tire_temperature_grip_scaling", false);
  this->declare_parameter<bool>("enable_track_grip_scale", false);
  this->get_parameter("enable_tire_temperature_grip_scaling", enable_tire_temperature_grip_scaling);
  this->get_parameter("enable_track_grip_scale", enable_track_grip_scale);

  auto qos = tam::ros::get_qos(tam::ros::TopicType::DEFAULT);

  if (enable_tire_temperature_grip_scaling) {
    tire_temp_scale_sub_ = this->create_subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>(
      "/simulation/tire_temp_grip_scale", qos,
      std::bind(&ExternalInfluenceGeneratorNode::tire_temp_callback, this, std::placeholders::_1));
  }
  if (enable_track_grip_scale) {
    track_grip_scale_sub_ = this->create_subscription<tum_msgs::msg::TUMFloat32Stamped>(
      "/simulation/track_grip_scale", qos,
      std::bind(
        &ExternalInfluenceGeneratorNode::track_grip_scale_callback, this, std::placeholders::_1));
  }
}
void ExternalInfluenceGeneratorNode::tire_temp_callback(
  const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  tire_temp_scale_ = tam::type_conversions::data_per_wheel_type_from_msg(msg->data);
}
void ExternalInfluenceGeneratorNode::track_grip_scale_callback(
  const tum_msgs::msg::TUMFloat32Stamped::SharedPtr msg)
{
  track_grip_scale_.front_left = msg->data;
  track_grip_scale_.front_right = msg->data;
  track_grip_scale_.rear_left = msg->data;
  track_grip_scale_.rear_right = msg->data;
}
tam::types::common::DataPerWheel<double>
ExternalInfluenceGeneratorNode::compute_friction_modifiers() const
{
  return {
    tire_temp_scale_.front_left * track_grip_scale_.front_left,
    tire_temp_scale_.front_right * track_grip_scale_.front_right,
    tire_temp_scale_.rear_left * track_grip_scale_.rear_left,
    tire_temp_scale_.rear_right * track_grip_scale_.rear_right,
  };
}
}  // namespace tam::road_geometry_coupling
