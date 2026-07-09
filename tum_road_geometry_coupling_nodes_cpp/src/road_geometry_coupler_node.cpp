// Copyright 2025 Simon Sagmeister
#include "tum_road_geometry_coupling_nodes_cpp/road_geometry_coupler_node.hpp"

#include "param_management_ros2_integration_cpp/helper_functions.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_types_cpp/coordinate_frames.hpp"
namespace tam::road_geometry_coupling
{
RoadGeometryCouplerNode::RoadGeometryCouplerNode(
  rclcpp::NodeOptions options, TrackData raceline_data, TrackData pitlane_data,
  std::string node_name)
: Node(std::move(node_name), std::string(ros_namespace_), options)
{
  model_ = std::make_unique<RoadGeometryCoupler>(
    std::move(raceline_data), std::move(pitlane_data));
  init_after_model();
}
RoadGeometryCouplerNode::RoadGeometryCouplerNode(
  rclcpp::NodeOptions options, TrackData const & track_data, std::string node_name)
: Node(std::move(node_name), std::string(ros_namespace_), options)
{
  model_ = std::make_unique<RoadGeometryCoupler>(track_data);
  init_after_model();
}
void RoadGeometryCouplerNode::init_after_model()
{
  debug_publisher_ = std::make_unique<tam::tsl::TSLPublisher>(this, model_->get_logger());

  param_callback_handle_ =
    tam::pmg::connect_param_manager_to_ros_cb(this, model_->get_param_manager());
  tam::pmg::declare_ros_params_from_param_manager(this, model_->get_param_manager().get());

  auto qos = tam::ros::get_qos(tam::ros::TopicType::DEFAULT);

  odometry_sub_.subscribe(
    this, std::string(sub_topic_odom_),
    qos.get_rmw_qos_profile());
  accel_sub_.subscribe(
    this, std::string(sub_topic_accel_),
    qos.get_rmw_qos_profile());
  time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<
    nav_msgs::msg::Odometry, geometry_msgs::msg::AccelWithCovarianceStamped>>(
    odometry_sub_, accel_sub_, 1);
  time_synchronizer_->registerCallback(std::bind(
    &RoadGeometryCouplerNode::on_sub_inputs, this, std::placeholders::_1, std::placeholders::_2));

  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(std::string(pub_topic_odom_), qos);
  accel_pub_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    std::string(pub_topic_accel_), qos);
  wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
    std::string(pub_topic_wrench_), qos);
}
void RoadGeometryCouplerNode::on_sub_inputs(
  nav_msgs::msg::Odometry::ConstSharedPtr odom,
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr accel)
{
  model_->set_odometry(tam::type_conversions::odometry_type_from_msg(*odom));
  model_->set_acceleration(
    tam::type_conversions::accel_with_covariance_stamped_type_from_msg(*accel));
  step();
}
void RoadGeometryCouplerNode::step()
{
  model_->step();
  auto stamp = get_clock()->now();
  auto odom_msg = tam::type_conversions::odometry_msg_from_type(model_->get_transformed_odometry());
  auto accel_msg = tam::type_conversions::accel_with_covariance_stamped_msg_from_type(
    model_->get_transformed_acceleration());

  auto vehicle_load = model_->get_vehicle_load();
  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.wrench.force = tam::type_conversions::vector_3d_msg_from_type(vehicle_load.force_N);
  wrench_msg.wrench.torque = tam::type_conversions::vector_3d_msg_from_type(vehicle_load.torque_Nm);

  odom_msg.header.stamp = stamp;
  accel_msg.header.stamp = stamp;
  wrench_msg.header.stamp = stamp;

  odom_msg.header.frame_id = CoordinateFrames::local_cartesian;
  odom_msg.child_frame_id = CoordinateFrames::vehicle_cg;
  accel_msg.header.frame_id = CoordinateFrames::vehicle_cg;
  wrench_msg.header.frame_id = CoordinateFrames::vehicle_cg_footprint;

  wrench_pub_->publish(std::move(wrench_msg));
  odometry_pub_->publish(std::move(odom_msg));
  accel_pub_->publish(std::move(accel_msg));

  debug_publisher_->trigger();
}
}  // namespace tam::road_geometry_coupling
