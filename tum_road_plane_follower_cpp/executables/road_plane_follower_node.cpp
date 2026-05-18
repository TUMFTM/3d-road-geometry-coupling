// Copyright 2025 Simon Sagmeister
#include <rclcpp/rclcpp.hpp>

#include "tum_road_plane_follower_cpp/road_plane_follower.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node =
    std::make_shared<tam::road_geometry_coupling::demo::RoadPlaneFollower>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
