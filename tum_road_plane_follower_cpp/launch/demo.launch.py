# Copyright 2025 Simon Sagmeister
"""Full end-to-end demo:
- `road_plane_follower_node` drives a virtual vehicle around the chosen
  example track and publishes road-plane odometry / acceleration.
- `road_geometry_coupler_node_from_csv` constructs `RoadGeometryCouplerNode`
  directly (no rclcpp_components / no track_handler_cpp configuration),
  consumes the road-plane state, lifts it into 3D, and publishes the
  road-geometry-induced vehicle load as a `geometry_msgs/WrenchStamped`.
- `ros2 bag record -a` records every active topic to a timestamped bag.

Run: ros2 launch tum_road_plane_follower_cpp demo.launch.py
     [ track_csv_path:=/path/to/track.csv ]
     [ velocity_mps:=30.0 ]
     [ bag_output_dir:=/tmp/road_geometry_coupling_demo ]
"""
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


EXAMPLE_TRACKS_DIR = (
    Path(get_package_share_directory("tum_road_plane_follower_cpp")) / "example_tracks"
)


def generate_launch_description():
    track_csv_path = LaunchConfiguration("track_csv_path")
    velocity_mps = LaunchConfiguration("velocity_mps")
    bag_output_dir = LaunchConfiguration("bag_output_dir")

    default_bag_dir = str(
        Path.cwd() / f"road_geometry_coupling_demo_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "track_csv_path",
                default_value=str(EXAMPLE_TRACKS_DIR / "fully_banked_track.csv"),
                description="Path to a TrackData CSV (see example_tracks/).",
            ),
            DeclareLaunchArgument(
                "velocity_mps",
                default_value=str(10.0 * 2.0**0.5),
                description="Constant longitudinal speed of the virtual vehicle [m/s].",
            ),
            DeclareLaunchArgument(
                "bag_output_dir",
                default_value=default_bag_dir,
                description="Directory where `ros2 bag record -a` writes the bag.",
            ),
            Node(
                package="tum_road_plane_follower_cpp",
                executable="road_plane_follower_node",
                name="road_plane_follower",
                output="screen",
                parameters=[
                    {
                        "track_csv_path": track_csv_path,
                        "velocity_mps": velocity_mps,
                    }
                ],
            ),
            Node(
                package="tum_road_geometry_coupling_nodes_cpp",
                executable="road_geometry_coupler_node_from_csv",
                name="road_geometry_coupler",
                output="screen",
                parameters=[
                    {
                        "track_csv_path": track_csv_path,
                    }
                ],
            ),
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-a", "-o", bag_output_dir],
                output="screen",
            ),
        ]
    )
