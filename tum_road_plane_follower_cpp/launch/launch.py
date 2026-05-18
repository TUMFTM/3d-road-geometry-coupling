# Copyright 2025 Simon Sagmeister
"""Demo launch: drive a virtual vehicle around one of the bundled example
tracks at constant speed and publish the resulting road-plane odometry /
acceleration. Pair with the `road_geometry_coupler_node` (or any other
consumer of `/simulation/road_plane/*`) to observe the 3D coupling.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

EXAMPLE_TRACKS_DIR = (
    Path(get_package_share_directory("tum_road_plane_follower_cpp")) / "example_tracks"
)


def generate_launch_description():
    track_csv_path = LaunchConfiguration("track_csv_path")
    velocity_mps = LaunchConfiguration("velocity_mps")
    export_road_plane_csv = LaunchConfiguration("export_road_plane_csv")
    export_path = LaunchConfiguration("export_path")

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
                "export_road_plane_csv",
                default_value="false",
                description="If true, dump the constructed road-plane reference to CSV.",
            ),
            DeclareLaunchArgument(
                "export_path",
                default_value="/tmp/road_plane_ref.csv",
                description="Destination for the exported road-plane reference CSV.",
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
                        "export_road_plane_csv": export_road_plane_csv,
                        "export_path": export_path,
                    }
                ],
            ),
        ]
    )
