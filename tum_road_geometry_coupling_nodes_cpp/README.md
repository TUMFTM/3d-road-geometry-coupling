# tum_road_geometry_coupling_nodes_cpp

ROS 2 node wrapping the [`tum_road_geometry_coupling_cpp`](../tum_road_geometry_coupling_cpp/) library:

- **`RoadGeometryCouplerNode`** &mdash; couples the planar road-plane vehicle state with the 3D track and publishes the road-geometry-induced vehicle load as a `geometry_msgs/WrenchStamped`.

The wrench is one source consumed by the external-influence aggregator (`ocd_external_influence_aggregator_cpp`), which combines it with any other force, grip and road-height sources into the message the vehicle model node consumes. Grip effects (e.g. tire temperature) are therefore no longer applied inside this node &mdash; they are published as separate grip sources and combined downstream.

For the high-level overview and the methodology, please refer to the [top-level README](../README.md) and the [paper](../README.md#5-citation).

## Topic Interface

| Direction | Topic | Type |
| --- | --- | --- |
| sub | `/simulation/road_plane/odometry` | `nav_msgs/Odometry` |
| sub | `/simulation/road_plane/acceleration` | `geometry_msgs/AccelWithCovarianceStamped` |
| pub | `/simulation/odometry` | `nav_msgs/Odometry` |
| pub | `/simulation/acceleration` | `geometry_msgs/AccelWithCovarianceStamped` |
| pub | `/simulation/external_influences/wrench/road_geometry` | `geometry_msgs/WrenchStamped` |

The two synchronized inputs (odometry, acceleration) drive a single simulation step. The published wrench carries the road-geometry-induced force and torque on the vehicle's center of gravity, expressed in the `vehicle_cg_footprint` frame.

## Run

As a standalone executable (loads tracks via `track_handler_cpp`):

```bash
ros2 run tum_road_geometry_coupling_nodes_cpp road_geometry_coupler_node
```

Or as a composable component inside a `component_container`:

```bash
ros2 component types tum_road_geometry_coupling_nodes_cpp
# RoadGeometryCouplerComponent
```

For demos and quick experiments, a CSV-based variant of the node is also provided. It constructs the node directly (bypassing `rclcpp_components` and `track_handler_cpp`) and loads its track from a `TrackData` CSV:

```bash
ros2 run tum_road_geometry_coupling_nodes_cpp road_geometry_coupler_node_from_csv \
  --ros-args -p track_csv_path:=/path/to/example_tracks/flat_track.csv
```

This is the executable used by the [`tum_road_plane_follower_cpp` demo launch](../tum_road_plane_follower_cpp/launch/demo.launch.py).

## Constructing from Code

For users that want to drive the node directly from explicit `TrackData` (without going through `track_handler_cpp`), the node class exposes constructors that take a `TrackData` (or a raceline + pitlane pair) plus the ROS `NodeOptions`. The `RoadGeometryCouplerComponent` wrapper used by the rclcpp_components system handles the track-loading via `track_handler_cpp` for you.

## Dependencies

- `tum_road_geometry_coupling_cpp` &mdash; the underlying library
- `rclcpp`, `rclcpp_components`, `nav_msgs`, `geometry_msgs`, `message_filters`
- `tum_msgs`, `tum_types_cpp`, `tum_type_conversions_ros_cpp`
- `tsl_ros2_publisher_cpp`, `param_management_ros2_integration_cpp`, `tum_ros_helpers_cpp`
- `track_handler_cpp` *(component wrappers only &mdash; the node library itself stays free of this dependency)*
