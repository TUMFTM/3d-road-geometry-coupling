# tum_road_geometry_coupling_nodes_cpp

ROS 2 nodes wrapping the [`tum_road_geometry_coupling_cpp`](../tum_road_geometry_coupling_cpp/) library. Two nodes are provided:

- **`RoadGeometryCouplerNode`** &mdash; pure road-geometry coupling.
- **`ExternalInfluenceGeneratorNode`** &mdash; inherits from the coupler and additionally applies external friction modifiers (e.g. from tire temperature or a global track grip scale).

Both nodes share the same publish / subscribe interface; the derived node only adds two extra subscribers and overrides how the per-wheel friction modifiers are computed.

For the high-level overview and the methodology, please refer to the [top-level README](../README.md) and the [paper](../README.md#5-citation).

## Topic Interface

| Direction | Topic | Type |
| --- | --- | --- |
| sub | `/simulation/road_plane/odometry` | `nav_msgs/Odometry` |
| sub | `/simulation/road_plane/acceleration` | `geometry_msgs/AccelWithCovarianceStamped` |
| pub | `/simulation/odometry` | `nav_msgs/Odometry` |
| pub | `/simulation/acceleration` | `geometry_msgs/AccelWithCovarianceStamped` |
| pub | `/simulation/road_plane/external_influences` | `tum_msgs/TUMExternalVehicleInfluences` |

The two synchronized inputs (odometry, acceleration) drive a single simulation step. The published `external_influences` message carries the road-geometry-induced force and torque on the vehicle's center of gravity, plus per-wheel `lambda_mue` friction modifiers (always `1.0` for `RoadGeometryCouplerNode`).

### Additional subscriptions for `ExternalInfluenceGeneratorNode`

Enabled via ROS parameters `enable_tire_temperature_grip_scaling` and `enable_track_grip_scale`:

| Direction | Topic | Type |
| --- | --- | --- |
| sub | `/simulation/tire_temp_grip_scale` | `tum_msgs/TUMFloat64PerWheelStamped` |
| sub | `/simulation/track_grip_scale` | `tum_msgs/TUMFloat32Stamped` |

The published `lambda_mue` is the per-wheel product of the two scales.

## Run

As a standalone executable (loads tracks via `track_handler_cpp`):

```bash
ros2 run tum_road_geometry_coupling_nodes_cpp road_geometry_coupler_node
ros2 run tum_road_geometry_coupling_nodes_cpp external_influence_generator_node
```

Or as a composable component inside a `component_container`:

```bash
ros2 component types tum_road_geometry_coupling_nodes_cpp
# RoadGeometryCouplerComponent
# ExternalInfluenceGeneratorComponent
```

For demos and quick experiments, a CSV-based variant of the external-influence node is also provided. It constructs the node directly (bypassing `rclcpp_components` and `track_handler_cpp`) and loads its track from a `TrackData` CSV:

```bash
ros2 run tum_road_geometry_coupling_nodes_cpp external_influence_generator_node_from_csv \
  --ros-args -p track_csv_path:=/path/to/example_tracks/flat_track.csv
```

This is the executable used by the [`tum_road_plane_follower_cpp` demo launch](../tum_road_plane_follower_cpp/launch/demo.launch.py).

## Constructing from Code

For users that want to drive the node directly from explicit `TrackData` (without going through `track_handler_cpp`), both node classes expose constructors that take a `TrackData` (or a raceline + pitlane pair) plus the ROS `NodeOptions`. The `*Component` wrappers used by the rclcpp_components system handle the track-loading via `track_handler_cpp` for you.

## Dependencies

- `tum_road_geometry_coupling_cpp` &mdash; the underlying library
- `rclcpp`, `rclcpp_components`, `nav_msgs`, `geometry_msgs`, `message_filters`
- `tum_msgs`, `tum_types_cpp`, `tum_type_conversions_ros_cpp`
- `tsl_ros2_publisher_cpp`, `param_management_ros2_integration_cpp`, `tum_ros_helpers_cpp`
- `track_handler_cpp` *(component wrappers only &mdash; the node library itself stays free of this dependency)*
