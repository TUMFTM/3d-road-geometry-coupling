# tum_road_plane_follower_cpp

Demo ROS 2 node that drives a virtual vehicle along the road-plane reference
of a 3D track at a constant longitudinal speed and publishes the resulting
**road-plane** odometry and acceleration on the same topics that
[`tum_road_geometry_coupling_nodes_cpp`](../tum_road_geometry_coupling_nodes_cpp/)
expects as input. This is the lightest possible end-to-end demo: connect this
node to a `RoadGeometryCouplerNode` and you get the lifted 3D state and the
road-geometry-induced vehicle load on the output topics.

The node loads its track directly from a `TrackData` CSV (see the bundled
[example tracks](../example_tracks/)) and constructs the road-plane reference
internally via `RoadPlaneGeometryTransformer` &mdash; no external track-handler
configuration required.

## Topic Interface

| Direction | Topic | Type |
| --- | --- | --- |
| pub | `/simulation/road_plane/odometry` | `nav_msgs/Odometry` |
| pub | `/simulation/road_plane/acceleration` | `geometry_msgs/AccelWithCovarianceStamped` |

## Parameters

| Name | Default | Description |
| --- | --- | --- |
| `track_csv_path` | *(required)* | Path to a TrackData CSV (see [`example_tracks/`](../example_tracks/)). |
| `velocity_mps` | `10 * sqrt(2)` (\~14.14&nbsp;m/s) | Constant longitudinal speed of the virtual vehicle [m/s]. |
| `export_road_plane_csv` | `false` | If `true`, dump the constructed road-plane reference to CSV at startup. |
| `export_path` | `./road_plane_ref_<YYYYMMDD_HHMMSS>.csv` | Destination for the exported road-plane reference CSV. Defaults to a timestamped file in the working directory. |

## Run

### Full end-to-end demo

After sourcing the install workspace, run the demo launch file. It starts the
follower, the external-influence-generator node, and a `ros2 bag record -a`
that captures every topic &mdash; no `track_handler_cpp` configuration required.

```bash
ros2 launch tum_road_plane_follower_cpp demo.launch.py
# optional overrides:
#   track_csv_path:=/path/to/example_tracks/elevated_track.csv
#   velocity_mps:=40.0
#   bag_output_dir:=/tmp/my_demo_bag
```

By default this drives around `fully_banked_track.csv` at \~14.14&nbsp;m/s and
records the bag to `./road_geometry_coupling_demo_<YYYYMMDD_HHMMSS>/` in the
working directory the launch was invoked from.

### Follower in isolation

If you only want the road-plane publisher (e.g. to feed a custom downstream node):

```bash
ros2 launch tum_road_plane_follower_cpp launch.py
```

By default this drives around `fully_banked_track.csv` at \~14.14&nbsp;m/s.
Override the track or speed via launch arguments:

```bash
ros2 launch tum_road_plane_follower_cpp launch.py \
  track_csv_path:=/path/to/example_tracks/elevated_track.csv \
  velocity_mps:=40.0 \
  export_road_plane_csv:=true
```

Or run the executable directly with explicit parameters:

```bash
ros2 run tum_road_plane_follower_cpp road_plane_follower_node --ros-args \
  -p track_csv_path:=/path/to/example_tracks/elevated_track.csv \
  -p velocity_mps:=30.0
```

## Dependencies

- `tum_road_geometry_coupling_cpp` &mdash; track type, CSV loader, and road-plane builder
- `tum_helpers_cpp`, `tum_ros_helpers_cpp`, `tum_type_conversions_ros_cpp`, `tum_types_cpp`
- `rclcpp`, `nav_msgs`, `geometry_msgs`
