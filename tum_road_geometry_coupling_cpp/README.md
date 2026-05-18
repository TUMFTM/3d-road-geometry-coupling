# tum_road_geometry_coupling_cpp

Core C++ library implementing the road-geometry-coupling algorithm. Independent of ROS 2 &mdash; it can be used standalone or driven by the ROS 2 nodes in [`tum_road_geometry_coupling_nodes_cpp`](../tum_road_geometry_coupling_nodes_cpp/).

For the high-level overview and the methodology, please refer to the [top-level README](../README.md) and the [paper](../README.md#5-citation).

## API Surface

All types live in the namespace `tam::road_geometry_coupling`.

| Type | Header | Purpose |
| --- | --- | --- |
| `RoadGeometryCoupler` | `road_geometry_coupler.hpp` | Main user-facing class. Orchestrates the geometry transformation, measurement transformation, and vehicle load calculation per simulation step. |
| `TrackData` | `track_data.hpp` | Plain-data description of a reference line: arc length, reference line position, orientation, curvilinear angular rates, and track widths. |
| `CartesianPose2D`, `CurvilinearPose`, `VehicleLoad` | `types.hpp` | Lightweight POD types used at the public interface. |
| `RoadPlaneGeometryTransformer` | `road_plane_transformer.hpp` | Step 1: maps a planar pose onto the actual 3D track surface. |
| `MeasurementTransformer` | `measurement_transformer.hpp` | Step 2: lifts planar velocity, acceleration, and angular rates into 3D. |
| `VehicleLoadCalculator` | `vehicle_load_calculator.hpp` | Step 3: computes the road-geometry-induced forces and moments to feed back to the planar model. |
| `load_track_data_from_csv` | `track_data_io.hpp` | Free function to load a `TrackData` from a CSV file whose headers match the `TrackData` fields. |

## Minimal Example

```cpp
#include "tum_road_geometry_coupling_cpp/road_geometry_coupler.hpp"
#include "tum_road_geometry_coupling_cpp/track_data.hpp"
#include "tum_road_geometry_coupling_cpp/track_data_io.hpp"

namespace rgc = tam::road_geometry_coupling;

// Load one of the bundled tracks (or any CSV with headers matching TrackData),
// or populate TrackData by hand.
auto track_data = rgc::load_track_data_from_csv("example_tracks/flat_track.csv");
rgc::RoadGeometryCoupler coupler(std::move(track_data));

coupler.set_odometry(odom_road_plane);          // tam::types::control::Odometry
coupler.set_acceleration(accel_road_plane);     // tam::types::control::AccelerationwithCovariances
coupler.step();

auto global_odom = coupler.get_transformed_odometry();
auto global_accel = coupler.get_transformed_acceleration();
auto vehicle_load = coupler.get_vehicle_load();
```

## Dependencies

- `tum_types_cpp` &mdash; common odometry / acceleration types
- `tum_helpers_cpp` &mdash; curvilinear coordinate system helpers
- `tsl_logger_cpp` &mdash; debug signal logging
- `param_management_cpp` &mdash; runtime parameter access
- Eigen3
