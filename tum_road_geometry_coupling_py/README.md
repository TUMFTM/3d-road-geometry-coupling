# tum_road_geometry_coupling_py

Python bindings around [`tum_road_geometry_coupling_cpp`](../tum_road_geometry_coupling_cpp/), built with [pybind11](https://pybind11.readthedocs.io/).

For the high-level overview and the methodology, please refer to the [top-level README](../README.md) and the [paper](../README.md#5-citation).

## What's Exposed

The package re-exports the public types of the C++ library:

- `RoadGeometryCoupler`
- `TrackData`
- `CartesianPose2D`, `CurvilinearPose`, `VehicleLoad`
- `load_track_data_from_csv(path)` &mdash; free function to load a `TrackData` from a CSV file whose headers match the `TrackData` fields.

`Odometry` and `AccelerationwithCovariances` are passed across the boundary using the bindings provided by [`tum_types_py`](https://github.com/TUMFTM/TAM__tum_common_cpp).

## Minimal Example

A complete runnable example is provided in [examples/basic_usage.py](./examples/basic_usage.py):

```python
import tum_road_geometry_coupling_py as rgc
from tum_types_py.control import Odometry, AccelerationwithCovariances

# Load one of the bundled example tracks (or your own CSV with matching headers).
track = rgc.load_track_data_from_csv("example_tracks/flat_track.csv")

coupler = rgc.RoadGeometryCoupler(track)
coupler.set_odometry(Odometry())
coupler.set_acceleration(AccelerationwithCovariances())
coupler.step()

global_odom = coupler.get_transformed_odometry()
global_accel = coupler.get_transformed_acceleration()
load = coupler.get_vehicle_load()
```

Run it after sourcing the install workspace:

```bash
python3 -m tum_road_geometry_coupling_py.examples.basic_usage path/to/track.csv
```

(or directly via the file path).

## Dependencies

- `tum_road_geometry_coupling_cpp` &mdash; the underlying C++ library
- `tum_types_py` &mdash; Python bindings for the shared C++ types
- `tsl_logger_py`, `param_management_py` &mdash; bindings for logging and parameter management
- `pybind11`
