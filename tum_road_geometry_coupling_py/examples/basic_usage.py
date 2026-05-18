# Copyright 2025 Simon Sagmeister
"""Minimal example exercising the road geometry coupler from Python.

Loads a track CSV (one of the bundled tracks under ``example_tracks/`` or
your own file with headers matching the ``TrackData`` fields) and feeds a
single odometry / acceleration sample through the coupler.

Usage:
    python3 -m tum_road_geometry_coupling_py.examples.basic_usage <track.csv>
"""
import sys

from tum_types_py.control import Odometry, AccelerationwithCovariances

from tum_road_geometry_coupling_py import (
    RoadGeometryCoupler,
    load_track_data_from_csv,
)


def main() -> None:
    if len(sys.argv) != 2:
        print(
            "Usage: python3 -m tum_road_geometry_coupling_py.examples.basic_usage "
            "<path/to/track.csv>",
            file=sys.stderr,
        )
        sys.exit(1)

    track = load_track_data_from_csv(sys.argv[1])
    coupler = RoadGeometryCoupler(track)

    odometry = Odometry()
    odometry.position_m.x = track.ref_line_x_m[0]
    odometry.position_m.y = track.ref_line_y_m[0]
    odometry.velocity_mps.x = 30.0
    acceleration = AccelerationwithCovariances()

    coupler.set_odometry(odometry)
    coupler.set_acceleration(acceleration)
    coupler.step()

    transformed = coupler.get_transformed_odometry()
    load = coupler.get_vehicle_load()
    print(f"Transformed position: ({transformed.position_m.x:.2f}, "
          f"{transformed.position_m.y:.2f}, {transformed.position_m.z:.2f})")
    print(f"Vehicle load force_N:  ({load.force_N.x:.2f}, "
          f"{load.force_N.y:.2f}, {load.force_N.z:.2f})")


if __name__ == "__main__":
    main()
