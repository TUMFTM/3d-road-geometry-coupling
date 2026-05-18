# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.0] - Initial Release

### Added
- Core C++ library `tum_road_geometry_coupling_cpp` implementing the road-geometry-coupling algorithm:
  - Geometry transformation from the planar road plane onto the 3D track surface.
  - Measurement transformation lifting planar velocity, acceleration, and angular rates into 3D.
  - Vehicle load calculation producing the road-geometry-induced forces and moments.
- Python bindings `tum_road_geometry_coupling_py` exposing the public C++ API via pybind11, with a runnable example.
- ROS 2 nodes `tum_road_geometry_coupling_nodes_cpp`:
  - `RoadGeometryCouplerNode` &mdash; pure road-geometry coupling.
  - `ExternalInfluenceGeneratorNode` &mdash; inherits from the coupler, additionally applies external friction modifiers based on tire temperature and track grip subscriptions.
- Apache 2.0 license, top-level and per-package READMEs, and this changelog.
