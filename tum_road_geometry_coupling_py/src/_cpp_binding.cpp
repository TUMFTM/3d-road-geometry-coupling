// Copyright 2025 Simon Sagmeister
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "tum_road_geometry_coupling_cpp/track_data.hpp"
#include "tum_road_geometry_coupling_cpp/track_data_io.hpp"
#include "tum_road_geometry_coupling_cpp/road_geometry_coupler.hpp"
#include "tum_road_geometry_coupling_cpp/types.hpp"

namespace py = pybind11;
namespace tgi = tam::road_geometry_coupling;

PYBIND11_MODULE(_cpp_binding, m)
{
  // Plain data description of a reference line.
  py::class_<tgi::TrackData>(m, "TrackData")
    .def(py::init<>())
    .def_readwrite("s_m", &tgi::TrackData::s_m)
    .def_readwrite("ref_line_x_m", &tgi::TrackData::ref_line_x_m)
    .def_readwrite("ref_line_y_m", &tgi::TrackData::ref_line_y_m)
    .def_readwrite("ref_line_z_m", &tgi::TrackData::ref_line_z_m)
    .def_readwrite("theta_rad", &tgi::TrackData::theta_rad)
    .def_readwrite("mu_rad", &tgi::TrackData::mu_rad)
    .def_readwrite("phi_rad", &tgi::TrackData::phi_rad)
    .def_readwrite("omega_x_radpm", &tgi::TrackData::omega_x_radpm)
    .def_readwrite("omega_y_radpm", &tgi::TrackData::omega_y_radpm)
    .def_readwrite("omega_z_radpm", &tgi::TrackData::omega_z_radpm)
    .def_readwrite("track_width_left_m", &tgi::TrackData::track_width_left_m)
    .def_readwrite("track_width_right_m", &tgi::TrackData::track_width_right_m);

  m.def(
    "load_track_data_from_csv", &tgi::load_track_data_from_csv, py::arg("path"),
    "Load a TrackData instance from a CSV file. Header column names must match "
    "the TrackData fields (s_m, ref_line_x_m, ..., track_width_right_m).");

  // Auxiliary pose types.
  py::class_<tgi::CartesianPose2D>(m, "CartesianPose2D")
    .def(py::init<>())
    .def_readwrite("x_m", &tgi::CartesianPose2D::x_m)
    .def_readwrite("y_m", &tgi::CartesianPose2D::y_m)
    .def_readwrite("yaw_rad", &tgi::CartesianPose2D::yaw_rad);

  py::class_<tgi::CurvilinearPose>(m, "CurvilinearPose")
    .def(py::init<>())
    .def(
      py::init<double, double, double, double>(), py::arg("s"), py::arg("n"), py::arg("chi"),
      py::arg("idx"))
    .def_readwrite("s_m", &tgi::CurvilinearPose::s_m)
    .def_readwrite("n_m", &tgi::CurvilinearPose::n_m)
    .def_readwrite("chi_rad", &tgi::CurvilinearPose::chi_rad)
    .def_readwrite("idx", &tgi::CurvilinearPose::idx);

  py::class_<tgi::VehicleLoad>(m, "VehicleLoad")
    .def(py::init<>())
    .def_readwrite("force_N", &tgi::VehicleLoad::force_N)
    .def_readwrite("torque_Nm", &tgi::VehicleLoad::torque_Nm);

  // Main generator class.
  py::class_<
    tgi::RoadGeometryCoupler, tgi::RoadGeometryCoupler::SharedPtr>(
    m, "RoadGeometryCoupler")
    .def(
      py::init<tgi::TrackData, tgi::TrackData>(), py::arg("raceline_data"),
      py::arg("pitlane_data"))
    .def(py::init<tgi::TrackData>(), py::arg("track_data"))
    .def("step", &tgi::RoadGeometryCoupler::step)
    .def("set_odometry", &tgi::RoadGeometryCoupler::set_odometry)
    .def("set_acceleration", &tgi::RoadGeometryCoupler::set_acceleration)
    .def(
      "get_transformed_odometry", &tgi::RoadGeometryCoupler::get_transformed_odometry)
    .def(
      "get_transformed_acceleration",
      &tgi::RoadGeometryCoupler::get_transformed_acceleration)
    .def("get_vehicle_load", &tgi::RoadGeometryCoupler::get_vehicle_load)
    .def("get_logger", &tgi::RoadGeometryCoupler::get_logger)
    .def("get_param_manager", &tgi::RoadGeometryCoupler::get_param_manager);
}
