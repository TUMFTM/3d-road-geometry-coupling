// Copyright 2025 Simon Sagmeister
#include "tum_road_geometry_coupling_cpp/track_data_io.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
namespace tam::road_geometry_coupling
{
namespace
{
std::vector<std::string> split_csv_line(std::string const & line)
{
  std::vector<std::string> tokens;
  std::string token;
  std::stringstream ss(line);
  while (std::getline(ss, token, ',')) {
    auto first = token.find_first_not_of(" \t\r\n");
    auto last = token.find_last_not_of(" \t\r\n");
    tokens.push_back(first == std::string::npos ? "" : token.substr(first, last - first + 1));
  }
  return tokens;
}
}  // namespace
TrackData load_track_data_from_csv(std::string const & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("load_track_data_from_csv: failed to open '" + path + "'");
  }

  std::string line;
  if (!std::getline(file, line)) {
    throw std::runtime_error("load_track_data_from_csv: file '" + path + "' is empty");
  }
  auto headers = split_csv_line(line);

  TrackData out;
  std::unordered_map<std::string, std::vector<double> *> field_map = {
    {"s_m", &out.s_m},
    {"ref_line_x_m", &out.ref_line_x_m},
    {"ref_line_y_m", &out.ref_line_y_m},
    {"ref_line_z_m", &out.ref_line_z_m},
    {"theta_rad", &out.theta_rad},
    {"mu_rad", &out.mu_rad},
    {"phi_rad", &out.phi_rad},
    {"omega_x_radpm", &out.omega_x_radpm},
    {"omega_y_radpm", &out.omega_y_radpm},
    {"omega_z_radpm", &out.omega_z_radpm},
    {"track_width_left_m", &out.track_width_left_m},
    {"track_width_right_m", &out.track_width_right_m},
  };

  std::vector<std::vector<double> *> column_targets(headers.size(), nullptr);
  for (size_t i = 0; i < headers.size(); ++i) {
    auto it = field_map.find(headers[i]);
    if (it != field_map.end()) {
      column_targets[i] = it->second;
    }
  }

  for (auto const & [name, target] : field_map) {
    bool found = std::any_of(
      column_targets.begin(), column_targets.end(),
      [t = target](auto const * c) { return c == t; });
    if (!found) {
      throw std::runtime_error(
        "load_track_data_from_csv: required column '" + name + "' not found in '" + path + "'");
    }
  }

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    auto tokens = split_csv_line(line);
    if (tokens.size() != headers.size()) {
      throw std::runtime_error(
        "load_track_data_from_csv: row in '" + path + "' has " + std::to_string(tokens.size()) +
        " columns, expected " + std::to_string(headers.size()));
    }
    for (size_t i = 0; i < tokens.size(); ++i) {
      if (column_targets[i] == nullptr) continue;
      column_targets[i]->push_back(std::stod(tokens[i]));
    }
  }

  return out;
}
}  // namespace tam::road_geometry_coupling
