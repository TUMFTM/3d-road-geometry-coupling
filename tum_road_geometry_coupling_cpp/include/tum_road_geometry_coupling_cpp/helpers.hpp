// Copyright 2025 Simon Sagmeister
#pragma once
#include <cstdint>
namespace tam::road_geometry_coupling
{
inline std::size_t wrap_index(std::size_t const & index, std::size_t const & max_index)
{
  // Wrap the index to be within the range of the track size
  return index % max_index;
}
inline double wrap_index(double const & index, std::size_t const & max_index)
{
  // Wrap the index to be within the range of the track size
  auto index_ = index;
  while (index_ > max_index) {
    index_ -= max_index;
  }
  return index_;
}
}  // namespace tam::road_geometry_coupling
