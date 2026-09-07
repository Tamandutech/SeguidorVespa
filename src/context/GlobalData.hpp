#pragma once

#include <vector>

#include "data_types.hpp"

/// Shared robot configuration and map. ControlTask appends points while
/// MAPPING and reads them while RUNNING; CLI edits when idle.
struct GlobalData {
  std::vector<MapPoint> mapData{};
  ParametersConfig      parametersConfig{};
};

inline GlobalData globalData{};
