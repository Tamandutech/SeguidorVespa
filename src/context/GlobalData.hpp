#ifndef GLOBAL_DATA_CONTEXT_HPP
#define GLOBAL_DATA_CONTEXT_HPP

#include <atomic>
#include <vector>

struct GlobalData {
  std::atomic<bool> isReadyToRun = false;

  std::atomic<int32_t> finishLineCount = 1000000;

  std::vector<int32_t> mapData;
} globalData;

#endif // GLOBAL_DATA_CONTEXT_HPP
