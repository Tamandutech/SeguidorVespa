#ifndef GLOBAL_DATA_CONTEXT_HPP
#define GLOBAL_DATA_CONTEXT_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <atomic>
#include <cstring>
#include <vector>

// Message types for inter-task communication
enum class MessageType { LOG };

#define MESSAGE_LOG_NAME_SIZE    32
#define MESSAGE_LOG_MESSAGE_SIZE 256

// Message structure for queue
struct Message {
  struct {
    MessageType type;
  } header;
  struct {
    char name[MESSAGE_LOG_NAME_SIZE];
    char message[MESSAGE_LOG_MESSAGE_SIZE];
  } data;
};

struct MapPoint {
  int32_t encoderMilimeters;
  int32_t baseMotorPWM;
};

struct GlobalData {
  std::atomic<bool> isReadyToRun = false;

  std::atomic<int32_t> finishLineCount = 14900;

  std::vector<MapPoint> mapData = {
      {.encoderMilimeters = 0, .baseMotorPWM = 10},
      // {.encoderMilimeters = 28000, .baseMotorPWM = 40},
      // {.encoderMilimeters = 28800, .baseMotorPWM = 35},
  };

  std::atomic<int32_t> markCount = 0;

  // FreeRTOS queue for inter-task communication
  QueueHandle_t communicationQueue;
} globalData;

#endif // GLOBAL_DATA_CONTEXT_HPP
