#ifndef GLOBAL_DATA_CONTEXT_HPP
#define GLOBAL_DATA_CONTEXT_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <atomic>
#include <vector>

// Message types for inter-task communication
enum class MessageType { SENSOR_DATA, STATUS_UPDATE, COMMAND, EMERGENCY_STOP };

// Message structure for queue
struct Message {
  MessageType type;
  union {
    struct {
      int32_t  sensorValue;
      uint32_t timestamp;
    } sensorData;
    struct {
      const char *status;
      int32_t     value;
    } statusUpdate;
    struct {
      const char *command;
      int32_t     parameter;
    } command;
  } data;
};

struct MapPoint {
  int32_t encoderMilimeters;
  int32_t baseMotorPWM;
};

struct GlobalData {
  std::atomic<bool> isReadyToRun = false;

  std::atomic<int32_t> finishLineCount = 1600000;

  std::vector<MapPoint> mapData = {
      {.encoderMilimeters = 0, .baseMotorPWM = 26},
      // {.encoderMilimeters = 1000, .baseMotorPWM = 10},
  };

  std::atomic<int32_t> markCount = 0;

  // FreeRTOS queue for inter-task communication
  QueueHandle_t communicationQueue;
} globalData;

#endif // GLOBAL_DATA_CONTEXT_HPP
