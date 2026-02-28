#ifndef GLOBAL_DATA_CONTEXT_HPP
#define GLOBAL_DATA_CONTEXT_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <atomic>
#include <cstring>
#include <vector>

#include "drivers/EncoderDriver/EncoderDriver.hpp"
#include "drivers/IRSensorDriver/IRSensorDriver.hpp"
#include "drivers/MotorDriver/MotorDriver.hpp"
#include "drivers/VacuumDriver/VacuumDriver.hpp"

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
  // FreeRTOS queue for inter-task communication
  QueueHandle_t communicationQueue;

  /* Communication should only write on the variables below when the robot is in
   * IDLE mode */

  std::atomic<int32_t> finishLineCount = 14900;

  std::vector<MapPoint> mapData = {
      {.encoderMilimeters = 0, .baseMotorPWM = 10},
      // {.encoderMilimeters = 28000, .baseMotorPWM = 40},
      // {.encoderMilimeters = 28800, .baseMotorPWM = 35},
  };

  std::atomic<int32_t> markCount = 0;

  // Pins and drivers (initialized in MainTask during calibration mode)
  MotorPins    motorPins   = {};
  MotorDriver *motorDriver = nullptr;

  IRSensorDriver *irSensorDriver = nullptr;

  EncoderDriver *encoderLeftDriver  = nullptr;
  EncoderDriver *encoderRightDriver = nullptr;

  VacuumPins    vacuumPins   = {};
  VacuumDriver *vacuumDriver = nullptr;

} static globalData;

#endif // GLOBAL_DATA_CONTEXT_HPP
