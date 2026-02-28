#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "context/GlobalData.hpp"
#include "context/RobotEnv.hpp"
#include "drivers/EncoderDriver/EncoderDriver.hpp"
#include "drivers/MotorDriver/MotorDriver.hpp"
#include "drivers/VacuumDriver/VacuumDriver.hpp"

enum class RobotState {
  CALIBRATION,
  IDLE,
  RUNNING,
};

class RobotStateMachine {
public:
  static RobotState get() { return state_.load(std::memory_order_relaxed); }

  /** Register the MainTask handle so it can be suspended/resumed on idle/run. */
  static void setMainTaskHandle(TaskHandle_t handle) { mainTaskHandle_ = handle; }

  static void toCalibration() { setState(RobotState::CALIBRATION); }

  static void toIdle(MotorDriver *motorDriver, VacuumDriver *vacuumDriver) {
    if(mainTaskHandle_ != nullptr) {
      vTaskSuspend(mainTaskHandle_);
    }

    RobotState last = get();
    if(last == RobotState::IDLE || last == RobotState::CALIBRATION) {
      return;
    }
    if(motorDriver == nullptr || vacuumDriver == nullptr) {
      setState(RobotState::IDLE);
      return;
    }

    setState(RobotState::IDLE);
    motorDriver->pwmOutput(0, 0);
    vacuumDriver->pwmOutput(0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  static void toRunning(EncoderDriver *encoderLeftDriver,
                        EncoderDriver *encoderRightDriver,
                        VacuumDriver  *vacuumDriver) {
    RobotState last = get();
    if(last == RobotState::RUNNING || last == RobotState::CALIBRATION) {
      return;
    }
    if(encoderLeftDriver == nullptr || encoderRightDriver == nullptr ||
       vacuumDriver == nullptr) {
      setState(RobotState::RUNNING);
      return;
    }

    setState(RobotState::RUNNING);
    encoderLeftDriver->clearCount();
    encoderRightDriver->clearCount();

    // To print the finish line count
    // finishLineCount =
    //     globalData.finishLineCount.load(std::memory_order_relaxed);

    vTaskDelay(4000 / portTICK_PERIOD_MS);
    vacuumDriver->pwmOutput(RobotEnv::BASE_VACUUM_PWM);
    vTaskDelay(1000);

    if(mainTaskHandle_ != nullptr) {
      vTaskResume(mainTaskHandle_);
    }
  }

private:
  static void setState(RobotState s) {
    state_.store(s, std::memory_order_relaxed);
  }

  static inline std::atomic<RobotState> state_{RobotState::IDLE};
  static inline TaskHandle_t mainTaskHandle_ = nullptr;
};

#endif // ROBOT_STATE_HPP
