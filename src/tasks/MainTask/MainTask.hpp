#ifndef MAIN_TASK_HPP
#define MAIN_TASK_HPP

#include "context/GlobalData.hpp"
#include "context/RobotStateMachine.hpp"

#include "drivers/EncoderDriver/EncoderDriver.hpp"
#include "drivers/IRSensorDriver/IRSensorDriver.hpp"
#include "drivers/MotorDriver/MotorDriver.hpp"
#include "drivers/VacuumDriver/VacuumDriver.hpp"

#include "tasks/MainTask/PathController/PathController.hpp"

struct MainTaskParamSchema {
  GlobalData &globalData;
};

void mainTaskLoop(void *params) {
  MainTaskParamSchema *param = static_cast<MainTaskParamSchema *>(params);
  RobotStateMachine::toCalibration();

  int32_t finishLineCount =
      globalData.finishLineCount.load(std::memory_order_relaxed);

  // uint16_t rawSensorValues[16];
  uint16_t sideSensorValues[4];
  uint16_t lineSensorValues[12];

  // Initialize pins and drivers in globalData during calibration mode
  if(globalData.motorDriver == nullptr) {
    globalData.motorPins   = {.gpioDirectionA = RobotEnv::GPIO_DIRECTION_A,
                              .gpioDirectionB = RobotEnv::GPIO_DIRECTION_B,
                              .gpioPWMA       = RobotEnv::GPIO_PWM_A,
                              .gpioPWMB       = RobotEnv::GPIO_PWM_B};
    globalData.motorDriver = new MotorDriver(globalData.motorPins);
  }
  if(globalData.irSensorDriver == nullptr) {
    IRSensorParamSchema irSensorParam = {
        .pins             = {.gpioMultiplexerDigitalAddress =
                                 RobotEnv::GPIO_MULTIPLEXER_DIGITAL_ADDRESS,
                             .gpioMultiplexerAnalogInput =
                                 RobotEnv::GPIO_MULTIPLEXER_ANALOG_INPUT},
        .lineSensorsCount = 12,
        .lineSensorsMultiplexerIndex =
            RobotEnv::GPIO_MULTIPLEXER_LINE_SENSORS_INDEX,
        .sideSensorsCount = 4,
        .sideSensorsMultiplexerIndex =
            RobotEnv::GPIO_MULTIPLEXER_SIDE_SENSORS_INDEX,
        .multiplexerPinCount = 4,
    };
    globalData.irSensorDriver = new IRSensorDriver(irSensorParam);
  }
  if(globalData.encoderLeftDriver == nullptr) {
    globalData.encoderLeftDriver = new EncoderDriver();
    globalData.encoderLeftDriver->attachFullQuad(RobotEnv::GPIO_ENCODER_LEFT_A,
                                                 RobotEnv::GPIO_ENCODER_LEFT_B);
  }
  if(globalData.encoderRightDriver == nullptr) {
    globalData.encoderRightDriver = new EncoderDriver();
    globalData.encoderRightDriver->attachFullQuad(
        RobotEnv::GPIO_ENCODER_RIGHT_A, RobotEnv::GPIO_ENCODER_RIGHT_B);
  }
  if(globalData.vacuumDriver == nullptr) {
    globalData.vacuumPins   = {.gpioPWM = RobotEnv::GPIO_PWM_VACUUM};
    globalData.vacuumDriver = new VacuumDriver(globalData.vacuumPins);
  }

  PathControllerParamSchema pathControllerParam = {
      .constants      = {.kP = 0.016F, .kI = 0.00F, .kD = 0.07F},
      .sensorQuantity = 12,
      .sensorValues   = lineSensorValues,
      .maxAngle       = 45.0F, // Ângulo máximo de 45 graus
      .radiusSensor   = 100, // Raio dos sensores em mm
      .sensorToCenter = 50, // Distância do sensor ao centro em mm
  };
  PathController *pathController = new PathController(pathControllerParam);

  RobotState lastRobotState = RobotState::IDLE;

  bool lastLeftReadIsOnMark  = false;
  bool lastRightReadIsOnMark = false;

  int32_t  sideSensorReadCount  = 0;
  uint32_t sideSensorAverage[4] = {0, 0, 0, 0};

  uint32_t mapPointIndex = 0;

  ESP_LOGI("MainTask", "Calibrando os sensores...");
  for(int i = 0; i < 50; i++) {
    globalData.irSensorDriver->calibrate();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  ESP_LOGI("MainTask", "Sensores calibrados");

  vTaskDelay(3000 / portTICK_PERIOD_MS);
  RobotStateMachine::toIdle(globalData.motorDriver, globalData.vacuumDriver);
  for(;;) {
    int32_t encoderMilimetersAverage =
        ((globalData.encoderLeftDriver->getCount() +
          globalData.encoderRightDriver->getCount()) /
         2) *
        RobotEnv::WHEEL_CIRCUMFERENCE / 4095;

    // Condição de parada
    if(encoderMilimetersAverage > finishLineCount) {
      RobotStateMachine::toIdle(globalData.motorDriver,
                                globalData.vacuumDriver);
      lastRobotState = RobotState::IDLE;
      continue;
    }

    globalData.irSensorDriver->readCalibrated(lineSensorValues,
                                              sideSensorValues);

    sideSensorReadCount++;
    for(int i = 0; i < 4; i++) {
      sideSensorAverage[i] += sideSensorValues[i];
    }

    if(sideSensorReadCount >= RobotEnv::SIDE_SENSOR_READ_AVERAGE_COUNT) {
      for(int i = 0; i < 4; i++) {
        sideSensorAverage[i] /= RobotEnv::SIDE_SENSOR_READ_AVERAGE_COUNT;
      }
      sideSensorReadCount = 0;

      bool leftIsOnMark =
          sideSensorAverage[0] < 200 || sideSensorAverage[1] < 200;
      bool rightIsOnMark =
          sideSensorAverage[2] < 200 || sideSensorAverage[3] < 200;
      if(!leftIsOnMark && !rightIsOnMark) {
        lastLeftReadIsOnMark  = false;
        lastRightReadIsOnMark = false;
      } else if(!leftIsOnMark && rightIsOnMark) {
        if(!lastRightReadIsOnMark) {
          lastLeftReadIsOnMark  = false;
          lastRightReadIsOnMark = true;
        }
      } else if(leftIsOnMark && !rightIsOnMark) {
        if(!lastLeftReadIsOnMark) {
          globalData.markCount.store(
              globalData.markCount.load(std::memory_order_relaxed) + 1,
              std::memory_order_relaxed);

          lastLeftReadIsOnMark  = true;
          lastRightReadIsOnMark = false;
        }
      } else {
        lastLeftReadIsOnMark  = true;
        lastRightReadIsOnMark = true;
      }

      for(int i = 0; i < 4; i++) {
        sideSensorAverage[i] = 0;
      }
    }

    if(globalData.mapData[mapPointIndex].encoderMilimeters >
           encoderMilimetersAverage &&
       (mapPointIndex + 1) < globalData.mapData.size()) {
      mapPointIndex++;
    }

    float pathPID = pathController->getPID();

    globalData.motorDriver->pwmOutput(
        globalData.mapData[mapPointIndex].baseMotorPWM + pathPID,
        globalData.mapData[mapPointIndex].baseMotorPWM - pathPID);

    globalData.vacuumDriver->pwmOutput(RobotEnv::BASE_VACUUM_PWM);

    // printf("\033[2J\033[H");
    // irSensorDriver->read(rawSensorValues);
    // for(int i = 0; i < 16; i++) {
    //   printf("%4d ", rawSensorValues[i]);
    // }
    // printf("L: ");
    // for(int i = 0; i < 12; i++) {
    //   printf("%4d ", lineSensorValues[i]);
    // }
    // printf("S: ");
    // for(int i = 0; i < 4; i++) {
    //   printf("%4d ", sideSensorValues[i]);
    // }
    // printf("\n");
    // printf("Base Motor PWM: %ld\n",
    //        globalData.mapData[mapPointIndex].baseMotorPWM);
    // printf("Path PID: %f\n", pathPID);
    // printf("Mark Count: %ld\n",
    //        globalData.markCount.load(std::memory_order_relaxed));
    // printf("Encoder Left: %ld\n", encoderLeftDriver->getCount());
    // printf("Encoder Right: %ld\n", encoderRightDriver->getCount());

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

#endif // MAIN_TASK_HPP
