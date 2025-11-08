#ifndef MAIN_TASK_HPP
#define MAIN_TASK_HPP

#include "context/GlobalData.hpp"

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

  int32_t finishLineCount =
      param->globalData.finishLineCount.load(std::memory_order_relaxed);

  uint16_t rawSensorValues[16];
  uint16_t sideSensorValues[4];
  uint16_t lineSensorValues[12];

  MotorPins    motorPins   = {.gpioDirectionA = RobotEnv::GPIO_DIRECTION_A,
                              .gpioDirectionB = RobotEnv::GPIO_DIRECTION_B,
                              .gpioPWMA       = RobotEnv::GPIO_PWM_A,
                              .gpioPWMB       = RobotEnv::GPIO_PWM_B};
  MotorDriver *motorDriver = new MotorDriver(motorPins);

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
  IRSensorDriver *irSensorDriver = new IRSensorDriver(irSensorParam);

  EncoderDriver *encoderLeftDriver  = new EncoderDriver();
  EncoderDriver *encoderRightDriver = new EncoderDriver();

  encoderLeftDriver->attachFullQuad(RobotEnv::GPIO_ENCODER_LEFT_A,
                                    RobotEnv::GPIO_ENCODER_LEFT_B);
  encoderRightDriver->attachFullQuad(RobotEnv::GPIO_ENCODER_RIGHT_A,
                                     RobotEnv::GPIO_ENCODER_RIGHT_B);

  VacuumPins    vacuumPins   = {.gpioPWM = RobotEnv::GPIO_PWM_VACUUM};
  VacuumDriver *vacuumDriver = new VacuumDriver(vacuumPins);

  PathControllerParamSchema pathControllerParam = {
      .constants      = {.kP = 0.0135F, .kI = 0.00F, .kD = 0.07F},
      .sensorQuantity = 12,
      .sensorValues   = lineSensorValues,
      .maxAngle       = 45.0F, // Ângulo máximo de 45 graus
      .radiusSensor   = 100, // Raio dos sensores em mm
      .sensorToCenter = 50, // Distância do sensor ao centro em mm
  };
  PathController *pathController = new PathController(pathControllerParam);

  bool lastIsReadyToRun = false;

  bool lastLeftReadIsOnMark  = false;
  bool lastRightReadIsOnMark = false;

  int32_t  sideSensorReadCount  = 0;
  uint32_t sideSensorAverage[4] = {0, 0, 0, 0};

  uint32_t mapPointIndex = 0;

  ESP_LOGI("MainTask", "Calibrando os sensores...");
  for(int i = 0; i < 50; i++) {
    irSensorDriver->calibrate();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  ESP_LOGI("MainTask", "Sensores calibrados");

  vTaskDelay(5000 / portTICK_PERIOD_MS);

  for(;;) {
    int32_t encoderMilimetersAverage =
        ((encoderLeftDriver->getCount() + encoderRightDriver->getCount()) / 2) *
        RobotEnv::WHEEL_CIRCUMFERENCE / 4095;

    // Condição de parada controlada
    if(!param->globalData.isReadyToRun.load(std::memory_order_relaxed) ||
       encoderMilimetersAverage > finishLineCount) {
      lastIsReadyToRun = false;
      motorDriver->pwmOutput(0, 0);
      vacuumDriver->pwmOutput(0);
      sendStatusUpdate(param->globalData, "Stopped at ",
                       encoderMilimetersAverage);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    } else { // Condição de início controlada
      if(param->globalData.isReadyToRun.load(std::memory_order_relaxed) !=
         lastIsReadyToRun) {
        lastIsReadyToRun = true;

        encoderLeftDriver->clearCount();
        encoderRightDriver->clearCount();

        finishLineCount =
            param->globalData.finishLineCount.load(std::memory_order_relaxed);

        vTaskDelay(4000 / portTICK_PERIOD_MS);
        vacuumDriver->pwmOutput(RobotEnv::BASE_VACUUM_PWM);
        vTaskDelay(1000);
      }
    }

    irSensorDriver->readCalibrated(lineSensorValues, sideSensorValues);

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
          sendStatusUpdate(param->globalData, "R ", encoderMilimetersAverage);

          lastLeftReadIsOnMark  = false;
          lastRightReadIsOnMark = true;
        }
      } else if(leftIsOnMark && !rightIsOnMark) {
        if(!lastLeftReadIsOnMark) {
          param->globalData.markCount.store(
              param->globalData.markCount.load(std::memory_order_relaxed) + 1,
              std::memory_order_relaxed);
          sendStatusUpdate(param->globalData, "L ", encoderMilimetersAverage);

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

    if(param->globalData.mapData[mapPointIndex].encoderMilimeters >
           encoderMilimetersAverage &&
       (mapPointIndex + 1) < param->globalData.mapData.size()) {
      mapPointIndex++;
    }

    float pathPID = pathController->getPID();

    motorDriver->pwmOutput(
        param->globalData.mapData[mapPointIndex].baseMotorPWM + pathPID,
        param->globalData.mapData[mapPointIndex].baseMotorPWM - pathPID);

    vacuumDriver->pwmOutput(RobotEnv::BASE_VACUUM_PWM);

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
    //        param->globalData.mapData[mapPointIndex].baseMotorPWM);
    // printf("Path PID: %f\n", pathPID);
    // printf("Mark Count: %ld\n",
    //        param->globalData.markCount.load(std::memory_order_relaxed));
    // printf("Encoder Left: %ld\n", encoderLeftDriver->getCount());
    // printf("Encoder Right: %ld\n", encoderRightDriver->getCount());

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

#endif // MAIN_TASK_HPP
