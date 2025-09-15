#ifndef MAIN_TASK_HPP
#define MAIN_TASK_HPP

#include "context/GlobalData.hpp"

#include "drivers/IRSensorDriver/IRSensorDriver.hpp"
#include "drivers/MotorDriver/MotorDriver.hpp"

#include "tasks/MainTask/PathController/PathController.hpp"

struct MainTaskParamSchema {
  GlobalData &globalData;
};

void mainTaskLoop(void *params) {
  MainTaskParamSchema *param = static_cast<MainTaskParamSchema *>(params);

  MotorPins    motorPins   = {.gpioDirectionA = RobotEnv::GPIO_DIRECTION_A,
                              .gpioDirectionB = RobotEnv::GPIO_DIRECTION_B,
                              .gpioPWMA       = RobotEnv::GPIO_PWM_A,
                              .gpioPWMB       = RobotEnv::GPIO_PWM_B};
  MotorDriver *motorDriver = new MotorDriver(motorPins);

  IRSensorParamSchema irSensorParam = {
      .pins                = {.gpioMultiplexerDigitalAddress =
                                  RobotEnv::GPIO_MULTIPLEXER_DIGITAL_ADDRESS,
                              .gpioMultiplexerAnalogInput =
                                  RobotEnv::GPIO_MULTIPLEXER_ANALOG_INPUT},
      .frontSensorsCount   = 12,
      .sideSensorsCount    = 4,
      .multiplexerPinCount = 4
  };
  IRSensorDriver *irSensorDriver = new IRSensorDriver(irSensorParam);

  // PathControllerParamSchema pathControllerParam = {
  //     .constants      = {.kP = 0.1F, .kI = 0.01F, .kD = 0.001F},
  //     .sensorQuantity = 12,
  //     .sensorValues   = &sensorValues[0],
  //     .maxAngle       = 45.0F, // Ângulo máximo de 45 graus
  //     .radiusSensor   = 100, // Raio dos sensores em mm
  //     .sensorToCenter = 50, // Distância do sensor ao centro em mm
  // };
  // PathController *pathController = new PathController(pathControllerParam);

  for(;;) {
    uint16_t sensorValues[16];
    irSensorDriver->read(sensorValues);
    // for(int i = 0; i < 16; i++) {
    //   ESP_LOGI("MainTask", "Sensor %d: %d", i, sensorValues[i]);
    // }
    // float pathPID = pathController->getPID();
    printf("ALGUMA COISA\n");

    // TODO: Use motorDriver to apply PID output to motors
    // motorDriver->setSpeed(pathPID);

    // Suppress unused variable warnings - these will be used for motor control
    // (void)motorDriver;
    // (void)pathPID;

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

#endif // MAIN_TASK_HPP
