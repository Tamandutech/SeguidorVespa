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

  // uint16_t sensorValuesRaw[16];
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
      .constants      = {.kP = 0.01F, .kI = 0.00F, .kD = 0.19F},
      .sensorQuantity = 12,
      .sensorValues   = lineSensorValues,
      .maxAngle       = 45.0F, // Ângulo máximo de 45 graus
      .radiusSensor   = 100, // Raio dos sensores em mm
      .sensorToCenter = 50, // Distância do sensor ao centro em mm
  };
  PathController *pathController = new PathController(pathControllerParam);

  ESP_LOGI("MainTask", "Calibrando os sensores...");
  for(int i = 0; i < 50; i++) {
    irSensorDriver->calibrate();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  ESP_LOGI("MainTask", "Sensores calibrados");

  vTaskDelay(5000 / portTICK_PERIOD_MS);

  for(;;) {
    printf("\033[2J\033[H");
    // irSensorDriver->read(sensorValuesRaw);
    // for(int i = 0; i < 16; i++) {
    //   printf("%2d:", i);
    //   printf("%4d ", sensorValuesRaw[i]);
    // }
    // printf("\n");

    irSensorDriver->readCalibrated(lineSensorValues, sideSensorValues);

    float pathPID = pathController->getPID();

    // printf("L: ");
    // for(int i = 0; i < 12; i++) {
    //   printf("%4d ", lineSensorValues[i]);
    // }
    // printf("S: ");
    // for(int i = 0; i < 4; i++) {
    //   printf("%4d ", sideSensorValues[i]);
    // }
    // printf("\n");
    // printf("Path PID: %f\n", pathPID);

    motorDriver->pwmOutput(RobotEnv::BASE_MOTOR_PWM + pathPID,
                           RobotEnv::BASE_MOTOR_PWM - pathPID);

    // motorDriver->pwmOutput(RobotEnv::BASE_MOTOR_PWM - pathPID,
    //                        RobotEnv::BASE_MOTOR_PWM + pathPID); // teste
    //                        semreh

    // motorDriver->pwmOutput(0, 0);
    // vacuumDriver->pwmOutput(RobotEnv::BASE_VACUUM_PWM);
    // printf("Encoder Left: %ld\n", encoderLeftDriver->getCount());
    // printf("Encoder Right: %ld\n", encoderRightDriver->getCount());

    // TODO: Use motorDriver to apply PID output to motors
    // motorDriver->setSpeed(pathPID);

    // Suppress unused variable warnings - these will be used for motor
    // control
    (void)motorDriver;
    (void)encoderLeftDriver;
    (void)encoderRightDriver;
    (void)vacuumDriver;
    // (void)pathPID;

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

#endif // MAIN_TASK_HPP
