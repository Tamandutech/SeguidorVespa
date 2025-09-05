#ifndef MAIN_TASK_HPP
#define MAIN_TASK_HPP

#include "context/UselessData.hpp"

#include "drivers/MotorDriver/MotorDriver.hpp"

#include "tasks/MainTask/PathController/PathController.hpp"

struct MainTaskParamSchema {
  UselessData &uselessData;
};

void mainTaskLoop(void *params) {
  MainTaskParamSchema *param = (MainTaskParamSchema *)params;
  MotorDriver *motorDriver   = new MotorDriver(param->uselessData);

  // Array de valores dos sensores (placeholder - deve ser inicializado com
  // dados reais)
  int sensor_values[12] = {0}; // Inicializa com zeros

  PathControllerParamSchema pathControllerParam = {
      .constants        = {.kP = 0.1F, .kI = 0.01F, .kD = 0.001F},
      .sensor_quantity  = 12,
      .sensor_values    = sensor_values,
      .max_angle        = 45.0F, // Ângulo máximo de 45 graus
      .radius_sensor    = 100, // Raio dos sensores em mm
      .sensor_to_center = 50, // Distância do sensor ao centro em mm
  };
  PathController *pathController = new PathController(pathControllerParam);

  for(;;) {
    pathController->getPID();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

#endif // MAIN_TASK_HPP
