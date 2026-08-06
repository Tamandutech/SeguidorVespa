#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/**
 * @brief Tipos de evento que podem ser postados na máquina de estados.
 *
 * Eventos são enviados via fila FreeRTOS e processados pela
 * @ref StateMachineTask para disparar transições de estado.
 */
enum class EventType {
  START,            // Inicia a execução do robô (IDLE → RUNNING).
  STOP,             // Para a operação atual e retorna a IDLE.
  CALIBRATE,        // Inicia a calibração dos sensores (IDLE → CALIBRATING).
  MAP,              // Inicia o mapeamento da pista (IDLE/RUNNING → MAPPING).
  CALIBRATION_DONE, // Sinaliza fim da calibração (CALIBRATING → IDLE).
  ERROR             // Erro fatal; força retorno imediato a IDLE.
};

/**
 * @brief Mensagem de evento enviada para a máquina de estados.
 *
 * Estrutura colocada na fila FreeRTOS por StateMachineTask::postEvent
 * e consumida pelo loop principal da task.
 */
struct Event {
  EventType type; // Tipo do evento a ser processado.
};

/**
 * @brief Estados operacionais possíveis do robô.
 *
 * Representa o modo atual da máquina de estados finita controlada
 * por StateMachineTask.
 */
enum class RobotState {
  IDLE,        ///< Aguardando comando; motores/sensores em repouso.
  CALIBRATING, ///< Calibrando sensores (ex.: IR).
  RUNNING,     ///< Seguindo a linha / em operação normal.
  MAPPING      ///< Mapeando a pista.
};

/**
 * @brief Estado global atual do robô, espelhado pela máquina de estados.
 *
 * @note Variável SOMENTE DE LEITURA para as demais tasks.
 * @note Deve ser escrita apenas pela task da máquina de estados.
 */
extern volatile RobotState gRobotState;

class StateMachineTask {
public:
  StateMachineTask();
  ~StateMachineTask();

  bool       start(uint32_t stackSizeWords = 2048, UBaseType_t priority = 3,
                   BaseType_t coreId = 0);
  bool       postEvent(const Event &event, TickType_t timeoutTicks = 0);
  RobotState getState() const;

private:
  static void taskEntry(void *param);
  void        run();

  void handleEvent(const Event &event);
  void transitionTo(RobotState newState);

  // State handlers
  void onIdle(const Event &event);
  void onCalibrating(const Event &event);
  void onRunning(const Event &event);
  void onMapping(const Event &event);

private:
  QueueHandle_t queue;
  TaskHandle_t  taskHandle;
  RobotState    currentState;
};