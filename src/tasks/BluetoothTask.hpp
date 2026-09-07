#pragma once

#include <cstdarg>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "cli/nimble-nordic-uart/nimble-nordic-uart.hpp"

class StateMachineTask;

struct BluetoothEvent {
  enum class Kind : uint8_t {
    UartRxLine,
    OutgoingMessage,
    BleConnected,
    BleDisconnected,
  } kind;
  static constexpr size_t kDataCap = 2048;
  /// Comprimento válido de `data` para tipos de texto; 0 para eventos de
  /// conexão.
  uint16_t len;
  char     data[kDataCap];
};

/**
 * Active Object: uma tarefa FreeRTOS, uma fila, E/S BLE e tratamento da CLI
 * apenas nesse contexto. Callbacks NimBLE apenas inserem na fila; notify BLE
 * (`nordic_uart_send`) ocorre nesta tarefa — imediatamente se o chamador já
 * for ela (CLI), senão via fila.
 */
class BluetoothTask {
public:
  /// Associa a tarefa à máquina de estados do robô usada pelos comandos da CLI.
  explicit BluetoothTask(StateMachineTask *stateMachine);

  /// Cria e inicia a tarefa FreeRTOS fixada no núcleo solicitado.
  bool start(uint32_t stackSizeWords = 16384, UBaseType_t priority = 2,
             BaseType_t coreId = 0);

  /// Formata e entrega uma linha para notify BLE (seguro de qualquer tarefa).
  bool postOutgoingMessage(const char *fmt, ...)
      __attribute__((format(printf, 2, 3)));

  /// Entrega uma mensagem já formatada: envia agora se já estiver nesta tarefa,
  /// senão enfileira (pode descartar se a fila estiver cheia).
  bool deliverOutgoing(const char *msg);

  /// Formata e entrega; um único buffer de evento quando chamado de outra tarefa.
  bool deliverFormatted(const char *fmt, va_list ap);

  /// API genérica para inserir eventos na fila; usada por callbacks e
  /// produtores externos.
  bool post(const BluetoothEvent &event, TickType_t timeoutTicks = 0);

  /// Callbacks NimBLE / Nordic UART (não bloqueiam; apenas inserem na fila).
  void notifyBleRx(struct ble_gatt_access_ctxt *ctxt);
  void notifyBleStatus(enum nordic_uart_callback_type callbackType);

private:
  /// Trampolim estático exigido por xTaskCreatePinnedToCore.
  static void taskEntry(void *param);
  /// Loop principal do Active Object: inicializa o BLE e esvazia a fila
  /// interna.
  void run();

  bool onThisTask() const;

  /// Despacha um evento bluetooth inserido na fila.
  void processEvent(const BluetoothEvent &event);
  /// Analisa e executa uma linha da CLI recebida por BLE.
  void processIncomingLine(char *line);
  /// Acumula fragmentos GATT até a mensagem wire terminar em `;`.
  void appendIncomingRx(const char *chunk, size_t chunkLen);

  StateMachineTask *stateMachine_;
  QueueHandle_t     queue_;
  TaskHandle_t      taskHandle_;
  char              rxAcc_[BluetoothEvent::kDataCap]{};
  size_t            rxAccLen_{};
};

/// Gancho global para outros subsistemas enviarem mensagens/telemetria por BLE
/// (não bloqueante).
bool bluetoothPushMessage(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));
