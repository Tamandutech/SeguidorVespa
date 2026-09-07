#include "BluetoothTask.hpp"

#include <cctype>
#include <cstdarg>
#include <cstring>

#include "esp_log.h"

#include "host/ble_gatt.h"
#include "os/os_mbuf.h"

#include "tasks/StateMachineTask.hpp"
#include "tasks/cli/cli.hpp"

namespace {
const char *TAG = "BluetoothTask";

constexpr UBaseType_t kQueueDepth = 8;

BluetoothTask *g_bluetoothTask = nullptr;

// Encaminha os callbacks de conexão do Nordic UART para a fila do
// BluetoothTask.
void bleStatusCallback(enum nordic_uart_callback_type callbackType) {
  if(g_bluetoothTask != nullptr) {
    g_bluetoothTask->notifyBleStatus(callbackType);
  }
}

// Encaminha o callback de RX do Nordic UART para a fila do BluetoothTask.
void bleRxCallback(struct ble_gatt_access_ctxt *ctxt) {
  if(g_bluetoothTask != nullptr) {
    g_bluetoothTask->notifyBleRx(ctxt);
  }
}
} // namespace

// Aloca a fila interna de eventos; a thread da tarefa é iniciada por start().
BluetoothTask::BluetoothTask(StateMachineTask *stateMachine)
    : stateMachine_(stateMachine), queue_(nullptr), taskHandle_(nullptr) {
  queue_ = xQueueCreate(kQueueDepth, sizeof(BluetoothEvent));
}

// Inicia a tarefa Active Object e registra esta instância para callbacks em C.
bool BluetoothTask::start(uint32_t stackSizeWords, UBaseType_t priority,
                          BaseType_t coreId) {
  if(queue_ == nullptr || taskHandle_ != nullptr || stateMachine_ == nullptr) {
    return false;
  }

  g_bluetoothTask     = this;
  const BaseType_t ok = xTaskCreatePinnedToCore(
      &BluetoothTask::taskEntry, "bluetooth_task", stackSizeWords, this,
      priority, &taskHandle_, coreId);
  if(ok != pdPASS) {
    g_bluetoothTask = nullptr;
    return false;
  }
  return true;
}

// Inserção na fila thread-safe usada por callbacks próximos a ISR e pelo código
// da aplicação.
bool BluetoothTask::post(const BluetoothEvent &event, TickType_t timeoutTicks) {
  if(queue_ == nullptr) {
    return false;
  }
  return xQueueSend(queue_, &event, timeoutTicks) == pdTRUE;
}

bool BluetoothTask::onThisTask() const {
  return taskHandle_ != nullptr && xTaskGetCurrentTaskHandle() == taskHandle_;
}

bool BluetoothTask::postOutgoingMessage(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  const bool ok = deliverFormatted(fmt, ap);
  va_end(ap);
  return ok;
}

// CLI (esta tarefa): formata num buffer curto e envia na hora, sem enfileirar
// dumps (`map_get`). ControlTask formata direto no BluetoothEvent e posta —
// um único ~2 KiB, não dois.
bool BluetoothTask::deliverFormatted(const char *fmt, va_list ap) {
  if(fmt == nullptr) {
    return false;
  }
  if(onThisTask()) {
    char buf[512];
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    buf[sizeof(buf) - 1] = '\0';
    return nordic_uart_send(buf) == ESP_OK;
  }
  BluetoothEvent ev{};
  ev.kind = BluetoothEvent::Kind::OutgoingMessage;
  (void)vsnprintf(ev.data, sizeof(ev.data), fmt, ap);
  ev.data[sizeof(ev.data) - 1] = '\0';
  ev.len                      = static_cast<uint16_t>(strlen(ev.data));
  if(!post(ev, 0)) {
    ESP_LOGW(TAG, "bluetooth queue full, dropping TX");
    return false;
  }
  return true;
}

bool BluetoothTask::deliverOutgoing(const char *msg) {
  if(msg == nullptr) {
    return false;
  }
  if(onThisTask()) {
    return nordic_uart_send(msg) == ESP_OK;
  }
  BluetoothEvent ev{};
  ev.kind           = BluetoothEvent::Kind::OutgoingMessage;
  const size_t n    = strlen(msg);
  const size_t copy = (n < sizeof(ev.data) - 1U) ? n : (sizeof(ev.data) - 1U);
  memcpy(ev.data, msg, copy);
  ev.data[copy] = '\0';
  ev.len        = static_cast<uint16_t>(copy);
  if(!post(ev, 0)) {
    ESP_LOGW(TAG, "bluetooth queue full, dropping TX");
    return false;
  }
  return true;
}

// Copia a cadeia completa de mbufs (um Write GATT pode vir fragmentado).
static size_t copyMbufChain(const struct os_mbuf *om, char *dst,
                            size_t maxCopy) {
  size_t off = 0;
  for(; om != nullptr && off < maxCopy; om = SLIST_NEXT(om, om_next)) {
    const size_t chunk = static_cast<size_t>(om->om_len);
    if(chunk == 0 || om->om_data == nullptr) {
      continue;
    }
    const size_t n = (off + chunk <= maxCopy) ? chunk : (maxCopy - off);
    memcpy(dst + off, om->om_data, n);
    off += n;
  }
  return off;
}

static bool wireMessageComplete(const char *buf, size_t len) {
  while(len > 0 && isspace(static_cast<unsigned char>(buf[len - 1]))) {
    --len;
  }
  return len > 0 && buf[len - 1] == ';';
}

// Manipulador do callback de RX BLE: copia o payload e insere na fila um evento
// de linha da CLI.
void BluetoothTask::notifyBleRx(struct ble_gatt_access_ctxt *ctxt) {
  if(ctxt == nullptr || ctxt->om == nullptr) {
    return;
  }
  BluetoothEvent ev{};
  ev.kind              = BluetoothEvent::Kind::UartRxLine;
  const size_t maxCopy = sizeof(ev.data) - 1U;
  const size_t copy_len = copyMbufChain(ctxt->om, ev.data, maxCopy);
  ev.data[copy_len]     = '\0';
  ev.len                = static_cast<uint16_t>(copy_len);
  if(!post(ev, 0)) {
    ESP_LOGW(TAG, "bluetooth queue full, dropping RX");
  }
}

// Manipulador do callback de estado da conexão BLE: insere na fila eventos de
// conexão/desconexão.
void BluetoothTask::notifyBleStatus(enum nordic_uart_callback_type t) {
  BluetoothEvent ev{};
  ev.kind    = (t == NORDIC_UART_CONNECTED)
                 ? BluetoothEvent::Kind::BleConnected
                 : BluetoothEvent::Kind::BleDisconnected;
  ev.len     = 0;
  ev.data[0] = '\0';
  if(!post(ev, 0)) {
    ESP_LOGW(TAG, "bluetooth queue full, dropping status");
  }
}

// Trampolim de entrada do FreeRTOS.
void BluetoothTask::taskEntry(void *param) {
  auto *self = static_cast<BluetoothTask *>(param);
  self->run();
}

// Tempo de execução do Active Object: inicializa o UART BLE e processa
// indefinidamente os eventos inseridos na fila.
void BluetoothTask::run() {
  if(nordic_uart_start("TT_SEMREH", bleStatusCallback) != ESP_OK) {
    ESP_LOGE(TAG, "nordic_uart_start failed");
    vTaskDelete(nullptr);
    return;
  }
  if(nordic_uart_yield(bleRxCallback) != ESP_OK) {
    ESP_LOGE(TAG, "nordic_uart_yield failed");
    vTaskDelete(nullptr);
    return;
  }

  BluetoothEvent event{};
  while(true) {
    if(xQueueReceive(queue_, &event, portMAX_DELAY) == pdTRUE) {
      processEvent(event);
      while(xQueueReceive(queue_, &event, 0) == pdTRUE) {
        processEvent(event);
      }
    }
  }
}

void BluetoothTask::appendIncomingRx(const char *chunk, size_t chunkLen) {
  if(chunk == nullptr || chunkLen == 0) {
    return;
  }
  const size_t room = sizeof(rxAcc_) - 1U;
  if(rxAccLen_ + chunkLen > room) {
    ESP_LOGW(TAG, "CLI RX overflow (%u+%u), dropping buffer",
             static_cast<unsigned>(rxAccLen_),
             static_cast<unsigned>(chunkLen));
    rxAccLen_ = 0;
    if(chunkLen > room) {
      return;
    }
  }
  memcpy(rxAcc_ + rxAccLen_, chunk, chunkLen);
  rxAccLen_ += chunkLen;
  rxAcc_[rxAccLen_] = '\0';

  if(!wireMessageComplete(rxAcc_, rxAccLen_)) {
    return;
  }
  processIncomingLine(rxAcc_);
  rxAccLen_    = 0;
  rxAcc_[0]    = '\0';
}

// Trata um evento: entrada da CLI, notify de saída ou registro de conexão.
void BluetoothTask::processEvent(const BluetoothEvent &event) {
  switch(event.kind) {
  case BluetoothEvent::Kind::UartRxLine: {
    const size_t n = static_cast<size_t>(event.len);
    appendIncomingRx(event.data, n);
    break;
  }
  case BluetoothEvent::Kind::OutgoingMessage:
    (void)nordic_uart_send(event.data);
    break;
  case BluetoothEvent::Kind::BleConnected:
    ESP_LOGI(TAG, "BLE UART connected");
    break;
  case BluetoothEvent::Kind::BleDisconnected:
    ESP_LOGI(TAG, "BLE UART disconnected");
    rxAccLen_ = 0;
    rxAcc_[0] = '\0';
    break;
  }
}

// Executa o parser da CLI e emite respostas de erro compatíveis com o
// protocolo.
void BluetoothTask::processIncomingLine(char *line) {
  const int cliResult = cli_process(line, stateMachine_);
  if(cliResult != CLI_SUCCESS) {
    const char *received = (line != nullptr) ? line : "";
    switch(cliResult) {
    case CLI_ERROR_EMPTY_COMMAND:
      ESP_LOGE(TAG, "CLI Error: Empty command (received: \"%s\")", received);
      (void)postOutgoingMessage("Error: Empty command\r\n");
      break;
    case CLI_ERROR_COMMAND_NOT_FOUND:
      ESP_LOGE(
          TAG,
          "CLI Error: Command not found / bad wire segment (received: \"%s\")",
          received);
      (void)postOutgoingMessage("Error: Command not found\r\n");
      break;
    case CLI_ERROR_TOO_MANY_ARGS:
      ESP_LOGE(TAG, "CLI Error: Too many wire segments (received: \"%s\")",
               received);
      (void)postOutgoingMessage("Error: Too many segments\r\n");
      break;
    default:
      ESP_LOGE(TAG, "CLI Error: Unknown error (code: %d, received: \"%s\")",
               cliResult, received);
      (void)postOutgoingMessage("Error: Unknown error (code: %d)\r\n",
                                cliResult);
      break;
    }
  }
}

// Auxiliar global usado pelos manipuladores de comando e telemetria.
bool bluetoothPushMessage(const char *fmt, ...) {
  if(g_bluetoothTask == nullptr) {
    return false;
  }
  va_list ap;
  va_start(ap, fmt);
  const bool ok = g_bluetoothTask->deliverFormatted(fmt, ap);
  va_end(ap);
  return ok;
}
