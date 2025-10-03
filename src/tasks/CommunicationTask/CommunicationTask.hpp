#ifndef COMMUNICATION_TASK_HPP
#define COMMUNICATION_TASK_HPP

#include <string>

#include "nimble-nordic-uart/nimble-nordic-uart.h"

#include "context/GlobalData.hpp"

struct CommunicationTaskParamSchema {
  GlobalData &globalData;
};

void uartStatusChangeCallback(enum nordic_uart_callback_type callback_type) {
  if(callback_type == NORDIC_UART_CONNECTED) {
    ESP_LOGI("CommunicationTask", "BLE UART connected");
  } else if(callback_type == NORDIC_UART_DISCONNECTED) {
    ESP_LOGI("CommunicationTask", "BLE UART disconnected");
  }
}

void uartReceiveCallback(struct ble_gatt_access_ctxt *ctxt) {
  ESP_LOGI("CommunicationTask", "BLE UART received data: %s",
           ctxt->om->om_data);

  if(strcmp((const char *)ctxt->om->om_data, "y") == 0) {
    globalData.isReadyToRun.store(true, std::memory_order_relaxed);
  } else if(strcmp((const char *)ctxt->om->om_data, "n") == 0) {
    globalData.isReadyToRun.store(false, std::memory_order_relaxed);
  }

  ESP_LOGI("CommunicationTask", "isReadyToRun: %d",
           globalData.isReadyToRun.load(std::memory_order_relaxed));

  std::string message =
      "Received data:" + std::string((const char *)ctxt->om->om_data);
  nordic_uart_send(message.c_str());
}

void communicationTaskLoop(void *params) {
  CommunicationTaskParamSchema *param =
      static_cast<CommunicationTaskParamSchema *>(params);

  nordic_uart_start("O robô mais rápido", uartStatusChangeCallback);

  nordic_uart_yield(uartReceiveCallback);

  // Suppress unused parameter warning - param will be used for
  // communication logic
  (void)param;

  for(;;) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

#endif // COMMUNICATION_TASK_HPP
