#ifndef COMMUNICATION_TASK_HPP
#define COMMUNICATION_TASK_HPP

#include <string>

#include "nimble-nordic-uart/nimble-nordic-uart.h"

#include "context/GlobalData.hpp"

struct CommunicationTaskParamSchema {
  GlobalData &globalData;
};

// Helper functions to send messages to the communication task
void sendSensorData(GlobalData &globalData, int32_t sensorValue) {
  Message msg;
  msg.type                        = MessageType::SENSOR_DATA;
  msg.data.sensorData.sensorValue = sensorValue;
  msg.data.sensorData.timestamp   = xTaskGetTickCount();

  if(xQueueSend(globalData.communicationQueue, &msg, pdMS_TO_TICKS(100)) !=
     pdTRUE) {
    ESP_LOGW("MainTask", "Failed to send sensor data to communication queue");
  }
}

void sendStatusUpdate(GlobalData &globalData, const char *status,
                      int32_t value) {
  Message msg;
  msg.type                     = MessageType::STATUS_UPDATE;
  msg.data.statusUpdate.status = status;
  msg.data.statusUpdate.value  = value;

  if(xQueueSend(globalData.communicationQueue, &msg, pdMS_TO_TICKS(100)) !=
     pdTRUE) {
    ESP_LOGW("MainTask", "Failed to send status update to communication queue");
  }
}

void sendCommand(GlobalData &globalData, const char *command,
                 int32_t parameter) {
  Message msg;
  msg.type                   = MessageType::COMMAND;
  msg.data.command.command   = command;
  msg.data.command.parameter = parameter;

  if(xQueueSend(globalData.communicationQueue, &msg, pdMS_TO_TICKS(100)) !=
     pdTRUE) {
    ESP_LOGW("MainTask", "Failed to send command to communication queue");
  }
}

void sendEmergencyStop(GlobalData &globalData) {
  Message msg;
  msg.type = MessageType::EMERGENCY_STOP;

  if(xQueueSend(globalData.communicationQueue, &msg, pdMS_TO_TICKS(100)) !=
     pdTRUE) {
    ESP_LOGW("MainTask",
             "Failed to send emergency stop to communication queue");
  }
}

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

  if(strcmp((const char *)ctxt->om->om_data, "y") == 0 ||
     strcmp((const char *)ctxt->om->om_data, "Y") == 0) {
    globalData.isReadyToRun.store(true, std::memory_order_relaxed);
  } else if(strcmp((const char *)ctxt->om->om_data, "n") == 0 ||
            strcmp((const char *)ctxt->om->om_data, "N") == 0) {
    globalData.isReadyToRun.store(false, std::memory_order_relaxed);
  }

  ESP_LOGI("CommunicationTask", "isReadyToRun: %d",
           globalData.isReadyToRun.load(std::memory_order_relaxed));

  std::string message =
      "Received data:" + std::string((const char *)ctxt->om->om_data);
  nordic_uart_send(message.c_str());
}

// Function to process messages from the queue
void processMessage(const Message &msg) {
  switch(msg.type) {
  case MessageType::SENSOR_DATA:
    ESP_LOGI("CommunicationTask", "Sensor data: %ld at %lu",
             msg.data.sensorData.sensorValue, msg.data.sensorData.timestamp);
    nordic_uart_send(std::to_string(msg.data.sensorData.sensorValue).c_str());
    break;

  case MessageType::STATUS_UPDATE:
    ESP_LOGI("CommunicationTask", "Status: %s = %ld",
             msg.data.statusUpdate.status, msg.data.statusUpdate.value);
    {
      std::string statusMsg = std::string(msg.data.statusUpdate.status) +
                              " = " +
                              std::to_string(msg.data.statusUpdate.value);
      nordic_uart_send(statusMsg.c_str());
    }
    break;

  case MessageType::COMMAND:
    ESP_LOGI("CommunicationTask", "Command: %s with param %ld",
             msg.data.command.command, msg.data.command.parameter);
    {
      std::string cmdMsg = std::string(msg.data.command.command) + " " +
                           std::to_string(msg.data.command.parameter);
      nordic_uart_send(cmdMsg.c_str());
    }
    break;

  case MessageType::EMERGENCY_STOP:
    ESP_LOGW("CommunicationTask", "Emergency stop received!");
    nordic_uart_send("EMERGENCY STOP!");
    break;
  }
}

void communicationTaskLoop(void *params) {
  CommunicationTaskParamSchema *param =
      static_cast<CommunicationTaskParamSchema *>(params);

  nordic_uart_start("O robô mais rápido", uartStatusChangeCallback);
  nordic_uart_yield(uartReceiveCallback);

  Message receivedMessage;

  for(;;) {
    // Check for messages from the queue
    if(xQueueReceive(param->globalData.communicationQueue, &receivedMessage,
                     pdMS_TO_TICKS(100)) == pdTRUE) {
      processMessage(receivedMessage);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

#endif // COMMUNICATION_TASK_HPP
