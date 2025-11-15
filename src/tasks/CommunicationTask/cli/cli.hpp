#ifndef CLI_HPP
#define CLI_HPP

#include "esp_log.h"
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>

#include "../lib/CommunicationUtils.hpp"
#include "context/GlobalData.hpp"
#include "context/RobotEnv.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

// CLI return codes
#define CLI_SUCCESS                 0
#define CLI_ERROR_EMPTY_COMMAND     1
#define CLI_ERROR_COMMAND_NOT_FOUND 2
#define CLI_ERROR_TOO_MANY_ARGS     3

// Parse error codes for className.parameterName format
enum class ParseError {
  SUCCESS = 0,
  EMPTY_STRING,
  NO_DOT,
  MULTIPLE_DOTS,
  EMPTY_CLASS_NAME,
  EMPTY_PARAMETER_NAME,
  INVALID_FORMAT
};

// Structure to hold parsed className and parameterName
struct ParsedReference {
  char className[64];
  char parameterName[64];
};

// Parse a string in the format <className>.<parameterName>
// Returns ParseError enum indicating success or specific error
ParseError parseClassNameParameter(const char *input, ParsedReference &result) {
  // Check for empty string
  if(input == nullptr || input[0] == '\0') {
    return ParseError::EMPTY_STRING;
  }

  // Find the dot position
  const char *dotPos = strchr(input, '.');

  // Check if dot exists
  if(dotPos == nullptr) {
    return ParseError::NO_DOT;
  }

  // Check for multiple dots
  if(strchr(dotPos + 1, '.') != nullptr) {
    return ParseError::MULTIPLE_DOTS;
  }

  // Calculate lengths
  size_t classNameLen     = dotPos - input;
  size_t parameterNameLen = strlen(dotPos + 1);

  // Check if className is empty
  if(classNameLen == 0) {
    return ParseError::EMPTY_CLASS_NAME;
  }

  // Check if parameterName is empty
  if(parameterNameLen == 0) {
    return ParseError::EMPTY_PARAMETER_NAME;
  }

  // Check buffer sizes
  if(classNameLen >= sizeof(result.className) ||
     parameterNameLen >= sizeof(result.parameterName)) {
    return ParseError::INVALID_FORMAT;
  }

  // Copy className (before dot)
  strncpy(result.className, input, classNameLen);
  result.className[classNameLen] = '\0';

  // Copy parameterName (after dot)
  strncpy(result.parameterName, dotPos + 1, parameterNameLen);
  result.parameterName[parameterNameLen] = '\0';

  return ParseError::SUCCESS;
}

// Helper function to get parameter value as string
bool getParameterValue(const char *className, const char *parameterName,
                       char *valueBuffer, size_t bufferSize) {
  // System parameters
  if(strcmp(className, "System") == 0) {
    if(strcmp(parameterName, "finishLineCount") == 0) {
      snprintf(valueBuffer, bufferSize, "%ld",
               globalData.finishLineCount.load(std::memory_order_relaxed));
      return true;
    }
    if(strcmp(parameterName, "markCount") == 0) {
      snprintf(valueBuffer, bufferSize, "%ld",
               globalData.markCount.load(std::memory_order_relaxed));
      return true;
    }
  }

  // State parameters
  if(strcmp(className, "State") == 0) {
    if(strcmp(parameterName, "isReadyToRun") == 0) {
      snprintf(valueBuffer, bufferSize, "%d",
               globalData.isReadyToRun.load(std::memory_order_relaxed) ? 1 : 0);
      return true;
    }
  }

  // TODO: Add more parameter classes (PID, etc.) as they become available
  return false;
}

// Helper function to set parameter value
bool setParameterValue(const char *className, const char *parameterName,
                       const char *value) {
  // Handle negative values (prefixed with !)
  bool        isNegative  = (value[0] == '!');
  const char *actualValue = isNegative ? value + 1 : value;

  // System parameters
  if(strcmp(className, "System") == 0) {
    if(strcmp(parameterName, "finishLineCount") == 0) {
      int32_t val = atoi(actualValue);
      if(isNegative) val = -val;
      globalData.finishLineCount.store(val, std::memory_order_relaxed);
      return true;
    }
    if(strcmp(parameterName, "markCount") == 0) {
      int32_t val = atoi(actualValue);
      if(isNegative) val = -val;
      globalData.markCount.store(val, std::memory_order_relaxed);
      return true;
    }
  }

  // State parameters
  if(strcmp(className, "State") == 0) {
    if(strcmp(parameterName, "isReadyToRun") == 0) {
      int val = atoi(actualValue);
      globalData.isReadyToRun.store(val != 0, std::memory_order_relaxed);
      return true;
    }
  }

  // TODO: Add more parameter classes (PID, etc.) as they become available
  return false;
}

// Command handler function pointer type
// Takes argc and argv array, returns CLI status code
typedef int (*CommandHandler)(int argc, char *argv[]);

// ========== Command Handler Functions ==========

// Parameter Commands
static int handleParamList(int argc, char *argv[]) {
  // List all available parameters
  pushMessageToQueue(globalData, "Parameters:\n");
  int index = 0;

  // System parameters
  char value[64];
  if(getParameterValue("System", "finishLineCount", value, sizeof(value))) {
    pushMessageToQueue(globalData, "  %d - System.finishLineCount: %s\n",
                       index++, value);
  }
  if(getParameterValue("System", "markCount", value, sizeof(value))) {
    pushMessageToQueue(globalData, "  %d - System.markCount: %s\n", index++,
                       value);
  }

  // State parameters
  if(getParameterValue("State", "isReadyToRun", value, sizeof(value))) {
    pushMessageToQueue(globalData, "  %d - State.isReadyToRun: %s\n", index++,
                       value);
  }

  // TODO: Add PID parameters and others as they become available
  return CLI_SUCCESS;
}

static int handleParamGet(int argc, char *argv[]) {
  // Get parameter value: param_get <className>.<parameterName>
  if(argc < 2) {
    ESP_LOGW("CLI", "param_get requires a parameter reference\n");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }

  ParsedReference ref;
  ParseError      parseError = parseClassNameParameter(argv[1], ref);
  if(parseError != ParseError::SUCCESS) {
    ESP_LOGW("CLI", "Invalid parameter format: %s\n", argv[1]);
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }

  char value[64];
  if(getParameterValue(ref.className, ref.parameterName, value,
                       sizeof(value))) {
    pushMessageToQueue(globalData, "%s", value);
    return CLI_SUCCESS;
  } else {
    ESP_LOGW("CLI", "Parameter not found: %s.%s\n", ref.className,
             ref.parameterName);
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
}

static int handleParamSet(int argc, char *argv[]) {
  // Set parameter value: param_set <className>.<parameterName> <value>
  if(argc < 3) {
    ESP_LOGW("CLI", "param_set requires parameter reference and value\n");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }

  ParsedReference ref;
  ParseError      parseError = parseClassNameParameter(argv[1], ref);
  if(parseError != ParseError::SUCCESS) {
    ESP_LOGW("CLI", "Invalid parameter format: %s\n", argv[1]);
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }

  if(setParameterValue(ref.className, ref.parameterName, argv[2])) {
    return CLI_SUCCESS;
  } else {
    ESP_LOGW("CLI", "Parameter not found or cannot be set: %s.%s\n",
             ref.className, ref.parameterName);
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
}

// Mapping Commands
static int handleMapClear(int argc, char *argv[]) {
  // Clear RAM mapping data
  globalData.mapData.clear();
  return CLI_SUCCESS;
}

static int handleMapClearFlash(int argc, char *argv[]) {
  // Clear Flash mapping data (for now, same as RAM since we don't have
  // separate Flash storage)
  globalData.mapData.clear();
  return CLI_SUCCESS;
}

static int handleMapAdd(int argc, char *argv[]) {
  // Add mapping records: map_add <payload>
  // Format: <id>,<time>,<encMedia>,<trackStatus>,<offset>;...
  if(argc < 2) {
    ESP_LOGW("CLI", "map_add requires a payload\n");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }

  // Make a copy of the payload since strtok modifies the string
  char payloadCopy[512];
  strncpy(payloadCopy, argv[1], sizeof(payloadCopy) - 1);
  payloadCopy[sizeof(payloadCopy) - 1] = '\0';

  // Parse payload: records separated by ';'
  char *record     = strtok(payloadCopy, ";");
  int   addedCount = 0;

  while(record != nullptr) {
    // Parse record: <id>,<time>,<encMedia>,<trackStatus>,<offset>
    int id, time, encMedia, trackStatus, offset;
    if(sscanf(record, "%d,%d,%d,%d,%d", &id, &time, &encMedia, &trackStatus,
              &offset) == 5) {
      MapPoint point;
      point.encoderMilimeters = encMedia;
      point.baseMotorPWM      = offset;
      globalData.mapData.push_back(point);
      addedCount++;
    }
    record = strtok(nullptr, ";");
  }

  if(addedCount > 0) {
    return CLI_SUCCESS;
  } else {
    ESP_LOGW("CLI", "Failed to parse any mapping records\n");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
}

static int handleMapSaveRuntime(int argc, char *argv[]) {
  // Save RAM mapping to Flash (for now, just acknowledge since we don't have
  // separate Flash storage)
  return CLI_SUCCESS;
}

static int handleMapGet(int argc, char *argv[]) {
  // Get Flash mapping data (for now, same as RAM)
  if(globalData.mapData.empty()) {
    return CLI_SUCCESS;
  }

  for(size_t i = 0; i < globalData.mapData.size(); i++) {
    const MapPoint &point = globalData.mapData[i];
    pushMessageToQueue(globalData, "%zu,%d,%ld,%d,%ld\n", i, 0,
                       point.encoderMilimeters, 1, point.baseMotorPWM);
  }
  return CLI_SUCCESS;
}

static int handleMapGetRuntime(int argc, char *argv[]) {
  // Get RAM mapping data
  if(globalData.mapData.empty()) {
    return CLI_SUCCESS;
  }

  for(size_t i = 0; i < globalData.mapData.size(); i++) {
    const MapPoint &point = globalData.mapData[i];
    pushMessageToQueue(globalData, "%zu,%d,%ld,%d,%ld\n", i, 0,
                       point.encoderMilimeters, 1, point.baseMotorPWM);
  }
  return CLI_SUCCESS;
}

// Runtime Commands
static int handleRuntimeList(int argc, char *argv[]) {
  // List runtime parameters
  pushMessageToQueue(globalData, "Runtime Parameters:\n");
  int index = 0;

  char value[64];
  if(getParameterValue("State", "isReadyToRun", value, sizeof(value))) {
    pushMessageToQueue(globalData, "  %d - State.isReadyToRun: %s\n", index++,
                       value);
  }
  if(getParameterValue("System", "markCount", value, sizeof(value))) {
    pushMessageToQueue(globalData, "  %d - System.markCount: %s\n", index++,
                       value);
  }

  // TODO: Add sensor readings and other runtime data as they become available
  return CLI_SUCCESS;
}

// Control Commands
static int handlePause(int argc, char *argv[]) {
  globalData.isReadyToRun.store(false, std::memory_order_relaxed);
  return CLI_SUCCESS;
}

static int handleResume(int argc, char *argv[]) {
  globalData.isReadyToRun.store(true, std::memory_order_relaxed);
  return CLI_SUCCESS;
}

// Initialize ADC for battery voltage reading (static handle)
static adc_oneshot_unit_handle_t getBatteryAdcHandle() {
  static adc_oneshot_unit_handle_t adc2_handle = nullptr;
  static bool                      initialized = false;

  if(!initialized) {
    // GPIO 18 is on ADC2 for ESP32-S3
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id  = ADC_UNIT_2,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc2_handle);
    if(ret != ESP_OK) {
      ESP_LOGE("CLI", "Failed to initialize ADC2 for battery voltage");
      return nullptr;
    }

    // Configure ADC channel for GPIO 18 (ADC2_CHANNEL_7 on ESP32-S3)
    adc_oneshot_chan_cfg_t config = {
        .atten    = ADC_ATTEN_DB_12, // 0-3.3V range
        .bitwidth = ADC_BITWIDTH_12, // 12-bit resolution (0-4095)
    };
    ret = adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_7, &config);
    if(ret != ESP_OK) {
      ESP_LOGE("CLI", "Failed to configure ADC2 channel for battery voltage");
      adc_oneshot_del_unit(adc2_handle);
      adc2_handle = nullptr;
      return nullptr;
    }

    initialized = true;
  }

  return adc2_handle;
}

static int handleBatVoltage(int argc, char *argv[]) {
  adc_oneshot_unit_handle_t adc_handle = getBatteryAdcHandle();
  if(adc_handle == nullptr) {
    pushMessageToQueue(globalData, "{\"data\": \"0.0\"}");
    return CLI_SUCCESS;
  }

  // Read ADC value
  int       adc_raw = 0;
  esp_err_t ret     = adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_raw);

  if(ret != ESP_OK) {
    ESP_LOGE("CLI", "Failed to read battery voltage ADC: %s",
             esp_err_to_name(ret));
    pushMessageToQueue(globalData, "{\"data\": \"0.0\"}");
    return CLI_SUCCESS;
  }

  // Convert ADC reading (0-4095) to millivolts (0-3300mV)
  // Formula: (adc_raw / 4095) * 3300
  // Using integer math: (adc_raw * 3300) / 4095
  uint32_t voltage_mv = (static_cast<uint32_t>(adc_raw) * 3300) / 4095;

  // Send response in format "bat_voltage <voltage>"
  pushMessageToQueue(globalData, "{\"data\": \"%d.0\"}", voltage_mv);
  return CLI_SUCCESS;
}

// Command map initialization function
static std::unordered_map<std::string, CommandHandler> &getCommandMap() {
  static std::unordered_map<std::string, CommandHandler> commandMap = {
      // Parameter Commands
      {"param_list",      handleParamList     },
      {"param_get",       handleParamGet      },
      {"param_set",       handleParamSet      },
      // Mapping Commands
      {"map_clear",       handleMapClear      },
      {"map_clearFlash",  handleMapClearFlash },
      {"map_add",         handleMapAdd        },
      {"map_SaveRuntime", handleMapSaveRuntime},
      {"map_get",         handleMapGet        },
      {"map_getRuntime",  handleMapGetRuntime },
      // Runtime Commands
      {"runtime_list",    handleRuntimeList   },
      // Control Commands
      {"pause",           handlePause         },
      {"resume",          handleResume        },
      {"bat_voltage",     handleBatVoltage    },
  };
  return commandMap;
}

int cli(char *command) {
  // Parse the command into argc and argv format (like C main function)
  const int MAX_ARGS = 16;
  char     *argv[MAX_ARGS];
  int       argc = 0;

  // Tokenize the command string by spaces
  char *token = strtok(command, " \t\n\r");
  while(token != nullptr && argc < MAX_ARGS) {
    argv[argc] = token;
    argc++;
    token = strtok(nullptr, " \t\n\r");
  }

  // Check if too many arguments
  if(token != nullptr) {
    ESP_LOGW("CLI", "Too many arguments, max %d\n", MAX_ARGS);
    return CLI_ERROR_TOO_MANY_ARGS;
  }

  // Ensure argv[argc] is NULL (like C main function)
  if(argc < MAX_ARGS) {
    argv[argc] = nullptr;
  }

  // Handle commands using argc and argv
  if(argc == 0) {
    ESP_LOGI("CLI", "Empty command\n");
    return CLI_ERROR_EMPTY_COMMAND;
  }

  // Look up command handler in map
  std::unordered_map<std::string, CommandHandler> &commandMap = getCommandMap();
  auto it = commandMap.find(std::string(argv[0]));

  if(it != commandMap.end()) {
    // Command found, call the handler function
    ESP_LOGI("CLI", "Command found: %s\n", argv[0]);
    return it->second(argc, argv);
  } else {
    // Command not found
    ESP_LOGI("CLI", "Command not found: %s\n", argv[0]);
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
}

#endif // CLI_HPP