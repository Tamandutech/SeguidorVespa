#include "cli.hpp"

#include <cctype>
#include <cstring>
#include <vector>

#include "esp_log.h"

#include "tasks/BluetoothTask.hpp"
#include "tasks/cli/commands/dispatch.hpp"
#include "tasks/cli/tamanducli/cli_map.hpp"
#include "tasks/StateMachineTask.hpp"

namespace {
const char *TAG = "cli";

StateMachineTask *g_cliSm = nullptr;

void cliPushMessage(const char *msg) {
  (void)bluetoothPushMessage("%s", msg);
}

cli::CliMap<kCliMessageSize> &cliMapInstance() {
  static cli::CliMap<kCliMessageSize> map(cliPushMessage);
  static bool                         registered = false;
  if(!registered) {
    cli_dispatch::registerCommands(map);
    registered = true;
  }
  return map;
}
} // namespace

StateMachineTask *cli_active_state_machine() { return g_cliSm; }

int cli_process(char *command, StateMachineTask *stateMachine) {
  if(command == nullptr || stateMachine == nullptr) {
    return CLI_ERROR_EMPTY_COMMAND;
  }

  while(*command != '\0' && isspace(static_cast<unsigned char>(*command))) {
    command++;
  }
  if(*command == '\0') {
    ESP_LOGI(TAG, "Empty command");
    return CLI_ERROR_EMPTY_COMMAND;
  }

  size_t n = strlen(command);
  while(n > 0 && isspace(static_cast<unsigned char>(command[n - 1]))) {
    command[--n] = '\0';
  }

  std::vector<wire::Command> cmds;
  if(!wire::parseMessage(command, cmds)) {
    ESP_LOGW(TAG, "Bad wire message");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  if(cmds.empty()) {
    return CLI_ERROR_EMPTY_COMMAND;
  }
  if(cmds.size() > 48) {
    ESP_LOGW(TAG, "Too many wire segments");
    return CLI_ERROR_TOO_MANY_ARGS;
  }

  g_cliSm           = stateMachine;
  const int ret     = cli_dispatch::processWireCommands(cliMapInstance(), cmds);
  g_cliSm           = nullptr;
  return ret;
}
