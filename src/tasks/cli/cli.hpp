#pragma once

#include <stddef.h>
#include <stdint.h>

#include "tasks/cli/tamanducli/wprotocol.hpp"

class StateMachineTask;

#define CLI_SUCCESS                 0
#define CLI_ERROR_EMPTY_COMMAND     1
#define CLI_ERROR_COMMAND_NOT_FOUND 2
#define CLI_ERROR_TOO_MANY_ARGS     3

/// FAT 8.3: nome base deve ter <= 8 caracteres quando LFN está desabilitado.
#define MAP_STORAGE_FILE        "map_data.dat"
#define PARAMETERS_STORAGE_FILE "params.dat"

/// Tamanho máximo de cada mensagem wire (alinhado ao NUS BLE).
constexpr size_t kCliMessageSize = 256;

using CliProtocol = wire::Protocol<kCliMessageSize>;

/// Ativo apenas durante a execução de \c cli_process (thread de comando UART BLE).
StateMachineTask *cli_active_state_machine();

/// CLI de protocolo wire (`param_list(...)`, `map_get(...)`, …).
int cli_process(char *command, StateMachineTask *stateMachine);
