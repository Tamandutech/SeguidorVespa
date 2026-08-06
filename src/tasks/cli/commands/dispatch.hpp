#pragma once

#include <vector>

#include "tasks/cli/cli.hpp"
#include "tasks/cli/tamanducli/cli_map.hpp"
#include "tasks/cli/tamanducli/wprotocol.hpp"

namespace cli_dispatch {

void registerCommands(cli::CliMap<kCliMessageSize> &cliMap);

int processWireCommands(cli::CliMap<kCliMessageSize> &cliMap,
                        std::vector<wire::Command>   &cmds);

} // namespace cli_dispatch
