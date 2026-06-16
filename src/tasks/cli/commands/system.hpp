#pragma once

#include "tasks/cli/cli.hpp"

namespace cli_system {

bool wirePause(CliProtocol &proto);
bool wireResume(CliProtocol &proto);
bool wireBatVoltage(CliProtocol &proto);

} // namespace cli_system
