#include "dispatch.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "context/GlobalData.hpp"
#include "data_types.hpp"
#include "tasks/cli/cli.hpp"
#include "tasks/cli/commands/map.hpp"
#include "tasks/cli/commands/param.hpp"
#include "tasks/cli/commands/system.hpp"

namespace cli_dispatch {

using WireCommand = wire::Command;

namespace {

static int batchMapAdd(std::vector<WireCommand> &cmds, size_t headerIdx,
                       const wire::ListHeader &hdr, CliProtocol &proto) {
  for(int k = 1; k <= hdr.C; k++) {
    WireCommand &bk = cmds[headerIdx + static_cast<size_t>(k)];
    wire::WireView view{bk};
    if(!cli_map::wireMapAddBody(view, proto, false)) {
      return CLI_ERROR_COMMAND_NOT_FOUND;
    }
  }
  std::sort(globalData.mapData.begin(), globalData.mapData.end(),
            [](const MapPoint &a, const MapPoint &b) {
              return a.encoderMilimeters < b.encoderMilimeters;
            });
  proto.emitBatchAck("map_add", hdr.j);
  return CLI_SUCCESS;
}

static int batchParamSet(std::vector<WireCommand> &cmds, size_t headerIdx,
                         const wire::ListHeader &hdr, CliProtocol &proto) {
  for(int k = 1; k <= hdr.C; k++) {
    WireCommand &bk = cmds[headerIdx + static_cast<size_t>(k)];
    wire::WireView view{bk};
    if(!cli_param::wireParamSetBodyRamOnly(view)) {
      return CLI_ERROR_COMMAND_NOT_FOUND;
    }
  }
  if(!cli_param::paramSetPersistWireError(proto)) {
    return CLI_SUCCESS;
  }
  proto.emitBatchAck("param_set", hdr.j);
  return CLI_SUCCESS;
}

static int dispatchListHeaderBatch(const WireCommand &header,
                                   std::vector<WireCommand> &cmds,
                                   size_t headerIdx, const wire::ListHeader &hdr,
                                   CliProtocol &proto) {
  const std::string key = wire::commandKeyLower(header.name);
  if(key == "map_add") {
    return batchMapAdd(cmds, headerIdx, hdr, proto);
  }
  if(key == "param_set") {
    return batchParamSet(cmds, headerIdx, hdr, proto);
  }
  return CLI_ERROR_COMMAND_NOT_FOUND;
}

static bool on_param_list(const WireCommand &cmd, wire::WireView view,
                          CliProtocol &proto) {
  (void)view;
  if(cmd.mode != 's' || cmd.role != 'r') {
    return false;
  }
  return cli_param::wireParamList(proto);
}

static bool on_param_get(const WireCommand &cmd, wire::WireView view,
                         CliProtocol &proto) {
  if(cmd.mode != 's' || cmd.role != 'r') {
    return false;
  }
  return cli_param::wireParamGet(view, proto);
}

static bool on_param_set(const WireCommand &cmd, wire::WireView view,
                         CliProtocol &proto) {
  if(cmd.role != 'r') {
    return false;
  }
  if(cmd.mode == 's') {
    return cli_param::wireParamSetSingle(view, proto);
  }
  if(cmd.mode == 'b') {
    return cli_param::wireParamSetLoneBody(view, proto);
  }
  return false;
}

static bool on_map_add(const WireCommand &cmd, wire::WireView view,
                       CliProtocol &proto) {
  if(cmd.mode != 'b' || cmd.role != 'r') {
    return false;
  }
  return cli_map::wireMapAddBody(view, proto, true);
}

static bool on_map_clear(const WireCommand &cmd, wire::WireView view,
                         CliProtocol &proto) {
  (void)view;
  if(cmd.mode != 's' || cmd.role != 'r') {
    return false;
  }
  return cli_map::wireMapClear(proto);
}

static bool on_map_clear_storage(const WireCommand &cmd, wire::WireView view,
                                 CliProtocol &proto) {
  (void)view;
  if(cmd.mode != 's' || cmd.role != 'r') {
    return false;
  }
  return cli_map::wireMapClearStorage(proto);
}

static bool on_map_save(const WireCommand &cmd, wire::WireView view,
                        CliProtocol &proto) {
  (void)view;
  if(cmd.mode != 's' || cmd.role != 'r') {
    return false;
  }
  return cli_map::wireMapSave(proto);
}

static bool on_map_get(const WireCommand &cmd, wire::WireView view,
                       CliProtocol &proto) {
  (void)view;
  if(cmd.mode != 's' || cmd.role != 'r') {
    return false;
  }
  return cli_map::wireMapGet(proto);
}

static bool on_pause(const WireCommand &cmd, wire::WireView view,
                     CliProtocol &proto) {
  (void)view;
  if(cmd.mode != 's' || cmd.role != 'r') {
    return false;
  }
  return cli_system::wirePause(proto);
}

static bool on_resume(const WireCommand &cmd, wire::WireView view,
                      CliProtocol &proto) {
  (void)view;
  if(cmd.mode != 's' || cmd.role != 'r') {
    return false;
  }
  return cli_system::wireResume(proto);
}

static bool on_bat_voltage(const WireCommand &cmd, wire::WireView view,
                           CliProtocol &proto) {
  (void)view;
  if(cmd.mode != 's' || cmd.role != 'r') {
    return false;
  }
  return cli_system::wireBatVoltage(proto);
}

} // namespace

void registerCommands(cli::CliMap<kCliMessageSize> &cliMap) {
  cliMap.registerCommand("param_list", on_param_list);
  cliMap.registerCommand("param_get", on_param_get);
  cliMap.registerCommand("param_set", on_param_set);
  cliMap.registerCommand("map_add", on_map_add);
  cliMap.registerCommand("map_clear", on_map_clear);
  cliMap.registerCommand("map_clear_storage", on_map_clear_storage);
  cliMap.registerCommand("map_save", on_map_save);
  cliMap.registerCommand("map_get", on_map_get);
  cliMap.registerCommand("pause", on_pause);
  cliMap.registerCommand("resume", on_resume);
  cliMap.registerCommand("bat_voltage", on_bat_voltage);
}

int processWireCommands(cli::CliMap<kCliMessageSize> &cliMap,
                        std::vector<wire::Command>   &cmds) {
  CliProtocol &proto = cliMap.protocol();

  for(size_t i = 0; i < cmds.size();) {
    WireCommand &w = cmds[i];
    if(w.role != 'r') {
      i++;
      continue;
    }
    if(w.mode == 'h') {
      wire::ListHeader hdr{};
      if(!wire::parseListHeader(w, hdr)) {
        return CLI_ERROR_COMMAND_NOT_FOUND;
      }
      if(hdr.C < 0 || i + 1 + static_cast<size_t>(hdr.C) > cmds.size()) {
        return CLI_ERROR_COMMAND_NOT_FOUND;
      }
      for(int k = 1; k <= hdr.C; k++) {
        WireCommand &bk = cmds[i + static_cast<size_t>(k)];
        if(bk.mode != 'b' || bk.role != 'r' || !wire::nameEq(bk.name, w.name)) {
          return CLI_ERROR_COMMAND_NOT_FOUND;
        }
      }
      const int r = dispatchListHeaderBatch(w, cmds, i, hdr, proto);
      if(r != CLI_SUCCESS) {
        return r;
      }
      i += 1 + static_cast<size_t>(hdr.C);
      continue;
    }
    if(w.mode == 's' || w.mode == 'b') {
      if(!cliMap.dispatch(w)) {
        return CLI_ERROR_COMMAND_NOT_FOUND;
      }
      i++;
      continue;
    }
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return CLI_SUCCESS;
}

} // namespace cli_dispatch
