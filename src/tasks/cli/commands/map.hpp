#pragma once

#include "data_types.hpp"
#include "tasks/cli/cli.hpp"
#include "tasks/cli/tamanducli/wprotocol.hpp"

namespace cli_map {

bool parseMapAddBodyFields(const wire::WireView &view, int32_t *encoderLeft,
                           int32_t *encoderRight, float *encoderDerivative,
                           float *encoderDerivativeAverage, float *speed,
                           MapPoint::PointType *pointType);

bool wireMapAddBody(const wire::WireView &view, CliProtocol &proto,
                    bool sortAfter);

bool wireMapClear(CliProtocol &proto);
bool wireMapClearStorage(CliProtocol &proto);
bool wireMapSave(CliProtocol &proto);
bool wireMapGet(CliProtocol &proto);

} // namespace cli_map
