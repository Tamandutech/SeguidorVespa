#pragma once

#include "tasks/cli/cli.hpp"
#include "tasks/cli/tamanducli/wprotocol.hpp"

namespace cli_param {

enum class ParseError {
  SUCCESS = 0,
  EMPTY_STRING,
  NO_DOT,
  MULTIPLE_DOTS,
  EMPTY_CLASS_NAME,
  EMPTY_PARAMETER_NAME,
  INVALID_FORMAT
};

struct ParsedReference {
  char className[64];
  char parameterName[64];
};

ParseError parseClassNameParameter(const char *input, ParsedReference &result);

bool parseCliFloat(const char *value, float *out);

bool getParameterValue(const char *className, const char *parameterName,
                       char *valueBuffer, size_t bufferSize);

bool setParameterValue(const char *className, const char *parameterName,
                       const char *value);

bool paramSetPersistWireError(CliProtocol &proto);

int paramSetRamOnly(const char *refWire, const char *valueWire);

bool wireParamList(CliProtocol &proto);
bool wireParamGet(const wire::WireView &view, CliProtocol &proto);
bool wireParamSetSingle(const wire::WireView &view, CliProtocol &proto);
bool wireParamSetBodyRamOnly(const wire::WireView &view);
bool wireParamSetLoneBody(const wire::WireView &view, CliProtocol &proto);

} // namespace cli_param
