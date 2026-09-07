#include "map.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "esp_err.h"
#include "esp_log.h"

#include "context/GlobalData.hpp"
#include "data_types.hpp"
#include "env.hpp"
#include "storage/storage.hpp"
#include "tasks/cli/cli.hpp"

namespace cli_map {

namespace {
const char *TAG = "cli_map";

MapPoint::PointType pointTypeFromInt(int value) {
  switch(value) {
  case MapPoint::AUTO_MARK:
  case MapPoint::MANUAL_MARK:
  case MapPoint::STOP_COMMAND_MARK:
  case MapPoint::UNKNOWN_MARK:
  case MapPoint::CURVE_START_MARK:
  case MapPoint::CURVE_END_MARK: return static_cast<MapPoint::PointType>(value);
  default: return MapPoint::UNKNOWN_MARK;
  }
}
} // namespace

bool parseMapAddBodyFields(const wire::WireView &view, int32_t *encoderLeft,
                           int32_t *encoderRight, float *encoderDerivative,
                           float *encoderDerivativeAverage, float *speed,
                           MapPoint::PointType *pointType) {
  if(view.payloadArgc() < 6 || encoderLeft == nullptr ||
     encoderRight == nullptr || encoderDerivative == nullptr ||
     encoderDerivativeAverage == nullptr || speed == nullptr ||
     pointType == nullptr) {
    return false;
  }
  int left  = 0;
  int right = 0;
  int type  = 0;
  if(!wire::parseInt(view.arg(0), left) ||
     !wire::parseInt(view.arg(1), right) ||
     !wire::parseFloat(view.arg(2), *encoderDerivative) ||
     !wire::parseFloat(view.arg(3), *encoderDerivativeAverage) ||
     !wire::parseFloat(view.arg(4), *speed) ||
     !wire::parseInt(view.arg(5), type)) {
    return false;
  }
  *encoderLeft  = static_cast<int32_t>(left);
  *encoderRight = static_cast<int32_t>(right);
  *pointType    = pointTypeFromInt(type);
  return true;
}

bool wireMapAddBody(const wire::WireView &view, CliProtocol &proto,
                    bool sortAfter) {
  int32_t             encoderLeft               = 0;
  int32_t             encoderRight              = 0;
  float               encoderDerivative         = 0.0F;
  float               encoderDerivativeAverage  = 0.0F;
  float               speed                     = 0.0F;
  MapPoint::PointType pointType                 = MapPoint::MANUAL_MARK;
  if(!parseMapAddBodyFields(view, &encoderLeft, &encoderRight,
                            &encoderDerivative, &encoderDerivativeAverage,
                            &speed, &pointType)) {
    ESP_LOGW(TAG, "map_add body: need 6 fields after idx "
                  "(encoder_left,encoder_right,derivative,average,speed,type)");
    return false;
  }
  if(globalData.mapData.size() >= static_cast<size_t>(MAP_POINT_MAX_COUNT)) {
    ESP_LOGW(TAG, "map_add: map full (%d)", MAP_POINT_MAX_COUNT);
    if(sortAfter) {
      return proto.emitSingleResponse("map_add", {"error", "map full"});
    }
    return false;
  }
  MapPoint point;
  point.encoderLeft              = encoderLeft;
  point.encoderRight             = encoderRight;
  point.encoderDerivative        = encoderDerivative;
  point.encoderDerivativeAverage = encoderDerivativeAverage;
  point.speed                    = speed;
  point.pointType                = pointType;
  globalData.mapData.push_back(point);
  if(sortAfter) {
    std::sort(globalData.mapData.begin(), globalData.mapData.end(),
              [](const MapPoint &a, const MapPoint &b) {
                return mapPointProgress(a) < mapPointProgress(b);
              });
    return proto.emitSingleResponse("map_add", {"ok"});
  }
  return true;
}

bool wireMapClear(CliProtocol &proto) {
  globalData.mapData.clear();
  return proto.emitSingleResponse("map_clear", {"ok"});
}

bool wireMapClearStorage(CliProtocol &proto) {
  Storage              *storage = Storage::getInstance();
  std::vector<MapPoint> emptyMap;
  esp_err_t             ret = storage->write_vector(emptyMap, MAP_STORAGE_FILE);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "map_clear_storage failed (%s)", esp_err_to_name(ret));
    return proto.emitSingleResponse("map_clear_storage",
                                    {"error", "Failed to clear Flash"});
  }
  return proto.emitSingleResponse("map_clear_storage", {"ok"});
}

bool wireMapSave(CliProtocol &proto) {
  Storage  *storage = Storage::getInstance();
  esp_err_t ret = storage->write_vector(globalData.mapData, MAP_STORAGE_FILE);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "map_save failed (%s)", esp_err_to_name(ret));
    return proto.emitSingleResponse("map_save",
                                    {"error", "Failed to save to Flash"});
  }
  return proto.emitSingleResponse("map_save", {"ok"});
}

bool wireMapGet(CliProtocol &proto) {
  std::vector<std::string> bodies;
  for(size_t i = 0; i < globalData.mapData.size(); i++) {
    const MapPoint &point = globalData.mapData[i];
    char leftBuf[16], rightBuf[16], derivBuf[16], avgBuf[16], speedBuf[16],
        typeBuf[16];
    snprintf(leftBuf, sizeof(leftBuf), "%ld",
             static_cast<long>(point.encoderLeft));
    snprintf(rightBuf, sizeof(rightBuf), "%ld",
             static_cast<long>(point.encoderRight));
    snprintf(derivBuf, sizeof(derivBuf), "%.3f",
             static_cast<double>(point.encoderDerivative));
    snprintf(avgBuf, sizeof(avgBuf), "%.3f",
             static_cast<double>(point.encoderDerivativeAverage));
    snprintf(speedBuf, sizeof(speedBuf), "%.3f",
             static_cast<double>(point.speed));
    snprintf(typeBuf, sizeof(typeBuf), "%d", static_cast<int>(point.pointType));
    std::string seg = proto.makeListBodySegment(
        "map_get", 's', static_cast<int>(i + 1),
        {leftBuf, rightBuf, derivBuf, avgBuf, speedBuf, typeBuf});
    if(seg.empty()) {
      return false;
    }
    bodies.push_back(std::move(seg));
  }
  proto.emitListResponse("map_get", bodies);
  return true;
}

} // namespace cli_map
