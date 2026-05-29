#pragma once
#include <Arduino.h>
#include <RTClib.h>
#include "core/Logger.h"
#include "storage/StorageManager.h"

#pragma pack(push, 1)
struct MetricRecord {
  uint32_t ts;
  int16_t value;
};
#pragma pack(pop)
static_assert(sizeof(MetricRecord) == 6, "MetricRecord must be packed");

class MetricStorage {
public:
  MetricStorage(const char* id, StorageManager& storage, Logger& logger)
      : id_(id), storage_(storage), logger_(logger) {}
  bool append(float value, const DateTime& now);
  static bool buildPath(const char* metric, uint16_t year, uint8_t month, uint8_t day, char* out, size_t outLen);
private:
  const char* id_;
  StorageManager& storage_;
  Logger& logger_;
};
