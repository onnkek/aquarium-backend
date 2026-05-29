#pragma once
#include <Arduino.h>
#include <RTClib.h>
#include "core/Logger.h"
#include "storage/StorageManager.h"

#pragma pack(push, 1)
struct EventRecord {
  uint32_t ts;
  uint8_t subtype;
  int16_t value;
};
#pragma pack(pop)
static_assert(sizeof(EventRecord) == 7, "EventRecord must be packed");

class EventStorage {
public:
  EventStorage(const char* type, StorageManager& storage, Logger& logger)
      : type_(type), storage_(storage), logger_(logger) {}
  bool write(uint8_t subtype, float value, const DateTime& now);
  static bool buildPath(const char* type, uint16_t year, uint8_t month, char* out, size_t outLen);
private:
  const char* type_;
  StorageManager& storage_;
  Logger& logger_;
};
