#include "storage/EventStorage.h"

bool EventStorage::buildPath(
  const char* type,
  uint16_t year,
  uint8_t month,
  char* out,
  size_t outLen
) {
  if (!type || !*type || !out || outLen == 0) return false;
  if (month < 1 || month > 12) return false;

  snprintf(
    out,
    outLen,
    "/events/%s/%04u/%02u.bin",
    type,
    year,
    month
  );

  return true;
}

bool EventStorage::write(uint8_t subtype, float value, const DateTime& now) {
  if (!storage_.available()) return false;

  EventRecord rec{
    now.unixtime(),
    subtype,
    static_cast<int16_t>(value * 100.0f)
  };

  char path[96];
  if (!buildPath(type_, now.year(), now.month(), path, sizeof(path))) {
    logger_.warn(LogCategory::Storage, "bad event path");
    return false;
  }

  if (!storage_.appendBinary(path, reinterpret_cast<const uint8_t*>(&rec), sizeof(rec))) {
    return false;
  }

  return true;
}
