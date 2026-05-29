#include "storage/MetricStorage.h"

bool MetricStorage::buildPath(
  const char* metric,
  uint16_t year,
  uint8_t month,
  uint8_t day,
  char* out,
  size_t outLen
) {
  if (!metric || !*metric || !out || outLen == 0) return false;
  if (month < 1 || month > 12 || day < 1 || day > 31) return false;

  snprintf(
    out,
    outLen,
    "/metrics/%s/%04u/%02u/%02u.bin",
    metric,
    year,
    month,
    day
  );

  return true;
}

bool MetricStorage::append(float value, const DateTime& now) {
  if (!storage_.available()) return false;

  MetricRecord rec{
    now.unixtime(),
    static_cast<int16_t>(value * 100.0f)
  };

  char path[96];
  if (!buildPath(id_, now.year(), now.month(), now.day(), path, sizeof(path))) {
    logger_.warn(LogCategory::Storage, "bad metric path");
    return false;
  }

  if (!storage_.appendBinary(path, reinterpret_cast<const uint8_t*>(&rec), sizeof(rec))) {
    return false;
  }

  return true;
}
