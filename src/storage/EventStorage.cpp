#include "EventStorage.h"

EventStorage::EventStorage(const char* type)
  : type(type) {}

String EventStorage::buildPath(DateTime now) {
  char path[80];

  snprintf(
    path,
    sizeof(path),
    "/events/%s/%04d/%02d.bin",
    type.c_str(),
    now.year(),
    now.month()
  );

  return String(path);
}

void EventStorage::ensureDirs(DateTime now) {
  if (!SD.exists("/events")) {
    SD.mkdir("/events");
  }

  String root = "/events/" + type;
  if (!SD.exists(root)) {
    SD.mkdir(root);
  }

  char y[64];
  snprintf(
    y,
    sizeof(y),
    "/events/%s/%04d",
    type.c_str(),
    now.year()
  );

  if (!SD.exists(y)) {
    SD.mkdir(y);
  }
}

void EventStorage::write(uint8_t subtype, float value, DateTime now) {
// String pathDebug = buildPath(now.unixtime());
// Serial.println("METRIC WRITE: " + pathDebug);
  EventRecord rec;

  rec.ts = now.unixtime();
  rec.subtype = subtype;
  rec.value = (int16_t)(value * 100.0f);

  ensureDirs(now);

  String path = buildPath(now);

  File file = SD.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("EventStorage: SD open failed");
    return;
  }

  file.write((uint8_t*)&rec, sizeof(rec));
  file.close();
}