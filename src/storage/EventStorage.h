#pragma once

#include <Arduino.h>
#include <SD.h>
#include <RTClib.h>

class EventStorage {
public:
  EventStorage(const char* type);

  void write(uint8_t subtype, float value, DateTime now);

private:
  struct EventRecord {
    uint32_t ts;
    uint8_t subtype;
    int16_t value; // *100
  };

  String buildPath(DateTime now);
  void ensureDirs(DateTime now);

  String type;
};