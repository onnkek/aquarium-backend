#pragma once
#include <Arduino.h>
#include <RTClib.h>
#include "core/JournalBus.h"

enum class LogLevel : uint8_t { Debug, Info, Warn, Error };
enum class LogCategory : uint8_t { System, Relay, Doser, Storage, Api, Sensor, Wifi };

class Logger {
public:
  void begin();
  void setTimeProvider(DateTime (*provider)());
  void setStorageEnabled(bool enabled);
  void setStoragePaused(bool paused);
  void info(LogCategory category, const char* msg);
  void warn(LogCategory category, const char* msg);
  void error(LogCategory category, const char* msg);
  void debug(LogCategory category, const char* msg);
  void log(LogLevel level, LogCategory category, const char* msg);
  static void taskEntry(void* arg);

private:
  JournalBus journal_;
  DateTime (*timeProvider_)() = nullptr;
  bool storageEnabled_ = false;
  volatile bool storagePaused_ = false;
  uint32_t storageDisabledUntilMs_ = 0;

  void task();
  void buildPath(LogCategory category, DateTime now, char* out, size_t outLen);
  const char* levelName(LogLevel level);
  const char* categoryFile(LogCategory category);
};
