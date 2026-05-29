#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include "core/Logger.h"

class WifiManager {
public:
  explicit WifiManager(Logger& logger) : logger_(logger) {}
  void begin(const char* ssid, const char* password);
  bool connected() const { return WiFi.status() == WL_CONNECTED; }
  static void taskEntry(void* arg);

private:
  Logger& logger_;
  const char* ssid_ = nullptr;
  const char* password_ = nullptr;
  uint32_t retryDelayMs_ = 5000;
  uint32_t lastTryMs_ = 0;
  bool wasConnected_ = false;
  void task();
};
