#pragma once

#include <Arduino.h>
#include <esp_heap_caps.h>
#include "models/AppTypes.h"

struct HealthSnapshot {
  uint32_t freeHeap = 0;
  uint32_t minFreeHeap = 0;
  uint32_t heapSize = 0;
  uint32_t largestFreeBlock = 0;
  uint32_t uptimeSec = 0;
  uint32_t lastUpdateMs = 0;

  bool sdOk = false;
  bool sdMounted = false;
  uint32_t sdFailureCount = 0;
  bool spiffsOk = false;
  bool wifiOk = false;
  bool rtcOk = false;
  bool emergencyMode = false;
  bool emergencyOverride = false;
};

class HealthService {
public:
  void begin() {
    lastLogMs_ = 0;
    update(RuntimeState{});
  }

  void update(const RuntimeState& state) {
    snapshot_.freeHeap = ESP.getFreeHeap();
    snapshot_.minFreeHeap = ESP.getMinFreeHeap();
    snapshot_.heapSize = ESP.getHeapSize();
    snapshot_.largestFreeBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    snapshot_.uptimeSec = millis() / 1000u;
    snapshot_.lastUpdateMs = millis();

    snapshot_.sdOk = state.sdOk;
    snapshot_.sdMounted = state.sdMounted;
    snapshot_.sdFailureCount = state.sdFailureCount;
    snapshot_.spiffsOk = state.spiffsOk;
    snapshot_.wifiOk = state.wifiOk;
    snapshot_.rtcOk = state.rtcValid;
    snapshot_.emergencyMode = state.emergencyMode;
    snapshot_.emergencyOverride = state.emergencyOverride;
  }

  const HealthSnapshot& snapshot() const { return snapshot_; }

  bool shouldLog(uint32_t intervalMs = 60000) {
    const uint32_t now = millis();
    if (now - lastLogMs_ < intervalMs) return false;
    lastLogMs_ = now;
    return true;
  }

private:
  HealthSnapshot snapshot_;
  uint32_t lastLogMs_ = 0;
};
