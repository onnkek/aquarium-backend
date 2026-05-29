#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class SdGuard {
public:
  static void begin();
  static SemaphoreHandle_t mutex();

  explicit SdGuard(uint32_t timeoutMs = 1000);
  ~SdGuard();
  bool locked() const { return locked_; }

private:
  bool locked_ = false;
};
