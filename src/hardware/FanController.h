#pragma once
#include <Arduino.h>

class FanController {
public:
  void begin();
  void setSpeedPercent(uint8_t percent);
  uint32_t rpm() const;
  static void tachIsr();
private:
  static volatile uint32_t pulses_;
  uint32_t lastSampleMs_ = 0;
  uint32_t lastPulses_ = 0;
  uint32_t rpm_ = 0;
  uint16_t toPwm(uint8_t percent) const;
};
