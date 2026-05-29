#pragma once
#include <Arduino.h>
#include <FastLED.h>
#include <RTClib.h>
#include "models/AppTypes.h"

class ArgbController {
public:
  void begin();
  void tick(const ArgbConfig& cfg, const DateTime& now, RuntimeState& state);
private:
  CRGB leds_[HardwareConfig::NUM_LEDS];
  uint8_t counter_ = 0;
  uint32_t lastCycleMs_ = 0;
  CRGB toCrgb(const RgbColor& c) const;
};
