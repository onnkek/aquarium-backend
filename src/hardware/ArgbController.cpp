#include "hardware/ArgbController.h"
#include "utils/TimeUtils.h"
#include "config/PinMap.h"

void ArgbController::begin() {
  FastLED.addLeds<WS2811, Pins::ARGB, GRB>(leds_, HardwareConfig::NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.clear(true);
}

CRGB ArgbController::toCrgb(const RgbColor& c) const { return CRGB(c.r, c.g, c.b); }

void ArgbController::tick(const ArgbConfig& cfg, const DateTime& now, RuntimeState& state) {
  bool active = cfg.mode == Mode::On || (cfg.mode == Mode::Auto && TimeUtils::isActiveWindow(cfg.on, cfg.off, now));
  if (cfg.mode == Mode::Off || !active) {
    if (state.argb != RelayStatus::Off) {
      FastLED.clear(true);
      state.argb = RelayStatus::Off;
    }
    return;
  }
  FastLED.setBrightness(cfg.brightness);
  switch (cfg.style) {
    case 1:
      fill_solid(leds_, HardwareConfig::NUM_LEDS, toCrgb(cfg.staticColor));
      FastLED.show();
      break;
    case 2:
      if (millis() - lastCycleMs_ >= cfg.cycleSpeedMs) {
        for (uint8_t i = 0; i < HardwareConfig::NUM_LEDS; ++i) leds_[i].setHue(counter_ + i * 255 / HardwareConfig::NUM_LEDS);
        counter_++;
        FastLED.show();
        lastCycleMs_ = millis();
      }
      break;
    case 3:
      for (uint8_t i = 0; i < HardwareConfig::NUM_LEDS; ++i) {
        float t = i / float(HardwareConfig::NUM_LEDS - 1);
        leds_[i] = blend(toCrgb(cfg.gradientStart), toCrgb(cfg.gradientEnd), uint8_t(t * 255));
      }
      FastLED.show();
      break;
    case 4:
      for (uint8_t i = 0; i < HardwareConfig::NUM_LEDS; ++i) leds_[i] = toCrgb(cfg.custom[i]);
      FastLED.show();
      break;
  }
  state.argb = RelayStatus::On;
}
