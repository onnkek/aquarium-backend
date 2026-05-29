#include "automation/SchedulerService.h"

void SchedulerService::begin(RuntimeState& state) {
  doser_.begin(state);
  logger_.info(LogCategory::System, "scheduler initialized");
}

void SchedulerService::tick(RuntimeState& state) {
  const uint32_t ms = millis();
  const DateTime now = time_.now();
  AppConfig& cfg = config_.data();

  if (ms - last10Ms_ >= 10) {
    last10Ms_ = ms;
    argb_.tick(cfg.argb, now, state);
  }

  if (ms - last100Ms_ >= 100) {
    last100Ms_ = ms;
    relays_.tick(0, cfg.co2, now, state);
    relays_.tick(1, cfg.o2, now, state);
    relays_.tick(2, cfg.filter, now, state);
    relays_.tick(3, cfg.light, now, state);
    doser_.tick(now, state);
    thermostat_.tick(cfg.temp, now, state);
    fan_.setSpeedPercent(cfg.system.fanPwmPercent);
  }

  if (ms - last1s_ >= 1000) {
    last1s_ = ms;
    sensors_.tick(state);
    state.fanRpm = fan_.rpm();
  }

  if (ms - last60s_ >= 60000) {
    last60s_ = ms;
    if (state.airTempValid) airMetric_.append(state.airTemp, now);
    if (state.humidityValid) humidityMetric_.append(state.humidity, now);
    if (state.waterTempValid) waterMetric_.append(state.waterTemp, now);
  }
}
