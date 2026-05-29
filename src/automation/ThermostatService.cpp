#include "automation/ThermostatService.h"

void ThermostatService::tick(const TempConfig& cfg, const DateTime& now, RuntimeState& state) {
  // Fail-safe: without a valid water temperature sensor, never allow heat/cool outputs,
  // even if config says manual heat/cool. This protects the aquarium and the test bench.
  if (!state.waterTempValid) {
    setCool(false, now, state);
    setHeat(false, now, state);
    if (!sensorFailLogged_ && cfg.mode != TempMode::Off) {
      logger_.warn(LogCategory::Sensor, "water temp invalid: thermostat outputs forced off");
      sensorFailLogged_ = true;
    }
    return;
  }
  sensorFailLogged_ = false;

  switch (cfg.mode) {
    case TempMode::Off: setCool(false, now, state); setHeat(false, now, state); return;
    case TempMode::Cool: setCool(true, now, state); setHeat(false, now, state); return;
    case TempMode::Heat: setCool(false, now, state); setHeat(true, now, state); return;
    case TempMode::CoolAndHeat: setCool(true, now, state); setHeat(true, now, state); return;
    case TempMode::Auto: break;
  }

  if (millis() - lastAutoMs_ < static_cast<uint32_t>(cfg.timeoutSec) * 1000u) return;
  lastAutoMs_ = millis();

  const float t = state.waterTemp;
  if (t > cfg.setting + cfg.hysteresis) { setCool(true, now, state); setHeat(false, now, state); }
  else if (t < cfg.setting - cfg.hysteresis) { setCool(false, now, state); setHeat(true, now, state); }
  else { setCool(false, now, state); setHeat(false, now, state); }
}

void ThermostatService::setCool(bool on, const DateTime& now, RuntimeState& state) {
  RelayStatus next = on ? RelayStatus::On : RelayStatus::Off;
  if (state.cool == next) return;
  relay_.setCool(on); state.cool = next;
  AquariumEvent event{};
  event.topic = AquariumEventTopic::Pid;
  event.subtype = 1;
  event.value = on ? 1.0f : 0.0f;
  event.unixTime = now.unixtime();
  events_.publish(event, 0);
  logger_.info(LogCategory::Relay, on ? "cool on" : "cool off");
}
void ThermostatService::setHeat(bool on, const DateTime& now, RuntimeState& state) {
  RelayStatus next = on ? RelayStatus::On : RelayStatus::Off;
  if (state.heat == next) return;
  relay_.setHeat(on); state.heat = next;
  AquariumEvent event{};
  event.topic = AquariumEventTopic::Pid;
  event.subtype = 2;
  event.value = on ? 1.0f : 0.0f;
  event.unixTime = now.unixtime();
  events_.publish(event, 0);
  logger_.info(LogCategory::Relay, on ? "heat on" : "heat off");
}
