#pragma once
#include <RTClib.h>
#include "models/AppTypes.h"
#include "hardware/RelayController.h"
#include "core/EventBus.h"
#include "core/Logger.h"

class ThermostatService {
public:
  ThermostatService(RelayController& relay, EventBus& events, Logger& logger)
      : relay_(relay), events_(events), logger_(logger) {}
  void tick(const TempConfig& cfg, const DateTime& now, RuntimeState& state);
private:
  RelayController& relay_;
  EventBus& events_;
  Logger& logger_;
  uint32_t lastAutoMs_ = 0;
  bool sensorFailLogged_ = false;
  void setCool(bool on, const DateTime& now, RuntimeState& state);
  void setHeat(bool on, const DateTime& now, RuntimeState& state);
};
