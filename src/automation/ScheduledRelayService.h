#pragma once
#include <RTClib.h>
#include "models/AppTypes.h"
#include "hardware/RelayController.h"
#include "core/EventBus.h"
#include "core/Logger.h"

class ScheduledRelayService {
public:
  ScheduledRelayService(RelayController& relay, EventBus& events, Logger& logger)
      : relay_(relay), events_(events), logger_(logger) {}
  void tick(uint8_t index, const ScheduleRelayConfig& cfg, const DateTime& now, RuntimeState& state);
  void force(uint8_t index, bool on, const char* reason, const DateTime& now, RuntimeState& state);
private:
  RelayController& relay_;
  EventBus& events_;
  Logger& logger_;
  void set(uint8_t index, bool on, const char* reason, const DateTime& now, RuntimeState& state);
};
