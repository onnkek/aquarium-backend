#pragma once
#include <RTClib.h>
#include "models/AppTypes.h"
#include "hardware/RelayController.h"
#include "core/EventBus.h"
#include "core/ConfigService.h"
#include "core/Logger.h"

class DoserService {
public:
  DoserService(RelayController& relay, EventBus& events, ConfigService& config, Logger& logger)
      : relay_(relay), events_(events), config_(config), logger_(logger) {}
  void begin(RuntimeState& state);
  void tick(const DateTime& now, RuntimeState& state);
  bool startManual(uint8_t index, const DateTime& now, RuntimeState& state);
  bool stopManual(uint8_t index, const DateTime& now, RuntimeState& state);
private:
  RelayController& relay_;
  EventBus& events_;
  ConfigService& config_;
  Logger& logger_;
  bool shouldStart(const DoserConfig& cfg, const DateTime& now);
  uint32_t durationMs(const DoserConfig& cfg);
  void normalizeLegacyDailyFlags(const DateTime& now);
  void start(uint8_t index, const DateTime& now, RuntimeState& state);
  void stop(uint8_t index, const DateTime& now, RuntimeState& state);
};
