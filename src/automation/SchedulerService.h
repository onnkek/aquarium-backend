#pragma once
#include "models/AppTypes.h"
#include "core/ConfigService.h"
#include "core/TimeService.h"
#include "core/Logger.h"
#include "automation/DoserService.h"
#include "automation/ScheduledRelayService.h"
#include "automation/ThermostatService.h"
#include "hardware/ArgbController.h"
#include "hardware/SensorService.h"
#include "hardware/FanController.h"
#include "storage/MetricStorage.h"

class SchedulerService {
public:
  SchedulerService(
      ConfigService& config,
      TimeService& time,
      Logger& logger,
      SensorService& sensors,
      FanController& fan,
      ArgbController& argb,
      ScheduledRelayService& relays,
      DoserService& doser,
      ThermostatService& thermostat,
      MetricStorage& airMetric,
      MetricStorage& humidityMetric,
      MetricStorage& waterMetric)
      : config_(config), time_(time), logger_(logger), sensors_(sensors), fan_(fan), argb_(argb), relays_(relays), doser_(doser), thermostat_(thermostat), airMetric_(airMetric), humidityMetric_(humidityMetric), waterMetric_(waterMetric) {}

  void begin(RuntimeState& state);
  void tick(RuntimeState& state);

private:
  ConfigService& config_;
  TimeService& time_;
  Logger& logger_;
  SensorService& sensors_;
  FanController& fan_;
  ArgbController& argb_;
  ScheduledRelayService& relays_;
  DoserService& doser_;
  ThermostatService& thermostat_;
  MetricStorage& airMetric_;
  MetricStorage& humidityMetric_;
  MetricStorage& waterMetric_;
  uint32_t last10Ms_ = 0;
  uint32_t last100Ms_ = 0;
  uint32_t last1s_ = 0;
  uint32_t last60s_ = 0;
};
