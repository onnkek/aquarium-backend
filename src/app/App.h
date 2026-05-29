#pragma once
#include <FTPServer.h>
#include "models/AppTypes.h"
#include "core/Logger.h"
#include "core/TimeService.h"
#include "core/WifiManager.h"
#include "core/ConfigService.h"
#include "core/WatchdogService.h"
#include "core/CommandBus.h"
#include "core/HealthService.h"
#include "core/EventBus.h"
#include "runtime/ControlTask.h"
#include "runtime/EventWriterTask.h"
#include "hardware/RelayController.h"
#include "hardware/FanController.h"
#include "hardware/SensorService.h"
#include "hardware/ArgbController.h"
#include "storage/StorageManager.h"
#include "storage/MetricStorage.h"
#include "storage/EventStorage.h"
#include "automation/ScheduledRelayService.h"
#include "automation/DoserService.h"
#include "automation/ThermostatService.h"
#include "automation/SchedulerService.h"
#include "api/ApiServer.h"

class App {
public:
  App();
  void begin();
  void loop();
  void tickControl();
  void processCommand(const Command& command);
  void applyEmergencyMode(const char* reason);
  void clearEmergencyMode(const char* reason);
  static DateTime nowStatic();
private:
  RuntimeState state_;
  Logger logger_;
  TimeService time_;
  WifiManager wifi_;
  ConfigService config_;
  WatchdogService watchdog_;
  CommandBus commandBus_;
  EventBus eventBus_;
  HealthService health_;
  StorageManager storage_;
  RelayController relaysHardware_;
  FanController fan_;
  SensorService sensors_;
  ArgbController argb_;
  MetricStorage airMetric_;
  MetricStorage humidityMetric_;
  MetricStorage waterMetric_;
  EventStorage doserEvents_;
  EventStorage relayEvents_;
  EventStorage pidEvents_;
  EventWriterTask eventWriterTask_;
  ScheduledRelayService scheduledRelays_;
  DoserService doser_;
  ThermostatService thermostat_;
  SchedulerService scheduler_;
  ApiServer api_;
  ControlTask controlTask_;
};
