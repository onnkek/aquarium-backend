#pragma once
#include <stdint.h>
#include <stddef.h>
#include "config/PinMap.h"

enum class Mode : uint8_t { Off = 0, On = 1, Auto = 2 };

enum class TempMode : uint8_t {
  Off = 0,
  Cool = 1,
  Heat = 2,
  CoolAndHeat = 3,
  Auto = 4,
};

enum class RelayStatus : uint8_t { Off = 0, On = 1, Running = 2 };

struct TimeOfDay {
  uint8_t hour = 0;
  uint8_t minute = 0;
};

struct SystemConfig {
  char name[32] = "System";
  uint16_t updateSec = 1;
  uint8_t fanPwmPercent = 30;
};

struct PeriodMask {
  uint8_t bits = 0x7F;
};

struct DoserConfig {
  char name[32] = "";
  PeriodMask period;
  TimeOfDay time;
  uint16_t currentVolumeMlX10 = 0;
  uint16_t maxVolumeMlX10 = 0;
  Mode mode = Mode::Off;
  uint16_t dosageMlX10 = 0;
  uint16_t rateMlPerSecX10 = 10;
  char lastRunYmd[11] = "";
};

struct ScheduleRelayConfig {
  char name[32] = "";
  TimeOfDay on;
  TimeOfDay off;
  Mode mode = Mode::Off;
};

struct RgbColor {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

struct ArgbConfig {
  char name[32] = "Backlighting";
  Mode mode = Mode::Off;
  uint8_t style = 1;
  uint8_t brightness = 100;
  TimeOfDay on;
  TimeOfDay off;
  RgbColor staticColor;
  RgbColor gradientStart;
  RgbColor gradientEnd;
  RgbColor custom[HardwareConfig::NUM_LEDS];
  uint16_t cycleSpeedMs = 20;
};

struct TempConfig {
  char name[32] = "Thermostat";
  float setting = 23.0f;
  float hysteresis = 1.0f;
  float k = 0.5f;
  uint16_t timeoutSec = 1;
  TempMode mode = TempMode::Auto;
};

struct AppConfig {
  SystemConfig system;
  DoserConfig doser[HardwareConfig::DOSER_COUNT];
  ScheduleRelayConfig co2;
  ScheduleRelayConfig o2;
  ScheduleRelayConfig filter;
  ScheduleRelayConfig light;
  ArgbConfig argb;
  TempConfig temp;
};

struct DoserRuntime {
  RelayStatus status = RelayStatus::Off;
  bool running = false;
  uint32_t startedAtMs = 0;
  uint32_t durationMs = 0;
  uint8_t progress = 0;
};

struct RuntimeState {
  RelayStatus extra[HardwareConfig::EXTRA_RELAY_COUNT];
  RelayStatus argb = RelayStatus::Off;
  RelayStatus cool = RelayStatus::Off;
  RelayStatus heat = RelayStatus::Off;
  DoserRuntime doser[HardwareConfig::DOSER_COUNT];
  float airTemp = 0;
  float humidity = 0;
  float waterTemp = 0;
  bool airTempValid = false;
  bool humidityValid = false;
  bool waterTempValid = false;
  uint32_t fanRpm = 0;
  bool rtcValid = false;
  bool emergencyMode = false;
  bool emergencyOverride = false;
  bool sdOk = false;
  bool sdMounted = false;
  uint32_t sdFailureCount = 0;
  bool spiffsOk = false;
  bool wifiOk = false;
};
