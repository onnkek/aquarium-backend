#include "app/App.h"
#include "secrets/secrets.h"
#include "config/PinMap.h"
#include "config/BuildConfig.h"
#include "core/SdGuard.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <SPIFFS.h>
#include <Wire.h>
#if AQUARIUM_ENABLE_FTP
#include <FTPServer.h>
#include "secrets/secrets.h"
#endif

static App* gApp = nullptr;
static TimeService* gTime = nullptr;
#if AQUARIUM_ENABLE_FTP
static FTPServer ftpSrv(SD);
#endif

App::App()
  : wifi_(logger_),
    config_(logger_),
    storage_(logger_),
    sensors_(logger_),
    airMetric_("air_temp", storage_, logger_),
    humidityMetric_("humidity", storage_, logger_),
    waterMetric_("water_temp", storage_, logger_),
    doserEvents_("doser", storage_, logger_),
    relayEvents_("relay", storage_, logger_),
    pidEvents_("pid", storage_, logger_),
    eventWriterTask_(eventBus_, doserEvents_, relayEvents_, pidEvents_, logger_),
    scheduledRelays_(relaysHardware_, eventBus_, logger_),
    doser_(relaysHardware_, eventBus_, config_, logger_),
    thermostat_(relaysHardware_, eventBus_, logger_),
    scheduler_(config_, time_, logger_, sensors_, fan_, argb_, scheduledRelays_, doser_, thermostat_, airMetric_, humidityMetric_, waterMetric_),
    api_(config_, time_, storage_, commandBus_, health_, logger_, state_),
    controlTask_(*this) {}

DateTime App::nowStatic() {
  return gTime ? gTime->now() : DateTime(2000, 1, 1, 0, 0, 0);
}

void App::begin() {
  gApp = this;
  gTime = &time_;

  Serial.begin(115200);
  delay(100);

  SdGuard::begin();
  logger_.begin();
  logger_.setTimeProvider(App::nowStatic);
  watchdog_.begin();
  commandBus_.begin();
  eventBus_.begin();
  health_.begin();

  state_.spiffsOk = SPIFFS.begin(true);
  if (!state_.spiffsOk) logger_.error(LogCategory::System, "SPIFFS mount failed");

  SPI.begin(Pins::CLK, Pins::MISO, Pins::MOSI, Pins::SD_CS);
#if AQUARIUM_USE_LEGACY_SD_INIT
  state_.sdOk = SD.begin(Pins::SD_CS);
#else
  state_.sdOk = SD.begin(Pins::SD_CS, SPI);
#endif
  if (state_.sdOk) {
    const uint64_t cardSizeMb = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.printf("SD Card Size: %lluMB\n", cardSizeMb);
  }
  state_.sdMounted = state_.sdOk;
  logger_.setStorageEnabled(state_.sdOk && AQUARIUM_ENABLE_SD_JOURNAL);
  storage_.setMounted(state_.sdOk);
  if (!state_.sdOk) logger_.error(LogCategory::Storage, "SD mount failed");
  storage_.begin();

  // Match the working live firmware: initialize I2C pins before RTC/AHT20.
  Wire.begin(Pins::SDA, Pins::SCL);

  state_.rtcValid = time_.begin();
  if (!state_.rtcValid) logger_.warn(LogCategory::System, "RTC unavailable or invalid: emergency mode will be applied");

  relaysHardware_.begin();
  fan_.begin();
  sensors_.begin();
  argb_.begin();
  config_.begin();
  if (config_.hasPreEmergencySnapshot()) {
    state_.emergencyMode = true;
    logger_.warn(LogCategory::System, "pre-emergency snapshot exists: emergency mode considered active until cleared");
  }
  wifi_.begin(WIFI_SSID, WIFI_PASSWORD);
  scheduler_.begin(state_);
#if AQUARIUM_ENABLE_SD_EVENTS
  if (!eventWriterTask_.begin()) {
    logger_.error(LogCategory::System, "event writer task start failed");
  } else {
    logger_.info(LogCategory::System, "sd event writer started");
  }
#else
  logger_.info(LogCategory::System, "sd event writer disabled");
#endif
#if AQUARIUM_ENABLE_FTP
  if (state_.sdOk) {
    ftpSrv.begin(FTP_USER, FTP_PASSWORD);
    logger_.info(LogCategory::Api, "ftp started");
  } else {
    logger_.warn(LogCategory::Api, "ftp disabled: sd not mounted");
  }
#endif
  api_.begin();
  if (!controlTask_.begin()) {
    logger_.error(LogCategory::System, "control task start failed");
  }

  logger_.info(LogCategory::System, "aquarium backend production rewrite started");
}

void App::loop() {
  // loopTask is registered in WatchdogService::begin(), so it must be fed here.
  watchdog_.feed();

  // Network/FTP side. Hardware automation runs in ControlTask on core 1.
  // Keep delay tiny: FTPServer expects handleFTP() to be called very often.
#if AQUARIUM_ENABLE_FTP
  if (state_.sdMounted) {
#if AQUARIUM_PAUSE_INTERNAL_SD_WRITES_DURING_FTP
    const bool wasFtpActive = ftpSrv.active();
    storage_.setExternalBusy(wasFtpActive);
    logger_.setStoragePaused(wasFtpActive);
#endif
    ftpSrv.handleFTP();
#if AQUARIUM_PAUSE_INTERNAL_SD_WRITES_DURING_FTP
    const bool isFtpActive = ftpSrv.active();
    storage_.setExternalBusy(isFtpActive);
    logger_.setStoragePaused(isFtpActive);
#endif
  }
#endif
  api_.tick();
  delay(1);
}

void App::tickControl() {
  watchdog_.feed();

  state_.wifiOk = wifi_.connected();
  state_.rtcValid = time_.valid();

  if (state_.rtcValid && state_.emergencyOverride) {
    state_.emergencyOverride = false;
    logger_.info(LogCategory::System, "emergency override cleared automatically: rtc valid");
  }

  if (!state_.rtcValid && !state_.emergencyOverride) {
    if (!state_.emergencyMode) {
      applyEmergencyMode("rtc unavailable or invalid");
    }
  }

  state_.sdMounted = storage_.mounted();
  state_.sdOk = storage_.available();
  state_.sdFailureCount = storage_.failureCount();

  Command command;
  while (commandBus_.receive(command, 0)) {
    processCommand(command);
  }

  scheduler_.tick(state_);
  health_.update(state_);

  if (health_.shouldLog()) {
    const HealthSnapshot& h = health_.snapshot();
    char msg[128];
    snprintf(
      msg,
      sizeof(msg),
      "heap free=%lu min=%lu largest=%lu uptime=%lu",
      static_cast<unsigned long>(h.freeHeap),
      static_cast<unsigned long>(h.minFreeHeap),
      static_cast<unsigned long>(h.largestFreeBlock),
      static_cast<unsigned long>(h.uptimeSec)
    );
    logger_.info(LogCategory::System, msg);
  }
}

void App::applyEmergencyMode(const char* reason) {
  const DateTime now = time_.now();

  // Aquarium emergency mode is NOT a total blackout.
  // Keep life-support running: O2, filter and fixture cooling fans (light relay) ON.
  // Disable dangerous outputs: CO2, heat/cool and all dosers OFF.
  // Enable visual alarm: ARGB static red.
  config_.applyEmergencyMode();

  relaysHardware_.setExtra(0, false); // CO2 OFF
  relaysHardware_.setExtra(1, true);  // O2 ON
  relaysHardware_.setExtra(2, true);  // Filter ON
  relaysHardware_.setExtra(3, true);  // Light fixture cooling fans ON
  relaysHardware_.tempOff();
  relaysHardware_.dosersOff();
  fan_.setSpeedPercent(0);

  state_.extra[0] = RelayStatus::Off;
  state_.extra[1] = RelayStatus::On;
  state_.extra[2] = RelayStatus::On;
  state_.extra[3] = RelayStatus::On;
  state_.cool = RelayStatus::Off;
  state_.heat = RelayStatus::Off;
  state_.emergencyMode = true;

  for (uint8_t i = 0; i < HardwareConfig::DOSER_COUNT; ++i) {
    state_.doser[i].running = false;
    state_.doser[i].status = RelayStatus::Off;
    state_.doser[i].progress = 0;
  }

  // Apply ARGB alarm immediately instead of waiting for the next scheduler tick.
  argb_.tick(config_.data().argb, now, state_);

  char msg[160];
  snprintf(
    msg,
    sizeof(msg),
    "emergency mode applied: reason=%s; co2/heat/cool/dosers off, o2/filter/light-cooling on, argb red",
    reason ? reason : "unknown"
  );
  logger_.warn(LogCategory::System, msg);
}

void App::clearEmergencyMode(const char* reason) {
  const bool restored = config_.restorePreEmergencySnapshot();

  state_.emergencyMode = false;

  if (!state_.rtcValid) {
    state_.emergencyOverride = true;
  } else {
    state_.emergencyOverride = false;
  }

  char msg[192];
  snprintf(
    msg,
    sizeof(msg),
    "emergency mode cleared manually: reason=%s; override=%s; restore=%s",
    reason ? reason : "unknown",
    state_.emergencyOverride ? "true" : "false",
    restored ? "ok" : "no snapshot"
  );
  logger_.warn(LogCategory::System, msg);
}

void App::processCommand(const Command& command) {
  const DateTime now = time_.now();

  switch (command.type) {
    case CommandType::DoserRun:
      doser_.startManual(command.target, now, state_);
      break;

    case CommandType::DoserStop:
      doser_.stopManual(command.target, now, state_);
      break;

    case CommandType::DoserRefill:
      config_.refillDoser(command.target, static_cast<uint16_t>(command.value));
      break;

    case CommandType::RelayOn:
      config_.setRelayMode(command.target, Mode::On);
      scheduledRelays_.force(command.target, true, "command on", now, state_);
      break;

    case CommandType::RelayOff:
      config_.setRelayMode(command.target, Mode::Off);
      scheduledRelays_.force(command.target, false, "command off", now, state_);
      break;

    case CommandType::ReloadConfig:
      config_.load();
      break;

    case CommandType::SaveConfig:
      config_.save();
      break;

    case CommandType::EmergencyOff:
      state_.emergencyOverride = false;
      applyEmergencyMode("api command");
      break;

    case CommandType::EmergencyClear:
      clearEmergencyMode("api command");
      break;

    case CommandType::EmergencyOverrideClear:
      state_.emergencyOverride = false;
      if (!state_.rtcValid && !state_.emergencyMode) {
        applyEmergencyMode("emergency override cleared");
      } else {
        logger_.warn(LogCategory::System, "emergency override cleared manually");
      }
      break;

    case CommandType::None:
    default:
      break;
  }
}
